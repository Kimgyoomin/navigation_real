#!/usr/bin/env python3
"""Interactive costmap review of 2026-10-30 20:09:54 recorded geometry.

Uses the prior read-only extraction/validation artifacts under ros2_ws/log.
Original cloud bytes/stamps are retained. Recorded poses are interpolated;
up_body is reconstructed, and delivery is scheduled at header + 120 ms.
This isolates the height gate, not raw-IMU estimation or recorded DDS delays.
Starts the actual RUBI Nav2 launch, but requests ComputePathToPose only.
Never starts LCM, sends NavigateToPose, or replays recorded cmd_vel.
"""
import argparse
import copy
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker


ROOT = Path(__file__).resolve().parents[1]
WS = ROOT.parents[1]


FILTER_OPTIONS = {
    'min_body_height': 'minimum height above BODY along gravity-up [m]',
    'max_body_height': 'maximum height above BODY along gravity-up [m]',
    'front_half_angle_deg': 'front sector half-angle [deg], strictly between 0 and 90',
    'front_min_range': 'front minimum sensor-origin XY range [m]',
    'other_min_range': 'side/rear minimum sensor-origin XY range [m]',
}


def review_parameters(args):
    """Copy the installed launch defaults; never write back to the source YAML."""
    source = (Path(args.params_file).expanduser() if args.params_file else
              Path(get_package_share_directory('pongbot_navigation')) / 'config/nav2_rubi_dwb.yaml')
    source = source.resolve()
    params = yaml.safe_load(source.read_text())
    filt = params['costmap_sector_filter']['ros__parameters']
    for name in FILTER_OPTIONS:
        override = getattr(args, name)
        value = float(filt[name] if override is None else override)
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
        filt[name] = value
    if filt['min_body_height'] > filt['max_body_height']:
        raise ValueError('min_body_height must be <= max_body_height')
    if not 0 < filt['front_half_angle_deg'] < 90:
        raise ValueError('front_half_angle_deg must be strictly between 0 and 90')
    if not 0 <= filt['front_min_range'] <= filt['other_min_range']:
        raise ValueError('require 0 <= front_min_range <= other_min_range')
    if not math.isfinite(filt['other_min_range'] ** 2):
        raise ValueError('range squared overflow')
    return source, params


def topic(name, transient=False, best_effort=False):
    return {'Value': name, 'Depth': 1, 'History Policy': 'Keep Last',
            'Reliability Policy': 'Best Effort' if best_effort else 'Reliable',
            'Durability Policy': 'Transient Local' if transient else 'Volatile'}


def rviz_config(path, center):
    template = yaml.safe_load((ROOT / 'FAST_LIO_LOCALIZATION2/rviz/fastlio_localization.rviz').read_text())
    original = template['Visualization Manager']['Displays']
    map_template = next(d for d in original if d['Class'] == 'rviz_default_plugins/Map')
    path_template = next(d for d in original if d['Class'] == 'rviz_default_plugins/Path')
    displays = []
    for name, name_topic, enabled, alpha, scheme in [
        ('1 Static map', '/map', True, .55, 'map'),
        ('2 NEW Local - BODY height', '/local_costmap/costmap', True, .9, 'costmap'),
        ('3 OLD Local - recorded (toggle 2 off)', '/review/old_local', False, .9, 'costmap'),
        ('4 NEW Global', '/global_costmap/costmap', False, .7, 'costmap'),
        ('5 OLD Global - recorded (toggle 4 off)', '/review/old_global', False, .7, 'costmap')]:
        d = copy.deepcopy(map_template)
        d.update({'Name': name, 'Enabled': enabled, 'Value': enabled,
                  'Alpha': alpha, 'Color Scheme': scheme, 'Draw Behind': False})
        d['Topic'] = topic(name_topic, name_topic == '/map' or name_topic.startswith('/review/'))
        d['Update Topic'] = topic(name_topic + '_updates')
        displays.append(d)
    for name, name_topic, color in [
        ('6 RECORDED route - cyan', '/review/recorded_plan', '0; 220; 255'),
        ('7 NEW A-star - yellow', '/review/new_plan', '255; 220; 0')]:
        d = copy.deepcopy(path_template)
        d.update({'Name': name, 'Enabled': True, 'Value': True,
                  'Color': color, 'Line Width': .04, 'Line Style': 'Billboards'})
        d['Topic'] = topic(name_topic, True)
        displays.append(d)
    displays += [
        {'Class': 'rviz_default_plugins/Axes', 'Name': '8 Robot BODY', 'Enabled': True,
         'Reference Frame': 'base_link', 'Length': .6, 'Radius': .04, 'Value': True},
        {'Class': 'rviz_default_plugins/Polygon', 'Name': '9 Current footprint',
         'Enabled': True, 'Value': True, 'Color': '0; 255; 120', 'Alpha': 1.,
         'Topic': topic('/local_costmap/published_footprint')},
        {'Class': 'rviz_default_plugins/PointCloud2', 'Name': '10 Retained points (optional)',
         'Enabled': False, 'Value': False, 'Topic': topic('/livox/lidar_costmap', best_effort=True),
         'Position Transformer': 'XYZ', 'Color Transformer': 'FlatColor',
         'Color': '0; 255; 120', 'Alpha': 1., 'Size (Pixels)': 3,
         'Size (m)': .03, 'Style': 'Points', 'Decay Time': 0., 'Use Fixed Frame': True},
        {'Class': 'rviz_default_plugins/Marker', 'Name': '11 Replay time and method',
         'Enabled': True, 'Value': True, 'Topic': topic('/review/status', True)}]
    template['Visualization Manager']['Displays'] = displays
    template['Visualization Manager']['Tools'] = [
        {'Class': 'rviz_default_plugins/Interact'}, {'Class': 'rviz_default_plugins/MoveCamera'},
        {'Class': 'rviz_default_plugins/Select'}, {'Class': 'rviz_default_plugins/Measure'},
        {'Class': 'rviz_default_plugins/SetGoal', 'Topic': topic('/review/goal_pose')}]
    template['Visualization Manager']['Global Options'] = {
        'Fixed Frame': 'map', 'Background Color': '45; 45; 45', 'Frame Rate': 20}
    template['Visualization Manager']['Views']['Current'] = {
        'Class': 'rviz_default_plugins/TopDownOrtho', 'Name': 'Current View',
        'Target Frame': '<Fixed Frame>', 'Scale': 65., 'X': float(center[0]), 'Y': float(center[1]),
        'Angle': 0., 'Near Clip Distance': .01}
    template['Panels'] = [{'Class': 'rviz_common/Displays', 'Name': 'Displays'},
                          {'Class': 'rviz_common/Views', 'Name': 'Views'}]
    template['Window Geometry'] = {'Width': 1500, 'Height': 950, 'X': 0, 'Y': 30}
    path.write_text(yaml.safe_dump(template, sort_keys=False))


class Review:
    def __init__(self, args, params_source, params):
        self.args = args
        if os.environ.get('ROS_DOMAIN_ID') != '89':
            raise RuntimeError('Use review_body_costmap.sh: this review requires isolated domain 89.')
        self.previous = WS / 'log/costmap_review_20261030_200954'
        self.validation = WS / 'log/body_height_validation_20261102'
        self.out = WS / 'log' / ('body_costmap_review_' + time.strftime('%Y%m%d_%H%M%S'))
        self.out.mkdir()
        self.params_path = self.out / 'review_params.yaml'
        self.params_path.write_text(yaml.safe_dump(params, sort_keys=False))
        filt = params['costmap_sector_filter']['ros__parameters']
        self.filter_settings = {k: filt[k] for k in FILTER_OPTIONS}
        self.filter_label = (
            f'BODY height [{filt["min_body_height"]:g}, {filt["max_body_height"]:g}] m | '
            f'front +/-{filt["front_half_angle_deg"]:g} deg >= {filt["front_min_range"]:g} m | '
            f'elsewhere >= {filt["other_min_range"]:g} m')
        (self.out / 'filter_settings.json').write_text(json.dumps({
            'source': str(params_source), 'review_only': True,
            'effective_filter': self.filter_settings}, indent=2))
        print('Review-only filter:', self.filter_label, flush=True)
        print('Parameter copy:', self.params_path, flush=True)
        meta = json.loads((self.previous / 'meta.json').read_text())
        self.epoch = meta['start_ns']
        series = np.load(self.previous / 'series.npz')
        localization = self.fill_constant_correction_gaps(series)
        self.poses = {}
        for name in ['/Odometry', '/localization']:
            a = localization if name == '/localization' else series[name]
            a = a[np.argsort(a[:, 1])]
            _, indices = np.unique(a[:, 1], return_index=True); a = a[indices]
            self.poses[name] = (a, Slerp(a[:, 1], Rotation.from_quat(a[:, 5:9])))
        self.up_world = np.load(self.validation / 'imu_up_200954.npz')['world_up']
        self.t = args.start
        self.playing = False
        self.pause_at = args.pause_at
        self.last_wall = time.monotonic()
        self.pose_index = 0
        self.seeded = False
        self.cloud_index = 0
        self.plan_index = 0
        self.old_index = {'local': 0, 'global': 0}
        self.processes = []; self.logs = []
        self.filtered_count = 0; self.local_count = 0; self.global_count = 0
        self.plan_status = 'Waiting for Nav2 activation'
        self.plan_pending = False; self.last_plan_wall = 0.
        self.plan_results = []
        self.request_plan = False
        self.last_status_wall = 0.
        self.goal = None
        self.load_bag()
        rclpy.init()
        self.node = rclpy.create_node('body_costmap_review')
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.clock_pub = self.node.create_publisher(Clock, '/clock', 10)
        self.tf_pub = self.node.create_publisher(TFMessage, '/tf', 100)
        self.static_pub = self.node.create_publisher(TFMessage, '/tf_static', transient)
        self.raw_pub = self.node.create_publisher(PointCloud2, '/livox/lidar_pointcloud', qos_profile_sensor_data)
        self.up_pub = self.node.create_publisher(Vector3Stamped, '/fastlio/up_body', qos_profile_sensor_data)
        self.odom_pub = self.node.create_publisher(Odometry, '/Odometry', 10)
        self.loc_pub = self.node.create_publisher(Odometry, '/localization', 10)
        self.old_pub = {k: self.node.create_publisher(OccupancyGrid, '/review/old_' + k, transient)
                        for k in ['local', 'global']}
        self.recorded_plan_pub = self.node.create_publisher(RosPath, '/review/recorded_plan', transient)
        self.new_plan_pub = self.node.create_publisher(RosPath, '/review/new_plan', transient)
        self.marker_pub = self.node.create_publisher(Marker, '/review/status', transient)
        self.plan_client = ActionClient(self.node, ComputePathToPose, '/compute_path_to_pose')
        self.subscriptions = [
            self.node.create_subscription(PointCloud2, '/livox/lidar_costmap', self.on_filtered, qos_profile_sensor_data),
            self.node.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_local, 1),
            self.node.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_global, 1),
            self.node.create_subscription(PoseStamped, '/review/goal_pose', self.on_goal, 10)]
        self.static_pub.publish(TFMessage(transforms=[
            self.tf('body', 'livox_frame', args.start, [-.011, .02329, -.04412], [0., 0., 0., 1.]),
            self.tf('body', 'BODY', args.start, [-.1105, .02329, .09338], [0., 0., 0., 1.]),
            self.tf('BODY', 'base_link', args.start, [0., 0., 0.], [0., 0., 0., 1.])]))
        self.publish_pose(args.start)
        self.clock_pub.publish(Clock(clock=self.stamp(self.t)))
        rviz_config(self.out / 'review.rviz', self.pose('/localization', args.start)[0])
        self.spawn('navigation', ['ros2', 'launch', 'pongbot_navigation', 'rubi_navigation_dwb.launch.py',
                   'use_sim_time:=true', 'dds_profile:=default', 'use_fastlio_velocity:=false',
                   'params_file:=' + str(self.params_path)])
        if not args.no_rviz:
            self.spawn('rviz', ['rviz2', '-d', str(self.out / 'review.rviz'),
                                '--ros-args', '-p', 'use_sim_time:=true'])
        print('Review output:', self.out, flush=True)

    def fill_constant_correction_gaps(self, series):
        """Recover missing fused poses only where recorded correction is constant.

        This uses actual recorded odometry samples, not interpolation of robot
        motion across the gap. Unknown/changing correction remains an error.
        """
        loc = series['/localization']; loc = loc[np.argsort(loc[:, 1])]
        odom = series['/Odometry']; correction = series['tf:map->camera_init']
        updates = series['/map_to_odom']
        additions = []; report = []
        for i in np.flatnonzero(np.diff(loc[:, 1]) > .35):
            lo, hi = loc[i, 1], loc[i + 1, 1]
            if hi < self.args.start or lo > self.args.end: continue
            a = correction[np.argmin(abs(correction[:, 1] - lo))]
            b = correction[np.argmin(abs(correction[:, 1] - hi))]
            if abs(a[1] - lo) > .001 or abs(b[1] - hi) > .001: continue
            rotation = Rotation.from_quat(a[5:9])
            if np.linalg.norm(a[2:5] - b[2:5]) > 1e-6: continue
            if (rotation.inv() * Rotation.from_quat(b[5:9])).magnitude() > 1e-6: continue
            if np.any((updates[:, 1] > lo) & (updates[:, 1] < hi)): continue
            samples = odom[(odom[:, 1] > lo) & (odom[:, 1] < hi)].copy()
            if not len(samples): continue
            stamps = np.sort(np.unique(np.r_[lo, samples[:, 1], hi]))
            if np.diff(stamps).max() > .25: continue
            samples[:, 2:5] = rotation.apply(samples[:, 2:5]) + a[2:5]
            samples[:, 5:9] = (rotation * Rotation.from_quat(samples[:, 5:9])).as_quat()
            additions.append(samples)
            report.append({'start_s': float(lo), 'end_s': float(hi), 'poses': len(samples),
                           'method': 'Recorded Odometry composed with verified unchanged map->camera_init; no recorded correction update inside gap.'})
        (self.out / 'pose_gap_reconstruction.json').write_text(json.dumps(report, indent=2))
        if report: print('Recorded-pose reconstruction:', report, flush=True)
        return np.vstack([loc, *additions]) if additions else loc

    def load_bag(self):
        bag = Path.home() / 'rosbag_record/rosbag2_2026_10_30-20_09_54'
        con = sqlite3.connect(next(bag.glob('*.db3')).as_uri() + '?mode=ro', uri=True)
        ids = dict(con.execute('select name,id from topics'))
        def rows(topic_name, lo, hi):
            return con.execute('select timestamp,data from messages where topic_id=? and timestamp>=? and timestamp<? order by timestamp',
                               (ids[topic_name], self.epoch + int(lo * 1e9), self.epoch + int(hi * 1e9)))
        self.clouds = []
        for _, blob in rows('/livox/lidar_pointcloud', self.args.start - 2, self.args.end + 3):
            msg = deserialize_message(blob, PointCloud2)
            t = (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec - self.epoch) / 1e9
            if self.args.start <= t <= self.args.end:
                self.clouds.append((t, msg))
        self.clouds.sort(key=lambda p: p[0])
        self.old_maps = {}
        for kind in ['local', 'global']:
            samples = []
            for ts, blob in rows('/' + kind + '_costmap/costmap', self.args.start - 5, self.args.end + 2):
                msg = deserialize_message(blob, OccupancyGrid)
                samples.append(((ts - self.epoch) / 1e9, msg))
            self.old_maps[kind] = samples
        self.plans = []
        for ts, blob in rows('/plan', 0., self.args.end + 1):
            msg = deserialize_message(blob, RosPath)
            if msg.poses:
                self.plans.append(((ts - self.epoch) / 1e9, msg))
        con.close()
        if not self.clouds or not self.plans:
            raise RuntimeError('No cloud/path samples in the requested window')
        # Larger gaps are allowed only explicitly for visual review; record
        # their intervals so they are not mistaken for estimator validation.
        self.interpolated_gaps = []
        for name, (a, _) in self.poses.items():
            mask = (a[:, 1] >= self.args.start - .4) & (a[:, 1] <= self.args.end + .4)
            stamps = a[mask, 1]; gaps = np.diff(stamps)
            gap = float(gaps.max())
            if gap > self.args.max_pose_gap:
                raise RuntimeError(f'{name} has {gap:.3f}s pose gap; allowed <= {self.args.max_pose_gap:.3f}s')
            for i in np.flatnonzero(gaps > .35):
                self.interpolated_gaps.append({'topic': name, 'start_s': float(stamps[i]),
                    'end_s': float(stamps[i + 1]), 'method': 'linear position / quaternion SLERP; visual review only'})
        (self.out / 'interpolated_pose_gaps.json').write_text(json.dumps(self.interpolated_gaps, indent=2))
        if self.interpolated_gaps: print('Visual-only interpolated intervals:', self.interpolated_gaps, flush=True)
        print(f'Loaded {len(self.clouds)} raw clouds; reconstructed up={self.up_world}', flush=True)

    def stamp(self, t):
        n = self.epoch + round(t * 1e9)
        return Time(sec=n // 10**9, nanosec=n % 10**9)

    def pose(self, name, t):
        a, interp = self.poses[name]
        return np.array([np.interp(t, a[:, 1], a[:, k]) for k in [2, 3, 4]]), interp(t)

    def tf(self, parent, child, t, p, q):
        m = TransformStamped(); m.header.frame_id = parent; m.child_frame_id = child
        m.header.stamp = self.stamp(t)
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, p)
        m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = map(float, q)
        return m

    def publish_pose(self, t, with_up=True):
        po, ro = self.pose('/Odometry', t); pl, rl = self.pose('/localization', t)
        correction = rl * ro.inv()
        self.tf_pub.publish(TFMessage(transforms=[
            self.tf('camera_init', 'body', t, po, ro.as_quat()),
            self.tf('map', 'camera_init', t, pl - correction.apply(po), correction.as_quat())]))
        up = Vector3Stamped(); up.header.frame_id = 'body'; up.header.stamp = self.stamp(t)
        up.vector.x, up.vector.y, up.vector.z = map(float, ro.inv().apply(self.up_world))
        if with_up:
            self.up_pub.publish(up)
        for name, pub, position, rotation in [('/Odometry', self.odom_pub, po, ro),
                                               ('/localization', self.loc_pub, pl, rl)]:
            msg = Odometry(); msg.header.frame_id = 'camera_init' if name == '/Odometry' else 'map'
            msg.header.stamp = self.stamp(t); msg.child_frame_id = 'body'
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, position)
            q = rotation.as_quat()
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = map(float, q)
            a = self.poses[name][0]
            values = [float(np.interp(t, a[:, 1], a[:, k])) for k in range(9, 15)]
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = values[:3]
            msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = values[3:]
            pub.publish(msg)

    def spawn(self, name, command):
        log = (self.out / (name + '.log')).open('w'); self.logs.append(log)
        p = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append(p)
        (self.out / (name + '.pid')).write_text(str(p.pid))

    def on_filtered(self, _): self.filtered_count += 1
    def on_local(self, _): self.local_count += 1
    def on_global(self, _): self.global_count += 1

    def on_goal(self, msg):
        if msg.header.frame_id != 'map':
            self.plan_status = 'Review goal must be in map frame'; return
        self.goal = copy.deepcopy(msg); self.request_plan = True

    def start_plan(self):
        if self.plan_pending or self.goal is None or not self.plan_client.server_is_ready(): return
        self.plan_pending = True; self.last_plan_wall = time.monotonic(); self.request_plan = False
        self.plan_status = 'Computing current A* (no FollowPath)'
        req = ComputePathToPose.Goal(); req.goal = copy.deepcopy(self.goal)
        req.goal.header.stamp = Time(); req.planner_id = 'GridBased'; req.use_start = False
        future = self.plan_client.send_goal_async(req)
        future.add_done_callback(self.on_plan_accepted)

    def on_plan_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.plan_pending = False; self.plan_status = 'A* request rejected'; return
        handle.get_result_async().add_done_callback(self.on_plan_result)

    def on_plan_result(self, future):
        self.plan_pending = False
        response = future.result(); path = response.result.path
        ok = response.status == 4 and bool(path.poses)
        self.plan_status = f'A*: {"SUCCESS" if ok else "FAILED"}, {len(path.poses)} poses'
        # Clear any previous success on failure so a stale path is not misleading.
        if not ok: path = RosPath(); path.header.frame_id = 'map'
        self.new_plan_pub.publish(path)
        record = {'t': self.t, 'success': ok, 'points': len(path.poses)}
        self.plan_results.append(record); print(record, flush=True)
        (self.out / 'plans.json').write_text(json.dumps(self.plan_results, indent=2))

    def tick(self):
        now = time.monotonic(); dt = now - self.last_wall; self.last_wall = now
        if self.playing:
            target = min(self.args.end, self.t + min(dt, .1) * self.args.rate)
            if self.pause_at is not None and self.t < self.pause_at <= target:
                target = self.pause_at; self.pause_at = None; self.playing = False
                self.request_plan = True
            self.t = target
            if self.t >= self.args.end: self.playing = False
        self.clock_pub.publish(Clock(clock=self.stamp(self.t)))
        if not self.seeded and self.up_pub.get_subscription_count():
            # The filter is launched after this player. Seed interpolation only
            # once its reader has been discovered, before releasing clouds.
            for t in [self.args.start - .2, self.args.start - .1, self.args.start]:
                self.publish_pose(t)
            self.seeded = True; self.pose_index = 1
        # Bracketing up samples precede every cloud; no stale/latest-pose fallback.
        while self.args.start + self.pose_index * .1 <= self.t + 1e-7:
            self.publish_pose(self.args.start + self.pose_index * .1); self.pose_index += 1
        while self.cloud_index < len(self.clouds) and self.clouds[self.cloud_index][0] + .12 <= self.t:
            self.raw_pub.publish(self.clouds[self.cloud_index][1]); self.cloud_index += 1
        for kind, rows in self.old_maps.items():
            latest = None
            while self.old_index[kind] < len(rows) and rows[self.old_index[kind]][0] <= self.t:
                latest = rows[self.old_index[kind]][1]; self.old_index[kind] += 1
            if latest is not None: self.old_pub[kind].publish(latest)
        latest = None
        while self.plan_index < len(self.plans) and self.plans[self.plan_index][0] <= self.t:
            latest = self.plans[self.plan_index][1]; self.plan_index += 1
        if latest is not None:
            # Displaying static map geometry: path timestamps in this bag are zero.
            self.recorded_plan_pub.publish(latest)
            if self.goal is None:
                self.goal = copy.deepcopy(latest.poses[-1]); self.goal.header.frame_id = 'map'
        if self.local_count and self.global_count and (self.request_plan or
                (self.playing and now - self.last_plan_wall >= 2.)):
            self.start_plan()
        if now - self.last_status_wall >= 1.:
            self.last_status_wall = now
            if self.t == self.args.start and not self.playing:
                self.publish_pose(self.t, with_up=False)
            m = Marker(); m.header.frame_id = 'map'; m.header.stamp = Time()
            m.ns = 'review'; m.id = 0; m.type = Marker.TEXT_VIEW_FACING; m.action = Marker.ADD
            m.pose.position.x = 9.5; m.pose.position.y = 44.5; m.pose.position.z = 1.
            m.pose.orientation.w = 1.; m.scale.z = .18
            m.color.r = 1.; m.color.g = 1.; m.color.b = 1.; m.color.a = 1.
            m.text = (f'OFFLINE t={self.t:.1f}s  {"PLAY" if self.playing else "PAUSED"}\n'
                      'Recorded pose + reconstructed up; controlled delivery\n' + self.plan_status)
            self.marker_pub.publish(m)
            (self.out / 'status.json').write_text(json.dumps({
                'bag_t': self.t, 'playing': self.playing, 'input': self.cloud_index,
                'filtered': self.filtered_count, 'local_grids': self.local_count,
                'global_grids': self.global_count, 'planner': self.plan_status,
                'filter': self.filter_settings}, indent=2))
        for _ in range(15): rclpy.spin_once(self.node, timeout_sec=0.)

    def close(self):
        for p in self.processes:
            if p.poll() is None: os.killpg(p.pid, signal.SIGINT)
        for p in self.processes:
            try: p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGTERM)
                try: p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL); p.wait()
        for log in self.logs: log.close()
        self.node.destroy_node(); rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', type=float, default=170.)
    parser.add_argument('--end', type=float, default=240.)
    parser.add_argument('--rate', type=float, default=1.)
    parser.add_argument('--pause-at', type=float, default=195.)
    parser.add_argument('--max-pose-gap', type=float, default=.35,
                        help='maximum recorded pose bracket in seconds; >0.35 is visual review only')
    parser.add_argument('--autoplay', action='store_true')
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--headless-seconds', type=float, default=0., help='bounded smoke test')
    parser.add_argument('--params-file', help='base Nav2 YAML; defaults to the installed RUBI YAML (copied, never modified)')
    for name, help_text in FILTER_OPTIONS.items():
        parser.add_argument('--' + name.replace('_', '-'), type=float,
                            help=help_text + '; review-only override, default from YAML')
    args = parser.parse_args()
    if not (0 < args.rate <= 1 and 0 <= args.start < args.end and 0 < args.max_pose_gap <= 1.):
        parser.error('invalid time/rate/pose-gap limit')
    try:
        params_source, params = review_parameters(args)
    except (OSError, ValueError, KeyError, TypeError, OverflowError, yaml.YAMLError) as error:
        parser.error(f'invalid review parameters: {error}')
    lock = (WS / 'log/body_costmap_review.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('A review is already running. Close its control window first.')
    review = Review(args, params_source, params)
    try:
        ready_wall = time.monotonic()
        def run_tick():
            review.tick()
            if args.autoplay and not review.local_count and time.monotonic() - ready_wall > 12:
                review.playing = True
            elif args.autoplay and review.local_count and review.t == args.start:
                review.playing = True
        if args.headless_seconds:
            deadline = time.monotonic() + args.headless_seconds
            while time.monotonic() < deadline:
                run_tick(); time.sleep(.01)
        else:
            import tkinter as tk
            root = tk.Tk(); root.title('BODY height review - recorded geometry - domain 89')
            root.geometry('760x320+1100+700')
            # Tk catches KeyboardInterrupt in callbacks; explicitly exit its loop
            # so the finally block always terminates the launched Nav2 children.
            signal.signal(signal.SIGINT, lambda *_: root.after_idle(root.destroy))
            signal.signal(signal.SIGTERM, lambda *_: root.after_idle(root.destroy))
            status = tk.StringVar()
            tk.Label(root, text=f'Recorded 20_09_54 bag | {args.start:g}-{args.end:g}s corridor',
                     font=('', 14, 'bold')).pack(pady=6)
            tk.Label(root, text='Recorded pose + reconstructed gravity. Original transport delays are removed.\n'
                     'RViz: NEW Local / OLD Local checkboxes. Cyan = recorded route; yellow = new A*.').pack()
            tk.Label(root, text=review.filter_label, fg='#155070').pack(pady=4)
            tk.Label(root, text='Filter settings apply to this review only. Close and restart to compare another value.').pack()
            if review.interpolated_gaps:
                tk.Label(root, text=f'VISUAL ONLY: {len(review.interpolated_gaps)} pose gaps interpolated '
                         f'(limit {args.max_pose_gap:.2f}s).', fg='#a04000').pack()
            tk.Label(root, textvariable=status, justify='left', font=('monospace', 11)).pack(pady=8)
            row = tk.Frame(root); row.pack()
            def toggle():
                review.playing = not review.playing; review.last_wall = time.monotonic()
            def step():
                review.playing = False; review.t = min(args.end, review.t + .2)
            tk.Button(row, text='Play / Pause', width=17, command=toggle).pack(side='left', padx=4)
            tk.Button(row, text='Step +0.2s', width=17, command=step).pack(side='left', padx=4)
            tk.Button(row, text='Recompute A* only', width=20,
                      command=lambda: setattr(review, 'request_plan', True)).pack(side='left', padx=4)
            tk.Label(root, text='2D Goal Pose in this RViz requests A* only. No FollowPath / LCM.\n'
                     'Close this control window to stop the whole review.').pack(pady=8)
            raised_at_pause = [False]
            def update():
                run_tick()
                if not review.playing and review.t > args.start and not raised_at_pause[0]:
                    root.lift(); raised_at_pause[0] = True
                status.set(f't={review.t:.1f}s  {"PLAYING" if review.playing else "PAUSED"}  '
                           f'clouds {review.cloud_index} -> {review.filtered_count}\n'
                           f'Local grids={review.local_count}  Global grids={review.global_count}\n'
                           + review.plan_status)
                root.after(15, update)
            root.after(15, update); root.mainloop()
    finally:
        review.close()
        lock.close()


if __name__ == '__main__':
    main()
