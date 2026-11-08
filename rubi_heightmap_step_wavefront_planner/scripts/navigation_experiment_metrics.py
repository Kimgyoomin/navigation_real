#!/usr/bin/env python3
"""One explicitly bounded navigation trial; never publishes robot commands."""
import json
import math
import re
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from navigation_metrics_core import calculate, joint_powers, start_readiness_error, JOINTS


class Metrics(Node):
    def __init__(self):
        super().__init__('navigation_experiment_metrics')
        defaults = dict(run_id='unspecified', scenario='unspecified', cost_mode='vanilla',
                        output_dir=str(Path.home()/'cot_experiment/navigation_cost_comparison'),
                        joint_topic='/rubi_gazebo_legacy/joint_states_500hz',
                        odom_topic='/ground_truth/odom', mass_kg=28.778101, gravity_m_s2=9.8,
                        joint_max_gap_s=0.05, odom_max_gap_s=0.10, map_manifest_json='{}')
        self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', self.p['run_id']) or self.p['run_id'] in ('.', '..'):
            raise ValueError('run_id must be a simple filename component')
        self.directory = Path(self.p['output_dir']).expanduser()/self.p['run_id']
        self.directory.mkdir(parents=True, exist_ok=False)  # Never overwrite another run.
        manifest = json.loads(self.p['map_manifest_json'])
        (self.directory/'map_manifest.json').write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
        self.events = (self.directory/'events.jsonl').open('x')
        self.joints, self.odom, self.plans = [], [], []
        self.last_receipt = {}
        self.errors, self.odom_frames = set(), set()
        self.start_time = self.end_time = None
        self.finished = False
        self.finish_deadline = 0.0
        self.outcome = None
        output_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.result_pub = self.create_publisher(String, '/rubi/experiment/run_metrics', output_qos)
        self.event_pub = self.create_publisher(String, '/rubi/experiment/events', output_qos)
        self.power_pub = self.create_publisher(Float64, '/rubi/experiment/power_abs_W', 10)
        self.signed_pub = self.create_publisher(Float64, '/rubi/experiment/power_signed_W', 10)
        self.create_subscription(JointState, self.p['joint_topic'], self.on_joint, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p['odom_topic'], self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, '/rubi/experiment/plan_metrics', self.on_plan, output_qos)
        self.create_service(Trigger, '/rubi/experiment/start', self.start)
        self.create_service(Trigger, '/rubi/experiment/finish_success',
                            lambda req, res: self.finish(req, res, 'success'))
        self.create_service(Trigger, '/rubi/experiment/finish_failure',
                            lambda req, res: self.finish(req, res, 'failure'))
        self.create_timer(0.05, self.poll_finish)
        self.emit('ready', parameters=self.p, joints=list(JOINTS))

    def now_s(self):
        return self.get_clock().now().nanoseconds/1e9

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec/1e9

    def emit(self, kind, **fields):
        data = dict(event=kind, run_id=self.p['run_id'], stamp_s=self.now_s(), **fields)
        text = json.dumps(data, allow_nan=False)
        self.events.write(text+'\n')
        self.events.flush()
        self.event_pub.publish(String(data=text))

    def append(self, target, stamp, values):
        if self.finished:
            return False
        if not math.isfinite(stamp) or not all(math.isfinite(x) for x in values):
            if self.start_time is not None:
                self.errors.add('nonfinite_sample')
            return False
        if target and stamp <= target[-1][0]:
            if self.start_time is not None:
                self.errors.add('duplicate_or_reversed_header_time')
                return False
            target.clear()
        target.append((stamp, values))
        if self.start_time is None:
            # /clock can update at 10 Hz while joints arrive at 500 Hz.
            # Keep a boundary bracket even when the newest header leads /clock.
            while len(target) > 2 and target[1][0] < stamp - 1.0:
                del target[0]
        return True

    def on_joint(self, msg):
        try:
            powers = joint_powers(list(msg.name), msg.effort, msg.velocity)
        except ValueError as error:
            if self.start_time is not None:
                self.errors.add(str(error))
            return
        if self.append(self.joints, self.stamp(msg), powers):
            self.last_receipt['joint'] = time.monotonic()
        self.power_pub.publish(Float64(data=sum(abs(p) for p in powers)))
        self.signed_pub.publish(Float64(data=sum(powers)))

    def on_odom(self, msg):
        if self.start_time is not None and not self.finished:
            self.odom_frames.add((msg.header.frame_id, msg.child_frame_id))
        else:
            self.odom_frames = {(msg.header.frame_id, msg.child_frame_id)}
        if self.append(self.odom, self.stamp(msg), (msg.pose.pose.position.x, msg.pose.pose.position.y)):
            self.last_receipt['odom'] = time.monotonic()

    def on_plan(self, msg):
        data = json.loads(msg.data)
        if data.get('run_id') != self.p['run_id'] or self.finished:
            return
        self.emit('plan', plan=data)
        if self.start_time is not None and data.get('stamp_s', -1) >= self.start_time and (
                self.end_time is None or data.get('stamp_s', -1) <= self.end_time):
            if data.get('cost_mode') != self.p['cost_mode']:
                self.errors.add('planner_cost_mode_mismatch')
            self.plans.append(data)

    def start(self, request, response):
        stamp = self.now_s()
        if self.start_time is not None or self.finished:
            response.message = 'This launch records one trial; use a new run_id and restart.'
            return response
        now = time.monotonic()
        error = start_readiness_error(self.joints, self.odom, stamp,
            {key: now-value for key, value in self.last_receipt.items()})
        if error:
            response.message = error
            return response
        self.start_time = stamp
        self.emit('start', boundary='manual_start_service_before_goal',
                  start_s=stamp, prestart_buffer_s=1.0, max_clock_header_skew_s=0.25)
        response.success, response.message = True, 'Started; now publish the navigation goal.'
        return response

    def finish(self, request, response, outcome):
        if self.start_time is None or self.end_time is not None or self.finished:
            response.message = 'No active unfinished trial.'
            return response
        self.end_time, self.outcome = self.now_s(), outcome
        self.finish_deadline = time.monotonic()+3.0
        self.emit('finish_requested', outcome=outcome, outcome_source='operator_service')
        response.success = True
        response.message = 'End time fixed; waiting for boundary coverage. Read run_metrics for QC.'
        return response

    def poll_finish(self):
        if self.finished or self.end_time is None:
            return
        covered = all(s and s[-1][0] >= self.end_time for s in (self.joints, self.odom))
        if not covered and time.monotonic() < self.finish_deadline:
            return
        if len(self.odom_frames) != 1 or any(not f[0] or not f[1] for f in self.odom_frames):
            self.errors.add('odom_frame_changed')
        measurement = None
        try:
            measurement = calculate(self.joints, self.odom, self.start_time, self.end_time,
                self.p['mass_kg'], self.p['gravity_m_s2'],
                self.p['joint_max_gap_s'], self.p['odom_max_gap_s'])
        except ValueError as error:
            self.errors.add(str(error))
        accepted = [p for p in self.plans if p.get('accepted')]
        if not accepted:
            self.errors.add('no_accepted_plan_in_trial')
        timings = [p['total_request_time_ms'] for p in self.plans if 'total_request_time_ms' in p]
        result = dict(schema_version=1, parameters=self.p, outcome=self.outcome,
                      outcome_source='operator_service', start_s=self.start_time, end_s=self.end_time,
                      qc_pass=not self.errors, qc_errors=sorted(self.errors),
                      metrics=measurement if not self.errors else None,
                      plan_attempts=len(self.plans), accepted_plan_count=len(accepted),
                      accepted_replans=sum(bool(p.get('automatic_replan')) for p in accepted),
                      initial_plan=accepted[0] if accepted else None,
                      latest_plan=accepted[-1] if accepted else None,
                      planning_ms_mean=sum(timings)/len(timings) if timings else None,
                      planning_ms_max=max(timings) if timings else None,
                      odom_frames=[list(f) for f in self.odom_frames])
        (self.directory/'result.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        self.result_pub.publish(String(data=json.dumps(result, allow_nan=False)))
        self.emit('finished', qc_pass=result['qc_pass'], outcome=self.outcome)
        self.finished = True
        self.get_logger().info(f'Result: {self.directory}/result.json; QC={result["qc_pass"]}')


def main():
    rclpy.init()
    node = Metrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.start_time is not None and not node.finished:
            node.emit('interrupted_without_final_result')
    finally:
        node.events.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
