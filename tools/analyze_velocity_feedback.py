#!/usr/bin/python3
"""Offline SQLite bag analysis. No ROS initialization, nodes, or publications.

Command derivatives use recorder time (Twist has no Header); estimator checks
use acquisition time. Pose derivatives are internal consistency checks, not
independent ground truth. Never interpolate across an estimator gap >0.16 s.
"""
import argparse
import csv
import json
import sqlite3
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict


def stats(a):
    a = np.asarray(a)
    a = a[np.isfinite(a)]
    return dict(zip(('min', 'p50', 'p95', 'p99', 'max'),
                    np.percentile(a, [0, 50, 95, 99, 100]).tolist())) if a.size else None


def vector(v):
    return [v.x, v.y, v.z]


def analyze(bag, out):
    arrays, frames, transforms, events, logs = {}, {}, {}, [], []
    start = None
    topics = ['/cmd_vel', '/Odometry', '/fastlio/odometry_velocity', '/livox/imu',
              '/tf_static', '/parameter_events', '/rosout']
    for db in sorted(bag.glob('*.db3')):
        with sqlite3.connect(db.resolve().as_uri() + '?mode=ro', uri=True) as con:
            con.execute('PRAGMA query_only=ON')
            first = con.execute('SELECT min(timestamp) FROM messages').fetchone()[0]
            start = first if start is None else min(first, start)
            for ident, name, typ in con.execute('SELECT id,name,type FROM topics'):
                if name not in topics:
                    continue
                cls = get_message(typ)
                for receipt, blob in con.execute(
                        'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (ident,)):
                    m = deserialize_message(blob, cls)
                    t = (receipt - start) * 1e-9
                    if hasattr(m, 'header'):
                        stamp = (m.header.stamp.sec * 10**9 + m.header.stamp.nanosec - start) * 1e-9
                    if typ == 'geometry_msgs/msg/Twist':
                        row = [t, *vector(m.linear), *vector(m.angular)]
                    elif typ == 'nav_msgs/msg/Odometry':
                        p, q = m.pose.pose.position, m.pose.pose.orientation
                        row = [t, stamp, *vector(p), q.x, q.y, q.z, q.w,
                               *vector(m.twist.twist.linear), *vector(m.twist.twist.angular)]
                        frames.setdefault(name, set()).add((m.header.frame_id, m.child_frame_id))
                    elif typ == 'sensor_msgs/msg/Imu':
                        row = [t, stamp, *vector(m.angular_velocity)]
                    elif typ == 'tf2_msgs/msg/TFMessage':
                        for tf in m.transforms:
                            key = tf.header.frame_id + '->' + tf.child_frame_id
                            q = tf.transform.rotation
                            value = [*vector(tf.transform.translation), q.x, q.y, q.z, q.w]
                            if key in transforms and not np.allclose(transforms[key], value):
                                raise ValueError('Static transform changed: ' + key)
                            transforms[key] = value
                        continue
                    elif typ == 'rcl_interfaces/msg/ParameterEvent':
                        events.append({'t': t, 'event': message_to_ordereddict(m)})
                        continue
                    elif typ == 'rcl_interfaces/msg/Log':
                        logs.append({'t': t, 'node': m.name, 'level': m.level, 'msg': m.msg})
                        continue
                    arrays.setdefault(name, []).append(row)
    a = {k: np.array(v) for k, v in arrays.items()}
    out.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(out / 'series.npz', **a)
    report = {'bag': str(bag), 'start_ns': start, 'frames': {k: list(v) for k, v in frames.items()},
              'static_tf': transforms, 'topics': {}}
    for name, v in a.items():
        s = {'count': len(v), 'hz': (len(v)-1)/(v[-1, 0]-v[0, 0]),
             'receipt_gap_s': stats(np.diff(v[:, 0]))}
        if name != '/cmd_vel':
            s.update(age_ms=stats((v[:, 0]-v[:, 1])*1000),
                     nonincreasing_stamps=int(sum(np.diff(v[:, 1]) <= 0)),
                     stamp_gaps_over_160ms=int(sum(np.diff(v[:, 1]) > .16)))
        if 'Odometry' in name or 'odometry' in name:
            s.update(twist_abs_max=np.max(np.abs(v[:, 9:15]), axis=0).tolist())
        report['topics'][name] = s

    cmd = a['/cmd_vel']
    dt = np.diff(cmd[:, 0])
    regular = (dt >= .10) & (dt <= .35)
    rates = np.diff(cmd[:, [1, 6]], axis=0) / dt[:, None]
    report['command'] = {'vx': stats(cmd[:, 1]), 'wz': stats(cmd[:, 6]),
                         'regular_intervals': int(sum(regular)),
                         'regular_dvx_dt': stats(rates[regular, 0]),
                         'regular_dwz_dt': stats(rates[regular, 1]),
                         'regular_vx_rises_over_0_5': int(sum(regular & (rates[:, 0] > .5 + .01))),
                         'regular_yaw_abs_rate_over_0_5': int(sum(regular & (abs(rates[:, 1]) > .5 + .01)))}
    with (out / 'command_steps.csv').open('w') as f:
        w = csv.writer(f)
        w.writerow(['t_prev_s', 't_s', 'dt_s', 'vx_prev', 'vx', 'wz_prev', 'wz', 'dvx_dt', 'dwz_dt', 'regular_interval'])
        for i in range(1, len(cmd)):
            w.writerow([cmd[i-1, 0], cmd[i, 0], dt[i-1], cmd[i-1, 1], cmd[i, 1],
                        cmd[i-1, 6], cmd[i, 6], *rates[i-1], bool(regular[i-1])])
    idx = np.flatnonzero(regular)
    report['command']['largest_regular_acceleration_steps'] = [
        {'t_s': float(cmd[i+1, 0]), 'dt_s': float(dt[i]), 'vx_before': float(cmd[i, 1]),
         'vx_after': float(cmd[i+1, 1]), 'dvx_dt': float(rates[i, 0])}
        for i in idx[np.argsort(rates[idx, 0])[-8:][::-1]]]

    v = a['/fastlio/odometry_velocity']
    # Record-time and sample-time axes are intentionally kept distinct.
    order = np.argsort(v[:, 1], kind='stable')
    v = v[order]
    rot = Rotation.from_quat(v[:, 5:9])
    comparisons = []
    for i in range(2, len(v)-2):
        window = v[i-2:i+3]
        intervals = np.diff(window[:, 1])
        if np.any(intervals <= 0) or np.any(intervals > .16):
            continue
        times = window[:, 1] - v[i, 1]
        design = np.stack([times, np.ones(len(times))], axis=1)
        world_v = np.linalg.lstsq(design, window[:, 2:5], rcond=None)[0][0]
        pose_v = rot[i].inv().apply(world_v)
        relative = rot[i].inv() * rot[i-2:i+3]
        pose_w = np.linalg.lstsq(design, relative.as_rotvec(), rcond=None)[0][0]
        native_avg = np.mean(window[:, 9:15], axis=0)
        comparisons.append([v[i, 1], *pose_v, *pose_w, *native_avg])
    compare = np.array(comparisons)
    np.savetxt(out / 'pose_velocity_check.csv', compare, delimiter=',',
               header='stamp_s,pose_vx,pose_vy,pose_vz,pose_wx,pose_wy,pose_wz,est_vx,est_vy,est_vz,est_wx,est_wy,est_wz')
    error = compare[:, 7:13] - compare[:, 1:7]
    report['internal_consistency_5_sample_fit'] = {
        'samples': len(compare), 'rmse_vx_vy_vz_wx_wy_wz': np.sqrt(np.mean(error**2, axis=0)).tolist(),
        'p95_abs_error': np.percentile(abs(error), 95, axis=0).tolist(),
        'correlations': [float(np.corrcoef(compare[:, 1+i], compare[:, 7+i])[0, 1]) for i in range(6)],
        'note': 'Same estimator pose/twist; not external ground truth. 5 samples ~0.4s span.'}
    fit_indices = np.searchsorted(v[:, 1], compare[:, 0])
    instant_error = v[fit_indices, 9:15] - compare[:, 1:7]
    report['instant_twist_vs_pose_fit'] = {
        'rmse': np.sqrt(np.mean(instant_error**2, axis=0)).tolist(),
        'p95_abs_error': np.percentile(abs(instant_error), 95, axis=0).tolist(),
        'note': 'Instant native output vs ~0.4s pose fit; includes vibration and smoothing differences.'}
    stationary = (np.linalg.norm(compare[:, 1:4], axis=1) < .03) & (np.linalg.norm(compare[:, 4:7], axis=1) < .03)
    report['pose_stationary_windows'] = {'count': int(sum(stationary)),
        'native_abs_vx': stats(abs(compare[stationary, 7])), 'native_abs_wz': stats(abs(compare[stationary, 12]))}
    # Angular lever-arm correction: velocity at base origin expressed in body.
    # This bag's static transforms have identity rotations; reject other mounts.
    b = transforms['body->BODY']; end = transforms['BODY->base_link']
    if not (np.allclose(b[3:], [0, 0, 0, 1]) and np.allclose(end[3:], [0, 0, 0, 1])):
        raise ValueError('Analysis expects identity static mount rotations; implement general composition')
    arm = np.array(b[:3]) + np.array(end[:3])
    corrected = v[:, 9:12] + np.cross(v[:, 12:15], arm)
    report['base_lever_arm_m_in_body'] = arm.tolist()
    report['base_velocity_correction_abs_max'] = np.max(abs(corrected-v[:, 9:12]), axis=0).tolist()
    # Estimate physical linear acceleration by differentiating velocity in the
    # fixed world frame, then express it at the center sample's base axes.
    # Direct d(v_body)/dt would omit omega cross v and is not the same quantity.
    world_velocity = rot.apply(corrected)
    world_angular = rot.apply(v[:, 12:15])
    accelerations = []
    for i in fit_indices:
        times = v[i-2:i+3, 1] - v[i, 1]
        design = np.stack([times, np.ones(5)], axis=1)
        linear_acc = rot[i].inv().apply(np.linalg.lstsq(design, world_velocity[i-2:i+3], rcond=None)[0][0])
        angular_acc = rot[i].inv().apply(np.linalg.lstsq(design, world_angular[i-2:i+3], rcond=None)[0][0])
        accelerations.append([v[i, 1], *linear_acc, *angular_acc])
    acceleration = np.array(accelerations)
    np.savetxt(out / 'estimated_acceleration.csv', acceleration, delimiter=',',
               header='stamp_s,base_ax,base_ay,base_az,base_alphax,base_alphay,base_alphaz')
    report['estimated_acceleration_5_sample_fit'] = {
        'ax_m_s2': stats(acceleration[:, 1]), 'alphaz_rad_s2': stats(acceleration[:, 6]),
        'note': 'Differentiated FAST-LIO velocity over ~0.4s; not independently measured acceleration; manual intervals not excluded.'}
    # Compare commands to latest available measured velocity at the recorded command time.
    # Do not turn a desired command into measurement truth, and do not label manual intervals autonomous.
    receipt_v = a['/fastlio/odometry_velocity']
    corrected_receipt = receipt_v[:, 9:12] + np.cross(receipt_v[:, 12:15], arm)
    j = np.searchsorted(receipt_v[:, 0], cmd[:, 0], side='right') - 1
    valid = j >= 0
    j = np.maximum(j, 0)
    ages = cmd[:, 0] - receipt_v[j, 1]
    valid &= (ages >= 0) & (ages <= .25)
    matching = np.column_stack([cmd[:, 0], cmd[:, 1], cmd[:, 6], corrected_receipt[j, 0], receipt_v[j, 14], ages, valid])
    np.savetxt(out / 'command_feedback.csv', matching, delimiter=',',
               header='cmd_receipt_s,cmd_vx,cmd_wz,base_est_vx,base_est_wz,measurement_age_s,valid_under_250ms')
    report['command_feedback'] = {'valid_pairs': int(sum(valid)), 'total_commands': len(cmd),
        'vx_difference': stats(cmd[valid, 1] - corrected_receipt[j[valid], 0]),
        'wz_difference': stats(cmd[valid, 6] - receipt_v[j[valid], 14]),
        'note': 'Tracking difference, not estimator error; navigation flag and LCM TX/RX absent.'}
    (out / 'summary.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
    (out / 'parameters.json').write_text(json.dumps(events, indent=2, ensure_ascii=False))
    (out / 'logs.json').write_text(json.dumps(logs, indent=2, ensure_ascii=False))

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    ax[0].step(cmd[:, 0], cmd[:, 1], where='post', label='cmd vx (recorder time)', alpha=.8)
    ax[0].plot(v[:, 1], corrected[:, 0], label='FAST-LIO base vx (sample time)', lw=.8)
    ax[0].plot(compare[:, 0], compare[:, 1], label='body pose derivative (~0.4s)', lw=.7, alpha=.6)
    ax[0].set_ylabel('m/s'); ax[0].legend(loc='upper right')
    ax[1].step(cmd[:, 0], cmd[:, 6], where='post', label='cmd wz', alpha=.8)
    ax[1].plot(v[:, 1], v[:, 14], label='FAST-LIO wz', lw=.7)
    ax[1].plot(compare[:, 0], compare[:, 6], label='pose angular derivative (~0.4s)', lw=.7)
    ax[1].set_ylabel('rad/s'); ax[1].legend(loc='upper right')
    ax[2].plot(cmd[1:, 0][regular], rates[regular, 0], '.', label='command dvx/dt')
    ax[2].plot(cmd[1:, 0][regular], rates[regular, 1], '.', label='command dwz/dt', alpha=.5)
    ax[2].plot(acceleration[:, 0], acceleration[:, 1], lw=.7, label='estimated base ax (~0.4s fit)')
    ax[2].axhline(.5, color='red', ls='--', label='current 0.5 reference (NOT bag config proof)')
    ax[2].axhline(-.5, color='red', ls='--'); ax[2].set_ylabel('command rate /s'); ax[2].legend(loc='upper right')
    ax[3].plot(v[:, 0], (v[:, 0]-v[:, 1])*1000, label='estimator receipt minus stamp')
    ax[3].set_ylabel('age ms'); ax[3].set_xlabel('Seconds from bag start'); ax[3].legend(loc='upper right')
    for axis in ax: axis.grid(alpha=.3)
    fig.suptitle(bag.name + ' | Offline comparison; no independent motion ground truth')
    fig.tight_layout(); fig.savefig(out / 'velocity_comparison.png', dpi=150); plt.close(fig)
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag', type=Path)
    p.add_argument('--output', required=True, type=Path)
    args = p.parse_args()
    if args.output.resolve().is_relative_to(args.bag.resolve()):
        p.error('Output must be outside the source bag')
    analyze(args.bag, args.output)
