#!/usr/bin/python3
"""Read SQLite ROS 2 bags offline; never initialize ROS or replay/publish messages.

CustomMsg's scan-start header must be distinguished from its last point time.
Output uses recorder timestamps, not a direct measurement of DDS transport alone.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import sqlite3
import struct

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    '/livox/lidar', '/livox/imu', '/livox/lidar_pointcloud', '/livox/lidar_costmap',
    '/Odometry', '/localization', '/fastlio/odometry_velocity', '/cloud_registered',
    '/cloud_pcd', '/local_costmap/costmap', '/global_costmap/costmap',
}


def cdr_header(blob):
    """ROS 2 XCDR1 Header at the beginning of a message; retain integer ns."""
    if blob[:2] not in (b'\x00\x00', b'\x00\x01'):
        raise ValueError('Expected plain XCDR1 encoding')
    endian = '<' if blob[1] == 1 else '>'
    sec, nsec, length = struct.unpack_from(endian + 'iII', blob, 4)
    end = 16 + length
    if not length or end > len(blob) or blob[end - 1] != 0 or nsec >= 10**9:
        raise ValueError('Malformed ROS Header')
    return sec * 10**9 + nsec, end, endian


def custom_scan(blob):
    """Vectorized offset read for this repository's livox_ros_driver2/CustomMsg.

    Each CustomPoint is uint32 + xyz float32 + three uint8 (20-byte CDR stride).
    The first recorded message is also checked against ROS deserialization.
    """
    stamp, offset, endian = cdr_header(blob)
    offset = 4 + ((offset - 4 + 7) // 8) * 8
    timebase, declared = struct.unpack_from(endian + 'QI', blob, offset)
    count = struct.unpack_from(endian + 'I', blob, offset + 16)[0]
    points_start = offset + 20
    if count != declared or not count:
        raise ValueError('Empty or inconsistent CustomMsg point count')
    if len(blob) != points_start + (count - 1) * 20 + 19:
        raise ValueError('Unexpected CustomPoint CDR layout; refusing to guess')
    offsets = np.ndarray((count,), dtype=endian + 'u4', buffer=blob,
                         offset=points_start, strides=(20,))
    return stamp, stamp + int(offsets.max()), int(offsets[-1]), count, timebase


def percentiles(values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return dict(zip(('min', 'p50', 'p95', 'p99', 'max'),
                    np.percentile(values, [0, 50, 95, 99, 100]).tolist()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path, help='bag directory or .db3 file')
    parser.add_argument('--output', required=True, type=Path, help='new output directory')
    parser.add_argument('--label', default='unspecified', help='operator-supplied configuration label')
    args = parser.parse_args()
    files = [args.bag] if args.bag.is_file() else sorted(args.bag.glob('*.db3'))
    if not files or any(p.suffix != '.db3' for p in files):
        parser.error('Expected SQLite .db3 files; MCAP is not supported by this tool')
    # Avoid overwriting a previous comparison or writing into the input bag.
    input_dir = args.bag.resolve().parent if args.bag.is_file() else args.bag.resolve()
    if args.output.resolve().is_relative_to(input_dir) or args.output.exists():
        parser.error('--output must be a new directory outside the input bag')
    rows = {}
    diagnostics = {}
    validated_custom = False
    log_cls = get_message('rcl_interfaces/msg/Log')
    for file in files:
        print('Reading offline:', file, flush=True)
        with sqlite3.connect(file.resolve().as_uri() + '?mode=ro', uri=True) as con:
            con.execute('PRAGMA query_only=ON')
            for topic_id, topic, typ in con.execute('SELECT id,name,type FROM topics'):
                if topic not in TOPICS and topic != '/rosout':
                    continue
                custom = typ == 'livox_ros_driver2/msg/CustomMsg'
                # Large PCD/costmap data need not be deserialized or retained.
                field = 'data' if custom or topic == '/rosout' else 'substr(data,1,512)'
                query = f'SELECT timestamp,length(data),{field} FROM messages WHERE topic_id=? ORDER BY timestamp'
                for receipt, size, blob in con.execute(query, (topic_id,)):
                    if topic == '/rosout':
                        msg = deserialize_message(blob, log_cls).msg
                        prefixes = ('Livox IMU diag ', 'FAST-LIO imu diag ',
                                    'FAST-LIO pipeline diag ', 'fusion timing:')
                        group = next((p.strip() for p in prefixes if msg.startswith(p)), None)
                        if group:
                            values = {k: float(v) for k, v in re.findall(
                                r'(\w+)=([-+]?\d+(?:\.\d+)?|nan)', msg)}
                            diagnostics.setdefault(group, []).append(values)
                        continue
                    if custom:
                        stamp, scan_end, last_offset, count, timebase = custom_scan(blob)
                        if not validated_custom:
                            msg = deserialize_message(blob, get_message(typ))
                            native_stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                            if (stamp != native_stamp or timebase != msg.timebase or
                                    count != len(msg.points) or last_offset != msg.points[-1].offset_time or
                                    scan_end != stamp + max(p.offset_time for p in msg.points)):
                                raise ValueError('CustomMsg parser does not match native ROS deserialization')
                            validated_custom = True
                    else:
                        stamp, _, _ = cdr_header(blob)
                        scan_end = last_offset = count = timebase = 0
                    rows.setdefault(topic, []).append(
                        (receipt, stamp, size, scan_end, last_offset, count, timebase))
    data = {k: np.array(sorted(v), dtype=np.int64) for k, v in rows.items() if v}
    if not data:
        parser.error('No supported topics found')
    report = {
        'bag_files': [str(f.resolve()) for f in files], 'label': args.label,
        'method': 'bag receipt timestamp minus header/scan-end timestamp; not DDS-only latency',
        'configuration_is_operator_label_not_verified_from_bag': True,
        'topics': {}, 'diagnostics': {},
    }
    for name, a in data.items():
        gaps = np.diff(a[:, 0]) / 1e6
        steps = np.diff(a[:, 1]) / 1e6
        valid = a[:, 1] != 0
        ages = (a[valid, 0] - a[valid, 1]) / 1e6
        stats = {
            'count': len(a), 'hz': (len(a) - 1) * 1e9 / (a[-1, 0] - a[0, 0]) if len(a) > 1 else None,
            'receipt_gap_ms': percentiles(gaps), 'stamp_step_ms': percentiles(steps),
            'header_age_ms': percentiles(ages), 'zero_stamps': int(sum(~valid)),
            'non_increasing_stamps': int(sum(steps <= 0)),
            'negative_header_ages': int(sum(ages < 0)),
            'gaps_over_500ms': int(sum(gaps > 500)),
            'max_serialized_bytes': int(a[:, 2].max()),
            'total_serialized_bytes': int(a[:, 2].sum()),
        }
        if name == '/livox/lidar' and validated_custom:
            stats.update(
                scan_duration_ms=percentiles((a[:, 3] - a[:, 1]) / 1e6),
                scan_end_age_ms=percentiles((a[:, 0] - a[:, 3]) / 1e6),
                header_timebase_mismatches=int(sum(a[:, 1] != a[:, 6])),
                last_point_not_latest=int(sum(a[:, 1] + a[:, 4] != a[:, 3])),
            )
        report['topics'][name] = stats
    # Match output stamps to recorded scan ends; recorder receive paths differ.
    if '/livox/lidar' in data and '/Odometry' in data:
        lidar, odom = data['/livox/lidar'], data['/Odometry']
        order = np.argsort(lidar[:, 3])
        lidar = lidar[order]
        positions = np.searchsorted(lidar[:, 3], odom[:, 1])
        matched = []
        for i, pos in enumerate(positions):
            choices = [j for j in (pos - 1, pos) if 0 <= j < len(lidar)]
            j = min(choices, key=lambda j: abs(int(lidar[j, 3]) - int(odom[i, 1])))
            if abs(int(lidar[j, 3]) - int(odom[i, 1])) <= 1000000:
                matched.append((odom[i, 0] - lidar[j, 0]) / 1e6)
        report['lidar_to_odometry_recorded_receive_delta'] = {
            'matched_within_1ms': len(matched), 'odom_count': len(odom),
            'delta_ms': percentiles(matched),
            'note': 'Not pure compute time; includes different DDS/recorder receive paths',
        }
    for name, windows in diagnostics.items():
        report['diagnostics'][name] = {
            'windows': len(windows),
            'fields': {key: percentiles([w.get(key, math.nan) for w in windows])
                       for key in sorted(set().union(*(w.keys() for w in windows)))},
        }
    args.output.mkdir(parents=True)
    (args.output / 'timing.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    np.savez_compressed(args.output / 'timestamps.npz', **data)
    origin = min(a[0, 0] for a in data.values())
    with (args.output / 'per_second.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['topic', 'bag_second', 'messages', 'header_age_p95_ms', 'header_age_max_ms'])
        for name, a in data.items():
            bins = (a[:, 0] - origin) // 10**9
            for second in range(int(bins.min()), int(bins.max()) + 1):
                b = a[bins == second]
                valid = b[b[:, 1] != 0]
                ages = (valid[:, 0] - valid[:, 1]) / 1e6
                writer.writerow([name, second, len(b),
                                 float(np.percentile(ages, 95)) if len(ages) else '',
                                 float(ages.max()) if len(ages) else ''])
    print(json.dumps(report, indent=2), flush=True)
    print('Saved:', args.output, flush=True)


if __name__ == '__main__':
    main()
