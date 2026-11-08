#!/usr/bin/env python3
"""Publish the selected immutable site; no mapping, transforms or robot commands."""
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from frozen_site import load_site


class FrozenMap(Node):
    def __init__(self):
        super().__init__('frozen_experiment_map')
        site = self.declare_parameter('site', '').value
        expected = self.declare_parameter('expected_height_sha256', '').value
        manifest, points = load_site(site)
        height = manifest['height']
        if expected and height['sha256'] != expected:
            raise ValueError('Frozen file changed after launch validation')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PointCloud2, '/rubi/experiment/frozen_heightmap', qos)
        self.info = self.create_publisher(String, '/rubi/experiment/frozen_map_info', qos)
        self.cloud = PointCloud2()
        self.cloud.header.frame_id = height['frame']
        sec, nsec = divmod(height['acquisition_stamp_ns'], 10**9)
        self.cloud.header.stamp.sec, self.cloud.header.stamp.nanosec = sec, nsec
        self.cloud.height, self.cloud.width = 1, height['usable_cells']
        self.cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32,
            count=1) for i, name in enumerate(('x', 'y', 'z'))]
        self.cloud.point_step = 12
        self.cloud.row_step = 12*self.cloud.width
        self.cloud.is_bigendian = False
        self.cloud.is_dense = True
        self.cloud.data = points
        self.metadata = String(data=json.dumps(manifest, ensure_ascii=False, allow_nan=False))
        self.publish()
        self.create_timer(1., self.publish)
        self.get_logger().info(f'Frozen site {manifest["site_id"]}: {self.cloud.width} cells, '
            f'sha256={height["sha256"]}, acquisition stamp retained')

    def publish(self):
        self.pub.publish(self.cloud)
        self.info.publish(self.metadata)


def main():
    rclpy.init()
    node = FrozenMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
