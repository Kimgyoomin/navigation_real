#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import tf_transformations
import tf2_ros
from geometry_msgs.msg import Transform


class TransformFusion(Node):
    def __init__(self):
        super().__init__("transform_fusion")

        self.cur_map_to_odom = None

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.pub_localization = self.create_publisher(Odometry, "/localization", 1)

        # Transform fusion only needs the newest state. A reliable reader can
        # retain stale odometry while the process is temporarily descheduled
        # by global ICP, which then makes map->camera_init unusable for Nav2.
        latest_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Keep explicit references as well, so the graph always retains and
        # reports both subscription endpoints for the node lifetime.
        self.sub_odom = self.create_subscription(
            Odometry, "/Odometry", self.cb_save_cur_odom, latest_state_qos)
        self.sub_map_to_odom = self.create_subscription(
            Odometry, "/map_to_odom", self.cb_save_map_to_odom,
            latest_state_qos)

    def pose_to_mat(self, pose_msg):
        trans = np.eye(4)
        trans[:3, 3] = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        quat = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
        trans[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return trans

    def transform_fusion(self, cur_odom):
        map_to_odom = self.cur_map_to_odom
        if map_to_odom is not None:
            T_map_to_odom = self.pose_to_mat(map_to_odom.pose.pose)
        else:
            T_map_to_odom = np.eye(4)

        transform_msg = Transform()
        transform_msg.translation.x = T_map_to_odom[0, 3]
        transform_msg.translation.y = T_map_to_odom[1, 3]
        transform_msg.translation.z = T_map_to_odom[2, 3]
        
        quat = tf_transformations.quaternion_from_matrix(T_map_to_odom)

        transform_msg.rotation.x = quat[0]
        transform_msg.rotation.y = quat[1]
        transform_msg.rotation.z = quat[2]
        transform_msg.rotation.w = quat[3]
        
        # Publish the global correction as soon as a new FAST-LIO odometry
        # sample arrives. Keep both dynamic TF edges on the same sensor stamp.
        # Copy individual header fields to avoid mutating the incoming message.
        transform_stamped_msg = tf2_ros.TransformStamped()
        transform_stamped_msg.header.stamp = cur_odom.header.stamp
        transform_stamped_msg.header.frame_id = "map"
        transform_stamped_msg.child_frame_id = "camera_init"
        transform_stamped_msg.transform = transform_msg
        self.tf_broadcaster.sendTransform(transform_stamped_msg)

        T_odom_to_base_link = self.pose_to_mat(cur_odom.pose.pose)
        T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)

        xyz = tf_transformations.translation_from_matrix(T_map_to_base_link)
        quat = tf_transformations.quaternion_from_matrix(T_map_to_base_link)

        localization = Odometry()
        localization.pose.pose = Pose(
            position = Point(x = xyz[0], y = xyz[1], z = xyz[2]),
            orientation = Quaternion(x = quat[0], y = quat[1], z = quat[2], w = quat[3])
        )
        localization.twist = cur_odom.twist

        localization.header.stamp = cur_odom.header.stamp
        localization.header.frame_id = "map"
        localization.child_frame_id = "body"
        self.pub_localization.publish(localization)


    def cb_save_cur_odom(self, msg):
        self.transform_fusion(msg)

    def cb_save_map_to_odom(self, msg):
        self.cur_map_to_odom = msg


def main(args=None):
    rclpy.init(args=args)
    node = TransformFusion()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
