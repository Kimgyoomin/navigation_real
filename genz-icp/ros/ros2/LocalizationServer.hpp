#pragma once

#include "genz_icp/pipeline/GenZICP.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <sophus/se3.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace genz_icp_ros {

class LocalizationServer : public rclcpp::Node {
public:
  LocalizationServer() = delete;
  explicit LocalizationServer(const rclcpp::NodeOptions &options);

private:
  void RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

  void InitialPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  bool LookupTransform(const std::string &target_frame,
                       const std::string &source_frame,
                       Sophus::SE3d *T_target_source);

  // Add 260506 for TF / timestamp
  bool LookupTransformAtTime(const std::string &target_frame,
                             const std::string &source_frame,
                             const rclcpp::Time &stamp,
                             Sophus::SE3d *T_target_source);

  bool InitializeICPIfNeeded(const std::string &cloud_frame_id);

  Sophus::SE3d ConvertCloudPoseToBasePose(
      const Sophus::SE3d &T_map_cloud,
      const std::string &cloud_frame_id);

  void PublishLocalization(const Sophus::SE3d &T_map_base,
                           const rclcpp::Time &stamp,
                           const std::string &child_frame_id);

  void PublishMapToOdomTF(const Sophus::SE3d &T_map_base,
                          const rclcpp::Time &stamp,
                          const std::string &child_frame_id);

  void PublishMapToBaseTF(const Sophus::SE3d &T_map_base,
                          const rclcpp::Time &stamp,
                          const std::string &child_frame_id);

  void PublishClouds(const rclcpp::Time &stamp,
                     const std::vector<Eigen::Vector3d> &planar_points,
                     const std::vector<Eigen::Vector3d> &non_planar_points);

  void PublishStaticMap();

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      planar_points_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      non_planar_points_publisher_;

  rclcpp::TimerBase::SharedPtr static_map_timer_;

  nav_msgs::msg::Path path_msg_;

  genz_icp::pipeline::GenZICP localization_;
  genz_icp::pipeline::GenZConfig config_;

  std::string map_path_;
  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};

  bool publish_map_to_odom_tf_{true};
  bool publish_map_to_base_tf_{false};
  bool publish_debug_clouds_{true};
  bool require_initial_pose_{true};
  // Add 260506 for TF / timestamp
  bool use_sensor_stamp_{true};

  bool received_initial_pose_{false};
  bool icp_initialized_{false};

  std::optional<Sophus::SE3d> pending_initial_pose_map_base_;
};

}  // namespace genz_icp_ros
