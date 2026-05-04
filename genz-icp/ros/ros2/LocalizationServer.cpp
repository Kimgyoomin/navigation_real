#include "LocalizationServer.hpp"
#include "Utils.hpp"

#include "genz_icp/pipeline/GenZICP.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcpputils/filesystem_helper.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/exceptions.h>
#include <yaml-cpp/yaml.h>

namespace genz_icp_ros {

namespace {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

std::vector<Eigen::Vector3d> LoadPCDAsEigen(const std::string &map_path) {
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, pcl_cloud) < 0) {
    throw std::runtime_error("Failed to load PCD map: " + map_path);
  }

  std::vector<Eigen::Vector3d> map_points;
  map_points.reserve(pcl_cloud.points.size());
  for (const auto &p : pcl_cloud.points) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      map_points.emplace_back(p.x, p.y, p.z);
    }
  }

  if (map_points.empty()) {
    throw std::runtime_error("Loaded PCD map has zero finite XYZ points: " +
                             map_path);
  }
  return map_points;
}

Sophus::SE3d PoseMsgToSophus(const geometry_msgs::msg::Pose &pose) {
  Sophus::SE3d::QuaternionType q(pose.orientation.w, pose.orientation.x,
                                 pose.orientation.y, pose.orientation.z);
  q.normalize();
  return Sophus::SE3d(q, Sophus::SE3d::Point(pose.position.x, pose.position.y,
                                             pose.position.z));
}

Sophus::SE3d PoseFromXYZRPY(double x, double y, double z, double roll,
                            double pitch, double yaw) {
  const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  Sophus::SE3d::QuaternionType q(rz * ry * rx);
  q.normalize();
  return Sophus::SE3d(q, Sophus::SE3d::Point(x, y, z));
}

}  // namespace

LocalizationServer::LocalizationServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("localization_node", options) {
  map_frame_ = declare_parameter("map_frame", map_frame_);
  odom_frame_ = declare_parameter("odom_frame", odom_frame_);
  base_frame_ = declare_parameter("base_frame", base_frame_);
  map_path_ = declare_parameter("map_path", map_path_);

  publish_map_to_odom_tf_ =
      declare_parameter("publish_map_to_odom_tf", publish_map_to_odom_tf_);
  publish_map_to_base_tf_ =
      declare_parameter("publish_map_to_base_tf", publish_map_to_base_tf_);
  publish_debug_clouds_ = declare_parameter("visualize", publish_debug_clouds_);
  require_initial_pose_ =
      declare_parameter("require_initial_pose", require_initial_pose_);
  declare_parameter("localization_mode", true);

  const bool use_initial_pose_from_params =
      declare_parameter("use_initial_pose_from_params", false);
  const double initial_pose_x = declare_parameter("initial_pose_x", 0.0);
  const double initial_pose_y = declare_parameter("initial_pose_y", 0.0);
  const double initial_pose_z = declare_parameter("initial_pose_z", 0.0);
  const double initial_pose_roll = declare_parameter("initial_pose_roll", 0.0);
  const double initial_pose_pitch = declare_parameter("initial_pose_pitch", 0.0);
  const double initial_pose_yaw = declare_parameter("initial_pose_yaw", 0.0);

  declare_parameter("max_range", config_.max_range);
  declare_parameter("min_range", config_.min_range);
  declare_parameter("deskew", config_.deskew);
  declare_parameter("voxel_size", config_.voxel_size);
  declare_parameter("map_cleanup_radius", config_.map_cleanup_radius);
  declare_parameter("planarity_threshold", config_.planarity_threshold);
  declare_parameter("max_points_per_voxel", config_.max_points_per_voxel);
  declare_parameter("desired_num_voxelized_points",
                    config_.desired_num_voxelized_points);
  declare_parameter("max_num_iterations", config_.max_num_iterations);
  declare_parameter("convergence_criterion", config_.convergence_criterion);
  declare_parameter("initial_threshold", config_.initial_threshold);
  declare_parameter("min_motion_th", config_.min_motion_th);
  declare_parameter("config_file", "");

  std::string config_file = get_parameter("config_file").as_string();
  if (!config_file.empty()) {
    rcpputils::fs::path path(config_file);
    if (!path.is_absolute()) {
      path = rcpputils::fs::path(
                 ament_index_cpp::get_package_share_directory("genz_icp")) /
             "config" / path;
    }

    YAML::Node yaml = YAML::LoadFile(path.string());
    std::vector<rclcpp::Parameter> overrides;
    for (const auto &param : yaml) {
      const auto name = param.first.as<std::string>();
      const auto &value = param.second;
      if (!value.IsScalar()) continue;
      if (!has_parameter(name)) {
        RCLCPP_WARN(get_logger(),
                    "Ignoring unknown parameter '%s' from config file '%s'",
                    name.c_str(), path.string().c_str());
        continue;
      }

      const auto descriptor = describe_parameter(name);
      using ParamType = rcl_interfaces::msg::ParameterType;
      switch (descriptor.type) {
        case ParamType::PARAMETER_DOUBLE:
          overrides.emplace_back(name, value.as<double>());
          break;
        case ParamType::PARAMETER_INTEGER:
          overrides.emplace_back(name, value.as<int>());
          break;
        case ParamType::PARAMETER_BOOL:
          overrides.emplace_back(name, value.as<bool>());
          break;
        case ParamType::PARAMETER_STRING:
          overrides.emplace_back(name, value.as<std::string>());
          break;
        default:
          break;
      }
    }
    set_parameters(overrides);
  }

  config_.max_range = get_parameter("max_range").as_double();
  config_.min_range = get_parameter("min_range").as_double();
  config_.deskew = get_parameter("deskew").as_bool();
  config_.voxel_size = get_parameter("voxel_size").as_double();
  config_.map_cleanup_radius = get_parameter("map_cleanup_radius").as_double();
  config_.planarity_threshold =
      get_parameter("planarity_threshold").as_double();
  config_.max_points_per_voxel =
      get_parameter("max_points_per_voxel").as_int();
  config_.desired_num_voxelized_points =
      get_parameter("desired_num_voxelized_points").as_int();
  config_.max_num_iterations = get_parameter("max_num_iterations").as_int();
  config_.convergence_criterion =
      get_parameter("convergence_criterion").as_double();
  config_.initial_threshold = get_parameter("initial_threshold").as_double();
  config_.min_motion_th = get_parameter("min_motion_th").as_double();

  if (config_.max_range < config_.min_range) {
    RCLCPP_WARN(get_logger(),
                "max_range is smaller than min_range. Setting min_range=0.0");
    config_.min_range = 0.0;
  }

  config_.localization_mode = true;

  if (map_path_.empty()) {
    throw std::runtime_error(
        "map_path is empty. localization_node requires a prebuilt .pcd map.");
  }

  localization_ = genz_icp::pipeline::GenZICP(config_);

  const auto map_points = LoadPCDAsEigen(map_path_);
  localization_.SetMap(map_points);

  RCLCPP_INFO(get_logger(), "Loaded prebuilt PCD map: %s, finite_points=%zu",
              map_path_.c_str(), map_points.size());

  if (use_initial_pose_from_params) {
    pending_initial_pose_map_base_ =
        PoseFromXYZRPY(initial_pose_x, initial_pose_y, initial_pose_z,
                       initial_pose_roll, initial_pose_pitch, initial_pose_yaw);
    received_initial_pose_ = true;
  } else if (!require_initial_pose_) {
    pending_initial_pose_map_base_ = Sophus::SE3d();
    received_initial_pose_ = true;
    RCLCPP_WARN(get_logger(),
                "require_initial_pose=false. Assuming initial base pose is map "
                "origin.");
  }

  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud_topic", rclcpp::SensorDataQoS(),
      std::bind(&LocalizationServer::RegisterFrame, this,
                std::placeholders::_1));

  initial_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", rclcpp::QoS(10),
          std::bind(&LocalizationServer::InitialPoseCallback, this,
                    std::placeholders::_1));

  rclcpp::QoS qos(
      (rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));

  pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(
          "/genz/localization_pose", qos);
  odom_publisher_ =
      create_publisher<nav_msgs::msg::Odometry>("/genz/localization_odom", qos);
  path_publisher_ =
      create_publisher<nav_msgs::msg::Path>("/genz/localization_path", qos);

  path_msg_.header.frame_id = map_frame_;

  if (publish_debug_clouds_) {
    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/genz/prebuilt_map", qos);
    planar_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/genz/planar_points", qos);
    non_planar_points_publisher_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(
            "/genz/non_planar_points", qos);

    static_map_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&LocalizationServer::PublishStaticMap, this));
  }

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_buffer_->setUsingDedicatedThread(true);
  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

  RCLCPP_INFO(get_logger(), "GenZ-ICP ROS 2 localization node initialized");
}

void LocalizationServer::InitialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
    RCLCPP_WARN(get_logger(),
                "Received /initialpose in frame '%s', expected '%s'. "
                "Interpreting it as map-frame pose.",
                msg->header.frame_id.c_str(), map_frame_.c_str());
  }

  pending_initial_pose_map_base_ = PoseMsgToSophus(msg->pose.pose);
  received_initial_pose_ = true;
  icp_initialized_ = false;
  path_msg_.poses.clear();

  const auto &p = msg->pose.pose.position;
  RCLCPP_INFO(get_logger(),
              "Received initial pose in map frame: x=%.3f y=%.3f z=%.3f",
              p.x, p.y, p.z);
}

bool LocalizationServer::LookupTransform(const std::string &target_frame,
                                         const std::string &source_frame,
                                         Sophus::SE3d *T_target_source) {
  if (target_frame.empty() || source_frame.empty()) return false;

  if (target_frame == source_frame) {
    *T_target_source = Sophus::SE3d();
    return true;
  }

  std::string err_msg;
  if (!tf2_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero,
                                 &err_msg)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Cannot transform %s <- %s. Reason=%s",
                         target_frame.c_str(), source_frame.c_str(),
                         err_msg.c_str());
    return false;
  }

  try {
    const auto tf =
        tf2_buffer_->lookupTransform(target_frame, source_frame,
                                     tf2::TimePointZero);
    *T_target_source = tf2::transformToSophus(tf);
    return true;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", ex.what());
    return false;
  }
}

bool LocalizationServer::InitializeICPIfNeeded(
    const std::string &cloud_frame_id) {
  if (icp_initialized_) return true;

  if (!received_initial_pose_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Waiting for /initialpose before scan-to-map "
                         "localization starts.");
    return false;
  }

  Sophus::SE3d T_map_cloud = *pending_initial_pose_map_base_;

  if (!base_frame_.empty() && base_frame_ != cloud_frame_id) {
    Sophus::SE3d T_base_cloud;
    if (!LookupTransform(base_frame_, cloud_frame_id, &T_base_cloud)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Cannot initialize until static TF %s <- %s exists.",
                           base_frame_.c_str(), cloud_frame_id.c_str());
      return false;
    }
    T_map_cloud = (*pending_initial_pose_map_base_) * T_base_cloud;
  }

  localization_.SetInitialPose(T_map_cloud);
  icp_initialized_ = true;
  RCLCPP_INFO(get_logger(), "Initialized GenZ scan-to-map ICP.");
  return true;
}

Sophus::SE3d LocalizationServer::ConvertCloudPoseToBasePose(
    const Sophus::SE3d &T_map_cloud,
    const std::string &cloud_frame_id) {
  if (base_frame_.empty() || base_frame_ == cloud_frame_id) {
    return T_map_cloud;
  }

  Sophus::SE3d T_base_cloud;
  if (!LookupTransform(base_frame_, cloud_frame_id, &T_base_cloud)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Falling back to cloud pose because static TF "
                         "%s <- %s is unavailable.",
                         base_frame_.c_str(), cloud_frame_id.c_str());
    return T_map_cloud;
  }

  return T_map_cloud * T_base_cloud.inverse();
}

void LocalizationServer::RegisterFrame(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  const std::string cloud_frame_id = utils::FixFrameId(msg->header.frame_id);

  if (!InitializeICPIfNeeded(cloud_frame_id)) return;

  const auto points = PointCloud2ToEigen(msg);

  const auto timestamps = [&]() -> std::vector<double> {
    if (!config_.deskew) return {};
    try {
      return GetTimestamps(msg);
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Timestamp extraction failed: %s. No deskew.",
                           e.what());
      return {};
    }
  }();

  const auto &[planar_points, non_planar_points] =
      localization_.RegisterFrame(points, timestamps);

  const Sophus::SE3d T_map_cloud = localization_.LatestPose();
  const Sophus::SE3d T_map_base =
      ConvertCloudPoseToBasePose(T_map_cloud, cloud_frame_id);

  const std::string child_frame_id =
      base_frame_.empty() ? cloud_frame_id : base_frame_;

  PublishLocalization(T_map_base, msg->header.stamp, child_frame_id);

  if (publish_map_to_odom_tf_) {
    PublishMapToOdomTF(T_map_base, msg->header.stamp, child_frame_id);
  }
  if (publish_map_to_base_tf_) {
    PublishMapToBaseTF(T_map_base, msg->header.stamp, child_frame_id);
  }
  if (publish_debug_clouds_) {
    PublishClouds(msg->header.stamp, planar_points, non_planar_points);
  }
}

void LocalizationServer::PublishLocalization(
    const Sophus::SE3d &T_map_base, const rclcpp::Time &stamp,
    const std::string &child_frame_id) {
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = map_frame_;
  pose_msg.pose = tf2::sophusToPose(T_map_base);
  pose_publisher_->publish(pose_msg);

  path_msg_.header.stamp = stamp;
  path_msg_.header.frame_id = map_frame_;
  path_msg_.poses.push_back(pose_msg);
  path_publisher_->publish(path_msg_);

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = map_frame_;
  odom_msg.child_frame_id = child_frame_id;
  odom_msg.pose.pose = tf2::sophusToPose(T_map_base);
  odom_publisher_->publish(odom_msg);
}

void LocalizationServer::PublishMapToOdomTF(const Sophus::SE3d &T_map_base,
                                            const rclcpp::Time &stamp,
                                            const std::string &child_frame_id) {
  Sophus::SE3d T_odom_base;
  if (!LookupTransform(odom_frame_, child_frame_id, &T_odom_base)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Cannot publish map->odom. Required TF %s <- %s "
                         "is unavailable.",
                         odom_frame_.c_str(), child_frame_id.c_str());
    return;
  }

  const Sophus::SE3d T_map_odom = T_map_base * T_odom_base.inverse();

  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = map_frame_;
  transform_msg.child_frame_id = odom_frame_;
  transform_msg.transform = tf2::sophusToTransform(T_map_odom);
  tf_broadcaster_->sendTransform(transform_msg);
}

void LocalizationServer::PublishMapToBaseTF(const Sophus::SE3d &T_map_base,
                                            const rclcpp::Time &stamp,
                                            const std::string &child_frame_id) {
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = map_frame_;
  transform_msg.child_frame_id = child_frame_id;
  transform_msg.transform = tf2::sophusToTransform(T_map_base);
  tf_broadcaster_->sendTransform(transform_msg);
}

void LocalizationServer::PublishClouds(
    const rclcpp::Time &stamp,
    const std::vector<Eigen::Vector3d> &planar_points,
    const std::vector<Eigen::Vector3d> &non_planar_points) {
  std_msgs::msg::Header map_header;
  map_header.stamp = stamp;
  map_header.frame_id = map_frame_;

  planar_points_publisher_->publish(
      std::move(EigenToPointCloud2(planar_points, map_header)));
  non_planar_points_publisher_->publish(
      std::move(EigenToPointCloud2(non_planar_points, map_header)));
}

void LocalizationServer::PublishStaticMap() {
  if (!publish_debug_clouds_ || !map_publisher_) return;

  std_msgs::msg::Header map_header;
  map_header.stamp = now();
  map_header.frame_id = map_frame_;

  map_publisher_->publish(
      std::move(EigenToPointCloud2(localization_.LocalMap(), map_header)));
}

}  // namespace genz_icp_ros

RCLCPP_COMPONENTS_REGISTER_NODE(genz_icp_ros::LocalizationServer)
