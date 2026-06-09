// MIT License
//
// ROS1 scan-to-map localization wrapper for GenZ-ICP.
// Added for navigation_ros1 branch.
//
// This file follows the original GenZ-ICP ROS1 OdometryServer style,
// but adds prebuilt-map localization, /initialpose initialization,
// and map -> odom TF publication.

#include "LocalizationServer.hpp"
#include "Utils.hpp"

// GenZ-ICP
#include "genz_icp/pipeline/GenZICP.hpp"

// Eigen / Sophus
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS1
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>

// STL
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace genz_icp_ros
{

namespace
{

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

std::vector<Eigen::Vector3d> LoadPCDAsEigen(const std::string &map_path)
{
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
        throw std::runtime_error("Loaded PCD map has zero finite XYZ points: " + map_path);
    }

    return map_points;
}

Sophus::SE3d PoseMsgToSophus(const geometry_msgs::Pose &pose)
{
    Sophus::SE3d::QuaternionType q(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);

    q.normalize();

    return Sophus::SE3d(
        q,
        Sophus::SE3d::Point(
            pose.position.x,
            pose.position.y,
            pose.position.z));
}

Sophus::SE3d PoseFromXYZRPY(
    double x,
    double y,
    double z,
    double roll,
    double pitch,
    double yaw)
{
    const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());

    Sophus::SE3d::QuaternionType q(rz * ry * rx);
    q.normalize();

    return Sophus::SE3d(q, Sophus::SE3d::Point(x, y, z));
}

std::string FixFrameId(const std::string &frame_id)
{
    if (!frame_id.empty() && frame_id.front() == '/') {
        return frame_id.substr(1);
    }
    return frame_id;
}

}  // namespace

LocalizationServer::LocalizationServer(
    const ros::NodeHandle &nh,
    const ros::NodeHandle &pnh)
    : nh_(nh),
      pnh_(pnh),
      tf2_listener_(tf2_buffer_)
{
    // -------------------------------------------------------------------------
    // 1) ROS / frame / topic parameters
    // -------------------------------------------------------------------------
    pnh_.param("pointcloud_topic", pointcloud_topic_, std::string("/velodyne_points"));
    pnh_.param("config_file", config_file_, std::string(""));
    pnh_.param("map_path", map_path_, std::string(""));

    pnh_.param("map_frame", map_frame_, std::string("map"));
    pnh_.param("odom_frame", odom_frame_, std::string("camera_init"));
    pnh_.param("base_frame", base_frame_, std::string("velodyne"));

    pnh_.param("publish_map_to_odom_tf", publish_map_to_odom_tf_, true);
    pnh_.param("publish_map_to_base_tf", publish_map_to_base_tf_, false);
    pnh_.param("visualize", publish_debug_clouds_, true);
    pnh_.param("require_initial_pose", require_initial_pose_, true);
    pnh_.param("use_sensor_stamp", use_sensor_stamp_, true);

    bool use_initial_pose_from_params = false;
    pnh_.param("use_initial_pose_from_params", use_initial_pose_from_params, false);

    double initial_pose_x = 0.0;
    double initial_pose_y = 0.0;
    double initial_pose_z = 0.0;
    double initial_pose_roll = 0.0;
    double initial_pose_pitch = 0.0;
    double initial_pose_yaw = 0.0;

    pnh_.param("initial_pose_x", initial_pose_x, 0.0);
    pnh_.param("initial_pose_y", initial_pose_y, 0.0);
    pnh_.param("initial_pose_z", initial_pose_z, 0.0);
    pnh_.param("initial_pose_roll", initial_pose_roll, 0.0);
    pnh_.param("initial_pose_pitch", initial_pose_pitch, 0.0);
    pnh_.param("initial_pose_yaw", initial_pose_yaw, 0.0);

    // -------------------------------------------------------------------------
    // 2) GenZ-ICP core parameters
    //
    // Keep the same parameter style as the original ROS1 OdometryServer.cpp.
    // -------------------------------------------------------------------------
    pnh_.param("max_range", config_.max_range, config_.max_range);
    pnh_.param("min_range", config_.min_range, config_.min_range);
    pnh_.param("deskew", config_.deskew, config_.deskew);
    pnh_.param("voxel_size", config_.voxel_size, config_.voxel_size);
    pnh_.param("map_cleanup_radius", config_.map_cleanup_radius, config_.map_cleanup_radius);
    pnh_.param("planarity_threshold", config_.planarity_threshold, config_.planarity_threshold);
    pnh_.param("max_points_per_voxel", config_.max_points_per_voxel, config_.max_points_per_voxel);
    pnh_.param(
        "desired_num_voxelized_points",
        config_.desired_num_voxelized_points,
        config_.desired_num_voxelized_points);
    pnh_.param("initial_threshold", config_.initial_threshold, config_.initial_threshold);
    pnh_.param("min_motion_th", config_.min_motion_th, config_.min_motion_th);
    pnh_.param("max_num_iterations", config_.max_num_iterations, config_.max_num_iterations);
    pnh_.param("convergence_criterion", config_.convergence_criterion, config_.convergence_criterion);

    if (config_.max_range < config_.min_range) {
        ROS_WARN("[GenZ Localization] max_range is smaller than min_range. Setting min_range to 0.0");
        config_.min_range = 0.0;
    }

    // Force scan-to-map localization mode.
    config_.localization_mode = true;

    // -------------------------------------------------------------------------
    // 3) Prebuilt map load
    // -------------------------------------------------------------------------
    if (map_path_.empty()) {
        throw std::runtime_error(
            "[GenZ Localization] map_path is empty. "
            "localization_node requires a prebuilt .pcd map.");
    }

    localization_ = genz_icp::pipeline::GenZICP(config_);

    const auto map_points = LoadPCDAsEigen(map_path_);
    localization_.SetMap(map_points);

    ROS_INFO(
        "[GenZ Localization] Loaded prebuilt PCD map: %s, finite_points=%zu",
        map_path_.c_str(),
        map_points.size());

    // -------------------------------------------------------------------------
    // 4) Initial pose policy
    // -------------------------------------------------------------------------
    if (use_initial_pose_from_params) {
        pending_initial_pose_map_base_ =
            PoseFromXYZRPY(
                initial_pose_x,
                initial_pose_y,
                initial_pose_z,
                initial_pose_roll,
                initial_pose_pitch,
                initial_pose_yaw);

        has_pending_initial_pose_ = true;
        received_initial_pose_ = true;

        ROS_INFO(
            "[GenZ Localization] Using initial pose from parameters: "
            "x=%.3f y=%.3f z=%.3f roll=%.3f pitch=%.3f yaw=%.3f",
            initial_pose_x,
            initial_pose_y,
            initial_pose_z,
            initial_pose_roll,
            initial_pose_pitch,
            initial_pose_yaw);
    } else if (!require_initial_pose_) {
        pending_initial_pose_map_base_ = Sophus::SE3d();
        has_pending_initial_pose_ = true;
        received_initial_pose_ = true;

        ROS_WARN(
            "[GenZ Localization] require_initial_pose=false. "
            "Assuming initial base pose is map origin.");
    }

    // -------------------------------------------------------------------------
    // 5) ROS subscribers / publishers
    // -------------------------------------------------------------------------
    pointcloud_sub_ =
        nh_.subscribe<sensor_msgs::PointCloud2>(
            pointcloud_topic_,
            queue_size_,
            &LocalizationServer::RegisterFrame,
            this);

    initial_pose_sub_ =
        nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            "/initialpose",
            1,
            &LocalizationServer::InitialPoseCallback,
            this);

    pose_publisher_ =
        pnh_.advertise<geometry_msgs::PoseStamped>(
            "/genz/localization_pose",
            queue_size_);

    odom_publisher_ =
        pnh_.advertise<nav_msgs::Odometry>(
            "/genz/localization_odom",
            queue_size_);

    path_publisher_ =
        pnh_.advertise<nav_msgs::Path>(
            "/genz/localization_path",
            queue_size_);

    path_msg_.header.frame_id = map_frame_;

    if (publish_debug_clouds_) {
        map_publisher_ =
            pnh_.advertise<sensor_msgs::PointCloud2>(
                "/genz/prebuilt_map",
                1,
                true);

        planar_points_publisher_ =
            pnh_.advertise<sensor_msgs::PointCloud2>(
                "/genz/localization_planar_points",
                queue_size_);

        non_planar_points_publisher_ =
            pnh_.advertise<sensor_msgs::PointCloud2>(
                "/genz/localization_non_planar_points",
                queue_size_);

        static_map_timer_ =
            nh_.createTimer(
                ros::Duration(1.0),
                &LocalizationServer::PublishStaticMap,
                this);
    }

    tf2_buffer_.setUsingDedicatedThread(true);

    ROS_INFO_STREAM(
        "[GenZ Localization] ROS1 localization node initialized. "
        << "pointcloud_topic=" << pointcloud_topic_
        << " map_path=" << map_path_
        << " map_frame=" << map_frame_
        << " odom_frame=" << odom_frame_
        << " base_frame=" << base_frame_
        << " publish_map_to_odom_tf=" << publish_map_to_odom_tf_
        << " publish_map_to_base_tf=" << publish_map_to_base_tf_
        << " use_sensor_stamp=" << use_sensor_stamp_
        << " require_initial_pose=" << require_initial_pose_);
}

void LocalizationServer::InitialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    const std::string msg_frame = FixFrameId(msg->header.frame_id);

    if (!msg_frame.empty() && msg_frame != map_frame_) {
        ROS_WARN(
            "[GenZ Localization] Received /initialpose in frame '%s', expected '%s'. "
            "Interpreting it as map-frame pose.",
            msg_frame.c_str(),
            map_frame_.c_str());
    }

    pending_initial_pose_map_base_ = PoseMsgToSophus(msg->pose.pose);
    has_pending_initial_pose_ = true;
    received_initial_pose_ = true;
    icp_initialized_ = false;
    path_msg_.poses.clear();

    const auto &p = msg->pose.pose.position;
    ROS_INFO(
        "[GenZ Localization] Received initial pose in map frame: x=%.3f y=%.3f z=%.3f",
        p.x,
        p.y,
        p.z);
}

bool LocalizationServer::LookupTransform(
    const std::string &target_frame,
    const std::string &source_frame,
    Sophus::SE3d *T_target_source) const
{
    if (target_frame.empty() || source_frame.empty()) {
        return false;
    }

    if (target_frame == source_frame) {
        *T_target_source = Sophus::SE3d();
        return true;
    }

    try {
        const auto tf =
            tf2_buffer_.lookupTransform(
                target_frame,
                source_frame,
                ros::Time(0),
                ros::Duration(0.10));

        *T_target_source = tf2::transformToSophus(tf);
        return true;
    } catch (const tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(
            2.0,
            "[GenZ Localization] Cannot transform %s <- %s. Reason=%s",
            target_frame.c_str(),
            source_frame.c_str(),
            ex.what());
        return false;
    }
}

bool LocalizationServer::LookupTransformAtTime(
    const std::string &target_frame,
    const std::string &source_frame,
    const ros::Time &stamp,
    Sophus::SE3d *T_target_source) const
{
    if (target_frame.empty() || source_frame.empty()) {
        return false;
    }

    if (target_frame == source_frame) {
        *T_target_source = Sophus::SE3d();
        return true;
    }

    try {
        const auto tf =
            tf2_buffer_.lookupTransform(
                target_frame,
                source_frame,
                stamp,
                ros::Duration(0.10));

        *T_target_source = tf2::transformToSophus(tf);
        return true;
    } catch (const tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(
            2.0,
            "[GenZ Localization] Cannot transform %s <- %s at stamp %.6f. Reason=%s",
            target_frame.c_str(),
            source_frame.c_str(),
            stamp.toSec(),
            ex.what());
        return false;
    }
}

bool LocalizationServer::InitializeICPIfNeeded(
    const std::string &cloud_frame_id)
{
    if (icp_initialized_) {
        return true;
    }

    if (!received_initial_pose_ || !has_pending_initial_pose_) {
        ROS_WARN_THROTTLE(
            2.0,
            "[GenZ Localization] Waiting for /initialpose before scan-to-map localization starts.");
        return false;
    }

    Sophus::SE3d T_map_cloud = pending_initial_pose_map_base_;

    if (!base_frame_.empty() && base_frame_ != cloud_frame_id) {
        Sophus::SE3d T_base_cloud;

        if (!LookupTransform(base_frame_, cloud_frame_id, &T_base_cloud)) {
            ROS_WARN_THROTTLE(
                2.0,
                "[GenZ Localization] Cannot initialize until static TF %s <- %s exists.",
                base_frame_.c_str(),
                cloud_frame_id.c_str());
            return false;
        }

        // pending_initial_pose_map_base_ is T_map_base.
        // Convert it to T_map_cloud:
        // T_map_cloud = T_map_base * T_base_cloud
        T_map_cloud = pending_initial_pose_map_base_ * T_base_cloud;
    }

    localization_.SetInitialPose(T_map_cloud);
    icp_initialized_ = true;

    ROS_INFO("[GenZ Localization] Initialized GenZ scan-to-map ICP.");
    return true;
}

Sophus::SE3d LocalizationServer::ConvertCloudPoseToBasePose(
    const Sophus::SE3d &T_map_cloud,
    const std::string &cloud_frame_id)
{
    if (base_frame_.empty() || base_frame_ == cloud_frame_id) {
        return T_map_cloud;
    }

    Sophus::SE3d T_base_cloud;
    if (!LookupTransform(base_frame_, cloud_frame_id, &T_base_cloud)) {
        ROS_WARN_THROTTLE(
            2.0,
            "[GenZ Localization] Falling back to cloud pose because static TF %s <- %s is unavailable.",
            base_frame_.c_str(),
            cloud_frame_id.c_str());
        return T_map_cloud;
    }

    // T_map_base = T_map_cloud * inv(T_base_cloud)
    return T_map_cloud * T_base_cloud.inverse();
}

void LocalizationServer::RegisterFrame(
    const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    const std::string cloud_frame_id = FixFrameId(msg->header.frame_id);

    if (!InitializeICPIfNeeded(cloud_frame_id)) {
        return;
    }

    const auto points = PointCloud2ToEigen(msg);

    const auto timestamps = [&]() -> std::vector<double> {
        if (!config_.deskew) {
            return {};
        }

        try {
            return GetTimestamps(msg);
        } catch (const std::exception &e) {
            ROS_WARN_THROTTLE(
                2.0,
                "[GenZ Localization] Timestamp extraction failed: %s. No deskew.",
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

    const ros::Time output_stamp =
        use_sensor_stamp_ ? msg->header.stamp : ros::Time::now();

    PublishLocalization(T_map_base, output_stamp, child_frame_id);

    if (publish_map_to_odom_tf_) {
        PublishMapToOdomTF(T_map_base, output_stamp, child_frame_id);
    }

    if (publish_map_to_base_tf_) {
        PublishMapToBaseTF(T_map_base, output_stamp, child_frame_id);
    }

    if (publish_debug_clouds_) {
        PublishClouds(output_stamp, planar_points, non_planar_points);
    }
}

void LocalizationServer::PublishLocalization(
    const Sophus::SE3d &T_map_base,
    const ros::Time &stamp,
    const std::string &child_frame_id)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = tf2::sophusToPose(T_map_base);
    pose_publisher_.publish(pose_msg);

    path_msg_.header.stamp = stamp;
    path_msg_.header.frame_id = map_frame_;
    path_msg_.poses.push_back(pose_msg);
    path_publisher_.publish(path_msg_);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id = child_frame_id;
    odom_msg.pose.pose = tf2::sophusToPose(T_map_base);
    odom_publisher_.publish(odom_msg);
}

void LocalizationServer::PublishMapToOdomTF(
    const Sophus::SE3d &T_map_base,
    const ros::Time &stamp,
    const std::string &child_frame_id)
{
    Sophus::SE3d T_odom_base;

    const bool got_odom_base =
        use_sensor_stamp_
            ? LookupTransformAtTime(odom_frame_, child_frame_id, stamp, &T_odom_base)
            : LookupTransform(odom_frame_, child_frame_id, &T_odom_base);

    if (!got_odom_base) {
        ROS_WARN_THROTTLE(
            2.0,
            "[GenZ Localization] Cannot publish map->odom. Required TF %s <- %s is unavailable.",
            odom_frame_.c_str(),
            child_frame_id.c_str());
        return;
    }

    // T_map_odom = T_map_base * inv(T_odom_base)
    const Sophus::SE3d T_map_odom = T_map_base * T_odom_base.inverse();

    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = stamp;
    transform_msg.header.frame_id = map_frame_;
    transform_msg.child_frame_id = odom_frame_;
    transform_msg.transform = tf2::sophusToTransform(T_map_odom);

    tf_broadcaster_.sendTransform(transform_msg);
}

void LocalizationServer::PublishMapToBaseTF(
    const Sophus::SE3d &T_map_base,
    const ros::Time &stamp,
    const std::string &child_frame_id)
{
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = stamp;
    transform_msg.header.frame_id = map_frame_;
    transform_msg.child_frame_id = child_frame_id;
    transform_msg.transform = tf2::sophusToTransform(T_map_base);

    tf_broadcaster_.sendTransform(transform_msg);
}

void LocalizationServer::PublishClouds(
    const ros::Time &stamp,
    const std::vector<Eigen::Vector3d> &planar_points,
    const std::vector<Eigen::Vector3d> &non_planar_points)
{
    std_msgs::Header map_header;
    map_header.stamp = stamp;
    map_header.frame_id = map_frame_;

    planar_points_publisher_.publish(
        *EigenToPointCloud2(planar_points, map_header));

    non_planar_points_publisher_.publish(
        *EigenToPointCloud2(non_planar_points, map_header));
}

void LocalizationServer::PublishStaticMap(const ros::TimerEvent &)
{
    if (!publish_debug_clouds_) {
        return;
    }

    if (!map_publisher_) {
        return;
    }

    std_msgs::Header map_header;
    map_header.stamp = ros::Time::now();
    map_header.frame_id = map_frame_;

    map_publisher_.publish(
        *EigenToPointCloud2(localization_.LocalMap(), map_header));
}

}  // namespace genz_icp_ros