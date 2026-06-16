#pragma once

#include "genz_icp/pipeline/GenZICP.hpp"

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <vector>
#include <mutex>

namespace genz_icp_ros
{

class LocalizationServer
{
public:
    LocalizationServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

private:
    void RegisterFrame(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void InitialPoseCallback(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    bool LookupTransform(const std::string &target_frame,
                         const std::string &source_frame,
                         Sophus::SE3d *T_target_source) const;

    bool LookupTransformAtTime(const std::string &target_frame,
                               const std::string &source_frame,
                               const ros::Time &stamp,
                               Sophus::SE3d *T_target_source) const;

    bool InitializeICPIfNeeded(const std::string &cloud_frame_id);

    Sophus::SE3d ConvertCloudPoseToBasePose(
        const Sophus::SE3d &T_map_cloud,
        const std::string &cloud_frame_id);

    void PublishLocalization(const Sophus::SE3d &T_map_base,
                             const ros::Time &stamp,
                             const std::string &child_frame_id);

    void PublishMapToOdomTF(const Sophus::SE3d &T_map_base,
                            const ros::Time &stamp,
                            const std::string &child_frame_id);

    void PublishMapToBaseTF(const Sophus::SE3d &T_map_base,
                            const ros::Time &stamp,
                            const std::string &child_frame_id);

    void PublishClouds(const ros::Time &stamp,
                       const std::vector<Eigen::Vector3d> &planar_points,
                       const std::vector<Eigen::Vector3d> &non_planar_points);

    void PublishStaticMap(const ros::TimerEvent &);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    int queue_size_{1};

    ros::Subscriber pointcloud_sub_;
    ros::Subscriber initial_pose_sub_;

    ros::Publisher pose_publisher_;
    ros::Publisher odom_publisher_;
    ros::Publisher path_publisher_;
    ros::Publisher map_publisher_;
    ros::Publisher planar_points_publisher_;
    ros::Publisher non_planar_points_publisher_;

    ros::Timer static_map_timer_;
    ros::Timer tf_publish_timer_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;

    nav_msgs::Path path_msg_;

    genz_icp::pipeline::GenZICP localization_;
    genz_icp::pipeline::GenZConfig config_;

    std::string pointcloud_topic_{"/velodyne_points"};
    std::string config_file_{};
    std::string map_path_{};

    std::string map_frame_{"map"};
    std::string odom_frame_{"camera_init"};
    std::string base_frame_{"velodyne"};

    bool publish_map_to_odom_tf_{true};
    bool publish_map_to_base_tf_{false};
    bool publish_debug_clouds_{true};
    bool require_initial_pose_{true};
    bool use_sensor_stamp_{true};
    double tf_future_tolerance_{0.03};
    double tf_publish_rate_{50.0};

    mutable std::mutex latest_tf_mutex_;
    bool has_latest_map_to_odom_tf_{false};
    geometry_msgs::TransformStamped latest_map_to_odom_tf_;

    bool received_initial_pose_{false};
    bool icp_initialized_{false};
    bool has_pending_initial_pose_{false};

    Sophus::SE3d pending_initial_pose_map_base_;

    void PublishLatestMapToOdomTF(const ros::TimerEvent &);
};

}  // namespace genz_icp_ros