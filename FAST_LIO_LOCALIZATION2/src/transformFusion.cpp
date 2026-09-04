#include <cmath>
#include <chrono>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

class TransformFusionNode : public rclcpp::Node
{
public:
  TransformFusionNode()
  : Node("transform_fusion"),
    map_to_odom_translation_(Eigen::Vector3d::Zero()),
    map_to_odom_rotation_(Eigen::Quaterniond::Identity()),
    previous_callback_time_(std::chrono::steady_clock::now())
  {
    // FAST-LIO publishes Odometry reliably.  Matching that QoS avoids the
    // unexpectedly sparse delivery observed with Fast DDS on this platform,
    // while depth 1 still guarantees that stale states cannot accumulate.
    auto latest_reliable_qos = rclcpp::QoS(1).reliable().durability_volatile();
    auto latest_sensor_qos = rclcpp::SensorDataQoS().keep_last(1);

    localization_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/localization", rclcpp::QoS(1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(
      *this, tf2_ros::DynamicBroadcasterQoS(1));

    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", latest_reliable_qos,
      std::bind(&TransformFusionNode::odometryCallback, this,
        std::placeholders::_1));
    map_to_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/map_to_odom", latest_sensor_qos,
      std::bind(&TransformFusionNode::mapToOdomCallback, this,
        std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "C++ transform fusion started (Odometry RELIABLE depth 1, TF depth 1)");
  }

private:
  static Eigen::Quaterniond normalizedQuaternion(
    const geometry_msgs::msg::Quaternion & quaternion)
  {
    Eigen::Quaterniond result(
      quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    if (result.squaredNorm() < 1.0e-12) {
      return Eigen::Quaterniond::Identity();
    }
    result.normalize();
    return result;
  }

  void mapToOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    map_to_odom_translation_ = Eigen::Vector3d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    map_to_odom_rotation_ = normalizedQuaternion(msg->pose.pose.orientation);
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto callback_start = std::chrono::steady_clock::now();
    const double callback_interval_ms =
      std::chrono::duration<double, std::milli>(
      callback_start - previous_callback_time_).count();
    previous_callback_time_ = callback_start;

    // Publish the piecewise-constant global correction at every fresh
    // FAST-LIO odometry stamp. Nav2 therefore always has a current
    // map->camera_init edge even though global ICP updates at only 0.5 Hz.
    geometry_msgs::msg::TransformStamped map_to_camera_init;
    map_to_camera_init.header.stamp = msg->header.stamp;
    map_to_camera_init.header.frame_id = "map";
    map_to_camera_init.child_frame_id = "camera_init";
    map_to_camera_init.transform.translation.x = map_to_odom_translation_.x();
    map_to_camera_init.transform.translation.y = map_to_odom_translation_.y();
    map_to_camera_init.transform.translation.z = map_to_odom_translation_.z();
    map_to_camera_init.transform.rotation.x = map_to_odom_rotation_.x();
    map_to_camera_init.transform.rotation.y = map_to_odom_rotation_.y();
    map_to_camera_init.transform.rotation.z = map_to_odom_rotation_.z();
    map_to_camera_init.transform.rotation.w = map_to_odom_rotation_.w();
    const Eigen::Vector3d odom_translation(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    const Eigen::Quaterniond odom_rotation =
      normalizedQuaternion(msg->pose.pose.orientation);

    const Eigen::Vector3d map_translation =
      map_to_odom_rotation_ * odom_translation + map_to_odom_translation_;
    Eigen::Quaterniond map_rotation = map_to_odom_rotation_ * odom_rotation;
    map_rotation.normalize();

    nav_msgs::msg::Odometry localization;
    localization.header.stamp = msg->header.stamp;
    localization.header.frame_id = "map";
    localization.child_frame_id = "body";
    localization.pose.pose.position.x = map_translation.x();
    localization.pose.pose.position.y = map_translation.y();
    localization.pose.pose.position.z = map_translation.z();
    localization.pose.pose.orientation.x = map_rotation.x();
    localization.pose.pose.orientation.y = map_rotation.y();
    localization.pose.pose.orientation.z = map_rotation.z();
    localization.pose.pose.orientation.w = map_rotation.w();
    localization.twist = msg->twist;
    localization.pose.covariance = msg->pose.covariance;

    // Keep pose delivery independent of any /tf writer backpressure.  This
    // ordering also lets the timing diagnostic identify which writer stalls.
    const auto before_localization_publish = std::chrono::steady_clock::now();
    localization_pub_->publish(localization);
    const auto before_tf_publish = std::chrono::steady_clock::now();
    tf_broadcaster_->sendTransform(map_to_camera_init);
    const auto callback_end = std::chrono::steady_clock::now();

    const double localization_publish_ms =
      std::chrono::duration<double, std::milli>(
      before_tf_publish - before_localization_publish).count();
    const double tf_publish_ms =
      std::chrono::duration<double, std::milli>(
      callback_end - before_tf_publish).count();
    const double callback_total_ms =
      std::chrono::duration<double, std::milli>(
      callback_end - callback_start).count();
    const double input_age_ms =
      (get_clock()->now() - rclcpp::Time(msg->header.stamp)).seconds() * 1000.0;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "fusion timing: input_interval=%.1f ms input_age=%.1f ms "
      "localization_publish=%.3f ms tf_publish=%.3f ms total=%.3f ms",
      callback_interval_ms, input_age_ms, localization_publish_ms,
      tf_publish_ms, callback_total_ms);
  }

  Eigen::Vector3d map_to_odom_translation_;
  Eigen::Quaterniond map_to_odom_rotation_;
  std::chrono::steady_clock::time_point previous_callback_time_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr localization_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr map_to_odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransformFusionNode>());
  rclcpp::shutdown();
  return 0;
}
