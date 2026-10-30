#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "pongbot_navigation/velocity_feedback.hpp"

class FastlioVelocityFeedback : public rclcpp::Node
{
public:
  FastlioVelocityFeedback() : Node("fastlio_velocity_feedback")
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    auto input = declare_parameter<std::string>("input_topic", "/fastlio/odometry_velocity", descriptor);
    auto output = declare_parameter<std::string>("output_topic", "/fastlio/odometry_base", descriptor);
    body_frame_ = declare_parameter<std::string>("input_child_frame", "body", descriptor);
    world_frame_ = declare_parameter<std::string>("input_frame", "camera_init", descriptor);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link", descriptor);
    max_age_ = declare_parameter<double>("max_age_s", .25, descriptor);
    future_tolerance_ = declare_parameter<double>("future_tolerance_s", .05, descriptor);
    if (input.empty() || output.empty() || body_frame_.empty() || base_frame_.empty() ||
        world_frame_.empty() || !std::isfinite(max_age_) || max_age_ <= 0 ||
        !std::isfinite(future_tolerance_) || future_tolerance_ < 0 ||
        get_node_topics_interface()->resolve_topic_name(input) ==
        get_node_topics_interface()->resolve_topic_name(output))
    {
      throw std::invalid_argument("Invalid velocity feedback configuration");
    }
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // Separate TF listener thread; lookup never waits inside the odometry callback.
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output, rclcpp::QoS(1).reliable());
    subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
      input, rclcpp::SensorDataQoS().keep_last(1),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {receive(*msg);});
    RCLCPP_INFO(get_logger(),
      "FAST-LIO velocity feedback %s -> %s: %s origin -> %s origin; reliable output, original stamp; max age %.3fs. No command watchdog.",
      input.c_str(), output.c_str(), body_frame_.c_str(), base_frame_.c_str(), max_age_);
  }

private:
  void receive(const nav_msgs::msg::Odometry & msg)
  {
    const auto begin = std::chrono::steady_clock::now();
    const auto current = now();
    // Permit a new epoch after /clock rewinds, without accepting old backlog.
    if (last_clock_ns_ && current.nanoseconds() < last_clock_ns_) {last_stamp_ns_ = 0;}
    last_clock_ns_ = current.nanoseconds();
    if (msg.header.stamp.sec < 0 || msg.header.stamp.nanosec >= 1000000000u) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Dropping invalid velocity timestamp");
      return;
    }
    const rclcpp::Time stamp(msg.header.stamp, current.get_clock_type());
    const double age = (current - stamp).seconds();
    if (msg.child_frame_id != body_frame_ || msg.header.frame_id != world_frame_ ||
        stamp.nanoseconds() <= 0 || stamp.nanoseconds() <= last_stamp_ns_ ||
        age > max_age_ || age < -future_tolerance_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Dropping stale/out-of-order/wrong-frame velocity (age %.3fs); never substituting zero or now()", age);
      return;
    }
    try {
      const auto mount = buffer_->lookupTransform(body_frame_, base_frame_, stamp,
        rclcpp::Duration::from_seconds(0.));
      const auto out = pongbot_navigation::velocity_at_base(msg, mount);
      publisher_->publish(out);
      last_stamp_ns_ = stamp.nanoseconds();
      const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now()-begin).count();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "velocity_feedback age_ms=%.2f work_ms=%.3f vx=%.3f vy=%.3f wz=%.3f",
        age*1000., ms, out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.angular.z);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Velocity feedback skipped (no fallback): %s", e.what());
    }
  }

  std::string body_frame_, world_frame_, base_frame_;
  double max_age_, future_tolerance_;
  int64_t last_stamp_ns_{0}, last_clock_ns_{0};
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<FastlioVelocityFeedback>());}
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("fastlio_velocity_feedback"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
