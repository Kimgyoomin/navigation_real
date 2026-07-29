#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string> 

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <lcm/lcm-cpp.hpp>
#include "robotlcm/joy_cmd.hpp"

using namespace std::chrono_literals;

class CmdVelToLcmNode : public rclcpp::Node
{
public:
  CmdVelToLcmNode()
  : Node("cmdvel_to_lcm")
  {
    // ------------------------------------------------------------------------
    // ROS / LCM interface parameters
    // ------------------------------------------------------------------------
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    lcm_channel_ = declare_parameter<std::string>("lcm_channel", "NAV_CMD_VEL");
    lcm_url_ = declare_parameter<std::string>(
      "lcm_url", "udpm://239.255.76.67:7667?ttl=1");

    // ------------------------------------------------------------------------
    // Publish / safety parameters
    // ------------------------------------------------------------------------
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    deadman_timeout_s_ = declare_parameter<double>("deadman_timeout_s", 1.0);

    // ------------------------------------------------------------------------
    // Scaling and saturation
    //
    // Important:
    //   This node does NOT apply hidden multipliers.
    //   Final LCM values are:
    //     sign * scale * clamped ROS command
    // ------------------------------------------------------------------------
    scale_linear_x_ = declare_parameter<double>("scale_linear_x", 1.0);
    scale_linear_y_ = declare_parameter<double>("scale_linear_y", 1.0);
    scale_angular_z_ = declare_parameter<double>("scale_angular_z", 1.0);

    sign_linear_x_ = declare_parameter<double>("sign_linear_x", 1.0);
    sign_linear_y_ = declare_parameter<double>("sign_linear_y", 1.0);
    sign_angular_z_ = declare_parameter<double>("sign_angular_z", 1.0);

    max_linear_x_ = declare_parameter<double>("max_linear_x", 1.0);
    max_linear_y_ = declare_parameter<double>("max_linear_y", 0.75);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 1.57);

    // ------------------------------------------------------------------------
    // Optional mode button injection
    //
    // Keep these disabled by default. If the low-level controller requires
    // a POLICY_ON / WALK_READY button bit to accept velocity commands,
    // enable it explicitly from the launch file.
    // ------------------------------------------------------------------------
    publish_policy_button_ =
      declare_parameter<bool>("publish_policy_button", false);
    policy_button_index_ =
      declare_parameter<int>("policy_button_index", 8);

    publish_walk_ready_button_ =
      declare_parameter<bool>("publish_walk_ready_button", false);
    walk_ready_button_index_ =
      declare_parameter<int>("walk_ready_button_index", 10);

    publish_stop_button_ =
      declare_parameter<bool>("publish_stop_button", false);
    stop_button_index_ =
      declare_parameter<int>("stop_button_index", 1);

    debug_ = declare_parameter<bool>("debug", true);

    // ------------------------------------------------------------------------
    // LCM initialization
    // ------------------------------------------------------------------------
    lcm_ = std::make_unique<lcm::LCM>(lcm_url_);
    if (!lcm_->good()) {
      throw std::runtime_error("LCM init failed with url: " + lcm_url_);
    }

    // ------------------------------------------------------------------------
    // ROS subscription
    //
    // nav2_rubi_pointb.yaml has:
    //   enable_stamped_cmd_vel: false
    // Therefore controller_server publishes geometry_msgs/msg/Twist.
    // ------------------------------------------------------------------------
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      rclcpp::QoS(10),
      std::bind(&CmdVelToLcmNode::cmdVelCallback, this, std::placeholders::_1));

    last_cmd_time_ = now();

    const double safe_hz = std::max(1.0, publish_rate_hz_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / safe_hz)),
      std::bind(&CmdVelToLcmNode::publishTick, this));

    RCLCPP_INFO(
      get_logger(),
      "[cmdvel_to_lcm] READY | topic=%s channel=%s url=%s rate=%.1fHz timeout=%.2fs",
      cmd_vel_topic_.c_str(),
      lcm_channel_.c_str(),
      lcm_url_.c_str(),
      safe_hz,
      deadman_timeout_s_);

    RCLCPP_INFO(
      get_logger(),
      "[cmdvel_to_lcm] limits | max_x=%.3f max_y=%.3f max_yaw=%.3f "
      "scale=(%.3f, %.3f, %.3f) sign=(%.1f, %.1f, %.1f)",
      max_linear_x_,
      max_linear_y_,
      max_angular_z_,
      scale_linear_x_,
      scale_linear_y_,
      scale_angular_z_,
      sign_linear_x_,
      sign_linear_y_,
      sign_angular_z_);
  }

private:
  static double clamp(double value, double min_value, double max_value)
  {
    return std::max(min_value, std::min(value, max_value));
  }

  static bool validButtonIndex(int index)
  {
    return index >= 0 && index < 16;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    received_cmd_ = true;
    last_cmd_time_ = now();

    const double x = clamp(
      msg->linear.x,
      -std::abs(max_linear_x_),
      std::abs(max_linear_x_));

    const double y = clamp(
      msg->linear.y,
      -std::abs(max_linear_y_),
      std::abs(max_linear_y_));

    const double yaw = clamp(
      msg->angular.z,
      -std::abs(max_angular_z_),
      std::abs(max_angular_z_));

    last_linear_x_ = sign_linear_x_ * scale_linear_x_ * x;
    last_linear_y_ = sign_linear_y_ * scale_linear_y_ * y;
    last_angular_z_ = sign_angular_z_ * scale_angular_z_ * yaw;

    if (debug_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[cmdvel_to_lcm] ROS cmd_vel received | x=%.3f y=%.3f yaw=%.3f "
        "-> LCM x=%.3f y=%.3f yaw=%.3f",
        msg->linear.x,
        msg->linear.y,
        msg->angular.z,
        last_linear_x_,
        last_linear_y_,
        last_angular_z_);
    }
  }

  void publishTick()
  {
    double linear_x = 0.0;
    double linear_y = 0.0;
    double angular_z = 0.0;

    if (received_cmd_) {
      const double dt = (now() - last_cmd_time_).seconds();

      if (dt <= deadman_timeout_s_) {
        linear_x = last_linear_x_;
        linear_y = last_linear_y_;
        angular_z = last_angular_z_;
        deadman_active_ = false;
      } else {
        if (!deadman_active_) {
          deadman_active_ = true;
          RCLCPP_WARN(
            get_logger(),
            "[cmdvel_to_lcm] /cmd_vel timeout %.3fs > %.3fs -> zero command",
            dt,
            deadman_timeout_s_);
        }
      }
    }

    robotlcm::joy_cmd out{};
    out.utime = static_cast<int64_t>(now().nanoseconds() / 1000);
    out.seq = seq_++;

    out.linear_x = static_cast<float>(linear_x);
    out.linear_y = static_cast<float>(linear_y);
    out.angular_z = static_cast<float>(angular_z);

    for (int i = 0; i < 8; ++i) {
      out.axes[i] = 0.0f;
    }

    for (int i = 0; i < 16; ++i) {
      out.buttons[i] = false;
    }

    if (publish_policy_button_ && validButtonIndex(policy_button_index_)) {
      out.buttons[policy_button_index_] = true;
    }

    if (publish_walk_ready_button_ && validButtonIndex(walk_ready_button_index_)) {
      out.buttons[walk_ready_button_index_] = true;
    }

    if (publish_stop_button_ && validButtonIndex(stop_button_index_)) {
      out.buttons[stop_button_index_] = true;
    }

    lcm_->publish(lcm_channel_, &out);

    if (debug_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[cmdvel_to_lcm] LCM publish | seq=%d x=%.3f y=%.3f yaw=%.3f "
        "policy=%d walk_ready=%d stop=%d",
        out.seq,
        out.linear_x,
        out.linear_y,
        out.angular_z,
        publish_policy_button_,
        publish_walk_ready_button_,
        publish_stop_button_);
    }
  }

private:
  // ROS / LCM interface
  std::string cmd_vel_topic_;
  std::string lcm_channel_;
  std::string lcm_url_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::unique_ptr<lcm::LCM> lcm_;

  // State
  bool received_cmd_{false};
  bool deadman_active_{false};
  rclcpp::Time last_cmd_time_;
  int32_t seq_{0};

  double last_linear_x_{0.0};
  double last_linear_y_{0.0};
  double last_angular_z_{0.0};

  // Parameters
  double publish_rate_hz_{50.0};
  double deadman_timeout_s_{0.3};

  double scale_linear_x_{1.0};
  double scale_linear_y_{1.0};
  double scale_angular_z_{1.0};

  double sign_linear_x_{1.0};
  double sign_linear_y_{1.0};
  double sign_angular_z_{1.0};

  double max_linear_x_{1.0};
  double max_linear_y_{0.75};
  double max_angular_z_{1.57};

  bool publish_policy_button_{false};
  int policy_button_index_{8};

  bool publish_walk_ready_button_{false};
  int walk_ready_button_index_{10};

  bool publish_stop_button_{false};
  int stop_button_index_{1};

  bool debug_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToLcmNode>());
  rclcpp::shutdown();
  return 0;
}
