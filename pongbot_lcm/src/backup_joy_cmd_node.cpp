엉 joy_cmd_t.lcm이랑 joy_lcm_node.cpp는 현재 다음과 같거든?
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <lcm/lcm-cpp.hpp>
#include "robotlcm/cmd_vel_t.hpp"

using namespace std::chrono_literals;

class JoyToLcmNode : public rclcpp::Node
{
public:
  JoyToLcmNode()
  : Node("joy_to_lcm")
  {
    joy_topic_    = declare_parameter<std::string>("joy_topic", "/joy");
    lcm_channel_  = declare_parameter<std::string>("lcm_channel", "CMD_VEL");
    lcm_url_      = declare_parameter<std::string>("lcm_url", "udpm://239.255.76.67:7667?ttl=255");

    // Asus ROG Ally X 실측 기반
    // axes[1] : left stick up    -> +1.0
    // axes[0] : left stick right -> -1.0
    // axes[3] : angular z        -> sign invert
    axis_linear_x_  = declare_parameter<int>("axis_linear_x", 1);
    axis_linear_y_  = declare_parameter<int>("axis_linear_y", 0);
    axis_angular_z_ = declare_parameter<int>("axis_angular_z", 3);

    sign_linear_x_  = declare_parameter<double>("sign_linear_x", 1.0);
    sign_linear_y_  = declare_parameter<double>("sign_linear_y", -1.0);
    sign_angular_z_ = declare_parameter<double>("sign_angular_z", -1.0);

    max_linear_x_   = declare_parameter<double>("max_linear_x", 1.0);
    max_linear_y_   = declare_parameter<double>("max_linear_y", 1.0);
    max_angular_z_  = declare_parameter<double>("max_angular_z", 1.0);

    scale_linear_x_  = declare_parameter<double>("scale_linear_x", 1.0);
    scale_linear_y_  = declare_parameter<double>("scale_linear_y", 1.0);
    scale_angular_z_ = declare_parameter<double>("scale_angular_z", 1.0);

    axis_deadzone_ = declare_parameter<double>("axis_deadzone", 0.08);

    publish_rate_hz_   = declare_parameter<double>("publish_rate_hz", 50.0);
    deadman_timeout_s_ = declare_parameter<double>("deadman_timeout_s", 0.3);

    debug_raw_joy_ = declare_parameter<bool>("debug_raw_joy", true);

    lcm_ = std::make_unique<lcm::LCM>(lcm_url_);
    if (!lcm_->good()) {
      throw std::runtime_error("LCM init failed. Check lcm_url/iface/permissions.");
    }

    sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&JoyToLcmNode::joyCb, this, std::placeholders::_1));

    last_joy_time_ = now();

    const double hz = std::max(1.0, publish_rate_hz_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / hz)),
      std::bind(&JoyToLcmNode::publishTick, this));

    RCLCPP_INFO(
      get_logger(),
      "[joy_to_lcm] sub=%s | LCM url=%s chan=%s | pub=%.1fHz deadman=%.2fs",
      joy_topic_.c_str(), lcm_url_.c_str(), lcm_channel_.c_str(),
      publish_rate_hz_, deadman_timeout_s_);

    RCLCPP_INFO(
      get_logger(),
      "[joy_to_lcm] axis map: linear_x=axes[%d], linear_y=axes[%d], angular_z=axes[%d]",
      axis_linear_x_, axis_linear_y_, axis_angular_z_);

    RCLCPP_INFO(
      get_logger(),
      "[joy_to_lcm] sign: linear_x=%.1f, linear_y=%.1f, angular_z=%.1f",
      sign_linear_x_, sign_linear_y_, sign_angular_z_);
  }

private:
  static inline double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  double applyDeadzone(double v) const
  {
    return (std::abs(v) < axis_deadzone_) ? 0.0 : v;
  }

  double getAxisValue(const std::vector<float> & axes, int idx) const
  {
    if (idx < 0 || idx >= static_cast<int>(axes.size())) {
      return 0.0;
    }
    return static_cast<double>(axes[idx]);
  }

  void joyCb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    received_joy_ = true;
    last_joy_time_ = now();

    const double raw_lx = applyDeadzone(getAxisValue(msg->axes, axis_linear_x_));
    const double raw_ly = applyDeadzone(getAxisValue(msg->axes, axis_linear_y_));
    const double raw_az = applyDeadzone(getAxisValue(msg->axes, axis_angular_z_));

    // joy normalized [-1, 1] -> physical command range [-max, max]
    last_lx_ = clamp(
      raw_lx * sign_linear_x_ * max_linear_x_ * scale_linear_x_,
      -max_linear_x_, max_linear_x_);

    last_ly_ = clamp(
      raw_ly * sign_linear_y_ * max_linear_y_ * scale_linear_y_,
      -max_linear_y_, max_linear_y_);

    last_az_ = clamp(
      raw_az * sign_angular_z_ * max_angular_z_ * scale_angular_z_,
      -max_angular_z_, max_angular_z_);

    if (debug_raw_joy_) {
      const double a0 = getAxisValue(msg->axes, 0);
      const double a1 = getAxisValue(msg->axes, 1);
      const double a2 = getAxisValue(msg->axes, 2);
      const double a3 = getAxisValue(msg->axes, 3);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "raw axes: [0]=%.3f [1]=%.3f [2]=%.3f [3]=%.3f | mapped cmd: lx=%.3f ly=%.3f az=%.3f",
        a0, a1, a2, a3, last_lx_, last_ly_, last_az_);
    }
  }

  void publishTick()
  {
    double lx = 0.0;
    double ly = 0.0;
    double az = 0.0;

    if (received_joy_) {
      const double dt = (now() - last_joy_time_).seconds();

      if (dt <= deadman_timeout_s_) {
        lx = last_lx_;
        ly = last_ly_;
        az = last_az_;
        deadman_active_ = false;
      } else {
        if (!deadman_active_) {
          deadman_active_ = true;
          RCLCPP_WARN(
            get_logger(),
            "joy timeout %.3fs -> sending ZERO at %.1fHz",
            dt, publish_rate_hz_);
        }
      }
    }

    robotlcm::cmd_vel_t out{};
    out.seq = seq_++;
    out.utime = static_cast<int64_t>(now().nanoseconds() / 1000);
    out.linear_x  = static_cast<float>(lx);
    out.linear_y  = static_cast<float>(ly);
    out.angular_z = static_cast<float>(az);
    // printf("TEST : %f\n", out.linear_x);
    // printf("TEST : %f\n", out.linear_x);
    lcm_->publish(lcm_channel_, &out);
  }

private:
  std::string joy_topic_;
  std::string lcm_channel_;
  std::string lcm_url_;

  int axis_linear_x_{1};
  int axis_linear_y_{0};
  int axis_angular_z_{3};

  double sign_linear_x_{1.0};
  double sign_linear_y_{-1.0};
  double sign_angular_z_{-1.0};

  double max_linear_x_{1.0};
  double max_linear_y_{0.2};
  double max_angular_z_{0.4};

  double scale_linear_x_{1.0};
  double scale_linear_y_{1.0};
  double scale_angular_z_{1.0};

  double axis_deadzone_{0.08};
  double publish_rate_hz_{50.0};
  double deadman_timeout_s_{0.3};

  bool debug_raw_joy_{true};
  bool received_joy_{false};
  bool deadman_active_{false};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Time last_joy_time_;

  std::unique_ptr<lcm::LCM> lcm_;

  int32_t seq_{0};
  double last_lx_{0.0};
  double last_ly_{0.0};
  double last_az_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToLcmNode>());
  rclcpp::shutdown();
  return 0;
}
