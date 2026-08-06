#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <lcm/lcm-cpp.hpp>
#include "robotlcm/joy_cmd.hpp"

using namespace std::chrono_literals;

class JoyToLcmNode : public rclcpp::Node
{
public:
  JoyToLcmNode()
  : Node("joy_to_lcm")
  {
    joy_topic_   = declare_parameter<std::string>("joy_topic", "/joy");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    lcm_channel_ = declare_parameter<std::string>("lcm_channel", "CMD_VEL");
    lcm_url_     = declare_parameter<std::string>("lcm_url", "udpm://239.255.76.67:7667?ttl=255");

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
      throw std::runtime_error("LCM init failed.");
    }

    sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&JoyToLcmNode::joyCb, this, std::placeholders::_1));

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      rclcpp::QoS(10),
      std::bind(&JoyToLcmNode::cmdVelCb, this, std::placeholders::_1));

    last_joy_time_ = now();

    const double hz = std::max(1.0, publish_rate_hz_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / hz)),
      std::bind(&JoyToLcmNode::publishTick, this));

    RCLCPP_INFO(get_logger(), "[joy_to_lcm] READY");
  }

private:
  // -------------------------
  // CMD_VEL CALLBACK
  // -------------------------
  void cmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_lx_ = msg->linear.x;
    last_cmd_ly_ = msg->linear.y;
    last_cmd_az_ = msg->angular.z;
  }

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
    if (idx < 0 || idx >= static_cast<int>(axes.size())) return 0.0;
    return static_cast<double>(axes[idx]);
  }

  bool getButton(const std::vector<int> & buttons, int idx) const
  {
    if (idx < 0 || idx >= static_cast<int>(buttons.size())) return false;
    return buttons[idx] != 0;
  }

  // -------------------------
  // JOY CALLBACK
  // -------------------------
  void joyCb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    received_joy_ = true;
    last_joy_time_ = now();

    // ---- velocity 계산 유지 ----
    const double raw_lx = applyDeadzone(getAxisValue(msg->axes, axis_linear_x_));
    const double raw_ly = applyDeadzone(getAxisValue(msg->axes, axis_linear_y_));
    const double raw_az = applyDeadzone(getAxisValue(msg->axes, axis_angular_z_));

    last_lx_ = clamp(raw_lx * sign_linear_x_ * max_linear_x_ * scale_linear_x_,
                     -max_linear_x_, max_linear_x_);

    last_ly_ = clamp(raw_ly * sign_linear_y_ * max_linear_y_ * scale_linear_y_,
                     -max_linear_y_, max_linear_y_);

    last_az_ = clamp(raw_az * sign_angular_z_ * max_angular_z_ * scale_angular_z_,
                     -max_angular_z_, max_angular_z_);

    // ---- axes 저장 ----
    for (int i = 0; i < 8; ++i)
      last_axes_[i] = 0.0f;

    for (size_t i = 0; i < msg->axes.size() && i < 8; ++i)
      last_axes_[i] = msg->axes[i];

    // ---- 버튼 remap ----
    for (int i = 0; i < 16; ++i)
      last_buttons_[i] = false;

    // 기존 button[13] = WALK READY <- ROG[6]
    // last_buttons_[13] = getButton(msg->buttons, 6);  // Pongbot  
    last_buttons_[10] = getButton(msg->buttons, 6);     // RUBI

    // 기존 button[0] = TROT ← ROG[2]
    // last_buttons_[0] = getButton(msg->buttons, 2);   // Pongbot Q
    last_buttons_[8] = getButton(msg->buttons, 2);      // RUBI

    // 기존 button[1] = STOP ← ROG[0]
    last_buttons_[1] = getButton(msg->buttons, 0);      

    // 기존 button[9] = TORQUE OFF ← ROG[7]
    last_buttons_[9] = getButton(msg->buttons, 7);      // Pongbot Q, RUBI

    if (debug_raw_joy_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        // "JOY remap | TROT=%d STOP=%d TORQUE=%d, linear_x=%.2f, linear_y=%.2f, ang_z=%.2f",
        // last_buttons_[0], last_buttons_[1], last_buttons_[9], last_lx_, last_ly_, last_az_);
        "JOY remap | WALKREADY=%d POLICYON=%d TORQUEOFF=%d, linear_x=%.2f, linear_y=%.2f, ang_z=%.2f",
        last_buttons_[10], last_buttons_[8], last_buttons_[9], last_lx_, last_ly_, last_az_);
    }
  }

  // -------------------------
  // PUBLISH
  // -------------------------
  void publishTick()
  {
    double lx = 0.0;
    double ly = 0.0;
    double az = 0.0;

    if (received_joy_) {
      const double dt = (now() - last_joy_time_).seconds();

      if (dt <= deadman_timeout_s_) {
        lx = last_lx_;
        ly = -last_ly_;
        az = -last_az_;
        deadman_active_ = false;
      } else {
        if (!deadman_active_) {
          deadman_active_ = true;
          RCLCPP_WARN(get_logger(), "joy timeout -> zero cmd");
        }
      }
    }

    robotlcm::joy_cmd out{};
    out.seq = seq_++;
    out.utime = static_cast<int64_t>(now().nanoseconds() / 1000);

    // velocity from Nav2 /cmd_vel
    out.linear_x  = static_cast<float>(last_cmd_lx_);
    out.linear_y  = static_cast<float>(last_cmd_ly_);
    out.angular_z = static_cast<float>(last_cmd_az_);

    // axes
    for (int i = 0; i < 8; ++i)
      out.axes[i] = last_axes_[i];

    // buttons
    for (int i = 0; i < 16; ++i)
      out.buttons[i] = last_buttons_[i];

    lcm_->publish(lcm_channel_, &out);
  }

private:
  std::string joy_topic_;
  std::string cmd_vel_topic_;
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
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Time last_joy_time_;

  std::unique_ptr<lcm::LCM> lcm_;

  int32_t seq_{0};

  double last_lx_{0.0};
  double last_ly_{0.0};
  double last_az_{0.0};

  double last_cmd_lx_{0.0};
  double last_cmd_ly_{0.0};
  double last_cmd_az_{0.0};

  float last_axes_[8]{};
  bool  last_buttons_[16]{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToLcmNode>());
  rclcpp::shutdown();
  return 0;
}
