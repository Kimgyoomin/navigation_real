#ifndef PONGBOT_NAVIGATION__SIMPLE_PURE_PURSUIT_CONTROLLER_HPP_
#define PONGBOT_NAVIGATION__SIMPLE_PURE_PURSUIT_CONTROLLER_HPP_

#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

#include "pongbot_navigation/pure_pursuit_core.hpp"

namespace pongbot_navigation
{

class SimplePurePursuitController : public rclcpp::Node
{
public:
  SimplePurePursuitController();
  ~SimplePurePursuitController() override;

private:
  using CoreCommand = pure_pursuit::Command;
  using Point2D = pure_pursuit::Point2D;
  using Pose2D = pure_pursuit::Pose2D;

  void onPath(nav_msgs::msg::Path::ConstSharedPtr path);
  void controlTimer();
  void publishCommand(const CoreCommand & command);
  void publishZeroCommand(const std::string & reason, bool deactivate);
  void publishMarker(
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher,
    const Point2D & point,
    const std::string & marker_namespace,
    float red, float green, float blue);
  bool pathIsValid(const nav_msgs::msg::Path & path, std::string & reason) const;

  std::string path_topic_;
  std::string cmd_vel_topic_;
  std::string map_frame_;
  std::string base_frame_;
  double transform_timeout_s_{0.03};
  double lookahead_distance_m_{0.35};
  double max_linear_acceleration_mps2_{0.50};
  double max_angular_acceleration_rps2_{1.50};
  double goal_tolerance_m_{0.20};
  double max_cross_track_error_m_{0.40};
  double path_timeout_s_{0.0};
  std::size_t nearest_search_backtrack_points_{2};
  pure_pursuit::Limits limits_;

  std::mutex state_mutex_;
  nav_msgs::msg::Path::ConstSharedPtr latest_path_;
  std::size_t nearest_path_index_{0};
  bool has_active_path_{false};
  CoreCommand previous_command_;
  rclcpp::Time last_path_time_;
  std::chrono::steady_clock::time_point last_control_time_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr nearest_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace pongbot_navigation

#endif  // PONGBOT_NAVIGATION__SIMPLE_PURE_PURSUIT_CONTROLLER_HPP_
