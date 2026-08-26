#include "pongbot_navigation/simple_pure_pursuit_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/duration.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pongbot_navigation
{
namespace
{

bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

std::vector<pure_pursuit::Point2D> pathPoints(const nav_msgs::msg::Path & path)
{
  std::vector<pure_pursuit::Point2D> points;
  points.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    points.push_back({pose.pose.position.x, pose.pose.position.y});
  }
  return points;
}

}  // namespace

SimplePurePursuitController::SimplePurePursuitController()
: Node("simple_pure_pursuit_controller"),
  last_path_time_(0, 0, get_clock()->get_clock_type()),
  last_control_time_(std::chrono::steady_clock::now())
{
  path_topic_ = declare_parameter("path_topic", "/rubi/heightmap_step_planner/path");
  cmd_vel_topic_ = declare_parameter("cmd_vel_topic", "/cmd_vel");
  map_frame_ = declare_parameter("map_frame", "map");
  base_frame_ = declare_parameter("base_frame", "base_link");
  const double control_frequency_hz = declare_parameter("control_frequency_hz", 20.0);
  transform_timeout_s_ = declare_parameter("transform_timeout_s", 0.03);
  lookahead_distance_m_ = declare_parameter("lookahead_distance_m", 0.35);
  limits_.nominal_linear_velocity = declare_parameter("nominal_linear_velocity_mps", 0.20);
  limits_.min_tracking_velocity = declare_parameter("min_tracking_velocity_mps", 0.05);
  limits_.max_linear_velocity = declare_parameter("max_linear_velocity_mps", 0.70);
  limits_.max_angular_velocity = declare_parameter("max_angular_velocity_rps", 1.50);
  limits_.curvature_velocity_gain = declare_parameter("curvature_velocity_gain", 1.0);
  max_linear_acceleration_mps2_ = declare_parameter("max_linear_acceleration_mps2", 0.50);
  max_angular_acceleration_rps2_ = declare_parameter("max_angular_acceleration_rps2", 1.50);
  goal_tolerance_m_ = declare_parameter("goal_tolerance_m", 0.20);
  max_cross_track_error_m_ = declare_parameter("max_cross_track_error_m", 0.40);
  path_timeout_s_ = declare_parameter("path_timeout_s", 0.0);
  const int nearest_search_backtrack_points =
    declare_parameter("nearest_search_backtrack_points", 2);
  (void)declare_parameter("heading_slowdown_threshold_rad", 0.70);

  if (control_frequency_hz <= 0.0 || transform_timeout_s_ < 0.0 ||
    lookahead_distance_m_ <= 0.0 || limits_.nominal_linear_velocity < 0.0 ||
    limits_.min_tracking_velocity < 0.0 || limits_.max_linear_velocity <= 0.0 ||
    limits_.min_tracking_velocity > limits_.max_linear_velocity ||
    limits_.max_angular_velocity <= 0.0 || limits_.curvature_velocity_gain < 0.0 ||
    max_linear_acceleration_mps2_ <= 0.0 || max_angular_acceleration_rps2_ <= 0.0 ||
    goal_tolerance_m_ < 0.0 || max_cross_track_error_m_ <= 0.0 ||
    nearest_search_backtrack_points < 0)
  {
    throw std::invalid_argument("invalid Simple Pure Pursuit parameter value");
  }
  nearest_search_backtrack_points_ =
    static_cast<std::size_t>(nearest_search_backtrack_points);

  command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  lookahead_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
    "/rubi/simple_pp/lookahead", 1);
  nearest_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
    "/rubi/simple_pp/nearest_path_point", 1);
  path_subscription_ = create_subscription<nav_msgs::msg::Path>(
    path_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&SimplePurePursuitController::onPath, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  const auto period = std::chrono::duration<double>(1.0 / control_frequency_hz);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SimplePurePursuitController::controlTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "Simple Pure Pursuit ready: path=%s cmd=%s frames=%s<-%s frequency=%.1f Hz",
    path_topic_.c_str(), cmd_vel_topic_.c_str(), map_frame_.c_str(), base_frame_.c_str(),
    control_frequency_hz);
}

SimplePurePursuitController::~SimplePurePursuitController()
{
  if (command_publisher_) {
    publishCommand({});
  }
}

bool SimplePurePursuitController::pathIsValid(
  const nav_msgs::msg::Path & path, std::string & reason) const
{
  if (path.poses.empty()) {
    reason = "empty_path";
    return false;
  }
  if (path.header.frame_id != map_frame_) {
    reason = "path_frame_mismatch";
    return false;
  }
  for (const auto & pose : path.poses) {
    if ((!pose.header.frame_id.empty() && pose.header.frame_id != map_frame_) ||
      !finitePose(pose.pose))
    {
      reason = !finitePose(pose.pose) ? "nonfinite_path" : "pose_frame_mismatch";
      return false;
    }
  }
  return true;
}

void SimplePurePursuitController::onPath(nav_msgs::msg::Path::ConstSharedPtr path)
{
  std::string reason;
  if (!pathIsValid(*path, reason)) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_path_.reset();
      has_active_path_ = false;
      nearest_path_index_ = 0;
    }
    publishZeroCommand(reason, true);
    return;
  }

  const std::size_t path_size = path->poses.size();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_path_ = std::move(path);
    nearest_path_index_ = 0;
    has_active_path_ = true;
    previous_command_ = {};
    last_path_time_ = now();
    last_control_time_ = std::chrono::steady_clock::now();
  }
  RCLCPP_INFO(get_logger(), "Accepted replacement Path: size=%zu; progress reset", path_size);
}

void SimplePurePursuitController::controlTimer()
{
  nav_msgs::msg::Path::ConstSharedPtr path;
  std::size_t previous_nearest = 0;
  CoreCommand previous_command;
  rclcpp::Time last_path_time(0, 0, get_clock()->get_clock_type());
  std::chrono::steady_clock::time_point previous_control_time;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_active_path_ || !latest_path_) {
      publishCommand({});
      previous_command_ = {};
      return;
    }
    path = latest_path_;
    previous_nearest = nearest_path_index_;
    previous_command = previous_command_;
    last_path_time = last_path_time_;
    previous_control_time = last_control_time_;
  }

  if (path_timeout_s_ > 0.0 && (now() - last_path_time).seconds() > path_timeout_s_) {
    publishZeroCommand("path_timeout", true);
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      map_frame_, base_frame_, tf2::TimePointZero,
      tf2::durationFromSec(transform_timeout_s_));
  } catch (const tf2::TransformException & exception) {
    publishZeroCommand("tf_unavailable", false);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "Pure Pursuit stopped: TF unavailable: %s",
      exception.what());
    return;
  }

  const Pose2D robot{
    transform.transform.translation.x,
    transform.transform.translation.y,
    tf2::getYaw(transform.transform.rotation)};
  if (!std::isfinite(robot.x) || !std::isfinite(robot.y) || !std::isfinite(robot.yaw)) {
    publishZeroCommand("nonfinite_tf", false);
    return;
  }

  const auto points = pathPoints(*path);
  const Point2D robot_point{robot.x, robot.y};
  const std::size_t nearest = pure_pursuit::findNearestPathIndex(
    points, robot_point, previous_nearest, nearest_search_backtrack_points_);
  const double cross_track_error = pure_pursuit::distance(robot_point, points[nearest]);
  const double goal_distance = pure_pursuit::distance(robot_point, points.back());

  if (pure_pursuit::withinGoalTolerance(robot_point, points.back(), goal_tolerance_m_)) {
    publishZeroCommand("goal_reached", true);
    return;
  }
  if (cross_track_error > max_cross_track_error_m_) {
    publishZeroCommand("cross_track_limit", false);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Pure Pursuit stopped: cross-track error %.3f m exceeds %.3f m",
      cross_track_error, max_cross_track_error_m_);
    return;
  }

  const auto target = pure_pursuit::selectForwardTarget(
    points, robot, nearest, lookahead_distance_m_);
  if (!target) {
    publishZeroCommand("no_forward_target", false);
    return;
  }
  const Point2D target_local = pure_pursuit::toRobotFrame(target->point, robot);
  const auto curvature = pure_pursuit::computeCurvature(target_local);
  if (!curvature) {
    publishZeroCommand("degenerate_target", false);
    return;
  }

  const CoreCommand desired = pure_pursuit::computeDesiredCommand(*curvature, limits_);
  const auto control_time = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(control_time - previous_control_time).count();
  dt = std::clamp(dt, 0.0, 0.25);
  const CoreCommand command = pure_pursuit::rateLimit(
    previous_command, desired, dt,
    max_linear_acceleration_mps2_, max_angular_acceleration_rps2_);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (latest_path_ != path || !has_active_path_) {
      return;
    }
    nearest_path_index_ = nearest;
    previous_command_ = command;
    last_control_time_ = control_time;
  }
  publishCommand(command);
  publishMarker(lookahead_publisher_, target->point, "lookahead", 0.1F, 1.0F, 0.1F);
  publishMarker(nearest_publisher_, points[nearest], "nearest", 0.1F, 0.4F, 1.0F);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "PP tracking: path_size=%zu nearest_idx=%zu lookahead_idx=%zu "
    "lookahead=(%.3f,%.3f) cte=%.3f curvature=%.3f cmd=(%.3f,%.3f) goal_dist=%.3f",
    points.size(), nearest, target->path_index, target->point.x, target->point.y,
    cross_track_error, *curvature, command.linear, command.angular, goal_distance);
}

void SimplePurePursuitController::publishCommand(const CoreCommand & command)
{
  geometry_msgs::msg::Twist message;
  message.linear.x = std::clamp(command.linear, 0.0, limits_.max_linear_velocity);
  message.angular.z = std::clamp(
    command.angular, -limits_.max_angular_velocity, limits_.max_angular_velocity);
  command_publisher_->publish(message);
}

void SimplePurePursuitController::publishZeroCommand(
  const std::string & reason, bool deactivate)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    previous_command_ = {};
    last_control_time_ = std::chrono::steady_clock::now();
    if (deactivate) {
      has_active_path_ = false;
      nearest_path_index_ = 0;
    }
  }
  publishCommand({});
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 1000, "Pure Pursuit zero command: stop_reason=%s",
    reason.c_str());
}

void SimplePurePursuitController::publishMarker(
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher,
  const Point2D & point,
  const std::string & marker_namespace,
  float red, float green, float blue)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = map_frame_;
  marker.header.stamp = now();
  marker.ns = marker_namespace;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.12;
  marker.scale.y = 0.12;
  marker.scale.z = 0.12;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = 1.0F;
  publisher->publish(marker);
}

}  // namespace pongbot_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pongbot_navigation::SimplePurePursuitController>());
  rclcpp::shutdown();
  return 0;
}
