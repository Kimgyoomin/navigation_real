#include "pongbot_local_graph_insertion_planner/astar_local_planner.hpp"
#include "pongbot_local_graph_insertion_planner/path_post_processor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pongbot_local_graph_insertion_planner
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && validPoseQuaternion(pose);
}

bool insideGrid(const GridSnapshot & grid, long long x, long long y)
{
  return x >= 0 && y >= 0 &&
         x < static_cast<long long>(grid.size_x) &&
         y < static_cast<long long>(grid.size_y);
}

double normalizedAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

}  // namespace

void AstarLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);

  auto node = parent_.lock();
  if (!node || !costmap_ros_) {
    throw std::runtime_error("AstarLocal requires lifecycle node and global costmap");
  }

  logger_ = node->get_logger();
  frame_ = costmap_ros_->getGlobalFrameID();

  auto declare = [&node, this](const std::string & key, auto value) {
      return node->declare_parameter(name_ + "." + key, value);
    };

  local_costmap_topic_ = declare("local_costmap_topic", local_costmap_topic_);
  require_local_costmap_ = declare("require_local_costmap", require_local_costmap_);
  local_costmap_timeout_ = declare("local_costmap_timeout", local_costmap_timeout_);
  transform_timeout_ = declare("transform_timeout", transform_timeout_);
  allow_latest_transform_fallback_ = declare(
    "allow_latest_transform_fallback",
    allow_latest_transform_fallback_);
  transform_jump_translation_threshold_ = declare(
    "transform_jump_translation_threshold",
    transform_jump_translation_threshold_);
  transform_jump_yaw_threshold_ = declare(
    "transform_jump_yaw_threshold",
    transform_jump_yaw_threshold_);
  local_unknown_policy_ = declare("local_unknown_policy", local_unknown_policy_);
  allow_unknown_ = declare("allow_unknown", allow_unknown_);
  blocked_cost_threshold_ = declare("blocked_cost_threshold", blocked_cost_threshold_);
  cost_penalty_scale_ = declare("cost_penalty_scale", cost_penalty_scale_);
  const auto ratio = declare("incremental_change_ratio_threshold", 0.20);
  max_planning_time_ = declare("max_planning_time", max_planning_time_);
  enable_incremental_reuse_ = declare(
    "enable_incremental_reuse",
    enable_incremental_reuse_);
  fused_costmap_topic_ = declare("fused_costmap_topic", fused_costmap_topic_);
  publish_debug_fused_grid_ = declare("publish_debug_fused_grid", publish_debug_fused_grid_);
  debug_fused_grid_topic_ = declare("debug_fused_grid_topic", debug_fused_grid_topic_);

  const bool invalid_parameters =
    blocked_cost_threshold_ < 1 ||
    blocked_cost_threshold_ > 255 ||
    local_costmap_timeout_ <= 0.0 ||
    transform_timeout_ < 0.0 ||
    transform_jump_translation_threshold_ < 0.0 ||
    transform_jump_yaw_threshold_ < 0.0 ||
    max_planning_time_ <= 0.0 ||
    !std::isfinite(cost_penalty_scale_) ||
    cost_penalty_scale_ < 0.0 ||
    local_unknown_policy_ != "ignore" ||
    ratio<0.0 ||
      ratio>1.0;
  if (invalid_parameters) {
    throw std::runtime_error("AstarLocal parameter contract violation");
  }

  dstar_ = DStarLite(ratio);
  local_subscription_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    local_costmap_topic_,
    rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&AstarLocalPlanner::localCostmapCallback, this, std::placeholders::_1));
  fused_costmap_publisher_ = node->create_publisher<nav2_msgs::msg::Costmap>(
    fused_costmap_topic_,
    rclcpp::QoS(1).transient_local().reliable());

  if (publish_debug_fused_grid_) {
    debug_publisher_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      debug_fused_grid_topic_,
      rclcpp::QoS(1).transient_local().reliable());
  }
}

void AstarLocalPlanner::cleanup()
{
  std::lock_guard<std::mutex> lock(planner_mutex_);
  local_subscription_.reset();
  fused_costmap_publisher_.reset();
  debug_publisher_.reset();
  dstar_ = DStarLite(0.20);
  previous_fused_ = {};
  have_last_overlay_transform_ = false;
  snapshot_version_ = 0;
  costmap_ros_.reset();
  tf_.reset();
}

void AstarLocalPlanner::activate() {}

void AstarLocalPlanner::deactivate() {}

void AstarLocalPlanner::localCostmapCallback(nav2_msgs::msg::Costmap::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(local_mutex_);
  local_snapshot_ = {std::move(message)};
}

GridSnapshot AstarLocalPlanner::copyGlobalCostmap()
{
  GridSnapshot snapshot;
  auto * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  snapshot.frame_id = frame_;
  snapshot.size_x = costmap->getSizeInCellsX();
  snapshot.size_y = costmap->getSizeInCellsY();
  snapshot.resolution = costmap->getResolution();
  snapshot.origin_x = costmap->getOriginX();
  snapshot.origin_y = costmap->getOriginY();
  snapshot.version = ++snapshot_version_;

  const auto cell_count = snapshot.size_x * snapshot.size_y;
  const auto * source = costmap->getCharMap();
  snapshot.costs.assign(source, source + cell_count);
  return snapshot;
}

bool AstarLocalPlanner::fuseLocalOverlay(
  GridSnapshot & fused,
  std::size_t & overlay_cells,
  double & local_age,
  std::string & failure)
{
  LocalSnapshot local;
  {
    std::lock_guard<std::mutex> lock(local_mutex_);
    local = local_snapshot_;
  }

  const auto node = parent_.lock();
  const auto now = node->now();
  if (!local.message) {
    failure = "local_costmap_missing";
    return !require_local_costmap_;
  }

  local_age = (now - rclcpp::Time(local.message->header.stamp)).seconds();
  if (!std::isfinite(local_age) || std::abs(local_age) > local_costmap_timeout_) {
    failure = "local_costmap_stale";
    return !require_local_costmap_;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(
      frame_,
      local.message->header.frame_id,
      rclcpp::Time(local.message->header.stamp),
      rclcpp::Duration::from_seconds(transform_timeout_));
  } catch (const tf2::TransformException & e) {
    if (!allow_latest_transform_fallback_) {
      failure = std::string("local_tf_failed: ") + e.what();
      return false;
    }
    try {
      transform = tf_->lookupTransform(
        frame_,
        local.message->header.frame_id,
        rclcpp::Time(0, 0, RCL_ROS_TIME),
        rclcpp::Duration::from_seconds(transform_timeout_));
    } catch (const tf2::TransformException & latest) {
      failure = std::string("local_tf_latest_failed: ") + latest.what();
      return false;
    }
  }

  const auto & local_grid = *local.message;
  const auto width = local_grid.metadata.size_x;
  const auto height = local_grid.metadata.size_y;
  const bool invalid_local_grid =
    width == 0 ||
    height == 0 ||
    local_grid.data.size() != static_cast<std::size_t>(width) * height ||
    !std::isfinite(local_grid.metadata.resolution) ||
    local_grid.metadata.resolution <= 0.0;
  if (invalid_local_grid) {
    failure = "local_costmap_invalid";
    return false;
  }

  tf2::Transform global_from_local;
  tf2::Transform local_origin;
  tf2::fromMsg(transform.transform, global_from_local);
  tf2::fromMsg(local_grid.metadata.origin, local_origin);

  const tf2::Transform global_from_local_grid = global_from_local * local_origin;
  const Transform2D planar_transform{
    global_from_local_grid.getOrigin().x(),
    global_from_local_grid.getOrigin().y(),
    tf2::getYaw(global_from_local_grid.getRotation())};

  if (have_last_overlay_transform_) {
    const double translation_jump = std::hypot(
      planar_transform.x - last_overlay_transform_.x,
      planar_transform.y - last_overlay_transform_.y);
    const double yaw_jump = std::abs(
      normalizedAngle(
        planar_transform.yaw - last_overlay_transform_.yaw));
    if (translation_jump > transform_jump_translation_threshold_ ||
      yaw_jump > transform_jump_yaw_threshold_)
    {
      failure = "local_tf_jump";
      return false;
    }
  }

  LocalGrid local_costs;
  local_costs.size_x = width;
  local_costs.size_y = height;
  local_costs.resolution = local_grid.metadata.resolution;
  local_costs.costs = local_grid.data;
  const auto overlay = applyLocalOverlay(fused, local_costs, planar_transform);
  if (!overlay.success) {
    failure = overlay.failure_reason;
    return false;
  }

  last_overlay_transform_ = planar_transform;
  have_last_overlay_transform_ = true;
  overlay_cells = overlay.overlay_cells;
  return true;
}

void AstarLocalPlanner::publishFusedGrid(const GridSnapshot & grid) const
{
  const auto stamp = parent_.lock()->now();
  nav2_msgs::msg::Costmap raw_message;
  raw_message.header.stamp = stamp;
  raw_message.header.frame_id = grid.frame_id;
  raw_message.metadata.map_load_time = stamp;
  raw_message.metadata.update_time = stamp;
  raw_message.metadata.layer = "astar_local_fused";
  raw_message.metadata.resolution = grid.resolution;
  raw_message.metadata.size_x = grid.size_x;
  raw_message.metadata.size_y = grid.size_y;
  raw_message.metadata.origin.position.x = grid.origin_x;
  raw_message.metadata.origin.position.y = grid.origin_y;
  raw_message.metadata.origin.orientation.w = 1.0;
  raw_message.data = grid.costs;
  fused_costmap_publisher_->publish(raw_message);

  if (!debug_publisher_) {
    return;
  }

  nav_msgs::msg::OccupancyGrid message;
  message.header.stamp = stamp;
  message.header.frame_id = grid.frame_id;
  message.info.resolution = grid.resolution;
  message.info.width = grid.size_x;
  message.info.height = grid.size_y;
  message.info.origin.position.x = grid.origin_x;
  message.info.origin.position.y = grid.origin_y;
  message.info.origin.orientation.w = 1.0;

  message.data.resize(grid.costs.size());
  for (std::size_t cell = 0; cell < grid.costs.size(); ++cell) {
    if (grid.costs[cell] == 255) {
      message.data[cell] = -1;
      continue;
    }
    const int occupancy = static_cast<int>(grid.costs[cell]) * 100 / 252;
    message.data[cell] = static_cast<int8_t>(std::min(100, occupancy));
  }
  debug_publisher_->publish(message);
}

bool AstarLocalPlanner::transformPoseToGlobal(
  const geometry_msgs::msg::PoseStamped & input,
  geometry_msgs::msg::PoseStamped & output,
  std::string & failure) const
{
  if (!finitePose(input.pose) || input.header.frame_id.empty()) {
    failure = "invalid_endpoint";
    return false;
  }

  if (input.header.frame_id == frame_) {
    output = input;
    output.header.frame_id = frame_;
    return true;
  }

  try {
    output = tf_->transform(
      input, frame_, tf2::durationFromSec(transform_timeout_));
    output.header.frame_id = frame_;
    if (!finitePose(output.pose)) {
      failure = "endpoint_tf_non_finite";
      return false;
    }
    return true;
  } catch (const tf2::TransformException & exception) {
    failure = std::string("endpoint_tf_failed: ") + exception.what();
    return false;
  }
}

bool AstarLocalPlanner::poseToCell(
  const GridSnapshot & grid,
  const geometry_msgs::msg::Pose & pose,
  std::size_t & cell) const
{
  const auto x = static_cast<long long>(
    std::floor((pose.position.x - grid.origin_x) / grid.resolution));
  const auto y = static_cast<long long>(
    std::floor((pose.position.y - grid.origin_y) / grid.resolution));
  if (!insideGrid(grid, x, y)) {
    return false;
  }

  cell = grid.index(static_cast<std::size_t>(x), static_cast<std::size_t>(y));
  return true;
}

std::size_t AstarLocalPlanner::countChangedCells(const GridSnapshot & grid) const
{
  if (!previous_fused_.valid() || !previous_fused_.geometryEquals(grid)) {
    return grid.costs.size();
  }

  std::size_t changed_cells = 0;
  for (std::size_t cell = 0; cell < grid.costs.size(); ++cell) {
    changed_cells += grid.costs[cell] != previous_fused_.costs[cell];
  }
  return changed_cells;
}

const char * AstarLocalPlanner::planningMode() const
{
  if (!enable_incremental_reuse_) {
    return "FRESH_ASTAR";
  }

  switch (dstar_.lastFallbackReason()) {
    case FallbackReason::kChangedRatio:
    case FallbackReason::kInvariantViolation:
    case FallbackReason::kExtractionFailure:
      return "DSTAR_FRESH_FALLBACK";
    case FallbackReason::kInitialPlan:
    case FallbackReason::kGeometryChanged:
    case FallbackReason::kGoalChanged:
    case FallbackReason::kOptionsChanged:
      return "DSTAR_INITIAL";
    default:
      return "DSTAR_REPAIR";
  }
}

void AstarLocalPlanner::logMetrics(
  const char * mode,
  double planning_time_ms,
  const SearchResult & result,
  std::size_t changed_cells,
  std::size_t overlay_cells,
  double local_age,
  double path_length,
  std::uint64_t snapshot_version,
  const char * fallback_reason,
  const std::string & failure_reason) const
{
  RCLCPP_INFO(
    logger_,
    "[AstarLocalMetrics] mode=%s planning_time_ms=%.3f expanded_nodes=%zu "
    "changed_cells=%zu overlay_cells=%zu local_costmap_age_sec=%.3f "
    "path_cells=%zu path_length_m=%.6f path_cost=%.6f snapshot_version=%llu "
    "fallback_reason=%s failure_reason=%s",
    mode,
    planning_time_ms,
    result.expanded,
    changed_cells,
    overlay_cells,
    local_age,
    result.path.size(),
    path_length,
    result.cost,
    static_cast<unsigned long long>(snapshot_version),
    fallback_reason,
    failure_reason.empty() ? "none" : failure_reason.c_str());
}

nav_msgs::msg::Path AstarLocalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  const auto planning_started = std::chrono::steady_clock::now();
  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = frame_;
  empty_path.header.stamp = parent_.lock()->now();

  std::lock_guard<std::mutex> lock(planner_mutex_);
  SearchResult result;
  std::size_t changed_cells = 0;
  std::size_t overlay_cells = 0;
  double local_age = -1.0;
  std::uint64_t snapshot_version = 0;
  std::string failure;

  const auto elapsedMilliseconds = [&planning_started]() {
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - planning_started).count();
    };
  const auto logFailure = [&](const std::string & reason) {
      logMetrics(
        "PREPROCESS_FAILURE", elapsedMilliseconds(), result, changed_cells,
        overlay_cells, local_age, 0.0, snapshot_version, "none", reason);
    };

  geometry_msgs::msg::PoseStamped transformed_start;
  geometry_msgs::msg::PoseStamped transformed_goal;
  if (!transformPoseToGlobal(start, transformed_start, failure) ||
    !transformPoseToGlobal(goal, transformed_goal, failure))
  {
    logFailure(failure);
    return empty_path;
  }

  GridSnapshot fused = copyGlobalCostmap();
  snapshot_version = fused.version;
  if (!fuseLocalOverlay(fused, overlay_cells, local_age, failure)) {
    logFailure(failure);
    return empty_path;
  }

  std::size_t start_cell = 0;
  std::size_t goal_cell = 0;
  if (!poseToCell(fused, transformed_start.pose, start_cell) ||
    !poseToCell(fused, transformed_goal.pose, goal_cell))
  {
    logFailure("endpoint_out_of_bounds");
    return empty_path;
  }

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(max_planning_time_);
  SearchOptions options;
  options.allow_unknown = allow_unknown_;
  options.blocked_cost_threshold = static_cast<unsigned char>(blocked_cost_threshold_);
  options.cost_penalty = cost_penalty_scale_;
  options.cancelled = [deadline] {
      return std::chrono::steady_clock::now() > deadline;
    };

  changed_cells = countChangedCells(fused);
  if (enable_incremental_reuse_) {
    result = dstar_.replan(fused, start_cell, goal_cell, options);
  } else {
    result = freshAstar(fused, start_cell, goal_cell, options);
  }
  publishFusedGrid(fused);
  previous_fused_ = fused;

  if (result.status != SearchStatus::kSuccess ||
    !validPath(fused, result.path, options))
  {
    failure = result.reason.empty() ? "invalid_search_path" : result.reason;
    logMetrics(
      planningMode(), elapsedMilliseconds(), result, changed_cells, overlay_cells,
      local_age, 0.0, snapshot_version,
      enable_incremental_reuse_ ?
      fallbackReasonName(dstar_.lastFallbackReason()) : "none",
      failure);
    return empty_path;
  }

  auto path = buildMetricPath(
    fused, result, transformed_start, transformed_goal, parent_.lock()->now());
  if (path.poses.empty()) {
    failure = "path_post_processing_failed";
    logMetrics(
      planningMode(), elapsedMilliseconds(), result, changed_cells, overlay_cells,
      local_age, 0.0, snapshot_version,
      enable_incremental_reuse_ ?
      fallbackReasonName(dstar_.lastFallbackReason()) : "none",
      failure);
    return empty_path;
  }

  logMetrics(
    planningMode(), elapsedMilliseconds(), result, changed_cells, overlay_cells,
    local_age, pathLength(path), snapshot_version,
    enable_incremental_reuse_ ?
    fallbackReasonName(dstar_.lastFallbackReason()) : "none",
    "");
  return path;
}
}  // namespace pongbot_local_graph_insertion_planner
PLUGINLIB_EXPORT_CLASS(
  pongbot_local_graph_insertion_planner::AstarLocalPlanner,
  nav2_core::GlobalPlanner)
