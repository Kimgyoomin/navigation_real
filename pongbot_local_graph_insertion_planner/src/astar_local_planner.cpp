#include "pongbot_local_graph_insertion_planner/astar_local_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pongbot_local_graph_insertion_planner
{
namespace
{

bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.orientation.x) && std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) && std::isfinite(pose.orientation.w);
}

bool insideGrid(const GridSnapshot & grid, long long x, long long y)
{
  return x >= 0 && y >= 0 &&
         x < static_cast<long long>(grid.size_x) &&
         y < static_cast<long long>(grid.size_y);
}

struct WorldBounds
{
  double min_x{std::numeric_limits<double>::infinity()};
  double min_y{std::numeric_limits<double>::infinity()};
  double max_x{-std::numeric_limits<double>::infinity()};
  double max_y{-std::numeric_limits<double>::infinity()};

  void include(const tf2::Vector3 & point)
  {
    min_x = std::min(min_x, point.x());
    min_y = std::min(min_y, point.y());
    max_x = std::max(max_x, point.x());
    max_y = std::max(max_y, point.y());
  }
};

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
  allow_unknown_ = declare("allow_unknown", allow_unknown_);
  blocked_cost_threshold_ = declare("blocked_cost_threshold", blocked_cost_threshold_);
  const auto ratio = declare("incremental_change_ratio_threshold", 0.20);
  max_planning_time_ = declare("max_planning_time", max_planning_time_);
  (void)declare("enable_shortcutting", false);
  publish_debug_fused_grid_ = declare("publish_debug_fused_grid", publish_debug_fused_grid_);
  debug_fused_grid_topic_ = declare("debug_fused_grid_topic", debug_fused_grid_topic_);

  const bool invalid_parameters =
    blocked_cost_threshold_ < 1 ||
    blocked_cost_threshold_ > 255 ||
    local_costmap_timeout_ <= 0.0 ||
    transform_timeout_ < 0.0 ||
    max_planning_time_ <= 0.0 ||
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
  debug_publisher_.reset();
  dstar_ = DStarLite(0.20);
  previous_fused_ = {};
  costmap_ros_.reset();
  tf_.reset();
}

void AstarLocalPlanner::activate() {}

void AstarLocalPlanner::deactivate() {}

void AstarLocalPlanner::localCostmapCallback(nav2_msgs::msg::Costmap::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(local_mutex_);
  local_snapshot_ = {
    std::move(message),
    rclcpp::Clock(RCL_ROS_TIME).now()
  };
}

GridSnapshot AstarLocalPlanner::copyGlobalCostmap() const
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

  const auto cell_count = snapshot.size_x * snapshot.size_y;
  const auto * source = costmap->getCharMap();
  snapshot.costs.assign(source, source + cell_count);
  return snapshot;
}

bool AstarLocalPlanner::fuseLocalOverlay(
  GridSnapshot & fused,
  std::size_t & overlay_cells,
  double & local_age,
  std::string & failure) const
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
  if (!std::isfinite(local_age) || local_age > local_costmap_timeout_) {
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
    local_grid.metadata.resolution <= 0.0;
  if (invalid_local_grid) {
    failure = "local_costmap_invalid";
    return false;
  }

  tf2::Transform global_from_local, local_origin;
  tf2::fromMsg(transform.transform, global_from_local);
  tf2::fromMsg(local_grid.metadata.origin, local_origin);

  const auto blocked_threshold = static_cast<unsigned char>(blocked_cost_threshold_);
  const double local_resolution = local_grid.metadata.resolution;
  for (std::size_t local_y = 0; local_y < height; ++local_y) {
    for (std::size_t local_x = 0; local_x < width; ++local_x) {
      const auto cost = local_grid.data[local_y * width + local_x];

      // Unknown and inflation soft costs remain available to the controller, but
      // only actual obstacles become D* Lite graph changes.
      if (cost == 255 || cost < blocked_threshold) {
        continue;
      }

      WorldBounds footprint;
      for (int corner_y = 0; corner_y < 2; ++corner_y) {
        for (int corner_x = 0; corner_x < 2; ++corner_x) {
          const tf2::Vector3 local_corner(
            (local_x + corner_x) * local_resolution,
            (local_y + corner_y) * local_resolution,
            0.0);
          footprint.include(global_from_local * local_origin * local_corner);
        }
      }

      const auto min_grid_x = static_cast<long long>(
        std::floor((footprint.min_x - fused.origin_x) / fused.resolution));
      const auto min_grid_y = static_cast<long long>(
        std::floor((footprint.min_y - fused.origin_y) / fused.resolution));
      const auto max_grid_x = static_cast<long long>(
        std::floor((footprint.max_x - fused.origin_x) / fused.resolution));
      const auto max_grid_y = static_cast<long long>(
        std::floor((footprint.max_y - fused.origin_y) / fused.resolution));

      for (long long global_y = min_grid_y; global_y <= max_grid_y; ++global_y) {
        for (long long global_x = min_grid_x; global_x <= max_grid_x; ++global_x) {
          if (!insideGrid(fused, global_x, global_y)) {
            continue;
          }

          const auto cell = fused.index(
            static_cast<std::size_t>(global_x),
            static_cast<std::size_t>(global_y));
          auto & target = fused.costs[cell];
          const auto previous_cost = target;
          target = std::max(target, cost);
          if (target != previous_cost) {
            ++overlay_cells;
          }
        }
      }
    }
  }
  return true;
}

void AstarLocalPlanner::publishFusedGrid(const GridSnapshot & grid) const
{
  if (!debug_publisher_) {
    return;
  }

  nav_msgs::msg::OccupancyGrid message;
  message.header.stamp = parent_.lock()->now();
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

geometry_msgs::msg::Quaternion AstarLocalPlanner::normalizedQuaternion(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
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
  switch (dstar_.lastFallbackReason()) {
    case FallbackReason::kChangedRatio:
      return "FRESH_FALLBACK";
    case FallbackReason::kInitialPlan:
      return "FRESH_INIT";
    default:
      return "DSTAR_REPAIR";
  }
}

nav_msgs::msg::Path AstarLocalPlanner::buildPath(
  const GridSnapshot & grid,
  const SearchResult & result,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_;
  path.header.stamp = parent_.lock()->now();
  path.poses.reserve(result.path.size());

  for (std::size_t i = 0; i < result.path.size(); ++i) {
    const auto cell = result.path[i];
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = grid.origin_x + (grid.x(cell) + 0.5) * grid.resolution;
    pose.pose.position.y = grid.origin_y + (grid.y(cell) + 0.5) * grid.resolution;

    if (i + 1 < result.path.size()) {
      const auto next = result.path[i + 1];
      const double dx =
        static_cast<double>(grid.x(next)) - static_cast<double>(grid.x(cell));
      const double dy =
        static_cast<double>(grid.y(next)) - static_cast<double>(grid.y(cell));
      pose.pose.orientation = normalizedQuaternion(std::atan2(dy, dx));
    } else {
      pose.pose.orientation = normalizedQuaternion(tf2::getYaw(goal.pose.orientation));
    }
    path.poses.push_back(pose);
  }

  // Preserve the exact poses supplied by Nav2 at both endpoints.
  path.poses.front() = start;
  path.poses.back() = goal;
  return path;
}

// createPlan
// 좌표 변환, 변경 셀 계산, planning mode, path 생성 함수로 분리
nav_msgs::msg::Path AstarLocalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path empty_path;
  empty_path.header.frame_id = frame_;
  empty_path.header.stamp = parent_.lock()->now();

  std::lock_guard<std::mutex> lock(planner_mutex_);

  const bool invalid_endpoint =
    start.header.frame_id != frame_ ||
    goal.header.frame_id != frame_ ||
    !finitePose(start.pose) ||
    !finitePose(goal.pose);
  if (invalid_endpoint) {
    RCLCPP_ERROR(logger_, "[AstarLocal] invalid endpoint/frame");
    return empty_path;
  }

  GridSnapshot fused = copyGlobalCostmap();
  std::size_t overlay_cells = 0;
  double local_age = -1.0;
  std::string failure;
  if (!fuseLocalOverlay(fused, overlay_cells, local_age, failure)) {
    RCLCPP_WARN(logger_, "[AstarLocal] planning failure: %s", failure.c_str());
    return empty_path;
  }

  std::size_t start_cell = 0;
  std::size_t goal_cell = 0;
  if (!poseToCell(fused, start.pose, start_cell) ||
    !poseToCell(fused, goal.pose, goal_cell))
  {
    RCLCPP_WARN(logger_, "[AstarLocal] endpoint out of bounds");
    return empty_path;
  }

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(max_planning_time_);
  SearchOptions options;
  options.allow_unknown = allow_unknown_;
  options.blocked_cost_threshold = static_cast<unsigned char>(blocked_cost_threshold_);
  options.cancelled = [deadline] {
      return std::chrono::steady_clock::now() > deadline;
    };

  const std::size_t changed_cells = countChangedCells(fused);
  const SearchResult result = dstar_.replan(fused, start_cell, goal_cell, options);
  publishFusedGrid(fused);

  RCLCPP_INFO(
    logger_,
    "[AstarLocal] mode=%s changed_cells=%zu overlay_cells=%zu local_age=%.3f "
    "fallback_reason=%s path_cost=%.6f path_size=%zu",
    planningMode(),
    changed_cells,
    overlay_cells,
    local_age,
    fallbackReasonName(dstar_.lastFallbackReason()),
    result.cost,
    result.path.size());

  previous_fused_ = fused;
  if (result.status != SearchStatus::kSuccess ||
    !validPath(fused, result.path, options))
  {
    RCLCPP_WARN(logger_, "[AstarLocal] planning failure: %s", result.reason.c_str());
    return empty_path;
  }

  return buildPath(fused, result, start, goal);
}
}  // namespace pongbot_local_graph_insertion_planner
PLUGINLIB_EXPORT_CLASS(
  pongbot_local_graph_insertion_planner::AstarLocalPlanner,
  nav2_core::GlobalPlanner)
