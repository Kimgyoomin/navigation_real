#include "pongbot_global_planner/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pongbot_global_planner
{

namespace
{

struct OpenNode
{
  unsigned int index;
  double f;
};

struct OpenNodeCompare
{
  bool operator()(const OpenNode & a, const OpenNode & b) const
  {
    // priority_queue is max-heap by default, so reverse for min-heap
    return a.f > b.f;
  }
};

}  // namespace

void AstarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node in AstarPlanner::configure");
  }

  logger_ = node->get_logger();
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  global_frame_ = costmap_ros_->getGlobalFrameID();

  RCLCPP_INFO(
    logger_,
    "Configured planner plugin: %s (global_frame: %s)",
    name_.c_str(),
    global_frame_.c_str());
}

void AstarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up planner plugin: %s", name_.c_str());
  tf_.reset();
  costmap_ros_.reset();
}

void AstarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating planner plugin: %s", name_.c_str());
}

void AstarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating planner plugin: %s", name_.c_str());
}

nav_msgs::msg::Path AstarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = start.header.stamp;
  path.header.frame_id = global_frame_;

  if (!costmap_ros_) {
    RCLCPP_ERROR(logger_, "costmap_ros_ is null (configure not called?)");
    return path;
  }

  auto * costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    RCLCPP_ERROR(logger_, "Costmap pointer is null.");
    return path;
  }

  // --------------------------------------------------------------------------
  // 1) Transform start / goal into global frame
  // --------------------------------------------------------------------------
  geometry_msgs::msg::PoseStamped start_g = start;
  geometry_msgs::msg::PoseStamped goal_g = goal;

  try {
    if (tf_ && start.header.frame_id != global_frame_) {
      start_g = tf_->transform(start, global_frame_, tf2::durationFromSec(0.1));
    }
    if (tf_ && goal.header.frame_id != global_frame_) {
      goal_g = tf_->transform(goal, global_frame_, tf2::durationFromSec(0.1));
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "TF transform failed: %s", ex.what());
    return path;
  }

  // --------------------------------------------------------------------------
  // 2) Convert world -> map indices
  // --------------------------------------------------------------------------
  unsigned int sx, sy, gx, gy;
  if (!costmap->worldToMap(start_g.pose.position.x, start_g.pose.position.y, sx, sy)) {
    RCLCPP_WARN(
      logger_,
      "Start outside costmap bounds: world=(%.3f, %.3f)",
      start_g.pose.position.x,
      start_g.pose.position.y);
    return path;
  }

  if (!costmap->worldToMap(goal_g.pose.position.x, goal_g.pose.position.y, gx, gy)) {
    RCLCPP_WARN(
      logger_,
      "Goal outside costmap bounds: world=(%.3f, %.3f)",
      goal_g.pose.position.x,
      goal_g.pose.position.y);
    return path;
  }

  const unsigned int size_x = costmap->getSizeInCellsX();
  const unsigned int size_y = costmap->getSizeInCellsY();
  const unsigned int total = size_x * size_y;

  auto toIndex = [size_x](unsigned int x, unsigned int y) -> unsigned int {
      return y * size_x + x;
    };

  auto toXY = [size_x](unsigned int index, unsigned int & x, unsigned int & y) {
      x = index % size_x;
      y = index / size_x;
    };

  // --------------------------------------------------------------------------
  // 3) Traversability rule
  //
  // Policy:
  //   - UNKNOWN              : blocked
  //   - LETHAL obstacle      : blocked
  //   - Inflated cost region : traversable, but penalized
  //
  // This is important because if you block INSCRIBED_INFLATED_OBSTACLE and above,
  // doors / narrow passages often become disconnected.
  // --------------------------------------------------------------------------
  auto isTraversable = [costmap](unsigned int x, unsigned int y) -> bool {
      const unsigned char c = costmap->getCost(x, y);

      // Unknown => do not allow
      if (c == nav2_costmap_2d::NO_INFORMATION) {
        return false;
      }

      // Only lethal obstacle is strictly forbidden
      if (c >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        return false;
      }

      return true;
    };

  const unsigned char start_cost = costmap->getCost(sx, sy);
  const unsigned char goal_cost = costmap->getCost(gx, gy);

  RCLCPP_INFO(
    logger_,
    "Planning request: start_world=(%.3f, %.3f) goal_world=(%.3f, %.3f)",
    start_g.pose.position.x, start_g.pose.position.y,
    goal_g.pose.position.x, goal_g.pose.position.y);

  RCLCPP_INFO(
    logger_,
    "Planning request: start_map=(%u, %u, cost=%u) goal_map=(%u, %u, cost=%u)",
    sx, sy, static_cast<unsigned int>(start_cost),
    gx, gy, static_cast<unsigned int>(goal_cost));

  if (!isTraversable(sx, sy)) {
    RCLCPP_WARN(
      logger_,
      "Start not traversable: map=(%u, %u), cost=%u",
      sx, sy, static_cast<unsigned int>(start_cost));
    return path;
  }

  if (!isTraversable(gx, gy)) {
    RCLCPP_WARN(
      logger_,
      "Goal not traversable: map=(%u, %u), cost=%u",
      gx, gy, static_cast<unsigned int>(goal_cost));
    return path;
  }

  const unsigned int start_i = toIndex(sx, sy);
  const unsigned int goal_i = toIndex(gx, gy);

  // --------------------------------------------------------------------------
  // 4) Heuristic (Octile distance for 8-connected grid)
  // --------------------------------------------------------------------------
  auto heuristic = [gx, gy](unsigned int x, unsigned int y) -> double {
      const double dx = std::abs(static_cast<int>(x) - static_cast<int>(gx));
      const double dy = std::abs(static_cast<int>(y) - static_cast<int>(gy));
      const double D = 1.0;
      const double D2 = std::sqrt(2.0);
      return D * (dx + dy) + (D2 - 2.0 * D) * std::min(dx, dy);
    };

  // --------------------------------------------------------------------------
  // 5) Cost penalty
  //
  // Since inflated areas are traversable, they should become "expensive", not blocked.
  // You can tune cost_scale later (e.g. 2.0, 5.0, 10.0) if planner still hugs walls.
  // --------------------------------------------------------------------------
  const double cost_scale = 10.0;
  auto stepPenalty = [costmap, cost_scale](unsigned int x, unsigned int y) -> double {
      const unsigned char c = costmap->getCost(x, y);
      return cost_scale * (static_cast<double>(c) / 255.0);
    };

  std::vector<double> g_score(total, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(total, -1);
  std::vector<bool> closed(total, false);

  std::priority_queue<OpenNode, std::vector<OpenNode>, OpenNodeCompare> open;
  g_score[start_i] = 0.0;
  open.push(OpenNode{start_i, heuristic(sx, sy)});

  const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const double step8[8] = {
    1.0, 1.0, 1.0, 1.0,
    std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0)
  };

  bool found = false;
  std::size_t expanded_nodes = 0;

  // --------------------------------------------------------------------------
  // 6) A* search
  // --------------------------------------------------------------------------
  while (!open.empty()) {
    const auto current = open.top();
    open.pop();

    const unsigned int ci = current.index;
    if (closed[ci]) {
      continue;
    }
    closed[ci] = true;
    ++expanded_nodes;

    if (ci == goal_i) {
      found = true;
      break;
    }

    unsigned int cx, cy;
    toXY(ci, cx, cy);

    for (int k = 0; k < 8; ++k) {
      const int tx = static_cast<int>(cx) + dx8[k];
      const int ty = static_cast<int>(cy) + dy8[k];

      if (tx < 0 || ty < 0) {
        continue;
      }

      const unsigned int nx = static_cast<unsigned int>(tx);
      const unsigned int ny = static_cast<unsigned int>(ty);

      if (nx >= size_x || ny >= size_y) {
        continue;
      }

      if (!isTraversable(nx, ny)) {
        continue;
      }

      // Prevent diagonal corner-cutting
      if (dx8[k] != 0 && dy8[k] != 0) {
        const unsigned int adj_x = static_cast<unsigned int>(static_cast<int>(cx) + dx8[k]);
        const unsigned int adj_y = static_cast<unsigned int>(static_cast<int>(cy) + dy8[k]);

        if (!isTraversable(adj_x, cy) || !isTraversable(cx, adj_y)) {
          continue;
        }
      }

      const unsigned int ni = toIndex(nx, ny);
      if (closed[ni]) {
        continue;
      }

      const double tentative_g = g_score[ci] + step8[k] + stepPenalty(nx, ny);

      if (tentative_g < g_score[ni]) {
        g_score[ni] = tentative_g;
        came_from[ni] = static_cast<int>(ci);
        const double f = tentative_g + heuristic(nx, ny);
        open.push(OpenNode{ni, f});
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(
      logger_,
      "A* failed to find a path. expanded_nodes=%zu start=(%u,%u,cost=%u) goal=(%u,%u,cost=%u)",
      expanded_nodes,
      sx, sy, static_cast<unsigned int>(start_cost),
      gx, gy, static_cast<unsigned int>(goal_cost));
    return path;
  }

  // --------------------------------------------------------------------------
  // 7) Reconstruct path indices
  // --------------------------------------------------------------------------
  std::vector<unsigned int> indices;
  indices.reserve(512);

  unsigned int trace = goal_i;
  indices.push_back(trace);

  while (trace != start_i) {
    const int prev = came_from[trace];
    if (prev < 0) {
      RCLCPP_WARN(logger_, "Reconstruction failed: broken came_from");
      return nav_msgs::msg::Path{};
    }
    trace = static_cast<unsigned int>(prev);
    indices.push_back(trace);
  }

  std::reverse(indices.begin(), indices.end());

  // --------------------------------------------------------------------------
  // 8) Convert indices -> world poses
  // --------------------------------------------------------------------------
  path.poses.clear();
  path.poses.reserve(indices.size() + 2);

  // exact start
  geometry_msgs::msg::PoseStamped ps = start_g;
  ps.header.frame_id = global_frame_;
  path.poses.push_back(ps);

  for (std::size_t i = 1; i + 1 < indices.size(); ++i) {
    unsigned int mx, my;
    toXY(indices[i], mx, my);

    double wx, wy;
    costmap->mapToWorld(mx, my, wx, wy);

    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = wx;
    p.pose.position.y = wy;
    p.pose.position.z = 0.0;
    path.poses.push_back(p);
  }

  // exact goal
  geometry_msgs::msg::PoseStamped pg = goal_g;
  pg.header.frame_id = global_frame_;
  path.poses.push_back(pg);

  // --------------------------------------------------------------------------
  // 8.5) Simple path smoothing
  //
  // Purpose:
  //   - Reduce jagged zigzag path caused by 8-connected grid A*
  //   - Keep exact start / exact goal unchanged
  //   - Only one very light smoothing pass
  //
  // Method:
  //   - 3-point moving average
  //   - For each interior point i:
  //       p_i_new = (p_{i-1} + p_i + p_{i+1}) / 3
  //
  // Why here?
  //   - Smoothing should be done after map->world conversion
  //   - Orientation must be computed AFTER smoothing
  // --------------------------------------------------------------------------
  if (path.poses.size() >= 3) {
    auto smoothed_poses = path.poses;

    for (std::size_t i = 1; i + 1 < path.poses.size(); ++i) {
      smoothed_poses[i].pose.position.x =
        (path.poses[i - 1].pose.position.x +
        path.poses[i].pose.position.x +
        path.poses[i + 1].pose.position.x) / 3.0;

      smoothed_poses[i].pose.position.y =
        (path.poses[i - 1].pose.position.y +
        path.poses[i].pose.position.y +
        path.poses[i + 1].pose.position.y) / 3.0;

      // Keep z as-is (currently planar path)
      smoothed_poses[i].pose.position.z = path.poses[i].pose.position.z;
    }

    path.poses = std::move(smoothed_poses);
  }


  // --------------------------------------------------------------------------
  // 9) Heading orientation
  // --------------------------------------------------------------------------
  if (path.poses.size() >= 2) {
    for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
      const auto & a = path.poses[i].pose.position;
      const auto & b = path.poses[i + 1].pose.position;
      const double yaw = std::atan2(b.y - a.y, b.x - a.x);

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      path.poses[i].pose.orientation = tf2::toMsg(q);
    }

    path.poses.back().pose.orientation =
      path.poses[path.poses.size() - 2].pose.orientation;
  }

  RCLCPP_INFO(
    logger_,
    "A* path found. num_points=%zu expanded_nodes=%zu",
    path.poses.size(),
    expanded_nodes);

  return path;
}

}  // namespace pongbot_global_planner

PLUGINLIB_EXPORT_CLASS(pongbot_global_planner::AstarPlanner, nav2_core::GlobalPlanner)