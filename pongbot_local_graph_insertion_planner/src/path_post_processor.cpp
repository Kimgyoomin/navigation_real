#include "pongbot_local_graph_insertion_planner/path_post_processor.hpp"

#include <cmath>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pongbot_local_graph_insertion_planner
{
namespace
{

geometry_msgs::msg::Quaternion normalizedQuaternion(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  quaternion.normalize();
  return tf2::toMsg(quaternion);
}

geometry_msgs::msg::PoseStamped normalizedPose(
  geometry_msgs::msg::PoseStamped pose,
  const std_msgs::msg::Header & header)
{
  tf2::Quaternion quaternion;
  tf2::fromMsg(pose.pose.orientation, quaternion);
  quaternion.normalize();
  pose.pose.orientation = tf2::toMsg(quaternion);
  pose.header = header;
  return pose;
}

}  // namespace

bool validPoseQuaternion(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double squared_norm =
    q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(q.x) && std::isfinite(q.y) &&
         std::isfinite(q.z) && std::isfinite(q.w) &&
         std::isfinite(squared_norm) && squared_norm > 1e-12;
}

nav_msgs::msg::Path buildMetricPath(
  const GridSnapshot & grid,
  const SearchResult & result,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = grid.frame_id;
  path.header.stamp = stamp;
  if (result.status != SearchStatus::kSuccess || result.path.empty() ||
    !validPoseQuaternion(start.pose) || !validPoseQuaternion(goal.pose))
  {
    return path;
  }

  // A one-cell search still needs both exact poses so RPP can rotate to the
  // supplied goal orientation without losing the current start pose.
  if (result.path.size() == 1) {
    path.poses.push_back(normalizedPose(start, path.header));
    path.poses.push_back(normalizedPose(goal, path.header));
    return path;
  }

  path.poses.reserve(result.path.size());
  path.poses.push_back(normalizedPose(start, path.header));

  for (std::size_t i = 1; i + 1 < result.path.size(); ++i) {
    const auto cell = result.path[i];
    const auto next = result.path[i + 1];
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x =
      grid.origin_x + (static_cast<double>(grid.x(cell)) + 0.5) * grid.resolution;
    pose.pose.position.y =
      grid.origin_y + (static_cast<double>(grid.y(cell)) + 0.5) * grid.resolution;
    const double dx =
      static_cast<double>(grid.x(next)) - static_cast<double>(grid.x(cell));
    const double dy =
      static_cast<double>(grid.y(next)) - static_cast<double>(grid.y(cell));
    pose.pose.orientation = normalizedQuaternion(std::atan2(dy, dx));
    path.poses.push_back(pose);
  }

  path.poses.push_back(normalizedPose(goal, path.header));
  return path;
}

double pathLength(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto & previous = path.poses[i - 1].pose.position;
    const auto & current = path.poses[i].pose.position;
    length += std::hypot(current.x - previous.x, current.y - previous.y);
  }
  return length;
}

}  // namespace pongbot_local_graph_insertion_planner
