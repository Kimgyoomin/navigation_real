#pragma once

#include "pongbot_local_graph_insertion_planner/fresh_astar.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"

namespace pongbot_local_graph_insertion_planner
{

bool validPoseQuaternion(const geometry_msgs::msg::Pose & pose);

nav_msgs::msg::Path buildMetricPath(
  const GridSnapshot & grid,
  const SearchResult & result,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const rclcpp::Time & stamp);

double pathLength(const nav_msgs::msg::Path & path);

}  // namespace pongbot_local_graph_insertion_planner
