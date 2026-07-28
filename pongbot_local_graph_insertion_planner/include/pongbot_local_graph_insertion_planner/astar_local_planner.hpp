#pragma once

#include "pongbot_local_graph_insertion_planner/dstar_lite.hpp"
#include "pongbot_local_graph_insertion_planner/local_overlay.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace pongbot_local_graph_insertion_planner
{
class AstarLocalPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  struct LocalSnapshot
  {
    nav2_msgs::msg::Costmap::SharedPtr message;
  };

  void localCostmapCallback(nav2_msgs::msg::Costmap::SharedPtr message);
  GridSnapshot copyGlobalCostmap();

  bool fuseLocalOverlay(
    GridSnapshot & fused,
    std::size_t & overlay_cells,
    double & local_age,
    std::string & failure);

  bool transformPoseToGlobal(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output,
    std::string & failure) const;

  bool poseToCell(
    const GridSnapshot & grid,
    const geometry_msgs::msg::Pose & pose,
    std::size_t & cell) const;

  std::size_t countChangedCells(const GridSnapshot & grid) const;
  const char * planningMode() const;

  void publishFusedGrid(const GridSnapshot & grid) const;
  void logMetrics(
    const char * mode,
    double planning_time_ms,
    const SearchResult & result,
    std::size_t changed_cells,
    std::size_t overlay_cells,
    double local_age,
    double path_length,
    std::uint64_t snapshot_version,
    const char * fallback_reason,
    const std::string & failure_reason) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("astar_local")};
  std::string name_;
  std::string frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr local_subscription_;
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr fused_costmap_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_publisher_;
  mutable std::mutex local_mutex_;
  LocalSnapshot local_snapshot_;
  mutable std::mutex planner_mutex_;
  DStarLite dstar_{0.20};
  GridSnapshot previous_fused_;
  bool require_local_costmap_{true};
  bool allow_unknown_{false};
  bool allow_latest_transform_fallback_{false};
  bool publish_debug_fused_grid_{true};
  bool enable_incremental_reuse_{false};
  bool have_last_overlay_transform_{false};
  int blocked_cost_threshold_{253};
  double local_costmap_timeout_{2.0};
  double transform_timeout_{0.2};
  double max_planning_time_{1.0};
  double cost_penalty_scale_{1.0};
  double transform_jump_translation_threshold_{1.0};
  double transform_jump_yaw_threshold_{0.785};
  std::uint64_t snapshot_version_{0};
  Transform2D last_overlay_transform_;
  std::string local_unknown_policy_{"ignore"};
  std::string local_costmap_topic_{"/local_costmap/costmap_raw"};
  std::string fused_costmap_topic_{"/astar_local/fused_costmap_raw"};
  std::string debug_fused_grid_topic_{"/astar_local/fused_grid"};
};
}  // namespace pongbot_local_graph_insertion_planner
