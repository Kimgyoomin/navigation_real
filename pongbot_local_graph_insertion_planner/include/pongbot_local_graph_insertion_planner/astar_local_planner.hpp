#pragma once

#include "pongbot_local_graph_insertion_planner/dstar_lite.hpp"

#include <memory>
#include <mutex>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace pongbot_local_graph_insertion_planner {
class AstarLocalPlanner : public nav2_core::GlobalPlanner {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  struct LocalSnapshot {
    nav2_msgs::msg::Costmap::SharedPtr message;
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
  };
  void localCostmapCallback(nav2_msgs::msg::Costmap::SharedPtr message);
  GridSnapshot copyGlobalCostmap() const;
  bool fuseLocalOverlay(GridSnapshot & fused, std::size_t & overlay_cells, double & local_age,
    std::string & failure) const;
  void publishFusedGrid(const GridSnapshot & grid) const;
  static geometry_msgs::msg::Quaternion normalizedQuaternion(double yaw);

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("astar_local")};
  std::string name_;
  std::string frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr local_subscription_;
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
  int blocked_cost_threshold_{253};
  double local_costmap_timeout_{2.0};
  double transform_timeout_{0.2};
  double max_planning_time_{1.0};
  std::string local_costmap_topic_{"/local_costmap/costmap_raw"};
  std::string debug_fused_grid_topic_{"/astar_local/fused_grid"};
};
}  // namespace pongbot_local_graph_insertion_planner
