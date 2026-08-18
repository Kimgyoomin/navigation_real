#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rubi_heightmap_wavefront_planner/terrain_evaluator.hpp"
#include "rubi_heightmap_wavefront_planner/wavefront_planner.hpp"

namespace pongbot_navigation
{

class HeightmapWavefrontNav2Planner final : public nav2_core::GlobalPlanner
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
  struct MapState
  {
    std::shared_ptr<const rubi_heightmap_wavefront_planner::TerrainSnapshot> snapshot;
    std::string frame_id;
    std::uint64_t content_hash{0U};
    std::uint64_t generation{0U};
    std::chrono::steady_clock::time_point received_at;
  };

  void onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
  std::shared_ptr<const MapState> mapState() const;
  void validateParameters() const;

  nav_msgs::msg::Path createPlanImpl(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::uint64_t & generation,
    rubi_heightmap_wavefront_planner::PlanResult & result);

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("heightmap_wavefront_nav2")};
  std::string name_;
  std::string global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;

  mutable std::mutex map_mutex_;
  std::shared_ptr<const MapState> map_state_;

  std::string input_cloud_topic_{"/fastdem/mapping/cloud_global"};
  double map_resolution_m_{0.05};
  double lattice_tolerance_m_{0.01};
  bool reject_duplicate_cells_{true};
  std::size_t max_grid_cells_{5000000U};
  double max_map_receive_age_s_{2.5};
  double path_output_spacing_m_{0.05};

  rubi_heightmap_wavefront_planner::TerrainEvaluatorParameters evaluator_parameters_;
  rubi_heightmap_wavefront_planner::WavefrontPlannerParameters planner_parameters_;
  std::unique_ptr<rubi_heightmap_wavefront_planner::WavefrontPlanner> planner_;
};

}  // namespace pongbot_navigation
