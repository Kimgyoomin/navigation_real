#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rubi_heightmap_step_wavefront_planner/planner_visualization.hpp"
#include "rubi_heightmap_step_wavefront_planner/planning/step_grid_astar_planner.hpp"
#include "rubi_heightmap_step_wavefront_planner/ros/nav2_costmap_adapter.hpp"
#include "rubi_heightmap_step_wavefront_planner/ros/pointcloud2_heightmap_adapter.hpp"
#include "rubi_heightmap_step_wavefront_planner/step_wavefront_planner.hpp"
#include "rubi_heightmap_step_wavefront_planner/terrain/height_evidence.hpp"

namespace rubi_heightmap_step_wavefront_planner
{
namespace
{
enum class PlannerRunMode {kGridOnly, kSamplingOnly, kBoth};

PlannerRunMode parseRunMode(const std::string & value)
{
  if (value == "grid_only") {return PlannerRunMode::kGridOnly;}
  if (value == "sampling_only") {return PlannerRunMode::kSamplingOnly;}
  if (value == "both") {return PlannerRunMode::kBoth;}
  throw std::invalid_argument("planner_run_mode must be grid_only, sampling_only, or both");
}

void recolor(
  visualization_msgs::msg::MarkerArray & markers, const float red,
  const float green, const float blue)
{
  for (auto & marker : markers.markers) {
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0F;
  }
}
}  // namespace

class HybridPlannerComparisonNode : public rclcpp::Node
{
public:
  HybridPlannerComparisonNode()
  : Node("rubi_hybrid_planner_comparison")
  {
    loadParameters();
    if (run_mode_ != PlannerRunMode::kSamplingOnly) {
      grid_planner_ = std::make_unique<StepGridAStarPlanner>(grid_parameters_);
    }
    if (run_mode_ != PlannerRunMode::kGridOnly) {
      sampling_planner_ = std::make_unique<StepWavefrontPlanner>(sampling_parameters_);
    }
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    const auto output_qos = rclcpp::QoS(1).reliable().transient_local();
    costmap_subscription_ = create_subscription<nav2_msgs::msg::Costmap>(
      input_costmap_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&HybridPlannerComparisonNode::onCostmap, this, std::placeholders::_1));
    heightmap_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_heightmap_topic_, rclcpp::QoS(1).reliable().durability_volatile(),
      std::bind(&HybridPlannerComparisonNode::onHeightmap, this, std::placeholders::_1));
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      comparison_goal_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&HybridPlannerComparisonNode::onGoal, this, std::placeholders::_1));
    grid_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/rubi/planner_comparison/grid/path", output_qos);
    sampling_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/rubi/planner_comparison/sampling/path", output_qos);
    grid_nodes_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/rubi/planner_comparison/grid/nodes", output_qos);
    grid_expanded_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/rubi/planner_comparison/grid/expanded_cells", output_qos);
    grid_edges_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/rubi/planner_comparison/grid/edges", output_qos);
    grid_search_edges_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/rubi/planner_comparison/grid/search_edges", output_qos);
    sampling_nodes_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/rubi/planner_comparison/sampling/nodes", output_qos);
    sampling_edges_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/rubi/planner_comparison/sampling/edges", output_qos);
    rejected_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/rubi/planner_comparison/rejected", output_qos);
    sampling_rejected_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/rubi/planner_comparison/sampling/rejected", output_qos);
    robot_trace_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/rubi/planner_comparison/robot_trace", output_qos);
    if (run_mode_ != PlannerRunMode::kBoth) {
      trace_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&HybridPlannerComparisonNode::updateRobotTrace, this));
    }
    RCLCPP_INFO(
      get_logger(), "Hybrid planner ready; mode=%s goal_topic='%s' planner_cmd_vel_publishers=0",
      run_mode_text_.c_str(), comparison_goal_topic_.c_str());
  }

private:
  template<typename MessageT>
  rclcpp::Time messageTime(const MessageT & message) const
  {
    return rclcpp::Time(message.header.stamp, get_clock()->get_clock_type());
  }

  std::size_t positiveSize(const std::string & name, const std::int64_t fallback)
  {
    const auto value = declare_parameter<std::int64_t>(name, fallback);
    if (value <= 0) {throw std::invalid_argument(name + " must be > 0");}
    return static_cast<std::size_t>(value);
  }

  void loadParameters()
  {
    run_mode_text_ = declare_parameter("planner_run_mode", "both");
    run_mode_ = parseRunMode(run_mode_text_);
    detailed_timing_ = declare_parameter("detailed_timing", false);
    input_costmap_topic_ = declare_parameter(
      "input_costmap_topic", "/global_costmap/costmap_raw");
    input_heightmap_topic_ = declare_parameter(
      "input_heightmap_topic", "/fastdem/mapping/cloud_global");
    comparison_goal_topic_ = declare_parameter(
      "comparison_goal_topic", "/rubi/planner_comparison/goal");
    base_frame_ = declare_parameter("base_frame", "base_link");
    required_map_frame_ = declare_parameter("required_map_frame", "map");
    max_costmap_age_s_ = declare_parameter("max_costmap_age_s", 1.0);
    max_heightmap_age_s_ = declare_parameter("max_heightmap_age_s", 2.5);
    max_input_stamp_skew_s_ = declare_parameter("max_input_stamp_skew_s", 2.0);
    transform_timeout_s_ = declare_parameter("transform_timeout_s", 0.20);
    heightmap_adapter_parameters_.resolution_m = declare_parameter("map_resolution_m", 0.05);
    heightmap_adapter_parameters_.lattice_tolerance_m = declare_parameter(
      "lattice_tolerance_m", 0.01);
    heightmap_adapter_parameters_.max_grid_cells = positiveSize("max_grid_cells", 5000000);
    evaluator_parameters_.edge_check_spacing_m = declare_parameter(
      "edge_check_spacing_m", 0.025);
    if (!declare_parameter("costmap_unknown_is_blocked", true)) {
      throw std::invalid_argument("hybrid comparison requires blocked unknown costmap cells");
    }
    evaluator_parameters_.inflation_cost_weight = declare_parameter(
      "inflation_cost_weight", 5.0);
    evaluator_parameters_.inflation_cost_exponent = declare_parameter(
      "inflation_cost_exponent", 1.0);
    evaluator_parameters_.max_crossable_height_jump_m = declare_parameter(
      "max_crossable_height_jump_m", 0.08);
    evaluator_parameters_.height_noise_floor_m = declare_parameter(
      "height_noise_floor_m", 0.01);
    evaluator_parameters_.height_cost_exponent = declare_parameter(
      "height_cost_exponent", 2.0);
    evaluator_parameters_.distance_weight = declare_parameter("distance_weight", 1.0);
    evaluator_parameters_.height_cost_weight = declare_parameter("height_cost_weight", 5.0);
    evaluator_parameters_.node_evidence_radius_m = declare_parameter(
      "node_evidence_radius_m", 0.10);
    evaluator_parameters_.node_min_observed_cells = positiveSize("node_min_observed_cells", 3);
    evaluator_parameters_.node_max_nearest_evidence_distance_m = declare_parameter(
      "node_max_nearest_evidence_distance_m", 0.075);
    evaluator_parameters_.node_height_outlier_threshold_m = declare_parameter(
      "node_height_outlier_threshold_m", 0.08);
    evaluator_parameters_.node_max_height_outlier_ratio = declare_parameter(
      "node_max_height_outlier_ratio", 0.30);
    evaluator_parameters_.edge_height_query_radius_m = declare_parameter(
      "edge_height_query_radius_m", 0.075);
    evaluator_parameters_.edge_max_height_evidence_gap_m = declare_parameter(
      "edge_max_height_evidence_gap_m", 0.10);
    evaluator_parameters_.edge_check_spacing_m = declare_parameter(
      "evaluation.edge_check_spacing_m", evaluator_parameters_.edge_check_spacing_m);
    evaluator_parameters_.max_crossable_height_jump_m = declare_parameter(
      "evaluation.max_crossable_height_jump_m",
      evaluator_parameters_.max_crossable_height_jump_m);
    evaluator_parameters_.height_noise_floor_m = declare_parameter(
      "evaluation.height_noise_floor_m", evaluator_parameters_.height_noise_floor_m);
    evaluator_parameters_.height_cost_exponent = declare_parameter(
      "evaluation.height_cost_exponent", evaluator_parameters_.height_cost_exponent);
    evaluator_parameters_.distance_weight = declare_parameter(
      "evaluation.distance_weight", evaluator_parameters_.distance_weight);
    evaluator_parameters_.height_cost_weight = declare_parameter(
      "evaluation.height_cost_weight", evaluator_parameters_.height_cost_weight);
    evaluator_parameters_.inflation_cost_weight = declare_parameter(
      "evaluation.inflation_cost_weight", evaluator_parameters_.inflation_cost_weight);
    evaluator_parameters_.inflation_cost_exponent = declare_parameter(
      "evaluation.inflation_cost_exponent", evaluator_parameters_.inflation_cost_exponent);
    grid_parameters_.allow_diagonal = declare_parameter("grid_allow_diagonal", true);
    grid_parameters_.max_expanded_states = positiveSize("grid_max_expanded_states", 500000);
    grid_parameters_.max_planning_time_ms = positiveSize("grid_max_planning_time_ms", 5000);
    grid_visualization_parameters_.max_nodes = positiveSize(
      "grid_visualization_max_cells", 10000);
    grid_visualization_parameters_.stride = positiveSize("grid_visualization_stride", 1);
    const std::string legacy_sampling_policy = declare_parameter(
      "sampling_policy", "trg_random_ring");
    const std::string sampling_policy = declare_parameter(
      "sampling.policy", legacy_sampling_policy);
    if (sampling_policy == "trg_random_ring") {
      sampling_parameters_.sampling_policy = SamplingPolicy::kTrgRandomRing;
    } else if (sampling_policy == "original_trg_random_ring") {
      sampling_parameters_.sampling_policy = SamplingPolicy::kOriginalTrgRandomRing;
    } else if (sampling_policy != "deterministic_ring") {
      throw std::invalid_argument(
              "sampling policy must be deterministic_ring, trg_random_ring, or "
              "original_trg_random_ring");
    }
    sampling_parameters_.random_seed = static_cast<std::uint32_t>(
      positiveSize("sampling_random_seed", 42));
    sampling_parameters_.max_sampling_trials_per_expansion = positiveSize(
      "max_sampling_trials_per_expansion", 1000);
    sampling_parameters_.node_sampling_distance_m = declare_parameter(
      "node_sampling_distance_m", 0.30);
    sampling_parameters_.samples_per_expansion = positiveSize("samples_per_expansion", 20);
    sampling_parameters_.merge_radius_m = declare_parameter("merge_radius_m", 0.20);
    sampling_parameters_.neighbor_connection_radius_m = declare_parameter(
      "neighbor_connection_radius_m", 0.45);
    sampling_parameters_.goal_connection_distance_m = declare_parameter(
      "goal_connection_distance_m", 0.45);
    sampling_parameters_.max_nodes = positiveSize("max_nodes", 4000);
    sampling_parameters_.max_expansions = positiveSize("max_expansions", 4000);
    sampling_parameters_.max_graph_build_time_ms = positiveSize(
      "max_graph_build_time_ms", 5000);
    sampling_parameters_.post_goal_expansions = positiveSize("post_goal_expansions", 50);
    sampling_parameters_.trg_expand_distance_m = declare_parameter(
      "sampling.trg_expand_distance_m", 0.30);
    sampling_parameters_.trg_robot_size_m = declare_parameter(
      "sampling.trg_robot_size_m", 0.20);
    sampling_parameters_.trg_sample_num = positiveSize("sampling.trg_sample_num", 20);
    sampling_parameters_.trg_max_trial_samples = positiveSize(
      "sampling.trg_max_trial_samples", 1000);
    sampling_parameters_.trg_height_threshold_m = declare_parameter(
      "sampling.trg_height_threshold_m", 0.08);
    sampling_parameters_.trg_collision_threshold = declare_parameter(
      "sampling.trg_collision_threshold", 0.10);
    sampling_parameters_.trg_random_seed = static_cast<std::uint32_t>(
      positiveSize("sampling.trg_random_seed", 42));
    sampling_parameters_.trg_randomize_seed = declare_parameter(
      "sampling.trg_randomize_seed", false);
    sampling_parameters_.trg_neighbor_connection_radius_m = declare_parameter(
      "sampling.trg_neighbor_connection_radius_m", 0.30);
    sampling_parameters_.goal_connection_distance_m = declare_parameter(
      "sampling.goal_connection_distance_m",
      sampling_parameters_.goal_connection_distance_m);
    sampling_parameters_.max_nodes = positiveSize(
      "sampling.max_nodes", static_cast<std::int64_t>(sampling_parameters_.max_nodes));
    sampling_parameters_.max_expansions = positiveSize(
      "sampling.max_expansions", static_cast<std::int64_t>(sampling_parameters_.max_expansions));
    sampling_parameters_.max_graph_build_time_ms = positiveSize(
      "sampling.max_graph_build_time_ms",
      static_cast<std::int64_t>(sampling_parameters_.max_graph_build_time_ms));
    sampling_parameters_.post_goal_expansions = positiveSize(
      "sampling.post_goal_expansions",
      static_cast<std::int64_t>(sampling_parameters_.post_goal_expansions));
  }

  void onCostmap(const nav2_msgs::msg::Costmap::ConstSharedPtr message)
  {
    try {
      auto snapshot = std::make_shared<const CostmapSnapshot>(
        Nav2CostmapAdapter{}.makeSnapshot(*message));
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      costmap_ = std::move(snapshot);
      costmap_frame_ = message->header.frame_id;
      costmap_stamp_ = messageTime(*message);
      ++costmap_generation_;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Rejected raw costmap: %s", error.what());
    }
  }

  void onHeightmap(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    try {
      auto snapshot = std::make_shared<const HeightmapSnapshot>(
        PointCloud2HeightmapAdapter{}.makeSnapshot(*message, heightmap_adapter_parameters_));
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      heightmap_ = std::move(snapshot);
      heightmap_frame_ = message->header.frame_id;
      heightmap_stamp_ = messageTime(*message);
      ++heightmap_generation_;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Rejected heightmap: %s", error.what());
    }
  }

  std::vector<TerrainPoint> terrainPath(const PlanResult & result) const
  {
    std::vector<TerrainPoint> path;
    for (const NodeId id : result.path_node_ids) {
      if (id < result.nodes.size()) {
        path.push_back({result.nodes[id].point.x, result.nodes[id].point.y,
          result.nodes[id].elevation_m});
      }
    }
    return path;
  }

  nav_msgs::msg::Path pathMessage(
    const PlanResult & result, const geometry_msgs::msg::Quaternion & goal_orientation,
    const rclcpp::Time & stamp) const
  {
    nav_msgs::msg::Path output;
    output.header.frame_id = required_map_frame_;
    output.header.stamp = stamp;
    const auto terrain = terrainPath(result);
    for (std::size_t index = 0U; index < terrain.size(); ++index) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = output.header;
      pose.pose.position.x = terrain[index].x;
      pose.pose.position.y = terrain[index].y;
      pose.pose.position.z = terrain[index].z;
      if (index + 1U < terrain.size()) {
        const double yaw = std::atan2(
          terrain[index + 1U].y - terrain[index].y,
          terrain[index + 1U].x - terrain[index].x);
        pose.pose.orientation.z = std::sin(0.5 * yaw);
        pose.pose.orientation.w = std::cos(0.5 * yaw);
      } else {
        pose.pose.orientation = goal_orientation;
      }
      output.poses.push_back(std::move(pose));
    }
    return output;
  }

  void logDiagnostics(
    const char * planner_name, const PlanResult & result,
    const HeightmapSnapshot & heightmap, const CostmapSnapshot & costmap)
  {
    std::size_t hard_blocked = 0U;
    std::size_t missing_height = 0U;
    for (const auto & rejection : result.rejected) {
      if (rejection.reason == StepInvalidReason::kCostmapCollision ||
        rejection.reason == StepInvalidReason::kCostmapUnknown)
      {
        ++hard_blocked;
        if (hard_blocked == 1U) {
          const auto raw = costmap.costAt(rejection.to);
          const auto height = queryEdgeHeight(
            heightmap, rejection.to, evaluator_parameters_.edge_height_query_radius_m);
          RCLCPP_WARN(
            get_logger(), "planner=%s hard_rejection=(%.3f,%.3f) raw_cost=%u "
            "height_evidence=%s known_height=%.3f",
            planner_name, rejection.to.x, rejection.to.y,
            raw ? static_cast<unsigned int>(*raw) : 255U,
            height ? "true" : "false", height ? height->elevation_m : 0.0);
        }
      }
      if (rejection.reason == StepInvalidReason::kInsufficientHeightEvidence ||
        rejection.reason == StepInvalidReason::kHeightEvidenceGap) {++missing_height;}
    }
    RCLCPP_INFO(
      get_logger(), "planner=%s success=%s length_m=%.3f total_cost=%.3f "
      "inflation_cost=%.3f height_cost=%.3f expanded=%zu nodes=%zu edges=%zu "
      "time_ms=%.3f costmap_hard_blocked_samples=%zu "
      "costmap_max_raw_cost_on_selected_path=%u height_evidence_missing_samples=%zu "
      "height_max_jump_m=%.3f",
      planner_name, result.success ? "true" : "false", result.path_metrics.length_xy_m,
      result.path_metrics.total_cost, result.path_metrics.inflation_cost,
      result.path_metrics.height_cost, result.expansions, result.nodes.size(),
      result.edges.size(), result.core_total_time_ms, hard_blocked,
      static_cast<unsigned int>(result.path_metrics.maximum_raw_cost), missing_height,
      result.path_metrics.max_height_jump_m);
    if (hard_blocked > 0U) {
      RCLCPP_WARN(
        get_logger(), "comparison input is confounded by Costmap obstacle marking "
        "planner=%s hard_samples=%zu", planner_name, hard_blocked);
    }
  }

  void logResultBlock(
    const char * title, const PlanResult & result, const std::uint64_t costmap_generation,
    const std::uint64_t heightmap_generation) const
  {
    const double distance_cost = evaluator_parameters_.distance_weight *
      result.path_metrics.length_xy_m;
    if (std::string(title) == "GRID A* RESULT") {
      RCLCPP_INFO(get_logger(),
        "\n============================================================\n"
        "[GRID A* RESULT]\n"
        "success                : %s\ntermination            : %s\n"
        "map_generation         : costmap=%lu heightmap=%lu\n"
        "path_length_m          : %.6f\ntotal_cost             : %.6f\n"
        "distance_cost          : %.6f\ninflation_cost         : %.6f\n"
        "height_cost            : %.6f\nmax_height_jump_m      : %.6f\n"
        "height_jump_events     : %zu\nexpanded_cells         : %zu\n"
        "neighbor_candidates    : %zu\nastar_open_pushes      : %zu\n"
        "node_eval_calls        : %zu\nedge_eval_calls        : %zu\n"
        "edge_samples_total     : %zu\nheight_evidence_queries: %zu\n"
        "costmap_queries        : %zu\nsearch_time_ms         : %.6f\n"
        "path_finalize_time_ms  : %.6f\ntotal_planning_time_ms : %.6f\n"
        "============================================================",
        result.success ? "true" : "false", std::string(toString(result.termination)).c_str(),
        static_cast<unsigned long>(costmap_generation),
        static_cast<unsigned long>(heightmap_generation), result.path_metrics.length_xy_m,
        result.path_metrics.total_cost, distance_cost, result.path_metrics.inflation_cost,
        result.path_metrics.height_cost, result.path_metrics.max_height_jump_m,
        result.path_metrics.height_event_count, result.expansions,
        result.statistics.neighbor_candidates, result.statistics.astar_open_pushes,
        result.statistics.node_evaluation_calls, result.statistics.edge_evaluation_calls,
        result.statistics.edge_samples_total, result.statistics.height_evidence_queries,
        result.statistics.costmap_queries, result.astar_time_ms,
        result.path_finalize_time_ms, result.core_total_time_ms);
      return;
    }
    RCLCPP_INFO(get_logger(),
      "\n============================================================\n"
      "[SAMPLING / TRG-CONSTRUCTION RESULT]\n"
      "success                : %s\ntermination            : %s\n"
      "sampling_policy        : %s\nrandom_seed            : %u\n"
      "map_generation         : costmap=%lu heightmap=%lu\n"
      "path_length_m          : %.6f\ntotal_cost             : %.6f\n"
      "distance_cost          : %.6f\ninflation_cost         : %.6f\n"
      "height_cost            : %.6f\nmax_height_jump_m      : %.6f\n"
      "height_jump_events     : %zu\nexpanded_reference_nodes: %zu\n"
      "sampling_trials        : %zu\n"
      "candidate_generated    : %zu\n"
      "candidate_accepts      : %zu\ncandidate_rejects      : %zu\n"
      "merge_queries          : %zu\nneighbor_radius_queries: %zu\n"
      "rejected_edges         : %zu\n"
      "trg_collision_rejects : %zu\ncostmap_rejects       : %zu\n"
      "existing_node_queries : %zu\nexisting_node_rewires : %zu\n"
      "new_nodes_created     : %zu\nisolated_nodes        : %zu\n"
      "neighbor_queries      : %zu\nneighbor_wire_attempts: %zu\n"
      "graph_nodes            : %zu\ngraph_edges            : %zu\n"
      "node_eval_calls        : %zu\nedge_eval_calls        : %zu\n"
      "edge_samples_total     : %zu\nheight_evidence_queries: %zu\n"
      "costmap_queries        : %zu\nastar_expanded_states  : %zu\n"
      "graph_build_total_ms   : %.6f\nastar_search_ms        : %.6f\n"
      "graph_clean_time_ms    : %.6f\n"
      "path_finalize_time_ms  : %.6f\ntotal_planning_time_ms : %.6f\n"
      "detailed_timing        : %s\n"
      "============================================================",
      result.success ? "true" : "false", std::string(toString(result.termination)).c_str(),
      sampling_parameters_.sampling_policy == SamplingPolicy::kOriginalTrgRandomRing ?
      "original_trg_random_ring" :
      (sampling_parameters_.sampling_policy == SamplingPolicy::kTrgRandomRing ?
      "trg_random_ring" : "deterministic_ring"),
      sampling_parameters_.sampling_policy == SamplingPolicy::kOriginalTrgRandomRing ?
      sampling_parameters_.trg_random_seed : sampling_parameters_.random_seed,
      static_cast<unsigned long>(costmap_generation),
      static_cast<unsigned long>(heightmap_generation), result.path_metrics.length_xy_m,
      result.path_metrics.total_cost, distance_cost, result.path_metrics.inflation_cost,
      result.path_metrics.height_cost, result.path_metrics.max_height_jump_m,
      result.path_metrics.height_event_count, result.expansions,
      result.statistics.sampling_trials,
      result.statistics.candidate_generated,
      result.statistics.candidate_valid, result.statistics.candidate_rejected,
      result.statistics.merge_queries, result.statistics.neighbor_radius_queries,
      result.statistics.rejected_edges,
      result.statistics.trg_collision_rejects, result.statistics.costmap_rejects,
      result.statistics.existing_node_queries, result.statistics.existing_node_rewires,
      result.statistics.new_nodes_created, result.statistics.isolated_nodes,
      result.statistics.neighbor_queries, result.statistics.neighbor_wire_attempts,
      result.nodes.size(), result.edges.size(), result.statistics.node_evaluation_calls,
      result.statistics.edge_evaluation_calls, result.statistics.edge_samples_total,
      result.statistics.height_evidence_queries, result.statistics.costmap_queries,
      result.statistics.expanded_states, result.graph_build_time_ms, result.astar_time_ms,
      result.graph_clean_time_ms,
      result.path_finalize_time_ms, result.core_total_time_ms,
      detailed_timing_ ? "enabled" : "disabled");
  }

  void resetRobotTrace(const Point2D goal)
  {
    if (run_mode_ == PlannerRunMode::kBoth) {return;}
    std::lock_guard<std::mutex> lock(trace_mutex_);
    robot_trace_.poses.clear();
    robot_trace_.header.frame_id = required_map_frame_;
    robot_trace_.header.stamp = now();
    trace_goal_ = goal;
    trace_active_ = true;
    robot_trace_publisher_->publish(robot_trace_);
  }

  void updateRobotTrace()
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        required_map_frame_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_s_));
      std::lock_guard<std::mutex> lock(trace_mutex_);
      if (!trace_active_) {return;}
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = required_map_frame_;
      pose.header.stamp = now();
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      if (robot_trace_.poses.empty() || std::hypot(
          pose.pose.position.x - robot_trace_.poses.back().pose.position.x,
          pose.pose.position.y - robot_trace_.poses.back().pose.position.y) > 0.005)
      {
        robot_trace_.poses.push_back(pose);
      }
      robot_trace_.header.stamp = pose.header.stamp;
      robot_trace_publisher_->publish(robot_trace_);
      if (trace_goal_ && std::hypot(
          pose.pose.position.x - trace_goal_->x,
          pose.pose.position.y - trace_goal_->y) <= 0.20)
      {
        trace_active_ = false;
      }
    } catch (const std::exception &) {
      // Inputs may legitimately start before TF; keep the trace dormant until TF appears.
    }
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal)
  {
    std::shared_ptr<const CostmapSnapshot> costmap;
    std::shared_ptr<const HeightmapSnapshot> heightmap;
    std::string costmap_frame;
    std::string heightmap_frame;
    rclcpp::Time costmap_stamp(0, 0, get_clock()->get_clock_type());
    rclcpp::Time heightmap_stamp(0, 0, get_clock()->get_clock_type());
    std::uint64_t costmap_generation = 0U;
    std::uint64_t heightmap_generation = 0U;
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      costmap = costmap_;
      heightmap = heightmap_;
      costmap_frame = costmap_frame_;
      heightmap_frame = heightmap_frame_;
      costmap_stamp = costmap_stamp_;
      heightmap_stamp = heightmap_stamp_;
      costmap_generation = costmap_generation_;
      heightmap_generation = heightmap_generation_;
    }
    if (!costmap || !heightmap) {
      RCLCPP_WARN(get_logger(), "Comparison rejected: both snapshots are required");
      return;
    }
    const rclcpp::Time current = now();
    const double costmap_age = (current - costmap_stamp).seconds();
    const double heightmap_age = (current - heightmap_stamp).seconds();
    const double skew = std::abs((costmap_stamp - heightmap_stamp).seconds());
    if (costmap_frame != required_map_frame_ || heightmap_frame != required_map_frame_ ||
      costmap_age < 0.0 || costmap_age > max_costmap_age_s_ || heightmap_age < 0.0 ||
      heightmap_age > max_heightmap_age_s_ || skew > max_input_stamp_skew_s_)
    {
      RCLCPP_WARN(
        get_logger(), "Comparison rejected: frame/age/skew contract frame_cost='%s' "
        "frame_height='%s' cost_age=%.3f height_age=%.3f skew=%.3f",
        costmap_frame.c_str(), heightmap_frame.c_str(), costmap_age, heightmap_age, skew);
      return;
    }
    try {
      geometry_msgs::msg::PoseStamped goal_in_map;
      if (goal->header.frame_id == required_map_frame_) {
        goal_in_map = *goal;
      } else {
        const auto transform = tf_buffer_->lookupTransform(
          required_map_frame_, goal->header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(transform_timeout_s_));
        tf2::doTransform(*goal, goal_in_map, transform);
      }
      const auto start_transform = tf_buffer_->lookupTransform(
        required_map_frame_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_s_));
      const auto start_cell = costmap->worldToCell({
          start_transform.transform.translation.x,
          start_transform.transform.translation.y});
      const auto goal_cell = costmap->worldToCell({
          goal_in_map.pose.position.x, goal_in_map.pose.position.y});
      if (!start_cell || !goal_cell || !costmap->inBounds(*start_cell) ||
        !costmap->inBounds(*goal_cell))
      {
        RCLCPP_WARN(get_logger(), "Comparison endpoint lies outside the costmap");
        return;
      }
      const Point2D start = costmap->cellCenter(*start_cell);
      const Point2D target = costmap->cellCenter(*goal_cell);
      resetRobotTrace(target);
      std::optional<PlanResult> grid;
      std::optional<PlanResult> sampling;
      if (grid_planner_) {
        StepEvaluator evaluator(*heightmap, *costmap, evaluator_parameters_);
        grid = grid_planner_->plan(evaluator, start, target);
      }
      if (sampling_planner_) {
        StepEvaluator evaluator(*heightmap, *costmap, evaluator_parameters_);
        sampling = sampling_planner_->plan(evaluator, start, target);
      }
      const rclcpp::Time stamp = now();
      const VisualizationParameters visualization_parameters;
      PlanResult combined;
      if (grid) {
        grid_path_publisher_->publish(pathMessage(*grid, goal_in_map.pose.orientation, stamp));
        auto visualization = makeVisualization(
          *grid, terrainPath(*grid), required_map_frame_, stamp,
          grid_visualization_parameters_);
        recolor(visualization.nodes, 0.0F, 0.8F, 1.0F);
        recolor(visualization.edges, 0.0F, 0.45F, 1.0F);
        grid_nodes_publisher_->publish(visualization.nodes);
        grid_expanded_publisher_->publish(visualization.nodes);
        grid_edges_publisher_->publish(visualization.edges);
        grid_search_edges_publisher_->publish(visualization.edges);
        combined.rejected = grid->rejected;
      }
      if (sampling) {
        sampling_path_publisher_->publish(pathMessage(
            *sampling, goal_in_map.pose.orientation, stamp));
        auto visualization = makeVisualization(
          *sampling, terrainPath(*sampling), required_map_frame_, stamp,
          visualization_parameters);
        recolor(visualization.edges, 1.0F, 0.45F, 0.0F);
        sampling_nodes_publisher_->publish(visualization.nodes);
        sampling_edges_publisher_->publish(visualization.edges);
        sampling_rejected_publisher_->publish(visualization.rejected);
        combined.rejected.insert(
          combined.rejected.end(), sampling->rejected.begin(), sampling->rejected.end());
      }
      rejected_publisher_->publish(makeVisualization(
          combined, {}, required_map_frame_, stamp, visualization_parameters).rejected);
      RCLCPP_INFO(
        get_logger(), "comparison_snapshot costmap_generation=%lu "
        "heightmap_generation=%lu start=(%.3f,%.3f) goal=(%.3f,%.3f) "
        "sampling_policy=%s seed=%u",
        static_cast<unsigned long>(costmap_generation),
        static_cast<unsigned long>(heightmap_generation), start.x, start.y,
        target.x, target.y,
        sampling_parameters_.sampling_policy == SamplingPolicy::kOriginalTrgRandomRing ?
        "original_trg_random_ring" :
        (sampling_parameters_.sampling_policy == SamplingPolicy::kTrgRandomRing ?
        "trg_random_ring" : "deterministic_ring"),
        sampling_parameters_.sampling_policy == SamplingPolicy::kOriginalTrgRandomRing ?
        sampling_parameters_.trg_random_seed : sampling_parameters_.random_seed);
      if (grid) {
        logDiagnostics("grid", *grid, *heightmap, *costmap);
        logResultBlock("GRID A* RESULT", *grid, costmap_generation, heightmap_generation);
      } else {
        RCLCPP_INFO(get_logger(), "planner=grid status=not_run mode=%s", run_mode_text_.c_str());
      }
      if (sampling) {
        logDiagnostics("sampling", *sampling, *heightmap, *costmap);
        logResultBlock(
          "SAMPLING A* RESULT", *sampling, costmap_generation, heightmap_generation);
      } else {
        RCLCPP_INFO(
          get_logger(), "planner=sampling status=not_run mode=%s", run_mode_text_.c_str());
      }
      if (grid && sampling) {
        const double ratio = grid->core_total_time_ms > 0.0 ?
          sampling->core_total_time_ms / grid->core_total_time_ms : 0.0;
        RCLCPP_INFO(get_logger(),
          "[COMPARISON] same_snapshot=true grid_total_ms=%.6f "
          "sampling_total_ms=%.6f sampling_over_grid_ratio=%.6f",
          grid->core_total_time_ms, sampling->core_total_time_ms, ratio);
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Comparison planning failed: %s", error.what());
    }
  }

  std::string input_costmap_topic_, input_heightmap_topic_, comparison_goal_topic_;
  std::string base_frame_, required_map_frame_;
  std::string run_mode_text_{"both"};
  PlannerRunMode run_mode_{PlannerRunMode::kBoth};
  bool detailed_timing_{false};
  double max_costmap_age_s_{1.0}, max_heightmap_age_s_{2.5};
  double max_input_stamp_skew_s_{2.0}, transform_timeout_s_{0.2};
  HeightmapAdapterParameters heightmap_adapter_parameters_;
  StepEvaluatorParameters evaluator_parameters_;
  GridAStarParameters grid_parameters_;
  VisualizationParameters grid_visualization_parameters_;
  StepWavefrontParameters sampling_parameters_;
  std::unique_ptr<StepGridAStarPlanner> grid_planner_;
  std::unique_ptr<StepWavefrontPlanner> sampling_planner_;
  std::mutex snapshot_mutex_;
  std::shared_ptr<const CostmapSnapshot> costmap_;
  std::shared_ptr<const HeightmapSnapshot> heightmap_;
  std::string costmap_frame_, heightmap_frame_;
  rclcpp::Time costmap_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time heightmap_stamp_{0, 0, RCL_ROS_TIME};
  std::uint64_t costmap_generation_{0U}, heightmap_generation_{0U};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr heightmap_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr grid_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr sampling_path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_nodes_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_expanded_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_edges_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_search_edges_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sampling_nodes_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sampling_edges_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rejected_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sampling_rejected_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr robot_trace_publisher_;
  rclcpp::TimerBase::SharedPtr trace_timer_;
  std::mutex trace_mutex_;
  nav_msgs::msg::Path robot_trace_;
  std::optional<Point2D> trace_goal_;
  bool trace_active_{false};
};

}  // namespace rubi_heightmap_step_wavefront_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<rubi_heightmap_step_wavefront_planner::HybridPlannerComparisonNode>());
  rclcpp::shutdown();
  return 0;
}
