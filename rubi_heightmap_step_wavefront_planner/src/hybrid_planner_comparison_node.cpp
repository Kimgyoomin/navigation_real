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
#include "rubi_heightmap_step_wavefront_planner/planning/path_cost_evaluator.hpp"
#include "rubi_heightmap_step_wavefront_planner/planning/step_grid_astar_planner.hpp"
#include "rubi_heightmap_step_wavefront_planner/planning/tracking_path_refiner.hpp"
#include "rubi_heightmap_step_wavefront_planner/ros/nav2_costmap_adapter.hpp"
#include "rubi_heightmap_step_wavefront_planner/ros/pointcloud2_heightmap_adapter.hpp"
#include "rubi_heightmap_step_wavefront_planner/step_wavefront_planner.hpp"
#include "rubi_heightmap_step_wavefront_planner/terrain/height_evidence.hpp"

namespace rubi_heightmap_step_wavefront_planner
{
namespace
{
enum class PlannerRunMode {kGridOnly, kSamplingOnly, kBoth};

struct ActivePlanState
{
  bool active{false};
  std::vector<TerrainPoint> raw_path;
  std::vector<TerrainPoint> tracking_path;
  double tracking_cost{0.0};
  std::size_t nearest_index{0U};
  std::uint64_t costmap_generation{0U};
  std::uint64_t heightmap_generation{0U};
};

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
    grid_tracking_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/rubi/planner_comparison/grid/tracking_path", output_qos);
    sampling_tracking_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/rubi/planner_comparison/sampling/tracking_path", output_qos);
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
    if (replanning_enabled_) {
      replan_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(replanning_check_period_s_)),
        std::bind(&HybridPlannerComparisonNode::processPendingReplan, this));
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

    tracking_refiner_parameters_.enabled = declare_parameter(
      "tracking_refiner.enabled", true);
    const std::string refiner_method = declare_parameter(
      "tracking_refiner.method", "trg_three_point_mean_validated");
    if (refiner_method != "trg_three_point_mean_validated") {
      throw std::invalid_argument("unsupported tracking_refiner.method");
    }
    tracking_refiner_parameters_.smoothing_passes = positiveSize(
      "tracking_refiner.smoothing_passes", 1);
    tracking_refiner_parameters_.resample_spacing_m = declare_parameter(
      "tracking_refiner.resample_spacing_m", 0.10);
    tracking_refiner_parameters_.max_cost_increase_ratio = declare_parameter(
      "tracking_refiner.max_cost_increase_ratio", 0.05);

    replanning_enabled_ = declare_parameter("replanning.enabled", true);
    replanning_check_period_s_ = declare_parameter("replanning.check_period_s", 0.10);
    replanning_min_interval_s_ = declare_parameter(
      "replanning.min_replan_interval_s", 0.50);
    soft_reoptimize_min_interval_s_ = declare_parameter(
      "replanning.soft_reoptimize_min_interval_s", 1.50);
    replan_on_heightmap_change_ = declare_parameter(
      "replanning.on_heightmap_change", true);
    replan_on_costmap_change_ = declare_parameter(
      "replanning.on_costmap_change", true);
    stop_before_hard_replan_ = declare_parameter(
      "replanning.stop_before_hard_replan", true);
    min_cost_improvement_ratio_ = declare_parameter(
      "replanning.min_cost_improvement_ratio", 0.05);
    replanning_goal_tolerance_m_ = declare_parameter(
      "replanning.goal_tolerance_m", 0.20);
    retry_failed_plan_ = declare_parameter("replanning.retry_failed_plan", true);
    failed_retry_interval_s_ = declare_parameter(
      "replanning.failed_retry_interval_s", 1.0);
    if (replanning_check_period_s_ <= 0.0 || replanning_min_interval_s_ < 0.0 ||
      soft_reoptimize_min_interval_s_ < 0.0 || min_cost_improvement_ratio_ < 0.0 ||
      min_cost_improvement_ratio_ > 1.0 || replanning_goal_tolerance_m_ < 0.0 ||
      failed_retry_interval_s_ <= 0.0)
    {
      throw std::invalid_argument("invalid replanning parameters");
    }
  }

  void onCostmap(const nav2_msgs::msg::Costmap::ConstSharedPtr message)
  {
    try {
      auto snapshot = std::make_shared<const CostmapSnapshot>(
        Nav2CostmapAdapter{}.makeSnapshot(*message));
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      const bool content_changed = !costmap_ ||
        snapshot->contentHash() != costmap_content_hash_;
      costmap_frame_ = message->header.frame_id;
      costmap_stamp_ = messageTime(*message);
      if (!content_changed) {return;}
      costmap_content_hash_ = snapshot->contentHash();
      costmap_ = std::move(snapshot);
      ++costmap_generation_;
      if (replan_on_costmap_change_) {pending_costmap_change_ = true;}
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
      const bool content_changed = !heightmap_ ||
        snapshot->contentHash() != heightmap_content_hash_;
      heightmap_frame_ = message->header.frame_id;
      heightmap_stamp_ = messageTime(*message);
      if (!content_changed) {return;}
      heightmap_content_hash_ = snapshot->contentHash();
      heightmap_ = std::move(snapshot);
      ++heightmap_generation_;
      if (replan_on_heightmap_change_) {pending_heightmap_change_ = true;}
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
    const std::vector<TerrainPoint> & terrain,
    const geometry_msgs::msg::Quaternion & goal_orientation,
    const rclcpp::Time & stamp) const
  {
    nav_msgs::msg::Path output;
    output.header.frame_id = required_map_frame_;
    output.header.stamp = stamp;
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

  nav_msgs::msg::Path pathMessage(
    const PlanResult & result, const geometry_msgs::msg::Quaternion & goal_orientation,
    const rclcpp::Time & stamp) const
  {
    return pathMessage(terrainPath(result), goal_orientation, stamp);
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

  void logRefiner(
    const char * mode, const TrackingPathRefinerResult & result) const
  {
    RCLCPP_INFO(
      get_logger(),
      "\n[TRACKING PATH REFINER]\nmode=%s raw_points=%zu smoothing_attempts=%zu "
      "smoothing_accepts=%zu smoothing_rejects=%zu tracking_points=%zu "
      "raw_cost=%.6f tracking_cost=%.6f raw_max_heading_change_rad=%.6f "
      "tracking_max_heading_change_rad=%.6f fallback=%s",
      mode, result.raw_point_count, result.smoothing_attempts,
      result.smoothing_accepts, result.smoothing_rejects,
      result.tracking_point_count, result.raw_cost, result.tracking_cost,
      result.raw_max_heading_change_rad, result.tracking_max_heading_change_rad,
      result.used_raw_fallback ? "true" : "false");
  }

  void logSoftDecision(
    const char * mode, const double current_cost, const double candidate_cost,
    const double improvement, const bool candidate_valid) const
  {
    RCLCPP_INFO(
      get_logger(),
      "[REPLAN RESULT] mode=%s current_valid=true candidate_valid=%s "
      "old_tracking_cost=%.6f new_tracking_cost=%.6f improvement_ratio=%.6f "
      "required_ratio=%.6f action=keep_current_path",
      mode, candidate_valid ? "true" : "false", current_cost, candidate_cost,
      improvement, min_cost_improvement_ratio_);
  }

  void processPendingReplan()
  {
    bool pending = false;
    std::shared_ptr<const CostmapSnapshot> costmap;
    std::shared_ptr<const HeightmapSnapshot> heightmap;
    std::uint64_t costmap_generation = 0U;
    std::uint64_t heightmap_generation = 0U;
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      pending = pending_costmap_change_ || pending_heightmap_change_;
      costmap = costmap_;
      heightmap = heightmap_;
      costmap_generation = costmap_generation_;
      heightmap_generation = heightmap_generation_;
    }
    if (!goal_active_) {
      if (pending) {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        pending_costmap_change_ = false;
        pending_heightmap_change_ = false;
      }
      return;
    }
    const bool stopped =
      (grid_active_.active && grid_active_.tracking_path.empty()) ||
      (sampling_active_.active && sampling_active_.tracking_path.empty());
    const double since_failed = (now() - last_failed_replan_time_).seconds();
    const bool retry = retry_failed_plan_ && stopped &&
      since_failed >= failed_retry_interval_s_;
    if ((!pending && !retry) || !costmap || !heightmap) {return;}

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        required_map_frame_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_s_));
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Automatic replan waiting for TF: %s", error.what());
      return;
    }
    const Point2D robot{transform.transform.translation.x, transform.transform.translation.y};
    if (std::hypot(robot.x - effective_goal_.x, robot.y - effective_goal_.y) <=
      replanning_goal_tolerance_m_)
    {
      goal_active_ = false;
      grid_active_.active = false;
      sampling_active_.active = false;
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      pending_costmap_change_ = false;
      pending_heightmap_change_ = false;
      return;
    }

    StepEvaluator evaluator(*heightmap, *costmap, evaluator_parameters_);
    bool hard_invalid = retry;
    StepInvalidReason invalid_reason = StepInvalidReason::kNone;
    std::size_t failing_segment = 0U;
    auto validate = [&](ActivePlanState & state) {
        if (!state.active || state.tracking_path.empty()) {return;}
        state.nearest_index = nearestPathIndex(
          state.tracking_path, robot, state.nearest_index);
        const auto validation = validateRemainingPath(
          state.tracking_path, state.nearest_index, evaluator);
        if (!validation.valid && !hard_invalid) {
          hard_invalid = true;
          invalid_reason = validation.reason;
          failing_segment = validation.failing_segment;
        }
      };
    validate(grid_active_);
    validate(sampling_active_);

    const rclcpp::Time current = now();
    if (hard_invalid && !hard_stop_latched_ && stop_before_hard_replan_) {
      const auto stamp = now();
      if (grid_active_.active) {
        grid_active_.tracking_path.clear();
        grid_tracking_path_publisher_->publish(pathMessage(
            std::vector<TerrainPoint>{}, active_goal_map_.pose.orientation, stamp));
      }
      if (sampling_active_.active) {
        sampling_active_.tracking_path.clear();
        sampling_tracking_path_publisher_->publish(pathMessage(
            std::vector<TerrainPoint>{}, active_goal_map_.pose.orientation, stamp));
      }
      hard_stop_latched_ = true;
      RCLCPP_WARN(
        get_logger(),
        "[REPLAN EVENT] trigger=map_content_changed mode=%s "
        "costmap_generation=%lu heightmap_generation=%lu remaining_path_valid=false "
        "invalid_reason=%s failing_segment=%zu action=stop_then_replan",
        run_mode_text_.c_str(), static_cast<unsigned long>(costmap_generation),
        static_cast<unsigned long>(heightmap_generation),
        std::string(toString(invalid_reason)).c_str(), failing_segment);
    }

    if ((current - last_replan_time_).seconds() < replanning_min_interval_s_) {return;}
    if (!hard_invalid &&
      (current - last_soft_replan_time_).seconds() < soft_reoptimize_min_interval_s_)
    {
      return;
    }
    automatic_plan_processed_ = false;
    executeGoal(
      std::make_shared<geometry_msgs::msg::PoseStamped>(active_goal_map_),
      true, hard_invalid);
    if (!automatic_plan_processed_) {return;}
    if (hard_invalid) {
      const bool success =
        (!grid_active_.active || !grid_active_.tracking_path.empty()) &&
        (!sampling_active_.active || !sampling_active_.tracking_path.empty());
      RCLCPP_INFO(
        get_logger(), "[REPLAN RESULT] mode=%s success=%s "
        "costmap_generation=%lu heightmap_generation=%lu "
        "grid_raw_points=%zu grid_tracking_points=%zu "
        "sampling_raw_points=%zu sampling_tracking_points=%zu action=%s",
        run_mode_text_.c_str(), success ? "true" : "false",
        static_cast<unsigned long>(costmap_generation),
        static_cast<unsigned long>(heightmap_generation),
        grid_active_.raw_path.size(), grid_active_.tracking_path.size(),
        sampling_active_.raw_path.size(), sampling_active_.tracking_path.size(),
        success ? "resume_tracking" : "remain_stopped");
    }
    last_replan_time_ = current;
    if (!hard_invalid) {last_soft_replan_time_ = current;}
    if ((grid_active_.active && grid_active_.tracking_path.empty()) ||
      (sampling_active_.active && sampling_active_.tracking_path.empty()))
    {
      last_failed_replan_time_ = current;
    } else {
      hard_stop_latched_ = false;
    }
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    last_processed_costmap_generation_ = costmap_generation_;
    last_processed_heightmap_generation_ = heightmap_generation_;
    pending_costmap_change_ = false;
    pending_heightmap_change_ = false;
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal)
  {
    executeGoal(goal, false, false);
  }

  void executeGoal(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal,
    const bool automatic, const bool hard_replan)
  {
    std::lock_guard<std::mutex> planning_lock(planning_mutex_);
    if (!automatic) {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      pending_costmap_change_ = false;
      pending_heightmap_change_ = false;
    }
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
      if (!automatic) {resetRobotTrace(target);}
      goal_active_ = true;
      active_goal_map_ = goal_in_map;
      effective_goal_ = target;

      std::optional<double> old_grid_cost;
      std::optional<double> old_sampling_cost;
      StepEvaluator current_evaluator(*heightmap, *costmap, evaluator_parameters_);
      const Point2D robot_position{
        start_transform.transform.translation.x,
        start_transform.transform.translation.y};
      if (grid_active_.active && !grid_active_.tracking_path.empty()) {
        grid_active_.nearest_index = nearestPathIndex(
          grid_active_.tracking_path, robot_position, grid_active_.nearest_index);
        const auto evaluation = evaluatePolyline(
          grid_active_.tracking_path, grid_active_.nearest_index, current_evaluator);
        if (evaluation.valid) {old_grid_cost = evaluation.total_cost;}
      }
      if (sampling_active_.active && !sampling_active_.tracking_path.empty()) {
        sampling_active_.nearest_index = nearestPathIndex(
          sampling_active_.tracking_path, robot_position, sampling_active_.nearest_index);
        const auto evaluation = evaluatePolyline(
          sampling_active_.tracking_path, sampling_active_.nearest_index, current_evaluator);
        if (evaluation.valid) {old_sampling_cost = evaluation.total_cost;}
      }
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
        const auto raw = terrainPath(*grid);
        StepEvaluator evaluator(*heightmap, *costmap, evaluator_parameters_);
        const auto refined = TrackingPathRefiner(tracking_refiner_parameters_).refine(
          raw, evaluator);
        const double candidate_cost = refined.tracking_cost;
        const double improvement = old_grid_cost && *old_grid_cost > 1.0e-12 ?
          (*old_grid_cost - candidate_cost) / *old_grid_cost : 0.0;
        const bool accept = grid->success && refined.success &&
          (!automatic || hard_replan || !old_grid_cost ||
          hasMinimumCostImprovement(
            *old_grid_cost, candidate_cost, min_cost_improvement_ratio_));
        if (accept) {
          grid_path_publisher_->publish(pathMessage(raw, goal_in_map.pose.orientation, stamp));
          grid_tracking_path_publisher_->publish(pathMessage(
              refined.path, goal_in_map.pose.orientation, stamp));
          grid_active_ = ActivePlanState{true, raw, refined.path, candidate_cost, 0U,
            costmap_generation, heightmap_generation};
          auto visualization = makeVisualization(
            *grid, raw, required_map_frame_, stamp, grid_visualization_parameters_);
          recolor(visualization.nodes, 0.0F, 0.8F, 1.0F);
          recolor(visualization.edges, 0.0F, 0.45F, 1.0F);
          grid_nodes_publisher_->publish(visualization.nodes);
          grid_expanded_publisher_->publish(visualization.nodes);
          grid_edges_publisher_->publish(visualization.edges);
          grid_search_edges_publisher_->publish(visualization.edges);
          logRefiner("grid", refined);
          if (automatic && !hard_replan) {
            RCLCPP_INFO(
              get_logger(), "[REPLAN RESULT] mode=grid success=true "
              "new_tracking_cost=%.6f improvement_ratio=%.6f action=replace_path",
              candidate_cost, improvement);
          }
        } else if (automatic && !hard_replan && old_grid_cost) {
          logSoftDecision("grid", *old_grid_cost, candidate_cost, improvement, refined.success);
        } else if (!grid->success || !refined.success) {
          grid_active_.active = true;
          grid_active_.tracking_path.clear();
          grid_tracking_path_publisher_->publish(pathMessage(
              std::vector<TerrainPoint>{}, goal_in_map.pose.orientation, stamp));
        }
        combined.rejected = grid->rejected;
      }
      if (sampling) {
        const auto raw = terrainPath(*sampling);
        StepEvaluator evaluator(*heightmap, *costmap, evaluator_parameters_);
        const auto refined = TrackingPathRefiner(tracking_refiner_parameters_).refine(
          raw, evaluator);
        const double candidate_cost = refined.tracking_cost;
        const double improvement = old_sampling_cost && *old_sampling_cost > 1.0e-12 ?
          (*old_sampling_cost - candidate_cost) / *old_sampling_cost : 0.0;
        const bool accept = sampling->success && refined.success &&
          (!automatic || hard_replan || !old_sampling_cost ||
          hasMinimumCostImprovement(
            *old_sampling_cost, candidate_cost, min_cost_improvement_ratio_));
        if (accept) {
          sampling_path_publisher_->publish(pathMessage(
              raw, goal_in_map.pose.orientation, stamp));
          sampling_tracking_path_publisher_->publish(pathMessage(
              refined.path, goal_in_map.pose.orientation, stamp));
          sampling_active_ = ActivePlanState{true, raw, refined.path, candidate_cost, 0U,
            costmap_generation, heightmap_generation};
          auto visualization = makeVisualization(
            *sampling, raw, required_map_frame_, stamp, visualization_parameters);
          recolor(visualization.edges, 1.0F, 0.45F, 0.0F);
          sampling_nodes_publisher_->publish(visualization.nodes);
          sampling_edges_publisher_->publish(visualization.edges);
          sampling_rejected_publisher_->publish(visualization.rejected);
          logRefiner("sampling", refined);
          if (automatic && !hard_replan) {
            RCLCPP_INFO(
              get_logger(), "[REPLAN RESULT] mode=sampling success=true "
              "new_tracking_cost=%.6f improvement_ratio=%.6f action=replace_path",
              candidate_cost, improvement);
          }
        } else if (automatic && !hard_replan && old_sampling_cost) {
          logSoftDecision(
            "sampling", *old_sampling_cost, candidate_cost, improvement, refined.success);
        } else if (!sampling->success || !refined.success) {
          sampling_active_.active = true;
          sampling_active_.tracking_path.clear();
          sampling_tracking_path_publisher_->publish(pathMessage(
              std::vector<TerrainPoint>{}, goal_in_map.pose.orientation, stamp));
        }
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
      automatic_plan_processed_ = true;
      if (!automatic) {
        last_replan_time_ = stamp;
        last_soft_replan_time_ = stamp;
        hard_stop_latched_ = false;
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Comparison planning failed: %s", error.what());
      automatic_plan_processed_ = true;
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
  TrackingPathRefinerParameters tracking_refiner_parameters_;
  std::unique_ptr<StepGridAStarPlanner> grid_planner_;
  std::unique_ptr<StepWavefrontPlanner> sampling_planner_;
  std::mutex snapshot_mutex_;
  std::shared_ptr<const CostmapSnapshot> costmap_;
  std::shared_ptr<const HeightmapSnapshot> heightmap_;
  std::string costmap_frame_, heightmap_frame_;
  rclcpp::Time costmap_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time heightmap_stamp_{0, 0, RCL_ROS_TIME};
  std::uint64_t costmap_generation_{0U}, heightmap_generation_{0U};
  std::uint64_t costmap_content_hash_{0U}, heightmap_content_hash_{0U};
  std::uint64_t last_processed_costmap_generation_{0U};
  std::uint64_t last_processed_heightmap_generation_{0U};
  bool pending_costmap_change_{false}, pending_heightmap_change_{false};
  bool replanning_enabled_{true};
  bool replan_on_heightmap_change_{true}, replan_on_costmap_change_{true};
  bool stop_before_hard_replan_{true}, retry_failed_plan_{true};
  bool goal_active_{false}, hard_stop_latched_{false};
  bool automatic_plan_processed_{false};
  double replanning_check_period_s_{0.10}, replanning_min_interval_s_{0.50};
  double soft_reoptimize_min_interval_s_{1.50};
  double min_cost_improvement_ratio_{0.05}, replanning_goal_tolerance_m_{0.20};
  double failed_retry_interval_s_{1.0};
  ActivePlanState grid_active_, sampling_active_;
  geometry_msgs::msg::PoseStamped active_goal_map_;
  Point2D effective_goal_;
  rclcpp::Time last_replan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_soft_replan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_failed_replan_time_{0, 0, RCL_ROS_TIME};
  std::mutex planning_mutex_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr heightmap_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr grid_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr sampling_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr grid_tracking_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr sampling_tracking_path_publisher_;
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
  rclcpp::TimerBase::SharedPtr replan_timer_;
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
