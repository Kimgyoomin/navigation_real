#pragma once

#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "rubi_heightmap_wavefront_planner/terrain_evaluator.hpp"

namespace rubi_heightmap_wavefront_planner
{

using NodeId = std::size_t;

struct WavefrontRiskWeights
{
  // The resulting edge cost is:
  //   length_3d_m * (distance + slope * normalized_slope_risk)
  // + step * max_step_m.
  //
  // Step is a hard validity gate in TerrainEvaluator. Its soft weight is zero
  // by default and is retained only for controlled comparisons.
  double distance{1.0};
  double slope{1.0};
  double step{0.0};
};

struct WavefrontPlannerParameters
{
  double node_sampling_distance_m{0.5};
  // The ROS parameter `samples_per_expansion` maps to this field.
  std::size_t num_expansion_samples{16};
  double merge_radius_m{0.20};
  double neighbor_connection_radius_m{0.75};
  std::size_t max_nodes{2000};
  std::size_t max_expansions{2000};
  std::size_t max_build_time_ms{2000};
  double goal_connection_distance_m{0.75};
  bool stop_when_goal_connected{true};
  // This should normally equal TerrainEvaluator's hard maximum slope.
  double slope_normalization_deg{15.0};
  WavefrontRiskWeights risk_weights{};
};

enum class GraphNodeRole
{
  kStart,
  kGoal,
  kSampled
};

struct GraphNode
{
  NodeId id{0};
  TerrainPoint point{};
  NodeEvaluation terrain{};
  GraphNodeRole role{GraphNodeRole::kSampled};
  std::size_t wavefront_depth{0};
};

struct GraphEdge
{
  NodeId from{0};
  NodeId to{0};
  EdgeEvaluation terrain{};
  double cost{std::numeric_limits<double>::infinity()};
  bool is_goal_connection{false};
  bool is_loop_closure{false};
};

enum class RejectedSampleKind
{
  kNodeInvalid,
  kExpansionEdgeInvalid,
  kMergeEdgeInvalid,
  kGoalEdgeInvalid,
  kNonFiniteEvaluation,
  kDuplicateEdge
};

struct RejectedSample
{
  NodeId source{0};
  Point2D candidate{};
  RejectedSampleKind kind{RejectedSampleKind::kNodeInvalid};
  TerrainInvalidReason terrain_reason{TerrainInvalidReason::kNone};
};

struct RejectionCounters
{
  std::size_t node_invalid{0};
  std::size_t expansion_edge_invalid{0};
  std::size_t merge_edge_invalid{0};
  std::size_t goal_edge_invalid{0};
  std::size_t non_finite_evaluation{0};
  std::size_t duplicate_edge{0};
};

enum class WavefrontTermination
{
  kInvalidRequest,
  kGoalConnected,
  kFrontierExhausted,
  kMaxNodesReached,
  kMaxExpansionsReached,
  kMaxBuildTimeReached
};

struct PlanResult
{
  bool success{false};
  std::string message;

  std::vector<GraphNode> nodes;
  std::vector<GraphEdge> edges;
  std::vector<NodeId> path_node_ids;
  std::vector<TerrainPoint> path;
  std::vector<RejectedSample> rejected;
  RejectionCounters reject_counts{};

  WavefrontTermination termination{WavefrontTermination::kInvalidRequest};
  std::size_t expansions{0};
  std::size_t rewires{0};
  std::size_t goal_connections{0};
  bool node_budget_reached{false};
  bool expansion_budget_reached{false};
  bool build_time_budget_reached{false};
  bool stopped_on_goal_connection{false};
  bool frontier_exhausted{false};
  double build_time_ms{0.0};
};

class WavefrontPlanner
{
public:
  using NodeEvaluator =
    std::function<NodeEvaluation(const Point2D &)>;
  using EdgeEvaluator =
    std::function<EdgeEvaluation(const Point2D &, const Point2D &)>;

  explicit WavefrontPlanner(
    WavefrontPlannerParameters parameters = WavefrontPlannerParameters{});

  const WavefrontPlannerParameters & parameters() const noexcept;

  PlanResult plan(
    const TerrainEvaluator & terrain,
    const Point2D & start,
    const Point2D & goal) const;

  // Callback overload used by ROS-independent unit tests and adapters. Callers
  // must return deterministic evaluations for deterministic graph construction.
  PlanResult plan(
    const Point2D & start,
    const Point2D & goal,
    const NodeEvaluator & evaluate_node,
    const EdgeEvaluator & evaluate_edge) const;

private:
  WavefrontPlannerParameters parameters_;
};

}  // namespace rubi_heightmap_wavefront_planner
