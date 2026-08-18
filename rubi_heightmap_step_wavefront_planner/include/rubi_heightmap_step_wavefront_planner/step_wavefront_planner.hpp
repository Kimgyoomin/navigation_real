#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "rubi_heightmap_step_wavefront_planner/step_evaluator.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

using NodeId = std::size_t;

enum class PlanTermination
{
  kInvalidRequest,
  kPostGoalComplete,
  kFrontierExhausted,
  kMaxNodes,
  kMaxExpansions,
  kMaxGraphBuildTime,
};

std::string_view toString(PlanTermination termination) noexcept;

enum class RejectionKind
{
  kNode,
  kEdge,
  kDuplicateEdge,
};

struct GraphNode
{
  NodeId id{0U};
  Point2D point;
  double elevation_m{0.0};
};

struct GraphEdge
{
  NodeId from{0U};
  NodeId to{0U};
  EdgeEvaluation evaluation;
};

struct RejectedProposal
{
  RejectionKind kind{RejectionKind::kNode};
  StepInvalidReason reason{StepInvalidReason::kInvalidInput};
  Point2D from;
  Point2D to;
};

struct StepWavefrontParameters
{
  double node_sampling_distance_m{0.30};
  std::size_t samples_per_expansion{20U};
  double merge_radius_m{0.20};
  double neighbor_connection_radius_m{0.45};
  double goal_connection_distance_m{0.45};
  std::size_t max_nodes{4000U};
  std::size_t max_expansions{4000U};
  std::size_t max_graph_build_time_ms{5000U};
  std::size_t post_goal_expansions{50U};
};

struct PathMetrics
{
  double length_xy_m{0.0};
  std::size_t height_event_count{0U};
  double max_height_jump_m{0.0};
  double height_score_m{0.0};
  double total_cost{0.0};
};

struct PlanResult
{
  bool success{false};
  PlanTermination termination{PlanTermination::kInvalidRequest};
  std::string message;
  std::vector<GraphNode> nodes;
  std::vector<GraphEdge> edges;
  std::vector<RejectedProposal> rejected;
  std::vector<NodeId> path_node_ids;
  std::size_t expansions{0U};
  double graph_build_time_ms{0.0};
  double astar_time_ms{0.0};
  double core_total_time_ms{0.0};
  PathMetrics path_metrics;
};

class StepWavefrontPlanner
{
public:
  explicit StepWavefrontPlanner(StepWavefrontParameters parameters);
  PlanResult plan(
    const StepEvaluator & evaluator,
    Point2D start,
    Point2D goal) const;

  static std::vector<NodeId> shortestPath(
    const std::vector<GraphNode> & nodes,
    const std::vector<GraphEdge> & edges,
    NodeId start,
    NodeId goal,
    double distance_weight,
    double * total_cost = nullptr);

private:
  StepWavefrontParameters parameters_;
};

}  // namespace rubi_heightmap_step_wavefront_planner
