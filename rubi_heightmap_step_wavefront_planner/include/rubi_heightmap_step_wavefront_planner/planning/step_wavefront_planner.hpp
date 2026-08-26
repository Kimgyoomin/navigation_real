#pragma once

#include <string>
#include <vector>

#include "rubi_heightmap_step_wavefront_planner/graph/wavefront_graph_builder.hpp"
#include "rubi_heightmap_step_wavefront_planner/search/astar_search.hpp"

namespace rubi_heightmap_step_wavefront_planner
{
using StepWavefrontParameters = WavefrontGraphParameters;
struct PathMetrics
{
  double length_xy_m{0.0};
  std::size_t height_event_count{0U};
  double max_height_jump_m{0.0};
  double height_score_m{0.0};
  double minimum_clearance_m{0.0};
  double clearance_score_m{0.0};
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
/** @brief Orchestrates graph construction, A* search, and path metrics. */
class StepWavefrontPlanner
{
public:
  explicit StepWavefrontPlanner(StepWavefrontParameters parameters);
  PlanResult plan(const StepEvaluator & evaluator, Point2D start, Point2D goal) const;
  static std::vector<NodeId> shortestPath(
    const std::vector<GraphNode> & nodes, const std::vector<GraphEdge> & edges,
    NodeId start, NodeId goal, double distance_weight, double * total_cost = nullptr);
private:
  WavefrontGraphBuilder graph_builder_;
  AStarSearch astar_search_;
};
}  // namespace rubi_heightmap_step_wavefront_planner
