#include "rubi_heightmap_step_wavefront_planner/planning/step_wavefront_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace rubi_heightmap_step_wavefront_planner
{
StepWavefrontPlanner::StepWavefrontPlanner(const StepWavefrontParameters parameters)
: graph_builder_(parameters) {}

std::vector<NodeId> StepWavefrontPlanner::shortestPath(
  const std::vector<GraphNode> & nodes, const std::vector<GraphEdge> & edges,
  const NodeId start, const NodeId goal, const double distance_weight, double * total_cost)
{
  const SearchResult result = AStarSearch{}.search({nodes, edges}, start, goal, distance_weight);
  if (total_cost) {*total_cost = result.total_cost;}
  return result.path_node_ids;
}

PlanResult StepWavefrontPlanner::plan(
  const StepEvaluator & evaluator, const Point2D start, const Point2D goal) const
{
  const auto core_start = std::chrono::steady_clock::now();
  const GraphBuildResult build = graph_builder_.build(evaluator, start, goal);
  PlanResult result;
  result.termination = build.termination;
  result.message = build.message;
  result.nodes = build.graph.nodes;
  result.edges = build.graph.edges;
  result.rejected = build.rejected;
  result.expansions = build.expansion_count;
  result.graph_build_time_ms = build.build_time_ms;
  if (build.goal_node_id) {
    const SearchResult search = astar_search_.search(
      build.graph, build.start_node_id, *build.goal_node_id,
      evaluator.parameters().distance_weight);
    result.success = search.success;
    result.path_node_ids = search.path_node_ids;
    result.path_metrics.total_cost = search.total_cost;
    result.astar_time_ms = search.search_time_ms;
  }
  if (result.success) {
    result.path_metrics.minimum_clearance_m = std::numeric_limits<double>::infinity();
    for (std::size_t path_index = 1U; path_index < result.path_node_ids.size(); ++path_index) {
      const NodeId from = result.path_node_ids[path_index - 1U];
      const NodeId to = result.path_node_ids[path_index];
      for (const auto & edge : result.edges) {
        if ((edge.from == from && edge.to == to) || (edge.from == to && edge.to == from)) {
          result.path_metrics.length_xy_m += edge.evaluation.length_xy_m;
          result.path_metrics.height_event_count += edge.evaluation.height_jump_event_count;
          result.path_metrics.max_height_jump_m = std::max(
            result.path_metrics.max_height_jump_m, edge.evaluation.max_height_jump_m);
          result.path_metrics.height_score_m += edge.evaluation.height_jump_score_m;
          result.path_metrics.clearance_score_m += edge.evaluation.clearance_score_m;
          result.path_metrics.minimum_clearance_m = std::min(
            result.path_metrics.minimum_clearance_m, edge.evaluation.minimum_clearance_m);
          break;
        }
      }
    }
    if (!std::isfinite(result.path_metrics.minimum_clearance_m)) {
      result.path_metrics.minimum_clearance_m = 0.0;
    }
  } else if (result.message.empty()) {result.message = "no accepted graph path to goal";}
  result.core_total_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - core_start).count();
  return result;
}
}  // namespace rubi_heightmap_step_wavefront_planner
