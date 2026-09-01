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
  result.statistics.expanded_states = build.expansion_count;
  result.statistics.accepted_nodes = build.graph.nodes.size();
  result.statistics.accepted_edges = build.graph.edges.size();
  result.graph_build_time_ms = build.build_time_ms;
  result.statistics.sampling_trials = build.sampling_trials;
  result.statistics.candidate_generated = build.candidate_generated;
  result.statistics.candidate_valid = build.candidate_valid;
  result.statistics.candidate_rejected = build.candidate_rejected;
  result.statistics.merge_queries = build.merge_queries;
  result.statistics.neighbor_radius_queries = build.neighbor_radius_queries;
  result.statistics.rejected_edges = build.rejected_edges;
  result.statistics.node_evaluation_calls = build.node_evaluation_calls;
  result.statistics.edge_evaluation_calls = build.edge_evaluation_calls;
  result.statistics.trg_collision_rejects = build.trg_collision_rejects;
  result.statistics.costmap_rejects = build.costmap_rejects;
  result.statistics.existing_node_queries = build.existing_node_queries;
  result.statistics.existing_node_rewires = build.existing_node_rewires;
  result.statistics.new_nodes_created = build.new_nodes_created;
  result.statistics.isolated_nodes = build.isolated_nodes;
  result.statistics.neighbor_queries = build.neighbor_queries;
  result.statistics.neighbor_wire_attempts = build.neighbor_wire_attempts;
  result.graph_clean_time_ms = build.graph_clean_time_ms;
  if (build.goal_node_id) {
    const SearchResult search = astar_search_.search(
      build.graph, build.start_node_id, *build.goal_node_id,
      evaluator.parameters().distance_weight);
    result.success = search.success;
    result.path_node_ids = search.path_node_ids;
    result.path_metrics.total_cost = search.total_cost;
    result.astar_time_ms = search.search_time_ms;
    result.statistics.expanded_states = search.expanded_state_count;
  }
  const auto finalize_start = std::chrono::steady_clock::now();
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
          result.path_metrics.inflation_score_m += edge.evaluation.inflation_score_m;
          result.path_metrics.maximum_raw_cost = std::max(
            result.path_metrics.maximum_raw_cost, edge.evaluation.maximum_raw_cost);
          result.path_metrics.minimum_clearance_m = std::min(
            result.path_metrics.minimum_clearance_m, edge.evaluation.minimum_clearance_m);
          break;
        }
      }
    }
    if (!std::isfinite(result.path_metrics.minimum_clearance_m)) {
      result.path_metrics.minimum_clearance_m = 0.0;
    }
    result.path_metrics.inflation_cost = evaluator.parameters().inflation_cost_weight *
      result.path_metrics.inflation_score_m;
    result.path_metrics.height_cost = evaluator.parameters().height_cost_weight *
      result.path_metrics.height_score_m;
  } else if (result.message.empty()) {result.message = "no accepted graph path to goal";}
  const auto & instrumentation = evaluator.instrumentation();
  result.statistics.edge_samples_total = instrumentation.edge_samples_total;
  result.statistics.height_evidence_queries = instrumentation.height_evidence_queries;
  result.statistics.costmap_queries = instrumentation.costmap_queries;
  result.path_finalize_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - finalize_start).count();
  result.core_total_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - core_start).count();
  return result;
}
}  // namespace rubi_heightmap_step_wavefront_planner
