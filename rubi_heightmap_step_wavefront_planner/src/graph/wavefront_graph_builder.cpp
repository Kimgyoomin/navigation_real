#include "rubi_heightmap_step_wavefront_planner/graph/wavefront_graph_builder.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_set>

#include "rubi_heightmap_step_wavefront_planner/graph/spatial_index_2d.hpp"

namespace rubi_heightmap_step_wavefront_planner
{
namespace
{
using Clock = std::chrono::steady_clock;
double milliseconds(const Clock::duration duration) noexcept
{
  return std::chrono::duration<double, std::milli>(duration).count();
}
std::uint64_t edgeKey(NodeId lhs, NodeId rhs) noexcept
{
  if (lhs > rhs) {std::swap(lhs, rhs);}
  return (static_cast<std::uint64_t>(lhs) << 32U) ^ static_cast<std::uint64_t>(rhs);
}
}  // namespace

std::string_view toString(const PlanTermination termination) noexcept
{
  switch (termination) {
    case PlanTermination::kInvalidRequest: return "invalid_request";
    case PlanTermination::kPostGoalComplete: return "post_goal_complete";
    case PlanTermination::kFrontierExhausted: return "frontier_exhausted";
    case PlanTermination::kMaxNodes: return "max_nodes";
    case PlanTermination::kMaxExpansions: return "max_expansions";
    case PlanTermination::kMaxGraphBuildTime: return "max_graph_build_time";
  }
  return "invalid_request";
}

WavefrontGraphBuilder::WavefrontGraphBuilder(const WavefrontGraphParameters parameters)
: parameters_(parameters)
{
  if (!std::isfinite(parameters_.node_sampling_distance_m) ||
    parameters_.node_sampling_distance_m <= 0.0 || parameters_.samples_per_expansion == 0U ||
    !std::isfinite(parameters_.merge_radius_m) || parameters_.merge_radius_m < 0.0 ||
    !std::isfinite(parameters_.neighbor_connection_radius_m) ||
    parameters_.neighbor_connection_radius_m <= 0.0 ||
    !std::isfinite(parameters_.goal_connection_distance_m) ||
    parameters_.goal_connection_distance_m <= 0.0 || parameters_.max_nodes < 2U ||
    parameters_.max_expansions == 0U)
  {throw std::invalid_argument("invalid WavefrontGraphParameters");}
}

GraphBuildResult WavefrontGraphBuilder::build(
  const StepEvaluator & evaluator, const Point2D start, const Point2D goal) const
{
  const auto build_start = Clock::now();
  GraphBuildResult result;
  const NodeEvaluation start_evaluation = evaluator.evaluateNode(start);
  const NodeEvaluation goal_evaluation = evaluator.evaluateNode(goal);
  if (!start_evaluation.valid || !goal_evaluation.valid) {
    result.message = !start_evaluation.valid ? "start is invalid" : "goal is invalid";
    result.rejected.push_back({RejectionKind::kNode,
      !start_evaluation.valid ? start_evaluation.reason : goal_evaluation.reason,
      !start_evaluation.valid ? start : goal, !start_evaluation.valid ? start : goal});
    result.build_time_ms = milliseconds(Clock::now() - build_start);
    return result;
  }
  result.graph.nodes.push_back({0U, start, start_evaluation.elevation_m});
  result.start_node_id = 0U;
  if (std::hypot(goal.x - start.x, goal.y - start.y) <= 1.0e-12) {
    result.goal_node_id = 0U;
    result.termination = PlanTermination::kPostGoalComplete;
    result.build_time_ms = milliseconds(Clock::now() - build_start);
    return result;
  }

  UniformGridSpatialIndex2D index(std::max(
    parameters_.merge_radius_m, parameters_.neighbor_connection_radius_m));
  index.insert(0U, start);
  std::queue<NodeId> frontier;
  frontier.push(0U);
  std::unordered_set<std::uint64_t> edge_keys;
  NodeId goal_id = std::numeric_limits<NodeId>::max();
  bool goal_connected = false;
  std::size_t post_goal_completed = 0U;
  const auto add_edge = [&](const NodeId from, const NodeId to, const EdgeEvaluation & evaluation) {
    if (edge_keys.insert(edgeKey(from, to)).second) {
      result.graph.edges.push_back({from, to, evaluation}); return true;
    }
    result.rejected.push_back({RejectionKind::kDuplicateEdge, StepInvalidReason::kNone,
      result.graph.nodes[from].point, result.graph.nodes[to].point});
    return false;
  };
  const auto connect_goal = [&](const NodeId source) {
    if (std::hypot(result.graph.nodes[source].point.x - goal.x,
      result.graph.nodes[source].point.y - goal.y) > parameters_.goal_connection_distance_m) {return;}
    const EdgeEvaluation edge = evaluator.evaluateEdge(result.graph.nodes[source].point, goal);
    if (!edge.valid) {
      result.rejected.push_back({RejectionKind::kEdge, edge.reason,
        result.graph.nodes[source].point, goal}); return;
    }
    if (!goal_connected) {
      goal_id = result.graph.nodes.size();
      result.graph.nodes.push_back({goal_id, goal, goal_evaluation.elevation_m});
      index.insert(goal_id, goal);
    }
    add_edge(source, goal_id, edge);
    goal_connected = true;
    result.goal_node_id = goal_id;
  };
  connect_goal(0U);

  while (!frontier.empty()) {
    if (goal_connected && post_goal_completed >= parameters_.post_goal_expansions) {
      result.termination = PlanTermination::kPostGoalComplete; break;
    }
    if (result.expansion_count >= parameters_.max_expansions) {
      result.termination = PlanTermination::kMaxExpansions; break;
    }
    if (parameters_.max_graph_build_time_ms > 0U &&
      milliseconds(Clock::now() - build_start) >= parameters_.max_graph_build_time_ms)
    {result.termination = PlanTermination::kMaxGraphBuildTime; break;}
    const NodeId source = frontier.front(); frontier.pop();
    if (source == goal_id) {continue;}
    ++result.expansion_count;
    for (std::size_t sample_index = 0U;
      sample_index < parameters_.samples_per_expansion; ++sample_index)
    {
      const double yaw_rad = 2.0 * std::acos(-1.0) * sample_index /
        parameters_.samples_per_expansion;
      const Point2D proposal{result.graph.nodes[source].point.x +
        parameters_.node_sampling_distance_m * std::cos(yaw_rad),
        result.graph.nodes[source].point.y +
        parameters_.node_sampling_distance_m * std::sin(yaw_rad)};
      const NodeEvaluation node_evaluation = evaluator.evaluateNode(proposal);
      if (!node_evaluation.valid) {
        result.rejected.push_back({RejectionKind::kNode, node_evaluation.reason,
          result.graph.nodes[source].point, proposal}); continue;
      }
      bool merged = false;
      for (const NodeId target : index.radiusSearch(proposal, parameters_.merge_radius_m)) {
        if (target == source) {continue;}
        const EdgeEvaluation edge = evaluator.evaluateEdge(
          result.graph.nodes[source].point, result.graph.nodes[target].point);
        if (edge.valid) {add_edge(source, target, edge); merged = true; break;}
        result.rejected.push_back({RejectionKind::kEdge, edge.reason,
          result.graph.nodes[source].point, result.graph.nodes[target].point});
      }
      if (merged) {continue;}
      if (result.graph.nodes.size() >= parameters_.max_nodes) {
        result.termination = PlanTermination::kMaxNodes; break;
      }
      const EdgeEvaluation parent_edge = evaluator.evaluateEdge(
        result.graph.nodes[source].point, proposal);
      if (!parent_edge.valid) {
        result.rejected.push_back({RejectionKind::kEdge, parent_edge.reason,
          result.graph.nodes[source].point, proposal}); continue;
      }
      const NodeId new_id = result.graph.nodes.size();
      result.graph.nodes.push_back({new_id, proposal, node_evaluation.elevation_m});
      add_edge(source, new_id, parent_edge);
      frontier.push(new_id);
      for (const NodeId neighbor : index.radiusSearch(
          proposal, parameters_.neighbor_connection_radius_m))
      {
        if (neighbor == source) {continue;}
        const EdgeEvaluation edge = evaluator.evaluateEdge(
          proposal, result.graph.nodes[neighbor].point);
        if (edge.valid) {add_edge(new_id, neighbor, edge);} else {
          result.rejected.push_back({RejectionKind::kEdge, edge.reason,
            proposal, result.graph.nodes[neighbor].point});
        }
      }
      index.insert(new_id, proposal);
      connect_goal(new_id);
    }
    if (goal_connected) {++post_goal_completed;}
    if (result.termination == PlanTermination::kMaxNodes) {break;}
  }
  if (result.termination == PlanTermination::kInvalidRequest) {
    result.termination = frontier.empty() ? PlanTermination::kFrontierExhausted :
      PlanTermination::kPostGoalComplete;
  }
  result.build_time_ms = milliseconds(Clock::now() - build_start);
  return result;
}

}  // namespace rubi_heightmap_step_wavefront_planner
