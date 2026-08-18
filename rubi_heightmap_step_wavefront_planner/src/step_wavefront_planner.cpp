#include "rubi_heightmap_step_wavefront_planner/step_wavefront_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <unordered_set>

namespace rubi_heightmap_step_wavefront_planner
{
namespace
{

using Clock = std::chrono::steady_clock;

double milliseconds(const Clock::duration duration) noexcept
{
  return std::chrono::duration<double, std::milli>(duration).count();
}

std::uint64_t edgeKey(NodeId a, NodeId b) noexcept
{
  if (a > b) {std::swap(a, b);}
  return (static_cast<std::uint64_t>(a) << 32U) ^ static_cast<std::uint64_t>(b);
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

StepWavefrontPlanner::StepWavefrontPlanner(const StepWavefrontParameters parameters)
: parameters_(parameters)
{
  if (!std::isfinite(parameters_.node_sampling_distance_m) ||
    parameters_.node_sampling_distance_m <= 0.0 ||
    parameters_.samples_per_expansion == 0U ||
    !std::isfinite(parameters_.merge_radius_m) || parameters_.merge_radius_m < 0.0 ||
    !std::isfinite(parameters_.neighbor_connection_radius_m) ||
    parameters_.neighbor_connection_radius_m <= 0.0 ||
    !std::isfinite(parameters_.goal_connection_distance_m) ||
    parameters_.goal_connection_distance_m <= 0.0 ||
    parameters_.max_nodes < 2U || parameters_.max_expansions == 0U)
  {
    throw std::invalid_argument("invalid StepWavefrontPlanner parameters");
  }
}

std::vector<NodeId> StepWavefrontPlanner::shortestPath(
  const std::vector<GraphNode> & nodes,
  const std::vector<GraphEdge> & edges,
  const NodeId start,
  const NodeId goal,
  const double distance_weight,
  double * total_cost)
{
  if (start >= nodes.size() || goal >= nodes.size() ||
    !std::isfinite(distance_weight) || distance_weight <= 0.0)
  {
    return {};
  }
  struct Adjacent {NodeId id; double cost;};
  std::vector<std::vector<Adjacent>> adjacency(nodes.size());
  for (const auto & edge : edges) {
    if (!edge.evaluation.valid || edge.from >= nodes.size() || edge.to >= nodes.size() ||
      !std::isfinite(edge.evaluation.cost) || edge.evaluation.cost < 0.0)
    {
      continue;
    }
    adjacency[edge.from].push_back({edge.to, edge.evaluation.cost});
    adjacency[edge.to].push_back({edge.from, edge.evaluation.cost});
  }
  for (auto & neighbors : adjacency) {
    std::sort(
      neighbors.begin(), neighbors.end(), [](const auto & a, const auto & b) {
        return std::tie(a.id, a.cost) < std::tie(b.id, b.cost);
      });
  }

  struct OpenEntry {double f; double g; NodeId id;};
  struct Greater
  {
    bool operator()(const OpenEntry & a, const OpenEntry & b) const noexcept
    {
      return std::tie(a.f, a.g, a.id) > std::tie(b.f, b.g, b.id);
    }
  };
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> g(nodes.size(), infinity);
  std::vector<NodeId> parent(nodes.size(), nodes.size());
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, Greater> open;
  auto heuristic = [&](const NodeId id) {
      // Every accepted edge costs at least w_d * L_xy because its height
      // penalty is nonnegative, so this Euclidean heuristic is admissible.
      return distance_weight * std::hypot(
        nodes[id].point.x - nodes[goal].point.x,
        nodes[id].point.y - nodes[goal].point.y);
    };
  g[start] = 0.0;
  open.push({heuristic(start), 0.0, start});
  while (!open.empty()) {
    const OpenEntry current = open.top();
    open.pop();
    if (current.g != g[current.id]) {continue;}
    if (current.id == goal) {break;}
    for (const auto & neighbor : adjacency[current.id]) {
      const double tentative = current.g + neighbor.cost;
      if (tentative + 1.0e-12 < g[neighbor.id] ||
        (std::abs(tentative - g[neighbor.id]) <= 1.0e-12 && current.id < parent[neighbor.id]))
      {
        g[neighbor.id] = tentative;
        parent[neighbor.id] = current.id;
        open.push({tentative + heuristic(neighbor.id), tentative, neighbor.id});
      }
    }
  }
  if (!std::isfinite(g[goal])) {return {};}
  std::vector<NodeId> reversed;
  for (NodeId id = goal; ; id = parent[id]) {
    reversed.push_back(id);
    if (id == start) {break;}
    if (parent[id] >= nodes.size() || reversed.size() > nodes.size()) {return {};}
  }
  std::reverse(reversed.begin(), reversed.end());
  if (total_cost) {*total_cost = g[goal];}
  return reversed;
}

PlanResult StepWavefrontPlanner::plan(
  const StepEvaluator & evaluator,
  const Point2D start,
  const Point2D goal) const
{
  const auto core_start = Clock::now();
  const auto build_start = core_start;
  PlanResult result;
  const NodeEvaluation start_evaluation = evaluator.evaluateNode(start);
  const NodeEvaluation goal_evaluation = evaluator.evaluateNode(goal);
  if (!start_evaluation.valid || !goal_evaluation.valid) {
    result.termination = PlanTermination::kInvalidRequest;
    result.message = !start_evaluation.valid ? "start is invalid" : "goal is invalid";
    result.rejected.push_back(
      {
        RejectionKind::kNode,
        !start_evaluation.valid ? start_evaluation.reason : goal_evaluation.reason,
        !start_evaluation.valid ? start : goal,
        !start_evaluation.valid ? start : goal});
    result.graph_build_time_ms = milliseconds(Clock::now() - build_start);
    result.core_total_time_ms = milliseconds(Clock::now() - core_start);
    return result;
  }
  result.nodes.push_back({0U, start, start_evaluation.elevation_m});
  if (std::hypot(goal.x - start.x, goal.y - start.y) <= 1.0e-12) {
    result.success = true;
    result.termination = PlanTermination::kPostGoalComplete;
    result.path_node_ids = {0U};
    result.graph_build_time_ms = milliseconds(Clock::now() - build_start);
    result.core_total_time_ms = milliseconds(Clock::now() - core_start);
    return result;
  }

  std::queue<NodeId> frontier;
  frontier.push(0U);
  std::unordered_set<std::uint64_t> edge_keys;
  NodeId goal_id = std::numeric_limits<NodeId>::max();
  bool goal_connected = false;
  std::size_t post_goal_completed = 0U;
  auto add_edge = [&](NodeId from, NodeId to, const EdgeEvaluation & evaluation) {
      const auto key = edgeKey(from, to);
      if (edge_keys.insert(key).second) {
        result.edges.push_back({from, to, evaluation});
        return true;
      }
      result.rejected.push_back(
      {
        RejectionKind::kDuplicateEdge, StepInvalidReason::kNone,
        result.nodes[from].point, result.nodes[to].point});
      return false;
    };
  auto connect_goal = [&](NodeId source) {
      if (std::hypot(
          result.nodes[source].point.x - goal.x,
          result.nodes[source].point.y - goal.y) > parameters_.goal_connection_distance_m)
      {
        return;
      }
      const EdgeEvaluation edge = evaluator.evaluateEdge(result.nodes[source].point, goal);
      if (!edge.valid) {
        result.rejected.push_back(
        {
          RejectionKind::kEdge, edge.reason, result.nodes[source].point, goal});
        return;
      }
      if (!goal_connected) {
        goal_id = result.nodes.size();
        result.nodes.push_back({goal_id, goal, goal_evaluation.elevation_m});
      }
      add_edge(source, goal_id, edge);
      goal_connected = true;
    };
  connect_goal(0U);

  while (!frontier.empty()) {
    if (goal_connected && post_goal_completed >= parameters_.post_goal_expansions) {
      result.termination = PlanTermination::kPostGoalComplete;
      break;
    }
    if (result.expansions >= parameters_.max_expansions) {
      result.termination = PlanTermination::kMaxExpansions;
      break;
    }
    if (parameters_.max_graph_build_time_ms > 0U &&
      milliseconds(Clock::now() - build_start) >=
      static_cast<double>(parameters_.max_graph_build_time_ms))
    {
      result.termination = PlanTermination::kMaxGraphBuildTime;
      break;
    }
    const NodeId source = frontier.front();
    frontier.pop();
    if (source == goal_id) {continue;}
    ++result.expansions;
    for (std::size_t sample = 0U; sample < parameters_.samples_per_expansion; ++sample) {
      const double angle = 2.0 * std::acos(-1.0) *
        static_cast<double>(sample) /
        static_cast<double>(parameters_.samples_per_expansion);
      const Point2D proposal{
        result.nodes[source].point.x + parameters_.node_sampling_distance_m * std::cos(angle),
        result.nodes[source].point.y + parameters_.node_sampling_distance_m * std::sin(angle)};
      const NodeEvaluation node_evaluation = evaluator.evaluateNode(proposal);
      if (!node_evaluation.valid) {
        result.rejected.push_back(
          {
            RejectionKind::kNode, node_evaluation.reason,
            result.nodes[source].point, proposal});
        continue;
      }
      std::vector<std::pair<double, NodeId>> merge_targets;
      for (const auto & node : result.nodes) {
        const double distance_squared =
          std::pow(node.point.x - proposal.x, 2) + std::pow(node.point.y - proposal.y, 2);
        if (distance_squared <= parameters_.merge_radius_m * parameters_.merge_radius_m) {
          merge_targets.emplace_back(distance_squared, node.id);
        }
      }
      std::sort(merge_targets.begin(), merge_targets.end());
      bool merged = false;
      for (const auto & target : merge_targets) {
        if (target.second == source) {continue;}
        const EdgeEvaluation edge =
          evaluator.evaluateEdge(result.nodes[source].point, result.nodes[target.second].point);
        if (edge.valid) {
          add_edge(source, target.second, edge);
          merged = true;
          break;
        }
        result.rejected.push_back(
          {
            RejectionKind::kEdge, edge.reason,
            result.nodes[source].point, result.nodes[target.second].point});
      }
      if (merged) {continue;}
      if (result.nodes.size() >= parameters_.max_nodes) {
        result.termination = PlanTermination::kMaxNodes;
        break;
      }
      const EdgeEvaluation parent_edge =
        evaluator.evaluateEdge(result.nodes[source].point, proposal);
      if (!parent_edge.valid) {
        result.rejected.push_back(
          {
            RejectionKind::kEdge, parent_edge.reason,
            result.nodes[source].point, proposal});
        continue;
      }
      const NodeId new_id = result.nodes.size();
      result.nodes.push_back({new_id, proposal, node_evaluation.elevation_m});
      add_edge(source, new_id, parent_edge);
      frontier.push(new_id);
      std::vector<std::pair<double, NodeId>> neighbors;
      for (NodeId id = 0U; id < new_id; ++id) {
        if (id == source) {continue;}
        const double distance_squared =
          std::pow(result.nodes[id].point.x - proposal.x, 2) +
          std::pow(result.nodes[id].point.y - proposal.y, 2);
        if (distance_squared <=
          parameters_.neighbor_connection_radius_m *
          parameters_.neighbor_connection_radius_m)
        {
          neighbors.emplace_back(distance_squared, id);
        }
      }
      std::sort(neighbors.begin(), neighbors.end());
      for (const auto & neighbor : neighbors) {
        const EdgeEvaluation edge =
          evaluator.evaluateEdge(proposal, result.nodes[neighbor.second].point);
        if (edge.valid) {add_edge(new_id, neighbor.second, edge);} else {
          result.rejected.push_back(
            {
              RejectionKind::kEdge, edge.reason,
              proposal, result.nodes[neighbor.second].point});
        }
      }
      connect_goal(new_id);
    }
    if (goal_connected) {++post_goal_completed;}
    if (result.termination == PlanTermination::kMaxNodes) {break;}
  }
  if (result.termination == PlanTermination::kInvalidRequest) {
    result.termination = frontier.empty() ?
      PlanTermination::kFrontierExhausted : PlanTermination::kPostGoalComplete;
  }
  result.graph_build_time_ms = milliseconds(Clock::now() - build_start);

  const auto astar_start = Clock::now();
  if (goal_connected) {
    result.path_node_ids = shortestPath(
      result.nodes, result.edges, 0U, goal_id,
      evaluator.parameters().distance_weight, &result.path_metrics.total_cost);
    result.success = !result.path_node_ids.empty();
  }
  if (result.success) {
    for (std::size_t index = 1U; index < result.path_node_ids.size(); ++index) {
      const NodeId a = result.path_node_ids[index - 1U];
      const NodeId b = result.path_node_ids[index];
      for (const auto & edge : result.edges) {
        if ((edge.from == a && edge.to == b) || (edge.from == b && edge.to == a)) {
          result.path_metrics.length_xy_m += edge.evaluation.length_xy_m;
          result.path_metrics.height_event_count += edge.evaluation.height_jump_event_count;
          result.path_metrics.max_height_jump_m = std::max(
            result.path_metrics.max_height_jump_m, edge.evaluation.max_height_jump_m);
          result.path_metrics.height_score_m += edge.evaluation.height_jump_score_m;
          break;
        }
      }
    }
  } else {
    result.message = "no accepted graph path to goal";
  }
  result.astar_time_ms = milliseconds(Clock::now() - astar_start);
  result.core_total_time_ms = milliseconds(Clock::now() - core_start);
  return result;
}

}  // namespace rubi_heightmap_step_wavefront_planner
