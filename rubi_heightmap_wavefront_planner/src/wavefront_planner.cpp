#include "rubi_heightmap_wavefront_planner/wavefront_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace rubi_heightmap_wavefront_planner
{
namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kGoldenAngle = kPi * (3.0 - 2.236067977499789696409173668731276);
constexpr double kComparisonTolerance = 1.0e-12;

bool isFinite(const double value)
{
  return std::isfinite(value);
}

bool isFinite(const Point2D & point)
{
  return isFinite(point.x) && isFinite(point.y);
}

double planarDistanceSquared(const Point2D & lhs, const Point2D & rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  return dx * dx + dy * dy;
}

double planarDistance(const Point2D & lhs, const Point2D & rhs)
{
  return std::sqrt(planarDistanceSquared(lhs, rhs));
}

Point2D toPoint2D(const TerrainPoint & point)
{
  return Point2D{point.x, point.y};
}

TerrainPoint toTerrainPoint(const Point2D & point, const NodeEvaluation & evaluation)
{
  return TerrainPoint{point.x, point.y, evaluation.elevation_m};
}

bool hasFiniteValidNodeEvaluation(const NodeEvaluation & evaluation)
{
  return evaluation.valid && isFinite(evaluation.elevation_m);
}

bool hasFiniteValidEdgeEvaluation(const EdgeEvaluation & evaluation)
{
  return
    evaluation.valid &&
    isFinite(evaluation.length_3d_m) &&
    isFinite(evaluation.max_slope_deg) &&
    isFinite(evaluation.max_step_m) &&
    evaluation.length_3d_m >= 0.0 &&
    evaluation.max_slope_deg >= 0.0 &&
    evaluation.max_step_m >= 0.0;
}

double edgeCost(
  const EdgeEvaluation & evaluation,
  const WavefrontPlannerParameters & parameters)
{
  const double normalized_slope_risk = std::clamp(
    evaluation.max_slope_deg / parameters.slope_normalization_deg, 0.0, 1.0);
  return
    evaluation.length_3d_m *
    (parameters.risk_weights.distance +
    parameters.risk_weights.slope * normalized_slope_risk) +
    parameters.risk_weights.step * evaluation.max_step_m;
}

std::pair<NodeId, NodeId> canonicalEdge(const NodeId lhs, const NodeId rhs)
{
  return (lhs < rhs) ? std::make_pair(lhs, rhs) : std::make_pair(rhs, lhs);
}

const char * terminationName(const WavefrontTermination termination)
{
  switch (termination) {
    case WavefrontTermination::kInvalidRequest:
      return "invalid request";
    case WavefrontTermination::kGoalConnected:
      return "stopped after goal connection";
    case WavefrontTermination::kFrontierExhausted:
      return "frontier exhausted";
    case WavefrontTermination::kMaxNodesReached:
      return "maximum node budget reached";
    case WavefrontTermination::kMaxExpansionsReached:
      return "maximum expansion budget reached";
    case WavefrontTermination::kMaxBuildTimeReached:
      return "maximum graph-build time reached";
  }
  return "unknown termination";
}

std::vector<NodeId> shortestPathAStar(
  const std::vector<GraphNode> & nodes,
  const std::vector<GraphEdge> & edges,
  const NodeId start,
  const NodeId goal,
  const double distance_weight)
{
  if (start >= nodes.size() || goal >= nodes.size()) {
    return {};
  }

  struct Adjacency
  {
    NodeId neighbor;
    double cost;
  };
  std::vector<std::vector<Adjacency>> adjacency(nodes.size());
  for (const auto & edge : edges) {
    if (
      edge.from >= nodes.size() || edge.to >= nodes.size() ||
      !isFinite(edge.cost) || edge.cost < 0.0)
    {
      continue;
    }
    adjacency[edge.from].push_back(Adjacency{edge.to, edge.cost});
    adjacency[edge.to].push_back(Adjacency{edge.from, edge.cost});
  }
  for (auto & neighbors : adjacency) {
    std::sort(
      neighbors.begin(), neighbors.end(),
      [](const Adjacency & lhs, const Adjacency & rhs) {
        if (lhs.neighbor != rhs.neighbor) {
          return lhs.neighbor < rhs.neighbor;
        }
        return lhs.cost < rhs.cost;
      });
  }

  const auto heuristic = [&](const NodeId id) {
      return distance_weight * planarDistance(
        toPoint2D(nodes[id].point), toPoint2D(nodes[goal].point));
    };

  struct QueueEntry
  {
    NodeId id;
    double g;
    double f;
  };
  struct QueueEntryGreater
  {
    bool operator()(const QueueEntry & lhs, const QueueEntry & rhs) const
    {
      if (lhs.f != rhs.f) {
        return lhs.f > rhs.f;
      }
      if (lhs.g != rhs.g) {
        return lhs.g > rhs.g;
      }
      return lhs.id > rhs.id;
    }
  };

  const NodeId no_parent = std::numeric_limits<NodeId>::max();
  std::vector<double> g_score(nodes.size(), std::numeric_limits<double>::infinity());
  std::vector<NodeId> parent(nodes.size(), no_parent);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater> open;

  g_score[start] = 0.0;
  open.push(QueueEntry{start, 0.0, heuristic(start)});

  while (!open.empty()) {
    const QueueEntry current = open.top();
    open.pop();

    if (current.g > g_score[current.id] + kComparisonTolerance) {
      continue;
    }
    if (current.id == goal) {
      break;
    }

    for (const auto & adjacent : adjacency[current.id]) {
      const double tentative = current.g + adjacent.cost;
      if (tentative + kComparisonTolerance >= g_score[adjacent.neighbor]) {
        continue;
      }
      g_score[adjacent.neighbor] = tentative;
      parent[adjacent.neighbor] = current.id;
      open.push(
        QueueEntry{
          adjacent.neighbor, tentative, tentative + heuristic(adjacent.neighbor)});
    }
  }

  if (!isFinite(g_score[goal])) {
    return {};
  }

  std::vector<NodeId> reverse_path;
  for (NodeId id = goal; id != no_parent; id = parent[id]) {
    reverse_path.push_back(id);
    if (id == start) {
      break;
    }
  }
  if (reverse_path.empty() || reverse_path.back() != start) {
    return {};
  }
  return std::vector<NodeId>(reverse_path.rbegin(), reverse_path.rend());
}

void validateParameters(const WavefrontPlannerParameters & parameters)
{
  const auto require_positive_finite = [](const double value, const char * name) {
      if (!isFinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and > 0");
      }
    };
  const auto require_nonnegative_finite = [](const double value, const char * name) {
      if (!isFinite(value) || value < 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and >= 0");
      }
    };

  require_positive_finite(
    parameters.node_sampling_distance_m, "node_sampling_distance_m");
  if (parameters.num_expansion_samples < 3U) {
    throw std::invalid_argument("num_expansion_samples must be >= 3");
  }
  require_positive_finite(parameters.merge_radius_m, "merge_radius_m");
  require_positive_finite(
    parameters.neighbor_connection_radius_m, "neighbor_connection_radius_m");
  if (parameters.max_nodes < 2U) {
    throw std::invalid_argument("max_nodes must reserve at least start and goal");
  }
  if (parameters.max_expansions == 0U) {
    throw std::invalid_argument("max_expansions must be > 0");
  }
  if (parameters.max_build_time_ms == 0U) {
    throw std::invalid_argument("max_build_time_ms must be > 0");
  }
  require_positive_finite(
    parameters.goal_connection_distance_m, "goal_connection_distance_m");
  require_positive_finite(
    parameters.slope_normalization_deg, "slope_normalization_deg");
  require_nonnegative_finite(parameters.risk_weights.distance, "risk_weights.distance");
  require_nonnegative_finite(parameters.risk_weights.slope, "risk_weights.slope");
  require_nonnegative_finite(parameters.risk_weights.step, "risk_weights.step");
  if (
    parameters.risk_weights.distance == 0.0 &&
    parameters.risk_weights.slope == 0.0 &&
    parameters.risk_weights.step == 0.0)
  {
    throw std::invalid_argument("at least one risk weight must be > 0");
  }
}

}  // namespace

WavefrontPlanner::WavefrontPlanner(WavefrontPlannerParameters parameters)
: parameters_(std::move(parameters))
{
  validateParameters(parameters_);
}

const WavefrontPlannerParameters & WavefrontPlanner::parameters() const noexcept
{
  return parameters_;
}

PlanResult WavefrontPlanner::plan(
  const TerrainEvaluator & terrain,
  const Point2D & start,
  const Point2D & goal) const
{
  return plan(
    start, goal,
    [&terrain](const Point2D & point) {
      return terrain.evaluateNode(point);
    },
    [&terrain](const Point2D & from, const Point2D & to) {
      return terrain.evaluateEdge(from, to);
    });
}

PlanResult WavefrontPlanner::plan(
  const Point2D & start,
  const Point2D & goal,
  const NodeEvaluator & evaluate_node,
  const EdgeEvaluator & evaluate_edge) const
{
  PlanResult result;
  const auto build_start = std::chrono::steady_clock::now();
  const auto update_build_time = [&]() {
      result.build_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - build_start).count();
      return result.build_time_ms;
    };
  if (!evaluate_node || !evaluate_edge) {
    result.message = "node and edge evaluators are required";
    return result;
  }
  if (!isFinite(start) || !isFinite(goal)) {
    result.message = "start and goal must contain finite coordinates";
    return result;
  }

  const NodeEvaluation start_evaluation = evaluate_node(start);
  if (!hasFiniteValidNodeEvaluation(start_evaluation)) {
    result.rejected.push_back(
      RejectedSample{
        0U, start,
        start_evaluation.valid ?
        RejectedSampleKind::kNonFiniteEvaluation :
        RejectedSampleKind::kNodeInvalid,
        start_evaluation.reason});
    if (start_evaluation.valid) {
      ++result.reject_counts.non_finite_evaluation;
      result.message = "start node evaluation was non-finite";
    } else {
      ++result.reject_counts.node_invalid;
      result.message = "start node is terrain-invalid";
    }
    return result;
  }

  const NodeEvaluation goal_evaluation = evaluate_node(goal);
  if (!hasFiniteValidNodeEvaluation(goal_evaluation)) {
    result.rejected.push_back(
      RejectedSample{
        0U, goal,
        goal_evaluation.valid ?
        RejectedSampleKind::kNonFiniteEvaluation :
        RejectedSampleKind::kNodeInvalid,
        goal_evaluation.reason});
    if (goal_evaluation.valid) {
      ++result.reject_counts.non_finite_evaluation;
      result.message = "goal node evaluation was non-finite";
    } else {
      ++result.reject_counts.node_invalid;
      result.message = "goal node is terrain-invalid";
    }
    return result;
  }

  constexpr NodeId start_id = 0U;
  constexpr NodeId goal_id = 1U;
  result.nodes.push_back(
    GraphNode{
      start_id, toTerrainPoint(start, start_evaluation), start_evaluation,
      GraphNodeRole::kStart, 0U});
  result.nodes.push_back(
    GraphNode{
      goal_id, toTerrainPoint(goal, goal_evaluation), goal_evaluation,
      GraphNodeRole::kGoal, 0U});

  std::deque<NodeId> frontier;
  frontier.push_back(start_id);
  std::set<std::pair<NodeId, NodeId>> edge_keys;

  const auto reject = [&](const NodeId source, const Point2D & candidate,
      const RejectedSampleKind kind, const TerrainInvalidReason terrain_reason) {
      result.rejected.push_back(
        RejectedSample{source, candidate, kind, terrain_reason});
      switch (kind) {
        case RejectedSampleKind::kNodeInvalid:
          ++result.reject_counts.node_invalid;
          break;
        case RejectedSampleKind::kExpansionEdgeInvalid:
          ++result.reject_counts.expansion_edge_invalid;
          break;
        case RejectedSampleKind::kMergeEdgeInvalid:
          ++result.reject_counts.merge_edge_invalid;
          break;
        case RejectedSampleKind::kGoalEdgeInvalid:
          ++result.reject_counts.goal_edge_invalid;
          break;
        case RejectedSampleKind::kNonFiniteEvaluation:
          ++result.reject_counts.non_finite_evaluation;
          break;
        case RejectedSampleKind::kDuplicateEdge:
          ++result.reject_counts.duplicate_edge;
          break;
      }
    };

  const auto add_edge = [&](
      const NodeId from, const NodeId to, const EdgeEvaluation & evaluation,
      const bool is_goal_connection, const bool is_loop_closure) {
      const auto key = canonicalEdge(from, to);
      if (from == to || edge_keys.find(key) != edge_keys.end()) {
        reject(
          from, toPoint2D(result.nodes[to].point),
          RejectedSampleKind::kDuplicateEdge, TerrainInvalidReason::kNone);
        return false;
      }

      const double cost = edgeCost(evaluation, parameters_);
      if (!isFinite(cost) || cost < 0.0) {
        reject(
          from, toPoint2D(result.nodes[to].point),
          RejectedSampleKind::kNonFiniteEvaluation, evaluation.reason);
        return false;
      }

      edge_keys.insert(key);
      result.edges.push_back(
        GraphEdge{
          from, to, evaluation, cost, is_goal_connection, is_loop_closure});
      return true;
    };

  const auto try_goal_connection = [&](const NodeId source) {
      if (source == goal_id) {
        return;
      }
      const Point2D source_point = toPoint2D(result.nodes[source].point);
      if (
        planarDistance(source_point, goal) >
        parameters_.goal_connection_distance_m + kComparisonTolerance)
      {
        return;
      }

      const EdgeEvaluation evaluation = evaluate_edge(source_point, goal);
      if (!evaluation.valid) {
        reject(
          source, goal, RejectedSampleKind::kGoalEdgeInvalid, evaluation.reason);
        return;
      }
      if (!hasFiniteValidEdgeEvaluation(evaluation)) {
        reject(
          source, goal, RejectedSampleKind::kNonFiniteEvaluation, evaluation.reason);
        return;
      }
      if (add_edge(source, goal_id, evaluation, true, false)) {
        ++result.goal_connections;
      }
    };

  try_goal_connection(start_id);

  bool stop_for_node_budget = false;
  bool stop_for_time_budget = false;
  bool stop_for_goal_connection =
    parameters_.stop_when_goal_connected && result.goal_connections > 0U;
  while (!frontier.empty()) {
    if (stop_for_goal_connection) {
      result.stopped_on_goal_connection = true;
      break;
    }
    if (update_build_time() >= static_cast<double>(parameters_.max_build_time_ms)) {
      result.build_time_budget_reached = true;
      break;
    }
    if (result.expansions >= parameters_.max_expansions) {
      result.expansion_budget_reached = true;
      break;
    }

    const NodeId source = frontier.front();
    frontier.pop_front();
    ++result.expansions;
    const Point2D source_point = toPoint2D(result.nodes[source].point);
    const double phase =
      std::fmod(
      static_cast<double>(result.nodes[source].wavefront_depth) * kGoldenAngle,
      2.0 * kPi);

    for (std::size_t sample = 0; sample < parameters_.num_expansion_samples; ++sample) {
      if (update_build_time() >= static_cast<double>(parameters_.max_build_time_ms)) {
        result.build_time_budget_reached = true;
        stop_for_time_budget = true;
        break;
      }
      const double angle =
        phase + 2.0 * kPi * static_cast<double>(sample) /
        static_cast<double>(parameters_.num_expansion_samples);
      const Point2D candidate{
        source_point.x + parameters_.node_sampling_distance_m * std::cos(angle),
        source_point.y + parameters_.node_sampling_distance_m * std::sin(angle)};

      const NodeEvaluation node_evaluation = evaluate_node(candidate);
      if (!node_evaluation.valid) {
        reject(
          source, candidate, RejectedSampleKind::kNodeInvalid,
          node_evaluation.reason);
        continue;
      }
      if (!hasFiniteValidNodeEvaluation(node_evaluation)) {
        reject(
          source, candidate, RejectedSampleKind::kNonFiniteEvaluation,
          node_evaluation.reason);
        continue;
      }

      NodeId merge_target = std::numeric_limits<NodeId>::max();
      double merge_distance_squared =
        parameters_.merge_radius_m * parameters_.merge_radius_m;
      for (const auto & existing : result.nodes) {
        if (existing.id == source || existing.role == GraphNodeRole::kGoal) {
          continue;
        }
        const double distance_squared =
          planarDistanceSquared(candidate, toPoint2D(existing.point));
        if (
          distance_squared < merge_distance_squared - kComparisonTolerance ||
          (std::abs(distance_squared - merge_distance_squared) <=
          kComparisonTolerance && existing.id < merge_target))
        {
          merge_target = existing.id;
          merge_distance_squared = distance_squared;
        }
      }

      if (merge_target != std::numeric_limits<NodeId>::max()) {
        const Point2D merge_point = toPoint2D(result.nodes[merge_target].point);
        const EdgeEvaluation edge_evaluation =
          evaluate_edge(source_point, merge_point);
        if (!edge_evaluation.valid) {
          reject(
            source, merge_point, RejectedSampleKind::kMergeEdgeInvalid,
            edge_evaluation.reason);
          continue;
        }
        if (!hasFiniteValidEdgeEvaluation(edge_evaluation)) {
          reject(
            source, merge_point, RejectedSampleKind::kNonFiniteEvaluation,
            edge_evaluation.reason);
          continue;
        }
        add_edge(source, merge_target, edge_evaluation, false, true);
        try_goal_connection(merge_target);
        if (parameters_.stop_when_goal_connected && result.goal_connections > 0U) {
          stop_for_goal_connection = true;
          break;
        }
        continue;
      }

      if (result.nodes.size() >= parameters_.max_nodes) {
        result.node_budget_reached = true;
        stop_for_node_budget = true;
        break;
      }

      const EdgeEvaluation edge_evaluation =
        evaluate_edge(source_point, candidate);
      if (!edge_evaluation.valid) {
        reject(
          source, candidate, RejectedSampleKind::kExpansionEdgeInvalid,
          edge_evaluation.reason);
        continue;
      }
      if (!hasFiniteValidEdgeEvaluation(edge_evaluation)) {
        reject(
          source, candidate, RejectedSampleKind::kNonFiniteEvaluation,
          edge_evaluation.reason);
        continue;
      }

      const NodeId new_id = result.nodes.size();
      result.nodes.push_back(
        GraphNode{
          new_id, toTerrainPoint(candidate, node_evaluation), node_evaluation,
          GraphNodeRole::kSampled, result.nodes[source].wavefront_depth + 1U});
      add_edge(source, new_id, edge_evaluation, false, false);

      const Point2D new_point = toPoint2D(result.nodes[new_id].point);
      const double neighbor_radius_squared =
        parameters_.neighbor_connection_radius_m *
        parameters_.neighbor_connection_radius_m;
      for (NodeId neighbor_id = 0U; neighbor_id < new_id; ++neighbor_id) {
        if (update_build_time() >= static_cast<double>(parameters_.max_build_time_ms)) {
          result.build_time_budget_reached = true;
          stop_for_time_budget = true;
          break;
        }
        if (neighbor_id == source || neighbor_id == goal_id) {
          continue;
        }
        const Point2D neighbor_point = toPoint2D(result.nodes[neighbor_id].point);
        if (
          planarDistanceSquared(new_point, neighbor_point) >
          neighbor_radius_squared + kComparisonTolerance)
        {
          continue;
        }
        const EdgeEvaluation neighbor_evaluation =
          evaluate_edge(new_point, neighbor_point);
        if (!neighbor_evaluation.valid) {
          reject(
            new_id, neighbor_point, RejectedSampleKind::kMergeEdgeInvalid,
            neighbor_evaluation.reason);
          continue;
        }
        if (!hasFiniteValidEdgeEvaluation(neighbor_evaluation)) {
          reject(
            new_id, neighbor_point, RejectedSampleKind::kNonFiniteEvaluation,
            neighbor_evaluation.reason);
          continue;
        }
        add_edge(new_id, neighbor_id, neighbor_evaluation, false, true);
      }

      frontier.push_back(new_id);
      if (stop_for_time_budget) {
        break;
      }
      try_goal_connection(new_id);
      if (parameters_.stop_when_goal_connected && result.goal_connections > 0U) {
        stop_for_goal_connection = true;
        break;
      }

      if (result.nodes.size() >= parameters_.max_nodes) {
        result.node_budget_reached = true;
        stop_for_node_budget = true;
        break;
      }
    }

    if (stop_for_node_budget || stop_for_time_budget || stop_for_goal_connection) {
      break;
    }
  }

  update_build_time();
  if (stop_for_goal_connection) {
    result.stopped_on_goal_connection = true;
    result.termination = WavefrontTermination::kGoalConnected;
  } else if (result.node_budget_reached) {
    result.termination = WavefrontTermination::kMaxNodesReached;
  } else if (result.build_time_budget_reached) {
    result.termination = WavefrontTermination::kMaxBuildTimeReached;
  } else if (result.expansion_budget_reached) {
    result.termination = WavefrontTermination::kMaxExpansionsReached;
  } else {
    result.frontier_exhausted = frontier.empty();
    result.termination = WavefrontTermination::kFrontierExhausted;
  }

  result.path_node_ids = shortestPathAStar(
    result.nodes, result.edges, start_id, goal_id,
    parameters_.risk_weights.distance);
  result.success = !result.path_node_ids.empty();
  result.path.reserve(result.path_node_ids.size());
  for (const NodeId id : result.path_node_ids) {
    result.path.push_back(result.nodes[id].point);
  }

  std::ostringstream message;
  if (result.success) {
    message << "path found";
  } else {
    message << "no graph path to goal";
  }
  message << "; " << terminationName(result.termination)
          << "; nodes=" << result.nodes.size()
          << "; edges=" << result.edges.size()
          << "; expansions=" << result.expansions;
  result.message = message.str();
  return result;
}

}  // namespace rubi_heightmap_wavefront_planner
