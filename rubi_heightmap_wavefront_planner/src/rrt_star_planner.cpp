#include "rubi_heightmap_wavefront_planner/rrt_star_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rubi_heightmap_wavefront_planner
{
namespace
{

constexpr double kTolerance = 1.0e-12;
constexpr NodeId kNoParent = std::numeric_limits<NodeId>::max();

bool finite(const double value)
{
  return std::isfinite(value);
}

bool finite(const Point2D & point)
{
  return finite(point.x) && finite(point.y);
}

double distanceSquared(const Point2D & lhs, const Point2D & rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  return dx * dx + dy * dy;
}

double distance(const Point2D & lhs, const Point2D & rhs)
{
  return std::sqrt(distanceSquared(lhs, rhs));
}

Point2D point2D(const TerrainPoint & point)
{
  return Point2D{point.x, point.y};
}

TerrainPoint terrainPoint(
  const Point2D & point, const NodeEvaluation & evaluation)
{
  return TerrainPoint{point.x, point.y, evaluation.elevation_m};
}

bool finiteValidNode(const NodeEvaluation & evaluation)
{
  return evaluation.valid && finite(evaluation.elevation_m);
}

bool finiteValidEdge(const EdgeEvaluation & evaluation)
{
  return
    evaluation.valid &&
    finite(evaluation.length_3d_m) &&
    finite(evaluation.max_slope_deg) &&
    finite(evaluation.max_step_m) &&
    evaluation.length_3d_m >= 0.0 &&
    evaluation.max_slope_deg >= 0.0 &&
    evaluation.max_step_m >= 0.0;
}

double edgeCost(
  const EdgeEvaluation & evaluation, const RrtStarParameters & parameters)
{
  const double normalized_slope = std::clamp(
    evaluation.max_slope_deg / parameters.slope_normalization_deg, 0.0, 1.0);
  return
    evaluation.length_3d_m *
    (parameters.risk_weights.distance +
    parameters.risk_weights.slope * normalized_slope) +
    parameters.risk_weights.step * evaluation.max_step_m;
}

bool finiteValidBounds(const SamplingBounds & bounds)
{
  return
    finite(bounds.min_x_m) && finite(bounds.max_x_m) &&
    finite(bounds.min_y_m) && finite(bounds.max_y_m) &&
    bounds.min_x_m < bounds.max_x_m &&
    bounds.min_y_m < bounds.max_y_m;
}

bool contains(const SamplingBounds & bounds, const Point2D & point)
{
  return
    point.x >= bounds.min_x_m - kTolerance &&
    point.x <= bounds.max_x_m + kTolerance &&
    point.y >= bounds.min_y_m - kTolerance &&
    point.y <= bounds.max_y_m + kTolerance;
}

void validateParameters(const RrtStarParameters & parameters)
{
  const auto positive = [](const double value, const char * name) {
      if (!finite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and > 0");
      }
    };
  const auto nonnegative = [](const double value, const char * name) {
      if (!finite(value) || value < 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and >= 0");
      }
    };

  if (parameters.max_iterations == 0U) {
    throw std::invalid_argument("max_iterations must be > 0");
  }
  if (!finite(parameters.goal_bias) || parameters.goal_bias < 0.0 ||
    parameters.goal_bias > 1.0)
  {
    throw std::invalid_argument("goal_bias must be finite and in [0, 1]");
  }
  positive(parameters.steer_distance_m, "steer_distance_m");
  positive(parameters.rewire_radius_min_m, "rewire_radius_min_m");
  positive(parameters.rewire_radius_max_m, "rewire_radius_max_m");
  if (parameters.rewire_radius_min_m > parameters.rewire_radius_max_m) {
    throw std::invalid_argument(
            "rewire_radius_min_m must be <= rewire_radius_max_m");
  }
  positive(
    parameters.goal_connection_distance_m, "goal_connection_distance_m");
  if (parameters.max_nodes < 2U) {
    throw std::invalid_argument("max_nodes must reserve start and goal");
  }
  positive(parameters.slope_normalization_deg, "slope_normalization_deg");
  nonnegative(parameters.risk_weights.distance, "risk_weights.distance");
  nonnegative(parameters.risk_weights.slope, "risk_weights.slope");
  nonnegative(parameters.risk_weights.step, "risk_weights.step");
  if (
    parameters.risk_weights.distance == 0.0 &&
    parameters.risk_weights.slope == 0.0 &&
    parameters.risk_weights.step == 0.0)
  {
    throw std::invalid_argument("at least one risk weight must be > 0");
  }
}

Point2D steer(
  const Point2D & from, const Point2D & target, const double maximum_distance)
{
  const double target_distance = distance(from, target);
  if (target_distance <= maximum_distance) {
    return target;
  }
  const double scale = maximum_distance / target_distance;
  return Point2D{
    from.x + scale * (target.x - from.x),
    from.y + scale * (target.y - from.y)};
}

double adaptiveRewireRadius(
  const std::size_t tree_node_count, const RrtStarParameters & parameters)
{
  const double n = static_cast<double>(std::max<std::size_t>(2U, tree_node_count));
  const double asymptotic =
    parameters.rewire_radius_max_m * std::sqrt(std::log(n) / n);
  return std::clamp(
    asymptotic,
    parameters.rewire_radius_min_m,
    parameters.rewire_radius_max_m);
}

bool isAncestor(
  const NodeId possible_ancestor,
  NodeId node,
  const std::vector<NodeId> & parents)
{
  while (node != kNoParent) {
    if (node == possible_ancestor) {
      return true;
    }
    node = parents[node];
  }
  return false;
}

void removeChild(
  std::vector<NodeId> & children, const NodeId child)
{
  children.erase(std::remove(children.begin(), children.end(), child), children.end());
}

void propagateDescendantCosts(
  const NodeId root,
  const std::vector<std::vector<NodeId>> & children,
  const std::vector<double> & incoming_cost,
  std::vector<double> & cost_from_start)
{
  std::vector<NodeId> stack{root};
  while (!stack.empty()) {
    const NodeId parent = stack.back();
    stack.pop_back();
    for (const NodeId child : children[parent]) {
      cost_from_start[child] = cost_from_start[parent] + incoming_cost[child];
      stack.push_back(child);
    }
  }
}

const char * terminationDescription(const WavefrontTermination termination)
{
  switch (termination) {
    case WavefrontTermination::kInvalidRequest:
      return "invalid request";
    case WavefrontTermination::kGoalConnected:
      return "stopped on first RRT* solution";
    case WavefrontTermination::kFrontierExhausted:
      return "RRT* sampling ended";
    case WavefrontTermination::kMaxNodesReached:
      return "maximum RRT* node budget reached";
    case WavefrontTermination::kMaxExpansionsReached:
      return "maximum RRT* iteration budget reached";
    case WavefrontTermination::kMaxBuildTimeReached:
      return "maximum RRT* planning time reached";
  }
  return "unknown RRT* termination";
}

}  // namespace

RrtStarPlanner::RrtStarPlanner(RrtStarParameters parameters)
: parameters_(std::move(parameters))
{
  validateParameters(parameters_);
}

const RrtStarParameters & RrtStarPlanner::parameters() const noexcept
{
  return parameters_;
}

PlanResult RrtStarPlanner::plan(
  const TerrainEvaluator & terrain,
  const Point2D & start,
  const Point2D & goal) const
{
  const TerrainSnapshot & snapshot = terrain.snapshot();
  const double half_cell = 0.5 * snapshot.resolution();
  const double max_x_exclusive =
    snapshot.maxXCenter() + half_cell;
  const double max_y_exclusive =
    snapshot.maxYCenter() + half_cell;
  return plan(
    start, goal,
    SamplingBounds{
      snapshot.minXCenter() - half_cell,
      std::nextafter(max_x_exclusive, snapshot.maxXCenter()),
      snapshot.minYCenter() - half_cell,
      std::nextafter(max_y_exclusive, snapshot.maxYCenter())},
    [&terrain](const Point2D & point) {
      return terrain.evaluateNode(point);
    },
    [&terrain](const Point2D & from, const Point2D & to) {
      return terrain.evaluateEdge(from, to);
    });
}

PlanResult RrtStarPlanner::plan(
  const Point2D & start,
  const Point2D & goal,
  const SamplingBounds & bounds,
  const NodeEvaluator & evaluate_node,
  const EdgeEvaluator & evaluate_edge) const
{
  PlanResult result;
  const auto planning_start = std::chrono::steady_clock::now();
  const auto update_time = [&]() {
      result.build_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - planning_start).count();
      return result.build_time_ms;
    };
  const auto time_reached = [&]() {
      return
        parameters_.max_planning_time_ms > 0U &&
        update_time() >= static_cast<double>(parameters_.max_planning_time_ms);
    };

  if (!evaluate_node || !evaluate_edge) {
    result.message = "node and edge evaluators are required";
    return result;
  }
  if (
    !finite(start) || !finite(goal) || !finiteValidBounds(bounds) ||
    !contains(bounds, start) || !contains(bounds, goal))
  {
    result.message = "finite bounds must contain finite start and goal coordinates";
    return result;
  }

  const NodeEvaluation start_evaluation = evaluate_node(start);
  const NodeEvaluation goal_evaluation = evaluate_node(goal);
  if (!finiteValidNode(start_evaluation) || !finiteValidNode(goal_evaluation)) {
    const bool start_invalid = !finiteValidNode(start_evaluation);
    const NodeEvaluation & evaluation =
      start_invalid ? start_evaluation : goal_evaluation;
    const Point2D rejected_point = start_invalid ? start : goal;
    result.rejected.push_back(
      RejectedSample{
        0U, rejected_point,
        evaluation.valid ?
        RejectedSampleKind::kNonFiniteEvaluation :
        RejectedSampleKind::kNodeInvalid,
        evaluation.reason});
    if (evaluation.valid) {
      ++result.reject_counts.non_finite_evaluation;
    } else {
      ++result.reject_counts.node_invalid;
    }
    result.message = start_invalid ?
      "start node is invalid" : "goal node is invalid";
    update_time();
    return result;
  }

  constexpr NodeId start_id = 0U;
  constexpr NodeId goal_id = 1U;
  result.nodes.push_back(
    GraphNode{
      start_id, terrainPoint(start, start_evaluation), start_evaluation,
      GraphNodeRole::kStart, 0U});
  result.nodes.push_back(
    GraphNode{
      goal_id, terrainPoint(goal, goal_evaluation), goal_evaluation,
      GraphNodeRole::kGoal, 0U});

  std::vector<NodeId> tree_nodes{start_id};
  std::vector<NodeId> parent(2U, kNoParent);
  std::vector<std::vector<NodeId>> children(2U);
  std::vector<EdgeEvaluation> incoming_evaluation(2U);
  std::vector<double> incoming_cost(
    2U, std::numeric_limits<double>::infinity());
  std::vector<double> cost_from_start(
    2U, std::numeric_limits<double>::infinity());
  cost_from_start[start_id] = 0.0;

  struct GoalConnection
  {
    EdgeEvaluation evaluation{};
    double cost{std::numeric_limits<double>::infinity()};
  };
  std::vector<std::optional<GoalConnection>> goal_connections(2U);

  const auto reject = [&](
      const NodeId source, const Point2D & candidate,
      const RejectedSampleKind kind, const TerrainInvalidReason reason) {
      result.rejected.push_back(RejectedSample{source, candidate, kind, reason});
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

  const auto store_goal_connection = [&](
      const NodeId source, const EdgeEvaluation & evaluation) {
      if (!finiteValidEdge(evaluation)) {
        reject(
          source, goal, RejectedSampleKind::kNonFiniteEvaluation,
          evaluation.reason);
        return false;
      }
      const double cost = edgeCost(evaluation, parameters_);
      if (!finite(cost) || cost < 0.0) {
        reject(
          source, goal, RejectedSampleKind::kNonFiniteEvaluation,
          evaluation.reason);
        return false;
      }
      if (!goal_connections[source]) {
        ++result.goal_connections;
      }
      if (
        !goal_connections[source] ||
        cost < goal_connections[source]->cost - kTolerance)
      {
        goal_connections[source] = GoalConnection{evaluation, cost};
      }
      return true;
    };

  const auto try_goal_connection = [&](const NodeId source) {
      const Point2D source_point = point2D(result.nodes[source].point);
      if (
        distance(source_point, goal) >
        parameters_.goal_connection_distance_m + kTolerance)
      {
        return false;
      }
      const EdgeEvaluation evaluation = evaluate_edge(source_point, goal);
      if (!evaluation.valid) {
        reject(
          source, goal, RejectedSampleKind::kGoalEdgeInvalid,
          evaluation.reason);
        return false;
      }
      return store_goal_connection(source, evaluation);
    };

  bool stop_on_solution = false;
  if (distance(start, goal) <= kTolerance) {
    result.goal_connections = 1U;
    result.path_node_ids = {start_id};
    result.path = {result.nodes[start_id].point};
    result.success = true;
    result.termination = WavefrontTermination::kGoalConnected;
    result.stopped_on_goal_connection = true;
    result.message = "start already equals goal";
    update_time();
    return result;
  }
  if (try_goal_connection(start_id) && parameters_.stop_on_first_solution) {
    stop_on_solution = true;
  }

  std::mt19937_64 random_engine(parameters_.random_seed);
  std::uniform_real_distribution<double> sample_x(bounds.min_x_m, bounds.max_x_m);
  std::uniform_real_distribution<double> sample_y(bounds.min_y_m, bounds.max_y_m);
  std::uniform_real_distribution<double> unit_random(0.0, 1.0);

  bool node_budget_reached = false;
  bool time_budget_reached = false;
  while (
    !stop_on_solution &&
    result.expansions < parameters_.max_iterations)
  {
    if (result.nodes.size() >= parameters_.max_nodes) {
      node_budget_reached = true;
      break;
    }
    if (time_reached()) {
      time_budget_reached = true;
      break;
    }
    ++result.expansions;

    const bool sample_goal = unit_random(random_engine) < parameters_.goal_bias;
    const Point2D sample = sample_goal ?
      goal : Point2D{sample_x(random_engine), sample_y(random_engine)};

    NodeId nearest_id = tree_nodes.front();
    double nearest_distance_squared =
      distanceSquared(point2D(result.nodes[nearest_id].point), sample);
    for (const NodeId id : tree_nodes) {
      const double candidate_distance_squared =
        distanceSquared(point2D(result.nodes[id].point), sample);
      if (
        candidate_distance_squared < nearest_distance_squared - kTolerance ||
        (std::abs(candidate_distance_squared - nearest_distance_squared) <=
        kTolerance && id < nearest_id))
      {
        nearest_id = id;
        nearest_distance_squared = candidate_distance_squared;
      }
    }

    const Point2D candidate = steer(
      point2D(result.nodes[nearest_id].point), sample,
      parameters_.steer_distance_m);
    if (
      distanceSquared(candidate, point2D(result.nodes[nearest_id].point)) <=
      kTolerance * kTolerance)
    {
      reject(
        nearest_id, candidate, RejectedSampleKind::kDuplicateEdge,
        TerrainInvalidReason::kNone);
      continue;
    }

    const bool candidate_is_goal = distanceSquared(candidate, goal) <=
      kTolerance * kTolerance;
    const NodeEvaluation node_evaluation =
      candidate_is_goal ? goal_evaluation : evaluate_node(candidate);
    if (!node_evaluation.valid) {
      reject(
        nearest_id, candidate, RejectedSampleKind::kNodeInvalid,
        node_evaluation.reason);
      continue;
    }
    if (!finiteValidNode(node_evaluation)) {
      reject(
        nearest_id, candidate, RejectedSampleKind::kNonFiniteEvaluation,
        node_evaluation.reason);
      continue;
    }
    if (time_reached()) {
      time_budget_reached = true;
      break;
    }

    const double near_radius =
      adaptiveRewireRadius(tree_nodes.size(), parameters_);
    const double near_radius_squared = near_radius * near_radius;
    std::vector<NodeId> near_ids;
    near_ids.reserve(tree_nodes.size());
    for (const NodeId id : tree_nodes) {
      if (
        distanceSquared(point2D(result.nodes[id].point), candidate) <=
        near_radius_squared + kTolerance)
      {
        near_ids.push_back(id);
      }
    }
    if (std::find(near_ids.begin(), near_ids.end(), nearest_id) == near_ids.end()) {
      near_ids.push_back(nearest_id);
    }
    std::sort(near_ids.begin(), near_ids.end());

    NodeId best_parent = kNoParent;
    double best_total_cost = std::numeric_limits<double>::infinity();
    double best_edge_cost = std::numeric_limits<double>::infinity();
    EdgeEvaluation best_edge_evaluation;
    for (const NodeId parent_id : near_ids) {
      if (time_reached()) {
        time_budget_reached = true;
        break;
      }
      const EdgeEvaluation evaluation =
        evaluate_edge(point2D(result.nodes[parent_id].point), candidate);
      if (!evaluation.valid) {
        reject(
          parent_id, candidate,
          parent_id == nearest_id ?
          RejectedSampleKind::kExpansionEdgeInvalid :
          RejectedSampleKind::kMergeEdgeInvalid,
          evaluation.reason);
        continue;
      }
      if (!finiteValidEdge(evaluation)) {
        reject(
          parent_id, candidate, RejectedSampleKind::kNonFiniteEvaluation,
          evaluation.reason);
        continue;
      }
      const double candidate_edge_cost = edgeCost(evaluation, parameters_);
      const double candidate_total =
        cost_from_start[parent_id] + candidate_edge_cost;
      if (
        finite(candidate_edge_cost) && candidate_edge_cost >= 0.0 &&
        (candidate_total < best_total_cost - kTolerance ||
        (std::abs(candidate_total - best_total_cost) <= kTolerance &&
        parent_id < best_parent)))
      {
        best_parent = parent_id;
        best_total_cost = candidate_total;
        best_edge_cost = candidate_edge_cost;
        best_edge_evaluation = evaluation;
      }
    }
    if (time_budget_reached) {
      break;
    }
    if (best_parent == kNoParent) {
      continue;
    }

    // A goal-biased extension that reaches the exact goal uses its selected
    // parent edge directly; it does not add a duplicate sampled goal node.
    if (candidate_is_goal) {
      if (store_goal_connection(best_parent, best_edge_evaluation) &&
        parameters_.stop_on_first_solution)
      {
        stop_on_solution = true;
      }
      continue;
    }

    const NodeId new_id = result.nodes.size();
    result.nodes.push_back(
      GraphNode{
        new_id, terrainPoint(candidate, node_evaluation), node_evaluation,
        GraphNodeRole::kSampled, 0U});
    tree_nodes.push_back(new_id);
    parent.push_back(best_parent);
    children.emplace_back();
    incoming_evaluation.push_back(best_edge_evaluation);
    incoming_cost.push_back(best_edge_cost);
    cost_from_start.push_back(best_total_cost);
    goal_connections.emplace_back(std::nullopt);
    children[best_parent].push_back(new_id);

    for (const NodeId neighbor_id : near_ids) {
      if (time_reached()) {
        time_budget_reached = true;
        break;
      }
      if (
        neighbor_id == start_id || neighbor_id == best_parent ||
        isAncestor(neighbor_id, new_id, parent))
      {
        continue;
      }
      const EdgeEvaluation evaluation =
        evaluate_edge(candidate, point2D(result.nodes[neighbor_id].point));
      if (!evaluation.valid) {
        reject(
          new_id, point2D(result.nodes[neighbor_id].point),
          RejectedSampleKind::kMergeEdgeInvalid, evaluation.reason);
        continue;
      }
      if (!finiteValidEdge(evaluation)) {
        reject(
          new_id, point2D(result.nodes[neighbor_id].point),
          RejectedSampleKind::kNonFiniteEvaluation, evaluation.reason);
        continue;
      }
      const double candidate_edge_cost = edgeCost(evaluation, parameters_);
      const double rewired_cost = cost_from_start[new_id] + candidate_edge_cost;
      if (
        !finite(candidate_edge_cost) || candidate_edge_cost < 0.0 ||
        rewired_cost + kTolerance >= cost_from_start[neighbor_id])
      {
        continue;
      }

      removeChild(children[parent[neighbor_id]], neighbor_id);
      parent[neighbor_id] = new_id;
      children[new_id].push_back(neighbor_id);
      incoming_evaluation[neighbor_id] = evaluation;
      incoming_cost[neighbor_id] = candidate_edge_cost;
      cost_from_start[neighbor_id] = rewired_cost;
      propagateDescendantCosts(
        neighbor_id, children, incoming_cost, cost_from_start);
      ++result.rewires;
    }

    if (time_budget_reached) {
      break;
    }
    if (try_goal_connection(new_id) && parameters_.stop_on_first_solution) {
      stop_on_solution = true;
    }
    if (time_reached()) {
      time_budget_reached = true;
      break;
    }
  }

  result.node_budget_reached = node_budget_reached;
  result.expansion_budget_reached =
    !node_budget_reached && !time_budget_reached && !stop_on_solution &&
    result.expansions >= parameters_.max_iterations;
  result.build_time_budget_reached = time_budget_reached;
  result.stopped_on_goal_connection = stop_on_solution;
  if (stop_on_solution) {
    result.termination = WavefrontTermination::kGoalConnected;
  } else if (node_budget_reached) {
    result.termination = WavefrontTermination::kMaxNodesReached;
  } else if (time_budget_reached) {
    result.termination = WavefrontTermination::kMaxBuildTimeReached;
  } else {
    result.termination = WavefrontTermination::kMaxExpansionsReached;
  }

  NodeId best_goal_parent = kNoParent;
  double best_goal_total = std::numeric_limits<double>::infinity();
  for (const NodeId id : tree_nodes) {
    if (!goal_connections[id]) {
      continue;
    }
    const double total = cost_from_start[id] + goal_connections[id]->cost;
    if (
      total < best_goal_total - kTolerance ||
      (std::abs(total - best_goal_total) <= kTolerance && id < best_goal_parent))
    {
      best_goal_parent = id;
      best_goal_total = total;
    }
  }

  result.edges.reserve(tree_nodes.size());
  for (const NodeId id : tree_nodes) {
    if (id == start_id) {
      continue;
    }
    result.edges.push_back(
      GraphEdge{
        parent[id], id, incoming_evaluation[id], incoming_cost[id], false, false});
  }

  if (best_goal_parent != kNoParent) {
    parent[goal_id] = best_goal_parent;
    cost_from_start[goal_id] = best_goal_total;
    result.edges.push_back(
      GraphEdge{
        best_goal_parent, goal_id,
        goal_connections[best_goal_parent]->evaluation,
        goal_connections[best_goal_parent]->cost, true, false});

    std::vector<NodeId> reverse_path{goal_id};
    for (NodeId id = best_goal_parent; id != kNoParent; id = parent[id]) {
      reverse_path.push_back(id);
      if (id == start_id) {
        break;
      }
    }
    if (reverse_path.back() == start_id) {
      result.path_node_ids.assign(reverse_path.rbegin(), reverse_path.rend());
      result.path.reserve(result.path_node_ids.size());
      for (const NodeId id : result.path_node_ids) {
        result.path.push_back(result.nodes[id].point);
      }
      result.success = true;
    }
  }

  update_time();
  std::ostringstream message;
  message << (result.success ? "RRT* path found" : "RRT* found no path")
          << "; " << terminationDescription(result.termination)
          << "; nodes=" << result.nodes.size()
          << "; tree_edges=" << result.edges.size()
          << "; iterations=" << result.expansions
          << "; rewires=" << result.rewires
          << "; goal_connections=" << result.goal_connections;
  result.message = message.str();
  return result;
}

}  // namespace rubi_heightmap_wavefront_planner
