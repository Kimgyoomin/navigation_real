#include "rubi_heightmap_step_wavefront_planner/graph/wavefront_graph_builder.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <random>
#include <stdexcept>
#include <unordered_set>

#include "rubi_heightmap_step_wavefront_planner/graph/spatial_index_2d.hpp"
#include "rubi_heightmap_step_wavefront_planner/terrain/height_evidence.hpp"

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
    parameters_.max_expansions == 0U || parameters_.max_sampling_trials_per_expansion == 0U ||
    !std::isfinite(parameters_.trg_expand_distance_m) || parameters_.trg_expand_distance_m <= 0.0 ||
    !std::isfinite(parameters_.trg_robot_size_m) || parameters_.trg_robot_size_m <= 0.0 ||
    parameters_.trg_sample_num == 0U || parameters_.trg_max_trial_samples == 0U ||
    !std::isfinite(parameters_.trg_height_threshold_m) ||
    parameters_.trg_height_threshold_m < 0.0 ||
    !std::isfinite(parameters_.trg_collision_threshold) ||
    parameters_.trg_collision_threshold < 0.0 || parameters_.trg_collision_threshold > 1.0 ||
    !std::isfinite(parameters_.trg_neighbor_connection_radius_m) ||
    parameters_.trg_neighbor_connection_radius_m <= 0.0)
  {throw std::invalid_argument("invalid WavefrontGraphParameters");}
}

GraphBuildResult WavefrontGraphBuilder::build(
  const StepEvaluator & evaluator, const Point2D start, const Point2D goal) const
{
  if (parameters_.sampling_policy == SamplingPolicy::kOriginalTrgRandomRing) {
    return buildOriginalTrg(evaluator, start, goal);
  }
  const auto build_start = Clock::now();
  GraphBuildResult result;
  const NodeEvaluation start_evaluation = evaluator.evaluateNode(start);
  const NodeEvaluation goal_evaluation = evaluator.evaluateNode(goal);
  result.node_evaluation_calls += 2U;
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
  std::mt19937 random_engine(parameters_.random_seed);
  std::uniform_real_distribution<double> random_angle(0.0, 2.0 * std::acos(-1.0));
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
    ++result.edge_evaluation_calls;
    if (!edge.valid) {
      ++result.rejected_edges;
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
    std::size_t sampling_trials = 0U;
    std::size_t accepted_candidates = 0U;
    const std::size_t trial_limit = parameters_.sampling_policy == SamplingPolicy::kTrgRandomRing ?
      parameters_.max_sampling_trials_per_expansion : parameters_.samples_per_expansion;
    while (sampling_trials < trial_limit &&
      (parameters_.sampling_policy == SamplingPolicy::kDeterministicRing ||
      accepted_candidates < parameters_.samples_per_expansion))
    {
      const double yaw_rad = parameters_.sampling_policy == SamplingPolicy::kTrgRandomRing ?
        random_angle(random_engine) : 2.0 * std::acos(-1.0) * sampling_trials /
        parameters_.samples_per_expansion;
      ++sampling_trials;
      ++result.sampling_trials;
      ++result.candidate_generated;
      const Point2D proposal{result.graph.nodes[source].point.x +
        parameters_.node_sampling_distance_m * std::cos(yaw_rad),
        result.graph.nodes[source].point.y +
        parameters_.node_sampling_distance_m * std::sin(yaw_rad)};
      const NodeEvaluation node_evaluation = evaluator.evaluateNode(proposal);
      ++result.node_evaluation_calls;
      if (!node_evaluation.valid) {
        ++result.candidate_rejected;
        result.rejected.push_back({RejectionKind::kNode, node_evaluation.reason,
          result.graph.nodes[source].point, proposal}); continue;
      }
      ++accepted_candidates;
      ++result.candidate_valid;
      bool merged = false;
      ++result.merge_queries;
      for (const NodeId target : index.radiusSearch(proposal, parameters_.merge_radius_m)) {
        if (target == source) {continue;}
        const EdgeEvaluation edge = evaluator.evaluateEdge(
          result.graph.nodes[source].point, result.graph.nodes[target].point);
        ++result.edge_evaluation_calls;
        if (edge.valid) {add_edge(source, target, edge); merged = true; break;}
        ++result.rejected_edges;
        result.rejected.push_back({RejectionKind::kEdge, edge.reason,
          result.graph.nodes[source].point, result.graph.nodes[target].point});
      }
      if (merged) {continue;}
      if (result.graph.nodes.size() >= parameters_.max_nodes) {
        result.termination = PlanTermination::kMaxNodes; break;
      }
      const EdgeEvaluation parent_edge = evaluator.evaluateEdge(
        result.graph.nodes[source].point, proposal);
      ++result.edge_evaluation_calls;
      if (!parent_edge.valid) {
        ++result.candidate_rejected;
        ++result.rejected_edges;
        result.rejected.push_back({RejectionKind::kEdge, parent_edge.reason,
          result.graph.nodes[source].point, proposal}); continue;
      }
      const NodeId new_id = result.graph.nodes.size();
      result.graph.nodes.push_back({new_id, proposal, node_evaluation.elevation_m});
      add_edge(source, new_id, parent_edge);
      frontier.push(new_id);
      ++result.neighbor_radius_queries;
      for (const NodeId neighbor : index.radiusSearch(
          proposal, parameters_.neighbor_connection_radius_m))
      {
        if (neighbor == source) {continue;}
        const EdgeEvaluation edge = evaluator.evaluateEdge(
          proposal, result.graph.nodes[neighbor].point);
        ++result.edge_evaluation_calls;
        if (edge.valid) {add_edge(new_id, neighbor, edge);} else {
          ++result.rejected_edges;
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

GraphBuildResult WavefrontGraphBuilder::buildOriginalTrg(
  const StepEvaluator & evaluator, const Point2D start, const Point2D goal) const
{
  const auto build_start = Clock::now();
  GraphBuildResult result;
  const CostmapSnapshot * costmap = evaluator.costmap();
  if (!costmap || evaluator.mode() != StepEvaluationMode::kCostmapHeightHybrid) {
    result.message = "Original TRG construction requires hybrid Costmap/Height evaluator";
    result.build_time_ms = milliseconds(Clock::now() - build_start);
    return result;
  }

  const auto evaluate_candidate = [&](const Point2D point) {
      NodeEvaluation node;
      ++result.node_evaluation_calls;
      const auto cell = costmap->worldToCell(point);
      if (!cell || !costmap->inBounds(*cell)) {
        node.reason = StepInvalidReason::kCostmapOutOfBounds;
        ++result.costmap_rejects;
        return node;
      }
      const std::uint8_t raw = *costmap->cost(*cell);
      node.raw_cost = raw;
      if (raw == 255U) {
        node.reason = StepInvalidReason::kCostmapUnknown;
        ++result.costmap_rejects;
        return node;
      }
      if (raw >= 253U) {
        node.reason = StepInvalidReason::kCostmapCollision;
        ++result.costmap_rejects;
        return node;
      }
      const HeightEvidence evidence = queryOriginalTrgHeightEvidence(
        evaluator.snapshot(), point, parameters_.trg_robot_size_m,
        parameters_.trg_height_threshold_m, parameters_.trg_collision_threshold);
      node.height_evidence_available = evidence.observed_cell_count > 0U;
      node.observed_support_ratio = evidence.valid ? 1.0 : 0.0;
      if (evidence.observed_cell_count == 0U) {
        node.reason = StepInvalidReason::kInsufficientHeightEvidence;
        ++result.trg_collision_rejects;
        return node;
      }
      if (!evidence.valid) {
        node.reason = StepInvalidReason::kTrgCollision;
        ++result.trg_collision_rejects;
        return node;
      }
      node.valid = true;
      node.reason = StepInvalidReason::kNone;
      node.elevation_m = evidence.nearest_elevation_m;
      return node;
    };

  const NodeEvaluation start_evaluation = evaluate_candidate(start);
  const NodeEvaluation goal_evaluation = evaluate_candidate(goal);
  if (!start_evaluation.valid || !goal_evaluation.valid) {
    result.message = !start_evaluation.valid ? "start is invalid" : "goal is invalid";
    result.rejected.push_back({RejectionKind::kNode,
      !start_evaluation.valid ? start_evaluation.reason : goal_evaluation.reason,
      !start_evaluation.valid ? start : goal, !start_evaluation.valid ? start : goal});
    result.build_time_ms = milliseconds(Clock::now() - build_start);
    return result;
  }

  result.graph.nodes.push_back({0U, start, start_evaluation.elevation_m, GraphNodeState::kValid});
  result.start_node_id = 0U;
  if (std::hypot(goal.x - start.x, goal.y - start.y) <= 1.0e-12) {
    result.goal_node_id = 0U;
    result.termination = PlanTermination::kPostGoalComplete;
    result.build_time_ms = milliseconds(Clock::now() - build_start);
    return result;
  }

  UniformGridSpatialIndex2D index(std::max(
    parameters_.trg_robot_size_m, parameters_.trg_neighbor_connection_radius_m));
  index.insert(0U, start);
  std::deque<NodeId> expand_queue;
  expand_queue.push_back(0U);
  std::unordered_set<std::uint64_t> edge_keys;
  std::mt19937 random_engine(parameters_.trg_randomize_seed ?
    std::random_device{}() : parameters_.trg_random_seed);
  std::uniform_real_distribution<double> random_angle(0.0, 2.0 * std::acos(-1.0));
  NodeId goal_id = std::numeric_limits<NodeId>::max();
  bool goal_connected = false;
  std::size_t post_goal_completed = 0U;

  const auto try_wire_edge = [&](const NodeId from, const NodeId to) {
      if (from == to || from >= result.graph.nodes.size() || to >= result.graph.nodes.size() ||
        result.graph.nodes[from].state == GraphNodeState::kInvalid ||
        result.graph.nodes[to].state == GraphNodeState::kInvalid ||
        edge_keys.count(edgeKey(from, to)) > 0U)
      {
        return false;
      }
      ++result.edge_evaluation_calls;
      const EdgeEvaluation edge = evaluator.evaluateEdge(
        result.graph.nodes[from].point, result.graph.nodes[to].point);
      if (!edge.valid) {
        ++result.rejected_edges;
        result.rejected.push_back({RejectionKind::kEdge, edge.reason,
          result.graph.nodes[from].point, result.graph.nodes[to].point});
        return false;
      }
      edge_keys.insert(edgeKey(from, to));
      result.graph.edges.push_back({from, to, edge});
      return true;
    };

  const auto nearest_accepted = [&](const Point2D query) -> std::optional<NodeId> {
      std::optional<NodeId> best;
      double best_squared = std::numeric_limits<double>::infinity();
      for (const auto & node : result.graph.nodes) {
        if (node.state == GraphNodeState::kInvalid || node.id == goal_id) {continue;}
        const double squared = std::pow(node.point.x - query.x, 2) +
          std::pow(node.point.y - query.y, 2);
        if (!best || squared < best_squared - 1.0e-12 ||
          (std::abs(squared - best_squared) <= 1.0e-12 && node.id < *best))
        {
          best = node.id;
          best_squared = squared;
        }
      }
      return best;
    };

  const auto connect_goal = [&](const NodeId source) {
      if (std::hypot(result.graph.nodes[source].point.x - goal.x,
        result.graph.nodes[source].point.y - goal.y) >
        parameters_.goal_connection_distance_m) {return;}
      if (!goal_connected) {
        goal_id = result.graph.nodes.size();
        result.graph.nodes.push_back(
          {goal_id, goal, goal_evaluation.elevation_m, GraphNodeState::kValid});
      }
      if (try_wire_edge(source, goal_id)) {
        goal_connected = true;
        result.goal_node_id = goal_id;
      } else if (!goal_connected) {
        result.graph.nodes.pop_back();
        goal_id = std::numeric_limits<NodeId>::max();
      }
    };
  connect_goal(0U);

  while (!expand_queue.empty()) {
    if (goal_connected && post_goal_completed >= parameters_.post_goal_expansions) {
      result.termination = PlanTermination::kPostGoalComplete;
      break;
    }
    if (result.expansion_count >= parameters_.max_expansions) {
      result.termination = PlanTermination::kMaxExpansions;
      break;
    }
    if (parameters_.max_graph_build_time_ms > 0U &&
      milliseconds(Clock::now() - build_start) >= parameters_.max_graph_build_time_ms)
    {
      result.termination = PlanTermination::kMaxGraphBuildTime;
      break;
    }
    const NodeId source = expand_queue.front();
    expand_queue.pop_front();
    if (source == goal_id || result.graph.nodes[source].state == GraphNodeState::kInvalid) {
      continue;
    }
    result.graph.nodes[source].state = GraphNodeState::kValid;
    ++result.expansion_count;

    std::vector<std::pair<Point2D, NodeEvaluation>> accepted_samples;
    std::size_t trial_samples = 0U;
    while (accepted_samples.size() < parameters_.trg_sample_num &&
      trial_samples < parameters_.trg_max_trial_samples)
    {
      ++trial_samples;
      ++result.sampling_trials;
      ++result.candidate_generated;
      const double angle = random_angle(random_engine);
      const Point2D sample{
        result.graph.nodes[source].point.x + parameters_.trg_expand_distance_m * std::cos(angle),
        result.graph.nodes[source].point.y + parameters_.trg_expand_distance_m * std::sin(angle)};
      const NodeEvaluation candidate = evaluate_candidate(sample);
      if (!candidate.valid) {
        ++result.candidate_rejected;
        result.rejected.push_back({RejectionKind::kNode, candidate.reason,
          result.graph.nodes[source].point, sample});
        continue;
      }
      ++result.candidate_valid;
      accepted_samples.emplace_back(sample, candidate);
    }

    for (const auto & [sample, candidate] : accepted_samples) {
      ++result.existing_node_queries;
      const auto existing = nearest_accepted(sample);
      if (existing && std::hypot(
          result.graph.nodes[*existing].point.x - sample.x,
          result.graph.nodes[*existing].point.y - sample.y) < parameters_.trg_robot_size_m)
      {
        ++result.existing_node_rewires;
        (void)try_wire_edge(source, *existing);
        continue;
      }
      if (result.graph.nodes.size() >= parameters_.max_nodes) {
        result.termination = PlanTermination::kMaxNodes;
        break;
      }

      const NodeId new_id = result.graph.nodes.size();
      result.graph.nodes.push_back(
        {new_id, sample, candidate.elevation_m, GraphNodeState::kFrontier});
      ++result.new_nodes_created;
      bool connected = try_wire_edge(source, new_id);
      ++result.neighbor_queries;
      for (const NodeId neighbor : index.radiusSearch(
          sample, parameters_.trg_neighbor_connection_radius_m))
      {
        if (neighbor == source || neighbor == new_id || neighbor == goal_id ||
          result.graph.nodes[neighbor].state == GraphNodeState::kInvalid) {continue;}
        ++result.neighbor_wire_attempts;
        connected = try_wire_edge(new_id, neighbor) || connected;
      }
      if (!connected) {
        result.graph.nodes[new_id].state = GraphNodeState::kInvalid;
        ++result.isolated_nodes;
        result.rejected.push_back({RejectionKind::kNode, StepInvalidReason::kIsolatedNode,
          result.graph.nodes[source].point, sample});
        continue;
      }
      index.insert(new_id, sample);
      expand_queue.push_back(new_id);
      connect_goal(new_id);
    }
    if (goal_connected) {++post_goal_completed;}
    if (result.termination == PlanTermination::kMaxNodes) {break;}
  }

  if (result.termination == PlanTermination::kInvalidRequest) {
    result.termination = expand_queue.empty() ? PlanTermination::kFrontierExhausted :
      PlanTermination::kPostGoalComplete;
  }

  const auto clean_start = Clock::now();
  std::vector<std::size_t> degree(result.graph.nodes.size(), 0U);
  for (const auto & edge : result.graph.edges) {
    if (edge.from < degree.size() && edge.to < degree.size()) {
      ++degree[edge.from];
      ++degree[edge.to];
    }
  }
  std::vector<NodeId> old_to_new(result.graph.nodes.size(), result.graph.nodes.size());
  TerrainGraph clean;
  for (const auto & node : result.graph.nodes) {
    if (node.state == GraphNodeState::kInvalid || degree[node.id] == 0U) {continue;}
    old_to_new[node.id] = clean.nodes.size();
    clean.nodes.push_back({clean.nodes.size(), node.point, node.elevation_m, node.state});
  }
  for (const auto & edge : result.graph.edges) {
    if (edge.from >= old_to_new.size() || edge.to >= old_to_new.size() ||
      old_to_new[edge.from] >= clean.nodes.size() || old_to_new[edge.to] >= clean.nodes.size())
    {continue;}
    clean.edges.push_back({old_to_new[edge.from], old_to_new[edge.to], edge.evaluation});
  }
  if (old_to_new[0U] < clean.nodes.size()) {result.start_node_id = old_to_new[0U];}
  if (result.goal_node_id && *result.goal_node_id < old_to_new.size() &&
    old_to_new[*result.goal_node_id] < clean.nodes.size())
  {
    result.goal_node_id = old_to_new[*result.goal_node_id];
  } else {
    result.goal_node_id.reset();
  }
  result.graph = std::move(clean);
  result.graph_clean_time_ms = milliseconds(Clock::now() - clean_start);
  result.build_time_ms = milliseconds(Clock::now() - build_start);
  return result;
}

}  // namespace rubi_heightmap_step_wavefront_planner
