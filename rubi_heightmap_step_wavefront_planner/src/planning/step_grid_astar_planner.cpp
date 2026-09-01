#include "rubi_heightmap_step_wavefront_planner/planning/step_grid_astar_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace rubi_heightmap_step_wavefront_planner
{
namespace
{
using Clock = std::chrono::steady_clock;
constexpr double kTolerance = 1.0e-12;

bool hardBlocked(const CostmapSnapshot & costmap, const GridCell cell)
{
  const auto cost = costmap.cost(cell);
  return !cost || *cost >= 253U;
}
}  // namespace

StepGridAStarPlanner::StepGridAStarPlanner(const GridAStarParameters parameters)
: parameters_(parameters)
{
  if (parameters_.max_expanded_states == 0U) {
    throw std::invalid_argument("grid max_expanded_states must be > 0");
  }
}

double StepGridAStarPlanner::octileDistance(const int dx, const int dy) noexcept
{
  const double x = std::abs(dx);
  const double y = std::abs(dy);
  return x + y + (std::sqrt(2.0) - 2.0) * std::min(x, y);
}

PlanResult StepGridAStarPlanner::plan(
  const StepEvaluator & evaluator, const Point2D start, const Point2D goal) const
{
  const auto started = Clock::now();
  PlanResult result;
  const CostmapSnapshot * costmap = evaluator.costmap();
  if (!costmap || evaluator.mode() != StepEvaluationMode::kCostmapHeightHybrid) {
    result.message = "grid planner requires hybrid evaluator";
    return result;
  }
  const auto start_cell = costmap->worldToCell(start);
  const auto goal_cell = costmap->worldToCell(goal);
  if (!start_cell || !goal_cell || !costmap->inBounds(*start_cell) ||
    !costmap->inBounds(*goal_cell))
  {
    result.message = "grid endpoint outside costmap";
    return result;
  }
  ++result.statistics.node_evaluation_calls;
  const NodeEvaluation start_evaluation = evaluator.evaluateNode(costmap->cellCenter(*start_cell));
  ++result.statistics.node_evaluation_calls;
  const NodeEvaluation goal_evaluation = evaluator.evaluateNode(costmap->cellCenter(*goal_cell));
  if (!start_evaluation.valid || !goal_evaluation.valid) {
    result.message = !start_evaluation.valid ? "grid start invalid" : "grid goal invalid";
    result.rejected.push_back({RejectionKind::kNode,
      !start_evaluation.valid ? start_evaluation.reason : goal_evaluation.reason,
      !start_evaluation.valid ? start : goal, !start_evaluation.valid ? start : goal});
    return result;
  }

  const std::size_t width = costmap->sizeX();
  const std::size_t count = costmap->cellCount();
  const auto toIndex = [width](const GridCell cell) {
      return static_cast<std::size_t>(cell.y) * width + static_cast<std::size_t>(cell.x);
    };
  const auto toCell = [width](const std::size_t index) {
      return GridCell{static_cast<int>(index % width), static_cast<int>(index / width)};
    };
  const std::size_t start_index = toIndex(*start_cell);
  const std::size_t goal_index = toIndex(*goal_cell);
  struct Entry {double f; double g; std::size_t index;};
  struct Greater {bool operator()(const Entry & left, const Entry & right) const noexcept {
    return std::tie(left.f, left.g, left.index) > std::tie(right.f, right.g, right.index);
  }};
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> g(count, infinity);
  std::vector<std::size_t> parent(count, count);
  std::vector<bool> closed(count, false);
  std::vector<double> elevation(count, std::numeric_limits<double>::quiet_NaN());
  std::vector<NodeId> graph_node_id(count, count);
  elevation[start_index] = start_evaluation.elevation_m;
  elevation[goal_index] = goal_evaluation.elevation_m;
  const auto materialize_expanded_cell = [&](const std::size_t index) {
      if (graph_node_id[index] != count) {
        return;
      }
      const Point2D point = costmap->cellCenter(toCell(index));
      if (!std::isfinite(elevation[index])) {
        elevation[index] = evaluator.evaluateNode(point).elevation_m;
      }
      graph_node_id[index] = result.nodes.size();
      result.nodes.push_back({graph_node_id[index], point, elevation[index]});
      if (parent[index] < count && graph_node_id[parent[index]] != count) {
        const EdgeEvaluation edge = evaluator.evaluateEdge(
          costmap->cellCenter(toCell(parent[index])), point);
        result.edges.push_back({graph_node_id[parent[index]], graph_node_id[index], edge});
      }
    };
  std::priority_queue<Entry, std::vector<Entry>, Greater> open;
  const auto heuristic = [&](const GridCell cell) {
      return evaluator.parameters().distance_weight * costmap->resolution() *
             octileDistance(cell.x - goal_cell->x, cell.y - goal_cell->y);
    };
  g[start_index] = 0.0;
  open.push({heuristic(*start_cell), 0.0, start_index});
  ++result.statistics.astar_open_pushes;
  constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  bool found = false;
  while (!open.empty()) {
    if (parameters_.max_planning_time_ms > 0U &&
      std::chrono::duration<double, std::milli>(Clock::now() - started).count() >=
      static_cast<double>(parameters_.max_planning_time_ms))
    {
      result.termination = PlanTermination::kMaxGraphBuildTime;
      result.message = "grid planning timeout";
      break;
    }
    const Entry current = open.top();
    open.pop();
    if (closed[current.index] || std::abs(current.g - g[current.index]) > kTolerance) {continue;}
    closed[current.index] = true;
    materialize_expanded_cell(current.index);
    ++result.expansions;
    if (result.expansions > parameters_.max_expanded_states) {
      result.termination = PlanTermination::kMaxExpansions;
      result.message = "grid expansion limit";
      break;
    }
    if (current.index == goal_index) {found = true; break;}
    const GridCell current_cell = toCell(current.index);
    for (int direction = 0; direction < (parameters_.allow_diagonal ? 8 : 4); ++direction) {
      ++result.statistics.neighbor_candidates;
      const GridCell neighbor{current_cell.x + kDx[direction], current_cell.y + kDy[direction]};
      if (!costmap->inBounds(neighbor)) {continue;}
      if (direction >= 4 && (hardBlocked(
          *costmap, {current_cell.x + kDx[direction], current_cell.y}) ||
        hardBlocked(*costmap, {current_cell.x, current_cell.y + kDy[direction]})))
      {
        continue;
      }
      ++result.statistics.node_evaluation_calls;
      const Point2D neighbor_point = costmap->cellCenter(neighbor);
      const NodeEvaluation node = evaluator.evaluateNode(neighbor_point);
      if (!node.valid) {
        result.rejected.push_back({RejectionKind::kNode, node.reason,
          costmap->cellCenter(current_cell), neighbor_point});
        continue;
      }
      elevation[toIndex(neighbor)] = node.elevation_m;
      ++result.statistics.edge_evaluation_calls;
      const EdgeEvaluation edge = evaluator.evaluateEdge(
        costmap->cellCenter(current_cell), neighbor_point);
      if (!edge.valid) {
        result.rejected.push_back({RejectionKind::kEdge, edge.reason,
          costmap->cellCenter(current_cell), neighbor_point});
        continue;
      }
      const std::size_t neighbor_index = toIndex(neighbor);
      const double tentative = g[current.index] + edge.cost;
      if (tentative + kTolerance < g[neighbor_index] ||
        (std::abs(tentative - g[neighbor_index]) <= kTolerance &&
        current.index < parent[neighbor_index]))
      {
        g[neighbor_index] = tentative;
        parent[neighbor_index] = current.index;
        open.push({tentative + heuristic(neighbor), tentative, neighbor_index});
        ++result.statistics.astar_open_pushes;
      }
    }
  }
  result.astar_time_ms = std::chrono::duration<double, std::milli>(
    Clock::now() - started).count();
  result.statistics.expanded_states = result.expansions;
  result.statistics.accepted_nodes = result.nodes.size();
  result.statistics.accepted_edges = result.edges.size();
  if (!found) {
    if (result.message.empty()) {
      result.termination = PlanTermination::kFrontierExhausted;
      result.message = "grid path not found";
    }
    result.core_total_time_ms = std::chrono::duration<double, std::milli>(
      Clock::now() - started).count();
    return result;
  }
  std::vector<std::size_t> path_cells;
  for (std::size_t trace = goal_index; ; trace = parent[trace]) {
    path_cells.push_back(trace);
    if (trace == start_index) {break;}
    if (parent[trace] >= count || path_cells.size() > count) {
      path_cells.clear();
      break;
    }
  }
  std::reverse(path_cells.begin(), path_cells.end());
  if (path_cells.empty()) {return result;}
  const auto finalize_started = Clock::now();
  for (std::size_t path_index = 0U; path_index < path_cells.size(); ++path_index) {
    const std::size_t cell_index = path_cells[path_index];
    materialize_expanded_cell(cell_index);
    const Point2D point = costmap->cellCenter(toCell(cell_index));
    result.path_node_ids.push_back(graph_node_id[cell_index]);
    if (path_index == 0U) {continue;}
    const EdgeEvaluation edge = evaluator.evaluateEdge(
      costmap->cellCenter(toCell(path_cells[path_index - 1U])), point);
    result.path_metrics.length_xy_m += edge.length_xy_m;
    result.path_metrics.height_event_count += edge.height_jump_event_count;
    result.path_metrics.max_height_jump_m = std::max(
      result.path_metrics.max_height_jump_m, edge.max_height_jump_m);
    result.path_metrics.height_score_m += edge.height_jump_score_m;
    result.path_metrics.inflation_score_m += edge.inflation_score_m;
    result.path_metrics.maximum_raw_cost = std::max(
      result.path_metrics.maximum_raw_cost, edge.maximum_raw_cost);
  }
  result.path_metrics.height_cost = evaluator.parameters().height_cost_weight *
    result.path_metrics.height_score_m;
  result.path_metrics.inflation_cost = evaluator.parameters().inflation_cost_weight *
    result.path_metrics.inflation_score_m;
  result.path_metrics.total_cost = g[goal_index];
  result.success = true;
  result.termination = PlanTermination::kPostGoalComplete;
  result.message = "grid path found";
  result.statistics.accepted_nodes = result.nodes.size();
  result.statistics.accepted_edges = result.edges.size();
  const auto & instrumentation = evaluator.instrumentation();
  result.statistics.edge_samples_total = instrumentation.edge_samples_total;
  result.statistics.height_evidence_queries = instrumentation.height_evidence_queries;
  result.statistics.costmap_queries = instrumentation.costmap_queries;
  result.path_finalize_time_ms = std::chrono::duration<double, std::milli>(
    Clock::now() - finalize_started).count();
  result.core_total_time_ms = std::chrono::duration<double, std::milli>(
    Clock::now() - started).count();
  return result;
}

}  // namespace rubi_heightmap_step_wavefront_planner
