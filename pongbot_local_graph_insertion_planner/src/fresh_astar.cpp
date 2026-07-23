#include "pongbot_local_graph_insertion_planner/fresh_astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace pongbot_local_graph_insertion_planner {
namespace {
constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kSqrt2 = 1.4142135623730950488;
struct QueueEntry { double f; double g; std::size_t cell; };
struct QueueCompare {
  bool operator()(const QueueEntry & a, const QueueEntry & b) const {
    return a.f == b.f ? a.g > b.g : a.f > b.f;
  }
};
bool validOptions(const SearchOptions & options) {
  return options.blocked_cost_threshold != 0 && std::isfinite(options.cost_penalty) &&
    options.cost_penalty >= 0.0;
}
}  // namespace

bool isBlocked(const GridSnapshot & grid, std::size_t cell, const SearchOptions & options) {
  if (!grid.validIndex(cell)) return true;
  const auto cost = grid.costs[cell];
  return cost == 255 ? !options.allow_unknown : cost >= options.blocked_cost_threshold;
}

double traversalMultiplier(const GridSnapshot & grid, std::size_t cell, const SearchOptions & options) {
  if (isBlocked(grid, cell, options)) return kInfinity;
  const auto cost = grid.costs[cell];
  // Unknown cells are explicitly permitted at the highest traversable cost.
  const double normalized = (cost == 255 ? 1.0 : static_cast<double>(cost) / 252.0);
  return 1.0 + options.cost_penalty * normalized;
}

std::vector<std::size_t> neighbors8(
  const GridSnapshot & grid, std::size_t cell, const SearchOptions & options)
{
  std::vector<std::size_t> result;
  if (!grid.validIndex(cell) || isBlocked(grid, cell, options)) return result;
  const auto cx = grid.x(cell);
  const auto cy = grid.y(cell);
  for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
    if (dx == 0 && dy == 0) continue;
    const auto nx_i = static_cast<long long>(cx) + dx;
    const auto ny_i = static_cast<long long>(cy) + dy;
    if (nx_i < 0 || ny_i < 0 || nx_i >= static_cast<long long>(grid.size_x) ||
      ny_i >= static_cast<long long>(grid.size_y)) continue;
    const auto next = grid.index(static_cast<std::size_t>(nx_i), static_cast<std::size_t>(ny_i));
    if (isBlocked(grid, next, options)) continue;
    if (dx != 0 && dy != 0) {
      const auto side_x = grid.index(static_cast<std::size_t>(static_cast<long long>(cx) + dx), cy);
      const auto side_y = grid.index(cx, static_cast<std::size_t>(static_cast<long long>(cy) + dy));
      if (isBlocked(grid, side_x, options) || isBlocked(grid, side_y, options)) continue;
    }
    result.push_back(next);
  }
  return result;
}

double edgeCost(const GridSnapshot & grid, std::size_t from, std::size_t to, const SearchOptions & options) {
  if (!grid.validIndex(from) || !grid.validIndex(to)) return kInfinity;
  const auto neighbors = neighbors8(grid, from, options);
  if (std::find(neighbors.begin(), neighbors.end(), to) == neighbors.end()) return kInfinity;
  const auto dx = grid.x(from) > grid.x(to) ? grid.x(from) - grid.x(to) : grid.x(to) - grid.x(from);
  const auto dy = grid.y(from) > grid.y(to) ? grid.y(from) - grid.y(to) : grid.y(to) - grid.y(from);
  const double metric = (dx == 1 && dy == 1 ? kSqrt2 : 1.0) * grid.resolution;
  return metric * traversalMultiplier(grid, to, options);
}

double heuristic(const GridSnapshot & grid, std::size_t from, std::size_t to) {
  const double dx = static_cast<double>(grid.x(from) > grid.x(to) ? grid.x(from) - grid.x(to) : grid.x(to) - grid.x(from));
  const double dy = static_cast<double>(grid.y(from) > grid.y(to) ? grid.y(from) - grid.y(to) : grid.y(to) - grid.y(from));
  return grid.resolution * (std::max(dx, dy) + (kSqrt2 - 1.0) * std::min(dx, dy));
}

bool validPath(const GridSnapshot & grid, const std::vector<std::size_t> & path,
  const SearchOptions & options, double * cost)
{
  if (path.empty()) return false;
  double total = 0.0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (!grid.validIndex(path[i]) || isBlocked(grid, path[i], options)) return false;
    if (i) {
      const double edge = edgeCost(grid, path[i - 1], path[i], options);
      if (!std::isfinite(edge)) return false;
      total += edge;
    }
  }
  if (cost) *cost = total;
  return std::isfinite(total);
}

SearchResult freshAstar(const GridSnapshot & grid, std::size_t start, std::size_t goal,
  const SearchOptions & options)
{
  SearchResult result;
  if (!grid.valid() || !validOptions(options) || !grid.validIndex(start) || !grid.validIndex(goal)) {
    result.status = SearchStatus::kInvalidInput; result.reason = "invalid grid, options, or endpoint"; return result;
  }
  if (isBlocked(grid, start, options) || isBlocked(grid, goal, options)) {
    result.status = SearchStatus::kNoPath; result.reason = "start or goal is blocked"; return result;
  }
  if (start == goal) {
    result.status = SearchStatus::kSuccess; result.path = {start}; return result;
  }
  std::vector<double> g(grid.costs.size(), kInfinity);
  std::vector<std::size_t> parent(grid.costs.size(), grid.costs.size());
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> open;
  g[start] = 0.0; open.push({heuristic(grid, start, goal), 0.0, start});
  while (!open.empty()) {
    if (options.cancelled && options.cancelled()) {
      result.status = SearchStatus::kCancelled; result.reason = "cancelled"; return result;
    }
    const auto current = open.top(); open.pop();
    if (current.g != g[current.cell]) continue;
    if (options.max_expansions && result.expanded >= options.max_expansions) {
      result.status = SearchStatus::kTimeout; result.reason = "expansion limit"; return result;
    }
    ++result.expanded;
    if (current.cell == goal) {
      std::vector<std::size_t> reversed;
      for (auto node = goal; node != start; node = parent[node]) {
        if (node >= parent.size() || parent[node] >= parent.size()) {
          result.status = SearchStatus::kNoPath; result.reason = "parent invariant failure"; return result;
        }
        reversed.push_back(node);
      }
      reversed.push_back(start); std::reverse(reversed.begin(), reversed.end());
      result.status = SearchStatus::kSuccess; result.path = std::move(reversed); result.cost = g[goal]; return result;
    }
    for (const auto next : neighbors8(grid, current.cell, options)) {
      const double tentative = g[current.cell] + edgeCost(grid, current.cell, next, options);
      if (tentative + 1e-12 < g[next]) {
        g[next] = tentative; parent[next] = current.cell;
        open.push({tentative + heuristic(grid, next, goal), tentative, next});
      }
    }
  }
  result.status = SearchStatus::kNoPath; result.reason = "open set exhausted"; return result;
}

}  // namespace pongbot_local_graph_insertion_planner
