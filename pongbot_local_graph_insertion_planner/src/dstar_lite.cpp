#include "pongbot_local_graph_insertion_planner/dstar_lite.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pongbot_local_graph_insertion_planner
{
namespace
{
constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kEpsilon = 1e-10;

bool equal(double a, double b)
{
  if (a == b) {
    return true;  // Includes +infinity == +infinity.
  }
  if (!std::isfinite(a) || !std::isfinite(b)) {
    return false;
  }
  return std::abs(a - b) <= kEpsilon;
}
}  // namespace

const char * fallbackReasonName(FallbackReason reason)
{
  switch (reason) {
    case FallbackReason::kNone:
      return "none";
    case FallbackReason::kInitialPlan:
      return "initial_plan";
    case FallbackReason::kGeometryChanged:
      return "geometry_changed";
    case FallbackReason::kGoalChanged:
      return "goal_changed";
    case FallbackReason::kOptionsChanged:
      return "options_changed";
    case FallbackReason::kChangedRatio:
      return "changed_ratio";
    case FallbackReason::kInvariantViolation:
      return "invariant_violation";
    case FallbackReason::kExtractionFailure:
      return "extraction_failure";
  }
  return "unknown";
}

DStarLite::DStarLite(double ratio)
: changed_ratio_fallback_(ratio) {}

bool DStarLite::EntryCompare::operator()(const Entry & a, const Entry & b) const
{
  return a.key.first == b.key.first ? a.key.second > b.key.second : a.key.first > b.key.first;
}

bool DStarLite::keyLess(const Key & a, const Key & b) const
{
  return a.first + kEpsilon < b.first ||
         (equal(a.first, b.first) && a.second + kEpsilon < b.second);
}

bool DStarLite::optionsEqual(const SearchOptions & o) const
{
  return options_.allow_unknown == o.allow_unknown &&
         options_.blocked_cost_threshold == o.blocked_cost_threshold &&
         options_.cost_penalty == o.cost_penalty;
}

bool DStarLite::cancelled() const
{
  return options_.cancelled && options_.cancelled();
}

void DStarLite::reset(
  const GridSnapshot & grid, std::size_t start, std::size_t goal, const SearchOptions & options)
{
  grid_ = grid;
  options_ = options;
  start_ = start;
  last_start_ = start;
  goal_ = goal;
  km_ = 0.0;
  g_.assign(grid.costs.size(), kInfinity);
  rhs_.assign(grid.costs.size(), kInfinity);
  open_ = {};

  rhs_[goal_] = 0.0;
  push(goal_);
  initialized_ = true;
}

DStarLite::Key DStarLite::calculateKey(std::size_t cell) const
{
  const double best = std::min(g_[cell], rhs_[cell]);
  return {best + heuristic(grid_, start_, cell) + km_, best};
}

void DStarLite::push(std::size_t cell)
{
  if (!equal(g_[cell], rhs_[cell])) {
    open_.push({calculateKey(cell), cell});
  }
}

void DStarLite::updateVertex(std::size_t cell)
{
  if (cell != goal_) {
    double minimum_rhs = kInfinity;
    if (!isBlocked(grid_, cell, options_)) {
      for (const auto successor : neighbors8(grid_, cell, options_)) {
        const double candidate =
          edgeCost(grid_, cell, successor, options_) + g_[successor];
        minimum_rhs = std::min(minimum_rhs, candidate);
      }
    }
    rhs_[cell] = minimum_rhs;
  }
  push(cell);
}

void DStarLite::applyChanges(const GridSnapshot & grid)
{
  std::vector<std::size_t> affected;

  for (std::size_t cell = 0; cell < grid.costs.size(); ++cell) {
    if (grid.costs[cell] == grid_.costs[cell]) {
      continue;
    }

    const auto changed_x = grid.x(cell);
    const auto changed_y = grid.y(cell);

    // Changed cell, its incident edges, and diagonal no-corner-cutting dependencies.
    for (int dy = -2; dy <= 2; ++dy) {
      for (int dx = -2; dx <= 2; ++dx) {
        const auto x = static_cast<long long>(changed_x) + dx;
        const auto y = static_cast<long long>(changed_y) + dy;
        const bool inside_grid =
          x >= 0 && y >= 0 &&
          x < static_cast<long long>(grid.size_x) &&
          y < static_cast<long long>(grid.size_y);
        if (inside_grid) {
          affected.push_back(
            grid.index(static_cast<std::size_t>(x), static_cast<std::size_t>(y)));
        }
      }
    }
  }

  grid_ = grid;
  std::sort(affected.begin(), affected.end());
  affected.erase(std::unique(affected.begin(), affected.end()), affected.end());

  for (const auto cell : affected) {
    updateVertex(cell);
  }
}

bool DStarLite::computeShortestPath(SearchResult & result)
{
  while (!open_.empty() &&
    (keyLess(open_.top().key, calculateKey(start_)) || !equal(g_[start_], rhs_[start_])))
  {
    if (cancelled()) {
      result.status = SearchStatus::kCancelled;
      result.reason = "cancelled";
      return false;
    }
    if (options_.max_expansions && result.expanded >= options_.max_expansions) {
      result.status = SearchStatus::kTimeout;
      result.reason = "expansion limit";
      return false;
    }

    const auto entry = open_.top();
    open_.pop();
    const auto current_key = calculateKey(entry.cell);

    if (keyLess(entry.key, current_key)) {
      open_.push({current_key, entry.cell});
      continue;
    }
    if (keyLess(current_key, entry.key) || equal(g_[entry.cell], rhs_[entry.cell])) {
      continue;
    }

    ++result.expanded;

    if (g_[entry.cell] > rhs_[entry.cell]) {
      g_[entry.cell] = rhs_[entry.cell];
    } else {
      g_[entry.cell] = kInfinity;
      updateVertex(entry.cell);
    }

    for (const auto predecessor : neighbors8(grid_, entry.cell, options_)) {
      updateVertex(predecessor);
    }
  }

  if (!equal(g_[start_], rhs_[start_])) {
    result.status = SearchStatus::kNoPath;
    result.reason = "open set exhausted with inconsistent start";
    return false;
  }
  return true;
}

SearchResult DStarLite::extractPath()
{
  SearchResult result;
  if (!std::isfinite(g_[start_])) {
    result.status = SearchStatus::kNoPath;
    result.reason = "start has infinite value";
    return result;
  }

  std::vector<bool> seen(grid_.costs.size(), false);
  std::size_t current = start_;
  result.path.push_back(current);

  while (current != goal_) {
    if (seen[current]) {
      result.status = SearchStatus::kNoPath;
      result.reason = "path extraction cycle";
      return result;
    }
    seen[current] = true;

    double best_cost = kInfinity;
    std::size_t best_successor = grid_.costs.size();
    for (const auto successor : neighbors8(grid_, current, options_)) {
      const double candidate =
        edgeCost(grid_, current, successor, options_) + g_[successor];
      if (candidate + kEpsilon < best_cost) {
        best_cost = candidate;
        best_successor = successor;
      }
    }

    if (best_successor >= grid_.costs.size() || !std::isfinite(best_cost)) {
      result.status = SearchStatus::kNoPath;
      result.reason = "path extraction dead end";
      return result;
    }

    current = best_successor;
    result.path.push_back(current);
    if (result.path.size() > grid_.costs.size()) {
      result.status = SearchStatus::kNoPath;
      result.reason = "path extraction length invariant";
      return result;
    }
  }

  result.status = SearchStatus::kSuccess;
  result.cost = g_[start_];

  double verified_cost = 0.0;
  if (!validPath(grid_, result.path, options_, &verified_cost) ||
    !equal(verified_cost, result.cost))
  {
    result.status = SearchStatus::kNoPath;
    result.reason = "path extraction cost invariant";
    result.path.clear();
  }
  return result;
}

SearchResult DStarLite::replan(
  const GridSnapshot & grid, std::size_t start, std::size_t goal,
  const SearchOptions & options)
{
  last_fallback_reason_ = FallbackReason::kNone;
  last_fallback_detail_.clear();

  const bool invalid_input =
    !grid.valid() ||
    !grid.validIndex(start) ||
    !grid.validIndex(goal) ||
    !std::isfinite(options.cost_penalty) ||
    options.cost_penalty < 0.0 ||
    options.blocked_cost_threshold == 0;
  if (invalid_input) {
    return {SearchStatus::kInvalidInput, "invalid grid, options, or endpoint", {}, 0, 0.0};
  }
  if (isBlocked(grid, start, options) || isBlocked(grid, goal, options)) {
    return {SearchStatus::kNoPath, "start or goal is blocked", {}, 0, 0.0};
  }

  if (!initialized_ || !grid.geometryEquals(grid_)) {
    last_fallback_reason_ =
      initialized_ ? FallbackReason::kGeometryChanged : FallbackReason::kInitialPlan;
    reset(grid, start, goal, options);
  } else if (goal != goal_) {
    last_fallback_reason_ = FallbackReason::kGoalChanged;
    reset(grid, start, goal, options);
  } else if (!optionsEqual(options)) {
    last_fallback_reason_ = FallbackReason::kOptionsChanged;
    reset(grid, start, goal, options);
  } else {
    std::size_t changed_cells = 0;
    for (std::size_t cell = 0; cell < grid.costs.size(); ++cell) {
      changed_cells += grid.costs[cell] != grid_.costs[cell];
    }

    const double changed_ratio =
      static_cast<double>(changed_cells) / static_cast<double>(grid.costs.size());
    if (changed_ratio > changed_ratio_fallback_) {
      last_fallback_reason_ = FallbackReason::kChangedRatio;
      last_fallback_detail_ =
        "fresh A* selected because changed-cell ratio exceeded threshold";
      reset(grid, start, goal, options);
      return freshAstar(grid, start, goal, options);
    }

    // The cost contract is unchanged, but cancellation and expansion limits are
    // per-request runtime controls. Never retain a previous planning deadline.
    options_.cancelled = options.cancelled;
    options_.max_expansions = options.max_expansions;

    km_ += heuristic(grid_, last_start_, start);
    start_ = start;
    last_start_ = start;
    applyChanges(grid);
  }

  SearchResult computation;
  if (!computeShortestPath(computation)) {
    return computation;
  }

  SearchResult result = extractPath();
  result.expanded = computation.expanded;
  if (result.status != SearchStatus::kSuccess) {
    last_fallback_reason_ = FallbackReason::kExtractionFailure;
    last_fallback_detail_ = result.reason;
    return freshAstar(grid_, start_, goal_, options_);
  }
  return result;
}
}  // namespace pongbot_local_graph_insertion_planner
