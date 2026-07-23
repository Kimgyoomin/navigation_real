#pragma once

#include "pongbot_local_graph_insertion_planner/grid_snapshot.hpp"

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace pongbot_local_graph_insertion_planner {

enum class SearchStatus { kSuccess, kNoPath, kInvalidInput, kCancelled, kTimeout };

struct SearchOptions {
  bool allow_unknown{false};
  unsigned char blocked_cost_threshold{253};
  double cost_penalty{0.0};
  std::size_t max_expansions{0};  // 0 means unlimited.
  std::function<bool()> cancelled;
};

struct SearchResult {
  SearchStatus status{SearchStatus::kNoPath};
  std::string reason;
  std::vector<std::size_t> path;
  std::size_t expanded{0};
  double cost{0.0};
};

bool isBlocked(const GridSnapshot & grid, std::size_t cell, const SearchOptions & options);
double traversalMultiplier(const GridSnapshot & grid, std::size_t cell, const SearchOptions & options);
double edgeCost(const GridSnapshot & grid, std::size_t from, std::size_t to, const SearchOptions & options);
std::vector<std::size_t> neighbors8(
  const GridSnapshot & grid, std::size_t cell, const SearchOptions & options);
double heuristic(const GridSnapshot & grid, std::size_t from, std::size_t to);
bool validPath(
  const GridSnapshot & grid, const std::vector<std::size_t> & path,
  const SearchOptions & options, double * cost = nullptr);
SearchResult freshAstar(
  const GridSnapshot & grid, std::size_t start, std::size_t goal, const SearchOptions & options);

}  // namespace pongbot_local_graph_insertion_planner
