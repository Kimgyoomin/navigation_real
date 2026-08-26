#include "rubi_heightmap_step_wavefront_planner/search/astar_search.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <tuple>

namespace rubi_heightmap_step_wavefront_planner
{

SearchResult AStarSearch::search(
  const TerrainGraph & graph, const NodeId start, const NodeId goal,
  const double distance_weight) const
{
  const auto search_start = std::chrono::steady_clock::now();
  SearchResult result;
  if (start >= graph.nodes.size() || goal >= graph.nodes.size() ||
    !std::isfinite(distance_weight) || distance_weight <= 0.0)
  {
    return result;
  }
  struct Adjacent {NodeId id; double cost;};
  std::vector<std::vector<Adjacent>> adjacency(graph.nodes.size());
  for (const auto & edge : graph.edges) {
    if (!edge.evaluation.valid || edge.from >= graph.nodes.size() ||
      edge.to >= graph.nodes.size() || !std::isfinite(edge.evaluation.cost) ||
      edge.evaluation.cost < 0.0)
    {continue;}
    adjacency[edge.from].push_back({edge.to, edge.evaluation.cost});
    adjacency[edge.to].push_back({edge.from, edge.evaluation.cost});
  }
  for (auto & neighbors : adjacency) {
    std::sort(neighbors.begin(), neighbors.end(), [](const auto & lhs, const auto & rhs) {
      return std::tie(lhs.id, lhs.cost) < std::tie(rhs.id, rhs.cost);
    });
  }
  struct OpenEntry {double f; double g; NodeId id;};
  struct Greater {bool operator()(const OpenEntry & lhs, const OpenEntry & rhs) const noexcept {
    return std::tie(lhs.f, lhs.g, lhs.id) > std::tie(rhs.f, rhs.g, rhs.id);
  }};
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> g(graph.nodes.size(), infinity);
  std::vector<NodeId> parent(graph.nodes.size(), graph.nodes.size());
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, Greater> open;
  const auto heuristic = [&](const NodeId id) {
    // Every accepted edge cost is at least distance_weight * XY length;
    // nonnegative height/clearance penalties preserve Euclidean admissibility.
    return distance_weight * std::hypot(
      graph.nodes[id].point.x - graph.nodes[goal].point.x,
      graph.nodes[id].point.y - graph.nodes[goal].point.y);
  };
  g[start] = 0.0;
  open.push({heuristic(start), 0.0, start});
  while (!open.empty()) {
    const OpenEntry current = open.top(); open.pop();
    if (current.g != g[current.id]) {continue;}
    ++result.expanded_state_count;
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
  if (std::isfinite(g[goal])) {
    for (NodeId id = goal; ; id = parent[id]) {
      result.path_node_ids.push_back(id);
      if (id == start) {break;}
      if (parent[id] >= graph.nodes.size() || result.path_node_ids.size() > graph.nodes.size()) {
        result.path_node_ids.clear();
        break;
      }
    }
    std::reverse(result.path_node_ids.begin(), result.path_node_ids.end());
    result.success = !result.path_node_ids.empty();
    result.total_cost = result.success ? g[goal] : 0.0;
  }
  result.search_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - search_start).count();
  return result;
}

}  // namespace rubi_heightmap_step_wavefront_planner
