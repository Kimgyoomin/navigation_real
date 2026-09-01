#pragma once

#include <string>
#include <vector>

#include "rubi_heightmap_step_wavefront_planner/graph/wavefront_graph_builder.hpp"
#include "rubi_heightmap_step_wavefront_planner/planning/plan_result.hpp"
#include "rubi_heightmap_step_wavefront_planner/search/astar_search.hpp"

namespace rubi_heightmap_step_wavefront_planner
{
using StepWavefrontParameters = WavefrontGraphParameters;
/** @brief Orchestrates graph construction, A* search, and path metrics. */
class StepWavefrontPlanner
{
public:
  explicit StepWavefrontPlanner(StepWavefrontParameters parameters);
  PlanResult plan(const StepEvaluator & evaluator, Point2D start, Point2D goal) const;
  static std::vector<NodeId> shortestPath(
    const std::vector<GraphNode> & nodes, const std::vector<GraphEdge> & edges,
    NodeId start, NodeId goal, double distance_weight, double * total_cost = nullptr);
private:
  WavefrontGraphBuilder graph_builder_;
  AStarSearch astar_search_;
};
}  // namespace rubi_heightmap_step_wavefront_planner
