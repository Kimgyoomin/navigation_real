#pragma once

#include "rubi_heightmap_step_wavefront_planner/graph/graph_types.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

/** @brief Deterministic A* over the accepted, undirected Phase-1 graph. */
class AStarSearch
{
public:
  SearchResult search(
    const TerrainGraph & graph, NodeId start, NodeId goal, double distance_weight) const;
};

}  // namespace rubi_heightmap_step_wavefront_planner
