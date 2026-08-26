#pragma once

#include "rubi_heightmap_step_wavefront_planner/graph/graph_types.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

struct WavefrontGraphParameters
{
  double node_sampling_distance_m{0.30};
  std::size_t samples_per_expansion{20U};
  double merge_radius_m{0.20};
  double neighbor_connection_radius_m{0.45};
  double goal_connection_distance_m{0.45};
  std::size_t max_nodes{4000U};
  std::size_t max_expansions{4000U};
  std::size_t max_graph_build_time_ms{5000U};
  std::size_t post_goal_expansions{50U};
};

/**
 * @brief Builds one deterministic accepted-edge graph for a single request.
 *
 * The builder owns graph topology only. Terrain validity and edge risk remain
 * exclusively owned by StepEvaluator.
 */
class WavefrontGraphBuilder
{
public:
  explicit WavefrontGraphBuilder(WavefrontGraphParameters parameters);
  GraphBuildResult build(
    const StepEvaluator & evaluator, Point2D start, Point2D goal) const;

private:
  WavefrontGraphParameters parameters_;
};

}  // namespace rubi_heightmap_step_wavefront_planner
