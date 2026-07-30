#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

#include "rubi_heightmap_wavefront_planner/terrain_evaluator.hpp"
#include "rubi_heightmap_wavefront_planner/terrain_snapshot.hpp"
#include "rubi_heightmap_wavefront_planner/wavefront_planner.hpp"

namespace planner = rubi_heightmap_wavefront_planner;

int main()
{
  // Keep the previous approximately 22 m x 21.4 m physical map while
  // exercising the Wavefront V0 5 cm profile. This has roughly four times as
  // many terrain cells as the former 10 cm benchmark. The route is named and
  // limited to 3 m so an unoptimized demo build can complete inside the V0
  // graph-build budget; this is not a wall-clock performance assertion.
  constexpr double resolution = 0.05;
  constexpr std::size_t size_x = 440;
  constexpr std::size_t size_y = 428;
  constexpr double min_x = -1.85;
  constexpr double min_y = -12.25;

  std::vector<planner::TerrainPoint> points;
  points.reserve(size_x * size_y);
  for (std::size_t iy = 0; iy < size_y; ++iy) {
    for (std::size_t ix = 0; ix < size_x; ++ix) {
      points.push_back(
      {
        min_x + static_cast<double>(ix) * resolution,
        min_y + static_cast<double>(iy) * resolution,
        0.0
      });
    }
  }

  const auto start_time = std::chrono::steady_clock::now();
  const planner::TerrainSnapshot snapshot =
    planner::TerrainSnapshot::fromPoints(points, resolution, 0.01);

  planner::TerrainEvaluatorParameters terrain_parameters;
  terrain_parameters.pca_radius_m = 0.30;
  terrain_parameters.min_pca_points = 6;
  terrain_parameters.footprint_radius_m = 0.20;
  terrain_parameters.min_footprint_observed_ratio = 1.00;
  terrain_parameters.max_slope_deg = 15.0;
  terrain_parameters.max_step_height_m = 0.08;
  terrain_parameters.edge_sample_spacing_m = 0.025;
  const planner::TerrainEvaluator terrain(snapshot, terrain_parameters);

  planner::WavefrontPlannerParameters wavefront_parameters;
  wavefront_parameters.node_sampling_distance_m = 0.30;
  wavefront_parameters.num_expansion_samples = 20;
  wavefront_parameters.merge_radius_m = 0.20;
  wavefront_parameters.neighbor_connection_radius_m = 0.45;
  wavefront_parameters.goal_connection_distance_m = 0.45;
  wavefront_parameters.max_nodes = 4000;
  wavefront_parameters.max_expansions = 4000;
  wavefront_parameters.max_build_time_ms = 5000;
  wavefront_parameters.stop_when_goal_connected = true;
  wavefront_parameters.slope_normalization_deg = 15.0;
  wavefront_parameters.risk_weights.distance = 1.0;
  wavefront_parameters.risk_weights.slope = 3.0;

  const planner::WavefrontPlanner wavefront(wavefront_parameters);
  const planner::PlanResult result =
    wavefront.plan(terrain, {0.0, 0.0}, {3.0, 0.0});
  const double total_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start_time).count();

  double path_length_m = 0.0;
  for (std::size_t index = 1U; index < result.path.size(); ++index) {
    const planner::TerrainPoint & from = result.path[index - 1U];
    const planner::TerrainPoint & to = result.path[index];
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double dz = to.z - from.z;
    path_length_m += std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  std::cout
    << "scenario=flat_22m_map_3m_route"
    << " success=" << result.success
    << " termination=" << static_cast<int>(result.termination)
    << " map_cells=" << snapshot.cellCount()
    << " nodes=" << result.nodes.size()
    << " edges=" << result.edges.size()
    << " rejected=" << result.rejected.size()
    << " expansions=" << result.expansions
    << " build_time_ms=" << result.build_time_ms
    << " total_time_ms=" << total_ms
    << " path_nodes=" << result.path.size()
    << " path_length_m=" << path_length_m
    << '\n';
  return result.success ? 0 : 2;
}
