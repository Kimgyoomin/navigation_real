#include <chrono>
#include <cstddef>
#include <iostream>
#include <vector>

#include "rubi_heightmap_wavefront_planner/terrain_evaluator.hpp"
#include "rubi_heightmap_wavefront_planner/terrain_snapshot.hpp"
#include "rubi_heightmap_wavefront_planner/wavefront_planner.hpp"

namespace planner = rubi_heightmap_wavefront_planner;

int main()
{
  constexpr double resolution = 0.1;
  constexpr std::size_t size_x = 220;
  constexpr std::size_t size_y = 214;
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
    planner::TerrainSnapshot::fromPoints(points, resolution, 0.02);

  planner::TerrainEvaluatorParameters terrain_parameters;
  terrain_parameters.pca_radius_m = 0.30;
  terrain_parameters.min_pca_points = 6;
  terrain_parameters.footprint_radius_m = 0.20;
  terrain_parameters.min_footprint_observed_ratio = 1.00;
  terrain_parameters.max_slope_deg = 15.0;
  terrain_parameters.max_step_height_m = 0.08;
  terrain_parameters.edge_sample_spacing_m = 0.05;
  const planner::TerrainEvaluator terrain(snapshot, terrain_parameters);

  planner::WavefrontPlannerParameters wavefront_parameters;
  wavefront_parameters.node_sampling_distance_m = 0.50;
  wavefront_parameters.num_expansion_samples = 12;
  wavefront_parameters.merge_radius_m = 0.25;
  wavefront_parameters.neighbor_connection_radius_m = 0.75;
  wavefront_parameters.goal_connection_distance_m = 0.75;
  wavefront_parameters.max_nodes = 4000;
  wavefront_parameters.max_expansions = 4000;
  wavefront_parameters.max_build_time_ms = 2000;
  wavefront_parameters.stop_when_goal_connected = false;
  wavefront_parameters.slope_normalization_deg = 15.0;
  wavefront_parameters.risk_weights.distance = 1.0;
  wavefront_parameters.risk_weights.slope = 3.0;

  const planner::WavefrontPlanner wavefront(wavefront_parameters);
  const planner::PlanResult result =
    wavefront.plan(terrain, {0.0, 0.0}, {10.0, 0.0});
  const double total_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start_time).count();

  std::cout
    << "success=" << result.success
    << " termination=" << static_cast<int>(result.termination)
    << " nodes=" << result.nodes.size()
    << " edges=" << result.edges.size()
    << " expansions=" << result.expansions
    << " build_ms=" << result.build_time_ms
    << " total_ms=" << total_ms
    << " path_nodes=" << result.path.size()
    << '\n';
  return result.success ? 0 : 2;
}
