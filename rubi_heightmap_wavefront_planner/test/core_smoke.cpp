#include <cassert>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <vector>

#include "rubi_heightmap_wavefront_planner/rrt_star_planner.hpp"
#include "rubi_heightmap_wavefront_planner/terrain_evaluator.hpp"
#include "rubi_heightmap_wavefront_planner/terrain_snapshot.hpp"
#include "rubi_heightmap_wavefront_planner/wavefront_planner.hpp"

namespace planner = rubi_heightmap_wavefront_planner;

namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;

std::vector<planner::TerrainPoint> makeTerrain(
  const std::size_t size_x,
  const std::size_t size_y,
  const double resolution,
  const double dz_dx,
  const double step_x = std::numeric_limits<double>::infinity(),
  const double step_height = 0.0)
{
  std::vector<planner::TerrainPoint> points;
  points.reserve(size_x * size_y);
  for (std::size_t iy = 0; iy < size_y; ++iy) {
    for (std::size_t ix = 0; ix < size_x; ++ix) {
      const double x = static_cast<double>(ix) * resolution;
      const double y =
        (static_cast<double>(iy) - static_cast<double>(size_y / 2U)) * resolution;
      const double z = dz_dx * x + (x >= step_x ? step_height : 0.0);
      points.push_back({x, y, z});
    }
  }
  return points;
}

planner::TerrainEvaluatorParameters permissiveTerrainParameters()
{
  planner::TerrainEvaluatorParameters parameters;
  parameters.pca_radius_m = 0.22;
  parameters.min_pca_points = 5;
  parameters.footprint_radius_m = 0.0;
  parameters.min_footprint_observed_ratio = 1.0;
  parameters.max_slope_deg = 89.0;
  parameters.max_roughness_m = std::numeric_limits<double>::infinity();
  parameters.max_step_height_m = 1.0;
  parameters.edge_sample_spacing_m = 0.05;
  parameters.slope_cost_weight = 1.0;
  return parameters;
}

void checkPcaSlope()
{
  constexpr double slope_deg = 10.0;
  const double dz_dx = std::tan(slope_deg * kPi / 180.0);
  const planner::TerrainSnapshot snapshot = planner::TerrainSnapshot::fromPoints(
    makeTerrain(15, 11, 0.1, dz_dx), 0.1, 1.0e-6);
  const planner::TerrainEvaluator evaluator(
    snapshot, permissiveTerrainParameters());
  const planner::SurfaceMetrics surface = evaluator.localSurface({0.7, 0.0});

  assert(surface.valid);
  assert(std::abs(surface.slope_deg - slope_deg) < 1.0e-6);
  assert(surface.roughness_m < 1.0e-8);
}

void checkUnknownAndStepGates()
{
  auto unknown_points = makeTerrain(15, 11, 0.1, 0.0);
  for (auto it = unknown_points.begin(); it != unknown_points.end(); ++it) {
    if (std::abs(it->x - 0.7) < 1.0e-9 && std::abs(it->y) < 1.0e-9) {
      unknown_points.erase(it);
      break;
    }
  }
  const planner::TerrainSnapshot unknown_snapshot =
    planner::TerrainSnapshot::fromPoints(unknown_points, 0.1, 1.0e-6);
  const planner::TerrainEvaluator unknown_evaluator(
    unknown_snapshot, permissiveTerrainParameters());
  const planner::EdgeEvaluation unknown_edge =
    unknown_evaluator.evaluateEdge({0.2, 0.0}, {1.2, 0.0});
  assert(!unknown_edge.valid);
  assert(unknown_edge.reason == planner::TerrainInvalidReason::kUnknown);

  const planner::TerrainSnapshot step_snapshot = planner::TerrainSnapshot::fromPoints(
    makeTerrain(15, 11, 0.1, 0.0, 0.7, 0.20), 0.1, 1.0e-6);
  auto step_parameters = permissiveTerrainParameters();
  step_parameters.max_step_height_m = 0.08;
  const planner::TerrainEvaluator step_evaluator(step_snapshot, step_parameters);
  const planner::EdgeEvaluation step_edge =
    step_evaluator.evaluateEdge({0.2, 0.0}, {1.2, 0.0});
  assert(!step_edge.valid);
  assert(step_edge.reason == planner::TerrainInvalidReason::kStepLimit);
  assert(step_edge.max_step_m > step_parameters.max_step_height_m);
}

void checkFootprintStepGate()
{
  std::vector<planner::TerrainPoint> points;
  for (std::size_t iy = 0U; iy < 11U; ++iy) {
    for (std::size_t ix = 0U; ix < 11U; ++ix) {
      const double x = -0.5 + static_cast<double>(ix) * 0.1;
      const double y = -0.5 + static_cast<double>(iy) * 0.1;
      const bool raised_side_row = std::abs(std::abs(y) - 0.1) < 1.0e-9;
      points.push_back({x, y, raised_side_row ? 0.20 : 0.0});
    }
  }
  const planner::TerrainSnapshot snapshot =
    planner::TerrainSnapshot::fromPoints(points, 0.1, 1.0e-6);
  auto parameters = permissiveTerrainParameters();
  parameters.footprint_radius_m = 0.20;
  parameters.min_footprint_observed_ratio = 1.0;
  parameters.max_step_height_m = 0.08;
  const planner::TerrainEvaluator evaluator(snapshot, parameters);

  const planner::EdgeEvaluation edge =
    evaluator.evaluateEdge({-0.3, 0.0}, {0.3, 0.0});
  assert(!edge.valid);
  assert(edge.reason == planner::TerrainInvalidReason::kStepLimit);
  assert(edge.max_step_m > parameters.max_step_height_m);
}

void checkWavefrontPlan()
{
  const planner::TerrainSnapshot snapshot = planner::TerrainSnapshot::fromPoints(
    makeTerrain(31, 31, 0.1, 0.0), 0.1, 1.0e-6);
  auto terrain_parameters = permissiveTerrainParameters();
  terrain_parameters.footprint_radius_m = 0.1;
  terrain_parameters.min_footprint_observed_ratio = 1.0;
  const planner::TerrainEvaluator evaluator(snapshot, terrain_parameters);

  planner::WavefrontPlannerParameters planner_parameters;
  planner_parameters.node_sampling_distance_m = 0.5;
  planner_parameters.num_expansion_samples = 12;
  planner_parameters.merge_radius_m = 0.20;
  planner_parameters.neighbor_connection_radius_m = 0.75;
  planner_parameters.max_nodes = 500;
  planner_parameters.max_expansions = 500;
  planner_parameters.max_build_time_ms = 2000;
  planner_parameters.goal_connection_distance_m = 0.75;
  planner_parameters.stop_when_goal_connected = true;
  planner_parameters.slope_normalization_deg = 15.0;

  const planner::WavefrontPlanner wavefront(planner_parameters);
  const planner::PlanResult first =
    wavefront.plan(evaluator, {0.5, 0.0}, {2.5, 0.0});
  const planner::PlanResult second =
    wavefront.plan(evaluator, {0.5, 0.0}, {2.5, 0.0});

  assert(first.success);
  assert(first.path.size() >= 2U);
  assert(first.path.size() == second.path.size());
  assert(first.nodes.size() == second.nodes.size());
  assert(first.edges.size() == second.edges.size());
  for (std::size_t i = 1; i < first.path.size(); ++i) {
    const planner::EdgeEvaluation edge = evaluator.evaluateEdge(
      {first.path[i - 1].x, first.path[i - 1].y},
      {first.path[i].x, first.path[i].y});
    assert(edge.valid);
  }
}

void checkWavefrontRoutesAroundUnknownBarrier()
{
  std::vector<planner::TerrainPoint> points;
  for (std::size_t iy = 0; iy < 61U; ++iy) {
    for (std::size_t ix = 0; ix < 61U; ++ix) {
      const double x = static_cast<double>(ix) * 0.1;
      const double y = -3.0 + static_cast<double>(iy) * 0.1;
      const bool wall_cell = ix >= 29U && ix <= 31U;
      const bool inside_gap = std::abs(y - 1.0) <= 0.65;
      if (wall_cell && !inside_gap) {
        continue;
      }
      points.push_back({x, y, 0.0});
    }
  }

  const planner::TerrainSnapshot snapshot =
    planner::TerrainSnapshot::fromPoints(points, 0.1, 1.0e-6);
  auto terrain_parameters = permissiveTerrainParameters();
  terrain_parameters.footprint_radius_m = 0.10;
  terrain_parameters.min_footprint_observed_ratio = 1.0;
  const planner::TerrainEvaluator terrain(snapshot, terrain_parameters);

  planner::WavefrontPlannerParameters wavefront_parameters;
  wavefront_parameters.node_sampling_distance_m = 0.5;
  wavefront_parameters.num_expansion_samples = 12;
  wavefront_parameters.merge_radius_m = 0.20;
  wavefront_parameters.neighbor_connection_radius_m = 0.75;
  wavefront_parameters.max_nodes = 2000;
  wavefront_parameters.max_expansions = 2000;
  wavefront_parameters.max_build_time_ms = 2000;
  wavefront_parameters.goal_connection_distance_m = 0.75;
  wavefront_parameters.stop_when_goal_connected = true;
  wavefront_parameters.slope_normalization_deg = 15.0;

  const planner::WavefrontPlanner wavefront(wavefront_parameters);
  const planner::PlanResult result =
    wavefront.plan(terrain, {0.5, 0.0}, {5.5, 0.0});
  assert(result.success);

  double maximum_y = -std::numeric_limits<double>::infinity();
  for (const planner::TerrainPoint & path_point : result.path) {
    maximum_y = std::max(maximum_y, path_point.y);
  }
  assert(maximum_y > 0.30);
  for (std::size_t index = 1U; index < result.path.size(); ++index) {
    assert(
      terrain.evaluateEdge(
        {result.path[index - 1U].x, result.path[index - 1U].y},
        {result.path[index].x, result.path[index].y}).valid);
  }
}

void checkRrtStarPlan()
{
  const planner::TerrainSnapshot snapshot = planner::TerrainSnapshot::fromPoints(
    makeTerrain(31, 31, 0.1, 0.0), 0.1, 1.0e-6);
  auto terrain_parameters = permissiveTerrainParameters();
  terrain_parameters.footprint_radius_m = 0.1;
  terrain_parameters.min_footprint_observed_ratio = 1.0;
  const planner::TerrainEvaluator terrain(snapshot, terrain_parameters);

  planner::RrtStarParameters parameters;
  parameters.max_iterations = 800U;
  parameters.goal_bias = 0.10;
  parameters.steer_distance_m = 0.50;
  parameters.rewire_radius_min_m = 0.30;
  parameters.rewire_radius_max_m = 1.00;
  parameters.goal_connection_distance_m = 0.60;
  parameters.max_nodes = 1000U;
  parameters.max_planning_time_ms = 0U;
  parameters.stop_on_first_solution = false;
  parameters.random_seed = 42U;
  parameters.slope_normalization_deg = 15.0;

  const planner::RrtStarPlanner rrt_star(parameters);
  const planner::PlanResult first =
    rrt_star.plan(terrain, {0.5, 0.0}, {2.5, 0.0});
  const planner::PlanResult second =
    rrt_star.plan(terrain, {0.5, 0.0}, {2.5, 0.0});

  assert(first.success);
  assert(first.path_node_ids == second.path_node_ids);
  assert(first.nodes.size() == second.nodes.size());
  assert(first.edges.size() == second.edges.size());
  assert(first.expansions == parameters.max_iterations);
  assert(first.rewires > 0U);
  assert(first.rewires == second.rewires);
  for (std::size_t index = 1U; index < first.path.size(); ++index) {
    assert(
      terrain.evaluateEdge(
        {first.path[index - 1U].x, first.path[index - 1U].y},
        {first.path[index].x, first.path[index].y}).valid);
  }
}

}  // namespace

int main()
{
  checkPcaSlope();
  checkUnknownAndStepGates();
  checkFootprintStepGate();
  checkWavefrontPlan();
  checkWavefrontRoutesAroundUnknownBarrier();
  checkRrtStarPlan();
  std::cout << "core_smoke: PASS\n";
  return 0;
}
