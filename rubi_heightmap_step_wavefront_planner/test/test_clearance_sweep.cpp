#include <iomanip>
#include <iostream>
#include <vector>

#include <gtest/gtest.h>
#include "rubi_heightmap_step_wavefront_planner/planning/step_wavefront_planner.hpp"

namespace planner = rubi_heightmap_step_wavefront_planner;

TEST(ClearanceSweep, ReportsPhaseOneHardRadiusTradeoff)
{
  std::vector<planner::HeightPoint> points;
  for (int y = -30; y <= 30; ++y) {
    for (int x = -30; x <= 30; ++x) {
      const bool raised_block = x >= 0 && x <= 3 && y >= -5 && y <= 5;
      points.push_back({0.05 * x, 0.05 * y, raised_block ? 0.081 : 0.0});
    }
  }
  const auto snapshot = planner::HeightmapSnapshot::fromPoints(
    points, 0.05, 0.01, 100000U);
  std::size_t successful_runs = 0U;
  for (const double hard_radius_m : {0.20, 0.25, 0.30, 0.35}) {
    planner::StepEvaluatorParameters evaluator_parameters;
    evaluator_parameters.hard_clearance_radius_m = hard_radius_m;
    evaluator_parameters.preferred_clearance_radius_m = hard_radius_m;
    planner::StepWavefrontParameters graph_parameters;
    graph_parameters.max_nodes = 1200U;
    graph_parameters.max_expansions = 1200U;
    graph_parameters.post_goal_expansions = 50U;
    const auto result = planner::StepWavefrontPlanner(graph_parameters).plan(
      planner::StepEvaluator(snapshot, evaluator_parameters), {-1.0, 0.0}, {1.0, 0.0});
    successful_runs += result.success ? 1U : 0U;
    std::cout << std::fixed << std::setprecision(3)
              << "SWEEP hard_radius_m=" << hard_radius_m
              << " success=" << (result.success ? "true" : "false")
              << " nodes=" << result.nodes.size()
              << " edges=" << result.edges.size()
              << " path_length_m=" << result.path_metrics.length_xy_m
              << " total_cost=" << result.path_metrics.total_cost
              << " min_clearance_m=" << result.path_metrics.minimum_clearance_m
              << " graph_ms=" << result.graph_build_time_ms
              << " astar_ms=" << result.astar_time_ms << '\n';
  }
  EXPECT_GT(successful_runs, 0U);
}
