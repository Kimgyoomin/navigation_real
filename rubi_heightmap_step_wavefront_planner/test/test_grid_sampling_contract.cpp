#include <vector>
#include <gtest/gtest.h>
#include "rubi_heightmap_step_wavefront_planner/planning/step_grid_astar_planner.hpp"
#include "rubi_heightmap_step_wavefront_planner/step_wavefront_planner.hpp"
namespace planner = rubi_heightmap_step_wavefront_planner;

TEST(GridSamplingContract, SameHybridEvaluatorProducesHardValidAdditivePaths)
{
  std::vector<planner::HeightPoint> points;
  for (int y = 0; y < 15; ++y) for (int x = 0; x < 25; ++x)
    points.push_back({0.025 + 0.05 * x, 0.025 + 0.05 * y, x == 12 && y == 7 ? 0.06 : 0.0});
  auto height = planner::HeightmapSnapshot::fromPoints(points, 0.05, 0.001, 1000U);
  auto cost = planner::CostmapSnapshot::fromData(
    25U, 15U, 0.05, 0.0, 0.0, std::vector<std::uint8_t>(375U, 0U));
  planner::StepEvaluatorParameters ep;
  ep.node_evidence_radius_m = 0.06;
  ep.node_min_observed_cells = 1U;
  ep.node_max_nearest_evidence_distance_m = 0.04;
  ep.edge_height_query_radius_m = 0.04;
  ep.edge_max_height_evidence_gap_m = 0.06;
  planner::StepEvaluator grid_evaluator(height, cost, ep);
  planner::StepEvaluator sampling_evaluator(height, cost, ep);
  const planner::Point2D start{0.125, 0.375};
  const planner::Point2D goal{1.075, 0.375};
  const auto grid = planner::StepGridAStarPlanner({}).plan(grid_evaluator, start, goal);
  planner::StepWavefrontParameters wp;
  wp.node_sampling_distance_m = 0.15;
  wp.merge_radius_m = 0.08;
  wp.neighbor_connection_radius_m = 0.22;
  wp.goal_connection_distance_m = 0.22;
  wp.post_goal_expansions = 5U;
  const auto sampling = planner::StepWavefrontPlanner(wp).plan(
    sampling_evaluator, start, goal);
  ASSERT_TRUE(grid.success);
  ASSERT_TRUE(sampling.success);
  for (const auto * result : {&grid, &sampling}) {
    EXPECT_NEAR(result->path_metrics.total_cost,
      ep.distance_weight * result->path_metrics.length_xy_m +
      result->path_metrics.inflation_cost + result->path_metrics.height_cost, 1e-9);
    for (const auto & edge : result->edges) EXPECT_TRUE(edge.evaluation.valid);
    EXPECT_LT(result->path_metrics.maximum_raw_cost, 253U);
  }
}
