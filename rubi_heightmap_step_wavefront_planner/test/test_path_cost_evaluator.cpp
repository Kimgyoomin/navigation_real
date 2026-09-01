#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "rubi_heightmap_step_wavefront_planner/planning/path_cost_evaluator.hpp"

namespace planner = rubi_heightmap_step_wavefront_planner;

namespace
{
planner::HeightmapSnapshot heights(const double right_z = 0.0)
{
  std::vector<planner::HeightPoint> points;
  for (int y = 0; y < 9; ++y) {
    for (int x = 0; x < 13; ++x) {
      points.push_back({0.025 + 0.05 * x, 0.025 + 0.05 * y, x >= 7 ? right_z : 0.0});
    }
  }
  return planner::HeightmapSnapshot::fromPoints(points, 0.05, 0.001, 1000U);
}

planner::StepEvaluatorParameters parameters()
{
  planner::StepEvaluatorParameters value;
  value.node_min_observed_cells = 1U;
  value.node_max_nearest_evidence_distance_m = 0.08;
  value.edge_height_query_radius_m = 0.04;
  value.edge_max_height_evidence_gap_m = 0.08;
  return value;
}
}  // namespace

TEST(PathCostEvaluator, AccumulatesTheSameHybridEdgeCostComponents)
{
  const auto heightmap = heights(0.05);
  std::vector<std::uint8_t> raw(13U * 9U, 0U);
  raw[4U * 13U + 5U] = 126U;
  const auto costmap = planner::CostmapSnapshot::fromData(
    13U, 9U, 0.05, 0.0, 0.0, raw);
  const planner::StepEvaluator evaluator(heightmap, costmap, parameters());
  const std::vector<planner::TerrainPoint> path{
    {0.125, 0.225, 0.0}, {0.275, 0.225, 0.0}, {0.475, 0.225, 0.05}};
  const auto evaluated = planner::evaluatePolyline(path, 0U, evaluator);
  ASSERT_TRUE(evaluated.valid);
  EXPECT_GT(evaluated.length_xy_m, 0.0);
  EXPECT_GT(evaluated.inflation_cost, 0.0);
  EXPECT_GT(evaluated.height_cost, 0.0);
  EXPECT_NEAR(evaluated.total_cost,
    parameters().distance_weight * evaluated.length_xy_m +
    evaluated.inflation_cost + evaluated.height_cost, 1.0e-12);
}

TEST(PathCostEvaluator, ReportsHardStepAndCostmapFailuresAtTheIncidentSegment)
{
  const std::vector<planner::TerrainPoint> path{
    {0.125, 0.225, 0.0}, {0.275, 0.225, 0.0}, {0.475, 0.225, 0.10}};
  std::vector<std::uint8_t> raw(13U * 9U, 0U);
  auto costmap = planner::CostmapSnapshot::fromData(13U, 9U, 0.05, 0.0, 0.0, raw);
  auto heightmap = heights(0.10);
  planner::StepEvaluator step_evaluator(heightmap, costmap, parameters());
  const auto step = planner::evaluatePolyline(path, 0U, step_evaluator);
  EXPECT_FALSE(step.valid);
  EXPECT_EQ(step.reason, planner::StepInvalidReason::kStepLimit);
  EXPECT_EQ(step.failing_segment, 1U);

  raw[4U * 13U + 5U] = 254U;
  costmap = planner::CostmapSnapshot::fromData(13U, 9U, 0.05, 0.0, 0.0, raw);
  heightmap = heights();
  planner::StepEvaluator blocked(heightmap, costmap, parameters());
  const auto collision = planner::evaluatePolyline(path, 0U, blocked);
  EXPECT_FALSE(collision.valid);
  EXPECT_EQ(collision.reason, planner::StepInvalidReason::kCostmapCollision);
}

TEST(PathCostEvaluator, MinimumImprovementHysteresisUsesTheExactConfiguredRatio)
{
  EXPECT_FALSE(planner::hasMinimumCostImprovement(100.0, 95.1, 0.05));
  EXPECT_TRUE(planner::hasMinimumCostImprovement(100.0, 95.0, 0.05));
  EXPECT_TRUE(planner::hasMinimumCostImprovement(100.0, 90.0, 0.05));
  EXPECT_FALSE(planner::hasMinimumCostImprovement(100.0, 100.0, 0.0));
  EXPECT_FALSE(planner::hasMinimumCostImprovement(0.0, 0.0, 0.05));
}
