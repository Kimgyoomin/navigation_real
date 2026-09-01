#include <cstdint>
#include <vector>
#include <gtest/gtest.h>
#include "rubi_heightmap_step_wavefront_planner/step_evaluator.hpp"
namespace planner = rubi_heightmap_step_wavefront_planner;

std::vector<planner::HeightPoint> hybridHeights(double right_z = 0.0, bool gap = false)
{
  std::vector<planner::HeightPoint> points;
  for (int y = 0; y < 5; ++y) for (int x = 0; x < 9; ++x) {
    if (gap && x == 4) continue;
    points.push_back({0.05 + 0.1 * x, 0.05 + 0.1 * y, x >= 4 ? right_z : 0.0});
  }
  return points;
}
planner::StepEvaluatorParameters hybridParameters()
{
  planner::StepEvaluatorParameters p;
  p.node_evidence_radius_m = 0.11;
  p.node_min_observed_cells = 1U;
  p.node_max_nearest_evidence_distance_m = 0.11;
  p.edge_height_query_radius_m = 0.075;
  p.edge_max_height_evidence_gap_m = 0.11;
  return p;
}

TEST(HybridStepEvaluator, RawHardCostsRejectAndInflationIsSoft)
{
  auto heights = planner::HeightmapSnapshot::fromPoints(hybridHeights(), 0.1, 0.001, 100U);
  auto costs = planner::CostmapSnapshot::fromData(5U, 1U, 0.1, 0.0, 0.0,
    {0U, 252U, 253U, 254U, 255U});
  planner::StepEvaluator evaluator(heights, costs, hybridParameters());
  EXPECT_TRUE(evaluator.evaluateNode({0.05, 0.05}).valid);
  EXPECT_TRUE(evaluator.evaluateNode({0.15, 0.05}).valid);
  EXPECT_EQ(evaluator.evaluateNode({0.25, 0.05}).reason, planner::StepInvalidReason::kCostmapCollision);
  EXPECT_EQ(evaluator.evaluateNode({0.35, 0.05}).reason, planner::StepInvalidReason::kCostmapCollision);
  EXPECT_EQ(evaluator.evaluateNode({0.45, 0.05}).reason, planner::StepInvalidReason::kCostmapUnknown);
  auto inflated = evaluator.evaluateEdge({0.05, 0.05}, {0.15, 0.05});
  ASSERT_TRUE(inflated.valid);
  EXPECT_GT(inflated.inflation_score_m, 0.0);
}

TEST(HybridStepEvaluator, MidEdgeObstacleAndStepThresholdsAreHard)
{
  auto heights = planner::HeightmapSnapshot::fromPoints(hybridHeights(0.08), 0.1, 0.001, 100U);
  std::vector<std::uint8_t> raw(45U, 0U);
  auto costs = planner::CostmapSnapshot::fromData(9U, 5U, 0.1, 0.0, 0.0, raw);
  planner::StepEvaluator evaluator(heights, costs, hybridParameters());
  auto exact = evaluator.evaluateEdge({0.25, 0.25}, {0.55, 0.25});
  ASSERT_TRUE(exact.valid);
  EXPECT_GT(exact.height_jump_score_m, 0.0);
  auto over_heights = planner::HeightmapSnapshot::fromPoints(
    hybridHeights(0.080001), 0.1, 0.001, 100U);
  planner::StepEvaluator over(over_heights, costs, hybridParameters());
  EXPECT_EQ(over.evaluateEdge({0.25, 0.25}, {0.55, 0.25}).reason,
    planner::StepInvalidReason::kStepLimit);
  raw[2U * 9U + 4U] = 254U;
  auto blocked_costs = planner::CostmapSnapshot::fromData(9U, 5U, 0.1, 0.0, 0.0, raw);
  planner::StepEvaluator blocked(heights, blocked_costs, hybridParameters());
  EXPECT_EQ(blocked.evaluateEdge({0.25, 0.25}, {0.55, 0.25}).reason,
    planner::StepInvalidReason::kCostmapCollision);
}

TEST(HybridStepEvaluator, SparseCellRecoversButLongEvidenceGapRejects)
{
  auto points = hybridHeights();
  points.erase(points.begin() + 2U * 9U + 4U);
  auto sparse = planner::HeightmapSnapshot::fromPoints(points, 0.1, 0.001, 100U);
  auto costs = planner::CostmapSnapshot::fromData(9U, 5U, 0.1, 0.0, 0.0,
    std::vector<std::uint8_t>(45U, 0U));
  auto p = hybridParameters();
  planner::StepEvaluator evaluator(sparse, costs, p);
  EXPECT_TRUE(evaluator.evaluateNode({0.45, 0.25}).valid);
  auto gap_points = hybridHeights(0.0, true);
  auto gap = planner::HeightmapSnapshot::fromPoints(gap_points, 0.1, 0.001, 100U);
  p.edge_height_query_radius_m = 0.11;
  p.edge_max_height_evidence_gap_m = 0.11;
  planner::StepEvaluator gap_eval(gap, costs, p);
  EXPECT_EQ(gap_eval.evaluateEdge({0.25, 0.25}, {0.65, 0.25}).reason,
    planner::StepInvalidReason::kHeightEvidenceGap);
}
