#include <cmath>
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
  EXPECT_GT(evaluator.instrumentation().edge_samples_total, 0U);
  EXPECT_EQ(evaluator.instrumentation().grid_transition_evaluations, 0U);
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

TEST(HybridStepEvaluator, DirectGridTransitionUsesCellGeometryAndNoSubcellSamples)
{
  const auto heights = planner::HeightmapSnapshot::fromPoints(
    hybridHeights(), 0.1, 0.001, 100U);
  const auto costs = planner::CostmapSnapshot::fromData(
    9U, 5U, 0.1, 0.0, 0.0, std::vector<std::uint8_t>(45U, 0U));
  planner::StepEvaluator evaluator(heights, costs, hybridParameters());
  const auto from = evaluator.evaluateNode({0.25, 0.25});
  const auto cardinal_to = evaluator.evaluateNode({0.35, 0.25});
  const auto diagonal_to = evaluator.evaluateNode({0.35, 0.35});

  const auto cardinal = evaluator.evaluateGridTransition(
    {2, 2}, {3, 2}, from, cardinal_to);
  ASSERT_TRUE(cardinal.valid);
  EXPECT_NEAR(cardinal.length_xy_m, 0.1, 1e-12);
  EXPECT_EQ(cardinal.sample_count, 2U);
  EXPECT_DOUBLE_EQ(cardinal.cost, 0.1);

  const auto diagonal = evaluator.evaluateGridTransition(
    {2, 2}, {3, 3}, from, diagonal_to);
  ASSERT_TRUE(diagonal.valid);
  EXPECT_NEAR(diagonal.length_xy_m, 0.1 * std::sqrt(2.0), 1e-12);
  EXPECT_DOUBLE_EQ(evaluator.instrumentation().edge_samples_total, 0U);
  EXPECT_EQ(evaluator.instrumentation().grid_transition_evaluations, 2U);
}

TEST(HybridStepEvaluator, DirectGridTransitionPreservesHeightAndInflationCosts)
{
  const auto heights = planner::HeightmapSnapshot::fromPoints(
    hybridHeights(0.08), 0.1, 0.001, 100U);
  std::vector<std::uint8_t> raw(45U, 0U);
  raw[2U * 9U + 4U] = 252U;
  const auto costs = planner::CostmapSnapshot::fromData(
    9U, 5U, 0.1, 0.0, 0.0, raw);
  auto p = hybridParameters();
  p.max_crossable_height_jump_m = 0.08;
  planner::StepEvaluator evaluator(heights, costs, p);
  const auto from = evaluator.evaluateNode({0.35, 0.25});
  const auto to = evaluator.evaluateNode({0.45, 0.25});
  const auto transition = evaluator.evaluateGridTransition({3, 2}, {4, 2}, from, to);
  ASSERT_TRUE(transition.valid);
  EXPECT_NEAR(transition.max_height_jump_m, 0.08, 1e-12);
  EXPECT_NEAR(transition.height_jump_score_m, 0.08, 1e-12);
  EXPECT_NEAR(transition.inflation_score_m, 0.1, 1e-12);
  EXPECT_NEAR(transition.cost, 1.0, 1e-12);

  const auto over_heights = planner::HeightmapSnapshot::fromPoints(
    hybridHeights(0.080001), 0.1, 0.001, 100U);
  planner::StepEvaluator over(over_heights, costs, p);
  const auto over_result = over.evaluateGridTransition(
    {3, 2}, {4, 2}, over.evaluateNode({0.35, 0.25}), over.evaluateNode({0.45, 0.25}));
  EXPECT_FALSE(over_result.valid);
  EXPECT_EQ(over_result.reason, planner::StepInvalidReason::kStepLimit);
}

TEST(HybridStepEvaluator, DirectGridTransitionRejectsUnknownAndObstacleEndpoints)
{
  const auto heights = planner::HeightmapSnapshot::fromPoints(
    hybridHeights(), 0.1, 0.001, 100U);
  std::vector<std::uint8_t> raw(45U, 0U);
  raw[2U * 9U + 3U] = 255U;
  raw[2U * 9U + 4U] = 254U;
  const auto costs = planner::CostmapSnapshot::fromData(
    9U, 5U, 0.1, 0.0, 0.0, raw);
  planner::StepEvaluator evaluator(heights, costs, hybridParameters());
  const auto valid = evaluator.evaluateNode({0.25, 0.25});
  const auto valid_right = evaluator.evaluateNode({0.55, 0.25});
  const auto unknown = evaluator.evaluateNode({0.35, 0.25});
  const auto obstacle = evaluator.evaluateNode({0.45, 0.25});
  EXPECT_EQ(
    evaluator.evaluateGridTransition({2, 2}, {3, 2}, valid, unknown).reason,
    planner::StepInvalidReason::kCostmapUnknown);
  EXPECT_EQ(
    evaluator.evaluateGridTransition({5, 2}, {4, 2}, valid_right, obstacle).reason,
    planner::StepInvalidReason::kCostmapCollision);
  EXPECT_EQ(
    evaluator.evaluateGridTransition({3, 2}, {4, 2}, unknown, obstacle).reason,
    planner::StepInvalidReason::kCostmapUnknown);
}

TEST(HybridStepEvaluator, LocalReliefUsesHeightmapNeighborhoodInHybridMode)
{
  std::vector<planner::HeightPoint> points;
  for (int y = -8; y <= 8; ++y) {
    for (int x = -12; x <= 12; ++x) {
      const int abs_y = std::abs(y);
      const int shifted_boundary = abs_y == 0 || abs_y == 3 ? 0 : (abs_y <= 2 ? -2 : 1);
      const int phase = x - shifted_boundary;
      double z = 0.03 * static_cast<double>(phase + 3);
      if (phase <= -3) {z = 0.0;}
      else if (phase >= 2) {z = 0.15;}
      points.push_back({0.05 * x, 0.05 * y, z});
    }
  }
  const auto heights = planner::HeightmapSnapshot::fromPoints(
    points, 0.05, 0.001, 1000U);
  const auto costs = planner::CostmapSnapshot::fromData(
    25U, 17U, 0.05, -0.625, -0.425,
    std::vector<std::uint8_t>(25U * 17U, 0U));
  auto p = hybridParameters();
  p.max_crossable_height_jump_m = 0.10;
  p.node_evidence_radius_m = 0.051;
  p.node_max_nearest_evidence_distance_m = 0.051;
  p.edge_height_query_radius_m = 0.04;
  p.edge_max_height_evidence_gap_m = 0.051;
  p.sobel_hard_reject_enabled = true;
  p.sobel_equivalent_step_height_m = 0.10;
  p.local_relief_hard_reject_enabled = true;
  const auto edge = planner::StepEvaluator(heights, costs, p).evaluateEdge(
    {-0.40, 0.0}, {0.40, 0.0});
  EXPECT_FALSE(edge.valid);
  EXPECT_EQ(edge.reason, planner::StepInvalidReason::kStepLimit);
  EXPECT_LT(edge.max_sobel_equivalent_step_height_m, 0.10);
  EXPECT_TRUE(edge.local_relief_hard_rejection);
  EXPECT_GT(edge.max_supported_local_relief_m, 0.10);

  planner::StepEvaluator grid_evaluator(heights, costs, p);
  const auto grid_from = grid_evaluator.evaluateNode({-0.05, 0.0});
  const auto grid_to = grid_evaluator.evaluateNode({0.0, 0.0});
  const auto grid_transition = grid_evaluator.evaluateGridTransition(
    {11, 8}, {12, 8}, grid_from, grid_to);
  EXPECT_FALSE(grid_transition.valid);
  EXPECT_EQ(grid_transition.reason, planner::StepInvalidReason::kStepLimit);
  EXPECT_TRUE(grid_transition.local_relief_hard_rejection);
}
