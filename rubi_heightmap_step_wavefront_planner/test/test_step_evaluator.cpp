#include <cmath>
#include <functional>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "rubi_heightmap_step_wavefront_planner/step_evaluator.hpp"

namespace planner = rubi_heightmap_step_wavefront_planner;

std::vector<planner::HeightPoint> grid(
  const std::function<double(int, int)> & elevation,
  const std::function<bool(int, int)> & include = [] (int, int) {return true;})
{
  std::vector<planner::HeightPoint> points;
  for (int y = -12; y <= 12; ++y) {
    for (int x = -20; x <= 20; ++x) {
      if (include(x, y)) {points.push_back({0.05 * x, 0.05 * y, elevation(x, y)});}
    }
  }
  return points;
}

planner::StepEvaluatorParameters parameters(double clearance = 0.0)
{
  planner::StepEvaluatorParameters value;
  value.hard_clearance_radius_m = clearance;
  return value;
}

TEST(StepEvaluator, SobelHardRejectRequiresExplicitOptIn)
{
  const planner::StepEvaluatorParameters defaults;
  EXPECT_FALSE(defaults.sobel_hard_reject_enabled);
  EXPECT_DOUBLE_EQ(defaults.sobel_equivalent_step_height_m, 0.10);
  EXPECT_DOUBLE_EQ(defaults.sobel_cost_weight, 0.0);
  EXPECT_DOUBLE_EQ(defaults.sobel_cost_exponent, 2.0);
  EXPECT_FALSE(defaults.local_relief_hard_reject_enabled);
  EXPECT_DOUBLE_EQ(defaults.local_relief_threshold_m, 0.10);
  EXPECT_DOUBLE_EQ(defaults.local_relief_first_window_radius_m, 0.10);
  EXPECT_DOUBLE_EQ(defaults.local_relief_second_window_radius_m, 0.10);
  EXPECT_DOUBLE_EQ(defaults.local_relief_lower_quantile, 0.10);
  EXPECT_DOUBLE_EQ(defaults.local_relief_upper_quantile, 0.90);
  EXPECT_DOUBLE_EQ(defaults.local_relief_min_observed_ratio, 0.70);
  EXPECT_EQ(defaults.local_relief_critical_cell_count, 3U);
}

TEST(StepEvaluator, RejectsInvalidLocalReliefParameters)
{
  const auto snapshot = planner::HeightmapSnapshot::fromPoints(
    grid([](int, int) {return 0.0;}), 0.05, 0.01, 10000U);
  const auto rejected = [&snapshot](planner::StepEvaluatorParameters p) {
      EXPECT_THROW(planner::StepEvaluator(snapshot, p), std::invalid_argument);
    };
  auto p = parameters();
  p.local_relief_threshold_m = 0.0; rejected(p);
  p = parameters(); p.local_relief_first_window_radius_m = 0.0; rejected(p);
  p = parameters(); p.local_relief_second_window_radius_m = -0.01; rejected(p);
  p = parameters(); p.local_relief_lower_quantile = -0.01; rejected(p);
  p = parameters(); p.local_relief_upper_quantile = 1.01; rejected(p);
  p = parameters(); p.local_relief_lower_quantile = 0.90;
  p.local_relief_upper_quantile = 0.90; rejected(p);
  p = parameters(); p.local_relief_min_observed_ratio = 0.0; rejected(p);
  p = parameters(); p.local_relief_critical_cell_count = 0U; rejected(p);
  p = parameters(); p.local_relief_threshold_m =
    std::numeric_limits<double>::quiet_NaN(); rejected(p);
}

TEST(StepEvaluator, FlatEdgeUsesMetricLengthOnly)
{
  const auto snapshot = planner::HeightmapSnapshot::fromPoints(
    grid([](int, int) {return 0.0;}), 0.05, 0.01, 10000U);
  const planner::StepEvaluator evaluator(snapshot, parameters());
  const auto edge = evaluator.evaluateEdge({-0.40, 0.0}, {0.40, 0.0});
  ASSERT_TRUE(edge.valid);
  EXPECT_NEAR(edge.length_xy_m, 0.80, 1e-12);
  EXPECT_EQ(edge.height_jump_event_count, 0U);
  EXPECT_DOUBLE_EQ(edge.height_jump_score_m, 0.0);
  EXPECT_DOUBLE_EQ(edge.max_sobel_gradient, 0.0);
  EXPECT_DOUBLE_EQ(edge.max_sobel_equivalent_step_height_m, 0.0);
  EXPECT_FALSE(edge.sobel_hard_rejection);
  EXPECT_NEAR(edge.cost, 0.80, 1e-12);
}

TEST(StepEvaluator, AccumulatesCrossableHeightEventsWithContractedCost)
{
  const auto one_step = planner::HeightmapSnapshot::fromPoints(
    grid([](int x, int) {return x >= 0 ? 0.045 : 0.0;}), 0.05, 0.01, 10000U);
  const planner::StepEvaluator evaluator(one_step, parameters());
  const auto edge = evaluator.evaluateEdge({-0.40, 0.0}, {0.40, 0.0});
  ASSERT_TRUE(edge.valid);
  EXPECT_EQ(edge.height_jump_event_count, 1U);
  EXPECT_NEAR(edge.height_jump_score_m, 0.02, 1e-12);
  EXPECT_NEAR(edge.cost, 0.90, 1e-12);

  const auto two_steps = planner::HeightmapSnapshot::fromPoints(
    grid([](int x, int) {return x >= -4 && x < 4 ? 0.045 : 0.0;}),
    0.05, 0.01, 10000U);
  const planner::StepEvaluator evaluator_two(two_steps, parameters());
  const auto twice = evaluator_two.evaluateEdge({-0.40, 0.0}, {0.40, 0.0});
  ASSERT_TRUE(twice.valid);
  EXPECT_EQ(twice.height_jump_event_count, 2U);
  EXPECT_NEAR(twice.height_jump_score_m, 0.04, 1e-12);
}

TEST(StepEvaluator, ExactLimitIsCrossableAndOverLimitIsRejected)
{
  const auto exact = planner::HeightmapSnapshot::fromPoints(
    grid([](int x, int) {return x >= 0 ? 0.08 : 0.0;}), 0.05, 0.01, 10000U);
  const auto exact_edge = planner::StepEvaluator(exact, parameters()).evaluateEdge(
    {-0.40, 0.0}, {0.40, 0.0});
  ASSERT_TRUE(exact_edge.valid);
  EXPECT_NEAR(exact_edge.height_jump_score_m, 0.08, 1e-12);
  EXPECT_NEAR(exact_edge.cost, 1.20, 1e-12);

  const auto over = planner::HeightmapSnapshot::fromPoints(
    grid([](int x, int) {return x >= 0 ? 0.080001 : 0.0;}), 0.05, 0.01, 10000U);
  const auto over_edge = planner::StepEvaluator(over, parameters()).evaluateEdge(
    {-0.40, 0.0}, {0.40, 0.0});
  EXPECT_FALSE(over_edge.valid);
  EXPECT_EQ(over_edge.reason, planner::StepInvalidReason::kStepLimit);
}

TEST(StepEvaluator, SobelRejectsSmearedFifteenCentimeterStepMissedByAdjacentJump)
{
  const auto smeared = planner::HeightmapSnapshot::fromPoints(
    grid([](int x, int) {
      if (x <= -2) {return 0.0;}
      if (x == -1) {return 0.03;}
      if (x == 0) {return 0.09;}
      return 0.15;
    }), 0.05, 0.01, 10000U);

  auto baseline = parameters();
  baseline.max_crossable_height_jump_m = 0.10;
  baseline.sobel_hard_reject_enabled = false;
  const auto baseline_edge = planner::StepEvaluator(smeared, baseline).evaluateEdge(
    {-0.40, 0.0}, {0.40, 0.0});
  ASSERT_TRUE(baseline_edge.valid);
  EXPECT_LT(baseline_edge.max_height_jump_m, 0.10);
  EXPECT_GT(baseline_edge.max_sobel_equivalent_step_height_m, 0.10);

  auto sobel = baseline;
  sobel.sobel_hard_reject_enabled = true;
  sobel.sobel_equivalent_step_height_m = 0.10;
  const auto sobel_edge = planner::StepEvaluator(smeared, sobel).evaluateEdge(
    {-0.40, 0.0}, {0.40, 0.0});
  EXPECT_FALSE(sobel_edge.valid);
  EXPECT_TRUE(sobel_edge.sobel_hard_rejection);
  EXPECT_GT(sobel_edge.max_sobel_equivalent_step_height_m, 0.10);
  EXPECT_EQ(sobel_edge.reason, planner::StepInvalidReason::kStepLimit);
}

TEST(StepEvaluator, SobelKeepsSharpFiveCentimeterStepCrossableAtTenCentimeterThreshold)
{
  const auto step = planner::HeightmapSnapshot::fromPoints(
    grid([](int x, int) {return x >= 0 ? 0.05 : 0.0;}), 0.05, 0.01, 10000U);
  auto p = parameters();
  p.max_crossable_height_jump_m = 0.10;
  p.sobel_hard_reject_enabled = true;
  p.sobel_equivalent_step_height_m = 0.10;
  const auto edge = planner::StepEvaluator(step, p).evaluateEdge(
    {-0.40, 0.0}, {0.40, 0.0});
  ASSERT_TRUE(edge.valid);
  EXPECT_FALSE(edge.sobel_hard_rejection);
  EXPECT_NEAR(edge.max_sobel_equivalent_step_height_m, 0.05, 1e-12);
}

TEST(StepEvaluator, LocalReliefRejectsSmearedFifteenCentimeterStepMissedByAdjacentAndSobel)
{
  const auto smeared = planner::HeightmapSnapshot::fromPoints(
    grid([](int x, int y) {
      const int abs_y = std::abs(y);
      const int shifted_boundary = abs_y == 0 || abs_y == 3 ? 0 : (abs_y <= 2 ? -2 : 1);
      const int phase = x - shifted_boundary;
      if (phase <= -3) {return 0.0;}
      if (phase >= 2) {return 0.15;}
      return 0.03 * static_cast<double>(phase + 3);
    }), 0.05, 0.01, 10000U);

  auto baseline = parameters();
  baseline.max_crossable_height_jump_m = 0.10;
  baseline.sobel_hard_reject_enabled = true;
  baseline.sobel_equivalent_step_height_m = 0.10;
  const auto baseline_edge = planner::StepEvaluator(smeared, baseline).evaluateEdge(
    {-0.40, 0.0}, {0.40, 0.0});
  ASSERT_TRUE(baseline_edge.valid);
  EXPECT_LT(baseline_edge.max_height_jump_m, 0.10);
  EXPECT_LT(baseline_edge.max_sobel_equivalent_step_height_m, 0.10);

  auto relief = baseline;
  relief.local_relief_hard_reject_enabled = true;
  const auto relief_edge = planner::StepEvaluator(smeared, relief).evaluateEdge(
    {-0.40, 0.0}, {0.40, 0.0});
  EXPECT_FALSE(relief_edge.valid);
  EXPECT_EQ(relief_edge.reason, planner::StepInvalidReason::kStepLimit);
  EXPECT_TRUE(relief_edge.local_relief_hard_rejection);
  EXPECT_GT(relief_edge.max_supported_local_relief_m, 0.10);
  EXPECT_GE(relief_edge.local_relief_max_critical_count, 3U);
}

TEST(StepEvaluator, LocalReliefKeepsSharpFiveCentimeterStepAndFlatTerrainValid)
{
  auto p = parameters();
  p.max_crossable_height_jump_m = 0.10;
  p.local_relief_hard_reject_enabled = true;
  const auto step = planner::HeightmapSnapshot::fromPoints(
    grid([](int x, int) {return x >= 0 ? 0.05 : 0.0;}), 0.05, 0.01, 10000U);
  const auto step_edge = planner::StepEvaluator(step, p).evaluateEdge(
    {-0.40, 0.0}, {0.40, 0.0});
  ASSERT_TRUE(step_edge.valid);
  EXPECT_LE(step_edge.max_local_relief_m, 0.05 + 1e-12);
  EXPECT_FALSE(step_edge.local_relief_hard_rejection);

  const auto flat = planner::HeightmapSnapshot::fromPoints(
    grid([](int, int) {return 0.0;}), 0.05, 0.01, 10000U);
  const auto flat_edge = planner::StepEvaluator(flat, p).evaluateEdge(
    {-0.40, 0.0}, {0.40, 0.0});
  ASSERT_TRUE(flat_edge.valid);
  EXPECT_DOUBLE_EQ(flat_edge.max_local_relief_m, 0.0);
  EXPECT_DOUBLE_EQ(flat_edge.max_supported_local_relief_m, 0.0);
  EXPECT_FALSE(flat_edge.local_relief_hard_rejection);
}

TEST(StepEvaluator, LocalReliefQuantilesIgnoreOneIsolatedOutlier)
{
  const auto outlier = planner::HeightmapSnapshot::fromPoints(
    grid([](int x, int y) {return x == 0 && y == 1 ? 0.15 : 0.0;}),
    0.05, 0.01, 10000U);
  auto p = parameters();
  p.max_crossable_height_jump_m = 0.20;
  p.local_relief_hard_reject_enabled = true;
  const auto edge = planner::StepEvaluator(outlier, p).evaluateEdge(
    {-0.40, 0.0}, {0.40, 0.0});
  ASSERT_TRUE(edge.valid);
  EXPECT_FALSE(edge.local_relief_hard_rejection);
  EXPECT_LE(edge.max_supported_local_relief_m, 0.10);
}

TEST(StepEvaluator, LocalReliefCreatesSpatiallySupportedExtendedBoundary)
{
  const auto boundary = planner::HeightmapSnapshot::fromPoints(
    grid([](int x, int y) {
      const int abs_y = std::abs(y);
      const int shifted_boundary = abs_y == 0 || abs_y == 3 ? 0 : (abs_y <= 2 ? -2 : 1);
      const int phase = x - shifted_boundary;
      if (phase <= -3) {return 0.0;}
      if (phase >= 2) {return 0.15;}
      return 0.03 * static_cast<double>(phase + 3);
    }), 0.05, 0.01, 10000U);
  auto p = parameters();
  p.max_crossable_height_jump_m = 0.10;
  p.local_relief_hard_reject_enabled = true;
  for (const double y : {-0.10, 0.0, 0.10}) {
    const auto edge = planner::StepEvaluator(boundary, p).evaluateEdge(
      {-0.30, y}, {0.30, y});
    EXPECT_FALSE(edge.valid);
    EXPECT_TRUE(edge.local_relief_hard_rejection);
    EXPECT_GT(edge.max_supported_local_relief_m, 0.10);
  }
}

TEST(StepEvaluator, LocalReliefObservedRatioControlsMissingNeighborhoods)
{
  auto p = parameters();
  p.hard_clearance_radius_m = 0.0;
  p.local_relief_hard_reject_enabled = true;
  p.local_relief_second_window_radius_m = 0.0;

  const auto case_a = planner::HeightmapSnapshot::fromPoints(
    grid(
      [](int x, int) {return x > 0 ? 0.05 : 0.0;},
      [](int x, int y) {
        return !((x == -2 && y == 0) || (x == 2 && y == 0) || (x == 0 && y == 2));
      }), 0.05, 0.01, 10000U);
  const auto valid = planner::StepEvaluator(case_a, p).evaluateEdge({0.0, 0.0}, {0.0, 0.0});
  ASSERT_TRUE(valid.valid);
  EXPECT_EQ(valid.local_relief_valid_cell_count, 1U);
  EXPECT_EQ(valid.local_relief_missing_cell_count, 0U);

  const auto case_b = planner::HeightmapSnapshot::fromPoints(
    grid(
      [](int x, int) {return x > 0 ? 0.05 : 0.0;},
      [](int x, int y) {
        return !((x == -2 && y == 0) || (x == 2 && y == 0) ||
          (x == 0 && y == 2) || (x == 0 && y == -2));
      }), 0.05, 0.01, 10000U);
  planner::StepEvaluator evaluator(case_b, p);
  const auto missing = evaluator.evaluateEdge({0.0, 0.0}, {0.0, 0.0});
  ASSERT_TRUE(missing.valid);
  EXPECT_EQ(missing.local_relief_valid_cell_count, 0U);
  EXPECT_EQ(missing.local_relief_missing_cell_count, 1U);
  EXPECT_GT(evaluator.instrumentation().local_relief_missing_neighborhoods, 0U);
}

TEST(StepEvaluator, LocalReliefCachesBothWindowPassesPerRequest)
{
  const auto snapshot = planner::HeightmapSnapshot::fromPoints(
    grid([](int x, int) {return 0.01 * static_cast<double>(x);}),
    0.05, 0.01, 10000U);
  planner::StepEvaluator evaluator(snapshot, parameters());
  ASSERT_TRUE(evaluator.evaluateEdge({-0.40, 0.0}, {0.40, 0.0}).valid);
  const auto first = evaluator.instrumentation();
  ASSERT_TRUE(evaluator.evaluateEdge({-0.40, 0.0}, {0.40, 0.0}).valid);
  const auto second = evaluator.instrumentation();
  EXPECT_GT(first.local_relief_cache_hits, 0U);
  EXPECT_EQ(second.local_relief_cache_hits, first.local_relief_cache_hits);
  EXPECT_GT(second.supported_relief_queries, first.supported_relief_queries);
  EXPECT_EQ(second.local_relief_queries, first.local_relief_queries);
}

TEST(StepEvaluator, CostIsSymmetricAndSegmentationInvariant)
{
  const auto snapshot = planner::HeightmapSnapshot::fromPoints(
    grid([](int x, int) {return x >= 0 ? 0.045 : 0.0;}), 0.05, 0.01, 10000U);
  const planner::StepEvaluator evaluator(snapshot, parameters());
  const auto forward = evaluator.evaluateEdge({-0.40, 0.0}, {0.40, 0.0});
  const auto reverse = evaluator.evaluateEdge({0.40, 0.0}, {-0.40, 0.0});
  const auto left = evaluator.evaluateEdge({-0.40, 0.0}, {-0.05, 0.0});
  const auto right = evaluator.evaluateEdge({-0.05, 0.0}, {0.40, 0.0});
  ASSERT_TRUE(forward.valid && reverse.valid && left.valid && right.valid);
  EXPECT_NEAR(forward.cost, reverse.cost, 1e-12);
  EXPECT_NEAR(forward.cost, left.cost + right.cost, 1e-12);
}

TEST(StepEvaluator, EnforcesUnknownClearanceDiscontinuityCornerAndBoundary)
{
  auto missing_center = grid(
    [](int, int) {return 0.0;}, [](int x, int y) {return x != 0 || y != 0;});
  const auto center_snapshot = planner::HeightmapSnapshot::fromPoints(
    missing_center, 0.05, 0.01, 10000U);
  EXPECT_EQ(
    planner::StepEvaluator(center_snapshot, parameters()).evaluateNode({0.0, 0.0}).reason,
    planner::StepInvalidReason::kUnknown);

  const auto support = planner::StepEvaluator(center_snapshot, parameters(0.10)).evaluateNode(
    {0.10, 0.0});
  EXPECT_EQ(support.reason, planner::StepInvalidReason::kInsufficientClearanceSupport);

  const auto high = planner::HeightmapSnapshot::fromPoints(
    grid([](int x, int y) {return x == 1 && y == 0 ? 0.09 : 0.0;}),
    0.05, 0.01, 10000U);
  EXPECT_EQ(
    planner::StepEvaluator(high, parameters(0.10)).evaluateNode({0.0, 0.0}).reason,
    planner::StepInvalidReason::kClearanceViolation);

  const auto diagonal = planner::HeightmapSnapshot::fromPoints(
    grid([](int, int) {return 0.0;}, [](int x, int y) {return x != 0 || y != 1;}),
    0.05, 0.01, 10000U);
  EXPECT_FALSE(
    planner::StepEvaluator(diagonal, parameters()).evaluateEdge(
      {-0.05, 0.0}, {0.0, 0.05}).valid);

  const auto flat = planner::HeightmapSnapshot::fromPoints(
    grid([](int, int) {return 0.0;}), 0.05, 0.01, 10000U);
  EXPECT_EQ(
    planner::StepEvaluator(flat, parameters(0.20)).evaluateNode({1.0, 0.0}).reason,
    planner::StepInvalidReason::kInsufficientClearanceSupport);
}

TEST(StepEvaluator, OptionalPreferredClearanceAddsOnlySoftNonnegativeRisk)
{
  const auto snapshot = planner::HeightmapSnapshot::fromPoints(
    grid([](int x, int) {return x >= 0 ? 0.081 : 0.0;}), 0.05, 0.01, 10000U);
  auto baseline_parameters = parameters(0.05);
  baseline_parameters.preferred_clearance_radius_m = 0.05;
  const auto baseline = planner::StepEvaluator(snapshot, baseline_parameters).evaluateEdge(
    {-0.20, -0.30}, {-0.20, 0.30});
  ASSERT_TRUE(baseline.valid);
  EXPECT_DOUBLE_EQ(baseline.clearance_score_m, 0.0);
  EXPECT_NEAR(baseline.cost, baseline.length_xy_m, 1.0e-12);

  auto safe_parameters = baseline_parameters;
  safe_parameters.preferred_clearance_radius_m = 0.30;
  safe_parameters.clearance_cost_weight = 5.0;
  const auto near_wall = planner::StepEvaluator(snapshot, safe_parameters).evaluateEdge(
    {-0.20, -0.30}, {-0.20, 0.30});
  const auto far_from_wall = planner::StepEvaluator(snapshot, safe_parameters).evaluateEdge(
    {-0.50, -0.30}, {-0.50, 0.30});
  ASSERT_TRUE(near_wall.valid && far_from_wall.valid);
  EXPECT_GT(near_wall.clearance_score_m, 0.0);
  EXPECT_GT(near_wall.cost, baseline.cost);
  EXPECT_DOUBLE_EQ(far_from_wall.clearance_score_m, 0.0);
  EXPECT_LT(near_wall.minimum_clearance_m, safe_parameters.preferred_clearance_radius_m);
}
