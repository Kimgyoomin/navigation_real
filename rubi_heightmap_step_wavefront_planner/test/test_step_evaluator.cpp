#include <cmath>
#include <functional>
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
