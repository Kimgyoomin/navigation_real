#include <cmath>
#include <limits>
#include <set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "rubi_heightmap_step_wavefront_planner/planning/planning_query_resolver.hpp"

namespace planner = rubi_heightmap_step_wavefront_planner;

namespace
{

planner::HeightmapSnapshot makeGrid(
  const std::set<std::pair<int, int>> & omitted = {},
  const std::set<std::pair<int, int>> & raised = {})
{
  std::vector<planner::HeightPoint> points;
  for (int y = 0; y <= 10; ++y) {
    for (int x = 0; x <= 10; ++x) {
      if (omitted.count({x, y}) != 0U) {
        continue;
      }
      points.push_back({0.1 * x, 0.1 * y, raised.count({x, y}) ? 0.09 : 0.0});
    }
  }
  return planner::HeightmapSnapshot::fromPoints(points, 0.1, 0.001, 1000U);
}

planner::StepEvaluator makeEvaluator(
  const planner::HeightmapSnapshot & snapshot, const double clearance_m = 0.0)
{
  planner::StepEvaluatorParameters parameters;
  parameters.hard_clearance_radius_m = clearance_m;
  parameters.preferred_clearance_radius_m = clearance_m;
  return planner::StepEvaluator(snapshot, parameters);
}

void expectPointNear(const planner::Point2D actual, const planner::Point2D expected)
{
  EXPECT_NEAR(actual.x, expected.x, 1.0e-12);
  EXPECT_NEAR(actual.y, expected.y, 1.0e-12);
}

}  // namespace

TEST(PlanningQueryResolver, ExactValidQueryIsUnchanged)
{
  const auto snapshot = makeGrid();
  const auto evaluator = makeEvaluator(snapshot);
  const auto resolution = planner::PlanningQueryResolver{}.resolve(
    snapshot, evaluator, {0.43, 0.47}, 0.3, true);
  ASSERT_TRUE(resolution);
  expectPointNear(resolution->effective, {0.43, 0.47});
  EXPECT_TRUE(resolution->requested_valid);
  EXPECT_FALSE(resolution->snapped);
  EXPECT_DOUBLE_EQ(resolution->snap_distance_m, 0.0);
  EXPECT_EQ(resolution->evaluated_candidate_count, 0U);
}

TEST(PlanningQueryResolver, UnknownRequestedCellSnapsToNearestValidCell)
{
  const auto snapshot = makeGrid({{5, 5}});
  const auto evaluator = makeEvaluator(snapshot);
  const auto resolution = planner::PlanningQueryResolver{}.resolve(
    snapshot, evaluator, {0.5, 0.5}, 0.15, true);
  ASSERT_TRUE(resolution);
  EXPECT_EQ(resolution->requested_reason, planner::StepInvalidReason::kUnknown);
  expectPointNear(resolution->effective, {0.4, 0.5});
  EXPECT_TRUE(resolution->snapped);
  EXPECT_NEAR(resolution->snap_distance_m, 0.1, 1.0e-12);
}

TEST(PlanningQueryResolver, InsufficientClearanceSupportSnapsToSupportedCell)
{
  const auto snapshot = makeGrid({{5, 5}});
  const auto evaluator = makeEvaluator(snapshot, 0.1);
  const auto resolution = planner::PlanningQueryResolver{}.resolve(
    snapshot, evaluator, {0.4, 0.5}, 0.25, true);
  ASSERT_TRUE(resolution);
  EXPECT_EQ(
    resolution->requested_reason,
    planner::StepInvalidReason::kInsufficientClearanceSupport);
  EXPECT_TRUE(resolution->effective_evaluation.valid);
  expectPointNear(resolution->effective, {0.3, 0.5});
  EXPECT_NEAR(resolution->snap_distance_m, 0.1, 1.0e-12);
}

TEST(PlanningQueryResolver, SelectsNearestValidCandidateRatherThanLoopFirst)
{
  const auto snapshot = makeGrid({{5, 5}, {4, 5}, {5, 4}, {5, 6}});
  const auto evaluator = makeEvaluator(snapshot);
  const auto resolution = planner::PlanningQueryResolver{}.resolve(
    snapshot, evaluator, {0.5, 0.5}, 0.4, true);
  ASSERT_TRUE(resolution);
  expectPointNear(resolution->effective, {0.6, 0.5});
  EXPECT_NEAR(resolution->snap_distance_m, 0.1, 1.0e-12);
}

TEST(PlanningQueryResolver, EqualDistanceTieBreakIsDeterministic)
{
  const auto snapshot = makeGrid({{5, 5}});
  const auto evaluator = makeEvaluator(snapshot);
  for (int repeat = 0; repeat < 20; ++repeat) {
    const auto resolution = planner::PlanningQueryResolver{}.resolve(
      snapshot, evaluator, {0.5, 0.5}, 0.15, true);
    ASSERT_TRUE(resolution);
    expectPointNear(resolution->effective, {0.4, 0.5});
  }
}

TEST(PlanningQueryResolver, SearchRadiusIsStrictlyBounded)
{
  const auto snapshot = makeGrid({{5, 5}, {4, 5}, {5, 4}, {5, 6}, {6, 5}});
  const auto evaluator = makeEvaluator(snapshot);
  EXPECT_FALSE(planner::PlanningQueryResolver{}.resolve(
      snapshot, evaluator, {0.5, 0.5}, 0.14, true));
}

TEST(PlanningQueryResolver, DisabledSnappingPreservesExactFailure)
{
  const auto snapshot = makeGrid({{5, 5}});
  const auto evaluator = makeEvaluator(snapshot);
  const auto attempt = planner::PlanningQueryResolver{}.resolveDetailed(
    snapshot, evaluator, {0.5, 0.5}, 0.3, false);
  EXPECT_FALSE(attempt.resolution);
  EXPECT_EQ(attempt.requested_reason, planner::StepInvalidReason::kUnknown);
  EXPECT_EQ(attempt.evaluated_candidate_count, 0U);
}

TEST(PlanningQueryResolver, OutOfBoundsQueryCanSnapBackInsideMap)
{
  const auto snapshot = makeGrid();
  const auto evaluator = makeEvaluator(snapshot);
  const auto resolution = planner::PlanningQueryResolver{}.resolve(
    snapshot, evaluator, {-0.06, 0.5}, 0.1, true);
  ASSERT_TRUE(resolution);
  EXPECT_EQ(resolution->requested_reason, planner::StepInvalidReason::kOutOfBounds);
  expectPointNear(resolution->effective, {0.0, 0.5});
}

TEST(PlanningQueryResolver, ZeroRadiusRetainsOnlyExactValidQueries)
{
  const auto snapshot = makeGrid({{5, 5}});
  const auto evaluator = makeEvaluator(snapshot);
  EXPECT_FALSE(planner::PlanningQueryResolver{}.resolve(
      snapshot, evaluator, {0.5, 0.5}, 0.0, true));
  const auto exact = planner::PlanningQueryResolver{}.resolve(
    snapshot, evaluator, {0.4, 0.4}, 0.0, true);
  ASSERT_TRUE(exact);
  expectPointNear(exact->effective, {0.4, 0.4});
  EXPECT_FALSE(exact->snapped);
}

TEST(PlanningQueryResolver, ObservedButClearanceInvalidCandidateIsRejected)
{
  const auto snapshot = makeGrid({}, {{5, 5}});
  const auto evaluator = makeEvaluator(snapshot, 0.1);
  const auto resolution = planner::PlanningQueryResolver{}.resolve(
    snapshot, evaluator, {0.5, 0.5}, 0.31, true);
  ASSERT_TRUE(resolution);
  EXPECT_EQ(
    resolution->requested_reason, planner::StepInvalidReason::kClearanceViolation);
  EXPECT_TRUE(resolution->effective_evaluation.valid);
  expectPointNear(resolution->effective, {0.4, 0.4});
  EXPECT_NEAR(resolution->snap_distance_m, std::sqrt(0.02), 1.0e-12);
}

TEST(PlanningQueryResolver, RejectsInvalidRadius)
{
  const auto snapshot = makeGrid();
  const auto evaluator = makeEvaluator(snapshot);
  EXPECT_THROW(
    planner::PlanningQueryResolver{}.resolve(
      snapshot, evaluator, {0.5, 0.5}, -0.1, true),
    std::invalid_argument);
  EXPECT_THROW(
    planner::PlanningQueryResolver{}.resolve(
      snapshot, evaluator, {0.5, 0.5}, std::numeric_limits<double>::infinity(), true),
    std::invalid_argument);
}
