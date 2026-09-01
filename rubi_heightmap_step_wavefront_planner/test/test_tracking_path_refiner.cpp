#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "rubi_heightmap_step_wavefront_planner/planning/tracking_path_refiner.hpp"

namespace planner = rubi_heightmap_step_wavefront_planner;

namespace
{
planner::HeightmapSnapshot flatHeightmap()
{
  std::vector<planner::HeightPoint> points;
  for (int y = 0; y < 21; ++y) {
    for (int x = 0; x < 21; ++x) {
      points.push_back({0.025 + 0.05 * x, 0.025 + 0.05 * y, 0.0});
    }
  }
  return planner::HeightmapSnapshot::fromPoints(points, 0.05, 0.001, 1000U);
}

planner::HeightmapSnapshot candidatePatchHeightmap(
  const double patch_z, const bool remove_patch)
{
  std::vector<planner::HeightPoint> points;
  const planner::Point2D candidate{0.2916666666666667, 0.4583333333333333};
  for (int y = 0; y < 21; ++y) {
    for (int x = 0; x < 21; ++x) {
      const double px = 0.025 + 0.05 * x;
      const double py = 0.025 + 0.05 * y;
      const bool patch = std::hypot(px - candidate.x, py - candidate.y) < 0.09;
      if (!remove_patch || !patch) {
        points.push_back({px, py, patch ? patch_z : 0.0});
      }
    }
  }
  return planner::HeightmapSnapshot::fromPoints(points, 0.05, 0.001, 1000U);
}

planner::StepEvaluatorParameters evaluatorParameters()
{
  planner::StepEvaluatorParameters value;
  value.node_min_observed_cells = 1U;
  value.node_max_nearest_evidence_distance_m = 0.08;
  value.edge_height_query_radius_m = 0.04;
  value.edge_max_height_evidence_gap_m = 0.08;
  return value;
}

planner::TrackingPathRefiner makeRefiner(const double increase = 0.05)
{
  planner::TrackingPathRefinerParameters parameters;
  parameters.resample_spacing_m = 0.10;
  parameters.max_cost_increase_ratio = increase;
  return planner::TrackingPathRefiner(parameters);
}
}  // namespace

TEST(TrackingPathRefiner, StraightLinePreservesEndpointsAndUsesFixedSpacing)
{
  const auto heightmap = flatHeightmap();
  const auto costmap = planner::CostmapSnapshot::fromData(
    21U, 21U, 0.05, 0.0, 0.0, std::vector<std::uint8_t>(441U, 0U));
  const planner::StepEvaluator evaluator(heightmap, costmap, evaluatorParameters());
  const std::vector<planner::TerrainPoint> raw{
    {0.125, 0.225, 0.0}, {0.425, 0.225, 0.0}, {0.725, 0.225, 0.0}};
  const auto result = makeRefiner().refine(raw, evaluator);
  ASSERT_TRUE(result.success);
  ASSERT_GT(result.path.size(), raw.size());
  EXPECT_DOUBLE_EQ(result.path.front().x, raw.front().x);
  EXPECT_DOUBLE_EQ(result.path.front().y, raw.front().y);
  EXPECT_DOUBLE_EQ(result.path.back().x, raw.back().x);
  EXPECT_DOUBLE_EQ(result.path.back().y, raw.back().y);
  for (std::size_t index = 1U; index < result.path.size(); ++index) {
    EXPECT_LE(std::hypot(result.path[index].x - result.path[index - 1U].x,
      result.path[index].y - result.path[index - 1U].y), 0.10 + 1.0e-12);
  }
  EXPECT_TRUE(planner::evaluatePolyline(result.path, 0U, evaluator).valid);
}

TEST(TrackingPathRefiner, ValidZigzagReducesMaximumHeadingChangeDeterministically)
{
  const auto heightmap = flatHeightmap();
  const auto costmap = planner::CostmapSnapshot::fromData(
    21U, 21U, 0.05, 0.0, 0.0, std::vector<std::uint8_t>(441U, 0U));
  const planner::StepEvaluator evaluator(heightmap, costmap, evaluatorParameters());
  const std::vector<planner::TerrainPoint> raw{{0.125, 0.225, 0.0},
    {0.325, 0.425, 0.0}, {0.525, 0.225, 0.0}, {0.725, 0.425, 0.0}};
  const auto reference = makeRefiner().refine(raw, evaluator);
  ASSERT_TRUE(reference.success);
  EXPECT_LT(reference.tracking_max_heading_change_rad,
    reference.raw_max_heading_change_rad);
  for (int run = 0; run < 20; ++run) {
    const auto result = makeRefiner().refine(raw, evaluator);
    ASSERT_EQ(result.path.size(), reference.path.size());
    EXPECT_EQ(result.smoothing_attempts, reference.smoothing_attempts);
    EXPECT_EQ(result.smoothing_accepts, reference.smoothing_accepts);
    EXPECT_DOUBLE_EQ(result.tracking_cost, reference.tracking_cost);
    for (std::size_t index = 0U; index < result.path.size(); ++index) {
      EXPECT_DOUBLE_EQ(result.path[index].x, reference.path[index].x);
      EXPECT_DOUBLE_EQ(result.path[index].y, reference.path[index].y);
      EXPECT_DOUBLE_EQ(result.path[index].z, reference.path[index].z);
    }
  }
}

TEST(TrackingPathRefiner, CostmapObstacleRejectsMeanCandidateWithoutShortcutting)
{
  const auto heightmap = flatHeightmap();
  std::vector<std::uint8_t> raw_cost(441U, 0U);
  raw_cost[9U * 21U + 5U] = 254U;
  const auto costmap = planner::CostmapSnapshot::fromData(
    21U, 21U, 0.05, 0.0, 0.0, raw_cost);
  const planner::StepEvaluator evaluator(heightmap, costmap, evaluatorParameters());
  const std::vector<planner::TerrainPoint> raw{
    {0.125, 0.125, 0.0}, {0.125, 0.625, 0.0}, {0.625, 0.625, 0.0}};
  const auto result = makeRefiner().refine(raw, evaluator);
  ASSERT_TRUE(result.success);
  EXPECT_GT(result.smoothing_rejects, 0U);
  EXPECT_TRUE(planner::evaluatePolyline(result.path, 0U, evaluator).valid);
}

TEST(TrackingPathRefiner, ExcessiveSoftCostIncreaseFallsBackToRawRoute)
{
  const auto heightmap = flatHeightmap();
  std::vector<std::uint8_t> raw_cost(441U, 0U);
  for (int y = 5; y <= 9; ++y) {
    for (int x = 5; x <= 9; ++x) {raw_cost[y * 21 + x] = 252U;}
  }
  const auto costmap = planner::CostmapSnapshot::fromData(
    21U, 21U, 0.05, 0.0, 0.0, raw_cost);
  const planner::StepEvaluator evaluator(heightmap, costmap, evaluatorParameters());
  const std::vector<planner::TerrainPoint> raw{
    {0.125, 0.125, 0.0}, {0.125, 0.625, 0.0}, {0.625, 0.625, 0.0}};
  const auto result = makeRefiner(0.0).refine(raw, evaluator);
  ASSERT_TRUE(result.success);
  EXPECT_TRUE(result.used_raw_fallback);
  EXPECT_TRUE(planner::evaluatePolyline(result.path, 0U, evaluator).valid);
}

TEST(TrackingPathRefiner, OverLimitStepAndEvidenceGapRejectMeanCandidate)
{
  const std::vector<planner::TerrainPoint> raw{
    {0.125, 0.125, 0.0}, {0.125, 0.625, 0.0}, {0.625, 0.625, 0.0}};
  const auto costmap = planner::CostmapSnapshot::fromData(
    21U, 21U, 0.05, 0.0, 0.0, std::vector<std::uint8_t>(441U, 0U));
  auto parameters = evaluatorParameters();
  parameters.node_max_height_outlier_ratio = 1.0;
  const auto step_heightmap = candidatePatchHeightmap(0.10, false);
  const planner::StepEvaluator step_evaluator(step_heightmap, costmap, parameters);
  const auto step_result = makeRefiner().refine(raw, step_evaluator);
  ASSERT_TRUE(step_result.success);
  EXPECT_GT(step_result.smoothing_rejects, 0U);
  EXPECT_TRUE(planner::evaluatePolyline(step_result.path, 0U, step_evaluator).valid);

  const auto gap_heightmap = candidatePatchHeightmap(0.0, true);
  const planner::StepEvaluator gap_evaluator(gap_heightmap, costmap, parameters);
  const auto gap_result = makeRefiner().refine(raw, gap_evaluator);
  ASSERT_TRUE(gap_result.success);
  EXPECT_GT(gap_result.smoothing_rejects, 0U);
  EXPECT_TRUE(planner::evaluatePolyline(gap_result.path, 0U, gap_evaluator).valid);
}

TEST(TrackingPathRefiner, AcceptedSmoothedPointUsesCurrentTerrainElevation)
{
  const auto heightmap = candidatePatchHeightmap(0.05, false);
  const auto costmap = planner::CostmapSnapshot::fromData(
    21U, 21U, 0.05, 0.0, 0.0, std::vector<std::uint8_t>(441U, 0U));
  auto parameters = evaluatorParameters();
  parameters.node_max_height_outlier_ratio = 1.0;
  const planner::StepEvaluator evaluator(heightmap, costmap, parameters);
  const std::vector<planner::TerrainPoint> raw{
    {0.125, 0.125, 0.0}, {0.125, 0.625, 0.0}, {0.625, 0.625, 0.0}};
  const auto result = makeRefiner(1.0).refine(raw, evaluator);
  ASSERT_TRUE(result.success);
  EXPECT_GT(result.smoothing_accepts, 0U);
  EXPECT_TRUE(std::any_of(result.path.begin(), result.path.end(), [](const auto & point) {
    return std::abs(point.z - 0.05) < 1.0e-12;
  }));
}
