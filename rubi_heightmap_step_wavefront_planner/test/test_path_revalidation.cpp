#include <vector>

#include <gtest/gtest.h>

#include "rubi_heightmap_step_wavefront_planner/path_revalidation.hpp"

namespace planner = rubi_heightmap_step_wavefront_planner;

std::vector<planner::HeightPoint> revalidationGrid(bool barrier, bool outside)
{
  std::vector<planner::HeightPoint> points;
  for (int y = -6; y <= 6; ++y) {
    for (int x = -12; x <= 12; ++x) {
      double z = 0.0;
      if (barrier && x >= 2) {z = 0.081;}
      if (outside && x == -12 && y == -6) {z = 0.081;}
      points.push_back({0.05 * x, 0.05 * y, z});
    }
  }
  return points;
}

TEST(PathRevalidation, ValidAndOutsideChangesKeepRemainingPath)
{
  const std::vector<planner::TerrainPoint> path{{-0.4, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.4, 0.0, 0.0}};
  planner::StepEvaluatorParameters parameters;
  parameters.hard_clearance_radius_m = 0.0;
  for (bool outside : {false, true}) {
    const auto snapshot = planner::HeightmapSnapshot::fromPoints(
      revalidationGrid(false, outside), 0.05, 0.01, 10000U);
    EXPECT_TRUE(
      planner::validateRemainingPath(
        path, 0U, planner::StepEvaluator(snapshot, parameters)).valid);
  }
}

TEST(PathRevalidation, PassedSegmentIgnoredAndFutureStepRejected)
{
  const std::vector<planner::TerrainPoint> path{{-0.4, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.4, 0.0, 0.0}};
  planner::StepEvaluatorParameters parameters;
  parameters.hard_clearance_radius_m = 0.0;
  const auto snapshot = planner::HeightmapSnapshot::fromPoints(
    revalidationGrid(true, false), 0.05, 0.01, 10000U);
  const planner::StepEvaluator evaluator(snapshot, parameters);
  EXPECT_FALSE(planner::validateRemainingPath(path, 1U, evaluator).valid);
  EXPECT_EQ(planner::nearestPathIndex(path, {0.39, 0.0}, 1U), 2U);
  EXPECT_EQ(planner::nearestPathIndex(path, {-0.4, 0.0}, 2U), 2U);
}
