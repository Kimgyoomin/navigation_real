#include <gtest/gtest.h>

#include <vector>

#include "rubi_heightmap_wavefront_planner/path_revalidation.hpp"

namespace rubi_heightmap_wavefront_planner
{
namespace
{

TerrainSnapshot makeSnapshot(
  const bool unknown_remaining = false, const bool step_remaining = false,
  const bool slope_remaining = false, const bool obstacle_passed = false)
{
  std::vector<TerrainPoint> points;
  for (int iy = -20; iy <= 20; ++iy) {
    for (int ix = -20; ix <= 60; ++ix) {
      if (
        (unknown_remaining && ix == 25 && iy == 0) ||
        (obstacle_passed && ix == 5 && iy == 0))
      {
        continue;
      }
      double z = 0.0;
      if (step_remaining && ix >= 25) {
        z = 0.10;
      }
      if (slope_remaining && ix >= 25) {
        z = 0.02 * static_cast<double>(ix - 25);
      }
      points.push_back(
        TerrainPoint{
            0.05 * static_cast<double>(ix), 0.05 * static_cast<double>(iy), z});
    }
  }
  return TerrainSnapshot::fromPoints(points, 0.05, 0.01);
}

TerrainEvaluator makeEvaluator(const TerrainSnapshot & snapshot)
{
  TerrainEvaluatorParameters parameters;
  parameters.pca_radius_m = 0.15;
  parameters.min_pca_points = 6U;
  parameters.footprint_radius_m = 0.0;
  parameters.min_footprint_observed_ratio = 1.0;
  parameters.max_slope_deg = 15.0;
  parameters.max_step_height_m = 0.08;
  parameters.edge_sample_spacing_m = 0.025;
  return TerrainEvaluator(snapshot, parameters);
}

const std::vector<Point2D> kRoute{
  {0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0}, {1.5, 0.0}, {2.0, 0.0}};

TEST(PathRevalidation, FlatRemainingPathIsValid)
{
  const TerrainSnapshot snapshot = makeSnapshot();
  const PathValidationResult result = validateRemainingPath(
    kRoute, {0.1, 0.0}, 0U, makeEvaluator(snapshot));
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(TerrainInvalidReason::kNone, result.reason);
}

TEST(PathRevalidation, ChangeOutsideCorridorDoesNotInvalidatePath)
{
  const TerrainSnapshot snapshot = makeSnapshot(false, false, false, true);
  const PathValidationResult result = validateRemainingPath(
    kRoute, {0.8, 0.0}, 0U, makeEvaluator(snapshot));
  EXPECT_TRUE(result.valid);
}

TEST(PathRevalidation, ObstacleOnPassedSegmentIsIgnored)
{
  const TerrainSnapshot snapshot = makeSnapshot(false, false, false, true);
  const PathValidationResult result = validateRemainingPath(
    kRoute, {1.25, 0.0}, 2U, makeEvaluator(snapshot));
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(2U, result.progress_segment);
}

TEST(PathRevalidation, UnknownRemainingSegmentIsInvalid)
{
  const TerrainSnapshot snapshot = makeSnapshot(true);
  const PathValidationResult result = validateRemainingPath(
    kRoute, {0.8, 0.0}, 0U, makeEvaluator(snapshot));
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(TerrainInvalidReason::kUnknown, result.reason);
}

TEST(PathRevalidation, StepRemainingSegmentIsInvalid)
{
  const TerrainSnapshot snapshot = makeSnapshot(false, true);
  const PathValidationResult result = validateRemainingPath(
    kRoute, {0.8, 0.0}, 0U, makeEvaluator(snapshot));
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(TerrainInvalidReason::kStepLimit, result.reason);
}

TEST(PathRevalidation, SlopeRemainingSegmentIsInvalid)
{
  const TerrainSnapshot snapshot = makeSnapshot(false, false, true);
  const PathValidationResult result = validateRemainingPath(
    kRoute, {0.8, 0.0}, 0U, makeEvaluator(snapshot));
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(TerrainInvalidReason::kSlopeLimit, result.reason);
}

TEST(PathRevalidation, ProgressNeverMovesBackward)
{
  const TerrainSnapshot snapshot = makeSnapshot();
  const PathValidationResult result = validateRemainingPath(
    kRoute, {0.1, 0.0}, 2U, makeEvaluator(snapshot));
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(2U, result.progress_segment);
}

}  // namespace
}  // namespace rubi_heightmap_wavefront_planner
