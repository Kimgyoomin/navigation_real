#include <set>
#include <utility>
#include <vector>
#include <gtest/gtest.h>
#include "rubi_heightmap_step_wavefront_planner/terrain/height_evidence.hpp"
namespace planner = rubi_heightmap_step_wavefront_planner;

planner::HeightmapSnapshot evidenceMap(
  const std::set<std::pair<int, int>> & omitted = {}, int high_count = 0)
{
  std::vector<planner::HeightPoint> points;
  for (int y = 0; y < 5; ++y) for (int x = 0; x < 5; ++x) {
    if (!omitted.count({x, y})) points.push_back({0.05 * x, 0.05 * y,
      high_count-- > 0 ? 0.2 : 0.0});
  }
  return planner::HeightmapSnapshot::fromPoints(points, 0.05, 0.001, 100U);
}

TEST(HeightEvidence, ExactAndNearbyMissingCellUseNearestObservedElevation)
{
  auto full = evidenceMap();
  auto exact = planner::queryNodeHeightEvidence(full, {0.1, 0.1}, 0.08, 3U, 0.01, 0.08, 0.3);
  ASSERT_TRUE(exact.valid);
  EXPECT_DOUBLE_EQ(exact.nearest_elevation_m, 0.0);
  auto sparse = evidenceMap({{2, 2}});
  auto nearby = planner::queryNodeHeightEvidence(
    sparse, {0.1, 0.1}, 0.08, 3U, 0.075, 0.08, 0.3);
  ASSERT_TRUE(nearby.valid);
  EXPECT_NEAR(nearby.nearest_distance_m, 0.05, 1e-12);
  EXPECT_EQ(nearby.nearest_cell, (planner::GridCell{1, 2}));
}

TEST(HeightEvidence, EnforcesCountDistanceAndOutlierRatio)
{
  auto map = evidenceMap({}, 3);
  EXPECT_FALSE(planner::queryNodeHeightEvidence(
      map, {0.1, 0.1}, 0.01, 2U, 0.01, 0.08, 0.3).valid);
  EXPECT_FALSE(planner::queryNodeHeightEvidence(
      map, {0.125, 0.125}, 0.2, 3U, 0.01, 0.08, 0.3).valid);
  const auto strict = planner::queryNodeHeightEvidence(
    map, {0.1, 0.1}, 0.2, 3U, 0.1, 0.08, 0.05);
  EXPECT_FALSE(strict.valid);
  const auto relaxed = planner::queryNodeHeightEvidence(
    map, {0.1, 0.1}, 0.2, 3U, 0.1, 0.08, 0.5);
  EXPECT_TRUE(relaxed.valid);
}

TEST(HeightEvidence, EqualDistanceTieBreakAndEdgeQueryAreDeterministic)
{
  auto sparse = evidenceMap({{2, 2}});
  for (int repeat = 0; repeat < 20; ++repeat) {
    auto sample = planner::queryEdgeHeight(sparse, {0.1, 0.1}, 0.075);
    ASSERT_TRUE(sample);
    EXPECT_EQ(sample->source_cell, (planner::GridCell{1, 2}));
  }
  EXPECT_FALSE(planner::queryEdgeHeight(sparse, {5.0, 5.0}, 0.075));
}

TEST(HeightEvidence, OriginalTrgUsesUpperMedianStrictRatioAndNearestZ)
{
  std::vector<planner::HeightPoint> ten;
  for (int x = -4; x <= 5; ++x) {
    ten.push_back({0.05 * x, 0.0, x == 0 ? 0.05 : 0.0});
  }
  auto snapshot = planner::HeightmapSnapshot::fromPoints(ten, 0.05, 0.001, 1000U);
  auto evidence = planner::queryOriginalTrgHeightEvidence(snapshot, {0.0, 0.0}, 0.30, 0.08, 0.10);
  ASSERT_TRUE(evidence.valid);
  EXPECT_DOUBLE_EQ(evidence.median_elevation_m, 0.0);
  EXPECT_DOUBLE_EQ(evidence.nearest_elevation_m, 0.05);

  ten[0].z = 0.20;
  snapshot = planner::HeightmapSnapshot::fromPoints(ten, 0.05, 0.001, 1000U);
  evidence = planner::queryOriginalTrgHeightEvidence(snapshot, {0.0, 0.0}, 0.30, 0.08, 0.10);
  EXPECT_TRUE(evidence.valid);  // exactly 1 / 10 == threshold
  ten[1].z = 0.20;
  snapshot = planner::HeightmapSnapshot::fromPoints(ten, 0.05, 0.001, 1000U);
  evidence = planner::queryOriginalTrgHeightEvidence(snapshot, {0.0, 0.0}, 0.30, 0.08, 0.10);
  EXPECT_FALSE(evidence.valid);  // 2 / 10 > threshold

  const auto far = planner::HeightmapSnapshot::fromPoints(
    {{10.0, 10.0, 0.0}}, 0.05, 0.001, 1000U);
  EXPECT_FALSE(planner::queryOriginalTrgHeightEvidence(
      far, {0.0, 0.0}, 0.20, 0.08, 0.10).valid);
}
