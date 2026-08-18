#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "rubi_heightmap_step_wavefront_planner/heightmap_snapshot.hpp"

namespace planner = rubi_heightmap_step_wavefront_planner;

TEST(HeightmapSnapshot, ReconstructsFiveCentimeterLatticeAndPreservesUnknown)
{
  const auto snapshot = planner::HeightmapSnapshot::fromPoints(
    {{0.0, 0.0, 1.0}, {0.10, 0.0, 2.0}}, 0.05, 0.01, 100U);
  EXPECT_EQ(snapshot.sizeX(), 3U);
  EXPECT_TRUE(snapshot.elevationAt({0.0, 0.0}).has_value());
  EXPECT_FALSE(snapshot.elevationAt({0.05, 0.0}).has_value());
}

TEST(HeightmapSnapshot, RejectsDuplicateOffLatticeNonFiniteAndMaxGrid)
{
  EXPECT_THROW(
    planner::HeightmapSnapshot::fromPoints(
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}, 0.05, 0.01, 100U),
    std::invalid_argument);
  EXPECT_THROW(
    planner::HeightmapSnapshot::fromPoints({{0.012, 0.0, 0.0}}, 0.05, 0.01, 100U),
    std::invalid_argument);
  EXPECT_THROW(
    planner::HeightmapSnapshot::fromPoints(
      {{0.0, 0.0, std::numeric_limits<double>::infinity()}}, 0.05, 0.01, 100U),
    std::invalid_argument);
  EXPECT_THROW(
    planner::HeightmapSnapshot::fromPoints(
      {{0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}}, 0.05, 0.01, 100U),
    std::invalid_argument);
}

TEST(HeightmapSnapshot, BoundaryRoundingAndCanonicalHashAreDeterministic)
{
  std::vector<planner::HeightPoint> points{{-0.05, 0.0, 1.0}, {0.0, 0.0, 2.0}};
  const auto first = planner::HeightmapSnapshot::fromPoints(points, 0.05, 0.01, 100U);
  std::reverse(points.begin(), points.end());
  const auto second = planner::HeightmapSnapshot::fromPoints(points, 0.05, 0.01, 100U);
  EXPECT_EQ(first.contentHash(), second.contentHash());
  EXPECT_DOUBLE_EQ(*first.elevationAt({-0.026, 0.0}), 1.0);
  EXPECT_DOUBLE_EQ(*first.elevationAt({-0.024, 0.0}), 2.0);
}

TEST(HeightmapSnapshot, DisappearingCellBecomesUnknownInNewSnapshot)
{
  const auto first = planner::HeightmapSnapshot::fromPoints(
    {{0.0, 0.0, 0.0}, {0.05, 0.0, 1.0}}, 0.05, 0.01, 100U);
  const auto second = planner::HeightmapSnapshot::fromPoints(
    {{0.0, 0.0, 0.0}, {0.10, 0.0, 0.0}}, 0.05, 0.01, 100U);
  EXPECT_TRUE(first.elevationAt({0.05, 0.0}).has_value());
  EXPECT_FALSE(second.elevationAt({0.05, 0.0}).has_value());
}
