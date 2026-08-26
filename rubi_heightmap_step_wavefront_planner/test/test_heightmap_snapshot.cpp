#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
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
    planner::HeightmapSnapshot::fromPoints(
      {{0.025, 0.0, 0.0}, {0.075, 0.0, 0.0}, {0.101, 0.0, 0.0}},
      0.05, 0.01, 100U),
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

TEST(HeightmapSnapshot, AcceptsShiftedRegularAndNegativeLattices)
{
  const auto shifted = planner::HeightmapSnapshot::fromPoints(
    {{0.025, -0.025, 1.0}, {0.075, -0.025, 2.0},
      {0.025, 0.025, 3.0}, {0.075, 0.025, 4.0}},
    0.05, 0.01, 100U);
  EXPECT_EQ(shifted.sizeX(), 2U);
  EXPECT_EQ(shifted.sizeY(), 2U);
  EXPECT_DOUBLE_EQ(shifted.originX(), 0.025);
  EXPECT_DOUBLE_EQ(shifted.originY(), -0.025);
  EXPECT_DOUBLE_EQ(*shifted.elevation({1, 1}), 4.0);

  const auto negative = planner::HeightmapSnapshot::fromPoints(
    {{-24.975, -12.425, 1.0}, {-24.925, -12.425, 2.0},
      {-24.975, -12.375, 3.0}, {-24.925, -12.375, 4.0}},
    0.05, 0.01, 100U);
  EXPECT_DOUBLE_EQ(negative.originX(), -24.975);
  EXPECT_DOUBLE_EQ(negative.originY(), -12.425);
  EXPECT_EQ(negative.sizeX(), 2U);
  EXPECT_EQ(negative.sizeY(), 2U);
}

TEST(HeightmapSnapshot, ShiftedSparseLatticePreservesUnknownAndRoundTrips)
{
  const auto snapshot = planner::HeightmapSnapshot::fromPoints(
    {{0.025, 0.025, 1.0}, {0.125, 0.025, 2.0}, {0.125, 0.075, 3.0}},
    0.05, 0.01, 100U);
  EXPECT_EQ(snapshot.sizeX(), 3U);
  EXPECT_EQ(snapshot.sizeY(), 2U);
  EXPECT_FALSE(snapshot.observed({1, 0}));
  for (const planner::GridCell cell :
    {planner::GridCell{0, 0}, planner::GridCell{2, 0}, planner::GridCell{2, 1}})
  {
    EXPECT_EQ(snapshot.worldToCell(snapshot.cellCenter(cell)), cell);
  }
}

TEST(HeightmapSnapshot, RejectsIrregularAndDuplicateShiftedCellsWithDiagnostic)
{
  try {
    static_cast<void>(planner::HeightmapSnapshot::fromPoints(
      {{0.025, -0.025, 0.0}, {0.075, -0.025, 0.0}, {0.101, -0.025, 0.0}},
      0.05, 0.01, 100U));
    FAIL() << "irregular lattice was accepted";
  } catch (const std::invalid_argument & error) {
    const std::string message = error.what();
    for (const char * field :
      {"axis=x", "value=", "origin=", "resolution=", "nearest_index=",
        "expected=", "residual=", "tolerance="})
    {EXPECT_NE(message.find(field), std::string::npos) << message;}
  }
  EXPECT_THROW(
    planner::HeightmapSnapshot::fromPoints(
      {{0.025, -0.025, 0.0}, {0.025, -0.025, 1.0}}, 0.05, 0.01, 100U),
    std::invalid_argument);
}

TEST(HeightmapSnapshot, ShiftedHashAndGeometryAreOrderIndependent)
{
  std::vector<planner::HeightPoint> points{
    {0.025, -0.025, 1.0}, {0.075, -0.025, 2.0},
    {0.025, 0.025, 3.0}, {0.075, 0.025, 4.0}};
  const auto reference = planner::HeightmapSnapshot::fromPoints(
    points, 0.05, 0.01, 100U);
  std::mt19937 generator(42U);
  for (int repeat = 0; repeat < 20; ++repeat) {
    std::shuffle(points.begin(), points.end(), generator);
    const auto candidate = planner::HeightmapSnapshot::fromPoints(
      points, 0.05, 0.01, 100U);
    EXPECT_DOUBLE_EQ(candidate.originX(), reference.originX());
    EXPECT_DOUBLE_EQ(candidate.originY(), reference.originY());
    EXPECT_EQ(candidate.sizeX(), reference.sizeX());
    EXPECT_EQ(candidate.sizeY(), reference.sizeY());
    EXPECT_EQ(candidate.contentHash(), reference.contentHash());
  }
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
