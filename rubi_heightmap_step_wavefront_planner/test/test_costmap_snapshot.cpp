#include <cstdint>
#include <vector>
#include <gtest/gtest.h>
#include "rubi_heightmap_step_wavefront_planner/map/costmap_snapshot.hpp"
namespace planner = rubi_heightmap_step_wavefront_planner;

TEST(CostmapSnapshot, GeometryIndexRoundTripAndRawSemantics)
{
  const auto map = planner::CostmapSnapshot::fromData(
    5U, 1U, 0.05, -1.0, 2.0, {0U, 252U, 253U, 254U, 255U});
  EXPECT_EQ(map.cellCount(), 5U);
  EXPECT_DOUBLE_EQ(map.originX(), -1.0);
  for (int x = 0; x < 5; ++x) {
    const planner::GridCell cell{x, 0};
    const auto world = map.cellCenter(cell);
    ASSERT_TRUE(map.worldToCell(world));
    EXPECT_EQ(*map.worldToCell(world), cell);
    EXPECT_EQ(*map.index(cell), static_cast<std::size_t>(x));
    EXPECT_EQ(*map.cost(cell), static_cast<std::uint8_t>(x == 0 ? 0 : 251 + x));
  }
  EXPECT_FALSE(map.cost({5, 0}));
  EXPECT_FALSE(map.worldToCell({std::numeric_limits<double>::infinity(), 0.0}));
}

TEST(CostmapSnapshot, RejectsMalformedGeometryAndData)
{
  EXPECT_THROW(
    planner::CostmapSnapshot::fromData(2U, 2U, 0.05, 0.0, 0.0, {0U}),
    std::invalid_argument);
  EXPECT_THROW(
    planner::CostmapSnapshot::fromData(1U, 1U, 0.0, 0.0, 0.0, {0U}),
    std::invalid_argument);
}

TEST(CostmapSnapshot, ContentHashDeduplicatesAndDetectsGeometryOrCellChanges)
{
  const auto first = planner::CostmapSnapshot::fromData(
    3U, 2U, 0.05, -1.0, 2.0, {0U, 1U, 2U, 3U, 4U, 5U});
  const auto same = planner::CostmapSnapshot::fromData(
    3U, 2U, 0.05, -1.0, 2.0, {0U, 1U, 2U, 3U, 4U, 5U});
  const auto changed_cell = planner::CostmapSnapshot::fromData(
    3U, 2U, 0.05, -1.0, 2.0, {0U, 1U, 254U, 3U, 4U, 5U});
  const auto changed_origin = planner::CostmapSnapshot::fromData(
    3U, 2U, 0.05, -0.95, 2.0, {0U, 1U, 2U, 3U, 4U, 5U});
  EXPECT_EQ(first.contentHash(), same.contentHash());
  EXPECT_NE(first.contentHash(), changed_cell.contentHash());
  EXPECT_NE(first.contentHash(), changed_origin.contentHash());
}
