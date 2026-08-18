#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "sensor_msgs/msg/point_field.hpp"

#include "pongbot_navigation/fastdem_snapshot_adapter.hpp"

namespace
{

std::uint32_t swap32(const std::uint32_t value)
{
  return ((value & 0xffU) << 24U) | ((value & 0xff00U) << 8U) |
         ((value & 0xff0000U) >> 8U) | ((value & 0xff000000U) >> 24U);
}

sensor_msgs::msg::PointCloud2 cloud(
  const std::vector<std::array<float, 3>> & points,
  const bool big_endian = false,
  const bool extra_field = false)
{
  sensor_msgs::msg::PointCloud2 message;
  message.header.frame_id = "map";
  message.height = 1U;
  message.width = points.size();
  message.is_bigendian = big_endian;
  message.is_dense = true;
  message.point_step = extra_field ? 16U : 12U;
  message.row_step = message.width * message.point_step;
  message.fields = {
    sensor_msgs::msg::PointField().set__name("x").set__offset(0U).set__datatype(
      sensor_msgs::msg::PointField::FLOAT32).set__count(1U),
    sensor_msgs::msg::PointField().set__name("y").set__offset(4U).set__datatype(
      sensor_msgs::msg::PointField::FLOAT32).set__count(1U),
    sensor_msgs::msg::PointField().set__name("z").set__offset(8U).set__datatype(
      sensor_msgs::msg::PointField::FLOAT32).set__count(1U)};
  if (extra_field) {
    message.fields.push_back(
      sensor_msgs::msg::PointField().set__name("variance").set__offset(12U).set__datatype(
        sensor_msgs::msg::PointField::FLOAT32).set__count(1U));
  }
  message.data.assign(message.row_step, 0U);
  for (std::size_t index = 0U; index < points.size(); ++index) {
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
      std::uint32_t bits = 0U;
      std::memcpy(&bits, &points[index][axis], sizeof(bits));
      if (big_endian) {bits = swap32(bits);}
      std::memcpy(
        message.data.data() + index * message.point_step + axis * sizeof(float),
        &bits, sizeof(bits));
    }
  }
  return message;
}

const std::vector<std::array<float, 3>> kGrid{
  {0.00F, 0.00F, 0.0F}, {0.05F, 0.00F, 0.0F},
  {0.00F, 0.05F, 0.0F}, {0.05F, 0.05F, 0.0F}};

}  // namespace

TEST(FastdemSnapshotAdapter, AcceptsXyzExtraFieldsAndBothEndianOrders)
{
  const auto little = pongbot_navigation::parseFastdemSnapshot(
    cloud(kGrid, false, true), 0.05, 0.01, true, 100U);
  const auto big = pongbot_navigation::parseFastdemSnapshot(
    cloud(kGrid, true, false), 0.05, 0.01, true, 100U);
  ASSERT_TRUE(little.snapshot && big.snapshot);
  EXPECT_EQ(little.snapshot->observedCount(), 4U);
  EXPECT_EQ(little.content_hash, big.content_hash);
  auto reversed = kGrid;
  std::reverse(reversed.begin(), reversed.end());
  EXPECT_EQ(
    little.content_hash,
    pongbot_navigation::parseFastdemSnapshot(
      cloud(reversed), 0.05, 0.01, true, 100U).content_hash);
}

TEST(FastdemSnapshotAdapter, RejectsMalformedFieldsPayloadAndNonFiniteValues)
{
  auto missing = cloud(kGrid);
  missing.fields.pop_back();
  EXPECT_THROW(
    pongbot_navigation::parseFastdemSnapshot(missing, 0.05, 0.01, true, 100U),
    std::invalid_argument);
  auto wrong = cloud(kGrid);
  wrong.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT64;
  EXPECT_THROW(
    pongbot_navigation::parseFastdemSnapshot(wrong, 0.05, 0.01, true, 100U),
    std::invalid_argument);
  auto count = cloud(kGrid);
  count.fields[1].count = 2U;
  EXPECT_THROW(
    pongbot_navigation::parseFastdemSnapshot(count, 0.05, 0.01, true, 100U),
    std::invalid_argument);
  auto payload = cloud(kGrid);
  ++payload.row_step;
  EXPECT_THROW(
    pongbot_navigation::parseFastdemSnapshot(payload, 0.05, 0.01, true, 100U),
    std::invalid_argument);
  auto nonfinite_points = kGrid;
  nonfinite_points[0][2] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(
    pongbot_navigation::parseFastdemSnapshot(
      cloud(nonfinite_points), 0.05, 0.01, true, 100U),
    std::invalid_argument);
}

TEST(FastdemSnapshotAdapter, EnforcesLatticeDuplicatePolicyAndCellBudget)
{
  auto off_lattice = kGrid;
  off_lattice[1][0] = 0.02F;
  EXPECT_THROW(
    pongbot_navigation::parseFastdemSnapshot(
      cloud(off_lattice), 0.05, 0.01, true, 100U),
    std::invalid_argument);
  auto duplicate = kGrid;
  duplicate.push_back(kGrid.front());
  EXPECT_THROW(
    pongbot_navigation::parseFastdemSnapshot(cloud(duplicate), 0.05, 0.01, true, 100U),
    std::invalid_argument);
  EXPECT_EQ(
    pongbot_navigation::parseFastdemSnapshot(
      cloud(duplicate), 0.05, 0.01, false, 100U).snapshot->observedCount(),
    4U);
  EXPECT_THROW(
    pongbot_navigation::parseFastdemSnapshot(cloud(kGrid), 0.05, 0.01, true, 3U),
    std::invalid_argument);
}
