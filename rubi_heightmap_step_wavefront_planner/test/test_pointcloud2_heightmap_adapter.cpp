#include <cstring>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>
#include <gtest/gtest.h>
#include "sensor_msgs/msg/point_field.hpp"
#include "rubi_heightmap_step_wavefront_planner/ros/pointcloud2_heightmap_adapter.hpp"

namespace planner = rubi_heightmap_step_wavefront_planner;

sensor_msgs::msg::PointCloud2 cloud()
{
  sensor_msgs::msg::PointCloud2 message;
  message.height = 1U; message.width = 2U; message.point_step = 12U; message.row_step = 24U;
  for (const auto & [name, offset] :
    std::vector<std::pair<std::string, std::uint32_t>>{{"x", 0U}, {"y", 4U}, {"z", 8U}})
  {
    sensor_msgs::msg::PointField field;
    field.name = name; field.offset = offset;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1U;
    message.fields.push_back(field);
  }
  message.data.resize(24U);
  const float values[6] = {0.0F, 0.0F, 0.1F, 0.05F, 0.0F, 0.2F};
  std::memcpy(message.data.data(), values, sizeof(values));
  return message;
}

sensor_msgs::msg::PointCloud2 shiftedCloud()
{
  auto message = cloud();
  message.width = 4U;
  message.row_step = 48U;
  message.data.resize(48U);
  const float values[12] = {
    0.025F, -0.025F, 0.1F,
    0.075F, -0.025F, 0.2F,
    0.025F, 0.025F, 0.3F,
    0.075F, 0.025F, 0.4F};
  std::memcpy(message.data.data(), values, sizeof(values));
  return message;
}

TEST(PointCloud2HeightmapAdapter, StrictLayoutAndSnapshot)
{
  planner::PointCloud2HeightmapAdapter adapter;
  const auto points = adapter.parse(cloud());
  ASSERT_EQ(points.size(), 2U);
  EXPECT_NEAR(points[1].z, 0.2, 1.0e-6);
  EXPECT_EQ(adapter.makeSnapshot(cloud(), {}).observedCount(), 2U);
  auto malformed = cloud(); malformed.row_step = 25U;
  EXPECT_THROW(adapter.parse(malformed), std::invalid_argument);
  malformed = cloud(); malformed.fields.pop_back();
  EXPECT_THROW(adapter.parse(malformed), std::invalid_argument);
}

TEST(PointCloud2HeightmapAdapter, AcceptsShiftedRuntimeEntryLattice)
{
  const auto snapshot = planner::PointCloud2HeightmapAdapter{}.makeSnapshot(
    shiftedCloud(), {0.05, 0.01, 100U});
  EXPECT_EQ(snapshot.sizeX(), 2U);
  EXPECT_EQ(snapshot.sizeY(), 2U);
  EXPECT_NEAR(snapshot.originX(), 0.025, 1.0e-6);
  EXPECT_NEAR(snapshot.originY(), -0.025, 1.0e-6);
  EXPECT_NEAR(*snapshot.elevation({1, 1}), 0.4, 1.0e-6);
}
