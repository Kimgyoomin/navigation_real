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
