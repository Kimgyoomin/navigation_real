#include "rubi_heightmap_step_wavefront_planner/ros/pointcloud2_heightmap_adapter.hpp"

#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

#include "sensor_msgs/msg/point_field.hpp"

namespace rubi_heightmap_step_wavefront_planner
{
namespace
{
bool hostBigEndian() noexcept
{
  const std::uint16_t value = 0x0102U;
  return *reinterpret_cast<const std::uint8_t *>(&value) == 0x01U;
}
std::uint32_t swap32(const std::uint32_t value) noexcept
{
  return ((value & 0xffU) << 24U) | ((value & 0xff00U) << 8U) |
         ((value & 0xff0000U) >> 8U) | ((value & 0xff000000U) >> 24U);
}
float readFloat32(const std::uint8_t * bytes, const bool message_big_endian)
{
  std::uint32_t bits;
  std::memcpy(&bits, bytes, sizeof(bits));
  if (message_big_endian != hostBigEndian()) {bits = swap32(bits);}
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
}  // namespace

std::vector<HeightPoint> PointCloud2HeightmapAdapter::parse(
  const sensor_msgs::msg::PointCloud2 & cloud) const
{
  const auto expected_row_step =
    static_cast<std::uint64_t>(cloud.width) * cloud.point_step;
  if (cloud.height != 1U || cloud.point_step == 0U || cloud.width == 0U ||
    expected_row_step > std::numeric_limits<std::uint32_t>::max() ||
    cloud.row_step != expected_row_step || cloud.data.size() != cloud.row_step)
  {throw std::invalid_argument("PointCloud2 must be a non-empty unorganized exact row");}
  const auto field_offset = [&](const std::string & name) {
    for (const auto & field : cloud.fields) {
      if (field.name != name) {continue;}
      if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count != 1U ||
        field.offset + sizeof(float) > cloud.point_step)
      {throw std::invalid_argument(name + " must be FLOAT32 count=1 within point_step");}
      return field.offset;
    }
    throw std::invalid_argument("PointCloud2 is missing " + name);
  };
  const auto x_offset = field_offset("x");
  const auto y_offset = field_offset("y");
  const auto z_offset = field_offset("z");
  std::vector<HeightPoint> points;
  points.reserve(cloud.width);
  for (std::size_t point_index = 0U; point_index < cloud.width; ++point_index) {
    const auto * bytes = cloud.data.data() + point_index * cloud.point_step;
    points.push_back({readFloat32(bytes + x_offset, cloud.is_bigendian),
      readFloat32(bytes + y_offset, cloud.is_bigendian),
      readFloat32(bytes + z_offset, cloud.is_bigendian)});
  }
  return points;
}

HeightmapSnapshot PointCloud2HeightmapAdapter::makeSnapshot(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const HeightmapAdapterParameters & parameters) const
{
  return HeightmapSnapshot::fromPoints(
    parse(cloud), parameters.resolution_m, parameters.lattice_tolerance_m,
    parameters.max_grid_cells);
}
}  // namespace rubi_heightmap_step_wavefront_planner
