#include "pongbot_navigation/fastdem_snapshot_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "sensor_msgs/msg/point_field.hpp"

namespace pongbot_navigation
{
namespace
{

using rubi_heightmap_wavefront_planner::TerrainPoint;

constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

bool hostIsBigEndian() noexcept
{
  const std::uint16_t value = 0x0102U;
  return *reinterpret_cast<const std::uint8_t *>(&value) == 0x01U;
}

std::uint32_t byteSwap32(const std::uint32_t value) noexcept
{
  return ((value & 0x000000ffU) << 24U) | ((value & 0x0000ff00U) << 8U) |
         ((value & 0x00ff0000U) >> 8U) | ((value & 0xff000000U) >> 24U);
}

float readFloat(const std::uint8_t * data, const bool message_big_endian)
{
  std::uint32_t bits = 0U;
  std::memcpy(&bits, data, sizeof(bits));
  if (message_big_endian != hostIsBigEndian()) {bits = byteSwap32(bits);}
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

const sensor_msgs::msg::PointField & requireField(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  const sensor_msgs::msg::PointField * match = nullptr;
  for (const auto & field : cloud.fields) {
    if (field.name != name) {continue;}
    if (match != nullptr) {
      throw std::invalid_argument("PointCloud2 has duplicate '" + name + "' fields");
    }
    match = &field;
  }
  if (match == nullptr) {
    throw std::invalid_argument("PointCloud2 is missing required '" + name + "' field");
  }
  if (match->datatype != sensor_msgs::msg::PointField::FLOAT32 || match->count != 1U) {
    throw std::invalid_argument("PointCloud2 field '" + name + "' must be FLOAT32 count=1");
  }
  if (match->offset > cloud.point_step || cloud.point_step - match->offset < sizeof(float)) {
    throw std::invalid_argument("PointCloud2 field '" + name + "' exceeds point_step");
  }
  return *match;
}

void hashBytes(std::uint64_t & hash, const void * data, const std::size_t size) noexcept
{
  const auto * bytes = static_cast<const std::uint8_t *>(data);
  for (std::size_t index = 0U; index < size; ++index) {
    hash ^= bytes[index];
    hash *= kFnvPrime;
  }
}

void hashDouble(std::uint64_t & hash, const double value) noexcept
{
  hashBytes(hash, &value, sizeof(value));
}

void validateOptions(
  const double resolution_m, const double lattice_tolerance_m,
  const std::size_t max_grid_cells)
{
  if (!std::isfinite(resolution_m) || resolution_m <= 0.0 ||
    !std::isfinite(lattice_tolerance_m) || lattice_tolerance_m < 0.0 ||
    lattice_tolerance_m >= 0.5 * resolution_m || max_grid_cells == 0U)
  {
    throw std::invalid_argument("invalid FastDEM snapshot adapter options");
  }
}

}  // namespace

ParsedFastdemSnapshot parseFastdemSnapshot(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const double resolution_m,
  const double lattice_tolerance_m,
  const bool reject_duplicate_cells,
  const std::size_t max_grid_cells)
{
  validateOptions(resolution_m, lattice_tolerance_m, max_grid_cells);
  if (cloud.header.frame_id.empty()) {
    throw std::invalid_argument("PointCloud2 header.frame_id must not be empty");
  }
  if (cloud.height != 1U || cloud.width == 0U || cloud.point_step == 0U) {
    throw std::invalid_argument("FastDEM PointCloud2 must be a non-empty unorganized cloud");
  }
  const auto expected_row_step =
    static_cast<std::uint64_t>(cloud.width) * static_cast<std::uint64_t>(cloud.point_step);
  if (expected_row_step > std::numeric_limits<std::uint32_t>::max() ||
    static_cast<std::uint64_t>(cloud.row_step) != expected_row_step ||
    cloud.data.size() != expected_row_step)
  {
    throw std::invalid_argument("PointCloud2 row_step/data size contract violation");
  }
  if (cloud.width > max_grid_cells) {
    throw std::invalid_argument("PointCloud2 observed point count exceeds max_grid_cells");
  }

  const auto & x_field = requireField(cloud, "x");
  const auto & y_field = requireField(cloud, "y");
  const auto & z_field = requireField(cloud, "z");
  std::vector<TerrainPoint> raw;
  raw.reserve(cloud.width);
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < cloud.width; ++index) {
    const auto * base = cloud.data.data() + index * cloud.point_step;
    const double x = readFloat(base + x_field.offset, cloud.is_bigendian);
    const double y = readFloat(base + y_field.offset, cloud.is_bigendian);
    const double z = readFloat(base + z_field.offset, cloud.is_bigendian);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      throw std::invalid_argument("PointCloud2 contains non-finite XYZ");
    }
    raw.push_back({x, y, z});
    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
  }

  using Cell = std::pair<std::size_t, std::size_t>;
  std::map<Cell, TerrainPoint> canonical;
  std::size_t max_ix = 0U;
  std::size_t max_iy = 0U;
  for (const auto & point : raw) {
    const double gx = (point.x - min_x) / resolution_m;
    const double gy = (point.y - min_y) / resolution_m;
    const double rounded_x = std::round(gx);
    const double rounded_y = std::round(gy);
    if (std::abs(point.x - (min_x + rounded_x * resolution_m)) > lattice_tolerance_m ||
      std::abs(point.y - (min_y + rounded_y * resolution_m)) > lattice_tolerance_m ||
      rounded_x < 0.0 || rounded_y < 0.0 ||
      rounded_x > static_cast<double>(std::numeric_limits<std::size_t>::max() - 1U) ||
      rounded_y > static_cast<double>(std::numeric_limits<std::size_t>::max() - 1U))
    {
      throw std::invalid_argument("PointCloud2 contains an off-lattice point");
    }
    const Cell cell{static_cast<std::size_t>(rounded_x), static_cast<std::size_t>(rounded_y)};
    const TerrainPoint canonical_point{
      min_x + rounded_x * resolution_m, min_y + rounded_y * resolution_m, point.z};
    const auto [iterator, inserted] = canonical.emplace(cell, canonical_point);
    if (!inserted) {
      if (reject_duplicate_cells) {
        throw std::invalid_argument("PointCloud2 contains a duplicate lattice cell");
      }
      iterator->second.z = std::min(iterator->second.z, point.z);
    }
    max_ix = std::max(max_ix, cell.first);
    max_iy = std::max(max_iy, cell.second);
  }
  if (max_ix + 1U > std::numeric_limits<std::size_t>::max() / (max_iy + 1U) ||
    (max_ix + 1U) * (max_iy + 1U) > max_grid_cells)
  {
    throw std::invalid_argument("PointCloud2 dense lattice exceeds max_grid_cells");
  }

  std::vector<TerrainPoint> points;
  points.reserve(canonical.size());
  std::uint64_t content_hash = kFnvOffset;
  hashDouble(content_hash, resolution_m);
  hashDouble(content_hash, min_x);
  hashDouble(content_hash, min_y);
  for (const auto & entry : canonical) {
    points.push_back(entry.second);
    hashBytes(content_hash, &entry.first.first, sizeof(entry.first.first));
    hashBytes(content_hash, &entry.first.second, sizeof(entry.first.second));
    hashDouble(content_hash, entry.second.z);
  }
  auto snapshot = std::make_shared<const rubi_heightmap_wavefront_planner::TerrainSnapshot>(
    rubi_heightmap_wavefront_planner::TerrainSnapshot::fromPoints(
      points, resolution_m, lattice_tolerance_m, max_grid_cells));
  return {std::move(snapshot), cloud.header.frame_id, content_hash};
}

}  // namespace pongbot_navigation
