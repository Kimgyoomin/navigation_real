#include "pongbot_navigation/sector_filter.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace pongbot_navigation
{
namespace
{
float read_float32(const uint8_t * bytes, bool big_endian)
{
  // Livox records may be unaligned; never reinterpret_cast a point buffer.
  uint32_t bits = 0;
  for (unsigned int i = 0; i < 4; ++i) {
    const unsigned int shift = big_endian ? 8 * (3 - i) : 8 * i;
    bits |= static_cast<uint32_t>(bytes[i]) << shift;
  }
  float result;
  static_assert(sizeof(result) == sizeof(bits), "FLOAT32 requires 32-bit float");
  std::memcpy(&result, &bits, sizeof(result));
  return result;
}
}  // namespace

SectorFilter::SectorFilter(const SectorConfig & config) : config_(config)
{
  if (config.expected_frame_id.empty() ||
    !std::isfinite(config.front_half_angle_deg) ||
    config.front_half_angle_deg <= 0.0 || config.front_half_angle_deg >= 90.0 ||
    !std::isfinite(config.front_min_range) || config.front_min_range < 0.0 ||
    !std::isfinite(config.other_min_range) ||
    config.other_min_range < config.front_min_range)
  {
    throw std::invalid_argument("Invalid sector filter frame, angle or range parameters");
  }
  const double front_cos = std::cos(config.front_half_angle_deg * std::acos(-1.0) / 180.0);
  front_cos_squared_ = front_cos * front_cos;
  front_range_squared_ = config.front_min_range * config.front_min_range;
  other_range_squared_ = config.other_min_range * config.other_min_range;
  if (!std::isfinite(front_range_squared_) || !std::isfinite(other_range_squared_)) {
    throw std::invalid_argument("Sector filter range squared overflow");
  }
}

bool SectorFilter::keep_xy(double x, double y) const
{
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return false;
  }
  const double range_squared = x * x + y * y;
  // Include the +/-60 degree boundary despite FLOAT32 coordinate rounding.
  // This avoids atan2/sqrt/trigonometry in the per-point hot path.
  const bool in_front = x >= 0.0 &&
    x * x + 1e-7 * range_squared >= front_cos_squared_ * range_squared;
  return range_squared >= (in_front ? front_range_squared_ : other_range_squared_);
}

FilterResult SectorFilter::filter(const sensor_msgs::msg::PointCloud2 & input) const
{
  if (input.header.frame_id != config_.expected_frame_id) {
    throw std::invalid_argument("Unexpected cloud frame; refusing to guess robot forward direction");
  }
  if (input.header.stamp.sec == 0 && input.header.stamp.nanosec == 0) {
    throw std::invalid_argument("Zero cloud timestamp; acquisition time must be preserved upstream");
  }
  std::array<int64_t, 3> offsets{{-1, -1, -1}};
  for (const auto & field : input.fields) {
    const int axis = field.name == "x" ? 0 : field.name == "y" ? 1 : field.name == "z" ? 2 : -1;
    if (axis < 0) {
      continue;
    }
    if (offsets[axis] >= 0 || field.count != 1 ||
      field.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      static_cast<uint64_t>(field.offset) + 4 > input.point_step)
    {
      throw std::invalid_argument("Expected unique FLOAT32 x/y/z fields within point_step");
    }
    offsets[axis] = field.offset;
  }
  for (const auto offset : offsets) {
    if (offset < 0) {
      throw std::invalid_argument("Missing x/y/z field");
    }
  }
  const uint64_t points = static_cast<uint64_t>(input.width) * input.height;
  const uint64_t row_bytes = static_cast<uint64_t>(input.width) * input.point_step;
  if (input.point_step == 0 || input.row_step < row_bytes ||
    points > std::numeric_limits<uint32_t>::max() / input.point_step)
  {
    throw std::invalid_argument("Invalid or oversized PointCloud2 dimensions");
  }
  const uint64_t required = input.height == 0 ? 0 :
    static_cast<uint64_t>(input.height - 1) * input.row_step + row_bytes;
  if (input.data.size() < required) {
    throw std::invalid_argument("Truncated PointCloud2 data");
  }

  FilterResult result;
  result.input_points = static_cast<std::size_t>(points);
  auto & output = result.cloud;
  output.header = input.header;
  output.fields = input.fields;
  output.is_bigendian = input.is_bigendian;
  output.is_dense = input.is_dense;
  output.point_step = input.point_step;
  output.height = 1;
  output.data.resize(static_cast<std::size_t>(points * input.point_step));
  for (uint32_t row = 0; row < input.height; ++row) {
    for (uint32_t col = 0; col < input.width; ++col) {
      const auto * record = input.data.data() +
        static_cast<std::size_t>(row) * input.row_step +
        static_cast<std::size_t>(col) * input.point_step;
      const double x = read_float32(record + offsets[0], input.is_bigendian);
      const double y = read_float32(record + offsets[1], input.is_bigendian);
      const double z = read_float32(record + offsets[2], input.is_bigendian);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        ++result.nonfinite_drops;
      } else if (!keep_xy(x, y)) {
        ++result.range_drops;
      } else {
        std::memcpy(output.data.data() + result.kept_points * input.point_step,
          record, input.point_step);
        ++result.kept_points;
      }
    }
  }
  output.width = static_cast<uint32_t>(result.kept_points);
  output.row_step = output.width * output.point_step;
  output.data.resize(output.row_step);
  return result;
}
}  // namespace pongbot_navigation
