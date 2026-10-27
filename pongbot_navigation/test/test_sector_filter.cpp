#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>
#include "pongbot_navigation/sector_filter.hpp"

using pongbot_navigation::SectorConfig;
using pongbot_navigation::SectorFilter;
using Cloud = sensor_msgs::msg::PointCloud2;

namespace
{
void write_float32(uint8_t * out, float value, bool big)
{
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(value));
  for (unsigned int i = 0; i < 4; ++i) {
    out[i] = static_cast<uint8_t>(bits >> (big ? 8 * (3 - i) : 8 * i));
  }
}

Cloud cloud(const std::vector<std::array<float, 3>> & points, bool big = false)
{
  Cloud out;
  out.header.frame_id = "livox_frame";
  out.header.stamp.sec = 1234;
  out.header.stamp.nanosec = 567890123;
  out.width = points.size();
  out.height = 1;
  out.point_step = 26;  // Livox XYZ/reflectivity/tag/line/timestamp layout
  out.row_step = out.width * out.point_step;
  out.is_bigendian = big;
  out.is_dense = false;
  const std::array<std::string, 7> names{{"x", "y", "z", "reflectivity", "tag", "line", "timestamp"}};
  const std::array<uint32_t, 7> offsets{{0, 4, 8, 12, 16, 17, 18}};
  for (unsigned int i = 0; i < names.size(); ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i];
    field.offset = offsets[i];
    field.count = 1;
    field.datatype = i < 4 ? field.FLOAT32 : i < 6 ? field.UINT8 : field.FLOAT64;
    out.fields.push_back(field);
  }
  out.data.resize(out.row_step);
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto * record = out.data.data() + i * out.point_step;
    for (unsigned int j = 0; j < out.point_step; ++j) {
      record[j] = static_cast<uint8_t>(i + j);
    }
    for (unsigned int axis = 0; axis < 3; ++axis) {
      write_float32(record + axis * 4, points[i][axis], big);
    }
  }
  return out;
}
}  // namespace

TEST(SectorFilter, FrontMinimumIsInclusive)
{
  SectorFilter filter(SectorConfig{});
  EXPECT_FALSE(filter.keep_xy(.299, 0.));
  EXPECT_TRUE(filter.keep_xy(.3, 0.));
  EXPECT_TRUE(filter.keep_xy(.8, 0.));
  EXPECT_FALSE(filter.keep_xy(0., 0.));
}

TEST(SectorFilter, SideRearMinimumIsInclusive)
{
  SectorFilter filter(SectorConfig{});
  EXPECT_FALSE(filter.keep_xy(0., .999));
  EXPECT_FALSE(filter.keep_xy(0., -.999));
  EXPECT_FALSE(filter.keep_xy(-.999, 0.));
  EXPECT_TRUE(filter.keep_xy(0., 1.));
  EXPECT_TRUE(filter.keep_xy(0., -1.));
  EXPECT_TRUE(filter.keep_xy(-1., 0.));
  EXPECT_TRUE(filter.keep_xy(-2., 0.));  // no additional max-range cutoff
}

TEST(SectorFilter, BothSixtyDegreeBoundariesAndOutside)
{
  const SectorFilter filter(SectorConfig{});
  const double outside = 60.01 * std::acos(-1.) / 180.;
  for (const double sign : {-1., 1.}) {
    EXPECT_TRUE(filter.keep_xy(.25, sign * .25 * std::sqrt(3.)));
    EXPECT_FALSE(filter.keep_xy(.5 * std::cos(outside), sign * .5 * std::sin(outside)));
  }
}

TEST(SectorFilter, PreservesHeaderAllFieldsAndPointBytesWithoutMutatingInput)
{
  const auto input = cloud({{.5f, 0.f, .2f}, {0.f, .5f, .2f}, {-1.1f, 0.f, .2f}});
  const auto before = input;
  const auto result = SectorFilter(SectorConfig{}).filter(input);
  EXPECT_EQ(input, before);
  EXPECT_EQ(result.cloud.header, input.header);
  EXPECT_EQ(result.cloud.fields, input.fields);
  EXPECT_EQ(result.cloud.point_step, input.point_step);
  EXPECT_EQ(result.cloud.is_bigendian, input.is_bigendian);
  EXPECT_EQ(result.cloud.is_dense, input.is_dense);
  ASSERT_EQ(result.cloud.data.size(), 52u);
  EXPECT_TRUE(std::equal(input.data.begin(), input.data.begin() + 26, result.cloud.data.begin()));
  EXPECT_TRUE(std::equal(input.data.begin() + 52, input.data.end(), result.cloud.data.begin() + 26));
  EXPECT_EQ(result.cloud.width, 2u);
  EXPECT_EQ(result.cloud.height, 1u);
  EXPECT_EQ(result.cloud.row_step, 52u);
  EXPECT_EQ(result.range_drops, 1u);
}

TEST(SectorFilter, RejectsNonfinitePointsAndUsesHorizontalNotSlantRange)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const auto result = SectorFilter(SectorConfig{}).filter(cloud({
    {nan, 0.f, 0.f}, {1.f, inf, 0.f}, {1.f, 0.f, nan},
    {.1f, 0.f, 10.f}, {0.f, .5f, 10.f}, {1.f, 0.f, 0.f}}));
  EXPECT_EQ(result.input_points, 6u);
  EXPECT_EQ(result.nonfinite_drops, 3u);
  EXPECT_EQ(result.range_drops, 2u);
  EXPECT_EQ(result.kept_points, 1u);
}

TEST(SectorFilter, BigEndianUnalignedRecords)
{
  const auto input = cloud({{.5f, 0.f, 0.f}, {0.f, .5f, 0.f}, {-1.5f, 0.f, 0.f}}, true);
  const auto result = SectorFilter(SectorConfig{}).filter(input);
  EXPECT_EQ(result.kept_points, 2u);
  EXPECT_TRUE(result.cloud.is_bigendian);
  EXPECT_TRUE(std::equal(input.data.begin() + 52, input.data.end(), result.cloud.data.begin() + 26));
}

TEST(SectorFilter, OrganizedRowsWithPadding)
{
  auto input = cloud({{.5f, 0.f, 0.f}, {.1f, 0.f, 0.f}, {0.f, 2.f, 0.f}, {0.f, .5f, 0.f}});
  const auto original = input.data;
  input.width = 2;
  input.height = 2;
  input.row_step = 56;
  input.data.assign(112, 0xff);
  std::copy(original.begin(), original.begin() + 52, input.data.begin());
  std::copy(original.begin() + 52, original.end(), input.data.begin() + 56);
  const auto result = SectorFilter(SectorConfig{}).filter(input);
  ASSERT_EQ(result.kept_points, 2u);
  EXPECT_EQ(result.cloud.row_step, 52u);
  EXPECT_TRUE(std::equal(original.begin() + 52, original.begin() + 78,
    result.cloud.data.begin() + 26));
}

TEST(SectorFilter, EmptyAndAllRemovedAreNotSyntheticClearingRays)
{
  const auto empty = SectorFilter(SectorConfig{}).filter(cloud({}));
  EXPECT_EQ(empty.cloud.width, 0u);
  EXPECT_TRUE(empty.cloud.data.empty());
  const auto removed = SectorFilter(SectorConfig{}).filter(cloud({{.1f, 0.f, 0.f}, {0.f, .5f, 0.f}}));
  EXPECT_EQ(removed.range_drops, 2u);
  EXPECT_TRUE(removed.cloud.data.empty());
}

TEST(SectorFilter, RejectsMissingMalformedAndTruncatedFields)
{
  SectorFilter filter(SectorConfig{});
  auto input = cloud({{1.f, 0.f, 0.f}});
  input.fields.erase(input.fields.begin());
  EXPECT_THROW(filter.filter(input), std::invalid_argument);
  input = cloud({{1.f, 0.f, 0.f}});
  input.fields[0].offset = 25;
  EXPECT_THROW(filter.filter(input), std::invalid_argument);
  input = cloud({{1.f, 0.f, 0.f}});
  input.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT64;
  EXPECT_THROW(filter.filter(input), std::invalid_argument);
  input = cloud({{1.f, 0.f, 0.f}});
  input.data.resize(25);
  EXPECT_THROW(filter.filter(input), std::invalid_argument);
  input = cloud({{1.f, 0.f, 0.f}});
  input.row_step = 1;
  EXPECT_THROW(filter.filter(input), std::invalid_argument);
  input = cloud({{1.f, 0.f, 0.f}});
  input.fields.push_back(input.fields[0]);
  EXPECT_THROW(filter.filter(input), std::invalid_argument);
}

TEST(SectorFilter, RefusesZeroStampAndUnexpectedFrame)
{
  SectorFilter filter(SectorConfig{});
  auto input = cloud({{1.f, 0.f, 0.f}});
  input.header.stamp.sec = 0;
  input.header.stamp.nanosec = 0;
  EXPECT_THROW(filter.filter(input), std::invalid_argument);
  input = cloud({{1.f, 0.f, 0.f}});
  input.header.frame_id = "camera_init";
  EXPECT_THROW(filter.filter(input), std::invalid_argument);
}

TEST(SectorFilter, ValidatesStartupParameters)
{
  SectorConfig config;
  config.front_half_angle_deg = 90.;
  EXPECT_THROW((void)SectorFilter{config}, std::invalid_argument);
  config.front_half_angle_deg = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)SectorFilter{config}, std::invalid_argument);
  config = SectorConfig{};
  config.other_min_range = .1;
  EXPECT_THROW((void)SectorFilter{config}, std::invalid_argument);
  config = SectorConfig{};
  config.front_min_range = -.1;
  EXPECT_THROW((void)SectorFilter{config}, std::invalid_argument);
  config = SectorConfig{};
  config.expected_frame_id.clear();
  EXPECT_THROW((void)SectorFilter{config}, std::invalid_argument);
}

TEST(SectorFilter, RepresentativeTwentyThousandPointPass)
{
  std::vector<std::array<float, 3>> points;
  for (int i = 0; i < 20000; ++i) {
    const float theta = i * .01f;
    const float range = (i % 400) * .01f;
    points.push_back({range * std::cos(theta), range * std::sin(theta), .2f});
  }
  const auto input = cloud(points);
  SectorFilter filter(SectorConfig{});
  std::vector<double> durations;
  for (int i = 0; i < 100; ++i) {
    const auto start = std::chrono::steady_clock::now();
    const auto result = filter.filter(input);
    durations.push_back(std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count());
    EXPECT_EQ(result.input_points, result.kept_points + result.range_drops + result.nonfinite_drops);
  }
  std::sort(durations.begin(), durations.end());
  RecordProperty("filter_only_20k_points_median_ms", std::to_string(durations[50]));
  RecordProperty("filter_only_20k_points_p95_ms", std::to_string(durations[95]));
  // Deliberately no timing pass/fail limit: this excludes DDS and Nav2 work.
}
