#pragma once

#include <cstddef>
#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pongbot_navigation
{
struct SectorConfig
{
  std::string expected_frame_id = "livox_frame";
  double front_half_angle_deg = 60.0;
  double front_min_range = 0.3;
  double other_min_range = 1.0;
};

struct FilterResult
{
  sensor_msgs::msg::PointCloud2 cloud;
  std::size_t input_points = 0;
  std::size_t kept_points = 0;
  std::size_t nonfinite_drops = 0;
  std::size_t range_drops = 0;
};

// XY range and bearing are measured at the cloud's sensor origin. The current
// RUBI mount aligns livox_frame +X with base_link +X. No TF, deskew, coordinate
// rewriting, timestamp rewriting, or synthetic clearing endpoints occur here.
class SectorFilter
{
public:
  explicit SectorFilter(const SectorConfig & config);
  bool keep_xy(double x, double y) const;
  FilterResult filter(const sensor_msgs::msg::PointCloud2 & input) const;

private:
  SectorConfig config_;
  double front_cos_squared_;
  double front_range_squared_;
  double other_range_squared_;
};
}  // namespace pongbot_navigation
