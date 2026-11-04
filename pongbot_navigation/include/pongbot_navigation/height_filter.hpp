#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>
#include <Eigen/Geometry>

namespace pongbot_navigation
{
// Signed vertical distance from the BODY origin, expressed in cloud coordinates.
// h = normal.dot(point) + offset. Neither global translation nor map Z enters it.
struct HeightPlane
{
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
  double offset{0.};
  double minimum{-0.05};
  double maximum{1.0};

  void validate() const
  {
    if (!normal.allFinite() || std::abs(normal.norm() - 1.) > 1e-6 ||
      !std::isfinite(offset) || !std::isfinite(minimum) || !std::isfinite(maximum) ||
      minimum > maximum)
    {
      throw std::invalid_argument("Invalid BODY-relative height plane");
    }
  }

  bool keep(double x, double y, double z) const
  {
    const double h = normal.x() * x + normal.y() * y + normal.z() * z + offset;
    return h >= minimum && h <= maximum;
  }
};

inline HeightPlane make_height_plane(
  const Eigen::Vector3d & up_in_imu,
  const Eigen::Isometry3d & imu_from_cloud,
  const Eigen::Vector3d & body_origin_in_imu, double minimum, double maximum)
{
  HeightPlane plane;
  plane.normal = imu_from_cloud.linear().transpose() * up_in_imu;
  plane.offset = up_in_imu.dot(imu_from_cloud.translation() - body_origin_in_imu);
  plane.minimum = minimum;
  plane.maximum = maximum;
  plane.validate();
  return plane;
}

// Bounded, acquisition-time interpolation. Never use latest orientation for an
// older cloud or extrapolate across missing odometry/IMU updates.
class TimedUpBuffer
{
public:
  bool add(int64_t stamp, const Eigen::Vector3d & up)
  {
    if (stamp <= 0 || !up.allFinite() || std::abs(up.norm() - 1.) > 1e-3 ||
      (!samples_.empty() && stamp <= samples_.back().stamp)) {return false;}
    samples_.push_back({stamp, up.normalized()});
    while (samples_.size() > 64) {samples_.pop_front();}
    return true;
  }

  std::optional<Eigen::Vector3d> at(int64_t stamp, int64_t max_gap_ns) const
  {
    for (std::size_t i = 0; i < samples_.size(); ++i) {
      const auto & right = samples_[i];
      if (right.stamp == stamp) {return right.up;}
      if (right.stamp > stamp) {
        if (i == 0) {return std::nullopt;}
        const auto & left = samples_[i - 1];
        const auto gap = right.stamp - left.stamp;
        if (gap > max_gap_ns || left.up.dot(right.up) <= 0.) {return std::nullopt;}
        const double fraction = static_cast<double>(stamp - left.stamp) / gap;
        return ((1. - fraction) * left.up + fraction * right.up).normalized().eval();
      }
    }
    return std::nullopt;
  }

  void clear() {samples_.clear();}

private:
  struct Sample {int64_t stamp; Eigen::Vector3d up;};
  std::deque<Sample> samples_;
};
}  // namespace pongbot_navigation
