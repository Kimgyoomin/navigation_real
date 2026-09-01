#include "rubi_heightmap_step_wavefront_planner/ros/nav2_costmap_adapter.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace rubi_heightmap_step_wavefront_planner
{

CostmapSnapshot Nav2CostmapAdapter::makeSnapshot(
  const nav2_msgs::msg::Costmap & message) const
{
  const auto & orientation = message.metadata.origin.orientation;
  if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
    !std::isfinite(orientation.z) || !std::isfinite(orientation.w) ||
    std::abs(orientation.x) > 1.0e-9 || std::abs(orientation.y) > 1.0e-9 ||
    std::abs(orientation.z) > 1.0e-9 || std::abs(std::abs(orientation.w) - 1.0) > 1.0e-9)
  {
    throw std::invalid_argument("costmap origin must have finite zero-yaw orientation");
  }
  return CostmapSnapshot::fromData(
    message.metadata.size_x, message.metadata.size_y,
    static_cast<double>(message.metadata.resolution),
    message.metadata.origin.position.x, message.metadata.origin.position.y,
    std::vector<std::uint8_t>(message.data.begin(), message.data.end()));
}

}  // namespace rubi_heightmap_step_wavefront_planner
