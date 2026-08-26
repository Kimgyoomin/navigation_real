#pragma once

#include <cstddef>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rubi_heightmap_step_wavefront_planner/map/heightmap_snapshot.hpp"

namespace rubi_heightmap_step_wavefront_planner
{
struct HeightmapAdapterParameters
{
  double resolution_m{0.05};
  double lattice_tolerance_m{0.01};
  std::size_t max_grid_cells{5000000U};
};

/** @brief Strict PointCloud2-to-immutable-heightmap boundary adapter. */
class PointCloud2HeightmapAdapter
{
public:
  std::vector<HeightPoint> parse(const sensor_msgs::msg::PointCloud2 & cloud) const;
  HeightmapSnapshot makeSnapshot(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const HeightmapAdapterParameters & parameters) const;
};
}  // namespace rubi_heightmap_step_wavefront_planner
