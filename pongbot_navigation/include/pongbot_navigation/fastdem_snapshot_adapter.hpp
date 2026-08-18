#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rubi_heightmap_wavefront_planner/terrain_snapshot.hpp"

namespace pongbot_navigation
{

struct ParsedFastdemSnapshot
{
  std::shared_ptr<const rubi_heightmap_wavefront_planner::TerrainSnapshot> snapshot;
  std::string frame_id;
  std::uint64_t content_hash{0U};
};

ParsedFastdemSnapshot parseFastdemSnapshot(
  const sensor_msgs::msg::PointCloud2 & cloud,
  double resolution_m,
  double lattice_tolerance_m,
  bool reject_duplicate_cells,
  std::size_t max_grid_cells);

}  // namespace pongbot_navigation
