#include "pongbot_local_graph_insertion_planner/local_overlay.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pongbot_local_graph_insertion_planner
{
namespace
{

bool insideGrid(const GridSnapshot & grid, long long x, long long y)
{
  return x >= 0 && y >= 0 &&
         x < static_cast<long long>(grid.size_x) &&
         y < static_cast<long long>(grid.size_y);
}

struct Bounds
{
  double min_x{std::numeric_limits<double>::infinity()};
  double min_y{std::numeric_limits<double>::infinity()};
  double max_x{-std::numeric_limits<double>::infinity()};
  double max_y{-std::numeric_limits<double>::infinity()};

  void include(double x, double y)
  {
    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
    max_x = std::max(max_x, x);
    max_y = std::max(max_y, y);
  }
};

}  // namespace

bool LocalGrid::valid() const
{
  return size_x != 0 && size_y != 0 &&
         size_x <= std::numeric_limits<std::size_t>::max() / size_y &&
         std::isfinite(resolution) && resolution > 0.0 &&
         costs.size() == size_x * size_y;
}

bool Transform2D::valid() const
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw);
}

OverlayResult applyLocalOverlay(
  GridSnapshot & fused,
  const LocalGrid & local,
  const Transform2D & global_from_local_grid)
{
  OverlayResult result;
  if (!fused.valid() || !local.valid() || !global_from_local_grid.valid()) {
    result.failure_reason = "invalid_overlay_geometry";
    return result;
  }

  const double cosine = std::cos(global_from_local_grid.yaw);
  const double sine = std::sin(global_from_local_grid.yaw);
  std::vector<std::size_t> modified_cells;

  for (std::size_t local_y = 0; local_y < local.size_y; ++local_y) {
    for (std::size_t local_x = 0; local_x < local.size_x; ++local_x) {
      const auto cost = local.costs[local_y * local.size_x + local_x];

      // Unknown is ignored by contract. Free observations cannot clear static
      // global obstacles; removal happens naturally from the next base snapshot.
      if (cost == 0 || cost == 255) {
        continue;
      }

      Bounds footprint;
      for (int corner_y = 0; corner_y < 2; ++corner_y) {
        for (int corner_x = 0; corner_x < 2; ++corner_x) {
          const double local_corner_x =
            (static_cast<double>(local_x) + corner_x) * local.resolution;
          const double local_corner_y =
            (static_cast<double>(local_y) + corner_y) * local.resolution;
          footprint.include(
            global_from_local_grid.x +
            cosine * local_corner_x - sine * local_corner_y,
            global_from_local_grid.y +
            sine * local_corner_x + cosine * local_corner_y);
        }
      }

      // Cell footprints are half-open. nextafter prevents a boundary exactly on
      // a global cell edge from spilling into the adjacent cell.
      const double max_x = std::nextafter(
        footprint.max_x, -std::numeric_limits<double>::infinity());
      const double max_y = std::nextafter(
        footprint.max_y, -std::numeric_limits<double>::infinity());
      const auto min_grid_x = static_cast<long long>(
        std::floor((footprint.min_x - fused.origin_x) / fused.resolution));
      const auto min_grid_y = static_cast<long long>(
        std::floor((footprint.min_y - fused.origin_y) / fused.resolution));
      const auto max_grid_x = static_cast<long long>(
        std::floor((max_x - fused.origin_x) / fused.resolution));
      const auto max_grid_y = static_cast<long long>(
        std::floor((max_y - fused.origin_y) / fused.resolution));

      for (long long global_y = min_grid_y; global_y <= max_grid_y; ++global_y) {
        for (long long global_x = min_grid_x; global_x <= max_grid_x; ++global_x) {
          if (!insideGrid(fused, global_x, global_y)) {
            continue;
          }

          const auto cell = fused.index(
            static_cast<std::size_t>(global_x),
            static_cast<std::size_t>(global_y));
          if (cost > fused.costs[cell]) {
            fused.costs[cell] = cost;
            modified_cells.push_back(cell);
          }
        }
      }
    }
  }

  std::sort(modified_cells.begin(), modified_cells.end());
  modified_cells.erase(
    std::unique(modified_cells.begin(), modified_cells.end()),
    modified_cells.end());
  result.success = true;
  result.overlay_cells = modified_cells.size();
  return result;
}

}  // namespace pongbot_local_graph_insertion_planner
