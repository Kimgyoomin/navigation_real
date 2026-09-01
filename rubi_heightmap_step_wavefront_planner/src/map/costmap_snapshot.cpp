#include "rubi_heightmap_step_wavefront_planner/map/costmap_snapshot.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace rubi_heightmap_step_wavefront_planner
{

CostmapSnapshot CostmapSnapshot::fromData(
  const std::size_t size_x, const std::size_t size_y, const double resolution_m,
  const double origin_x, const double origin_y, std::vector<std::uint8_t> costs)
{
  if (size_x == 0U || size_y == 0U ||
    size_x > std::numeric_limits<std::size_t>::max() / size_y ||
    costs.size() != size_x * size_y || !std::isfinite(resolution_m) ||
    resolution_m <= 0.0 || !std::isfinite(origin_x) || !std::isfinite(origin_y))
  {
    throw std::invalid_argument("invalid costmap snapshot geometry or data size");
  }
  CostmapSnapshot snapshot;
  snapshot.size_x_ = size_x;
  snapshot.size_y_ = size_y;
  snapshot.resolution_m_ = resolution_m;
  snapshot.origin_x_ = origin_x;
  snapshot.origin_y_ = origin_y;
  snapshot.costs_ = std::move(costs);
  return snapshot;
}

bool CostmapSnapshot::inBounds(const GridCell cell) const noexcept
{
  return cell.x >= 0 && cell.y >= 0 &&
         static_cast<std::size_t>(cell.x) < size_x_ &&
         static_cast<std::size_t>(cell.y) < size_y_;
}

std::optional<std::size_t> CostmapSnapshot::index(const GridCell cell) const noexcept
{
  if (!inBounds(cell)) {return std::nullopt;}
  return static_cast<std::size_t>(cell.y) * size_x_ + static_cast<std::size_t>(cell.x);
}

std::optional<GridCell> CostmapSnapshot::worldToCell(const Point2D point) const noexcept
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y)) {return std::nullopt;}
  const double x = std::floor((point.x - origin_x_) / resolution_m_);
  const double y = std::floor((point.y - origin_y_) / resolution_m_);
  if (x < static_cast<double>(std::numeric_limits<int>::min()) ||
    x > static_cast<double>(std::numeric_limits<int>::max()) ||
    y < static_cast<double>(std::numeric_limits<int>::min()) ||
    y > static_cast<double>(std::numeric_limits<int>::max()))
  {
    return std::nullopt;
  }
  return GridCell{static_cast<int>(x), static_cast<int>(y)};
}

Point2D CostmapSnapshot::cellCenter(const GridCell cell) const noexcept
{
  return {origin_x_ + (static_cast<double>(cell.x) + 0.5) * resolution_m_,
    origin_y_ + (static_cast<double>(cell.y) + 0.5) * resolution_m_};
}

std::optional<std::uint8_t> CostmapSnapshot::cost(const GridCell cell) const noexcept
{
  const auto cell_index = index(cell);
  return cell_index ? std::optional<std::uint8_t>{costs_[*cell_index]} : std::nullopt;
}

std::optional<std::uint8_t> CostmapSnapshot::costAt(const Point2D point) const noexcept
{
  const auto cell = worldToCell(point);
  return cell ? cost(*cell) : std::nullopt;
}

}  // namespace rubi_heightmap_step_wavefront_planner
