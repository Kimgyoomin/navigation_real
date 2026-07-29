#include "rubi_heightmap_wavefront_planner/terrain_snapshot.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace rubi_heightmap_wavefront_planner
{
namespace
{

bool finiteAndPositive(double value) noexcept
{
  return std::isfinite(value) && value > 0.0;
}

std::size_t checkedCellCount(std::size_t size_x, std::size_t size_y)
{
  if (size_x == 0U || size_y == 0U) {
    throw std::invalid_argument("TerrainSnapshot dimensions must be non-zero");
  }
  if (size_x > std::numeric_limits<std::size_t>::max() / size_y) {
    throw std::invalid_argument("TerrainSnapshot dimensions overflow size_t");
  }
  return size_x * size_y;
}

}  // namespace

TerrainSnapshot::TerrainSnapshot(
  double resolution_m,
  double min_x_center_m,
  double min_y_center_m,
  std::size_t size_x,
  std::size_t size_y,
  std::vector<double> elevation_m,
  std::vector<std::uint8_t> observed)
: resolution_m_(resolution_m),
  min_x_center_m_(min_x_center_m),
  min_y_center_m_(min_y_center_m),
  size_x_(size_x),
  size_y_(size_y),
  elevation_m_(std::move(elevation_m)),
  observed_(std::move(observed))
{
  if (!finiteAndPositive(resolution_m_)) {
    throw std::invalid_argument("TerrainSnapshot resolution must be finite and positive");
  }
  if (!std::isfinite(min_x_center_m_) || !std::isfinite(min_y_center_m_)) {
    throw std::invalid_argument("TerrainSnapshot lattice origin must be finite");
  }

  const std::size_t expected_size = checkedCellCount(size_x_, size_y_);
  if (elevation_m_.size() != expected_size || observed_.size() != expected_size) {
    throw std::invalid_argument("TerrainSnapshot data sizes do not match its geometry");
  }

  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t i = 0; i < expected_size; ++i) {
    if (observed_[i] != 0U) {
      observed_[i] = 1U;
      if (!std::isfinite(elevation_m_[i])) {
        throw std::invalid_argument("Observed TerrainSnapshot cells require finite elevation");
      }
      ++observed_count_;
    } else {
      elevation_m_[i] = nan;
    }
  }
}

TerrainSnapshot TerrainSnapshot::fromPoints(
  const std::vector<TerrainPoint> & points,
  double resolution_m,
  double lattice_tolerance_m,
  std::size_t max_cell_count)
{
  if (points.empty()) {
    throw std::invalid_argument("Cannot construct TerrainSnapshot from an empty point set");
  }
  if (!finiteAndPositive(resolution_m)) {
    throw std::invalid_argument("TerrainSnapshot resolution must be finite and positive");
  }
  if (!std::isfinite(lattice_tolerance_m) || lattice_tolerance_m < 0.0) {
    throw std::invalid_argument("Lattice tolerance must be finite and non-negative");
  }
  if (lattice_tolerance_m >= 0.5 * resolution_m) {
    throw std::invalid_argument("Lattice tolerance must be less than half the resolution");
  }
  if (max_cell_count == 0U) {
    throw std::invalid_argument("Maximum TerrainSnapshot cell count must be positive");
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  for (const TerrainPoint & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      throw std::invalid_argument("Terrain points must contain finite x, y, and z values");
    }
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
  }

  struct IndexedPoint
  {
    std::size_t ix;
    std::size_t iy;
    double z;
  };

  std::vector<IndexedPoint> indexed_points;
  indexed_points.reserve(points.size());
  std::size_t max_ix = 0U;
  std::size_t max_iy = 0U;

  for (const TerrainPoint & point : points) {
    const double grid_x = (point.x - min_x) / resolution_m;
    const double grid_y = (point.y - min_y) / resolution_m;
    const double rounded_x = std::round(grid_x);
    const double rounded_y = std::round(grid_y);

    const double residual_x = std::abs(point.x - (min_x + rounded_x * resolution_m));
    const double residual_y = std::abs(point.y - (min_y + rounded_y * resolution_m));
    if (residual_x > lattice_tolerance_m || residual_y > lattice_tolerance_m) {
      throw std::invalid_argument("Terrain point is not aligned to the requested lattice");
    }
    if (
      rounded_x < 0.0 || rounded_y < 0.0 ||
      rounded_x > static_cast<double>(std::numeric_limits<std::size_t>::max() - 1U) ||
      rounded_y > static_cast<double>(std::numeric_limits<std::size_t>::max() - 1U))
    {
      throw std::invalid_argument("Terrain lattice index is outside the supported range");
    }

    const auto ix = static_cast<std::size_t>(rounded_x);
    const auto iy = static_cast<std::size_t>(rounded_y);
    indexed_points.push_back({ix, iy, point.z});
    max_ix = std::max(max_ix, ix);
    max_iy = std::max(max_iy, iy);
  }

  const std::size_t size_x = max_ix + 1U;
  const std::size_t size_y = max_iy + 1U;
  const std::size_t cell_count = checkedCellCount(size_x, size_y);
  if (cell_count > max_cell_count) {
    throw std::invalid_argument("TerrainSnapshot exceeds the configured cell-count limit");
  }
  std::vector<double> elevation(
    cell_count, std::numeric_limits<double>::quiet_NaN());
  std::vector<std::uint8_t> observed(cell_count, 0U);

  for (const IndexedPoint & point : indexed_points) {
    const std::size_t flat = point.iy * size_x + point.ix;
    if (observed[flat] != 0U) {
      throw std::invalid_argument("Duplicate TerrainPoint entries map to the same lattice cell");
    }
    observed[flat] = 1U;
    elevation[flat] = point.z;
  }

  return TerrainSnapshot(
    resolution_m, min_x, min_y, size_x, size_y, std::move(elevation), std::move(observed));
}

double TerrainSnapshot::resolution() const noexcept
{
  return resolution_m_;
}

double TerrainSnapshot::minXCenter() const noexcept
{
  return min_x_center_m_;
}

double TerrainSnapshot::minYCenter() const noexcept
{
  return min_y_center_m_;
}

double TerrainSnapshot::maxXCenter() const noexcept
{
  return min_x_center_m_ + static_cast<double>(size_x_ - 1U) * resolution_m_;
}

double TerrainSnapshot::maxYCenter() const noexcept
{
  return min_y_center_m_ + static_cast<double>(size_y_ - 1U) * resolution_m_;
}

std::size_t TerrainSnapshot::sizeX() const noexcept
{
  return size_x_;
}

std::size_t TerrainSnapshot::sizeY() const noexcept
{
  return size_y_;
}

std::size_t TerrainSnapshot::cellCount() const noexcept
{
  return elevation_m_.size();
}

std::size_t TerrainSnapshot::observedCount() const noexcept
{
  return observed_count_;
}

bool TerrainSnapshot::inBounds(std::size_t ix, std::size_t iy) const noexcept
{
  return ix < size_x_ && iy < size_y_;
}

bool TerrainSnapshot::isObserved(std::size_t ix, std::size_t iy) const noexcept
{
  return inBounds(ix, iy) && observed_[flatIndex(ix, iy)] != 0U;
}

std::optional<GridIndex> TerrainSnapshot::worldToIndex(double x, double y) const noexcept
{
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return std::nullopt;
  }

  const double grid_x = (x - min_x_center_m_) / resolution_m_;
  const double grid_y = (y - min_y_center_m_) / resolution_m_;
  const double index_x = std::floor(grid_x + 0.5);
  const double index_y = std::floor(grid_y + 0.5);
  if (
    index_x < 0.0 || index_y < 0.0 ||
    index_x >= static_cast<double>(size_x_) ||
    index_y >= static_cast<double>(size_y_))
  {
    return std::nullopt;
  }

  return GridIndex{
    static_cast<std::size_t>(index_x),
    static_cast<std::size_t>(index_y)};
}

std::optional<Point2D> TerrainSnapshot::cellCenter(
  std::size_t ix, std::size_t iy) const noexcept
{
  if (!inBounds(ix, iy)) {
    return std::nullopt;
  }
  return Point2D{
    min_x_center_m_ + static_cast<double>(ix) * resolution_m_,
    min_y_center_m_ + static_cast<double>(iy) * resolution_m_};
}

std::optional<double> TerrainSnapshot::elevationAtCell(
  std::size_t ix, std::size_t iy) const noexcept
{
  if (!isObserved(ix, iy)) {
    return std::nullopt;
  }
  return elevation_m_[flatIndex(ix, iy)];
}

std::optional<double> TerrainSnapshot::elevationAt(double x, double y) const noexcept
{
  const auto index = worldToIndex(x, y);
  if (!index) {
    return std::nullopt;
  }
  return elevationAtCell(index->x, index->y);
}

std::optional<TerrainCell> TerrainSnapshot::query(double x, double y) const noexcept
{
  const auto index = worldToIndex(x, y);
  if (!index || !isObserved(index->x, index->y)) {
    return std::nullopt;
  }

  const auto center = cellCenter(index->x, index->y);
  return TerrainCell{
    *index,
    *center,
    elevation_m_[flatIndex(index->x, index->y)]};
}

const std::vector<double> & TerrainSnapshot::elevations() const noexcept
{
  return elevation_m_;
}

const std::vector<std::uint8_t> & TerrainSnapshot::observedMask() const noexcept
{
  return observed_;
}

std::size_t TerrainSnapshot::flatIndex(std::size_t ix, std::size_t iy) const noexcept
{
  return iy * size_x_ + ix;
}

}  // namespace rubi_heightmap_wavefront_planner
