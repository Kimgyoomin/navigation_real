#include "rubi_heightmap_step_wavefront_planner/heightmap_snapshot.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace rubi_heightmap_step_wavefront_planner
{
namespace
{

constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

void hashBytes(std::uint64_t & hash, const void * data, std::size_t size) noexcept
{
  const auto * bytes = static_cast<const unsigned char *>(data);
  for (std::size_t index = 0U; index < size; ++index) {
    hash ^= bytes[index];
    hash *= kFnvPrime;
  }
}

std::int64_t latticeCoordinate(double value, double resolution, double tolerance)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument("heightmap point contains non-finite coordinate");
  }
  const double scaled = value / resolution;
  if (!std::isfinite(scaled) ||
    scaled < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
    scaled > static_cast<double>(std::numeric_limits<std::int64_t>::max()))
  {
    throw std::invalid_argument("heightmap lattice coordinate overflow");
  }
  const auto coordinate = static_cast<std::int64_t>(std::llround(scaled));
  if (std::abs(value - static_cast<double>(coordinate) * resolution) > tolerance) {
    throw std::invalid_argument("heightmap point is off the configured lattice");
  }
  return coordinate;
}

}  // namespace

HeightmapSnapshot HeightmapSnapshot::fromPoints(
  const std::vector<HeightPoint> & points,
  const double resolution_m,
  const double lattice_tolerance_m,
  const std::size_t max_grid_cells)
{
  if (points.empty()) {
    throw std::invalid_argument("heightmap snapshot is empty");
  }
  if (!std::isfinite(resolution_m) || resolution_m <= 0.0 ||
    !std::isfinite(lattice_tolerance_m) || lattice_tolerance_m < 0.0 ||
    lattice_tolerance_m >= 0.5 * resolution_m || max_grid_cells == 0U)
  {
    throw std::invalid_argument("invalid heightmap snapshot configuration");
  }

  struct CanonicalPoint {std::int64_t x; std::int64_t y; double z;};
  std::vector<CanonicalPoint> canonical;
  canonical.reserve(points.size());
  std::int64_t min_x = std::numeric_limits<std::int64_t>::max();
  std::int64_t max_x = std::numeric_limits<std::int64_t>::min();
  std::int64_t min_y = std::numeric_limits<std::int64_t>::max();
  std::int64_t max_y = std::numeric_limits<std::int64_t>::min();
  for (const auto & point : points) {
    if (!std::isfinite(point.z)) {
      throw std::invalid_argument("heightmap point contains non-finite elevation");
    }
    const std::int64_t x = latticeCoordinate(point.x, resolution_m, lattice_tolerance_m);
    const std::int64_t y = latticeCoordinate(point.y, resolution_m, lattice_tolerance_m);
    canonical.push_back({x, y, point.z});
    min_x = std::min(min_x, x);
    max_x = std::max(max_x, x);
    min_y = std::min(min_y, y);
    max_y = std::max(max_y, y);
  }
  const std::uint64_t width = static_cast<std::uint64_t>(max_x - min_x) + 1U;
  const std::uint64_t height = static_cast<std::uint64_t>(max_y - min_y) + 1U;
  if (width > std::numeric_limits<std::size_t>::max() ||
    height > std::numeric_limits<std::size_t>::max() ||
    (height != 0U && width > std::numeric_limits<std::size_t>::max() / height))
  {
    throw std::invalid_argument("heightmap grid dimension overflow");
  }
  const std::size_t cell_count = static_cast<std::size_t>(width * height);
  if (cell_count > max_grid_cells) {
    throw std::invalid_argument("heightmap snapshot exceeds max_grid_cells");
  }

  HeightmapSnapshot snapshot;
  snapshot.resolution_m_ = resolution_m;
  snapshot.origin_x_ = static_cast<double>(min_x) * resolution_m;
  snapshot.origin_y_ = static_cast<double>(min_y) * resolution_m;
  snapshot.size_x_ = static_cast<std::size_t>(width);
  snapshot.size_y_ = static_cast<std::size_t>(height);
  snapshot.elevations_.assign(cell_count, 0.0);
  snapshot.observed_.assign(cell_count, false);
  for (const auto & point : canonical) {
    const std::size_t x = static_cast<std::size_t>(point.x - min_x);
    const std::size_t y = static_cast<std::size_t>(point.y - min_y);
    const std::size_t index = y * snapshot.size_x_ + x;
    if (snapshot.observed_[index]) {
      throw std::invalid_argument("heightmap snapshot contains a duplicate cell");
    }
    snapshot.observed_[index] = true;
    snapshot.elevations_[index] = point.z;
    ++snapshot.observed_count_;
  }

  std::uint64_t hash = kFnvOffset;
  hashBytes(hash, &snapshot.size_x_, sizeof(snapshot.size_x_));
  hashBytes(hash, &snapshot.size_y_, sizeof(snapshot.size_y_));
  hashBytes(hash, &snapshot.origin_x_, sizeof(snapshot.origin_x_));
  hashBytes(hash, &snapshot.origin_y_, sizeof(snapshot.origin_y_));
  hashBytes(hash, &snapshot.resolution_m_, sizeof(snapshot.resolution_m_));
  for (std::size_t index = 0U; index < cell_count; ++index) {
    const unsigned char observed = snapshot.observed_[index] ? 1U : 0U;
    hashBytes(hash, &observed, sizeof(observed));
    if (snapshot.observed_[index]) {
      hashBytes(hash, &snapshot.elevations_[index], sizeof(double));
    }
  }
  snapshot.content_hash_ = hash;
  return snapshot;
}

bool HeightmapSnapshot::inBounds(const GridCell cell) const noexcept
{
  return cell.x >= 0 && cell.y >= 0 &&
         static_cast<std::size_t>(cell.x) < size_x_ &&
         static_cast<std::size_t>(cell.y) < size_y_;
}

std::optional<std::size_t> HeightmapSnapshot::index(const GridCell cell) const noexcept
{
  if (!inBounds(cell)) {return std::nullopt;}
  return static_cast<std::size_t>(cell.y) * size_x_ + static_cast<std::size_t>(cell.x);
}

GridCell HeightmapSnapshot::worldToCell(const Point2D point) const noexcept
{
  return GridCell{
    static_cast<int>(std::llround((point.x - origin_x_) / resolution_m_)),
    static_cast<int>(std::llround((point.y - origin_y_) / resolution_m_))};
}

Point2D HeightmapSnapshot::cellCenter(const GridCell cell) const noexcept
{
  return Point2D{
    origin_x_ + static_cast<double>(cell.x) * resolution_m_,
    origin_y_ + static_cast<double>(cell.y) * resolution_m_};
}

bool HeightmapSnapshot::observed(const GridCell cell) const noexcept
{
  const auto index_value = index(cell);
  return index_value && observed_[*index_value];
}

std::optional<double> HeightmapSnapshot::elevation(const GridCell cell) const noexcept
{
  const auto index_value = index(cell);
  if (!index_value || !observed_[*index_value]) {return std::nullopt;}
  return elevations_[*index_value];
}

std::optional<double> HeightmapSnapshot::elevationAt(const Point2D point) const noexcept
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y)) {return std::nullopt;}
  return elevation(worldToCell(point));
}

}  // namespace rubi_heightmap_step_wavefront_planner
