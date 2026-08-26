#include "rubi_heightmap_step_wavefront_planner/graph/spatial_index_2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace rubi_heightmap_step_wavefront_planner
{

bool UniformGridSpatialIndex2D::BucketKey::operator==(const BucketKey & other) const noexcept
{
  return x == other.x && y == other.y;
}

std::size_t UniformGridSpatialIndex2D::BucketHash::operator()(const BucketKey & key) const noexcept
{
  const auto x = static_cast<std::uint32_t>(key.x);
  const auto y = static_cast<std::uint32_t>(key.y);
  return (static_cast<std::size_t>(x) << 32U) ^ static_cast<std::size_t>(y);
}

UniformGridSpatialIndex2D::UniformGridSpatialIndex2D(const double cell_size_m)
: cell_size_m_(cell_size_m)
{
  if (!std::isfinite(cell_size_m_) || cell_size_m_ <= 0.0) {
    throw std::invalid_argument("spatial-index cell size must be positive and finite");
  }
}

UniformGridSpatialIndex2D::BucketKey UniformGridSpatialIndex2D::bucket(
  const Point2D point) const noexcept
{
  return {static_cast<int>(std::floor(point.x / cell_size_m_)),
    static_cast<int>(std::floor(point.y / cell_size_m_))};
}

void UniformGridSpatialIndex2D::clear()
{
  positions_.clear();
  buckets_.clear();
}

void UniformGridSpatialIndex2D::insert(const NodeId id, const Point2D position)
{
  if (!std::isfinite(position.x) || !std::isfinite(position.y)) {
    throw std::invalid_argument("spatial-index position must be finite");
  }
  const auto existing = positions_.find(id);
  if (existing != positions_.end()) {
    auto & old_bucket = buckets_.at(bucket(existing->second));
    old_bucket.erase(std::remove(old_bucket.begin(), old_bucket.end(), id), old_bucket.end());
  }
  positions_[id] = position;
  buckets_[bucket(position)].push_back(id);
}

std::vector<NodeId> UniformGridSpatialIndex2D::radiusSearch(
  const Point2D query, const double radius_m) const
{
  if (!std::isfinite(query.x) || !std::isfinite(query.y) ||
    !std::isfinite(radius_m) || radius_m < 0.0)
  {
    return {};
  }
  const BucketKey center = bucket(query);
  const int extent = static_cast<int>(std::ceil(radius_m / cell_size_m_));
  const double radius_squared = radius_m * radius_m;
  std::vector<std::pair<double, NodeId>> matches;
  for (int dy = -extent; dy <= extent; ++dy) {
    for (int dx = -extent; dx <= extent; ++dx) {
      const auto found = buckets_.find({center.x + dx, center.y + dy});
      if (found == buckets_.end()) {continue;}
      for (const NodeId id : found->second) {
        const Point2D point = positions_.at(id);
        const double distance_squared =
          std::pow(point.x - query.x, 2) + std::pow(point.y - query.y, 2);
        if (distance_squared <= radius_squared + 1.0e-12) {
          matches.emplace_back(distance_squared, id);
        }
      }
    }
  }
  std::sort(matches.begin(), matches.end());
  std::vector<NodeId> result;
  result.reserve(matches.size());
  for (const auto & match : matches) {result.push_back(match.second);}
  return result;
}

std::optional<NodeId> UniformGridSpatialIndex2D::nearest(const Point2D query) const
{
  if (positions_.empty() || !std::isfinite(query.x) || !std::isfinite(query.y)) {
    return std::nullopt;
  }
  std::pair<double, NodeId> best{std::numeric_limits<double>::infinity(), 0U};
  for (const auto & [id, point] : positions_) {
    const double distance_squared =
      std::pow(point.x - query.x, 2) + std::pow(point.y - query.y, 2);
    best = std::min(best, std::make_pair(distance_squared, id));
  }
  return best.second;
}

}  // namespace rubi_heightmap_step_wavefront_planner
