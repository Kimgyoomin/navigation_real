#include "rubi_heightmap_step_wavefront_planner/terrain/local_relief_hazard_snapshot.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rubi_heightmap_step_wavefront_planner
{
namespace
{
constexpr double kClearanceTolerance = 1.0e-12;

double pointRectangleDistance(
  const Point2D point, const Point2D center, const double half_extent) noexcept
{
  const double dx = std::max(std::abs(point.x - center.x) - half_extent, 0.0);
  const double dy = std::max(std::abs(point.y - center.y) - half_extent, 0.0);
  return std::hypot(dx, dy);
}

double cross(const Point2D a, const Point2D b, const Point2D c) noexcept
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool onSegment(const Point2D a, const Point2D b, const Point2D p) noexcept
{
  return std::abs(cross(a, b, p)) <= kClearanceTolerance &&
         p.x >= std::min(a.x, b.x) - kClearanceTolerance &&
         p.x <= std::max(a.x, b.x) + kClearanceTolerance &&
         p.y >= std::min(a.y, b.y) - kClearanceTolerance &&
         p.y <= std::max(a.y, b.y) + kClearanceTolerance;
}

bool segmentsIntersect(
  const Point2D a, const Point2D b, const Point2D c, const Point2D d) noexcept
{
  const double c1 = cross(a, b, c);
  const double c2 = cross(a, b, d);
  const double c3 = cross(c, d, a);
  const double c4 = cross(c, d, b);
  if (((c1 > kClearanceTolerance && c2 < -kClearanceTolerance) ||
    (c1 < -kClearanceTolerance && c2 > kClearanceTolerance)) &&
    ((c3 > kClearanceTolerance && c4 < -kClearanceTolerance) ||
    (c3 < -kClearanceTolerance && c4 > kClearanceTolerance)))
  {
    return true;
  }
  return (std::abs(c1) <= kClearanceTolerance && onSegment(a, b, c)) ||
         (std::abs(c2) <= kClearanceTolerance && onSegment(a, b, d)) ||
         (std::abs(c3) <= kClearanceTolerance && onSegment(c, d, a)) ||
         (std::abs(c4) <= kClearanceTolerance && onSegment(c, d, b));
}

double pointSegmentDistance(const Point2D point, const Point2D from, const Point2D to) noexcept
{
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double squared_length = dx * dx + dy * dy;
  if (squared_length <= kClearanceTolerance * kClearanceTolerance) {
    return std::hypot(point.x - from.x, point.y - from.y);
  }
  const double ratio = std::clamp(
    ((point.x - from.x) * dx + (point.y - from.y) * dy) / squared_length, 0.0, 1.0);
  return std::hypot(
    point.x - (from.x + ratio * dx), point.y - (from.y + ratio * dy));
}

double segmentRectangleDistance(
  const Point2D from, const Point2D to, const Point2D center,
  const double half_extent) noexcept
{
  if (pointRectangleDistance(from, center, half_extent) == 0.0 ||
    pointRectangleDistance(to, center, half_extent) == 0.0)
  {
    return 0.0;
  }
  const Point2D corners[4] = {
    {center.x - half_extent, center.y - half_extent},
    {center.x + half_extent, center.y - half_extent},
    {center.x + half_extent, center.y + half_extent},
    {center.x - half_extent, center.y + half_extent}};
  double minimum = std::min(
    pointRectangleDistance(from, center, half_extent),
    pointRectangleDistance(to, center, half_extent));
  for (int index = 0; index < 4; ++index) {
    const Point2D edge_from = corners[index];
    const Point2D edge_to = corners[(index + 1) % 4];
    if (segmentsIntersect(from, to, edge_from, edge_to)) {return 0.0;}
    minimum = std::min(minimum, pointSegmentDistance(edge_from, from, to));
    minimum = std::min(minimum, pointSegmentDistance(from, edge_from, edge_to));
    minimum = std::min(minimum, pointSegmentDistance(to, edge_from, edge_to));
  }
  return minimum;
}
}  // namespace

LocalReliefHazardSnapshot LocalReliefHazardSnapshot::build(const StepEvaluator & evaluator)
{
  const auto & heightmap = evaluator.snapshot();
  LocalReliefHazardSnapshot result;
  result.heightmap_hash_ = heightmap.contentHash();
  result.resolution_m_ = heightmap.resolution();
  result.origin_x_ = heightmap.originX();
  result.origin_y_ = heightmap.originY();
  result.size_x_ = heightmap.sizeX();
  result.size_y_ = heightmap.sizeY();
  result.relief_threshold_m_ = evaluator.parameters().local_relief_threshold_m;
  result.first_window_radius_m_ =
    evaluator.parameters().local_relief_first_window_radius_m;
  result.second_window_radius_m_ =
    evaluator.parameters().local_relief_second_window_radius_m;
  result.lower_quantile_ = evaluator.parameters().local_relief_lower_quantile;
  result.upper_quantile_ = evaluator.parameters().local_relief_upper_quantile;
  result.minimum_observed_ratio_ =
    evaluator.parameters().local_relief_min_observed_ratio;
  result.critical_cell_count_ = evaluator.parameters().local_relief_critical_cell_count;
  result.evidence_valid_.assign(heightmap.cellCount(), false);
  result.seed_mask_.assign(heightmap.cellCount(), false);
  for (std::size_t y = 0U; y < result.size_y_; ++y) {
    for (std::size_t x = 0U; x < result.size_x_; ++x) {
      const GridCell cell{static_cast<int>(x), static_cast<int>(y)};
      const std::size_t cell_index = y * result.size_x_ + x;
      if (!heightmap.observed(cell)) {
        ++result.evidence_invalid_count_;
        continue;
      }
      const auto supported_relief = evaluator.supportedLocalReliefAt(cell);
      if (!supported_relief) {
        ++result.evidence_invalid_count_;
        continue;
      }
      result.evidence_valid_[cell_index] = true;
      if (*supported_relief > result.relief_threshold_m_) {
        result.seed_mask_[cell_index] = true;
        result.seed_cells_.push_back(cell);
      }
    }
  }
  return result;
}

double LocalReliefHazardSnapshot::pointToCellAreaDistance(
  const Point2D point, const Point2D cell_center, const double resolution_m) noexcept
{
  if (!std::isfinite(resolution_m) || resolution_m <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return pointRectangleDistance(point, cell_center, 0.5 * resolution_m);
}

double LocalReliefHazardSnapshot::segmentToCellAreaDistance(
  const Point2D from, const Point2D to, const Point2D cell_center,
  const double resolution_m) noexcept
{
  if (!std::isfinite(resolution_m) || resolution_m <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return segmentRectangleDistance(from, to, cell_center, 0.5 * resolution_m);
}

bool LocalReliefHazardSnapshot::matches(const StepEvaluator & evaluator) const noexcept
{
  const auto & heightmap = evaluator.snapshot();
  return heightmap_hash_ == heightmap.contentHash() &&
    resolution_m_ == heightmap.resolution() && origin_x_ == heightmap.originX() &&
    origin_y_ == heightmap.originY() && size_x_ == heightmap.sizeX() &&
    size_y_ == heightmap.sizeY() &&
    relief_threshold_m_ == evaluator.parameters().local_relief_threshold_m &&
    first_window_radius_m_ == evaluator.parameters().local_relief_first_window_radius_m &&
    second_window_radius_m_ == evaluator.parameters().local_relief_second_window_radius_m &&
    lower_quantile_ == evaluator.parameters().local_relief_lower_quantile &&
    upper_quantile_ == evaluator.parameters().local_relief_upper_quantile &&
    minimum_observed_ratio_ == evaluator.parameters().local_relief_min_observed_ratio &&
    critical_cell_count_ == evaluator.parameters().local_relief_critical_cell_count;
}

std::optional<std::size_t> LocalReliefHazardSnapshot::index(const GridCell cell) const noexcept
{
  if (cell.x < 0 || cell.y < 0 || static_cast<std::size_t>(cell.x) >= size_x_ ||
    static_cast<std::size_t>(cell.y) >= size_y_)
  {
    return std::nullopt;
  }
  return static_cast<std::size_t>(cell.y) * size_x_ + static_cast<std::size_t>(cell.x);
}

GridCell LocalReliefHazardSnapshot::worldToCell(const Point2D point) const noexcept
{
  return {static_cast<int>(std::llround((point.x - origin_x_) / resolution_m_)),
    static_cast<int>(std::llround((point.y - origin_y_) / resolution_m_))};
}

Point2D LocalReliefHazardSnapshot::cellCenter(const GridCell cell) const noexcept
{
  return {origin_x_ + resolution_m_ * cell.x, origin_y_ + resolution_m_ * cell.y};
}

bool LocalReliefHazardSnapshot::segmentEvidenceValid(
  const Point2D from, const Point2D to) const
{
  GridCell cell = worldToCell(from);
  const GridCell goal = worldToCell(to);
  const auto valid = [this](const GridCell candidate) {
      const auto cell_index = index(candidate);
      return cell_index && evidence_valid_[*cell_index];
    };
  if (!valid(cell)) {return false;}
  if (cell == goal) {return true;}

  // Amanatides-Woo supercover traversal in the height-grid coordinate frame.
  // Unlike fixed-distance sampling, every crossed cell is visited. At a grid
  // corner both incident side cells are checked conservatively before the
  // diagonal cell is entered.
  const double start_x = (from.x - origin_x_) / resolution_m_;
  const double start_y = (from.y - origin_y_) / resolution_m_;
  const double delta_x = (to.x - from.x) / resolution_m_;
  const double delta_y = (to.y - from.y) / resolution_m_;
  const int step_x = (delta_x > 0.0) - (delta_x < 0.0);
  const int step_y = (delta_y > 0.0) - (delta_y < 0.0);
  const double infinity = std::numeric_limits<double>::infinity();
  double t_max_x = infinity;
  double t_max_y = infinity;
  const double t_delta_x = step_x == 0 ? infinity : 1.0 / std::abs(delta_x);
  const double t_delta_y = step_y == 0 ? infinity : 1.0 / std::abs(delta_y);
  if (step_x != 0) {
    const double boundary = static_cast<double>(cell.x) + 0.5 * step_x;
    t_max_x = (boundary - start_x) / delta_x;
  }
  if (step_y != 0) {
    const double boundary = static_cast<double>(cell.y) + 0.5 * step_y;
    t_max_y = (boundary - start_y) / delta_y;
  }
  const std::size_t maximum_visits = size_x_ + size_y_ + 4U;
  for (std::size_t visits = 0U; visits < maximum_visits && !(cell == goal); ++visits) {
    if (std::abs(t_max_x - t_max_y) <= kClearanceTolerance) {
      if (!valid({cell.x + step_x, cell.y}) ||
        !valid({cell.x, cell.y + step_y}))
      {
        return false;
      }
      cell.x += step_x;
      cell.y += step_y;
      t_max_x += t_delta_x;
      t_max_y += t_delta_y;
    } else if (t_max_x < t_max_y) {
      cell.x += step_x;
      t_max_x += t_delta_x;
    } else {
      cell.y += step_y;
      t_max_y += t_delta_y;
    }
    if (!valid(cell)) {return false;}
  }
  return cell == goal;
}

TerrainClearanceQuery LocalReliefHazardSnapshot::queryPoint(
  const Point2D point, const double clearance_m) const
{
  return querySegment(point, point, clearance_m);
}

TerrainClearanceQuery LocalReliefHazardSnapshot::querySegment(
  const Point2D from, const Point2D to, const double clearance_m) const
{
  if (!std::isfinite(clearance_m) || clearance_m < 0.0 ||
    !std::isfinite(from.x) || !std::isfinite(from.y) ||
    !std::isfinite(to.x) || !std::isfinite(to.y))
  {
    return {};
  }
  TerrainClearanceQuery result;
  result.evidence_valid = segmentEvidenceValid(from, to);
  if (!result.evidence_valid || seed_cells_.empty()) {return result;}

  const double half_extent = 0.5 * resolution_m_;
  const double halo = clearance_m + half_extent + kClearanceTolerance;
  const int min_x = std::max(0, static_cast<int>(std::floor(
      (std::min(from.x, to.x) - halo - origin_x_) / resolution_m_)));
  const int max_x = std::min(static_cast<int>(size_x_) - 1, static_cast<int>(std::ceil(
      (std::max(from.x, to.x) + halo - origin_x_) / resolution_m_)));
  const int min_y = std::max(0, static_cast<int>(std::floor(
      (std::min(from.y, to.y) - halo - origin_y_) / resolution_m_)));
  const int max_y = std::min(static_cast<int>(size_y_) - 1, static_cast<int>(std::ceil(
      (std::max(from.y, to.y) + halo - origin_y_) / resolution_m_)));
  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      const GridCell cell{x, y};
      const auto cell_index = index(cell);
      if (!cell_index || !seed_mask_[*cell_index]) {continue;}
      const Point2D center = cellCenter(cell);
      const double distance = segmentRectangleDistance(from, to, center, half_extent);
      if (distance < result.minimum_distance_m) {
        result.minimum_distance_m = distance;
        result.nearest_seed_center = center;
      }
    }
  }
  // Equality is rejected conservatively. A value must exceed the configured
  // clearance by more than numerical tolerance to be accepted.
  result.violation = result.minimum_distance_m <= clearance_m + kClearanceTolerance;
  if (!result.violation && !seed_cells_.empty()) {
    // Candidate search is deliberately limited to the hard-decision halo.
    // A passing query therefore reports a strict lower bound, not a fabricated
    // global nearest-seed distance.
    result.minimum_distance_m = std::nextafter(
      clearance_m, std::numeric_limits<double>::infinity());
    result.minimum_distance_exact = false;
  }
  return result;
}

}  // namespace rubi_heightmap_step_wavefront_planner
