#include "rubi_heightmap_wavefront_planner/path_revalidation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rubi_heightmap_wavefront_planner
{
namespace
{

constexpr double kEpsilon = 1.0e-12;

bool finite(const Point2D point) noexcept
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

double pointToSegmentDistance(
  const Point2D point, const Point2D from, const Point2D to) noexcept
{
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double length_squared = dx * dx + dy * dy;
  if (!std::isfinite(length_squared) || length_squared <= kEpsilon) {
    return std::numeric_limits<double>::infinity();
  }
  const double projection = std::clamp(
    ((point.x - from.x) * dx + (point.y - from.y) * dy) / length_squared,
    0.0, 1.0);
  return std::hypot(
    point.x - (from.x + projection * dx),
    point.y - (from.y + projection * dy));
}

}  // namespace

PathValidationResult validateRemainingPath(
  const std::vector<Point2D> & route,
  const Point2D robot_position,
  const std::size_t previous_progress_segment,
  const TerrainEvaluator & terrain)
{
  PathValidationResult result;
  if (
    route.size() < 2U || previous_progress_segment >= route.size() - 1U ||
    !finite(robot_position))
  {
    return result;
  }
  for (const Point2D point : route) {
    if (!finite(point)) {
      return result;
    }
  }

  std::size_t nearest_segment = 0U;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t segment = 0U; segment + 1U < route.size(); ++segment) {
    const double distance = pointToSegmentDistance(
      robot_position, route[segment], route[segment + 1U]);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_segment = segment;
    }
  }
  if (!std::isfinite(nearest_distance)) {
    return result;
  }

  result.progress_segment = std::max(previous_progress_segment, nearest_segment);
  result.distance_to_path_m = nearest_distance;
  for (
    std::size_t segment = result.progress_segment;
    segment + 1U < route.size(); ++segment)
  {
    const EdgeEvaluation edge = terrain.evaluateEdge(
      route[segment], route[segment + 1U]);
    if (!edge.valid || !std::isfinite(edge.cost)) {
      result.reason = edge.reason;
      result.failing_segment = segment;
      return result;
    }
  }

  result.valid = true;
  result.reason = TerrainInvalidReason::kNone;
  result.failing_segment = result.progress_segment;
  return result;
}

}  // namespace rubi_heightmap_wavefront_planner
