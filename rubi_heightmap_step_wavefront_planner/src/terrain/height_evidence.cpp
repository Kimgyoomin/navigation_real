#include "rubi_heightmap_step_wavefront_planner/terrain/height_evidence.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace rubi_heightmap_step_wavefront_planner
{
namespace
{
constexpr double kTieTolerance = 1.0e-12;

void validateRadius(const double value, const char * name)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and >= 0");
  }
}

bool betterNearest(
  const double distance, const GridCell cell,
  const double best_distance, const GridCell best_cell) noexcept
{
  if (distance < best_distance - kTieTolerance) {return true;}
  if (std::abs(distance - best_distance) > kTieTolerance) {return false;}
  return cell.x != best_cell.x ? cell.x < best_cell.x : cell.y < best_cell.y;
}
}  // namespace

HeightEvidence queryNodeHeightEvidence(
  const HeightmapSnapshot & snapshot, const Point2D query, const double radius_m,
  const std::size_t min_observed_cells, const double max_nearest_distance_m,
  const double outlier_threshold_m, const double max_outlier_ratio)
{
  validateRadius(radius_m, "node evidence radius");
  validateRadius(max_nearest_distance_m, "maximum nearest evidence distance");
  validateRadius(outlier_threshold_m, "height outlier threshold");
  if (min_observed_cells == 0U || !std::isfinite(max_outlier_ratio) ||
    max_outlier_ratio < 0.0 || max_outlier_ratio > 1.0 ||
    !std::isfinite(query.x) || !std::isfinite(query.y))
  {
    throw std::invalid_argument("invalid height evidence query parameters");
  }

  HeightEvidence result;
  result.query = query;
  const GridCell center = snapshot.worldToCell(query);
  const int radius_cells = static_cast<int>(std::ceil(radius_m / snapshot.resolution())) + 1;
  std::vector<double> elevations;
  double nearest_distance = std::numeric_limits<double>::infinity();
  GridCell nearest_cell;
  for (int offset_y = -radius_cells; offset_y <= radius_cells; ++offset_y) {
    for (int offset_x = -radius_cells; offset_x <= radius_cells; ++offset_x) {
      const GridCell cell{center.x + offset_x, center.y + offset_y};
      const auto elevation = snapshot.elevation(cell);
      if (!elevation) {continue;}
      const Point2D cell_point = snapshot.cellCenter(cell);
      const double distance = std::hypot(cell_point.x - query.x, cell_point.y - query.y);
      if (distance > radius_m + kTieTolerance) {continue;}
      elevations.push_back(*elevation);
      if (elevations.size() == 1U || betterNearest(
          distance, cell, nearest_distance, nearest_cell))
      {
        nearest_distance = distance;
        nearest_cell = cell;
        result.nearest_elevation_m = *elevation;
      }
    }
  }
  result.observed_cell_count = elevations.size();
  if (elevations.empty()) {return result;}
  result.nearest_cell = nearest_cell;
  result.nearest_distance_m = nearest_distance;
  std::sort(elevations.begin(), elevations.end());
  const std::size_t middle = elevations.size() / 2U;
  result.median_elevation_m = elevations.size() % 2U == 0U ?
    0.5 * (elevations[middle - 1U] + elevations[middle]) : elevations[middle];
  result.outlier_cell_count = static_cast<std::size_t>(std::count_if(
      elevations.begin(), elevations.end(), [&](const double elevation) {
        return std::abs(elevation - result.median_elevation_m) > outlier_threshold_m;
      }));
  result.outlier_ratio = static_cast<double>(result.outlier_cell_count) /
    static_cast<double>(result.observed_cell_count);
  result.valid = result.observed_cell_count >= min_observed_cells &&
    result.nearest_distance_m <= max_nearest_distance_m + kTieTolerance &&
    result.outlier_ratio <= max_outlier_ratio + kTieTolerance;
  return result;
}

std::optional<HeightEvidenceSample> queryEdgeHeight(
  const HeightmapSnapshot & snapshot, const Point2D query, const double max_query_radius_m)
{
  validateRadius(max_query_radius_m, "edge height query radius");
  if (!std::isfinite(query.x) || !std::isfinite(query.y)) {return std::nullopt;}
  const GridCell center = snapshot.worldToCell(query);
  const int radius_cells = static_cast<int>(
    std::ceil(max_query_radius_m / snapshot.resolution())) + 1;
  std::optional<HeightEvidenceSample> best;
  for (int offset_y = -radius_cells; offset_y <= radius_cells; ++offset_y) {
    for (int offset_x = -radius_cells; offset_x <= radius_cells; ++offset_x) {
      const GridCell cell{center.x + offset_x, center.y + offset_y};
      const auto elevation = snapshot.elevation(cell);
      if (!elevation) {continue;}
      const Point2D cell_point = snapshot.cellCenter(cell);
      const double distance = std::hypot(cell_point.x - query.x, cell_point.y - query.y);
      if (distance > max_query_radius_m + kTieTolerance) {continue;}
      if (!best || betterNearest(distance, cell, best->distance_m, best->source_cell)) {
        best = HeightEvidenceSample{cell, *elevation, distance};
      }
    }
  }
  return best;
}

HeightEvidence queryOriginalTrgHeightEvidence(
  const HeightmapSnapshot & snapshot, const Point2D query, const double robot_size_m,
  const double height_threshold_m, const double collision_threshold)
{
  validateRadius(robot_size_m, "TRG robot size");
  validateRadius(height_threshold_m, "TRG height threshold");
  if (!std::isfinite(collision_threshold) || collision_threshold < 0.0 ||
    collision_threshold > 1.0 || !std::isfinite(query.x) || !std::isfinite(query.y))
  {
    throw std::invalid_argument("invalid Original TRG collision query parameters");
  }
  HeightEvidence result;
  result.query = query;
  const GridCell center = snapshot.worldToCell(query);
  const int radius_cells = static_cast<int>(
    std::ceil(robot_size_m / snapshot.resolution())) + 1;
  std::vector<double> elevations;
  double nearest_distance = std::numeric_limits<double>::infinity();
  GridCell nearest_cell;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const GridCell cell{center.x + dx, center.y + dy};
      const auto elevation = snapshot.elevation(cell);
      if (!elevation) {continue;}
      const Point2D observed = snapshot.cellCenter(cell);
      const double distance = std::hypot(observed.x - query.x, observed.y - query.y);
      if (distance > robot_size_m + kTieTolerance) {continue;}
      elevations.push_back(*elevation);
      if (elevations.size() == 1U || betterNearest(
          distance, cell, nearest_distance, nearest_cell))
      {
        nearest_distance = distance;
        nearest_cell = cell;
        result.nearest_elevation_m = *elevation;
      }
    }
  }
  result.observed_cell_count = elevations.size();
  if (elevations.empty()) {return result;}
  result.nearest_cell = nearest_cell;
  result.nearest_distance_m = nearest_distance;
  std::sort(elevations.begin(), elevations.end());
  // Original TRG selects pts[pts.size() / 2], including for even-sized sets.
  result.median_elevation_m = elevations[elevations.size() / 2U];
  result.outlier_cell_count = static_cast<std::size_t>(std::count_if(
      elevations.begin(), elevations.end(), [&](const double elevation) {
        return std::abs(elevation - result.median_elevation_m) > height_threshold_m;
      }));
  result.outlier_ratio = static_cast<double>(result.outlier_cell_count) /
    static_cast<double>(result.observed_cell_count);
  // Original TRG isCollision() rejects strictly greater-than threshold.
  result.valid = !(result.outlier_ratio > collision_threshold);
  return result;
}

}  // namespace rubi_heightmap_step_wavefront_planner
