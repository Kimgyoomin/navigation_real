#pragma once

#include <cstddef>
#include <optional>

#include "rubi_heightmap_step_wavefront_planner/heightmap_snapshot.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

struct HeightEvidence
{
  bool valid{false};
  Point2D query;
  GridCell nearest_cell;
  double nearest_distance_m{0.0};
  double nearest_elevation_m{0.0};
  double median_elevation_m{0.0};
  std::size_t observed_cell_count{0U};
  std::size_t outlier_cell_count{0U};
  double outlier_ratio{0.0};
};

struct HeightEvidenceSample
{
  GridCell source_cell;
  double elevation_m{0.0};
  double distance_m{0.0};
};

HeightEvidence queryNodeHeightEvidence(
  const HeightmapSnapshot & snapshot, Point2D query, double radius_m,
  std::size_t min_observed_cells, double max_nearest_distance_m,
  double outlier_threshold_m, double max_outlier_ratio);

std::optional<HeightEvidenceSample> queryEdgeHeight(
  const HeightmapSnapshot & snapshot, Point2D query, double max_query_radius_m);

/** Original TRG median/outlier collision test with deterministic nearest-z assignment. */
HeightEvidence queryOriginalTrgHeightEvidence(
  const HeightmapSnapshot & snapshot, Point2D query, double robot_size_m,
  double height_threshold_m, double collision_threshold);

}  // namespace rubi_heightmap_step_wavefront_planner
