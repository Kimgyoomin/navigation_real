#pragma once

#include <cstddef>
#include <vector>

#include "rubi_heightmap_wavefront_planner/terrain_evaluator.hpp"

namespace rubi_heightmap_wavefront_planner
{

struct PathValidationResult
{
  bool valid{false};
  TerrainInvalidReason reason{TerrainInvalidReason::kInvalidInput};
  std::size_t failing_segment{0U};
  std::size_t progress_segment{0U};
  double distance_to_path_m{0.0};
};

/**
 * @brief Validate only the route that has not yet been passed by the robot.
 *
 * The nearest segment is projected in XY and never moves the supplied progress
 * index backwards. Every remaining segment is evaluated through the shared
 * TerrainEvaluator edge contract.
 */
PathValidationResult validateRemainingPath(
  const std::vector<Point2D> & route,
  Point2D robot_position,
  std::size_t previous_progress_segment,
  const TerrainEvaluator & terrain);

}  // namespace rubi_heightmap_wavefront_planner
