#pragma once

#include <cstddef>
#include <limits>
#include <vector>

#include "rubi_heightmap_step_wavefront_planner/step_evaluator.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

struct TerrainPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct PathValidationResult
{
  bool valid{false};
  StepInvalidReason reason{StepInvalidReason::kInvalidInput};
  std::size_t failing_segment{0U};
  Point2D failing_from;
  Point2D failing_to;
  double minimum_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double max_height_jump_m{std::numeric_limits<double>::quiet_NaN()};
  double max_clearance_height_jump_m{std::numeric_limits<double>::quiet_NaN()};
  double observed_support_ratio{std::numeric_limits<double>::quiet_NaN()};
};

std::size_t nearestPathIndex(
  const std::vector<TerrainPoint> & path,
  Point2D position,
  std::size_t previous_index) noexcept;

PathValidationResult validateRemainingPath(
  const std::vector<TerrainPoint> & path,
  std::size_t start_index,
  const StepEvaluator & evaluator);

}  // namespace rubi_heightmap_step_wavefront_planner
