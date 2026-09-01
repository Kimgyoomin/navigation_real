#pragma once

#include <cstddef>
#include <vector>

#include "rubi_heightmap_step_wavefront_planner/path_revalidation.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

struct PolylineEvaluation
{
  bool valid{false};
  StepInvalidReason reason{StepInvalidReason::kInvalidInput};
  double total_cost{0.0};
  double length_xy_m{0.0};
  double inflation_cost{0.0};
  double height_cost{0.0};
  std::size_t failing_segment{0U};
};

PolylineEvaluation evaluatePolyline(
  const std::vector<TerrainPoint> & path,
  std::size_t start_index,
  const StepEvaluator & evaluator);

bool hasMinimumCostImprovement(
  double current_cost,
  double candidate_cost,
  double minimum_improvement_ratio) noexcept;

}  // namespace rubi_heightmap_step_wavefront_planner
