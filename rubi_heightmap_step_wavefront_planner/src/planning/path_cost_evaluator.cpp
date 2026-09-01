#include "rubi_heightmap_step_wavefront_planner/planning/path_cost_evaluator.hpp"

#include <cmath>

namespace rubi_heightmap_step_wavefront_planner
{

PolylineEvaluation evaluatePolyline(
  const std::vector<TerrainPoint> & path,
  const std::size_t start_index,
  const StepEvaluator & evaluator)
{
  PolylineEvaluation result;
  if (path.empty() || start_index >= path.size()) {return result;}
  const auto node = evaluator.evaluateNode({path[start_index].x, path[start_index].y});
  if (!node.valid) {
    result.reason = node.reason;
    result.failing_segment = start_index;
    return result;
  }
  for (std::size_t index = start_index + 1U; index < path.size(); ++index) {
    const auto edge = evaluator.evaluateEdge(
      {path[index - 1U].x, path[index - 1U].y},
      {path[index].x, path[index].y});
    if (!edge.valid) {
      result.reason = edge.reason;
      result.failing_segment = index - 1U;
      return result;
    }
    result.total_cost += edge.cost;
    result.length_xy_m += edge.length_xy_m;
    result.inflation_cost += evaluator.parameters().inflation_cost_weight *
      edge.inflation_score_m;
    result.height_cost += evaluator.parameters().height_cost_weight *
      edge.height_jump_score_m;
  }
  result.valid = std::isfinite(result.total_cost);
  result.reason = result.valid ? StepInvalidReason::kNone : StepInvalidReason::kInvalidInput;
  return result;
}

bool hasMinimumCostImprovement(
  const double current_cost, const double candidate_cost,
  const double minimum_improvement_ratio) noexcept
{
  if (!std::isfinite(current_cost) || !std::isfinite(candidate_cost) ||
    !std::isfinite(minimum_improvement_ratio) || current_cost <= 0.0 ||
    candidate_cost < 0.0 || minimum_improvement_ratio < 0.0)
  {
    return false;
  }
  return candidate_cost < current_cost &&
    (current_cost - candidate_cost) / current_cost + 1.0e-12 >= minimum_improvement_ratio;
}

}  // namespace rubi_heightmap_step_wavefront_planner
