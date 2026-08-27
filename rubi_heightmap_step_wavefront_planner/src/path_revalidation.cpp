#include "rubi_heightmap_step_wavefront_planner/path_revalidation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rubi_heightmap_step_wavefront_planner
{

std::size_t nearestPathIndex(
  const std::vector<TerrainPoint> & path,
  const Point2D position,
  const std::size_t previous_index) noexcept
{
  if (path.empty()) {return 0U;}
  std::size_t best = std::min(previous_index, path.size() - 1U);
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = best; index < path.size(); ++index) {
    const double distance = std::hypot(path[index].x - position.x, path[index].y - position.y);
    if (distance < best_distance) {
      best_distance = distance;
      best = index;
    }
  }
  return best;
}

PathValidationResult validateRemainingPath(
  const std::vector<TerrainPoint> & path,
  const std::size_t start_index,
  const StepEvaluator & evaluator)
{
  PathValidationResult result;
  if (path.empty() || start_index >= path.size()) {return result;}
  const Point2D start{path[start_index].x, path[start_index].y};
  const auto start_evaluation = evaluator.evaluateNode(start);
  if (!start_evaluation.valid) {
    result.reason = start_evaluation.reason;
    result.failing_segment = start_index;
    result.failing_from = start;
    result.failing_to = start;
    result.minimum_clearance_m = start_evaluation.minimum_clearance_m;
    result.max_clearance_height_jump_m = start_evaluation.max_clearance_height_jump_m;
    result.observed_support_ratio = start_evaluation.observed_support_ratio;
    return result;
  }
  for (std::size_t index = start_index + 1U; index < path.size(); ++index) {
    const EdgeEvaluation edge = evaluator.evaluateEdge(
      Point2D{path[index - 1U].x, path[index - 1U].y},
      Point2D{path[index].x, path[index].y});
    if (!edge.valid) {
      result.reason = edge.reason;
      result.failing_segment = index - 1U;
      result.failing_from = {path[index - 1U].x, path[index - 1U].y};
      result.failing_to = {path[index].x, path[index].y};
      result.minimum_clearance_m = edge.minimum_clearance_m;
      result.max_height_jump_m = edge.max_height_jump_m;
      result.max_clearance_height_jump_m = edge.max_clearance_height_jump_m;
      result.observed_support_ratio = edge.observed_support_ratio;
      return result;
    }
  }
  result.valid = true;
  result.reason = StepInvalidReason::kNone;
  return result;
}

}  // namespace rubi_heightmap_step_wavefront_planner
