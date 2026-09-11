#include "rubi_heightmap_step_wavefront_planner/planning/grid_path_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rubi_heightmap_step_wavefront_planner
{
namespace
{
constexpr double kTolerance = 1.0e-12;
}

StepEvaluatorParameters GridPathEvaluator::continuousParameters(
  const StepEvaluator & evaluator)
{
  auto parameters = evaluator.parameters();
  // The legacy Sampling switch must never leak into Grid post-processing.
  // Grid's explicit hard switch controls the same Sobel evidence gate used by
  // direct Grid transitions, while the 5x5 scalar remains a soft line cost.
  parameters.sobel_hard_reject_enabled = parameters.grid_sobel_hard_reject_enabled;
  parameters.sobel_cost_weight = 0.0;
  parameters.grid_sobel_hard_reject_enabled = false;
  return parameters;
}

GridPathEvaluator::GridPathEvaluator(
  const StepEvaluator & evaluator,
  const LocalReliefHazardSnapshot * terrain_hazards)
: evaluator_(evaluator), terrain_hazards_(terrain_hazards),
  continuous_evaluator_(
    evaluator.snapshot(),
    evaluator.costmap() ? *evaluator.costmap() : throw std::invalid_argument(
      "GridPathEvaluator requires hybrid evaluator"),
    continuousParameters(evaluator))
{
  if (evaluator.parameters().grid_terrain_clearance_enabled &&
    (!terrain_hazards_ || !terrain_hazards_->matches(evaluator)))
  {
    throw std::invalid_argument("GridPathEvaluator terrain clearance context unavailable");
  }
}

GridPointEvaluation GridPathEvaluator::evaluatePoint(const Point2D point) const
{
  GridPointEvaluation result;
  const NodeEvaluation node = evaluator_.evaluateNode(point);
  result.reason = node.reason;
  result.elevation_m = node.elevation_m;
  if (!node.valid) {return result;}
  if (evaluator_.parameters().grid_terrain_clearance_enabled) {
    const TerrainClearanceQuery clearance = terrain_hazards_->queryPoint(
      point, evaluator_.parameters().grid_terrain_clearance_distance_m);
    result.minimum_terrain_clearance_m = clearance.minimum_distance_m;
    result.minimum_terrain_clearance_exact = clearance.minimum_distance_exact;
    if (!clearance.evidence_valid) {
      result.reason = StepInvalidReason::kTerrainClearanceEvidenceMissing;
      return result;
    }
    if (clearance.violation) {
      result.reason = StepInvalidReason::kTerrainClearanceViolation;
      return result;
    }
  }
  result.valid = true;
  result.reason = StepInvalidReason::kNone;
  return result;
}

bool GridPathEvaluator::isAdjacentCellCenter(
  const Point2D point, GridCell & cell, NodeEvaluation & node) const
{
  const CostmapSnapshot * costmap = evaluator_.costmap();
  const auto candidate = costmap->worldToCell(point);
  if (!candidate || !costmap->inBounds(*candidate)) {return false;}
  const Point2D center = costmap->cellCenter(*candidate);
  if (std::hypot(center.x - point.x, center.y - point.y) > kTolerance) {return false;}
  cell = *candidate;
  node = evaluator_.evaluateNode(center);
  return true;
}

double GridPathEvaluator::sobelExposure(const Point2D from, const Point2D to) const
{
  const double length = std::hypot(to.x - from.x, to.y - from.y);
  if (length <= kTolerance) {return 0.0;}
  const double spacing = std::min(
    evaluator_.parameters().edge_check_spacing_m,
    0.5 * evaluator_.snapshot().resolution());
  const std::size_t intervals = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::ceil(length / spacing)));
  const double interval_length = length / static_cast<double>(intervals);
  std::optional<double> previous;
  double exposure = 0.0;
  for (std::size_t index = 0U; index <= intervals; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(intervals);
    const auto gradient = continuous_evaluator_.gridSobelGradientAt({
        from.x + ratio * (to.x - from.x), from.y + ratio * (to.y - from.y)});
    if (index > 0U && (previous || gradient)) {
      const double mean = previous && gradient ? 0.5 * (*previous + *gradient) :
        (previous ? *previous : *gradient);
      exposure += interval_length * mean;
    }
    previous = gradient;
  }
  return exposure;
}

bool GridPathEvaluator::applyTerrainClearance(
  const Point2D from, const Point2D to, EdgeEvaluation & edge) const
{
  if (!evaluator_.parameters().grid_terrain_clearance_enabled) {return true;}
  const TerrainClearanceQuery clearance = terrain_hazards_->querySegment(
    from, to, evaluator_.parameters().grid_terrain_clearance_distance_m);
  edge.minimum_terrain_clearance_m = clearance.minimum_distance_m;
  edge.minimum_terrain_clearance_exact = clearance.minimum_distance_exact;
  if (!clearance.evidence_valid) {
    edge.reason = StepInvalidReason::kTerrainClearanceEvidenceMissing;
    edge.valid = false;
    return false;
  }
  if (clearance.violation) {
    edge.reason = StepInvalidReason::kTerrainClearanceViolation;
    edge.valid = false;
    return false;
  }
  return true;
}

EdgeEvaluation GridPathEvaluator::evaluateSegment(
  const Point2D from, const Point2D to) const
{
  GridCell from_cell;
  GridCell to_cell;
  NodeEvaluation from_node;
  NodeEvaluation to_node;
  const bool from_is_center = isAdjacentCellCenter(from, from_cell, from_node);
  const bool to_is_center = isAdjacentCellCenter(to, to_cell, to_node);
  EdgeEvaluation edge;
  if (from_is_center && to_is_center &&
    std::abs(to_cell.x - from_cell.x) <= 1 &&
    std::abs(to_cell.y - from_cell.y) <= 1 && !(from_cell == to_cell))
  {
    edge = evaluator_.evaluateGridTransition(from_cell, to_cell, from_node, to_node);
  } else {
    edge = continuous_evaluator_.evaluateEdge(from, to);
    if (edge.valid) {
      edge.sobel_gradient_exposure_m = sobelExposure(from, to);
      edge.cost += evaluator_.parameters().grid_sobel_gradient_cost_weight *
        edge.sobel_gradient_exposure_m;
      edge.valid = std::isfinite(edge.cost);
      edge.reason = edge.valid ? StepInvalidReason::kNone : StepInvalidReason::kInvalidInput;
    }
  }
  if (!edge.valid) {return edge;}
  applyTerrainClearance(from, to, edge);
  return edge;
}

PolylineEvaluation GridPathEvaluator::evaluatePolyline(
  const std::vector<TerrainPoint> & path, const std::size_t start_index) const
{
  PolylineEvaluation result;
  if (path.empty() || start_index >= path.size()) {return result;}
  const auto start = evaluatePoint({path[start_index].x, path[start_index].y});
  result.minimum_terrain_clearance_m = start.minimum_terrain_clearance_m;
  result.minimum_terrain_clearance_exact = start.minimum_terrain_clearance_exact;
  if (!start.valid) {
    result.reason = start.reason;
    result.failing_segment = start_index;
    result.minimum_terrain_clearance_m = start.minimum_terrain_clearance_m;
    return result;
  }
  for (std::size_t index = start_index + 1U; index < path.size(); ++index) {
    const EdgeEvaluation edge = evaluateSegment(
      {path[index - 1U].x, path[index - 1U].y}, {path[index].x, path[index].y});
    result.minimum_terrain_clearance_m = std::min(
      result.minimum_terrain_clearance_m, edge.minimum_terrain_clearance_m);
    result.minimum_terrain_clearance_exact =
      result.minimum_terrain_clearance_exact && edge.minimum_terrain_clearance_exact;
    if (!edge.valid) {
      result.reason = edge.reason;
      result.failing_segment = index - 1U;
      return result;
    }
    result.total_cost += edge.cost;
    result.length_xy_m += edge.length_xy_m;
    result.inflation_cost += evaluator_.parameters().inflation_cost_weight *
      edge.inflation_score_m;
    result.height_cost += evaluator_.parameters().height_cost_weight * edge.height_jump_score_m;
    result.sobel_gradient_exposure_m += edge.sobel_gradient_exposure_m;
  }
  result.sobel_cost = evaluator_.parameters().grid_sobel_gradient_cost_weight *
    result.sobel_gradient_exposure_m;
  result.valid = std::isfinite(result.total_cost);
  result.reason = result.valid ? StepInvalidReason::kNone : StepInvalidReason::kInvalidInput;
  return result;
}

}  // namespace rubi_heightmap_step_wavefront_planner
