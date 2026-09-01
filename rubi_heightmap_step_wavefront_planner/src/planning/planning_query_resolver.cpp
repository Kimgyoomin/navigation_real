#include "rubi_heightmap_step_wavefront_planner/planning/planning_query_resolver.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace rubi_heightmap_step_wavefront_planner
{
namespace
{

constexpr double kDistanceTieTolerance = 1.0e-12;

bool isBetterCandidate(
  const double candidate_distance_m,
  const GridCell candidate_cell,
  const double best_distance_m,
  const GridCell best_cell) noexcept
{
  if (candidate_distance_m < best_distance_m - kDistanceTieTolerance) {
    return true;
  }
  if (std::abs(candidate_distance_m - best_distance_m) > kDistanceTieTolerance) {
    return false;
  }
  if (candidate_cell.x != best_cell.x) {
    return candidate_cell.x < best_cell.x;
  }
  return candidate_cell.y < best_cell.y;
}

}  // namespace

std::optional<PlanningQueryResolution> PlanningQueryResolver::resolve(
  const HeightmapSnapshot & snapshot,
  const StepEvaluator & evaluator,
  const Point2D requested,
  const double search_radius_m,
  const bool snapping_enabled) const
{
  return resolveDetailed(
    snapshot, evaluator, requested, search_radius_m, snapping_enabled).resolution;
}

PlanningQueryResolutionAttempt PlanningQueryResolver::resolveDetailed(
  const HeightmapSnapshot & snapshot,
  const StepEvaluator & evaluator,
  const Point2D requested,
  const double search_radius_m,
  const bool snapping_enabled) const
{
  if (!std::isfinite(search_radius_m) || search_radius_m < 0.0) {
    throw std::invalid_argument("planning query snap radius must be finite and >= 0");
  }

  PlanningQueryResolutionAttempt attempt;
  attempt.requested = requested;

  // Exact valid queries preserve the legacy planner input byte-for-byte.
  const NodeEvaluation requested_evaluation = evaluator.evaluateNode(requested);
  attempt.requested_reason = requested_evaluation.reason;
  if (requested_evaluation.valid) {
    PlanningQueryResolution resolution;
    resolution.requested = requested;
    resolution.effective = requested;
    resolution.requested_valid = true;
    resolution.requested_reason = StepInvalidReason::kNone;
    resolution.effective_evaluation = requested_evaluation;
    attempt.resolution = resolution;
    return attempt;
  }
  if (!snapping_enabled || !std::isfinite(requested.x) || !std::isfinite(requested.y)) {
    return attempt;
  }

  const double radius_in_cells = std::ceil(search_radius_m / snapshot.resolution());
  if (!std::isfinite(radius_in_cells) ||
    radius_in_cells > static_cast<double>(std::numeric_limits<int>::max()))
  {
    throw std::invalid_argument("planning query snap radius exceeds supported grid range");
  }

  // Search lattice centers so the effective query is deterministic and aligned
  // with the exact terrain representation consumed by StepEvaluator.
  const GridCell requested_cell = snapshot.worldToCell(requested);
  const int radius_cells = static_cast<int>(radius_in_cells);
  std::optional<GridCell> best_cell;
  Point2D best_point;
  NodeEvaluation best_evaluation;
  double best_distance_m = std::numeric_limits<double>::infinity();

  for (int offset_y = -radius_cells; offset_y <= radius_cells; ++offset_y) {
    for (int offset_x = -radius_cells; offset_x <= radius_cells; ++offset_x) {
      const GridCell candidate_cell{
        requested_cell.x + offset_x, requested_cell.y + offset_y};
      if (!snapshot.inBounds(candidate_cell)) {
        continue;
      }
      const Point2D candidate_point = snapshot.cellCenter(candidate_cell);
      const double candidate_distance_m = std::hypot(
        candidate_point.x - requested.x, candidate_point.y - requested.y);
      if (candidate_distance_m > search_radius_m + kDistanceTieTolerance) {
        continue;
      }

      // Observed-only acceptance is unsafe: every candidate must pass the full
      // hard-clearance support and discontinuity contract.
      const NodeEvaluation candidate_evaluation = evaluator.evaluateNode(candidate_point);
      ++attempt.evaluated_candidate_count;
      if (!candidate_evaluation.valid) {
        continue;
      }
      if (!best_cell || isBetterCandidate(
          candidate_distance_m, candidate_cell, best_distance_m, *best_cell))
      {
        best_cell = candidate_cell;
        best_point = candidate_point;
        best_evaluation = candidate_evaluation;
        best_distance_m = candidate_distance_m;
      }
    }
  }

  if (!best_cell) {
    return attempt;
  }
  PlanningQueryResolution resolution;
  resolution.requested = requested;
  resolution.effective = best_point;
  resolution.requested_valid = false;
  resolution.snapped = true;
  resolution.snap_distance_m = best_distance_m;
  resolution.requested_reason = requested_evaluation.reason;
  resolution.effective_evaluation = best_evaluation;
  resolution.evaluated_candidate_count = attempt.evaluated_candidate_count;
  attempt.resolution = resolution;
  return attempt;
}

}  // namespace rubi_heightmap_step_wavefront_planner
