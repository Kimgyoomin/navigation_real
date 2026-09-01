#pragma once

#include <cstddef>
#include <optional>

#include "rubi_heightmap_step_wavefront_planner/heightmap_snapshot.hpp"
#include "rubi_heightmap_step_wavefront_planner/step_evaluator.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

struct PlanningQueryResolution
{
  Point2D requested;
  Point2D effective;
  bool requested_valid{false};
  bool snapped{false};
  double snap_distance_m{0.0};
  StepInvalidReason requested_reason{StepInvalidReason::kInvalidInput};
  NodeEvaluation effective_evaluation;
  std::size_t evaluated_candidate_count{0U};
};

struct PlanningQueryResolutionAttempt
{
  Point2D requested;
  StepInvalidReason requested_reason{StepInvalidReason::kInvalidInput};
  std::size_t evaluated_candidate_count{0U};
  std::optional<PlanningQueryResolution> resolution;
};

/** @brief Resolves a planning query to the nearest strictly terrain-valid lattice cell. */
class PlanningQueryResolver
{
public:
  /**
   * @brief Resolve a query, returning no value when bounded snapping cannot find a valid cell.
   * @throws std::invalid_argument if search_radius_m is negative or non-finite.
   */
  std::optional<PlanningQueryResolution> resolve(
    const HeightmapSnapshot & snapshot,
    const StepEvaluator & evaluator,
    Point2D requested,
    double search_radius_m,
    bool snapping_enabled) const;

  /** @brief Resolve a query while retaining diagnostics for an unsuccessful search. */
  PlanningQueryResolutionAttempt resolveDetailed(
    const HeightmapSnapshot & snapshot,
    const StepEvaluator & evaluator,
    Point2D requested,
    double search_radius_m,
    bool snapping_enabled) const;
};

}  // namespace rubi_heightmap_step_wavefront_planner
