#pragma once

#include <vector>

#include "rubi_heightmap_step_wavefront_planner/planning/path_cost_evaluator.hpp"
#include "rubi_heightmap_step_wavefront_planner/terrain/local_relief_hazard_snapshot.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

struct GridPointEvaluation
{
  bool valid{false};
  StepInvalidReason reason{StepInvalidReason::kInvalidInput};
  double elevation_m{0.0};
  double minimum_terrain_clearance_m{std::numeric_limits<double>::infinity()};
  bool minimum_terrain_clearance_exact{true};
};

class GridPathEvaluator
{
public:
  GridPathEvaluator(
    const StepEvaluator & evaluator,
    const LocalReliefHazardSnapshot * terrain_hazards);

  GridPointEvaluation evaluatePoint(Point2D point) const;
  EdgeEvaluation evaluateSegment(Point2D from, Point2D to) const;
  PolylineEvaluation evaluatePolyline(
    const std::vector<TerrainPoint> & path, std::size_t start_index = 0U) const;

private:
  static StepEvaluatorParameters continuousParameters(const StepEvaluator & evaluator);
  bool isAdjacentCellCenter(
    Point2D point, GridCell & cell, NodeEvaluation & node) const;
  double sobelExposure(Point2D from, Point2D to) const;
  bool applyTerrainClearance(Point2D from, Point2D to, EdgeEvaluation & edge) const;

  const StepEvaluator & evaluator_;
  const LocalReliefHazardSnapshot * terrain_hazards_{nullptr};
  StepEvaluator continuous_evaluator_;
};

}  // namespace rubi_heightmap_step_wavefront_planner
