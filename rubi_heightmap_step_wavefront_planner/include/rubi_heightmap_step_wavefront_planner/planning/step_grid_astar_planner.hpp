#pragma once

#include <cstddef>

#include "rubi_heightmap_step_wavefront_planner/planning/plan_result.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

struct GridAStarParameters
{
  bool allow_diagonal{true};
  std::size_t max_expanded_states{500000U};
  std::size_t max_planning_time_ms{5000U};
};

class StepGridAStarPlanner
{
public:
  explicit StepGridAStarPlanner(GridAStarParameters parameters);
  PlanResult plan(const StepEvaluator & evaluator, Point2D start, Point2D goal) const;

  static double octileDistance(int dx, int dy) noexcept;

private:
  GridAStarParameters parameters_;
};

}  // namespace rubi_heightmap_step_wavefront_planner
