#include <cassert>
#include <iostream>
#include <vector>

#include "rubi_heightmap_step_wavefront_planner/step_wavefront_planner.hpp"

using namespace rubi_heightmap_step_wavefront_planner;

int main()
{
  std::vector<HeightPoint> points;
  for (int y = -10; y <= 10; ++y) {
    for (int x = -10; x <= 10; ++x) {
      points.push_back({0.05 * x, 0.05 * y, 0.0});
    }
  }
  const auto snapshot = HeightmapSnapshot::fromPoints(points, 0.05, 0.01, 10000U);
  StepEvaluatorParameters evaluator_parameters;
  evaluator_parameters.hard_clearance_radius_m = 0.10;
  const StepEvaluator evaluator(snapshot, evaluator_parameters);
  StepWavefrontParameters planner_parameters;
  planner_parameters.post_goal_expansions = 2U;
  const StepWavefrontPlanner planner(planner_parameters);
  const auto result = planner.plan(evaluator, {-0.30, 0.0}, {0.30, 0.0});
  assert(result.success);
  assert(!result.path_node_ids.empty());
  std::cout << "core_smoke: PASS\n";
}
