#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>
#include <gtest/gtest.h>
#include "rubi_heightmap_step_wavefront_planner/planning/step_grid_astar_planner.hpp"
namespace planner = rubi_heightmap_step_wavefront_planner;

struct GridFixture
{
  planner::HeightmapSnapshot heights;
  planner::CostmapSnapshot costs;
};

GridFixture gridFixture(
  int width, int height, std::vector<std::uint8_t> raw,
  double raised_z = 0.0, bool raised_column = false)
{
  std::vector<planner::HeightPoint> points;
  for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
    const bool raised = raised_column ? x == width / 2 : (x == width / 2 && y == height / 2);
    points.push_back({0.05 + 0.1 * x, 0.05 + 0.1 * y, raised ? raised_z : 0.0});
  }
  return {planner::HeightmapSnapshot::fromPoints(points, 0.1, 0.001, 10000U),
    planner::CostmapSnapshot::fromData(width, height, 0.1, 0.0, 0.0, std::move(raw))};
}

planner::StepEvaluatorParameters gridParams(double height_weight = 5.0)
{
  planner::StepEvaluatorParameters p;
  p.node_evidence_radius_m = 0.01;
  p.node_min_observed_cells = 1U;
  p.node_max_nearest_evidence_distance_m = 0.01;
  p.edge_height_query_radius_m = 0.075;
  p.edge_max_height_evidence_gap_m = 0.15;
  p.height_cost_weight = height_weight;
  return p;
}

TEST(StepGridAStarPlanner, FlatEightConnectedUsesDiagonalMetricAndOctileHeuristic)
{
  auto fixture = gridFixture(5, 5, std::vector<std::uint8_t>(25U, 0U));
  planner::StepEvaluator evaluator(fixture.heights, fixture.costs, gridParams());
  auto result = planner::StepGridAStarPlanner({}).plan(evaluator, {0.05, 0.05}, {0.45, 0.45});
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.path_metrics.length_xy_m, 4.0 * std::sqrt(2.0) * 0.1, 1e-12);
  EXPECT_EQ(result.statistics.edge_samples_total, 0U);
  EXPECT_GT(result.statistics.grid_transition_evaluations, 0U);
  EXPECT_GE(
    result.statistics.grid_transition_evaluations,
    result.statistics.edge_evaluation_calls);
  EXPECT_NEAR(planner::StepGridAStarPlanner::octileDistance(3, 2), 1.0 + 2.0 * std::sqrt(2.0), 1e-12);
  auto cardinal = planner::StepGridAStarPlanner({false, 1000U, 5000U}).plan(
    evaluator, {0.05, 0.05}, {0.15, 0.05});
  EXPECT_NEAR(cardinal.path_metrics.length_xy_m, 0.1, 1e-12);
}

TEST(StepGridAStarPlanner, DiagonalCornerCuttingIsBlocked)
{
  std::vector<std::uint8_t> raw(9U, 0U);
  raw[1] = 254U;
  raw[3] = 254U;
  auto fixture = gridFixture(3, 3, raw);
  planner::StepEvaluator evaluator(fixture.heights, fixture.costs, gridParams());
  EXPECT_FALSE(planner::StepGridAStarPlanner({}).plan(
      evaluator, {0.05, 0.05}, {0.15, 0.15}).success);
}

TEST(StepGridAStarPlanner, InflationAndHeightWeightsSelectSaferDetours)
{
  std::vector<std::uint8_t> inflated(35U, 0U);
  for (int x = 1; x < 6; ++x) inflated[2 * 7 + x] = 252U;
  auto inflation_fixture = gridFixture(7, 5, inflated);
  planner::StepEvaluator inflation_eval(
    inflation_fixture.heights, inflation_fixture.costs, gridParams());
  auto inflation_path = planner::StepGridAStarPlanner({}).plan(
    inflation_eval, {0.05, 0.25}, {0.65, 0.25});
  ASSERT_TRUE(inflation_path.success);
  EXPECT_TRUE(std::any_of(inflation_path.nodes.begin(), inflation_path.nodes.end(),
      [](const auto & node) {return std::abs(node.point.y - 0.25) > 0.01;}));

  auto step_fixture = gridFixture(7, 5, std::vector<std::uint8_t>(35U, 0U), 0.08, false);
  planner::StepEvaluator short_eval(step_fixture.heights, step_fixture.costs, gridParams(0.0));
  planner::StepEvaluator safe_eval(step_fixture.heights, step_fixture.costs, gridParams(50.0));
  auto short_path = planner::StepGridAStarPlanner({}).plan(
    short_eval, {0.05, 0.25}, {0.65, 0.25});
  auto safe_path = planner::StepGridAStarPlanner({}).plan(
    safe_eval, {0.05, 0.25}, {0.65, 0.25});
  ASSERT_TRUE(short_path.success && safe_path.success);
  EXPECT_GT(short_path.path_metrics.height_score_m, 0.0);
  EXPECT_DOUBLE_EQ(safe_path.path_metrics.height_score_m, 0.0);
  EXPECT_GT(safe_path.path_metrics.length_xy_m, short_path.path_metrics.length_xy_m);
}

TEST(StepGridAStarPlanner, OverLimitBarrierRejectedAndTwentyRunsDeterministic)
{
  auto barrier = gridFixture(7, 5, std::vector<std::uint8_t>(35U, 0U), 0.080001, true);
  planner::StepEvaluator blocked(barrier.heights, barrier.costs, gridParams());
  EXPECT_FALSE(planner::StepGridAStarPlanner({}).plan(
      blocked, {0.05, 0.25}, {0.65, 0.25}).success);

  auto flat = gridFixture(7, 5, std::vector<std::uint8_t>(35U, 0U));
  planner::StepEvaluator evaluator(flat.heights, flat.costs, gridParams());
  const auto reference = planner::StepGridAStarPlanner({}).plan(
    evaluator, {0.05, 0.05}, {0.65, 0.45});
  ASSERT_TRUE(reference.success);
  for (int repeat = 1; repeat < 20; ++repeat) {
    const auto candidate = planner::StepGridAStarPlanner({}).plan(
      evaluator, {0.05, 0.05}, {0.65, 0.45});
    ASSERT_EQ(candidate.nodes.size(), reference.nodes.size());
    EXPECT_DOUBLE_EQ(candidate.path_metrics.total_cost, reference.path_metrics.total_cost);
    for (std::size_t i = 0; i < reference.nodes.size(); ++i) {
      EXPECT_DOUBLE_EQ(candidate.nodes[i].point.x, reference.nodes[i].point.x);
      EXPECT_DOUBLE_EQ(candidate.nodes[i].point.y, reference.nodes[i].point.y);
    }
  }
}

TEST(StepGridAStarPlanner, ReportsSelectedSobelMetricAndSobelOnlyRejections)
{
  constexpr int width = 17;
  constexpr int height = 3;
  std::vector<planner::HeightPoint> points;
  for (int y = -1; y <= height; ++y) {
    for (int x = -1; x <= width; ++x) {
      double elevation = 0.15;
      if (x <= 6) {elevation = 0.0;}
      else if (x == 7) {elevation = 0.03;}
      else if (x == 8) {elevation = 0.09;}
      points.push_back({0.025 + 0.05 * x, 0.025 + 0.05 * y, elevation});
    }
  }
  const auto heights = planner::HeightmapSnapshot::fromPoints(
    points, 0.05, 0.001, 10000U);
  const auto costs = planner::CostmapSnapshot::fromData(
    width, height, 0.05, 0.0, 0.0,
    std::vector<std::uint8_t>(width * height, 0U));
  auto params = gridParams();
  params.max_crossable_height_jump_m = 0.10;
  params.edge_height_query_radius_m = 0.04;
  params.edge_max_height_evidence_gap_m = 0.10;
  params.grid_sobel_hard_reject_enabled = false;
  const auto baseline = planner::StepGridAStarPlanner({}).plan(
    planner::StepEvaluator(heights, costs, params),
    {0.025, 0.075}, {0.825, 0.075});
  ASSERT_TRUE(baseline.success);
  EXPECT_LT(baseline.path_metrics.max_height_jump_m, 0.10);
  EXPECT_GT(baseline.path_metrics.max_sobel_equivalent_step_height_m, 0.10);
  EXPECT_GT(baseline.path_metrics.max_sobel_gradient, 0.0);

  params.grid_sobel_hard_reject_enabled = true;
  const auto sobel = planner::StepGridAStarPlanner({}).plan(
    planner::StepEvaluator(heights, costs, params),
    {0.025, 0.075}, {0.825, 0.075});
  EXPECT_FALSE(sobel.success);
  EXPECT_EQ(sobel.statistics.adjacent_step_rejects, 0U);
  EXPECT_GT(sobel.statistics.sobel_step_rejects, 0U);
  EXPECT_EQ(sobel.statistics.both_step_rejects, 0U);
}

TEST(StepGridAStarPlanner, SobelSoftCostPrefersLongerLowGradientPath)
{
  constexpr int width = 27;
  constexpr int height = 7;
  constexpr double resolution = 0.1;
  std::vector<planner::HeightPoint> points;
  for (int y = -1; y <= height; ++y) {
    for (int x = -1; x <= width; ++x) {
      double elevation = 0.0;
      if (y >= 2 && y <= 4 && x >= 1 && x <= 25) {
        const int period_phase = (x - 1) % 8;
        const int ramp_phase = period_phase <= 4 ? period_phase : 8 - period_phase;
        elevation = 0.02 * static_cast<double>(ramp_phase);
      }
      points.push_back({
        0.05 + resolution * x, 0.05 + resolution * y, elevation});
    }
  }
  const auto heights = planner::HeightmapSnapshot::fromPoints(
    points, resolution, 0.001, 10000U);
  const auto costs = planner::CostmapSnapshot::fromData(
    width, height, resolution, 0.0, 0.0,
    std::vector<std::uint8_t>(width * height, 0U));
  auto parameters = gridParams(0.0);
  parameters.max_crossable_height_jump_m = 0.10;
  const std::vector<double> weights{0.0, 0.25, 0.5, 1.0, 2.0, 4.0};
  std::vector<planner::PlanResult> sweep;
  for (const double weight : weights) {
    parameters.grid_sobel_gradient_cost_weight = weight;
    sweep.push_back(planner::StepGridAStarPlanner({}).plan(
        planner::StepEvaluator(heights, costs, parameters),
        {0.15, 0.35}, {2.55, 0.35}));
    ASSERT_TRUE(sweep.back().success);
    std::cout << "SOBEL_SWEEP weight=" << weight
              << " length_m=" << sweep.back().path_metrics.length_xy_m
              << " exposure_m=" << sweep.back().path_metrics.sobel_gradient_exposure_m
              << " sobel_cost=" << sweep.back().path_metrics.sobel_cost
              << " total_cost=" << sweep.back().path_metrics.total_cost
              << " planning_ms=" << sweep.back().core_total_time_ms << '\n';
  }
  const auto & direct = sweep.front();
  const auto & detour = sweep.back();
  EXPECT_NEAR(direct.path_metrics.length_xy_m, 2.4, 1e-12);
  EXPECT_GT(direct.path_metrics.sobel_gradient_exposure_m, 0.0);
  EXPECT_GT(detour.path_metrics.length_xy_m, direct.path_metrics.length_xy_m);
  EXPECT_LT(
    detour.path_metrics.sobel_gradient_exposure_m,
    direct.path_metrics.sobel_gradient_exposure_m);
  EXPECT_NEAR(
    detour.path_metrics.total_cost,
    detour.path_metrics.length_xy_m + detour.path_metrics.sobel_cost, 1e-12);
}
