#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "rubi_heightmap_step_wavefront_planner/planning/grid_path_evaluator.hpp"
#include "rubi_heightmap_step_wavefront_planner/planning/step_grid_astar_planner.hpp"
#include "rubi_heightmap_step_wavefront_planner/planning/tracking_path_refiner.hpp"
#include "rubi_heightmap_step_wavefront_planner/terrain/local_relief_hazard_snapshot.hpp"

namespace planner = rubi_heightmap_step_wavefront_planner;

namespace
{
planner::StepEvaluatorParameters parameters()
{
  planner::StepEvaluatorParameters value;
  value.node_evidence_radius_m = 0.01;
  value.node_min_observed_cells = 1U;
  value.node_max_nearest_evidence_distance_m = 0.01;
  value.edge_height_query_radius_m = 0.075;
  value.edge_max_height_evidence_gap_m = 0.15;
  value.local_relief_hard_reject_enabled = true;
  value.local_relief_threshold_m = 0.10;
  value.local_relief_first_window_radius_m = 0.10;
  value.local_relief_second_window_radius_m = 0.10;
  value.local_relief_min_observed_ratio = 0.70;
  value.local_relief_critical_cell_count = 3U;
  return value;
}

struct Fixture
{
  planner::HeightmapSnapshot heightmap;
  planner::CostmapSnapshot costmap;
};

Fixture flatFixture(const double resolution = 0.05, const int cells = 25)
{
  std::vector<planner::HeightPoint> points;
  for (int y = 0; y < cells; ++y) {
    for (int x = 0; x < cells; ++x) {
      points.push_back({
        -0.60 + resolution * x, -0.60 + resolution * y, 0.0});
    }
  }
  return {
    planner::HeightmapSnapshot::fromPoints(points, resolution, 0.001, 100000U),
    planner::CostmapSnapshot::fromData(
      cells, cells, resolution, -0.625, -0.625,
      std::vector<std::uint8_t>(static_cast<std::size_t>(cells * cells), 0U))};
}

Fixture raisedPatchFixture()
{
  constexpr int width = 35;
  constexpr int height = 25;
  constexpr double resolution = 0.05;
  std::vector<planner::HeightPoint> points;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const bool raised = x >= 15 && x <= 19 && y >= 10 && y <= 14;
      points.push_back({
        0.025 + resolution * x, 0.025 + resolution * y,
        raised ? 0.15 : 0.0});
    }
  }
  return {
    planner::HeightmapSnapshot::fromPoints(points, resolution, 0.001, 100000U),
    planner::CostmapSnapshot::fromData(
      width, height, resolution, 0.0, 0.0,
      std::vector<std::uint8_t>(static_cast<std::size_t>(width * height), 0U))};
}
}  // namespace

TEST(LocalReliefHazardGeometry, AxisBoundaryUsesClosedCellAreaNotCellCenter)
{
  constexpr double resolution = 0.05;
  const planner::Point2D center{0.0, 0.0};
  EXPECT_NEAR(
    planner::LocalReliefHazardSnapshot::pointToCellAreaDistance(
      {0.025 + 0.199, 0.0}, center, resolution), 0.199, 1.0e-12);
  EXPECT_NEAR(
    planner::LocalReliefHazardSnapshot::pointToCellAreaDistance(
      {0.025 + 0.200, 0.0}, center, resolution), 0.200, 1.0e-12);
  EXPECT_NEAR(
    planner::LocalReliefHazardSnapshot::pointToCellAreaDistance(
      {0.025 + 0.201, 0.0}, center, resolution), 0.201, 1.0e-12);

  // A point 0.20 m from the center is only 0.175 m from the cell area.
  EXPECT_NEAR(
    planner::LocalReliefHazardSnapshot::pointToCellAreaDistance(
      {0.20, 0.0}, center, resolution), 0.175, 1.0e-12);
}

TEST(LocalReliefHazardGeometry, CornerDistanceAndContinuousSegmentAreExact)
{
  constexpr double resolution = 0.05;
  const planner::Point2D center{0.0, 0.0};
  const double offset = 0.025 + 0.20 / std::sqrt(2.0);
  EXPECT_NEAR(
    planner::LocalReliefHazardSnapshot::pointToCellAreaDistance(
      {offset, offset}, center, resolution), 0.20, 1.0e-12);

  // Both endpoints are farther than 0.20 m, while the continuous segment
  // crosses the seed square and therefore has zero clearance.
  EXPECT_GT(
    planner::LocalReliefHazardSnapshot::pointToCellAreaDistance(
      {-0.30, 0.10}, center, resolution), 0.20);
  EXPECT_GT(
    planner::LocalReliefHazardSnapshot::pointToCellAreaDistance(
      {0.30, -0.10}, center, resolution), 0.20);
  EXPECT_DOUBLE_EQ(
    planner::LocalReliefHazardSnapshot::segmentToCellAreaDistance(
      {-0.30, 0.10}, {0.30, -0.10}, center, resolution), 0.0);

  EXPECT_NEAR(
    planner::LocalReliefHazardSnapshot::segmentToCellAreaDistance(
      {0.225, -0.4}, {0.225, 0.4}, center, resolution), 0.20, 1.0e-12);
  EXPECT_DOUBLE_EQ(
    planner::LocalReliefHazardSnapshot::segmentToCellAreaDistance(
      {0.0, 0.0}, {0.0, 0.0}, center, resolution), 0.0);
}

TEST(LocalReliefHazardGeometry, PhysicalDistanceIsResolutionIndependent)
{
  for (const double resolution : {0.025, 0.05, 0.10}) {
    const double coordinate = 0.5 * resolution + 0.20;
    EXPECT_NEAR(
      planner::LocalReliefHazardSnapshot::pointToCellAreaDistance(
        {coordinate, 0.0}, {0.0, 0.0}, resolution), 0.20, 1.0e-12);
  }
}

TEST(LocalReliefHazardSnapshot, FullyObservedFlatMapHasNoSeedsAndCachesByReliefSignature)
{
  const auto fixture = flatFixture();
  const auto p = parameters();
  const planner::StepEvaluator evaluator(fixture.heightmap, fixture.costmap, p);
  const auto hazards = planner::LocalReliefHazardSnapshot::build(evaluator);
  EXPECT_EQ(hazards.seedCount(), 0U);
  EXPECT_EQ(hazards.evidenceInvalidCount(), 0U);
  EXPECT_TRUE(hazards.matches(evaluator));
  const auto query = hazards.querySegment({-0.4, -0.4}, {0.4, 0.4}, 0.20);
  EXPECT_TRUE(query.evidence_valid);
  EXPECT_FALSE(query.violation);
  EXPECT_TRUE(std::isinf(query.minimum_distance_m));

  auto changed = p;
  changed.local_relief_critical_cell_count = 4U;
  EXPECT_FALSE(hazards.matches(
      planner::StepEvaluator(fixture.heightmap, fixture.costmap, changed)));

  std::vector<planner::HeightPoint> changed_points;
  for (int y = 0; y < 25; ++y) {
    for (int x = 0; x < 25; ++x) {
      changed_points.push_back({
        -0.60 + 0.05 * x, -0.60 + 0.05 * y,
        (x == 12 && y == 12) ? 0.01 : 0.0});
    }
  }
  const auto changed_heightmap = planner::HeightmapSnapshot::fromPoints(
    changed_points, 0.05, 0.001, 100000U);
  EXPECT_FALSE(hazards.matches(
      planner::StepEvaluator(changed_heightmap, fixture.costmap, p)));

  const auto changed_geometry = flatFixture(0.10, 13);
  EXPECT_FALSE(hazards.matches(planner::StepEvaluator(
      changed_geometry.heightmap, changed_geometry.costmap, p)));
}

TEST(LocalReliefHazardSnapshot, MissingReliefEvidenceIsNotTreatedAsSafe)
{
  constexpr int cells = 11;
  std::vector<planner::HeightPoint> points;
  for (int y = 0; y < cells; ++y) {
    for (int x = 0; x < cells; ++x) {
      if (x == 5 && y == 5) {continue;}
      points.push_back({0.025 + 0.05 * x, 0.025 + 0.05 * y, 0.0});
    }
  }
  const auto heightmap = planner::HeightmapSnapshot::fromPoints(
    points, 0.05, 0.001, 10000U);
  const auto costmap = planner::CostmapSnapshot::fromData(
    cells, cells, 0.05, 0.0, 0.0,
    std::vector<std::uint8_t>(cells * cells, 0U));
  auto p = parameters();
  p.local_relief_min_observed_ratio = 1.0;
  const planner::StepEvaluator evaluator(heightmap, costmap, p);
  const auto hazards = planner::LocalReliefHazardSnapshot::build(evaluator);
  EXPECT_GT(hazards.evidenceInvalidCount(), 0U);
  const auto query = hazards.querySegment({0.125, 0.275}, {0.425, 0.275}, 0.20);
  EXPECT_FALSE(query.evidence_valid);
  EXPECT_FALSE(query.violation);
}

TEST(GridPathEvaluator, RawAdjacentReplayEqualsAStarSearchCostAndDisabledRefinerIsIdentity)
{
  const auto fixture = flatFixture(0.05, 25);
  auto p = parameters();
  p.grid_sobel_kernel_size = 5;
  p.grid_sobel_gradient_cost_weight = 2.0;
  p.grid_terrain_clearance_enabled = false;
  planner::StepEvaluator evaluator(fixture.heightmap, fixture.costmap, p);
  const auto plan = planner::StepGridAStarPlanner({}).plan(
    evaluator, {-0.50, -0.50}, {0.50, 0.50});
  ASSERT_TRUE(plan.success);
  std::vector<planner::TerrainPoint> raw;
  for (const auto id : plan.path_node_ids) {
    const auto & node = plan.nodes[id];
    raw.push_back({node.point.x, node.point.y, node.elevation_m});
  }
  const planner::GridPathEvaluator grid_evaluator(evaluator, nullptr);
  const auto replay = grid_evaluator.evaluatePolyline(raw);
  ASSERT_TRUE(replay.valid);
  EXPECT_NEAR(replay.total_cost, plan.path_metrics.grid_search_cost, 1.0e-10);

  planner::TrackingPathRefinerParameters refiner_parameters;
  refiner_parameters.enabled = false;
  const auto refined = planner::TrackingPathRefiner(refiner_parameters).refineGrid(
    raw, grid_evaluator);
  ASSERT_TRUE(refined.success);
  EXPECT_EQ(refined.path.size(), raw.size());
  EXPECT_NEAR(refined.tracking_cost, replay.total_cost, 1.0e-10);
}

TEST(GridTerrainClearanceConfiguration, EnabledRequiresLocalReliefHardGate)
{
  const auto fixture = flatFixture();
  auto p = parameters();
  p.local_relief_hard_reject_enabled = false;
  p.grid_terrain_clearance_enabled = true;
  EXPECT_THROW(
    planner::StepEvaluator(fixture.heightmap, fixture.costmap, p),
    std::invalid_argument);
}

TEST(GridTerrainClearancePlanning, ClearanceOffPreservesSearchAndOnMakesExactDetour)
{
  const auto fixture = raisedPatchFixture();
  auto off_parameters = parameters();
  off_parameters.grid_sobel_kernel_size = 5;
  off_parameters.grid_sobel_gradient_cost_weight = 2.0;
  off_parameters.grid_terrain_clearance_enabled = false;
  planner::StepEvaluator off_evaluator(
    fixture.heightmap, fixture.costmap, off_parameters);
  const auto hazards = planner::LocalReliefHazardSnapshot::build(off_evaluator);
  ASSERT_GT(hazards.seedCount(), 0U);
  const auto top_seed = *std::max_element(
    hazards.seedCells().begin(), hazards.seedCells().end(),
    [](const auto & left, const auto & right) {
      return left.y < right.y || (left.y == right.y && left.x < right.x);
    });
  const auto top_center = fixture.heightmap.cellCenter(top_seed);
  const auto at_distance = [&](const double distance) {
      return hazards.queryPoint(
        {top_center.x, top_center.y + 0.025 + distance}, 0.20);
    };
  const auto inside = at_distance(0.199);
  const auto boundary = at_distance(0.200);
  const auto outside = at_distance(0.201);
  ASSERT_TRUE(inside.evidence_valid && boundary.evidence_valid && outside.evidence_valid);
  EXPECT_TRUE(inside.violation);
  EXPECT_TRUE(boundary.violation);  // conservative equality policy
  EXPECT_FALSE(outside.violation);
  const auto off_without_context = planner::StepGridAStarPlanner({}).plan(
    off_evaluator, {0.125, 0.625}, {1.625, 0.625});
  const auto off_with_context = planner::StepGridAStarPlanner({}).plan(
    off_evaluator, {0.125, 0.625}, {1.625, 0.625}, &hazards);
  ASSERT_TRUE(off_without_context.success);
  ASSERT_TRUE(off_with_context.success);
  EXPECT_EQ(off_without_context.path_node_ids, off_with_context.path_node_ids);
  EXPECT_DOUBLE_EQ(
    off_without_context.path_metrics.total_cost,
    off_with_context.path_metrics.total_cost);

  auto on_parameters = off_parameters;
  on_parameters.grid_terrain_clearance_enabled = true;
  on_parameters.grid_terrain_clearance_distance_m = 0.20;
  planner::StepEvaluator on_evaluator(fixture.heightmap, fixture.costmap, on_parameters);
  ASSERT_TRUE(hazards.matches(on_evaluator));
  const auto on = planner::StepGridAStarPlanner({}).plan(
    on_evaluator, {0.125, 0.625}, {1.625, 0.625}, &hazards);
  ASSERT_TRUE(on.success) << on.message;
  EXPECT_GT(on.statistics.grid_terrain_clearance_rejects, 0U);
  EXPECT_GT(on.path_metrics.minimum_terrain_clearance_m, 0.20);
  EXPECT_GT(on.path_metrics.length_xy_m, off_without_context.path_metrics.length_xy_m);

  std::vector<planner::TerrainPoint> raw;
  for (const auto id : on.path_node_ids) {
    const auto & node = on.nodes[id];
    raw.push_back({node.point.x, node.point.y, node.elevation_m});
  }
  const planner::GridPathEvaluator grid_evaluator(on_evaluator, &hazards);
  const auto independent_replay = grid_evaluator.evaluatePolyline(raw);
  ASSERT_TRUE(independent_replay.valid);
  EXPECT_GT(independent_replay.minimum_terrain_clearance_m, 0.20);
  EXPECT_NEAR(independent_replay.total_cost, on.path_metrics.grid_search_cost, 1.0e-10);

  const auto start_equals_goal = planner::StepGridAStarPlanner({}).plan(
    on_evaluator, {0.125, 0.125}, {0.125, 0.125}, &hazards);
  ASSERT_TRUE(start_equals_goal.success);
  EXPECT_TRUE(std::isfinite(
      start_equals_goal.path_metrics.minimum_terrain_clearance_m));
}
