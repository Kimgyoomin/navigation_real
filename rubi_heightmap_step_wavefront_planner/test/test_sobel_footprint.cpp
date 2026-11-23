#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include "rubi_heightmap_step_wavefront_planner/step_evaluator.hpp"
#include "rubi_heightmap_step_wavefront_planner/planning/grid_path_evaluator.hpp"
#include "rubi_heightmap_step_wavefront_planner/planning/step_grid_astar_planner.hpp"

namespace p = rubi_heightmap_step_wavefront_planner;
namespace
{
constexpr double res = 0.05;
constexpr int width = 101, height = 61;
p::HeightmapSnapshot terrain(double step = 0.09)
{
  std::vector<p::HeightPoint> points;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      points.push_back({0.025 + res*x, 0.025 + res*y, y >= 30 ? step : 0.0});
    }
  }
  return p::HeightmapSnapshot::fromPoints(points, res, 0.001, 10000U);
}
p::CostmapSnapshot costs()
{
  return p::CostmapSnapshot::fromData(width, height, res, 0., 0.,
    std::vector<std::uint8_t>(width*height, 0U));
}
p::StepEvaluatorParameters parameters(bool expanded = true)
{
  p::StepEvaluatorParameters v;
  v.max_crossable_height_jump_m = 0.10;
  v.node_evidence_radius_m = 0.025;
  v.node_min_observed_cells = 1;
  v.node_max_nearest_evidence_distance_m = 0.04;
  v.node_max_height_outlier_ratio = 1.0;
  v.grid_sobel_kernel_size = 5;
  v.grid_sobel_gradient_cost_weight = 2.0;
  v.local_relief_hard_reject_enabled = true;
  v.grid_sobel_footprint_radius_m = expanded ? 0.30 : 0.0;
  v.grid_sobel_footprint_falloff_m = expanded ? 0.10 : 0.0;
  return v;
}
}

TEST(SobelFootprint, ChargesFlatSideWherePointGradientCannotSeeStep)
{
  auto h = terrain(); auto c = costs();
  p::StepEvaluator point(h, c, parameters(false)), expanded(h, c, parameters());
  const p::Point2D a{1.025, 1.225}, b{1.075, 1.225};
  p::GridPathEvaluator raw(point,nullptr), wide(expanded,nullptr);
  const auto before = raw.evaluateSegment(a,b), after = wide.evaluateSegment(a,b);
  ASSERT_TRUE(before.valid); ASSERT_TRUE(after.valid);
  EXPECT_DOUBLE_EQ(before.height_jump_score_m, 0.0);
  EXPECT_DOUBLE_EQ(after.height_jump_score_m, 0.0);
  EXPECT_NEAR(before.sobel_gradient_exposure_m, 0.0, 1e-12);
  EXPECT_GT(after.sobel_gradient_exposure_m, 0.0);
  EXPECT_GT(after.cost, before.cost);
}

TEST(SobelFootprint, FiniteFalloffAndFlatRegion)
{
  auto h=terrain(); auto c=costs(); p::StepEvaluator e(h,c,parameters());
  auto near=e.gridSobelGradientAt({2.025,1.225});
  auto middle=e.gridSobelGradientAt({2.025,1.075});
  auto far=e.gridSobelGradientAt({2.025,0.825});
  ASSERT_TRUE(near); ASSERT_TRUE(middle); ASSERT_TRUE(far);
  EXPECT_GT(*near,*middle); EXPECT_GT(*middle,0.0); EXPECT_NEAR(*far,0.0,1e-12);
  EXPECT_FALSE(e.gridSobelGradientAt({-10.,-10.}));
  auto flat=terrain(0.0); p::StepEvaluator flat_eval(flat,c,parameters());
  EXPECT_NEAR(*flat_eval.gridSobelGradientAt({2.025,1.225}),0.0,1e-12);
}

TEST(SobelFootprint, CrossingRemainsAllowedAndHardLimitUnchanged)
{
  auto h=terrain(); auto c=costs();
  for (bool enabled : {false,true}) {
    p::StepEvaluator e(h,c,parameters(enabled)); p::GridPathEvaluator paths(e,nullptr);
    const auto crossing=paths.evaluateSegment({2.025,1.475},{2.025,1.525});
    EXPECT_TRUE(crossing.valid);
    EXPECT_TRUE(std::isfinite(crossing.cost));
    auto over=terrain(0.11); p::StepEvaluator blocked(over,c,parameters(enabled));
    EXPECT_FALSE(p::GridPathEvaluator(blocked,nullptr).evaluateSegment(
      {2.025,1.475},{2.025,1.525}).valid);
  }
}

TEST(SobelFootprint, GridAndContinuousReplayShareSameExpandedDensity)
{
  auto h=terrain(); auto c=costs(); p::StepEvaluator e(h,c,parameters());
  p::GridPathEvaluator paths(e,nullptr);
  const p::Point2D a{1.025,1.225}, b{1.075,1.225}, mid{1.05,1.225};
  const auto edge=paths.evaluateSegment(a,b);
  const auto left=paths.evaluateSegment(a,mid), right=paths.evaluateSegment(mid,b);
  ASSERT_TRUE(edge.valid); ASSERT_TRUE(left.valid); ASSERT_TRUE(right.valid);
  EXPECT_NEAR(edge.sobel_gradient_exposure_m,
    left.sobel_gradient_exposure_m+right.sobel_gradient_exposure_m,1e-12);
  auto no_cost=parameters(); no_cost.grid_sobel_gradient_cost_weight=0.0;
  p::StepEvaluator zero(h,c,no_cost);
  EXPECT_NEAR(p::GridPathEvaluator(zero,nullptr).evaluateSegment(a,b).cost,0.05,1e-12);
}

TEST(SobelFootprint, PlannerMovesAwayFromParallelEdgeWithoutBlockingCrossing)
{
  auto h=terrain(); auto c=costs();
  const p::Point2D start{0.525,1.225}, goal{4.525,1.225};
  p::StepEvaluator point(h,c,parameters(false)), wide(h,c,parameters());
  const auto before=p::StepGridAStarPlanner({}).plan(point,start,goal);
  const auto after=p::StepGridAStarPlanner({}).plan(wide,start,goal);
  ASSERT_TRUE(before.success); ASSERT_TRUE(after.success);
  p::GridPathEvaluator evaluator(wide,nullptr);
  std::vector<p::TerrainPoint> former_path;
  for (const auto id : before.path_node_ids) {
    const auto & node=before.nodes.at(id);
    former_path.push_back({node.point.x,node.point.y,node.elevation_m});
  }
  const auto former_cost=evaluator.evaluatePolyline(former_path);
  ASSERT_TRUE(former_cost.valid);
  EXPECT_GT(after.path_metrics.length_xy_m,before.path_metrics.length_xy_m);
  EXPECT_LT(after.path_metrics.sobel_gradient_exposure_m,former_cost.sobel_gradient_exposure_m);
  EXPECT_LT(after.path_metrics.total_cost,former_cost.total_cost);
}

TEST(SobelFootprint, InvalidRadiusAndFalloffRejected)
{
  auto h=terrain(); auto c=costs();
  for (double bad : {-0.1,std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN()})
  {
    auto v=parameters(); v.grid_sobel_footprint_radius_m=bad;
    EXPECT_THROW(p::StepEvaluator(h,c,v),std::invalid_argument);
    v=parameters(); v.grid_sobel_footprint_falloff_m=bad;
    EXPECT_THROW(p::StepEvaluator(h,c,v),std::invalid_argument);
  }
}
