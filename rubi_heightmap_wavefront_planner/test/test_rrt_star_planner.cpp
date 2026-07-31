#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "rubi_heightmap_wavefront_planner/rrt_star_planner.hpp"

namespace rubi_heightmap_wavefront_planner
{
namespace
{

NodeEvaluation flatNode(const Point2D &)
{
  NodeEvaluation evaluation;
  evaluation.valid = true;
  evaluation.reason = TerrainInvalidReason::kNone;
  evaluation.elevation_m = 0.0;
  evaluation.footprint_observed_ratio = 1.0;
  evaluation.surface.valid = true;
  evaluation.surface.slope_deg = 0.0;
  evaluation.surface.roughness_m = 0.0;
  evaluation.surface.normal_z = 1.0;
  return evaluation;
}

EdgeEvaluation flatEdge(const Point2D & from, const Point2D & to)
{
  EdgeEvaluation evaluation;
  const double length = std::hypot(to.x - from.x, to.y - from.y);
  if (length <= 1.0e-12) {
    return evaluation;
  }
  evaluation.valid = true;
  evaluation.reason = TerrainInvalidReason::kNone;
  evaluation.sample_count = 2U;
  evaluation.length_xy_m = length;
  evaluation.length_3d_m = length;
  evaluation.max_step_m = 0.0;
  evaluation.max_slope_deg = 0.0;
  evaluation.mean_slope_deg = 0.0;
  evaluation.max_roughness_m = 0.0;
  evaluation.min_footprint_observed_ratio = 1.0;
  evaluation.cost = length;
  return evaluation;
}

RrtStarParameters testParameters()
{
  RrtStarParameters parameters;
  parameters.max_iterations = 500U;
  parameters.goal_bias = 0.10;
  parameters.steer_distance_m = 0.75;
  parameters.rewire_radius_min_m = 0.75;
  parameters.rewire_radius_max_m = 2.00;
  parameters.goal_connection_distance_m = 0.80;
  parameters.max_nodes = 1000U;
  parameters.max_planning_time_ms = 0U;
  parameters.stop_on_first_solution = false;
  parameters.random_seed = 17U;
  parameters.slope_normalization_deg = 15.0;
  parameters.risk_weights.distance = 1.0;
  parameters.risk_weights.slope = 0.0;
  parameters.risk_weights.step = 0.0;
  return parameters;
}

double pathCost(const PlanResult & result)
{
  double total = 0.0;
  for (std::size_t index = 1U; index < result.path.size(); ++index) {
    total += std::hypot(
      result.path[index].x - result.path[index - 1U].x,
      result.path[index].y - result.path[index - 1U].y);
  }
  return total;
}

TEST(RrtStarPlanner, FixedSeedProducesIdenticalTreeAndPath)
{
  const RrtStarPlanner planner(testParameters());
  const SamplingBounds bounds{0.0, 8.0, 0.0, 6.0};

  const PlanResult first = planner.plan(
    {0.5, 0.5}, {7.5, 5.5}, bounds, flatNode, flatEdge);
  const PlanResult second = planner.plan(
    {0.5, 0.5}, {7.5, 5.5}, bounds, flatNode, flatEdge);

  ASSERT_TRUE(first.success) << first.message;
  ASSERT_EQ(first.nodes.size(), second.nodes.size());
  ASSERT_EQ(first.edges.size(), second.edges.size());
  ASSERT_EQ(first.path_node_ids, second.path_node_ids);
  EXPECT_GT(first.rewires, 0U);
  EXPECT_EQ(first.rewires, second.rewires);
  for (std::size_t index = 0U; index < first.nodes.size(); ++index) {
    EXPECT_DOUBLE_EQ(first.nodes[index].point.x, second.nodes[index].point.x);
    EXPECT_DOUBLE_EQ(first.nodes[index].point.y, second.nodes[index].point.y);
  }
  for (std::size_t index = 0U; index < first.edges.size(); ++index) {
    EXPECT_EQ(first.edges[index].from, second.edges[index].from);
    EXPECT_EQ(first.edges[index].to, second.edges[index].to);
    EXPECT_DOUBLE_EQ(first.edges[index].cost, second.edges[index].cost);
  }
  EXPECT_EQ(first.expansions, testParameters().max_iterations);
  EXPECT_EQ(first.termination, WavefrontTermination::kMaxExpansionsReached);
}

TEST(RrtStarPlanner, TreeNeverContainsTerrainInvalidBarrierEdge)
{
  auto parameters = testParameters();
  parameters.max_iterations = 1800U;
  parameters.random_seed = 31U;
  const RrtStarPlanner planner(parameters);
  const SamplingBounds bounds{0.0, 6.0, 0.0, 4.0};

  const auto barrier_edge = [](const Point2D & from, const Point2D & to) {
      EdgeEvaluation evaluation = flatEdge(from, to);
      const bool crosses =
        (from.x < 3.0 && to.x >= 3.0) ||
        (to.x < 3.0 && from.x >= 3.0);
      if (crosses) {
        const double interpolation = (3.0 - from.x) / (to.x - from.x);
        const double crossing_y =
          from.y + interpolation * (to.y - from.y);
        if (crossing_y < 3.1) {
          evaluation.valid = false;
          evaluation.reason = TerrainInvalidReason::kStepLimit;
        }
      }
      return evaluation;
    };

  const PlanResult result = planner.plan(
    {0.5, 1.0}, {5.5, 1.0}, bounds, flatNode, barrier_edge);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_GT(
    result.reject_counts.expansion_edge_invalid +
    result.reject_counts.merge_edge_invalid +
    result.reject_counts.goal_edge_invalid,
    0U);
  for (const GraphEdge & edge : result.edges) {
    const Point2D from = {
      result.nodes[edge.from].point.x, result.nodes[edge.from].point.y};
    const Point2D to = {
      result.nodes[edge.to].point.x, result.nodes[edge.to].point.y};
    EXPECT_TRUE(barrier_edge(from, to).valid);
  }
}

TEST(RrtStarPlanner, AdditionalIterationsDoNotIncreaseBestPathCost)
{
  auto short_parameters = testParameters();
  short_parameters.max_iterations = 100U;
  short_parameters.goal_bias = 0.20;
  auto long_parameters = short_parameters;
  long_parameters.max_iterations = 800U;

  const SamplingBounds bounds{0.0, 8.0, 0.0, 6.0};
  const PlanResult short_result = RrtStarPlanner(short_parameters).plan(
    {0.5, 0.5}, {7.5, 5.5}, bounds, flatNode, flatEdge);
  const PlanResult long_result = RrtStarPlanner(long_parameters).plan(
    {0.5, 0.5}, {7.5, 5.5}, bounds, flatNode, flatEdge);

  ASSERT_TRUE(short_result.success) << short_result.message;
  ASSERT_TRUE(long_result.success) << long_result.message;
  EXPECT_LE(pathCost(long_result), pathCost(short_result) + 1.0e-10);
}

TEST(RrtStarPlanner, GoalBiasCanConnectExactGoalWithoutDuplicateGoalNode)
{
  auto parameters = testParameters();
  parameters.max_iterations = 1U;
  parameters.goal_bias = 1.0;
  parameters.steer_distance_m = 2.0;
  parameters.goal_connection_distance_m = 0.1;
  parameters.stop_on_first_solution = true;
  const RrtStarPlanner planner(parameters);

  const PlanResult result = planner.plan(
    {0.0, 0.0}, {1.0, 0.0},
    SamplingBounds{-1.0, 2.0, -1.0, 1.0}, flatNode, flatEdge);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.nodes.size(), 2U);
  ASSERT_EQ(result.edges.size(), 1U);
  EXPECT_EQ(result.edges.front().from, 0U);
  EXPECT_EQ(result.edges.front().to, 1U);
  EXPECT_TRUE(result.edges.front().is_goal_connection);
}

TEST(RrtStarPlanner, StartEqualToGoalReturnsSingleNodePath)
{
  const RrtStarPlanner planner(testParameters());
  const Point2D position{1.0, 1.0};
  const PlanResult result = planner.plan(
    position, position, SamplingBounds{0.0, 2.0, 0.0, 2.0},
    flatNode, flatEdge);

  ASSERT_TRUE(result.success) << result.message;
  ASSERT_EQ(result.path_node_ids.size(), 1U);
  EXPECT_EQ(result.path_node_ids.front(), 0U);
  ASSERT_EQ(result.path.size(), 1U);
  EXPECT_DOUBLE_EQ(result.path.front().x, position.x);
  EXPECT_DOUBLE_EQ(result.path.front().y, position.y);
}

TEST(RrtStarPlanner, TerrainEvaluatorOverloadUsesSnapshotBounds)
{
  std::vector<TerrainPoint> points;
  for (std::size_t iy = 0U; iy < 21U; ++iy) {
    for (std::size_t ix = 0U; ix < 21U; ++ix) {
      points.push_back(
        TerrainPoint{
            -1.0 + 0.1 * static_cast<double>(ix),
            -1.0 + 0.1 * static_cast<double>(iy),
            0.0});
    }
  }
  const TerrainSnapshot snapshot =
    TerrainSnapshot::fromPoints(points, 0.1, 1.0e-6);
  TerrainEvaluatorParameters terrain_parameters;
  terrain_parameters.pca_radius_m = 0.21;
  terrain_parameters.min_pca_points = 5U;
  terrain_parameters.footprint_radius_m = 0.0;
  terrain_parameters.max_slope_deg = 15.0;
  terrain_parameters.max_step_height_m = 0.08;
  terrain_parameters.edge_sample_spacing_m = 0.05;
  const TerrainEvaluator terrain(snapshot, terrain_parameters);

  auto parameters = testParameters();
  parameters.max_iterations = 20U;
  parameters.goal_bias = 1.0;
  parameters.steer_distance_m = 0.30;
  parameters.goal_connection_distance_m = 0.31;
  parameters.stop_on_first_solution = true;
  const PlanResult result =
    RrtStarPlanner(parameters).plan(terrain, {-0.8, 0.0}, {1.04, 0.0});

  ASSERT_TRUE(result.success) << result.message;
  ASSERT_FALSE(result.path.empty());
  EXPECT_NEAR(result.path.front().x, -0.8, 1.0e-12);
  EXPECT_NEAR(result.path.back().x, 1.04, 1.0e-12);
}

TEST(RrtStarPlanner, FiveCentimeterFlatGridUsesSharedTerrainEvaluator)
{
  std::vector<TerrainPoint> points;
  for (std::size_t iy = 0U; iy < 41U; ++iy) {
    for (std::size_t ix = 0U; ix < 61U; ++ix) {
      points.push_back(
        TerrainPoint{
            -1.5 + 0.05 * static_cast<double>(ix),
            -1.0 + 0.05 * static_cast<double>(iy), 0.0});
    }
  }
  const TerrainSnapshot snapshot =
    TerrainSnapshot::fromPoints(points, 0.05, 0.01);
  TerrainEvaluatorParameters terrain_parameters;
  terrain_parameters.pca_radius_m = 0.30;
  terrain_parameters.min_pca_points = 6U;
  terrain_parameters.footprint_radius_m = 0.20;
  terrain_parameters.min_footprint_observed_ratio = 1.00;
  terrain_parameters.max_slope_deg = 15.0;
  terrain_parameters.max_step_height_m = 0.08;
  terrain_parameters.edge_sample_spacing_m = 0.025;
  const TerrainEvaluator terrain(snapshot, terrain_parameters);

  auto parameters = testParameters();
  parameters.max_iterations = 500U;
  parameters.goal_bias = 0.20;
  parameters.steer_distance_m = 0.50;
  parameters.rewire_radius_min_m = 0.30;
  parameters.rewire_radius_max_m = 1.00;
  parameters.goal_connection_distance_m = 0.75;
  parameters.max_planning_time_ms = 0U;
  parameters.random_seed = 42U;
  const PlanResult result =
    RrtStarPlanner(parameters).plan(terrain, {-0.75, 0.0}, {0.75, 0.0});

  ASSERT_TRUE(result.success) << result.message;
  ASSERT_FALSE(result.path.empty());
  EXPECT_NEAR(result.path.front().x, -0.75, 1.0e-12);
  EXPECT_NEAR(result.path.back().x, 0.75, 1.0e-12);
}

TEST(RrtStarPlanner, RejectsInvalidParametersAndRequestBounds)
{
  auto parameters = testParameters();
  parameters.max_iterations = 0U;
  EXPECT_THROW(RrtStarPlanner{parameters}, std::invalid_argument);

  parameters = testParameters();
  parameters.goal_bias = 1.1;
  EXPECT_THROW(RrtStarPlanner{parameters}, std::invalid_argument);

  parameters = testParameters();
  parameters.rewire_radius_min_m = 2.0;
  parameters.rewire_radius_max_m = 1.0;
  EXPECT_THROW(RrtStarPlanner{parameters}, std::invalid_argument);

  const PlanResult result = RrtStarPlanner(testParameters()).plan(
    {0.0, 0.0}, {1.0, 1.0},
    SamplingBounds{0.0, 0.0, 0.0, 1.0}, flatNode, flatEdge);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.termination, WavefrontTermination::kInvalidRequest);
}

}  // namespace
}  // namespace rubi_heightmap_wavefront_planner
