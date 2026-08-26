#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "rubi_heightmap_step_wavefront_planner/step_wavefront_planner.hpp"

namespace planner = rubi_heightmap_step_wavefront_planner;

planner::EdgeEvaluation edge(double length, double score, double weight)
{
  planner::EdgeEvaluation value;
  value.valid = true;
  value.reason = planner::StepInvalidReason::kNone;
  value.length_xy_m = length;
  value.height_jump_score_m = score;
  value.cost = length + weight * score;
  return value;
}

TEST(StepWavefrontPlanner, HeightWeightSelectsFlatDetourOnSameGraph)
{
  const std::vector<planner::GraphNode> nodes{
    {0U, {0.0, 0.0}, 0.0}, {1U, {0.5, 0.0}, 0.045},
    {2U, {0.5, 0.3}, 0.0}, {3U, {1.0, 0.0}, 0.0}};
  std::vector<planner::GraphEdge> distance_edges{
    {0U, 1U, edge(0.5, 0.02, 0.0)}, {1U, 3U, edge(0.5, 0.02, 0.0)},
    {0U, 2U, edge(0.59, 0.0, 0.0)}, {2U, 3U, edge(0.59, 0.0, 0.0)}};
  EXPECT_EQ(
    planner::StepWavefrontPlanner::shortestPath(nodes, distance_edges, 0U, 3U, 1.0),
    (std::vector<planner::NodeId>{0U, 1U, 3U}));
  for (auto & item : distance_edges) {
    item.evaluation.cost = item.evaluation.length_xy_m +
      5.0 * item.evaluation.height_jump_score_m;
  }
  EXPECT_EQ(
    planner::StepWavefrontPlanner::shortestPath(nodes, distance_edges, 0U, 3U, 1.0),
    (std::vector<planner::NodeId>{0U, 2U, 3U}));
}

TEST(StepWavefrontPlanner, AStarMatchesDijkstraGroundTruthAndTieBreakIsDeterministic)
{
  const std::vector<planner::GraphNode> nodes{
    {0U, {0.0, 0.0}, 0.0}, {1U, {1.0, 1.0}, 0.0},
    {2U, {1.0, -1.0}, 0.0}, {3U, {2.0, 0.0}, 0.0}};
  const std::vector<planner::GraphEdge> edges{
    {0U, 1U, edge(1.0, 0.0, 0.0)}, {1U, 3U, edge(1.0, 0.0, 0.0)},
    {0U, 2U, edge(1.0, 0.0, 0.0)}, {2U, 3U, edge(1.0, 0.0, 0.0)}};
  double cost = 0.0;
  const auto path = planner::StepWavefrontPlanner::shortestPath(
    nodes, edges, 0U, 3U, 1.0, &cost);
  EXPECT_EQ(path, (std::vector<planner::NodeId>{0U, 1U, 3U}));
  EXPECT_DOUBLE_EQ(cost, 2.0);
}

std::vector<planner::HeightPoint> flatGrid()
{
  std::vector<planner::HeightPoint> points;
  for (int y = -20; y <= 20; ++y) {
    for (int x = -30; x <= 30; ++x) {
      points.push_back({0.05 * x, 0.05 * y, 0.0});
    }
  }
  return points;
}

TEST(StepWavefrontPlanner, PostGoalExpansionsContinueDeterministicGraphBuild)
{
  auto points = flatGrid();
  for (auto & point : points) {
    if (std::abs(point.x - 0.15) < 1.0e-9 && std::abs(point.y) < 1.0e-9) {
      point.z = 0.045;
    }
  }
  const auto snapshot = planner::HeightmapSnapshot::fromPoints(points, 0.05, 0.01, 100000U);
  planner::StepEvaluatorParameters evaluator_parameters;
  evaluator_parameters.hard_clearance_radius_m = 0.0;
  const planner::StepEvaluator evaluator(snapshot, evaluator_parameters);
  planner::StepWavefrontParameters immediate;
  immediate.merge_radius_m = 0.05;
  immediate.post_goal_expansions = 0U;
  const auto first = planner::StepWavefrontPlanner(immediate).plan(
    evaluator, {0.0, 0.0}, {0.30, 0.0});
  planner::StepWavefrontParameters continued = immediate;
  continued.post_goal_expansions = 1U;
  const auto later = planner::StepWavefrontPlanner(continued).plan(
    evaluator, {0.0, 0.0}, {0.30, 0.0});
  ASSERT_TRUE(first.success && later.success);
  for (const auto * result : {&first, &later}) {
    EXPECT_TRUE(std::isfinite(result->graph_build_time_ms));
    EXPECT_TRUE(std::isfinite(result->astar_time_ms));
    EXPECT_TRUE(std::isfinite(result->core_total_time_ms));
    EXPECT_GE(result->graph_build_time_ms, 0.0);
    EXPECT_GE(result->astar_time_ms, 0.0);
    EXPECT_GE(result->core_total_time_ms, 0.0);
  }
  EXPECT_EQ(first.expansions, 0U);
  EXPECT_EQ(first.path_node_ids.size(), 2U);
  EXPECT_EQ(first.path_metrics.height_event_count, 2U);
  EXPECT_NEAR(first.path_metrics.height_score_m, 0.04, 1.0e-12);
  EXPECT_GE(later.expansions, 1U);
  EXPECT_GT(later.nodes.size(), first.nodes.size());
  EXPECT_GT(later.path_node_ids.size(), 2U);
  EXPECT_EQ(later.path_metrics.height_event_count, 0U);
  EXPECT_DOUBLE_EQ(later.path_metrics.height_score_m, 0.0);
  EXPECT_LT(later.path_metrics.total_cost, first.path_metrics.total_cost);
}

TEST(StepWavefrontPlanner, OverLimitEdgesNeverEnterAcceptedGraph)
{
  auto points = flatGrid();
  for (auto & point : points) {
    if (point.x >= 0.0) {
      point.z = 0.081;
    }
  }
  const auto snapshot = planner::HeightmapSnapshot::fromPoints(points, 0.05, 0.01, 100000U);
  planner::StepEvaluatorParameters evaluator_parameters;
  evaluator_parameters.hard_clearance_radius_m = 0.0;
  const planner::StepEvaluator evaluator(snapshot, evaluator_parameters);
  planner::StepWavefrontParameters parameters;
  parameters.post_goal_expansions = 2U;
  const auto result = planner::StepWavefrontPlanner(parameters).plan(
    evaluator, {-0.30, 0.0}, {0.30, 0.0});
  for (const auto & accepted : result.edges) {
    EXPECT_TRUE(accepted.evaluation.valid);
    EXPECT_NE(accepted.evaluation.reason, planner::StepInvalidReason::kStepLimit);
  }
}

TEST(StepWavefrontPlanner, FullGraphAndPathAreDeterministicAcrossTwentyRuns)
{
  const auto snapshot = planner::HeightmapSnapshot::fromPoints(
    flatGrid(), 0.05, 0.01, 100000U);
  planner::StepEvaluatorParameters evaluator_parameters;
  evaluator_parameters.hard_clearance_radius_m = 0.0;
  evaluator_parameters.preferred_clearance_radius_m = 0.0;
  const planner::StepEvaluator evaluator(snapshot, evaluator_parameters);
  planner::StepWavefrontParameters parameters;
  parameters.post_goal_expansions = 3U;
  const auto reference = planner::StepWavefrontPlanner(parameters).plan(
    evaluator, {-0.60, 0.0}, {0.60, 0.0});
  ASSERT_TRUE(reference.success);
  for (int repeat = 1; repeat < 20; ++repeat) {
    const auto candidate = planner::StepWavefrontPlanner(parameters).plan(
      evaluator, {-0.60, 0.0}, {0.60, 0.0});
    ASSERT_EQ(candidate.nodes.size(), reference.nodes.size());
    ASSERT_EQ(candidate.edges.size(), reference.edges.size());
    EXPECT_EQ(candidate.path_node_ids, reference.path_node_ids);
    EXPECT_DOUBLE_EQ(candidate.path_metrics.total_cost, reference.path_metrics.total_cost);
    for (std::size_t index = 0U; index < reference.nodes.size(); ++index) {
      EXPECT_EQ(candidate.nodes[index].id, reference.nodes[index].id);
      EXPECT_DOUBLE_EQ(candidate.nodes[index].point.x, reference.nodes[index].point.x);
      EXPECT_DOUBLE_EQ(candidate.nodes[index].point.y, reference.nodes[index].point.y);
    }
    for (std::size_t index = 0U; index < reference.edges.size(); ++index) {
      EXPECT_EQ(candidate.edges[index].from, reference.edges[index].from);
      EXPECT_EQ(candidate.edges[index].to, reference.edges[index].to);
    }
  }
}
