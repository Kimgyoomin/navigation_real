#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "rubi_heightmap_step_wavefront_planner/step_wavefront_planner.hpp"

namespace planner = rubi_heightmap_step_wavefront_planner;

namespace
{
struct Fixture
{
  planner::HeightmapSnapshot heightmap;
  planner::CostmapSnapshot costmap;
};

Fixture flatFixture(std::vector<std::uint8_t> raw = std::vector<std::uint8_t>(81U * 81U, 0U))
{
  std::vector<planner::HeightPoint> points;
  for (int y = -40; y <= 40; ++y) for (int x = -40; x <= 40; ++x) {
    points.push_back({0.05 * x, 0.05 * y, 0.0});
  }
  return {planner::HeightmapSnapshot::fromPoints(points, 0.05, 0.001, 10000U),
    planner::CostmapSnapshot::fromData(81U, 81U, 0.05, -2.025, -2.025, std::move(raw))};
}

planner::StepEvaluatorParameters evaluationParameters()
{
  planner::StepEvaluatorParameters p;
  p.edge_height_query_radius_m = 0.075;
  p.edge_max_height_evidence_gap_m = 0.10;
  return p;
}

planner::StepWavefrontParameters trgParameters()
{
  planner::StepWavefrontParameters p;
  p.sampling_policy = planner::SamplingPolicy::kOriginalTrgRandomRing;
  p.trg_expand_distance_m = 0.30;
  p.trg_robot_size_m = 0.20;
  p.trg_sample_num = 5U;
  p.trg_max_trial_samples = 100U;
  p.trg_height_threshold_m = 0.08;
  p.trg_collision_threshold = 0.10;
  p.trg_random_seed = 42U;
  p.trg_randomize_seed = false;
  p.trg_neighbor_connection_radius_m = 0.30;
  p.goal_connection_distance_m = 0.20;
  p.max_nodes = 100U;
  p.max_expansions = 1U;
  p.post_goal_expansions = 1U;
  return p;
}
}  // namespace

TEST(OriginalTrgConstruction, FixedRadiusSeedIsDeterministicAndFrontierIsQueued)
{
  auto fixture = flatFixture();
  planner::StepEvaluator evaluator(fixture.heightmap, fixture.costmap, evaluationParameters());
  const auto params = trgParameters();
  const auto first = planner::StepWavefrontPlanner(params).plan(evaluator, {0.0, 0.0}, {1.5, 1.5});
  planner::StepEvaluator second_evaluator(
    fixture.heightmap, fixture.costmap, evaluationParameters());
  const auto second = planner::StepWavefrontPlanner(params).plan(
    second_evaluator, {0.0, 0.0}, {1.5, 1.5});
  ASSERT_EQ(first.nodes.size(), second.nodes.size());
  ASSERT_GT(first.nodes.size(), 1U);
  EXPECT_EQ(first.statistics.sampling_trials, 5U);
  for (std::size_t index = 0U; index < first.nodes.size(); ++index) {
    EXPECT_DOUBLE_EQ(first.nodes[index].point.x, second.nodes[index].point.x);
    EXPECT_DOUBLE_EQ(first.nodes[index].point.y, second.nodes[index].point.y);
    if (index > 0U) {
      EXPECT_NEAR(std::hypot(first.nodes[index].point.x, first.nodes[index].point.y), 0.30, 1e-12);
      EXPECT_EQ(first.nodes[index].state, planner::GraphNodeState::kFrontier);
    }
  }
  auto other_params = params;
  other_params.trg_random_seed = 7U;
  planner::StepEvaluator other_evaluator(
    fixture.heightmap, fixture.costmap, evaluationParameters());
  const auto other = planner::StepWavefrontPlanner(other_params).plan(
    other_evaluator, {0.0, 0.0}, {1.5, 1.5});
  ASSERT_GT(other.nodes.size(), 1U);
  EXPECT_TRUE(other.nodes.size() != first.nodes.size() ||
    other.nodes[1].point.x != first.nodes[1].point.x ||
    other.nodes[1].point.y != first.nodes[1].point.y);
}

TEST(OriginalTrgConstruction, ExistingNodeInsideRobotSizeIsRewiredWithoutCreation)
{
  auto fixture = flatFixture();
  planner::StepEvaluator evaluator(fixture.heightmap, fixture.costmap, evaluationParameters());
  auto params = trgParameters();
  params.trg_expand_distance_m = 0.10;
  const auto result = planner::StepWavefrontPlanner(params).plan(
    evaluator, {0.0, 0.0}, {1.5, 1.5});
  EXPECT_EQ(result.statistics.existing_node_queries, params.trg_sample_num);
  EXPECT_EQ(result.statistics.existing_node_rewires, params.trg_sample_num);
  EXPECT_EQ(result.statistics.new_nodes_created, 0U);
}

TEST(OriginalTrgConstruction, FailedWiringMarksIsolatedAndCleanRemovesNode)
{
  auto fixture = flatFixture();
  auto ep = evaluationParameters();
  ep.edge_max_height_evidence_gap_m = 0.01;
  planner::StepEvaluator evaluator(fixture.heightmap, fixture.costmap, ep);
  const auto result = planner::StepWavefrontPlanner(trgParameters()).plan(
    evaluator, {0.0, 0.0}, {1.5, 1.5});
  EXPECT_GT(result.statistics.new_nodes_created, 0U);
  EXPECT_EQ(result.statistics.isolated_nodes, result.statistics.new_nodes_created);
  EXPECT_TRUE(result.nodes.empty());
  EXPECT_TRUE(std::any_of(result.rejected.begin(), result.rejected.end(), [](const auto & item) {
    return item.reason == planner::StepInvalidReason::kIsolatedNode;
  }));
}

TEST(OriginalTrgConstruction, CostmapHardBlockRejectsBeforeGraphCommit)
{
  std::vector<std::uint8_t> raw(81U * 81U, 254U);
  const auto index = [](int x, int y) {return static_cast<std::size_t>(y * 81 + x);};
  raw[index(40, 40)] = 0U;
  raw[index(70, 70)] = 0U;
  auto fixture = flatFixture(std::move(raw));
  planner::StepEvaluator evaluator(fixture.heightmap, fixture.costmap, evaluationParameters());
  auto params = trgParameters();
  params.trg_max_trial_samples = 20U;
  const auto result = planner::StepWavefrontPlanner(params).plan(
    evaluator, {0.0, 0.0}, {1.5, 1.5});
  EXPECT_EQ(result.statistics.sampling_trials, 20U);
  EXPECT_GT(result.statistics.costmap_rejects, 0U);
  EXPECT_EQ(result.statistics.new_nodes_created, 0U);
}

TEST(OriginalTrgConstruction, HybridEdgeKeepsExactStepAndRejectsOverLimit)
{
  std::vector<planner::HeightPoint> points;
  for (int y = -4; y <= 4; ++y) for (int x = -8; x <= 8; ++x) {
    points.push_back({0.05 * x, 0.05 * y, x >= 0 ? 0.08 : 0.0});
  }
  auto heights = planner::HeightmapSnapshot::fromPoints(points, 0.05, 0.001, 1000U);
  auto costs = planner::CostmapSnapshot::fromData(
    17U, 9U, 0.05, -0.425, -0.225, std::vector<std::uint8_t>(153U, 0U));
  planner::StepEvaluator exact(heights, costs, evaluationParameters());
  EXPECT_TRUE(exact.evaluateEdge({-0.20, 0.0}, {0.20, 0.0}).valid);
  points.clear();
  for (int y = -4; y <= 4; ++y) for (int x = -8; x <= 8; ++x) {
    points.push_back({0.05 * x, 0.05 * y, x >= 0 ? 0.080001 : 0.0});
  }
  heights = planner::HeightmapSnapshot::fromPoints(points, 0.05, 0.001, 1000U);
  planner::StepEvaluator over(heights, costs, evaluationParameters());
  EXPECT_EQ(over.evaluateEdge({-0.20, 0.0}, {0.20, 0.0}).reason,
    planner::StepInvalidReason::kStepLimit);
}
