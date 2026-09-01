#include <vector>
#include <gtest/gtest.h>
#include "rubi_heightmap_step_wavefront_planner/step_wavefront_planner.hpp"
#include "rubi_heightmap_step_wavefront_planner/map/costmap_snapshot.hpp"
namespace planner = rubi_heightmap_step_wavefront_planner;

std::vector<planner::HeightPoint> samplingHeights()
{
  std::vector<planner::HeightPoint> points;
  for (int y = -15; y <= 15; ++y) for (int x = -20; x <= 20; ++x)
    points.push_back({0.05 * x, 0.05 * y, 0.0});
  return points;
}

planner::StepWavefrontParameters randomParameters(std::uint32_t seed)
{
  planner::StepWavefrontParameters p;
  p.sampling_policy = planner::SamplingPolicy::kTrgRandomRing;
  p.random_seed = seed;
  p.max_sampling_trials_per_expansion = 1000U;
  p.post_goal_expansions = 3U;
  p.max_nodes = 300U;
  p.max_expansions = 300U;
  return p;
}

TEST(TrgSamplingPolicy, Seed42IsDeterministicAcrossTwentyRuns)
{
  auto snapshot = planner::HeightmapSnapshot::fromPoints(samplingHeights(), 0.05, 0.001, 10000U);
  planner::StepEvaluatorParameters ep;
  ep.hard_clearance_radius_m = 0.0;
  ep.preferred_clearance_radius_m = 0.0;
  planner::StepEvaluator evaluator(snapshot, ep);
  const auto reference = planner::StepWavefrontPlanner(randomParameters(42U)).plan(
    evaluator, {-0.6, 0.0}, {0.6, 0.0});
  ASSERT_TRUE(reference.success);
  for (int repeat = 1; repeat < 20; ++repeat) {
    const auto result = planner::StepWavefrontPlanner(randomParameters(42U)).plan(
      evaluator, {-0.6, 0.0}, {0.6, 0.0});
    EXPECT_EQ(result.path_node_ids, reference.path_node_ids);
    EXPECT_EQ(result.nodes.size(), reference.nodes.size());
    EXPECT_EQ(result.edges.size(), reference.edges.size());
    EXPECT_DOUBLE_EQ(result.path_metrics.total_cost, reference.path_metrics.total_cost);
  }
  const auto other = planner::StepWavefrontPlanner(randomParameters(7U)).plan(
    evaluator, {-0.6, 0.0}, {0.6, 0.0});
  ASSERT_TRUE(other.success);
  EXPECT_TRUE(other.nodes.size() != reference.nodes.size() ||
    other.path_node_ids != reference.path_node_ids);
}

TEST(TrgSamplingPolicy, HybridAcceptedGraphContainsOnlyValidNodesAndEdges)
{
  auto heights = planner::HeightmapSnapshot::fromPoints(samplingHeights(), 0.05, 0.001, 10000U);
  auto costs = planner::CostmapSnapshot::fromData(
    41U, 31U, 0.05, -1.025, -0.775, std::vector<std::uint8_t>(41U * 31U, 0U));
  planner::StepEvaluatorParameters ep;
  ep.node_evidence_radius_m = 0.08;
  ep.node_min_observed_cells = 3U;
  ep.node_max_nearest_evidence_distance_m = 0.075;
  ep.edge_height_query_radius_m = 0.075;
  ep.edge_max_height_evidence_gap_m = 0.10;
  planner::StepEvaluator evaluator(heights, costs, ep);
  auto result = planner::StepWavefrontPlanner(randomParameters(42U)).plan(
    evaluator, {-0.6, 0.0}, {0.6, 0.0});
  ASSERT_TRUE(result.success);
  for (const auto & node : result.nodes) EXPECT_TRUE(evaluator.evaluateNode(node.point).valid);
  for (const auto & edge : result.edges) EXPECT_TRUE(edge.evaluation.valid);
}
