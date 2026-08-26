#include <gtest/gtest.h>
#include "rubi_heightmap_step_wavefront_planner/search/astar_search.hpp"

namespace planner = rubi_heightmap_step_wavefront_planner;

planner::EdgeEvaluation accepted(const double cost)
{
  planner::EdgeEvaluation evaluation;
  evaluation.valid = true;
  evaluation.reason = planner::StepInvalidReason::kNone;
  evaluation.length_xy_m = cost;
  evaluation.cost = cost;
  return evaluation;
}

TEST(AStarSearch, DeterministicTieBreakAndInvalidEdges)
{
  planner::TerrainGraph graph;
  graph.nodes = {{0U, {0.0, 0.0}, 0.0}, {1U, {1.0, 1.0}, 0.0},
    {2U, {1.0, -1.0}, 0.0}, {3U, {2.0, 0.0}, 0.0}};
  graph.edges = {{0U, 1U, accepted(1.0)}, {1U, 3U, accepted(1.0)},
    {0U, 2U, accepted(1.0)}, {2U, 3U, accepted(1.0)}};
  auto invalid = accepted(0.01);
  invalid.valid = false;
  graph.edges.push_back({0U, 3U, invalid});
  for (int repeat = 0; repeat < 20; ++repeat) {
    const auto result = planner::AStarSearch{}.search(graph, 0U, 3U, 1.0);
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.path_node_ids, (std::vector<planner::NodeId>{0U, 1U, 3U}));
    EXPECT_DOUBLE_EQ(result.total_cost, 2.0);
  }
}

TEST(AStarSearch, NonnegativeClearanceRiskSelectsSaferDetour)
{
  planner::TerrainGraph graph;
  graph.nodes = {{0U, {0.0, 0.0}, 0.0}, {1U, {1.0, 0.0}, 0.0},
    {2U, {1.0, 1.0}, 0.0}, {3U, {2.0, 0.0}, 0.0}};
  graph.edges = {{0U, 1U, accepted(1.0)}, {1U, 3U, accepted(1.0)},
    {0U, 2U, accepted(1.2)}, {2U, 3U, accepted(1.2)}};
  EXPECT_EQ(planner::AStarSearch{}.search(graph, 0U, 3U, 1.0).path_node_ids,
    (std::vector<planner::NodeId>{0U, 1U, 3U}));
  graph.edges[0].evaluation.clearance_score_m = 0.2;
  graph.edges[1].evaluation.clearance_score_m = 0.2;
  graph.edges[0].evaluation.cost += 1.0;
  graph.edges[1].evaluation.cost += 1.0;
  EXPECT_EQ(planner::AStarSearch{}.search(graph, 0U, 3U, 1.0).path_node_ids,
    (std::vector<planner::NodeId>{0U, 2U, 3U}));
}
