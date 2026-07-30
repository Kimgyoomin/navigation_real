#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rubi_heightmap_wavefront_planner/wavefront_planner.hpp"

namespace rubi_heightmap_wavefront_planner
{
namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;

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
  evaluation.surface.normal_x = 0.0;
  evaluation.surface.normal_y = 0.0;
  evaluation.surface.normal_z = 1.0;
  return evaluation;
}

EdgeEvaluation flatEdge(const Point2D & from, const Point2D & to)
{
  EdgeEvaluation evaluation;
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double length = std::hypot(dx, dy);
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

WavefrontPlannerParameters smallParameters()
{
  WavefrontPlannerParameters parameters;
  parameters.node_sampling_distance_m = 1.0;
  parameters.num_expansion_samples = 4U;
  parameters.merge_radius_m = 0.10;
  parameters.neighbor_connection_radius_m = 1.50;
  parameters.max_nodes = 100U;
  parameters.max_expansions = 100U;
  parameters.max_build_time_ms = 10000U;
  parameters.goal_connection_distance_m = 0.25;
  parameters.stop_when_goal_connected = true;
  parameters.slope_normalization_deg = 10.0;
  parameters.risk_weights.distance = 1.0;
  parameters.risk_weights.slope = 1.0;
  parameters.risk_weights.step = 0.0;
  return parameters;
}

TEST(WavefrontPlanner, DirectGoalConnectionStopsBeforeExpansion)
{
  auto parameters = smallParameters();
  parameters.goal_connection_distance_m = 1.1;
  const WavefrontPlanner planner(parameters);

  const PlanResult result = planner.plan(
    Point2D{0.0, 0.0}, Point2D{1.0, 0.0}, flatNode, flatEdge);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.termination, WavefrontTermination::kGoalConnected);
  EXPECT_TRUE(result.stopped_on_goal_connection);
  EXPECT_EQ(result.expansions, 0U);
  EXPECT_EQ(result.nodes.size(), 2U);
  ASSERT_EQ(result.edges.size(), 1U);
  EXPECT_TRUE(result.edges.front().is_goal_connection);
  ASSERT_EQ(result.path_node_ids.size(), 2U);
  EXPECT_EQ(result.path_node_ids[0], 0U);
  EXPECT_EQ(result.path_node_ids[1], 1U);
}

TEST(WavefrontPlanner, ExpandsFifoWithDeterministicUniformRing)
{
  auto parameters = smallParameters();
  parameters.max_nodes = 6U;
  parameters.stop_when_goal_connected = false;
  const WavefrontPlanner planner(parameters);

  const auto run = [&]() {
      return planner.plan(
        Point2D{0.0, 0.0}, Point2D{10.0, 10.0}, flatNode, flatEdge);
    };
  const PlanResult first = run();
  const PlanResult second = run();

  EXPECT_FALSE(first.success);
  EXPECT_EQ(first.termination, WavefrontTermination::kMaxNodesReached);
  EXPECT_TRUE(first.node_budget_reached);
  EXPECT_EQ(first.expansions, 1U);
  ASSERT_EQ(first.nodes.size(), 6U);

  // IDs 0/1 are reserved for start/goal. Depth-zero expansion is ordered
  // counter-clockwise from +X.
  EXPECT_NEAR(first.nodes[2].point.x, 1.0, 1.0e-12);
  EXPECT_NEAR(first.nodes[2].point.y, 0.0, 1.0e-12);
  EXPECT_NEAR(first.nodes[3].point.x, 0.0, 1.0e-12);
  EXPECT_NEAR(first.nodes[3].point.y, 1.0, 1.0e-12);
  EXPECT_NEAR(first.nodes[4].point.x, -1.0, 1.0e-12);
  EXPECT_NEAR(first.nodes[4].point.y, 0.0, 1.0e-12);
  EXPECT_NEAR(first.nodes[5].point.x, 0.0, 1.0e-12);
  EXPECT_NEAR(first.nodes[5].point.y, -1.0, 1.0e-12);

  EXPECT_EQ(first.nodes.size(), second.nodes.size());
  EXPECT_EQ(first.edges.size(), second.edges.size());
  for (std::size_t index = 0; index < first.nodes.size(); ++index) {
    EXPECT_DOUBLE_EQ(first.nodes[index].point.x, second.nodes[index].point.x);
    EXPECT_DOUBLE_EQ(first.nodes[index].point.y, second.nodes[index].point.y);
  }
  for (std::size_t index = 0; index < first.edges.size(); ++index) {
    EXPECT_EQ(first.edges[index].from, second.edges[index].from);
    EXPECT_EQ(first.edges[index].to, second.edges[index].to);
    EXPECT_DOUBLE_EQ(first.edges[index].cost, second.edges[index].cost);
  }
  EXPECT_TRUE(
    std::any_of(
      first.edges.begin(), first.edges.end(),
      [](const GraphEdge & edge) {return edge.is_loop_closure;}));
}

TEST(WavefrontPlanner, EvaluatesTwentyProposalsOnThirtyCentimeterRing)
{
  auto parameters = smallParameters();
  parameters.node_sampling_distance_m = 0.30;
  parameters.num_expansion_samples = 20U;
  parameters.merge_radius_m = 0.20;
  parameters.neighbor_connection_radius_m = 0.45;
  parameters.goal_connection_distance_m = 0.45;
  parameters.max_nodes = 100U;
  parameters.max_expansions = 1U;
  parameters.stop_when_goal_connected = false;
  const WavefrontPlanner planner(parameters);

  std::vector<Point2D> evaluated_positions;
  const auto recording_flat_node = [&evaluated_positions](const Point2D & point) {
      evaluated_positions.push_back(point);
      return flatNode(point);
    };

  const Point2D start{0.0, 0.0};
  const Point2D far_away_goal{100.0, 100.0};
  const PlanResult result = planner.plan(
    start, far_away_goal, recording_flat_node, flatEdge);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.termination, WavefrontTermination::kMaxExpansionsReached);
  EXPECT_EQ(result.expansions, 1U);

  // Start and goal are evaluated first. The next 20 callback invocations are
  // the depth-zero proposals. Do not assert that all proposals survive merge.
  ASSERT_EQ(evaluated_positions.size(), 22U);
  EXPECT_DOUBLE_EQ(evaluated_positions[0].x, start.x);
  EXPECT_DOUBLE_EQ(evaluated_positions[0].y, start.y);
  EXPECT_DOUBLE_EQ(evaluated_positions[1].x, far_away_goal.x);
  EXPECT_DOUBLE_EQ(evaluated_positions[1].y, far_away_goal.y);
  for (std::size_t sample = 0U; sample < 20U; ++sample) {
    const double angle =
      2.0 * kPi * static_cast<double>(sample) / 20.0;
    const Point2D & proposal = evaluated_positions[sample + 2U];
    EXPECT_NEAR(proposal.x, 0.30 * std::cos(angle), 1.0e-12);
    EXPECT_NEAR(proposal.y, 0.30 * std::sin(angle), 1.0e-12);
    EXPECT_NEAR(
      std::hypot(proposal.x - start.x, proposal.y - start.y),
      0.30, 1.0e-12);
  }
}

TEST(WavefrontPlanner, MergeRadiusSuppressesDuplicateAndAddsLoopEdge)
{
  auto parameters = smallParameters();
  parameters.merge_radius_m = 0.80;
  parameters.neighbor_connection_radius_m = 0.05;
  parameters.max_nodes = 20U;
  parameters.max_expansions = 2U;
  parameters.stop_when_goal_connected = false;
  const WavefrontPlanner planner(parameters);

  const PlanResult result = planner.plan(
    Point2D{0.0, 0.0}, Point2D{10.0, 10.0}, flatNode, flatEdge);

  EXPECT_EQ(result.termination, WavefrontTermination::kMaxExpansionsReached);
  EXPECT_EQ(result.expansions, 2U);
  EXPECT_TRUE(
    std::any_of(
      result.edges.begin(), result.edges.end(),
      [](const GraphEdge & edge) {return edge.is_loop_closure;}));
}

TEST(WavefrontPlanner, TerrainInvalidEdgesCannotEnterGraph)
{
  auto parameters = smallParameters();
  parameters.max_expansions = 8U;
  parameters.max_nodes = 100U;
  parameters.goal_connection_distance_m = 0.6;
  parameters.stop_when_goal_connected = false;
  const WavefrontPlanner planner(parameters);

  const auto barrier_edge = [](const Point2D & from, const Point2D & to) {
      EdgeEvaluation evaluation = flatEdge(from, to);
      const bool crosses_positive =
        from.x <= 0.5 && to.x > 0.5;
      const bool crosses_negative =
        to.x <= 0.5 && from.x > 0.5;
      if (crosses_positive || crosses_negative) {
        evaluation.valid = false;
        evaluation.reason = TerrainInvalidReason::kStepLimit;
      }
      return evaluation;
    };

  const PlanResult result = planner.plan(
    Point2D{0.0, 0.0}, Point2D{2.0, 0.0}, flatNode, barrier_edge);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.termination, WavefrontTermination::kMaxExpansionsReached);
  EXPECT_GT(
    result.reject_counts.expansion_edge_invalid +
    result.reject_counts.merge_edge_invalid +
    result.reject_counts.goal_edge_invalid,
    0U);
  for (const auto & edge : result.edges) {
    const double from_x = result.nodes[edge.from].point.x;
    const double to_x = result.nodes[edge.to].point.x;
    EXPECT_FALSE(
      (from_x <= 0.5 && to_x > 0.5) ||
      (to_x <= 0.5 && from_x > 0.5));
  }
}

TEST(WavefrontPlanner, FailedPlanPreservesPartialAcceptedGraphAndRejections)
{
  auto parameters = smallParameters();
  parameters.max_expansions = 1U;
  parameters.max_nodes = 100U;
  parameters.goal_connection_distance_m = 0.25;
  parameters.stop_when_goal_connected = false;
  const WavefrontPlanner planner(parameters);

  const auto barrier_edge = [](const Point2D & from, const Point2D & to) {
      EdgeEvaluation evaluation = flatEdge(from, to);
      const bool crosses_positive = from.x <= 0.5 && to.x > 0.5;
      const bool crosses_negative = to.x <= 0.5 && from.x > 0.5;
      if (crosses_positive || crosses_negative) {
        evaluation.valid = false;
        evaluation.reason = TerrainInvalidReason::kStepLimit;
      }
      return evaluation;
    };

  const PlanResult result = planner.plan(
    Point2D{0.0, 0.0}, Point2D{2.0, 0.0}, flatNode, barrier_edge);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.termination, WavefrontTermination::kMaxExpansionsReached);
  EXPECT_GT(result.nodes.size(), 2U);
  EXPECT_FALSE(result.edges.empty());
  EXPECT_FALSE(result.rejected.empty());
  EXPECT_GT(result.reject_counts.expansion_edge_invalid, 0U);
  for (const auto & edge : result.edges) {
    ASSERT_LT(edge.from, result.nodes.size());
    ASSERT_LT(edge.to, result.nodes.size());
    EXPECT_TRUE(edge.terrain.valid);
    const double from_x = result.nodes[edge.from].point.x;
    const double to_x = result.nodes[edge.to].point.x;
    EXPECT_FALSE(
      (from_x <= 0.5 && to_x > 0.5) ||
      (to_x <= 0.5 && from_x > 0.5));
  }
}

TEST(WavefrontPlanner, InvalidStartOrGoalReturnsDiagnosticWithoutAcceptedGraph)
{
  const WavefrontPlanner planner(smallParameters());
  const Point2D start{0.0, 0.0};
  const Point2D goal{2.0, 0.0};
  const std::vector<Point2D> invalid_positions{start, goal};

  for (const Point2D & invalid_position : invalid_positions) {
    SCOPED_TRACE(
      "invalid_position=(" + std::to_string(invalid_position.x) + ", " +
      std::to_string(invalid_position.y) + ")");
    const auto invalidating_node = [invalid_position](const Point2D & point) {
        NodeEvaluation evaluation = flatNode(point);
        if (
          point.x == invalid_position.x &&
          point.y == invalid_position.y)
        {
          evaluation.valid = false;
          evaluation.reason = TerrainInvalidReason::kUnknown;
        }
        return evaluation;
      };

    const PlanResult result = planner.plan(
      start, goal, invalidating_node, flatEdge);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.termination, WavefrontTermination::kInvalidRequest);
    EXPECT_TRUE(result.nodes.empty());
    EXPECT_TRUE(result.edges.empty());
    EXPECT_TRUE(result.path_node_ids.empty());
    EXPECT_TRUE(result.path.empty());
    ASSERT_EQ(result.rejected.size(), 1U);
    EXPECT_EQ(result.rejected.front().kind, RejectedSampleKind::kNodeInvalid);
    EXPECT_EQ(result.rejected.front().terrain_reason, TerrainInvalidReason::kUnknown);
    EXPECT_DOUBLE_EQ(result.rejected.front().candidate.x, invalid_position.x);
    EXPECT_DOUBLE_EQ(result.rejected.front().candidate.y, invalid_position.y);
    EXPECT_EQ(result.reject_counts.node_invalid, 1U);
  }
}

TEST(WavefrontPlanner, AStarAvoidsShortButHighSlopeEdge)
{
  auto parameters = smallParameters();
  parameters.max_nodes = 6U;
  parameters.goal_connection_distance_m = 2.1;
  parameters.stop_when_goal_connected = false;
  parameters.risk_weights.slope = 10.0;
  const WavefrontPlanner planner(parameters);

  const auto risk_edge = [](const Point2D & from, const Point2D & to) {
      EdgeEvaluation evaluation = flatEdge(from, to);
      if (evaluation.length_xy_m > 1.5) {
        evaluation.max_slope_deg = 10.0;
      }
      return evaluation;
    };

  const PlanResult result = planner.plan(
    Point2D{0.0, 0.0}, Point2D{2.0, 0.0}, flatNode, risk_edge);

  ASSERT_TRUE(result.success) << result.message;
  ASSERT_EQ(result.path_node_ids.size(), 3U);
  EXPECT_EQ(result.path_node_ids[0], 0U);
  EXPECT_EQ(result.path_node_ids[1], 2U);
  EXPECT_EQ(result.path_node_ids[2], 1U);
}

TEST(WavefrontPlanner, EnforcesGraphBuildTimeBudget)
{
  auto parameters = smallParameters();
  parameters.max_build_time_ms = 1U;
  parameters.stop_when_goal_connected = false;
  const WavefrontPlanner planner(parameters);

  const auto slow_flat_node = [](const Point2D & point) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      return flatNode(point);
    };

  const PlanResult result = planner.plan(
    Point2D{0.0, 0.0}, Point2D{10.0, 10.0}, slow_flat_node, flatEdge);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.termination, WavefrontTermination::kMaxBuildTimeReached);
  EXPECT_TRUE(result.build_time_budget_reached);
  EXPECT_EQ(result.expansions, 0U);
  EXPECT_GE(result.build_time_ms, 1.0);
}

TEST(WavefrontPlanner, RejectsInvalidConfiguration)
{
  auto parameters = smallParameters();
  parameters.num_expansion_samples = 2U;
  EXPECT_THROW(WavefrontPlanner{parameters}, std::invalid_argument);
}

}  // namespace
}  // namespace rubi_heightmap_wavefront_planner
