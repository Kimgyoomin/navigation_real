#include <gtest/gtest.h>

#include "rubi_heightmap_step_wavefront_planner/planner_visualization.hpp"

namespace planner = rubi_heightmap_step_wavefront_planner;

TEST(PlannerVisualization, UsesContractedBatchesColorsAndCap)
{
  planner::PlanResult result;
  result.nodes = {{0U, {0.0, 0.0}, 0.0}, {1U, {1.0, 0.0}, 0.0}};
  planner::EdgeEvaluation accepted;
  accepted.valid = true;
  result.edges = {{0U, 1U, accepted}};
  result.rejected = {
    {planner::RejectionKind::kEdge, planner::StepInvalidReason::kUnknown, {0.0, 0.0}, {0.1, 0.0}},
    {planner::RejectionKind::kEdge, planner::StepInvalidReason::kClearanceViolation, {0.0, 0.0},
      {0.2, 0.0}},
    {planner::RejectionKind::kEdge, planner::StepInvalidReason::kStepLimit, {0.0, 0.0},
      {0.3, 0.0}}};
  planner::VisualizationParameters parameters;
  parameters.max_rejected_markers = 2U;
  const builtin_interfaces::msg::Time stamp;
  const auto output = planner::makeVisualization(
    result, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}, "map", stamp, parameters);
  ASSERT_EQ(output.nodes.markers.size(), 1U);
  EXPECT_FLOAT_EQ(output.nodes.markers[0].color.g, 1.0F);
  ASSERT_EQ(output.edges.markers.size(), 2U);
  EXPECT_FLOAT_EQ(output.edges.markers[1].color.r, 1.0F);
  EXPECT_FLOAT_EQ(output.edges.markers[1].color.g, 1.0F);
  EXPECT_EQ(output.rejected_total, 3U);
  EXPECT_EQ(output.rejected_shown, 2U);
}

TEST(PlannerVisualization, FullResetIsSingleDeleteAllWithFrameAndStamp)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 42;
  const auto clear = planner::makeDeleteAllMarkers("map", stamp);
  ASSERT_EQ(clear.markers.size(), 1U);
  EXPECT_EQ(clear.markers[0].action, visualization_msgs::msg::Marker::DELETEALL);
  EXPECT_EQ(clear.markers[0].header.frame_id, "map");
  EXPECT_EQ(clear.markers[0].header.stamp.sec, 42);
}
