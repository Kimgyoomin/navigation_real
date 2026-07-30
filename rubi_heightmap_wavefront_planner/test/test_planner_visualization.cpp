#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rubi_heightmap_wavefront_planner/planner_visualization.hpp"

namespace rubi_heightmap_wavefront_planner
{
namespace
{

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

TerrainSnapshot makeObservedTerrain()
{
  constexpr std::size_t kWidth = 8U;
  constexpr std::size_t kHeight = 8U;
  return TerrainSnapshot(
    1.0, 0.0, 0.0, kWidth, kHeight,
    std::vector<double>(kWidth * kHeight, 2.0),
    std::vector<std::uint8_t>(kWidth * kHeight, 1U));
}

PlannerVisualizationParameters makeParameters(
  const std::size_t rejection_cap = 100U)
{
  PlannerVisualizationParameters parameters;
  parameters.marker_namespace = "visualization_test";
  parameters.node_marker_scale_m = 0.10;
  parameters.edge_marker_width_m = 0.02;
  parameters.path_marker_width_m = 0.07;
  parameters.rejected_marker_scale_m = 0.09;
  parameters.max_rejected_markers = rejection_cap;
  return parameters;
}

builtin_interfaces::msg::Time makeStamp()
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 123;
  stamp.nanosec = 456U;
  return stamp;
}

GraphNode makeNode(
  const NodeId id,
  const TerrainPoint & point,
  const GraphNodeRole role)
{
  GraphNode node;
  node.id = id;
  node.point = point;
  node.role = role;
  return node;
}

GraphEdge makeEdge(
  const NodeId from,
  const NodeId to,
  const bool terrain_valid)
{
  GraphEdge edge;
  edge.from = from;
  edge.to = to;
  edge.terrain.valid = terrain_valid;
  return edge;
}

RejectedSample makeRejection(
  const NodeId source,
  const Point2D & candidate,
  const RejectedSampleKind kind)
{
  RejectedSample rejected;
  rejected.source = source;
  rejected.candidate = candidate;
  rejected.kind = kind;
  rejected.terrain_reason = TerrainInvalidReason::kUnknown;
  return rejected;
}

PlannerVisualizationSnapshot render(
  const PlanResult & result,
  const std::vector<TerrainPoint> & dense_path = {},
  const std::size_t rejection_cap = 100U)
{
  return makePlannerVisualization(
    result, makeObservedTerrain(), dense_path, "map", makeStamp(),
    makeParameters(rejection_cap));
}

std::vector<const Marker *> addedMarkersOfType(
  const MarkerArray & array,
  const std::int32_t type)
{
  std::vector<const Marker *> matches;
  for (const auto & marker : array.markers) {
    if (marker.action == Marker::ADD && marker.type == type) {
      matches.push_back(&marker);
    }
  }
  return matches;
}

bool hasDeleteAll(const MarkerArray & array)
{
  for (const auto & marker : array.markers) {
    if (marker.action == Marker::DELETEALL) {
      return true;
    }
  }
  return false;
}

void expectVisibleRgb(
  const std_msgs::msg::ColorRGBA & color,
  const float red,
  const float green,
  const float blue)
{
  EXPECT_FLOAT_EQ(color.r, red);
  EXPECT_FLOAT_EQ(color.g, green);
  EXPECT_FLOAT_EQ(color.b, blue);
  EXPECT_GT(color.a, 0.0F);
}

void expectAllPointsHaveColor(
  const Marker & marker,
  const float red,
  const float green,
  const float blue)
{
  if (marker.colors.empty()) {
    expectVisibleRgb(marker.color, red, green, blue);
    return;
  }

  ASSERT_EQ(marker.colors.size(), marker.points.size());
  for (const auto & color : marker.colors) {
    expectVisibleRgb(color, red, green, blue);
  }
}

bool containsPointAt(
  const MarkerArray & array,
  const Point2D & expected)
{
  for (const auto & marker : array.markers) {
    if (marker.action != Marker::ADD) {
      continue;
    }
    for (const auto & point : marker.points) {
      if (point.x == expected.x && point.y == expected.y) {
        return true;
      }
    }
  }
  return false;
}

TEST(PlannerVisualization, AcceptedNodesUseRoleColorsAndSampledGreen)
{
  PlanResult result;
  result.nodes = {
    makeNode(0U, TerrainPoint{0.0, 0.0, 0.5}, GraphNodeRole::kStart),
    makeNode(1U, TerrainPoint{2.0, 0.0, 0.7}, GraphNodeRole::kGoal),
    makeNode(2U, TerrainPoint{1.0, 1.0, 0.6}, GraphNodeRole::kSampled),
  };

  const PlannerVisualizationSnapshot snapshot = render(result);
  const auto node_markers =
    addedMarkersOfType(snapshot.nodes, Marker::SPHERE_LIST);
  ASSERT_EQ(node_markers.size(), 1U);
  const Marker & nodes = *node_markers.front();

  ASSERT_EQ(nodes.points.size(), 3U);
  ASSERT_EQ(nodes.colors.size(), nodes.points.size());
  EXPECT_DOUBLE_EQ(nodes.points[0].x, 0.0);
  EXPECT_DOUBLE_EQ(nodes.points[1].x, 2.0);
  EXPECT_DOUBLE_EQ(nodes.points[2].x, 1.0);

  // Start and goal remain semantic endpoints; accepted samples are green.
  expectVisibleRgb(nodes.colors[0], 0.0F, 1.0F, 1.0F);
  expectVisibleRgb(nodes.colors[1], 1.0F, 0.0F, 1.0F);
  expectVisibleRgb(nodes.colors[2], 0.10F, 0.90F, 0.20F);
}

TEST(PlannerVisualization, ValidEdgesAreGreenOrderedSourceToTargetSegments)
{
  PlanResult result;
  result.nodes = {
    makeNode(0U, TerrainPoint{0.0, 0.0, 0.5}, GraphNodeRole::kStart),
    makeNode(1U, TerrainPoint{2.0, 0.0, 0.7}, GraphNodeRole::kGoal),
    makeNode(2U, TerrainPoint{1.0, 1.0, 0.6}, GraphNodeRole::kSampled),
  };
  result.edges = {
    makeEdge(0U, 2U, true),
    makeEdge(2U, 1U, false),
    makeEdge(0U, 99U, true),
  };

  const PlannerVisualizationSnapshot snapshot = render(result);
  const auto edge_markers =
    addedMarkersOfType(snapshot.edges, Marker::LINE_LIST);
  ASSERT_EQ(edge_markers.size(), 1U);
  const Marker & edges = *edge_markers.front();

  // Neither a terrain-invalid edge nor a malformed graph endpoint is drawn.
  ASSERT_EQ(edges.points.size(), 2U);
  EXPECT_DOUBLE_EQ(edges.points[0].x, result.nodes[0].point.x);
  EXPECT_DOUBLE_EQ(edges.points[0].y, result.nodes[0].point.y);
  EXPECT_DOUBLE_EQ(edges.points[1].x, result.nodes[2].point.x);
  EXPECT_DOUBLE_EQ(edges.points[1].y, result.nodes[2].point.y);
  expectAllPointsHaveColor(edges, 0.10F, 0.90F, 0.20F);
}

TEST(PlannerVisualization, DenseFinalPathIsOneYellowLineStrip)
{
  const std::vector<TerrainPoint> dense_path{
    {0.0, 0.0, 0.5},
    {1.0, 0.0, 0.6},
    {2.0, 1.0, 0.8},
  };

  const PlannerVisualizationSnapshot snapshot =
    render(PlanResult{}, dense_path);
  const auto path_markers =
    addedMarkersOfType(snapshot.edges, Marker::LINE_STRIP);
  ASSERT_EQ(path_markers.size(), 1U);
  const Marker & path = *path_markers.front();

  ASSERT_EQ(path.points.size(), dense_path.size());
  for (std::size_t index = 0U; index < dense_path.size(); ++index) {
    EXPECT_DOUBLE_EQ(path.points[index].x, dense_path[index].x);
    EXPECT_DOUBLE_EQ(path.points[index].y, dense_path[index].y);
    EXPECT_DOUBLE_EQ(path.points[index].z, dense_path[index].z + 0.04);
  }
  expectAllPointsHaveColor(path, 1.0F, 1.0F, 0.0F);
}

TEST(PlannerVisualization, RejectionKindsUseOnlyTheirContractedRedGeometry)
{
  PlanResult result;
  result.nodes = {
    makeNode(0U, TerrainPoint{0.0, 0.0, 0.5}, GraphNodeRole::kStart),
  };
  const std::vector<Point2D> invalid_edge_candidates{
    {1.0, 1.0},
    {2.0, 1.0},
    {3.0, 1.0},
  };
  const Point2D node_invalid_candidate{1.0, 0.0};
  const Point2D non_finite_diagnostic{4.0, 1.0};
  const Point2D duplicate_candidate{5.0, 1.0};
  result.rejected = {
    makeRejection(
      0U, node_invalid_candidate, RejectedSampleKind::kNodeInvalid),
    makeRejection(
      0U, invalid_edge_candidates[0],
      RejectedSampleKind::kExpansionEdgeInvalid),
    makeRejection(
      0U, invalid_edge_candidates[1],
      RejectedSampleKind::kMergeEdgeInvalid),
    makeRejection(
      0U, invalid_edge_candidates[2],
      RejectedSampleKind::kGoalEdgeInvalid),
    // The diagnostic kind can retain a finite last-known candidate position.
    makeRejection(
      0U, non_finite_diagnostic,
      RejectedSampleKind::kNonFiniteEvaluation),
    makeRejection(
      0U, duplicate_candidate, RejectedSampleKind::kDuplicateEdge),
  };

  const PlannerVisualizationSnapshot snapshot = render(result);
  EXPECT_EQ(snapshot.rejected_total, result.rejected.size());
  EXPECT_EQ(snapshot.rejected_shown, 5U);
  EXPECT_FALSE(snapshot.rejected_truncated);

  const auto rejected_nodes =
    addedMarkersOfType(snapshot.rejected, Marker::SPHERE_LIST);
  ASSERT_EQ(rejected_nodes.size(), 1U);
  ASSERT_EQ(rejected_nodes.front()->points.size(), 1U);
  EXPECT_DOUBLE_EQ(
    rejected_nodes.front()->points.front().x, node_invalid_candidate.x);
  EXPECT_DOUBLE_EQ(
    rejected_nodes.front()->points.front().y, node_invalid_candidate.y);
  expectAllPointsHaveColor(*rejected_nodes.front(), 1.0F, 0.05F, 0.05F);

  const auto rejected_edges =
    addedMarkersOfType(snapshot.rejected, Marker::LINE_LIST);
  ASSERT_EQ(rejected_edges.size(), 1U);
  const Marker & edges = *rejected_edges.front();
  ASSERT_EQ(edges.points.size(), 2U * invalid_edge_candidates.size());
  for (std::size_t index = 0U;
    index < invalid_edge_candidates.size(); ++index)
  {
    const auto & source = edges.points[2U * index];
    const auto & candidate = edges.points[2U * index + 1U];
    EXPECT_DOUBLE_EQ(source.x, result.nodes[0].point.x);
    EXPECT_DOUBLE_EQ(source.y, result.nodes[0].point.y);
    EXPECT_DOUBLE_EQ(candidate.x, invalid_edge_candidates[index].x);
    EXPECT_DOUBLE_EQ(candidate.y, invalid_edge_candidates[index].y);
  }
  expectAllPointsHaveColor(edges, 1.0F, 0.05F, 0.05F);

  const auto diagnostics =
    addedMarkersOfType(snapshot.rejected, Marker::POINTS);
  ASSERT_EQ(diagnostics.size(), 1U);
  ASSERT_EQ(diagnostics.front()->points.size(), 1U);
  EXPECT_DOUBLE_EQ(
    diagnostics.front()->points.front().x, non_finite_diagnostic.x);
  EXPECT_DOUBLE_EQ(
    diagnostics.front()->points.front().y, non_finite_diagnostic.y);
  expectAllPointsHaveColor(*diagnostics.front(), 1.0F, 0.05F, 0.05F);

  // A duplicate is bookkeeping, not a red terrain-invalid entity.
  EXPECT_FALSE(containsPointAt(snapshot.rejected, duplicate_candidate));
}

TEST(PlannerVisualization, OutOfRangeRejectedEdgeSourcesAreIgnoredSafely)
{
  PlanResult result;
  result.nodes = {
    makeNode(0U, TerrainPoint{0.0, 0.0, 0.5}, GraphNodeRole::kStart),
  };
  const NodeId bad_source = std::numeric_limits<NodeId>::max();
  result.rejected = {
    makeRejection(
      bad_source, Point2D{1.0, 0.0},
      RejectedSampleKind::kExpansionEdgeInvalid),
    makeRejection(
      bad_source, Point2D{2.0, 0.0},
      RejectedSampleKind::kMergeEdgeInvalid),
    makeRejection(
      bad_source, Point2D{3.0, 0.0},
      RejectedSampleKind::kGoalEdgeInvalid),
  };

  PlannerVisualizationSnapshot snapshot;
  EXPECT_NO_THROW(snapshot = render(result));
  EXPECT_EQ(snapshot.rejected_total, 3U);
  EXPECT_EQ(snapshot.rejected_shown, 0U);
  EXPECT_FALSE(snapshot.rejected_truncated);
  for (const auto & marker : snapshot.rejected.markers) {
    if (marker.action == Marker::ADD) {
      EXPECT_TRUE(marker.points.empty());
    }
  }
}

TEST(PlannerVisualization, NonFiniteCandidateCoordinatesAreNeverPublished)
{
  PlanResult result;
  result.rejected = {
    makeRejection(
      0U,
      Point2D{std::numeric_limits<double>::quiet_NaN(), 1.0},
      RejectedSampleKind::kNonFiniteEvaluation),
    makeRejection(
      0U,
      Point2D{1.0, std::numeric_limits<double>::infinity()},
      RejectedSampleKind::kNodeInvalid),
  };

  const PlannerVisualizationSnapshot snapshot = render(result);
  EXPECT_EQ(snapshot.rejected_total, 2U);
  EXPECT_EQ(snapshot.rejected_shown, 0U);
  EXPECT_FALSE(snapshot.rejected_truncated);
  for (const auto & marker : snapshot.rejected.markers) {
    if (marker.action == Marker::ADD) {
      EXPECT_TRUE(marker.points.empty());
    }
  }
}

TEST(PlannerVisualization, EmptyDensePathDeletesOldPathAndAddsNoFinalPath)
{
  const PlannerVisualizationSnapshot snapshot = render(PlanResult{}, {});

  EXPECT_TRUE(hasDeleteAll(snapshot.edges));
  EXPECT_TRUE(
    addedMarkersOfType(snapshot.edges, Marker::LINE_STRIP).empty());
}

TEST(PlannerVisualization, RejectionCapReportsAttemptsShownTotalAndTruncation)
{
  PlanResult result;
  result.nodes = {
    makeNode(0U, TerrainPoint{0.0, 0.0, 0.5}, GraphNodeRole::kStart),
  };
  result.rejected = {
    makeRejection(
      0U, Point2D{1.0, 0.0}, RejectedSampleKind::kNodeInvalid),
    makeRejection(
      0U, Point2D{1.0, 1.0},
      RejectedSampleKind::kExpansionEdgeInvalid),
    // An undrawn duplicate must neither consume the cap nor count as shown.
    makeRejection(
      0U, Point2D{6.0, 6.0}, RejectedSampleKind::kDuplicateEdge),
    makeRejection(
      0U, Point2D{2.0, 1.0},
      RejectedSampleKind::kMergeEdgeInvalid),
    makeRejection(
      0U, Point2D{3.0, 1.0},
      RejectedSampleKind::kGoalEdgeInvalid),
    makeRejection(
      0U, Point2D{4.0, 1.0},
      RejectedSampleKind::kNonFiniteEvaluation),
    // An invalid source is part of total attempts, but is not drawable.
    makeRejection(
      99U, Point2D{5.0, 1.0},
      RejectedSampleKind::kExpansionEdgeInvalid),
  };

  const PlannerVisualizationSnapshot snapshot = render(result, {}, 3U);
  EXPECT_EQ(snapshot.rejected_total, 7U);
  EXPECT_EQ(snapshot.rejected_shown, 3U);
  EXPECT_TRUE(snapshot.rejected_truncated);

  const auto rejected_nodes =
    addedMarkersOfType(snapshot.rejected, Marker::SPHERE_LIST);
  ASSERT_EQ(rejected_nodes.size(), 1U);
  EXPECT_EQ(rejected_nodes.front()->points.size(), 1U);

  const auto rejected_edges =
    addedMarkersOfType(snapshot.rejected, Marker::LINE_LIST);
  ASSERT_EQ(rejected_edges.size(), 1U);
  // The cap is three rejection attempts: one sphere plus two two-point lines.
  EXPECT_EQ(rejected_edges.front()->points.size(), 4U);

  EXPECT_TRUE(addedMarkersOfType(snapshot.rejected, Marker::POINTS).empty());
}

}  // namespace
}  // namespace rubi_heightmap_wavefront_planner
