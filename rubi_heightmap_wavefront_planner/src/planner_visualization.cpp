#include "rubi_heightmap_wavefront_planner/planner_visualization.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace rubi_heightmap_wavefront_planner
{
namespace
{

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

std_msgs::msg::ColorRGBA makeColor(
  const float red, const float green, const float blue, const float alpha)
{
  std_msgs::msg::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
  return color;
}

bool finitePoint(const Point2D & point) noexcept
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool finitePoint(const TerrainPoint & point) noexcept
{
  return
    std::isfinite(point.x) && std::isfinite(point.y) &&
    std::isfinite(point.z);
}

void validateParameters(const PlannerVisualizationParameters & parameters)
{
  const auto require_positive_finite = [](const double value, const char * name) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and > 0");
      }
    };
  if (parameters.marker_namespace.empty()) {
    throw std::invalid_argument("marker_namespace must not be empty");
  }
  require_positive_finite(
    parameters.node_marker_scale_m, "node_marker_scale_m");
  require_positive_finite(
    parameters.edge_marker_width_m, "edge_marker_width_m");
  require_positive_finite(
    parameters.path_marker_width_m, "path_marker_width_m");
  require_positive_finite(
    parameters.rejected_marker_scale_m, "rejected_marker_scale_m");
  if (parameters.max_rejected_markers == 0U) {
    throw std::invalid_argument("max_rejected_markers must be > 0");
  }
}

Marker makeDeleteAllMarker(
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.action = Marker::DELETEALL;
  return marker;
}

Marker makeBatchedMarker(
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  std::string marker_namespace,
  const int id,
  const int type)
{
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = std::move(marker_namespace);
  marker.id = id;
  marker.type = type;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  return marker;
}

geometry_msgs::msg::Point elevatedPoint(
  const Point2D & point,
  const double fallback_z,
  const double offset_z,
  const TerrainSnapshot & terrain)
{
  geometry_msgs::msg::Point message;
  message.x = point.x;
  message.y = point.y;
  const auto elevation = terrain.elevationAt(point.x, point.y);
  message.z = (elevation && std::isfinite(*elevation)) ? *elevation : fallback_z;
  message.z += offset_z;
  return message;
}

bool isTerrainInvalidEdge(const RejectedSampleKind kind) noexcept
{
  return
    kind == RejectedSampleKind::kExpansionEdgeInvalid ||
    kind == RejectedSampleKind::kMergeEdgeInvalid ||
    kind == RejectedSampleKind::kGoalEdgeInvalid;
}

bool drawableRejection(
  const RejectedSample & rejected,
  const PlanResult & result) noexcept
{
  if (!finitePoint(rejected.candidate)) {
    return false;
  }
  if (
    rejected.kind == RejectedSampleKind::kNodeInvalid ||
    rejected.kind == RejectedSampleKind::kNonFiniteEvaluation)
  {
    return true;
  }
  return
    isTerrainInvalidEdge(rejected.kind) &&
    rejected.source < result.nodes.size() &&
    finitePoint(result.nodes[rejected.source].point);
}

}  // namespace

visualization_msgs::msg::MarkerArray makeDeleteAllMarkerArray(
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  MarkerArray array;
  array.markers.push_back(makeDeleteAllMarker(frame_id, stamp));
  return array;
}

PlannerVisualizationSnapshot makePlannerVisualization(
  const PlanResult & result,
  const TerrainSnapshot & terrain,
  const std::vector<TerrainPoint> & dense_path,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  const PlannerVisualizationParameters & parameters)
{
  validateParameters(parameters);
  if (frame_id.empty()) {
    throw std::invalid_argument("frame_id must not be empty");
  }

  constexpr float kGreenRed = 0.10F;
  constexpr float kGreenGreen = 0.90F;
  constexpr float kGreenBlue = 0.20F;
  const auto valid_green =
    makeColor(kGreenRed, kGreenGreen, kGreenBlue, 0.90F);
  const auto rejected_red = makeColor(1.00F, 0.05F, 0.05F, 0.95F);

  PlannerVisualizationSnapshot snapshot;
  snapshot.rejected_total = result.rejected.size();
  snapshot.nodes = makeDeleteAllMarkerArray(frame_id, stamp);
  snapshot.edges = makeDeleteAllMarkerArray(frame_id, stamp);
  snapshot.rejected = makeDeleteAllMarkerArray(frame_id, stamp);

  Marker nodes = makeBatchedMarker(
    frame_id, stamp, parameters.marker_namespace + "_nodes", 0,
    Marker::SPHERE_LIST);
  nodes.scale.x = parameters.node_marker_scale_m;
  nodes.scale.y = parameters.node_marker_scale_m;
  nodes.scale.z = parameters.node_marker_scale_m;
  nodes.points.reserve(result.nodes.size());
  nodes.colors.reserve(result.nodes.size());
  for (const auto & node : result.nodes) {
    if (!finitePoint(node.point)) {
      continue;
    }
    geometry_msgs::msg::Point point;
    point.x = node.point.x;
    point.y = node.point.y;
    point.z = node.point.z + 0.5 * parameters.node_marker_scale_m;
    nodes.points.push_back(point);
    switch (node.role) {
      case GraphNodeRole::kStart:
        nodes.colors.push_back(makeColor(0.00F, 1.00F, 1.00F, 1.00F));
        break;
      case GraphNodeRole::kGoal:
        nodes.colors.push_back(makeColor(1.00F, 0.00F, 1.00F, 1.00F));
        break;
      case GraphNodeRole::kSampled:
        nodes.colors.push_back(valid_green);
        break;
    }
  }
  snapshot.nodes.markers.push_back(std::move(nodes));

  Marker edges = makeBatchedMarker(
    frame_id, stamp, parameters.marker_namespace + "_valid_edges", 0,
    Marker::LINE_LIST);
  edges.scale.x = parameters.edge_marker_width_m;
  edges.color = valid_green;
  edges.points.reserve(2U * result.edges.size());
  for (const auto & edge : result.edges) {
    if (
      edge.from >= result.nodes.size() || edge.to >= result.nodes.size() ||
      !edge.terrain.valid ||
      !finitePoint(result.nodes[edge.from].point) ||
      !finitePoint(result.nodes[edge.to].point))
    {
      continue;
    }
    for (const NodeId id : {edge.from, edge.to}) {
      geometry_msgs::msg::Point point;
      point.x = result.nodes[id].point.x;
      point.y = result.nodes[id].point.y;
      point.z = result.nodes[id].point.z + 0.02;
      edges.points.push_back(point);
      edges.colors.push_back(valid_green);
    }
  }
  snapshot.edges.markers.push_back(std::move(edges));

  if (!dense_path.empty()) {
    Marker path = makeBatchedMarker(
      frame_id, stamp, parameters.marker_namespace + "_final_path", 1,
      Marker::LINE_STRIP);
    path.scale.x = parameters.path_marker_width_m;
    path.color = makeColor(1.00F, 1.00F, 0.00F, 1.00F);
    path.points.reserve(dense_path.size());
    for (const auto & sample : dense_path) {
      if (!finitePoint(sample)) {
        continue;
      }
      geometry_msgs::msg::Point point;
      point.x = sample.x;
      point.y = sample.y;
      point.z = sample.z + 0.04;
      path.points.push_back(point);
    }
    if (!path.points.empty()) {
      snapshot.edges.markers.push_back(std::move(path));
    }
  }

  Marker rejected_nodes = makeBatchedMarker(
    frame_id, stamp, parameters.marker_namespace + "_rejected_nodes", 0,
    Marker::SPHERE_LIST);
  rejected_nodes.scale.x = parameters.rejected_marker_scale_m;
  rejected_nodes.scale.y = parameters.rejected_marker_scale_m;
  rejected_nodes.scale.z = parameters.rejected_marker_scale_m;
  rejected_nodes.color = rejected_red;

  Marker rejected_edges = makeBatchedMarker(
    frame_id, stamp, parameters.marker_namespace + "_rejected_edges", 1,
    Marker::LINE_LIST);
  rejected_edges.scale.x = parameters.edge_marker_width_m;
  rejected_edges.color = rejected_red;

  Marker non_finite = makeBatchedMarker(
    frame_id, stamp, parameters.marker_namespace + "_non_finite_diagnostics", 2,
    Marker::POINTS);
  non_finite.scale.x = parameters.rejected_marker_scale_m;
  non_finite.scale.y = parameters.rejected_marker_scale_m;
  non_finite.color = rejected_red;

  std::size_t drawable_total = 0U;
  for (const auto & rejected : result.rejected) {
    if (drawableRejection(rejected, result)) {
      ++drawable_total;
    }
  }
  snapshot.rejected_truncated =
    drawable_total > parameters.max_rejected_markers;

  for (const auto & rejected : result.rejected) {
    if (
      snapshot.rejected_shown >= parameters.max_rejected_markers ||
      !drawableRejection(rejected, result))
    {
      continue;
    }

    const double fallback_z =
      rejected.source < result.nodes.size() &&
      std::isfinite(result.nodes[rejected.source].point.z) ?
      result.nodes[rejected.source].point.z : 0.0;
    if (rejected.kind == RejectedSampleKind::kNodeInvalid) {
      rejected_nodes.points.push_back(
        elevatedPoint(
          rejected.candidate, fallback_z,
          0.5 * parameters.rejected_marker_scale_m, terrain));
      ++snapshot.rejected_shown;
      continue;
    }
    if (rejected.kind == RejectedSampleKind::kNonFiniteEvaluation) {
      non_finite.points.push_back(
        elevatedPoint(
          rejected.candidate, fallback_z,
          0.5 * parameters.rejected_marker_scale_m, terrain));
      ++snapshot.rejected_shown;
      continue;
    }
    if (isTerrainInvalidEdge(rejected.kind)) {
      const auto & source = result.nodes[rejected.source].point;
      geometry_msgs::msg::Point source_point;
      source_point.x = source.x;
      source_point.y = source.y;
      source_point.z = source.z + 0.02;
      rejected_edges.points.push_back(source_point);
      rejected_edges.colors.push_back(rejected_red);
      rejected_edges.points.push_back(
        elevatedPoint(rejected.candidate, source.z, 0.02, terrain));
      rejected_edges.colors.push_back(rejected_red);
      ++snapshot.rejected_shown;
    }
  }

  if (!rejected_nodes.points.empty()) {
    snapshot.rejected.markers.push_back(std::move(rejected_nodes));
  }
  if (!rejected_edges.points.empty()) {
    snapshot.rejected.markers.push_back(std::move(rejected_edges));
  }
  if (!non_finite.points.empty()) {
    snapshot.rejected.markers.push_back(std::move(non_finite));
  }
  return snapshot;
}

}  // namespace rubi_heightmap_wavefront_planner
