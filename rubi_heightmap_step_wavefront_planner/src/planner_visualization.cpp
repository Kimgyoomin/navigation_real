#include "rubi_heightmap_step_wavefront_planner/planner_visualization.hpp"

#include <algorithm>

#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace rubi_heightmap_step_wavefront_planner
{
namespace
{

std_msgs::msg::ColorRGBA color(float red, float green, float blue)
{
  std_msgs::msg::ColorRGBA value;
  value.r = red;
  value.g = green;
  value.b = blue;
  value.a = 1.0F;
  return value;
}

geometry_msgs::msg::Point point(Point2D xy, double z = 0.0)
{
  geometry_msgs::msg::Point value;
  value.x = xy.x;
  value.y = xy.y;
  value.z = z;
  return value;
}

visualization_msgs::msg::Marker marker(
  const std::string & frame,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & name_space,
  int id,
  int type)
{
  visualization_msgs::msg::Marker value;
  value.header.frame_id = frame;
  value.header.stamp = stamp;
  value.ns = name_space;
  value.id = id;
  value.type = type;
  value.action = visualization_msgs::msg::Marker::ADD;
  value.pose.orientation.w = 1.0;
  return value;
}

}  // namespace

VisualizationSnapshot makeVisualization(
  const PlanResult & result,
  const std::vector<TerrainPoint> & dense_path,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  const VisualizationParameters & parameters)
{
  VisualizationSnapshot output;
  auto accepted_nodes = marker(
    frame_id, stamp, "sampling_nodes/accepted", 0,
    visualization_msgs::msg::Marker::SPHERE_LIST);
  auto frontier_nodes = marker(
    frame_id, stamp, "sampling_nodes/frontier", 1,
    visualization_msgs::msg::Marker::SPHERE_LIST);
  auto invalid_nodes = marker(
    frame_id, stamp, "sampling_nodes/invalid", 2,
    visualization_msgs::msg::Marker::SPHERE_LIST);
  auto goal_nodes = marker(
    frame_id, stamp, "sampling_nodes/goal_connected", 3,
    visualization_msgs::msg::Marker::SPHERE_LIST);
  for (auto * item : {&accepted_nodes, &frontier_nodes, &invalid_nodes, &goal_nodes}) {
    item->scale.x = parameters.node_scale_m;
    item->scale.y = parameters.node_scale_m;
    item->scale.z = parameters.node_scale_m;
  }
  accepted_nodes.color = color(0.0F, 1.0F, 0.0F);
  frontier_nodes.color = color(0.0F, 1.0F, 1.0F);
  invalid_nodes.color = color(1.0F, 0.0F, 0.0F);
  goal_nodes.color = color(1.0F, 1.0F, 0.0F);
  const std::size_t stride = std::max<std::size_t>(1U, parameters.stride);
  std::size_t shown_nodes = 0U;
  for (std::size_t index = 0U; index < result.nodes.size(); index += stride) {
    if (parameters.max_nodes > 0U && shown_nodes >= parameters.max_nodes) {break;}
    const auto & node = result.nodes[index];
    auto * target = &accepted_nodes;
    if (!result.path_node_ids.empty() && node.id == result.path_node_ids.back()) {
      target = &goal_nodes;
    }
    if (node.state == GraphNodeState::kFrontier) {target = &frontier_nodes;}
    if (node.state == GraphNodeState::kInvalid) {target = &invalid_nodes;}
    target->points.push_back(point(node.point, node.elevation_m));
    ++shown_nodes;
  }
  if (!accepted_nodes.points.empty()) {output.nodes.markers.push_back(std::move(accepted_nodes));}
  if (!frontier_nodes.points.empty()) {output.nodes.markers.push_back(std::move(frontier_nodes));}
  if (!invalid_nodes.points.empty()) {output.nodes.markers.push_back(std::move(invalid_nodes));}
  if (!goal_nodes.points.empty()) {output.nodes.markers.push_back(std::move(goal_nodes));}

  auto edges = marker(
    frame_id, stamp, "step_wavefront_valid_edges", 0,
    visualization_msgs::msg::Marker::LINE_LIST);
  edges.scale.x = parameters.edge_width_m;
  edges.color = color(0.0F, 1.0F, 0.0F);
  std::size_t shown_edges = 0U;
  for (std::size_t index = 0U; index < result.edges.size(); index += stride) {
    if (parameters.max_nodes > 0U && shown_edges >= parameters.max_nodes) {break;}
    const auto & edge = result.edges[index];
    if (edge.from >= result.nodes.size() || edge.to >= result.nodes.size()) {continue;}
    edges.points.push_back(
      point(
        result.nodes[edge.from].point,
        result.nodes[edge.from].elevation_m));
    edges.points.push_back(point(result.nodes[edge.to].point, result.nodes[edge.to].elevation_m));
    ++shown_edges;
  }
  output.edges.markers.push_back(std::move(edges));
  auto path = marker(
    frame_id, stamp, "step_wavefront_final_path", 1,
    visualization_msgs::msg::Marker::LINE_STRIP);
  path.scale.x = parameters.path_width_m;
  path.color = color(1.0F, 1.0F, 0.0F);
  for (const auto & pose : dense_path) {
    path.points.push_back(point(Point2D{pose.x, pose.y}, pose.z));
  }
  output.edges.markers.push_back(std::move(path));

  output.rejected_total = result.rejected.size();
  const std::size_t shown = std::min(result.rejected.size(), parameters.max_rejected_markers);
  auto red = marker(
    frame_id, stamp, "sampling_rejected/costmap", 0,
    visualization_msgs::msg::Marker::LINE_LIST);
  auto orange = marker(
    frame_id, stamp, "sampling_rejected/edge", 1,
    visualization_msgs::msg::Marker::LINE_LIST);
  auto magenta = marker(
    frame_id, stamp, "step_wavefront_step_rejections", 2,
    visualization_msgs::msg::Marker::LINE_LIST);
  auto purple = marker(
    frame_id, stamp, "sampling_rejected/trg_collision", 3,
    visualization_msgs::msg::Marker::LINE_LIST);
  auto gray = marker(
    frame_id, stamp, "sampling_rejected/isolated", 4,
    visualization_msgs::msg::Marker::LINE_LIST);
  for (auto * item : {&red, &orange, &magenta, &purple, &gray}) {
    item->scale.x = parameters.rejected_scale_m;
  }
  red.color = color(1.0F, 0.0F, 0.0F);
  orange.color = color(1.0F, 0.5F, 0.0F);
  magenta.color = color(1.0F, 0.0F, 1.0F);
  purple.color = color(0.6F, 0.0F, 1.0F);
  gray.color = color(0.5F, 0.5F, 0.5F);
  for (std::size_t index = 0U; index < shown; ++index) {
    const auto & rejection = result.rejected[index];
    visualization_msgs::msg::Marker * target = &red;
    if (rejection.reason == StepInvalidReason::kClearanceViolation ||
      rejection.reason == StepInvalidReason::kInsufficientClearanceSupport)
    {
      target = &orange;
    } else if (rejection.reason == StepInvalidReason::kStepLimit) {
      target = &magenta;
    } else if (rejection.reason == StepInvalidReason::kTrgCollision) {
      target = &purple;
    } else if (rejection.reason == StepInvalidReason::kIsolatedNode) {
      target = &gray;
    }
    target->points.push_back(point(rejection.from));
    target->points.push_back(point(rejection.to));
    ++output.rejected_shown;
  }
  output.rejected.markers.push_back(std::move(red));
  output.rejected.markers.push_back(std::move(orange));
  output.rejected.markers.push_back(std::move(magenta));
  output.rejected.markers.push_back(std::move(purple));
  output.rejected.markers.push_back(std::move(gray));
  return output;
}

visualization_msgs::msg::MarkerArray makeDeleteAllMarkers(
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  visualization_msgs::msg::MarkerArray output;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = frame_id;
  clear.header.stamp = stamp;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  output.markers.push_back(clear);
  return output;
}

visualization_msgs::msg::Marker makeRevalidationFailureMarker(
  const TerrainPoint & from,
  const TerrainPoint & to,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  const double width_m)
{
  auto output = marker(
    frame_id, stamp, "step_wavefront_revalidation_failure", 0,
    visualization_msgs::msg::Marker::LINE_LIST);
  output.scale.x = width_m;
  output.color = color(1.0F, 0.0F, 0.0F);
  output.points.push_back(point({from.x, from.y}, from.z));
  output.points.push_back(point({to.x, to.y}, to.z));
  return output;
}

visualization_msgs::msg::Marker makeDeleteRevalidationFailureMarker(
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  auto output = marker(
    frame_id, stamp, "step_wavefront_revalidation_failure", 0,
    visualization_msgs::msg::Marker::LINE_LIST);
  output.action = visualization_msgs::msg::Marker::DELETE;
  return output;
}

visualization_msgs::msg::MarkerArray makeQuerySnapMarkers(
  const std::optional<PlanningQueryResolution> & start,
  const std::optional<PlanningQueryResolution> & goal,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp)
{
  visualization_msgs::msg::MarkerArray output = makeDeleteAllMarkers(frame_id, stamp);
  auto snap_lines = marker(
    frame_id, stamp, "step_wavefront_query_snap_lines", 0,
    visualization_msgs::msg::Marker::LINE_LIST);
  snap_lines.scale.x = 0.02;
  snap_lines.color = color(1.0F, 1.0F, 1.0F);

  const auto add_query = [&output, &snap_lines, &frame_id, &stamp](
      const PlanningQueryResolution & query, const bool is_start)
    {
      const float effective_red = is_start ? 0.0F : 1.0F;
      const float effective_green = is_start ? 1.0F : 0.0F;
      const float effective_blue = 1.0F;
      auto effective = marker(
        frame_id, stamp, "step_wavefront_effective_queries", is_start ? 0 : 1,
        visualization_msgs::msg::Marker::SPHERE);
      effective.scale.x = 0.12;
      effective.scale.y = 0.12;
      effective.scale.z = 0.12;
      effective.color = color(effective_red, effective_green, effective_blue);
      effective.pose.position = point(
        query.effective, query.effective_evaluation.elevation_m + 0.06);
      output.markers.push_back(std::move(effective));

      if (!query.snapped) {
        return;
      }
      auto requested = marker(
        frame_id, stamp, "step_wavefront_requested_queries", is_start ? 0 : 1,
        visualization_msgs::msg::Marker::SPHERE);
      requested.scale.x = 0.09;
      requested.scale.y = 0.09;
      requested.scale.z = 0.09;
      requested.color = is_start ? color(0.55F, 0.55F, 0.55F) :
        color(1.0F, 0.55F, 0.0F);
      requested.pose.position = point(
        query.requested, query.effective_evaluation.elevation_m + 0.06);
      output.markers.push_back(std::move(requested));
      snap_lines.points.push_back(point(
          query.requested, query.effective_evaluation.elevation_m + 0.06));
      snap_lines.points.push_back(point(
          query.effective, query.effective_evaluation.elevation_m + 0.06));
    };

  if (start) {
    add_query(*start, true);
  }
  if (goal) {
    add_query(*goal, false);
  }
  if (!snap_lines.points.empty()) {
    output.markers.push_back(std::move(snap_lines));
  }
  return output;
}

}  // namespace rubi_heightmap_step_wavefront_planner
