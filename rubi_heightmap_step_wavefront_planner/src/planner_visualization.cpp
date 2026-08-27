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
  auto nodes = marker(
    frame_id, stamp, "step_wavefront_valid_nodes", 0,
    visualization_msgs::msg::Marker::SPHERE_LIST);
  nodes.scale.x = parameters.node_scale_m;
  nodes.scale.y = parameters.node_scale_m;
  nodes.scale.z = parameters.node_scale_m;
  nodes.color = color(0.0F, 1.0F, 0.0F);
  for (const auto & node : result.nodes) {
    nodes.points.push_back(point(node.point, node.elevation_m));
  }
  output.nodes.markers.push_back(std::move(nodes));

  auto edges = marker(
    frame_id, stamp, "step_wavefront_valid_edges", 0,
    visualization_msgs::msg::Marker::LINE_LIST);
  edges.scale.x = parameters.edge_width_m;
  edges.color = color(0.0F, 1.0F, 0.0F);
  for (const auto & edge : result.edges) {
    if (edge.from >= result.nodes.size() || edge.to >= result.nodes.size()) {continue;}
    edges.points.push_back(
      point(
        result.nodes[edge.from].point,
        result.nodes[edge.from].elevation_m));
    edges.points.push_back(point(result.nodes[edge.to].point, result.nodes[edge.to].elevation_m));
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
    frame_id, stamp, "step_wavefront_unknown_rejections", 0,
    visualization_msgs::msg::Marker::LINE_LIST);
  auto orange = marker(
    frame_id, stamp, "step_wavefront_clearance_rejections", 1,
    visualization_msgs::msg::Marker::LINE_LIST);
  auto magenta = marker(
    frame_id, stamp, "step_wavefront_step_rejections", 2,
    visualization_msgs::msg::Marker::LINE_LIST);
  for (auto * item : {&red, &orange, &magenta}) {
    item->scale.x = parameters.rejected_scale_m;
  }
  red.color = color(1.0F, 0.0F, 0.0F);
  orange.color = color(1.0F, 0.5F, 0.0F);
  magenta.color = color(1.0F, 0.0F, 1.0F);
  for (std::size_t index = 0U; index < shown; ++index) {
    const auto & rejection = result.rejected[index];
    visualization_msgs::msg::Marker * target = &red;
    if (rejection.reason == StepInvalidReason::kClearanceViolation ||
      rejection.reason == StepInvalidReason::kInsufficientClearanceSupport)
    {
      target = &orange;
    } else if (rejection.reason == StepInvalidReason::kStepLimit) {
      target = &magenta;
    }
    target->points.push_back(point(rejection.from));
    target->points.push_back(point(rejection.to));
    ++output.rejected_shown;
  }
  output.rejected.markers.push_back(std::move(red));
  output.rejected.markers.push_back(std::move(orange));
  output.rejected.markers.push_back(std::move(magenta));
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

}  // namespace rubi_heightmap_step_wavefront_planner
