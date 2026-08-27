#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "rubi_heightmap_step_wavefront_planner/path_revalidation.hpp"
#include "rubi_heightmap_step_wavefront_planner/step_wavefront_planner.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

struct VisualizationParameters
{
  double node_scale_m{0.08};
  double edge_width_m{0.025};
  double path_width_m{0.08};
  double rejected_scale_m{0.07};
  std::size_t max_rejected_markers{5000U};
};

struct VisualizationSnapshot
{
  visualization_msgs::msg::MarkerArray nodes;
  visualization_msgs::msg::MarkerArray edges;
  visualization_msgs::msg::MarkerArray rejected;
  std::size_t rejected_total{0U};
  std::size_t rejected_shown{0U};
};

VisualizationSnapshot makeVisualization(
  const PlanResult & result,
  const std::vector<TerrainPoint> & dense_path,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  const VisualizationParameters & parameters);

visualization_msgs::msg::MarkerArray makeDeleteAllMarkers(
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp);

visualization_msgs::msg::Marker makeRevalidationFailureMarker(
  const TerrainPoint & from,
  const TerrainPoint & to,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  double width_m);

visualization_msgs::msg::Marker makeDeleteRevalidationFailureMarker(
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp);

}  // namespace rubi_heightmap_step_wavefront_planner
