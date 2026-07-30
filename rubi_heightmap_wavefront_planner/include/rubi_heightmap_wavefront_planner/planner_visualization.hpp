#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rubi_heightmap_wavefront_planner/terrain_snapshot.hpp"
#include "rubi_heightmap_wavefront_planner/wavefront_planner.hpp"

namespace rubi_heightmap_wavefront_planner
{

struct PlannerVisualizationParameters
{
  std::string marker_namespace{"wavefront"};
  double node_marker_scale_m{0.08};
  double edge_marker_width_m{0.025};
  double path_marker_width_m{0.08};
  double rejected_marker_scale_m{0.07};
  std::size_t max_rejected_markers{5000U};
};

struct PlannerVisualizationSnapshot
{
  visualization_msgs::msg::MarkerArray nodes;
  visualization_msgs::msg::MarkerArray edges;
  visualization_msgs::msg::MarkerArray rejected;
  std::size_t rejected_total{0U};
  std::size_t rejected_shown{0U};
  bool rejected_truncated{false};
};

/**
 * @brief Build one final batched RViz snapshot for a planning result.
 *
 * The helper creates ROS visualization messages only. It does not evaluate
 * terrain or alter the planner result.
 */
PlannerVisualizationSnapshot makePlannerVisualization(
  const PlanResult & result,
  const TerrainSnapshot & terrain,
  const std::vector<TerrainPoint> & dense_path,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  const PlannerVisualizationParameters & parameters);

visualization_msgs::msg::MarkerArray makeDeleteAllMarkerArray(
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp);

}  // namespace rubi_heightmap_wavefront_planner
