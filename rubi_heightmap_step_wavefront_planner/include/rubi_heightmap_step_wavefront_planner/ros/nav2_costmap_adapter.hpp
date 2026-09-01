#pragma once

#include "nav2_msgs/msg/costmap.hpp"
#include "rubi_heightmap_step_wavefront_planner/map/costmap_snapshot.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

class Nav2CostmapAdapter
{
public:
  CostmapSnapshot makeSnapshot(const nav2_msgs::msg::Costmap & message) const;
};

}  // namespace rubi_heightmap_step_wavefront_planner
