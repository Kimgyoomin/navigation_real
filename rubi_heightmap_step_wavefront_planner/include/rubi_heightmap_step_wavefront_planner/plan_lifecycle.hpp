#pragma once

#include <cstdint>
#include <string>

namespace rubi_heightmap_step_wavefront_planner
{

struct PlanLifecycleToken
{
  std::uint64_t goal_epoch{0U};
  std::uint64_t map_generation{0U};
  std::string frame_id;
};

bool mayPublish(
  const PlanLifecycleToken & token,
  std::uint64_t current_goal_epoch,
  std::uint64_t current_map_generation,
  const std::string & current_frame_id,
  bool revalidated_latest_generation) noexcept;

}  // namespace rubi_heightmap_step_wavefront_planner
