#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "rubi_heightmap_step_wavefront_planner/step_evaluator.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

bool isImmediateInvalidation(StepInvalidReason reason) noexcept;

struct SoftFailureTracker
{
  StepInvalidReason last_reason{StepInvalidReason::kNone};
  std::size_t streak{0U};

  bool observe(StepInvalidReason reason, std::size_t confirmations) noexcept;
  void reset() noexcept;
};

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
