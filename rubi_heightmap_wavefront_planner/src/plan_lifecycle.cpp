#include "rubi_heightmap_wavefront_planner/plan_lifecycle.hpp"

namespace rubi_heightmap_wavefront_planner
{

bool mayCommitAfterRevalidation(
  const PlanLifecycleToken & token,
  const std::uint64_t current_goal_epoch,
  const std::string & current_map_frame) noexcept
{
  return token.goal_epoch == current_goal_epoch && token.map_frame == current_map_frame;
}

bool shouldReplaceQueuedRequest(
  const bool queued_request_is_automatic,
  const bool incoming_request_is_automatic) noexcept
{
  return !incoming_request_is_automatic || queued_request_is_automatic;
}

bool AutoReplanGate::tryAcquire() noexcept
{
  if (acquired_) {
    return false;
  }
  acquired_ = true;
  return true;
}

void AutoReplanGate::reset() noexcept
{
  acquired_ = false;
}

bool AutoReplanGate::acquired() const noexcept
{
  return acquired_;
}

}  // namespace rubi_heightmap_wavefront_planner
