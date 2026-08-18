#include "rubi_heightmap_step_wavefront_planner/plan_lifecycle.hpp"

namespace rubi_heightmap_step_wavefront_planner
{

bool isImmediateInvalidation(const StepInvalidReason reason) noexcept
{
  return reason == StepInvalidReason::kStepLimit ||
         reason == StepInvalidReason::kClearanceViolation ||
         reason == StepInvalidReason::kInvalidInput;
}

bool SoftFailureTracker::observe(
  const StepInvalidReason reason,
  const std::size_t confirmations) noexcept
{
  if (isImmediateInvalidation(reason)) {return true;}
  if (reason == StepInvalidReason::kNone) {
    reset();
    return false;
  }
  if (last_reason == reason) {++streak;} else {
    last_reason = reason;
    streak = 1U;
  }
  return streak >= confirmations;
}

void SoftFailureTracker::reset() noexcept
{
  last_reason = StepInvalidReason::kNone;
  streak = 0U;
}

bool mayPublish(
  const PlanLifecycleToken & token,
  const std::uint64_t current_goal_epoch,
  const std::uint64_t current_map_generation,
  const std::string & current_frame_id,
  const bool revalidated_latest_generation) noexcept
{
  return token.goal_epoch == current_goal_epoch &&
         token.frame_id == current_frame_id &&
         (token.map_generation == current_map_generation || revalidated_latest_generation);
}

}  // namespace rubi_heightmap_step_wavefront_planner
