#include "rubi_heightmap_step_wavefront_planner/planning/planning_fsm.hpp"

#include <stdexcept>
#include <utility>

namespace rubi_heightmap_step_wavefront_planner
{

std::string_view toString(const PlanningState state) noexcept
{
  switch (state) {
    case PlanningState::kWaitingForMap: return "WAITING_FOR_MAP";
    case PlanningState::kIdle: return "IDLE";
    case PlanningState::kPlanning: return "PLANNING";
    case PlanningState::kTracking: return "TRACKING";
    case PlanningState::kVerifyingPath: return "VERIFYING_PATH";
    case PlanningState::kReplanning: return "REPLANNING";
    case PlanningState::kWaitingRetry: return "WAITING_RETRY";
    case PlanningState::kBlocked: return "BLOCKED";
  }
  return "BLOCKED";
}

std::string_view toString(const PlanningEvent event) noexcept
{
  switch (event) {
    case PlanningEvent::kMapReceived: return "MAP_RECEIVED";
    case PlanningEvent::kGoalReceived: return "GOAL_RECEIVED";
    case PlanningEvent::kPlanSucceeded: return "PLAN_SUCCEEDED";
    case PlanningEvent::kPlanFailed: return "PLAN_FAILED";
    case PlanningEvent::kPathValid: return "PATH_VALID";
    case PlanningEvent::kPathSuspectedInvalid: return "PATH_SUSPECTED";
    case PlanningEvent::kPathInvalidConfirmed: return "PATH_INVALID_CONFIRMED";
    case PlanningEvent::kPathRecovered: return "PATH_RECOVERED";
    case PlanningEvent::kRetryReady: return "RETRY_READY";
    case PlanningEvent::kRetryExhausted: return "RETRY_EXHAUSTED";
    case PlanningEvent::kFrameChanged: return "FRAME_CHANGED";
  }
  return "RETRY_EXHAUSTED";
}

PlanningFsm::PlanningFsm(PlanningFsmParameters parameters)
: parameters_(std::move(parameters))
{
  if (parameters_.path_invalid_confirmations == 0U ||
    parameters_.path_recovery_confirmations == 0U ||
    parameters_.max_replan_attempts == 0U ||
    parameters_.replan_retry_period < Clock::duration::zero())
  {
    throw std::invalid_argument("invalid Planning FSM parameters");
  }
}

void PlanningFsm::transitionTo(
  const PlanningState next_state,
  const PlanningEvent event,
  const std::string_view reason)
{
  transitions_.push_back({state_, next_state, event, std::string(reason)});
  state_ = next_state;
}

void PlanningFsm::resetGoalLifecycle() noexcept
{
  invalid_streak_ = 0U;
  valid_streak_ = 0U;
  replan_attempt_count_ = 0U;
  last_replan_generation_ = 0U;
  retry_ready_at_ = Clock::time_point{};
}

void PlanningFsm::onMapReceived(const bool pending_goal)
{
  if (state_ != PlanningState::kWaitingForMap) {return;}
  transitionTo(
    pending_goal ? PlanningState::kPlanning : PlanningState::kIdle,
    PlanningEvent::kMapReceived,
    pending_goal ? "first_map_with_pending_goal" : "first_map");
}

void PlanningFsm::onGoalReceived(const bool map_available)
{
  resetGoalLifecycle();
  transitionTo(
    map_available ? PlanningState::kPlanning : PlanningState::kWaitingForMap,
    PlanningEvent::kGoalReceived,
    map_available ? "external_goal" : "pending_goal_without_map");
}

void PlanningFsm::onFrameChanged(const bool map_available)
{
  resetGoalLifecycle();
  transitionTo(
    map_available ? PlanningState::kIdle : PlanningState::kWaitingForMap,
    PlanningEvent::kFrameChanged,
    "map_frame_changed");
}

void PlanningFsm::onPlanSucceeded()
{
  resetGoalLifecycle();
  transitionTo(
    PlanningState::kTracking, PlanningEvent::kPlanSucceeded, "valid_path_published");
}

void PlanningFsm::onPlanFailed(const Clock::time_point now)
{
  if (state_ == PlanningState::kReplanning) {
    transitionTo(
      PlanningState::kWaitingRetry, PlanningEvent::kPlanFailed, "replan_failed");
    retry_ready_at_ = now + parameters_.replan_retry_period;
    if (replan_attempt_count_ >= parameters_.max_replan_attempts) {
      transitionTo(
        PlanningState::kBlocked, PlanningEvent::kRetryExhausted,
        "replan_attempt_budget_exhausted");
    }
    return;
  }
  transitionTo(
    PlanningState::kBlocked, PlanningEvent::kPlanFailed, "external_plan_failed");
}

PathVerificationDecision PlanningFsm::observePath(
  const bool valid,
  const bool invalid_input,
  const std::uint64_t map_generation,
  const std::string_view reason)
{
  PathVerificationDecision decision;
  if (state_ != PlanningState::kTracking && state_ != PlanningState::kVerifyingPath) {
    return decision;
  }

  if (invalid_input) {
    decision.suspend_motion = state_ == PlanningState::kTracking;
    decision.clear_retained_path = true;
    decision.blocked = true;
    invalid_streak_ = 0U;
    valid_streak_ = 0U;
    transitionTo(
      PlanningState::kBlocked, PlanningEvent::kPathInvalidConfirmed, reason);
    return decision;
  }

  if (valid) {
    invalid_streak_ = 0U;
    if (state_ == PlanningState::kTracking) {
      valid_streak_ = 0U;
      return decision;
    }
    ++valid_streak_;
    if (valid_streak_ >= parameters_.path_recovery_confirmations) {
      valid_streak_ = 0U;
      decision.republish_retained_path = true;
      transitionTo(
        PlanningState::kTracking, PlanningEvent::kPathRecovered,
        "retained_path_revalidated");
    } else {
      transitionTo(
        PlanningState::kVerifyingPath, PlanningEvent::kPathValid,
        "path_recovery_confirmation_pending");
    }
    return decision;
  }

  valid_streak_ = 0U;
  ++invalid_streak_;
  if (state_ == PlanningState::kTracking) {
    decision.suspend_motion = true;
    transitionTo(
      PlanningState::kVerifyingPath, PlanningEvent::kPathSuspectedInvalid, reason);
  } else {
    transitionTo(
      PlanningState::kVerifyingPath, PlanningEvent::kPathSuspectedInvalid, reason);
  }

  if (invalid_streak_ >= parameters_.path_invalid_confirmations) {
    decision.clear_retained_path = true;
    decision.start_replan = true;
    replan_attempt_count_ = 1U;
    last_replan_generation_ = map_generation;
    transitionTo(
      PlanningState::kReplanning, PlanningEvent::kPathInvalidConfirmed, reason);
  }
  return decision;
}

bool PlanningFsm::retryReady(
  const Clock::time_point now, const std::uint64_t map_generation)
{
  if (state_ != PlanningState::kWaitingRetry) {return false;}
  if (replan_attempt_count_ >= parameters_.max_replan_attempts) {
    transitionTo(
      PlanningState::kBlocked, PlanningEvent::kRetryExhausted,
      "replan_attempt_budget_exhausted");
    return false;
  }
  if (now < retry_ready_at_) {return false;}
  if (parameters_.replan_retry_requires_new_map &&
    map_generation <= last_replan_generation_)
  {
    return false;
  }
  ++replan_attempt_count_;
  last_replan_generation_ = map_generation;
  transitionTo(
    PlanningState::kReplanning, PlanningEvent::kRetryReady,
    "retry_period_and_map_gate_satisfied");
  return true;
}

std::vector<PlanningTransition> PlanningFsm::takeTransitions()
{
  std::vector<PlanningTransition> output;
  output.swap(transitions_);
  return output;
}

}  // namespace rubi_heightmap_step_wavefront_planner
