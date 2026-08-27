#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace rubi_heightmap_step_wavefront_planner
{

enum class PlanningState
{
  kWaitingForMap,
  kIdle,
  kPlanning,
  kTracking,
  kVerifyingPath,
  kReplanning,
  kWaitingRetry,
  kBlocked,
};

enum class PlanningEvent
{
  kMapReceived,
  kGoalReceived,
  kPlanSucceeded,
  kPlanFailed,
  kPathValid,
  kPathSuspectedInvalid,
  kPathInvalidConfirmed,
  kPathRecovered,
  kRetryReady,
  kRetryExhausted,
  kFrameChanged,
};

std::string_view toString(PlanningState state) noexcept;
std::string_view toString(PlanningEvent event) noexcept;

struct PlanningFsmParameters
{
  std::size_t path_invalid_confirmations{2U};
  std::size_t path_recovery_confirmations{2U};
  std::size_t max_replan_attempts{5U};
  std::chrono::steady_clock::duration replan_retry_period{std::chrono::milliseconds(500)};
  bool replan_retry_requires_new_map{true};
};

struct PlanningTransition
{
  PlanningState from{PlanningState::kWaitingForMap};
  PlanningState to{PlanningState::kWaitingForMap};
  PlanningEvent event{PlanningEvent::kMapReceived};
  std::string reason;
};

struct PathVerificationDecision
{
  bool suspend_motion{false};
  bool clear_retained_path{false};
  bool republish_retained_path{false};
  bool start_replan{false};
  bool blocked{false};
};

class PlanningFsm
{
public:
  using Clock = std::chrono::steady_clock;

  explicit PlanningFsm(PlanningFsmParameters parameters = {});

  PlanningState state() const noexcept {return state_;}
  std::size_t invalidStreak() const noexcept {return invalid_streak_;}
  std::size_t validStreak() const noexcept {return valid_streak_;}
  std::size_t replanAttemptCount() const noexcept {return replan_attempt_count_;}
  std::uint64_t lastReplanGeneration() const noexcept {return last_replan_generation_;}
  const PlanningFsmParameters & parameters() const noexcept {return parameters_;}

  void transitionTo(
    PlanningState next_state,
    PlanningEvent event,
    std::string_view reason);

  void onMapReceived(bool pending_goal);
  void onGoalReceived(bool map_available);
  void onFrameChanged(bool map_available);
  void onPlanSucceeded();
  void onPlanFailed(Clock::time_point now);

  PathVerificationDecision observePath(
    bool valid,
    bool invalid_input,
    std::uint64_t map_generation,
    std::string_view reason);

  bool retryReady(Clock::time_point now, std::uint64_t map_generation);
  std::vector<PlanningTransition> takeTransitions();

private:
  void resetGoalLifecycle() noexcept;

  PlanningFsmParameters parameters_;
  PlanningState state_{PlanningState::kWaitingForMap};
  std::size_t invalid_streak_{0U};
  std::size_t valid_streak_{0U};
  std::size_t replan_attempt_count_{0U};
  std::uint64_t last_replan_generation_{0U};
  Clock::time_point retry_ready_at_{};
  std::vector<PlanningTransition> transitions_;
};

}  // namespace rubi_heightmap_step_wavefront_planner
