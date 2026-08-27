#include <chrono>

#include <gtest/gtest.h>

#include "rubi_heightmap_step_wavefront_planner/planning/planning_fsm.hpp"

namespace planner = rubi_heightmap_step_wavefront_planner;
using namespace std::chrono_literals;

planner::PlanningFsm trackingFsm(planner::PlanningFsmParameters parameters = {})
{
  planner::PlanningFsm fsm(parameters);
  fsm.onMapReceived(false);
  fsm.onGoalReceived(true);
  fsm.onPlanSucceeded();
  (void)fsm.takeTransitions();
  return fsm;
}

TEST(PlanningFsm, PendingGoalStartsPlanningOnFirstMap)
{
  planner::PlanningFsm fsm;
  fsm.onGoalReceived(false);
  EXPECT_EQ(fsm.state(), planner::PlanningState::kWaitingForMap);
  fsm.onMapReceived(true);
  EXPECT_EQ(fsm.state(), planner::PlanningState::kPlanning);
}

TEST(PlanningFsm, SuccessfulExternalPlanningTracks)
{
  planner::PlanningFsm fsm;
  fsm.onMapReceived(false);
  fsm.onGoalReceived(true);
  fsm.onPlanSucceeded();
  EXPECT_EQ(fsm.state(), planner::PlanningState::kTracking);
}

TEST(PlanningFsm, FirstInvalidFrameSuspendsButRetainsPath)
{
  auto fsm = trackingFsm();
  const auto decision = fsm.observePath(false, false, 10U, "clearance_violation");
  EXPECT_EQ(fsm.state(), planner::PlanningState::kVerifyingPath);
  EXPECT_TRUE(decision.suspend_motion);
  EXPECT_FALSE(decision.clear_retained_path);
  EXPECT_FALSE(decision.start_replan);
  EXPECT_EQ(fsm.invalidStreak(), 1U);
}

TEST(PlanningFsm, TwoValidFramesRecoverRetainedPathWithoutReplan)
{
  auto fsm = trackingFsm();
  (void)fsm.observePath(false, false, 10U, "clearance_violation");
  EXPECT_FALSE(fsm.observePath(true, false, 11U, "none").republish_retained_path);
  const auto recovered = fsm.observePath(true, false, 12U, "none");
  EXPECT_TRUE(recovered.republish_retained_path);
  EXPECT_FALSE(recovered.start_replan);
  EXPECT_EQ(fsm.state(), planner::PlanningState::kTracking);
  EXPECT_EQ(fsm.replanAttemptCount(), 0U);
}

TEST(PlanningFsm, TwoInvalidFramesConfirmAndStartReplanning)
{
  auto fsm = trackingFsm();
  (void)fsm.observePath(false, false, 20U, "clearance_violation");
  const auto confirmed = fsm.observePath(false, false, 21U, "clearance_violation");
  EXPECT_TRUE(confirmed.clear_retained_path);
  EXPECT_TRUE(confirmed.start_replan);
  EXPECT_EQ(fsm.state(), planner::PlanningState::kReplanning);
  EXPECT_EQ(fsm.replanAttemptCount(), 1U);
  EXPECT_EQ(fsm.lastReplanGeneration(), 21U);
}

TEST(PlanningFsm, ReplanSuccessReturnsToTracking)
{
  auto fsm = trackingFsm();
  (void)fsm.observePath(false, false, 20U, "unknown");
  (void)fsm.observePath(false, false, 21U, "unknown");
  fsm.onPlanSucceeded();
  EXPECT_EQ(fsm.state(), planner::PlanningState::kTracking);
  EXPECT_EQ(fsm.replanAttemptCount(), 0U);
}

TEST(PlanningFsm, ReplanFailureWaitsForRetry)
{
  auto fsm = trackingFsm();
  (void)fsm.observePath(false, false, 20U, "step_limit");
  (void)fsm.observePath(false, false, 21U, "step_limit");
  fsm.onPlanFailed(planner::PlanningFsm::Clock::time_point{});
  EXPECT_EQ(fsm.state(), planner::PlanningState::kWaitingRetry);
}

TEST(PlanningFsm, RetryRequiresBothPeriodAndNewMap)
{
  auto fsm = trackingFsm();
  (void)fsm.observePath(false, false, 20U, "out_of_bounds");
  (void)fsm.observePath(false, false, 21U, "out_of_bounds");
  const auto failed_at = planner::PlanningFsm::Clock::time_point{};
  fsm.onPlanFailed(failed_at);
  EXPECT_FALSE(fsm.retryReady(failed_at + 1s, 21U));
  EXPECT_FALSE(fsm.retryReady(failed_at + 100ms, 22U));
  EXPECT_TRUE(fsm.retryReady(failed_at + 500ms, 22U));
  EXPECT_EQ(fsm.state(), planner::PlanningState::kReplanning);
  EXPECT_EQ(fsm.replanAttemptCount(), 2U);
}

TEST(PlanningFsm, RetryBudgetExhaustionBlocks)
{
  planner::PlanningFsmParameters parameters;
  parameters.max_replan_attempts = 2U;
  auto fsm = trackingFsm(parameters);
  (void)fsm.observePath(false, false, 1U, "unknown");
  (void)fsm.observePath(false, false, 2U, "unknown");
  const auto start = planner::PlanningFsm::Clock::time_point{};
  fsm.onPlanFailed(start);
  ASSERT_TRUE(fsm.retryReady(start + 500ms, 3U));
  fsm.onPlanFailed(start + 500ms);
  EXPECT_EQ(fsm.state(), planner::PlanningState::kBlocked);
}

TEST(PlanningFsm, ExternalGoalSupersedesBlockedState)
{
  planner::PlanningFsm fsm;
  fsm.onMapReceived(false);
  fsm.onGoalReceived(true);
  fsm.onPlanFailed(planner::PlanningFsm::Clock::now());
  ASSERT_EQ(fsm.state(), planner::PlanningState::kBlocked);
  fsm.onGoalReceived(true);
  EXPECT_EQ(fsm.state(), planner::PlanningState::kPlanning);
  EXPECT_EQ(fsm.replanAttemptCount(), 0U);
}

TEST(PlanningFsm, NewGoalDiscardsVerificationState)
{
  auto fsm = trackingFsm();
  (void)fsm.observePath(false, false, 3U, "clearance_violation");
  ASSERT_EQ(fsm.state(), planner::PlanningState::kVerifyingPath);
  fsm.onGoalReceived(true);
  EXPECT_EQ(fsm.state(), planner::PlanningState::kPlanning);
  EXPECT_EQ(fsm.invalidStreak(), 0U);
  EXPECT_EQ(fsm.validStreak(), 0U);
}

TEST(PlanningFsm, InvalidInputBlocksWithoutVerification)
{
  auto fsm = trackingFsm();
  const auto decision = fsm.observePath(false, true, 4U, "invalid_input");
  EXPECT_EQ(fsm.state(), planner::PlanningState::kBlocked);
  EXPECT_TRUE(decision.suspend_motion);
  EXPECT_TRUE(decision.clear_retained_path);
  EXPECT_TRUE(decision.blocked);
  EXPECT_FALSE(decision.start_replan);
}

TEST(PlanningFsm, FrameChangeFullyResetsToIdleWhenNewMapExists)
{
  auto fsm = trackingFsm();
  (void)fsm.observePath(false, false, 5U, "unknown");
  fsm.onFrameChanged(true);
  EXPECT_EQ(fsm.state(), planner::PlanningState::kIdle);
  EXPECT_EQ(fsm.invalidStreak(), 0U);
  EXPECT_EQ(fsm.replanAttemptCount(), 0U);
}

TEST(PlanningFsm, TransitionRecordHasRequiredNamesAndReason)
{
  auto fsm = trackingFsm();
  (void)fsm.observePath(false, false, 9U, "clearance_violation");
  const auto transitions = fsm.takeTransitions();
  ASSERT_EQ(transitions.size(), 1U);
  EXPECT_EQ(planner::toString(transitions[0].from), "TRACKING");
  EXPECT_EQ(planner::toString(transitions[0].event), "PATH_SUSPECTED");
  EXPECT_EQ(transitions[0].reason, "clearance_violation");
  EXPECT_EQ(planner::toString(transitions[0].to), "VERIFYING_PATH");
}
