#include <gtest/gtest.h>

#include "rubi_heightmap_step_wavefront_planner/plan_lifecycle.hpp"

namespace planner = rubi_heightmap_step_wavefront_planner;

TEST(PlanLifecycle, SoftFailureRecoversOrConfirmsAndStepIsImmediate)
{
  planner::SoftFailureTracker tracker;
  EXPECT_FALSE(tracker.observe(planner::StepInvalidReason::kUnknown, 2U));
  tracker.reset();
  EXPECT_FALSE(tracker.observe(planner::StepInvalidReason::kUnknown, 2U));
  EXPECT_TRUE(tracker.observe(planner::StepInvalidReason::kUnknown, 2U));
  tracker.reset();
  EXPECT_TRUE(tracker.observe(planner::StepInvalidReason::kStepLimit, 2U));
  EXPECT_TRUE(tracker.observe(planner::StepInvalidReason::kClearanceViolation, 2U));
}

TEST(PlanLifecycle, StaleGoalMapAndFrameCannotPublish)
{
  const planner::PlanLifecycleToken token{2U, 3U, "map"};
  EXPECT_TRUE(planner::mayPublish(token, 2U, 3U, "map", false));
  EXPECT_TRUE(planner::mayPublish(token, 2U, 4U, "map", true));
  EXPECT_FALSE(planner::mayPublish(token, 3U, 3U, "map", false));
  EXPECT_FALSE(planner::mayPublish(token, 2U, 4U, "other", true));
  EXPECT_FALSE(planner::mayPublish(token, 2U, 4U, "map", false));
}
