#include <gtest/gtest.h>

#include "rubi_heightmap_wavefront_planner/plan_lifecycle.hpp"

namespace rubi_heightmap_wavefront_planner
{
namespace
{

TEST(PlanLifecycle, ExternalGoalEpochRejectsOldResult)
{
  const PlanLifecycleToken old_result{4U, 214U, "map"};
  EXPECT_FALSE(mayCommitAfterRevalidation(old_result, 5U, "map"));
}

TEST(PlanLifecycle, LatestGenerationCanCommitAfterRevalidation)
{
  const PlanLifecycleToken candidate{4U, 214U, "map"};
  EXPECT_TRUE(mayCommitAfterRevalidation(candidate, 4U, "map"));
}

TEST(PlanLifecycle, MapGenerationBurstKeepsOnlyNewestAutomaticRequest)
{
  bool queued_automatic = true;
  EXPECT_TRUE(shouldReplaceQueuedRequest(queued_automatic, true));
  queued_automatic = true;
  EXPECT_FALSE(shouldReplaceQueuedRequest(false, true));
  EXPECT_TRUE(shouldReplaceQueuedRequest(false, false));
}

TEST(PlanLifecycle, OneInvalidEpisodeAcquiresAtMostOneAutomaticReplan)
{
  AutoReplanGate gate;
  EXPECT_TRUE(gate.tryAcquire());
  EXPECT_FALSE(gate.tryAcquire());
  gate.reset();
  EXPECT_TRUE(gate.tryAcquire());
}

}  // namespace
}  // namespace rubi_heightmap_wavefront_planner
