#pragma once

#include <cstdint>
#include <string>

namespace rubi_heightmap_wavefront_planner
{

struct PlanLifecycleToken
{
  std::uint64_t goal_epoch{0U};
  std::uint64_t map_generation{0U};
  std::string map_frame;
};

/** A changed generation is allowed after post-revalidation; a new Goal/frame is not. */
bool mayCommitAfterRevalidation(
  const PlanLifecycleToken & token,
  std::uint64_t current_goal_epoch,
  const std::string & current_map_frame) noexcept;

/** An external request always supersedes an automatic request. */
bool shouldReplaceQueuedRequest(
  bool queued_request_is_automatic,
  bool incoming_request_is_automatic) noexcept;

/** One automatic retry is permitted for each active-path invalidation episode. */
class AutoReplanGate
{
public:
  bool tryAcquire() noexcept;
  void reset() noexcept;
  bool acquired() const noexcept;

private:
  bool acquired_{false};
};

}  // namespace rubi_heightmap_wavefront_planner
