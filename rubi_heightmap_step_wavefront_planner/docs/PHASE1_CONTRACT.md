# Phase 1 Contract

- Input is a complete immutable 5 cm global elevation snapshot.
- A “step” is an adjacent-cell height discontinuity, not a footstep or gait event.
- Unknown, bounds, clearance support, over-limit discontinuity, diagonal corner,
  and numeric failures are hard invalid. Crossable discontinuities and optional
  preferred-clearance risk are soft nonnegative costs.
- The 8 cm `max_crossable_height_jump_m` default is a software threshold, not a
  measured RUBI hardware limit.
- Every planning request builds a fresh deterministic sparse graph. No persistent
  graph is incrementally updated.
- Changed maps revalidate only the remaining active path. Hard invalidation is
  immediate; soft unknown/support failures use confirmation count. Cost-only
  changes retain the path and do not optimize it again.
- At most one automatic fresh replan follows invalidation. External goals win.
- There is no dynamic-object tracking or prediction.
- This is a standalone global planner. Nav2 and controller behavior are outside
  this package's Phase 1 scope.
