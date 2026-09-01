# Phase 1 Contract

- Input is a complete immutable 5 cm global elevation snapshot. Its world-frame
  lattice origin may be arbitrary; observed coordinates must share the configured
  spacing within `lattice_tolerance_m`.
- A “step” is an adjacent-cell height discontinuity, not a footstep or gait event.
- Unknown, bounds, clearance support, over-limit discontinuity, diagonal corner,
  and numeric failures are hard invalid. Crossable discontinuities and optional
  preferred-clearance risk are soft nonnegative costs.
- The 8 cm `max_crossable_height_jump_m` default is a software threshold, not a
  measured RUBI hardware limit.
- Every planning request builds a fresh deterministic sparse graph. No persistent
  graph is incrementally updated.
- Exact-valid Start/Goal positions remain unchanged. Optional snapping searches
  only a fixed metric radius and accepts only full `StepEvaluator`-valid lattice
  centers, with deterministic grid-coordinate tie-breaking. It does not make
  unknown terrain traversable and does not alter Goal orientation.
- Changed maps revalidate only the remaining active path. Every terrain-derived
  invalid result suspends motion immediately; deletion/replanning requires the
  configured consecutive-invalid confirmation count. Consecutive valid maps can
  recover and republish the retained Path. Cost-only changes retain the path.
- Automatic fresh replanning has a bounded retry budget, period gate, and an
  optional newer-map requirement. External goals always supersede it.
- There is no dynamic-object tracking or prediction.
- This is a standalone global planner. Nav2 and controller behavior are outside
  this package's Phase 1 scope.
- The optional comparison executable treats master-costmap 253/254/255 as hard
  invalid and 1..252 as nonnegative inflation risk. Missing bounded height
  evidence remains hard invalid. It compares representation/neighbor creation,
  not different safety or cost contracts, and never commands the robot.
