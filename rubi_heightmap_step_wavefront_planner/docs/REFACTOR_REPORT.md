# Phase 1 Refactor Report

## Baseline

- Commit: `6b95e2d017b77a9125ca49b2f5c1778bf61a63e2`
- Branch created: `refactor/step-wavefront-phase1`
- Baseline build: PASS, 14.6 s
- Baseline CTest: 10/10 PASS, 0 failures, 2.77 s

## File mapping

The monolithic planner algorithm moved from `src/step_wavefront_planner.cpp` to
`graph/wavefront_graph_builder.cpp`, `search/astar_search.cpp`, and the small
`planning/step_wavefront_planner.cpp` orchestrator. PointCloud2 parsing moved
from `planner_node.cpp` to `ros/pointcloud2_heightmap_adapter.cpp`. Existing
top-level headers remain compatibility forwarding headers.

The builder now uses a deterministic uniform-grid spatial hash instead of
repeated all-node merge/neighbor scans. Query membership remains exact and is
sorted by squared distance then NodeId.

## Behavior

Existing hard-validity, accumulated height-risk, deterministic A*, post-goal
expansion, remaining-path revalidation, and map-frame reset contracts are
unchanged. Preferred-clearance risk is new and opt-in; its zero-weight default
is behavior-compatible.

## Dependency audit

No reference to the legacy wavefront package, navigation integration package,
controller plugins, or local planners is present. ROS launch runtime dependencies are explicit. The core
is PIC, standard-C++ only, installed and exported for downstream consumers.

## Known limits

No RUBI URDF/SDF collision geometry exists in this repository, so robot radius
and whether 0.20 m is physically sufficient are NOT VERIFIED. Dynamic object
prediction, persistent graph mutation, Nav2 integration, and controller tuning
remain out of scope.

## Synthetic clearance sweep

On the frozen 5 cm finite-step fixture, hard radii 0.20/0.25/0.30/0.35 m all
remained connected. Results `(path length m, minimum clearance m, graph ms)`
were respectively `(2.432, 0.200, 82.081)`, `(2.683, 0.250, 120.783)`,
`(2.501, 0.300, 162.112)`, and `(2.642, 0.350, 164.033)`. No passage-closing
threshold occurred through 0.35 m on this synthetic map; this is not a physical
robot clearance validation.

## Final validation

- Clean workspace build: PASS, 15.2 s, no compiler warnings reported.
- Workspace package CTest: 14/14 targets PASS, 4.46 s.
- Workspace xUnit: 33 tests, 0 errors, 0 failures, 0 skipped.
- Isolated package build/test: PASS; 14/14 targets and 33 xUnit tests PASS.
- Determinism: 20 identical builds matched node IDs/coordinates, edge pairs,
  path IDs, and total cost.
- Dependency string audit: 0 forbidden references.
- `git diff --check`: PASS.
- RUBI collision radius: NOT VERIFIED because no URDF/SDF collision model was
  found under the repository source tree.
- Changes remain uncommitted on `refactor/step-wavefront-phase1`; baseline HEAD
  is still the recorded commit above.
