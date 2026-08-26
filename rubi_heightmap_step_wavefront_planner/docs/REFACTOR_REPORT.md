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
- The modular Phase-1 refactor is committed at
  `e5988cd3f997ea8fb45d78691df3112701c361b6e`. The FastDEM lattice fix is
  committed as `fdc025d`, followed by the standalone RViz/report commit.

## FastDEM lattice compatibility follow-up

The previous snapshot parser implicitly required `x=i*resolution` and
`y=j*resolution`. It now derives an order-independent `(min(x), min(y))` origin
and validates `value=origin+index*resolution`. Shifted, negative, sparse, and
shuffled fixtures are covered without changing resolution, tolerance, unknown,
or duplicate-cell behavior. RViz now restores the full standalone panels,
tools, view, QoS, elevation cloud, graph markers, and Path displays.

Validation on 2026-08-26: clean Release build PASS with no compiler warnings;
14/14 CTest targets and 38/38 xUnit cases PASS. Source and installed RViz files
resolve to the same symlink target and have no diff. A FastDEM publisher was
discovered with Reliable/Volatile QoS, but both the cloud and `/clock` produced
no samples during repeated timed checks. `map -> base_link` was also absent with
two disconnected TF trees. Consequently real FastDEM acceptance, live Goal
planning, and visual RViz confirmation are `BLOCKED_BY_TF` / `NOT VERIFIED`;
the synthetic ROS launch integration remains PASS.
