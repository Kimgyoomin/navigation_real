# RUBI Height-discontinuity Wavefront Planner

ROS 2 Humble standalone global route planner for indoor environments containing
flat floor and discrete, low height discontinuities. In this package, **step**
means the relative height discontinuity between adjacent heightmap cells. It
does not mean footstep, foothold, gait phase, leg kinematics, or foot placement.

The planner consumes a complete FastDEM 5 cm global heightmap snapshot,
`map <- base_link` TF, and a Goal pose. It builds a deterministic sampled
Node–Edge graph, runs A*, and publishes `nav_msgs/Path`. It is not a general
traversability planner, Nav2 plugin, local planner, controller, or `/cmd_vel`
source. It uses no PCA, slope, roughness, learning model, OccupancyGrid, RRT*,
path smoothing, unknown interpolation, or hole filling.

## Cost and hard-validity contract

Absolute global `z` is never a route cost. Only relative adjacent-cell height
changes are used:

```text
delta_i = abs(z_i - z_(i-1))
r_i = clamp((delta_i - h_noise) / (h_max - h_noise), 0, 1)
S_e = h_max * sum(r_i ^ p)
C_e = w_d * L_xy + w_h * S_e + w_c * S_clearance
```

Every crossable height event is accumulated; the maximum event alone is not
used as the soft cost. The XY distance is used rather than 3-D length, avoiding
double-counting vertical change. With the V0 defaults, one maximum-crossable
8 cm discontinuity adds `5.0 * 0.08 = 0.40 m`. Thus the planner prefers a flat
detour when avoiding that event costs less than 40 cm. The 8 cm limit is a
software bootstrap value, not a measured RUBI physical limit.

The following are hard-invalid and never converted to a large finite cost:

- unknown or out-of-bounds center cell;
- unknown or out-of-bounds cell inside `hard_clearance_radius_m`;
- an adjacent observed-cell jump above `max_crossable_height_jump_m` inside
  the clearance disk;
- an over-limit height transition along an edge;
- diagonal corner cutting through an unknown cell.

The A* heuristic is `w_d * EuclideanDistance(node, goal)`. Since height
penalties are nonnegative and every edge costs at least `w_d * L_xy`, this is
admissible. Ties are resolved by lower `f`, lower `g`, then lower Node ID.

Optional preferred-clearance risk is disabled by default (`w_c=0` and
`preferred_clearance_radius_m == hard_clearance_radius_m`). When enabled, it
adds a continuous nonnegative penalty between the hard and preferred radii;
terrain inside the hard radius remains rejected. Its hazard definition is
unknown/out-of-bounds terrain and adjacent-cell discontinuities over 8 cm.

## Heightmap snapshots

Only `x`, `y`, and `z` `FLOAT32 count=1` fields are consumed. Each PointCloud2
is a complete immutable snapshot. Missing lattice cells remain explicitly
unknown; disappearing cells are not restored from an older snapshot. Parsing
checks the unorganized row layout, endianness, finite coordinates, duplicates,
overflow, and `max_grid_cells`. The map may use an arbitrary world-frame
lattice origin. All observed points must remain consistent with the configured
5 cm spacing within `lattice_tolerance_m`; coordinates are never forced onto a
world-zero lattice.

The content hash follows canonical lattice-cell order, so input point ordering
does not create a new generation. For reproducible experiments, use a frozen
or conditioned FastDEM snapshot: fluctuating raw disappearing cells remain
visible to this planner by design.

`max_grid_cells` limits the dense bounding grid, not the resolution. At 5 cm,
5,000,000 cells form a square about 111.8 m on each side. Inspect the runtime
`grid=XxY` log before changing this safety cap.

## Deterministic graph and post-goal expansion

FIFO nodes produce 20 uniformly spaced proposals on a deterministic ring.
Merge candidates are sorted by `(distance_squared, NodeId)`, undirected edges
are unique, and only evaluator-accepted nodes and edges enter A*.

The first valid Goal connection is not an immediate stop. The builder completes
`post_goal_expansions` additional FIFO source expansions so a later flat detour
can exist before A* chooses between it and an earlier discontinuity route.
`post_goal_expansions: 0` restores first-solution behavior. A node, expansion,
or steady-clock time budget can end graph construction earlier; if a Goal path
already exists, that budget termination can still yield planning success.

## Map lifecycle

This package performs active-path safety revalidation and bounded online
global replanning on changed complete height-map snapshots. It does not
incrementally mutate a persistent graph, track dynamic objects, predict
obstacle motion, or re-optimize for cost-only map changes.

Only the unpassed active-Path corridor is rechecked on a changed same-frame
snapshot. Any terrain-derived invalid observation immediately publishes a
latched empty Path to suspend motion, but the internal Path is retained while
the FSM is `VERIFYING_PATH`. `path_invalid_confirmations` consecutive invalid
snapshots delete the retained Path and start fresh replanning. Conversely,
`path_recovery_confirmations` consecutive valid snapshots republish the retained
Path without rebuilding the graph. `kInvalidInput` bypasses verification and
enters `BLOCKED`.

Failed replanning enters `WAITING_RETRY`. Retry requires the configured period,
and by default a newer map generation, until `max_replan_attempts` is exhausted.
An external Goal supersedes every state and resets verification/retry counters.

**Map updates trigger safety revalidation, not cost-only route optimization.**
If hard validity remains intact, a changed height score alone does not trigger
automatic replanning. A frame-changing full reset publishes empty Path, all
three `DELETEALL` MarkerArrays, and deletes the revalidation marker.

## Timing

All core and node durations use `std::chrono::steady_clock`:

- `graph_build_time_ms`: start/Goal evaluation through final graph build;
- `astar_time_ms`: adjacency/A* start through path restoration;
- `core_total_time_ms`: core entry through completed `PlanResult`;
- `tf_time_ms`: TF lookup and Goal transformation;
- `postprocess_time_ms`: densification, messages, and publication;
- `total_planning_time_ms`: request processing through publication;
- `revalidation_time_ms`: active-corridor validation.

Logs also report request type, planned/validated generation, termination,
nodes, edges, expansions, Path pose count, XY length, height-event count,
maximum jump, accumulated height score, and total route cost. These measurements
are diagnostic; this package is not claimed real-time or robot-validated.

## Build and test

The exported consumer target is
`rubi_heightmap_step_wavefront_planner::rubi_heightmap_step_wavefront_planner_core`.

```bash
cd ~/ros2_ws_nav
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select rubi_heightmap_step_wavefront_planner \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
colcon test \
  --packages-select rubi_heightmap_step_wavefront_planner \
  --event-handlers console_direct+
colcon test-result --verbose
```

## Run

Robot/localization bring-up must provide the heightmap-frame to `base_link` TF.
The package does not publish a static robot transform.

```bash
source ~/ros2_ws_nav/install/setup.bash
ros2 launch rubi_heightmap_step_wavefront_planner \
  step_wavefront_v0.launch.py
```

Default topics:

| Direction | Topic | Type |
|---|---|---|
| Input | `/fastdem/mapping/cloud_global` | `sensor_msgs/msg/PointCloud2` |
| Input | `/goal_pose` | `geometry_msgs/msg/PoseStamped` |
| Output | `/rubi/heightmap_step_planner/path` | `nav_msgs/msg/Path` |
| Debug | `/rubi/heightmap_step_planner/debug/nodes` | `visualization_msgs/msg/MarkerArray` |
| Debug | `/rubi/heightmap_step_planner/debug/edges` | `visualization_msgs/msg/MarkerArray` |
| Debug | `/rubi/heightmap_step_planner/debug/rejected` | `visualization_msgs/msg/MarkerArray` |
| Debug | `/rubi/heightmap_step_planner/debug/revalidation_failure` | `visualization_msgs/msg/Marker` |

Cloud and Goal use reliable volatile QoS. Path and debug outputs use reliable,
transient-local, keep-last-one QoS. All output frames are the accepted map
frame. Path intermediate orientations follow segment tangent; the last pose
preserves the validated, normalized Goal orientation transformed into map.

RViz displays valid nodes/edges in green, the final Path in yellow,
unknown/out-of-bounds rejections in red, clearance rejections in orange, and
over-limit discontinuity rejections in magenta, and the currently failing
revalidation segment as a thick red line. Displays are top-level PointCloud2,
Path, Marker, and MarkerArray entries; no Group display is used.

## V0 limitations

This planner targets flat indoor floor mixed with discrete low discontinuities.
It does not establish robot safety, physical step capability, global optimality
outside its constructed graph, or real-time performance. Dynamic obstacle
stopping remains the responsibility of a controller/local safety layer.
