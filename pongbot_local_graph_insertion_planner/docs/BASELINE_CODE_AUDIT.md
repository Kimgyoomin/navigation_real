# Baseline code audit — Local Graph Insertion experiment

## 2026-07-28 implementation audit

Repository state at the start of this pass:

- branch: `rubi_simulation`
- commit: `fd2b49179f34984c678c52bbbdb731cad89bfaf4`
- pre-existing dirty state: nested `Livox-SDK2` only

The current plugin is
`pongbot_local_graph_insertion_planner/AstarLocalPlanner`. Its production
baseline is periodic fresh 8-connected global A* over an immutable fusion of:

```text
global StaticLayer + InflationLayer snapshot
  max known costs from the latest 8 m x 8 m local costmap
  -> fresh A* from current robot pose to final goal
  -> collision-checked SimpleSmoother BT action
  -> RPP
```

`local` describes the update footprint, not the search extent. No path splicing
or explicit per-cell graph objects are used.

### Confirmed implementation contracts

- `enable_incremental_reuse=false` calls `freshAstar()` directly.
- `enable_incremental_reuse=true` calls the existing D* Lite implementation.
- Global snapshot copying holds the Costmap2D mutex only during the char-map
  copy. Search reads only the private snapshot.
- Local unknown and free cells do not erase global costs. Known costs 1 through
  254 use max aggregation, so inflation is traversable but expensive and 253+
  remains blocked.
- Each plan starts from a new global base snapshot, so removed local obstacles
  do not accumulate.
- Cardinal/diagonal costs are `resolution` and `sqrt(2) * resolution`.
  `cost_penalty_scale` multiplies destination-cell normalized cost in the one
  shared edge-cost function used by fresh A* and D* Lite.
- Start and goal are transformed into the global costmap frame. Exact endpoint
  positions and normalized goal orientation are retained. A one-cell result
  returns both start and goal poses for rotate-to-heading.
- The planner publishes the exact fused snapshot as
  `/astar_local/fused_costmap_raw`. The BT invokes ComputePathToPose at 1 Hz and
  then Humble `SmoothPath` with `check_for_collisions=true`; SmootherServer uses
  that fused raw costmap before FollowPath.
- Runtime metrics use one `[AstarLocalMetrics]` record containing mode, time,
  expansions, changed/overlay cells, age, path size/length/cost, snapshot
  version, fallback, and failure.

### Parameter resolution

The previously dead `unknown_traversal_cost`, `enable_shortcutting`, and
`publish_debug_metrics` entries were removed. All remaining `AstarLocal`
parameters are declared, validated, and used. Latest-TF fallback is default-off.

### Known performance boundary

The optional D* implementation still scans the complete cost vector for change
ratio and affected-cell discovery. It is state reuse, not a dirty-region-only
pipeline. The 2026-07-28 synthetic 210,656-cell benchmark showed fewer expanded
nodes but worse latency than fresh A*, so no D* speedup is claimed.

## Historical read-only audit

Audit date: 2026-07-23 (read-only phase)

## Scope and repository state

Repository: `/home/kim/ros2_ws_nav/src/navigation_real`, branch `rubi_simulation`,
commit `4f4828fc4da3f403ac46b39331cf8314816f8d42`.

Before this audit the worktree was already dirty:

- `pongbot_navigation/config/global_costmap_params.yaml`
- `pongbot_navigation/config/nav2_rubi.yaml`
- nested `Livox-SDK2` has untracked `COLCON_IGNORE`

No existing file was changed by this audit. Protected-file Git index object IDs were
recorded before this document was created.

## Nav2 Humble contract

`/opt/ros/humble/include/nav2_core/global_planner.hpp:50-78` defines
`configure(parent, name, tf, costmap_ros)`, lifecycle cleanup/activate/deactivate,
and `createPlan(start, goal)`. The plugin receives PlannerServer's Costmap2DROS and
TF buffer.

The Humble PlannerServer source configures one global costmap, passes it to each
plugin, and invokes `createPlan()` only for ComputePath actions.
`expected_planner_frequency` is an overrun-warning threshold, not a periodic
replanning trigger. BT selection controls repeated ComputePathToPose calls.

Installed BT assets include `navigate_w_replanning_time.xml`,
`navigate_w_replanning_distance.xml`, and validity/goal-only variants under
`/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/`. The active
`nav2_rubi.yaml` does not select a BT XML, so its actual replan policy requires
runtime confirmation.

## Existing A* findings

| Finding | Evidence | Impact |
|---|---|---|
| It is 8-connected 2-D A*, not Hybrid A*. | `astar_planner.cpp:246-251`; no yaw state/motion primitives. `plugins.xml:6` says Hybrid A*. | Plugin description is inaccurate. |
| State is rebuilt per request. | `astar_planner.cpp:238-242` allocates scores, parents, closed, queue inside `createPlan`. | No reuse; 2663×2499 is about 6.65M cells before queue storage. |
| It reads live costmap cells during planning. | `astar_planner.cpp:95,165-178,233-236`. | A concurrent update can produce a mixed snapshot. |
| 253 is traversable; 254/255 are blocked. | `astar_planner.cpp:165-178`. | Different from the experiment's conservative >=253 rule. |
| Corner cutting is prevented. | `astar_planner.cpp:297-305`. | Positive baseline behaviour. |
| Edge cost ignores resolution. | `astar_planner.cpp:221-223,248-251`. | Costs are cells, not metres. |
| astar_params.yaml is unused. | `astar_params.yaml:1-15`; configure at `astar_planner.cpp:41-63` reads no parameters. | Existing YAML knobs have no effect. |
| Smoothing has no collision recheck. | `astar_planner.cpp:402-421`. | A valid grid path can become unsafe. |
| Goal orientation is discarded. | `astar_planner.cpp:427-440`. | Violates goal-orientation preservation. |
| Basic TF/bounds/occupied/empty handling exists. | `astar_planner.cpp:90-117,123-139,196-210,323-347`. | Start=goal and quaternion contracts still need tests. |

## Costmap, sensor, and TF baseline

The dirty active `pongbot_navigation/config/nav2_rubi.yaml:1-60` contains planner and
controller parameters only; it defines no global/local costmap or BT navigator settings.
`pongbot_navigation/launch/rubi_navigation.launch.py:61-122` passes exactly one
`params_file` to PlannerServer, ControllerServer, SmootherServer, BehaviorServer, and
BT Navigator; it does not merge the standalone costmap YAML files.

The related but unselected `nav2_rubi_fastlio_point.yaml:49-77` has only a
StaticLayer plus InflationLayer globally, and its local costmap at lines 97-118 has
InflationLayer only. The separate `local_costmap_params.yaml:16-29` uses an
ObstacleLayer on `/scan` as LaserScan, not `/livox/lidar`.

At audit time:

- `/livox/lidar` is `livox_ros_driver2/msg/CustomMsg`, not PointCloud2.
- `map`, `camera_init`, and `base_link` were absent from the active TF tree.

Therefore PointCloud2 VoxelLayer/ObstacleLayer YAML and runtime validation are
**BLOCKED**. The new package must not silently treat CustomMsg as PointCloud2.

## Isolated Phase-B design

New sibling package and plugin identity:

```text
package: pongbot_local_graph_insertion_planner
namespace: pongbot_local_graph_insertion_planner
class: LocalGraphInsertionPlanner
plugin: pongbot_local_graph_insertion_planner/LocalGraphInsertionPlanner
planner ID: LocalGraphInsertion
```

Core state is D* Lite over an implicit 8-neighbor grid:

```text
previous GridSnapshot + current GridSnapshot
  -> changed cells and incident edges
  -> UpdateVertex on affected vertices
  -> g/rhs/open/km D* Lite repair
  -> final collision-validated path
```

GridSnapshot must copy metadata and char-map data under `Costmap2D::getMutex()`,
then release the mutex before search. Goal/geometry changes, changed ratio above a
parameterized threshold, invariant failure, timeout, or extraction failure require a
fresh full-A* fallback on the same immutable snapshot.

Default contract: metric cardinal cost `resolution`, diagonal cost
`sqrt(2)*resolution`, no corner cutting, `cost >= 253` blocked, unknown blocked
unless namespaced `allow_unknown` permits it. Cost penalties must be non-negative and
identical in D* Lite and fresh-A* reference calculations.

The path layer must preserve valid exact start/goal poses and supplied goal orientation,
set finite normalized intermediate yaw toward the next segment, and validate all final
segments. Shortcutting is default-off; failed validation returns the raw path.

## Entry criteria and runtime checks

Do not implement runtime costmap integration until:

1. the selected observation source is confirmed PointCloud2;
2. `map -> camera_init -> base_link -> livox_frame` is valid;
3. a new experimental YAML (passed through existing `params_file`) defines matching
   obstacle-before-inflation layers and RPP collision detection;
4. pure-core incremental results match fresh A* under identical contracts.

Read-only runtime checks:

```bash
ros2 topic type /livox/lidar
ros2 node list
ros2 run tf2_ros tf2_echo map camera_init
ros2 run tf2_ros tf2_echo camera_init base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
```

## Limitations

This design replans on the latest 2-D costmap snapshot only. It does not predict moving
obstacles, generate controller-side detours, solve foothold feasibility, or provide 3-D
traversability. Performance claims require identical-event measurements against fresh
full A* and the existing planner.
