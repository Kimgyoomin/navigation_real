# RUBI Heightmap Sampling Planner

ROS 2 Humble standalone sampling-planner baseline for the FastDEM global
elevation map. The package contains both a TRG-inspired wavefront roadmap and
a conventional RRT* core selected by YAML. The bundled `wavefront_v0.yaml`
profile is the 5 cm Wavefront demo described below.

```text
/fastdem/mapping/cloud_global  sensor_msgs/PointCloud2 (x, y, z=elevation)
          │
          ▼
strict regular TerrainSnapshot (shared)
          │
          ├─ local PCA: slope / roughness
          ├─ exact observed support: unknown hard reject
          └─ sampled edge: consecutive height-step hard reject
          │
          ▼
wavefront roadmap + A*  OR  RRT* choose-parent + rewire
          │
          ├─ /rubi/heightmap_planner/path
          └─ RViz node / edge / rejected markers
```

The wavefront planner is an independent implementation of the high-level
wavefront-roadmap idea. It does not copy TRG source code.

## Scope

V0 consumes the regular 2.5-D observed elevation cells (`x`, `y`, `z`) from
FastDEM, not raw LiDAR. FastDEM's optional `slope` or `step` fields are not
required.

- PCA over the local elevation neighborhood estimates the plane normal, slope,
  and roughness around every checked pose.
- A step is **not** inferred from PCA.
- Every edge is sampled at `edge_check_spacing_m`; consecutive elevations are
  compared and the edge is rejected when `abs(delta_z) > max_step_height_m`.
- Missing cells are unknown and are never filled by nearest-neighbor lookup.
- Excessive slope and step are hard-invalid. Feasible slope is also a soft edge
  cost.

The planner currently outputs `nav_msgs/Path` as a standalone node. It is not a
`nav2_core::GlobalPlanner` plugin, is not connected to a Nav2 costmap or
controller, and does not generate `cmd_vel`.

### Wavefront sampling versus RRT*

These controls are intentionally different:

| Mode | Sampling contract | Main YAML controls |
|---|---|---|
| `wavefront` | expand each queued node with \(n\) points on a ring | `samples_per_expansion`, `node_sampling_distance_m` |
| `rrt_star` | draw one global XY sample per iteration | `rrt_star.max_iterations`, `rrt_star.goal_bias`, `rrt_star.steer_distance_m` |

Therefore “\(n\) samples per node” is the wavefront mode, not standard RRT*.
RRT* performs `nearest → steer → choose-parent → insert → rewire`, and updates
all descendant costs after rewiring.

## Wavefront V0 profile

The checked-in `config/wavefront_v0.yaml` profile fixes the demo contract:

| Control | V0 value |
|---|---:|
| FastDEM/elevation lattice | 0.05 m |
| Lattice tolerance | 0.01 m |
| Edge terrain-check spacing | 0.025 m |
| Wavefront sampling radius | 0.30 m |
| Proposal directions per expansion | 20 |
| Node merge radius | 0.20 m |
| Neighbor connection radius | 0.45 m |
| Goal connection distance | 0.45 m |
| Node / expansion budget | 4000 / 4000 |
| Graph build-time budget | 5000 ms |
| Path output spacing | 0.05 m |

`map_resolution_m` tells the consumer how to interpret the incoming lattice.
The planner does **not** resample a coarse cloud or convert it into a 5 cm map.
The running FastDEM producer must itself be configured for 0.05 m resolution;
points from a 10 cm producer may happen to lie on a 5 cm lattice and therefore
are not proof of a 5 cm producer.

The 20 directions are 20 proposals evaluated around each expanded node, not 20
guaranteed accepted graph nodes. Terrain gates can reject a proposal and
`merge_radius_m` can merge it with an existing node. At a 0.30 m ring radius,
adjacent proposals are only about 0.094 m apart.

## Build

Copy the package to the elevation workspace:

```bash
cp -a rubi_heightmap_wavefront_planner \
  ~/grid_map_ws/src/

cd ~/grid_map_ws
rosdep install \
  --from-paths src \
  --ignore-src \
  -r -y

colcon build \
  --symlink-install \
  --packages-select rubi_heightmap_wavefront_planner \
  --cmake-args -DBUILD_TESTING=ON

source install/setup.bash
```

Run the tests:

```bash
colcon test \
  --packages-select rubi_heightmap_wavefront_planner \
  --event-handlers console_direct+

colcon test-result --verbose
```

Run the non-gating benchmark from the CMake build tree:

```bash
./build/rubi_heightmap_wavefront_planner/benchmark_flat_map
```

The benchmark reports the configured physical scenario, map-cell count, graph
size, termination, build/total time, and path metrics. Wall-clock time is
reported rather than used as a CTest pass/fail threshold.

The ROS-independent smoke test can also be compiled directly:

```bash
cd ~/grid_map_ws/src/rubi_heightmap_wavefront_planner

g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  -Iinclude \
  src/terrain_snapshot.cpp \
  src/terrain_evaluator.cpp \
  src/wavefront_planner.cpp \
  src/rrt_star_planner.cpp \
  test/core_smoke.cpp \
  -o /tmp/rubi_wavefront_core_smoke

/tmp/rubi_wavefront_core_smoke
```

## Confirm the live FastDEM contract

Do not start tuning before confirming the producer's source configuration and
the actual message:

```bash
ros2 param get /fastdem config_file
ros2 topic info -v /fastdem/mapping/cloud_global

ros2 topic echo \
  /fastdem/mapping/cloud_global \
  sensor_msgs/msg/PointCloud2 \
  --once --field header

ros2 topic echo \
  /fastdem/mapping/cloud_global \
  sensor_msgs/msg/PointCloud2 \
  --once --field fields
```

Required fields are `x`, `y`, and `z`, all `FLOAT32`. The message frame is used
as the planning frame, so TF must connect it to `base_frame` and RViz's fixed
frame. Follow the reported FastDEM config path back to its source-workspace
file—not an installed copy—and verify `map.resolution: 0.05`. The planner's
`map_resolution_m: 0.05` must match it; the parser rejects points that do not
fit that lattice, but lattice alignment alone cannot prove producer density.

Also check that both the observed point count and the dense bounding lattice
fit `max_grid_cells`. A 200 m by 200 m region at 5 cm is approximately 16
million cells, above the V0 limit of 5 million cells.

## Run

Start FastDEM first, then:

```bash
# TRG-inspired wavefront
ros2 launch rubi_heightmap_wavefront_planner wavefront_v0.launch.py

# True RRT*
ros2 launch rubi_heightmap_wavefront_planner rrt_star_v0.launch.py
```

For a robot whose base TF is `base_link` rather than `body`:

```bash
ros2 launch rubi_heightmap_wavefront_planner wavefront_v0.launch.py \
  launch_rviz:=true \
  base_frame:=base_link
```

Normally the RViz `2D Goal Pose` tool publishes `/goal_pose`. A terminal example
is:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: map
pose:
  position: {x: 8.0, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"
```

The node obtains the start from:

```text
terrain message frame <- base_frame
```

through TF.

### Goal and map lifecycle

A goal received before the first accepted map is held as the pending goal and
is processed after that map arrives. Once a snapshot has been published:

- An identical map content hash keeps the existing Path and Marker snapshots.
- A materially changed map clears the existing Path and all Markers.
- V0 does not automatically replan after a changed map. Wait for the map to
  stabilize and send the goal again.

Automatic map-update replanning requires cancellation, request epochs, stale
result checks, and work coalescing and is intentionally deferred.

### Planning-result snapshots

The output contract distinguishes a core planning result from failures that
occur before a valid result exists:

| Outcome | Path | Accepted graph | Rejections |
|---|---|---|---|
| Success | non-empty | complete accepted nodes and edges | shown up to the display cap |
| Core failure with `PlanResult` | empty, clearing the previous Path | returned partial graph, if any | returned diagnostics up to the cap |
| Invalid start/goal | empty | may be empty | available diagnostics |
| TF, malformed goal, or exception | empty | cleared | cleared |

For core success and failure, the Path, accepted nodes, accepted edges/final
Path Marker, and rejection snapshot come from the same accepted map state, use
one timestamp, and are published in that fixed order. ROS publishers are not an
atomic aggregate; a state/generation check immediately before publication
prevents a result from a superseded map from being published.

## ROS interface

| Direction | Topic | Type |
|---|---|---|
| Input | `/fastdem/mapping/cloud_global` | `sensor_msgs/msg/PointCloud2` |
| Input | `/goal_pose` | `geometry_msgs/msg/PoseStamped` |
| Output | `/rubi/heightmap_planner/path` | `nav_msgs/msg/Path` |
| Debug | `/rubi/heightmap_planner/debug/nodes` | `visualization_msgs/msg/MarkerArray` |
| Debug | `/rubi/heightmap_planner/debug/edges` | `visualization_msgs/msg/MarkerArray` |
| Debug | `/rubi/heightmap_planner/debug/rejected` | `visualization_msgs/msg/MarkerArray` |

All topic names and the base frame are parameters. The Path and three debug
outputs use reliable, transient-local QoS so a late subscriber receives the
latest snapshot.

On success, each `nav_msgs/Path` pose contains graph/densified `x` and `y`, the
elevation lattice value in `z`, and a normalized yaw quaternion tangent to the
next path segment (the last pose keeps the final segment direction). The Path
uses the map snapshot frame and one common timestamp. A core planning failure
publishes an empty Path to remove any previous successful Path.

RViz legend:

- FastDEM elevation: blue
- start / goal semantic endpoints: cyan / magenta
- accepted sampled nodes: green
- accepted valid edges: green
- `kNodeInvalid` proposal: red sphere
- terrain-invalid edge proposal: red line from source to candidate
- non-finite evaluation: diagnostic point
- final Path Marker and RViz `nav_msgs/Path`: yellow

An edge-invalid candidate is not necessarily an invalid node, so it is not also
drawn as a red sphere. Duplicate-edge suppression is not a terrain failure and
is not drawn as a red invalid line. Accepted graph nodes/edges are displayed in
full. Rejected attempts are capped by `max_rejected_markers` (5000 in V0);
summary logs report `rejected_shown` and `rejected_total`, and truncation is
reported explicitly. Snapshot geometry remains batched (`SPHERE_LIST` for
nodes and `LINE_LIST` for edges) rather than creating one Marker per entity.

The bundled RViz fixed frame is `map`. If FastDEM publishes another frame such
as `camera_init`, change RViz's Fixed Frame to that message frame or provide the
corresponding TF.

## Important parameters

These are values in `wavefront_v0.yaml`, not necessarily compiled defaults.
Terrain thresholds are software bootstrap values and are **not** validated
physical limits of RUBI.

| Parameter | Wavefront V0 | Meaning |
|---|---:|---|
| `planner_mode` | `wavefront` | `wavefront` or `rrt_star` |
| `map_resolution_m` | 0.05 m | consumer lattice; must equal the FastDEM producer resolution |
| `lattice_tolerance_m` | 0.01 m | maximum point-to-lattice alignment error |
| `pca_analysis_radius_m` | 0.30 m | local PCA neighborhood |
| `support_radius_m` | 0.20 m | circular observed-support check |
| `minimum_observed_support_ratio` | 1.00 | minimum observed fraction; strict unknown rejection |
| `max_slope_deg` | 15 deg | software bootstrap hard gate, not a measured RUBI limit |
| `max_step_height_m` | 0.08 m | consecutive edge-sample height gate |
| `edge_check_spacing_m` | 0.025 m | half-cell terrain-check spacing |
| `node_sampling_distance_m` | 0.30 m | wavefront proposal-ring radius |
| `samples_per_expansion` | 20 | proposal directions, not accepted-node count |
| `merge_radius_m` | 0.20 m | suppress duplicate graph nodes |
| `neighbor_connection_radius_m` | 0.45 m | valid loop-edge radius |
| `goal_connection_distance_m` | 0.45 m | actual-goal connection radius |
| `max_nodes` | 4000 | graph node budget |
| `max_expansions` | 4000 | expanded-reference budget |
| `max_build_time_ms` | 5000 ms | graph build wall-time budget |
| `path_output_spacing_m` | 0.05 m | maximum densified Path segment spacing |
| `max_rejected_markers` | 5000 | displayed rejection-attempt cap |

The RRT* controls below are present in the same YAML but inactive while
`planner_mode=wavefront`:

| Parameter | YAML value | Meaning |
|---|---:|---|
| `rrt_star.max_iterations` | 5000 | global random-sample budget |
| `rrt_star.goal_bias` | 0.05 | probability of sampling the exact goal |
| `rrt_star.steer_distance_m` | 0.50 m | maximum nearest-to-new extension |
| `rrt_star.rewire_radius_min_m` | 0.30 m | lower rewire-radius clamp |
| `rrt_star.rewire_radius_max_m` | 1.00 m | upper rewire-radius clamp |
| `rrt_star.goal_connection_distance_m` | 0.75 m | actual-goal connection radius |
| `rrt_star.max_nodes` | 4000 | RRT* tree-node budget, including start/goal |
| `rrt_star.max_planning_time_ms` | 2000 ms | wall-clock budget; `0` disables it |
| `rrt_star.stop_on_first_solution` | false | keep optimizing after first connection |
| `rrt_star.random_seed` | 42 | deterministic random sequence |

For reproducible comparisons, use a fixed `rrt_star.random_seed`, set
`rrt_star.max_planning_time_ms: 0`, and terminate on fixed iteration/node
budgets. Wall-clock termination can produce different trees under different CPU
loads.

`stop_when_goal_connected: true` makes wavefront return the first feasible
constructed graph. It is the fast V0 setting, not a global risk-optimality
guarantee. Use `false` with a fixed graph budget for planner comparisons.

The 15° slope gate comes from local elevation-neighborhood PCA, not a FastDEM
precomputed slope field. It is only a software bootstrap threshold. Before
claiming robot feasibility, physical experiments must measure at least:

- maximum reliable step-up and step-down separately;
- forward slope versus cross-slope limits;
- support/corridor radius;
- observed-support ratio near real map boundaries.

## Runtime verification

After sourcing the workspace, launch without RViz and inspect the effective
profile:

```bash
source ~/grid_map_ws/install/setup.bash

ros2 launch rubi_heightmap_wavefront_planner wavefront_v0.launch.py \
  launch_rviz:=false
```

```bash
for parameter in \
  map_resolution_m \
  edge_check_spacing_m \
  node_sampling_distance_m \
  samples_per_expansion \
  merge_radius_m \
  neighbor_connection_radius_m \
  goal_connection_distance_m \
  max_build_time_ms
do
  ros2 param get /rubi_heightmap_wavefront_planner "${parameter}"
done
```

Inspect endpoint types and QoS, then observe one result:

```bash
ros2 topic info -v /fastdem/mapping/cloud_global
ros2 topic info -v /rubi/heightmap_planner/path
ros2 topic info -v /rubi/heightmap_planner/debug/nodes
ros2 topic info -v /rubi/heightmap_planner/debug/edges
ros2 topic info -v /rubi/heightmap_planner/debug/rejected

ros2 topic echo \
  /rubi/heightmap_planner/path \
  nav_msgs/msg/Path \
  --once
```

A successful real-map check requires a non-empty finite Path in the map
snapshot frame, normalized quaternions, green accepted graph, red rejected
nodes/terrain edges, and yellow Path. A failure check must show an empty Path;
a barrier/budget failure keeps any returned partial graph, while invalid
start/goal may legitimately have no accepted graph. Synthetic tests do not
replace verification of the real 5 cm FastDEM source config, cloud, and TF.

## Validation already represented in tests

- strict sparse-point to regular-lattice reconstruction;
- off-lattice, duplicate, and non-finite point rejection;
- no nearest-neighbor fallback across unknown cells;
- analytic plane slope and near-zero roughness from PCA;
- incomplete footprint rejection;
- unknown-hole edge rejection;
- explicit height-step rejection;
- excessive PCA-slope rejection;
- deterministic FIFO wavefront construction;
- merge and loop edges;
- goal connection to the actual goal;
- risk-aware A*;
- node, expansion, and build-time budgets;
- exact 0.05 m config and half-cell edge-spacing contract;
- 0.30 m / 20-direction proposal evaluation without assuming 20 accepted nodes;
- failed-plan partial graph and invalid start/goal behavior;
- accepted/rejected/final-Path Marker colors, edge semantics, cap, and bounds
  safety;
- fixed-seed deterministic RRT* construction;
- RRT* choose-parent, observed rewiring, and descendant-cost propagation;
- final RRT* parent tree contains no rejected terrain edge;
- additional RRT* iterations do not increase the best retained path cost;
- exact `start == goal` and half-cell map-boundary goal handling.

`planner_node.cpp` performs one more terrain revalidation of every selected graph
edge before publishing the densified path.

## V0 limitations

This profile is a standalone immutable-snapshot demonstration. It does not
combine an occupancy grid, Nav2 costmaps, inflation, or dynamic obstacles; it
is not a Nav2 planner plugin and has no local planner or controller. It also
does not animate every expansion, persist a global graph, perform local graph
repair, or automatically replan after a map update.

Heading state, directional step-up/down limits, foothold feasibility, and a
roughness soft cost remain future work. The current terrain gates and timing
results must not be described as physically RUBI-valid, safe, optimal, or
real-time without corresponding robot experiments.

Map-update auto-replanning is a separate second-stage feature. It needs a
worker/coalescing design, request epochs or cancellation, and stale-result
validation; calling the full planner directly from the cloud callback is not
the V0 contract.

## References

- [FastDEM global PointCloud2 publisher](https://github.com/Ikhyeon-Cho/FastDEM/blob/f97b404af5decb8d41b09c343bde96fe5ec4e53f/ros2/src/fastdem_ros_node.cpp)
- [FastDEM ElevationMap to PointCloud2 conversion](https://github.com/Ikhyeon-Cho/FastDEM/blob/f97b404af5decb8d41b09c343bde96fe5ec4e53f/fastdem/include/fastdem/bridge/ros/impl.hpp)
- [TRG paper: wavefront graph construction](https://arxiv.org/html/2501.01806v1#S3.SS2)
- [RRT* paper](https://arxiv.org/abs/1105.1186)
- [ROS 2 Humble PointCloud2 iterator API](https://docs.ros.org/en/humble/p/sensor_msgs/generated/classsensor__msgs_1_1PointCloud2Iterator.html)
- [ROS 2 Humble RViz marker types](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html)
