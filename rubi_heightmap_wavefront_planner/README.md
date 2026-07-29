# RUBI Heightmap Sampling Planner

ROS 2 Humble standalone sampling-planner baseline for the FastDEM global
elevation map. The package contains both a TRG-inspired wavefront roadmap and
a conventional RRT* core selected by YAML.

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

V0 deliberately uses only the FastDEM elevation cells (`x`, `y`, `z`).
FastDEM's optional `slope` or `step` fields are not required.

- PCA estimates the local plane slope and roughness around every checked pose.
- A step is **not** inferred from PCA.
- Every edge is sampled at `edge_check_spacing_m`; consecutive elevations are
  compared and the edge is rejected when `abs(delta_z) > max_step_height_m`.
- Missing cells are unknown and are never filled by nearest-neighbor lookup.
- Excessive slope and step are hard-invalid. Feasible slope is also a soft edge
  cost.

The planner currently outputs `nav_msgs/Path` as a standalone node. It does not
yet implement a `nav2_core::GlobalPlanner` plugin and it does not generate
`cmd_vel`.

### Wavefront sampling versus RRT*

These controls are intentionally different:

| Mode | Sampling contract | Main YAML controls |
|---|---|---|
| `wavefront` | expand each queued node with \(n\) points on a ring | `samples_per_expansion`, `node_sampling_distance_m` |
| `rrt_star` | draw one global XY sample per iteration | `rrt_star.max_iterations`, `rrt_star.goal_bias`, `rrt_star.steer_distance_m` |

Therefore “\(n\) samples per node” is the wavefront mode, not standard RRT*.
RRT* performs `nearest → steer → choose-parent → insert → rewire`, and updates
all descendant costs after rewiring.

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
  --packages-select rubi_heightmap_wavefront_planner

source install/setup.bash
```

Run the tests:

```bash
colcon test \
  --packages-select rubi_heightmap_wavefront_planner

colcon test-result --verbose
```

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

Do not start tuning before confirming the actual message:

```bash
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
as the planning frame. The configured `map_resolution_m` must match the running
FastDEM configuration; the parser rejects points that do not fit that lattice.

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
ros2 launch rubi_heightmap_wavefront_planner rrt_star_v0.launch.py \
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

## ROS interface

| Direction | Topic | Type |
|---|---|---|
| Input | `/fastdem/mapping/cloud_global` | `sensor_msgs/msg/PointCloud2` |
| Input | `/goal_pose` | `geometry_msgs/msg/PoseStamped` |
| Output | `/rubi/heightmap_planner/path` | `nav_msgs/msg/Path` |
| Debug | `/rubi/heightmap_planner/debug/nodes` | `visualization_msgs/msg/MarkerArray` |
| Debug | `/rubi/heightmap_planner/debug/edges` | `visualization_msgs/msg/MarkerArray` |
| Debug | `/rubi/heightmap_planner/debug/rejected` | `visualization_msgs/msg/MarkerArray` |

All topic names and the base frame are parameters.

RViz colors:

- FastDEM elevation: blue
- start / goal / sampled nodes: cyan / purple / green
- valid edge: white to red as slope risk rises
- unknown / support / slope / step reject: gray / orange / red / magenta
- final path: thick green

The bundled RViz fixed frame is `map`. If FastDEM publishes another frame such
as `camera_init`, change RViz's Fixed Frame to that message frame or provide the
corresponding TF.

## Important parameters

The default numerical limits are software smoke-test values. They are **not**
validated physical limits of RUBI.

| Parameter | Default | Meaning |
|---|---:|---|
| `planner_mode` | `wavefront` | `wavefront` or `rrt_star` |
| `map_resolution_m` | 0.10 m | FastDEM elevation lattice resolution |
| `pca_analysis_radius_m` | 0.30 m | local PCA neighborhood |
| `support_radius_m` | 0.20 m | circular observed-support check |
| `minimum_observed_support_ratio` | 1.00 | minimum observed fraction; strict unknown rejection |
| `max_slope_deg` | 15 deg | hard slope gate |
| `max_step_height_m` | 0.08 m | consecutive edge-sample height gate |
| `edge_check_spacing_m` | 0.05 m | must be no larger than about half a map cell |
| `node_sampling_distance_m` | 0.50 m | wavefront expansion radius |
| `samples_per_expansion` | 12 | samples on every expansion ring |
| `merge_radius_m` | 0.25 m | suppress duplicate graph nodes |
| `neighbor_connection_radius_m` | 0.75 m | valid loop-edge radius |
| `goal_connection_distance_m` | 0.75 m | actual-goal connection radius |
| `max_nodes` | 4000 | graph node budget |
| `max_expansions` | 4000 | expanded-reference budget |
| `max_build_time_ms` | 2000 ms | graph build wall-time budget |
| `rrt_star.max_iterations` | 5000 | global random-sample budget |
| `rrt_star.goal_bias` | 0.05 | probability of sampling the exact goal |
| `rrt_star.steer_distance_m` | 0.50 m | maximum nearest-to-new extension |
| `rrt_star.rewire_radius_min_m` | 0.30 m | lower rewire-radius clamp |
| `rrt_star.rewire_radius_max_m` | 1.00 m | upper rewire-radius clamp |
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

The next robot experiment must measure at least:

- maximum reliable step-up and step-down separately;
- forward slope versus cross-slope limits;
- support/corridor radius;
- observed-support ratio near real map boundaries.

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
- node, expansion, and build-time budgets.
- fixed-seed deterministic RRT* construction;
- RRT* choose-parent, observed rewiring, and descendant-cost propagation;
- final RRT* parent tree contains no rejected terrain edge;
- additional RRT* iterations do not increase the best retained path cost.
- exact `start == goal` and half-cell map-boundary goal handling.

`planner_node.cpp` performs one more terrain revalidation of every selected graph
edge before publishing the densified path.

## Next integration step

After the standalone topic and RViz acceptance tests pass on the real map:

1. keep `TerrainSnapshot`, `TerrainEvaluator`, `WavefrontPlanner`, and
   `RrtStarPlanner` unchanged;
2. add a thin `nav2_core::GlobalPlanner` wrapper;
3. return the same `nav_msgs/Path` to Nav2 `ComputePathToPose`;
4. compare Grid A*, wavefront, and RRT* on the same immutable terrain map;
5. then add heading, directional step-up/down, and foothold feasibility.

## References

- [FastDEM global PointCloud2 publisher](https://github.com/Ikhyeon-Cho/FastDEM/blob/f97b404af5decb8d41b09c343bde96fe5ec4e53f/ros2/src/fastdem_ros_node.cpp)
- [FastDEM ElevationMap to PointCloud2 conversion](https://github.com/Ikhyeon-Cho/FastDEM/blob/f97b404af5decb8d41b09c343bde96fe5ec4e53f/fastdem/include/fastdem/bridge/ros/impl.hpp)
- [TRG paper: wavefront graph construction](https://arxiv.org/html/2501.01806v1#S3.SS2)
- [RRT* paper](https://arxiv.org/abs/1105.1186)
- [ROS 2 Humble PointCloud2 iterator API](https://docs.ros.org/en/humble/p/sensor_msgs/generated/classsensor__msgs_1_1PointCloud2Iterator.html)
- [ROS 2 Humble RViz marker types](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html)
