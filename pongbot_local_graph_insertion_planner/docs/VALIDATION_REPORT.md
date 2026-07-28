# AstarLocal validation report

Date: 2026-07-28  
Branch: `rubi_simulation`  
Baseline HEAD: `fd2b49179f34984c678c52bbbdb731cad89bfaf4`

## Build and automated tests

Commands:

```bash
source /opt/ros/humble/setup.bash
source /home/kim/rubi_ws/install/setup.bash
cd /home/kim/ros2_ws_nav
colcon build --symlink-install \
  --packages-up-to pongbot_local_graph_insertion_planner
source install/setup.bash
colcon test --packages-select pongbot_local_graph_insertion_planner \
  --event-handlers console_direct+
colcon test-result --verbose
```

Result:

- Build: PASS, 1 package.
- CTest targets: 2/2 PASS.
- GTest cases: 23/23 PASS.
- Colcon result accounting: 25 tests, 0 errors, 0 failures, 0 skipped.
- Static randomized fresh A*/D* differential: 250 grids, 0 mismatch.
- Incremental randomized differential: 50 scenarios with 10 repairs each
  (550 plans including initial plans), 0 mismatch.

The differential assertions compare reachability, path validity, and total cost
with `1e-9` tolerance under the same grid and cost contract.

## Synthetic benchmark

Command:

```bash
ros2 run pongbot_local_graph_insertion_planner benchmark_replanning
```

Conditions: 464 x 454 grid (210,656 cells), 49 changed cells per alternating
insertion/removal event, 30 measured events per mode.

| Mode | median ms | p95 ms | max ms | mean expanded |
|---|---:|---:|---:|---:|
| FRESH_ASTAR | 3.043 | 21.314 | 21.335 | 2173.000 |
| DSTAR_REPAIR | 13.565 | 24.780 | 168.818 | 447.633 |

Differential mismatches: 0. D* expanded fewer nodes but was slower because the
current implementation still performs full-grid scans. Incremental speedup is
not established.

The existing `pongbot_global_planner/AstarPlanner` runtime latency is
NOT_EVALUATED because the TF blocker prevented a comparable action request. Its
cost contract also differs, so its raw path cost must not be compared directly.

## Install and runtime preflight

- Shared plugin library: PASS.
- Plugin XML: PASS.
- Launch, YAML, and BT installation: PASS.
- `ldd` unresolved dependencies: 0.
- BT/package/plugin XML parsing: PASS.
- AstarLocal plugin configure/load: PASS.
- SimpleSmoother configure/load: PASS.
- SmoothPath collision costmap is the planner-published
  `/astar_local/fused_costmap_raw`, not the static global costmap.
- Map load: PASS (`464 x 454 @ 0.05 m/cell`).
- `/livox/lidar_PointCloud2` declared type: PASS
  (`sensor_msgs/msg/PointCloud2`).

## Runtime blocker

Full navigation activation and obstacle insertion/removal/no-path scenarios are
BLOCKED. The already-running Gazebo graph declared `/clock`, Livox, and Odometry
topics but produced no samples during the measurement windows. FAST-LIO accepted
an `/initialpose` but did not produce `camera_init`.

Nav2 stopped safely at local costmap activation:

```text
Timed out waiting for transform from base_link to camera_init
Invalid frame ID "camera_init"
```

No static identity transform or latest-TF default was introduced to mask the
missing runtime chain.
