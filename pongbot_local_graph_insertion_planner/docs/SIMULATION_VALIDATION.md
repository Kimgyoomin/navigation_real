# Simulation validation

## Launch order

```bash
# Terminal 1
source /home/kim/rubi_ws/install/setup.bash
ros2 launch rubi_gazebo_sim rubi_gazebo_lidar.launch.py

# Terminal 2
source /opt/ros/humble/setup.bash
source /home/kim/rubi_ws/install/setup.bash
source /home/kim/ros2_ws_nav/install/setup.bash
ros2 launch fast_lio_localization localization_nav_rubi.launch.py \
  use_sim_time:=true use_rviz:=false publish_2d_map:=false

# Publish a valid /initialpose in RViz or on the command line.

# Terminal 3
source /opt/ros/humble/setup.bash
source /home/kim/rubi_ws/install/setup.bash
source /home/kim/ros2_ws_nav/install/setup.bash
ros2 launch pongbot_local_graph_insertion_planner \
  rubi_navigation_astar_local.launch.py \
  use_sim_time:=true autostart:=true
```

Do not start navigation until these transforms produce values:

```bash
ros2 run tf2_ros tf2_echo map camera_init
ros2 run tf2_ros tf2_echo camera_init base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
```

## Preflight

```bash
ros2 topic type /livox/lidar_PointCloud2
ros2 topic hz /livox/lidar_PointCloud2
ros2 topic echo /clock --once
ros2 topic type /local_costmap/costmap_raw
ros2 topic hz /local_costmap/costmap_raw
ros2 topic echo /local_costmap/costmap_raw --once --field header
ros2 topic type /astar_local/fused_costmap_raw
ros2 param get /planner_server planner_plugins
ros2 param get /planner_server AstarLocal.plugin
ros2 lifecycle get /planner_server
```

Expected plugin values are `[AstarLocal]` and
`pongbot_local_graph_insertion_planner/AstarLocalPlanner`. The local raw
costmap type must be `nav2_msgs/msg/Costmap`.

In RViz display `/map`, `/global_costmap/costmap`,
`/local_costmap/costmap`, `/astar_local/fused_grid`, `/plan`, TF, and
RobotModel.

## Obstacle event sequence

1. Save the first obstacle-free plan and `[AstarLocalMetrics]` line.
2. Insert a Gazebo box on the current corridor.
3. Confirm obstacle and inflation costs in local and fused grids.
4. Within the next 1 Hz planning cycle, confirm a collision-free detour and
   RPP path replacement.
5. Remove the box and wait for ObstacleLayer raytracing clearing.
6. Confirm the fused grid is restored and a shorter path reopens.
7. Completely block the passage and confirm planner failure and safe stop.

The default mode log is `FRESH_ASTAR`. `DSTAR_REPAIR` is expected only when
`enable_incremental_reuse` is explicitly set true.

## Validation status on 2026-07-28

- Gazebo nodes and `/livox/lidar_PointCloud2` type:
  `sensor_msgs/msg/PointCloud2` — PASS.
- Map load: `464 x 454 @ 0.05 m/cell` — PASS.
- AstarLocal plugin load — PASS.
- SimpleSmoother plugin configure — PASS.
- Full lifecycle activation and obstacle scenario — BLOCKED.

The active Gazebo process exposed topics but produced no `/clock`, Livox, or
Odometry samples during the measurement window. Consequently localization
created its nodes and accepted `/initialpose`, but did not publish
`camera_init`; Nav2 stopped at:

```text
Timed out waiting for transform from base_link to camera_init
Invalid frame ID "camera_init"
```

This is a runtime simulation/localization input blocker, not a planner build or
plugin-loading failure. No identity/latest transform was introduced to hide it.
