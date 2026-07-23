# Simulation validation

Run:
```bash
ros2 launch pongbot_local_graph_insertion_planner rubi_navigation_astar_local.launch.py use_sim_time:=true
```

Confirm the plugin with:
```bash
ros2 param get /planner_server planner_plugins
ros2 param get /planner_server AstarLocal.plugin
ros2 lifecycle get /planner_server
```
Expected values are `[AstarLocal]`, `pongbot_local_graph_insertion_planner/AstarLocalPlanner`, and `active`.

Confirm the local overlay input:
```bash
ros2 topic type /local_costmap/costmap_raw
ros2 topic hz /local_costmap/costmap_raw
ros2 topic echo /local_costmap/costmap_raw --once --field header
```
Its type must be `nav2_msgs/msg/Costmap`. In RViz display `/map`, `/global_costmap/costmap`,
`/local_costmap/costmap`, `/astar_local/fused_grid`, and `/plan`.

Set a Nav2 goal, then place a Gazebo box on the path. The local costmap and fused grid must show it;
within about one second the log should show `mode=DSTAR_REPAIR`, nonzero changed cells, and a detoured
`/plan`. Remove the box and confirm the fused overlay is replaced (not accumulated), the passage reopens,
and the plan shortens. If a local obstacle is absent from the fused grid, investigate subscription/TF/rasterization;
if it is fused but crossed by the plan, investigate incremental update/path validation.
