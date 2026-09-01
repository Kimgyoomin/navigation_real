# Hybrid Self-contained Bringup

The Grid, Sampling, and comparison launches provide their own installed PGM
map and standalone Nav2 global costmap. The costmap intentionally contains only
`StaticLayer` and `InflationLayer`; live dynamic-obstacle handling is outside
this profile.

## External prerequisites

Start localization and FastDEM before a navigation launch. The runtime must
provide `map -> base_link` TF and `/fastdem/mapping/cloud_global`.

Do **not** run `pongbot_navigation/rubi_navigation_dwb.launch.py` at the same
time. It would duplicate the map server, global costmap, controller, and
`/cmd_vel` publisher. To intentionally reuse an external map/costmap, pass
`launch_map_costmap:=false`.

## Commands

Grid navigation (one Pure Pursuit controller):

```bash
ros2 launch rubi_heightmap_step_wavefront_planner hybrid_grid_navigation.launch.py
```

Sampling navigation (the same controller profile):

```bash
ros2 launch rubi_heightmap_step_wavefront_planner hybrid_sampling_navigation.launch.py
```

Grid/TRG comparison (no controller and no `/cmd_vel` publisher):

```bash
ros2 launch rubi_heightmap_step_wavefront_planner hybrid_grid_trg_comparison.launch.py
```

The RViz 2D Goal Pose tool publishes directly to
`/rubi/planner_comparison/goal`. Before allowing motion, run
`check_hybrid_nav_inputs.sh` and confirm that `/cmd_vel` has exactly one
expected Pure Pursuit publisher.

## Data flow

`RUBI_occupancy_map.yaml` -> `/map` -> static + inflation global costmap ->
`/global_costmap/costmap_raw` -> Grid or TRG planner -> `nav_msgs/Path` ->
Pure Pursuit (navigation modes only) -> `/cmd_vel`.

The installed map YAML and PGM remain beside each other in `share/.../maps`, so
the YAML's relative `image: RUBI_occupancy_map.pgm` reference is portable.

On ROS 2 Humble, the upstream `nav2_costmap_2d` executable hard-codes the FQN
`/costmap/costmap` even when launch name/namespace remaps are supplied. The
package therefore provides `hybrid_global_costmap_node`, a thin main around the
unmodified public `nav2_costmap_2d::Costmap2DROS` class, to establish the required
`/global_costmap/global_costmap` lifecycle and topic namespace.

The lifecycle manager performs deactivate/cleanup/shutdown. After ROS SIGINT
invalidates the Humble context, the thin main bypasses only the known-crashing
Costmap2DROS process-exit destructor; plugin cleanup has already completed.
