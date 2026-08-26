# RUBI Phase 1 Simple Pure Pursuit

This standalone controller connects the heightmap planner to the locomotion
command interface without a Nav2 controller or costmap:

```text
/rubi/heightmap_step_planner/path -> Simple Pure Pursuit -> /cmd_vel
```

Run the planner separately, then start the controller with:

```bash
ros2 launch pongbot_navigation rubi_step_pure_pursuit.launch.py use_sim_time:=true
```

The controller follows the input `nav_msgs/Path` geometry without smoothing.
It uses `map <- base_link` TF on every control cycle and stops immediately for
an empty/invalid Path, unavailable TF, excessive cross-track error, or arrival
within the XY goal tolerance. Phase 1 deliberately ignores the final Path
orientation and performs no goal-yaw alignment or reverse driving.

The controller itself performs no obstacle detection or prediction.

Unexpected terrain/obstacles may trigger a stop and reroute only when they are
represented by the updated global height map, invalidate the remaining global
Path, and cause the planner to publish an empty Path or a new valid Path.

This is reactive map-triggered replanning, not predictive dynamic obstacle
avoidance.

Known limitations: no dynamic-object prediction, independent collision
detection, costmap, path smoothing, or footstep planning. Pure Pursuit may cut
sharp corners depending on lookahead; the moderate default lookahead and
cross-track stop threshold bound, but do not eliminate, that behavior.
