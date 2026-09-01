# Hybrid Grid/TRG lab demo workflow

## Raw and tracking path contract

`grid/path` and `sampling/path` remain the unmodified global-planner research
outputs. Both planners feed the same validated tracking refiner: one sequential
TRG three-point-mean pass, terrain-elevation re-query, Hybrid evaluator node and
edge gates, a 5% cost-increase gate, 0.10 m resampling, and final full-path
validation. Pure Pursuit subscribes only to `grid/tracking_path` or
`sampling/tracking_path`; endpoints and the requested Goal orientation remain
unchanged.

Distinct map content triggers validation of the remaining tracking path. A hard
costmap, height-evidence, or over-8-cm step failure first publishes an empty
tracking path so Pure Pursuit stops, then replans from the latest robot pose to
the retained Goal. A still-valid update is reoptimized at a limited rate and is
adopted only at 5% or greater cost improvement. Timestamp-only republishes are
deduplicated and update freshness without causing path churn.

ObstacleLayer remains **MANUAL / OUT OF V5 SCOPE**. If added later, any changed
raw costmap content enters the same validation and replanning mechanism.

The hybrid launches are self-contained for the PGM map and static+inflation
global costmap. Do not launch `rubi_navigation_dwb.launch.py` alongside them.
Localization (`map -> base_link`) and FastDEM remain external prerequisites.
See `HYBRID_SELF_CONTAINED_BRINGUP.md` for the complete contract.

## Safety contract

`grid_only` and `sampling_only` each start exactly one
`pongbot_navigation/simple_pure_pursuit_controller`. The planner node itself
never publishes velocity commands. `both` starts no controller.

Do not send a Nav2 BT/DWB goal and a comparison Goal at the same time. Before
real motion, always inspect command ownership:

```bash
ros2 topic info -v /cmd_vel
ros2 run rubi_heightmap_step_wavefront_planner check_hybrid_nav_inputs.sh
```

Do not continue if another controller publishes `/cmd_vel`.

The controller profile is shared by both modes in
`config/hybrid_navigation_controller.yaml`. It intentionally copies the
currently used local Simple Pure Pursuit hardware profile; this experiment did
not retune it.

For a controlled follow-up experiment, keep every other controller parameter
fixed and compare `lookahead_distance_m` at `0.35` (the current baseline),
`0.60`, and `0.70`. V5 does not select a new default or change velocity and
acceleration limits; record real-robot tracking evidence before adopting one of
these values.

## Prerequisites

Bring up simulation or robot localization, Livox, FastDEM, and the Nav2 global
costmap. `rubi_navigation_dwb.launch.py` may supply the costmap, but do not send
its Nav2 navigation action while the hybrid controller is active.

Verify `/global_costmap/costmap_raw`, `/fastdem/mapping/cloud_global`, and the
`map -> base_link` TF before sending a Goal.

## A. Grid navigation

```bash
ros2 launch rubi_heightmap_step_wavefront_planner \
  hybrid_grid_navigation.launch.py
ros2 topic info -v /cmd_vel
```

## B. Sampling navigation

Stop the Grid launch/controller first, then run:

```bash
ros2 launch rubi_heightmap_step_wavefront_planner \
  hybrid_sampling_navigation.launch.py
ros2 topic info -v /cmd_vel
```

Use the same start and Goal as the Grid run.

## C. Controller-free comparison

```bash
ros2 launch rubi_heightmap_step_wavefront_planner \
  hybrid_grid_trg_comparison.launch.py
```

All launches accept `launch_rviz:=false`. The default RViz config overlays the
master costmap, FastDEM cloud, Grid expansion tree/path, Sampling graph/path and
rejections, and the navigation-mode robot trace.

## Goal

```bash
ros2 topic pub --once /rubi/planner_comparison/goal \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

Each new Goal resets the robot trace. Trace collection stops within 0.20 m of
the snapped Goal and the last trace remains transient-local for presentation.

## Presentation capture

Capture the terminal result blocks and one RViz view containing:

- `/global_costmap/costmap`
- `/fastdem/mapping/cloud_global`
- Grid expanded cells/search edges/path
- Sampling accepted nodes/edges/rejections/path
- `/rubi/planner_comparison/robot_trace`

Treat the timing breakdown as one-scene diagnostics. Do not claim statistical
superiority without repeated controlled trials.
