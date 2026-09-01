#!/usr/bin/env bash
set -euo pipefail
ros2 launch rubi_heightmap_step_wavefront_planner hybrid_grid_trg_comparison.launch.py "$@"
