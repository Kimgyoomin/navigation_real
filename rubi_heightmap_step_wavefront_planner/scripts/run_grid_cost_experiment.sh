#!/usr/bin/env bash
set -euo pipefail
if [[ $# -lt 1 || ( "$1" != vanilla && "$1" != cot ) ]]; then
  echo "Usage: $0 vanilla|cot [ros2 launch arguments...]" >&2
  exit 2
fi
mode="$1"
shift
exec ros2 launch rubi_heightmap_step_wavefront_planner grid_cost_experiment.launch.py "cost_mode:=$mode" "$@"
