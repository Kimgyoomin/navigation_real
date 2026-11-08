#!/usr/bin/env bash
# All visible topics, simulated receipt timestamps, plus retained static/metadata topics.
set -euo pipefail
if [[ $# -ne 1 || ! "$1" =~ ^[A-Za-z0-9_.-]+$ || "$1" == . || "$1" == .. ]]; then
  echo "Usage: $0 UNIQUE_RUN_ID" >&2
  exit 2
fi
output="${RUBI_COST_EXPERIMENT_DIR:-$HOME/cot_experiment/navigation_cost_comparison}/bags/$1"
if [[ -e "$output" ]]; then
  echo "Refusing to overwrite $output" >&2
  exit 2
fi
mkdir -p "$(dirname "$output")"
share="$(ros2 pkg prefix --share rubi_heightmap_step_wavefront_planner)"
exec ros2 bag record -a --use-sim-time -o "$output" \
  --qos-profile-overrides-path "$share/config/grid_cost_record_qos.yaml"
