#!/usr/bin/env bash
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
exec ros2 bag record -o "$output" \
  /clock /tf /tf_static /cmd_vel /ground_truth/odom \
  /rubi_gazebo_legacy/joint_states_500hz /rubi_gazebo_legacy/status \
  /rubi/experiment/controller_sample /rubi/experiment/metadata \
  /rubi/experiment/contact_frame /rubi/experiment/bumper/left /rubi/experiment/bumper/right \
  /global_costmap/costmap_raw /fastdem/mapping/cloud_global \
  /rubi/planner_comparison/goal /rubi/planner_comparison/grid/path \
  /rubi/planner_comparison/grid/tracking_path /rubi/planner_comparison/robot_trace \
  /rubi/experiment/plan_metrics /rubi/experiment/events \
  /rubi/experiment/run_metrics /rubi/experiment/power_abs_W /rubi/experiment/power_signed_W
