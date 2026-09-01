#!/usr/bin/env bash
set -u

check_topic() {
  local topic_name="$1"
  local expected_type="$2"
  if ros2 topic list | rg -Fxq "$topic_name"; then
    echo "PASS topic: $topic_name"
    local actual_type
    actual_type="$(ros2 topic type "$topic_name" 2>/dev/null || true)"
    if [[ "$actual_type" == "$expected_type" ]]; then
      echo "PASS type: $topic_name -> $actual_type"
    else
      echo "FAIL type: $topic_name -> ${actual_type:-unknown} (expected $expected_type)"
    fi
  else
    echo "FAIL missing topic: $topic_name"
  fi
}

check_lifecycle() {
  local node_name="$1"
  local state
  state="$(ros2 lifecycle get "$node_name" 2>/dev/null || true)"
  if [[ "$state" == *"active"* ]]; then
    echo "PASS lifecycle: $node_name -> active"
  else
    echo "FAIL lifecycle: $node_name -> ${state:-unavailable}"
  fi
}

check_topic /map nav_msgs/msg/OccupancyGrid
check_topic /global_costmap/costmap nav_msgs/msg/OccupancyGrid
check_topic /global_costmap/costmap_raw nav2_msgs/msg/Costmap
check_topic /fastdem/mapping/cloud_global sensor_msgs/msg/PointCloud2
check_lifecycle /map_server
check_lifecycle /global_costmap/global_costmap

echo "INFO TF map <- base_link (read-only, 2 second timeout)"
timeout 2 ros2 run tf2_ros tf2_echo map base_link || true

echo "INFO /cmd_vel endpoints"
ros2 topic info -v /cmd_vel || true

echo "INFO duplicate planner/controller node names"
ros2 node list 2>/dev/null | sort | uniq -d || true
