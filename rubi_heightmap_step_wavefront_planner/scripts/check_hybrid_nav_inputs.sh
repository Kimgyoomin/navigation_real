#!/usr/bin/env bash
set -u

check_topic() {
  local topic_name="$1"
  if ros2 topic list | grep -Fxq "$topic_name"; then
    echo "PASS topic: $topic_name"
    ros2 topic info -v "$topic_name"
  else
    echo "FAIL missing topic: $topic_name"
  fi
}

check_topic /global_costmap/costmap_raw
check_topic /fastdem/mapping/cloud_global

echo "INFO TF map <- base_link (read-only, 2 second timeout)"
timeout 2 ros2 run tf2_ros tf2_echo map base_link || true

echo "INFO /cmd_vel endpoints"
ros2 topic info -v /cmd_vel || true

echo "INFO duplicate planner/controller node names"
ros2 node list 2>/dev/null | sort | uniq -d || true
