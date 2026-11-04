#!/usr/bin/env bash
# Offline recorded-geometry review; deliberately separate from robot domain 13.
set -e
review_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
review_ws=$(cd -- "$review_root/../.." && pwd)
source /opt/ros/humble/setup.bash
source "$review_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=89
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$review_root/navigation_dds/config/fastdds_common.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
exec /usr/bin/python3 "$review_root/tools/review_body_costmap.py" "$@"
