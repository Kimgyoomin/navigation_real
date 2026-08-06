#!/usr/bin/env bash
set -o pipefail

ROLE="${1:-}"
ROBOT_HOME="/home/rclab"
WORKSPACE="$ROBOT_HOME/ros2_ws"
NAVIGATION_DIR="$WORKSPACE/src/navigation_real"
LOG_DIR="${ROBOT_LOG_DIR:-$ROBOT_HOME/.local/state/robot_startup}"
FASTDDS_XML_PATH="/tmp/fastdds_profile.xml"
CYCLONEDDS_XML_PATH="/tmp/cyclonedds_profile.xml"

mkdir -p "$LOG_DIR"

export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-13}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_XML_PATH"
export CYCLONEDDS_URI="file://$CYCLONEDDS_XML_PATH"

setup_ros()
{
    source /opt/ros/humble/setup.bash
}

case "$ROLE" in
    zenoh)
        exec "$NAVIGATION_DIR/scripts/zenoh_bridge_supervisor.sh"
        ;;

    ros2_lcm)
        exec > >(tee -a "$LOG_DIR/ros2_lcm.log") 2>&1
        setup_ros
        source "$WORKSPACE/install/setup.bash"
        exec ros2 launch pongbot_lcm joy_lcm.launch.py
        ;;

    camera)
        exec > >(tee -a "$LOG_DIR/camera.log") 2>&1

        camera_pid=""
        stop_camera()
        {
            trap - EXIT HUP INT TERM
            if [[ -n "$camera_pid" ]] && kill -0 "$camera_pid" 2>/dev/null; then
                echo "[camera] stopping GStreamer pid=$camera_pid"
                kill -TERM "$camera_pid" 2>/dev/null || true
                for _ in $(seq 1 30); do
                    kill -0 "$camera_pid" 2>/dev/null || break
                    sleep 0.1
                done
                if kill -0 "$camera_pid" 2>/dev/null; then
                    kill -KILL "$camera_pid" 2>/dev/null || true
                fi
                wait "$camera_pid" 2>/dev/null || true
            fi
        }
        trap stop_camera EXIT HUP INT TERM

        gst-launch-1.0 \
            v4l2src device=/dev/video0 \
            ! video/x-raw,width=640,height=480,framerate=30/1 \
            ! videoconvert \
            ! x264enc tune=zerolatency bitrate=1000 speed-preset=ultrafast \
            ! video/x-h264,profile=baseline \
            ! rtspclientsink location=rtsp://115.31.99.192:8554/cam1 protocols=tcp &
        camera_pid=$!
        wait "$camera_pid"
        camera_status=$?
        camera_pid=""
        trap - EXIT HUP INT TERM
        exit "$camera_status"
        ;;

    network)
        exec > >(tee -a "$LOG_DIR/network_monitor.log") 2>&1
        setup_ros
        cd "$ROBOT_HOME"
        exec python3 network_monitor.py
        ;;

    epm)
        exec > >(tee -a "$LOG_DIR/epm.log") 2>&1
        setup_ros
        cd "$ROBOT_HOME"
        exec python3 epm_station_control.py
        ;;

    motor)
        exec > >(tee -a "$LOG_DIR/motor_monitor.log") 2>&1
        setup_ros
        cd "$ROBOT_HOME"
        exec python3 motor_monitor.py
        ;;

    *)
        echo "usage: $0 {zenoh|ros2_lcm|camera|network|epm|motor}" >&2
        exit 2
        ;;
esac
