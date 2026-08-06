#!/usr/bin/env bash
set -euo pipefail

# Install this file on the Jetson next to the existing startup script. The
# restricted dashboard SSH key may invoke only the three commands below.
START_SCRIPT="${ROBOT_COMM_START_SCRIPT:-/home/rclab/ros2_ws/src/navigation_real/scripts/robot_startup.sh}"
TMUX_SESSION="robot"
ZENOH_PID_FILE="/tmp/robot_zenoh_supervisor.pid"

action="${1:-}"
if [[ -n "${SSH_ORIGINAL_COMMAND:-}" ]]; then
    case "$SSH_ORIGINAL_COMMAND" in
        "robot-communication start") action="start" ;;
        "robot-communication stop") action="stop" ;;
        "robot-communication status") action="status" ;;
        *) echo "command denied" >&2; exit 64 ;;
    esac
fi

zenoh_supervisor_running()
{
    local pid=""
    local cmdline=""
    [[ -r "$ZENOH_PID_FILE" ]] || return 1
    read -r pid < "$ZENOH_PID_FILE" || return 1
    [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null || return 1
    cmdline=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)
    [[ "$cmdline" == *"zenoh_bridge_supervisor.sh"* ]]
}

communication_running()
{
    command -v tmux >/dev/null 2>&1 &&
        tmux has-session -t "$TMUX_SESSION" 2>/dev/null &&
        zenoh_supervisor_running
}

stop_communication()
{
    local pid=""
    if command -v tmux >/dev/null 2>&1; then
        tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true
    fi
    if [[ -r "$ZENOH_PID_FILE" ]]; then
        read -r pid < "$ZENOH_PID_FILE" || true
        if [[ "$pid" =~ ^[0-9]+$ ]] && zenoh_supervisor_running; then
            kill -TERM "$pid" 2>/dev/null || true
        fi
        unlink "$ZENOH_PID_FILE" 2>/dev/null || true
    fi
    pkill -TERM -x zenoh-bridge-ros2dds 2>/dev/null || true
    pkill -TERM -x joy_lcm_node 2>/dev/null || true
    pkill -TERM -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch pongbot_lcm joy_lcm.launch.py$' 2>/dev/null || true
    pkill -TERM -f '^(/usr/bin/)?python3 /home/rclab/(network_monitor|motor_monitor|epm_station_control)\.py$' 2>/dev/null || true
}

case "$action" in
    status)
        if communication_running; then
            echo "active"
            exit 0
        fi
        echo "inactive"
        exit 3
        ;;
    start)
        if communication_running; then
            echo "active"
            exit 0
        fi
        [[ -x "$START_SCRIPT" ]] || { echo "startup script is not executable: $START_SCRIPT" >&2; exit 1; }
        "$START_SCRIPT"
        communication_running || { echo "startup completed without a healthy robot session" >&2; exit 1; }
        echo "active"
        ;;
    stop)
        stop_communication
        echo "inactive"
        ;;
    *)
        echo "usage: $0 {start|stop|status}" >&2
        exit 64
        ;;
esac
