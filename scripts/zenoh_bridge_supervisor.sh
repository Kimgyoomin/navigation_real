#!/usr/bin/env bash
set -o pipefail

ZENOH_SERVER="${ZENOH_SERVER:-115.31.99.192}"
ZENOH_PORT="${ZENOH_PORT:-7447}"
ZENOH_IF="${ZENOH_IF:-ppp0}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-13}"
ZENOH_CONFIG="${ZENOH_CONFIG:-/home/rclab/zenoh_config.json5}"
PID_FILE="${ZENOH_PID_FILE:-/tmp/robot_zenoh_supervisor.pid}"
LOG_DIR="${ROBOT_LOG_DIR:-/home/rclab/.local/state/robot_startup}"
LOG_FILE="$LOG_DIR/zenoh_bridge.log"
RESTART_DELAY="${ZENOH_RESTART_DELAY:-2}"

mkdir -p "$LOG_DIR"

log()
{
    printf '%s %s\n' "$(date --iso-8601=seconds)" "$*" | tee -a "$LOG_FILE"
}

if ! command -v zenoh-bridge-ros2dds >/dev/null 2>&1; then
    log "[ERROR] zenoh-bridge-ros2dds is not installed"
    exit 1
fi

if [[ ! -r "$ZENOH_CONFIG" ]]; then
    log "[ERROR] Zenoh config is not readable: $ZENOH_CONFIG"
    exit 1
fi

if [[ -r "$PID_FILE" ]]; then
    read -r old_pid < "$PID_FILE" || true
    if [[ "${old_pid:-}" =~ ^[0-9]+$ ]] && kill -0 "$old_pid" 2>/dev/null; then
        log "[ERROR] another Zenoh supervisor is already running (pid=$old_pid)"
        exit 1
    fi
fi

printf '%s\n' "$$" > "$PID_FILE"

child_pid=""
cleanup()
{
    trap - EXIT
    if [[ -n "$child_pid" ]] && kill -0 "$child_pid" 2>/dev/null; then
        kill -TERM "$child_pid" 2>/dev/null || true
        wait "$child_pid" 2>/dev/null || true
    fi
    if [[ -r "$PID_FILE" ]] && [[ "$(<"$PID_FILE")" == "$$" ]]; then
        unlink "$PID_FILE"
    fi
}

shutdown()
{
    trap - INT TERM
    cleanup
    exit 0
}

trap cleanup EXIT
trap shutdown INT TERM

source /opt/ros/humble/setup.bash
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID

# Zenoh supports '#iface=<device>' on Linux. Quoting the endpoint is required
# because '#' otherwise starts a shell comment.
ZENOH_ENDPOINT="tcp/${ZENOH_SERVER}:${ZENOH_PORT}#iface=${ZENOH_IF}"

while true; do
    if ! ip -4 address show dev "$ZENOH_IF" | grep -q 'inet '; then
        log "[WARN] waiting for 5G interface $ZENOH_IF"
        sleep "$RESTART_DELAY"
        continue
    fi

    log "[START] Zenoh bridge -> $ZENOH_ENDPOINT (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
    zenoh-bridge-ros2dds client \
        -e "$ZENOH_ENDPOINT" \
        --no-multicast-scouting \
        -d "$ROS_DOMAIN_ID" \
        -c "$ZENOH_CONFIG" > >(tee -a "$LOG_FILE") 2>&1 &
    child_pid=$!
    wait "$child_pid"
    exit_code=$?
    child_pid=""
    log "[WARN] Zenoh bridge exited with code $exit_code; restarting in ${RESTART_DELAY}s"
    sleep "$RESTART_DELAY"
done
