#!/usr/bin/env bash
set -Eeuo pipefail

echo "=== Robot startup: 5G Zenoh + wired LCM ==="

TARGET_ZENOH_IP="${TARGET_ZENOH_IP:-115.31.99.192}"
TARGET_ZENOH_PORT="${TARGET_ZENOH_PORT:-7447}"
TARGET_CONTROL_PC="${TARGET_CONTROL_PC:-192.168.100.14}"
MOBILE_IF="${MOBILE_IF:-ppp0}"
LAN_IF="${LAN_IF:-enP7p1s0}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-13}"

ROBOT_HOME="/home/rclab"
WORKSPACE="$ROBOT_HOME/ros2_ws"
NAVIGATION_DIR="$WORKSPACE/src/navigation_real"
ZENOH_CONFIG="$ROBOT_HOME/zenoh_config.json5"
ZENOH_SUPERVISOR="$NAVIGATION_DIR/scripts/zenoh_bridge_supervisor.sh"
ZENOH_PID_FILE="/tmp/robot_zenoh_supervisor.pid"
ROBOT_LOG_DIR="$ROBOT_HOME/.local/state/robot_startup"
FASTDDS_XML_PATH="/tmp/fastdds_profile.xml"
CYCLONEDDS_XML_PATH="/tmp/cyclonedds_profile.xml"
TMUX_SESSION="robot"
COMPONENT_RUNNER="$NAVIGATION_DIR/scripts/robot_component.sh"
EXPECTED_WINDOWS=(zenoh ros2_lcm camera network epm motor)

mkdir -p "$ROBOT_LOG_DIR"

log()
{
    printf '%s %s\n' "$(date --iso-8601=seconds)" "$*"
}

wait_for_ipv4()
{
    local interface="$1"
    local max_retries="$2"
    local retry=0

    while ! ip -4 address show dev "$interface" 2>/dev/null | grep -q 'inet '; do
        retry=$((retry + 1))
        if (( retry >= max_retries )); then
            return 1
        fi
        log "[WAIT] IPv4 on $interface ($retry/$max_retries)"
        sleep 1
    done
}

route_device()
{
    ip route get "$1" 2>/dev/null |
        awk '{for (i=1; i<=NF; i++) if ($i=="dev") {print $(i+1); exit}}'
}

stop_zenoh_supervisor()
{
    [[ -r "$ZENOH_PID_FILE" ]] || return 0

    local old_pid=""
    local cmdline=""
    read -r old_pid < "$ZENOH_PID_FILE" || true

    if [[ "$old_pid" =~ ^[0-9]+$ ]] && kill -0 "$old_pid" 2>/dev/null; then
        cmdline=$(tr '\0' ' ' < "/proc/$old_pid/cmdline" 2>/dev/null || true)
        if [[ "$cmdline" == *"zenoh_bridge_supervisor.sh"* ]]; then
            log "[STOP] previous Zenoh supervisor pid=$old_pid"
            kill -TERM "$old_pid" 2>/dev/null || true
            for _ in $(seq 1 30); do
                kill -0 "$old_pid" 2>/dev/null || break
                sleep 0.1
            done
            if kill -0 "$old_pid" 2>/dev/null; then
                cmdline=$(tr '\0' ' ' < "/proc/$old_pid/cmdline" 2>/dev/null || true)
                if [[ "$cmdline" == *"zenoh_bridge_supervisor.sh"* ]]; then
                    log "[KILL] unresponsive Zenoh supervisor pid=$old_pid"
                    kill -KILL "$old_pid" 2>/dev/null || true
                fi
            fi
        else
            log "[WARN] stale PID file did not point to the Zenoh supervisor"
        fi
    fi

    unlink "$ZENOH_PID_FILE" 2>/dev/null || true
}

zenoh_bridge_running()
{
    pgrep -f '(^|/)zenoh-bridge-ros2dds([[:space:]]|$)' >/dev/null 2>&1
}

for command_name in ip ping pgrep timeout tmux zenoh-bridge-ros2dds; do
    command -v "$command_name" >/dev/null 2>&1 || {
        log "[ERROR] required command is missing: $command_name"
        exit 1
    }
done
[[ -r "$ZENOH_CONFIG" ]] || { log "[ERROR] cannot read $ZENOH_CONFIG"; exit 1; }
[[ "$TARGET_ZENOH_PORT" =~ ^[0-9]+$ ]] || { log "[ERROR] invalid Zenoh port"; exit 1; }

# -----------------------------------------------------------------------------
# 1. Require the 5G and robot-LAN interfaces before starting communication.
# -----------------------------------------------------------------------------
if ! wait_for_ipv4 "$MOBILE_IF" 60; then
    log "[ERROR] 5G interface $MOBILE_IF has no IPv4 address"
    exit 1
fi

log "[OK] 5G interface ready: $(ip -4 -brief address show dev "$MOBILE_IF")"

if ping -c 1 -W 2 -I "$MOBILE_IF" "$TARGET_ZENOH_IP" >/dev/null 2>&1; then
    log "[OK] Zenoh host answers ICMP through $MOBILE_IF"
else
    log "[WARN] Zenoh host did not answer ICMP; checking the actual TCP service"
fi

if ! wait_for_ipv4 "$LAN_IF" 30; then
    log "[ERROR] robot LAN $LAN_IF has no IPv4 address"
    exit 1
fi

LAN_ROUTE_IF=$(route_device "$TARGET_CONTROL_PC")
LAN_IP=$(ip route get "$TARGET_CONTROL_PC" 2>/dev/null |
    awk '{for (i=1; i<=NF; i++) if ($i=="src") {print $(i+1); exit}}')

if [[ "$LAN_ROUTE_IF" != "$LAN_IF" ]] || [[ -z "$LAN_IP" ]]; then
    log "[ERROR] Main PC route must use $LAN_IF (actual=${LAN_ROUTE_IF:-none})"
    exit 1
fi

if ! ping -c 1 -W 1 -I "$LAN_IF" "$TARGET_CONTROL_PC" >/dev/null 2>&1; then
    log "[WARN] Main PC $TARGET_CONTROL_PC did not answer ping; LCM will still start"
else
    log "[OK] Main PC reachable through $LAN_IF (source=$LAN_IP)"
fi

# The NetworkManager profile normally installs this route persistently.
LCM_ROUTE_IF=$(route_device "239.255.76.67")
if [[ "$LCM_ROUTE_IF" != "$LAN_IF" ]]; then
    log "[WARN] repairing LCM multicast route on $LAN_IF"
    if ! "$NAVIGATION_DIR/run_lcm_start.sh"; then
        log "[ERROR] failed to install the LCM multicast route"
        exit 1
    fi
fi

# Zenoh itself is bound to ppp0 below. Try to route the other server traffic
# (for example RTSP) over 5G too, without blocking startup if sudo is unavailable.
SERVER_ROUTE_IF=$(route_device "$TARGET_ZENOH_IP")
if [[ "$SERVER_ROUTE_IF" != "$MOBILE_IF" ]]; then
    if sudo -n ip route replace "$TARGET_ZENOH_IP/32" dev "$MOBILE_IF" 2>/dev/null; then
        log "[OK] server host route moved to $MOBILE_IF"
    else
        log "[WARN] default server route uses ${SERVER_ROUTE_IF:-unknown}; Zenoh is still forced to $MOBILE_IF"
    fi
fi

if ! timeout 5 bash -c 'exec 3<>"/dev/tcp/$1/$2"' bash \
    "$TARGET_ZENOH_IP" "$TARGET_ZENOH_PORT" 2>/dev/null; then
    log "[ERROR] Zenoh TCP service is not reachable: $TARGET_ZENOH_IP:$TARGET_ZENOH_PORT"
    exit 1
fi
log "[OK] Zenoh TCP service reachable: $TARGET_ZENOH_IP:$TARGET_ZENOH_PORT"

# -----------------------------------------------------------------------------
# 2. Bind all local ROS 2 DDS traffic to the dedicated Ethernet interface.
# -----------------------------------------------------------------------------
cat > "$FASTDDS_XML_PATH" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>udp_transport</transport_id>
                <type>UDPv4</type>
                <interfaceWhiteList>
                    <address>$LAN_IP</address>
                </interfaceWhiteList>
            </transport_descriptor>
        </transport_descriptors>
        <participant profile_name="restricted_network_profile" is_default_profile="true">
            <rtps>
                <userTransports>
                    <transport_id>udp_transport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
EOF
log "[OK] FastDDS bound to $LAN_IP on $LAN_IF"

# zenoh-bridge-ros2dds embeds CycloneDDS, so it needs its own interface profile.
cat > "$CYCLONEDDS_XML_PATH" <<EOF
<CycloneDDS>
    <Domain>
        <General>
            <Interfaces>
                <NetworkInterface address="$LAN_IP"/>
            </Interfaces>
            <AllowMulticast>true</AllowMulticast>
        </General>
    </Domain>
</CycloneDDS>
EOF
log "[OK] CycloneDDS bridge side bound to $LAN_IP on $LAN_IF"

# -----------------------------------------------------------------------------
# 3. Stop only communication processes owned by this startup sequence.
# -----------------------------------------------------------------------------
stop_zenoh_supervisor

if command -v tmux >/dev/null 2>&1 && tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
    log "[STOP] previous tmux session: $TMUX_SESSION"
    tmux kill-session -t "$TMUX_SESSION"
fi

pkill -TERM -f '(^|/)zenoh-bridge-ros2dds([[:space:]]|$)' 2>/dev/null || true
pkill -TERM -x joy_lcm_node 2>/dev/null || true
pkill -TERM -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch pongbot_lcm joy_lcm.launch.py$' 2>/dev/null || true
pkill -TERM -f '^(/usr/bin/)?python3 /home/rclab/(network_monitor|motor_monitor|epm_station_control)\.py$' 2>/dev/null || true

set +u
source /opt/ros/humble/setup.bash
set -u
command -v ros2 >/dev/null 2>&1 || { log "[ERROR] ROS 2 setup did not provide ros2"; exit 1; }
ros2 pkg prefix rmw_fastrtps_cpp >/dev/null 2>&1 || {
    log "[ERROR] FastDDS RMW is not installed: ros-humble-rmw-fastrtps-cpp"
    exit 1
}
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1

# -----------------------------------------------------------------------------
# 4. Run all robot processes in one tmux session.
# -----------------------------------------------------------------------------
if ! command -v tmux >/dev/null 2>&1; then
    log "[ERROR] tmux is not installed"
    exit 1
fi

if [[ ! -x "$COMPONENT_RUNNER" ]]; then
    log "[ERROR] component runner is not executable: $COMPONENT_RUNNER"
    exit 1
fi

export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ZENOH_SERVER="$TARGET_ZENOH_IP"
export ZENOH_PORT="$TARGET_ZENOH_PORT"
export ZENOH_IF="$MOBILE_IF"
export ZENOH_CONFIG
export ZENOH_PID_FILE
export ROBOT_LOG_DIR
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_XML_PATH"
export CYCLONEDDS_URI="file://$CYCLONEDDS_XML_PATH"

ZENOH_TMUX_COMMAND="env ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION CYCLONEDDS_URI=$CYCLONEDDS_URI ZENOH_SERVER=$TARGET_ZENOH_IP ZENOH_PORT=$TARGET_ZENOH_PORT ZENOH_IF=$MOBILE_IF ZENOH_CONFIG=$ZENOH_CONFIG ZENOH_PID_FILE=$ZENOH_PID_FILE ROBOT_LOG_DIR=$ROBOT_LOG_DIR $COMPONENT_RUNNER zenoh"

tmux new-session -d -s "$TMUX_SESSION" -n zenoh "$ZENOH_TMUX_COMMAND"
tmux set-environment -t "$TMUX_SESSION" ROS_DOMAIN_ID "$ROS_DOMAIN_ID"
tmux set-environment -t "$TMUX_SESSION" RMW_IMPLEMENTATION "$RMW_IMPLEMENTATION"
tmux set-environment -t "$TMUX_SESSION" FASTRTPS_DEFAULT_PROFILES_FILE "$FASTRTPS_DEFAULT_PROFILES_FILE"
tmux set-environment -t "$TMUX_SESSION" CYCLONEDDS_URI "$CYCLONEDDS_URI"
tmux set-option -t "$TMUX_SESSION" remain-on-exit on
tmux set-option -t "$TMUX_SESSION" allow-rename off
tmux set-option -t "$TMUX_SESSION" mouse on
tmux set-option -t "$TMUX_SESSION" status-interval 2
tmux set-option -t "$TMUX_SESSION" status-left "[robot] "
tmux set-option -t "$TMUX_SESSION" status-right "#(date '+%H:%M:%S')"

sleep 3
if [[ ! -r "$ZENOH_PID_FILE" ]]; then
    log "[ERROR] Zenoh supervisor failed to start; check $ROBOT_LOG_DIR/zenoh_bridge.log"
    tmux capture-pane -p -t "$TMUX_SESSION:zenoh" 2>/dev/null || true
    exit 1
fi
for _ in {1..30}; do
    zenoh_bridge_running && break
    sleep 0.2
done
if ! zenoh_bridge_running; then
    log "[ERROR] Zenoh supervisor is running but the bridge process is not"
    tail -n 80 "$ROBOT_LOG_DIR/zenoh_bridge.log" 2>/dev/null || true
    exit 1
fi

tmux new-window -d -t "$TMUX_SESSION:" -n ros2_lcm "$COMPONENT_RUNNER ros2_lcm"
tmux new-window -d -t "$TMUX_SESSION:" -n camera "$COMPONENT_RUNNER camera"
tmux new-window -d -t "$TMUX_SESSION:" -n network "$COMPONENT_RUNNER network"
tmux new-window -d -t "$TMUX_SESSION:" -n epm "$COMPONENT_RUNNER epm"
tmux new-window -d -t "$TMUX_SESSION:" -n motor "$COMPONENT_RUNNER motor"
tmux select-window -t "$TMUX_SESSION:ros2_lcm"

sleep 2
COMPONENT_FAILURE=0
for window in "${EXPECTED_WINDOWS[@]}"; do
    PANE_DEAD=$(tmux display-message -p -t "$TMUX_SESSION:$window" '#{pane_dead}' 2>/dev/null || true)
    if [[ "$PANE_DEAD" != "0" ]]; then
        log "[ERROR] robot component exited: $window"
        tmux capture-pane -p -S -80 -t "$TMUX_SESSION:$window" 2>/dev/null >&2 || true
        COMPONENT_FAILURE=1
    fi
done
if (( COMPONENT_FAILURE )); then
    exit 1
fi

log "[DONE] all robot processes are in tmux session '$TMUX_SESSION'"

if [[ -n "${TMUX:-}" ]]; then
    tmux switch-client -t "$TMUX_SESSION"
elif [[ -t 0 ]] && [[ -t 1 ]]; then
    exec tmux attach-session -t "$TMUX_SESSION"
fi
