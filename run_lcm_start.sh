#!/usr/bin/env bash
set -euo pipefail

# LCM multicast must stay on the dedicated Jetson <-> Main PC Ethernet link.
LCM_IF="${LCM_IF:-enP7p1s0}"
LCM_GROUP="${LCM_GROUP:-239.255.76.67}"

if ! link_info=$(ip -o link show dev "$LCM_IF" 2>/dev/null); then
    echo "[LCM] interface not found: $LCM_IF" >&2
    exit 1
fi

# Administrative UP alone also occurs with NO-CARRIER. Require a physical link
# before installing a route or claiming that the Main PC connection is ready.
link_flags=${link_info#*<}
link_flags=${link_flags%%>*}
if [[ ",$link_flags," != *,UP,* ]]; then
    echo "[LCM] interface is not up: $LCM_IF" >&2
    exit 1
fi
if [[ "$(cat "/sys/class/net/$LCM_IF/carrier" 2>/dev/null || true)" != "1" ]]; then
    echo "[LCM] no carrier on $LCM_IF; check the cable and Main PC port first" >&2
    exit 1
fi

SUDO=()
if (( EUID != 0 )); then
    SUDO=(sudo)
fi

"${SUDO[@]}" ip link set dev "$LCM_IF" multicast on
# 'replace' is idempotent, unlike the previous legacy 'route add' command.
"${SUDO[@]}" ip route replace 224.0.0.0/4 dev "$LCM_IF"

route_info=$(ip route get "$LCM_GROUP")
if [[ " $route_info " != *" dev $LCM_IF "* ]] ||
   [[ "$(cat "/sys/class/net/$LCM_IF/carrier" 2>/dev/null || true)" != "1" ]]; then
    echo "[LCM] link or route changed; multicast route is not ready" >&2
    printf '%s\n' "$route_info" >&2
    exit 1
fi

echo "[LCM] multicast route ready (link/route check only; not a receive confirmation)"
printf '%s\n' "$route_info"
