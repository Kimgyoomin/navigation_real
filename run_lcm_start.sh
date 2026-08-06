#!/usr/bin/env bash
set -euo pipefail

# LCM multicast must stay on the dedicated Jetson <-> Main PC Ethernet link.
LCM_IF="${LCM_IF:-enP7p1s0}"
LCM_GROUP="${LCM_GROUP:-239.255.76.67}"

if ! ip link show dev "$LCM_IF" >/dev/null 2>&1; then
    echo "[LCM] interface not found: $LCM_IF" >&2
    exit 1
fi

if ! ip link show dev "$LCM_IF" | grep -q "UP"; then
    echo "[LCM] interface is not up: $LCM_IF" >&2
    exit 1
fi

SUDO=()
if (( EUID != 0 )); then
    SUDO=(sudo)
fi

"${SUDO[@]}" ip link set dev "$LCM_IF" multicast on
# 'replace' is idempotent, unlike the previous legacy 'route add' command.
"${SUDO[@]}" ip route replace 224.0.0.0/4 dev "$LCM_IF"

echo "[LCM] multicast route ready"
ip route get "$LCM_GROUP"
