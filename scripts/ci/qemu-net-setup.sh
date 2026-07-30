#!/bin/sh
# Create the "zeth" TAP interface the Zephyr QEMU targets attach to
# (same convention as zephyr net-tools' net-setup.sh, without needing
# that repo). Run as root:
#
#   sudo scripts/ci/qemu-net-setup.sh          # create + configure
#   sudo scripts/ci/qemu-net-setup.sh down     # tear down
#
# The firmware has no DHCP server on this segment and adopts its
# MAC-derived link-local address (169.254.x.y) after the 30 s DHCP
# grace period; the host route below makes that reachable.
set -eu

IFACE="${ZETH_IFACE:-zeth}"
# The node adopts a MAC-derived 169.254.x.y identity (no DHCP server on
# this segment) and has no gateway: the host must talk to it from a
# link-local source address, or the node cannot route the replies. This
# must also be the FIRST (primary) address — nmos-testing binds its mock
# registry to the first IPv4 of the interface, and the node has to be
# able to reach it.
HOST_ADDR="${ZETH_HOST_ADDR:-169.254.152.2/16}"
TAP_USER="${SUDO_USER:-$(id -un)}"

if [ "${1:-up}" = "down" ]; then
    ip link del "$IFACE" 2>/dev/null || true
    exit 0
fi

# Creating and configuring a TAP device needs CAP_NET_ADMIN. Containers
# (act, plain docker) usually lack it, and the failure mode further down
# is a bare "Operation not permitted" from ip(8) — say so up front.
ip link del "${IFACE}-probe" 2>/dev/null || true
if ! ip tuntap add "${IFACE}-probe" mode tap 2>/dev/null; then
    echo "error: cannot create TAP devices (CAP_NET_ADMIN missing?)." >&2
    echo "       In a container, add: --cap-add=NET_ADMIN --device=/dev/net/tun" >&2
    exit 1
fi
ip link del "${IFACE}-probe"

if ! ip link show "$IFACE" >/dev/null 2>&1; then
    ip tuntap add "$IFACE" mode tap user "$TAP_USER"
fi
ip addr flush dev "$IFACE"
ip addr add "$HOST_ADDR" dev "$IFACE"
ip link set "$IFACE" up

echo "$IFACE up ($HOST_ADDR)"
