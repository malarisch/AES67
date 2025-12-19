#!/usr/bin/env python3
"""
Periodic broadcast packet sender for quick link testing.

Sends UDP broadcast frames to FF:FF:FF:FF:FF:FF / 255.255.255.255 with a payload
length that increments each packet and wraps at 1500 bytes.
"""

import argparse
import socket
import struct
import time


def make_socket(iface: str, broadcast_addr: str) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    if iface:
        # Bind to interface so packets egress the requested port.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, iface.encode())
    # Bind to wildcard so we have a local port.
    sock.bind(("", 0))
    sock.connect((broadcast_addr, 4000))
    return sock


def main() -> None:
    parser = argparse.ArgumentParser(description="Send periodic broadcast packets with ascending length (wrap at 1500).")
    parser.add_argument("--iface", required=True, help="Outgoing network interface (e.g. eth0)")
    parser.add_argument("--broadcast", default="255.255.255.255", help="Broadcast IPv4 address (default: %(default)s)")
    parser.add_argument("--interval", type=float, default=0.05, help="Interval between packets in seconds (default: %(default)s)")
    parser.add_argument("--start-len", type=int, default=1, help="Starting payload length (default: %(default)s)")
    parser.add_argument("--max-len", type=int, default=1500, help="Wrap payload length after this value (default: %(default)s)")
    args = parser.parse_args()

    if args.start_len < 1 or args.max_len < 1:
        raise SystemExit("Lengths must be positive.")
    if args.start_len > args.max_len:
        raise SystemExit("start-len must be <= max-len.")

    sock = make_socket(args.iface, args.broadcast)

    length = args.start_len
    counter = 0
    try:
        while True:
            # Payload: 4-byte counter + padding pattern.
            payload = struct.pack("!I", counter) + bytes((i & 0xFF) for i in range(length - 4 if length >= 4 else 0))
            sock.send(payload)
            counter += 1
            length += 1
            if length > args.max_len:
                length = args.start_len
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
