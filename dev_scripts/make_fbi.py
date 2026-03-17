#!/usr/bin/env python3
"""
Wrap a raw binary into a LiteX Flash Boot Image (.fbi).

Prepends an 8-byte header (little-endian uint32 length + uint32 CRC-32)
that the LiteX BIOS expects at FLASH_BOOT_ADDRESS.

Usage:
    python make_fbi.py firmware.bin [firmware.fbi]
"""

import struct
import binascii
import sys
import os

if len(sys.argv) < 2:
    print(f"Usage: {sys.argv[0]} <input.bin> [output.fbi]")
    sys.exit(1)

input_path = sys.argv[1]
output_path = sys.argv[2] if len(sys.argv) >= 3 else os.path.splitext(input_path)[0] + ".fbi"

with open(input_path, "rb") as f:
    payload = f.read()

crc = binascii.crc32(payload) & 0xFFFFFFFF
header = struct.pack("<II", len(payload), crc)

with open(output_path, "wb") as f:
    f.write(header + payload)

print(f"{input_path} -> {output_path}")
print(f"  payload: {len(payload)} bytes")
print(f"  crc32:   0x{crc:08X}")
print(f"  total:   {len(header) + len(payload)} bytes")
