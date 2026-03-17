#!/usr/bin/env python3
"""
Combine boot_stub.bin + bios.bin into a single flash image.

Flash layout:
  0x000 .. 0x1FF  boot_stub (512 bytes, padded)
  0x200 .. 0xXXX  bios.bin
"""

import sys
import os

STUB_SIZE = 0x200  # 512 bytes

script_dir = os.path.dirname(os.path.abspath(__file__))
stub_path = os.path.join(script_dir, "boot_stub.bin")
bios_path = os.path.join(script_dir, "..", "build", "software", "bios", "bios.bin")
output_path = os.path.join(script_dir, "..", "build", "flash_image.bin")

# Allow overriding paths via command line
if len(sys.argv) >= 2:
    bios_path = sys.argv[1]
if len(sys.argv) >= 3:
    output_path = sys.argv[2]

# Read boot stub
with open(stub_path, "rb") as f:
    stub = f.read()

if len(stub) > STUB_SIZE:
    print(f"ERROR: boot_stub.bin is {len(stub)} bytes (max {STUB_SIZE})")
    sys.exit(1)

# Pad stub to STUB_SIZE
stub = stub + b'\xff' * (STUB_SIZE - len(stub))

# Read BIOS
with open(bios_path, "rb") as f:
    bios = f.read()

# Combine
image = stub + bios

with open(output_path, "wb") as f:
    f.write(image)

print(f"Flash image: {output_path}")
print(f"  boot_stub: {len(stub):6d} bytes @ 0x000")
print(f"  bios.bin:  {len(bios):6d} bytes @ 0x{STUB_SIZE:03X}")
print(f"  total:     {len(image):6d} bytes")
