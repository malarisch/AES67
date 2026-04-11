#!/usr/bin/env python3
"""
Combine a target-specific boot_stub_<target>.bin + bios.bin into a flash image.

Flash layout:
  0x000 .. 0x1FF  boot_stub (512 bytes, padded)
  0x200 .. 0xXXX  bios.bin

Usage:
  make_flash_image.py [--target TARGET] [bios.bin] [flash_image.bin]

Defaults:
  --target          cyclone10
  bios.bin          ../build/<target>/software/bios/bios.bin
  flash_image.bin   ../build/<target>/flash_image.bin
"""

import argparse
import os
import sys

STUB_SIZE = 0x200  # 512 bytes

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BUILD_DIR  = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "build"))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--target", default="cyclone10",
                    help="LiteX SoC target (cyclone10|cyc1000|gowin). Selects "
                         "boot_stub_<target>.bin and default bios/output paths.")
    ap.add_argument("bios", nargs="?", default=None,
                    help="BIOS binary (default: build/<target>/software/bios/bios.bin)")
    ap.add_argument("output", nargs="?", default=None,
                    help="Output flash image (default: build/<target>/flash_image.bin)")
    args = ap.parse_args()

    stub_path   = os.path.join(SCRIPT_DIR, f"boot_stub_{args.target}.bin")
    bios_path   = args.bios   or os.path.join(BUILD_DIR, args.target, "software", "bios", "bios.bin")
    output_path = args.output or os.path.join(BUILD_DIR, args.target, "flash_image.bin")

    if not os.path.exists(stub_path):
        print(f"ERROR: {stub_path} not found. "
              f"Run `make` in {SCRIPT_DIR} to build all boot stub variants.",
              file=sys.stderr)
        sys.exit(1)
    if not os.path.exists(bios_path):
        print(f"ERROR: BIOS not found at {bios_path}", file=sys.stderr)
        sys.exit(1)

    with open(stub_path, "rb") as f:
        stub = f.read()
    if len(stub) > STUB_SIZE:
        print(f"ERROR: {stub_path} is {len(stub)} bytes (max {STUB_SIZE})", file=sys.stderr)
        sys.exit(1)
    stub = stub + b'\xff' * (STUB_SIZE - len(stub))

    with open(bios_path, "rb") as f:
        bios = f.read()

    image = stub + bios

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "wb") as f:
        f.write(image)

    print(f"Flash image: {output_path}")
    print(f"  target:    {args.target}")
    print(f"  boot_stub: {len(stub):6d} bytes @ 0x000  ({os.path.basename(stub_path)})")
    print(f"  bios.bin:  {len(bios):6d} bytes @ 0x{STUB_SIZE:03X}")
    print(f"  total:     {len(image):6d} bytes")


if __name__ == "__main__":
    main()
