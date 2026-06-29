"""AES67 VexRiscv SoC generation package.

Modularised from the former monolithic ``generate.py``:
  - :mod:`aes67_soc.platform`     — IO definitions + HDL-only build stubs
  - :mod:`aes67_soc.crg`          — per-target Clock-Reset Generators
  - :mod:`aes67_soc.peripherals`  — Wishbone/CSR peripheral modules (one per file)
  - :mod:`aes67_soc.soc`          — SoC top-level assembly (AES67SoC)
"""

from .soc import AES67SoC
from .platform import (
    Cyclone10StubPlatform,
    Cyc1000StubPlatform,
    GowinStubPlatform,
    SpiboneStubPlatform,
    UartboneStubPlatform,
    BridgeStubPlatform,
    SpiboneLatticeStubPlatform,
    UartboneLatticeStubPlatform,
    BridgeLatticeStubPlatform,
)

__all__ = [
    "AES67SoC",
    "Cyclone10StubPlatform",
    "Cyc1000StubPlatform",
    "GowinStubPlatform",
    "SpiboneStubPlatform",
    "UartboneStubPlatform",
    "BridgeStubPlatform",
    "SpiboneLatticeStubPlatform",
    "UartboneLatticeStubPlatform",
    "BridgeLatticeStubPlatform",
]
