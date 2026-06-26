"""AES67 SoC Wishbone/CSR peripheral modules.

Each peripheral lives in its own module and exposes both its Migen class and an
``add_*()`` helper that instantiates it on a SoC, registers any Wishbone slave /
IRQ, requests its pads and wires everything up.  ``soc.py`` composes the SoC by
calling these helpers, so peripherals are added (or omitted) with a single line.
"""

from .hyperram import AES67HyperRAM
from .aes67_csr import AES67CSRs, add_aes67_csr
from .eth_buffer import EthPacketBuffer, add_eth_buffer
from .stream_cfg import StreamConfigRAM, add_stream_cfg
from .spibone import SPIBone, add_spibone
from .uartbone import add_uartbone
from .wb_port import (
    add_external_wb_master, add_external_wb_slave,
    add_eth_irq_output, add_eth_irq_input,
)

__all__ = [
    "AES67HyperRAM",
    "AES67CSRs", "add_aes67_csr",
    "EthPacketBuffer", "add_eth_buffer",
    "StreamConfigRAM", "add_stream_cfg",
    "SPIBone", "add_spibone",
    "add_uartbone",
    "add_external_wb_master", "add_external_wb_slave",
    "add_eth_irq_output", "add_eth_irq_input",
]
