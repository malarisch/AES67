#
# This file is part of LiteX.
#
# Copyright (c) 2019 Sean Cross <sean@xobs.io>
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.fhdl.specials import Tristate, TSTriple
from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.soc.integration.doc import ModuleDoc, AutoDoc
from litex.soc.interconnect import wishbone, stream

# SPIBone Doc for 4, 3 and 2 wires modes  ----------------------------------------------------------

class SPI4WireDocumentation(ModuleDoc):
    """4-Wire SPI Protocol

    The 4-wire SPI protocol does not require any pins to change direction, and
    is therefore suitable for designs with level-shifters or without GPIOs that
    can change direction.
    
    While waiting for the response, the ``MISO`` line remains high.  As soon as
    a response is available, the device pulls the `MISO` line low and clocks
    out either a ``0x00`` or `0x01` byte indicating whether it's a READ or a WRITE
    that is being answered.  Note that if the operation is fast enough, the
    device will not pull the `MISO` line high and will immediately respond
    with ``0x00`` or ``0x01``.

    You can abort the operation by driving ``CS`` high.  However, if a WRITE or
    READ has already been initiated then it will not be aborted.

    .. wavedrom::
        :caption: 4-Wire SPI Operation

        { "signal": [
            ["Read",
                {  "name": 'MOSI',        "wave": 'x23...x|xxxxxx', "data": '0x01 [ADDRESS]'},
                {  "name": 'MISO',        "wave": 'x.....x|25...x', "data": '0x01 [DATA]'   },
                {  "name": 'CS',          "wave": 'x0.....|.....x', "data": '1 2 3'},
                {  "name": 'data bits',   "wave": 'xx2222x|x2222x', "data": '31:24 23:16 15:8 7:0 31:24 23:16 15:8 7:0'}
            ],
            {},
            ["Write",
                {  "name": 'MOSI',        "wave": 'x23...3...x|xx', "data": '0x00 [ADDRESS] [DATA]'},
                {  "name": 'MISO',        "wave": 'x.........1|2x', "data": '0x00'   },
                {  "name": 'CS',          "wave": 'x0.........|.x', "data": '1 2 3'},
                {  "name": 'data bits',   "wave": 'xx22222222x|xx', "data": '31:24 23:16 15:8 7:0 31:24 23:16 15:8 7:0'}
            ]
        ]}
    """

class SPI3WireDocumentation(ModuleDoc):
    """3-Wire SPI Protocol

    The 3-wire SPI protocol repurposes the ``MOSI`` line for both data input and
    data output.  The direction of the line changes immediately after the
    address (for read) or the data (for write) and the device starts writing
    ``0xFF``.

    As soon as data is available (read) or the data has been written (write),
    the device drives the ``MOSI`` line low in order to clock out ``0x00``
    or ``0x01``.  This will always happen on a byte boundary.

    You can abort the operation by driving ``CS`` high.  However, if a WRITE or
    READ has already been initiated then it will not be aborted.

    .. wavedrom::
        :caption: 3-Wire SPI Operation

        { "signal": [
            ["Read",
                {  "name": 'MOSI',        "wave": 'x23...5|55...x', "data": '0x01 [ADDRESS] 0xFF 0x01 [DATA]'},
                {  "name": 'CS',          "wave": 'x0.....|.....x', "data": '1 2 3'},
                {  "name": 'data bits',   "wave": 'xx2222x|x2222x', "data": '31:24 23:16 15:8 7:0 31:24 23:16 15:8 7:0'}
            ],
            {},
            ["Write",
                {  "name": 'MOSI',        "wave": 'x23...3...5|50', "data": '0x00 [ADDRESS] [DATA] 0xFF 0x00'},
                {  "name": 'CS',          "wave": 'x0.........|.x', "data": '1 2 3'},
                {  "name": 'data bits',   "wave": 'xx22222222x|xx', "data": '31:24 23:16 15:8 7:0 31:24 23:16 15:8 7:0'}
            ]
        ]}
        """

class SPI2WireDocumentation(ModuleDoc):
    """2-Wire SPI Protocol

    The 2-wire SPI protocol removes the ``CS`` line in favor of a sync byte.
    Note that the 2-wire protocol has no way of interrupting communication,
    so if the bus locks up the device must be reset.  The direction of the
    data line changes immediately after the address (for read) or the data
    (for write) and the device starts writing ``0xFF``.

    As soon as data is available (read) or the data has been written (write),
    the device drives the ``MOSI`` line low in order to clock out ``0x00``
    or ``0x01``.  This will always happen on a byte boundary.

    All transactions begin with a sync byte of ``0xAB``.

    .. wavedrom::
        :caption: 2-Wire SPI Operation

        { "signal": [
            ["Write",
                {  "name": 'MOSI',        "wave": '223...5|55...', "data": '0xAB 0x01 [ADDRESS] 0xFF 0x01 [DATA]'},
                {  "name": 'data bits',   "wave": 'xx2222x|x2222', "data": '31:24 23:16 15:8 7:0 31:24 23:16 15:8 7:0'}
            ],
            {},
            ["Read",
                {  "name": 'MOSI',        "wave": '223...3...5|5', "data": '0xAB 0x00 [ADDRESS] [DATA] 0xFF 0x00'},
                {  "name": 'data bits',   "wave": 'xx22222222x|x', "data": '31:24 23:16 15:8 7:0 31:24 23:16 15:8 7:0'}
            ]
        ]}
        """

# SPIBone Core -------------------------------------------------------------------------------------

class SPIBone(LiteXModule, ModuleDoc):
    """Wishbone Bridge over SPI

    This module allows for accessing a Wishbone bridge over a {}-wire protocol.
    All operations occur on byte boundaries, and are big-endian.

    The device can take a variable amount of time to respond, so the host should
    continue polling after the operation begins.  If the Wishbone bus is
    particularly busy, such as during periods of heavy processing when the
    CPU's icache is empty, responses can take many thousands of cycles.

    The bridge core is designed to run at 1/4 the system clock.

    Burst mode (``with_burst``, AES67 extension)
    --------------------------------------------
    The stock single-word protocol pays a full command + 32-bit address per
    32-bit word, so streaming a buffer costs one SPI round-trip per word.  Two
    extra commands add the auto-incrementing bursts that uartbone has and stock
    spibone lacks, while leaving the ``0x00``/``0x01`` single-word framing
    byte-for-byte identical (so existing hosts keep working):

      - ``0x02`` burst write: ``[0x02][addr:32][count:16][data0:32]..[dataN:32]``.
        Each word is committed to ``addr + 4*i`` (word-incrementing) as it
        arrives; after the last word a single ``0x00`` ack byte is returned.
      - ``0x03`` burst read:  ``[0x03][addr:32][count:16]`` then, per word, the
        device drives ``MISO`` high until the read completes, clocks out a
        ``0x01`` sync byte and the 32-bit value (big-endian), and advances to
        ``addr + 4*i``.  The host scans for each sync just like a single read.

    ``count`` is the number of 32-bit words (1..65535), big-endian.  Burst write
    relies on the slave acking within one SPI bit period (true for on-chip RAM
    at the bridge's sys/4 rate); burst read is self-paced and tolerates any
    read latency by emitting more ``MISO``-high padding.
    """
    def __init__(self, pads, wires=4, with_tristate=True, with_burst=True):
        self.bus = bus = wishbone.Interface(address_width=32, data_width=32, addressing="word")

        # # #

        # Parameters.
        # -----------
        if wires not in [2, 3, 4]:
            raise ValueError("`wires` must be 2, 3, or 4")

        # Doc.
        # ----
        self.__doc__ = self.__doc__.format(wires)
        self.mod_doc = {
            4 : SPI4WireDocumentation(),
            3 : SPI3WireDocumentation(),
            2 : SPI2WireDocumentation(),
        }[wires]

        # SPI IOs.
        # -------
        clk     = Signal()
        cs_n    = Signal()
        mosi    = Signal()
        miso    = Signal()
        miso_en = Signal()

        # Clk (Resynchronize).
        self.specials += MultiReg(pads.clk, clk)

        # CSn (Resynchronize).
        if wires in [3, 4]:
            self.specials += MultiReg(pads.cs_n, cs_n)

        # MOSI/MISO (Resynchronize + Tristate)
        if wires in [2, 3]:
            io = TSTriple()
            self.specials += io.get_tristate(pads.mosi)
            self.specials += MultiReg(io.i, mosi)
            self.comb += io.o.eq(miso)
            self.comb += io.oe.eq(miso_en)
        if wires in [4]:
            self.specials += MultiReg(pads.mosi, mosi)
            if with_tristate:
                self.specials += Tristate(pads.miso, miso, ~cs_n)
            else:
                self.comb += pads.miso.eq(miso)

        # Clk Edges Detection.
        # --------------------
        clk_d       = Signal()
        clk_negedge = Signal()
        clk_posedge = Signal()
        self.sync += clk_d.eq(clk)
        self.comb += clk_posedge.eq( clk & ~clk_d)
        self.comb += clk_negedge.eq(~clk &  clk_d)

        # Signals.
        # --------
        count   = Signal(8)
        offset  = Signal(5)
        synchro = Signal(8)
        command = Signal(8)
        address = Signal(32)
        data    = Signal(32)
        write   = Signal()

        # Burst-mode state (only driven when with_burst).
        burst   = Signal()       # current transaction is a burst (cmd 0x02/0x03)
        blen    = Signal(16)     # remaining 32-bit words in the burst
        wbit    = Signal(6)      # bit counter within the current burst-write word

        # FSM.
        # ----
        fsm = FSM(reset_state="IDLE")
        fsm = ResetInserter()(fsm)
        self.submodules += fsm
        self.comb += fsm.reset.eq(cs_n)

        # Connect the Wishbone bus up to our datas.
        self.comb += [
            bus.adr.eq(address[2:]),
            bus.dat_w.eq(data),
            bus.sel.eq(2**len(bus.sel) - 1)
        ]

        # Constantly have the count increase, except when it's reset in the IDLE state.
        self.sync += [
            If(cs_n,
                count.eq(0)
            ).Elif(clk_posedge,
                count.eq(count + 1)
            )
        ]

        if wires in [2]:
            fsm.act("IDLE",
                miso_en.eq(0),
                NextValue(miso, 1),
                If(clk_posedge,
                    NextValue(synchro, Cat(mosi, synchro))
                ),
                If(synchro[0:7] == 0b101011,
                    NextState("GET-COMMAND"),
                    NextValue(count, 0),
                    NextValue(command, mosi)
                )
            )
        if wires in [3, 4]:
            fsm.act("IDLE",
                miso_en.eq(0),
                NextValue(miso, 1),
                If(clk_posedge,
                    NextState("GET-COMMAND"),
                    NextValue(command, mosi)
                )
            )

        # Determine the transaction type from the command byte:
        #   0x00 single write   0x01 single read
        #   0x02 burst  write   0x03 burst  read   (only when with_burst)
        # Burst commands set `burst` so GET-ADDRESS collects a word count next.
        if with_burst:
            command_decode = If(command == 0,
                NextValue(write, 1),
                NextValue(burst, 0),
                NextState("GET-ADDRESS")
            ).Elif(command == 1,
                NextValue(write, 0),
                NextValue(burst, 0),
                NextState("GET-ADDRESS")
            ).Elif(command == 2,
                NextValue(write, 1),
                NextValue(burst, 1),
                NextState("GET-ADDRESS")
            ).Elif(command == 3,
                NextValue(write, 0),
                NextValue(burst, 1),
                NextState("GET-ADDRESS")
            ).Else(
                NextState("END")
            )
        else:
            command_decode = If(command == 0,
                NextValue(write, 1),
                NextState("GET-ADDRESS")
            ).Elif(command == 1,
                NextValue(write, 0),
                NextState("GET-ADDRESS")
            ).Else(
                NextState("END")
            )

        fsm.act("GET-COMMAND",
            miso_en.eq(0),
            NextValue(miso, 1),
            If(count == 8,
                command_decode,
            ),
            If(clk_posedge,
                NextValue(command, Cat(mosi, command))
            )
        )

        if with_burst:
            address_done = If(burst,
                NextState("GET-BURST-COUNT"),
            ).Elif(write,
                NextState("GET-DATA"),
            ).Else(
                NextState("BUS-MMAP-READ"),
            )
        else:
            address_done = If(write,
                NextState("GET-DATA"),
            ).Else(
                NextState("BUS-MMAP-READ"),
            )

        fsm.act("GET-ADDRESS",
            miso_en.eq(0),
            If(count == (32 + 8),
                address_done,
            ),
            If(clk_posedge,
                NextValue(address, Cat(mosi, address))
            )
        )

        fsm.act("GET-DATA",
            miso_en.eq(0),
            If(count == (32 + 32 + 8),
                NextState("BUS-MMAP-WRITE"),
            ),
            If(clk_posedge,
                NextValue(data, Cat(mosi, data))
            )
        )

        fsm.act("BUS-MMAP-WRITE",
            bus.stb.eq(1),
            bus.we.eq(1),
            bus.cyc.eq(1),
            miso_en.eq(1),
            If(bus.ack | bus.err,
                NextState("WAIT-BYTE-BOUNDARY")
            )
        )

        fsm.act("BUS-MMAP-READ",
            bus.stb.eq(1),
            bus.we.eq(0),
            bus.cyc.eq(1),
            miso_en.eq(1),
            If(bus.ack | bus.err,
                NextState("WAIT-BYTE-BOUNDARY"),
                NextValue(data, bus.dat_r)
            )
        )

        fsm.act("WAIT-BYTE-BOUNDARY",
            miso_en.eq(1),
            If(clk_negedge,
                If(count[0:3] == 0,
                    NextValue(miso, 0),
                    # For writes, fill in the 0 byte response
                    If(write,
                        NextState("WRITE-WR-RESPONSE"),
                    ).Else(
                        NextState("WRITE-RESPONSE"),
                    )
                )
            )
        )

        # Write the "01" byte that indicates a response
        fsm.act("WRITE-RESPONSE",
            miso_en.eq(1),
            If(clk_negedge,
                If(count[0:3] == 0b111,
                    NextValue(miso, 1),
                ).Elif(count[0:3] == 0,
                    NextValue(offset, 31),
                    NextState("WRITE-DATA")
                )
            )
        )

        # Write the actual data.  On a burst read, instead of ending after the
        # word, bump to the next (word-incrementing) address and loop back to
        # read+stream it; keep MISO high (0xff) through the next read's latency
        # so the host's sync scan skips it just like the first word.
        if with_burst:
            data_done = If((burst == 1) & (blen != 1),
                NextValue(miso, 1),
                NextValue(address, address + 4),
                NextValue(blen, blen - 1),
                NextState("BUS-MMAP-READ"),
            ).Else(
                NextValue(miso, 0),
                NextState("END"),
            )
        else:
            data_done = [
                NextValue(miso, 0),
                NextState("END"),
            ]

        fsm.act("WRITE-DATA",
            miso_en.eq(1),
            NextValue(miso, data >> offset),
            If(clk_negedge,
                NextValue(offset, offset - 1),
                If(offset == 0,
                    data_done,
                )
            )
        )

        fsm.act("WRITE-WR-RESPONSE",
            miso_en.eq(1),
            If(clk_negedge,
                If(count[0:3] == 0,
                    NextState("END")
                )
            )
        )

        if wires in [2]:
            fsm.act("END",
                miso_en.eq(0),
                NextValue(synchro, 0),
                NextState("IDLE")
            )
        if wires in [3, 4]:
            fsm.act("END",
                miso_en.eq(1)
            )

        # Burst states (auto-incrementing multi-word transfers).
        # ------------------------------------------------------
        if with_burst:
            # Collect the 16-bit word count (big-endian, MSB first), then enter
            # the write- or read-burst datapath.  At this point address holds the
            # base (byte) address; bus.adr drops its low two bits as usual.
            fsm.act("GET-BURST-COUNT",
                miso_en.eq(0),
                If(count == (32 + 8 + 16),
                    NextValue(wbit, 0),
                    If(write,
                        NextState("GET-BURST-DATA"),
                    ).Else(
                        # Reads reuse the single-word read datapath; WRITE-DATA
                        # loops back here-equivalent until blen words are sent.
                        NextState("BUS-MMAP-READ"),
                    )
                ),
                If(clk_posedge,
                    NextValue(blen, Cat(mosi, blen))
                )
            )

            # Burst write: clock in one 32-bit word (MSB first), then commit it.
            fsm.act("GET-BURST-DATA",
                miso_en.eq(0),
                If(clk_posedge,
                    NextValue(data, Cat(mosi, data)),
                    If(wbit == 31,
                        NextValue(wbit, 0),
                        NextState("BURST-WRITE"),
                    ).Else(
                        NextValue(wbit, wbit + 1),
                    )
                )
            )

            # Commit one burst word.  The slave must ack within one SPI bit
            # period so the next word's incoming bits are not missed (holds for
            # on-chip RAM at the bridge's sys/4 rate).  After the last word, emit
            # the single 0x00 ack via the shared write-response path.
            fsm.act("BURST-WRITE",
                miso_en.eq(0),
                bus.stb.eq(1),
                bus.we.eq(1),
                bus.cyc.eq(1),
                If(bus.ack | bus.err,
                    NextValue(address, address + 4),
                    If(blen == 1,
                        NextState("WAIT-BYTE-BOUNDARY"),
                    ).Else(
                        NextValue(blen, blen - 1),
                        NextState("GET-BURST-DATA"),
                    )
                )
            )
