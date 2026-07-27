"""Application-specific AES67 control/status registers (CSR-only, no Wishbone slave)."""

from litex.gen import *

from migen import Cat, If
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus, CSRField


# -- Custom CSRs -------------------------------------------------------------

class AES67CSRs(LiteXModule, AutoCSR):
    """Application-specific control/status registers for AES67 FPGA logic.

    ``with_servo`` / ``with_metering`` gate whole groups of CSRs.  When the FPGA
    statically configures the PTP servo (via a VHDL generic) or disables audio
    metering, those CSRs are dead weight, so they can be dropped to save fabric.
    The *ports* (the ``o_*`` / ``i_*`` signals, hence the ``aes67_ctrl`` pads)
    are still declared so the board top-level wires unchanged and Quartus does
    not complain; the now-unused outputs are simply left undriven (constant 0)
    and the unused inputs dangling, both optimized away during synthesis.
    """
    def __init__(self, with_servo=True, with_metering=True):
        self.with_servo    = with_servo
        self.with_metering = with_metering

        # =====================================================================
        # Input signals (directly active from external FPGA logic)
        # =====================================================================
        self.i_pll_ppb_valid       = Signal()
        self.i_pll_ppb_wc_count    = Signal(25)
        self.i_pll_ppb_pll_count   = Signal(25)
        self.i_wallclock_locked    = Signal()
        self.i_wallclock_configured = Signal()
        self.i_eth_link_up         = Signal()
        self.i_eth_speed           = Signal(2)
        self.i_ptp_path_delay      = Signal(32)
        self.i_ptp_offset          = Signal(32)
        self.i_eth_tx_done         = Signal()
        self.i_eth_rx_overflow     = Signal()
        # RX stream-underrun diagnostics (stream_underrun / mute_channels, LSBs)
        self.i_rx_underrun         = Signal(4)
        self.i_rx_mute             = Signal(8)
        # PTP BMA results (from FPGA BMC)
        self.i_ptp_is_leader       = Signal()
        self.i_ptp_is_follower     = Signal()
        self.i_ptp_leader_id       = Signal(64)
        self.i_rx_meter_signal     = Signal(16)
        self.i_rx_meter_clip       = Signal(16)
        self.i_tx_meter_signal     = Signal(16)
        self.i_tx_meter_clip       = Signal(16)
        # PTP-in-software wallclock interface (FPGA -> SoC)
        self.i_wallclock_seconds_in     = Signal(48)
        self.i_wallclock_nanoseconds_in = Signal(30)
        self.i_tx_timestamp_sec_in      = Signal(4)
        self.i_tx_timestamp_nsec_in     = Signal(30)
        # Static FPGA build configuration (system_cfg_pkg.vhd's
        # system_cfg_to_vector(syscfg), 72 bits, constant per bitstream)
        self.i_system_cfg               = Signal(72)

        # =====================================================================
        # Output signals (directly active to external FPGA logic)
        # =====================================================================
        self.o_pll_ppb_start       = Signal()
        self.o_mac_addr            = Signal(48)
        self.o_ip_addr             = Signal(32)
        self.o_ptp_time_source     = Signal(8)
        self.o_ptp_log_msg_interval = Signal(8)
        self.o_ptp_announce_msg_interval = Signal(8)
        self.o_ptp_gm_priority1    = Signal(8)
        self.o_ptp_gm_priority2    = Signal(8)
        self.o_ptp_gm_clock_class  = Signal(8)
        self.o_ptp_gm_clock_accuracy = Signal(8)
        self.o_eth_tx_request      = Signal()
        self.o_adda_nrst           = Signal()  # AD/DA card hardware reset (active-high → nRST pin)
        self.o_meter_clear         = Signal()  # Metering clear toggle (to FPGA)
        # Reset outputs (active high), all driven by the single "reset" CSR.
        self.o_ptp_reset           = Signal()  # PTP module + wallclock reset
        self.o_tx_reset            = Signal()  # audio TX path reset
        self.o_rx_reset            = Signal()  # audio RX path reset
        self.o_eth_reset           = Signal()  # Ethernet MAC/PHY path reset

        # PTP servo / parser tuning (SoC -> FPGA)
        self.o_servo_kp_gain              = Signal(8)
        self.o_servo_ki_gain              = Signal(8)
        self.o_servo_gain_shift           = Signal(5)
        self.o_servo_gain_shift_locked    = Signal(5)
        self.o_servo_ki_extra_shift       = Signal(5)
        self.o_servo_filter_shift         = Signal(5)
        self.o_servo_warmup_samples       = Signal(8)
        self.o_servo_lock_threshold_ns    = Signal(32)
        self.o_servo_unlock_threshold_ns  = Signal(32)
        self.o_servo_lock_count_threshold = Signal(8)
        self.o_parser_min_filter_enable        = Signal()
        self.o_parser_min_filter_active_depth  = Signal(8)
        self.o_parser_delay_asymmetry_ns       = Signal(32)

        # PTP servo monitoring (FPGA -> SoC)
        self.i_servo_mon_filtered_offset      = Signal(32)
        self.i_servo_mon_integral_sum         = Signal(32)
        self.i_servo_mon_pi_proportional      = Signal(32)
        self.i_servo_mon_pi_sum_raw           = Signal(32)
        self.i_servo_mon_effective_gain_shift = Signal(8)
        self.i_servo_mon_lock_counter         = Signal(16)
        self.i_servo_mon_sample_count         = Signal(16)
        self.i_servo_mon_first_lock_achieved  = Signal()

        # PTP-in-software wallclock control (SoC -> FPGA)
        self.o_wallclock_seconds_out     = Signal(48)
        self.o_wallclock_nanoseconds_out = Signal(30)
        self.o_wallclock_set             = Signal()
        self.o_wallclock_phasejump       = Signal()
        self.o_wallclock_ppb             = Signal(20)
        # Software-smoothed NCO (media clock) correction (SoC -> FPGA)
        self.o_nco_ppb_adj               = Signal(48)
        self.o_nco_ppb_adj_valid         = Signal()

        # =====================================================================
        # CSR: Status registers (RO) — directly sampled from input signals
        # =====================================================================
        self.pll_ppb_status = CSRStatus(32, fields=[
            CSRField("valid",     size=1,  offset=0,  description="PPB measurement valid"),
        ])
        self.pll_ppb_wc_count = CSRStatus(32, description="PPB measurement wallclock count [24:0]")
        self.pll_ppb_pll_count = CSRStatus(32, description="PPB measurement PLL count [24:0]")

        # NOTE: bits 1 (was wallclock_phasejump) and 3 (was ptp_sync_lost) are
        # reserved gaps — those functions were removed.  The remaining fields
        # keep their original offsets so firmware reading the other status bits
        # is unaffected.
        #
        # wallclock_locked is only meaningful with the FPGA (hardware) PTP servo;
        # a --ptp-in-software gateware ties it to 0 and the host uses its own
        # servo state instead.
        self.status = CSRStatus(32, fields=[
            CSRField("wallclock_locked",    size=1, offset=0, description="Wallclock locked (hardware PTP servo only; 0 in software-PTP gateware)"),
            CSRField("wallclock_configured", size=1, offset=2, description="Wallclock configured"),
            CSRField("eth_link_up",         size=1, offset=4, description="Ethernet link up"),
            CSRField("eth_speed",           size=2, offset=5, description="Ethernet speed (00=10M,01=100M,10=1G)"),
            CSRField("eth_tx_done",         size=1, offset=7, description="Ethernet TX done"),
            CSRField("eth_rx_overflow",     size=1, offset=8, description="Ethernet RX overflow"),
            CSRField("ptp_is_leader",       size=1, offset=9, description="PTP BMA result: node is leader"),
            CSRField("ptp_is_follower",     size=1, offset=10, description="PTP BMA result: node is follower"),
            # RX stream-underrun diagnostics (bits 16-27), e.g. for web-UI
            # monitoring; adding fields does not move the register, existing
            # readers see them as previously-zero bits.
            CSRField("rx_underrun",         size=4, offset=16, description="RX stream underrun flags (streams 3..0)"),
            CSRField("rx_mute",             size=8, offset=20, description="RX underrun-muted channels (channels 7..0)"),
        ])

        # PTP path delay / offset from master as computed by the FPGA PTP servo.
        # Always present so the CSR layout is identical for hardware- and
        # software-PTP gateware (a --ptp-in-software FPGA feeds constant 0 and
        # the host sources these from its own PTP stack).  The map used to
        # change with the PTP mode, which silently broke hosts built against
        # the other variant's headers.
        self.ptp_path_delay = CSRStatus(32, description="PTP path delay (ns, hardware PTP servo; 0 in software-PTP gateware)")
        self.ptp_offset     = CSRStatus(32, description="PTP offset from master (ns, hardware PTP servo; 0 in software-PTP gateware)")

        # =====================================================================
        # CSR: Control registers (RW) — directly drive output signals
        # =====================================================================
        self.ctrl = CSRStorage(32, fields=[
            CSRField("pll_ppb_start",   size=1, offset=0, description="Start PPB measurement"),
            CSRField("eth_tx_request",  size=1, offset=3, description="Request Ethernet TX"),
            CSRField("adda_nrst",       size=1, offset=4, description="AD/DA card reset release: 0=held in reset, 1=released"),
        ])

        self.mac_addr_lo = CSRStorage(32, description="MAC address [31:0]")
        self.mac_addr_hi = CSRStorage(32, description="MAC address [47:32] (bits [15:0] used)")
        self.ip_addr     = CSRStorage(32, description="IP address [31:0]")

        self.ptp_leader_id_lo = CSRStatus(32, description="PTP leader identity [31:0] (from FPGA BMA)")
        self.ptp_leader_id_hi = CSRStatus(32, description="PTP leader identity [63:32] (from FPGA BMA)")

        self.ptp_time_source = CSRStorage(8, description="PTP time source")
        self.ptp_log_msg_interval = CSRStorage(8, description="PTP logMessageInterval")
        self.ptp_announce_msg_interval = CSRStorage(8, description="PTP announce logMessageInterval")

        self.ptp_gm_priority1     = CSRStorage(8, description="PTP GM priority1")
        self.ptp_gm_priority2     = CSRStorage(8, description="PTP GM priority2")
        self.ptp_gm_clock_class   = CSRStorage(8, description="PTP GM clock class")
        self.ptp_gm_clock_accuracy = CSRStorage(8, description="PTP GM clock accuracy")

        # -- Audio metering CSRs (RO status + RW clear) --
        # Dropped entirely when metering is disabled in the FPGA (the i_*meter*
        # inputs / o_meter_clear output stay declared and get optimized away).
        if with_metering:
            self.rx_meter_signal = CSRStatus(16, description="RX signal detect (1 bit per channel, sticky)")
            self.rx_meter_clip   = CSRStatus(16, description="RX clip detect (1 bit per channel, sticky)")
            self.tx_meter_signal = CSRStatus(16, description="TX signal detect (1 bit per channel, sticky)")
            self.tx_meter_clip   = CSRStatus(16, description="TX clip detect (1 bit per channel, sticky)")
            self.meter_clear     = CSRStorage(1, description="Toggle to clear metering sticky bits in FPGA")

        # -- Scratch register (RW, no HW connection) --
        self.scratch = CSRStorage(32, description="Scratch register (read/write, no HW effect)")

        # =====================================================================
        # PTP servo tuning + monitoring CSRs — dropped when the FPGA configures
        # the servo statically via a VHDL generic.  The o_servo_* outputs / the
        # i_servo_mon_* inputs stay declared (constant 0 / dangling) so the pads
        # remain and are optimized away.  Parser tuning below is independent and
        # always present.
        # =====================================================================
        if with_servo:
            self.servo_kp_gain              = CSRStorage(8,  reset=40,   description="Servo Kp gain (signed)")
            self.servo_ki_gain              = CSRStorage(8,  reset=5,    description="Servo Ki gain (signed)")
            self.servo_gain_shift           = CSRStorage(5,  reset=3,    description="Servo base gain shift")
            self.servo_gain_shift_locked    = CSRStorage(5,  reset=0,    description="Servo extra gain shift when locked")
            self.servo_ki_extra_shift       = CSRStorage(5,  reset=3,    description="Servo Ki extra shift relative to Kp")
            self.servo_filter_shift         = CSRStorage(5,  reset=0,    description="Servo offset EMA filter shift (0=no filter)")
            self.servo_warmup_samples       = CSRStorage(8,  reset=16,   description="Servo warmup sample count")
            self.servo_lock_threshold_ns    = CSRStorage(32, reset=500,  description="Servo lock threshold (ns)")
            self.servo_unlock_threshold_ns  = CSRStorage(32, reset=5000, description="Servo unlock threshold (ns)")
            self.servo_lock_count_threshold = CSRStorage(8,  reset=24,   description="Servo consecutive samples needed to lock")

        self.parser_min_filter = CSRStorage(32, reset=(2 << 8), fields=[
            CSRField("enable",       size=1, offset=0,  description="Enable min filter"),
            CSRField("active_depth", size=8, offset=8,  description="Active min filter depth (clamped to MIN_FILTER_DEPTH)"),
        ])

        # IEEE 1588 delayAsymmetry (signed ns). Positive = downstream
        # (Master->Slave) path is longer than upstream. Used to compensate
        # static PHY/MAC TX vs RX latency mismatch (e.g. LAN8720A).
        self.parser_delay_asymmetry_ns = CSRStorage(32, reset=0,
            description="PTP delayAsymmetry (signed ns); positive = M2S path longer than S2M")

        # Unified reset register: one CSR, one bit per resettable domain (active
        # high, held while the bit is set).  Replaces the former standalone
        # ptp_reset register.
        self.reset = CSRStorage(32, fields=[
            CSRField("ptp", size=1, offset=0, reset=1, description="Reset PTP module + wallclock (1 = held in reset)"),
            CSRField("tx",  size=1, offset=1, reset=1, description="Reset audio TX path (1 = held in reset)"),
            CSRField("rx",  size=1, offset=2, reset=1, description="Reset audio RX path (1 = held in reset)"),
            CSRField("eth", size=1, offset=3, reset=1, description="Reset Ethernet MAC/PHY path (1 = held in reset)"),
        ])

        # =====================================================================
        # PTP servo monitoring CSRs (RO) — same gate as the servo tuning above.
        # =====================================================================
        if with_servo:
            self.servo_mon_filtered_offset      = CSRStatus(32, description="Servo: filtered offset (signed ns)")
            self.servo_mon_integral_sum         = CSRStatus(32, description="Servo: PI integrator state (signed)")
            self.servo_mon_pi_proportional      = CSRStatus(32, description="Servo: PI proportional term (signed)")
            self.servo_mon_pi_sum_raw           = CSRStatus(32, description="Servo: PI raw sum pre-clamp (signed)")
            self.servo_mon_status = CSRStatus(32, fields=[
                CSRField("effective_gain_shift", size=8,  offset=0,  description="Effective gain shift"),
                CSRField("lock_counter",         size=16, offset=8,  description="Lock counter"),
                CSRField("first_lock_achieved",  size=1,  offset=24, description="First lock achieved"),
            ])
            self.servo_mon_sample_count = CSRStatus(16, description="Servo: warmup sample count")

        # =====================================================================
        # PTP-in-software CSRs — used when a host PTP stack disciplines the
        # wallclock (gateware ``PTP_IN_SOFTWARE = true``).  Always present so the
        # CSR layout does not depend on the PTP mode: hardware-PTP gateware
        # simply ignores the wallclock control outputs (the FPGA servo owns the
        # wallclock) and the snapshot inputs still read the live wallclock.
        # =====================================================================
        # Wallclock snapshot from FPGA (RO)
        self.wallclock_seconds_in_lo  = CSRStatus(32, description="Wallclock seconds [31:0] (FPGA -> SoC)")
        self.wallclock_seconds_in_hi  = CSRStatus(32, description="Wallclock seconds [47:32] (bits [15:0] used)")
        self.wallclock_nanoseconds_in = CSRStatus(32, description="Wallclock nanoseconds [29:0] (FPGA -> SoC)")
        self.tx_timestamp_sec_in      = CSRStatus(4,  description="TX timestamp seconds [3:0] (FPGA -> SoC)")
        self.tx_timestamp_nsec_in     = CSRStatus(32, description="TX timestamp nanoseconds [29:0] (FPGA -> SoC)")

        # Wallclock control to FPGA (RW)
        self.wallclock_seconds_out_lo  = CSRStorage(32, description="Wallclock set value seconds [31:0] (SoC -> FPGA)")
        self.wallclock_seconds_out_hi  = CSRStorage(32, description="Wallclock set value seconds [47:32] (bits [15:0] used)")
        self.wallclock_nanoseconds_out = CSRStorage(32, description="Wallclock set value nanoseconds [29:0] (SoC -> FPGA)")
        self.wallclock_ppb             = CSRStorage(20, description="Wallclock frequency correction (signed ppb, SoC -> FPGA)")
        self.wallclock_ctrl = CSRStorage(32, fields=[
            CSRField("set",       size=1, offset=0, description="Load wallclock_seconds_out/nanoseconds_out into the FPGA wallclock"),
            CSRField("phasejump", size=1, offset=1, description="Apply a one-shot phase jump to the FPGA wallclock"),
        ])

        # Appended after wallclock_ctrl: keeps all pre-existing CSR addresses
        # stable (old bitstreams stay compatible; they just ignore these).
        self.nco_ppb_adj_lo = CSRStorage(32, description="NCO (media clock) increment adjustment [31:0] (signed, 48-bit NCO increment units = ppb x 55340 @ 48k/125MHz; SoC -> FPGA). Write LO first, then HI — the value commits on the HI write.")
        self.nco_ppb_adj_hi = CSRStorage(32, description="NCO increment adjustment [47:32] (bits [15:0] used, sign extension; commits the value)")
        self.nco_ppb_adj_valid = CSRStorage(1, description="Write strobe: pulse 1 -> 0 AFTER writing nco_ppb_adj_lo/hi — the NCO captures the value on any edge.")

        # =====================================================================
        # Static FPGA build configuration (RO) — the syscfg generic the gateware
        # was built with (system_cfg_pkg.vhd, system_cfg_to_vector()).  Constant
        # per bitstream; the firmware reads it once at boot and starts the
        # matching services (hardware vs. software PTP, metering, stream/channel
        # limits) instead of baking the choice in at compile time.  Appended
        # after nco_ppb_adj_valid so every pre-existing CSR address is stable.
        # =====================================================================
        self.system_cfg_flags = CSRStatus(32, fields=[
            CSRField("ptp_in_software",   size=1, offset=0, description="Gateware built with PTP_IN_SOFTWARE (host PTP stack disciplines the wallclock; RX frames carry the timestamp trailer)"),
            CSRField("static_ptp_config", size=1, offset=1, description="Gateware built with STATIC_PTP_CONFIG (servo tuning fixed via generic)"),
            CSRField("metering",          size=1, offset=2, description="Gateware built with ENABLE_METERING (audio metering CSRs are live)"),
        ])
        self.system_cfg_rx = CSRStatus(32, fields=[
            CSRField("max_streams",  size=8,  offset=0,  description="RX/DA path: maximum RTP streams"),
            CSRField("channels",     size=8,  offset=8,  description="RX/DA path: audio channels"),
            CSRField("buffer_depth", size=16, offset=16, description="RX/DA path: ring buffer depth (samples)"),
        ])
        self.system_cfg_tx = CSRStatus(32, fields=[
            CSRField("max_streams",  size=8,  offset=0,  description="TX/AD path: maximum RTP streams"),
            CSRField("channels",     size=8,  offset=8,  description="TX/AD path: audio channels"),
            CSRField("buffer_depth", size=16, offset=16, description="TX/AD path: ring buffer depth (samples)"),
        ])

        # =====================================================================
        # Wiring: input signals -> CSR status fields
        # =====================================================================
        self.comb += [
            self.pll_ppb_status.fields.valid.eq(self.i_pll_ppb_valid),
            self.pll_ppb_wc_count.status.eq(self.i_pll_ppb_wc_count),
            self.pll_ppb_pll_count.status.eq(self.i_pll_ppb_pll_count),

            self.status.fields.wallclock_configured.eq(self.i_wallclock_configured),
            self.status.fields.eth_link_up.eq(self.i_eth_link_up),
            self.status.fields.eth_speed.eq(self.i_eth_speed),
            self.status.fields.eth_tx_done.eq(self.i_eth_tx_done),
            self.status.fields.eth_rx_overflow.eq(self.i_eth_rx_overflow),
            self.status.fields.ptp_is_leader.eq(self.i_ptp_is_leader),
            self.status.fields.ptp_is_follower.eq(self.i_ptp_is_follower),
            self.status.fields.rx_underrun.eq(self.i_rx_underrun),
            self.status.fields.rx_mute.eq(self.i_rx_mute),

            self.ptp_leader_id_lo.status.eq(self.i_ptp_leader_id[:32]),
            self.ptp_leader_id_hi.status.eq(self.i_ptp_leader_id[32:]),

            # PTP lock / path delay / offset readouts (constant 0 from a
            # software-PTP FPGA — the ports dangle there).
            self.status.fields.wallclock_locked.eq(self.i_wallclock_locked),
            self.ptp_path_delay.status.eq(self.i_ptp_path_delay),
            self.ptp_offset.status.eq(self.i_ptp_offset),

            # Static build configuration — bit layout per system_cfg_to_vector()
            # (system_cfg_pkg.vhd): [71]=ptp_in_software, [70]=static_ptp_config,
            # [69]=metering, [68:64]=reserved, [63:56]/[55:48]/[47:32]=RX
            # max_streams/channels/buffer_depth, [31:24]/[23:16]/[15:0]=TX dito.
            self.system_cfg_flags.fields.ptp_in_software.eq(self.i_system_cfg[71]),
            self.system_cfg_flags.fields.static_ptp_config.eq(self.i_system_cfg[70]),
            self.system_cfg_flags.fields.metering.eq(self.i_system_cfg[69]),
            self.system_cfg_rx.fields.max_streams.eq(self.i_system_cfg[56:64]),
            self.system_cfg_rx.fields.channels.eq(self.i_system_cfg[48:56]),
            self.system_cfg_rx.fields.buffer_depth.eq(self.i_system_cfg[32:48]),
            self.system_cfg_tx.fields.max_streams.eq(self.i_system_cfg[24:32]),
            self.system_cfg_tx.fields.channels.eq(self.i_system_cfg[16:24]),
            self.system_cfg_tx.fields.buffer_depth.eq(self.i_system_cfg[0:16]),
        ]

        # =====================================================================
        # Wiring: CSR storage fields -> output signals (always present)
        # =====================================================================
        self.comb += [
            self.o_pll_ppb_start.eq(self.ctrl.fields.pll_ppb_start),
            self.o_eth_tx_request.eq(self.ctrl.fields.eth_tx_request),
            self.o_adda_nrst.eq(self.ctrl.fields.adda_nrst),

            self.o_mac_addr.eq(Cat(self.mac_addr_lo.storage, self.mac_addr_hi.storage[:16])),
            self.o_ip_addr.eq(self.ip_addr.storage),

            self.o_ptp_time_source.eq(self.ptp_time_source.storage),
            self.o_ptp_log_msg_interval.eq(self.ptp_log_msg_interval.storage),
            self.o_ptp_announce_msg_interval.eq(self.ptp_announce_msg_interval.storage),

            self.o_ptp_gm_priority1.eq(self.ptp_gm_priority1.storage),
            self.o_ptp_gm_priority2.eq(self.ptp_gm_priority2.storage),
            self.o_ptp_gm_clock_class.eq(self.ptp_gm_clock_class.storage),
            self.o_ptp_gm_clock_accuracy.eq(self.ptp_gm_clock_accuracy.storage),

            # Unified reset CSR -> per-domain reset outputs
            self.o_ptp_reset.eq(self.reset.fields.ptp),
            self.o_tx_reset.eq(self.reset.fields.tx),
            self.o_rx_reset.eq(self.reset.fields.rx),
            self.o_eth_reset.eq(self.reset.fields.eth),

            # PTP parser tuning (always present) -> output signals
            self.o_parser_min_filter_enable.eq(self.parser_min_filter.fields.enable),
            self.o_parser_min_filter_active_depth.eq(self.parser_min_filter.fields.active_depth),
            self.o_parser_delay_asymmetry_ns.eq(self.parser_delay_asymmetry_ns.storage),
        ]

        # -- Audio metering wiring (only when metering CSRs exist) -------------
        if with_metering:
            self.comb += [
                self.rx_meter_signal.status.eq(self.i_rx_meter_signal),
                self.rx_meter_clip.status.eq(self.i_rx_meter_clip),
                self.tx_meter_signal.status.eq(self.i_tx_meter_signal),
                self.tx_meter_clip.status.eq(self.i_tx_meter_clip),
                self.o_meter_clear.eq(self.meter_clear.storage),
            ]

        # -- Servo tuning + monitoring wiring (only when servo CSRs exist) -----
        if with_servo:
            self.comb += [
                self.o_servo_kp_gain.eq(self.servo_kp_gain.storage),
                self.o_servo_ki_gain.eq(self.servo_ki_gain.storage),
                self.o_servo_gain_shift.eq(self.servo_gain_shift.storage),
                self.o_servo_gain_shift_locked.eq(self.servo_gain_shift_locked.storage),
                self.o_servo_ki_extra_shift.eq(self.servo_ki_extra_shift.storage),
                self.o_servo_filter_shift.eq(self.servo_filter_shift.storage),
                self.o_servo_warmup_samples.eq(self.servo_warmup_samples.storage),
                self.o_servo_lock_threshold_ns.eq(self.servo_lock_threshold_ns.storage),
                self.o_servo_unlock_threshold_ns.eq(self.servo_unlock_threshold_ns.storage),
                self.o_servo_lock_count_threshold.eq(self.servo_lock_count_threshold.storage),

                self.servo_mon_filtered_offset.status.eq(self.i_servo_mon_filtered_offset),
                self.servo_mon_integral_sum.status.eq(self.i_servo_mon_integral_sum),
                self.servo_mon_pi_proportional.status.eq(self.i_servo_mon_pi_proportional),
                self.servo_mon_pi_sum_raw.status.eq(self.i_servo_mon_pi_sum_raw),
                self.servo_mon_status.fields.effective_gain_shift.eq(self.i_servo_mon_effective_gain_shift),
                self.servo_mon_status.fields.lock_counter.eq(self.i_servo_mon_lock_counter),
                self.servo_mon_status.fields.first_lock_achieved.eq(self.i_servo_mon_first_lock_achieved),
                self.servo_mon_sample_count.status.eq(self.i_servo_mon_sample_count),
            ]

        # -- PTP-in-software wiring (hardware-PTP gateware ignores the o_* side)
        self.comb += [
            # FPGA -> SoC status
            self.wallclock_seconds_in_lo.status.eq(self.i_wallclock_seconds_in[:32]),
            self.wallclock_seconds_in_hi.status.eq(self.i_wallclock_seconds_in[32:]),
            self.wallclock_nanoseconds_in.status.eq(self.i_wallclock_nanoseconds_in),
            self.tx_timestamp_sec_in.status.eq(self.i_tx_timestamp_sec_in),
            self.tx_timestamp_nsec_in.status.eq(self.i_tx_timestamp_nsec_in),

            # SoC -> FPGA control
            self.o_wallclock_seconds_out.eq(
                Cat(self.wallclock_seconds_out_lo.storage, self.wallclock_seconds_out_hi.storage[:16])),
            self.o_wallclock_nanoseconds_out.eq(self.wallclock_nanoseconds_out.storage[:30]),
            self.o_wallclock_ppb.eq(self.wallclock_ppb.storage),
            self.o_nco_ppb_adj_valid.eq(self.nco_ppb_adj_valid.storage),
            self.o_wallclock_set.eq(self.wallclock_ctrl.fields.set),
            self.o_wallclock_phasejump.eq(self.wallclock_ctrl.fields.phasejump),
        ]

        # 48-bit NCO correction: commit atomically on the HI write (the host
        # writes LO first), so the NCO never sees a torn LO/HI pair — the HI
        # word is the sign extension and a mismatch would be a ±2^40 glitch.
        self.sync += If(self.nco_ppb_adj_hi.re,
            self.o_nco_ppb_adj.eq(Cat(self.nco_ppb_adj_lo.storage,
                                      self.nco_ppb_adj_hi.storage[:16])))


def add_aes67_csr(soc, platform):
    """Wire ``soc.aes67_csr`` to the ``aes67_ctrl`` pads.

    The instance must already be attached as ``soc.aes67_csr`` *in the SoC's own
    ``__init__``* (see soc.py).  Migen derives generated net names from the
    attribute an object is first stored into; instantiating here would prefix
    every net with this function's name (``add_aes67_csr_*``) and break the
    Quartus timing constraints (FPGA/sdc/litex_csr.sdc) that match ``aes67_csr_*``.
    """
    csr = soc.aes67_csr

    aes67_pads = platform.request("aes67_ctrl")
    soc.comb += [
        # Inputs: pads -> CSR
        csr.i_pll_ppb_valid.eq(aes67_pads.pll_ppb_valid),
        csr.i_pll_ppb_wc_count.eq(aes67_pads.pll_ppb_wc_count),
        csr.i_pll_ppb_pll_count.eq(aes67_pads.pll_ppb_pll_count),
        csr.i_wallclock_locked.eq(aes67_pads.wallclock_locked),
        csr.i_wallclock_configured.eq(aes67_pads.wallclock_configured),
        csr.i_eth_link_up.eq(aes67_pads.eth_link_up),
        csr.i_eth_speed.eq(aes67_pads.eth_speed),
        csr.i_ptp_path_delay.eq(aes67_pads.ptp_path_delay),
        csr.i_ptp_offset.eq(aes67_pads.ptp_offset),
        csr.i_eth_tx_done.eq(aes67_pads.eth_tx_done),
        csr.i_eth_rx_overflow.eq(aes67_pads.eth_rx_overflow),
        csr.i_rx_underrun.eq(aes67_pads.rx_underrun),
        csr.i_rx_mute.eq(aes67_pads.rx_mute),
        csr.i_ptp_is_leader.eq(aes67_pads.ptp_is_leader),
        csr.i_ptp_is_follower.eq(aes67_pads.ptp_is_follower),
        csr.i_ptp_leader_id.eq(aes67_pads.ptp_leader_id),
        csr.i_rx_meter_signal.eq(aes67_pads.rx_meter_signal),
        csr.i_rx_meter_clip.eq(aes67_pads.rx_meter_clip),
        csr.i_tx_meter_signal.eq(aes67_pads.tx_meter_signal),
        csr.i_tx_meter_clip.eq(aes67_pads.tx_meter_clip),
        # PTP-in-software wallclock interface (FPGA -> SoC)
        csr.i_wallclock_seconds_in.eq(aes67_pads.wallclock_seconds_in),
        csr.i_wallclock_nanoseconds_in.eq(aes67_pads.wallclock_nanoseconds_in),
        csr.i_tx_timestamp_sec_in.eq(aes67_pads.tx_timestamp_sec_in),
        csr.i_tx_timestamp_nsec_in.eq(aes67_pads.tx_timestamp_nsec_in),
        # Static FPGA build configuration (constant vector from the bridge top)
        csr.i_system_cfg.eq(aes67_pads.system_cfg),
        # Outputs: CSR -> pads
        aes67_pads.pll_ppb_start.eq(csr.o_pll_ppb_start),
        aes67_pads.mac_addr.eq(csr.o_mac_addr),
        aes67_pads.ip_addr.eq(csr.o_ip_addr),
        aes67_pads.ptp_time_source.eq(csr.o_ptp_time_source),
        aes67_pads.ptp_log_msg_interval.eq(csr.o_ptp_log_msg_interval),
        aes67_pads.ptp_announce_msg_interval.eq(csr.o_ptp_announce_msg_interval),
        aes67_pads.ptp_gm_priority1.eq(csr.o_ptp_gm_priority1),
        aes67_pads.ptp_gm_priority2.eq(csr.o_ptp_gm_priority2),
        aes67_pads.ptp_gm_clock_class.eq(csr.o_ptp_gm_clock_class),
        aes67_pads.ptp_gm_clock_accuracy.eq(csr.o_ptp_gm_clock_accuracy),
        aes67_pads.eth_tx_request.eq(csr.o_eth_tx_request),
        aes67_pads.adda_nrst.eq(csr.o_adda_nrst),
        aes67_pads.meter_clear.eq(csr.o_meter_clear),

        # Reset outputs (from the unified "reset" CSR)
        aes67_pads.ptp_reset.eq(csr.o_ptp_reset),
        aes67_pads.tx_reset.eq(csr.o_tx_reset),
        aes67_pads.rx_reset.eq(csr.o_rx_reset),
        aes67_pads.eth_reset.eq(csr.o_eth_reset),

        # PTP servo / parser tuning (SoC -> FPGA)
        aes67_pads.servo_kp_gain.eq(csr.o_servo_kp_gain),
        aes67_pads.servo_ki_gain.eq(csr.o_servo_ki_gain),
        aes67_pads.servo_gain_shift.eq(csr.o_servo_gain_shift),
        aes67_pads.servo_gain_shift_locked.eq(csr.o_servo_gain_shift_locked),
        aes67_pads.servo_ki_extra_shift.eq(csr.o_servo_ki_extra_shift),
        aes67_pads.servo_filter_shift.eq(csr.o_servo_filter_shift),
        aes67_pads.servo_warmup_samples.eq(csr.o_servo_warmup_samples),
        aes67_pads.servo_lock_threshold_ns.eq(csr.o_servo_lock_threshold_ns),
        aes67_pads.servo_unlock_threshold_ns.eq(csr.o_servo_unlock_threshold_ns),
        aes67_pads.servo_lock_count_threshold.eq(csr.o_servo_lock_count_threshold),
        aes67_pads.parser_min_filter_enable.eq(csr.o_parser_min_filter_enable),
        aes67_pads.parser_min_filter_active_depth.eq(csr.o_parser_min_filter_active_depth),
        aes67_pads.parser_delay_asymmetry_ns.eq(csr.o_parser_delay_asymmetry_ns),

        # PTP servo monitoring (FPGA -> SoC)
        csr.i_servo_mon_filtered_offset.eq(aes67_pads.servo_mon_filtered_offset),
        csr.i_servo_mon_integral_sum.eq(aes67_pads.servo_mon_integral_sum),
        csr.i_servo_mon_pi_proportional.eq(aes67_pads.servo_mon_pi_proportional),
        csr.i_servo_mon_pi_sum_raw.eq(aes67_pads.servo_mon_pi_sum_raw),
        csr.i_servo_mon_effective_gain_shift.eq(aes67_pads.servo_mon_effective_gain_shift),
        csr.i_servo_mon_lock_counter.eq(aes67_pads.servo_mon_lock_counter),
        csr.i_servo_mon_sample_count.eq(aes67_pads.servo_mon_sample_count),
        csr.i_servo_mon_first_lock_achieved.eq(aes67_pads.servo_mon_first_lock_achieved),

        # PTP-in-software wallclock control (SoC -> FPGA)
        aes67_pads.wallclock_seconds_out.eq(csr.o_wallclock_seconds_out),
        aes67_pads.wallclock_nanoseconds_out.eq(csr.o_wallclock_nanoseconds_out),
        aes67_pads.wallclock_set.eq(csr.o_wallclock_set),
        aes67_pads.wallclock_phasejump.eq(csr.o_wallclock_phasejump),
        aes67_pads.wallclock_ppb.eq(csr.o_wallclock_ppb),
        aes67_pads.nco_ppb_adj.eq(csr.o_nco_ppb_adj),
        aes67_pads.nco_ppb_adj_valid.eq(csr.o_nco_ppb_adj_valid),
    ]
    return csr
