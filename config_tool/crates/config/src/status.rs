//! Decoders for the gateware's read-only status CSRs.
//!
//! Shared by the CLI, the web dashboard and the daemon so the bit layout lives
//! in one place. It mirrors `litex_soc/aes67_soc/peripherals/aes67_csr.py`
//! (and the firmware's `eth_litex.h` / `fpga_hal.h`):
//!
//! * `aes67_csr_status` — link, PTP role, wallclock state and the RX
//!   stream-underrun diagnostics (bits 16–27).
//! * `aes67_csr_system_cfg_{flags,rx,tx}` — the static build configuration
//!   (`syscfg` generic) of the loaded bitstream: PTP mode, metering, and the
//!   per-direction stream/channel limits.
//! * `aes67_csr_{rx,tx}_meter_{signal,clip}` — sticky audio metering, present
//!   only when the bridge was generated with metering CSRs.
//!
//! Registers are looked up by name. A register missing from the map (older
//! gateware, or a build that dropped the group) reads as `None` rather than an
//! error, so callers can show "n/a" instead of failing.

use serde::Serialize;

use crate::control::ControlApi;
use crate::ConfigError;

pub const REG_STATUS: &str = "aes67_csr_status";
pub const REG_SYSCFG_FLAGS: &str = "aes67_csr_system_cfg_flags";
pub const REG_SYSCFG_RX: &str = "aes67_csr_system_cfg_rx";
pub const REG_SYSCFG_TX: &str = "aes67_csr_system_cfg_tx";
pub const REG_RX_METER_SIGNAL: &str = "aes67_csr_rx_meter_signal";
pub const REG_RX_METER_CLIP: &str = "aes67_csr_rx_meter_clip";
pub const REG_TX_METER_SIGNAL: &str = "aes67_csr_tx_meter_signal";
pub const REG_TX_METER_CLIP: &str = "aes67_csr_tx_meter_clip";
pub const REG_METER_CLEAR: &str = "aes67_csr_meter_clear";

// aes67_csr_status bit layout.
const S_WC_LOCKED: u32 = 1 << 0;
const S_WC_CONFIGURED: u32 = 1 << 2;
const S_ETH_LINK_UP: u32 = 1 << 4;
const S_ETH_SPEED_SHIFT: u32 = 5;
const S_ETH_TX_DONE: u32 = 1 << 7;
const S_ETH_RX_OVERFLOW: u32 = 1 << 8;
const S_PTP_LEADER: u32 = 1 << 9;
const S_PTP_FOLLOWER: u32 = 1 << 10;
const S_RX_UNDERRUN_SHIFT: u32 = 16;
const S_RX_MUTE_SHIFT: u32 = 20;

/// RX streams covered by the `rx_underrun` field (streams 0..3).
pub const RX_UNDERRUN_STREAMS: u8 = 4;
/// Output channels covered by the `rx_mute` field (channels 0..7).
pub const RX_MUTE_CHANNELS: u8 = 8;

// system_cfg_flags bits.
const F_PTP_IN_SOFTWARE: u32 = 1 << 0;
const F_STATIC_PTP_CONFIG: u32 = 1 << 1;
const F_METERING: u32 = 1 << 2;

/// A decoded `aes67_csr_status` word.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StatusWord(pub u32);

impl StatusWord {
    pub fn raw(self) -> u32 {
        self.0
    }
    /// Hardware PTP servo locked (always 0 in software-PTP gateware).
    pub fn wallclock_locked(self) -> bool {
        self.0 & S_WC_LOCKED != 0
    }
    pub fn wallclock_configured(self) -> bool {
        self.0 & S_WC_CONFIGURED != 0
    }
    pub fn eth_link_up(self) -> bool {
        self.0 & S_ETH_LINK_UP != 0
    }
    /// PHY speed in Mbit/s (`None` for the reserved code).
    pub fn eth_speed_mbps(self) -> Option<u32> {
        match (self.0 >> S_ETH_SPEED_SHIFT) & 0b11 {
            0 => Some(10),
            1 => Some(100),
            2 => Some(1000),
            _ => None,
        }
    }
    pub fn eth_tx_done(self) -> bool {
        self.0 & S_ETH_TX_DONE != 0
    }
    pub fn eth_rx_overflow(self) -> bool {
        self.0 & S_ETH_RX_OVERFLOW != 0
    }
    /// FPGA BMC result (hardware PTP only).
    pub fn ptp_leader(self) -> bool {
        self.0 & S_PTP_LEADER != 0
    }
    pub fn ptp_follower(self) -> bool {
        self.0 & S_PTP_FOLLOWER != 0
    }
    /// Per-stream underrun flags, bit n = RX stream slot n (slots 0..3). Live
    /// state: the gateware sets the flag when the stream's write pointer falls
    /// behind playout and clears it with the next complete packet.
    pub fn rx_underrun_mask(self) -> u8 {
        ((self.0 >> S_RX_UNDERRUN_SHIFT) & 0xf) as u8
    }
    /// Output channels muted because their source stream is in underrun,
    /// bit n = output channel n (channels 0..7).
    pub fn rx_mute_mask(self) -> u8 {
        ((self.0 >> S_RX_MUTE_SHIFT) & 0xff) as u8
    }
    /// Underrun state of one RX slot, `None` for slots the CSR doesn't cover.
    pub fn rx_stream_underrun(self, slot: u8) -> Option<bool> {
        (slot < RX_UNDERRUN_STREAMS).then(|| self.rx_underrun_mask() >> slot & 1 != 0)
    }

    /// Read and decode the status register.
    pub fn read(dev: &mut (impl ControlApi + ?Sized)) -> Result<Self, ConfigError> {
        Ok(Self(dev.read_register(REG_STATUS)? as u32))
    }
}

/// Stream/channel limits of one audio direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct PathCfg {
    /// Number of RTP stream slots the gateware implements.
    pub max_streams: u8,
    /// Audio channels on the converter side.
    pub channels: u8,
    /// Ring buffer depth (samples).
    pub buffer_depth: u16,
}

impl PathCfg {
    fn decode(v: u32) -> Self {
        Self {
            max_streams: (v & 0xff) as u8,
            channels: ((v >> 8) & 0xff) as u8,
            buffer_depth: (v >> 16) as u16,
        }
    }
}

/// The static build configuration of the loaded bitstream (`system_cfg_*`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct SystemCfg {
    /// Built with `PTP_IN_SOFTWARE`: a host PTP stack (ptp4l via the kernel
    /// driver) disciplines the wallclock, the FPGA servo CSRs read 0, and RX
    /// frames carry the 5-byte timestamp trailer.
    pub ptp_in_software: bool,
    /// Servo tuning fixed via generic (no servo CSRs).
    pub static_ptp_config: bool,
    /// Audio metering is live.
    pub metering: bool,
    /// RX / DA path (network → converter).
    pub rx: PathCfg,
    /// TX / AD path (converter → network).
    pub tx: PathCfg,
}

impl SystemCfg {
    pub fn decode(flags: u32, rx: u32, tx: u32) -> Self {
        Self {
            ptp_in_software: flags & F_PTP_IN_SOFTWARE != 0,
            static_ptp_config: flags & F_STATIC_PTP_CONFIG != 0,
            metering: flags & F_METERING != 0,
            rx: PathCfg::decode(rx),
            tx: PathCfg::decode(tx),
        }
    }

    /// Read the build configuration. `Ok(None)` when the gateware predates
    /// the `system_cfg` CSRs (they are not in the map).
    pub fn read(dev: &mut (impl ControlApi + ?Sized)) -> Result<Option<Self>, ConfigError> {
        let flags = read_optional(dev, REG_SYSCFG_FLAGS)?;
        let rx = read_optional(dev, REG_SYSCFG_RX)?;
        let tx = read_optional(dev, REG_SYSCFG_TX)?;
        Ok(match (flags, rx, tx) {
            (Some(f), Some(r), Some(t)) => Some(Self::decode(f, r, t)),
            _ => None,
        })
    }
}

/// Sticky audio metering bits, one bit per channel (bit n = channel n).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct Metering {
    pub rx_signal: u16,
    pub rx_clip: u16,
    pub tx_signal: u16,
    pub tx_clip: u16,
}

impl Metering {
    /// Read the four metering registers, then toggle `meter_clear` so the
    /// gateware restarts its sticky detectors (same sequence as the firmware's
    /// `fpga_hal_read_metering`). Each call therefore reports what happened
    /// since the previous call. `Ok(None)` when the metering CSRs are absent.
    pub fn read_and_clear(dev: &mut (impl ControlApi + ?Sized)) -> Result<Option<Self>, ConfigError> {
        let Some(rx_signal) = read_optional(dev, REG_RX_METER_SIGNAL)? else {
            return Ok(None);
        };
        let m = Self {
            rx_signal: rx_signal as u16,
            rx_clip: dev.read_register(REG_RX_METER_CLIP)? as u16,
            tx_signal: dev.read_register(REG_TX_METER_SIGNAL)? as u16,
            tx_clip: dev.read_register(REG_TX_METER_CLIP)? as u16,
        };
        let cur = dev.read_register(REG_METER_CLEAR)?;
        dev.write_register(REG_METER_CLEAR, (cur ^ 1) & 1)?;
        Ok(Some(m))
    }
}

/// Read a register that may be absent from the map: `Ok(None)` if unknown.
fn read_optional(
    dev: &mut (impl ControlApi + ?Sized),
    name: &str,
) -> Result<Option<u32>, ConfigError> {
    match dev.read_register(name) {
        Ok(v) => Ok(Some(v as u32)),
        Err(ConfigError::UnknownRegister(_)) => Ok(None),
        Err(e) => Err(e),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_decodes_underrun_and_mute() {
        // link up, 100M, follower, stream 1+3 underrun, channels 2,3,6 muted.
        let s = StatusWord((1 << 4) | (1 << 5) | (1 << 10) | (0b1010 << 16) | (0b0100_1100 << 20));
        assert!(s.eth_link_up());
        assert_eq!(s.eth_speed_mbps(), Some(100));
        assert!(s.ptp_follower() && !s.ptp_leader());
        assert_eq!(s.rx_underrun_mask(), 0b1010);
        assert_eq!(s.rx_mute_mask(), 0b0100_1100);
        assert_eq!(s.rx_stream_underrun(0), Some(false));
        assert_eq!(s.rx_stream_underrun(1), Some(true));
        assert_eq!(s.rx_stream_underrun(3), Some(true));
        assert_eq!(s.rx_stream_underrun(4), None);
    }

    #[test]
    fn syscfg_decodes_fields() {
        // SW PTP + metering; RX 4 streams/8 ch/depth 1024; TX 2 streams/8 ch/512.
        let c = SystemCfg::decode(0b101, (1024 << 16) | (8 << 8) | 4, (512 << 16) | (8 << 8) | 2);
        assert!(c.ptp_in_software && c.metering && !c.static_ptp_config);
        assert_eq!(c.rx, PathCfg { max_streams: 4, channels: 8, buffer_depth: 1024 });
        assert_eq!(c.tx, PathCfg { max_streams: 2, channels: 8, buffer_depth: 512 });
    }
}
