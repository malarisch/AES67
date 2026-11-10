//! Build a monitoring snapshot (and a full register dump) from the daemon's
//! [`ControlApi`]. The web server is just a client: it reads the FPGA status
//! CSRs by name and decodes them — no protocol or daemon changes needed.

use aes67_config::status::{RX_MUTE_CHANNELS, RX_UNDERRUN_STREAMS};
use aes67_config::{ConfigError, ControlApi, Metering, StatusWord, SystemCfg};
use serde::Serialize;

const REG_PTP_OFFSET: &str = "aes67_csr_ptp_offset";
const REG_PTP_PATH_DELAY: &str = "aes67_csr_ptp_path_delay";
const REG_PTP_LEADER_ID_LO: &str = "aes67_csr_ptp_leader_id_lo";
const REG_PTP_LEADER_ID_HI: &str = "aes67_csr_ptp_leader_id_hi";

const REG_GM_PRIORITY1: &str = "aes67_csr_ptp_gm_priority1";
const REG_GM_PRIORITY2: &str = "aes67_csr_ptp_gm_priority2";
const REG_GM_CLOCK_CLASS: &str = "aes67_csr_ptp_gm_clock_class";
const REG_GM_CLOCK_ACCURACY: &str = "aes67_csr_ptp_gm_clock_accuracy";
const REG_PTP_SYNC_INTERVAL: &str = "aes67_csr_ptp_log_msg_interval";
const REG_PTP_ANNOUNCE_INTERVAL: &str = "aes67_csr_ptp_announce_msg_interval";
const REG_PTP_TIME_SOURCE: &str = "aes67_csr_ptp_time_source";

/// Decoded monitoring snapshot, serialised to `/api/status`.
#[derive(Serialize)]
pub struct Snapshot {
    pub link: Link,
    pub ptp: Ptp,
    pub wallclock: Wallclock,
    pub eth: Eth,
    pub network: Network,
    /// RX stream health (underrun detection + auto-mute).
    pub audio_rx: AudioRx,
    /// Static build configuration of the loaded bitstream, or null if the
    /// gateware predates the `system_cfg` CSRs.
    pub gateware: Option<SystemCfg>,
    /// Audio metering since the previous poll, or null when the bridge has no
    /// metering CSRs.
    pub metering: Option<Metering>,
    /// Raw status word (hex), or null if the status read failed.
    pub status_raw: Option<String>,
}

#[derive(Serialize)]
pub struct Link {
    pub up: bool,
    pub speed_mbps: Option<u32>,
}

#[derive(Serialize)]
pub struct Ptp {
    /// "hardware" (FPGA PTP engine) or "software" (host ptp4l disciplines the
    /// wallclock through the kernel PHC); null if the gateware doesn't say.
    pub mode: Option<&'static str>,
    /// FPGA BMC role bits. Only meaningful in hardware mode.
    pub leader: bool,
    pub follower: bool,
    /// FPGA servo readouts; null in software mode (the CSRs read 0 there and
    /// ptp4l owns these numbers).
    pub offset_ns: Option<i32>,
    pub path_delay_ns: Option<u32>,
    pub leader_id: Option<String>,
}

#[derive(Serialize)]
pub struct Wallclock {
    /// FPGA servo lock; null in software-PTP mode (no FPGA servo).
    pub locked: Option<bool>,
    pub configured: bool,
}

#[derive(Serialize)]
pub struct Eth {
    pub tx_done: bool,
    pub rx_overflow: bool,
}

#[derive(Serialize)]
pub struct Network {
    pub ip: Option<String>,
    pub mac: Option<String>,
}

#[derive(Serialize)]
pub struct AudioRx {
    /// RX slots whose underrun flag is set (playout overtook the last packet;
    /// clears with the next complete packet).
    pub underrun_streams: Vec<u8>,
    /// Output channels the gateware muted because their stream is in underrun.
    pub muted_channels: Vec<u8>,
    /// How many RX slots the underrun field covers (slots 0..n-1).
    pub monitored_streams: u8,
    /// How many output channels the mute field covers.
    pub monitored_channels: u8,
}

/// Indices of the set bits in `mask`, limited to the low `n` bits.
fn bits(mask: u32, n: u8) -> Vec<u8> {
    (0..n).filter(|i| mask >> i & 1 != 0).collect()
}

/// Read and decode the monitoring snapshot. The status read propagates errors
/// (so a dead daemon connection is detected and reconnected); everything else is
/// best-effort (missing/optional CSRs simply read as null).
pub fn snapshot(dev: &mut dyn ControlApi) -> Result<Snapshot, ConfigError> {
    let status = StatusWord::read(dev)?;
    let gateware = SystemCfg::read(dev).ok().flatten();
    let sw_ptp = gateware.map(|g| g.ptp_in_software);

    // The FPGA servo readouts are constant 0 in software-PTP gateware.
    let hw_servo = sw_ptp != Some(true);
    let offset_ns = hw_servo
        .then(|| dev.read_register(REG_PTP_OFFSET).ok().map(|v| v as u32 as i32))
        .flatten();
    let path_delay_ns = hw_servo
        .then(|| dev.read_register(REG_PTP_PATH_DELAY).ok().map(|v| v as u32))
        .flatten();
    let leader_id = match (
        dev.read_register(REG_PTP_LEADER_ID_LO).ok(),
        dev.read_register(REG_PTP_LEADER_ID_HI).ok(),
    ) {
        (Some(lo), Some(hi)) if hw_servo => {
            Some(format!("{:016x}", (hi << 32) | (lo & 0xffff_ffff)))
        }
        _ => None,
    };

    // Clip the mute field to the channels the gateware actually has.
    let rx_channels = gateware
        .map(|g| g.rx.channels.min(RX_MUTE_CHANNELS))
        .unwrap_or(RX_MUTE_CHANNELS);
    let rx_slots = gateware
        .map(|g| g.rx.max_streams.min(RX_UNDERRUN_STREAMS))
        .unwrap_or(RX_UNDERRUN_STREAMS);

    // Skip the metering read (and its clear toggle) when the gateware says
    // metering is off; the CSRs may still exist but would read 0.
    let metering = if gateware.is_some_and(|g| !g.metering) {
        None
    } else {
        Metering::read_and_clear(dev).ok().flatten()
    };

    let ip = dev.get_ip().ok().map(|i| i.to_string());
    let mac = dev.get_mac().ok().map(|m| {
        format!("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", m[0], m[1], m[2], m[3], m[4], m[5])
    });

    Ok(Snapshot {
        link: Link { up: status.eth_link_up(), speed_mbps: status.eth_speed_mbps() },
        ptp: Ptp {
            mode: sw_ptp.map(|sw| if sw { "software" } else { "hardware" }),
            leader: status.ptp_leader(),
            follower: status.ptp_follower(),
            offset_ns,
            path_delay_ns,
            leader_id,
        },
        wallclock: Wallclock {
            locked: hw_servo.then(|| status.wallclock_locked()),
            configured: status.wallclock_configured(),
        },
        eth: Eth { tx_done: status.eth_tx_done(), rx_overflow: status.eth_rx_overflow() },
        network: Network { ip, mac },
        audio_rx: AudioRx {
            underrun_streams: bits(status.rx_underrun_mask().into(), rx_slots),
            muted_channels: bits(status.rx_mute_mask().into(), rx_channels),
            monitored_streams: rx_slots,
            monitored_channels: rx_channels,
        },
        gateware,
        metering,
        status_raw: Some(format!("0x{:08x}", status.raw())),
    })
}

/// Current PTP configuration read back from the CSRs, used to pre-fill the
/// config form. Intervals are signed log2 seconds.
#[derive(Serialize)]
pub struct PtpConfig {
    pub priority1: Option<u8>,
    pub priority2: Option<u8>,
    pub clock_class: Option<u8>,
    pub clock_accuracy: Option<u8>,
    pub sync_interval: Option<i8>,
    pub announce_interval: Option<i8>,
    pub time_source: Option<u8>,
}

/// Read the current PTP config. The first read propagates errors (connection
/// liveness); the rest are best-effort.
pub fn ptp_config(dev: &mut dyn ControlApi) -> Result<PtpConfig, ConfigError> {
    let priority1 = dev.read_register(REG_GM_PRIORITY1)? as u8;
    Ok(PtpConfig {
        priority1: Some(priority1),
        priority2: dev.read_register(REG_GM_PRIORITY2).ok().map(|v| v as u8),
        clock_class: dev.read_register(REG_GM_CLOCK_CLASS).ok().map(|v| v as u8),
        clock_accuracy: dev.read_register(REG_GM_CLOCK_ACCURACY).ok().map(|v| v as u8),
        sync_interval: dev.read_register(REG_PTP_SYNC_INTERVAL).ok().map(|v| v as u8 as i8),
        announce_interval: dev.read_register(REG_PTP_ANNOUNCE_INTERVAL).ok().map(|v| v as u8 as i8),
        time_source: dev.read_register(REG_PTP_TIME_SOURCE).ok().map(|v| v as u8),
    })
}

/// One register's metadata + current value, serialised to `/api/registers`.
#[derive(Serialize)]
pub struct RegDump {
    pub name: String,
    pub addr: String,
    pub size: u32,
    pub access: String,
    /// Hex value, or null if the read failed.
    pub value: Option<String>,
}

/// Dump the entire CSR map with current values. `list_registers` propagates
/// errors (connection liveness); per-register reads are best-effort.
pub fn registers(dev: &mut dyn ControlApi) -> Result<Vec<RegDump>, ConfigError> {
    let regs = dev.list_registers()?;
    Ok(regs
        .into_iter()
        .map(|r| {
            let value = dev.read_register(&r.name).ok().map(|v| format!("0x{v:x}"));
            RegDump {
                name: r.name,
                addr: format!("0x{:08x}", r.addr),
                size: r.size,
                access: format!("{:?}", r.access).to_lowercase(),
                value,
            }
        })
        .collect())
}
