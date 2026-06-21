//! Build a monitoring snapshot (and a full register dump) from the daemon's
//! [`ControlApi`]. The web server is just a client: it reads the FPGA status
//! CSRs by name and decodes them — no protocol or daemon changes needed.

use aes67_config::{ConfigError, ControlApi};
use serde::Serialize;

const REG_STATUS: &str = "aes67_csr_status";
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

// aes67_csr_status bit layout (mirrors the gateware).
const S_WC_LOCKED: u64 = 1 << 0;
const S_WC_CONFIGURED: u64 = 1 << 2;
const S_ETH_LINK_UP: u64 = 1 << 4;
const S_ETH_SPEED_SHIFT: u32 = 5;
const S_ETH_SPEED_MASK: u64 = 0b11;
const S_ETH_TX_DONE: u64 = 1 << 7;
const S_ETH_RX_OVERFLOW: u64 = 1 << 8;
const S_PTP_LEADER: u64 = 1 << 9;
const S_PTP_FOLLOWER: u64 = 1 << 10;

/// Decoded monitoring snapshot, serialised to `/api/status`.
#[derive(Serialize)]
pub struct Snapshot {
    pub link: Link,
    pub ptp: Ptp,
    pub wallclock: Wallclock,
    pub eth: Eth,
    pub network: Network,
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
    pub leader: bool,
    pub follower: bool,
    pub offset_ns: Option<i32>,
    pub path_delay_ns: Option<u32>,
    pub leader_id: Option<String>,
}

#[derive(Serialize)]
pub struct Wallclock {
    pub locked: bool,
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

/// Read and decode the monitoring snapshot. The status read propagates errors
/// (so a dead daemon connection is detected and reconnected); everything else is
/// best-effort (missing/optional CSRs simply read as null).
pub fn snapshot(dev: &mut dyn ControlApi) -> Result<Snapshot, ConfigError> {
    let status = dev.read_register(REG_STATUS)?;

    let speed = match (status >> S_ETH_SPEED_SHIFT) & S_ETH_SPEED_MASK {
        0 => Some(10),
        1 => Some(100),
        2 => Some(1000),
        _ => None,
    };

    let offset_ns = dev.read_register(REG_PTP_OFFSET).ok().map(|v| v as u32 as i32);
    let path_delay_ns = dev.read_register(REG_PTP_PATH_DELAY).ok().map(|v| v as u32);
    let leader_id = match (
        dev.read_register(REG_PTP_LEADER_ID_LO).ok(),
        dev.read_register(REG_PTP_LEADER_ID_HI).ok(),
    ) {
        (Some(lo), Some(hi)) => Some(format!("{:016x}", (hi << 32) | (lo & 0xffff_ffff))),
        _ => None,
    };

    let ip = dev.get_ip().ok().map(|i| i.to_string());
    let mac = dev.get_mac().ok().map(|m| {
        format!("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", m[0], m[1], m[2], m[3], m[4], m[5])
    });

    Ok(Snapshot {
        link: Link { up: status & S_ETH_LINK_UP != 0, speed_mbps: speed },
        ptp: Ptp {
            leader: status & S_PTP_LEADER != 0,
            follower: status & S_PTP_FOLLOWER != 0,
            offset_ns,
            path_delay_ns,
            leader_id,
        },
        wallclock: Wallclock {
            locked: status & S_WC_LOCKED != 0,
            configured: status & S_WC_CONFIGURED != 0,
        },
        eth: Eth {
            tx_done: status & S_ETH_TX_DONE != 0,
            rx_overflow: status & S_ETH_RX_OVERFLOW != 0,
        },
        network: Network { ip, mac },
        status_raw: Some(format!("0x{status:08x}")),
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
