//! Configuration endpoints: PTP parameters and RX/TX stream setup.
//!
//! Request DTOs are deserialised from the POST bodies and applied through the
//! daemon's [`ControlApi`] — the same path the CLI uses, so writes are persisted
//! to the daemon's config and replayed on an FPGA reset, and new RX/TX stream
//! groups are picked up by the IGMP membership reconciler.

use std::net::Ipv4Addr;

use aes67_config::{ConfigError, ControlApi, PtpGrandmaster, RxStream, TxStream};
use serde::Deserialize;

const REG_PTP_SYNC_INTERVAL: &str = "aes67_csr_ptp_log_msg_interval";
const REG_PTP_ANNOUNCE_INTERVAL: &str = "aes67_csr_ptp_announce_msg_interval";
const REG_PTP_TIME_SOURCE: &str = "aes67_csr_ptp_time_source";

/// `POST /api/ptp` body — grandmaster announce params + message intervals. Every
/// field is optional; only the ones present are written.
#[derive(Deserialize)]
pub struct PtpReq {
    pub priority1: Option<u8>,
    pub priority2: Option<u8>,
    pub clock_class: Option<u8>,
    pub clock_accuracy: Option<u8>,
    /// Sync / Delay-Req interval (logMessageInterval, signed log2 seconds).
    pub sync_interval: Option<i8>,
    /// Announce interval (signed log2 seconds).
    pub announce_interval: Option<i8>,
    pub time_source: Option<u8>,
}

/// `POST /api/tx-stream` body.
#[derive(Deserialize)]
pub struct TxReq {
    pub id: u8,
    pub dst_ip: String,
    pub channels: Option<u8>,
    pub samples_per_packet: Option<u8>,
    pub ch_ids: Option<Vec<u8>>,
    pub ssrc: Option<u32>,
    /// Session name announced over SAP/SDP.
    pub name: Option<String>,
}

/// `POST /api/rx-stream` body.
#[derive(Deserialize)]
pub struct RxReq {
    pub id: u8,
    pub dst_ip: String,
    pub dst_port: u16,
    pub ch_map: Option<Vec<u8>>,
    pub channels: Option<u8>,
    pub output_delay: Option<u8>,
    pub samples_per_channel: Option<u8>,
}

/// `POST /api/tx-stream/stop` and `/api/rx-stream/stop` body — the slot to clear.
#[derive(Deserialize)]
pub struct StopReq {
    pub id: u8,
}

/// Apply PTP grandmaster params and message intervals.
pub fn apply_ptp(dev: &mut dyn ControlApi, req: &PtpReq) -> Result<(), ConfigError> {
    dev.set_grandmaster(PtpGrandmaster {
        priority1: req.priority1,
        priority2: req.priority2,
        clock_class: req.clock_class,
        clock_accuracy: req.clock_accuracy,
    })?;
    // Intervals are plain CSRs (signed log2 seconds → 8-bit two's complement).
    if let Some(v) = req.sync_interval {
        dev.write_register(REG_PTP_SYNC_INTERVAL, (v as u8) as u64)?;
    }
    if let Some(v) = req.announce_interval {
        dev.write_register(REG_PTP_ANNOUNCE_INTERVAL, (v as u8) as u64)?;
    }
    if let Some(v) = req.time_source {
        dev.write_register(REG_PTP_TIME_SOURCE, v as u64)?;
    }
    Ok(())
}

/// Configure a transmit stream.
pub fn apply_tx(dev: &mut dyn ControlApi, req: &TxReq) -> Result<(), ConfigError> {
    dev.write_tx_stream(&TxStream {
        id: req.id,
        dst_ip: parse_ip(&req.dst_ip)?,
        channels: req.channels,
        samples_per_packet: req.samples_per_packet.unwrap_or(0),
        ch_ids: req.ch_ids.clone().unwrap_or_default(),
        ssrc: req.ssrc.unwrap_or(0),
        name: req.name.clone().filter(|n| !n.is_empty()),
    })
}

/// Configure a receive stream.
pub fn apply_rx(dev: &mut dyn ControlApi, req: &RxReq) -> Result<(), ConfigError> {
    dev.write_rx_stream(&RxStream {
        id: req.id,
        dst_ip: parse_ip(&req.dst_ip)?,
        dst_port: req.dst_port,
        ch_map: req.ch_map.clone().unwrap_or_default(),
        channels: req.channels,
        output_delay: req.output_delay.unwrap_or(0),
        samples_per_channel: req.samples_per_channel.unwrap_or(0),
    })
}

/// Tear down a transmit stream.
pub fn stop_tx(dev: &mut dyn ControlApi, req: &StopReq) -> Result<(), ConfigError> {
    dev.clear_tx_stream(req.id)
}

/// Tear down a receive stream (the daemon also leaves its multicast group).
pub fn stop_rx(dev: &mut dyn ControlApi, req: &StopReq) -> Result<(), ConfigError> {
    dev.clear_rx_stream(req.id)
}

fn parse_ip(s: &str) -> Result<Ipv4Addr, ConfigError> {
    s.parse()
        .map_err(|_| ConfigError::Parse(format!("invalid IPv4 address '{s}'")))
}
