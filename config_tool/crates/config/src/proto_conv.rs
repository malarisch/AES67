//! Conversions between the wire DTOs (`aes67-proto`) and the internal types.
//!
//! Centralised here so the daemon (`aes67d`) and the client (`aes67-client`)
//! share one mapping and cannot drift. The wire side carries IPs as strings and
//! access modes as `&str`; the internal side uses `Ipv4Addr` / [`Access`].

use std::net::Ipv4Addr;

use aes67_proto as proto;

use crate::config::PtpGrandmaster;
use crate::control::RegisterInfo;
use crate::csr::Access;
use crate::stream::{RxStream, TxStream};
use crate::ConfigError;

// -- PTP grandmaster --------------------------------------------------------

impl From<proto::GrandmasterParams> for PtpGrandmaster {
    fn from(p: proto::GrandmasterParams) -> Self {
        PtpGrandmaster {
            priority1: p.priority1,
            priority2: p.priority2,
            clock_class: p.clock_class,
            clock_accuracy: p.clock_accuracy,
        }
    }
}

impl From<PtpGrandmaster> for proto::GrandmasterParams {
    fn from(g: PtpGrandmaster) -> Self {
        proto::GrandmasterParams {
            priority1: g.priority1,
            priority2: g.priority2,
            clock_class: g.clock_class,
            clock_accuracy: g.clock_accuracy,
        }
    }
}

// -- Streams ----------------------------------------------------------------

impl TryFrom<proto::TxStreamParams> for TxStream {
    type Error = ConfigError;
    fn try_from(p: proto::TxStreamParams) -> Result<Self, ConfigError> {
        Ok(TxStream {
            id: p.id,
            dst_ip: parse_ip(&p.dst_ip)?,
            channels: p.channels,
            samples_per_packet: p.samples_per_packet,
            ch_ids: p.ch_ids,
            ssrc: p.ssrc,
            name: p.name,
        })
    }
}

impl From<&TxStream> for proto::TxStreamParams {
    fn from(s: &TxStream) -> Self {
        proto::TxStreamParams {
            id: s.id,
            dst_ip: s.dst_ip.to_string(),
            channels: s.channels,
            samples_per_packet: s.samples_per_packet,
            ch_ids: s.ch_ids.clone(),
            ssrc: s.ssrc,
            name: s.name.clone(),
        }
    }
}

impl TryFrom<proto::RxStreamParams> for RxStream {
    type Error = ConfigError;
    fn try_from(p: proto::RxStreamParams) -> Result<Self, ConfigError> {
        Ok(RxStream {
            id: p.id,
            dst_ip: parse_ip(&p.dst_ip)?,
            dst_port: p.dst_port,
            ch_map: p.ch_map,
            channels: p.channels,
            output_delay: p.output_delay,
            samples_per_channel: p.samples_per_channel,
        })
    }
}

impl From<&RxStream> for proto::RxStreamParams {
    fn from(s: &RxStream) -> Self {
        proto::RxStreamParams {
            id: s.id,
            dst_ip: s.dst_ip.to_string(),
            dst_port: s.dst_port,
            ch_map: s.ch_map.clone(),
            channels: s.channels,
            output_delay: s.output_delay,
            samples_per_channel: s.samples_per_channel,
        }
    }
}

// -- Register info ----------------------------------------------------------

impl From<RegisterInfo> for proto::RegisterInfo {
    fn from(r: RegisterInfo) -> Self {
        proto::RegisterInfo {
            name: r.name,
            addr: r.addr,
            size: r.size,
            access: r.access.as_str().to_string(),
        }
    }
}

impl From<proto::RegisterInfo> for RegisterInfo {
    fn from(r: proto::RegisterInfo) -> Self {
        RegisterInfo {
            name: r.name,
            addr: r.addr,
            size: r.size,
            access: Access::from(r.access.as_str()),
        }
    }
}

fn parse_ip(s: &str) -> Result<Ipv4Addr, ConfigError> {
    s.parse()
        .map_err(|_| ConfigError::Parse(format!("invalid IPv4 address '{s}'")))
}
