//! AES67 audio SDP: a small, transport-agnostic model of an AES67 RTP audio
//! stream plus a [generator](generate) and [parser](parse) for its SDP session
//! description.
//!
//! This crate knows nothing about *how* a description travels — that is the job
//! of the discovery transport. Today [`aes67-sap`](../aes67_sap) carries it over
//! the Session Announcement Protocol; an mDNS/DNS-SD discovery layer can reuse
//! the very same [`AudioStream`] ↔ SDP mapping later. Keep it dependency-light
//! and free of any socket/IO code.
//!
//! The model targets the AES67 SDP profile (RFC 7587 / SMPTE ST 2110-30 style):
//! a single `m=audio` media line, an `L16`/`L24` linear PCM `rtpmap`, an explicit
//! `ptime`, a multicast `c=` line, and the PTP `ts-refclk` / `mediaclk` clocking
//! attributes. Example:
//!
//! ```text
//! v=0
//! o=- 1311738121 1311738121 IN IP4 192.168.1.1
//! s=AES67 stream 0
//! c=IN IP4 239.69.1.0/32
//! t=0 0
//! m=audio 5004 RTP/AVP 97
//! a=rtpmap:97 L24/48000/2
//! a=ptime:1
//! a=ts-refclk:ptp=IEEE1588-2008:00-1D-C1-FF-FE-01-02-03:0
//! a=mediaclk:direct=0
//! ```
//!
//! Generation is hand-rolled (the AES67 output is a fixed, small shape, so a
//! direct writer is clearer and guarantees the canonical form); parsing delegates
//! to the `sdp-rs` tokeniser so we accept the full range of real-world senders.

use std::net::Ipv4Addr;

mod generate;
mod parse;

pub use parse::SdpError;

/// PCM sample encoding carried in the `rtpmap` (`L16`/`L24`/`L32`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Encoding {
    L16,
    L24,
    L32,
}

impl Encoding {
    /// The RTP encoding name as it appears in `a=rtpmap` (e.g. `L24`).
    pub fn name(self) -> &'static str {
        match self {
            Encoding::L16 => "L16",
            Encoding::L24 => "L24",
            Encoding::L32 => "L32",
        }
    }

    /// Parse an `rtpmap` encoding name (case-insensitive). `None` for unknown
    /// names so the caller can reject non-PCM streams it cannot ingest.
    pub fn from_name(s: &str) -> Option<Self> {
        match s.to_ascii_uppercase().as_str() {
            "L16" => Some(Encoding::L16),
            "L24" => Some(Encoding::L24),
            "L32" => Some(Encoding::L32),
            _ => None,
        }
    }

    /// Bytes per sample per channel.
    pub fn bytes_per_sample(self) -> u8 {
        match self {
            Encoding::L16 => 2,
            Encoding::L24 => 3,
            Encoding::L32 => 4,
        }
    }
}

/// An AES67 RTP audio stream described independently of its discovery transport.
///
/// The same value round-trips through [`to_sdp`](AudioStream::to_sdp) /
/// [`from_sdp`](AudioStream::from_sdp), and is the unit a discovery layer (SAP
/// today, mDNS later) advertises or learns.
#[derive(Debug, Clone, PartialEq)]
pub struct AudioStream {
    /// Human-readable session name (`s=`).
    pub session_name: String,
    /// Originating node's unicast address (`o=` unicast-address) — the sender.
    pub origin_addr: Ipv4Addr,
    /// SDP session id (`o=` sess-id); a stable per-stream identifier.
    pub session_id: u64,
    /// SDP session version (`o=` sess-version); bumped on every change so
    /// receivers can tell a re-announcement from a modification.
    pub session_version: u64,
    /// Destination (multicast) group the RTP flows to (`c=`).
    pub dst_addr: Ipv4Addr,
    /// Destination UDP port (`m=audio <port>`).
    pub dst_port: u16,
    /// Multicast TTL/scope (the `/<ttl>` suffix on the `c=` line).
    pub ttl: u8,
    /// RTP dynamic payload type (`m=` fmt and the `rtpmap` PT).
    pub payload_type: u8,
    /// PCM sample encoding (`rtpmap` encoding name).
    pub encoding: Encoding,
    /// Media sample rate in Hz (`rtpmap` clock rate), e.g. 48000.
    pub sample_rate: u32,
    /// Channel count (`rtpmap` encoding parameters).
    pub channels: u8,
    /// Packet time in milliseconds (`a=ptime`), e.g. 1.0 for AES67's default.
    pub ptime_ms: f32,
    /// PTP grandmaster identity from `a=ts-refclk:ptp=...`, if announced (the
    /// EUI-64/clock-identity string, e.g. `00-1D-C1-FF-FE-01-02-03`).
    pub ptp_gmid: Option<String>,
}

impl AudioStream {
    /// A stream with AES67 defaults: port 5004, TTL 32, payload type 97, L24,
    /// 48 kHz, 2 channels, 1 ms packet time, no PTP grandmaster id. Override the
    /// fields that differ.
    pub fn new(session_name: impl Into<String>, origin_addr: Ipv4Addr, dst_addr: Ipv4Addr) -> Self {
        Self {
            session_name: session_name.into(),
            origin_addr,
            session_id: 0,
            session_version: 0,
            dst_addr,
            dst_port: 5004,
            ttl: 32,
            payload_type: 97,
            encoding: Encoding::L24,
            sample_rate: 48_000,
            channels: 2,
            ptime_ms: 1.0,
            ptp_gmid: None,
        }
    }

    /// Render this stream as an AES67 SDP session description (CRLF line endings,
    /// as SDP requires).
    pub fn to_sdp(&self) -> String {
        generate::to_sdp(self)
    }

    /// Parse an SDP session description into an [`AudioStream`], taking the first
    /// `m=audio` media block. Errors if it is not an IPv4 multicast PCM stream we
    /// can represent.
    pub fn from_sdp(sdp: &str) -> Result<Self, SdpError> {
        parse::from_sdp(sdp)
    }
}
