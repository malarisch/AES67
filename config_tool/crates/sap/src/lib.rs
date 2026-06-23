//! Session Announcement Protocol (RFC 2974) for AES67 discovery.
//!
//! Two layers, kept apart so the wire format is testable without sockets:
//!
//! * [`packet`] — the [`SapPacket`] codec (header + `application/sdp` payload).
//! * [`net`] — a [`SapSender`] and [`SapListener`] over a `UdpSocket`.
//!
//! This crate carries an opaque SDP string; turning that into an AES67 stream is
//! [`aes67-sdp`](../aes67_sdp)'s job. The daemon wires the two together (and owns
//! the socket bound to the TAP, joined to [`SAP_GROUP`]). A future mDNS discovery
//! layer would reuse `aes67-sdp` the same way, with its own transport in place of
//! this crate.

mod net;
mod packet;

pub use net::{SapListener, SapSender};
pub use packet::{origin_hash, SapError, SapKind, SapPacket, SAP_GROUP, SAP_PORT};
