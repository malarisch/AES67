//! RAVENNA-style RTSP connection management (RFC 2326 / RAVENNA Operating
//! Principles §3.4).
//!
//! RAVENNA uses RTSP only to exchange the SDP *session description* — the audio
//! itself is separate multicast RTP described by that SDP. So this crate is small
//! and specific, layered on the pure-Rust [`rtsp_types`] message codec:
//!
//! * [`server`] — a transmitting node's RTSP **server**. It answers `OPTIONS` and
//!   `DESCRIBE` (on the RAVENNA `/by-id/<id>` and `/by-name/<name>` paths) with
//!   the session's SDP, returns `404 Not Found` for sessions that do not (yet)
//!   exist while **keeping the connection open**, and pushes an `ANNOUNCE` once a
//!   watched session appears or changes (§3.4.1).
//! * [`client`] — a receiving node's RTSP **client**: a one-shot [`describe`] plus
//!   a [`Subscription`] that also consumes server-pushed `ANNOUNCE` updates.
//!
//! SDP payloads are modelled by [`aes67_sdp`]. The crate is pure Rust and has no
//! C dependencies, so it cross-compiles cleanly.

mod client;
mod server;

pub use client::{describe, Subscription};
pub use server::{serve, Sessions};

/// A streaming session addressed by an RTSP URL path (RAVENNA §3.5.3.2/§3.5.3.3):
/// `/by-id/<id>` or `/by-name/<name>`.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum SessionTarget {
    /// `/by-id/<id>` — the device-local numeric session id.
    ById(u32),
    /// `/by-name/<name>` — a named session.
    ByName(String),
}

impl SessionTarget {
    /// Parse the RAVENNA session path of an RTSP request URI (percent-decoded).
    /// `None` if it is not a recognised `/by-id/` or `/by-name/` path.
    pub fn from_path(path: &str) -> Option<Self> {
        let path = percent_decode(path.trim_start_matches('/'));
        if let Some(id) = path.strip_prefix("by-id/") {
            return id.trim_end_matches('/').parse().ok().map(SessionTarget::ById);
        }
        if let Some(name) = path.strip_prefix("by-name/") {
            let name = name.trim_end_matches('/');
            if !name.is_empty() {
                return Some(SessionTarget::ByName(name.to_string()));
            }
        }
        None
    }

    /// The RTSP URL path for this target (e.g. `/by-id/3`).
    pub fn to_path(&self) -> String {
        match self {
            SessionTarget::ById(id) => format!("/by-id/{id}"),
            SessionTarget::ByName(name) => format!("/by-name/{name}"),
        }
    }
}

/// Errors from the RTSP client/server.
#[derive(Debug, thiserror::Error)]
pub enum RtspError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("serialising RTSP message: {0}")]
    Write(#[from] rtsp_types::WriteError),
    #[error("malformed RTSP message")]
    Protocol,
    #[error("invalid RTSP URL '{0}'")]
    Url(String),
    #[error("server replied {0}")]
    Status(String),
    #[error("session not found")]
    NotFound,
    #[error("response carried no SDP body")]
    NoSdp,
    #[error("invalid SDP: {0}")]
    Sdp(#[from] aes67_sdp::SdpError),
    #[error("connection closed")]
    Closed,
}

/// Minimal percent-decoding for request paths (a session name may contain spaces,
/// encoded as `%20`).
fn percent_decode(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'%' && i + 2 < b.len() {
            if let (Some(h), Some(l)) = (hex(b[i + 1]), hex(b[i + 2])) {
                out.push(h * 16 + l);
                i += 3;
                continue;
            }
        }
        out.push(b[i]);
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

fn hex(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_by_id_and_by_name_paths() {
        assert_eq!(SessionTarget::from_path("/by-id/0815"), Some(SessionTarget::ById(815)));
        assert_eq!(
            SessionTarget::from_path("/by-name/Stage%20left"),
            Some(SessionTarget::ByName("Stage left".into()))
        );
        assert_eq!(SessionTarget::from_path("/by-name/Line"), Some(SessionTarget::ByName("Line".into())));
        assert_eq!(SessionTarget::from_path("/"), None);
        assert_eq!(SessionTarget::from_path("/by-id/x"), None);
        assert_eq!(SessionTarget::from_path("/by-name/"), None);
    }

    #[test]
    fn to_path_round_trips() {
        for t in [SessionTarget::ById(7), SessionTarget::ByName("Line".into())] {
            assert_eq!(SessionTarget::from_path(&t.to_path()), Some(t));
        }
    }
}
