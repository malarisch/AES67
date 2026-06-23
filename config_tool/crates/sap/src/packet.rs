//! SAP packet wire format (RFC 2974).
//!
//! ```text
//!  0               1               2               3
//!  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//! +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//! | V=1 |A|R|T|E|C|   auth len    |         msg id hash           |
//! +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//! |              originating source (32 bits for IPv4)            |
//! +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//! |       optional auth data (auth len * 32 bits)                |
//! |       optional payload type (NUL-terminated MIME string)     |
//! |       payload (the SDP text) ...                             |
//! ```
//!
//! We emit and accept only the common AES67 shape: IPv4, no authentication, no
//! encryption, no compression, an explicit `application/sdp` content type.

use std::net::Ipv4Addr;

/// Default SAP UDP port (RFC 2974).
pub const SAP_PORT: u16 = 9875;

/// The administratively-scoped SAP multicast group AES67/Ravenna devices use for
/// announcements (239.255.255.255). (RFC 2974 also defines the global-scope
/// 224.2.127.254, but the link-local admin group is what AES67 gear listens on.)
pub const SAP_GROUP: Ipv4Addr = Ipv4Addr::new(239, 255, 255, 255);

/// The MIME content type carried before the SDP payload.
const SDP_CONTENT_TYPE: &str = "application/sdp";

/// SAP message type: a session announcement or a session deletion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SapKind {
    Announce,
    Delete,
}

/// A decoded SAP packet carrying an SDP payload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SapPacket {
    pub kind: SapKind,
    /// 16-bit message id hash; together with `source` it identifies a session,
    /// so a deletion can be matched to its announcement.
    pub msg_id_hash: u16,
    /// Originating source address from the header.
    pub source: Ipv4Addr,
    /// The SDP payload (announcement) or the SDP/origin to delete.
    pub sdp: String,
}

/// Why a received datagram is not a SAP/SDP packet we handle.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum SapError {
    #[error("datagram too short ({0} bytes)")]
    TooShort(usize),
    #[error("unsupported SAP version {0} (expected 1)")]
    Version(u8),
    #[error("IPv6 SAP source is not supported")]
    Ipv6Source,
    #[error("encrypted SAP packets are not supported")]
    Encrypted,
    #[error("compressed SAP packets are not supported")]
    Compressed,
    #[error("unsupported content type '{0}' (expected application/sdp)")]
    ContentType(String),
    #[error("payload is not valid UTF-8")]
    NotUtf8,
}

// Byte-0 bit layout (MSB first): VVV A R T E C.
const VERSION_SHIFT: u8 = 5;
const FLAG_ADDR_IPV6: u8 = 1 << 4;
const FLAG_DELETE: u8 = 1 << 2;
const FLAG_ENCRYPTED: u8 = 1 << 1;
const FLAG_COMPRESSED: u8 = 1 << 0;

impl SapPacket {
    /// Build an announcement for `sdp` from `source`. The message id hash is
    /// derived from the SDP origin so a later [`delete`](SapPacket::delete) of the
    /// same session carries a matching hash.
    pub fn announce(source: Ipv4Addr, sdp: impl Into<String>) -> Self {
        let sdp = sdp.into();
        Self { kind: SapKind::Announce, msg_id_hash: origin_hash(&sdp), source, sdp }
    }

    /// Build a deletion for `sdp` from `source` (same hashing as `announce`).
    pub fn delete(source: Ipv4Addr, sdp: impl Into<String>) -> Self {
        let sdp = sdp.into();
        Self { kind: SapKind::Delete, msg_id_hash: origin_hash(&sdp), source, sdp }
    }

    /// Serialise to the on-the-wire SAP datagram.
    pub fn encode(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(8 + SDP_CONTENT_TYPE.len() + 1 + self.sdp.len());
        let mut byte0 = 1u8 << VERSION_SHIFT; // V=1, IPv4, not encrypted/compressed
        if self.kind == SapKind::Delete {
            byte0 |= FLAG_DELETE;
        }
        buf.push(byte0);
        buf.push(0); // auth len: no authentication
        buf.extend_from_slice(&self.msg_id_hash.to_be_bytes());
        buf.extend_from_slice(&self.source.octets());
        // Explicit content type, NUL-terminated, then the SDP payload.
        buf.extend_from_slice(SDP_CONTENT_TYPE.as_bytes());
        buf.push(0);
        buf.extend_from_slice(self.sdp.as_bytes());
        buf
    }

    /// Parse a received SAP datagram.
    pub fn decode(buf: &[u8]) -> Result<Self, SapError> {
        if buf.len() < 8 {
            return Err(SapError::TooShort(buf.len()));
        }
        let byte0 = buf[0];
        let version = byte0 >> VERSION_SHIFT;
        if version != 1 {
            return Err(SapError::Version(version));
        }
        if byte0 & FLAG_ADDR_IPV6 != 0 {
            return Err(SapError::Ipv6Source);
        }
        if byte0 & FLAG_ENCRYPTED != 0 {
            return Err(SapError::Encrypted);
        }
        if byte0 & FLAG_COMPRESSED != 0 {
            return Err(SapError::Compressed);
        }
        let kind = if byte0 & FLAG_DELETE != 0 { SapKind::Delete } else { SapKind::Announce };

        let auth_len = buf[1] as usize * 4; // auth len counts 32-bit words
        let msg_id_hash = u16::from_be_bytes([buf[2], buf[3]]);
        let source = Ipv4Addr::new(buf[4], buf[5], buf[6], buf[7]);

        let mut pos = 8 + auth_len;
        if pos > buf.len() {
            return Err(SapError::TooShort(buf.len()));
        }
        let rest = &buf[pos..];

        // Optional content type: a NUL-terminated MIME string preceding the
        // payload. RFC 2974 allows omitting it when the payload starts with "v="
        // (then it is assumed to be SDP).
        if rest.starts_with(b"v=") {
            // No content type; the payload is the rest.
        } else if let Some(nul) = rest.iter().position(|&b| b == 0) {
            let ct = std::str::from_utf8(&rest[..nul]).map_err(|_| SapError::NotUtf8)?;
            if !ct.eq_ignore_ascii_case(SDP_CONTENT_TYPE) {
                return Err(SapError::ContentType(ct.to_string()));
            }
            pos += nul + 1;
        } else {
            return Err(SapError::ContentType("<unterminated>".into()));
        }

        let sdp = std::str::from_utf8(&buf[pos..]).map_err(|_| SapError::NotUtf8)?.to_string();
        Ok(Self { kind, msg_id_hash, source, sdp })
    }
}

/// A 16-bit hash over the SDP `o=` (origin) line so announce/delete of the same
/// session share a message id hash. Falls back to hashing the whole text if no
/// origin line is present.
pub fn origin_hash(sdp: &str) -> u16 {
    let key = sdp.lines().find(|l| l.starts_with("o=")).unwrap_or(sdp);
    // FNV-1a folded to 16 bits — small, dependency-free, stable.
    let mut h: u32 = 0x811c_9dc5;
    for b in key.bytes() {
        h ^= b as u32;
        h = h.wrapping_mul(0x0100_0193);
    }
    ((h >> 16) ^ (h & 0xffff)) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    const SDP: &str = "v=0\r\no=- 5 5 IN IP4 192.168.1.1\r\ns=x\r\n";

    #[test]
    fn announce_round_trips() {
        let src = Ipv4Addr::new(192, 168, 1, 1);
        let pkt = SapPacket::announce(src, SDP);
        let back = SapPacket::decode(&pkt.encode()).unwrap();
        assert_eq!(back, pkt);
        assert_eq!(back.kind, SapKind::Announce);
        assert_eq!(back.source, src);
        assert_eq!(back.sdp, SDP);
    }

    #[test]
    fn delete_keeps_the_announce_hash() {
        let src = Ipv4Addr::new(10, 0, 0, 9);
        let ann = SapPacket::announce(src, SDP);
        let del = SapPacket::delete(src, SDP);
        assert_eq!(del.kind, SapKind::Delete);
        assert_eq!(del.msg_id_hash, ann.msg_id_hash);
        assert_eq!(SapPacket::decode(&del.encode()).unwrap().kind, SapKind::Delete);
    }

    #[test]
    fn decodes_without_a_content_type() {
        // Header + SDP starting with "v=" and no MIME prefix.
        let mut buf = vec![0x20, 0, 0x12, 0x34, 192, 168, 1, 1];
        buf.extend_from_slice(SDP.as_bytes());
        let pkt = SapPacket::decode(&buf).unwrap();
        assert_eq!(pkt.sdp, SDP);
        assert_eq!(pkt.msg_id_hash, 0x1234);
    }

    #[test]
    fn skips_auth_data() {
        let mut pkt = SapPacket::announce(Ipv4Addr::new(1, 2, 3, 4), SDP);
        pkt.msg_id_hash = 0xbeef;
        let mut bytes = pkt.encode();
        // Splice in one 32-bit auth word and bump the auth-len field.
        bytes[1] = 1;
        bytes.splice(8..8, [0xaa, 0xbb, 0xcc, 0xdd]);
        let back = SapPacket::decode(&bytes).unwrap();
        assert_eq!(back.sdp, SDP);
        assert_eq!(back.msg_id_hash, 0xbeef);
    }

    #[test]
    fn rejects_foreign_content_type() {
        let mut buf = vec![0x20, 0, 0, 0, 1, 1, 1, 1];
        buf.extend_from_slice(b"application/xml\0<x/>");
        assert!(matches!(SapPacket::decode(&buf), Err(SapError::ContentType(_))));
    }

    #[test]
    fn rejects_short_and_wrong_version() {
        assert!(matches!(SapPacket::decode(&[0; 4]), Err(SapError::TooShort(4))));
        // version 2 in the top three bits.
        let buf = [0x40, 0, 0, 0, 1, 1, 1, 1, b'v', b'='];
        assert!(matches!(SapPacket::decode(&buf), Err(SapError::Version(2))));
    }
}
