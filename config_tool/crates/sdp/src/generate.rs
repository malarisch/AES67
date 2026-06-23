//! AES67 SDP generation.
//!
//! Hand-rolled rather than built through `sdp-rs`: the AES67 profile is a small,
//! fixed shape, so writing the lines directly is clearer than assembling the
//! library's strongly-typed line structs, and it pins the output to exactly the
//! canonical form. SDP mandates CRLF line endings.

use std::fmt::Write;

use crate::AudioStream;

/// Render `s` as an AES67 SDP session description.
pub fn to_sdp(s: &AudioStream) -> String {
    let mut out = String::with_capacity(256);
    // unwrap: writing into a String is infallible.
    let _ = write!(out, "v=0\r\n");
    let _ = write!(
        out,
        "o=- {} {} IN IP4 {}\r\n",
        s.session_id, s.session_version, s.origin_addr
    );
    let _ = write!(out, "s={}\r\n", s.session_name);
    let _ = write!(out, "c=IN IP4 {}/{}\r\n", s.dst_addr, s.ttl);
    let _ = write!(out, "t=0 0\r\n");
    let _ = write!(out, "m=audio {} RTP/AVP {}\r\n", s.dst_port, s.payload_type);
    let _ = write!(
        out,
        "a=rtpmap:{} {}/{}/{}\r\n",
        s.payload_type,
        s.encoding.name(),
        s.sample_rate,
        s.channels
    );
    let _ = write!(out, "a=ptime:{}\r\n", fmt_ptime(s.ptime_ms));
    if let Some(gmid) = &s.ptp_gmid {
        let _ = write!(out, "a=ts-refclk:ptp=IEEE1588-2008:{gmid}:0\r\n");
    }
    let _ = write!(out, "a=mediaclk:direct=0\r\n");
    out
}

/// Format `ptime` without a trailing `.0` for whole-millisecond values (the
/// common AES67 case prints `1`, not `1.0`), but keep fractional times intact.
fn fmt_ptime(ms: f32) -> String {
    if ms.fract() == 0.0 {
        format!("{}", ms as i64)
    } else {
        // Trim to a sensible precision; AES67 uses 0.125/0.25/… ms granularities.
        let s = format!("{ms:.4}");
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Encoding;
    use std::net::Ipv4Addr;

    fn sample() -> AudioStream {
        AudioStream {
            session_name: "AES67 stream 0".into(),
            origin_addr: Ipv4Addr::new(192, 168, 1, 1),
            session_id: 1311738121,
            session_version: 1311738121,
            dst_addr: Ipv4Addr::new(239, 69, 1, 0),
            dst_port: 5004,
            ttl: 32,
            payload_type: 97,
            encoding: Encoding::L24,
            sample_rate: 48_000,
            channels: 2,
            ptime_ms: 1.0,
            ptp_gmid: Some("00-1D-C1-FF-FE-01-02-03".into()),
        }
    }

    #[test]
    fn renders_canonical_aes67_sdp() {
        let sdp = to_sdp(&sample());
        let expected = "v=0\r\n\
            o=- 1311738121 1311738121 IN IP4 192.168.1.1\r\n\
            s=AES67 stream 0\r\n\
            c=IN IP4 239.69.1.0/32\r\n\
            t=0 0\r\n\
            m=audio 5004 RTP/AVP 97\r\n\
            a=rtpmap:97 L24/48000/2\r\n\
            a=ptime:1\r\n\
            a=ts-refclk:ptp=IEEE1588-2008:00-1D-C1-FF-FE-01-02-03:0\r\n\
            a=mediaclk:direct=0\r\n";
        assert_eq!(sdp, expected);
    }

    #[test]
    fn omits_ts_refclk_without_a_grandmaster() {
        let mut s = sample();
        s.ptp_gmid = None;
        assert!(!to_sdp(&s).contains("ts-refclk"));
    }

    #[test]
    fn fractional_ptime_is_kept() {
        assert_eq!(fmt_ptime(1.0), "1");
        assert_eq!(fmt_ptime(0.125), "0.125");
        assert_eq!(fmt_ptime(0.25), "0.25");
    }
}
