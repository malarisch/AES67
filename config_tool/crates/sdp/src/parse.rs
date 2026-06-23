//! AES67 SDP parsing, on top of the `sdp-rs` tokeniser.
//!
//! We take the first `m=audio` block and pull out exactly the fields the
//! [`AudioStream`] model carries. Anything that is not an IPv4 multicast linear
//! PCM stream is rejected — a discovery layer should skip descriptions it cannot
//! turn into an FPGA RX stream rather than guess.

use std::net::{IpAddr, Ipv4Addr};

use sdp_rs::lines::Attribute;
use sdp_rs::{MediaDescription, SessionDescription};

use crate::{AudioStream, Encoding};

/// Why an SDP description could not be turned into an [`AudioStream`].
#[derive(Debug, thiserror::Error)]
pub enum SdpError {
    /// The `sdp-rs` tokeniser/parser rejected the input.
    #[error("malformed SDP: {0}")]
    Malformed(String),
    /// No `m=audio` media description was present.
    #[error("no audio media description")]
    NoAudio,
    /// A required line/attribute was missing (e.g. no `c=` or no `rtpmap`).
    #[error("missing {0}")]
    Missing(&'static str),
    /// A field was present but not something we can represent (e.g. IPv6, a
    /// non-PCM encoding, or a non-multicast destination).
    #[error("unsupported: {0}")]
    Unsupported(String),
}

/// Parse an SDP string into an [`AudioStream`].
pub fn from_sdp(sdp: &str) -> Result<AudioStream, SdpError> {
    let sd = SessionDescription::try_from(sdp).map_err(|e| SdpError::Malformed(e.to_string()))?;

    let media = sd
        .media_descriptions
        .iter()
        .find(|m| m.media.media.to_string() == "audio")
        .ok_or(SdpError::NoAudio)?;

    let origin_addr = require_ipv4(sd.origin.unicast_address, "o= unicast-address")?;

    // The destination group may sit on the media block or, less commonly, on the
    // session block; prefer the media-level `c=`.
    let conn = media
        .connections
        .first()
        .or(sd.connection.as_ref())
        .ok_or(SdpError::Missing("c= connection"))?;
    let dst_addr = require_ipv4(conn.connection_address.base, "c= address")?;
    if !dst_addr.is_multicast() {
        return Err(SdpError::Unsupported(format!("destination {dst_addr} is not multicast")));
    }
    let ttl = conn.connection_address.ttl.unwrap_or(32).min(255) as u8;

    let payload_type = first_fmt(&media.media.fmt)?;
    let rtpmap = find_rtpmap(media, payload_type)?;
    let encoding = Encoding::from_name(&rtpmap.encoding_name)
        .ok_or_else(|| SdpError::Unsupported(format!("encoding {}", rtpmap.encoding_name)))?;
    let channels = rtpmap.encoding_params.unwrap_or(1).clamp(1, 255) as u8;
    let sample_rate = rtpmap.clock_rate.max(0) as u32;

    let ptime_ms = attrs(media, &sd)
        .find_map(|a| match a {
            Attribute::Ptime(p) => Some(*p),
            _ => None,
        })
        .unwrap_or(1.0);

    let ptp_gmid = attrs(media, &sd).find_map(|a| match a {
        Attribute::Other(k, Some(v)) if k.eq_ignore_ascii_case("ts-refclk") => parse_gmid(v),
        _ => None,
    });

    // RAVENNA `a=clock-domain:PTPv2 <domain>` (session-level) and
    // `a=sync-time:<rtp-timestamp>` (stream-level), accepted alongside the AES67
    // `ts-refclk`/`mediaclk` attributes above.
    let ptp_domain = attrs(media, &sd).find_map(|a| match a {
        Attribute::Other(k, Some(v)) if k.eq_ignore_ascii_case("clock-domain") => parse_clock_domain(v),
        _ => None,
    });
    let sync_time = attrs(media, &sd).find_map(|a| match a {
        Attribute::Other(k, Some(v)) if k.eq_ignore_ascii_case("sync-time") => v.trim().parse().ok(),
        _ => None,
    });

    Ok(AudioStream {
        session_name: sd.session_name.value().to_string(),
        origin_addr,
        session_id: sd.origin.sess_id.parse().unwrap_or(0),
        session_version: sd.origin.sess_version.parse().unwrap_or(0),
        dst_addr,
        dst_port: media.media.port,
        ttl,
        payload_type,
        encoding,
        sample_rate,
        channels,
        ptime_ms,
        ptp_gmid,
        ptp_domain,
        sync_time,
    })
}

/// Parse the domain out of `a=clock-domain:PTPv2 <domain>`. Only the PTPv2 sync
/// source is defined by RAVENNA; anything else yields `None`.
fn parse_clock_domain(v: &str) -> Option<u8> {
    let mut parts = v.split_whitespace();
    match parts.next() {
        Some(src) if src.eq_ignore_ascii_case("PTPv2") => parts.next()?.parse().ok(),
        _ => None,
    }
}

/// Media-level attributes followed by session-level ones (media takes priority
/// since `find`/`find_map` stops at the first hit).
fn attrs<'a>(
    media: &'a MediaDescription,
    sd: &'a SessionDescription,
) -> impl Iterator<Item = &'a Attribute> {
    media.attributes.iter().chain(sd.attributes.iter())
}

/// The first payload type listed on the `m=` line.
fn first_fmt(fmt: &str) -> Result<u8, SdpError> {
    fmt.split_whitespace()
        .next()
        .and_then(|t| t.parse().ok())
        .ok_or(SdpError::Missing("m= payload type"))
}

/// The `rtpmap` for `pt`. Falls back to the sole `rtpmap` if none names `pt`
/// (some senders mismatch the PT), and errors if there is none at all.
fn find_rtpmap(
    media: &MediaDescription,
    pt: u8,
) -> Result<&sdp_rs::lines::attribute::Rtpmap, SdpError> {
    let mut only = None;
    let mut count = 0;
    for a in &media.attributes {
        if let Attribute::Rtpmap(r) = a {
            count += 1;
            only = Some(r);
            if r.payload_type == pt as u32 {
                return Ok(r);
            }
        }
    }
    if count == 1 {
        return Ok(only.unwrap());
    }
    Err(SdpError::Missing("a=rtpmap"))
}

/// Pull the clock-identity out of `ptp=IEEE1588-2008:<gmid>:<domain>`.
fn parse_gmid(v: &str) -> Option<String> {
    // value form: "ptp=IEEE1588-2008:00-1D-C1-FF-FE-01-02-03:0"
    let rest = v.strip_prefix("ptp=")?;
    // Drop the PTP profile token ("IEEE1588-2008"), leaving "<gmid>:<domain>".
    let (_ptp_profile, tail) = rest.split_once(':')?;
    // The gmid is everything up to the final ":<domain>", if present.
    match tail.rsplit_once(':') {
        Some((gmid, _domain)) => Some(gmid.to_string()),
        None => Some(tail.to_string()),
    }
}

fn require_ipv4(ip: IpAddr, what: &'static str) -> Result<Ipv4Addr, SdpError> {
    match ip {
        IpAddr::V4(v4) => Ok(v4),
        IpAddr::V6(_) => Err(SdpError::Unsupported(format!("{what} is IPv6"))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = "v=0\r\n\
        o=- 1311738121 1311738121 IN IP4 192.168.1.1\r\n\
        s=AES67 stream 0\r\n\
        c=IN IP4 239.69.1.0/32\r\n\
        t=0 0\r\n\
        a=clock-domain:PTPv2 0\r\n\
        m=audio 5004 RTP/AVP 97\r\n\
        a=rtpmap:97 L24/48000/2\r\n\
        a=ptime:1\r\n\
        a=ts-refclk:ptp=IEEE1588-2008:00-1D-C1-FF-FE-01-02-03:0\r\n\
        a=mediaclk:direct=0\r\n\
        a=sync-time:1234567\r\n";

    #[test]
    fn parses_the_aes67_and_ravenna_fields() {
        let s = from_sdp(SAMPLE).unwrap();
        assert_eq!(s.session_name, "AES67 stream 0");
        assert_eq!(s.origin_addr, Ipv4Addr::new(192, 168, 1, 1));
        assert_eq!(s.session_id, 1311738121);
        assert_eq!(s.dst_addr, Ipv4Addr::new(239, 69, 1, 0));
        assert_eq!(s.dst_port, 5004);
        assert_eq!(s.ttl, 32);
        assert_eq!(s.payload_type, 97);
        assert_eq!(s.encoding, Encoding::L24);
        assert_eq!(s.sample_rate, 48_000);
        assert_eq!(s.channels, 2);
        assert_eq!(s.ptime_ms, 1.0);
        assert_eq!(s.ptp_gmid.as_deref(), Some("00-1D-C1-FF-FE-01-02-03"));
        assert_eq!(s.ptp_domain, Some(0));
        assert_eq!(s.sync_time, Some(1234567));
    }

    #[test]
    fn round_trips_through_generation() {
        let s = from_sdp(SAMPLE).unwrap();
        let again = from_sdp(&s.to_sdp()).unwrap();
        assert_eq!(s, again);
    }

    #[test]
    fn rejects_unicast_destination() {
        let sdp = SAMPLE.replace("239.69.1.0", "192.168.1.50");
        assert!(matches!(from_sdp(&sdp), Err(SdpError::Unsupported(_))));
    }

    #[test]
    fn rejects_non_pcm_encoding() {
        let sdp = SAMPLE.replace("L24/48000/2", "opus/48000/2");
        assert!(matches!(from_sdp(&sdp), Err(SdpError::Unsupported(_))));
    }

    #[test]
    fn errors_without_audio_media() {
        let sdp = SAMPLE.replace("m=audio", "m=video");
        assert!(matches!(from_sdp(&sdp), Err(SdpError::NoAudio)));
    }
}
