//! The receiving-node RTSP client: a one-shot [`describe`] plus a [`Subscription`]
//! that also consumes server-pushed `ANNOUNCE` updates (RAVENNA §3.4.1).

use std::io::Read;
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

use aes67_sdp::AudioStream;
use rtsp_types::{headers, Message, Method, ParseError, Request, Response, StatusCode, Url, Version};

use crate::RtspError;

/// Socket read timeout; reads simply retry on timeout, so this only bounds how
/// often a blocked read wakes (it is not an overall deadline).
const READ_TIMEOUT: Duration = Duration::from_secs(5);
/// Bounds the TCP connect so a one-shot DESCRIBE against an unreachable host
/// (e.g. from an mDNS browse hit) fails promptly instead of hanging.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(3);
/// Default RTSP port (RFC 2326) when the URL omits one.
const DEFAULT_PORT: u16 = 554;

/// One-shot `DESCRIBE`: connect, fetch and parse the SDP, then disconnect.
pub fn describe(url: &str) -> Result<AudioStream, RtspError> {
    Subscription::connect(url)?.describe()
}

/// A live RTSP client connection to one session: the initial [`describe`] plus
/// any server-pushed `ANNOUNCE` updates via [`next_update`].
///
/// [`describe`]: Subscription::describe
/// [`next_update`]: Subscription::next_update
pub struct Subscription {
    stream: TcpStream,
    url: Url,
    buf: Vec<u8>,
    cseq: u32,
}

impl Subscription {
    /// Connect to the RTSP server named by `url` (`rtsp://host[:port]/path`).
    pub fn connect(url: &str) -> Result<Self, RtspError> {
        let parsed = Url::parse(url).map_err(|_| RtspError::Url(url.into()))?;
        if parsed.scheme() != "rtsp" {
            return Err(RtspError::Url(format!("{url} (not an rtsp:// URL)")));
        }
        let host = parsed.host_str().ok_or_else(|| RtspError::Url(url.into()))?.to_string();
        let port = parsed.port().unwrap_or(DEFAULT_PORT);
        // Resolve and connect with a bounded timeout.
        let addr = (host.as_str(), port)
            .to_socket_addrs()?
            .next()
            .ok_or_else(|| RtspError::Url(format!("{url} (cannot resolve {host})")))?;
        let stream = TcpStream::connect_timeout(&addr, CONNECT_TIMEOUT)?;
        stream.set_read_timeout(Some(READ_TIMEOUT))?;
        Ok(Self { stream, url: parsed, buf: Vec::new(), cseq: 0 })
    }

    /// Send `DESCRIBE` and parse the returned SDP. `Err(NotFound)` on a 404 (the
    /// connection stays open, so [`next_update`](Self::next_update) can still wait
    /// for the server to `ANNOUNCE` the session later).
    pub fn describe(&mut self) -> Result<AudioStream, RtspError> {
        self.cseq += 1;
        Request::builder(Method::Describe, Version::V1_0)
            .request_uri(self.url.clone())
            .header(headers::CSEQ, self.cseq.to_string())
            .header(headers::ACCEPT, "application/sdp")
            .empty()
            .write(&mut self.stream)?;

        let resp = self.read_response()?;
        match resp.status() {
            StatusCode::Ok => sdp_from_body(resp.body()),
            StatusCode::NotFound => Err(RtspError::NotFound),
            other => Err(RtspError::Status(format!("{other:?}"))),
        }
    }

    /// Block for the next server-pushed `ANNOUNCE` and parse its SDP, replying
    /// `200 OK` so the server's CSeq accounting stays consistent.
    pub fn next_update(&mut self) -> Result<AudioStream, RtspError> {
        loop {
            if let Message::Request(req) = self.read_message()? {
                if *req.method() == Method::Announce {
                    if let Some(cseq) = req.header(&headers::CSEQ).cloned() {
                        Response::builder(Version::V1_0, StatusCode::Ok)
                            .header(headers::CSEQ, cseq)
                            .empty()
                            .write(&mut self.stream)?;
                    }
                    return sdp_from_body(req.body());
                }
            }
        }
    }

    fn read_response(&mut self) -> Result<Response<Vec<u8>>, RtspError> {
        loop {
            // Ignore any server-initiated requests until the response arrives.
            if let Message::Response(r) = self.read_message()? {
                return Ok(r);
            }
        }
    }

    /// Read and parse one complete RTSP message, reading more from the socket as
    /// needed (retrying on read timeouts).
    fn read_message(&mut self) -> Result<Message<Vec<u8>>, RtspError> {
        let mut tmp = [0u8; 4096];
        loop {
            match Message::<Vec<u8>>::parse(&self.buf) {
                Ok((msg, consumed)) => {
                    self.buf.drain(..consumed);
                    return Ok(msg);
                }
                Err(ParseError::Incomplete(_)) => {}
                Err(ParseError::Error) => return Err(RtspError::Protocol),
            }
            match self.stream.read(&mut tmp) {
                Ok(0) => return Err(RtspError::Closed),
                Ok(n) => self.buf.extend_from_slice(&tmp[..n]),
                Err(e)
                    if matches!(
                        e.kind(),
                        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                    ) => {}
                Err(e) => return Err(RtspError::Io(e)),
            }
        }
    }
}

fn sdp_from_body(body: &[u8]) -> Result<AudioStream, RtspError> {
    if body.is_empty() {
        return Err(RtspError::NoSdp);
    }
    let sdp = std::str::from_utf8(body).map_err(|_| RtspError::Protocol)?;
    Ok(AudioStream::from_sdp(sdp)?)
}
