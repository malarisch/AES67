//! The transmitting-node RTSP server: answers `DESCRIBE`/`OPTIONS` and pushes
//! `ANNOUNCE` updates (RAVENNA §3.4.1).

use std::collections::HashMap;
use std::io::Read;
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;
use std::time::Duration;

use rtsp_types::{headers, Message, Method, ParseError, Request, Response, StatusCode, Url, Version};

use crate::{RtspError, SessionTarget};

/// How long a connection blocks on a read before waking to push `ANNOUNCE`s for
/// sessions that have appeared/changed. Server-side only — clients never poll.
const POLL: Duration = Duration::from_millis(1000);

/// Provides the SDP for the streaming sessions this node hosts.
pub trait Sessions: Send + Sync {
    /// The SDP for `target`, or `None` if no such (configured and started)
    /// session exists yet — RAVENNA then expects a `404`, with the connection
    /// kept open so the server can `ANNOUNCE` it once it appears.
    fn describe(&self, target: &SessionTarget) -> Option<String>;
}

/// Serve RTSP on `listener` until it stops yielding connections, one thread per
/// connection. Intended to run on its own thread.
pub fn serve(listener: TcpListener, sessions: Arc<dyn Sessions>) {
    for conn in listener.incoming() {
        match conn {
            Ok(stream) => {
                let sessions = Arc::clone(&sessions);
                std::thread::spawn(move || {
                    if let Err(e) = handle(stream, sessions) {
                        if !matches!(e, RtspError::Closed) {
                            eprintln!("aes67 rtsp: connection ended: {e}");
                        }
                    }
                });
            }
            Err(e) => eprintln!("aes67 rtsp: accept error: {e}"),
        }
    }
}

/// A session a client DESCRIBEd on this connection, tracked so changes can be
/// pushed: `uri` is the URL it used (the ANNOUNCE target), `last` the SDP last
/// delivered (`None` after a 404, until the session appears).
struct Watch {
    uri: Url,
    last: Option<String>,
}

fn handle(mut stream: TcpStream, sessions: Arc<dyn Sessions>) -> Result<(), RtspError> {
    stream.set_read_timeout(Some(POLL))?;
    let mut buf: Vec<u8> = Vec::new();
    let mut tmp = [0u8; 4096];
    let mut watched: HashMap<SessionTarget, Watch> = HashMap::new();
    let mut server_cseq: u32 = 0;

    loop {
        // Drain every complete request already buffered.
        loop {
            match Message::<Vec<u8>>::parse(&buf) {
                Ok((msg, consumed)) => {
                    buf.drain(..consumed);
                    if let Message::Request(req) = msg {
                        handle_request(&req, &sessions, &mut stream, &mut watched)?;
                    }
                }
                Err(ParseError::Incomplete(_)) => break,
                Err(ParseError::Error) => return Err(RtspError::Protocol),
            }
        }
        // Push ANNOUNCE for watched sessions that became available or changed.
        push_updates(&sessions, &mut stream, &mut watched, &mut server_cseq)?;
        // Wait for more data, timing out to re-run the push check.
        match stream.read(&mut tmp) {
            Ok(0) => return Ok(()), // client closed
            Ok(n) => buf.extend_from_slice(&tmp[..n]),
            Err(e) if would_block(&e) => {}
            Err(e) => return Err(RtspError::Io(e)),
        }
    }
}

fn handle_request(
    req: &Request<Vec<u8>>,
    sessions: &Arc<dyn Sessions>,
    stream: &mut TcpStream,
    watched: &mut HashMap<SessionTarget, Watch>,
) -> Result<(), RtspError> {
    let cseq = req.header(&headers::CSEQ).cloned();
    // Start a response builder pre-seeded with the echoed CSeq.
    let resp = |status: StatusCode| {
        let mut b = Response::builder(Version::V1_0, status);
        if let Some(c) = &cseq {
            b = b.header(headers::CSEQ, c.clone());
        }
        b
    };

    match req.method() {
        Method::Options => {
            resp(StatusCode::Ok)
                .header(headers::PUBLIC, "OPTIONS, DESCRIBE")
                .empty()
                .write(stream)?;
        }
        Method::Describe => {
            let uri = req.request_uri().cloned();
            let target = uri.as_ref().and_then(|u| SessionTarget::from_path(u.path()));
            match target {
                Some(t) => match sessions.describe(&t) {
                    Some(sdp) => {
                        resp(StatusCode::Ok)
                            .header(headers::CONTENT_TYPE, "application/sdp")
                            .build(sdp.clone().into_bytes())
                            .write(stream)?;
                        if let Some(u) = uri {
                            watched.insert(t, Watch { uri: u, last: Some(sdp) });
                        }
                    }
                    None => {
                        // 404, but keep the connection open and remember the
                        // request so we can ANNOUNCE the session once it appears.
                        resp(StatusCode::NotFound).empty().write(stream)?;
                        if let Some(u) = uri {
                            watched.entry(t).or_insert(Watch { uri: u, last: None });
                        }
                    }
                },
                None => resp(StatusCode::BadRequest).empty().write(stream)?,
            }
        }
        _ => resp(StatusCode::MethodNotAllowed).empty().write(stream)?,
    }
    Ok(())
}

/// For each watched session whose SDP has appeared or changed since we last sent
/// it, push an `ANNOUNCE` with the new description.
fn push_updates(
    sessions: &Arc<dyn Sessions>,
    stream: &mut TcpStream,
    watched: &mut HashMap<SessionTarget, Watch>,
    server_cseq: &mut u32,
) -> Result<(), RtspError> {
    for (target, watch) in watched.iter_mut() {
        let current = sessions.describe(target);
        if current == watch.last {
            continue;
        }
        if let Some(sdp) = &current {
            *server_cseq += 1;
            Request::builder(Method::Announce, Version::V1_0)
                .request_uri(watch.uri.clone())
                .header(headers::CSEQ, server_cseq.to_string())
                .header(headers::CONTENT_TYPE, "application/sdp")
                .build(sdp.clone().into_bytes())
                .write(stream)?;
        }
        watch.last = current;
    }
    Ok(())
}

fn would_block(e: &std::io::Error) -> bool {
    matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{describe, Subscription};
    use std::net::Ipv4Addr;
    use std::sync::Mutex;

    fn sample_sdp(name: &str) -> String {
        aes67_sdp::AudioStream::new(
            name,
            Ipv4Addr::new(192, 168, 1, 1),
            Ipv4Addr::new(239, 69, 1, 0),
        )
        .to_sdp()
    }

    struct Fixed(String);
    impl Sessions for Fixed {
        fn describe(&self, t: &SessionTarget) -> Option<String> {
            matches!(t, SessionTarget::ById(0)).then(|| self.0.clone())
        }
    }

    fn spawn_server(sessions: Arc<dyn Sessions>) -> u16 {
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let port = listener.local_addr().unwrap().port();
        std::thread::spawn(move || serve(listener, sessions));
        port
    }

    #[test]
    fn describe_returns_sdp_and_404() {
        let port = spawn_server(Arc::new(Fixed(sample_sdp("Line"))));
        let s = describe(&format!("rtsp://127.0.0.1:{port}/by-id/0")).unwrap();
        assert_eq!(s.dst_addr, Ipv4Addr::new(239, 69, 1, 0));
        assert!(matches!(
            describe(&format!("rtsp://127.0.0.1:{port}/by-id/9")),
            Err(RtspError::NotFound)
        ));
    }

    /// A session that starts absent and can be switched on, to exercise the
    /// 404-keep-alive → ANNOUNCE push path.
    struct Lazy(Mutex<Option<String>>);
    impl Sessions for Lazy {
        fn describe(&self, t: &SessionTarget) -> Option<String> {
            if matches!(t, SessionTarget::ById(0)) {
                self.0.lock().unwrap().clone()
            } else {
                None
            }
        }
    }

    #[test]
    fn announce_pushed_when_session_appears() {
        let lazy = Arc::new(Lazy(Mutex::new(None)));
        let port = spawn_server(Arc::clone(&lazy) as Arc<dyn Sessions>);

        let mut sub = Subscription::connect(&format!("rtsp://127.0.0.1:{port}/by-id/0")).unwrap();
        assert!(matches!(sub.describe(), Err(RtspError::NotFound)));

        // Switch the session on; the server pushes ANNOUNCE within one poll tick.
        *lazy.0.lock().unwrap() = Some(sample_sdp("Now Live"));
        let s = sub.next_update().unwrap();
        assert_eq!(s.session_name, "Now Live");
    }
}
