//! mDNS/DNS-SD for RAVENNA streaming sessions (Operating Principles §3.5).
//!
//! Three jobs on one [`Responder`] (one daemon thread / 5353 socket):
//!
//! * **Node service** — advertise this node's RTSP service as `_rtsp._tcp`
//!   (RAVENNA requires a session source to publish its RTSP service).
//! * **Session advertisement** (§3.5.2) — advertise each configured TX stream as
//!   `<session-name>._rtsp._tcp` carrying the `ravenna_session` subtype, so the
//!   stream is directly discoverable; register/unregister as the config changes.
//! * **Browse** — discover remote `ravenna_session` instances, RTSP-`DESCRIBE`
//!   each (via [`aes67_rtsp`]) to fetch its SDP, and feed the resulting stream
//!   into the shared discovery [`Registry`](crate::discovery::Registry) — the same
//!   one SAP fills, so clients see both.

use std::collections::{HashMap, HashSet};
use std::net::Ipv4Addr;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use aes67_mdns::{BrowseEvent, Responder, Service};

use crate::discovery::{session_name, wait_for_ip, SharedDiscovery, Transport};
use crate::{SharedConfig, SharedDevice};

/// DNS-SD subtype marking a RAVENNA streaming session.
const RAVENNA_SUBTYPE: &str = "ravenna_session";
/// Full browse target: RAVENNA session instances under `_rtsp._tcp`.
const SESSION_BROWSE: &str = "_ravenna_session._sub._rtsp._tcp.local.";
/// How often the session advertisement is reconciled against the live config.
const ADVERTISE_POLL: Duration = Duration::from_secs(5);

/// The DNS-SD full names this node is advertising itself, shared so the browse
/// loop can skip its own sessions (our own responder resolves them back to us,
/// and the resolved A-records are not a reliable self-test — they can arrive
/// empty or without the TAP address).
type OwnNames = Arc<Mutex<HashSet<String>>>;

/// Spawn the mDNS session service. Waits for the TAP address itself, so it need
/// not be sequenced with startup.
pub fn spawn(
    device: SharedDevice,
    config: SharedConfig,
    registry: SharedDiscovery,
    tap_name: String,
    rtsp_port: u16,
    node_name: String,
    verbose: u8,
) {
    std::thread::spawn(move || run(device, config, registry, tap_name, rtsp_port, node_name, verbose));
}

fn run(
    _device: SharedDevice,
    config: SharedConfig,
    registry: SharedDiscovery,
    tap_name: String,
    rtsp_port: u16,
    node_name: String,
    verbose: u8,
) {
    let local_ip = wait_for_ip(&tap_name);
    let responder = match Responder::new() {
        Ok(r) => r,
        Err(e) => {
            eprintln!("aes67d: mdns: cannot start responder: {e} — mDNS disabled");
            return;
        }
    };

    // Full names we advertise ourselves, so the browse loop skips our own sessions.
    let own: OwnNames = Arc::new(Mutex::new(HashSet::new()));

    // Node RTSP service (no subtype).
    match responder.register(&Service {
        service_type: "rtsp".into(),
        protocol: "tcp".into(),
        subtype: None,
        name: node_name,
        port: rtsp_port,
        txt: Vec::new(),
    }) {
        Ok(fullname) => {
            own.lock().unwrap().insert(fullname);
        }
        Err(e) => eprintln!("aes67d: mdns: cannot advertise node RTSP service: {e}"),
    }

    // Browse for remote sessions on a helper thread.
    match responder.browse(SESSION_BROWSE) {
        Ok(events) => {
            let reg = registry.clone();
            let own = own.clone();
            std::thread::spawn(move || browse_loop(events, reg, own, local_ip, verbose));
        }
        Err(e) => eprintln!("aes67d: mdns: cannot browse sessions: {e}"),
    }

    // Advertise our TX streams, reconciling against the live config.
    advertise_loop(&responder, &config, &own, rtsp_port, verbose);
}

/// Register/unregister `<session-name>._rtsp._tcp` (ravenna_session) per
/// configured TX stream, keyed by slot id. `advertised` maps slot → (name,
/// fullname) so a renamed or removed stream is withdrawn.
fn advertise_loop(
    responder: &Responder,
    config: &SharedConfig,
    own: &OwnNames,
    rtsp_port: u16,
    verbose: u8,
) {
    let mut advertised: HashMap<u8, (String, String)> = HashMap::new();
    loop {
        let tx_streams = config.lock().unwrap().settings.tx_streams.clone();
        // Desired set: streams with a parseable destination → their session name.
        let desired: HashMap<u8, String> = tx_streams
            .values()
            .filter(|p| p.dst_ip.parse::<Ipv4Addr>().is_ok())
            .map(|p| (p.id, session_name(p)))
            .collect();

        // Withdraw stale / renamed advertisements.
        advertised.retain(|id, (name, fullname)| match desired.get(id) {
            Some(d) if d == name => true,
            _ => {
                if let Err(e) = responder.unregister(fullname) {
                    eprintln!("aes67d: mdns: unregister {fullname} failed: {e}");
                } else if verbose >= 1 {
                    eprintln!("aes67d: mdns: withdrew session '{name}'");
                }
                own.lock().unwrap().remove(fullname);
                false
            }
        });

        // Register newly-present / renamed advertisements.
        for (id, name) in &desired {
            if advertised.contains_key(id) {
                continue;
            }
            let svc = Service {
                service_type: "rtsp".into(),
                protocol: "tcp".into(),
                subtype: Some(RAVENNA_SUBTYPE.into()),
                name: name.clone(),
                port: rtsp_port,
                txt: Vec::new(),
            };
            match responder.register(&svc) {
                Ok(fullname) => {
                    if verbose >= 1 {
                        eprintln!("aes67d: mdns: advertising session '{name}'");
                    }
                    own.lock().unwrap().insert(fullname.clone());
                    advertised.insert(*id, (name.clone(), fullname));
                }
                Err(e) => eprintln!("aes67d: mdns: advertise session '{name}' failed: {e}"),
            }
        }

        std::thread::sleep(ADVERTISE_POLL);
    }
}

/// Handle browse events: on a resolved remote session, RTSP-DESCRIBE it for the
/// SDP and register it; on removal, drop it. Our own sessions (resolving to
/// `local_ip`) are skipped.
fn browse_loop(
    events: std::sync::mpsc::Receiver<BrowseEvent>,
    registry: SharedDiscovery,
    own: OwnNames,
    local_ip: Ipv4Addr,
    verbose: u8,
) {
    for ev in events {
        match ev {
            BrowseEvent::Found(svc) => {
                // Skip our own advertisement: primarily by the names we publish
                // (deterministic), and as a backstop if it resolves to us.
                if own.lock().unwrap().contains(&svc.fullname) || svc.hosts.contains(&local_ip) {
                    continue;
                }
                // Describe via a routable address (skip loopback / link-local).
                let Some(&host) = svc
                    .hosts
                    .iter()
                    .find(|ip| !ip.is_loopback() && !ip.is_link_local())
                    .or_else(|| svc.hosts.first())
                else {
                    continue; // no address resolved
                };
                let url = format!("rtsp://{host}:{}/by-name/{}", svc.port, encode_segment(&svc.instance));
                match aes67_rtsp::describe(&url) {
                    Ok(stream) => {
                        if verbose >= 1 {
                            eprintln!(
                                "aes67d: discovery: + '{}' {} from {host} (mdns)",
                                stream.session_name, stream.dst_addr
                            );
                        }
                        registry.lock().unwrap().upsert(
                            format!("mdns:{}", svc.fullname),
                            stream,
                            host,
                            Transport::Mdns,
                        );
                    }
                    Err(e) if verbose >= 1 => eprintln!("aes67d: mdns: DESCRIBE {url} failed: {e}"),
                    Err(_) => {}
                }
            }
            BrowseEvent::Removed { fullname } => {
                if verbose >= 1 {
                    eprintln!("aes67d: discovery: - {fullname} (mdns)");
                }
                registry.lock().unwrap().remove(&format!("mdns:{fullname}"));
            }
        }
    }
}

/// Percent-encode a URL path segment (a session name may contain spaces etc.).
fn encode_segment(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for b in s.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => out.push(b as char),
            _ => out.push_str(&format!("%{b:02X}")),
        }
    }
    out
}
