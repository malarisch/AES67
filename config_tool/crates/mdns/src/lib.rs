//! mDNS / DNS-SD service announcement over [`mdns_sd`] — pure Rust, no C
//! libraries to link and no `avahi-daemon` needed at runtime.
//!
//! The first user is the web dashboard advertising itself as `_http._tcp`, so the
//! config tool shows up in any mDNS browser ("AES67 Configuration") without the
//! operator hunting for an IP. The API is deliberately a thin wrapper: a plain,
//! `Send` [`Service`] description plus [`spawn`], which owns the `mdns_sd`
//! responder for the life of the process.
//!
//! A pure-Rust responder (rather than binding the system Avahi) keeps the tool's
//! "no native C dependencies → cross-compiles cleanly" property — see the README.
//! `mdns_sd` runs its own background thread and coexists with a system responder
//! on UDP 5353. A later step will add the browse/resolve side here (reusing
//! [`aes67-sdp`](../aes67_sdp) for AES67 stream discovery over mDNS), alongside
//! the SAP path in [`aes67-sap`](../aes67_sap).

use std::ffi::CStr;
use std::thread::{self, JoinHandle};

use std::net::Ipv4Addr;
use std::sync::mpsc::{self, Receiver};

use mdns_sd::{ServiceDaemon, ServiceEvent, ServiceInfo};

/// Errors from the mDNS responder.
#[derive(Debug, thiserror::Error)]
pub enum MdnsError {
    #[error("mDNS error: {0}")]
    Daemon(#[from] mdns_sd::Error),
}

/// A service to advertise on the local link.
#[derive(Debug, Clone)]
pub struct Service {
    /// DNS-SD service type, without the leading underscore, e.g. `"http"`.
    pub service_type: String,
    /// Transport protocol: `"tcp"` or `"udp"`.
    pub protocol: String,
    /// DNS-SD subtype, without underscores, e.g. `"ravenna_session"`. Advertised
    /// as `_<subtype>._sub._<service_type>._<protocol>` so browsers can filter.
    pub subtype: Option<String>,
    /// Human-readable instance name. mDNS resolves collisions by renaming, so it
    /// need not be unique.
    pub name: String,
    /// Port the service listens on.
    pub port: u16,
    /// TXT key/value records (e.g. `("path", "/")` for an HTTP service).
    pub txt: Vec<(String, String)>,
}

impl Service {
    /// An `_http._tcp` service with the conventional `path=/` TXT record.
    pub fn http(name: impl Into<String>, port: u16) -> Self {
        Self {
            service_type: "http".into(),
            protocol: "tcp".into(),
            subtype: None,
            name: name.into(),
            port,
            txt: vec![("path".into(), "/".into())],
        }
    }

    /// The DNS-SD type domain for this service, in the `_sub` form when a subtype
    /// is set: `[_<subtype>._sub.]_<service_type>._<protocol>.local.`.
    fn type_domain(&self) -> String {
        let base = format!("_{}._{}.local.", self.service_type, self.protocol);
        match &self.subtype {
            Some(sub) => format!("_{sub}._sub.{base}"),
            None => base,
        }
    }
}

/// A live mDNS responder: a single `mdns_sd` daemon (one thread, one 5353 socket)
/// that can register/unregister services and browse for them. Holds the daemon
/// alive for as long as the `Responder` lives.
pub struct Responder {
    daemon: ServiceDaemon,
    host: String,
}

impl Responder {
    /// Start the responder. The SRV target host is a label derived from the
    /// system hostname, distinct from any system responder's `<host>.local`.
    pub fn new() -> Result<Self, MdnsError> {
        let daemon = ServiceDaemon::new()?;
        Ok(Self { daemon, host: format!("{}-aes67.local.", hostname_label()) })
    }

    /// Register `service`. Returns its full DNS-SD name (for [`unregister`]).
    /// Addresses are auto-detected and tracked, so this works before DHCP, too.
    ///
    /// [`unregister`]: Responder::unregister
    pub fn register(&self, service: &Service) -> Result<String, MdnsError> {
        let info = build_info(service, &self.host)?;
        let fullname = info.get_fullname().to_string();
        self.daemon.register(info)?;
        Ok(fullname)
    }

    /// Withdraw a previously [`register`](Responder::register)ed service by its
    /// full name.
    pub fn unregister(&self, fullname: &str) -> Result<(), MdnsError> {
        self.daemon.unregister(fullname)?;
        Ok(())
    }

    /// Browse for `service_type` (e.g. `_ravenna_session._sub._rtsp._tcp.local.`).
    /// Returns a channel of [`BrowseEvent`]s, translated off the `mdns_sd` daemon
    /// channel on a helper thread that ends when the receiver is dropped.
    pub fn browse(&self, service_type: &str) -> Result<Receiver<BrowseEvent>, MdnsError> {
        let raw = self.daemon.browse(service_type)?;
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            for ev in raw.iter() {
                let mapped = match ev {
                    ServiceEvent::ServiceResolved(r) => Some(BrowseEvent::Found(DiscoveredService {
                        instance: instance_label(&r.fullname),
                        fullname: r.fullname.clone(),
                        hosts: r.get_addresses_v4().into_iter().collect(),
                        port: r.get_port(),
                    })),
                    ServiceEvent::ServiceRemoved(_, fullname) => Some(BrowseEvent::Removed { fullname }),
                    _ => None,
                };
                if let Some(m) = mapped {
                    if tx.send(m).is_err() {
                        break; // consumer gone
                    }
                }
            }
        });
        Ok(rx)
    }
}

/// A service instance discovered while browsing.
#[derive(Debug, Clone)]
pub struct DiscoveredService {
    /// The instance (left-most) label — for a RAVENNA session, its name.
    pub instance: String,
    /// The full DNS-SD name.
    pub fullname: String,
    /// All resolved IPv4 addresses (a node may advertise several interfaces).
    pub hosts: Vec<Ipv4Addr>,
    /// The advertised port.
    pub port: u16,
}

/// The DNS-SD instance label: the first (left-most) label of `fullname`, with
/// `\.` escapes decoded. Robust across plain and `_sub`-type browse results,
/// where the trailing type domain differs.
fn instance_label(fullname: &str) -> String {
    let mut out = String::new();
    let mut chars = fullname.chars();
    while let Some(c) = chars.next() {
        match c {
            '\\' => {
                if let Some(escaped) = chars.next() {
                    out.push(escaped);
                }
            }
            '.' => break,
            _ => out.push(c),
        }
    }
    out
}

/// A browse result: an instance appeared (resolved) or went away.
#[derive(Debug, Clone)]
pub enum BrowseEvent {
    Found(DiscoveredService),
    Removed { fullname: String },
}

/// Build a `mdns_sd::ServiceInfo` for `service` with `host` as the SRV target.
fn build_info(service: &Service, host: &str) -> Result<ServiceInfo, mdns_sd::Error> {
    let ty_domain = service.type_domain();
    let props: Vec<(&str, &str)> = service.txt.iter().map(|(k, v)| (k.as_str(), v.as_str())).collect();
    Ok(ServiceInfo::new(&ty_domain, &service.name, host, (), service.port, &props[..])?.enable_addr_auto())
}

/// Announce `service` on a dedicated thread for the life of the process.
///
/// Errors (failing to start the responder, an invalid name, …) are logged and
/// end the thread — announcement is best-effort and never blocks the caller. The
/// returned handle is usually detached; the thread holds the responder alive.
pub fn spawn(service: Service) -> JoinHandle<()> {
    thread::spawn(move || run(service))
}

fn run(service: Service) {
    let responder = match Responder::new() {
        Ok(r) => r,
        Err(e) => {
            eprintln!("aes67 mdns: cannot start responder: {e}");
            return;
        }
    };
    match responder.register(&service) {
        Ok(_) => eprintln!(
            "aes67 mdns: announcing '{}' as {} on port {}",
            service.name,
            service.type_domain(),
            service.port
        ),
        Err(e) => {
            eprintln!("aes67 mdns: cannot register {}: {e}", service.type_domain());
            return;
        }
    }
    // Hold the responder (and thus the registration) for the process lifetime;
    // `mdns_sd` does the actual work on its own background thread.
    let _responder = responder;
    loop {
        thread::park();
    }
}

/// The system hostname reduced to a single, DNS-safe label (first component,
/// `[A-Za-z0-9-]` only). Falls back to `"aes67"`.
fn hostname_label() -> String {
    let mut buf = [0 as libc::c_char; 256];
    // SAFETY: buf is a valid, sufficiently large buffer; gethostname NUL-terminates.
    if unsafe { libc::gethostname(buf.as_mut_ptr(), buf.len()) } != 0 {
        return "aes67".into();
    }
    let raw = unsafe { CStr::from_ptr(buf.as_ptr()) }.to_string_lossy();
    let label: String = raw
        .split('.')
        .next()
        .unwrap_or("")
        .chars()
        .filter(|c| c.is_ascii_alphanumeric() || *c == '-')
        .collect();
    if label.is_empty() {
        "aes67".into()
    } else {
        label
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn http_service_has_conventional_defaults() {
        let s = Service::http("AES67 Configuration", 8080);
        assert_eq!(s.service_type, "http");
        assert_eq!(s.protocol, "tcp");
        assert_eq!(s.port, 8080);
        assert_eq!(s.txt, vec![("path".to_string(), "/".to_string())]);
    }

    #[test]
    fn subtype_builds_the_sub_type_domain() {
        let s = Service {
            service_type: "rtsp".into(),
            protocol: "tcp".into(),
            subtype: Some("ravenna_session".into()),
            name: "Line".into(),
            port: 554,
            txt: vec![],
        };
        assert_eq!(s.type_domain(), "_ravenna_session._sub._rtsp._tcp.local.");
    }

    #[test]
    #[ignore = "uses real mDNS multicast; run with `--ignored`"]
    fn register_then_browse_finds_session() {
        use std::time::{Duration, Instant};

        let responder = Responder::new().unwrap();
        responder
            .register(&Service {
                service_type: "rtsp".into(),
                protocol: "tcp".into(),
                subtype: Some("ravenna_session".into()),
                name: "TestSession".into(),
                port: 5004,
                txt: vec![],
            })
            .unwrap();

        let browser = Responder::new().unwrap();
        let rx = browser.browse("_ravenna_session._sub._rtsp._tcp.local.").unwrap();

        let deadline = Instant::now() + Duration::from_secs(8);
        let mut found = false;
        while Instant::now() < deadline && !found {
            if let Ok(BrowseEvent::Found(s)) = rx.recv_timeout(Duration::from_millis(500)) {
                found = s.instance == "TestSession";
            }
        }
        assert!(found, "session not discovered via mDNS browse");
    }

    #[test]
    fn hostname_label_is_a_clean_dns_label() {
        let h = hostname_label();
        assert!(!h.is_empty());
        assert!(h.chars().all(|c| c.is_ascii_alphanumeric() || c == '-'));
    }
}
