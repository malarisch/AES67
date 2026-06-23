//! SAP/SDP discovery service.
//!
//! Two cooperating tasks run on the TAP once it has an address:
//!
//! * **Listener** — joins the [SAP group](aes67_sap::SAP_GROUP) on the TAP,
//!   decodes incoming [`SapPacket`]s, parses their SDP into an
//!   [`AudioStream`](aes67_sdp::AudioStream), and keeps a [`Registry`] of the
//!   remote streams currently on the air (announcements refresh, deletions and
//!   timeouts remove). This is what a client lists as RX subscription candidates.
//! * **Announcer** — periodically advertises the daemon's own TX streams (built
//!   from the live config plus the FPGA's IP) as SAP/SDP, and sends a SAP
//!   deletion when a previously announced TX stream is torn down.
//!
//! The SDP ↔ stream mapping lives in [`aes67_sdp`] and the wire protocol in
//! [`aes67_sap`]; this module is just the glue plus the registry/policy. A future
//! mDNS discovery path can reuse the same [`Registry`] and `aes67-sdp` mapping.

use std::collections::HashMap;
use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use aes67_config::ControlApi;
use aes67_proto::TxStreamParams;
use aes67_sap::{SapKind, SapListener, SapSender, SAP_GROUP, SAP_PORT};
use aes67_sdp::AudioStream;

use crate::monitor::interface_ipv4;
use crate::{SharedConfig, SharedDevice};

/// How often the daemon re-announces its own TX streams.
const ANNOUNCE_INTERVAL: Duration = Duration::from_secs(30);
/// Drop a discovered stream that has not been re-announced within this window
/// (3× the announce interval, matching common SAP receiver timeouts).
const STALE_AFTER: Duration = Duration::from_secs(90);
/// Listener wake-up cadence, so stale entries are expired even when the network
/// is silent.
const LISTEN_TICK: Duration = Duration::from_secs(5);
/// Multicast TTL/scope for announcements (matches the `c=` line default).
const SAP_TTL: u32 = 32;
/// CSRs holding the elected PTP grandmaster's 64-bit clock identity (the same
/// "Leader ID" the web dashboard shows).
const REG_LEADER_ID_LO: &str = "aes67_csr_ptp_leader_id_lo";
const REG_LEADER_ID_HI: &str = "aes67_csr_ptp_leader_id_hi";
/// AES67 fixed media UDP port (the gateware transmits to 5004).
const RTP_PORT: u16 = 5004;
/// Poll cadence while waiting for the TAP to acquire an address.
const IP_WAIT_TICK: Duration = Duration::from_millis(500);

/// The transport a stream was discovered over.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Transport {
    /// SAP/SDP announcement.
    Sap,
    /// mDNS browse + RTSP DESCRIBE.
    Mdns,
}

/// A stream learned from the network, with bookkeeping. The same RTP flow seen
/// over several transports is collapsed into one entry (see [`Registry::snapshot`]).
#[derive(Clone)]
pub struct DiscoveredEntry {
    pub stream: AudioStream,
    pub source: Ipv4Addr,
    pub age: Duration,
    /// Discovered via SAP.
    pub via_sap: bool,
    /// Discovered via mDNS/RTSP.
    pub via_mdns: bool,
}

/// The streams currently discovered on the network, stored one entry per
/// *sighting* keyed by a transport-namespaced string (`sap:<source>:<hash>` or
/// `mdns:<fullname>`). [`snapshot`](Registry::snapshot) collapses sightings of the
/// same RTP flow (same multicast group + port) into one merged result.
#[derive(Default)]
pub struct Registry {
    streams: HashMap<String, Stored>,
}

struct Stored {
    stream: AudioStream,
    source: Ipv4Addr,
    last_seen: Instant,
    transport: Transport,
}

impl Registry {
    pub(crate) fn upsert(&mut self, key: String, stream: AudioStream, source: Ipv4Addr, transport: Transport) {
        self.streams.insert(key, Stored { stream, source, last_seen: Instant::now(), transport });
    }

    pub(crate) fn remove(&mut self, key: &str) {
        self.streams.remove(key);
    }

    /// Drop SAP sightings not refreshed within `after` (they are re-announced
    /// periodically); mDNS sightings are managed by explicit add/remove events.
    fn expire(&mut self, after: Duration) {
        let now = Instant::now();
        self.streams
            .retain(|_, s| s.transport != Transport::Sap || now.duration_since(s.last_seen) < after);
    }

    /// An owned, age-stamped snapshot for the control API, with sightings of the
    /// same RTP flow (multicast group + port) merged into one entry that records
    /// every transport it was seen over. The freshest sighting supplies the
    /// representative description, source and age.
    pub fn snapshot(&self) -> Vec<DiscoveredEntry> {
        let now = Instant::now();
        let mut merged: HashMap<(Ipv4Addr, u16), DiscoveredEntry> = HashMap::new();
        for s in self.streams.values() {
            let age = now.duration_since(s.last_seen);
            let id = (s.stream.dst_addr, s.stream.dst_port);
            let entry = merged.entry(id).or_insert_with(|| DiscoveredEntry {
                stream: s.stream.clone(),
                source: s.source,
                age,
                via_sap: false,
                via_mdns: false,
            });
            match s.transport {
                Transport::Sap => entry.via_sap = true,
                Transport::Mdns => entry.via_mdns = true,
            }
            // Keep the freshest sighting as the representative.
            if age < entry.age {
                entry.stream = s.stream.clone();
                entry.source = s.source;
                entry.age = age;
            }
        }
        let mut out: Vec<DiscoveredEntry> = merged.into_values().collect();
        out.sort_by(|a, b| {
            a.stream.session_name.cmp(&b.stream.session_name).then(a.source.cmp(&b.source))
        });
        out
    }
}

/// A registry shared between the discovery threads and the control server.
pub type SharedDiscovery = Arc<Mutex<Registry>>;

/// Spawn the discovery service. It waits (off the caller's thread) for the TAP to
/// get an address, then runs the listener and announcer for the daemon's life.
pub fn spawn(
    device: SharedDevice,
    config: SharedConfig,
    tap_name: String,
    registry: SharedDiscovery,
    verbose: u8,
) {
    std::thread::spawn(move || run(device, config, tap_name, registry, verbose));
}

fn run(
    device: SharedDevice,
    config: SharedConfig,
    tap_name: String,
    registry: SharedDiscovery,
    verbose: u8,
) {
    let local_ip = wait_for_ip(&tap_name);
    if verbose >= 1 {
        eprintln!("aes67d: discovery: {tap_name} has {local_ip}, starting SAP on {SAP_GROUP}:{SAP_PORT}");
    }

    let listen_sock = match listen_socket(local_ip) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("aes67d: discovery: cannot open SAP listen socket: {e} — discovery disabled");
            return;
        }
    };
    let send_sock = match send_socket(local_ip) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("aes67d: discovery: cannot open SAP send socket: {e} — announcements disabled");
            // Listening can still work without the sender.
            return run_listener(listen_sock, registry, local_ip, verbose);
        }
    };

    // Listener on its own thread; announcer on this one.
    let reg = Arc::clone(&registry);
    std::thread::spawn(move || run_listener(listen_sock, reg, local_ip, verbose));
    run_announcer(SapSender::to_default_group(send_sock, local_ip), device, config, verbose);
}

/// Block until the TAP has a usable IPv4 address.
pub(crate) fn wait_for_ip(tap_name: &str) -> Ipv4Addr {
    loop {
        if let Some(ip) = interface_ipv4(tap_name).filter(|ip| !ip.is_unspecified() && !ip.is_loopback())
        {
            return ip;
        }
        std::thread::sleep(IP_WAIT_TICK);
    }
}

/// A socket bound to the SAP port, joined to the SAP group on the TAP, with a
/// read timeout so the receive loop can periodically expire stale entries.
fn listen_socket(local_ip: Ipv4Addr) -> std::io::Result<UdpSocket> {
    let sock = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, SAP_PORT))?;
    sock.join_multicast_v4(&SAP_GROUP, &local_ip)?;
    sock.set_read_timeout(Some(LISTEN_TICK))?;
    Ok(sock)
}

/// A socket bound to the TAP address for sending announcements out that
/// interface. Loopback is disabled so we don't discover our own streams.
fn send_socket(local_ip: Ipv4Addr) -> std::io::Result<UdpSocket> {
    let sock = UdpSocket::bind(SocketAddrV4::new(local_ip, 0))?;
    sock.set_multicast_ttl_v4(SAP_TTL)?;
    let _ = sock.set_multicast_loop_v4(false);
    Ok(sock)
}

/// Receive, decode and register SAP announcements; expire stale entries on each
/// tick. `local_ip` is filtered out so we ignore our own announcements.
fn run_listener(socket: UdpSocket, registry: SharedDiscovery, local_ip: Ipv4Addr, verbose: u8) {
    let mut listener = SapListener::new(socket);
    loop {
        match listener.recv() {
            Ok(Some(pkt)) if pkt.source == local_ip => {} // our own announcement
            Ok(Some(pkt)) => {
                let key = format!("sap:{}:{}", pkt.source, pkt.msg_id_hash);
                match pkt.kind {
                    SapKind::Announce => match AudioStream::from_sdp(&pkt.sdp) {
                        Ok(stream) => {
                            if verbose >= 1 {
                                eprintln!(
                                    "aes67d: discovery: + '{}' {} from {} (sap)",
                                    stream.session_name, stream.dst_addr, pkt.source
                                );
                            }
                            registry.lock().unwrap().upsert(key, stream, pkt.source, Transport::Sap);
                        }
                        Err(e) if verbose >= 2 => {
                            eprintln!("aes67d: discovery: ignoring SDP from {}: {e}", pkt.source);
                        }
                        Err(_) => {}
                    },
                    SapKind::Delete => {
                        if verbose >= 1 {
                            eprintln!("aes67d: discovery: - session from {} (sap)", pkt.source);
                        }
                        registry.lock().unwrap().remove(&key);
                    }
                }
            }
            Ok(None) => {} // not a SAP/SDP datagram
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {} // read timeout
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => eprintln!("aes67d: discovery: SAP recv error: {e}"),
        }
        registry.lock().unwrap().expire(STALE_AFTER);
    }
}

/// Periodically announce the daemon's TX streams, and send a SAP deletion when a
/// previously announced stream is gone (torn down or reconfigured away).
fn run_announcer(sender: SapSender, device: SharedDevice, config: SharedConfig, verbose: u8) {
    // The SDP we last announced per TX id, so we can diff and emit deletions.
    let mut announced: HashMap<u8, String> = HashMap::new();

    loop {
        // Read the node's IP (the SDP origin) and the elected PTP grandmaster id
        // (advertised as ts-refclk) under one bus lock.
        let (origin, gmid) = {
            let mut d = device.lock().unwrap();
            match d.get_ip() {
                Ok(ip) if !ip.is_unspecified() && !ip.is_loopback() => (ip, read_leader_gmid(&mut *d)),
                _ => {
                    // No identity yet — try again next cycle.
                    drop(d);
                    std::thread::sleep(ANNOUNCE_INTERVAL);
                    continue;
                }
            }
        };

        let tx_streams = config.lock().unwrap().settings.tx_streams.clone();
        let mut current: HashMap<u8, String> = HashMap::new();
        for p in tx_streams.values() {
            if let Some(stream) = tx_to_stream(p, origin, gmid.as_deref()) {
                let sdp = stream.to_sdp();
                if let Err(e) = sender.announce(&sdp) {
                    eprintln!("aes67d: discovery: announce of TX {} failed: {e}", p.id);
                } else if verbose >= 2 {
                    eprintln!("aes67d: discovery: announced TX {} ({})", p.id, stream.dst_addr);
                }
                current.insert(p.id, sdp);
            }
        }

        // Withdraw streams that were announced before but are no longer present.
        for (id, sdp) in &announced {
            if !current.contains_key(id) {
                if let Err(e) = sender.delete(sdp) {
                    eprintln!("aes67d: discovery: delete of TX {id} failed: {e}");
                } else if verbose >= 1 {
                    eprintln!("aes67d: discovery: withdrew TX {id}");
                }
            }
        }

        announced = current;
        std::thread::sleep(ANNOUNCE_INTERVAL);
    }
}

/// The session name for a TX stream: its configured name, or a default derived
/// from the slot. Used as the SDP `s=` line, the SAP/mDNS instance name, and the
/// RTSP `by-name` key, so all three agree.
pub(crate) fn session_name(p: &TxStreamParams) -> String {
    match &p.name {
        Some(n) if !n.is_empty() => n.clone(),
        _ => format!("AES67 TX {}", p.id),
    }
}

/// The PTPv2 domain advertised in the RAVENNA `a=clock-domain` SDP attribute.
/// RAVENNA requires the domain; we use the default media domain 0 (no dedicated
/// CSR exposes it yet).
pub(crate) const PTP_DOMAIN: u8 = 0;

/// Build an AES67/RAVENNA [`AudioStream`] for a configured TX stream. `None` if
/// the destination IP does not parse. `gmid` is the elected PTP grandmaster
/// identity (EUI-64), advertised as `ts-refclk` when known. Shared by the SAP
/// announcer and the RTSP server.
pub(crate) fn tx_to_stream(p: &TxStreamParams, origin: Ipv4Addr, gmid: Option<&str>) -> Option<AudioStream> {
    let dst: Ipv4Addr = p.dst_ip.parse().ok()?;
    let channels = p
        .channels
        .filter(|&c| c != 0)
        .unwrap_or(p.ch_ids.len() as u8)
        .max(1);

    let mut s = AudioStream::new(session_name(p), origin, dst);
    s.dst_port = RTP_PORT;
    s.channels = channels;
    // 48 samples = 1 ms at 48 kHz; fall back to AES67's 1 ms default.
    s.ptime_ms = if p.samples_per_packet > 0 { p.samples_per_packet as f32 / 48.0 } else { 1.0 };
    s.ptp_gmid = gmid.map(str::to_string);
    s.ptp_domain = Some(PTP_DOMAIN);
    // A per-stream stable id (origin + slot) and a version that changes whenever
    // the announced parameters change, so receivers can spot modifications.
    s.session_id = (u32::from(origin) as u64) << 8 | p.id as u64;
    s.session_version = content_version(p);
    Some(s)
}

/// Read the elected PTP grandmaster's clock identity from the FPGA and format it
/// as an EUI-64 (`00-1D-C1-FF-FE-01-02-03`) for SDP `ts-refclk`. `None` when the
/// registers are unreadable or the id is still zero (no grandmaster yet). Shared
/// by the SAP announcer and the RTSP server.
pub(crate) fn read_leader_gmid(dev: &mut impl ControlApi) -> Option<String> {
    let lo = dev.read_register(REG_LEADER_ID_LO).ok()?;
    let hi = dev.read_register(REG_LEADER_ID_HI).ok()?;
    let id = (hi << 32) | (lo & 0xffff_ffff);
    if id == 0 {
        return None;
    }
    let bytes = id.to_be_bytes();
    Some(
        bytes
            .iter()
            .map(|b| format!("{b:02X}"))
            .collect::<Vec<_>>()
            .join("-"),
    )
}

/// A stable hash of the announce-affecting TX parameters (FNV-1a). Stays constant
/// while the stream is unchanged and changes on any edit → SDP sess-version bump.
fn content_version(p: &TxStreamParams) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut mix = |b: &[u8]| {
        for &x in b {
            h ^= x as u64;
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    };
    mix(p.dst_ip.as_bytes());
    mix(&[p.channels.unwrap_or(0), p.samples_per_packet]);
    mix(&p.ssrc.to_le_bytes());
    mix(&p.ch_ids);
    mix(p.name.as_deref().unwrap_or("").as_bytes());
    h
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tx(id: u8, dst: &str) -> TxStreamParams {
        TxStreamParams {
            id,
            dst_ip: dst.into(),
            channels: Some(2),
            samples_per_packet: 48,
            ch_ids: vec![0, 1],
            ssrc: 0x1234,
            name: None,
        }
    }

    #[test]
    fn tx_stream_maps_to_aes67_sdp() {
        let gmid = "00-1D-C1-FF-FE-01-02-03";
        let s = tx_to_stream(&tx(3, "239.69.1.3"), Ipv4Addr::new(192, 168, 1, 5), Some(gmid)).unwrap();
        assert_eq!(s.dst_addr, Ipv4Addr::new(239, 69, 1, 3));
        assert_eq!(s.origin_addr, Ipv4Addr::new(192, 168, 1, 5));
        assert_eq!(s.dst_port, 5004);
        assert_eq!(s.channels, 2);
        assert_eq!(s.ptime_ms, 1.0);
        assert_eq!(s.ptp_gmid.as_deref(), Some(gmid));
        assert_eq!(s.session_name, "AES67 TX 3"); // default when unnamed
        // Round-trips through SDP (incl. the ts-refclk grandmaster id).
        assert_eq!(AudioStream::from_sdp(&s.to_sdp()).unwrap(), s);
    }

    #[test]
    fn configured_name_is_announced_and_bumps_version() {
        let origin = Ipv4Addr::new(192, 168, 1, 5);
        let mut p = tx(0, "239.69.1.0");
        let unnamed = content_version(&p);

        p.name = Some("Stage left".into());
        let s = tx_to_stream(&p, origin, None).unwrap();
        assert_eq!(s.session_name, "Stage left");
        // A name change is a modification → the SDP session-version must change.
        assert_ne!(content_version(&p), unnamed);

        // Empty name falls back to the default.
        p.name = Some(String::new());
        assert_eq!(tx_to_stream(&p, origin, None).unwrap().session_name, "AES67 TX 0");
    }

    #[test]
    fn leader_id_reads_as_eui64_and_skips_zero() {
        use aes67_config::{CsrMap, Device};
        use aes67_transport::MockTransport;

        let map = CsrMap::from_csv(
            "csr_register,aes67_csr_ptp_leader_id_lo,0x00,1,rw\n\
             csr_register,aes67_csr_ptp_leader_id_hi,0x04,1,rw\n",
        )
        .unwrap();
        let mut dev = Device::new(MockTransport::new(), map);

        // Zero id ⇒ no grandmaster yet ⇒ None.
        assert_eq!(read_leader_gmid(&mut dev), None);

        // 0x001DC1FFFE010203 split across hi/lo.
        dev.write_register("aes67_csr_ptp_leader_id_hi", 0x001D_C1FF).unwrap();
        dev.write_register("aes67_csr_ptp_leader_id_lo", 0xFE01_0203).unwrap();
        assert_eq!(read_leader_gmid(&mut dev).as_deref(), Some("00-1D-C1-FF-FE-01-02-03"));
    }

    #[test]
    fn version_changes_only_on_edit() {
        let v0 = content_version(&tx(0, "239.69.1.0"));
        assert_eq!(v0, content_version(&tx(0, "239.69.1.0")));
        assert_ne!(v0, content_version(&tx(0, "239.69.1.9")));
    }

    #[test]
    fn registry_upsert_remove_and_expire() {
        let mut reg = Registry::default();
        let src = Ipv4Addr::new(10, 0, 0, 1);
        let s = tx_to_stream(&tx(0, "239.69.1.0"), src, None).unwrap();
        reg.upsert("sap:10.0.0.1:42".into(), s, src, Transport::Sap);
        assert_eq!(reg.snapshot().len(), 1);

        reg.expire(Duration::from_secs(0)); // SAP + "older than 0" → dropped
        assert!(reg.snapshot().is_empty());

        // mDNS sightings survive expiry and go only on remove.
        let s = tx_to_stream(&tx(1, "239.69.1.1"), src, None).unwrap();
        reg.upsert("mdns:Line._rtsp._tcp.local.".into(), s, src, Transport::Mdns);
        reg.expire(Duration::from_secs(0));
        assert_eq!(reg.snapshot().len(), 1);
        reg.remove("mdns:Line._rtsp._tcp.local.");
        assert!(reg.snapshot().is_empty());
    }

    #[test]
    fn same_flow_over_sap_and_mdns_is_merged() {
        let mut reg = Registry::default();
        let a = Ipv4Addr::new(10, 0, 0, 1);
        // Same destination group:port discovered over both transports.
        let s_sap = tx_to_stream(&tx(0, "239.69.1.0"), a, None).unwrap();
        let s_mdns = tx_to_stream(&tx(0, "239.69.1.0"), a, None).unwrap();
        reg.upsert("sap:10.0.0.1:42".into(), s_sap, a, Transport::Sap);
        reg.upsert("mdns:Line._rtsp._tcp.local.".into(), s_mdns, a, Transport::Mdns);

        let snap = reg.snapshot();
        assert_eq!(snap.len(), 1); // collapsed into one entry
        assert!(snap[0].via_sap && snap[0].via_mdns);

        // A different group:port stays a separate entry.
        let other = tx_to_stream(&tx(1, "239.69.1.9"), a, None).unwrap();
        reg.upsert("sap:10.0.0.1:99".into(), other, a, Transport::Sap);
        assert_eq!(reg.snapshot().len(), 2);
    }
}
