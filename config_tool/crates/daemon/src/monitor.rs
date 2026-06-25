//! Link/address monitor: keep the FPGA and the Linux TAP in sync, both ways.
//!
//! A single thread polls once a second and reconciles two pieces of state that
//! straddle the bus boundary:
//!
//! * **FPGA link → Linux carrier.** Reads the FPGA's `eth_link_up` status bit and
//!   drives the TAP's carrier (`TUNSETCARRIER`). When the FPGA PHY link is down,
//!   Linux marks the interface `NO-CARRIER`, so the stack — and `dhcpcd` — wait
//!   for a real link instead of sending into a dead port.
//! * **Linux address → FPGA IP.** Reads the address Linux assigns to the TAP
//!   (DHCP lease or static) and writes it into the FPGA's IP CSR, so the data
//!   plane stamps the right source IP into outgoing RTP/UDP and answers ARP for
//!   it. Link-local (169.254/16) is kept — zeroconf will need it once the Ravenna
//!   discovery layer lands — only the unspecified/loopback addresses are skipped.
//!
//! Polling (rather than rtnetlink) keeps this dependency-free; a once-per-second
//! check is invisible next to DHCP and link-state timing.
//!
//! Two link backings are supported via [`NetLink`]: the userspace **TAP** bridge
//! (the daemon drives the TAP carrier from the FPGA link and runs DHCP), and the
//! **kernel** `aes67_eth` netdev (the module drives its own carrier and the
//! system network manager runs DHCP — the monitor then only mirrors the IP, joins
//! IGMP groups, and recovers from FPGA resets). The IP/IGMP/reset-recovery logic
//! is identical for both; only carrier driving and DHCP triggering differ.

use std::ffi::CStr;
use std::fs::File;
use std::net::Ipv4Addr;
use std::process::Command;
use std::time::Duration;

use aes67_config::ControlApi;

use crate::igmp::IgmpManager;
use crate::persist::{NetworkCfg, Settings};
use crate::{netif, startup, SharedConfig, SharedDevice};

/// PTP (IEEE 1588) primary multicast group — Sync/Follow_Up/Announce/Delay.
const PTP_PRIMARY: Ipv4Addr = Ipv4Addr::new(224, 0, 1, 129);

/// How often to reconcile link state and address.
const POLL_INTERVAL: Duration = Duration::from_millis(1000);

/// AES67 status register and the PHY-link/speed fields within it.
const REG_STATUS: &str = "aes67_csr_status";
const STATUS_ETH_LINK_UP: u64 = 1 << 4;
/// `eth_speed` is a 2-bit field at bit 5: 00=10M, 01=100M, 10=1G.
const STATUS_ETH_SPEED_SHIFT: u32 = 5;
const STATUS_ETH_SPEED_MASK: u64 = 0b11;

/// How the FPGA's Ethernet is presented to Linux, and thus what the monitor must
/// drive itself.
pub enum NetLink {
    /// Userspace TAP bridge: the daemon drives the TAP carrier from the FPGA PHY
    /// link and triggers DHCP on link-up. Carries the tap fd handle.
    Tap(File),
    /// Kernel `aes67_eth` netdev: the module drives its own carrier and the
    /// system runs DHCP. The monitor only mirrors the IP, joins IGMP, and
    /// recovers from FPGA resets.
    Kernel,
}

/// Spawn the monitor thread. It runs for the life of the daemon. `iface` is the
/// interface whose address is mirrored into the FPGA and on which IGMP groups are
/// joined (the TAP in TAP mode, the kernel netdev in kernel mode). `config` is the
/// live shared config, read at need so an FPGA-reset recovery rebuilds the
/// *current* state (including config made after startup) and DHCP uses the current
/// command.
pub fn spawn(
    device: SharedDevice,
    iface: String,
    link: NetLink,
    config: SharedConfig,
    verbose: u8,
) {
    std::thread::spawn(move || run(device, iface, link, config, verbose));
}

fn run(
    device: SharedDevice,
    tap_name: String,
    link: NetLink,
    config: SharedConfig,
    verbose: u8,
) {
    // TAP mode drives the carrier from the FPGA link and runs DHCP itself; the
    // kernel netdev does both on its own, so the monitor only mirrors IP/IGMP.
    let (carrier, drive_dhcp) = match link {
        NetLink::Tap(f) => (Some(f), true),
        NetLink::Kernel => (None, false),
    };
    // Seed the IP from the FPGA's current value so a daemon restart doesn't
    // re-write an address that already matches.
    let mut last_ip = device.lock().unwrap().get_ip().ok().filter(|ip| usable(*ip));
    let mut last_link: Option<bool> = None;
    // DHCP must run on the first link-up. Even on a warm restart (FPGA still
    // running, only the daemon restarted) the TAP is created fresh — it is
    // owned by the daemon process and not IFF_PERSIST, so its DHCP lease and
    // address vanished with the old process. Only a *reset recovery* (the TAP
    // kept running while the FPGA was reset under us) leaves the lease standing,
    // and that path sets this in the loop below. A down edge clears it.
    let mut suppress_dhcp_on_up = false;

    // IGMP membership for the FPGA's multicast groups (PTP + RX/TX streams) on
    // the TAP. Joined once a valid IP exists; re-reported after every link-up.
    let mut igmp = match IgmpManager::new(&tap_name) {
        Ok(m) => Some(m),
        Err(e) => {
            eprintln!("aes67d: igmp: cannot manage groups on {tap_name}: {e}");
            None
        }
    };
    let mut igmp_rejoin_pending = true; // force a fresh join at startup

    if verbose >= 1 {
        eprintln!("aes67d: monitor: watching FPGA link and {tap_name} address");
    }

    loop {
        std::thread::sleep(POLL_INTERVAL);

        // If the FPGA reverted to its all-held power-on reset state, it was reset
        // from under us — restore the whole last-known configuration from the
        // *live* config (registers/MAC/PTP/streams, including changes made after
        // startup) plus the last IP (which reconcile_ip alone would not re-apply,
        // the TAP address being unchanged).
        if startup::all_resets_asserted(&device) {
            let (network, settings) = {
                let c = config.lock().unwrap();
                (c.network.clone(), c.settings.clone())
            };
            if let Err(e) = startup::reconfigure(&device, &network, &settings, last_ip, verbose) {
                eprintln!("aes67d: monitor: FPGA reconfigure failed: {e:#}");
            }
            // The reset bounced the PHY link, but the Linux lease still stands and
            // the FPGA IP was just restored — so suppress the DHCP re-trigger for
            // the recovery's link-up (a later real down→up still triggers it).
            last_link = None;
            suppress_dhcp_on_up = true;
            igmp_rejoin_pending = true; // the bounce dropped switch forwarding
        }

        // On the link-up edge (cold start, daemon warm restart, or recovery from
        // a link-down), kick off DHCP: the kernel has no DHCP client, so a
        // userspace one must run. A reset-recovery suppresses only the first up.
        match reconcile_link(&device, carrier.as_ref(), &mut last_link) {
            Some(true) => {
                // Link came up — the switch flushed memberships; re-report once
                // we have a valid IP.
                igmp_rejoin_pending = true;
                if !drive_dhcp {
                    // Kernel netdev mode: the system network manager owns DHCP.
                } else if suppress_dhcp_on_up {
                    suppress_dhcp_on_up = false;
                    if verbose >= 1 {
                        eprintln!(
                            "aes67d: monitor: reset recovery — link up, leaving the existing lease alone"
                        );
                    }
                } else {
                    // Use the live DHCP command from the shared config.
                    let net = config.lock().unwrap().network.clone();
                    trigger_dhcp(&net, &tap_name);
                }
            }
            // A real link-down means the next up is a recovery → allow DHCP again.
            Some(false) => suppress_dhcp_on_up = false,
            None => {}
        }
        reconcile_ip(&device, &tap_name, &mut last_ip);
        reconcile_igmp(&mut igmp, &config, &tap_name, &mut igmp_rejoin_pending, verbose);
    }
}

/// Join the FPGA's multicast groups (PTP + RX/TX streams) on the TAP, but only
/// once a valid IP exists (the IGMP report needs a proper source). With a
/// pending re-report (startup, link-up, reset recovery) all groups are re-joined
/// to emit fresh reports; otherwise membership is reconciled incrementally so a
/// newly configured RX/TX stream's group is picked up.
fn reconcile_igmp(
    igmp: &mut Option<IgmpManager>,
    config: &SharedConfig,
    tap_name: &str,
    pending: &mut bool,
    verbose: u8,
) {
    let Some(mgr) = igmp.as_mut() else { return };
    // Gate on a usable IP — without one the kernel can't source IGMP reports.
    if interface_ipv4(tap_name).filter(|ip| usable(*ip)).is_none() {
        return;
    }
    let settings = config.lock().unwrap().settings.clone();
    let desired = desired_groups(&settings);
    mgr.apply(&desired, *pending, verbose);
    *pending = false;
}

/// The multicast groups the FPGA needs forwarded, in join order: PTP and the SAP
/// discovery group first, then every RX stream, then every TX stream. (The SAP
/// listener socket also joins the group itself; including it here re-reports the
/// membership after a link-up so switch forwarding survives a bounce.)
fn desired_groups(settings: &Settings) -> Vec<Ipv4Addr> {
    let mut groups = vec![PTP_PRIMARY, aes67_sap::SAP_GROUP];
    for rx in settings.rx_streams.values() {
        if let Ok(ip) = rx.dst_ip.parse::<Ipv4Addr>() {
            groups.push(ip);
        }
    }
    for tx in settings.tx_streams.values() {
        if let Ok(ip) = tx.dst_ip.parse::<Ipv4Addr>() {
            groups.push(ip);
        }
    }
    groups
}

/// FPGA PHY link → carrier. Returns `Some(true)`/`Some(false)` on a link
/// up/down transition, or `None` if the state was unchanged. With `carrier_tap`
/// (TAP mode) the transition also drives the TAP carrier; without it (kernel
/// netdev mode, which manages its own carrier) the edge is only tracked, so the
/// IGMP re-report still fires on link-up.
fn reconcile_link(
    device: &SharedDevice,
    carrier_tap: Option<&File>,
    last: &mut Option<bool>,
) -> Option<bool> {
    let status = match device.lock().unwrap().read_register(REG_STATUS) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("aes67d: monitor: read status failed: {e}");
            return None;
        }
    };
    let up = status & STATUS_ETH_LINK_UP != 0;
    if *last == Some(up) {
        return None;
    }
    let speed = link_speed(status);
    let Some(carrier_tap) = carrier_tap else {
        // Kernel netdev mode: the driver owns the carrier; just note the edge.
        if up {
            eprintln!("aes67d: monitor: FPGA link up ({speed})");
        } else {
            eprintln!("aes67d: monitor: FPGA link down");
        }
        *last = Some(up);
        return Some(up);
    };
    match netif::set_carrier(carrier_tap, up) {
        Ok(()) => {
            if up {
                eprintln!("aes67d: monitor: FPGA link up ({speed}) → tap carrier on");
            } else {
                eprintln!("aes67d: monitor: FPGA link down → tap carrier off");
            }
            *last = Some(up);
            Some(up)
        }
        Err(e) => {
            eprintln!("aes67d: monitor: set carrier {} failed: {e}", carrier(up));
            None
        }
    }
}

/// Run the configured DHCP-trigger command for `iface` (detached). Covers both
/// the cold-start link-up and recovery after a link-down. Idempotent clients
/// (e.g. `dhcpcd`) simply rebind when re-invoked.
fn trigger_dhcp(network: &NetworkCfg, iface: &str) {
    let Some(template) = network.dhcp_template() else {
        return; // disabled — rely on the system network manager
    };
    let args: Vec<String> = template.iter().map(|a| a.replace("{iface}", iface)).collect();
    let Some((prog, rest)) = args.split_first() else {
        return;
    };
    eprintln!("aes67d: monitor: link up — invoking DHCP: {}", args.join(" "));
    let (prog, rest) = (prog.clone(), rest.to_vec());
    std::thread::spawn(move || match Command::new(&prog).args(&rest).status() {
        Ok(s) if s.success() => {}
        Ok(s) => eprintln!("aes67d: monitor: DHCP command exited with {s}"),
        Err(e) => eprintln!("aes67d: monitor: failed to run DHCP command '{prog}': {e}"),
    });
}

/// Decode the `eth_speed` field into a human label.
fn link_speed(status: u64) -> &'static str {
    match (status >> STATUS_ETH_SPEED_SHIFT) & STATUS_ETH_SPEED_MASK {
        0b00 => "10 Mbps",
        0b01 => "100 Mbps",
        0b10 => "1 Gbps",
        _ => "unknown speed",
    }
}

/// TAP address → FPGA IP CSR.
fn reconcile_ip(device: &SharedDevice, tap_name: &str, last: &mut Option<Ipv4Addr>) {
    let current = match interface_ipv4(tap_name) {
        Some(ip) if usable(ip) => ip,
        _ => return, // no usable address yet; leave the FPGA as-is
    };
    if *last == Some(current) {
        return;
    }
    match device.lock().unwrap().set_ip(current) {
        Ok(()) => {
            eprintln!("aes67d: monitor: programmed FPGA IP {current} (from {tap_name})");
            *last = Some(current);
        }
        Err(e) => eprintln!("aes67d: monitor: set_ip {current} failed: {e}"),
    }
}

fn carrier(on: bool) -> &'static str {
    if on {
        "on"
    } else {
        "off"
    }
}

/// An address worth programming into the FPGA: anything real the interface
/// actually carries. Only the unspecified (0.0.0.0) and loopback addresses are
/// rejected; link-local (169.254/16) is kept for zeroconf.
fn usable(ip: Ipv4Addr) -> bool {
    !ip.is_unspecified() && !ip.is_loopback()
}

/// Read the first IPv4 address bound to `name`, or `None` if it has none.
pub(crate) fn interface_ipv4(name: &str) -> Option<Ipv4Addr> {
    unsafe {
        let mut ifap: *mut libc::ifaddrs = std::ptr::null_mut();
        if libc::getifaddrs(&mut ifap) != 0 {
            return None;
        }
        let mut result = None;
        let mut cur = ifap;
        while !cur.is_null() {
            let ifa = &*cur;
            if !ifa.ifa_addr.is_null()
                && (*ifa.ifa_addr).sa_family as i32 == libc::AF_INET
                && CStr::from_ptr(ifa.ifa_name).to_bytes() == name.as_bytes()
            {
                let sin = ifa.ifa_addr as *const libc::sockaddr_in;
                // s_addr is stored in network byte order; to_ne_bytes preserves
                // that in-memory layout, giving the octets directly.
                let octets = (*sin).sin_addr.s_addr.to_ne_bytes();
                result = Some(Ipv4Addr::from(octets));
                break;
            }
            cur = ifa.ifa_next;
        }
        libc::freeifaddrs(ifap);
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use aes67_proto::{RxStreamParams, TxStreamParams};

    #[test]
    fn desired_groups_orders_ptp_then_rx_then_tx() {
        let mut s = Settings::default();
        s.rx_streams.insert(
            0,
            RxStreamParams {
                id: 0,
                dst_ip: "239.69.2.1".into(),
                dst_port: 5004,
                ch_map: vec![],
                channels: None,
                output_delay: 0,
                samples_per_channel: 0,
                name: None,
            },
        );
        s.tx_streams.insert(
            0,
            TxStreamParams {
                id: 0,
                dst_ip: "239.69.1.1".into(),
                channels: None,
                samples_per_packet: 0,
                ch_ids: vec![],
                ssrc: 0,
                name: None,
            },
        );

        assert_eq!(
            desired_groups(&s),
            vec![
                PTP_PRIMARY,
                aes67_sap::SAP_GROUP,
                Ipv4Addr::new(239, 69, 2, 1),
                Ipv4Addr::new(239, 69, 1, 1)
            ]
        );
    }
}
