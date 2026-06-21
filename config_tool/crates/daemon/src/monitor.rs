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

use std::ffi::CStr;
use std::fs::File;
use std::net::Ipv4Addr;
use std::process::Command;
use std::time::Duration;

use aes67_config::ControlApi;

use crate::persist::NetworkCfg;
use crate::{netif, startup, SharedConfig, SharedDevice};

/// How often to reconcile link state and address.
const POLL_INTERVAL: Duration = Duration::from_millis(1000);

/// AES67 status register and the PHY-link/speed fields within it.
const REG_STATUS: &str = "aes67_csr_status";
const STATUS_ETH_LINK_UP: u64 = 1 << 4;
/// `eth_speed` is a 2-bit field at bit 5: 00=10M, 01=100M, 10=1G.
const STATUS_ETH_SPEED_SHIFT: u32 = 5;
const STATUS_ETH_SPEED_MASK: u64 = 0b11;

/// Spawn the monitor thread. It runs for the life of the daemon. `carrier_tap`
/// is a tap fd handle used only to drive the carrier state. `config` is the live
/// shared config, read at need so an FPGA-reset recovery rebuilds the *current*
/// state (including config made after startup) and DHCP uses the current command.
pub fn spawn(
    device: SharedDevice,
    tap_name: String,
    carrier_tap: File,
    config: SharedConfig,
    warm: bool,
    verbose: u8,
) {
    std::thread::spawn(move || run(device, tap_name, carrier_tap, config, warm, verbose));
}

fn run(
    device: SharedDevice,
    tap_name: String,
    carrier_tap: File,
    config: SharedConfig,
    warm: bool,
    verbose: u8,
) {
    // Seed the IP from the FPGA's current value so a daemon restart doesn't
    // re-write an address that already matches.
    let mut last_ip = device.lock().unwrap().get_ip().ok().filter(|ip| usable(*ip));
    let mut last_link: Option<bool> = None;
    // On a warm restart the system was already operational, so the initial
    // link-up must not re-trigger DHCP — only a genuine link recovery (a
    // down→up after a real link-down) should. A down edge clears this.
    let mut suppress_dhcp_on_up = warm;
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
        }

        // On the link-up edge (cold start or recovery from a link-down), kick
        // off DHCP: the kernel has no DHCP client, so a userspace one must run.
        // A warm restart / reset-recovery suppresses only the first up.
        match reconcile_link(&device, &carrier_tap, &mut last_link) {
            Some(true) => {
                if suppress_dhcp_on_up {
                    suppress_dhcp_on_up = false;
                    if verbose >= 1 {
                        eprintln!(
                            "aes67d: monitor: warm restart — link up, leaving the existing lease alone"
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
    }
}

/// FPGA PHY link → TAP carrier. Returns `Some(true)`/`Some(false)` on a link
/// up/down transition, or `None` if the state was unchanged.
fn reconcile_link(device: &SharedDevice, carrier_tap: &File, last: &mut Option<bool>) -> Option<bool> {
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
fn interface_ipv4(name: &str) -> Option<Ipv4Addr> {
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
