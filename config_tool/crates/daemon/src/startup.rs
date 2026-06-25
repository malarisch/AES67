//! FPGA startup sequence.
//!
//! The gateware powers up with **every module held in reset** (the `reset` CSR
//! resets to all-ones, active-high) and the AD/DA converter held (`adda_nrst`
//! low). This module brings the system up in a defined order so each stage only
//! starts once its prerequisites are in place:
//!
//! 1. **Write all known registers** — replay the persisted settings (MAC, PTP
//!    grandmaster, servo/parser tuning, streams) while everything is still in
//!    reset, so modules see a stable configuration the instant they release.
//! 2. **Ethernet out of reset** — release the MAC/PHY so the link can come up and
//!    packets flow; bring up the TAP bridge so DHCP can run.
//! 3. **Wait for an IP** — block until the FPGA's IP CSR is populated (the
//!    [monitor](crate::monitor) writes it from the TAP's lease).
//! 4. **PTP out of reset** — start the clock once the network identity is known.
//! 5. **Audio out of reset** — release the RX/TX paths and the AD/DA converter
//!    last, so audio only starts on a configured, time-synced node.
//!
//! To make the bring-up deterministic even on a daemon restart (the FPGA may
//! still be running from a previous session), step 0 re-asserts every reset, so
//! the sequence always starts from the documented all-in-reset state.

use std::net::Ipv4Addr;
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{anyhow, Context, Result};

use aes67_config::{ConfigError, ControlApi, RxStream, TxStream};

use crate::persist::{NetworkCfg, Settings, TransportCfg};
use crate::{
    bridge, gpio::IrqLine, monitor, netif, netif::Tap, replay_settings, SharedConfig, SharedDevice,
};

/// Unified reset CSR (active-high, 1 = held in reset).
const REG_RESET: &str = "aes67_csr_reset";
const RESET_PTP: u64 = 1 << 0;
const RESET_TX: u64 = 1 << 1;
const RESET_RX: u64 = 1 << 2;
const RESET_ETH: u64 = 1 << 3;
const RESET_ALL: u64 = RESET_PTP | RESET_TX | RESET_RX | RESET_ETH;

/// Control CSR holding the AD/DA converter reset-release (1 = released).
const REG_CTRL: &str = "aes67_csr_ctrl";
const CTRL_ADDA_NRST: u64 = 1 << 4;

const DEFAULT_MTU: u32 = 1500;
const DEFAULT_POLL_MS: u64 = 50;

/// How long to wait for an IP before proceeding anyway (with a warning), so the
/// daemon never wedges if the network never hands out a lease.
const IP_WAIT_TIMEOUT: Duration = Duration::from_secs(30);
const IP_WAIT_LOG_EVERY: Duration = Duration::from_secs(5);

/// Bring the FPGA up at daemon start. Intended to run on its own thread; errors
/// are returned for the caller to log.
///
/// Chooses between two paths based on the FPGA's current state:
///
/// * **Warm restart** — the FPGA is already running (no reset domains held) *and*
///   has an IP. This means the FPGA kept running while only the daemon restarted
///   (e.g. it crashed), so its MAC/IP/audio must not be disturbed: we only
///   refresh our PTP and stream config and resume the TAP bridge.
/// * **Cold start** — anything else (the FPGA is freshly reset, or half
///   configured): run the full staged bring-up from the all-held state.
pub fn run(device: SharedDevice, config: SharedConfig, verbose: u8) -> Result<()> {
    // Snapshot of the run config for this bring-up; the monitor keeps the shared
    // handle so later config changes are picked up on FPGA-reset recovery.
    let (network, settings, kernel) = {
        let c = config.lock().unwrap();
        let kernel = matches!(c.transport, Some(TransportCfg::Kernel { .. }));
        (c.network.clone(), c.settings.clone(), kernel)
    };
    if is_warm_restart(&device) {
        warm_resume(&device, &config, &network, &settings, kernel, verbose)
    } else {
        cold_start(&device, &config, &network, &settings, kernel, verbose)
    }
}

/// The FPGA is already up (resets released) and has an IP — only the daemon
/// restarted. Refresh PTP + stream config and resume bridging; touch nothing else.
fn warm_resume(
    device: &SharedDevice,
    config: &SharedConfig,
    network: &NetworkCfg,
    settings: &Settings,
    kernel: bool,
    verbose: u8,
) -> Result<()> {
    eprintln!(
        "aes67d: startup: FPGA already running with an IP — warm restart, \
         refreshing PTP/stream config only"
    );
    apply_ptp_and_streams(device, settings, verbose);

    if network.tap.is_some() || kernel {
        // Use the FPGA's current MAC; do not reprogram it. In TAP mode the TAP is
        // recreated fresh (it died with the previous daemon process), so its DHCP
        // lease is gone — the monitor re-runs DHCP on the first link-up. In kernel
        // mode the netdev keeps running; the monitor just re-attaches IP/IGMP.
        let mac = device.lock().unwrap().get_mac().context("reading MAC")?;
        bring_up_network(device, config, network, kernel, mac, verbose)
            .context("bringing up network services")?;
    }
    eprintln!("aes67d: startup: warm restart complete");
    Ok(())
}

/// Full staged bring-up from the all-held reset state.
fn cold_start(
    device: &SharedDevice,
    config: &SharedConfig,
    network: &NetworkCfg,
    settings: &Settings,
    kernel: bool,
    verbose: u8,
) -> Result<()> {
    // Step 0: known state — hold everything in reset.
    hold_all_resets(device).context("asserting all resets")?;
    if verbose >= 1 {
        eprintln!("aes67d: startup: cold start — all domains held in reset");
    }

    // Step 1: write all known registers while modules are held. A MAC failure
    // here must NOT abort the sequence — otherwise the eth domain below would be
    // left asserted and the MAC stuck in reset.
    replay_settings(device, settings, verbose);
    let mac = match apply_mac(device, network) {
        Ok(mac) => {
            if verbose >= 1 {
                eprintln!("aes67d: startup: registers written, MAC configured");
            }
            Some(mac)
        }
        Err(e) => {
            eprintln!("aes67d: startup: WARNING — MAC config failed, continuing: {e:#}");
            None
        }
    };

    // Step 2: Ethernet out of reset, then bring up the network so DHCP can run.
    release(device, RESET_ETH, "Ethernet").context("releasing Ethernet reset")?;

    if network.tap.is_some() || kernel {
        // Fall back to the FPGA's current MAC if programming above failed.
        let mac = match mac {
            Some(m) => m,
            None => device.lock().unwrap().get_mac().context("reading MAC")?,
        };
        // Cold start: the first link-up should trigger DHCP (TAP mode); in kernel
        // mode the system network manager owns DHCP.
        let networked = bring_up_network(device, config, network, kernel, mac, verbose)
            .context("bringing up network services")?;
        if networked {
            // Step 3: wait until the monitor has programmed an IP from the lease.
            wait_for_ip(device, verbose);
        }
    } else {
        eprintln!("aes67d: startup: no TAP / kernel netdev configured — skipping IP wait");
    }

    // Step 4: PTP out of reset.
    release(device, RESET_PTP, "PTP").context("releasing PTP reset")?;

    // Step 5: audio RX/TX (and the AD/DA converter) out of reset, last.
    release_audio(device).context("releasing audio reset")?;
    eprintln!("aes67d: startup: audio TX/RX released from reset — startup complete");

    Ok(())
}

/// A warm restart is when the FPGA already has **every** reset domain released:
/// the FPGA kept running while only the daemon restarted, so we must not disturb
/// it. The gateware powers up with all resets held (`reset` CSR resets to
/// all-ones), and only a daemon ever releases them in the staged bring-up — so a
/// fully-released state unambiguously means "already brought up". The IP is *not*
/// part of this test: it is mirrored asynchronously (DHCP + monitor), so a running
/// FPGA may momentarily read no IP, and resetting it then would needlessly glitch
/// audio/PTP — exactly the daemon-restart wipe we want to avoid. A partially
/// released state (interrupted bring-up) still reads as cold and is rebuilt.
fn is_warm_restart(device: &SharedDevice) -> bool {
    matches!(
        device.lock().unwrap().read_register(REG_RESET),
        Ok(v) if v & RESET_ALL == 0
    )
}

/// True once the FPGA's reset CSR reads all-domains-held again. Since the daemon
/// leaves it cleared after startup and is the only writer, seeing the power-on
/// value during normal operation means the FPGA was reset/reconfigured.
pub fn all_resets_asserted(device: &SharedDevice) -> bool {
    matches!(
        device.lock().unwrap().read_register(REG_RESET),
        Ok(v) if v & RESET_ALL == RESET_ALL
    )
}

/// Re-apply the full configuration after the FPGA was reset back to its power-on
/// state. Restores all known registers, the MAC, and the last-known IP, then
/// releases the reset domains in order. Unlike [`run`] it does not touch the
/// (still-running) TAP bridge — only the FPGA-side state is rebuilt.
pub fn reconfigure(
    device: &SharedDevice,
    network: &NetworkCfg,
    settings: &Settings,
    ip: Option<Ipv4Addr>,
    verbose: u8,
) -> Result<()> {
    eprintln!("aes67d: startup: FPGA reset detected — restoring last-known state");
    hold_all_resets(device)?;
    replay_settings(device, settings, verbose);
    if let Err(e) = apply_mac(device, network) {
        eprintln!("aes67d: startup: WARNING — MAC restore failed: {e:#}");
    }
    release(device, RESET_ETH, "Ethernet")?;
    if let Some(ip) = ip {
        device.lock().unwrap().set_ip(ip)?;
        eprintln!("aes67d: startup: restored IP {ip}");
    }
    release(device, RESET_PTP, "PTP")?;
    release_audio(device)?;
    eprintln!("aes67d: startup: FPGA reconfiguration complete");
    Ok(())
}

/// Assert every reset domain (known all-in-reset state).
fn hold_all_resets(device: &SharedDevice) -> Result<(), ConfigError> {
    let mut d = device.lock().unwrap();
    d.write_register(REG_RESET, RESET_ALL)?;
    let got = d.read_register(REG_RESET)?;
    eprintln!("aes67d: startup: reset CSR = {got:#06x} (all held; wrote {RESET_ALL:#06x})");
    Ok(())
}

/// Clear the given reset bits (release those domains), preserving the others,
/// then read back so a write-decode problem (readback ≠ written) or a polarity
/// surprise is visible in the log.
fn release(device: &SharedDevice, bits: u64, label: &str) -> Result<(), ConfigError> {
    let mut d = device.lock().unwrap();
    let cur = d.read_register(REG_RESET)?;
    let new = cur & !bits;
    d.write_register(REG_RESET, new)?;
    let got = d.read_register(REG_RESET)?;
    eprintln!(
        "aes67d: startup: {label} released — reset CSR {cur:#06x} -> {got:#06x} (wrote {new:#06x})"
    );
    if got != new {
        eprintln!(
            "aes67d: startup: WARNING — reset CSR readback {got:#06x} != written {new:#06x}; \
             bus write or CSR-decode problem?"
        );
    }
    Ok(())
}

/// Release the audio paths and the AD/DA converter together.
fn release_audio(device: &SharedDevice) -> Result<(), ConfigError> {
    release(device, RESET_TX | RESET_RX, "audio TX/RX")?;
    // adda_nrst is active-high release; set it without disturbing other ctrl bits.
    let mut d = device.lock().unwrap();
    let ctrl = d.read_register(REG_CTRL)?;
    d.write_register(REG_CTRL, ctrl | CTRL_ADDA_NRST)
}

/// Re-apply only the PTP grandmaster and the RX/TX stream config (the daemon's
/// "live" config), without touching MAC/IP/resets. Used on a warm restart.
fn apply_ptp_and_streams(device: &SharedDevice, settings: &Settings, verbose: u8) {
    let mut d = device.lock().unwrap();
    let mut n = 0usize;

    if let Some(gm) = settings.grandmaster {
        match d.set_grandmaster(gm.into()) {
            Ok(()) => n += 1,
            Err(e) => eprintln!("aes67d: startup: refresh grandmaster failed: {e}"),
        }
    }
    for p in settings.tx_streams.values() {
        match TxStream::try_from(p.clone()).and_then(|s| d.write_tx_stream(&s)) {
            Ok(()) => n += 1,
            Err(e) => eprintln!("aes67d: startup: refresh tx stream {} failed: {e}", p.id),
        }
    }
    for p in settings.rx_streams.values() {
        match RxStream::try_from(p.clone()).and_then(|s| d.write_rx_stream(&s)) {
            Ok(()) => n += 1,
            Err(e) => eprintln!("aes67d: startup: refresh rx stream {} failed: {e}", p.id),
        }
    }

    if verbose >= 1 && n > 0 {
        eprintln!("aes67d: startup: refreshed {n} PTP/stream setting(s)");
    }
}

/// Program the configured MAC (if any) and read back the effective one.
fn apply_mac(device: &SharedDevice, net: &NetworkCfg) -> Result<[u8; 6]> {
    let mut d = device.lock().unwrap();
    if let Some(s) = &net.mac {
        d.set_mac(parse_mac(s)?)?;
    }
    Ok(d.get_mac()?)
}

/// Poll the FPGA's IP CSR until it is populated (or the timeout elapses).
fn wait_for_ip(device: &SharedDevice, verbose: u8) {
    let start = Instant::now();
    let mut last_log = start;
    loop {
        match device.lock().unwrap().get_ip() {
            Ok(ip) if !ip.is_unspecified() => {
                eprintln!("aes67d: startup: FPGA has IP {ip}");
                return;
            }
            Ok(_) => {}
            Err(e) => eprintln!("aes67d: startup: reading IP failed: {e}"),
        }
        if start.elapsed() >= IP_WAIT_TIMEOUT {
            eprintln!(
                "aes67d: startup: WARNING — no IP after {}s, proceeding without one",
                IP_WAIT_TIMEOUT.as_secs()
            );
            return;
        }
        if verbose >= 1 && last_log.elapsed() >= IP_WAIT_LOG_EVERY {
            eprintln!("aes67d: startup: waiting for an IP lease…");
            last_log = Instant::now();
        }
        std::thread::sleep(Duration::from_millis(250));
    }
}

/// Bring up the network backing for the FPGA's Ethernet and spawn the monitor.
/// Returns `true` if a backing was started (so the caller should wait for an IP),
/// `false` if neither a TAP nor a kernel netdev applies.
///
/// * **TAP mode** (`net.tap` set): create the TAP, bridge frames, and run the
///   monitor with carrier + DHCP driving.
/// * **Kernel mode**: the `aes67_eth` netdev carries frames and owns its carrier;
///   the monitor only mirrors the IP and joins IGMP on that interface (resolved
///   from `net.iface` or the FPGA MAC).
fn bring_up_network(
    device: &SharedDevice,
    config: &SharedConfig,
    net: &NetworkCfg,
    kernel: bool,
    mac: [u8; 6],
    verbose: u8,
) -> Result<bool> {
    if net.tap.is_some() {
        bring_up_tap(device, config, net, mac, verbose)?;
        return Ok(true);
    }
    if kernel {
        match resolve_kernel_iface(net, mac) {
            Some(iface) => {
                eprintln!(
                    "aes67d: startup: kernel netdev mode — mirroring IP and joining IGMP on {iface}"
                );
                monitor::spawn(
                    Arc::clone(device),
                    iface,
                    monitor::NetLink::Kernel,
                    Arc::clone(config),
                    verbose,
                );
                return Ok(true);
            }
            None => {
                eprintln!(
                    "aes67d: startup: WARNING — kernel mode but no netdev interface found \
                     (set network.iface / --iface, or ensure the FPGA MAC matches a NIC); \
                     FPGA IP mirroring and IGMP are disabled"
                );
                return Ok(false);
            }
        }
    }
    Ok(false)
}

/// Resolve the kernel-transport netdev interface: the explicitly configured name,
/// else the interface whose MAC matches the FPGA's (the `aes67_eth` netdev).
pub fn resolve_kernel_iface(net: &NetworkCfg, mac: [u8; 6]) -> Option<String> {
    if let Some(iface) = net.iface.clone() {
        return Some(iface);
    }
    netif::interface_by_mac(mac)
}

/// Create the TAP, configure it, and spawn the bridge + monitor services.
fn bring_up_tap(
    device: &SharedDevice,
    config: &SharedConfig,
    net: &NetworkCfg,
    mac: [u8; 6],
    verbose: u8,
) -> Result<()> {
    let tap_name = net.tap.as_deref().expect("bring_up_tap without a tap name");
    let mtu = net.mtu.unwrap_or(DEFAULT_MTU);
    let poll_ms = net.poll_ms.unwrap_or(DEFAULT_POLL_MS);

    if verbose >= 1 {
        if let Some((base, size)) = device.lock().unwrap().map().region("eth_buf") {
            eprintln!(
                "aes67d: eth_buf region @ 0x{base:08x} ({size} bytes), RX @ 0x{base:08x}, TX @ 0x{:08x}",
                base + 0x2000
            );
        } else {
            eprintln!("aes67d: WARNING: no 'eth_buf' region in the CSR map");
        }
    }

    let tap = Tap::create(tap_name).context("creating TAP (needs CAP_NET_ADMIN)")?;
    tap.configure(mac, mtu)
        .with_context(|| format!("configuring {}", tap.name()))?;
    eprintln!(
        "aes67d: bridge up on {} (mac {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}, mtu {})",
        tap.name(),
        mac[0],
        mac[1],
        mac[2],
        mac[3],
        mac[4],
        mac[5],
        mtu
    );

    let (rx_tap, tx_tap) = tap.split().context("splitting TAP handle")?;
    let irq = match &net.irq_gpio {
        Some(spec) => Some(open_irq(spec)?),
        None => {
            eprintln!("aes67d: no irq_gpio; polling RX every {poll_ms} ms");
            None
        }
    };

    bridge::spawn(Arc::clone(device), rx_tap, tx_tap, irq, poll_ms, verbose);

    // FPGA PHY link → TAP carrier, the TAP's assigned address → FPGA IP CSR, and
    // FPGA-reset recovery. The monitor needs the config to rebuild FPGA state.
    let carrier_tap = tap.clone_handle().context("cloning TAP handle for link monitor")?;
    monitor::spawn(
        Arc::clone(device),
        tap.name().to_string(),
        monitor::NetLink::Tap(carrier_tap),
        Arc::clone(config),
        verbose,
    );
    Ok(())
}

/// Parse a `CHIP:LINE` GPIO spec and request the line.
fn open_irq(spec: &str) -> Result<IrqLine> {
    let (chip, line) = spec
        .rsplit_once(':')
        .ok_or_else(|| anyhow!("irq_gpio must be CHIP:LINE, e.g. /dev/gpiochip0:17"))?;
    let chip = if chip.contains('/') {
        chip.to_string()
    } else {
        format!("/dev/gpiochip{chip}")
    };
    let offset: u32 = line.parse().context("GPIO line offset")?;
    IrqLine::open(&chip, offset).map_err(|e| anyhow!("opening GPIO {chip}:{offset}: {e}"))
}

/// Parse a MAC like `02:00:00:12:34:56` (or `-` separated).
pub fn parse_mac(s: &str) -> Result<[u8; 6]> {
    let parts: Vec<&str> = s.split([':', '-']).collect();
    if parts.len() != 6 {
        return Err(anyhow!("MAC must have 6 octets separated by ':' or '-', got '{s}'"));
    }
    let mut mac = [0u8; 6];
    for (i, p) in parts.iter().enumerate() {
        mac[i] = u8::from_str_radix(p, 16).map_err(|e| anyhow!("invalid MAC octet '{p}': {e}"))?;
    }
    Ok(mac)
}

#[cfg(test)]
mod tests {
    use super::*;
    use aes67_config::{CsrMap, Device, Transport};
    use aes67_transport::MockTransport;
    use std::sync::Mutex;

    fn mock_device() -> SharedDevice {
        let map = CsrMap::from_csv(
            "\
csr_register,aes67_csr_reset,0x10,1,rw
csr_register,aes67_csr_ctrl,0x14,1,rw
csr_register,aes67_csr_ip_addr,0x18,1,rw
",
        )
        .unwrap();
        let t: Box<dyn Transport + Send> = Box::new(MockTransport::new());
        Arc::new(Mutex::new(Device::new(t, map)))
    }

    fn reg(d: &SharedDevice, name: &str) -> u64 {
        d.lock().unwrap().read_register(name).unwrap()
    }

    #[test]
    fn reset_release_order_clears_the_right_bits() {
        let d = mock_device();

        // Step 0: everything held.
        hold_all_resets(&d).unwrap();
        assert_eq!(reg(&d, REG_RESET), RESET_ALL);

        // Step 2: Ethernet released, the rest still held.
        release(&d, RESET_ETH, "eth").unwrap();
        assert_eq!(reg(&d, REG_RESET), RESET_PTP | RESET_TX | RESET_RX);

        // Step 4: PTP released; only audio still held.
        release(&d, RESET_PTP, "ptp").unwrap();
        assert_eq!(reg(&d, REG_RESET), RESET_TX | RESET_RX);

        // Step 5: audio released and the AD/DA converter taken out of reset.
        release_audio(&d).unwrap();
        assert_eq!(reg(&d, REG_RESET), 0);
        assert_eq!(reg(&d, REG_CTRL) & CTRL_ADDA_NRST, CTRL_ADDA_NRST);
    }

    #[test]
    fn warm_restart_is_decided_by_released_resets_alone() {
        let d = mock_device();

        // Fresh power-on: the FPGA holds every reset domain → cold start.
        hold_all_resets(&d).unwrap();
        assert!(!is_warm_restart(&d));

        // A prior daemon released every domain → warm, regardless of IP (the FPGA
        // is running; a restart must not reset it even before the IP is mirrored).
        d.lock().unwrap().write_register(REG_RESET, 0).unwrap();
        assert!(is_warm_restart(&d));

        // Partially released (interrupted bring-up) → still cold, gets rebuilt.
        d.lock().unwrap().write_register(REG_RESET, RESET_PTP).unwrap();
        assert!(!is_warm_restart(&d));
    }

    #[test]
    fn release_audio_preserves_other_ctrl_bits() {
        let d = mock_device();
        // Pretend another control bit (e.g. eth_tx_request) is set.
        d.lock().unwrap().write_register(REG_CTRL, 1 << 3).unwrap();
        release_audio(&d).unwrap();
        let ctrl = reg(&d, REG_CTRL);
        assert_eq!(ctrl & CTRL_ADDA_NRST, CTRL_ADDA_NRST); // released
        assert_eq!(ctrl & (1 << 3), 1 << 3); // untouched
    }
}
