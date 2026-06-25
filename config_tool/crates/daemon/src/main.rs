//! `aes67d` — the AES67 control-plane daemon binary.
//!
//! Loads the JSON config (daemon settings + persisted FPGA state), lets CLI flags
//! override it, opens the SPI/UART transport, then runs the FPGA [startup
//! sequence](aes67_daemon::startup) on a background thread while serving the Unix
//! control socket. All server logic lives in the library so it can be
//! integration-tested with a mock transport.

use std::os::unix::net::UnixListener;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use anyhow::{anyhow, Context, Result};
use clap::Parser;

use aes67_config::{
    ControlApi, CsrMap, Device, KernelTransport, SpiTransport, Transport, UartTransport,
};
use aes67_daemon::discovery::{self, Registry, SharedDiscovery};
use aes67_daemon::persist::{DaemonConfig, TransportCfg};
use aes67_daemon::{mdns, rtsp, startup, Persist, Server};

/// Default config-file path (overridable with `--config`).
const DEFAULT_CONFIG: &str = "/etc/aes67d.json";
const DEFAULT_CSR: &str = "litex_soc/build/aes67_bridge/csr.csv";
const DEFAULT_SOCKET: &str = "/run/aes67d.sock";
const DEFAULT_MTU: u32 = 1500;
const DEFAULT_POLL_MS: u64 = 50;

#[derive(Parser)]
#[command(name = "aes67d", version, about)]
struct Cli {
    /// JSON config file: daemon settings + persisted FPGA state. Created/updated
    /// to reflect the running configuration; FPGA settings made over the control
    /// API are saved here and replayed on the next start (everything but the IP).
    #[arg(long, value_name = "FILE", default_value = DEFAULT_CONFIG)]
    config: PathBuf,

    /// LiteX CSR map (csr.csv or csr.json) describing the register layout.
    #[arg(long, value_name = "FILE")]
    csr: Option<PathBuf>,

    /// uartbone serial device, e.g. /dev/ttyUSB0
    #[arg(long, value_name = "DEV", conflicts_with = "spi")]
    uart: Option<String>,
    /// Baud rate for --uart.
    #[arg(long)]
    baud: Option<u32>,
    /// spibone SPI device, e.g. /dev/spidev0.0
    #[arg(long, value_name = "DEV", conflicts_with = "uart")]
    spi: Option<String>,
    /// SPI clock in Hz for --spi.
    #[arg(long, value_name = "HZ")]
    spi_speed: Option<u32>,
    /// aes67_eth kernel control device, e.g. /dev/aes67ctl. Bus access goes
    /// through the kernel module (which also owns the netdev + PHC), so the TAP
    /// bridge is disabled in this mode.
    #[arg(long, value_name = "DEV", conflicts_with_all = ["uart", "spi"])]
    kernel: Option<String>,

    /// Unix socket to listen on for the control API.
    #[arg(long, value_name = "PATH")]
    socket: Option<PathBuf>,

    // -- Network bridge (optional; enabled by --tap) --
    /// TAP interface name to create for the FPGA network bridge (e.g. aes67d0).
    /// Omit to run the control server only (no networking). Needs CAP_NET_ADMIN.
    #[arg(long, value_name = "NAME")]
    tap: Option<String>,
    /// Kernel-transport netdev interface the `aes67_eth` module creates (e.g.
    /// eth1). Used only with --kernel: the daemon mirrors this interface's IP
    /// into the FPGA and joins multicast on it. Auto-detected by FPGA MAC if
    /// omitted.
    #[arg(long, value_name = "NAME")]
    iface: Option<String>,
    /// MTU for the TAP interface.
    #[arg(long)]
    mtu: Option<u32>,
    /// Program this MAC into the FPGA and the TAP (e.g. 02:00:00:12:34:56).
    /// If omitted, the FPGA's current MAC is read and used for the TAP.
    #[arg(long, value_name = "MAC")]
    mac: Option<String>,
    /// GPIO line for the eth_buf RX IRQ, as CHIP:LINE (e.g. /dev/gpiochip0:17 or
    /// 0:17). Omit to poll instead.
    #[arg(long, value_name = "CHIP:LINE")]
    irq_gpio: Option<String>,
    /// RX poll interval / IRQ safety-net timeout in milliseconds.
    #[arg(long)]
    poll_ms: Option<u64>,

    /// Diagnostics: -v periodic frame/byte/drop stats, -vv per-frame decode
    /// (MACs / EtherType / UDP ports), -vvv hex dump of the first bytes.
    #[arg(short, long, action = clap::ArgAction::Count)]
    verbose: u8,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    // Load the persisted config (missing file -> defaults), then let CLI flags
    // override the run configuration. `config` ends up reflecting what we run.
    let mut config = DaemonConfig::load(&cli.config)
        .with_context(|| format!("loading config {}", cli.config.display()))?;
    resolve_config(&mut config, &cli)?;

    let transport_cfg = config
        .transport
        .clone()
        .ok_or_else(|| anyhow!("no transport: pass --uart <dev> or --spi <dev> (saved to config)"))?;
    let csr_path = config.csr.clone().unwrap();
    let socket_path = config.socket.clone().unwrap();
    let verbose = config.verbose.unwrap_or(0);

    // Persist the resolved run configuration so the file always reflects reality.
    config
        .save(&cli.config)
        .with_context(|| format!("saving config {}", cli.config.display()))?;

    let map = CsrMap::from_path(&csr_path)
        .with_context(|| format!("loading CSR map {}", csr_path.display()))?;
    let transport = open_transport(&transport_cfg)?;
    let device = Arc::new(Mutex::new(Device::new(transport, map)));

    // Bind the control socket first, so clients can connect and watch the FPGA
    // bring-up while it runs in the background.
    if socket_path.exists() {
        std::fs::remove_file(&socket_path)
            .with_context(|| format!("removing stale socket {}", socket_path.display()))?;
    }
    let listener = UnixListener::bind(&socket_path)
        .with_context(|| format!("binding socket {}", socket_path.display()))?;
    eprintln!("aes67d: listening on {}", socket_path.display());

    // The live config is shared: the control server records mutations into it,
    // and the startup/monitor threads read the *current* settings from it (so an
    // FPGA-reset recovery replays config made after startup, too).
    let config = Arc::new(Mutex::new(config));

    // Run the staged FPGA startup sequence off the serving thread.
    let startup_dev = Arc::clone(&device);
    let startup_cfg = Arc::clone(&config);
    std::thread::spawn(move || {
        if let Err(e) = startup::run(startup_dev, startup_cfg, verbose) {
            eprintln!("aes67d: startup sequence failed: {e:#}");
        }
    });

    // Network services bind to the FPGA's network interface, which is the TAP in
    // TAP mode and the `aes67_eth` kernel netdev in kernel mode. Read the relevant
    // config once.
    let (discovery_on, rtsp_on, rtsp_port, node_name, kernel) = {
        let c = config.lock().unwrap();
        (
            c.network.discovery != Some(false),
            c.network.rtsp != Some(false),
            c.network.rtsp_port.unwrap_or(rtsp::DEFAULT_RTSP_PORT),
            c.network.node_name.clone().unwrap_or_else(|| "AES67".to_string()),
            matches!(c.transport, Some(TransportCfg::Kernel { .. })),
        )
    };

    // The interface the discovery services run on: the TAP, or the kernel netdev
    // (explicit `network.iface`/--iface, else resolved from the FPGA MAC). Resolve
    // the network config without nesting the device lock under the config lock.
    let net_iface: Option<String> = {
        let (tap, kernel_net) = {
            let c = config.lock().unwrap();
            (c.network.tap.clone(), kernel.then(|| c.network.clone()))
        };
        match (tap, kernel_net) {
            (Some(tap), _) => Some(tap),
            (None, Some(net)) => device
                .lock()
                .unwrap()
                .get_mac()
                .ok()
                .and_then(|mac| startup::resolve_kernel_iface(&net, mac)),
            (None, None) => None,
        }
    };
    if kernel && net_iface.is_none() {
        eprintln!(
            "aes67d: WARNING — kernel mode but no netdev interface resolved; \
             discovery/RTSP/mDNS disabled (set network.iface / --iface)"
        );
    }

    // One shared discovery registry (exists when a network interface does), fed by
    // both SAP and mDNS, and read by the control API.
    let registry: Option<SharedDiscovery> =
        net_iface.as_ref().map(|_| Arc::new(Mutex::new(Registry::default())));

    // SAP/SDP discovery (announce local TX streams, learn remote ones). It waits
    // for the interface address itself, so it need not be sequenced with startup.
    if let (Some(reg), Some(iface), true) = (&registry, &net_iface, discovery_on) {
        discovery::spawn(Arc::clone(&device), Arc::clone(&config), iface.clone(), Arc::clone(reg), verbose);
    }

    // RAVENNA RTSP server (transmitting node serves its TX streams) plus mDNS:
    // the `_rtsp._tcp` node service, per-session `ravenna_session` advertisements,
    // and browsing remote sessions into the same registry.
    if let (Some(reg), Some(iface), true) = (&registry, &net_iface, rtsp_on) {
        rtsp::spawn(Arc::clone(&device), Arc::clone(&config), rtsp_port, verbose);
        mdns::spawn(
            Arc::clone(&device),
            Arc::clone(&config),
            Arc::clone(reg),
            iface.clone(),
            rtsp_port,
            node_name,
            verbose,
        );
    }

    // Hand the same config to the server so control-API mutations are persisted.
    let server = Server::new(device, Some(Persist::new(cli.config.clone(), config)), registry);
    aes67_daemon::run(listener, server);
    Ok(())
}

/// Apply CLI overrides onto the loaded config, filling defaults where neither the
/// CLI nor the file supplied a value.
fn resolve_config(config: &mut DaemonConfig, cli: &Cli) -> Result<()> {
    // Transport: a CLI --uart/--spi/--kernel replaces the stored transport entirely.
    if let Some(dev) = &cli.uart {
        config.transport = Some(TransportCfg::Uart { device: dev.clone(), baud: cli.baud });
    } else if let Some(dev) = &cli.spi {
        config.transport = Some(TransportCfg::Spi { device: dev.clone(), speed_hz: cli.spi_speed });
    } else if let Some(dev) = &cli.kernel {
        config.transport = Some(TransportCfg::Kernel { device: dev.clone() });
    }

    if let Some(p) = &cli.csr {
        config.csr = Some(p.clone());
    }
    config.csr.get_or_insert_with(|| PathBuf::from(DEFAULT_CSR));

    if let Some(p) = &cli.socket {
        config.socket = Some(p.clone());
    }
    config.socket.get_or_insert_with(|| PathBuf::from(DEFAULT_SOCKET));

    // A non-zero -v on the command line overrides the stored verbosity.
    if cli.verbose > 0 {
        config.verbose = Some(cli.verbose);
    }
    config.verbose.get_or_insert(0);

    let net = &mut config.network;
    if cli.tap.is_some() {
        net.tap = cli.tap.clone();
    }
    if cli.iface.is_some() {
        net.iface = cli.iface.clone();
    }
    if cli.mtu.is_some() {
        net.mtu = cli.mtu;
    }
    net.mtu.get_or_insert(DEFAULT_MTU);
    if cli.mac.is_some() {
        net.mac = cli.mac.clone();
    }
    if cli.irq_gpio.is_some() {
        net.irq_gpio = cli.irq_gpio.clone();
    }
    if cli.poll_ms.is_some() {
        net.poll_ms = cli.poll_ms;
    }
    net.poll_ms.get_or_insert(DEFAULT_POLL_MS);

    // In kernel mode the aes67_eth netdev carries frames itself, so the daemon
    // must not also run the userspace TAP bridge — clear any configured tap.
    if matches!(config.transport, Some(TransportCfg::Kernel { .. })) {
        config.network.tap = None;
    }

    Ok(())
}

/// Open the FPGA bus transport described by the config.
fn open_transport(cfg: &TransportCfg) -> Result<Box<dyn Transport + Send>> {
    Ok(match cfg {
        TransportCfg::Uart { device, baud } => {
            let baud = baud.unwrap_or(aes67_config::DEFAULT_BAUD_RATE);
            Box::new(
                UartTransport::open(device, baud).with_context(|| format!("opening UART {device}"))?,
            )
        }
        TransportCfg::Spi { device, speed_hz } => Box::new(
            SpiTransport::open(device, *speed_hz).with_context(|| format!("opening SPI {device}"))?,
        ),
        TransportCfg::Kernel { device } => Box::new(
            KernelTransport::open(device)
                .with_context(|| format!("opening kernel control device {device}"))?,
        ),
    })
}
