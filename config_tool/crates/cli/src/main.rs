//! `aes67cfg` — CLI front-end for configuring the AES67 FPGA registers.
//!
//! Thin layer over the [`ControlApi`](aes67_config::ControlApi): it parses
//! arguments and dispatches to a backend that is either a **direct** transport
//! (uartbone/spibone, when `--uart`/`--spi` is given) or the **daemon** (`aes67d`
//! over its Unix socket, the default). The command logic is identical for both.
//! No register addresses live here — they come from the CSR map / the daemon.

use std::net::Ipv4Addr;
use std::path::PathBuf;

use anyhow::{anyhow, bail, Context, Result};
use clap::{Args, Parser, Subcommand};

use aes67_client::RemoteDevice;
use aes67_config::{
    ControlApi, CsrMap, Device, PtpGrandmaster, RxStream, SpiTransport, Transport, TxStream,
    UartTransport,
};

/// Register name constants used by the convenience subcommands.
const REG_SCRATCH: &str = "aes67_csr_scratch";
const REG_STATUS: &str = "aes67_csr_status";
const REG_PTP_SYNC_INTERVAL: &str = "aes67_csr_ptp_log_msg_interval";
const REG_PTP_ANNOUNCE_INTERVAL: &str = "aes67_csr_ptp_announce_msg_interval";
const REG_PTP_TIME_SOURCE: &str = "aes67_csr_ptp_time_source";

/// Configure the AES67 FPGA over the uartbone (UART) or spibone (SPI) bridge.
#[derive(Parser)]
#[command(name = "aes67cfg", version, about)]
struct Cli {
    /// LiteX CSR map (csr.csv or csr.json) describing the register layout.
    #[arg(
        long,
        global = true,
        value_name = "FILE",
        default_value = "litex_soc/build/aes67_bridge/csr.csv"
    )]
    csr: PathBuf,

    #[command(flatten)]
    transport: TransportArgs,

    #[command(subcommand)]
    command: Command,
}

/// Backend selection. Giving `--uart`/`--spi` talks to the FPGA **directly**;
/// otherwise the CLI connects to the daemon at `--socket`.
#[derive(Args)]
struct TransportArgs {
    /// Direct uartbone serial device, e.g. /dev/ttyUSB0 (bypasses the daemon).
    #[arg(long, global = true, value_name = "DEV", conflicts_with = "spi")]
    uart: Option<String>,

    /// Baud rate for --uart.
    #[arg(long, global = true, default_value_t = aes67_config::DEFAULT_BAUD_RATE)]
    baud: u32,

    /// Direct spibone SPI device, e.g. /dev/spidev0.0 (bypasses the daemon).
    #[arg(long, global = true, value_name = "DEV")]
    spi: Option<String>,

    /// SPI clock in Hz for --spi (default: 1 MHz).
    #[arg(long, global = true, value_name = "HZ")]
    spi_speed: Option<u32>,

    /// Daemon control socket (used when neither --uart nor --spi is given).
    #[arg(long, global = true, value_name = "PATH", default_value = "/run/aes67d.sock")]
    socket: PathBuf,
}

#[derive(Subcommand)]
enum Command {
    /// List the registers in the CSR map (no device access).
    Regs,
    /// Read every register and print its current value.
    Dump,
    /// Read one register by name.
    Get {
        /// Register name (e.g. aes67_csr_status).
        reg: String,
    },
    /// Write one register by name.
    Set {
        /// Register name (e.g. aes67_csr_scratch).
        reg: String,
        /// Value (decimal, or 0x.. / 0b.. / 0o..).
        #[arg(value_parser = parse_u64)]
        value: u64,
    },
    /// Read a raw 32-bit word at an absolute byte address.
    Peek {
        #[arg(value_parser = parse_u32)]
        addr: u32,
    },
    /// Write a raw 32-bit word at an absolute byte address.
    Poke {
        #[arg(value_parser = parse_u32)]
        addr: u32,
        #[arg(value_parser = parse_u32)]
        value: u32,
    },
    /// Get or set the MAC address. Omit VALUE to read it back.
    Mac {
        /// e.g. 02:00:00:12:34:56
        value: Option<String>,
    },
    /// Get or set the IPv4 address. Omit VALUE to read it back.
    Ip {
        /// e.g. 192.168.1.42
        value: Option<Ipv4Addr>,
    },
    /// Get or set the scratch register (round-trip link test).
    Scratch {
        #[arg(value_parser = parse_u32)]
        value: Option<u32>,
    },
    /// Read the AES67 status register.
    Status,
    /// Set PTP grandmaster announce parameters.
    Ptp(PtpArgs),
    /// Configure a transmit (sender) audio stream.
    TxStream(TxStreamArgs),
    /// Configure a receive audio stream.
    RxStream(RxStreamArgs),
}

#[derive(Args)]
struct TxStreamArgs {
    /// Stream slot (0..7).
    #[arg(long)]
    id: u8,
    /// Destination (multicast) IPv4 address.
    #[arg(long)]
    dst_ip: Ipv4Addr,
    /// Channel count (default: number of --ch-ids).
    #[arg(long)]
    channels: Option<u8>,
    /// Samples per RTP packet per channel.
    #[arg(long, default_value_t = 0)]
    spp: u8,
    /// Comma-separated source channel IDs, e.g. 0,1.
    #[arg(long, value_delimiter = ',', default_value = "")]
    ch_ids: Vec<u8>,
    /// RTP SSRC (decimal or 0x..).
    #[arg(long, value_parser = parse_u32, default_value_t = 0)]
    ssrc: u32,
}

#[derive(Args)]
struct RxStreamArgs {
    /// Stream slot (0..7).
    #[arg(long)]
    id: u8,
    /// Destination (multicast) IPv4 address to subscribe to.
    #[arg(long)]
    dst_ip: Ipv4Addr,
    /// Destination UDP port.
    #[arg(long)]
    dst_port: u16,
    /// Comma-separated output channel map, e.g. 0,1.
    #[arg(long, value_delimiter = ',', default_value = "")]
    ch_map: Vec<u8>,
    /// Channel count (default: number of --ch-map entries).
    #[arg(long)]
    channels: Option<u8>,
    /// Output delay in samples.
    #[arg(long, default_value_t = 0)]
    delay: u8,
    /// Samples per channel per packet.
    #[arg(long, default_value_t = 0)]
    spc: u8,
}

#[derive(Args)]
#[command(allow_negative_numbers = true)]
struct PtpArgs {
    #[arg(long)]
    priority1: Option<u8>,
    #[arg(long)]
    priority2: Option<u8>,
    #[arg(long)]
    clock_class: Option<u8>,
    #[arg(long)]
    clock_accuracy: Option<u8>,
    /// Sync / Delay-Req interval (IEEE 1588 logMessageInterval, log2 seconds;
    /// e.g. -3 = 8 msg/s, the AES67 default). Writes ptp_log_msg_interval.
    #[arg(long)]
    sync_interval: Option<i8>,
    /// Announce interval (log2 seconds; e.g. 0 = 1/s, 1 = every 2 s). Writes
    /// ptp_announce_msg_interval.
    #[arg(long)]
    announce_interval: Option<i8>,
    /// PTP timeSource (IEEE 1588 enumeration, e.g. 0xa0 = internal oscillator).
    #[arg(long)]
    time_source: Option<u8>,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    // `regs` is an offline map listing — no backend (hardware or daemon) needed.
    if let Command::Regs = cli.command {
        let map = CsrMap::from_path(&cli.csr)
            .with_context(|| format!("loading CSR map {}", cli.csr.display()))?;
        print_regs(&map);
        return Ok(());
    }

    let mut backend = build_backend(&cli)?;
    dispatch(backend.as_mut(), cli.command)
}

/// Build the backend: a direct transport `Device` when `--uart`/`--spi` is set,
/// otherwise a `RemoteDevice` connected to the daemon. Both are `ControlApi`.
fn build_backend(cli: &Cli) -> Result<Box<dyn ControlApi>> {
    let t = &cli.transport;
    match (&t.uart, &t.spi) {
        (Some(dev), None) => {
            let map = load_map(cli)?;
            let transport: Box<dyn Transport> = Box::new(
                UartTransport::open(dev, t.baud).with_context(|| format!("opening UART {dev}"))?,
            );
            Ok(Box::new(Device::new(transport, map)))
        }
        (None, Some(dev)) => {
            let map = load_map(cli)?;
            let transport: Box<dyn Transport> = Box::new(
                SpiTransport::open(dev, t.spi_speed).with_context(|| format!("opening SPI {dev}"))?,
            );
            Ok(Box::new(Device::new(transport, map)))
        }
        (Some(_), Some(_)) => bail!("--uart and --spi are mutually exclusive"),
        // Default: talk to the daemon.
        (None, None) => {
            let dev = RemoteDevice::connect(&t.socket)
                .with_context(|| format!("connecting to daemon at {}", t.socket.display()))?;
            Ok(Box::new(dev))
        }
    }
}

fn load_map(cli: &Cli) -> Result<CsrMap> {
    CsrMap::from_path(&cli.csr).with_context(|| format!("loading CSR map {}", cli.csr.display()))
}

fn dispatch(dev: &mut dyn ControlApi, cmd: Command) -> Result<()> {
    match cmd {
        Command::Regs => unreachable!("handled before backend construction"),

        Command::Dump => {
            for reg in dev.list_registers()? {
                match dev.read_register(&reg.name) {
                    Ok(v) => println!(
                        "{:<44} @ 0x{:08x} [{}] = 0x{:08x} ({})",
                        reg.name,
                        reg.addr,
                        reg.access.as_str(),
                        v,
                        v
                    ),
                    Err(e) => println!(
                        "{:<44} @ 0x{:08x} [{}] = <error: {e}>",
                        reg.name,
                        reg.addr,
                        reg.access.as_str()
                    ),
                }
            }
        }

        Command::Get { reg } => {
            let v = dev.read_register(&reg)?;
            println!("{reg} = 0x{v:08x} ({v})");
        }

        Command::Set { reg, value } => {
            dev.write_register(&reg, value)?;
            println!("{reg} <- 0x{value:08x}");
        }

        Command::Peek { addr } => {
            let v = dev.read_addr(addr)?;
            println!("0x{addr:08x} = 0x{v:08x} ({v})");
        }

        Command::Poke { addr, value } => {
            dev.write_addr(addr, value)?;
            println!("0x{addr:08x} <- 0x{value:08x}");
        }

        Command::Mac { value } => match value {
            Some(s) => {
                let mac = parse_mac(&s)?;
                dev.set_mac(mac)?;
                println!("MAC <- {}", fmt_mac(mac));
            }
            None => println!("MAC = {}", fmt_mac(dev.get_mac()?)),
        },

        Command::Ip { value } => match value {
            Some(ip) => {
                dev.set_ip(ip)?;
                println!("IP <- {ip}");
            }
            None => println!("IP = {}", dev.get_ip()?),
        },

        Command::Scratch { value } => match value {
            Some(v) => {
                dev.write_register(REG_SCRATCH, v as u64)?;
                let rb = dev.read_register(REG_SCRATCH)? as u32;
                if rb == v {
                    println!("scratch <- 0x{v:08x} (read back OK)");
                } else {
                    bail!("scratch read-back mismatch: wrote 0x{v:08x}, read 0x{rb:08x}");
                }
            }
            None => {
                let v = dev.read_register(REG_SCRATCH)? as u32;
                println!("scratch = 0x{v:08x}");
            }
        },

        Command::Status => {
            let v = dev.read_register(REG_STATUS)? as u32;
            println!("status = 0x{v:08x}");
        }

        Command::Ptp(p) => {
            dev.set_grandmaster(PtpGrandmaster {
                priority1: p.priority1,
                priority2: p.priority2,
                clock_class: p.clock_class,
                clock_accuracy: p.clock_accuracy,
            })?;
            // Message intervals are plain CSRs (signed log2 seconds, 8-bit
            // two's complement). Written only when given.
            if let Some(v) = p.sync_interval {
                dev.write_register(REG_PTP_SYNC_INTERVAL, (v as u8) as u64)?;
            }
            if let Some(v) = p.announce_interval {
                dev.write_register(REG_PTP_ANNOUNCE_INTERVAL, (v as u8) as u64)?;
            }
            if let Some(v) = p.time_source {
                dev.write_register(REG_PTP_TIME_SOURCE, v as u64)?;
            }
            println!("PTP parameters updated");
        }

        Command::TxStream(a) => {
            dev.write_tx_stream(&TxStream {
                id: a.id,
                dst_ip: a.dst_ip,
                channels: a.channels,
                samples_per_packet: a.spp,
                ch_ids: a.ch_ids,
                ssrc: a.ssrc,
            })?;
            println!("tx stream {} configured -> {}", a.id, a.dst_ip);
        }

        Command::RxStream(a) => {
            dev.write_rx_stream(&RxStream {
                id: a.id,
                dst_ip: a.dst_ip,
                dst_port: a.dst_port,
                ch_map: a.ch_map,
                channels: a.channels,
                output_delay: a.delay,
                samples_per_channel: a.spc,
            })?;
            println!("rx stream {} configured <- {}:{}", a.id, a.dst_ip, a.dst_port);
        }
    }
    Ok(())
}

fn print_regs(map: &CsrMap) {
    println!("{} registers:", map.len());
    for reg in map.registers() {
        println!(
            "  {:<44} @ 0x{:08x}  [{}]  ({} word{})",
            reg.name,
            reg.addr,
            reg.access.as_str(),
            reg.size,
            if reg.size == 1 { "" } else { "s" }
        );
    }
}

// -- value parsers ----------------------------------------------------------

fn parse_u32(s: &str) -> Result<u32, String> {
    aes67_config::csr::parse_u32(s)
}

fn parse_u64(s: &str) -> Result<u64, String> {
    let s = s.trim();
    let parsed = if let Some(h) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u64::from_str_radix(h, 16)
    } else if let Some(b) = s.strip_prefix("0b").or_else(|| s.strip_prefix("0B")) {
        u64::from_str_radix(b, 2)
    } else if let Some(o) = s.strip_prefix("0o").or_else(|| s.strip_prefix("0O")) {
        u64::from_str_radix(o, 8)
    } else {
        s.parse::<u64>()
    };
    parsed.map_err(|e| format!("invalid number '{s}': {e}"))
}

fn parse_mac(s: &str) -> Result<[u8; 6]> {
    let parts: Vec<&str> = s.split([':', '-']).collect();
    if parts.len() != 6 {
        bail!("MAC must have 6 octets separated by ':' or '-', got '{s}'");
    }
    let mut mac = [0u8; 6];
    for (i, p) in parts.iter().enumerate() {
        mac[i] = u8::from_str_radix(p, 16)
            .map_err(|e| anyhow!("invalid MAC octet '{p}': {e}"))?;
    }
    Ok(mac)
}

fn fmt_mac(mac: [u8; 6]) -> String {
    mac.iter()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join(":")
}
