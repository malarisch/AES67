//! The TAP ↔ `eth_buf` network bridge services.
//!
//! Two threads share the daemon's single [`SharedDevice`] (the transport
//! arbiter), so packet traffic and control transactions serialise cleanly on the
//! one Wishbone link:
//!
//! * **RX** (FPGA → TAP): wait for the `eth_buf` IRQ (or a poll tick), then drain
//!   every ready frame and write it to the tap.
//! * **TX** (TAP → FPGA): block on tap reads and inject each frame into the
//!   `eth_buf` TX buffer.
//!
//! `verbose` (from `-v`/`-vv`) controls diagnostics: level ≥ 1 logs periodic
//! frame/byte/drop counters; level ≥ 2 logs a one-line decode of every frame
//! (MACs, EtherType, IPv4/UDP ports — so DHCP `68→67` out / `67→68` in are
//! visible); level ≥ 3 adds a hex dump of the first bytes.

use std::fs::File;
use std::io::Write;
use std::sync::Arc;
use std::time::{Duration, Instant};

use aes67_config::EthBufBridge;

use crate::gpio::IrqLine;
use crate::{netif, SharedDevice};

/// Spawn the RX and TX bridge services on their own threads.
pub fn spawn(
    device: SharedDevice,
    rx_tap: File,
    tx_tap: File,
    irq: Option<IrqLine>,
    poll_ms: u64,
    verbose: u8,
) {
    let rx_dev = Arc::clone(&device);
    std::thread::spawn(move || rx_service(rx_dev, rx_tap, irq, poll_ms, verbose));
    std::thread::spawn(move || tx_service(device, tx_tap, verbose));
}

/// FPGA → TAP: wake on the IRQ (or timeout), drain all ready frames to the tap.
fn rx_service(device: SharedDevice, mut tap: File, mut irq: Option<IrqLine>, poll_ms: u64, verbose: u8) {
    if let Err(e) = device.lock().unwrap().eth_irq_enable() {
        eprintln!("aes67d: eth_irq_enable failed: {e}");
    }
    let mut stats = Stats::new("RX←FPGA", verbose);

    loop {
        match &mut irq {
            Some(line) => {
                let _ = line.wait(poll_ms as i32);
            }
            None => std::thread::sleep(Duration::from_millis(poll_ms)),
        }

        // Drain every frame currently available (lock held only per transaction).
        loop {
            let frame = {
                let mut d = device.lock().unwrap();
                match d.eth_rx_ready() {
                    Ok(true) => d.eth_rx_take_one(),
                    Ok(false) => break,
                    Err(e) => {
                        eprintln!("aes67d: rx_ready: {e}");
                        break;
                    }
                }
            };
            match frame {
                Ok(Some(f)) => {
                    stats.frame(&f);
                    if let Err(e) = tap.write_all(&f) {
                        eprintln!("aes67d: tap write: {e}");
                    }
                }
                Ok(None) => stats.drop_invalid(),
                Err(e) => {
                    eprintln!("aes67d: rx_take: {e}");
                    break;
                }
            }
        }
        stats.tick();
    }
}

/// TAP → FPGA: block on tap reads, inject each frame into `eth_buf` TX.
fn tx_service(device: SharedDevice, mut tap: File, verbose: u8) {
    let mut buf = vec![0u8; 2048];
    let mut stats = Stats::new("TX→FPGA", verbose);
    // eth_tx_done starts de-asserted at reset, so the first frame must not wait
    // for it (there is no in-flight transmit yet).
    let mut first = true;
    loop {
        match netif::recv(&mut tap, &mut buf) {
            Ok(0) => break, // tap closed
            Ok(n) => {
                stats.frame(&buf[..n]);
                let mut d = device.lock().unwrap();
                if let Err(e) = d.eth_tx(&buf[..n], !first) {
                    eprintln!("aes67d: eth_tx: {e}");
                }
                first = false;
                stats.tick();
            }
            Err(e) => {
                eprintln!("aes67d: tap read: {e}");
                break;
            }
        }
    }
}

/// Per-direction frame statistics and decode logging.
struct Stats {
    dir: &'static str,
    verbose: u8,
    frames: u64,
    bytes: u64,
    dropped: u64,
    last: Instant,
}

impl Stats {
    fn new(dir: &'static str, verbose: u8) -> Self {
        Self { dir, verbose, frames: 0, bytes: 0, dropped: 0, last: Instant::now() }
    }

    fn frame(&mut self, f: &[u8]) {
        self.frames += 1;
        self.bytes += f.len() as u64;
        if self.verbose >= 2 {
            eprintln!("aes67d: {}: {}", self.dir, summarize(f));
        }
        if self.verbose >= 3 {
            let n = f.len().min(64);
            eprintln!("aes67d: {}: {}", self.dir, hex(&f[..n]));
        }
    }

    fn drop_invalid(&mut self) {
        self.dropped += 1;
        if self.verbose >= 2 {
            eprintln!("aes67d: {}: dropped frame (invalid length)", self.dir);
        }
    }

    /// Emit a counter summary at most every 2 s when verbose ≥ 1.
    fn tick(&mut self) {
        if self.verbose >= 1 && self.last.elapsed() >= Duration::from_secs(2) {
            eprintln!(
                "aes67d: {} stats: {} frames, {} bytes, {} dropped",
                self.dir, self.frames, self.bytes, self.dropped
            );
            self.last = Instant::now();
        }
    }
}

/// One-line decode: MACs, EtherType, and IPv4/UDP ports where present.
fn summarize(f: &[u8]) -> String {
    if f.len() < 14 {
        return format!("runt {} bytes", f.len());
    }
    let et = u16::from_be_bytes([f[12], f[13]]);
    let mut s = format!(
        "{} bytes dst={} src={} type=0x{et:04x}",
        f.len(),
        mac(&f[0..6]),
        mac(&f[6..12])
    );
    match et {
        0x0806 => s.push_str(" arp"),
        0x0800 if f.len() >= 34 => {
            let proto = f[23];
            let ihl = (f[14] & 0x0f) as usize * 4;
            s.push_str(&format!(" ipv4 proto={proto}"));
            if proto == 17 && f.len() >= 14 + ihl + 8 {
                let sp = u16::from_be_bytes([f[14 + ihl], f[14 + ihl + 1]]);
                let dp = u16::from_be_bytes([f[14 + ihl + 2], f[14 + ihl + 3]]);
                s.push_str(&format!(" udp {sp}->{dp}"));
            }
        }
        _ => {}
    }
    s
}

fn mac(b: &[u8]) -> String {
    format!("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", b[0], b[1], b[2], b[3], b[4], b[5])
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect::<Vec<_>>().join(" ")
}
