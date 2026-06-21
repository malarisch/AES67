//! SPI backend — the LiteX `spibone` 4-wire SPI→Wishbone bridge.
//!
//! Wire protocol (4-wire mode, see `litex/soc/cores/spi/spi_bone.py`):
//!   write: master clocks out `0x00`, the 32-bit address (big-endian), then the
//!          32-bit value (big-endian). The device holds `MISO` high (`0xFF`)
//!          until the Wishbone write completes, then clocks out a `0x00` ack.
//!   read:  master clocks out `0x01` and the 32-bit address (big-endian). The
//!          device holds `MISO` high until data is ready, clocks out a `0x01`
//!          sync byte, then the 32-bit value (big-endian).
//!
//! spibone drops the low two address bits in gateware
//! (`bus.adr.eq(address[2:])`), so the **full byte address** goes on the wire.
//!
//! The host backend is Linux `spidev`. Each transaction is a single full-duplex
//! `spidev` transfer so chip-select stays asserted across the variable-latency
//! response; we send trailing `0xFF` padding to clock the response out and scan
//! the returned bytes for the sync/ack marker.

#[cfg(target_os = "linux")]
pub use linux::SpiTransport;

#[cfg(not(target_os = "linux"))]
pub use stub::SpiTransport;

/// Default SPI clock in Hz. Conservative; spibone runs at sys/4 and is far
/// faster, so the host clock is the limit. Raise with `--spi-speed` if wiring
/// allows.
pub const DEFAULT_SPI_HZ: u32 = 1_000_000;

/// SPI command bytes (shared so the stub documents them too).
const CMD_WRITE: u8 = 0x00;
const CMD_READ: u8 = 0x01;

/// Trailing padding bytes clocked out to capture the device's response. spibone
/// answers within a couple of bytes at any sane host clock, so this is generous.
const RESPONSE_SLACK: usize = 24;

#[cfg(target_os = "linux")]
mod linux {
    use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};

    use super::{CMD_READ, CMD_WRITE, DEFAULT_SPI_HZ, RESPONSE_SLACK};
    use crate::{Transport, TransportError};

    /// A connection to the FPGA over the `spibone` SPI bridge via Linux spidev.
    pub struct SpiTransport {
        dev: Spidev,
    }

    impl SpiTransport {
        /// Open a spidev node (e.g. `/dev/spidev0.0`) at `speed_hz`, SPI mode 0,
        /// 8 bits per word. Pass `None` for [`DEFAULT_SPI_HZ`].
        pub fn open(path: &str, speed_hz: Option<u32>) -> Result<Self, TransportError> {
            let mut dev = Spidev::open(path)
                .map_err(|e| TransportError::Serial(format!("opening {path}: {e}")))?;
            let options = SpidevOptions::new()
                .bits_per_word(8)
                .max_speed_hz(speed_hz.unwrap_or(DEFAULT_SPI_HZ))
                .mode(SpiModeFlags::SPI_MODE_0)
                .build();
            dev.configure(&options)
                .map_err(|e| TransportError::Serial(format!("configuring {path}: {e}")))?;
            Ok(Self { dev })
        }

        /// Full-duplex transfer: clock `tx` out and capture the same number of
        /// bytes back. Chip-select stays asserted for the whole buffer.
        fn xfer(&mut self, tx: &[u8]) -> Result<Vec<u8>, TransportError> {
            let mut rx = vec![0u8; tx.len()];
            let mut transfer = SpidevTransfer::read_write(tx, &mut rx);
            self.dev.transfer(&mut transfer)?;
            Ok(rx)
        }
    }

    impl Transport for SpiTransport {
        fn peek(&mut self, addr: u32) -> Result<u32, TransportError> {
            // [0x01][addr BE] then padding to clock the sync byte + 4 data bytes.
            let mut tx = Vec::with_capacity(5 + RESPONSE_SLACK);
            tx.push(CMD_READ);
            tx.extend_from_slice(&addr.to_be_bytes());
            tx.resize(tx.len() + RESPONSE_SLACK, 0xFF);
            let rx = self.xfer(&tx)?;

            // Scan past the address echo for the sync byte, skipping the 0xFF
            // the device drives while the Wishbone read is in flight.
            let mut i = 5;
            while i < rx.len() {
                match rx[i] {
                    CMD_READ => {
                        let data = rx
                            .get(i + 1..i + 5)
                            .ok_or(TransportError::Timeout)?;
                        return Ok(u32::from_be_bytes(data.try_into().unwrap()));
                    }
                    0xFF => i += 1,
                    other => {
                        return Err(TransportError::WrongResponse(format!(
                            "read sync byte was 0x{other:02x}, expected 0x01 or 0xff"
                        )))
                    }
                }
            }
            Err(TransportError::Timeout)
        }

        fn poke(&mut self, addr: u32, value: u32) -> Result<(), TransportError> {
            // [0x00][addr BE][value BE] then padding to clock the ack byte.
            let mut tx = Vec::with_capacity(9 + RESPONSE_SLACK);
            tx.push(CMD_WRITE);
            tx.extend_from_slice(&addr.to_be_bytes());
            tx.extend_from_slice(&value.to_be_bytes());
            tx.resize(tx.len() + RESPONSE_SLACK, 0xFF);
            let rx = self.xfer(&tx)?;

            let mut i = 9;
            while i < rx.len() {
                match rx[i] {
                    CMD_WRITE => return Ok(()),
                    0xFF => i += 1,
                    other => {
                        return Err(TransportError::WrongResponse(format!(
                            "write ack byte was 0x{other:02x}, expected 0x00 or 0xff"
                        )))
                    }
                }
            }
            Err(TransportError::Timeout)
        }
    }
}

#[cfg(not(target_os = "linux"))]
mod stub {
    use crate::{Transport, TransportError};

    /// Placeholder used on non-Linux hosts, where no `spidev` is available.
    pub struct SpiTransport;

    impl SpiTransport {
        pub fn open(_path: &str, _speed_hz: Option<u32>) -> Result<Self, TransportError> {
            Err(TransportError::Unsupported(
                "SPI (spidev) backend is only available on Linux".into(),
            ))
        }
    }

    impl Transport for SpiTransport {
        fn peek(&mut self, _addr: u32) -> Result<u32, TransportError> {
            unreachable!("SpiTransport cannot be constructed on this platform")
        }
        fn poke(&mut self, _addr: u32, _value: u32) -> Result<(), TransportError> {
            unreachable!("SpiTransport cannot be constructed on this platform")
        }
    }
}
