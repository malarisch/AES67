//! Transport HAL for the AES67 FPGA Wishbone bus.
//!
//! This is the lowest layer of the config tool. It abstracts *how* a 32-bit
//! word reaches the FPGA's Wishbone bus into a single [`Transport`] trait with
//! exactly two operations — [`peek`](Transport::peek) and
//! [`poke`](Transport::poke) — over a byte address.
//!
//! Two backends implement it, both speaking the LiteX `litex_server` wire
//! protocol that the matching gateware bridge decodes:
//!   * [`UartTransport`] — the `uartbone` UART→Wishbone bridge.
//!   * [`SpiTransport`]   — the `spibone` 4-wire SPI→Wishbone bridge (Linux
//!     `spidev` host backend).
//!
//! Both take a **byte** address (the address straight out of the LiteX
//! `csr.csv`/`csr.json`). Each backend applies the address convention its
//! gateware expects:
//!   * uartbone consumes a *word* address, so [`UartTransport`] sends `addr >> 2`.
//!   * spibone drops the low two bits in gateware (`bus.adr.eq(address[2:])`),
//!     so [`SpiTransport`] sends the full byte address.
//!
//! Higher layers never see this difference — they always pass byte addresses.

use thiserror::Error;

mod uart;
pub use uart::{UartTransport, DEFAULT_BAUD_RATE};

mod spi;
pub use spi::{SpiTransport, DEFAULT_SPI_HZ};

mod kernel;
pub use kernel::{KernelTransport, DEFAULT_CTL_PATH};

#[cfg(feature = "mock")]
pub use mock::MockTransport;

/// In-memory transport for tests across crates (enable the `mock` feature).
#[cfg(feature = "mock")]
mod mock {
    use std::collections::HashMap;

    use crate::{Transport, TransportError};

    /// A fake Wishbone bus: a flat `addr -> word` map. Reads of unwritten
    /// addresses return 0. `Send`, so it can back a daemon `Device` in tests.
    #[derive(Default)]
    pub struct MockTransport {
        mem: HashMap<u32, u32>,
    }

    impl MockTransport {
        pub fn new() -> Self {
            Self::default()
        }
    }

    impl Transport for MockTransport {
        fn peek(&mut self, addr: u32) -> Result<u32, TransportError> {
            Ok(*self.mem.get(&addr).unwrap_or(&0))
        }
        fn poke(&mut self, addr: u32, value: u32) -> Result<(), TransportError> {
            self.mem.insert(addr, value);
            Ok(())
        }
    }
}

/// Errors a transport backend can raise.
#[derive(Debug, Error)]
pub enum TransportError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("serial port error: {0}")]
    Serial(String),

    #[error("timed out waiting for device response")]
    Timeout,

    #[error("unexpected response from device: {0}")]
    WrongResponse(String),

    #[error("transport backend not supported on this platform: {0}")]
    Unsupported(String),

    #[error("invalid configuration: {0}")]
    Config(String),
}

/// A Wishbone transport: read and write 32-bit words by byte address.
///
/// Implementors handle framing and any address-convention adjustment their
/// gateware bridge requires; callers always work in **byte** addresses (as
/// found in the LiteX CSR map).
pub trait Transport {
    /// Read the 32-bit word at `addr`.
    fn peek(&mut self, addr: u32) -> Result<u32, TransportError>;

    /// Write the 32-bit `value` to the word at `addr`.
    fn poke(&mut self, addr: u32, value: u32) -> Result<(), TransportError>;

    /// Read `count` consecutive words starting at byte `addr` (the address
    /// advances by 4 per word). Needed for bulk transfers (e.g. shuttling
    /// Ethernet frames through `eth_buf`).
    ///
    /// The default loops [`peek`](Transport::peek); backends with a native burst
    /// (uartbone) override it. spibone has no burst, so it keeps the default.
    fn read_burst(&mut self, addr: u32, count: usize) -> Result<Vec<u32>, TransportError> {
        let mut out = Vec::with_capacity(count);
        for i in 0..count {
            out.push(self.peek(addr + 4 * i as u32)?);
        }
        Ok(out)
    }

    /// Write `values` to consecutive words starting at byte `addr`. Default loops
    /// [`poke`](Transport::poke); uartbone overrides with a native burst.
    fn write_burst(&mut self, addr: u32, values: &[u32]) -> Result<(), TransportError> {
        for (i, &v) in values.iter().enumerate() {
            self.poke(addr + 4 * i as u32, v)?;
        }
        Ok(())
    }
}

// Allow boxing a transport behind the trait while still using it as a Transport
// (so `Device<Box<dyn Transport>>` and `Device<Box<dyn Transport + Send>>` both
// work). Generic over `?Sized` so any boxed backend — sized or trait object,
// with or without `+ Send` — is covered, and burst overrides are forwarded.
impl<T: Transport + ?Sized> Transport for Box<T> {
    fn peek(&mut self, addr: u32) -> Result<u32, TransportError> {
        (**self).peek(addr)
    }
    fn poke(&mut self, addr: u32, value: u32) -> Result<(), TransportError> {
        (**self).poke(addr, value)
    }
    fn read_burst(&mut self, addr: u32, count: usize) -> Result<Vec<u32>, TransportError> {
        (**self).read_burst(addr, count)
    }
    fn write_burst(&mut self, addr: u32, values: &[u32]) -> Result<(), TransportError> {
        (**self).write_burst(addr, values)
    }
}
