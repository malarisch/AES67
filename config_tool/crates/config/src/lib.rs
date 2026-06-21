//! AES67 FPGA configuration library.
//!
//! The middle layer of the config tool. It sits on top of the
//! [`aes67_transport`] HAL and turns raw peek/poke into named, typed register
//! access:
//!   * [`CsrMap`] — the register map, loaded dynamically from the LiteX
//!     `csr.csv`/`csr.json` (no hard-coded addresses).
//!   * [`Device`] — a transport bound to a map; read/write CSRs by name.
//!   * [`Aes67Config`] — semantic helpers (MAC, IP, PTP grandmaster, …).
//!
//! ```ignore
//! use aes67_config::{CsrMap, Device, Aes67Config};
//! use aes67_transport::UartTransport;
//!
//! let map = CsrMap::from_path("litex_soc/build/aes67_bridge/csr.csv")?;
//! let transport = UartTransport::open("/dev/ttyUSB0", 115_200)?;
//! let mut dev = Device::new(transport, map);
//! dev.set_ip("192.168.1.42".parse()?)?;
//! ```

use thiserror::Error;

pub mod config;
pub mod control;
pub mod csr;
pub mod device;
pub mod ethbuf;
pub mod proto_conv;
pub mod stream;

pub use config::{Aes67Config, PtpGrandmaster};
pub use control::{ControlApi, RegisterInfo};
pub use csr::{Access, CsrMap, Register};
pub use device::Device;
pub use ethbuf::EthBufBridge;
pub use stream::{Aes67Streams, RxStream, TxStream};

// Re-export the transport surface so downstream users (the CLI) need only
// depend on this crate.
pub use aes67_transport::{
    SpiTransport, Transport, TransportError, UartTransport, DEFAULT_BAUD_RATE, DEFAULT_SPI_HZ,
};

/// Errors from the configuration library.
#[derive(Debug, Error)]
pub enum ConfigError {
    #[error("I/O error: {0}")]
    Io(String),

    #[error("parse error: {0}")]
    Parse(String),

    #[error("unknown register: '{0}'")]
    UnknownRegister(String),

    #[error("register '{0}' is read-only")]
    ReadOnly(String),

    #[error("value does not fit register '{name}' ({bits} bits)")]
    ValueTooWide { name: String, bits: u32 },

    #[error("unsupported: {0}")]
    Unsupported(String),

    #[error("daemon error: {0}")]
    Remote(String),

    #[error(transparent)]
    Transport(#[from] TransportError),
}
