//! `ControlApi` — the operation surface the daemon exposes and that both the
//! in-process [`Device`] and a future remote client implement.
//!
//! The CLI (and any other consumer) is written against this trait, so it does
//! not care whether it holds a direct-transport `Device` or a socket-backed
//! `RemoteDevice`. This is the seam that lets the tool move from "owns the
//! SPI/UART link" to "talks to `aes67d`" without rewriting command logic.

use std::net::Ipv4Addr;

use aes67_transport::Transport;

use crate::config::Aes67Config;
use crate::csr::Access;
use crate::device::Device;
use crate::stream::{Aes67Streams, RxStream, TxStream};
use crate::{ConfigError, PtpGrandmaster};

/// Name of the unified reset CSR (bit 0 ptp, 1 tx, 2 rx, 3 eth).
const REG_RESET: &str = "aes67_csr_reset";

/// An owned, serialisable view of a CSR map entry (no borrow of the map).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RegisterInfo {
    pub name: String,
    pub addr: u32,
    pub size: u32,
    pub access: Access,
}

/// The control-plane operation surface.
///
/// Implementors: [`Device`] (direct transport, used by the daemon) and — later —
/// a remote client over the daemon socket.
pub trait ControlApi {
    fn list_registers(&mut self) -> Result<Vec<RegisterInfo>, ConfigError>;
    fn read_register(&mut self, name: &str) -> Result<u64, ConfigError>;
    fn write_register(&mut self, name: &str, value: u64) -> Result<(), ConfigError>;
    fn read_addr(&mut self, addr: u32) -> Result<u32, ConfigError>;
    fn write_addr(&mut self, addr: u32, value: u32) -> Result<(), ConfigError>;

    fn get_mac(&mut self) -> Result<[u8; 6], ConfigError>;
    fn set_mac(&mut self, mac: [u8; 6]) -> Result<(), ConfigError>;
    fn get_ip(&mut self) -> Result<Ipv4Addr, ConfigError>;
    fn set_ip(&mut self, ip: Ipv4Addr) -> Result<(), ConfigError>;

    fn set_grandmaster(&mut self, gm: PtpGrandmaster) -> Result<(), ConfigError>;
    fn write_tx_stream(&mut self, stream: &TxStream) -> Result<(), ConfigError>;
    fn write_rx_stream(&mut self, stream: &RxStream) -> Result<(), ConfigError>;

    /// Pulse the selected reset domains: assert the bits, then release them.
    fn reset(&mut self, ptp: bool, tx: bool, rx: bool, eth: bool) -> Result<(), ConfigError>;
}

impl<T: Transport> ControlApi for Device<T> {
    fn list_registers(&mut self) -> Result<Vec<RegisterInfo>, ConfigError> {
        Ok(self
            .map()
            .registers()
            .map(|r| RegisterInfo {
                name: r.name.clone(),
                addr: r.addr,
                size: r.size,
                access: r.access,
            })
            .collect())
    }

    fn read_register(&mut self, name: &str) -> Result<u64, ConfigError> {
        self.read(name)
    }

    fn write_register(&mut self, name: &str, value: u64) -> Result<(), ConfigError> {
        self.write(name, value)
    }

    fn read_addr(&mut self, addr: u32) -> Result<u32, ConfigError> {
        self.peek(addr)
    }

    fn write_addr(&mut self, addr: u32, value: u32) -> Result<(), ConfigError> {
        self.poke(addr, value)
    }

    fn get_mac(&mut self) -> Result<[u8; 6], ConfigError> {
        Aes67Config::get_mac(self)
    }

    fn set_mac(&mut self, mac: [u8; 6]) -> Result<(), ConfigError> {
        Aes67Config::set_mac(self, mac)
    }

    fn get_ip(&mut self) -> Result<Ipv4Addr, ConfigError> {
        Aes67Config::get_ip(self)
    }

    fn set_ip(&mut self, ip: Ipv4Addr) -> Result<(), ConfigError> {
        Aes67Config::set_ip(self, ip)
    }

    fn set_grandmaster(&mut self, gm: PtpGrandmaster) -> Result<(), ConfigError> {
        self.set_ptp_grandmaster(gm)
    }

    fn write_tx_stream(&mut self, stream: &TxStream) -> Result<(), ConfigError> {
        Aes67Streams::write_tx_stream(self, stream)
    }

    fn write_rx_stream(&mut self, stream: &RxStream) -> Result<(), ConfigError> {
        Aes67Streams::write_rx_stream(self, stream)
    }

    fn reset(&mut self, ptp: bool, tx: bool, rx: bool, eth: bool) -> Result<(), ConfigError> {
        let mask = (ptp as u64) | (tx as u64) << 1 | (rx as u64) << 2 | (eth as u64) << 3;
        // Pulse: assert the selected domains, then release all.
        self.write(REG_RESET, mask)?;
        self.write(REG_RESET, 0)?;
        Ok(())
    }
}
