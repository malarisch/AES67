//! Register-level device access: read/write CSRs **by name** over a transport.
//!
//! Wraps a [`Transport`] together with a [`CsrMap`] so callers work with LiteX
//! register names instead of raw addresses. Handles CSRs wider than the 32-bit
//! bus, which LiteX splits across consecutive word addresses, most-significant
//! word first.

use aes67_transport::Transport;

use crate::csr::{CsrMap, Register};
use crate::ConfigError;

/// A configurable AES67 FPGA: a transport bound to its CSR map.
pub struct Device<T: Transport> {
    transport: T,
    map: CsrMap,
    /// Cached `eth_buf` packing probe (see `ethbuf::Device::eth_buf_packed`):
    /// `None` until first queried, then whether the gateware packs 4 payload
    /// bytes per 32-bit buffer word. Constant per bitstream.
    pub(crate) eth_buf_packed: Option<bool>,
}

impl<T: Transport> Device<T> {
    pub fn new(transport: T, map: CsrMap) -> Self {
        Self {
            transport,
            map,
            eth_buf_packed: None,
        }
    }

    /// Borrow the CSR map (e.g. to enumerate registers for `dump`).
    pub fn map(&self) -> &CsrMap {
        &self.map
    }

    fn lookup(&self, name: &str) -> Result<Register, ConfigError> {
        self.map
            .get(name)
            .cloned()
            .ok_or_else(|| ConfigError::UnknownRegister(name.to_string()))
    }

    /// Read a register by name. Multi-word CSRs are assembled MSW-first.
    pub fn read(&mut self, name: &str) -> Result<u64, ConfigError> {
        let reg = self.lookup(name)?;
        self.read_reg(&reg)
    }

    /// Read a register from its map entry (no name lookup).
    pub fn read_reg(&mut self, reg: &Register) -> Result<u64, ConfigError> {
        if reg.size == 0 || reg.size > 2 {
            return Err(ConfigError::Unsupported(format!(
                "register '{}' has unsupported width of {} words",
                reg.name, reg.size
            )));
        }
        let mut value: u64 = 0;
        for i in 0..reg.size {
            let word = self.transport.peek(reg.addr + 4 * i)?;
            value = (value << 32) | word as u64;
        }
        Ok(value)
    }

    /// Write a register by name. Rejects writes to read-only registers and to
    /// values that don't fit the register's width.
    pub fn write(&mut self, name: &str, value: u64) -> Result<(), ConfigError> {
        let reg = self.lookup(name)?;
        if !reg.access.writable() {
            return Err(ConfigError::ReadOnly(name.to_string()));
        }
        if reg.size < 2 && value > u32::MAX as u64 {
            return Err(ConfigError::ValueTooWide {
                name: name.to_string(),
                bits: 32,
            });
        }
        for i in 0..reg.size {
            // MSW first: sub-register i holds bits [32*(size-1-i) .. +32).
            let shift = 32 * (reg.size - 1 - i);
            let word = ((value >> shift) & 0xFFFF_FFFF) as u32;
            self.transport.poke(reg.addr + 4 * i, word)?;
        }
        Ok(())
    }

    /// Read a raw word at an absolute byte address (escape hatch).
    pub fn peek(&mut self, addr: u32) -> Result<u32, ConfigError> {
        Ok(self.transport.peek(addr)?)
    }

    /// Write a raw word at an absolute byte address (escape hatch).
    pub fn poke(&mut self, addr: u32, value: u32) -> Result<(), ConfigError> {
        Ok(self.transport.poke(addr, value)?)
    }

    /// Burst-read `count` consecutive words from byte address `addr`.
    pub fn read_words(&mut self, addr: u32, count: usize) -> Result<Vec<u32>, ConfigError> {
        Ok(self.transport.read_burst(addr, count)?)
    }

    /// Burst-write `values` to consecutive words from byte address `addr`.
    pub fn write_words(&mut self, addr: u32, values: &[u32]) -> Result<(), ConfigError> {
        Ok(self.transport.write_burst(addr, values)?)
    }
}
