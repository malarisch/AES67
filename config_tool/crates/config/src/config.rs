//! High-level AES67 configuration helpers.
//!
//! These build on [`Device`]'s by-name register access to offer semantic
//! operations (set the MAC address, the IP, PTP grandmaster parameters, …) with
//! the exact bit/byte layout the FPGA expects. The byte conventions mirror the
//! firmware driver (`soc_firmware/app/drivers/eth_litex/eth_litex.c`) so the
//! tool and the on-target firmware agree.
//!
//! The trait is implemented for any [`Device`], so callers do:
//! ```ignore
//! use aes67_config::Aes67Config;
//! device.set_mac([0x02, 0x00, 0x00, 0x12, 0x34, 0x56])?;
//! ```

use std::net::Ipv4Addr;

use aes67_transport::Transport;

use crate::device::Device;
use crate::ConfigError;

// Register names in the LiteX CSR map (resolved to addresses dynamically).
pub(crate) const REG_MAC_LO: &str = "aes67_csr_mac_addr_lo";
pub(crate) const REG_MAC_HI: &str = "aes67_csr_mac_addr_hi";
pub(crate) const REG_IP: &str = "aes67_csr_ip_addr";
const REG_SCRATCH: &str = "aes67_csr_scratch";
const REG_STATUS: &str = "aes67_csr_status";
const REG_CTRL: &str = "aes67_csr_ctrl";
pub(crate) const REG_GM_PRIORITY1: &str = "aes67_csr_ptp_gm_priority1";
pub(crate) const REG_GM_PRIORITY2: &str = "aes67_csr_ptp_gm_priority2";
pub(crate) const REG_GM_CLOCK_CLASS: &str = "aes67_csr_ptp_gm_clock_class";
pub(crate) const REG_GM_CLOCK_ACCURACY: &str = "aes67_csr_ptp_gm_clock_accuracy";

/// Semantic configuration operations layered over a [`Device`].
pub trait Aes67Config {
    /// Program the 48-bit MAC address (`mac[0]` is the most significant octet).
    fn set_mac(&mut self, mac: [u8; 6]) -> Result<(), ConfigError>;
    /// Read back the 48-bit MAC address.
    fn get_mac(&mut self) -> Result<[u8; 6], ConfigError>;

    /// Program the IPv4 address.
    fn set_ip(&mut self, ip: Ipv4Addr) -> Result<(), ConfigError>;
    /// Read back the IPv4 address.
    fn get_ip(&mut self) -> Result<Ipv4Addr, ConfigError>;

    /// Write the scratch register (handy round-trip link test).
    fn set_scratch(&mut self, value: u32) -> Result<(), ConfigError>;
    /// Read the scratch register.
    fn get_scratch(&mut self) -> Result<u32, ConfigError>;

    /// Read the raw status word.
    fn get_status(&mut self) -> Result<u32, ConfigError>;

    /// Write the control register.
    fn set_ctrl(&mut self, value: u32) -> Result<(), ConfigError>;

    /// Set PTP grandmaster announce parameters in one call. `None` leaves a
    /// field untouched.
    fn set_ptp_grandmaster(&mut self, gm: PtpGrandmaster) -> Result<(), ConfigError>;
}

/// PTP grandmaster announce parameters. Each `None` field is left unchanged.
#[derive(Debug, Default, Clone, Copy)]
pub struct PtpGrandmaster {
    pub priority1: Option<u8>,
    pub priority2: Option<u8>,
    pub clock_class: Option<u8>,
    pub clock_accuracy: Option<u8>,
}

impl<T: Transport> Aes67Config for Device<T> {
    fn set_mac(&mut self, mac: [u8; 6]) -> Result<(), ConfigError> {
        // mac[0] → bits 47..40 … mac[5] → bits 7..0.
        let hi = (u32::from(mac[0]) << 8) | u32::from(mac[1]);
        let lo = (u32::from(mac[2]) << 24)
            | (u32::from(mac[3]) << 16)
            | (u32::from(mac[4]) << 8)
            | u32::from(mac[5]);
        self.write(REG_MAC_LO, lo as u64)?;
        self.write(REG_MAC_HI, hi as u64)?;
        Ok(())
    }

    fn get_mac(&mut self) -> Result<[u8; 6], ConfigError> {
        let lo = self.read(REG_MAC_LO)? as u32;
        let hi = self.read(REG_MAC_HI)? as u32;
        Ok([
            (hi >> 8) as u8,
            hi as u8,
            (lo >> 24) as u8,
            (lo >> 16) as u8,
            (lo >> 8) as u8,
            lo as u8,
        ])
    }

    fn set_ip(&mut self, ip: Ipv4Addr) -> Result<(), ConfigError> {
        // ip_addr(31..24) = first octet.
        let o = ip.octets();
        let val = (u32::from(o[0]) << 24)
            | (u32::from(o[1]) << 16)
            | (u32::from(o[2]) << 8)
            | u32::from(o[3]);
        self.write(REG_IP, val as u64)
    }

    fn get_ip(&mut self) -> Result<Ipv4Addr, ConfigError> {
        let v = self.read(REG_IP)? as u32;
        Ok(Ipv4Addr::new(
            (v >> 24) as u8,
            (v >> 16) as u8,
            (v >> 8) as u8,
            v as u8,
        ))
    }

    fn set_scratch(&mut self, value: u32) -> Result<(), ConfigError> {
        self.write(REG_SCRATCH, value as u64)
    }

    fn get_scratch(&mut self) -> Result<u32, ConfigError> {
        Ok(self.read(REG_SCRATCH)? as u32)
    }

    fn get_status(&mut self) -> Result<u32, ConfigError> {
        Ok(self.read(REG_STATUS)? as u32)
    }

    fn set_ctrl(&mut self, value: u32) -> Result<(), ConfigError> {
        self.write(REG_CTRL, value as u64)
    }

    fn set_ptp_grandmaster(&mut self, gm: PtpGrandmaster) -> Result<(), ConfigError> {
        if let Some(v) = gm.priority1 {
            self.write(REG_GM_PRIORITY1, v as u64)?;
        }
        if let Some(v) = gm.priority2 {
            self.write(REG_GM_PRIORITY2, v as u64)?;
        }
        if let Some(v) = gm.clock_class {
            self.write(REG_GM_CLOCK_CLASS, v as u64)?;
        }
        if let Some(v) = gm.clock_accuracy {
            self.write(REG_GM_CLOCK_ACCURACY, v as u64)?;
        }
        Ok(())
    }
}
