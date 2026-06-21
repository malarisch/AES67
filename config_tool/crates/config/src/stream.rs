//! RX / TX audio stream configuration.
//!
//! Stream parameters do not live in CSRs but in two write-only configuration
//! RAMs exposed as memory regions: `tx_stream_cfg` and `rx_stream_cfg`. Each
//! RAM holds up to 8 streams of 32 bytes (`stream_id * 32`); a 32-bit word write
//! delivers one byte (bits 7:0) to RAM address `byte_offset >> 2`.
//!
//! The byte layout mirrors the on-target firmware
//! (`soc_firmware/app/drivers/eth_litex/eth_litex.c`):
//!
//! TX stream (20 bytes):
//! ```text
//!   0      stream_id (0..7)
//!   1..4   destination IP (a.b.c.d)
//!   5      channel count
//!   6      samples per packet
//!   7..14  channel IDs (up to 8)
//!   15     reserved
//!   16..19 SSRC (big-endian)
//! ```
//!
//! RX stream (17 bytes):
//! ```text
//!   0..3   destination IP (a.b.c.d)
//!   4..5   destination UDP port (big-endian)
//!   6..13  output channel map (up to 8)
//!   14     channel count
//!   15     output delay (samples)
//!   16     samples per channel per packet
//! ```
//!
//! The RAMs are write-only, so there is no stream read-back / export.

use std::net::Ipv4Addr;

use aes67_transport::Transport;

use crate::device::Device;
use crate::ConfigError;

/// Region names in the LiteX CSR map.
const REGION_TX: &str = "tx_stream_cfg";
const REGION_RX: &str = "rx_stream_cfg";

/// Maximum number of streams per direction and channels per stream.
pub const MAX_STREAMS: u8 = 8;
pub const MAX_CHANNELS: usize = 8;

/// A transmit (sender) audio stream.
#[derive(Debug, Clone)]
pub struct TxStream {
    /// Stream slot (0..7).
    pub id: u8,
    /// Destination (multicast) IPv4 address.
    pub dst_ip: Ipv4Addr,
    /// Channel count. Defaults to `ch_ids.len()` when omitted/zero.
    pub channels: Option<u8>,
    /// Samples per RTP packet per channel.
    pub samples_per_packet: u8,
    /// Source channel IDs routed into this stream (up to 8).
    pub ch_ids: Vec<u8>,
    /// RTP SSRC (must match the SDP announcement).
    pub ssrc: u32,
}

/// A receive audio stream.
#[derive(Debug, Clone)]
pub struct RxStream {
    /// Stream slot (0..7).
    pub id: u8,
    /// Destination (multicast) IPv4 address to subscribe to.
    pub dst_ip: Ipv4Addr,
    /// Destination UDP port.
    pub dst_port: u16,
    /// Output channel map: output channel for each input channel (up to 8).
    pub ch_map: Vec<u8>,
    /// Channel count. Defaults to `ch_map.len()` when omitted/zero.
    pub channels: Option<u8>,
    /// Output delay in samples.
    pub output_delay: u8,
    /// Samples per channel per packet.
    pub samples_per_channel: u8,
}

impl TxStream {
    /// Encode to the 20-byte RAM image, validating ranges.
    fn encode(&self) -> Result<[u8; 20], ConfigError> {
        if self.id >= MAX_STREAMS {
            return Err(ConfigError::Unsupported(format!(
                "tx stream id {} out of range (0..{})",
                self.id,
                MAX_STREAMS - 1
            )));
        }
        if self.ch_ids.len() > MAX_CHANNELS {
            return Err(ConfigError::Unsupported(format!(
                "tx stream {} has {} channel IDs (max {MAX_CHANNELS})",
                self.id,
                self.ch_ids.len()
            )));
        }
        let channels = self
            .channels
            .filter(|&c| c != 0)
            .unwrap_or(self.ch_ids.len() as u8);

        let mut buf = [0u8; 20];
        buf[0] = self.id & 0x07;
        buf[1..5].copy_from_slice(&self.dst_ip.octets());
        buf[5] = channels;
        buf[6] = self.samples_per_packet;
        for (i, &ch) in self.ch_ids.iter().enumerate() {
            buf[7 + i] = ch;
        }
        // byte 15 reserved
        buf[16..20].copy_from_slice(&self.ssrc.to_be_bytes());
        Ok(buf)
    }
}

impl RxStream {
    /// Encode to the 17-byte RAM image, validating ranges.
    fn encode(&self) -> Result<[u8; 17], ConfigError> {
        if self.id >= MAX_STREAMS {
            return Err(ConfigError::Unsupported(format!(
                "rx stream id {} out of range (0..{})",
                self.id,
                MAX_STREAMS - 1
            )));
        }
        if self.ch_map.len() > MAX_CHANNELS {
            return Err(ConfigError::Unsupported(format!(
                "rx stream {} has {} channel map entries (max {MAX_CHANNELS})",
                self.id,
                self.ch_map.len()
            )));
        }
        let channels = self
            .channels
            .filter(|&c| c != 0)
            .unwrap_or(self.ch_map.len() as u8);

        let mut buf = [0u8; 17];
        buf[0..4].copy_from_slice(&self.dst_ip.octets());
        buf[4..6].copy_from_slice(&self.dst_port.to_be_bytes());
        for (i, &ch) in self.ch_map.iter().enumerate() {
            buf[6 + i] = ch;
        }
        buf[14] = channels;
        buf[15] = self.output_delay;
        buf[16] = self.samples_per_channel;
        Ok(buf)
    }
}

/// Stream configuration operations layered over a [`Device`].
pub trait Aes67Streams {
    /// Configure a transmit stream.
    fn write_tx_stream(&mut self, stream: &TxStream) -> Result<(), ConfigError>;
    /// Configure a receive stream.
    fn write_rx_stream(&mut self, stream: &RxStream) -> Result<(), ConfigError>;
}

impl<T: Transport> Aes67Streams for Device<T> {
    fn write_tx_stream(&mut self, stream: &TxStream) -> Result<(), ConfigError> {
        let buf = stream.encode()?;
        write_stream_ram(self, REGION_TX, stream.id, &buf)
    }

    fn write_rx_stream(&mut self, stream: &RxStream) -> Result<(), ConfigError> {
        let buf = stream.encode()?;
        write_stream_ram(self, REGION_RX, stream.id, &buf)
    }
}

/// Write `buf` into a stream config RAM at slot `stream_id`: one byte per 32-bit
/// word, RAM address `stream_id*32 + i`, byte address `base + (ram_addr << 2)`.
fn write_stream_ram<T: Transport>(
    dev: &mut Device<T>,
    region: &str,
    stream_id: u8,
    buf: &[u8],
) -> Result<(), ConfigError> {
    let (base, _size) = dev.map().region(region).ok_or_else(|| {
        ConfigError::Unsupported(format!(
            "memory region '{region}' not in CSR map — wrong build? \
             (need the aes67_bridge map with stream config RAMs)"
        ))
    })?;
    let base_addr = stream_id as u32 * 32;
    for (i, &b) in buf.iter().enumerate() {
        let ram_addr = base_addr + i as u32;
        dev.poke(base + (ram_addr << 2), b as u32)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tx_encoding_matches_firmware_layout() {
        let s = TxStream {
            id: 3,
            dst_ip: Ipv4Addr::new(239, 69, 1, 2),
            channels: None,
            samples_per_packet: 48,
            ch_ids: vec![0, 1],
            ssrc: 0x1234_5678,
        };
        let b = s.encode().unwrap();
        assert_eq!(b[0], 3);
        assert_eq!(&b[1..5], &[239, 69, 1, 2]);
        assert_eq!(b[5], 2); // derived from ch_ids.len()
        assert_eq!(b[6], 48);
        assert_eq!(&b[7..9], &[0, 1]);
        assert_eq!(&b[16..20], &[0x12, 0x34, 0x56, 0x78]);
    }

    #[test]
    fn rx_encoding_matches_firmware_layout() {
        let s = RxStream {
            id: 0,
            dst_ip: Ipv4Addr::new(239, 69, 2, 1),
            dst_port: 5004,
            ch_map: vec![0, 1, 2],
            channels: Some(3),
            output_delay: 16,
            samples_per_channel: 48,
        };
        let b = s.encode().unwrap();
        assert_eq!(&b[0..4], &[239, 69, 2, 1]);
        assert_eq!(&b[4..6], &[0x13, 0x8c]); // 5004 big-endian
        assert_eq!(&b[6..9], &[0, 1, 2]);
        assert_eq!(b[14], 3);
        assert_eq!(b[15], 16);
        assert_eq!(b[16], 48);
    }

    #[test]
    fn rejects_out_of_range() {
        let s = TxStream {
            id: 9,
            dst_ip: Ipv4Addr::UNSPECIFIED,
            channels: None,
            samples_per_packet: 0,
            ch_ids: vec![],
            ssrc: 0,
        };
        assert!(s.encode().is_err());
    }
}
