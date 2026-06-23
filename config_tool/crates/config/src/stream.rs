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

/// Bytes per stream slot in either config RAM (`stream_id * SLOT_BYTES`).
const SLOT_BYTES: usize = 32;

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
    /// Session name announced over SAP/SDP. Metadata only — it is not part of the
    /// FPGA RAM layout (see [`encode`](TxStream::encode)); `None` ⇒ daemon default.
    pub name: Option<String>,
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
    /// Human-readable stream name (from the SAP/SDP `s=` line when subscribed via
    /// discovery). Metadata only — it is not part of the FPGA RAM layout (see
    /// [`encode`](RxStream::encode)); `None` for a manually configured stream.
    pub name: Option<String>,
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
    /// Tear down a transmit stream: zero its config slot so the gateware stops
    /// sending it (`samples_per_packet` = 0 marks the slot inactive).
    fn clear_tx_stream(&mut self, id: u8) -> Result<(), ConfigError>;
    /// Tear down a receive stream: zero its config slot so no incoming packet
    /// matches (cleared destination IP/port). The caller is responsible for
    /// dropping the corresponding IGMP multicast membership.
    fn clear_rx_stream(&mut self, id: u8) -> Result<(), ConfigError>;
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

    fn clear_tx_stream(&mut self, id: u8) -> Result<(), ConfigError> {
        clear_stream_ram(self, REGION_TX, id)
    }

    fn clear_rx_stream(&mut self, id: u8) -> Result<(), ConfigError> {
        clear_stream_ram(self, REGION_RX, id)
    }
}

/// Zero a stream's whole 32-byte config slot, disabling it. For TX this clears
/// `samples_per_packet` (offset 6), which the gateware uses as the active flag;
/// for RX it clears the destination IP/port, so no incoming packet matches.
fn clear_stream_ram<T: Transport>(
    dev: &mut Device<T>,
    region: &str,
    stream_id: u8,
) -> Result<(), ConfigError> {
    if stream_id >= MAX_STREAMS {
        return Err(ConfigError::Unsupported(format!(
            "stream id {stream_id} out of range (0..{})",
            MAX_STREAMS - 1
        )));
    }
    write_stream_ram(dev, region, stream_id, &[0u8; SLOT_BYTES])
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
            name: None,
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
            name: None,
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
            name: None,
        };
        assert!(s.encode().is_err());
    }

    /// Minimal in-test Wishbone: a flat addr→word map (the shared `MockTransport`
    /// lives behind a feature this crate does not enable).
    #[derive(Default)]
    struct MapTransport(std::collections::HashMap<u32, u32>);
    impl aes67_transport::Transport for MapTransport {
        fn peek(&mut self, addr: u32) -> Result<u32, aes67_transport::TransportError> {
            Ok(*self.0.get(&addr).unwrap_or(&0))
        }
        fn poke(&mut self, addr: u32, value: u32) -> Result<(), aes67_transport::TransportError> {
            self.0.insert(addr, value);
            Ok(())
        }
    }

    #[test]
    fn clear_zeros_the_slot() {
        use crate::device::Device;

        let map = crate::csr::CsrMap::from_csv(
            "csr_register,aes67_csr_scratch,0x0,1,rw\n\
             memory_region,tx_stream_cfg,0x1000,256,linker\n\
             memory_region,rx_stream_cfg,0x2000,256,linker\n",
        )
        .unwrap();
        let mut dev = Device::new(MapTransport::default(), map);

        // Configure TX slot 2, then clear it; the active byte (offset 6,
        // samples_per_packet) and the destination IP must read back as zero.
        dev.write_tx_stream(&TxStream {
            id: 2,
            dst_ip: Ipv4Addr::new(239, 69, 1, 1),
            channels: Some(2),
            samples_per_packet: 48,
            ch_ids: vec![0, 1],
            ssrc: 0xdead_beef,
            name: None,
        })
        .unwrap();
        dev.clear_tx_stream(2).unwrap();

        let (base, _) = dev.map().region("tx_stream_cfg").unwrap();
        let slot = 2u32 * 32;
        for off in 0..SLOT_BYTES as u32 {
            assert_eq!(dev.peek(base + ((slot + off) << 2)).unwrap(), 0);
        }

        // Out-of-range id is rejected.
        assert!(dev.clear_rx_stream(MAX_STREAMS).is_err());
    }
}
