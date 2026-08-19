//! `eth_buf` packet-buffer bridge: move Ethernet frames between the FPGA's
//! `eth_buf` and the host, over the Wishbone transport.
//!
//! This is the FPGA-protocol half of the daemon's TAP network bridge. It mirrors
//! the on-target firmware (`drivers/eth_litex/eth_litex.c`):
//!
//! * **RX** (FPGA → host): when `rx_ready` is set, read `rx_len` (which includes
//!   the 4-byte FCS), burst-read `rx_len-4` bytes from the RX region, pulse
//!   `rx_ack` (1 then 0) to release the buffer, and clear the `ev_pending`
//!   RX-ready event (which de-asserts the IRQ line).
//! * **TX** (host → FPGA): burst-write the frame bytes to the TX region
//!   (region base + 0x2000) and set `tx_len`; the MAC appends the FCS.
//!
//! The buffer word packing depends on the loaded gateware, probed once via the
//! `eth_buf_bytes_per_word` CSR: packed builds carry 4 payload bytes per
//! 32-bit word (little-endian lanes, frame byte 4i+k in word i bits
//! [8k+7:8k]); legacy builds carry 1 byte per word (low byte), and do not
//! have the CSR at all.
//!
//! Register/region names come from the CSR map, so nothing is hard-coded.

use aes67_transport::Transport;

use crate::device::Device;
use crate::ConfigError;

const REGION: &str = "eth_buf";
const TX_OFFSET: u32 = 0x2000; // TX buffer sits 0x2000 bytes into the region.

const REG_RX_LEN: &str = "eth_buf_rx_len";
const REG_RX_READY: &str = "eth_buf_rx_ready";
const REG_RX_ACK: &str = "eth_buf_rx_ack";
const REG_TX_LEN: &str = "eth_buf_tx_len";
const REG_EV_ENABLE: &str = "eth_buf_ev_enable";
const REG_EV_PENDING: &str = "eth_buf_ev_pending";
const REG_BYTES_PER_WORD: &str = "eth_buf_bytes_per_word";

// The transmit trigger and completion live in the AES67 control/status CSRs, not
// in eth_buf: writing tx_len only stages the frame — the MAC transmits when
// `eth_tx_request` (ctrl bit 3) is pulsed, and reports completion via
// `eth_tx_done` (status bit 7). Mirrors the firmware TX worker.
const REG_CTRL: &str = "aes67_csr_ctrl";
const REG_STATUS: &str = "aes67_csr_status";
const CTRL_ETH_TX_REQUEST: u64 = 1 << 3;
const STATUS_ETH_TX_DONE: u64 = 1 << 7;
/// Bounded poll for tx-done (each read is a real bus transaction, far longer
/// than a frame's wire time, so this resolves in one or two iterations).
const TX_DONE_POLL_LIMIT: u32 = 64;

/// EventManager bit for the single "RX packet received" source.
const EV_RX_READY: u64 = 0x1;
/// The MAC includes the FCS in the received length; strip it for the stack.
const FCS_LEN: u16 = 4;
/// Sanity bounds for a received frame (without FCS).
const MIN_FRAME: usize = 14; // Ethernet header
const MAX_FRAME: usize = 1518;

/// `eth_buf` packet-buffer operations layered over a [`Device`].
pub trait EthBufBridge {
    /// Enable the RX-ready interrupt event (drives the IRQ line).
    fn eth_irq_enable(&mut self) -> Result<(), ConfigError>;
    /// Clear the pending RX-ready event (de-asserts the IRQ line).
    fn eth_irq_clear(&mut self) -> Result<(), ConfigError>;
    /// Is an RX frame waiting?
    fn eth_rx_ready(&mut self) -> Result<bool, ConfigError>;
    /// Consume one RX frame (precondition: [`eth_rx_ready`] was true). Reads the
    /// frame, acks the buffer, and clears the event. Returns the frame (FCS
    /// stripped), or `None` if the length was invalid (frame dropped, buffer
    /// still released).
    fn eth_rx_take_one(&mut self) -> Result<Option<Vec<u8>>, ConfigError>;
    /// Transmit one frame (without FCS; the MAC appends it). Stages the frame in
    /// the TX buffer, sets `tx_len`, then pulses `eth_tx_request` — the actual
    /// transmit trigger. With `wait_done`, first waits for the previous TX to
    /// finish (`eth_tx_done`) so the buffer is not overwritten mid-transmit;
    /// pass `false` only for the very first frame after reset.
    fn eth_tx(&mut self, frame: &[u8], wait_done: bool) -> Result<(), ConfigError>;
}

impl<T: Transport> Device<T> {
    /// Does the loaded gateware pack 4 payload bytes per buffer word?
    /// Probed once (a legacy bitstream has no `eth_buf_bytes_per_word` CSR,
    /// and a stale bitstream under a new map reads junk there — only the
    /// exact value 4 selects the packed layout), then cached.
    fn eth_buf_packed(&mut self) -> Result<bool, ConfigError> {
        if let Some(packed) = self.eth_buf_packed {
            return Ok(packed);
        }
        let packed = self.map().get(REG_BYTES_PER_WORD).is_some()
            && self.read(REG_BYTES_PER_WORD)? == 4;
        self.eth_buf_packed = Some(packed);
        Ok(packed)
    }

    /// `(rx_base, tx_base)` byte addresses from the CSR map's `eth_buf` region.
    fn eth_region(&self) -> Result<(u32, u32), ConfigError> {
        let (base, _size) = self.map().region(REGION).ok_or_else(|| {
            ConfigError::Unsupported(format!(
                "memory region '{REGION}' not in CSR map — wrong build?"
            ))
        })?;
        Ok((base, base + TX_OFFSET))
    }

    fn eth_rx_ack_pulse(&mut self) -> Result<(), ConfigError> {
        self.write(REG_RX_ACK, 1)?;
        self.write(REG_RX_ACK, 0)
    }

    /// Poll `eth_tx_done` until set (bounded); returns even on timeout, matching
    /// the firmware which warns and proceeds.
    fn eth_wait_tx_done(&mut self) -> Result<(), ConfigError> {
        for _ in 0..TX_DONE_POLL_LIMIT {
            if self.read(REG_STATUS)? & STATUS_ETH_TX_DONE != 0 {
                break;
            }
        }
        Ok(())
    }

    /// Pulse `eth_tx_request` (ctrl bit 3) high then low to trigger the MAC,
    /// preserving the other control bits (e.g. `adda_nrst`). The bus latency
    /// between the two writes (one full transaction each) comfortably exceeds
    /// the gateware's CDC capture window.
    fn eth_tx_pulse_request(&mut self) -> Result<(), ConfigError> {
        let ctrl = self.read(REG_CTRL)?;
        self.write(REG_CTRL, ctrl | CTRL_ETH_TX_REQUEST)?;
        self.write(REG_CTRL, ctrl & !CTRL_ETH_TX_REQUEST)
    }
}

impl<T: Transport> EthBufBridge for Device<T> {
    fn eth_irq_enable(&mut self) -> Result<(), ConfigError> {
        self.write(REG_EV_ENABLE, EV_RX_READY)
    }

    fn eth_irq_clear(&mut self) -> Result<(), ConfigError> {
        self.write(REG_EV_PENDING, EV_RX_READY)
    }

    fn eth_rx_ready(&mut self) -> Result<bool, ConfigError> {
        Ok(self.read(REG_RX_READY)? & 1 != 0)
    }

    fn eth_rx_take_one(&mut self) -> Result<Option<Vec<u8>>, ConfigError> {
        let raw_len = self.read(REG_RX_LEN)? as u16;
        let pkt_len = raw_len.saturating_sub(FCS_LEN) as usize;

        let frame = if (MIN_FRAME..=MAX_FRAME).contains(&pkt_len) {
            let (rx_base, _) = self.eth_region()?;
            let bytes = if self.eth_buf_packed()? {
                // 4 bytes per word, little-endian lanes; the surplus lanes
                // of the final word are cut off by truncate().
                let words = self.read_words(rx_base, pkt_len.div_ceil(4))?;
                let mut bytes: Vec<u8> = words
                    .iter()
                    .flat_map(|w| w.to_le_bytes())
                    .collect();
                bytes.truncate(pkt_len);
                bytes
            } else {
                // Legacy layout: one payload byte in each word's low byte.
                let words = self.read_words(rx_base, pkt_len)?;
                words.iter().map(|w| *w as u8).collect()
            };
            Some(bytes)
        } else {
            None // invalid length: drop, but still release the buffer below
        };

        // Release the buffer and clear the event regardless, so the FPGA can
        // accept the next frame and the IRQ line de-asserts.
        self.eth_rx_ack_pulse()?;
        self.eth_irq_clear()?;
        Ok(frame)
    }

    fn eth_tx(&mut self, frame: &[u8], wait_done: bool) -> Result<(), ConfigError> {
        if frame.len() > MAX_FRAME {
            return Err(ConfigError::Unsupported(format!(
                "TX frame too long: {} bytes (max {MAX_FRAME})",
                frame.len()
            )));
        }
        // Don't clobber a frame the MAC is still reading.
        if wait_done {
            self.eth_wait_tx_done()?;
        }
        let (_, tx_base) = self.eth_region()?;
        let words: Vec<u32> = if self.eth_buf_packed()? {
            // 4 bytes per word, little-endian lanes; zero-pad the final
            // partial word (tx_len bounds the frame, the pad is never sent).
            frame
                .chunks(4)
                .map(|c| {
                    let mut lanes = [0u8; 4];
                    lanes[..c.len()].copy_from_slice(c);
                    u32::from_le_bytes(lanes)
                })
                .collect()
        } else {
            // Legacy layout: one payload byte per word.
            frame.iter().map(|&b| b as u32).collect()
        };
        self.write_words(tx_base, &words)?;
        self.write(REG_TX_LEN, frame.len() as u64)?;
        // The real trigger: pulse eth_tx_request.
        self.eth_tx_pulse_request()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csr::CsrMap;
    use std::collections::HashMap;

    #[derive(Default)]
    struct MockBus(HashMap<u32, u32>);
    impl Transport for MockBus {
        fn peek(&mut self, a: u32) -> Result<u32, aes67_transport::TransportError> {
            Ok(*self.0.get(&a).unwrap_or(&0))
        }
        fn poke(&mut self, a: u32, v: u32) -> Result<(), aes67_transport::TransportError> {
            self.0.insert(a, v);
            Ok(())
        }
    }

    fn map() -> CsrMap {
        // Registers + the eth_buf region (RX base 0x90000000, TX base +0x2000).
        CsrMap::from_csv(
            "\
csr_register,eth_buf_rx_len,0x9001100c,1,ro
csr_register,eth_buf_rx_ready,0x90011010,1,ro
csr_register,eth_buf_rx_ack,0x90011014,1,rw
csr_register,eth_buf_tx_len,0x90011018,1,rw
csr_register,eth_buf_ev_enable,0x90011008,1,rw
csr_register,eth_buf_ev_pending,0x90011004,1,rw
csr_register,aes67_csr_ctrl,0x90010018,1,rw
csr_register,aes67_csr_status,0x9001000c,1,ro
memory_region,eth_buf,0x90000000,16384,io
",
        )
        .unwrap()
    }

    /// Helper to write a register's value into the mock by name.
    fn seed(dev: &mut Device<MockBus>, name: &str, val: u32) {
        let addr = dev.map().get(name).unwrap().addr;
        dev.poke(addr, val).unwrap();
    }

    #[test]
    fn rx_take_reads_frame_and_acks() {
        let mut dev = Device::new(MockBus::default(), map());
        // 14-byte frame + 4-byte FCS = rx_len 18. Seed the RX buffer words.
        let frame: Vec<u8> = (0..14).collect();
        for (i, b) in frame.iter().enumerate() {
            dev.poke(0x9000_0000 + 4 * i as u32, *b as u32).unwrap();
        }
        seed(&mut dev, "eth_buf_rx_ready", 1);
        seed(&mut dev, "eth_buf_rx_len", 18);

        assert!(dev.eth_rx_ready().unwrap());
        let got = dev.eth_rx_take_one().unwrap();
        assert_eq!(got, Some(frame));

        // ack pulse left rx_ack at 0; pending was cleared (written EV_RX_READY).
        let ack = dev.map().get("eth_buf_rx_ack").unwrap().addr;
        let pend = dev.map().get("eth_buf_ev_pending").unwrap().addr;
        assert_eq!(dev.peek(ack).unwrap(), 0);
        assert_eq!(dev.peek(pend).unwrap(), EV_RX_READY as u32);
    }

    #[test]
    fn rx_take_drops_invalid_length_but_still_acks() {
        let mut dev = Device::new(MockBus::default(), map());
        seed(&mut dev, "eth_buf_rx_ready", 1);
        seed(&mut dev, "eth_buf_rx_len", 5); // 5-4=1 < MIN_FRAME → dropped
        assert_eq!(dev.eth_rx_take_one().unwrap(), None);
        let pend = dev.map().get("eth_buf_ev_pending").unwrap().addr;
        assert_eq!(dev.peek(pend).unwrap(), EV_RX_READY as u32);
    }

    /// Same map, but for packed gateware (4 bytes per word) — advertised by
    /// the eth_buf_bytes_per_word CSR, which legacy maps do not have.
    fn map_packed() -> CsrMap {
        CsrMap::from_csv(
            "\
csr_register,eth_buf_rx_len,0x9001100c,1,ro
csr_register,eth_buf_rx_ready,0x90011010,1,ro
csr_register,eth_buf_rx_ack,0x90011014,1,rw
csr_register,eth_buf_tx_len,0x90011018,1,rw
csr_register,eth_buf_ev_enable,0x90011008,1,rw
csr_register,eth_buf_ev_pending,0x90011004,1,rw
csr_register,eth_buf_bytes_per_word,0x9001101c,1,ro
csr_register,aes67_csr_ctrl,0x90010018,1,rw
csr_register,aes67_csr_status,0x9001000c,1,ro
memory_region,eth_buf,0x90000000,16384,io
",
        )
        .unwrap()
    }

    #[test]
    fn rx_take_packed_unpacks_le_lanes() {
        let mut dev = Device::new(MockBus::default(), map_packed());
        seed(&mut dev, "eth_buf_bytes_per_word", 4);
        // 14-byte frame + 4-byte FCS = rx_len 18 → ceil(14/4) = 4 words.
        let frame: Vec<u8> = (0..14).collect();
        for (i, c) in frame.chunks(4).enumerate() {
            let mut lanes = [0u8; 4];
            lanes[..c.len()].copy_from_slice(c);
            dev.poke(0x9000_0000 + 4 * i as u32, u32::from_le_bytes(lanes))
                .unwrap();
        }
        seed(&mut dev, "eth_buf_rx_ready", 1);
        seed(&mut dev, "eth_buf_rx_len", 18);

        assert_eq!(dev.eth_rx_take_one().unwrap(), Some(frame));
    }

    #[test]
    fn tx_packed_packs_le_lanes_and_pads() {
        let mut dev = Device::new(MockBus::default(), map_packed());
        seed(&mut dev, "eth_buf_bytes_per_word", 4);
        let frame: Vec<u8> = vec![0x11, 0x22, 0x33, 0x44, 0x55];
        dev.eth_tx(&frame, false).unwrap();
        assert_eq!(dev.peek(0x9000_2000).unwrap(), 0x4433_2211);
        assert_eq!(dev.peek(0x9000_2004).unwrap(), 0x0000_0055);
        let tx_len = dev.map().get("eth_buf_tx_len").unwrap().addr;
        assert_eq!(dev.peek(tx_len).unwrap(), 5);
    }

    #[test]
    fn stale_bitstream_value_falls_back_to_legacy() {
        // Map advertises the CSR (new csr.csv) but the device answers junk
        // there (old bitstream): must use the legacy one-byte-per-word path.
        let mut dev = Device::new(MockBus::default(), map_packed());
        seed(&mut dev, "eth_buf_bytes_per_word", 0);
        let frame: Vec<u8> = vec![0xaa, 0xbb];
        // 14-byte minimum doesn't apply to TX; write and check legacy layout.
        dev.eth_tx(&frame, false).unwrap();
        assert_eq!(dev.peek(0x9000_2000).unwrap(), 0xaa);
        assert_eq!(dev.peek(0x9000_2004).unwrap(), 0xbb);
    }

    #[test]
    fn tx_writes_frame_and_len() {
        let mut dev = Device::new(MockBus::default(), map());
        let frame: Vec<u8> = vec![0xaa, 0xbb, 0xcc, 0xdd];
        dev.eth_tx(&frame, false).unwrap();
        // TX region base + 0x2000 = 0x90002000.
        for (i, b) in frame.iter().enumerate() {
            assert_eq!(dev.peek(0x9000_2000 + 4 * i as u32).unwrap(), *b as u32);
        }
        let tx_len = dev.map().get("eth_buf_tx_len").unwrap().addr;
        assert_eq!(dev.peek(tx_len).unwrap(), frame.len() as u32);
        // eth_tx_request (ctrl bit 3) was pulsed and left low again.
        let ctrl = dev.map().get("aes67_csr_ctrl").unwrap().addr;
        assert_eq!(dev.peek(ctrl).unwrap() & (1 << 3), 0);
    }
}
