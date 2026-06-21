//! UART backend — the LiteX `uartbone` UART→Wishbone bridge.
//!
//! Wire protocol (see `litex/tools/remote/comm_uart.py`):
//!   write: `[0x01, len]`, then `len` not used beyond 1 here; `addr>>2` as 4
//!          big-endian bytes, then the 32-bit value big-endian.
//!   read:  `[0x02, len]`, then `addr>>2` big-endian; device returns `4*len`
//!          big-endian data bytes.
//!
//! uartbone consumes a **word** address, hence the `addr >> 2`.

use std::io::{Read, Write};
use std::time::Duration;

use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits};

use crate::{Transport, TransportError};

const CMD_WRITE: u8 = 0x01; // CMD_WRITE_BURST_INCR
const CMD_READ: u8 = 0x02; //  CMD_READ_BURST_INCR

/// uartbone carries the burst length in a single byte, so a burst is at most
/// 255 words; longer transfers are chunked.
const MAX_BURST: usize = 255;

/// Default baud rate; matches the `uartbone` gateware (115200 8N1).
pub const DEFAULT_BAUD_RATE: u32 = 115_200;

/// A connection to the FPGA over the `uartbone` serial bridge.
pub struct UartTransport {
    port: Box<dyn SerialPort>,
}

impl UartTransport {
    /// Open `path` (e.g. `/dev/ttyUSB0`) at `baud` baud, 8N1, no flow control.
    pub fn open(path: &str, baud: u32) -> Result<Self, TransportError> {
        let port = serialport::new(path, baud)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .timeout(Duration::from_millis(1000))
            .open()
            .map_err(|e| TransportError::Serial(format!("opening {path}: {e}")))?;
        Ok(Self { port })
    }

    /// Drop any stale bytes the device may have left in the input buffer so a
    /// fresh read sees only this transaction's response.
    fn flush_input(&mut self) -> Result<(), TransportError> {
        self.port
            .clear(ClearBuffer::Input)
            .map_err(|e| TransportError::Serial(e.to_string()))
    }

    fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), TransportError> {
        let mut filled = 0;
        while filled < buf.len() {
            match self.port.read(&mut buf[filled..]) {
                Ok(0) => return Err(TransportError::Timeout),
                Ok(n) => filled += n,
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    return Err(TransportError::Timeout)
                }
                Err(e) => return Err(TransportError::Io(e)),
            }
        }
        Ok(())
    }
}

impl Transport for UartTransport {
    fn peek(&mut self, addr: u32) -> Result<u32, TransportError> {
        self.flush_input()?;
        let word_addr = (addr >> 2).to_be_bytes();
        let mut req = Vec::with_capacity(6);
        req.extend_from_slice(&[CMD_READ, 0x01]); // read, length = 1 word
        req.extend_from_slice(&word_addr);
        self.port.write_all(&req)?;
        self.port.flush()?;

        let mut data = [0u8; 4];
        self.read_exact(&mut data)?;
        Ok(u32::from_be_bytes(data))
    }

    fn poke(&mut self, addr: u32, value: u32) -> Result<(), TransportError> {
        self.flush_input()?;
        let word_addr = (addr >> 2).to_be_bytes();
        let mut req = Vec::with_capacity(10);
        req.extend_from_slice(&[CMD_WRITE, 0x01]); // write, length = 1 word
        req.extend_from_slice(&word_addr);
        req.extend_from_slice(&value.to_be_bytes());
        self.port.write_all(&req)?;
        self.port.flush()?;
        Ok(())
    }

    fn read_burst(&mut self, addr: u32, count: usize) -> Result<Vec<u32>, TransportError> {
        // uartbone READ_BURST_INCR: [cmd, len], word-addr (BE); device returns
        // 4*len bytes and auto-increments the address. `len` is one byte, so
        // chunk into runs of <= 255 words.
        self.flush_input()?;
        let mut out = Vec::with_capacity(count);
        let mut word_addr = addr >> 2;
        let mut remaining = count;
        while remaining > 0 {
            let chunk = remaining.min(MAX_BURST);
            let mut req = Vec::with_capacity(6);
            req.extend_from_slice(&[CMD_READ, chunk as u8]);
            req.extend_from_slice(&word_addr.to_be_bytes());
            self.port.write_all(&req)?;
            self.port.flush()?;

            let mut data = vec![0u8; 4 * chunk];
            self.read_exact(&mut data)?;
            for w in data.chunks_exact(4) {
                out.push(u32::from_be_bytes(w.try_into().unwrap()));
            }
            word_addr += chunk as u32;
            remaining -= chunk;
        }
        Ok(out)
    }

    fn write_burst(&mut self, addr: u32, values: &[u32]) -> Result<(), TransportError> {
        // uartbone WRITE_BURST_INCR: [cmd, len], word-addr (BE), then 4*len data
        // bytes. Chunk into runs of <= 255 words.
        self.flush_input()?;
        let mut word_addr = addr >> 2;
        for chunk in values.chunks(MAX_BURST) {
            let mut req = Vec::with_capacity(6 + 4 * chunk.len());
            req.extend_from_slice(&[CMD_WRITE, chunk.len() as u8]);
            req.extend_from_slice(&word_addr.to_be_bytes());
            for &v in chunk {
                req.extend_from_slice(&v.to_be_bytes());
            }
            self.port.write_all(&req)?;
            self.port.flush()?;
            word_addr += chunk.len() as u32;
        }
        Ok(())
    }
}
