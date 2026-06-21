//! GPIO interrupt line for `eth_buf` RX-ready.
//!
//! The FPGA drives `eth_buf_irq` (= `ev.irq`, a level that is high while an RX
//! packet is pending). We watch it as a **rising-edge** event on a GPIO
//! character device and combine it with a poll timeout so a missed edge can
//! never wedge RX — the timeout is a safety net, the edge is the low-latency
//! path.

use std::io;
use std::os::unix::io::AsRawFd;

use gpio_cdev::{Chip, EventRequestFlags, LineEventHandle, LineRequestFlags};

/// A requested GPIO line delivering rising-edge events.
pub struct IrqLine {
    handle: LineEventHandle,
}

impl IrqLine {
    /// Request `offset` on the GPIO chip at `chip_path` (e.g. `/dev/gpiochip0`)
    /// for rising-edge events.
    pub fn open(chip_path: &str, offset: u32) -> Result<Self, gpio_cdev::Error> {
        let mut chip = Chip::new(chip_path)?;
        let line = chip.get_line(offset)?;
        let handle = line.events(
            LineRequestFlags::INPUT,
            EventRequestFlags::RISING_EDGE,
            "aes67d-ethirq",
        )?;
        Ok(Self { handle })
    }

    /// Wait up to `timeout_ms` for an edge. Returns `true` if an edge fired (and
    /// is consumed), `false` on timeout.
    pub fn wait(&mut self, timeout_ms: i32) -> io::Result<bool> {
        let mut pfd = libc::pollfd {
            fd: self.handle.as_raw_fd(),
            events: libc::POLLIN,
            revents: 0,
        };
        let ret = unsafe { libc::poll(&mut pfd, 1, timeout_ms) };
        if ret < 0 {
            return Err(io::Error::last_os_error());
        }
        if ret == 0 {
            return Ok(false); // timeout
        }
        // Drain the event so the fd is not perpetually readable.
        let _ = self.handle.get_event();
        Ok(true)
    }
}
