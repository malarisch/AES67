//! Kernel backend — talk to the FPGA Wishbone bus through the `spibone` kernel
//! bus driver's char device (`/dev/spibone0`).
//!
//! The **kernel** owns the SPI link: the spibone bus driver (driver/spibone) is
//! the sole Wishbone master, and the aes67_eth driver (netdev datapath + PHC) is
//! a client of it. Userspace no longer opens `spidev` directly; instead it
//! issues peek/poke ioctls on `/dev/spiboneN`, which the bus driver serialises
//! with the kernel's own bus access. This backend is a thin wrapper over those
//! ioctls and plugs into the same [`Transport`](crate::Transport) trait as
//! SPI/UART. Addresses are the absolute byte addresses of the csr.csv.
//!
//! The ioctl ABI mirrors `driver/spibone/spibone_uapi.h` — keep them in sync.

/// Default control device created by the kernel module (misc char device).
pub const DEFAULT_CTL_PATH: &str = "/dev/spibone0";

#[cfg(target_os = "linux")]
pub use linux::KernelTransport;

#[cfg(not(target_os = "linux"))]
pub use stub::KernelTransport;

#[cfg(target_os = "linux")]
mod linux {
    use std::fs::{File, OpenOptions};
    use std::os::unix::io::AsRawFd;

    use crate::{Transport, TransportError};

    /// `struct spibone_wb_xfer` from spibone_uapi.h: one word transfer by byte addr.
    #[repr(C)]
    struct WbXfer {
        addr: u32,
        val: u32,
    }

    // _IOC encoding (asm-generic, used by x86 and arm64): dir<<30 | size<<16 |
    // type<<8 | nr. dir: _IOC_WRITE=1, _IOC_READ=2, _IOWR=3. Magic 0xB5
    // (SPIBONE_IOC_MAGIC), the struct is 8 bytes.
    const fn ioc(dir: u32, nr: u32) -> libc::Ioctl {
        let size = core::mem::size_of::<WbXfer>() as u32;
        ((dir << 30) | (size << 16) | (0xB5u32 << 8) | nr) as libc::Ioctl
    }
    const SPIBONE_IOC_PEEK: libc::Ioctl = ioc(3, 1); // _IOWR(0xB5, 1, xfer)
    const SPIBONE_IOC_POKE: libc::Ioctl = ioc(1, 2); // _IOW (0xB5, 2, xfer)

    /// A connection to the FPGA through the kernel driver's control device.
    pub struct KernelTransport {
        dev: File,
    }

    impl KernelTransport {
        /// Open the control char device (e.g. `/dev/spibone0`).
        pub fn open(path: &str) -> Result<Self, TransportError> {
            let dev = OpenOptions::new()
                .read(true)
                .write(true)
                .open(path)
                .map_err(|e| TransportError::Serial(format!("opening {path}: {e}")))?;
            Ok(Self { dev })
        }
    }

    impl Transport for KernelTransport {
        fn peek(&mut self, addr: u32) -> Result<u32, TransportError> {
            let mut x = WbXfer { addr, val: 0 };
            // SAFETY: valid fd, ioctl number and the pointed-to struct match the
            // kernel ABI; the kernel only reads/writes `x`.
            let ret =
                unsafe { libc::ioctl(self.dev.as_raw_fd(), SPIBONE_IOC_PEEK, &mut x as *mut WbXfer) };
            if ret < 0 {
                return Err(TransportError::Io(std::io::Error::last_os_error()));
            }
            Ok(x.val)
        }

        fn poke(&mut self, addr: u32, value: u32) -> Result<(), TransportError> {
            let x = WbXfer { addr, val: value };
            // SAFETY: as above; the kernel only reads `x` for POKE.
            let ret =
                unsafe { libc::ioctl(self.dev.as_raw_fd(), SPIBONE_IOC_POKE, &x as *const WbXfer) };
            if ret < 0 {
                return Err(TransportError::Io(std::io::Error::last_os_error()));
            }
            Ok(())
        }
    }
}

#[cfg(not(target_os = "linux"))]
mod stub {
    use crate::{Transport, TransportError};

    /// Placeholder on non-Linux hosts, where `/dev/spibone0` does not exist.
    pub struct KernelTransport;

    impl KernelTransport {
        pub fn open(_path: &str) -> Result<Self, TransportError> {
            Err(TransportError::Unsupported(
                "kernel (/dev/spibone0) backend is only available on Linux".into(),
            ))
        }
    }

    impl Transport for KernelTransport {
        fn peek(&mut self, _addr: u32) -> Result<u32, TransportError> {
            unreachable!("KernelTransport cannot be constructed on this platform")
        }
        fn poke(&mut self, _addr: u32, _value: u32) -> Result<(), TransportError> {
            unreachable!("KernelTransport cannot be constructed on this platform")
        }
    }
}
