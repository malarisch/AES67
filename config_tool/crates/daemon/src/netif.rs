//! TAP network interface: a layer-2 Ethernet device whose frames the daemon
//! shuttles to/from the FPGA's `eth_buf`. The kernel side appears as a normal
//! NIC, so standard Linux tooling (dhcpcd, multicast, sockets) works on it.
//!
//! The interface is created via `TUNSETIFF` on `/dev/net/tun`; the MAC/MTU/up
//! configuration is applied with `ip link` (avoids fiddly netlink code). Data
//! I/O is plain `read`/`write` on the tap fd, so the RX and TX services can each
//! own a cloned handle and run concurrently.

use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::os::unix::io::AsRawFd;
use std::process::Command;

// _IOW('T', 202, int) on Linux.
const TUNSETIFF: libc::c_ulong = 0x4004_54ca;
// _IOW('T', 226, int) — set the tap's carrier (link) state (Linux ≥ 5.0).
const TUNSETCARRIER: libc::c_ulong = 0x4004_54e2;
const IFF_TAP: libc::c_short = 0x0002;
const IFF_NO_PI: libc::c_short = 0x1000;

#[repr(C)]
struct IfReq {
    ifr_name: [libc::c_char; libc::IFNAMSIZ],
    ifr_flags: libc::c_short,
    _pad: [u8; 22], // pad to sizeof(struct ifreq) = 40
}

/// An open TAP device.
pub struct Tap {
    file: File,
    name: String,
}

impl Tap {
    /// Create (or attach to) a TAP device. `requested` may contain a `%d`
    /// pattern (e.g. `aes67%d`) for the kernel to fill in; the actual name is
    /// read back. Requires `CAP_NET_ADMIN` (typically root).
    pub fn create(requested: &str) -> io::Result<Self> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open("/dev/net/tun")?;

        let mut req: IfReq = unsafe { std::mem::zeroed() };
        for (i, b) in requested.bytes().take(libc::IFNAMSIZ - 1).enumerate() {
            req.ifr_name[i] = b as libc::c_char;
        }
        req.ifr_flags = IFF_TAP | IFF_NO_PI;

        let ret = unsafe { libc::ioctl(file.as_raw_fd(), TUNSETIFF, &mut req) };
        if ret < 0 {
            return Err(io::Error::last_os_error());
        }

        let name = req
            .ifr_name
            .iter()
            .take_while(|&&c| c != 0)
            .map(|&c| c as u8 as char)
            .collect();
        Ok(Self { file, name })
    }

    /// The kernel interface name (e.g. `aes67d0`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Set the MAC address, MTU, and bring the interface up via `ip link`.
    pub fn configure(&self, mac: [u8; 6], mtu: u32) -> io::Result<()> {
        let mac = format!(
            "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
        );
        run_ip(&[
            "link", "set", "dev", &self.name, "address", &mac, "mtu", &mtu.to_string(), "up",
        ])
    }

    /// Two cloned handles `(reader, writer)` over the same tap fd, so the RX and
    /// TX services can run on separate threads.
    pub fn split(&self) -> io::Result<(File, File)> {
        Ok((self.file.try_clone()?, self.file.try_clone()?))
    }

    /// A standalone cloned handle on the tap fd, e.g. for the link monitor to
    /// drive the carrier independently of the RX/TX I/O handles.
    pub fn clone_handle(&self) -> io::Result<File> {
        self.file.try_clone()
    }
}

/// Set the tap's carrier (link) state. With carrier off the kernel marks the
/// interface `NO-CARRIER`, so the stack (and e.g. `dhcpcd`) treats the link as
/// down even while the interface is administratively up. Mirrors the FPGA PHY
/// link so Linux sees the real Ethernet link state. `tap` must be a tap fd.
pub fn set_carrier(tap: &File, on: bool) -> io::Result<()> {
    let val: libc::c_int = on as libc::c_int;
    let ret = unsafe { libc::ioctl(tap.as_raw_fd(), TUNSETCARRIER, &val) };
    if ret < 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(())
}

/// Find the name of the network interface whose hardware (MAC) address matches
/// `mac`, or `None` if none does. Used in kernel-transport mode to locate the
/// `aes67_eth` netdev (whose kernel-assigned name, e.g. `eth1`, is not fixed)
/// from the FPGA MAC the daemon already knows.
pub fn interface_by_mac(mac: [u8; 6]) -> Option<String> {
    use std::ffi::CStr;
    // SAFETY: getifaddrs allocates a list we free with freeifaddrs; each node is
    // read only while the list is alive.
    unsafe {
        let mut ifap: *mut libc::ifaddrs = std::ptr::null_mut();
        if libc::getifaddrs(&mut ifap) != 0 {
            return None;
        }
        let mut result = None;
        let mut cur = ifap;
        while !cur.is_null() {
            let ifa = &*cur;
            if !ifa.ifa_addr.is_null()
                && (*ifa.ifa_addr).sa_family as i32 == libc::AF_PACKET
            {
                let sll = std::ptr::read_unaligned(ifa.ifa_addr as *const libc::sockaddr_ll);
                if sll.sll_halen == 6 && sll.sll_addr[..6] == mac[..] {
                    result = Some(
                        CStr::from_ptr(ifa.ifa_name).to_string_lossy().into_owned(),
                    );
                    break;
                }
            }
            cur = ifa.ifa_next;
        }
        libc::freeifaddrs(ifap);
        result
    }
}

fn run_ip(args: &[&str]) -> io::Result<()> {
    let status = Command::new("ip").args(args).status()?;
    if !status.success() {
        return Err(io::Error::other(format!("`ip {}` failed", args.join(" "))));
    }
    Ok(())
}

/// Read one frame from a tap handle into `buf`; returns its length (0 on close).
pub fn recv(tap: &mut File, buf: &mut [u8]) -> io::Result<usize> {
    tap.read(buf)
}

/// Write one frame to a tap handle.
pub fn send(tap: &mut File, frame: &[u8]) -> io::Result<()> {
    tap.write_all(frame)
}
