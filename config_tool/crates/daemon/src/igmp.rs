//! IGMP multicast-group membership on the TAP interface.
//!
//! The FPGA does the audio/PTP data plane in hardware, but an upstream switch
//! only forwards a multicast group to the FPGA's port if a member reports
//! interest via IGMP. The Linux host — reachable through the TAP bridge — is the
//! IGMP speaker, so joining the relevant groups *on the TAP* makes the switch
//! forward that multicast to the FPGA MAC. The Linux stack itself does not
//! consume the audio (the FPGA hardware does); the membership only steers switch
//! forwarding.
//!
//! Memberships are held via `setsockopt(IP_ADD_MEMBERSHIP)` on a dedicated
//! socket, scoped to the TAP by interface index. The kernel uses the interface's
//! address as the report source, so callers join only once a valid IP exists.

use std::collections::BTreeSet;
use std::ffi::CString;
use std::io;
use std::mem::size_of;
use std::net::{Ipv4Addr, UdpSocket};
use std::os::unix::io::AsRawFd;

/// Holds IGMP memberships for one interface on a throwaway socket.
pub struct IgmpManager {
    iface: String,
    if_index: u32,
    sock: UdpSocket,
    joined: BTreeSet<Ipv4Addr>,
}

impl IgmpManager {
    /// Bind a membership socket scoped to `iface`.
    pub fn new(iface: &str) -> io::Result<Self> {
        let cname =
            CString::new(iface).map_err(|_| io::Error::other("interface name contains NUL"))?;
        // SAFETY: cname is a valid NUL-terminated C string.
        let if_index = unsafe { libc::if_nametoindex(cname.as_ptr()) };
        if if_index == 0 {
            return Err(io::Error::last_os_error());
        }
        // A socket only to carry memberships; never used for data I/O.
        let sock = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0))?;
        Ok(Self { iface: iface.to_string(), if_index, sock, joined: BTreeSet::new() })
    }

    fn membership(&self, op: libc::c_int, group: Ipv4Addr) -> io::Result<()> {
        let mreq = libc::ip_mreqn {
            // s_addr is network byte order; the octets are already in that order,
            // so from_ne_bytes preserves the in-memory layout.
            imr_multiaddr: libc::in_addr { s_addr: u32::from_ne_bytes(group.octets()) },
            imr_address: libc::in_addr { s_addr: 0 },
            imr_ifindex: self.if_index as libc::c_int,
        };
        // SAFETY: fd is valid for the socket's lifetime; mreq is the expected type.
        let ret = unsafe {
            libc::setsockopt(
                self.sock.as_raw_fd(),
                libc::IPPROTO_IP,
                op,
                &mreq as *const _ as *const libc::c_void,
                size_of::<libc::ip_mreqn>() as libc::socklen_t,
            )
        };
        if ret < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(())
        }
    }

    /// Reconcile memberships to exactly `desired` (multicast groups only),
    /// preserving order for fresh joins. With `force`, drop and re-add every
    /// group so the kernel emits fresh IGMP reports — used after a link-up so the
    /// switch re-learns the groups. Returns the number of (re)joins performed.
    pub fn apply(&mut self, desired: &[Ipv4Addr], force: bool, verbose: u8) -> usize {
        let desired: Vec<Ipv4Addr> =
            desired.iter().copied().filter(Ipv4Addr::is_multicast).collect();

        if force {
            // Fresh reports for everything: drop all, then re-add below.
            for g in std::mem::take(&mut self.joined) {
                let _ = self.membership(libc::IP_DROP_MEMBERSHIP, g);
            }
        } else {
            // Leave groups no longer wanted.
            let stale: Vec<Ipv4Addr> =
                self.joined.iter().copied().filter(|g| !desired.contains(g)).collect();
            for g in stale {
                match self.membership(libc::IP_DROP_MEMBERSHIP, g) {
                    Ok(()) => {
                        self.joined.remove(&g);
                        if verbose >= 1 {
                            eprintln!("aes67d: igmp: left {g} on {}", self.iface);
                        }
                    }
                    Err(e) => eprintln!("aes67d: igmp: leave {g} failed: {e}"),
                }
            }
        }

        let mut joins = 0;
        for g in desired {
            if self.joined.contains(&g) {
                continue;
            }
            match self.membership(libc::IP_ADD_MEMBERSHIP, g) {
                Ok(()) => {
                    self.joined.insert(g);
                    joins += 1;
                    if verbose >= 1 {
                        eprintln!("aes67d: igmp: joined {g} on {}", self.iface);
                    }
                }
                Err(e) => eprintln!("aes67d: igmp: join {g} failed: {e}"),
            }
        }
        joins
    }
}
