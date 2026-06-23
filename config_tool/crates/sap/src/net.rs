//! SAP over UDP: a [`SapSender`] and a [`SapListener`] wrapping a `UdpSocket`.
//!
//! Both take an already-bound socket so the *caller* owns interface binding,
//! multicast joins and TTL — in this project the daemon binds to the TAP address
//! and joins [`SAP_GROUP`] on that interface. Keeping the socket setup out of
//! here lets the same protocol code back a different transport (e.g. a test
//! loopback socket, or a future unicast SAP path) unchanged.

use std::io;
use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};

use crate::packet::{SapPacket, SAP_GROUP, SAP_PORT};

/// Sends SAP announcements / deletions to a multicast group.
pub struct SapSender {
    socket: UdpSocket,
    dest: SocketAddrV4,
    source: Ipv4Addr,
}

impl SapSender {
    /// Wrap `socket`, sending to `dest` and stamping `source` into the SAP
    /// header (the originating node's address).
    pub fn new(socket: UdpSocket, dest: SocketAddrV4, source: Ipv4Addr) -> Self {
        Self { socket, dest, source }
    }

    /// Send to the standard SAP group ([`SAP_GROUP`]:[`SAP_PORT`]).
    pub fn to_default_group(socket: UdpSocket, source: Ipv4Addr) -> Self {
        Self::new(socket, SocketAddrV4::new(SAP_GROUP, SAP_PORT), source)
    }

    /// Announce a session description.
    pub fn announce(&self, sdp: &str) -> io::Result<()> {
        self.send(&SapPacket::announce(self.source, sdp))
    }

    /// Withdraw a previously announced session description.
    pub fn delete(&self, sdp: &str) -> io::Result<()> {
        self.send(&SapPacket::delete(self.source, sdp))
    }

    fn send(&self, pkt: &SapPacket) -> io::Result<()> {
        let bytes = pkt.encode();
        let n = self.socket.send_to(&bytes, self.dest)?;
        if n != bytes.len() {
            return Err(io::Error::new(io::ErrorKind::WriteZero, "short SAP send"));
        }
        Ok(())
    }
}

/// Receives and decodes SAP packets from a (multicast-joined) socket.
pub struct SapListener {
    socket: UdpSocket,
    buf: Vec<u8>,
}

impl SapListener {
    /// Wrap an already-bound (and, for receiving multicast, group-joined) socket.
    pub fn new(socket: UdpSocket) -> Self {
        // Max IPv4 UDP payload comfortably covers any AES67 SDP announcement.
        Self { socket, buf: vec![0u8; 4096] }
    }

    /// The underlying socket, e.g. to set a read timeout or non-blocking mode.
    pub fn socket(&self) -> &UdpSocket {
        &self.socket
    }

    /// Receive the next datagram and decode it.
    ///
    /// * `Ok(Some(pkt))` — a valid SAP/SDP packet.
    /// * `Ok(None)` — a datagram that is not a SAP/SDP packet we handle (a
    ///   malformed or foreign packet); the caller should simply keep listening.
    /// * `Err(_)` — a socket error (including a read timeout, as `WouldBlock`).
    pub fn recv(&mut self) -> io::Result<Option<SapPacket>> {
        let (n, _from) = self.socket.recv_from(&mut self.buf)?;
        Ok(SapPacket::decode(&self.buf[..n]).ok())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::packet::SapKind;

    const SDP: &str = "v=0\r\no=- 7 7 IN IP4 127.0.0.1\r\ns=loop\r\n";

    #[test]
    fn sender_to_listener_over_loopback() {
        // Listener bound to an ephemeral loopback port.
        let rx = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let dest = match rx.local_addr().unwrap() {
            std::net::SocketAddr::V4(a) => a,
            _ => unreachable!(),
        };
        let mut listener = SapListener::new(rx);

        // Sender aimed straight at the listener (unicast loopback exercises the
        // exact same encode/recv/decode path as multicast).
        let tx = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let sender = SapSender::new(tx, dest, Ipv4Addr::LOCALHOST);

        sender.announce(SDP).unwrap();
        let pkt = listener.recv().unwrap().expect("a SAP packet");
        assert_eq!(pkt.kind, SapKind::Announce);
        assert_eq!(pkt.sdp, SDP);

        sender.delete(SDP).unwrap();
        let pkt = listener.recv().unwrap().expect("a SAP packet");
        assert_eq!(pkt.kind, SapKind::Delete);
    }

    #[test]
    fn non_sap_datagram_yields_none() {
        let rx = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let dest = rx.local_addr().unwrap();
        let mut listener = SapListener::new(rx);

        let tx = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        tx.send_to(b"garbage", dest).unwrap();
        assert!(listener.recv().unwrap().is_none());
    }
}
