//! RTSP server exposing the node's TX streams (RAVENNA §3.4: a transmitting node
//! acts as RTSP server).
//!
//! A `DESCRIBE` on `/by-id/<id>` or `/by-name/<name>` returns the SDP for the
//! matching TX stream — built exactly like the SAP announcement (same
//! [`tx_to_stream`](crate::discovery::tx_to_stream)), so SAP and RTSP describe a
//! stream identically. Unknown sessions get a `404` with the connection kept open
//! so the server can `ANNOUNCE` them once configured (handled in `aes67-rtsp`).

use std::net::{Ipv4Addr, TcpListener};
use std::sync::Arc;

use aes67_config::ControlApi;
use aes67_rtsp::{SessionTarget, Sessions};

use crate::discovery::{read_leader_gmid, session_name, tx_to_stream};
use crate::{SharedConfig, SharedDevice};

/// Default RTSP port (RFC 2326). Privileged, so the daemon must hold the right to
/// bind it (it already runs with `CAP_NET_*` for the TAP).
pub const DEFAULT_RTSP_PORT: u16 = 554;

/// Serves SDP for the daemon's configured TX streams.
struct DaemonSessions {
    device: SharedDevice,
    config: SharedConfig,
}

impl Sessions for DaemonSessions {
    fn describe(&self, target: &SessionTarget) -> Option<String> {
        // Resolve the target to a configured TX stream.
        let params = {
            let cfg = self.config.lock().unwrap();
            match target {
                SessionTarget::ById(id) => {
                    u8::try_from(*id).ok().and_then(|id| cfg.settings.tx_streams.get(&id).cloned())
                }
                // Match the effective session name, so unnamed streams resolve by
                // their default "AES67 TX <id>" too (the name we also advertise).
                SessionTarget::ByName(name) => {
                    cfg.settings.tx_streams.values().find(|p| session_name(p) == *name).cloned()
                }
            }
        }?;

        // Origin IP and grandmaster id come from the FPGA (one bus lock).
        let (origin, gmid) = {
            let mut d = self.device.lock().unwrap();
            let ip = d.get_ip().ok().filter(|ip| !ip.is_unspecified() && !ip.is_loopback())?;
            (ip, read_leader_gmid(&mut *d))
        };

        Some(tx_to_stream(&params, origin, gmid.as_deref())?.to_sdp())
    }
}

/// Spawn the RTSP server on `port` for the life of the process. Binding failures
/// are logged and disable RTSP (best-effort), so they never abort the daemon.
pub fn spawn(device: SharedDevice, config: SharedConfig, port: u16, verbose: u8) {
    std::thread::spawn(move || {
        let listener = match TcpListener::bind((Ipv4Addr::UNSPECIFIED, port)) {
            Ok(l) => l,
            Err(e) => {
                eprintln!("aes67d: rtsp: cannot bind port {port}: {e} — RTSP disabled");
                return;
            }
        };
        if verbose >= 1 {
            eprintln!("aes67d: rtsp: serving TX streams on port {port}");
        }
        let sessions: Arc<dyn Sessions> = Arc::new(DaemonSessions { device, config });
        aes67_rtsp::serve(listener, sessions);
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::persist::DaemonConfig;
    use aes67_config::{CsrMap, Device, Transport};
    use aes67_proto::TxStreamParams;
    use aes67_transport::MockTransport;
    use std::sync::Mutex;

    #[test]
    fn describe_resolves_tx_streams_by_id_and_name() {
        let map = CsrMap::from_csv(
            "csr_register,aes67_csr_ip_addr,0x18,1,rw\n\
             csr_register,aes67_csr_ptp_leader_id_lo,0x00,1,rw\n\
             csr_register,aes67_csr_ptp_leader_id_hi,0x04,1,rw\n\
             memory_region,tx_stream_cfg,0x1000,256,linker\n",
        )
        .unwrap();
        let t: Box<dyn Transport + Send> = Box::new(MockTransport::new());
        let device: SharedDevice = Arc::new(Mutex::new(Device::new(t, map)));
        device.lock().unwrap().set_ip(Ipv4Addr::new(192, 168, 1, 9)).unwrap();

        let mut cfg = DaemonConfig::default();
        cfg.settings.tx_streams.insert(
            0,
            TxStreamParams {
                id: 0,
                dst_ip: "239.69.1.0".into(),
                channels: Some(2),
                samples_per_packet: 48,
                ch_ids: vec![0, 1],
                ssrc: 0,
                name: Some("Line".into()),
            },
        );
        let config: SharedConfig = Arc::new(Mutex::new(cfg));

        let sessions = DaemonSessions { device, config };

        // By id and by name resolve to the same RAVENNA SDP.
        let by_id = sessions.describe(&SessionTarget::ById(0)).expect("by-id");
        assert!(by_id.contains("c=IN IP4 239.69.1.0"));
        assert!(by_id.contains("s=Line"));
        assert!(by_id.contains("a=clock-domain:PTPv2 0")); // RAVENNA attribute
        assert_eq!(sessions.describe(&SessionTarget::ByName("Line".into())).as_deref(), Some(by_id.as_str()));

        // Unknown slot / name → no session.
        assert!(sessions.describe(&SessionTarget::ById(5)).is_none());
        assert!(sessions.describe(&SessionTarget::ByName("Nope".into())).is_none());
    }
}
