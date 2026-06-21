//! Persistent daemon configuration / state (`aes67d.json`).
//!
//! One JSON file does double duty:
//!
//! * **Daemon configuration** — how to run: which transport, CSR map, control
//!   socket, and TAP/network parameters. CLI flags override what the file says
//!   (so the file holds the durable defaults and the command line is the
//!   override), and whatever transport/network is resolved at startup is written
//!   back so the file always reflects the running configuration.
//! * **FPGA settings** — every persistent setting made over the control API
//!   (MAC, register writes, PTP grandmaster, RX/TX streams). On startup these are
//!   replayed into the FPGA so it comes up configured as it was left. The **IP is
//!   deliberately not persisted**: it comes from DHCP (or a static host config)
//!   and is owned by the Linux side, mirrored live by the [monitor](crate::monitor).
//!
//! Settings are stored as the proto DTOs, which already capture exactly the
//! mutating control requests, so recording a change is just stashing the request.

use std::collections::BTreeMap;
use std::io;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use aes67_proto::{self as proto, GrandmasterParams, RxStreamParams, TxStreamParams};

/// The whole persisted document.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct DaemonConfig {
    /// How to reach the FPGA bus. `None` until set by CLI or a prior run.
    pub transport: Option<TransportCfg>,
    /// LiteX CSR map path.
    pub csr: Option<PathBuf>,
    /// Control socket path.
    pub socket: Option<PathBuf>,
    /// TAP network bridge parameters.
    pub network: NetworkCfg,
    /// Verbosity (0–3).
    pub verbose: Option<u8>,
    /// Replayed FPGA settings (everything except the IP).
    pub settings: Settings,
}

/// Transport selection, mirrors the two CPU-less bus bridges.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum TransportCfg {
    Uart {
        device: String,
        #[serde(default)]
        baud: Option<u32>,
    },
    Spi {
        device: String,
        #[serde(default)]
        speed_hz: Option<u32>,
    },
}

/// TAP bridge configuration.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct NetworkCfg {
    pub tap: Option<String>,
    pub mtu: Option<u32>,
    /// MAC for the FPGA + TAP (e.g. "02:00:00:12:34:56").
    pub mac: Option<String>,
    /// RX IRQ GPIO line, as "CHIP:LINE".
    pub irq_gpio: Option<String>,
    pub poll_ms: Option<u64>,
    /// Command run when the link comes up (cold start and link recovery) to
    /// (re)start DHCP. `{iface}` is replaced with the TAP name. Absent ⇒ default
    /// `["dhcpcd", "{iface}"]`; an empty list disables it (rely on the system's
    /// network manager, which reacts to the carrier the daemon already drives).
    pub dhcp_command: Option<Vec<String>>,
}

/// FPGA settings replayed on startup. Stored as proto DTOs (the exact mutating
/// requests). Keyed maps keep the latest value per register / stream id. The MAC
/// is not here — it lives in [`NetworkCfg::mac`] (one human-readable location,
/// shared with the `--mac` startup config).
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    pub registers: BTreeMap<String, u64>,
    pub grandmaster: Option<GrandmasterParams>,
    pub tx_streams: BTreeMap<u8, TxStreamParams>,
    pub rx_streams: BTreeMap<u8, RxStreamParams>,
}

impl NetworkCfg {
    /// The resolved DHCP-trigger command template (with the `{iface}`
    /// placeholder), or `None` if DHCP triggering is disabled.
    pub fn dhcp_template(&self) -> Option<Vec<String>> {
        match &self.dhcp_command {
            None => Some(vec!["dhcpcd".into(), "{iface}".into()]),
            Some(c) if c.is_empty() => None,
            Some(c) => Some(c.clone()),
        }
    }
}

impl DaemonConfig {
    /// Load from `path`. A missing file yields the default (empty) config; a
    /// present-but-malformed file is an error (don't silently drop settings).
    pub fn load(path: &Path) -> io::Result<Self> {
        match std::fs::read(path) {
            Ok(bytes) => serde_json::from_slice(&bytes)
                .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e)),
            Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(Self::default()),
            Err(e) => Err(e),
        }
    }

    /// Write to `path` atomically (write a temp file, then rename), so a crash
    /// mid-write can't truncate the live config.
    pub fn save(&self, path: &Path) -> io::Result<()> {
        let mut json = serde_json::to_vec_pretty(self)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
        json.push(b'\n');
        let tmp = path.with_extension("json.tmp");
        std::fs::write(&tmp, &json)?;
        std::fs::rename(&tmp, path)
    }

    /// Fold one successful mutating request into the stored settings. Returns
    /// `true` if anything persistable changed (caller then re-saves). The IP and
    /// transient/raw operations (SetIp, Reset, raw address writes) are not
    /// persisted.
    pub fn record(&mut self, req: &proto::Request) -> bool {
        use proto::Request as R;
        match req {
            R::SetMac { mac } => {
                self.network.mac = Some(format_mac(mac));
                true
            }
            R::WriteRegister { name, value } => {
                self.settings.registers.insert(name.clone(), *value);
                true
            }
            R::SetGrandmaster(p) => {
                let gm = self.settings.grandmaster.get_or_insert_with(Default::default);
                merge_grandmaster(gm, p);
                true
            }
            R::SetTxStream(p) => {
                self.settings.tx_streams.insert(p.id, p.clone());
                true
            }
            R::SetRxStream(p) => {
                self.settings.rx_streams.insert(p.id, p.clone());
                true
            }
            // Not persisted: IP is DHCP-owned; resets are transient; raw address
            // writes aren't reproducible by name here.
            _ => false,
        }
    }
}

/// Format a MAC as the canonical lowercase colon-separated string.
fn format_mac(mac: &[u8; 6]) -> String {
    format!(
        "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
    )
}

/// Merge the `Some` fields of `new` into `dst`, leaving the rest untouched, so
/// partial grandmaster updates accumulate.
fn merge_grandmaster(dst: &mut GrandmasterParams, new: &GrandmasterParams) {
    if new.priority1.is_some() {
        dst.priority1 = new.priority1;
    }
    if new.priority2.is_some() {
        dst.priority2 = new.priority2;
    }
    if new.clock_class.is_some() {
        dst.clock_class = new.clock_class;
    }
    if new.clock_accuracy.is_some() {
        dst.clock_accuracy = new.clock_accuracy;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use aes67_proto::Request;

    #[test]
    fn records_persistable_ops_and_skips_the_rest() {
        let mut cfg = DaemonConfig::default();

        // MAC goes to network.mac (human-readable, single location).
        assert!(cfg.record(&Request::SetMac { mac: [0x02, 0, 0, 0x12, 0x34, 0x56] }));
        assert_eq!(cfg.network.mac.as_deref(), Some("02:00:00:12:34:56"));

        // Register writes accumulate by name.
        assert!(cfg.record(&Request::WriteRegister { name: "aes67_csr_scratch".into(), value: 7 }));
        assert_eq!(cfg.settings.registers["aes67_csr_scratch"], 7);

        // IP is never persisted (DHCP-owned), nor are resets.
        assert!(!cfg.record(&Request::SetIp { ip: "192.168.1.2".into() }));
        assert!(!cfg.record(&Request::Reset(Default::default())));
        assert!(cfg.settings.registers.len() == 1);
    }

    #[test]
    fn grandmaster_updates_merge() {
        let mut cfg = DaemonConfig::default();
        cfg.record(&Request::SetGrandmaster(GrandmasterParams {
            priority1: Some(128),
            ..Default::default()
        }));
        cfg.record(&Request::SetGrandmaster(GrandmasterParams {
            clock_class: Some(6),
            ..Default::default()
        }));
        let gm = cfg.settings.grandmaster.unwrap();
        // Both partial updates survive.
        assert_eq!(gm.priority1, Some(128));
        assert_eq!(gm.clock_class, Some(6));
    }

    #[test]
    fn save_then_load_round_trips() {
        let mut cfg = DaemonConfig {
            transport: Some(TransportCfg::Spi {
                device: "/dev/spidev0.0".into(),
                speed_hz: Some(1_000_000),
            }),
            ..Default::default()
        };
        cfg.record(&Request::WriteRegister { name: "aes67_csr_scratch".into(), value: 0xdead });

        let path =
            std::env::temp_dir().join(format!("aes67d-persist-{}.json", std::process::id()));
        cfg.save(&path).unwrap();
        let back = DaemonConfig::load(&path).unwrap();
        let _ = std::fs::remove_file(&path);

        assert_eq!(back.transport, cfg.transport);
        assert_eq!(back.settings.registers["aes67_csr_scratch"], 0xdead);
    }

    #[test]
    fn dhcp_template_default_disabled_and_custom() {
        // Absent → default dhcpcd.
        let net = NetworkCfg::default();
        assert_eq!(net.dhcp_template(), Some(vec!["dhcpcd".into(), "{iface}".into()]));

        // Empty list → disabled.
        let net = NetworkCfg { dhcp_command: Some(vec![]), ..Default::default() };
        assert_eq!(net.dhcp_template(), None);

        // Custom command preserved.
        let cmd = vec!["udhcpc".to_string(), "-i".to_string(), "{iface}".to_string()];
        let net = NetworkCfg { dhcp_command: Some(cmd.clone()), ..Default::default() };
        assert_eq!(net.dhcp_template(), Some(cmd));
    }

    #[test]
    fn missing_file_loads_default() {
        let path = std::env::temp_dir().join("aes67d-does-not-exist-xyz.json");
        let _ = std::fs::remove_file(&path);
        let cfg = DaemonConfig::load(&path).unwrap();
        assert!(cfg.transport.is_none());
        assert!(cfg.settings.registers.is_empty());
    }
}
