//! Server core for `aes67d` — kept in a library so it can be integration-tested
//! with a mock transport (see `tests/`). The binary ([`main`](../main.rs)) only
//! parses arguments, opens the real transport, and calls [`run`].
//!
//! The daemon is the **single owner** of the Wishbone link: the shared
//! [`Device`] behind a mutex is the "transport arbiter" — every client request
//! serialises through it.

use std::io::{self, BufReader};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use aes67_config::{ControlApi, Device, RxStream, Transport, TxStream};
use aes67_proto::{self as proto, framing};

pub mod bridge;
pub mod gpio;
pub mod igmp;
pub mod monitor;
pub mod netif;
pub mod persist;
pub mod startup;

use persist::{DaemonConfig, Settings};

/// A device whose transport is chosen at runtime, shared across connections.
pub type SharedDevice = Arc<Mutex<Device<Box<dyn Transport + Send>>>>;

/// The live daemon config (run settings + persisted FPGA state), shared between
/// the control server (which records mutations into it) and the startup/monitor
/// threads (which read the *current* settings to (re)configure the FPGA — e.g.
/// on an FPGA-reset recovery, which must see config made after startup).
pub type SharedConfig = Arc<Mutex<DaemonConfig>>;

/// Persists FPGA settings to the JSON config as control requests succeed.
pub struct Persist {
    path: PathBuf,
    config: SharedConfig,
}

impl Persist {
    pub fn new(path: PathBuf, config: SharedConfig) -> Self {
        Self { path, config }
    }

    /// Fold a successful request into the stored settings and re-save if it
    /// changed anything persistable.
    fn record(&self, req: &proto::Request) {
        let mut cfg = self.config.lock().expect("config mutex poisoned");
        if cfg.record(req) {
            if let Err(e) = cfg.save(&self.path) {
                eprintln!("aes67d: persist: saving {} failed: {e}", self.path.display());
            }
        }
    }
}

/// The serving context shared across connections: the device arbiter plus an
/// optional persistence sink (absent in tests with no config file).
pub struct Server {
    pub device: SharedDevice,
    pub persist: Option<Persist>,
}

impl Server {
    pub fn new(device: SharedDevice, persist: Option<Persist>) -> Arc<Self> {
        Arc::new(Self { device, persist })
    }
}

/// Accept connections forever, serving each on its own thread.
pub fn run(listener: UnixListener, server: Arc<Server>) {
    for conn in listener.incoming() {
        match conn {
            Ok(stream) => {
                let srv = Arc::clone(&server);
                std::thread::spawn(move || {
                    if let Err(e) = serve_connection(stream, srv) {
                        eprintln!("aes67d: connection ended: {e}");
                    }
                });
            }
            Err(e) => eprintln!("aes67d: accept error: {e}"),
        }
    }
}

/// Serve one client until it disconnects: read request lines, dispatch through
/// the shared device, persist successful mutations, write response lines.
pub fn serve_connection(stream: UnixStream, server: Arc<Server>) -> io::Result<()> {
    let mut writer = stream.try_clone()?;
    let mut reader = BufReader::new(stream);

    while let Some(env) = framing::read_line::<_, proto::RequestEnvelope>(&mut reader)? {
        // Keep a copy for persistence only if there's a sink to record into.
        let to_record = server.persist.as_ref().map(|_| env.request.clone());
        // GetConfig is answered from the persisted config, not the device.
        let result = if matches!(env.request, proto::Request::GetConfig) {
            Ok(config_snapshot(&server))
        } else {
            let mut dev = server.device.lock().expect("device mutex poisoned");
            dispatch(&mut dev, env.request)
        };
        if result.is_ok() {
            if let (Some(p), Some(req)) = (&server.persist, &to_record) {
                p.record(req);
            }
        }
        framing::write_line(&mut writer, &proto::ResponseEnvelope { id: env.id, result })?;
    }
    Ok(())
}

/// Replay persisted FPGA settings into the device on startup, so it comes up
/// configured as it was left. The IP is intentionally excluded — it is owned by
/// the Linux side and mirrored live by the [monitor].
pub fn replay_settings(device: &SharedDevice, settings: &Settings, verbose: u8) {
    let mut dev = device.lock().expect("device mutex poisoned");
    let mut n = 0usize;

    for (name, value) in &settings.registers {
        match dev.write_register(name, *value) {
            Ok(()) => n += 1,
            Err(e) => eprintln!("aes67d: replay: write {name} failed: {e}"),
        }
    }
    if let Some(gm) = settings.grandmaster {
        match dev.set_grandmaster(gm.into()) {
            Ok(()) => n += 1,
            Err(e) => eprintln!("aes67d: replay: set_grandmaster failed: {e}"),
        }
    }
    for p in settings.tx_streams.values() {
        match TxStream::try_from(p.clone()).and_then(|s| dev.write_tx_stream(&s)) {
            Ok(()) => n += 1,
            Err(e) => eprintln!("aes67d: replay: tx stream {} failed: {e}", p.id),
        }
    }
    for p in settings.rx_streams.values() {
        match RxStream::try_from(p.clone()).and_then(|s| dev.write_rx_stream(&s)) {
            Ok(()) => n += 1,
            Err(e) => eprintln!("aes67d: replay: rx stream {} failed: {e}", p.id),
        }
    }

    if verbose >= 1 && n > 0 {
        eprintln!("aes67d: replayed {n} persisted setting(s) into the FPGA");
    }
}

/// Map one request onto the device's [`ControlApi`], producing a wire response.
fn dispatch(
    dev: &mut Device<Box<dyn Transport + Send>>,
    request: proto::Request,
) -> Result<proto::Response, proto::RpcError> {
    use proto::{Request as Req, Response as Resp};

    let ok = |()| Resp::Ok;
    match request {
        Req::Hello { proto_version, .. } => {
            if proto_version != proto::PROTO_VERSION {
                return Err(rpc(
                    proto::ErrorCode::Version,
                    format!("client proto {proto_version}, daemon {}", proto::PROTO_VERSION),
                ));
            }
            Ok(Resp::Hello {
                server: format!("aes67d/{}", env!("CARGO_PKG_VERSION")),
                proto_version: proto::PROTO_VERSION,
            })
        }

        Req::ListRegisters => dev
            .list_registers()
            .map(|v| Resp::Registers(v.into_iter().map(Into::into).collect()))
            .map_err(err),
        Req::ReadRegister { name } => dev.read_register(&name).map(Resp::RegisterValue).map_err(err),
        Req::WriteRegister { name, value } => dev.write_register(&name, value).map(ok).map_err(err),
        Req::ReadAddr { addr } => dev.read_addr(addr).map(Resp::Word).map_err(err),
        Req::WriteAddr { addr, value } => dev.write_addr(addr, value).map(ok).map_err(err),

        Req::GetMac => dev.get_mac().map(Resp::Mac).map_err(err),
        Req::SetMac { mac } => dev.set_mac(mac).map(ok).map_err(err),
        Req::GetIp => dev.get_ip().map(|ip| Resp::Ip(ip.to_string())).map_err(err),
        Req::SetIp { ip } => {
            let parsed = ip
                .parse()
                .map_err(|_| rpc(proto::ErrorCode::BadRequest, format!("invalid IPv4 '{ip}'")))?;
            dev.set_ip(parsed).map(ok).map_err(err)
        }

        Req::SetGrandmaster(p) => dev.set_grandmaster(p.into()).map(ok).map_err(err),
        Req::SetTxStream(p) => {
            let s = p.try_into().map_err(err)?;
            dev.write_tx_stream(&s).map(ok).map_err(err)
        }
        Req::SetRxStream(p) => {
            let s = p.try_into().map_err(err)?;
            dev.write_rx_stream(&s).map(ok).map_err(err)
        }
        Req::Reset(m) => dev.reset(m.ptp, m.tx, m.rx, m.eth).map(ok).map_err(err),

        // Answered in serve_connection (needs the persisted config, not the bus).
        Req::GetConfig => Err(rpc(proto::ErrorCode::Internal, "GetConfig not dispatchable".into())),
    }
}

/// Build a [`proto::Response::Config`] from the persisted config (empty when
/// persistence is disabled).
fn config_snapshot(server: &Server) -> proto::Response {
    let snapshot = match &server.persist {
        Some(p) => {
            let cfg = p.config.lock().expect("config mutex poisoned");
            proto::ConfigSnapshot {
                tx_streams: cfg.settings.tx_streams.values().cloned().collect(),
                rx_streams: cfg.settings.rx_streams.values().cloned().collect(),
            }
        }
        None => proto::ConfigSnapshot::default(),
    };
    proto::Response::Config(snapshot)
}

fn rpc(code: proto::ErrorCode, message: String) -> proto::RpcError {
    proto::RpcError { code, message }
}

/// Map a [`ConfigError`](aes67_config::ConfigError) onto a wire error code.
fn err(e: aes67_config::ConfigError) -> proto::RpcError {
    use aes67_config::ConfigError as C;
    use proto::ErrorCode as E;
    let code = match &e {
        C::UnknownRegister(_) => E::NotFound,
        C::ReadOnly(_) => E::ReadOnly,
        C::ValueTooWide { .. } | C::Parse(_) | C::Unsupported(_) => E::BadRequest,
        C::Transport(_) => E::Transport,
        C::Io(_) | C::Remote(_) => E::Internal,
    };
    rpc(code, e.to_string())
}
