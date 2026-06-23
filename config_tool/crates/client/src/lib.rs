//! Client for the AES67 daemon (`aes67d`).
//!
//! [`RemoteDevice`] connects to the daemon's Unix control socket and implements
//! the same [`ControlApi`](aes67_config::ControlApi) as the in-process
//! `Device`, so the CLI (and any other Rust consumer) is identical whether it
//! talks to hardware directly or through the daemon.
//!
//! Framing and message types come from `aes67-proto`; conversions between wire
//! DTOs and internal types live in `aes67-config`.

use std::io::BufReader;
use std::net::Ipv4Addr;
use std::os::unix::net::UnixStream;
use std::path::Path;

use aes67_config::{
    ConfigError, ControlApi, PtpGrandmaster, RegisterInfo, RxStream, TxStream,
};
use aes67_proto::{self as proto, framing};

/// A connection to a running `aes67d` over its Unix control socket.
pub struct RemoteDevice {
    writer: UnixStream,
    reader: BufReader<UnixStream>,
    next_id: u64,
}

impl RemoteDevice {
    /// Connect to the daemon socket at `path` and perform the version handshake.
    pub fn connect<P: AsRef<Path>>(path: P) -> Result<Self, ConfigError> {
        let writer = UnixStream::connect(path.as_ref())
            .map_err(|e| ConfigError::Remote(format!("connecting to {}: {e}", path.as_ref().display())))?;
        let reader = BufReader::new(
            writer
                .try_clone()
                .map_err(|e| ConfigError::Remote(e.to_string()))?,
        );
        let mut dev = Self { writer, reader, next_id: 1 };

        // Handshake: agree on the protocol version up front.
        match dev.call(proto::Request::Hello {
            client: format!("aes67-client/{}", env!("CARGO_PKG_VERSION")),
            proto_version: proto::PROTO_VERSION,
        })? {
            proto::Response::Hello { proto_version, .. } if proto_version == proto::PROTO_VERSION => {
                Ok(dev)
            }
            proto::Response::Hello { proto_version, .. } => Err(ConfigError::Remote(format!(
                "protocol version mismatch: daemon {proto_version}, client {}",
                proto::PROTO_VERSION
            ))),
            other => Err(unexpected(&other)),
        }
    }

    /// Send one request and read its response, mapping daemon errors.
    fn call(&mut self, request: proto::Request) -> Result<proto::Response, ConfigError> {
        let id = self.next_id;
        self.next_id += 1;

        framing::write_line(&mut self.writer, &proto::RequestEnvelope { id, request })
            .map_err(|e| ConfigError::Remote(e.to_string()))?;

        let env: proto::ResponseEnvelope = framing::read_line(&mut self.reader)
            .map_err(|e| ConfigError::Remote(e.to_string()))?
            .ok_or_else(|| ConfigError::Remote("daemon closed the connection".into()))?;

        if env.id != id {
            return Err(ConfigError::Remote(format!(
                "response id {} does not match request id {id}",
                env.id
            )));
        }
        env.result.map_err(map_rpc_error)
    }

    /// Helper for methods that expect a bare `Ok` acknowledgement.
    fn call_ok(&mut self, request: proto::Request) -> Result<(), ConfigError> {
        match self.call(request)? {
            proto::Response::Ok => Ok(()),
            other => Err(unexpected(&other)),
        }
    }

    /// Fetch the daemon's persisted config snapshot (configured RX/TX streams).
    /// Daemon-specific, so it is not part of [`ControlApi`].
    pub fn get_config(&mut self) -> Result<proto::ConfigSnapshot, ConfigError> {
        match self.call(proto::Request::GetConfig)? {
            proto::Response::Config(c) => Ok(c),
            other => Err(unexpected(&other)),
        }
    }

    /// List the AES67 streams discovered on the network (via SAP/SDP).
    /// Daemon-specific, so it is not part of [`ControlApi`].
    pub fn get_discovered(&mut self) -> Result<Vec<proto::DiscoveredStream>, ConfigError> {
        match self.call(proto::Request::GetDiscovered)? {
            proto::Response::Discovered(v) => Ok(v),
            other => Err(unexpected(&other)),
        }
    }

    /// Subscribe to a remote RAVENNA session over RTSP: the daemon DESCRIBEs `url`
    /// and configures the returned stream into RX slot `rx_id`. Daemon-specific.
    pub fn subscribe_rtsp(&mut self, url: &str, rx_id: u8) -> Result<(), ConfigError> {
        self.call_ok(proto::Request::SubscribeRtsp { url: url.to_string(), rx_id })
    }
}

fn map_rpc_error(e: proto::RpcError) -> ConfigError {
    match e.code {
        proto::ErrorCode::NotFound => ConfigError::UnknownRegister(e.message),
        proto::ErrorCode::ReadOnly => ConfigError::ReadOnly(e.message),
        proto::ErrorCode::BadRequest => ConfigError::Parse(e.message),
        other => ConfigError::Remote(format!("{other:?}: {}", e.message)),
    }
}

fn unexpected(r: &proto::Response) -> ConfigError {
    ConfigError::Remote(format!("unexpected daemon response: {r:?}"))
}

impl ControlApi for RemoteDevice {
    fn list_registers(&mut self) -> Result<Vec<RegisterInfo>, ConfigError> {
        match self.call(proto::Request::ListRegisters)? {
            proto::Response::Registers(v) => Ok(v.into_iter().map(Into::into).collect()),
            other => Err(unexpected(&other)),
        }
    }

    fn read_register(&mut self, name: &str) -> Result<u64, ConfigError> {
        match self.call(proto::Request::ReadRegister { name: name.into() })? {
            proto::Response::RegisterValue(v) => Ok(v),
            other => Err(unexpected(&other)),
        }
    }

    fn write_register(&mut self, name: &str, value: u64) -> Result<(), ConfigError> {
        self.call_ok(proto::Request::WriteRegister { name: name.into(), value })
    }

    fn read_addr(&mut self, addr: u32) -> Result<u32, ConfigError> {
        match self.call(proto::Request::ReadAddr { addr })? {
            proto::Response::Word(v) => Ok(v),
            other => Err(unexpected(&other)),
        }
    }

    fn write_addr(&mut self, addr: u32, value: u32) -> Result<(), ConfigError> {
        self.call_ok(proto::Request::WriteAddr { addr, value })
    }

    fn get_mac(&mut self) -> Result<[u8; 6], ConfigError> {
        match self.call(proto::Request::GetMac)? {
            proto::Response::Mac(m) => Ok(m),
            other => Err(unexpected(&other)),
        }
    }

    fn set_mac(&mut self, mac: [u8; 6]) -> Result<(), ConfigError> {
        self.call_ok(proto::Request::SetMac { mac })
    }

    fn get_ip(&mut self) -> Result<Ipv4Addr, ConfigError> {
        match self.call(proto::Request::GetIp)? {
            proto::Response::Ip(s) => s
                .parse()
                .map_err(|_| ConfigError::Remote(format!("daemon returned invalid IP '{s}'"))),
            other => Err(unexpected(&other)),
        }
    }

    fn set_ip(&mut self, ip: Ipv4Addr) -> Result<(), ConfigError> {
        self.call_ok(proto::Request::SetIp { ip: ip.to_string() })
    }

    fn set_grandmaster(&mut self, gm: PtpGrandmaster) -> Result<(), ConfigError> {
        self.call_ok(proto::Request::SetGrandmaster(gm.into()))
    }

    fn write_tx_stream(&mut self, stream: &TxStream) -> Result<(), ConfigError> {
        self.call_ok(proto::Request::SetTxStream(stream.into()))
    }

    fn write_rx_stream(&mut self, stream: &RxStream) -> Result<(), ConfigError> {
        self.call_ok(proto::Request::SetRxStream(stream.into()))
    }

    fn clear_tx_stream(&mut self, id: u8) -> Result<(), ConfigError> {
        self.call_ok(proto::Request::StopTxStream { id })
    }

    fn clear_rx_stream(&mut self, id: u8) -> Result<(), ConfigError> {
        self.call_ok(proto::Request::StopRxStream { id })
    }

    fn reset(&mut self, ptp: bool, tx: bool, rx: bool, eth: bool) -> Result<(), ConfigError> {
        self.call_ok(proto::Request::Reset(proto::ResetMask { ptp, tx, rx, eth }))
    }
}
