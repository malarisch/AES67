//! Control-plane wire protocol for the AES67 daemon.
//!
//! This crate is the **contract** between `aes67d` (the daemon that owns the
//! SPI/UART transport) and its clients — the CLI today, a web UI or a ravennakit
//! bridge tomorrow. It is deliberately dependency-light (only `serde`): a client
//! must be able to speak it without linking the transport or device layers.
//!
//! Framing: newline-delimited JSON. Each line from a client is a
//! [`RequestEnvelope`]; each line from the server is a [`ResponseEnvelope`] with
//! the matching `id`. The first exchange should be [`Request::Hello`] so both
//! sides agree on [`PROTO_VERSION`].
//!
//! The wire DTOs here are intentionally separate from the internal types in
//! `aes67-config` (e.g. IP addresses travel as strings) so the on-the-wire
//! format can stay stable independently of the implementation.

use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use std::io::{self, BufRead, Write};

/// Protocol version. Bumped on any breaking change to the message set.
pub const PROTO_VERSION: u32 = 3;

/// Newline-delimited JSON framing, shared by the daemon and every client so the
/// wire format has a single source of truth.
pub mod framing {
    use super::*;

    /// Serialise `msg` as one JSON line (terminated by `\n`) and flush.
    pub fn write_line<W: Write, T: Serialize>(w: &mut W, msg: &T) -> io::Result<()> {
        let mut buf = serde_json::to_vec(msg).map_err(io::Error::other)?;
        buf.push(b'\n');
        w.write_all(&buf)?;
        w.flush()
    }

    /// Read one JSON line. Returns `Ok(None)` on clean EOF.
    pub fn read_line<R: BufRead, T: DeserializeOwned>(r: &mut R) -> io::Result<Option<T>> {
        let mut line = String::new();
        if r.read_line(&mut line)? == 0 {
            return Ok(None); // EOF
        }
        let msg = serde_json::from_str(line.trim_end()).map_err(io::Error::other)?;
        Ok(Some(msg))
    }
}

/// A request with a correlation id (echoed in the response).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RequestEnvelope {
    pub id: u64,
    pub request: Request,
}

/// A response with the id of the request it answers.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResponseEnvelope {
    pub id: u64,
    pub result: Result<Response, RpcError>,
}

/// Client → server requests. Tagged by `method`, params inline under `params`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "method", content = "params", rename_all = "snake_case")]
pub enum Request {
    /// Version/capability handshake; should be the first message.
    Hello { client: String, proto_version: u32 },

    /// Enumerate the CSR map.
    ListRegisters,
    /// Read a register by name (value may span >32 bits).
    ReadRegister { name: String },
    /// Write a register by name.
    WriteRegister { name: String, value: u64 },
    /// Read a raw 32-bit word at an absolute byte address.
    ReadAddr { addr: u32 },
    /// Write a raw 32-bit word at an absolute byte address.
    WriteAddr { addr: u32, value: u32 },

    GetMac,
    SetMac { mac: [u8; 6] },
    GetIp,
    /// IPv4 address as a dotted string, e.g. "192.168.1.42".
    SetIp { ip: String },

    SetGrandmaster(GrandmasterParams),
    SetTxStream(TxStreamParams),
    SetRxStream(RxStreamParams),
    /// Pulse the selected reset domains.
    Reset(ResetMask),

    /// Read the daemon's persisted config (e.g. the configured RX/TX streams,
    /// which the write-only FPGA stream RAMs cannot report back).
    GetConfig,
}

/// Server → client successful results.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "kind", content = "data", rename_all = "snake_case")]
pub enum Response {
    Hello { server: String, proto_version: u32 },
    Registers(Vec<RegisterInfo>),
    RegisterValue(u64),
    Word(u32),
    Mac([u8; 6]),
    /// IPv4 address as a dotted string.
    Ip(String),
    /// The daemon's persisted config snapshot.
    Config(ConfigSnapshot),
    /// A successful operation with no payload.
    Ok,
}

/// The daemon's persisted configuration relevant to clients — currently the
/// configured RX/TX streams (the write-only FPGA RAMs can't be read back).
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ConfigSnapshot {
    pub tx_streams: Vec<TxStreamParams>,
    pub rx_streams: Vec<RxStreamParams>,
}

/// An error result. `code` groups failures for programmatic handling.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RpcError {
    pub code: ErrorCode,
    pub message: String,
}

/// Coarse error classification (the human-readable detail is in `message`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ErrorCode {
    /// Malformed request or bad parameters.
    BadRequest,
    /// Unknown register / address / name.
    NotFound,
    /// Target is read-only or otherwise not writable.
    ReadOnly,
    /// Transport / bus failure talking to the FPGA.
    Transport,
    /// Protocol version mismatch.
    Version,
    /// Anything else.
    Internal,
}

// -- Wire DTOs --------------------------------------------------------------

/// A CSR map entry as seen by clients.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegisterInfo {
    pub name: String,
    pub addr: u32,
    pub size: u32,
    /// "ro", "rw", or "??".
    pub access: String,
}

/// PTP grandmaster announce parameters; `None` leaves a field untouched.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct GrandmasterParams {
    pub priority1: Option<u8>,
    pub priority2: Option<u8>,
    pub clock_class: Option<u8>,
    pub clock_accuracy: Option<u8>,
}

/// Reset domains to pulse.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct ResetMask {
    pub ptp: bool,
    pub tx: bool,
    pub rx: bool,
    pub eth: bool,
}

/// Transmit stream parameters (IP as a dotted string on the wire).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TxStreamParams {
    pub id: u8,
    pub dst_ip: String,
    #[serde(default)]
    pub channels: Option<u8>,
    #[serde(default)]
    pub samples_per_packet: u8,
    #[serde(default)]
    pub ch_ids: Vec<u8>,
    #[serde(default)]
    pub ssrc: u32,
}

/// Receive stream parameters (IP as a dotted string on the wire).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RxStreamParams {
    pub id: u8,
    pub dst_ip: String,
    pub dst_port: u16,
    #[serde(default)]
    pub ch_map: Vec<u8>,
    #[serde(default)]
    pub channels: Option<u8>,
    #[serde(default)]
    pub output_delay: u8,
    #[serde(default)]
    pub samples_per_channel: u8,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_round_trips() {
        let req = RequestEnvelope {
            id: 7,
            request: Request::WriteRegister {
                name: "aes67_csr_scratch".into(),
                value: 0xdead_beef,
            },
        };
        let s = serde_json::to_string(&req).unwrap();
        assert!(s.contains("\"method\":\"write_register\""));
        let back: RequestEnvelope = serde_json::from_str(&s).unwrap();
        assert_eq!(back.id, 7);
        match back.request {
            Request::WriteRegister { name, value } => {
                assert_eq!(name, "aes67_csr_scratch");
                assert_eq!(value, 0xdead_beef);
            }
            _ => panic!("wrong variant"),
        }
    }

    #[test]
    fn response_ok_and_err_round_trip() {
        let ok = ResponseEnvelope { id: 1, result: Ok(Response::Word(0x1234)) };
        let s = serde_json::to_string(&ok).unwrap();
        let back: ResponseEnvelope = serde_json::from_str(&s).unwrap();
        assert!(matches!(back.result, Ok(Response::Word(0x1234))));

        let err = ResponseEnvelope {
            id: 2,
            result: Err(RpcError { code: ErrorCode::NotFound, message: "nope".into() }),
        };
        let s = serde_json::to_string(&err).unwrap();
        let back: ResponseEnvelope = serde_json::from_str(&s).unwrap();
        match back.result {
            Err(e) => assert_eq!(e.code, ErrorCode::NotFound),
            _ => panic!("expected err"),
        }
    }

    #[test]
    fn stream_params_default_optionals() {
        let p: TxStreamParams =
            serde_json::from_str(r#"{"id":0,"dst_ip":"239.69.1.1"}"#).unwrap();
        assert_eq!(p.id, 0);
        assert_eq!(p.samples_per_packet, 0);
        assert!(p.ch_ids.is_empty());
    }
}
