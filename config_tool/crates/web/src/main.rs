//! `aes67web` — a small monitoring web server for the AES67 daemon.
//!
//! It is purely a **client** of `aes67d`: it connects to the daemon's Unix
//! control socket (like the CLI) and exposes monitoring data over a tiny REST
//! API plus a self-contained one-page dashboard. The daemon and wire protocol
//! are unchanged — the web server only reads CSRs by name through `ControlApi`.
//!
//! Routes:
//! * `GET /`              — the dashboard (single embedded HTML page)
//! * `GET /api/status`    — decoded monitoring snapshot (link, PTP, network, …)
//! * `GET /api/registers` — full CSR map with current values

use std::io::Cursor;
use std::path::{Path, PathBuf};

use anyhow::{anyhow, Result};
use clap::Parser;
use serde::de::DeserializeOwned;
use serde::Serialize;
use tiny_http::{Header, Method, Request, Response, Server, StatusCode};

use aes67_client::RemoteDevice;
use aes67_config::{ConfigError, ControlApi};

mod apply;
mod snapshot;

/// The dashboard page, compiled into the binary so it serves standalone.
const INDEX_HTML: &str = include_str!("index.html");

#[derive(Parser)]
#[command(name = "aes67web", version, about)]
struct Cli {
    /// Daemon control socket to read monitoring data from.
    #[arg(long, value_name = "PATH", default_value = "/run/aes67d.sock")]
    socket: PathBuf,
    /// Address to serve the dashboard / REST API on.
    #[arg(long, value_name = "ADDR", default_value = "0.0.0.0:8080")]
    listen: String,
    /// mDNS/DNS-SD instance name to advertise the dashboard under (`_http._tcp`).
    #[arg(long, value_name = "NAME", default_value = "AES67 Configuration")]
    mdns_name: String,
    /// Disable the mDNS/DNS-SD announcement of the dashboard.
    #[arg(long)]
    no_mdns: bool,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let server = Server::http(&cli.listen)
        .map_err(|e| anyhow!("binding HTTP server on {}: {e}", cli.listen))?;
    eprintln!("aes67web: serving dashboard on http://{}/ (daemon {})", cli.listen, cli.socket.display());

    // Advertise the dashboard over mDNS/DNS-SD so it is discoverable as
    // `_http._tcp` without hunting for an IP. Best-effort: runs on its own thread.
    if !cli.no_mdns {
        let port = listen_port(&cli.listen);
        aes67_mdns::spawn(aes67_mdns::Service::http(cli.mdns_name.clone(), port));
    }

    // One lazily-(re)connected daemon link; requests are served sequentially.
    let mut dev: Option<RemoteDevice> = None;

    for mut req in server.incoming_requests() {
        // Own method + path so the body reader can borrow `req` mutably below.
        let method = req.method().clone();
        let path = req.url().split('?').next().unwrap_or("/").to_string();
        let response = match (&method, path.as_str()) {
            (Method::Get, "/") => html(INDEX_HTML),
            (Method::Get, "/api/status") => {
                json(with_dev(&cli.socket, &mut dev, |d| snapshot::snapshot(d)))
            }
            (Method::Get, "/api/registers") => {
                json(with_dev(&cli.socket, &mut dev, |d| snapshot::registers(d)))
            }
            (Method::Get, "/api/ptp") => {
                json(with_dev(&cli.socket, &mut dev, |d| snapshot::ptp_config(d)))
            }
            (Method::Get, "/api/streams") => {
                json(with_dev(&cli.socket, &mut dev, |d| d.get_config()))
            }
            (Method::Get, "/api/discovered") => {
                json(with_dev(&cli.socket, &mut dev, |d| d.get_discovered()))
            }
            (Method::Post, "/api/ptp") => {
                post(&cli.socket, &mut dev, &mut req, apply::apply_ptp)
            }
            (Method::Post, "/api/tx-stream") => {
                post(&cli.socket, &mut dev, &mut req, apply::apply_tx)
            }
            (Method::Post, "/api/rx-stream") => {
                post(&cli.socket, &mut dev, &mut req, apply::apply_rx)
            }
            (Method::Post, "/api/tx-stream/stop") => {
                post(&cli.socket, &mut dev, &mut req, apply::stop_tx)
            }
            (Method::Post, "/api/rx-stream/stop") => {
                post(&cli.socket, &mut dev, &mut req, apply::stop_rx)
            }
            (Method::Get, _) => text(404, "not found"),
            _ => text(405, "method not allowed"),
        };
        let _ = req.respond(response);
    }
    Ok(())
}

/// The TCP port from a `--listen` address (e.g. `0.0.0.0:8080` → 8080), so the
/// mDNS record points at the right port. Defaults to 8080 if unparseable.
fn listen_port(listen: &str) -> u16 {
    listen
        .parse::<std::net::SocketAddr>()
        .map(|a| a.port())
        .ok()
        .or_else(|| listen.rsplit_once(':').and_then(|(_, p)| p.parse().ok()))
        .unwrap_or(8080)
}

/// Run `f` against a connected daemon, lazily connecting and reconnecting once
/// if the link is stale (e.g. the daemon restarted between requests).
fn with_dev<T>(
    socket: &Path,
    slot: &mut Option<RemoteDevice>,
    f: impl Fn(&mut RemoteDevice) -> Result<T, ConfigError>,
) -> Result<T, ConfigError> {
    let mut last_err = None;
    for final_attempt in [false, true] {
        if slot.is_none() {
            match RemoteDevice::connect(socket) {
                Ok(d) => *slot = Some(d),
                Err(e) => {
                    last_err = Some(e);
                    continue;
                }
            }
        }
        match f(slot.as_mut().unwrap()) {
            Ok(v) => return Ok(v),
            Err(e) => {
                *slot = None; // drop the (probably dead) connection and retry once
                last_err = Some(e);
                if final_attempt {
                    break;
                }
            }
        }
    }
    Err(last_err.unwrap())
}

/// Read a JSON POST body, apply it through `f` against a (re)connected daemon,
/// and return a small JSON `{ok, message}` result.
fn post<R: DeserializeOwned>(
    socket: &Path,
    slot: &mut Option<RemoteDevice>,
    req: &mut Request,
    f: impl Fn(&mut dyn ControlApi, &R) -> Result<(), ConfigError>,
) -> Body {
    let mut body = String::new();
    if req.as_reader().read_to_string(&mut body).is_err() {
        return json_msg(400, false, "could not read request body");
    }
    let dto: R = match serde_json::from_str(&body) {
        Ok(d) => d,
        Err(e) => return json_msg(400, false, &format!("invalid JSON: {e}")),
    };
    match with_dev(socket, slot, |d| f(d, &dto)) {
        Ok(()) => json_msg(200, true, "applied"),
        Err(e) => json_msg(503, false, &e.to_string()),
    }
}

type Body = Response<Cursor<Vec<u8>>>;

fn header(name: &str, value: &str) -> Header {
    Header::from_bytes(name.as_bytes(), value.as_bytes()).expect("valid header")
}

fn html(body: &str) -> Body {
    Response::from_string(body).with_header(header("Content-Type", "text/html; charset=utf-8"))
}

fn text(code: u16, body: &str) -> Body {
    Response::from_string(body).with_status_code(StatusCode(code))
}

/// Serialise `r` to JSON; on error return a 503 with a small JSON error body so
/// the dashboard can show "daemon unreachable" instead of breaking.
fn json<T: Serialize>(r: Result<T, ConfigError>) -> Body {
    let json_ct = header("Content-Type", "application/json");
    match r {
        Ok(value) => {
            let body = serde_json::to_string(&value).unwrap_or_else(|e| error_json(&e.to_string()));
            Response::from_string(body).with_header(json_ct)
        }
        Err(e) => Response::from_string(error_json(&e.to_string()))
            .with_header(json_ct)
            .with_status_code(StatusCode(503)),
    }
}

fn error_json(msg: &str) -> String {
    format!("{{\"error\":{}}}", serde_json::to_string(msg).unwrap_or_else(|_| "\"error\"".into()))
}

/// A `{"ok": bool, "message": "..."}` JSON response with the given status code.
fn json_msg(code: u16, ok: bool, message: &str) -> Body {
    let msg = serde_json::to_string(message).unwrap_or_else(|_| "\"\"".into());
    Response::from_string(format!("{{\"ok\":{ok},\"message\":{msg}}}"))
        .with_header(header("Content-Type", "application/json"))
        .with_status_code(StatusCode(code))
}
