# config_tool — AES67 FPGA register configuration (human designed, AI-coded, human checked)

Host-side CLI + library to read and write the AES67 FPGA's Wishbone/CSR
registers over either of the two CPU-less bus bridges in the gateware:

* **uartbone** — UART → Wishbone (serial)
* **spibone**  — 4-wire SPI → Wishbone

The register layout is **never hard-coded**: the tool loads the LiteX-generated
`csr.csv` (or `csr.json`) for the `aes67_bridge` build and resolves every name to
an address at runtime. Re-generate the gateware, and the tool tracks it.

## Architecture (three layered crates)

```
crates/cli         aes67cfg          CLI front-end (clap)
        │  depends on
crates/config      aes67-config      CSR map + Device (by-name access) + Aes67Config helpers
        │  depends on
crates/transport   aes67-transport   HAL: Transport trait + UART / SPI backends
```

* `aes67-transport` knows only *addresses*: a `Transport` trait with `peek` /
  `poke`, plus `UartTransport` (uartbone) and `SpiTransport` (spibone, Linux
  `spidev`). Each backend applies its bridge's address convention internally
  (uartbone wants a word address → `addr>>2`; spibone drops the low 2 bits in
  gateware → full byte address on the wire). Callers always pass byte addresses.
* `aes67-config` loads the `CsrMap`, exposes `Device::{read,write}` by register
  name (handling multi-word CSRs), the `Aes67Config` trait with semantic helpers
  (MAC, IP, PTP grandmaster, scratch, status), and the `Aes67Streams` trait for
  RX/TX stream config RAMs. Byte/bit conventions match the on-target firmware
  (`drivers/eth_litex/eth_litex.c`).
* `aes67-config-tool` wires arguments to the library; it holds no addresses.

The daemon (`aes67-daemon`) additionally runs **SAP/SDP discovery** on the TAP,
built from two transport-agnostic crates so the SDP layer can be reused for mDNS
later:

* `aes67-sdp` — the `AudioStream` model with an AES67 SDP generator (hand-rolled,
  canonical) and parser (via `sdp-rs`). Knows nothing about sockets.
* `aes67-sap` — the Session Announcement Protocol (RFC 2974) packet codec plus a
  `SapSender` / `SapListener` over a `UdpSocket`.

The daemon announces its configured TX streams (over SAP **and** mDNS) and
registers remote streams it learns — from SAP announcements and from browsing
`ravenna_session` mDNS instances (each RTSP-`DESCRIBE`d for its SDP) — into one
registry. Clients read it with `aes67cfg discovered` or `GET /api/discovered`.

* `aes67-mdns` — mDNS/DNS-SD over the pure-Rust `mdns-sd` (no C libraries to link,
  no `avahi-daemon` needed, so the cross-compile stays clean). A `Responder`
  registers/unregisters services (with DNS-SD subtypes) and browses for them. The
  web server advertises `_http._tcp` ("AES67 Configuration"); the daemon advertises
  its `_rtsp._tcp` node service **and** each TX stream as a
  `<name>._rtsp._tcp`/`ravenna_session` session (RAVENNA §3.5.2), and browses
  remote `ravenna_session` instances. Disable the web one with `aes67web
  --no-mdns`; rename with `--mdns-name`.
* `aes67-rtsp` — RAVENNA RTSP connection management (§3.4) over the pure-Rust
  `rtsp-types` codec. The daemon runs the **server** (a transmitting node), serving
  each TX stream's SDP under `/by-id/<id>` and `/by-name/<name>`, replying `404`
  with the connection kept open and pushing an `ANNOUNCE` once a session appears.
  The **client** (`aes67cfg subscribe rtsp://… --rx-id N`) DESCRIBEs a remote
  session and configures it into an RX slot. SDP carries both the AES67
  (`ts-refclk`/`mediaclk`) and RAVENNA (`clock-domain`/`sync-time`) clocking
  attributes.

## Build

Requires a Rust toolchain (`cargo`, stable). From this directory:

```bash
cargo build --release
# binary: target/release/aes67cfg
```

## Cross-compiling for Raspberry Pi

The tool has no native C dependencies (the `serialport` libudev feature is
disabled — we open device paths directly), so it cross-compiles cleanly. Pick
the target for your Pi:

| Pi model                         | OS        | Rust target                      |
|----------------------------------|-----------|----------------------------------|
| Pi 3 / 4 / 5                     | 64-bit    | `aarch64-unknown-linux-gnu`      |
| Pi 2 / 3 / 4 (32-bit)           | 32-bit    | `armv7-unknown-linux-gnueabihf`  |
| Pi 1 / Zero / Zero W (ARMv6)    | 32-bit    | `arm-unknown-linux-gnueabihf`    |

### Option A — build on the Pi (simplest)

Copy the source over (or `git clone`), install Rust, and:

```bash
cargo build --release          # binary: target/release/aes67cfg
```

### Option B — cross-compile from an x86 Linux host

Install the Rust target and a matching GCC cross-linker (the linker paths are
preconfigured in `.cargo/config.toml`):

```bash
# 64-bit Pi (most common today)
rustup target add aarch64-unknown-linux-gnu
sudo apt install gcc-aarch64-linux-gnu

cargo build --release --target aarch64-unknown-linux-gnu
# -> target/aarch64-unknown-linux-gnu/release/aes67cfg

# 32-bit Pi instead:
rustup target add armv7-unknown-linux-gnueabihf
sudo apt install gcc-arm-linux-gnueabihf
cargo build --release --target armv7-unknown-linux-gnueabihf
```

Copy the resulting binary to the Pi (e.g. `scp …/release/aes67cfg pi@host:`).

### Option C — `cross` (zero host setup, needs Docker)

```bash
cargo install cross
cross build --release --target aarch64-unknown-linux-gnu
```

`cross` runs the build in a container with the toolchain and sysroot already set
up, so no apt packages are needed on the host.

## Daemon mode (`aes67d`)

The CLI can either talk to the FPGA **directly** (`--uart`/`--spi`, opens the
transport itself) or, by default, connect to the **daemon** `aes67d` over a Unix
socket. The daemon is the single owner of the SPI/UART link and serves the
control API to any client (CLI now; web UI / ravennakit later) — see
[docs/control-plane-plan.md](docs/control-plane-plan.md).

```bash
# Start the daemon (owns the link)
aes67d --spi /dev/spidev0.0 --csr litex_soc/build/aes67_bridge/csr.csv \
       --socket /run/aes67d.sock --config /etc/aes67d.json

# CLI without --uart/--spi talks to the daemon:
aes67cfg --socket /run/aes67d.sock dump
aes67cfg set aes67_csr_scratch 0xdeadbeef     # default socket /run/aes67d.sock

# CLI with --uart/--spi bypasses the daemon (direct, for bring-up/debug):
aes67cfg --uart /dev/ttyUSB0 dump
```

### Network bridge

The daemon can also bridge the FPGA's Ethernet (`eth_buf`) into Linux as a TAP
interface, so the host runs the control-plane network (DHCP, multicast, SAP/SDP)
through the FPGA MAC. Audio stays in FPGA hardware — only control traffic crosses
the bus. Needs `CAP_NET_ADMIN` (root) and an RX IRQ GPIO line:

```bash
sudo aes67d --spi /dev/spidev0.0 \
    --tap aes67d0 --mac 02:00:00:12:34:56 \
    --irq-gpio /dev/gpiochip0:17        # omit --irq-gpio to poll (--poll-ms)

# Then, on the host, treat aes67d0 like any NIC:
sudo dhcpcd aes67d0          # or systemd-networkd
```

The daemon keeps the FPGA and the Linux TAP in sync, both directions, polling
once a second:

* **FPGA link + speed → Linux carrier.** The FPGA's `eth_link_up` status bit
  drives the TAP's carrier (`TUNSETCARRIER`): when the FPGA PHY link is down, the
  interface shows `NO-CARRIER`, so the stack and `dhcpcd` wait for a real link
  instead of sending into a dead port. The negotiated speed (`eth_speed`:
  10/100/1000) is logged alongside, e.g. `FPGA link up (1 Gbps) → tap carrier on`.
* **Linux address → FPGA IP.** Once Linux assigns an address to the TAP (DHCP
  lease or static), it is mirrored into the FPGA's IP CSR, so the data plane
  stamps the right source IP into outgoing RTP/UDP and answers ARP for it.
  Link-local (169.254/16) is kept (zeroconf needs it for later Ravenna
  discovery); only unspecified/loopback addresses are skipped.

### Startup sequence

The gateware powers up with **every module held in reset** (the `aes67_csr_reset`
CSR resets to all-ones, active-high) and the AD/DA converter held.

On start the daemon first checks whether the FPGA is **already running** (all
reset domains released *and* an IP configured). If so, only the daemon
restarted (e.g. it crashed) while the FPGA kept going — so it does **not** touch
the running MAC/IP/audio: it just refreshes its PTP + stream config and resumes
the TAP bridge (*warm restart*). Otherwise it runs the full staged bring-up on a
background thread (so the control socket is reachable throughout):

0. **Hold all resets** — re-assert every domain, so the bring-up is deterministic
   even on a daemon restart (FPGA may still be running).
1. **Write all known registers** — replay the persisted settings (MAC, PTP
   grandmaster, servo/parser tuning, streams) while modules are held.
2. **Ethernet out of reset** — release the MAC/PHY and bring up the TAP bridge so
   the link comes up and DHCP can run.
3. **Wait for an IP** — block until the FPGA's IP CSR is populated (the monitor
   writes it from the TAP lease); proceeds with a warning after 30 s.
4. **PTP out of reset** — start the clock once the network identity is known.
5. **Audio out of reset** — release the RX/TX paths and the AD/DA converter last.

Progress is logged (`aes67d: startup: …`). Without a `--tap` the IP wait is
skipped (no network source for an address).

**DHCP trigger.** The kernel has no DHCP client, so a userspace one must run.
On a link-up edge — cold start and recovery after a real link-down — the daemon
runs a configurable command (`network.dhcp_command`, default `["dhcpcd",
"{iface}"]`, where `{iface}` is the TAP name). A **warm restart does not**
trigger it: the FPGA was already running with a lease, so the initial link-up is
left alone (a subsequent down→up recovery still triggers it). Set the command to
`[]` to disable and let the system's network manager handle DHCP instead (it
reacts to the carrier the daemon already drives). Idempotent clients like
`dhcpcd` simply rebind when re-invoked.

**IGMP membership.** The FPGA's audio/PTP data plane is in hardware, but an
upstream switch only forwards a multicast group to the FPGA's port if a member
reports interest. The daemon joins the relevant groups **on the TAP** (the Linux
host is the IGMP speaker), so the switch forwards them to the FPGA MAC — the
Linux stack does not consume the audio, it only steers switch forwarding. Each
second the monitor reconciles memberships against the live config, in order:
the PTP group (`224.0.1.129`), then every RX stream's group, then every TX
stream's. Joins happen only once the TAP has a **valid IP** (the kernel needs a
source for the report). A newly configured RX/TX stream's group is picked up
within ~1 s; after every link-up (and FPGA-reset recovery) all groups are
re-reported so the switch re-learns them.

**Auto-recovery.** The monitor (once per second) also watches the reset CSR: if
it reads all-domains-held again during normal operation, the FPGA was reset or
reconfigured out from under the daemon. The daemon then rebuilds the whole
last-known state — all registers, MAC, and the last IP (which the address
mirror alone would not re-apply, since the TAP address is unchanged) — and
releases the reset domains in order, no restart needed.

### Config file (`/etc/aes67d.json`)

The daemon keeps a single JSON file (`--config`, default `/etc/aes67d.json`)
that holds both **how the daemon runs** and the **FPGA settings made over the
control API**:

* CLI flags override the file; whatever transport/CSR/socket/network is resolved
  at startup is written back, so the file always mirrors the running config. Run
  it once with `--spi …`/`--uart …` and subsequent starts need no flags.
* Every persistent FPGA setting made through the control API (MAC, register
  writes, PTP grandmaster, RX/TX streams) is saved here and **replayed into the
  FPGA on the next start**, so it comes up configured as it was left. The **IP is
  deliberately not persisted** — it is DHCP/host-owned and mirrored live (above).
  Transient/raw operations (resets, raw `poke`, profile applies) aren't recorded.

```json
{
  "transport": { "kind": "spi", "device": "/dev/spidev0.0", "speed_hz": 1000000 },
  "csr": "litex_soc/build/aes67_bridge/csr.csv",
  "socket": "/run/aes67d.sock",
  "network": { "tap": "aes67d0", "mtu": 1500, "mac": "02:00:00:12:34:56",
               "irq_gpio": "/dev/gpiochip0:17", "poll_ms": 50 },
  "verbose": 0,
  "settings": {
    "registers": { "aes67_csr_scratch": 3735928559 },
    "grandmaster": { "priority1": 128, "clock_class": 6 },
    "tx_streams": {}, "rx_streams": {}
  }
}
```

### Web dashboard (`aes67web`)

A small monitoring web server, itself just another **client** of the daemon (it
reads CSRs by name over the control socket — no protocol or daemon changes). It
serves a self-contained one-page dashboard plus a tiny REST API:

```bash
aes67web --socket /run/aes67d.sock --listen 0.0.0.0:8080
# then open http://<pi>:8080/
```

| Route                 | Method | Effect                                                  |
|-----------------------|--------|---------------------------------------------------------|
| `/`                   | GET    | the dashboard (single embedded HTML page, refreshes 2 s) |
| `/api/status`         | GET    | decoded snapshot: link/speed, PTP role+offset+path-delay+leader, wallclock lock, IP/MAC, RX overflow |
| `/api/registers`      | GET    | the full CSR map with current values                    |
| `/api/ptp`            | GET    | current PTP params (pre-fills the form) / POST sets them |
| `/api/streams`        | GET    | the daemon's configured RX/TX streams                   |
| `/api/ptp`            | POST   | set PTP grandmaster params + sync/announce intervals    |
| `/api/tx-stream`      | POST   | configure a transmit stream                             |
| `/api/rx-stream`      | POST   | configure a receive stream                              |

The dashboard shows the active RX/TX streams and pre-fills the PTP form from the
current values. Stream config can't be read back from the write-only FPGA RAMs,
so `/api/streams` returns the daemon's **persisted** stream config (via a
`GetConfig` request — the live source of truth), while `/api/ptp` (GET) reads the
PTP grandmaster/interval CSRs directly.

The dashboard polls `/api/status` (and has forms for the three POST endpoints);
if the daemon is down the API returns a `503` JSON error and the page shows
"daemon unreachable". The config POSTs take a JSON body with the same fields as
the CLI's `ptp` / `tx-stream` / `rx-stream` commands (all optional except stream
`id`/`dst_ip`/`dst_port`); each returns `{"ok":bool,"message":...}`. Because they
go through the daemon's `ControlApi`, writes are persisted and replayed on an
FPGA reset, and new stream groups are picked up by the IGMP reconciler — exactly
like CLI writes.

```bash
curl -X POST http://<pi>:8080/api/ptp -d '{"priority1":128,"sync_interval":-3}'
curl -X POST http://<pi>:8080/api/rx-stream \
     -d '{"id":0,"dst_ip":"239.69.2.1","dst_port":5004,"ch_map":[0,1]}'
```

The HTML/JS is compiled into the binary, so the server is fully standalone (no
static files to ship). It has no native C deps, so it cross-compiles to the Pi
like the CLI. **Note:** the API is unauthenticated and now allows configuration
writes — serve it on a trusted network (or bind `--listen 127.0.0.1:8080` and
front it with a reverse proxy) rather than exposing it openly.

Crates: `aes67-transport` (HAL + bursts) · `aes67-proto` (wire protocol) ·
`aes67-config` (device/CSR/streams/`eth_buf` + `ControlApi`) · `aes67-client`
(`RemoteDevice`) · `aes67d` (daemon: control server + TAP bridge) · `aes67cfg`
(CLI) · `aes67web` (monitoring dashboard + REST API).

## Usage

Pick a transport, point at the CSR map (defaults to the bridge build output):

```bash
# List the register map (no device needed)
aes67cfg regs

# Over uartbone
aes67cfg --uart /dev/ttyUSB0 --baud 115200 dump
aes67cfg --uart /dev/ttyUSB0 get aes67_csr_status
aes67cfg --uart /dev/ttyUSB0 set aes67_csr_scratch 0xdeadbeef
aes67cfg --uart /dev/ttyUSB0 mac 02:00:00:12:34:56
aes67cfg --uart /dev/ttyUSB0 ip 192.168.1.42
aes67cfg --uart /dev/ttyUSB0 ptp --priority1 128 --clock-class 6
aes67cfg --uart /dev/ttyUSB0 ptp --sync-interval -3 --announce-interval 1

# Over spibone (Linux spidev)
aes67cfg --spi /dev/spidev0.0 --spi-speed 1000000 dump

# Different / explicit map
aes67cfg --csr ../litex_soc/build/aes67_bridge/csr.json --uart /dev/ttyUSB0 regs
```

Persistent configuration is handled by the daemon's JSON config (see
[Config file](#config-file-etcaes67djson)): settings made over the control API
are saved there and replayed on the next start, so there is no separate
profile-file mechanism.

### Streams

RX/TX audio streams live in two write-only config RAMs (`tx_stream_cfg` /
`rx_stream_cfg`, up to 8 streams each), not in CSRs — so they can be set but not
read back. Configure them one-off:

```bash
aes67cfg --uart /dev/ttyUSB0 tx-stream --id 0 --dst-ip 239.69.1.1 \
    --channels 2 --spp 48 --ch-ids 0,1 --ssrc 0x12345678
aes67cfg --uart /dev/ttyUSB0 rx-stream --id 0 --dst-ip 239.69.2.1 \
    --dst-port 5004 --ch-map 0,1 --channels 2 --delay 16 --spc 48
```

The byte layout matches the on-target firmware
(`drivers/eth_litex/eth_litex.c`); channel lists are capped at 8. When set
through the daemon, stream config is persisted and replayed like other settings.

Defaults: `--csr litex_soc/build/aes67_bridge/csr.csv` (relative to the current
directory — run from the repo root, or pass `--csr`), `--baud 115200`,
`--spi-speed 1000000`.

### Commands

| Command            | Effect                                                |
|--------------------|-------------------------------------------------------|
| `regs`             | List the CSR map (offline, no device)                 |
| `dump`             | Read every register                                   |
| `get <reg>`        | Read one register by name                             |
| `set <reg> <val>`  | Write one register by name                            |
| `peek <addr>`      | Read a raw word at an absolute byte address           |
| `poke <addr> <val>`| Write a raw word at an absolute byte address          |
| `mac [VALUE]`      | Get / set the MAC address                             |
| `ip [VALUE]`       | Get / set the IPv4 address                            |
| `scratch [VALUE]`  | Get / set the scratch register (round-trip link test) |
| `status`           | Read the AES67 status word                            |
| `ptp [--..]`       | Set PTP grandmaster params + sync/announce intervals  |
| `tx-stream --..`   | Configure a transmit (sender) audio stream            |
| `rx-stream --..`   | Configure a receive audio stream                      |

Numbers accept `0x` / `0b` / `0o` prefixes or plain decimal.

## Notes

* The SPI backend uses Linux `spidev`; on non-Linux hosts `--spi` returns an
  "unsupported" error and only `--uart` is available.
* The relevant gateware must expose the matching bridge. Generate it with:
  `python litex_soc/generate.py --target uartbone` (or `--target spibone`), and
  the AES67 register map with `--target aes67_bridge`.
