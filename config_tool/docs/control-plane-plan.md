# Plan: AES67 control plane on an external Linux SoC

Status: **draft / design** — June 2026.

## 1. Goal

Move the **entire AES67 control plane** off the in-FPGA VexRiscv softcore onto an
external **Linux SoC** connected to the FPGA over **SPI (spibone) or UART
(uartbone)**. The FPGA keeps the hard-real-time **data plane** (Ethernet MAC,
PTP timestamping, RTP audio, media clock); Linux takes over everything the
Zephyr firmware does today (DHCP, PTP BMC, SAP/SDP, management, web UI).

Concretely we want:

1. A **Linux network interface** that bridges the FPGA's Ethernet (already on the
   Wishbone bus via `eth_buf`) into the Linux network stack — so standard Linux
   tooling (DHCP client, multicast/IGMP, sockets) "just works".
2. A **daemon** that exclusively owns the SPI/UART link and performs all
   Wishbone transactions (packet shuttling + register config).
3. The **CLI** becomes a thin client talking to the daemon over a **BSD/Unix
   socket** — modular enough that a **web UI** or, long-term, **ravennakit**
   integration can replace/augment it without touching the transport layer.

### Reference projects

* **bondagit/aes67-linux-daemon** — AES67 Linux daemon + WebUI (C++/REST). Closest
  existing architecture; differs in that it does audio in software (ALSA kernel
  module) while we keep audio in FPGA hardware. Good reference for the
  daemon/REST/WebUI split and the SAP/SDP + session model.
* **soundondigital/ravennakit** — C++17 RAVENNA/AES67/ST2110-30 SDK (AGPLv3) with
  NMOS IS-04/05, ST2022-7, PTPv2. Long-term integration target for discovery &
  connection management; the FPGA would be its audio transport backend.

## 2. End-state architecture

```
┌───────────────────────── Linux SoC ─────────────────────────┐
│                                                              │
│  aes67cfg (CLI)   web UI        ravennakit (future)          │
│       │             │                │                       │
│       └──── BSD/Unix socket (JSON-RPC control API) ──────────┤
│                          │                                   │
│                     ┌────┴───────────────── aes67d (daemon) ─┤
│                     │  control server  │  TAP bridge         │
│                     │  (CSR/streams)   │  (eth_buf <-> tapX)  │
│                     │        └────┬─────────┘                │
│                     │      transport arbiter (single owner)  │
│                     │             │                          │
│                     │     aes67-transport (SPI/UART)         │
│                     └─────────────┼──────────────────────────┤
│                                   │ SPI/UART  (+GPIO IRQ)     │
└───────────────────────────────────┼──────────────────────────┘
                                    │
┌───────────────────────────────────┼────────── FPGA ──────────┐
│            spibone / uartbone  (Wishbone master)              │
│                                   │                          │
│   aes67_bridge: aes67_csr │ eth_buf │ tx/rx_stream_cfg        │
│                              │                               │
│   Ethernet MAC ── RX classifier ──┬── audio RTP → HW ringbuf │
│        │                          └── other frames → eth_buf │
│   PTP timestamping · wallclock · media clock · RTP TX (HW)   │
└──────────────────────────────────────────────────────────────┘
```

Key boundary: the FPGA RX classifier **already** demuxes audio RTP into the
hardware ring buffer and sends everything else (DHCP, ARP, PTP, SAP/SDP, HTTP) to
`eth_buf`. So the TAP bridge only carries **control-plane** traffic — never
audio. This is what makes the slow SPI/UART link viable (see §4).

## 3. Division of responsibilities (what moves where)

| Function                        | Today            | Target                         |
|---------------------------------|------------------|--------------------------------|
| Ethernet MAC, PHY               | FPGA HW          | FPGA HW (unchanged)            |
| RTP audio TX/RX, media clock    | FPGA HW          | FPGA HW (unchanged)            |
| PTP sync/delay timestamping     | FPGA HW          | FPGA HW (unchanged)            |
| PTP servo (wallclock discipline)| FPGA HW          | FPGA HW (unchanged / static)  |
| PTP protocol (sync/announce/BMC)| FPGA + softcore  | **FPGA HW, autonomous** (§3a)  |
| DHCP / IP config                | Zephyr (lwip)    | **standard Linux (dhcpcd)**   |
| SAP/SDP announce + discovery    | Zephyr firmware  | **daemon module** (or reuse)  |
| Stream config (tx/rx_stream RAM)| Zephyr firmware  | **daemon → control API**      |
| CSR config / status             | Zephyr + FMC/HAL | **daemon → control API**      |
| Web UI / REST                   | Zephyr webserver | **daemon gateway / separate** |
| Audio packet relay to CPU       | eth_buf → Zephyr | **eth_buf → TAP → Linux**     |

**There is no LiteX/VexRiscv softcore on the FPGA when the external Linux SoC is
used.** That setup *is* the existing `aes67_bridge` target (CPU-less) driven by
spibone/uartbone. So the **sole Wishbone master is spibone/uartbone** — there is
**no two-master bus and no arbiter needed**. The softcore isn't "kept for PTP"
and isn't "removed later"; in this architecture it simply isn't built.

### 3a. PTP: stays in FPGA hardware, never moved off HW timestamping

**Now (default): PTP runs autonomously in the FPGA.** HW timestamping, the
sync/follow-up/announce/delay state machine, and the servo run in the FPGA data
plane with no software in the loop. The grandmaster selection that used to be the
firmware BMC is covered by **static PTP config** (the existing VHDL generic)
and/or the **daemon writing GM parameters via the control API** (the
`aes67_csr_ptp_gm_*` CSRs) — that is plain register configuration, **not** running
the PTP protocol on Linux. The TAP bridge may still *carry* PTP frames to Linux
for visibility, but the protocol logic does not move. A dynamic multi-master BMC,
if ever needed, belongs to the optional Phase 5 offload.

**Later (selectable option): PTP offload to the SoC.** A build/run option where
the FPGA keeps doing the **hardware timestamping** of the PTP frames it forwards
to the SoC, and the FPGA's wallclock takes only a **PPB frequency-correction
input** from the SoC. The PTP protocol engine + servo then run on Linux
(processing the HW-timestamped frames and computing the PPB correction), while
the FPGA remains the timestamp source and clock. This is an *option*, not a
migration step — the FPGA-resident PTP must keep working unchanged when it is
off.

## 4. The hard constraint: bus bandwidth

The SPI/UART link is a **single, slow Wishbone master path**, and `eth_buf` is
**1 byte per 32-bit word** — so moving a packet costs 4× its size on the bus,
plus per-word protocol overhead.

Rough budget (1500-byte frame, burst transfers):

* **UART @ 115200**: ~11.5 KB/s. A 1500-byte frame ≈ 1500 words. Even with bursts
  this is hundreds of ms/frame → only usable for very low control rates. Keep
  UART for **debug/bring-up**, not the network bridge.
* **SPI @ ~20 MHz**: ~2.5 MB/s raw → ~ms/frame → low-hundreds of frames/s. Fine
  for control traffic (DHCP, PTP Announce 1/s, SAP every few s, occasional HTTP).

Consequences baked into the plan:

* The bridge carries **control traffic only** (audio is in FPGA HW) → the rate is
  inherently low → SPI is sufficient.
* The transport HAL must gain **burst (block) read/write** so per-word address
  overhead is amortised (one `[cmd,len,addr]` then N words).
* **FPGA optimisation (optional, high value):** add a *packed* read/write port to
  `eth_buf` that returns 4 bytes per 32-bit word, cutting bus traffic 4×. Listed
  as an optimisation, not a prerequisite.
* MTU on the TAP interface can be lowered (e.g. 1500 is fine for control; jumbo
  not needed) and we should expose live throughput/queue stats for tuning.

## 5. Component / crate layout (Rust workspace under `config_tool/`)

Existing (reused):

* `aes67-transport` — SPI/UART Wishbone transport HAL. **+ add burst read/write.**
* `aes67-config` — `CsrMap`, `Device`, `Aes67Config`, `Aes67Streams`.
  Stays the in-process register/stream logic the **daemon** uses directly.
  (Persistent config now lives in the daemon's JSON state, not a TOML profile.)

New:

* `aes67-proto` — the **control API types** (requests/responses) shared by daemon
  and clients. `serde`-serialisable; this is the contract a web UI / ravennakit
  also targets. Versioned.
* `aes67d` — the **daemon** binary: owns the transport, runs the TAP bridge and
  the control server, multiplexes both onto the single bus.
* `aes67-client` — **client library**: connects to the daemon socket and exposes
  the same high-level API surface as `aes67-config::Device`, but over IPC. Lets
  the CLI (and any Rust consumer) be transport-agnostic.
* `aes67cfg` (existing CLI) — switch its default to talk to the daemon via
  `aes67-client`; keep a `--direct <spi|uart>` escape hatch for bring-up/debug
  when the daemon isn't running.

Abstraction that keeps it modular: define a `trait ControlApi` (read/write reg by
name, get/set MAC/IP/PTP, configure streams, reset, subscribe to status). Two
implementors:
* `aes67-config::Device` (direct transport) — used inside the daemon.
* `aes67-client::RemoteDevice` (over the socket) — used by the CLI.
The CLI code is written against `ControlApi` and doesn't care which it holds.

## 6. Daemon internals (`aes67d`)

Single owner of the transport; everything else is a client. Internally:

```
            ┌── control server (Unix socket, JSON-RPC) ──┐  N client conns
            │                                            │
  clients ──┤                                            │
            │   request queue ─┐                         │
            └──────────────────┤                         │
                               ▼                         │
   TAP fd ── rx/tx queues ── transport arbiter ──────────┘
                               │ (mutex / single task)
                          aes67-transport (SPI)
                               │
                          FPGA Wishbone
```

* **Transport arbiter**: the only code touching `aes67-transport`. A single async
  task (or a mutex-guarded handle) serialises three job kinds:
  1. eth_buf **RX drain** (triggered by GPIO IRQ or poll),
  2. eth_buf **TX inject** (TAP → FPGA),
  3. **control transactions** (CSR/stream from clients).
  Priority: keep RX latency low but never starve control; bounded queues + simple
  fairness. All jobs are short, run-to-completion bus transactions.
* **TAP bridge task**: owns the `tapX` fd. RX: on IRQ/poll, read `rx_len`, burst-
  read the frame, write to TAP, write `rx_ack`. TX: read frames from TAP, burst-
  write to `eth_buf` TX, set `tx_len`.
* **Control server task**: accepts Unix-socket connections, parses JSON-RPC,
  dispatches to `aes67-config::Device` through the arbiter, streams status/events
  back (for live dashboards).
* **RX notification**: prefer a **GPIO line** wired from `eth_buf_irq` to the
  Linux SoC (epoll on `/sys/class/gpio` or gpiod) for low-latency,
  interrupt-driven RX. Fallback: poll `eth_buf_rx_ready` at a configurable rate.
* **Config**: a small daemon config (TOML) — transport (spi/uart + device +
  speed), tap name/MTU, IRQ gpio, socket path, poll interval.
* Runs as a systemd service; the Unix socket has restricted permissions.

## 7. Control API (the modularity contract)

* **Transport**: Unix domain socket (`/run/aes67d.sock`). Local-only, fast,
  permission-controlled. (A TCP bind is a later option for remote UIs.)
* **Encoding**: **JSON-RPC 2.0**, newline- or length-framed. Rationale: trivially
  consumable from a web backend (any language), a C++ ravennakit module, shell
  (`socat`/`jq`), and Rust. Schema lives in `aes67-proto`, versioned with a
  `hello`/capabilities handshake.
* **Methods (initial)**:
  * `reg.list` / `reg.get {name}` / `reg.set {name,value}`
  * `net.get_mac` / `net.set_mac` / `net.get_ip` / `net.set_ip`
  * `ptp.get_status` / `ptp.set_grandmaster {...}`
  * `stream.tx_set {...}` / `stream.rx_set {...}` / `stream.list`
  * `reset {ptp|tx|rx|eth}`
  * `status.subscribe` → server-push status/metering/PTP events (for dashboards)
* The CLI's existing subcommands map 1:1 onto these methods → minimal CLI change.
* A later **HTTP/WebSocket gateway** (in `aes67d` or a sidecar) can expose the
  same methods as REST + WS for the web UI, reusing `aes67-proto`.

## 8. Network bridge details (TAP)

* **TAP (L2)**, not TUN: `eth_buf` carries raw Ethernet frames and the FPGA MAC is
  layer-2, so a TAP device is the natural match. Linux sees an Ethernet NIC.
* **MAC address**: set the FPGA MAC (`aes67_csr_mac_addr_*`) and the `tapX` MAC to
  the **same** address so the host stack and the FPGA agree. The daemon programs
  the FPGA MAC from the tap's MAC at startup.
* **Bring-up**: `aes67d` creates `tapX`, sets MAC/MTU, brings it up. Then standard
  Linux: `dhcpcd tapX` (or systemd-networkd) gets an IP; multicast joins for
  SAP/SDP and RTP control work through the normal stack.
* **Crates**: `tun`/`tappers` (Rust TAP), `tokio` for async I/O + sockets,
  `nix`/`gpiod` for the IRQ line.
* **Filtering**: optionally drop frames the FPGA already handles to save bus
  bandwidth (e.g. don't bridge audio multicast if the HW path owns it) — but the
  FPGA classifier should already keep those off `eth_buf`.

## 9. Phased roadmap

Each phase is independently testable and leaves the system working.

### Phase 0 — Foundations (no behaviour change) — ✅ DONE

* Add **burst read/write** to `aes67-transport` (uartbone & spibone burst framing;
  both protocols already support `CMD_*_BURST` + length).
* Create `aes67-proto` with the JSON-RPC request/response types + version
  handshake.
* Define the `ControlApi` trait in `aes67-config`; make `Device` implement it.
* *Deliverable*: workspace builds; CLI unchanged (still direct); bursts unit-
  tested against the mock transport.

### Phase 1 — Daemon + control API + CLI-as-client — ✅ DONE

* `aes67d`: own the transport, expose the Unix-socket JSON-RPC control server
  wrapping `Device` (reg/net/ptp/stream/reset).
* `aes67-client`: `RemoteDevice` implementing `ControlApi` over the socket.
* CLI: default to the daemon; keep `--direct`.
* *Deliverable*: `aes67cfg dump`/`set`/`apply`/`tx-stream` all work **through the
  daemon**. Modular IPC proven. (No networking yet.)

### Phase 2 — TAP network bridge — 🟡 IMPLEMENTED (pending hardware validation)
* TAP bridge threads share the `Arc<Mutex<Device>>` arbiter with the control
  server. RX wakes on the GPIO edge (`--irq-gpio CHIP:LINE`) with a poll-timeout
  safety net (`--poll-ms`); TX blocks on tap reads. `eth_buf` protocol
  (`rx_len`/FCS-strip/burst-read/`rx_ack`/`ev_pending`; TX burst + `tx_len`) is in
  `aes67-config::ethbuf` and **unit-tested with the mock bus**. TAP via
  `TUNSETIFF` + `ip link`; GPIO via `gpio-cdev`.
* Still to validate on hardware (needs root + FPGA + GPIO wiring): real frame
  flow, `dhcpcd` lease, throughput/latency, edge vs. poll behaviour.
* *Deliverable*: `tapX` exists; `dhcpcd tapX` obtains an IP through the FPGA MAC;
  `ping` and multicast work. Throughput/latency stats exposed via the API.

### Phase 3 — Control-plane services on Linux (PTP stays in FPGA)
* DHCP via standard Linux on `tapX`.
* **SAP/SDP** module: announce local streams, discover foreign ones, program the
  FPGA stream RAMs via the control API (reuse the byte layouts already in
  `aes67-config::stream`).
* **PTP is explicitly out of scope here** — it runs autonomously in the FPGA HW
  (§3a). The daemon only configures GM params / reads PTP status via CSR.
* *Deliverable*: a Linux SoC runs DHCP/SAP-SDP/stream/CSR/management; audio **and
  PTP** still run in the FPGA, untouched. The FPGA build is the CPU-less
  `aes67_bridge` with spibone/uartbone as the sole master.

### Phase 4 — Interfaces & ecosystem
* **Web UI**: HTTP/WebSocket gateway in `aes67d` (or sidecar) over `aes67-proto`;
  small SPA for config + live status/metering/PTP.
* **ravennakit**: integrate as the discovery/connection-management layer (NMOS
  IS-04/05, SDP), using the daemon/FPGA as the audio transport backend. AGPLv3 is
  a non-issue — this project is fully open source.
* *Deliverable*: browser-based config; path to NMOS/RAVENNA interop.

### Phase 5 (optional) — PTP offload to the SoC
* Selectable option (§3a): FPGA keeps HW-timestamping the forwarded PTP frames and
  exposes the timestamps to the SoC; the FPGA wallclock takes a **PPB correction
  input** from the SoC. The PTP protocol engine + servo + BMC run as a Linux
  daemon module; the FPGA stays the timestamp source and clock.
* Define the CSR/IRQ contract: per-frame HW timestamps out, PPB correction in.
* This is purely a PTP-location choice — the FPGA already has no softcore in this
  architecture; nothing else changes on the bus.
* *Deliverable*: PTP runs on Linux against FPGA HW timestamps; FPGA HW PTP can be
  reduced to the timestamp/PPB datapath.

## 10. Risks & open questions

* **Bus bandwidth / latency** for bridged control frames — quantify on real SPI
  early (Phase 2). Mitigations: SPI clock up, burst transfers, the 4-byte/word
  `eth_buf` FPGA optimisation, MTU tuning.
* **RX IRQ line**: does the chosen Linux SoC expose a free GPIO wired to
  `eth_buf_irq`? If not, polling adds latency/CPU — measure.
* **Single Wishbone master** (settled): with the external Linux SoC, the FPGA runs
  **no softcore** — it is the CPU-less `aes67_bridge` and spibone/uartbone is the
  only master. No multi-master arbiter, no bus contention to design around.
* **PTP stays in FPGA HW** (decided): no PTP logic moves to Linux until the
  optional Phase 5 offload. Confirm the FPGA's static/CSR PTP config is sufficient
  for grandmaster selection without the old firmware BMC in the target topology.
* **Phase 5 PTP contract**: when the offload option is built, define how per-frame
  HW timestamps reach the SoC and how the PPB correction returns to the wallclock.
* **Which Linux SoC**: needs SPI (fast), a spare GPIO (IRQ), enough CPU; a Pi
  works for bring-up. Decide target platform before Phase 2.
* **Security**: Unix-socket permissions; if a TCP/HTTP gateway is added, authn/z.
* **ravennakit licensing**: AGPLv3 is fine — this project is fully open source, so
  Phase 4 integration has no licensing blocker.

## 11. Immediate next step

Phase 0 is low-risk and unblocks everything: **burst transfers in
`aes67-transport`** + the **`aes67-proto`** crate + the **`ControlApi`** trait.
None of it changes current behaviour, and it makes the daemon a thin assembly
job in Phase 1.
