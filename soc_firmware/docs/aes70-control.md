# AES70 (OCA) remote control

The Zephyr firmware exposes the whole node as an **AES70-2023 Device**:
an OCP.1 binary-protocol server (AES70-3) over TCP, advertised through
DNS-SD as `_oca._tcp`, with a fixed object tree that any AES70 controller
(AES70 Explorer, OCA.js based tools, console firmware) can enumerate
from the root block.

Sources: `soc_firmware/app/src/aes70/`

| File | Purpose |
|---|---|
| `ocp1.[ch]` | Wire format: big-endian marshaling, OcaString/Blob/List, PDU header, `OcaStatus` codes. Pure data code. |
| `aes70_model.h` | Object model types (kinds = classes, objects, sessions, subscriptions) and the core API. |
| `aes70_core.c` | Transport-agnostic session core: command dispatch, OcaRoot/OcaWorker/OcaAgent/OcaBlock methods, locks, OcaSubscriptionManager (EV2 + deprecated EV1), PropertyChanged detection, keep-alive. |
| `aes70_device.c` | *This* device's object tree: device manager, card channels, stream slots, PTP clock, network. |
| `aes70_server.c` | TCP listener/session thread, `aes70` shell commands, public `aes70_start()`. |

Unit tests: `soc_firmware/tests/aes70/` (twister, `qemu_riscv64`), covering the
marshaling and the core through the same byte-level session API the TCP
server uses.

## Spec sources

`external/aes70/` holds AES70-1 (framework), AES70-2 (class structure
overview) and AES70-3 (OCP.1). The normative class definitions with the
method/property indices are in **AES70-2A** (the UML model), which is not in
the repo; the indices used here were cross-checked against DeusO's rendering
of that model (<https://docs.deuso.de/AES70-OCC/>). Everything class-related
is concentrated in the `oca_kind` tables of `aes70_device.c` and the
`*_method()` functions of `aes70_core.c`, so a correction is a one-line
change.

## Protocol

* **Transport:** TCP (AES70-3 8.4.3.1), port `CONFIG_AES70_PORT` (default
  65000, IANA dynamic range as required). Up to `CONFIG_AES70_MAX_SESSIONS`
  controllers. Every session gets its own `oca_session` (receive
  reassembly buffer, subscriptions, keep-alive state); all sessions are
  served by one thread with `zsock_poll`.
* **PDUs:** Command / CommandRrq. Command PDUs are consumed **one command
  at a time** as the bytes arrive, so a PDU may be larger than the receive
  buffer (AES70 Explorer sends its whole discovery as one ~90-command,
  1.5 KB PDU). Responses of the commands handled in one read are batched
  into one Response PDU (same handles); a response that does not fit next
  to the earlier ones opens a new PDU, KeepAlive (both the 16-bit seconds and the
  32-bit milliseconds form), Notification EV2 (`Ntf2`) and, for older
  controllers, EV1 (`Ntf1`). A wrong sync byte or an oversized PDU closes
  the connection, as the standard demands.
* **Protocol version:** the header field is accepted with any non-zero
  value. AES70-3 mandates 1, but AES70.js (AES70 Explorer) writes its
  edition number (4 for 2024) and ignores the field on receive; the device
  always answers with 1.
* **Keep-alive (6.4):** after the first KeepAlive from the controller the
  device answers it, sends one per heartbeat interval when it has nothing
  else to say, and drops the connection after three silent intervals.
  Locks and subscriptions of the connection are released.
* **Locks:** `SetLockNoWrite` / `SetLockNoReadWrite` / `Unlock` /
  `GetLockState` per object, owner = session. Actuators and the device
  manager are lockable; sensors and blocks report `Lockable = false`.
* **Notifications:** Normal delivery mode only (same TCP connection).
  Lightweight (UDP) subscriptions are answered with `NotImplemented`.
  Device Reset (`SetResetKey`) is not implemented (6.5.2).

### PropertyChanged without hooks

Rather than instrumenting every write path (web API, shell, NMOS IS-05,
card settings restore, PTP state machine), the core keeps a **snapshot of
the notifiable properties of every subscribed object** and re-reads them
every 250 ms (immediately when an observer flags a change, and right after
a Set over AES70). A difference emits `PropertyChanged` to all matching
subscriptions. The cost is one getter call per subscribed property per
scan; the getters are cheap (driver shadow registers, stream tables, PTP
status struct).

## Object tree

Object numbers are fixed so controllers can address objects directly.

```
1     OcaDeviceManager        3  OcaFirmwareManager
4     OcaSubscriptionManager  7  OcaMediaClockManager
100   Root block (OcaBlock)
 ├─1000 Inputs                     (MI / IO card only)
 │   └─1100+10n  "In n+1"  +1 OcaGain Gain (preamp dB)  +2 OcaMute  +3 OcaBooleanActuator Phantom
 ├─2000 Outputs                    (LO / IO card only)
 │   └─2100+10n  "Out n+1" +1 OcaGain Level (clip dB)   +2 OcaMute
 ├─3000 Streams
 │   ├─3100 TX ── 3110+10s "TX s+1": +1 Active +2 Name +3 Destination +4 Channels +5 SamplesPerPacket
 │   └─3200 RX ── 3210+10s "RX s+1": +1 Active +2 Name +3 Address +4 ChannelMap +5 OutputDelay +6 SamplesPerChannel
 ├─4000 Clock
 │   4001 OcaTimeSource PTP   4002 OcaMediaClock3 MediaClock
 │   4010 Role  4011 OffsetNs  4012 PathDelayNs  4013 Locked  4014 Grandmaster   (sensors)
 │   4020 Domain  4021 Priority1  4022 Priority2                                  (OcaUint8Actuator)
 ├─5000 Network
 │   5001 IPAddress  5002 MACAddress  5003 Hostname (sensors)   5010 SAPAnnounce (actuator)
 └─6000 Auracast                      (Auracast card only)
     6001 Enabled  6002 Name  6003 Code  6004 Preset  6005 Channels  6006 Context
     6007 ImmediateRendering  6008 TxPowerDbm  6009 PresentationDelayUs  6010 FifoTarget
     6011 BroadcastID ("random" or hex)  6012 HQPreset (255 = none)  6013 HQChannels  6014 HQContext
     6020 State  6021 Error  6022 Streaming  6023 ActiveBroadcastID  6024 FifoFill  6025 DriftCppm (sensors)
```

* **Card channels** map onto the card drivers (`mi_card` / `lo_card` /
  `io_card`) exactly like the web API does; writes schedule the debounced
  `card_settings` persist. The blocks are created from the card detected at
  `aes70_start()` (after `card_manager_init()`).
* **Streams** use generic actuators because the AES70-2023 media-transport
  classes (`OcaMediaTransportApplication` & co.) are large and not yet
  supported by the common controllers. Editing an *inactive* slot stores the
  fields in a shadow; setting `Active = true` applies them through
  `aes67_conn_configure_*_stream()`. Editing an *active* slot reconfigures
  it immediately. `Channels`/`ChannelMap` are comma-separated lists,
  `Address` is `ip:port`, `Destination` the multicast IP (TX port is the
  configured default).
* **Auracast card:** every field of `struct auracast_config` is an actuator;
  a Set reads the stored config, patches the field, validates it against the
  protocol limits (`aura_proto_config_validate`) and applies it with
  `auracast_card_set_config()` (restarts the broadcast only when needed,
  as the REST API does), then schedules the debounced persist. Sensors
  mirror the driver's status cache (state, last error, streaming flag,
  active broadcast ID, FIFO fill, drift).
* **Clock:** `OcaTimeSource` reports IEEE 1588-2008 delivery, availability
  and sync status from `ptp_ctrl`, `GetReferenceID` the grandmaster identity,
  `GetTimeDeliveryParameters` a JSON record with the domain.
  `OcaMediaClock3` reports the configured sample rate and the lock state.
  Domain / priorities write the device config and re-apply it to the
  running PTP stack.
* **Device manager:** serial, device name (settable, persisted, re-announced
  via mDNS), model description, manufacturer / product records, operational
  state, managers list. Role, inventory code and message are RAM only.

## Discovery

`mdns_sd.c` adds `<vendor node id>._oca._tcp.local` with the TXT records
`txtvers=1`, `protovers=3` (AES70-3 8.5.5) and lists it in the
`_services._dns-sd._udp` enumeration.

## Limits and knobs (Kconfig, menu "AES70 (OCA) control")

`CONFIG_AES70` (default y), `AES70_PORT`, `AES70_MAX_SESSIONS` (4),
`AES70_MAX_SUBSCRIPTIONS` per session (64), `AES70_MAX_OBJECTS` (256; a
16-in/8-out card plus 8+8 stream slots needs ~210), `AES70_RX_BUF_SIZE`
(1 KB per session), `AES70_TX_BUF_SIZE` (8 KB shared; recursive enumeration
of the whole tree is ~24 B/object), `AES70_STACK_SIZE` (6 KB).

Large tables (sessions, snapshots, TX buffer) carry `AES67_BIG_BSS` and
live in PSRAM on the ESP32-S3.

## Shell

```
aes70 status     # server state, connected controllers, per-session counters
aes70 objects    # dump the object tree (ONo, owner, class, role)
```

## Not done / open

* Hardware test against a real controller (AES70 Explorer) is still
  outstanding; the unit tests cover marshaling, dispatch, locks,
  subscriptions, notifications and keep-alive.
* `OcaBlock.FindActionObjectsByRole*` marshals only the flagged result
  fields (ONo=1, ClassIdentification=2, ContainerPath=4, Role=8, Label=16);
  verify against AES70-2A if a controller complains.
* Card hot-swap (`card_manager_rescan()` with a *different* card type) does
  not rebuild the tree.
* No UDP / WebSocket / TLS transports, no Lightweight notifications, no
  Device Reset, no datasets/logging.
