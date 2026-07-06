# Local Zephyr patches

Zephyr is pinned in `soc_firmware/app/west-manifest/west.yml` to a main
snapshot (see the `revision:` comment there). These patches are applied on
top; the canonical branch in `external/zephyr` is `aes67/main-20260703`.

After a fresh clone / `west update`, re-apply with:

    cd external/zephyr
    git checkout aes67/main-20260703   # or: git am ../../soc_firmware/zephyr-patches/*.patch

- 0001 spi_litex: local behaviour change (SPI_CS_ACTIVE_HIGH deasserts CS
  during transfer for SD-card init clocking; SPI_LOCK_ON ignored).
- 0002 net_if: use-after-free race in net_if_add_tx_timestamp (ref before
  fifo put) — candidate for upstream submission.
- 0003 ptp servo: step threshold 1 s → 5 ms via atomic ptp_clock_adjust()
  phase jump + anti-windup clamp on the PI integral. Without it the
  saturating wallclock rate correction (±524287 ppb) pins at the clamp
  and pi_drift winds up by millions of ppb after the boot-time step —
  candidate for upstream submission.
- 0004 ptp announce: header logMessageInterval carried logSyncInterval
  instead of logAnnounceInterval. Receivers size the foreign-master
  qualification window from it (4 × announce interval), so with AES67
  intervals (sync -3, announce 1) the clock never qualified anywhere:
  linuxptp saw but never selected it, Zephyr peers showed an all-zero
  foreign record and stayed LISTENING — candidate for upstream submission.
- 0005 ptp step: flush in-flight Delay_Reqs on a clock step. The epoch
  reset alone leaves a pre-step egress timestamp in flight; the first
  mean_delay of the new epoch absorbs the step size and the next Sync
  steps right back — a self-sustaining ±step limit cycle (observed at
  ±110 ms / 1 Hz) — candidate for upstream submission.

Dropped since the v4.2.0 era (now upstream): uptime-based message aging
(3ccd07b2), servo epoch reset after clock step, configurable PI servo gains
(9803584), ingress-timestamp validity guards. (The phase-jump step came back
as 0003 — upstream's epoch reset only steps above 1 s.)
