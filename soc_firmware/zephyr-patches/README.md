# Local Zephyr patches (alright to call it "AI slop")

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
- 0006 ptp filter: linuxptp-style moving-median prefilters on both servo
  inputs — E2E path delay (CONFIG_PTP_PATH_DELAY_FILTER_LENGTH, default 9)
  and offset (CONFIG_PTP_OFFSET_FILTER_LENGTH, default 5, applied before
  the step check so a single bad timestamp can't step the clock). Filters
  reset on clock step / servo reset; length 1 disables — candidate for
  upstream submission.
- 0007 ptp servo: adaptive linear-regression servo (port of linuxptp's
  linreg.c) as Kconfig alternative to the PI controller
  (CONFIG_PTP_SERVO_LINREG; default stays CONFIG_PTP_SERVO_PI, whose
  KP/KI options now depend on it). Least-squares fit over 4–64 samples,
  window chosen by smallest long-term prediction error — no gain tuning,
  averaging adapts to measurement noise. Stepping/outlier logic stays
  outside the servo; ptp_clock_sync_interval() feeds the
  timeTransmitter's Sync interval to the phase-correction term. Fed the
  RAW offset, not the 0006 median — the median's ~2-sample group delay
  in the loop drives linreg into a full-scale ±524287 ppb limit cycle
  (seen on HW, confirmed in sim); the median still guards step/outlier
  checks — candidate for upstream submission.
- 0008 esp32s3: re-init esp_flash default chip after PSRAM init.
- 0009 ptp servo: linreg samples weighted à la linuxptp tsproc
  filter_weight (weight = filtered/raw path delay from current Sync +
  last Delay_Req) and DROPPED below weight 0.7 (max 8 in a row,
  starvation guard). Queuing delay is one-sided; without this the
  slightest cross traffic pulled the locked clock ~10 µs (weighting
  alone in sim: ~17 µs; with drop gate: ~35 ns) — candidate for
  upstream submission.
- 0010 ptp msg: drop unparseable/foreign-version messages instead of
  faulting the port. Any post_recv error raised PTP_EVT_FAULT_DETECTED,
  so PTPv1 traffic (Dante shares 224.0.1.129:319/320) kept the port
  permanently FAULTY (err-log spam per packet); also a trivial remote
  DoS. Version now checked first, -EPROTONOSUPPORT at dbg level,
  parse errors → drop (linuxptp: EV_NONE) — candidate for upstream
  submission.
- 0011 ptp servo: robust offset outlier gate + linreg phase-slew limit.
  The 0009 delay-weight gate only catches queuing spikes (raw delay
  rises); a corrupted timestamp that shifts the offset WITHOUT raising
  the delay passes it and poisons up to 64 linreg fits — observed on one
  board as periodic ~10 s bursts of ±200k ppb at 8 Hz sync while an
  identical board stayed clean. New gate rejects samples deviating
  > NSIGMA (8) × tracked-EWMA-noise (floor 5 µs) from the moving median;
  median still absorbs rejected samples (genuine level change re-opens
  the gate), escape hatch after 16 consecutive rejects. Additionally
  CONFIG_PTP_SERVO_PHASE_CORR_MAX_PPB (default 0 = off; AES67 ESP32
  conf: 2000) clamps the linreg intercept term for media-friendly
  bounded frequency excursions — candidate for upstream submission
  (gate part).
- 0012 ptp step: acquisition phase trim via atomic phase jump. Offsets
  below the 5 ms step threshold but far from zero were left to the
  servo to slew out — minutes with the 2000 ppb media clamp from 0011.
  Two observed cases: the coarse get/compute/set boot-epoch step ages by
  the SPI round-trip and lands ~800 µs off; a warm FPGA wallclock
  surviving an ESP32 reboot starts ~300–500 µs off with no step at all.
  Design: trim BUDGET (2 jumps), granted at boot and by every genuine
  step, consumed per trim, dropped once a full median lands ≤ threshold.
  Deliberately NOT re-armed by servo reset, and threshold 10 µs > one
  median-refill of free-run drift (~5 ppm × 1 s): a self-re-arming trim
  with a 1 µs threshold retriggered on its own drift — permanent ~1.7 µs
  step loop at 1 Hz, servo pinned at 0 ppb. Median-full gating keeps
  single corrupted timestamps from triggering bogus jumps — candidate
  for upstream submission.
- 0008 esp32s3 soc: re-init the esp_flash default chip after PSRAM init.
  PSRAM timing tuning (octal @ 80 MHz) switches the shared MSPI core
  clock 80→160 MHz after esp_flash_config() captured the old source
  frequency — every SPI1 command transaction then runs at 160 MHz, all
  esp_flash erases/writes fail the WEL check with ESP_ERR_NOT_FOUND
  (261) while XIP keeps working. ESP-IDF inits the chip after PSRAM for
  this reason — candidate for upstream submission.

- 0013 net_if: use a link-local IPv4 address as the last-resort source
  address regardless of CONFIG_NET_IPV4_AUTO. The firmware configures its
  own 169.254.x.y fallback (MAC-derived, added by main.c) instead of using
  Zephyr's autoconf, because NET_IPV4_AUTO force-selects NET_IPV4_ACD and
  ACD has a lock inversion: ipv4_acd_timeout() holds the ACD mutex while
  calling net_if_ipv4_acd_succeeded()/net_if_send_data() (which take
  net_if_lock), while net_if_ipv4_addr_rm() holds net_if_lock and calls
  net_ipv4_acd_cancel() (which takes the ACD mutex). Any address removal
  concurrent with an ACD timer tick — a link-down carrier teardown or a
  net_dhcpv4_stop() from another thread — deadlocks both threads with no
  panic and no log output. Without this patch the self-configured address
  would only be usable for link-local destinations; everything multicast
  (mDNS, SAP, PTP) would go out from 0.0.0.0 — candidate for upstream
  submission.

- 0014 ptp: marshal link events to the PTP thread. port_link_monitor()
  (net_mgmt thread) ran ptp_port_event_handle(FAULT_DETECTED) directly on
  link-down: port_disable() -> zsock_close() on the sockets the PTP thread
  was sleeping on in zsock_poll(-1) — freeing net_contexts still registered
  with the poller (k_poll wait-list use-after-free). On the ESP32-S3 this
  froze the whole system silently (no fault; the K_PRIO_COOP(1) PTP thread
  starves everything preemptible), triggered by pulling the cable while
  TIME_TRANSMITTER. Now the monitor records the event atomically and wakes
  the PTP thread (ptp_clock_signal_timeout); the thread handles it at the
  top of its loop, so sockets are only closed/reopened by their poller —
  candidate for upstream submission.

- 0015 ptp: re-arm the acquisition trim (0012) when a full-median offset
  parks beyond 100 us for 16 consecutive samples. Steady-state offsets of
  hundreds of us (drift across a FAULTY/link-down phase, GM wander, peer
  restart) sit below the 5 ms step threshold with no trim budget left, so
  only the 2000 ppb media-clamped slew remained: 0.5-1 ms took 4-8
  minutes (observed parked at ~500 us, ppb frequency-matched). Hysteresis
  10 us disarm / 100 us re-arm (both >> ~5 us median-refill drift) plus
  the 16-sample streak keep the 0012 self-retrigger loop impossible —
  candidate for upstream submission.

Dropped since the v4.2.0 era (now upstream): uptime-based message aging
(3ccd07b2), servo epoch reset after clock step, configurable PI servo gains
(9803584), ingress-timestamp validity guards. (The phase-jump step came back
as 0003 — upstream's epoch reset only steps above 1 s.)
