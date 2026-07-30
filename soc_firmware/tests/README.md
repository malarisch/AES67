# Firmware unit tests

Twister/ztest suites for the Zephyr control plane. They compile the real
application sources into a test image and run them on `qemu_riscv64` —
the same platform and toolchain as the CI boot test, so no extra SDK
target is needed.

```bash
source soc_firmware/.venv/bin/activate
export ZEPHYR_BASE=$PWD/external/zephyr

# everything (unit suites + the full-image boot test under app/)
west twister -T soc_firmware/tests -T soc_firmware/app -p qemu_riscv64 --inline-logs

# one suite
west twister -T soc_firmware/tests/parsers -p qemu_riscv64 --inline-logs

# one scenario, by name
west twister -p qemu_riscv64 -s aes67.unit.config
```

`sdk_qemu.cmake` points Zephyr's emulator lookup at the SDK hosttools in
`external/zephyr/toolchains/`, mirroring what the app's CMakeLists does.
Set `QEMU_BIN_PATH` yourself to override it.

## Suites

| Directory | Scenario | Covers |
| --- | --- | --- |
| `parsers/` | `aes67.unit.parsers` | `nmos_json` DOM parser, the REST `json_util` writer/parser, the shared SDP builder + parser, the IEEE 1588 clock-identity helpers |
| `config/` | `aes67.unit.config` | `aes67_config` defaults and derived identities (MAC, hostname, node id), the `config_json` persistence round trip, `aes67_conn` stream tables / observers / foreign-stream registry |
| `fpga_hal/` | `aes67.unit.fpga_hal` | the FPGA HAL mock backend (register file, wallclock snapshot protocol, status word), the `system_cfg` bitfield decoder, the PPB arithmetic in `fpga_regs` |
| `ptp_ctrl/` | `aes67.unit.ptp_ctrl` | the mode-invariant PTP status facade and its hardware backend (`ptp_ctrl_hw` + `ptp_bmc` state) |

Everything that needs FPGA registers links `drivers/fpga_hal` with
`CONFIG_FPGA_HAL_MOCK=y`, so the generated `aes67_bridge` CSR headers
must exist:

```bash
python litex_soc/generate.py --target aes67_bridge
```

The `fpga_hal` suite deliberately sets non-default
`CONFIG_FPGA_HAL_MOCK_*` stream/channel counts, so the `system_cfg`
assertions cannot pass by coincidence.

## JSON

Config persistence (`config_json.c`) and every REST request body are
decoded with Zephyr's descriptor-driven JSON library
(`CONFIG_JSON_LIBRARY`), so a suite that touches either needs that
symbol in its `prj.conf`. Two consequences show up in the tests:

* **Documents must live in writable memory.** The parser
  NUL-terminates tokens in place while decoding them (and restores the
  bytes afterwards), so tests pass `char body[] = "..."`, never a string
  literal.
* **Absent keys are not written.** Pre-fill the target struct with the
  current values and parse over it — that is how the handlers implement
  PATCH semantics. Where presence itself is the decision, the parser's
  return value is a bitmap indexed by descriptor position.

The response writer (`jo_*`) stays hand-rolled: NMOS and REST documents
are assembled at runtime with a variable number of resources, which a
descriptor-based encoder cannot express. `nmos_json.c` likewise keeps
its own DOM — see its header comment.

## Notes for new suites

* Add a directory with `CMakeLists.txt`, `prj.conf`, `testcase.yaml` and
  `src/`; include `../sdk_qemu.cmake` before `find_package(Zephyr)`.
* To use application Kconfig symbols (the FPGA HAL backends), add a
  `Kconfig` that does `rsource "../../app/drivers/Kconfig"` followed by
  `source "Kconfig.zephyr"`.
* Module state is global and lives for the whole image: reset it in the
  suite's `before` hook rather than assuming a fresh start per test.
* Test case names appear in the CI log — keep them descriptive of the
  behaviour, not of the function name.
