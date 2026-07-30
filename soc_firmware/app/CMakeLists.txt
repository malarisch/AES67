cmake_minimum_required(VERSION 3.20.0)

# Allow the in-tree board variants under boards/aes67_soc/ to be found
# alongside upstream Zephyr boards.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# QEMU boards (CI / FPGA-HAL-mock builds): the qemu-system-* binaries come
# from the SDK hosttools (install once via
# toolchains/zephyr-sdk-1.0.1/hosttools/*.sh). Point Zephyr's emulator
# resolution at them unless the caller already provides a QEMU.
if(NOT DEFINED ENV{QEMU_BIN_PATH})
    set(_sdk_qemu_bin
        "${CMAKE_CURRENT_SOURCE_DIR}/../../external/zephyr/toolchains/zephyr-sdk-1.0.1/sysroots/x86_64-pokysdk-linux/usr/bin")
    if(EXISTS "${_sdk_qemu_bin}")
        set(ENV{QEMU_BIN_PATH} "${_sdk_qemu_bin}")
    endif()
endif()

# ---------- LiteX per-target CSR dtsi regeneration ----------
# Each LiteX board variant has a checked-in `litex_vexriscv_<target>.dts`
# that `#include`s `<target>_csr.dtsi`. We regenerate that dtsi from the
# matching `litex_soc/build/<target>/csr.json` whenever cmake runs, so
# adding/removing CSRs in LiteX flows through to the DT without manual
# edits. Non-LiteX boards (ESP32, …) skip this block entirely.
set(_litex_target "")
if(BOARD STREQUAL "litex_vexriscv_cyclone10")
    set(_litex_target "cyclone10")
elseif(BOARD STREQUAL "litex_vexriscv_cyc1000")
    set(_litex_target "cyc1000")
endif()

if(_litex_target)
    set(LITEX_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../litex_soc/build/${_litex_target}")
    set(LITEX_CSR_JSON  "${LITEX_BUILD_DIR}/csr.json")
    if(NOT EXISTS "${LITEX_CSR_JSON}")
        message(FATAL_ERROR
            "LiteX csr.json not found at ${LITEX_CSR_JSON}.\n"
            "Run: cd litex_soc && python generate.py --target ${_litex_target}")
    endif()

    set(LITEX_BOARD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/boards/aes67_soc/litex_vexriscv_${_litex_target}")
    execute_process(
        COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_litex_overlay.py"
                --csr-json "${LITEX_CSR_JSON}"
                --target   "litex_vexriscv_${_litex_target}"
                --out-dir  "${LITEX_BOARD_DIR}"
        RESULT_VARIABLE _gen_rc
    )
    if(NOT _gen_rc EQUAL 0)
        message(FATAL_ERROR "gen_litex_overlay.py failed (rc=${_gen_rc})")
    endif()

    # ---------- Merge CPU + AES67-bridge generated CSR headers ----------
    # The Wishbone-compliance refactor split the design into two LiteX SoCs:
    # the CPU SoC (this target) and the standalone aes67_bridge.  Each emits its
    # own generated/{csr,mem,soc}.h, but the firmware needs registers from BOTH
    # (CPU-local SPIFLASH/timer/... here, the AES67 surface in the bridge).  The
    # bridge is mapped into the CPU address space at the same absolute addresses
    # its headers use, so we merge them into one include tree the firmware
    # consumes via <generated/csr.h>.
    set(LITEX_BRIDGE_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/../../litex_soc/build/aes67_bridge")
    set(LITEX_BRIDGE_JSON "${LITEX_BRIDGE_DIR}/csr.json")
    if(NOT EXISTS "${LITEX_BRIDGE_JSON}")
        message(FATAL_ERROR
            "LiteX aes67_bridge csr.json not found at ${LITEX_BRIDGE_JSON}.\n"
            "The AES67 CSRs now live in the standalone bridge SoC.\n"
            "Run: cd litex_soc && python generate.py --target aes67_bridge")
    endif()

    set(LITEX_GENERATED_INC "${CMAKE_CURRENT_BINARY_DIR}/litex_merged")
    execute_process(
        COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/scripts/merge_litex_csr.py"
                --cpu-inc    "${LITEX_BUILD_DIR}/software/include"
                --bridge-inc "${LITEX_BRIDGE_DIR}/software/include"
                --out-inc    "${LITEX_GENERATED_INC}"
        RESULT_VARIABLE _merge_rc
    )
    if(NOT _merge_rc EQUAL 0)
        message(FATAL_ERROR "merge_litex_csr.py failed (rc=${_merge_rc})")
    endif()

    # Re-run cmake if either build's csr.json or a generator script changes.
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${LITEX_CSR_JSON}"
        "${LITEX_BRIDGE_JSON}"
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_litex_overlay.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/merge_litex_csr.py")

    message(STATUS "AES67: LiteX target = ${_litex_target} (CSRs merged with aes67_bridge)")
endif()

# ---------- Per-board DT overlay ----------
# Only the ESP32 build needs an overlay on top of the board DTS; the LiteX
# variants carry their full DT in-board.
if(BOARD MATCHES "^esp32s3_devkitc")
    set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/boards/esp32s3_devkitc.overlay")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
target_include_directories(app PRIVATE ${ZEPHYR_BASE}/subsys/net/lib)

project(app)

add_subdirectory(drivers)

# ---------- Application sources ----------
# The full control plane (HTTP / mDNS / RTSP / SD / card manager) builds
# for every HAL backend that has networking — LiteX and SPI (ESP32).
# Hardware that only exists on specific boards (Si5351A, analog cards,
# display controller, SPI flash, SD card) is gated inside main.c on the
# matching Kconfig symbols, so the SPI build simply leaves those out.
# The legacy bring-up stub is still selectable via CONFIG_APP_SPI_BRINGUP.
if(CONFIG_APP_SPI_BRINGUP)
    target_sources(app PRIVATE src/main_spi_bringup.c)
elseif(CONFIG_FPGA_HAL_LITEX OR CONFIG_FPGA_HAL_SPI OR CONFIG_FPGA_HAL_MOCK)
    # HTTP server: register per-service resource linker sections.
    zephyr_linker_sources(SECTIONS sections-rom.ld)
    # ESP32-S3 PSRAM: keep net-stack thread stacks in internal DRAM while the
    # packet pools live in PSRAM (content is #ifdef'd on the SPIRAM symbol).
    zephyr_linker_sources(SECTIONS sections-psram-stacks.ld)

    if(CONFIG_AES67_PTP_SOFTWARE)
      # ptp_ctrl's software backend reads the Zephyr PTP stack's internal
      # datasets — the public zephyr/net/ptp.h exposes no status/config API.
      target_include_directories(app PRIVATE ${ZEPHYR_BASE}/subsys/net/lib/ptp)
      target_sources(app PRIVATE src/ptp_ctrl_sw.c)
    endif()
    zephyr_linker_section(NAME http_resource_desc_aes67_http
                          KVMA RAM_REGION GROUP RODATA_REGION)

    # Generate the single-file React UI when its local npm dependencies are
    # available.  The generated gzip include is checked in as a fallback so a
    # normal firmware build does not need Node.js or network access.
    set(WEBUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/webui)
    set(WEB_SRC  ${CMAKE_CURRENT_SOURCE_DIR}/src/web/index.html)
    set(WEB_GZ   ${CMAKE_CURRENT_BINARY_DIR}/index.html.gz)
    set(WEB_INC  ${CMAKE_CURRENT_SOURCE_DIR}/src/index.html.gz.inc)

    if(EXISTS "${WEBUI_DIR}/node_modules/.package-lock.json")
      find_program(NPM_EXECUTABLE npm REQUIRED)
      file(GLOB_RECURSE WEBUI_SOURCES CONFIGURE_DEPENDS
        ${WEBUI_DIR}/src/*.ts
        ${WEBUI_DIR}/src/*.tsx
        ${WEBUI_DIR}/src/*.css
      )
      add_custom_command(
        OUTPUT  ${WEB_SRC}
        COMMAND ${NPM_EXECUTABLE} run build
        WORKING_DIRECTORY ${WEBUI_DIR}
        DEPENDS
          ${WEBUI_SOURCES}
          ${WEBUI_DIR}/index.html
          ${WEBUI_DIR}/package.json
          ${WEBUI_DIR}/package-lock.json
          ${WEBUI_DIR}/vite.config.ts
          ${WEBUI_DIR}/tsconfig.app.json
          ${WEBUI_DIR}/tsconfig.node.json
        COMMENT "Building single-file React web UI"
      )
    endif()

    if(EXISTS "${WEB_SRC}" OR EXISTS "${WEBUI_DIR}/node_modules/.package-lock.json")
      add_custom_command(
        OUTPUT  ${WEB_INC}
        COMMAND gzip -n -c -9 ${WEB_SRC} > ${WEB_GZ}
        COMMAND xxd -i < ${WEB_GZ} | sed "s/^unsigned.*$$//" | sed "/^$$/d" | sed "$$ s/}\\;$$//" > ${WEB_INC}
        DEPENDS ${WEB_SRC}
        COMMENT "Generating gzipped React web UI include"
      )
    else()
      message(STATUS
        "Web UI dependencies are not installed; using checked-in index.html.gz.inc")
    endif()

    set(DBG_SRC  ${CMAKE_CURRENT_SOURCE_DIR}/src/web/debug.html)
    set(DBG_GZ   ${CMAKE_CURRENT_BINARY_DIR}/debug.html.gz)
    set(DBG_INC  ${CMAKE_CURRENT_SOURCE_DIR}/src/debug.html.gz.inc)
    if(EXISTS "${DBG_SRC}")
      add_custom_command(
        OUTPUT  ${DBG_INC}
        COMMAND gzip -c -9 ${DBG_SRC} > ${DBG_GZ}
        COMMAND xxd -i < ${DBG_GZ} | sed "s/^unsigned.*$$//" | sed "/^$$/d" | sed "$$ s/}\\;$$//" > ${DBG_INC}
        DEPENDS ${DBG_SRC}
        COMMENT "Generating gzipped debug page include"
      )
    endif()

    add_custom_target(web_assets DEPENDS ${WEB_INC} ${DBG_INC})

    target_sources(app PRIVATE
      src/main.c
      src/fpga_regs.c
      src/fpga_poll.c
      src/ui_display.c
      src/ptp_bmc.c
      src/ptp_ctrl.c
      src/ptp_ctrl_hw.c
      src/aes67_conn.c
      src/sap_sdp.c
      src/aes67_shell.c
      src/aes67_config.c
      src/config_json.c
      src/aes67_sdp_utils.c
      src/webapi/json_util.c
      src/webapi/webapi_core.c
      src/webapi/webapi_network.c
      src/webapi/webapi_fpga.c
      src/webapi/webapi_ptp.c
      src/webapi/webapi_streams.c
      src/webapi/webapi_config.c
      src/webapi/webapi_system.c
      src/webapi/webapi_cards.c
    )
    target_sources_ifdef(CONFIG_DISPLAY_CTRL app PRIVATE src/webapi/webapi_display.c)

    # card_manager itself is portable — it only pulls in the per-card
    # drivers (mi/lo/io) when those Kconfigs are set, so it's safe to
    # always compile. card_settings persists gain/48V/mute through the
    # config_json backends and is likewise Kconfig-gated internally.
    target_sources(app PRIVATE src/card_manager.c src/card_settings.c)
    # Si5351A PI controller is only meaningful on boards that ship the
    # clock generator. Boards without it (ESP32 SPI bring-up) skip the
    # PPB feedback loop entirely; pll_ctrl is stubbed in pll_ctrl.h.
    target_sources_ifdef(CONFIG_SI5351A    app PRIVATE src/pll_ctrl.c)

    target_sources_ifdef(CONFIG_SPI_FLASH_LITESPI app PRIVATE src/fw_update.c)
    target_sources_ifdef(CONFIG_RTSP              app PRIVATE src/rtsp.c)
    target_sources_ifdef(CONFIG_MDNS_SD           app PRIVATE src/mdns_sd.c)
    target_sources_ifdef(CONFIG_NMOS              app PRIVATE
      src/nmos/nmos_node.c
      src/nmos/nmos_api.c
    )
    target_sources_ifdef(CONFIG_NMOS_REGISTRATION app PRIVATE src/nmos/nmos_reg.c)
    target_sources_ifdef(CONFIG_NMOS_IS05 app PRIVATE src/nmos/nmos_is05.c)
    target_sources_ifdef(CONFIG_NMOS_IS08 app PRIVATE src/nmos/nmos_is08.c)
    if(CONFIG_NMOS_IS05 OR CONFIG_NMOS_IS08)
      target_sources(app PRIVATE src/nmos/nmos_json.c)
    endif()
    target_sources_ifdef(CONFIG_SD_CONFIG         app PRIVATE src/sd_config.c)
    target_sources_ifdef(CONFIG_FLASH_CONFIG      app PRIVATE src/flash_config.c)

    add_dependencies(app web_assets)
endif()

# ---------- Generate Flash Boot Image (.fbi) for LiteX targets ----------
# The LiteX BIOS expects an 8-byte header (length + CRC-32) before the
# firmware payload. This is only meaningful for the LiteX SoC boot flow.
if(_litex_target)
    set(MAKE_FBI    ${CMAKE_CURRENT_SOURCE_DIR}/../../dev_scripts/make_fbi.py)
    set(ZEPHYR_BIN  ${CMAKE_BINARY_DIR}/zephyr/zephyr.bin)
    set(FBI_STAMP   ${CMAKE_BINARY_DIR}/zephyr/zephyr.fbi)
    set(TARGET_BIN  ${CMAKE_BINARY_DIR}/zephyr/zephyr_${_litex_target}.bin)
    set(TARGET_FBI  ${CMAKE_BINARY_DIR}/zephyr/zephyr_${_litex_target}.fbi)

    add_custom_command(
      OUTPUT  ${FBI_STAMP} ${TARGET_BIN} ${TARGET_FBI}
      DEPENDS ${ZEPHYR_BIN}
      COMMAND ${PYTHON_EXECUTABLE} ${MAKE_FBI} ${ZEPHYR_BIN} ${FBI_STAMP}
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${ZEPHYR_BIN} ${TARGET_BIN}
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FBI_STAMP}  ${TARGET_FBI}
      COMMENT "Generating LiteX flash boot image (zephyr.fbi, zephyr_${_litex_target}.{bin,fbi})"
    )
    add_custom_target(fbi ALL DEPENDS ${FBI_STAMP} ${TARGET_BIN} ${TARGET_FBI})
endif()
