
cmake_minimum_required(VERSION 3.20.0)


set(BOARD "litex_vexriscv")

# ---------- Select LiteX SoC target ----------
# Pick which litex_soc/build/<target>/ directory provides csr.json, memory
# layout, CSR base addresses, etc.  Override with `-DLITEX_TARGET=cyc1000`.
if(NOT DEFINED LITEX_TARGET)
    set(LITEX_TARGET "cyclone10" CACHE STRING "LiteX SoC target (cyclone10|cyc1000|gowin)")
endif()
message(STATUS "AES67: building for LiteX target '${LITEX_TARGET}'")

set(LITEX_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../litex_soc/build/${LITEX_TARGET}")
set(LITEX_CSR_JSON  "${LITEX_BUILD_DIR}/csr.json")
if(NOT EXISTS "${LITEX_CSR_JSON}")
    message(FATAL_ERROR
        "LiteX csr.json not found at ${LITEX_CSR_JSON}.\n"
        "Run: cd litex_soc && python generate.py --target ${LITEX_TARGET}")
endif()

# ---------- Generate per-target DT overlay fragment ----------
# Runs at configure time so Zephyr sees the file before DTC.  Reading csr.json
# yields all target-specific base addresses (SDRAM, CSR peripherals, …) which
# get emitted as a short `&label { reg = <...>; }` overlay that is appended
# AFTER the static overlay — overriding its hardcoded addresses.
set(LITEX_GEN_DIR     "${CMAKE_CURRENT_BINARY_DIR}/litex_gen")
set(LITEX_GEN_OVERLAY "${LITEX_GEN_DIR}/litex_soc_generated.overlay")
execute_process(
    COMMAND ${CMAKE_COMMAND} -E make_directory "${LITEX_GEN_DIR}"
)
execute_process(
    COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_litex_overlay.py"
            --csr-json "${LITEX_CSR_JSON}"
            --out-dir  "${LITEX_GEN_DIR}"
    RESULT_VARIABLE _gen_rc
)
if(NOT _gen_rc EQUAL 0)
    message(FATAL_ERROR "gen_litex_overlay.py failed (rc=${_gen_rc})")
endif()

# Re-run cmake if csr.json or the generator script changes.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${LITEX_CSR_JSON}"
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_litex_overlay.py")

# Static overlay (baseline), generated fragment wins via EXTRA_DTC_OVERLAY_FILE
# which Zephyr applies after DTC_OVERLAY_FILE.
set(DTC_OVERLAY_FILE       "${CMAKE_CURRENT_SOURCE_DIR}/litex_vexriscv.overlay")
set(EXTRA_DTC_OVERLAY_FILE "${LITEX_GEN_OVERLAY}")
#set(CONFIG_BOOTLOADER_MCUBOOT y)
#set(DOPENOCD /home/wolff/openocd-esp32/openocd-esp32/bin/openocd)
#set(DOPENOCD_DEFAULT_PATH /home/wolff/openocd-esp32/openocd-esp32/share/openocd/scripts)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
target_include_directories(app PRIVATE ${ZEPHYR_BASE}/subsys/net/lib)

project(app)

add_subdirectory(drivers)

# ---------- HTTP server: register per-service resource linker sections ----------
zephyr_linker_sources(SECTIONS sections-rom.ld)
zephyr_linker_section(NAME http_resource_desc_aes67_http
                      KVMA RAM_REGION GROUP RODATA_REGION)

# ---------- Generate gzipped web UI include at build time ----------
set(WEB_SRC  ${CMAKE_CURRENT_SOURCE_DIR}/src/web/index.html)
set(WEB_GZ   ${CMAKE_CURRENT_BINARY_DIR}/index.html.gz)
set(WEB_INC  ${CMAKE_CURRENT_SOURCE_DIR}/src/index.html.gz.inc)

add_custom_command(
  OUTPUT  ${WEB_INC}
  COMMAND gzip -c -9 ${WEB_SRC} > ${WEB_GZ}
  COMMAND xxd -i < ${WEB_GZ} | sed "s/^unsigned.*$$//" | sed "/^$$/d" | sed "$$ s/}\\;$$//" > ${WEB_INC}
  DEPENDS ${WEB_SRC}
  COMMENT "Generating gzipped web UI include"
)

# ---------- Debug page (display/LED tester) ----------
set(DBG_SRC  ${CMAKE_CURRENT_SOURCE_DIR}/src/web/debug.html)
set(DBG_GZ   ${CMAKE_CURRENT_BINARY_DIR}/debug.html.gz)
set(DBG_INC  ${CMAKE_CURRENT_SOURCE_DIR}/src/debug.html.gz.inc)

add_custom_command(
  OUTPUT  ${DBG_INC}
  COMMAND gzip -c -9 ${DBG_SRC} > ${DBG_GZ}
  COMMAND xxd -i < ${DBG_GZ} | sed "s/^unsigned.*$$//" | sed "/^$$/d" | sed "$$ s/}\\;$$//" > ${DBG_INC}
  DEPENDS ${DBG_SRC}
  COMMENT "Generating gzipped debug page include"
)

add_custom_target(web_assets DEPENDS ${WEB_INC} ${DBG_INC})

target_sources(app PRIVATE
  src/main.c
  src/fpga_regs.c
  src/pll_ctrl.c
  src/fpga_poll.c
  src/ui_display.c
  src/ptp_bmc.c
  src/sap_sdp.c
  src/aes67_config.c
  src/config_json.c
  src/aes67_sdp_utils.c
  src/webserver.c
  src/fw_update.c
  src/card_manager.c
)

# RTSP server/client (RAVENNA stream control)
target_sources_ifdef(CONFIG_RTSP app PRIVATE src/rtsp.c)

# RAVENNA mDNS/DNS-SD service advertisement
target_sources_ifdef(CONFIG_MDNS_SD app PRIVATE src/mdns_sd.c)

# SD card configuration storage (optional)
target_sources_ifdef(CONFIG_SD_CONFIG app PRIVATE src/sd_config.c)

# SPI flash configuration storage (optional)
target_sources_ifdef(CONFIG_FLASH_CONFIG app PRIVATE src/flash_config.c)

add_dependencies(app web_assets)

# ---------- Generate Flash Boot Image (.fbi) after build ----------
# The LiteX BIOS expects an 8-byte header (length + CRC-32) before the
# firmware payload.  This runs make_fbi.py on the raw zephyr.bin to
# produce zephyr.fbi, ready to flash at FLASH_BOOT_ADDRESS (0x010000).
set(MAKE_FBI    ${CMAKE_CURRENT_SOURCE_DIR}/../../dev_scripts/make_fbi.py)
set(ZEPHYR_BIN  ${CMAKE_BINARY_DIR}/zephyr/zephyr.bin)
set(FBI_STAMP   ${CMAKE_BINARY_DIR}/zephyr/zephyr.fbi)

# Per-target output copies so building multiple LITEX_TARGETs does not
# overwrite each other's artifacts.  Named `zephyr_<target>.{bin,fbi}`.
set(TARGET_BIN  ${CMAKE_BINARY_DIR}/zephyr/zephyr_${LITEX_TARGET}.bin)
set(TARGET_FBI  ${CMAKE_BINARY_DIR}/zephyr/zephyr_${LITEX_TARGET}.fbi)

add_custom_command(
  OUTPUT  ${FBI_STAMP} ${TARGET_BIN} ${TARGET_FBI}
  DEPENDS ${ZEPHYR_BIN}
  COMMAND ${PYTHON_EXECUTABLE} ${MAKE_FBI} ${ZEPHYR_BIN} ${FBI_STAMP}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${ZEPHYR_BIN}  ${TARGET_BIN}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FBI_STAMP}   ${TARGET_FBI}
  COMMENT "Generating LiteX flash boot image (zephyr.fbi, zephyr_${LITEX_TARGET}.{bin,fbi})"
)
add_custom_target(fbi ALL DEPENDS ${FBI_STAMP} ${TARGET_BIN} ${TARGET_FBI})
