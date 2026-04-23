cmake_minimum_required(VERSION 3.20.0)

# Allow the in-tree board variants under boards/aes67_soc/ to be found
# alongside upstream Zephyr boards.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

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

    # Re-run cmake if csr.json or the generator script changes.
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${LITEX_CSR_JSON}"
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_litex_overlay.py")

    message(STATUS "AES67: LiteX target = ${_litex_target}")
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
# The networking / HTTP / mDNS / RTSP stacks only exist on the LiteX boards.
# The ESP32 SPI-bring-up build uses a much smaller main that just reads
# FPGA status over SPI.
if(CONFIG_NETWORKING)
    # HTTP server: register per-service resource linker sections.
    zephyr_linker_sources(SECTIONS sections-rom.ld)
    zephyr_linker_section(NAME http_resource_desc_aes67_http
                          KVMA RAM_REGION GROUP RODATA_REGION)

    # Generate gzipped web UI include at build time.
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
      src/card_manager.c
    )

    target_sources_ifdef(CONFIG_SPI_FLASH_LITESPI app PRIVATE src/fw_update.c)
    target_sources_ifdef(CONFIG_RTSP              app PRIVATE src/rtsp.c)
    target_sources_ifdef(CONFIG_MDNS_SD           app PRIVATE src/mdns_sd.c)
    target_sources_ifdef(CONFIG_SD_CONFIG         app PRIVATE src/sd_config.c)
    target_sources_ifdef(CONFIG_FLASH_CONFIG      app PRIVATE src/flash_config.c)

    add_dependencies(app web_assets)
else()
    # SPI-only build (external-MCU targets).
    target_sources(app PRIVATE src/main_spi_bringup.c)
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
