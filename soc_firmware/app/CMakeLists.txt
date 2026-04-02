# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
if(NOT flash-16M IN_LIST SNIPPET)
    #set(SNIPPET flash-16M ${SNIPPET} CACHE STRING "" FORCE)
endif()
if(NOT psram-8M IN_LIST SNIPPET)
    #set(SNIPPET psram-8M ${SNIPPET} CACHE STRING "" FORCE)
endif()


#set(BOARD "esp32s3_devkitc/esp32s3/procpu")
#set(BOARD "nucleo_h753zi")
set(BOARD "litex_vexriscv")
set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/litex_vexriscv.overlay")
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
add_custom_target(web_assets DEPENDS ${WEB_INC})

target_sources(app PRIVATE
  src/main.c
  src/fpga_regs.c
  src/pll_ctrl.c
  src/fpga_poll.c
  src/ui_display.c
  src/ptp_bmc.c
  src/sap_sdp.c
  src/aes67_config.c
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

add_dependencies(app web_assets)

# ---------- Generate Flash Boot Image (.fbi) after build ----------
# The LiteX BIOS expects an 8-byte header (length + CRC-32) before the
# firmware payload.  This runs make_fbi.py on the raw zephyr.bin to
# produce zephyr.fbi, ready to flash at FLASH_BOOT_ADDRESS (0x010000).
set(MAKE_FBI ${CMAKE_CURRENT_SOURCE_DIR}/../../dev_scripts/make_fbi.py)
set(FBI_STAMP ${CMAKE_BINARY_DIR}/zephyr/zephyr.fbi)
add_custom_command(
  OUTPUT  ${FBI_STAMP}
  DEPENDS ${CMAKE_BINARY_DIR}/zephyr/zephyr.bin
  COMMAND ${PYTHON_EXECUTABLE} ${MAKE_FBI}
          ${CMAKE_BINARY_DIR}/zephyr/zephyr.bin
          ${FBI_STAMP}
  COMMENT "Generating LiteX flash boot image (zephyr.fbi)"
)
add_custom_target(fbi ALL DEPENDS ${FBI_STAMP})
