# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
if(NOT flash-16M IN_LIST SNIPPET)
    #set(SNIPPET flash-16M ${SNIPPET} CACHE STRING "" FORCE)
endif()
if(NOT psram-8M IN_LIST SNIPPET)
    #set(SNIPPET psram-8M ${SNIPPET} CACHE STRING "" FORCE)
endif()


#set(BOARD "esp32s3_devkitc/esp32s3/procpu")
set(BOARD "nucleo_h753zi")
set(CONFIG_BOOTLOADER_MCUBOOT y)
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
  src/webserver.c
)

# SD card configuration storage (optional)
target_sources_ifdef(CONFIG_SD_CONFIG app PRIVATE src/sd_config.c)

add_dependencies(app web_assets)
