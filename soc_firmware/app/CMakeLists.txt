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
project(app)

target_sources(app PRIVATE
  src/main.c
  drivers/eth_spi_basic/eth_spi_basic.c
)
