# Unit tests for the device configuration and its JSON persistence
# format, and for the connection-management stream tables underneath.
#
# aes67_conn writes through the FPGA HAL, so the mock backend is linked
# in — the same one the QEMU firmware image uses. No interface is
# brought up: the IGMP proxy joins are skipped while iface is NULL.

cmake_minimum_required(VERSION 3.20.0)
include(${CMAKE_CURRENT_SOURCE_DIR}/../sdk_qemu.cmake)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(aes67_config_test)

set(APP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../app)

# fpga_hal (mock + syscfg) — brings its own include paths for the
# generated aes67_bridge CSR headers.
add_subdirectory(${APP_DIR}/drivers ${CMAKE_CURRENT_BINARY_DIR}/app_drivers)

target_sources(app PRIVATE
  src/test_aes67_config.c
  src/test_config_json.c
  src/test_aes67_conn.c

  ${APP_DIR}/src/aes67_config.c
  ${APP_DIR}/src/config_json.c
  ${APP_DIR}/src/aes67_conn.c
  ${APP_DIR}/src/card_manager.c
  ${APP_DIR}/src/card_settings.c
)

target_include_directories(app PRIVATE ${APP_DIR}/src)
