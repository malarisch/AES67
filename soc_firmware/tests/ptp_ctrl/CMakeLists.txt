# Unit tests for the mode-invariant PTP status facade (ptp_ctrl) and the
# hardware backend underneath it (ptp_ctrl_hw + the FPGA-BMC glue in
# ptp_bmc).
#
# The mock gateware reports hardware PTP, so the runtime dispatch in
# ptp_ctrl.c resolves to the hardware backend — the same path a real
# HW-PTP bitstream takes.

cmake_minimum_required(VERSION 3.20.0)
include(${CMAKE_CURRENT_SOURCE_DIR}/../sdk_qemu.cmake)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(aes67_ptp_ctrl_test)

set(APP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../app)

add_subdirectory(${APP_DIR}/drivers ${CMAKE_CURRENT_BINARY_DIR}/app_drivers)

target_sources(app PRIVATE
  src/test_ptp_ctrl.c

  ${APP_DIR}/src/ptp_ctrl.c
  ${APP_DIR}/src/ptp_ctrl_hw.c
  ${APP_DIR}/src/ptp_bmc.c
  ${APP_DIR}/src/aes67_config.c
)

target_include_directories(app PRIVATE ${APP_DIR}/src ${APP_DIR}/drivers)
