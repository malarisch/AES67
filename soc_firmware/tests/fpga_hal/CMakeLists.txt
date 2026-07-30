# Unit tests for the FPGA HAL: the mock register-map backend that the
# QEMU/CI firmware image runs on, the backend-neutral system_cfg parser
# on top of it, and the PPB arithmetic in fpga_regs.
#
# Keeping the mock under test matters twice over: a wrong emulation
# makes every higher-level QEMU test (NMOS conformance included) assert
# against fiction.

cmake_minimum_required(VERSION 3.20.0)
include(${CMAKE_CURRENT_SOURCE_DIR}/../sdk_qemu.cmake)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(aes67_fpga_hal_test)

set(APP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../app)

add_subdirectory(${APP_DIR}/drivers ${CMAKE_CURRENT_BINARY_DIR}/app_drivers)

target_sources(app PRIVATE
  src/test_fpga_hal_mock.c
  src/test_fpga_syscfg.c
  src/test_fpga_regs.c

  ${APP_DIR}/src/fpga_regs.c
)

target_include_directories(app PRIVATE
  ${APP_DIR}/src
  ${APP_DIR}/drivers
  ${APP_DIR}/drivers/fpga_hal
)
