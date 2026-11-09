# Unit tests for the AES70 / OCP.1 control stack: wire-format helpers and
# the transport-agnostic session core (dispatch, locks, subscriptions,
# notifications, keep-alive) driven with a synthetic object model.
#
# The device model (aes70_device.c) is not linked: it binds to the card
# drivers, stream tables and PTP stack. The core is exercised through the
# same session API the TCP server uses.

cmake_minimum_required(VERSION 3.20.0)
include(${CMAKE_CURRENT_SOURCE_DIR}/../sdk_qemu.cmake)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(aes67_aes70_test)

set(APP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../app)

target_sources(app PRIVATE
  src/test_ocp1.c
  src/test_core.c

  ${APP_DIR}/src/aes70/ocp1.c
  ${APP_DIR}/src/aes70/aes70_core.c
)

target_include_directories(app PRIVATE ${APP_DIR}/src ${APP_DIR}/src/aes70)
