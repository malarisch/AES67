# Unit tests for the Auracast card's register protocol helpers
# (drivers/auracast_card/auracast_card_proto.c): validation, register
# packing/unpacking, the adopt-running-stream comparison and the name
# tables. Pure functions, no I2C, no kernel objects.

cmake_minimum_required(VERSION 3.20.0)
include(${CMAKE_CURRENT_SOURCE_DIR}/../sdk_qemu.cmake)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(aes67_auracast_test)

set(APP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../app)

target_sources(app PRIVATE
  src/test_auracast_proto.c
  ${APP_DIR}/drivers/auracast_card/auracast_card_proto.c
)

target_include_directories(app PRIVATE ${APP_DIR}/drivers/auracast_card)
