# Unit tests for the firmware's pure parsing / formatting modules.
#
# These modules have no hardware or network dependency of their own —
# they are compiled straight into the test image, so a failure points at
# the parser and not at the surrounding stack.

cmake_minimum_required(VERSION 3.20.0)
include(${CMAKE_CURRENT_SOURCE_DIR}/../sdk_qemu.cmake)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(aes67_parsers_test)

set(APP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../app)

target_sources(app PRIVATE
  src/test_nmos_json.c
  src/test_json_util.c
  src/test_sdp_utils.c
  src/test_ieee1588.c

  ${APP_DIR}/src/nmos/nmos_json.c
  ${APP_DIR}/src/webapi/json_util.c
  ${APP_DIR}/src/aes67_sdp_utils.c
)

target_include_directories(app PRIVATE
  ${APP_DIR}/src
  ${APP_DIR}/src/nmos
  ${APP_DIR}/src/webapi
)
