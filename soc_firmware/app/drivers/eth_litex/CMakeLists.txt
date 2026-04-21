target_sources(app PRIVATE eth_litex.c)

# Stubs for LiteX BIOS headers (system.h) that don't exist in Zephyr.
# Must come BEFORE the LiteX generated directory so our stubs take priority.
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/litex_stubs)

# LiteX generated headers (csr.h, mem.h, soc.h) — per-target build dir.
# LITEX_BUILD_DIR is set by the top-level app CMakeLists based on LITEX_TARGET.
if(NOT DEFINED LITEX_BUILD_DIR)
    message(FATAL_ERROR "LITEX_BUILD_DIR is not set — expected from top-level app CMakeLists")
endif()
target_include_directories(app PRIVATE ${LITEX_BUILD_DIR}/software/include)
