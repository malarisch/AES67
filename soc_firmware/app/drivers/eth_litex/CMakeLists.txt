target_sources(app PRIVATE eth_litex.c)

# Software-PTP: the FPGA wallclock exposed as a Zephyr ptp_clock device.
target_sources_ifdef(CONFIG_AES67_PTP_SOFTWARE app PRIVATE ptp_clock_aes67.c)

# Stubs for LiteX BIOS headers (system.h) that don't exist in Zephyr.
# Must come BEFORE the LiteX generated directory so our stubs take priority.
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/litex_stubs)

# LiteX generated headers (csr.h, mem.h, soc.h) — merged CPU + aes67_bridge
# tree produced by the top-level app CMakeLists (merge_litex_csr.py).
if(NOT DEFINED LITEX_GENERATED_INC)
    message(FATAL_ERROR "LITEX_GENERATED_INC is not set — expected from top-level app CMakeLists")
endif()
target_include_directories(app PRIVATE ${LITEX_GENERATED_INC})
