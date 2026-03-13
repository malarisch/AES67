target_sources(app PRIVATE eth_litex.c)

# Stubs for LiteX BIOS headers (system.h) that don't exist in Zephyr.
# Must come BEFORE the LiteX generated directory so our stubs take priority.
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/litex_stubs)

# LiteX generated headers (csr.h, mem.h, soc.h)
set(LITEX_GENERATED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../litex_soc/build/software/include)
target_include_directories(app PRIVATE ${LITEX_GENERATED_DIR})
