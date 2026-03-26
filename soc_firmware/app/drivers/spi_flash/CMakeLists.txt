target_sources(app PRIVATE spi_flash.c)

# LiteX generated headers (csr.h, mem.h, soc.h) + stubs
set(LITEX_GENERATED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../litex_soc/build/software/include)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../eth_litex/litex_stubs)
target_include_directories(app PRIVATE ${LITEX_GENERATED_DIR})
