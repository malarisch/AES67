target_sources(app PRIVATE spi_flash.c)

# LiteX generated headers (csr.h, mem.h, soc.h) + stubs. `_litex_target`
# is set in the top-level CMakeLists.txt based on BOARD.
if(NOT _litex_target)
  message(FATAL_ERROR
    "CONFIG_SPI_FLASH_LITESPI=y but no LiteX target resolved from BOARD=${BOARD}")
endif()
set(LITEX_GENERATED_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../litex_soc/build/${_litex_target}/software/include)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../eth_litex/litex_stubs)
target_include_directories(app PRIVATE ${LITEX_GENERATED_DIR})
