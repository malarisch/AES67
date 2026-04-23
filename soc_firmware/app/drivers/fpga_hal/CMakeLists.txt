if(CONFIG_FPGA_HAL_FMC)
  target_sources(app PRIVATE fpga_hal_fmc.c)
endif()

if(CONFIG_FPGA_HAL_SPI)
  target_sources(app PRIVATE fpga_hal_spi.c)
endif()

if(CONFIG_FPGA_HAL_SHELL)
  target_sources(app PRIVATE fpga_hal_shell.c)
endif()

if(CONFIG_FPGA_HAL_LITEX)
  target_sources(app PRIVATE fpga_hal_litex.c)

  # `_litex_target` is set in the top-level CMakeLists.txt based on BOARD.
  if(NOT _litex_target)
    message(FATAL_ERROR
      "CONFIG_FPGA_HAL_LITEX=y but no LiteX target resolved from BOARD=${BOARD}")
  endif()

  # LiteX stubs must come before the generated directory.
  target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../eth_litex/litex_stubs)
  set(LITEX_GENERATED_DIR
      ${CMAKE_CURRENT_SOURCE_DIR}/../../../../litex_soc/build/${_litex_target}/software/include)
  target_include_directories(app PRIVATE ${LITEX_GENERATED_DIR})
endif()
