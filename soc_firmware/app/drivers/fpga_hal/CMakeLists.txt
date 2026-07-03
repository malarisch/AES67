if(CONFIG_FPGA_HAL_FMC)
  target_sources(app PRIVATE fpga_hal_fmc.c)
endif()

if(CONFIG_FPGA_HAL_SPI)
  target_sources(app PRIVATE fpga_hal_spi.c)

  # External MCU: no CPU SoC, the register map comes straight from the
  # aes67_bridge LiteX build (the same absolute addresses every host uses).
  set(LITEX_BRIDGE_INC
      "${CMAKE_CURRENT_SOURCE_DIR}/../../../../litex_soc/build/aes67_bridge/software/include")
  if(NOT EXISTS "${LITEX_BRIDGE_INC}/generated/csr.h")
    message(FATAL_ERROR
      "aes67_bridge generated headers not found at ${LITEX_BRIDGE_INC}.\n"
      "Run: cd litex_soc && python generate.py --target aes67_bridge")
  endif()
  # LiteX stubs must come before the generated directory.
  target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../eth_litex/litex_stubs)
  target_include_directories(app PRIVATE ${LITEX_BRIDGE_INC})
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
  # Merged CPU + aes67_bridge generated headers (see top-level CMakeLists).
  if(NOT DEFINED LITEX_GENERATED_INC)
    message(FATAL_ERROR "LITEX_GENERATED_INC is not set — expected from top-level app CMakeLists")
  endif()
  target_include_directories(app PRIVATE ${LITEX_GENERATED_INC})
endif()
