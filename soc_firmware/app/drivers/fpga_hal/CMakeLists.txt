if(CONFIG_FPGA_HAL_FMC)
  target_sources(app PRIVATE fpga_hal_fmc.c)
endif()

if(CONFIG_FPGA_HAL_LITEX)
  target_sources(app PRIVATE fpga_hal_litex.c)

  # LiteX stubs must come before the generated directory
  target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../eth_litex/litex_stubs)
  set(LITEX_GENERATED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../litex_soc/build/software/include)
  target_include_directories(app PRIVATE ${LITEX_GENERATED_DIR})
endif()
