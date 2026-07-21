target_sources(app PRIVATE fpga_jtag.c)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if(CONFIG_AES67_FPGA_JTAG_BOOT_LOAD)
  set(fpga_jtag_svf ${CONFIG_AES67_FPGA_JTAG_BITSTREAM})
  if(NOT IS_ABSOLUTE ${fpga_jtag_svf})
    set(fpga_jtag_svf ${CMAKE_SOURCE_DIR}/${fpga_jtag_svf})
  endif()
  if(NOT EXISTS ${fpga_jtag_svf})
    message(FATAL_ERROR
      "CONFIG_AES67_FPGA_JTAG_BOOT_LOAD: SVF not found: ${fpga_jtag_svf}\n"
      "Generate it in the Quartus Programmer (File -> Create JAM/SVF/JBC "
      "File, SVF, JTAG) or point CONFIG_AES67_FPGA_JTAG_BITSTREAM at one.")
  endif()

  # Altera's JTAG configuration data is not the serial RBF; extract the
  # JTAG-format blob from the SVF's large SDR-TDI payload, then embed it.
  # Also emit the USERCODE for post-config read-back verification.
  set(fpga_jtag_bin ${CMAKE_CURRENT_BINARY_DIR}/fpga_jtag_config.bin)
  set(fpga_jtag_meta ${ZEPHYR_BINARY_DIR}/include/generated/fpga_jtag_meta.h)
  set(fpga_jtag_svf_script ${CMAKE_SOURCE_DIR}/scripts/svf_to_bin.py)
  add_custom_command(
    OUTPUT ${fpga_jtag_bin} ${fpga_jtag_meta}
    COMMAND ${PYTHON_EXECUTABLE} ${fpga_jtag_svf_script}
            ${fpga_jtag_svf} ${fpga_jtag_bin} ${fpga_jtag_meta}
    DEPENDS ${fpga_jtag_svf} ${fpga_jtag_svf_script}
    COMMENT "Extracting JTAG config blob + USERCODE from ${fpga_jtag_svf}")
  add_custom_target(fpga_jtag_config_blob DEPENDS ${fpga_jtag_bin} ${fpga_jtag_meta})
  add_dependencies(app fpga_jtag_config_blob)

  generate_inc_file_for_target(app ${fpga_jtag_bin}
    ${ZEPHYR_BINARY_DIR}/include/generated/fpga_bitstream_jtag.inc)
endif()
