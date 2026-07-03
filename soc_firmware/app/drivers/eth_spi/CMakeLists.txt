target_sources(app PRIVATE eth_spi.c)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Software-PTP: the FPGA wallclock exposed as a Zephyr ptp_clock device.
# Shared with the eth_litex backend — register access inside is dispatched
# to spibone when CONFIG_FPGA_HAL_SPI is set.
target_sources_ifdef(CONFIG_AES67_PTP_SOFTWARE app PRIVATE
                     ../eth_litex/ptp_clock_aes67.c)
