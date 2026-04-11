
# Audio master clock from external PLL (48kHz * 512 = 24.576 MHz)
create_clock -name {audio_mclk} -period 40.690 -waveform { 0.000 20.345 } [get_ports {pll_512fs_i}]


# cnt(0) -> clk_256fs / bclk_r : 12.288 MHz (audio_mclk / 2)
create_generated_clock -name {clk_256fs} \
    -source [get_ports {pll_512fs_i}] \
    -divide_by 2 \
    -master_clock {audio_mclk} \
    [get_registers {aes67_top_inst|audioclocks_inst|cnt[0]}]

# cnt(1) -> clk_128fs : 6.144 MHz (audio_mclk / 4)
create_generated_clock -name {clk_128fs} \
    -source [get_ports {pll_512fs_i}] \
    -divide_by 4 \
    -master_clock {audio_mclk} \
    [get_registers {aes67_top_inst|audioclocks_inst|cnt[1]}]

# cnt(2) -> clk_64fs : 3.072 MHz (audio_mclk / 8)
create_generated_clock -name {clk_64fs} \
    -source [get_ports {pll_512fs_i}] \
    -divide_by 8 \
    -master_clock {audio_mclk} \
    [get_registers {aes67_top_inst|audioclocks_inst|cnt[2]}]

# cnt(8) -> fs : 48 kHz (audio_mclk / 512)
create_generated_clock -name {fs} \
    -source [get_ports {pll_512fs_i}] \
    -divide_by 512 \
    -master_clock {audio_mclk} \
    [get_registers {aes67_top_inst|audioclocks_inst|cnt[8]}]

# bclk_f_r : 12.288 MHz, registered on falling edge of audio_mclk
# This register is clocked by the inverted mclk — declare as generated
# clock with invert so Quartus tracks timing correctly.
create_generated_clock -name {bclk_f} \
    -source [get_ports {pll_512fs_i}] \
    -divide_by 2 \
    -invert \
    -master_clock {audio_mclk} \
    [get_registers {aes67_top_inst|audioclocks_inst|bclk_f_r}]
