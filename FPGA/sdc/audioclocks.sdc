
# Audio master clock from external PLL (48kHz * 512 = 24.576 MHz)
create_clock -name {audio_mclk} -period 40.690 -waveform { 0.000 20.345 } [get_ports {pll_512fs_i}]

# wallclock NCO MSB (nco_phase[31]) is a *data* register clocked by sys_clk_125m,
# not a real clock. Quartus auto-detects it as a clock because it fans out into
# logic that looks MCLK-shaped. The NCO output (wc_mclk) is only used when
# USE_EXTERNAL_PLL = "false" — with the external Si5351 PLL it is dead silicon.
# Quartus auto-detects nco_phase[31] as a base clock target because it fans
# out into clock-shaped logic. Give it an explicit constraint so it stops
# being "unconstrained", then group it asynchronously from everything else.
# wc_mclk is unused with the external Si5351 PLL (USE_EXTERNAL_PLL = "true").
create_clock -name {nco_mclk} -period 40.7 \
    [get_registers {aes67_top:aes67_top_inst|ptp_module:ptp_inst|wallclock:b2v_wallclock|nco_phase[31]}]

set_clock_groups -asynchronous -group {nco_mclk}

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
