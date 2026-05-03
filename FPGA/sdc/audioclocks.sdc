
# ----------------------------------------------------------------------------
# Audio clock domain.
#
# The 512fs MCLK that drives audioclock_generator comes from one of two
# sources, selected at elaboration time by the USE_EXTERNAL_PLL generic in
# aes67_top:
#   "true"  -> external Si5351 PLL on port pll_512fs_i (clock: audio_mclk)
#   "false" -> wallclock NCO MSB nco_phase[31]         (clock: nco_mclk)
#
# Both source clocks are declared so this file stays valid for both build
# variants. set_clock_groups -exclusive at the bottom tells the analyzer the
# two never coexist in any single build.
#
# All derived audio clocks (clk_256fs/128fs/64fs, fs, bclk_f) are sourced
# from nco_phase[31] (the wallclock NCO output register). When the build
# is in external-PLL mode, that register still exists in the netlist but
# is a dead leaf — and the -exclusive group keeps the analyzer from
# chasing nonsensical phase relations to audio_mclk.
# ----------------------------------------------------------------------------

create_clock -name {audio_mclk} -period 40.690 -waveform { 0.000 20.345 } \
    [get_ports {*pll_512fs_i}]

# nco_phase[31] is a register bit, but it fans out as a logic clock when
# USE_EXTERNAL_PLL = "false". Naming it as a clock here both stops Quartus
# from treating it as unconstrained and gives us a name to use elsewhere.
set nco_reg {*ptp_module:ptp_inst|wallclock:b2v_wallclock|nco_phase[31]}

create_clock -name {nco_mclk} -period 40.690 [get_registers $nco_reg]

# Derived audio clocks. -source must be a port/pin/reg/net/keeper, NOT a clock
# name. Use the nco_phase[31] register itself as the source: it is a keeper
# and Quartus accepts it (collection type "kpr").
#
# Targets are register cells inside audioclock_generator. Hierarchy paths use
# the {entity:instance} format Quartus emits in netlist names.

# cnt(0) -> clk_256fs / bclk_r : 12.288 MHz (mclk / 2)
create_generated_clock -name {clk_256fs} \
    -source [get_registers $nco_reg] -master_clock {nco_mclk} \
    -divide_by 2 \
    [get_registers {*audioclock_generator:audioclocks_inst|cnt[0]}]

# cnt(1) -> clk_128fs : 6.144 MHz (mclk / 4)
create_generated_clock -name {clk_128fs} \
    -source [get_registers $nco_reg] -master_clock {nco_mclk} \
    -divide_by 4 \
    [get_registers {*audioclock_generator:audioclocks_inst|cnt[1]}]

# cnt(2) -> clk_64fs : 3.072 MHz (mclk / 8)
create_generated_clock -name {clk_64fs} \
    -source [get_registers $nco_reg] -master_clock {nco_mclk} \
    -divide_by 8 \
    [get_registers {*audioclock_generator:audioclocks_inst|cnt[2]}]

# cnt(8) -> fs : 48 kHz (mclk / 512)
create_generated_clock -name {fs} \
    -source [get_registers $nco_reg] -master_clock {nco_mclk} \
    -divide_by 512 \
    [get_registers {*audioclock_generator:audioclocks_inst|cnt[8]}]

# bclk_f_r : 12.288 MHz, registered on the falling edge of mclk
create_generated_clock -name {bclk_f} \
    -source [get_registers $nco_reg] -master_clock {nco_mclk} \
    -divide_by 2 -invert \
    [get_registers {*audioclock_generator:audioclocks_inst|bclk_f_r}]

# ----------------------------------------------------------------------------
# Cross-domain handling.
#
# nco_mclk is generated from the 125 MHz sys clock (nco_phase[31] is a
# register bit on that domain). The audio signal path is fully sys-clk-
# launched on the source side; the audio clocks only re-sample. Realistic
# margin is one sys_clk period (8 ns), not one mclk period (~40 ns) —
# relax with a multicycle.
#
# $sys_pll_clk0 is set in the parent SDC (c10evalkit.sdc) to the atom path
# of the 125 MHz PLL output. That is the only stable name the post-
# derive_pll_clocks netlist gives it.
# ----------------------------------------------------------------------------

if {[info exists sys_pll_clk0]} {
    set_multicycle_path -setup -end 5 -from [get_clocks $sys_pll_clk0] -to [get_clocks {nco_mclk}]
    set_multicycle_path -hold  -end 4 -from [get_clocks $sys_pll_clk0] -to [get_clocks {nco_mclk}]
}

# audio_mclk (external) and nco_mclk (NCO register) are mutually exclusive:
# only one drives audioclock_generator in any single build (USE_EXTERNAL_PLL).
# All derived clocks (clk_*, fs, bclk_f) hang off nco_mclk above, so they
# go in the same exclusive group.
set_clock_groups -exclusive \
    -group [get_clocks {audio_mclk}] \
    -group [get_clocks {nco_mclk clk_256fs clk_128fs clk_64fs fs bclk_f}]
