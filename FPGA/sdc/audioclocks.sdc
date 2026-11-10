# ----------------------------------------------------------------------------
# Audio clock domain.
#
#   audio_mclk : external 512fs clock on pll_512fs_i (legacy USE_EXTERNAL_PLL
#                builds only; absent otherwise)
#   nco_mclk, clk_256fs / clk_128fs / clk_64fs : registers in the wallclock
#                that are used as logic clocks, see below. A clock whose
#                register was optimised away in a build (e.g. the 64fs tree
#                in a TDM8-only design) is skipped.
#
# The two families never coexist in one build (-exclusive at the bottom).
# Boards with an external VCXO / PLL MCLK (VCXO_DOMAIN_CLOCKS) declare that
# clock on its input port in their own SDC.
# ----------------------------------------------------------------------------

create_clock -name {audio_mclk} -period 40.690 -waveform { 0.000 20.345 } \
    [get_ports {*pll_512fs_i}]

# The NCO MSB (nco_phase[47] in wallclock_nco) and the bclk registers of
# the sys_clk-domain clock tree (wallclock_clk_div, u_clk_div) are
# sys_clk registers that fan out as logic clocks (TDM in/out logic is
# clocked by bclk and handed to sys_clk without synchronisers). Every edge
# of these clocks therefore coincides with a sys_clk edge (+ the register
# and clock routing delay, which the analyzer takes from the netlist).
# Model: generated clocks of the register's own sys_clk pin, divide-by-2.
# The edge alignment is exact; only the frequency is simplified, which
# does not matter for the single-cycle sys_clk <-> audio-clock paths:
# they get one sys_clk period, as in the hardware.
set wc_path {*soc_top:soc_top_inst|wb_bridge_top:wb_bridge_top_inst|aes67_wb_bridge:aes67_wb_bridge_inst|aes67_top:aes67_top_inst|ptp_module:ptp_inst|wallclock:b2v_wallclock}

proc aes67_audio_clock {name reg} {
    set regs [get_registers -nowarn $reg]
    if {[get_collection_size $regs] == 0} {
        post_message -type info "audioclocks.sdc: $name not present ($reg)"
        return
    }
    create_generated_clock -name $name -source [get_pins "$reg|clk"] \
        -divide_by 2 $regs
}

aes67_audio_clock nco_mclk  "$wc_path|wallclock_nco:u_nco|nco_phase\[47\]"
aes67_audio_clock clk_256fs "$wc_path|wallclock_clk_div:u_clk_div|clocks_o.clk_256fs.bclk"
aes67_audio_clock clk_128fs "$wc_path|wallclock_clk_div:u_clk_div|clocks_o.clk_128fs.bclk"
aes67_audio_clock clk_64fs  "$wc_path|wallclock_clk_div:u_clk_div|clocks_o.clk_64fs.bclk"

# sys_clk -> audio clock, hold: the bclk-clocked TDM logic samples sys_clk
# data (tdm_in_sync2, fsclk_tdm, tdm_byte_latch, ...) on the bclk edge that
# the same sys_clk edge creates. The bclk arrives a routing delay later, so
# the hardware captures the NEW value -- like the RTL simulation does (bclk
# and data update in the same delta). Hold-fixing these paths would flip
# that to the old value, so the fitter must not pad them. Setup is still
# analysed (one sys_clk period).
foreach aes67_clk {nco_mclk clk_256fs clk_128fs clk_64fs} {
    if {[get_collection_size [get_clocks -nowarn $aes67_clk]] > 0} {
        set_false_path -hold \
            -from [get_clocks [get_clock_info -master_clock $aes67_clk]] \
            -to   [get_clocks $aes67_clk]
    }
}

# audio_mclk (external) and the wallclock audio clocks are mutually
# exclusive: only one family exists in any single build.
set aes67_nco_clocks [get_clocks -nowarn {nco_mclk clk_256fs clk_128fs clk_64fs}]
if {[get_collection_size [get_clocks -nowarn {audio_mclk}]] > 0 && [get_collection_size $aes67_nco_clocks] > 0} {
    set_clock_groups -exclusive \
        -group [get_clocks {audio_mclk}] \
        -group $aes67_nco_clocks
}
