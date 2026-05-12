# --- PTP ClockConfigurator FSM (ptpv2_parser clock_config_process) ---
# The FSM advances one step every 8 sys_clk_125m cycles: configure_wait_cycle
# is an unsigned(2 downto 0) that decrements unconditionally and wraps
# (0 -> 7 -> 6 -> ... -> 1 -> 0), gating state transitions to wait_cycle = 0.
# So the combinational chain feeding elapsed_ns, ns_sum, and
# clock_configure_timestamp_*_o has 8 clock periods to settle.
# Setup multicycle = 8, hold multicycle = 7 to keep hold analysis on the
# original launch edge.
set_multicycle_path -setup -end 8 \
    -to [get_registers {*ptp_inst|b2v_ptpparser|elapsed_ns*}]
set_multicycle_path -hold  -end 7 \
    -to [get_registers {*ptp_inst|b2v_ptpparser|elapsed_ns*}]

set_multicycle_path -setup -end 8 \
    -to [get_registers {*ptp_inst|b2v_ptpparser|ns_sum*}]
set_multicycle_path -hold  -end 7 \
    -to [get_registers {*ptp_inst|b2v_ptpparser|ns_sum*}]

set_multicycle_path -setup -end 8 \
    -to [get_registers {*ptp_inst|b2v_ptpparser|clock_configure_timestamp_nanoseconds_o*}]
set_multicycle_path -hold  -end 7 \
    -to [get_registers {*ptp_inst|b2v_ptpparser|clock_configure_timestamp_nanoseconds_o*}]

set_multicycle_path -setup -end 8 \
    -to [get_registers {*ptp_inst|b2v_ptpparser|clock_configure_timestamp_seconds_o*}]
set_multicycle_path -hold  -end 7 \
    -to [get_registers {*ptp_inst|b2v_ptpparser|clock_configure_timestamp_seconds_o*}]

# --- PTP Servo PI controller FSM (ptpv2_servo PI process) ---
# The PI FSM toggles pi_wait_state every cycle, so multiplications, shifts and
# clamp arithmetic have 2 sys_clk_125m periods between register updates.
# Setup multicycle = 2, hold multicycle = 1.
set_multicycle_path -setup -end 2 \
    -to [get_registers {*ptp_inst|b2v_servo|pi_input[*]}]
set_multicycle_path -hold  -end 1 \
    -to [get_registers {*ptp_inst|b2v_servo|pi_input[*]}]

set_multicycle_path -setup -end 2 \
    -to [get_registers {*ptp_inst|b2v_servo|pi_mult_p[*]}]
set_multicycle_path -hold  -end 1 \
    -to [get_registers {*ptp_inst|b2v_servo|pi_mult_p[*]}]

set_multicycle_path -setup -end 2 \
    -to [get_registers {*ptp_inst|b2v_servo|pi_mult_i[*]}]
set_multicycle_path -hold  -end 1 \
    -to [get_registers {*ptp_inst|b2v_servo|pi_mult_i[*]}]

set_multicycle_path -setup -end 2 \
    -to [get_registers {*ptp_inst|b2v_servo|pi_proportional[*]}]
set_multicycle_path -hold  -end 1 \
    -to [get_registers {*ptp_inst|b2v_servo|pi_proportional[*]}]

set_multicycle_path -setup -end 2 \
    -to [get_registers {*ptp_inst|b2v_servo|pi_int_update[*]}]
set_multicycle_path -hold  -end 1 \
    -to [get_registers {*ptp_inst|b2v_servo|pi_int_update[*]}]

set_multicycle_path -setup -end 2 \
    -to [get_registers {*ptp_inst|b2v_servo|pi_sum_raw[*]}]
set_multicycle_path -hold  -end 1 \
    -to [get_registers {*ptp_inst|b2v_servo|pi_sum_raw[*]}]

set_multicycle_path -setup -end 2 \
    -to [get_registers {*ptp_inst|b2v_servo|integral_sum[*]}]
set_multicycle_path -hold  -end 1 \
    -to [get_registers {*ptp_inst|b2v_servo|integral_sum[*]}]

set_multicycle_path -setup -end 2 \
    -to [get_registers {*ptp_inst|b2v_servo|freq_correction[*]}]
set_multicycle_path -hold  -end 1 \
    -to [get_registers {*ptp_inst|b2v_servo|freq_correction[*]}]

# --- Wallclock NCO ppb adjustment (wallclock nco_ppb_adj_proc) ---
# ppb_adj_reg is written every second cycle (gated by nco_ppb_adj_wait), giving
# the signed multiply + shift 2 sys_clk_125m periods. Setup multicycle = 2,
# hold multicycle = 1.
set_multicycle_path -setup -end 2 \
    -to [get_registers {*ptp_inst|b2v_wallclock|ppb_adj_reg[*]}]
set_multicycle_path -hold  -end 1 \
    -to [get_registers {*ptp_inst|b2v_wallclock|ppb_adj_reg[*]}]

# --- BMC dataset comparators (ptpv2_bmc P_CMP_ANN_EXT / P_CMP_EXT_SELF) ---
# The two long 136-bit dataset comparisons are registered into ann_better_r
# and ext_beats_self_r. The main BMC FSM inserts a one-cycle wait state
# (update_wait_r) after updating the ext slot, so the comparator outputs
# have 2 sys_clk_125m periods to settle before being sampled.
# Setup multicycle = 2, hold multicycle = 1.
set_multicycle_path -setup -end 2 \
    -to [get_registers {*ptp_inst|b2v_bmc|ann_better_r}]
set_multicycle_path -hold  -end 1 \
    -to [get_registers {*ptp_inst|b2v_bmc|ann_better_r}]

set_multicycle_path -setup -end 2 \
    -to [get_registers {*ptp_inst|b2v_bmc|ext_beats_self_r}]
set_multicycle_path -hold  -end 1 \
    -to [get_registers {*ptp_inst|b2v_bmc|ext_beats_self_r}]


set_multicycle_path -setup -end 2 -from [get_registers *media_clock_nsec_latch*] -to [get_registers *media_mult_reg*]
set_multicycle_path -hold  -end 1 -from [get_registers *media_clock_nsec_latch*] -to [get_registers *media_mult_reg*]
