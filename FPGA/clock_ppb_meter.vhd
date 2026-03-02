-- ============================================================================
-- clock_ppb_meter.vhd
--
-- Measures edge counts of two clocks (wallclock_64fs vs. pll_64fs) over a 
-- one-second measurement window defined by wallclock_second_pulse_i.
-- The raw counter values are output for MCU to calculate PPB.
--
-- Operation:
--   1. Assert start_i -> module waits for the next second pulse.
--   2. On first second pulse: valid_o goes low, edge counters start.
--   3. On next second pulse:  counting stops, counters latched.
--   4. Results appear on count_wc_o / count_pll_o, valid_o goes high.
--
-- MCU calculates PPB:  ppb = (count_pll - count_wc) * 1e9 / count_wc
--   Positive -> PLL is running fast relative to wallclock.
--   Negative -> PLL is running slow relative to wallclock.
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity clock_ppb_meter is
    port (
        sys_clk                  : in  std_logic;   -- 125 MHz system clock
        reset_n                  : in  std_logic;   -- Active-low reset
        -- Asynchronous clock inputs to be measured
        wallclock_64fs_in        : in  std_logic;   -- 64·fs from wallclock
        pll_64fs_in              : in  std_logic;   -- 64·fs from local PLL
        -- Control (active in sys_clk domain)
        wallclock_second_pulse_i : in  std_logic;   -- 1-PPS from wallclock
        start_i                  : in  std_logic;   -- Start a measurement
        -- Results: raw counter values for MCU to compute PPB
        count_wc_o               : out unsigned(21 downto 0);  -- Wallclock edge count
        count_pll_o              : out unsigned(21 downto 0);  -- PLL edge count
        valid_o                  : out std_logic               -- High when counts are valid
    );
end entity clock_ppb_meter;

architecture rtl of clock_ppb_meter is

    -- ======================================================================
    -- FSM (simplified - no calculation needed)
    -- ======================================================================
    type state_t is (
        S_IDLE,        -- Waiting for start_i
        S_WAIT_PULSE,  -- Waiting for first second-pulse (alignment)
        S_COUNTING,    -- Counting edges between two second-pulses
        S_DONE         -- Counting complete, output valid
    );
    signal state : state_t := S_IDLE;

    -- ======================================================================
    -- CDC synchronisers  (3-FF: meta-stability + edge detection)
    -- ======================================================================
    signal wc_sync  : std_logic_vector(2 downto 0) := (others => '0');
    signal pll_sync : std_logic_vector(2 downto 0) := (others => '0');

    signal wc_rise  : std_logic;
    signal pll_rise : std_logic;

    -- ======================================================================
    -- Edge detection for signals assumed to be in sys_clk domain
    -- ======================================================================
    signal sec_d      : std_logic := '0';
    signal sec_rise   : std_logic;
    signal start_d    : std_logic := '0';
    signal start_rise : std_logic;

    -- ======================================================================
    -- Edge counters  (22 bits → max 4 194 303, covers 64·48 000 = 3 072 000)
    -- ======================================================================
    signal count_wc  : unsigned(21 downto 0) := (others => '0');
    signal count_pll : unsigned(21 downto 0) := (others => '0');

    -- ======================================================================
    -- Output registers
    -- ======================================================================
    signal count_wc_reg  : unsigned(21 downto 0) := (others => '0');
    signal count_pll_reg : unsigned(21 downto 0) := (others => '0');
    signal valid_reg     : std_logic             := '0';

begin

    count_wc_o  <= count_wc_reg;
    count_pll_o <= count_pll_reg;
    valid_o     <= valid_reg;

    -- ==================================================================
    -- CDC: Synchronise the two asynchronous 64-fs clocks into sys_clk
    -- ==================================================================
    p_cdc : process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            wc_sync  <= wc_sync(1 downto 0)  & wallclock_64fs_in;
            pll_sync <= pll_sync(1 downto 0) & pll_64fs_in;
        end if;
    end process p_cdc;

    -- Rising-edge detectors (compare stage 2 and stage 3)
    wc_rise  <= wc_sync(1) and not wc_sync(2);
    pll_rise <= pll_sync(1) and not pll_sync(2);

    -- ==================================================================
    -- Edge detection for second-pulse and start (sys_clk domain)
    -- ==================================================================
    p_edge : process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            sec_d   <= wallclock_second_pulse_i;
            start_d <= start_i;
        end if;
    end process p_edge;

    sec_rise   <= wallclock_second_pulse_i and not sec_d;
    start_rise <= start_i                  and not start_d;

    -- ==================================================================
    -- Main state machine (simplified - just count, no calculation)
    -- ==================================================================
    p_fsm : process(sys_clk, reset_n)
    begin
        if reset_n = '0' then
            state        <= S_IDLE;
            valid_reg    <= '0';
            count_wc     <= (others => '0');
            count_pll    <= (others => '0');
            count_wc_reg <= (others => '0');
            count_pll_reg<= (others => '0');

        elsif rising_edge(sys_clk) then
            case state is

                -- =====================================================
                -- IDLE – wait for start_i rising edge
                -- =====================================================
                when S_IDLE =>
                    if start_rise = '1' then
                        valid_reg <= '0';
                        state     <= S_WAIT_PULSE;
                    end if;

                -- =====================================================
                -- WAIT_PULSE – align to the next second boundary
                -- =====================================================
                when S_WAIT_PULSE =>
                    if sec_rise = '1' then
                        count_wc  <= (others => '0');
                        count_pll <= (others => '0');
                        state     <= S_COUNTING;
                    end if;

                -- =====================================================
                -- COUNTING – accumulate rising edges for one second
                -- =====================================================
                when S_COUNTING =>
                    if wc_rise = '1' then
                        count_wc <= count_wc + 1;
                    end if;
                    if pll_rise = '1' then
                        count_pll <= count_pll + 1;
                    end if;

                    if sec_rise = '1' then
                        -- Latch final counts to output registers
                        count_wc_reg  <= count_wc;
                        count_pll_reg <= count_pll;
                        state         <= S_DONE;
                    end if;

                -- =====================================================
                -- DONE – output is valid, wait for next measurement
                -- =====================================================
                when S_DONE =>
                    valid_reg <= '1';
                    state     <= S_IDLE;

            end case;
        end if;
    end process p_fsm;

end architecture rtl;
