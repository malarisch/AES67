-- ============================================================================
-- clock_ppb_meter.vhd
--
-- Measures the PPB (parts-per-billion) frequency difference between two
-- clocks (wallclock_64fs vs. pll_64fs) over a one-second measurement window
-- defined by wallclock_second_pulse_i.
--
-- Operation:
--   1. Assert start_i -> module waits for the next second pulse.
--   2. On first second pulse: valid_o goes low, edge counters start.
--   3. On next second pulse:  counting stops, PPB is calculated.
--   4. Result appears on ppb_o, valid_o goes high.
--
-- PPB formula:  ppb = (count_pll - count_wc) * 1e9 / count_wc
--   Positive -> PLL is running fast relative to wallclock.
--   Negative -> PLL is running slow relative to wallclock.
--
-- Division is performed by a restoring binary long divider (52 cycles).
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
        -- Result
        ppb_o                    : out signed(31 downto 0);  -- PPB difference
        valid_o                  : out std_logic             -- High when ppb_o is valid
    );
end entity clock_ppb_meter;

architecture rtl of clock_ppb_meter is

    -- ======================================================================
    -- Constants
    -- ======================================================================
    constant ONE_BILLION : unsigned(29 downto 0) := to_unsigned(1_000_000_000, 30);
    constant DIV_BITS    : integer := 52;  -- dividend width (22 + 30)

    -- ======================================================================
    -- FSM
    -- ======================================================================
    type state_t is (
        S_IDLE,        -- Waiting for start_i
        S_WAIT_PULSE,  -- Waiting for first second-pulse (alignment)
        S_COUNTING,    -- Counting edges between two second-pulses
        S_CALC_PREP,   -- Compute |diff|*1e9, prepare divider
        S_DIVIDING,    -- Restoring binary long division (52 cycles)
        S_DONE         -- Apply sign, output result
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
    -- Divider datapath
    -- ======================================================================
    signal dividend   : unsigned(DIV_BITS-1 downto 0) := (others => '0');
    signal divisor    : unsigned(21 downto 0)          := (others => '0');
    signal quotient   : unsigned(31 downto 0)          := (others => '0');
    signal rem_reg    : unsigned(22 downto 0)          := (others => '0');
    signal div_step   : integer range 0 to DIV_BITS-1  := 0;
    signal result_neg : std_logic                      := '0';

    -- ======================================================================
    -- Output registers
    -- ======================================================================
    signal ppb_reg   : signed(31 downto 0) := (others => '0');
    signal valid_reg : std_logic           := '0';

begin

    ppb_o   <= ppb_reg;
    valid_o <= valid_reg;

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
    -- Main state machine
    -- ==================================================================
    p_fsm : process(sys_clk, reset_n)
        variable v_rem     : unsigned(22 downto 0);
        variable v_diff    : unsigned(21 downto 0);
        variable v_product : unsigned(51 downto 0);   -- 22 + 30 = 52 bits
    begin
        if reset_n = '0' then
            state      <= S_IDLE;
            valid_reg  <= '0';
            ppb_reg    <= (others => '0');
            count_wc   <= (others => '0');
            count_pll  <= (others => '0');
            dividend   <= (others => '0');
            divisor    <= (others => '0');
            quotient   <= (others => '0');
            rem_reg    <= (others => '0');
            result_neg <= '0';
            div_step   <= 0;

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
                        state <= S_CALC_PREP;
                    end if;

                -- =====================================================
                -- CALC_PREP – compute |diff| * 1 000 000 000 and set
                --             up the restoring divider
                -- =====================================================
                when S_CALC_PREP =>
                    -- Determine sign and absolute difference
                    if count_pll >= count_wc then
                        v_diff     := count_pll - count_wc;
                        result_neg <= '0';
                    else
                        v_diff     := count_wc - count_pll;
                        result_neg <= '1';
                    end if;

                    -- 22-bit × 30-bit = 52-bit product
                    v_product := v_diff * ONE_BILLION;

                    dividend <= resize(v_product, DIV_BITS);
                    divisor  <= count_wc;
                    quotient <= (others => '0');
                    rem_reg  <= (others => '0');
                    div_step <= DIV_BITS - 1;

                    -- Guard against division by zero
                    if count_wc = 0 then
                        ppb_reg   <= (others => '0');
                        valid_reg <= '1';
                        state     <= S_IDLE;
                    else
                        state <= S_DIVIDING;
                    end if;

                -- =====================================================
                -- DIVIDING – restoring binary long division (1 bit/clk)
                --   Iterates DIV_BITS (52) cycles.
                -- =====================================================
                when S_DIVIDING =>
                    -- Shift remainder left and bring in next dividend bit
                    v_rem := rem_reg(21 downto 0) & dividend(div_step);

                    if v_rem >= resize(divisor, 23) then
                        v_rem := v_rem - resize(divisor, 23);
                        if div_step <= 31 then
                            quotient(div_step) <= '1';
                        end if;
                    end if;

                    rem_reg <= v_rem;

                    if div_step = 0 then
                        state <= S_DONE;
                    else
                        div_step <= div_step - 1;
                    end if;

                -- =====================================================
                -- DONE – apply sign and present result
                -- =====================================================
                when S_DONE =>
                    if result_neg = '1' then
                        ppb_reg <= -signed(quotient);
                    else
                        ppb_reg <= signed(quotient);
                    end if;
                    valid_reg <= '1';
                    state     <= S_IDLE;

            end case;
        end if;
    end process p_fsm;

end architecture rtl;
