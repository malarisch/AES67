library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ============================================================
-- Ethernet Timestamp Latch
-- Captures wallclock time when timestamp_set_i pulses
--
-- CRITICAL: This module must handle the case where seconds and
-- nanoseconds could be captured at a boundary (e.g., nsec wrapping
-- from 999999999 to 0 while seconds increments). This can cause
-- garbage values like a sudden jump of ~1 billion nanoseconds.
--
-- Solution: Use a two-stage capture with consistency check.
-- If nanoseconds wrapped between two consecutive reads, re-read
-- to get consistent values.
-- ============================================================

entity ethernet_timestamp is
    port(
        clk			: in std_logic;
        reset_n			: in std_logic;
        
        wallclock_seconds_i : in unsigned(47 downto 0);
        wallclock_nanoseconds_i : in unsigned(31 downto 0);
        timestamp_seconds_o	: out unsigned(47 downto 0);
        timestamp_nanoseconds_o : out unsigned(31 downto 0);
        
        timestamp_set_i : in std_logic

        
    );
end ethernet_timestamp;

architecture Behavioral of ethernet_timestamp is
    signal timestamp_sec_reg  : unsigned(47 downto 0) := (others => '0');
    signal timestamp_nsec_reg : unsigned(31 downto 0) := (others => '0');

    -- Synchronizer for timestamp_set_i (may come from RX clock domain)
    signal ts_set_meta : std_logic := '0';
    signal ts_set_sync : std_logic := '0';
    signal ts_set_prev : std_logic := '0';
    
    -- Two-stage capture for atomicity
    signal capture_state : integer range 0 to 2 := 0;
    signal sec_first     : unsigned(47 downto 0) := (others => '0');
    signal nsec_first    : unsigned(31 downto 0) := (others => '0');
    
    -- Threshold for detecting wrap-around (900 million ns = 0.9 seconds)
    constant WRAP_THRESHOLD : unsigned(31 downto 0) := to_unsigned(900_000_000, 32);
    constant NS_PER_SEC     : unsigned(31 downto 0) := to_unsigned(1_000_000_000, 32);
    
begin
    process(clk, reset_n)
        variable sec_candidate  : unsigned(47 downto 0);
        variable nsec_candidate : unsigned(31 downto 0);
    begin
        if reset_n = '0' then
            timestamp_sec_reg <= (others => '0');
            timestamp_nsec_reg <= (others => '0');
            capture_state <= 0;
            sec_first <= (others => '0');
            nsec_first <= (others => '0');
            ts_set_meta <= '0';
            ts_set_sync <= '0';
            ts_set_prev <= '0';
        elsif rising_edge(clk) then
            -- Synchronize timestamp_set_i into clk domain
            ts_set_meta <= timestamp_set_i;
            ts_set_sync <= ts_set_meta;
            ts_set_prev <= ts_set_sync;

            case capture_state is
                when 0 =>
                    -- Idle: wait for synchronized rising edge
                    if (ts_set_prev = '0' and ts_set_sync = '1') then
                        -- First capture
                        sec_first <= wallclock_seconds_i;
                        nsec_first <= wallclock_nanoseconds_i;
                        capture_state <= 1;
                    end if;
                    
                when 1 =>
                    -- Second capture and consistency check
                    -- If nsec_first was high (near wrap) and current is low (wrapped),
                    -- the seconds value may have incremented between captures.
                    sec_candidate := sec_first;
                    nsec_candidate := nsec_first;

                    if (nsec_first > WRAP_THRESHOLD and wallclock_nanoseconds_i < WRAP_THRESHOLD) then
                        -- Wrap detected! Use current seconds (already incremented)
                        -- with current nanoseconds for consistency
                        sec_candidate := wallclock_seconds_i;
                        nsec_candidate := wallclock_nanoseconds_i;
                    end if;

                    -- Sanity clamp: nanoseconds must stay within [0, 1_000_000_000)
                    if nsec_candidate >= NS_PER_SEC then
                        nsec_candidate := nsec_candidate - NS_PER_SEC;
                        sec_candidate := sec_candidate + 1;
                    end if;

                    timestamp_sec_reg <= sec_candidate;
                    timestamp_nsec_reg <= nsec_candidate;
                    capture_state <= 0;
                    
                when others =>
                    capture_state <= 0;
            end case;
        end if;
    end process;
    
    timestamp_seconds_o <= timestamp_sec_reg;
    timestamp_nanoseconds_o <= timestamp_nsec_reg;
    
end Behavioral;