library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity wallclock is
    generic(
        -- nanoseconds added per clock tick (125 MHz -> 8 ns)
        -- WARNING: Ensure this matches your clock! 125MHz=8ns, 62.5MHz=16ns
        increment_interval : natural := 8
    );
    port(
        clk                     : in  std_logic;
        reset_n                 : in  std_logic;

        wallclock_seconds_o     : out unsigned(47 downto 0);
        wallclock_nanoseconds_o : out unsigned(31 downto 0);

        wallclock_set_i         : in  std_logic;
        wallclock_seconds_i     : in  unsigned(47 downto 0);
        wallclock_nanoseconds_i : in  unsigned(31 downto 0);

        -- PTP Servo correction inputs
        freq_correction_ppb_i   : in  signed(31 downto 0);  -- PPB correction (parts per billion)
        phase_jump_ns_i         : in  signed(31 downto 0);  -- One-time phase adjustment in ns
        phase_jump_valid_i      : in  std_logic;            -- Pulse to apply phase jump

        second_pulse_o          : out std_logic
    );
end wallclock;

architecture Behavioral of wallclock is
    constant NS_PER_SEC : signed(31 downto 0) := to_signed(1_000_000_000, 32);
    
    -- Main time registers (as SIGNALS, not variables)
    signal nsec_reg : signed(31 downto 0) := (others => '0');
    signal sec_reg  : unsigned(47 downto 0) := (others => '0');
    
    -- Fractional nanosecond accumulator for sub-nanosecond precision
    -- Range limited to ±2^31 to prevent runaway
    signal frac_ns_accum : signed(31 downto 0) := (others => '0');
    
    -- Constants for overflow detection (use 2^30 as threshold)
    constant FRAC_OVERFLOW  : signed(31 downto 0) := to_signed(2**30, 32);
    constant FRAC_UNDERFLOW : signed(31 downto 0) := to_signed(-(2**30), 32);
    
begin

    -- Output assignments
    wallclock_nanoseconds_o <= unsigned(nsec_reg);
    wallclock_seconds_o     <= sec_reg;

    process(clk, reset_n)
        variable frac_increment : signed(31 downto 0);
        variable new_frac       : signed(31 downto 0);
        variable ns_adjust      : integer range -1 to 1;
        variable new_nsec       : signed(31 downto 0);
        variable sec_inc        : integer range -1 to 1;
    begin
        if reset_n = '0' then
            second_pulse_o <= '0';
            nsec_reg       <= (others => '0');
            sec_reg        <= (others => '0');
            frac_ns_accum  <= (others => '0');

        elsif rising_edge(clk) then
            second_pulse_o <= '0';
            sec_inc := 0;

            if wallclock_set_i = '1' then
                -- Hard set of time
                nsec_reg <= signed(wallclock_nanoseconds_i);
                sec_reg  <= wallclock_seconds_i;
                frac_ns_accum <= (others => '0');
                
            elsif phase_jump_valid_i = '1' then
                -- Apply one-time phase correction
                new_nsec := nsec_reg + resize(phase_jump_ns_i, 32);
                
                -- Handle wrap-around
                if new_nsec >= NS_PER_SEC then
                    nsec_reg <= new_nsec - NS_PER_SEC;
                    sec_reg  <= sec_reg + 1;
                    second_pulse_o <= '1';
                elsif new_nsec < 0 then
                    nsec_reg <= new_nsec + NS_PER_SEC;
                    sec_reg  <= sec_reg - 1;
                else
                    nsec_reg <= new_nsec;
                end if;
                
            else
                -- ============================================
                -- Normal tick with frequency correction
                -- ============================================
                -- PPB correction: frac_increment = increment_interval * ppb
                -- Scale: when frac_ns_accum reaches ±2^30, adjust by ±1 ns
                -- This gives effective resolution of ~1 PPB
                
                -- Calculate fractional increment (limit to 16-bit ppb to prevent overflow)
                -- For ±500000 PPB max: 8 * 500000 = 4M, fits in 32 bits
                frac_increment := to_signed(increment_interval, 16) * 
                                  resize(freq_correction_ppb_i(23 downto 0), 16);  -- Limit to ±8M PPB
                
                -- Accumulate and check overflow
                new_frac := frac_ns_accum + frac_increment;
                ns_adjust := 0;
                
                if new_frac >= FRAC_OVERFLOW then
                    new_frac := new_frac - FRAC_OVERFLOW;
                    ns_adjust := 1;
                elsif new_frac <= FRAC_UNDERFLOW then
                    new_frac := new_frac - FRAC_UNDERFLOW;  -- Subtracting negative = adding
                    ns_adjust := -1;
                end if;
                
                frac_ns_accum <= new_frac;
                
                -- Update nanoseconds
                new_nsec := nsec_reg + to_signed(increment_interval + ns_adjust, 32);
                
                -- Handle second rollover
                if new_nsec >= NS_PER_SEC then
                    nsec_reg <= new_nsec - NS_PER_SEC;
                    sec_reg  <= sec_reg + 1;
                    second_pulse_o <= '1';
                elsif new_nsec < 0 then
                    -- Can happen with negative adjustment at nsec=0
                    nsec_reg <= new_nsec + NS_PER_SEC;
                    sec_reg  <= sec_reg - 1;
                else
                    nsec_reg <= new_nsec;
                end if;
            end if;
        end if;
    end process;

end Behavioral;
