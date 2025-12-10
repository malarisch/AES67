library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ptp_timer is
  port (
    clk_i       : in  std_ulogic;
    rst_i       : in  std_ulogic;
    
    -- Configuration
    time_inc_i  : in  unsigned(31 downto 0); -- 8.24 fixed point (8 bit ns, 24 bit frac)
    
    -- Adjustment (Strobe interface)
    set_time_i  : in  std_ulogic;
    set_val_i   : in  unsigned(63 downto 0);
    
    -- Current Time
    current_time_o : out unsigned(63 downto 0)
  );
end entity ptp_timer;

architecture rtl of ptp_timer is
  signal counter_ns : unsigned(63 downto 0) := (others => '0');
  signal accum      : unsigned(23 downto 0) := (others => '0');
begin

  process(clk_i, rst_i)
    variable inc_int : unsigned(7 downto 0);
    variable inc_frac : unsigned(23 downto 0);
    variable next_accum : unsigned(24 downto 0); -- 1 bit carry
  begin
    if rst_i = '1' then
      counter_ns <= (others => '0');
      accum      <= (others => '0');
    elsif rising_edge(clk_i) then
      if set_time_i = '1' then
        counter_ns <= set_val_i;
        accum      <= (others => '0');
      else
        inc_int  := time_inc_i(31 downto 24);
        inc_frac := time_inc_i(23 downto 0);
        
        next_accum := resize(accum, 25) + resize(inc_frac, 25);
        
        counter_ns <= counter_ns + resize(inc_int, 64) + resize(next_accum(24 downto 24), 64);
        accum      <= next_accum(23 downto 0);
      end if;
    end if;
  end process;

  current_time_o <= counter_ns;

end architecture rtl;
