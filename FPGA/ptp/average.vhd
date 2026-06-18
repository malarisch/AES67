library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
-- moving average with ^2 delay line width
entity average is
    generic (
        DATA_WIDTH: integer := 16;
        DEPTH : integer := 4
    );
    port (
        clk_i   : in std_logic;
        rst_n_i : in std_logic;
        data_i : in SIGNED(DATA_WIDTH - 1 downto 0);
        data_valid_i : in std_logic;
        data_o : out SIGNED(DATA_WIDTH - 1 downto 0);
        data_valid_o : out std_logic
    );
end entity;

architecture rtl of average is
    function clog2(n : positive) return natural is
        variable result : natural := 0;
        variable val    : natural := n - 1;
    begin
        while val > 0 loop
            result := result + 1;
            val := val / 2;
        end loop;
        return result;
    end function;


	type t_delay_line is array (0 to DEPTH - 1) of signed(DATA_WIDTH - 1 downto 0);
	signal delay_line : t_delay_line := (others => (others => '0'));
    signal delay_line_pointer : integer range 0 to DEPTH -1 := 0;
    signal data_valid_z : std_logic;
    signal accumulator : signed(DATA_WIDTH - 1 + clog2(depth) downto 0);
begin

process (clk_i, rst_n_i)
begin
    if (rst_n_i = '0') then
        delay_line <= (others => (others => '0'));
        delay_line_pointer <= 0;
        data_valid_z <= '0';
        accumulator <= (others => '0');
    elsif rising_edge(clk_i) then
        data_valid_z <= data_valid_i;
        data_valid_o <= data_valid_z;
        if (data_valid_i = '1' and data_valid_z = '0') then
            delay_line(delay_line_pointer) <= data_i;
            accumulator <= accumulator + data_i - delay_line(delay_line_pointer);
            if (delay_line_pointer + 1 = DEPTH) then
                delay_line_pointer <= 0;
            else
                delay_line_pointer <= delay_line_pointer +1;
            end if;
        end if;
    end if;
end process;

data_o <= accumulator(DATA_WIDTH - 1 + clog2(depth) downto clog2(DEPTH));
    

end architecture;