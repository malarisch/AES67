
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity system_config_reg is
	generic(
		lastAddress : integer := 512
	);
	port(
		wr_clk			: in std_logic;
		wr_enable		: in std_logic;
		writeAddr		: in unsigned(10 downto 0); -- 0..1531
		data_in			: in std_logic_vector(7 downto 0); -- 8 bit

		done_in			: in std_logic;
		
		read0Addr		: in unsigned(10 downto 0); -- 0..1531
		data0_out		: out std_logic_vector(7 downto 0); -- 8 bit

		read1Addr		: in unsigned(10 downto 0); -- 0..1531
		data1_out		: out std_logic_vector(7 downto 0); -- 8 bit

		read2Addr		: in unsigned(10 downto 0); -- 0..1531
		data2_out		: out std_logic_vector(7 downto 0); -- 8 bit
		
		valid_out			: out std_logic;
        mac_addr_out    : out std_logic_vector(47 downto 0);
        ip_addr_out     : out std_logic_vector(31 downto 0)
	);
end system_config_reg;

architecture Behavioral of system_config_reg is
	type t_ram is array(lastAddress downto 0) of std_logic_vector(7 downto 0);
	signal ram: t_ram;
begin
	-- writing data to ram
	process(wr_clk)
	begin
		if rising_edge(wr_clk) then
			-- set output values when new frame is ready
			if (done_in = '1') then
				valid_out <= '1';
				mac_addr_out <= ram(0) & ram(1) & ram(2) & ram(3) & ram(4) & ram(5);
				ip_addr_out <= ram(6) & ram(7) & ram(8) & ram(9);
				
			else
				valid_out <= '0';
			end if;

			
			-- write to RAM
			if (wr_enable = '1') and (writeAddr <= lastAddress) then
				ram(to_integer(writeAddr)) <= data_in;
			end if;
		end if;
	end process;
	
	data0_out <= ram(to_integer(read0Addr));
	data1_out <= ram(to_integer(read1Addr));
	data2_out <= ram(to_integer(read2Addr));
end Behavioral;
