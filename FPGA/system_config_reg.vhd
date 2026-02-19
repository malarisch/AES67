
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
        ip_addr_out     : out std_logic_vector(31 downto 0);

		ptp_current_leader_id: out std_logic_vector(63 downto 0);
		ptp_time_source_o: out std_logic_vector(7 downto 0);
		ptp_log_message_interval_o: out std_logic_vector(7 downto 0);
		ptp_announce_log_message_interval_o: out std_logic_vector(7 downto 0);
		
		sys_clock_i		: in std_logic
	);
end system_config_reg;

architecture Behavioral of system_config_reg is
	type t_ram is array(lastAddress downto 0) of std_logic_vector(7 downto 0);
	signal ram: t_ram;
	
	-- Sequential read state machine
	type state_t is (IDLE, READ_SEQ);
	signal state      : state_t := IDLE;
	signal read_cnt   : unsigned(4 downto 0) := (others => '0');  -- 0..21
	signal ram_data   : std_logic_vector(7 downto 0);
	signal read_addr  : unsigned(4 downto 0) := (others => '0');

	signal done_i_sync : std_logic := '0';
	signal done_i_sync_1 : std_logic := '0';
	signal done_i_sync_2 : std_logic := '0';
	
begin
	-- Single registered RAM read (breaks combinatorial path)
	process(wr_clk)
	begin
		if rising_edge(wr_clk) then
			ram_data <= ram(to_integer(read_addr));
		end if;
	end process;

	-- Main state machine: sequential RAM read pipeline
	process(wr_clk)
	begin
		if rising_edge(wr_clk) then
			
			
			-- write to RAM (independent of read state machine)
			if (wr_enable = '1') and (writeAddr <= lastAddress) then
				ram(to_integer(writeAddr)) <= data_in;
			end if;
		end if;
	end process;
	

	process(sys_clock_i)
	begin
		if rising_edge(sys_clock_i) then
			done_i_sync <= done_in;
			done_i_sync_1 <= done_i_sync;
			done_i_sync_2 <= done_i_sync_1;
			
			case state is
				when IDLE =>
					
					if (done_i_sync_2 = '1') then
						valid_out <= '0';
						state     <= READ_SEQ;
						read_addr <= (others => '0');
						read_cnt  <= (others => '0');
					end if;
					
				when READ_SEQ =>
					-- Advance address for next cycle (1 cycle RAM latency)
					if read_cnt < 21 then
						read_addr <= read_addr + 1;
						read_cnt  <= read_cnt + 1;
					end if;
					
					-- Write directly to output ports (delayed by 1 due to RAM latency)
					case to_integer(read_cnt) is
						-- MAC address bytes 0-5
						when 1 => mac_addr_out(47 downto 40) <= ram_data;
						when 2 => mac_addr_out(39 downto 32) <= ram_data;
						when 3 => mac_addr_out(31 downto 24) <= ram_data;
						when 4 => mac_addr_out(23 downto 16) <= ram_data;
						when 5 => mac_addr_out(15 downto 8)  <= ram_data;
						when 6 => mac_addr_out(7 downto 0)   <= ram_data;
						
						-- IP address bytes 6-9
						when 7  => ip_addr_out(31 downto 24) <= ram_data;
						when 8  => ip_addr_out(23 downto 16) <= ram_data;
						when 9  => ip_addr_out(15 downto 8)  <= ram_data;
						when 10 => ip_addr_out(7 downto 0)   <= ram_data;
						
						-- PTP leader ID bytes 10-17
						when 11 => ptp_current_leader_id(63 downto 56) <= ram_data;
						when 12 => ptp_current_leader_id(55 downto 48) <= ram_data;
						when 13 => ptp_current_leader_id(47 downto 40) <= ram_data;
						when 14 => ptp_current_leader_id(39 downto 32) <= ram_data;
						when 15 => ptp_current_leader_id(31 downto 24) <= ram_data;
						when 16 => ptp_current_leader_id(23 downto 16) <= ram_data;
						when 17 => ptp_current_leader_id(15 downto 8)  <= ram_data;
						when 18 => ptp_current_leader_id(7 downto 0)   <= ram_data;
						
						-- PTP config bytes 18-20
						when 19 => ptp_time_source_o          <= ram_data;
						when 20 => ptp_log_message_interval_o <= ram_data;
						when 21 => 
							ptp_announce_log_message_interval_o <= ram_data;
							valid_out <= '1';
							state     <= IDLE;
							
						when others => null;
					end case;
			end case;
		end if;
	end process;
	data0_out <= ram(to_integer(read0Addr));
	data1_out <= ram(to_integer(read1Addr));
	data2_out <= ram(to_integer(read2Addr));
end Behavioral;
