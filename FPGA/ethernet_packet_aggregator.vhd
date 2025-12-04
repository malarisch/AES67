library ieee;
use ieee.std_logic_1164.all;

entity ethernet_packet_aggregator is
	port
	(
		tx_en0_i	: in std_logic;
		data0_i	: in std_logic_vector(7 downto 0);
		tx_en1_i	: in std_logic;
		data1_i	: in std_logic_vector(7 downto 0);
		tx_en2_i	: in std_logic;
		data2_i	: in std_logic_vector(7 downto 0);
		tx_en3_i	: in std_logic;
		data3_i	: in std_logic_vector(7 downto 0);
		tx_en4_i	: in std_logic;
		data4_i	: in std_logic_vector(7 downto 0);
		
		tx_en5_i	: in std_logic;
		data5_i	: in std_logic_vector(7 downto 0);

		tx_en_o	: out std_logic;
		data_o	: out std_logic_vector(7 downto 0)
	);
end entity;

architecture Behavioral of ethernet_packet_aggregator is
begin
	tx_en_o <= tx_en0_i or tx_en1_i or tx_en2_i or tx_en3_i or tx_en4_i or tx_en5_i;
	data_o <= data0_i or data1_i or data2_i or data3_i or data4_i or data5_i;
end Behavioral;