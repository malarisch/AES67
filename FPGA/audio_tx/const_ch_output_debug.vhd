library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity const_ch_output_debug is

    port
    (

        ch0_o		: out std_logic_vector(23 downto 0); -- 8 bit
        ch1_o		: out std_logic_vector(23 downto 0); -- 8 bit
        ch2_o		: out std_logic_vector(23 downto 0); -- 8 bit
        ch3_o		: out std_logic_vector(23 downto 0); -- 8 bit
        ch4_o		: out std_logic_vector(23 downto 0); -- 8 bit
        ch5_o		: out std_logic_vector(23 downto 0); -- 8 bit
        ch6_o		: out std_logic_vector(23 downto 0); -- 8 bit
        ch7_o		: out std_logic_vector(23 downto 0); -- 8 bit
        ch8_o		: out std_logic_vector(23 downto 0); -- 8 bit
        ch9_o		: out std_logic_vector(23 downto 0); -- 8 bit
        ch10_o		: out std_logic_vector(23 downto 0); -- 8 bit
        ch11_o		: out std_logic_vector(23 downto 0); -- 8 bit
        ch12_o		: out std_logic_vector(23 downto 0); -- 8 bit
        ch13_o		: out std_logic_vector(23 downto 0); -- 8 bit
        ch14_o		: out std_logic_vector(23 downto 0); -- 8 bit
        ch15_o		: out std_logic_vector(23 downto 0) -- 8 bit
        
    );
end entity;

architecture Behavioral of const_ch_output_debug is
begin

    ch0_o <= x"030201";
    ch1_o <= x"131211";
    ch2_o <= x"232221";
    ch3_o <= x"333231";
    ch4_o <= x"434241";
    ch5_o <= x"535251";
    ch6_o <= x"636261";
    ch7_o <= x"737271";
    ch8_o <= x"838281";
    ch9_o <= x"939291";
    ch10_o <= x"A3A2A1";
    ch11_o <= x"B3B2B1";
    ch12_o <= x"C3C2C1";
    ch13_o <= x"D3D2D1";
    ch14_o <= x"E3E2E1";
    ch15_o <= x"F3F2F1";

end Behavioral;