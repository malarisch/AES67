library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.wallclock_signals_pkg.all;
-- ============================================================
-- Ethernet Timestamp latcher
-- ============================================================

entity ethernet_timestamp_mii is
    generic (
        PATH : string := "TX";
        MII_WIDTH : integer := 2;
        SYS_CLK_NS_PER_TICK : integer := 8; -- 125 MHz
        MII_CLK_NS_PER_TICK : integer := 20 -- 50 MHz
    );
    port(
        sys_clk_i			: in std_logic; -- sys clock domain
        mii_clk_i           : in std_logic;
        
        reset_n			: in std_logic;
        
        wallclock_i : in t_wallclock_signals;
        timestamp_o : out t_eth_timestamp;
        mii_in : in STD_LOGIC_VECTOR(MII_WIDTH-1 downto 0);
        gmii_in : in STD_LOGIC_VECTOR(7 downto 0);
        mii_en_in : in STD_LOGIC
    );
end ethernet_timestamp_mii;

architecture Behavioral of ethernet_timestamp_mii is

    -- synchronizers
    signal sof_tog : std_logic := '0';
    signal sof_tog_i_meta : std_logic := '0';
    signal sof_tog_i_sync : std_logic := '0';
    signal sof_tog_i_prev : std_logic := '0';

    signal PIPELINE_LATENCY : integer := 3*SYS_CLK_NS_PER_TICK;
    signal MII_LATENCY : integer := (MII_CLK_NS_PER_TICK/2) + MII_CLK_NS_PER_TICK; -- 3 Sys Clocks for CDC, Half of a Mii clock to account for Asynchronus clock latency

    signal s_reg : UNSIGNED(3 downto 0);
    signal stamp_valid : std_logic := '0';
    signal adjusted : integer;
    signal mii_en_prev : std_logic;
    signal mii_data_prev : STD_LOGIC_VECTOR(MII_WIDTH - 1 downto 0);
    signal gmii_data_reg : STD_LOGIC_VECTOR(7 downto 0);
    signal gmii_data_prev : STD_LOGIC_VECTOR(7 downto 0);

    type t_rmii_sm is (s_Idle, s_Wait1, s_Wait2, s_Wait3);
    signal rmii_sm : t_rmii_sm := s_Idle;
begin

rmii_sof_gen: if MII_WIDTH = 2 generate -- rmii
    rmii_sof_qualifier: process (mii_clk_i)
    begin
        if rising_edge(mii_clk_i) then
            mii_en_prev <= mii_en_in;
            mii_data_prev <= mii_in;
            if (mii_en_in = '0') then
                rmii_sm <= s_Idle;
            end if;
            -- Preamble dibits (LSB-first of 0x55) = "01" repeated.
            -- SFD (0xD5) LSB-first = dibits "01","01","01","11".
            -- The "11" dibit ends the SFD and marks the SOF; the next mii_clk
            -- edge after this case-evaluation is the start of the first byte
            -- of frame data, so toggling here marks the SOF precisely.
            case rmii_sm is
                when s_Idle =>
                    if (mii_en_prev = '0' and mii_en_in = '1') then
                        rmii_sm <= s_Wait1;
                    end if;
                when s_Wait1 =>
                    -- Wait until we see the SFD's terminating "11" dibit
                    -- following a preamble "01" dibit.
                    if (mii_in = "11" and mii_data_prev = "01") then
                        sof_tog <= not sof_tog;
                        rmii_sm <= s_Idle;
                    end if;
                when others =>
                    rmii_sm <= s_Idle;
            end case;

        end if;
    end process;
end generate;
gmii_sof_gen: if (MII_WIDTH = 4 or MII_WIDTH = 8) and (MII_CLK_NS_PER_TICK = 8) generate -- rgmii
    
    gmii_sof_qualifier: process (mii_clk_i)
    begin
        if rising_edge(mii_clk_i) then
            mii_en_prev <= mii_en_in;
            gmii_data_reg <= gmii_in;
            gmii_data_prev <= gmii_data_reg;
            if (mii_en_in = '0') then
                rmii_sm <= s_Idle;
            end if;
            case rmii_sm is
                when s_Idle =>
                    if (mii_en_prev = '0' and mii_en_in = '1') then
                        rmii_sm <= s_Wait1;
                    end if;
                when s_Wait1 =>
                    if gmii_data_reg = x"55" then
                        rmii_sm <= s_Wait2;
                    end if;
                when s_Wait2 =>
                    if (gmii_data_reg = x"D5" and gmii_data_prev = x"55") then
                        sof_tog <= not sof_tog;
                        rmii_sm <= s_Idle;
                    end if;
                when s_Wait3 =>
                    rmii_sm <= s_Idle;
                when others => 
                    rmii_sm <= s_Idle;
            end case;

        end if;
    end process;
end generate;
    mii_sof_gen: if MII_WIDTH = 4 and MII_CLK_NS_PER_TICK = 40 generate -- mii
    
    mii_sof_qualifier: process (mii_clk_i)
    begin
        if rising_edge(mii_clk_i) then
            mii_en_prev <= mii_en_in;
            mii_data_prev <= mii_in;
            if (mii_en_in = '0') then
                rmii_sm <= s_Idle;
            end if;
            case rmii_sm is
                when s_Idle =>
                    if (mii_en_prev = '0' and mii_en_in = '1') then
                        if (mii_in = "1101") then
                            sof_tog <= not sof_tog;
                            rmii_sm <= s_Idle;
                        else
                            rmii_sm <= s_Wait1;
                        end if;
                    end if;
                when s_Wait1 =>
                    if (mii_in = "1101") then
                        sof_tog <= not sof_tog;
                        rmii_sm <= s_Idle;
                    end if;
                when s_Wait2 =>
                    rmii_sm <= s_Idle;
                when s_Wait3 =>
                    rmii_sm <= s_Idle;
                when others => 
                    rmii_sm <= s_Idle;
            end case;

        end if;
    end process;
end generate;

    process(sys_clk_i, reset_n)
    begin
        if reset_n = '0' then
            sof_tog_i_meta <= '0';
            sof_tog_i_sync <= '0';
            sof_tog_i_prev <= '0';
            stamp_valid <= '0';
        elsif rising_edge(sys_clk_i) then
            -- Synchronize timestamp_set_i into clk domain
            sof_tog_i_meta <= sof_tog;
            sof_tog_i_sync <= sof_tog_i_meta;
            sof_tog_i_prev <= sof_tog_i_sync;
            
            stamp_valid <= '0';
            if (sof_tog_i_prev /= sof_tog_i_sync) then
                if (PATH = "TX") then
                    adjusted   <= to_integer(wallclock_i.wallclock_nanoseconds_o) - PIPELINE_LATENCY + MII_LATENCY;
                else
                    adjusted   <= to_integer(wallclock_i.wallclock_nanoseconds_o) - PIPELINE_LATENCY - MII_LATENCY;
                end if;
                s_reg <= wallclock_i.wallclock_seconds_o(3 downto 0);
                stamp_valid <= '1';
            end if;


        end if;
    end process;
    output_proc: process (sys_clk_i) begin
        if rising_edge(sys_clk_i) then
            if (stamp_valid = '1') then
                if adjusted >= 1000000000 then
                    timestamp_o.nanoseconds <= to_unsigned(adjusted - 1000000000, 30);
                    timestamp_o.seconds     <= s_reg + 1;
                elsif adjusted < 0 then
                    timestamp_o.nanoseconds <= to_unsigned(adjusted + 1000000000, 30);
                    timestamp_o.seconds     <= s_reg - 1;
                else
                    timestamp_o.nanoseconds <= to_unsigned(adjusted, 30);
                    timestamp_o.seconds     <= s_reg;
                end if;
            end if;
        end if;
    end process;
end Behavioral;