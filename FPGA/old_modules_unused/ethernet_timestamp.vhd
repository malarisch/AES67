library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ============================================================
-- Ethernet Timestamp latcher
-- ============================================================

entity ethernet_timestamp is
    generic (
        MAC_TO_PHY_NS_100M : integer := 0;
        -- Measurements with LA:
        -- RX:
        -- MII to MAC: 2 RXCLK Cycles -> 10 sysclk cycles -> 80ns
        -- RMII TO MII: 40 ns

        -- TX: RMII TO MII 1 RXCLK Cycle => 40ns
        -- MII to MAC: 4 RXCLK Cycles => 20 Sysclk Cycles -> 160ns

        -- LAN8720A round trip with loopback: 86 sysclk periods = 688ns rtt
        -- LAN8720A datasheet typical values:
        --   TX (MII -> wire):  ~90ns  (scrambler + MLT-3 encode)
        --   RX (wire -> MII): ~250ns  (descrambler + decode + elastic buffer)
        -- Sum 90+250 = 340ns ~ measured 344ns one-way average. Asymmetry matters
        -- for PTP since slave offset = (RTT/2) only if TX and RX are symmetric.
        MAC_TO_PHY_NS_1G : integer := 0;
        PHY_TX_TO_WIRE_NS : integer := 0;
        PHY_WIRE_TO_RX_NS : integer := 0;
        PATH : string := "TX"
    );
    port(
        clk			: in std_logic; -- sys clock domain

        mac_speed_i     : in std_logic_vector(1 downto 0);
        reset_n			: in std_logic;
        
        wallclock_seconds_i : in unsigned(3 downto 0);
        wallclock_nanoseconds_i : in unsigned(31 downto 0);
        timestamp_seconds_o	: out unsigned(3 downto 0);
        timestamp_nanoseconds_o : out unsigned(31 downto 0);
        

        -- signals from ethernet clock domain
        sof_tog_i : in std_logic


        
    );
end ethernet_timestamp;

architecture Behavioral of ethernet_timestamp is

    -- synchronizers
    signal sof_tog_i_meta : std_logic := '0';
    signal sof_tog_i_sync : std_logic := '0';
    signal sof_tog_i_prev : std_logic := '0';

    signal PHY_LATENCY : integer;
    signal PIPELINE_LATENCY : integer;

    signal s_reg : UNSIGNED(3 downto 0);
    signal stamp_valid : std_logic := '0';
    signal adjusted : integer;
begin

    txphylat: if (PATH = "TX") generate
    PHY_LATENCY <= (MAC_TO_PHY_NS_100M + PHY_TX_TO_WIRE_NS) when mac_speed_i = "01" else
                   (MAC_TO_PHY_NS_1G   + PHY_TX_TO_WIRE_NS);
    PIPELINE_LATENCY <= 0 when mac_speed_i = "10" else (-20 * 8);

    end generate;

    rxphylat: if (PATH = "RX") generate
    PHY_LATENCY <= -(MAC_TO_PHY_NS_100M + PHY_WIRE_TO_RX_NS) when mac_speed_i = "01" else
                   -(MAC_TO_PHY_NS_1G   + PHY_WIRE_TO_RX_NS);
    PIPELINE_LATENCY <= 24 when mac_speed_i = "10" else -24;
    end generate;
    process(clk, reset_n)
        variable correction : integer;
    begin
        if reset_n = '0' then
            sof_tog_i_meta <= '0';
            sof_tog_i_sync <= '0';
            sof_tog_i_prev <= '0';
            stamp_valid <= '0';
        elsif rising_edge(clk) then
            -- Synchronize timestamp_set_i into clk domain
            sof_tog_i_meta <= sof_tog_i;
            sof_tog_i_sync <= sof_tog_i_meta;
            sof_tog_i_prev <= sof_tog_i_sync;

            stamp_valid <= '0';
            if (sof_tog_i_prev /= sof_tog_i_sync) then
                correction := PHY_LATENCY - PIPELINE_LATENCY;
                adjusted   <= to_integer(wallclock_nanoseconds_i) + correction;
                s_reg <= wallclock_seconds_i;
                stamp_valid <= '1';
            end if;


        end if;
    end process;
    output_proc: process (clk) begin
        if rising_edge(clk) then
            if (stamp_valid = '1') then
                if adjusted >= 1000000000 then
                    timestamp_nanoseconds_o <= to_unsigned(adjusted - 1000000000, 32);
                    timestamp_seconds_o     <= s_reg + 1;
                elsif adjusted < 0 then
                    timestamp_nanoseconds_o <= to_unsigned(adjusted + 1000000000, 32);
                    timestamp_seconds_o     <= s_reg - 1;
                else
                    timestamp_nanoseconds_o <= to_unsigned(adjusted, 32);
                    timestamp_seconds_o     <= s_reg;
                end if;
            end if;
        end if;
    end process;
end Behavioral;