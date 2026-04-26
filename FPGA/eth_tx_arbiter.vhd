library ieee;
use ieee.std_logic_1164.all;

entity eth_tx_arbiter is
	port
	(
        clk_i		: in std_logic;
        rst_n_i	: in std_logic;

		ptp_req_i    : in std_logic;
        ptp_allow_o   : out std_logic;
		
        audio_req_i  : in std_logic;
        audio_allow_o : out std_logic;

        mcu_req_i    : in std_logic;
        mcu_allow_o   : out std_logic

	);
end entity;

architecture Behavioral of eth_tx_arbiter is
    type t_SM_Arbiter is (s_Idle, s_txPTP, s_txAUDIO, s_txMCU);
    signal s_SM_Arbiter : t_SM_Arbiter := s_Idle;
    attribute syn_encoding : string;
    attribute syn_encoding of s_SM_Arbiter : signal is "safe, one-hot";
    attribute altera_attribute : string;
    attribute altera_attribute of Behavioral : architecture is "-name SAFE_STATE_MACHINE ON";

begin
	process(clk_i, rst_n_i)
    begin
        if rst_n_i = '0' then
            s_SM_Arbiter <= s_Idle;
        elsif rising_edge(clk_i) then

            case s_SM_Arbiter is
                when s_Idle =>


                    if ptp_req_i = '1' then
                        s_SM_Arbiter <= s_txPTP;
                    elsif mcu_req_i = '1' then
                        s_SM_Arbiter <= s_txMCU;
                    elsif audio_req_i = '1' then
                        s_SM_Arbiter <= s_txAUDIO;
                    else
                        s_SM_Arbiter <= s_Idle;
                    end if;
                when s_txPTP =>
                    if ptp_req_i = '0' then
                        s_SM_Arbiter <= s_Idle;
                    else
                        s_SM_Arbiter <= s_txPTP;

                    end if;
                when s_txAUDIO =>
                    if audio_req_i = '0' then
                        s_SM_Arbiter <= s_Idle;
                    else
                        s_SM_Arbiter <= s_txAUDIO;
                    end if;
                when s_txMCU =>
                    if mcu_req_i = '0' then
                        s_SM_Arbiter <= s_Idle;
                    else
                        s_SM_Arbiter <= s_txMCU;
                    end if;
                when others =>
                    s_SM_Arbiter <= s_Idle;
            end case;
        end if;
    end process;
    ptp_allow_o   <= '1' when s_SM_Arbiter = s_txPTP   else '0';
    audio_allow_o <= '1' when s_SM_Arbiter = s_txAUDIO else '0';
    mcu_allow_o   <= '1' when s_SM_Arbiter = s_txMCU   else '0';

end Behavioral;