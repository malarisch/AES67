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
begin
	process(clk_i, rst_n_i)
    begin
        if rst_n_i = '0' then
            ptp_allow_o <= '0';
            audio_allow_o <= '0';
            mcu_allow_o <= '0';
            s_SM_Arbiter <= s_Idle;
        elsif rising_edge(clk_i) then

            case s_SM_Arbiter is
                when s_Idle =>

                    ptp_allow_o <= '0';
                    audio_allow_o <= '0';
                    mcu_allow_o <= '0';

                    if ptp_req_i = '1' then
                        s_SM_Arbiter <= s_txPTP;
                        ptp_allow_o <= '1';
                    elsif mcu_req_i = '1' then
                        s_SM_Arbiter <= s_txMCU;
                        mcu_allow_o <= '1';
                    elsif audio_req_i = '1' then
                        s_SM_Arbiter <= s_txAUDIO;
                        audio_allow_o <= '1';
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
end Behavioral;