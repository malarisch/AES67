library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity ptpv2_controller is
    port(
        clk                 : in  std_logic;
        reset_n             : in  std_logic;
        tx_message_type_o   : out std_logic_vector(3 downto 0);
        sequence_id_o       : out unsigned(15 downto 0);
        frame_start_o       : out std_logic;
        request_port_identity_o : out std_logic_vector(79 downto 0);
        wallclock_seconds_i     : in  unsigned(31 downto 0);
        wallclock_nanoseconds_i : in  unsigned(31 downto 0);
        tx_ready_timestamp_seconds_i     : in  unsigned(31 downto 0);
        tx_ready_timestamp_nanoseconds_i : in  unsigned(31 downto 0);
        timestamp_seconds_o         : out unsigned(31 downto 0);
        timestamp_nanoseconds_o     : out unsigned(31 downto 0);
        rx_timestamp_seconds_i     : in  unsigned(31 downto 0);
        rx_timestamp_nanoseconds_i : in  unsigned(31 downto 0);


        sequence_id_i         : in  unsigned(15 downto 0);
        send_delay_resp_in        : in std_logic;

        second_pulse_i       : in  std_logic;

        is_leader_override_i   : in  std_logic;

        tx_en_i: in  std_logic;
        request_port_identity_i : in std_logic_vector(79 downto 0)
    );
    type t_state_leader is (s_Idle, s_Send_Sync, s_Wait_for_Sync_Ready, s_Send_Follow_Up, s_Wait_for_Follow_Up_Ready, s_Send_Delay_Resp, s_Wait_for_Delay_Resp_Ready);

    signal leader_state : t_state_leader := s_Idle;

    signal sequence_id_reg : unsigned(15 downto 0) := (others => '0');
    signal tx_en_i_reg : std_logic := '0';
    signal tx_en_i_sync : std_logic := '0';

    signal is_leader : std_logic := '0';
    signal second_pulse_i_reg : std_logic := '0';
    signal send_delay_resp_in_reg : std_logic := '0';
    signal sequence_id_i_reg : unsigned(15 downto 0) := (others => '0');
    signal request_port_identity_i_reg : std_logic_vector(79 downto 0) := (others => '0');
    signal send_delay_resp : std_logic := '0';
    signal rx_timestamp_nanoseconds_i_reg : unsigned(31 downto 0) := (others => '0');
    signal rx_timestamp_seconds_i_reg : unsigned(31 downto 0) := (others => '0');


    end entity;
architecture Behavioral of ptpv2_controller is
begin
    
    process(clk, reset_n)
    begin
        if reset_n = '0' then
            tx_message_type_o       <= (others => '0');
            sequence_id_o           <= (others => '0');
            frame_start_o           <= '0';
            request_port_identity_o <= (others => '0');
            sequence_id_reg         <= (others => '0');
            tx_en_i_reg             <= '0';
            is_leader               <= '0';
            second_pulse_i_reg      <= '0';
            leader_state           <= s_Idle;
            send_delay_resp_in_reg  <= '0';
            timestamp_seconds_o     <= (others => '0');
            timestamp_nanoseconds_o <= (others => '0');
        elsif rising_edge(clk) then
            tx_en_i_reg <= tx_en_i;
            tx_en_i_sync <= tx_en_i_reg;
            second_pulse_i_reg <= second_pulse_i;
            send_delay_resp_in_reg <= send_delay_resp_in;


            if (tx_en_i_reg = '0' and tx_en_i = '1') then
                frame_start_o <= '0';
            end if;

            -- PTPv2 controller logic to be implemented here
            if (is_leader_override_i = '1') then
                is_leader <= '1';
            else
                is_leader <= '0';
            end if;

            if (is_leader = '1') then
                -- Leader logic
                -- Send a Sync Message every second and a Follow Up Message after that
                case leader_state is
                    when s_Idle =>
                        if (second_pulse_i_reg = '0' and second_pulse_i = '1') then
                            leader_state <= s_Send_Sync;
                        end if;
                        if (send_delay_resp = '1') then
                            send_delay_resp <= '0'; -- reset send_delay_resp signal when acknowledged
                            leader_state <= s_Send_Delay_Resp;
                        end if;

                    when s_Send_Sync =>
                        
                        tx_message_type_o <= "0000";
                        frame_start_o <= '1';
                        sequence_id_reg <= sequence_id_reg + 1;
                        sequence_id_o <= sequence_id_reg + 1;
                        leader_state <= s_Wait_for_Sync_Ready;
                        timestamp_nanoseconds_o <= wallclock_nanoseconds_i;
                        timestamp_seconds_o <= wallclock_seconds_i;

                    when s_Wait_for_Sync_Ready =>
                        
                        if (tx_en_i_sync = '1' and tx_en_i_reg = '0') then
                            leader_state <= s_Send_Follow_Up;
                            frame_start_o <= '0';
                            timestamp_nanoseconds_o <= tx_ready_timestamp_nanoseconds_i;
                            timestamp_seconds_o <= tx_ready_timestamp_seconds_i;
                        end if;

                    when s_Send_Follow_Up =>
                        tx_message_type_o <= x"8";
                        frame_start_o <= '1';
                        
                        sequence_id_o <= sequence_id_reg;
                        leader_state <= s_Wait_for_Follow_Up_Ready;
                        
                    when s_Wait_for_Follow_Up_Ready =>
                        if (tx_en_i_sync = '1' and tx_en_i_reg = '0') then
                            leader_state <= s_Idle;
                        end if;
                        
                    when s_Send_Delay_Resp =>
                        tx_message_type_o <= "1001";
                        frame_start_o <= '1';
                        sequence_id_o <= sequence_id_i_reg;
                        request_port_identity_o <= request_port_identity_i_reg;
                        leader_state <= s_Wait_for_Delay_Resp_Ready;
                        timestamp_nanoseconds_o <= rx_timestamp_nanoseconds_i_reg; -- output the rx timestamps received from the parser
                        timestamp_seconds_o <= rx_timestamp_seconds_i_reg;

                    when s_Wait_for_Delay_Resp_Ready =>
                        if (tx_en_i_sync = '1' and tx_en_i_reg = '0') then
                            leader_state <= s_Idle;
                        end if;
                    when others =>
                        null;

                end case;
                if (send_delay_resp_in_reg = '0' and send_delay_resp_in = '1') then
                    send_delay_resp <= '1';
                    request_port_identity_i_reg <= request_port_identity_i;
                    sequence_id_i_reg <= sequence_id_i;
                    rx_timestamp_nanoseconds_i_reg <= rx_timestamp_nanoseconds_i;
                    rx_timestamp_seconds_i_reg <= rx_timestamp_seconds_i;
                end if;

                

            else
                -- Follower logic

            end if;



        end if;
    end process;

end architecture;