library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;
entity ptpv2_parser is
    port(
        clk                 : in std_logic;
        ram_data            : in std_logic_vector(7 downto 0);
        ram_read_address    : out unsigned(10 downto 0);
        parse_ptp_packet    : in std_logic;
        is_leader           : in std_logic; -- '1' if this node is PTP leader; will answer to delay_req

        rx_timestamp_seconds_i     : in std_logic_vector(31 downto 0);
        rx_timestamp_nanoseconds_i : in std_logic_vector(31 downto 0);

        rx_timestamp_seconds_o     : out std_logic_vector(31 downto 0);
        rx_timestamp_nanoseconds_o : out std_logic_vector(31 downto 0);
        rx_follower_identity_o      : out std_logic_vector(79 downto 0);

        send_delay_resp_o         : out std_logic;

        sequence_id_o              : out std_logic_vector(15 downto 0)
        
    );
end entity;

architecture Behavioral of ptpv2_parser is
    type t_SM_PtpParser is (s_Idle, s_ReadHeader, s_Interpret_Packet, s_Done);
    signal s_SM_PtpParser : t_SM_PtpParser := s_Idle;
    signal byte_counter   : integer range 0 to 1500 := 0;

    signal parse_ptp_packet_reg : std_logic := '0';

    signal udp_port: std_logic_vector(15 downto 0);
    signal udp_length: std_logic_vector(15 downto 0);
    signal ptp_message_type: std_logic_vector(7 downto 0);
    signal ptp_message_length: std_logic_vector(15 downto 0);
    signal ptp_domain_number: std_logic_vector(7 downto 0);
    signal ptp_flag_field: std_logic_vector(15 downto 0);
    signal ptp_correction_field: std_logic_vector(63 downto 0);
    signal ptp_source_port_identity: std_logic_vector(63 downto 0);
    signal ptp_source_port_port_number: std_logic_vector(15 downto 0);
    signal ptp_sequence_id: std_logic_vector(15 downto 0);
    signal ptp_control_field: std_logic_vector(7 downto 0);
    signal ptp_log_msg_interval: std_logic_vector(7 downto 0);
    signal ptp_origin_timestamp_seconds: std_logic_vector(47 downto 0);
    signal ptp_origin_timestamp_nanoseconds: std_logic_vector(31 downto 0);

begin
    process(clk)
    begin
        if rising_edge(clk) then
            parse_ptp_packet_reg <= parse_ptp_packet;
            send_delay_resp_o <= '0';


            if (s_SM_PtpParser = s_Idle) then
                if (parse_ptp_packet_reg = '0' and parse_ptp_packet = '1') then
                    -- start parsing PTPv2 packet
                    byte_counter <= 0;
                    ram_read_address <= to_unsigned(0, 11);
                    s_SM_PtpParser <= s_ReadHeader;
                end if;

            elsif (s_SM_PtpParser = s_ReadHeader) then
                -- read PTPv2 header fields based on byte_counter
                case byte_counter is
                    when 36 =>
                        udp_port(15 downto 8) <= ram_data; -- UDP Destination Port
                    when 37 =>
                        udp_port(7 downto 0) <= ram_data; -- UDP Destination Port
                    when 38 =>
                        udp_length(15 downto 8) <= ram_data; -- UDP Length
                    when 39 =>
                        udp_length(7 downto 0) <= ram_data; -- UDP Length
                    when 42 =>
                        ptp_message_type(7 downto 0) <= ram_data; -- PTP Message Type
                    when 44 =>
                        ptp_message_length(15 downto 8) <= ram_data; -- PTP Message Length
                    when 45 =>
                        ptp_message_length(7 downto 0) <= ram_data; -- PTP Message Length
                    when 46 =>
                        ptp_domain_number(7 downto 0) <= ram_data; -- PTP Domain Number
                    when 48 =>
                        ptp_flag_field(15 downto 8) <= ram_data; -- PTP Flag Field
                    when 49 =>
                        ptp_flag_field(7 downto 0) <= ram_data; -- PTP Flag Field
                    when 50 =>
                        ptp_correction_field(63 downto 56) <= ram_data; -- PTP Correction Field
                    when 51 =>
                        ptp_correction_field(55 downto 48) <= ram_data; -- PTP Correction Field
                    when 52 =>
                        ptp_correction_field(47 downto 40) <= ram_data; -- PTP Correction Field
                    when 53 =>
                        ptp_correction_field(39 downto 32) <= ram_data; -- PTP Correction Field
                    when 54 =>
                        ptp_correction_field(31 downto 24) <= ram_data; -- PTP Correction Field
                    when 55 =>
                        ptp_correction_field(23 downto 16) <= ram_data; -- PTP Correction Field
                    when 56 =>
                        ptp_correction_field(15 downto 8) <= ram_data; -- PTP Correction Field
                    when 57 =>
                        ptp_correction_field(7 downto 0) <= ram_data; -- PTP Correction Field
                    when 62 =>
                        ptp_source_port_identity(63 downto 56) <= ram_data; -- PTP Source Port Identity
                    when 63 =>
                        ptp_source_port_identity(55 downto 48) <= ram_data; -- PTP Source Port Identity
                    when 64 =>
                        ptp_source_port_identity(47 downto 40) <= ram_data; -- PTP Source Port Identity
                    when 65 =>
                        ptp_source_port_identity(39 downto 32) <= ram_data; -- PTP Source Port Identity
                    when 66 =>
                        ptp_source_port_identity(31 downto 24) <= ram_data; -- PTP Source Port Identity
                    when 67 =>
                        ptp_source_port_identity(23 downto 16) <= ram_data; -- PTP Source Port Identity
                    when 68 =>
                        ptp_source_port_identity(15 downto 8) <= ram_data; -- PTP Source Port Identity
                    when 69 =>
                        ptp_source_port_identity(7 downto 0) <= ram_data; -- PTP Source Port Identity
                    when 70 =>
                        ptp_source_port_port_number(15 downto 8) <= ram_data; -- PTP Source Port Port Number
                    when 71 =>
                        ptp_source_port_port_number(7 downto 0) <= ram_data; -- PTP Source Port Port Number
                    when 72 =>
                        ptp_sequence_id(15 downto 8) <= ram_data; -- PTP Sequence ID
                    when 73 =>
                        ptp_sequence_id(7 downto 0) <= ram_data; -- PTP Sequence ID
                    when 74 =>
                        ptp_control_field(7 downto 0) <= ram_data; -- PTP Control Field
                    when 75 =>
                        ptp_log_msg_interval(7 downto 0) <= ram_data; -- PTP Log Message Interval
                    when 76 =>
                        ptp_origin_timestamp_seconds(47 downto 40) <= ram_data; -- PTP Origin Timestamp Seconds
                    when 77 =>
                        ptp_origin_timestamp_seconds(39 downto 32) <= ram_data; -- PTP Origin Timestamp Seconds
                    when 78 =>
                        ptp_origin_timestamp_seconds(31 downto 24) <= ram_data; -- PTP Origin Timestamp Seconds
                    when 79 =>
                        ptp_origin_timestamp_seconds(23 downto 16) <= ram_data; -- PTP Origin Timestamp Seconds
                    when 80 =>
                        ptp_origin_timestamp_seconds(15 downto 8) <= ram_data; -- PTP Origin Timestamp Seconds
                    when 81 =>
                        ptp_origin_timestamp_seconds(7 downto 0) <= ram_data; -- PTP Origin Timestamp Seconds
                    when 82 =>
                        ptp_origin_timestamp_nanoseconds(31 downto 24) <= ram_data; -- PTP Origin Timestamp Nanoseconds
                    when 83 =>
                        ptp_origin_timestamp_nanoseconds(23 downto 16) <= ram_data; -- PTP Origin Timestamp Nanoseconds
                    when 84 =>
                        ptp_origin_timestamp_nanoseconds(15 downto 8) <= ram_data; -- PTP Origin Timestamp Nanoseconds
                    when 85 =>
                        ptp_origin_timestamp_nanoseconds(7 downto 0) <= ram_data; -- PTP Origin Timestamp Nanoseconds
                    when others =>
                        null;
                end case;

                byte_counter <= byte_counter + 1;
                ram_read_address <= to_unsigned(byte_counter + 1, 11);

                if byte_counter = 85 then
                    s_SM_PtpParser <= s_Interpret_Packet;
                end if;



            elsif (s_SM_PtpParser = s_Interpret_Packet) then
                -- interpret the PTPv2 packet based on message type
                if ptp_message_type = x"00" then
                    -- Sync Message
                    -- Handle Sync message processing here
                elsif ptp_message_type = x"01" then
                    -- Delay_Req Message
                    -- Handle Delay_Req message processing here
                    if is_leader = '1' then
                        -- Prepare Delay_Resp message
                        rx_follower_identity_o <= ptp_source_port_identity & ptp_source_port_port_number;
                        rx_timestamp_seconds_o <= rx_timestamp_seconds_i;
                        rx_timestamp_nanoseconds_o <= rx_timestamp_nanoseconds_i;
                        sequence_id_o <= ptp_sequence_id;
                        send_delay_resp_o <= '1';
                        s_SM_PtpParser <= s_Done;
                        

                    end if;
                elsif ptp_message_type = x"02" then
                    -- Pdelay_Req Message
                    -- Handle Pdelay_Req message processing here
                elsif ptp_message_type = x"03" then
                    -- Pdelay_Resp Message
                    -- Handle Pdelay_Resp message processing here
                elsif ptp_message_type = x"08" then
                    -- Follow_Up Message
                    -- Handle Follow_Up message processing here
                    
                elsif ptp_message_type = x"09" then
                    -- Delay_resp Message
                    -- Handle Delay_resp message processing here
                elsif ptp_message_type = x"0A" then
                    -- Pdelay_Resp_Follow_Up Message
                    -- Handle Pdelay_Resp_Follow_Up message processing here
                elsif ptp_message_type = x"0B" then
                    -- Announce Message
                    -- Handle Announce message processing here
                elsif ptp_message_type = x"0C" then
                    -- Signaling Message
                    -- Handle Signaling message processing here
                elsif ptp_message_type = x"0D" then
                    -- Management Message
                    -- Handle Management message processing here
                else
                    -- Unknown Message Type
                end if;

            elsif (s_SM_PtpParser = s_Done) then
                -- done parsing
                s_SM_PtpParser <= s_Idle;
            end if;
        end if;
    end process;
end Behavioral;
            