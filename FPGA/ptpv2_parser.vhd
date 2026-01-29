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

        rx_timestamp_seconds_i     : in std_logic_vector(47 downto 0);
        rx_timestamp_nanoseconds_i : in std_logic_vector(31 downto 0);

        rx_timestamp_seconds_o     : out std_logic_vector(47 downto 0);
        rx_timestamp_nanoseconds_o : out std_logic_vector(31 downto 0);
        rx_follower_identity_o      : out std_logic_vector(79 downto 0);

        send_delay_resp_o         : out std_logic;


        sequence_id_o              : out std_logic_vector(15 downto 0);
        reset_n             : in std_logic;

        send_delay_req_o         : out std_logic;

        tx_timestamp_seconds_i     : in std_logic_vector(47 downto 0);
        tx_timestamp_nanoseconds_i : in std_logic_vector(31 downto 0);
        t3_valid_i                 : in std_logic;

        src_mac_address		: in std_logic_vector(47 downto 0);

        -- PTP calculation outputs
        mean_path_delay_ns_o       : out signed(63 downto 0);  -- Mean path delay in nanoseconds (signed for negative values)
        offset_from_master_ns_o    : out signed(63 downto 0);  -- Offset from master in nanoseconds (signed)
        ptp_calc_valid_o           : out std_logic              -- Pulse when calculation is complete

    );
end entity;

architecture Behavioral of ptpv2_parser is

    constant my_clock_id : std_logic_vector(79 downto 0) := src_mac_address(47 downto 40) xor x"02" & src_mac_address(39 downto 24) & x"FFFE" & src_mac_address(23 downto 0) & x"0001";    
    -- Constant: 1 second in nanoseconds
    constant ONE_SECOND_NS : signed(63 downto 0) := to_signed(1_000_000_000, 64);
    
    type t_SM_PtpParser is (s_Idle, s_ReadHeader, s_Interpret_Packet, s_Done);
    signal s_SM_PtpParser : t_SM_PtpParser := s_Idle;
    signal byte_counter   : integer range 0 to 1500 := 0;

    signal delay_resp_tx_en_reg : std_logic := '0';

    -- ============================================================
    -- PTP Calculation Function
    -- Converts seconds + nanoseconds timestamp to total nanoseconds (signed 64-bit)
    -- ============================================================
    function timestamp_to_ns(
        seconds     : std_logic_vector(47 downto 0);
        nanoseconds : std_logic_vector(31 downto 0)
    ) return signed is
        variable sec_ns  : signed(63 downto 0);
        variable nsec    : signed(63 downto 0);
        variable mult_result : signed(127 downto 0);
    begin
        -- Convert seconds to nanoseconds (seconds * 1e9)
        -- Multiplication produces 128-bit result, resize to 64-bit
        mult_result := resize(signed('0' & seconds), 64) * ONE_SECOND_NS;
        sec_ns := mult_result(63 downto 0);
        -- Add nanoseconds portion
        nsec := resize(signed('0' & nanoseconds), 64);
        return sec_ns + nsec;
    end function;

    -- ============================================================
    -- Calculate Mean Path Delay and Offset from Master
    -- Formula: 
    --   mean_path_delay = ((t2 - t1) + (t4 - t3)) / 2
    --   offset_from_master = ((t2 - t1) - (t4 - t3)) / 2
    -- ============================================================
    -- Note: This is implemented as inline calculation in the process
    -- because VHDL procedures cannot drive signal outputs directly.

    -- ============================================================
    -- CDC (Clock Domain Crossing) Synchronizers
    -- parse_ptp_packet comes from ethernet_packet_parser (MAC RX clock domain)
    -- ============================================================
    signal parse_ptp_packet_meta : std_logic := '0';
    signal parse_ptp_packet_sync : std_logic := '0';
    signal parse_ptp_packet_prev : std_logic := '0';  -- for edge detection

    -- Prevent register optimization/merging for metastability registers
    attribute PRESERVE : boolean;
    attribute PRESERVE of parse_ptp_packet_meta : signal is true;
    attribute PRESERVE of parse_ptp_packet_sync : signal is true;

    

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

    -- ============================================================
    -- PTP Timestamp Storage (as signals for use with procedure)
    -- ============================================================
    signal stored_t1_seconds     : std_logic_vector(47 downto 0) := (others => '0');
    signal stored_t1_nanoseconds : std_logic_vector(31 downto 0) := (others => '0');
    signal stored_t2_seconds     : std_logic_vector(47 downto 0) := (others => '0');
    signal stored_t2_nanoseconds : std_logic_vector(31 downto 0) := (others => '0');
    signal stored_t3_seconds     : std_logic_vector(47 downto 0) := (others => '0');
    signal stored_t3_nanoseconds : std_logic_vector(31 downto 0) := (others => '0');
    signal stored_t4_seconds     : std_logic_vector(47 downto 0) := (others => '0');
    signal stored_t4_nanoseconds : std_logic_vector(31 downto 0) := (others => '0');


        attribute PRESERVE of stored_t1_nanoseconds : signal is true;
    attribute PRESERVE of stored_t2_nanoseconds : signal is true;
    attribute PRESERVE of stored_t3_nanoseconds : signal is true;
    attribute PRESERVE of stored_t4_nanoseconds : signal is true;
    attribute PRESERVE of stored_t1_seconds : signal is true;
    attribute PRESERVE of stored_t2_seconds : signal is true;
    attribute PRESERVE of stored_t3_seconds : signal is true;
    attribute PRESERVE of stored_t4_seconds : signal is true;
    
    type t_packet_type is (t_Sync, t_Delay_Req, t_Follow_Up, t_Delay_Resp, t_Announce, t_Pdelay_Req, t_Pdelay_Resp, t_Pdelay_Follow_Up);
    
    function get_message_type(message_type : std_logic_vector(3 downto 0)) return t_packet_type is
    begin
        case message_type is
            when x"0" =>
                return t_Sync;
            when x"1" =>
                return t_Delay_Req;
            when x"2" =>
                return t_Pdelay_Req;
            when x"3" =>
                return t_Pdelay_Resp;
            when x"8" =>
                return t_Follow_Up;
            when x"9" =>
                return t_Delay_Resp;
            when x"A" =>
                return t_Pdelay_Follow_Up;
            when x"B" =>
                return t_Announce;
                
            when others =>
                return t_Sync; -- default
        end case;
    end function;

begin
    -- ============================================================
    -- CDC Synchronization Process (separate from main state machine)
    -- This ensures proper timing analysis and prevents race conditions
    -- ============================================================


    
    cdc_sync_proc: process(clk, reset_n)

    begin
        if reset_n = '0' then
            parse_ptp_packet_meta <= '0';
            parse_ptp_packet_sync <= '0';
            parse_ptp_packet_prev <= '0';
        elsif rising_edge(clk) then
            -- 2-stage synchronizer for parse_ptp_packet
            parse_ptp_packet_meta <= parse_ptp_packet;
            parse_ptp_packet_sync <= parse_ptp_packet_meta;
            parse_ptp_packet_prev <= parse_ptp_packet_sync;  -- for edge detection
        end if;
    end process cdc_sync_proc;

    -- ============================================================
    -- Main State Machine Process
    -- Uses only synchronized signals from CDC process
    -- ============================================================
    main_proc: process(clk, reset_n)
        variable active_sequence_id: std_logic_vector(15 downto 0);
        variable requesting_port_identity: std_logic_vector(79 downto 0);
        variable message_length: integer := 85;
        
        -- Variables for PTP calculation
        variable t1_total : signed(63 downto 0);
        variable t2_total : signed(63 downto 0);
        variable t3_total : signed(63 downto 0);
        variable t4_total : signed(63 downto 0);
        variable delta_master_to_slave : signed(63 downto 0);  -- t2 - t1
        variable delta_slave_to_master : signed(63 downto 0);  -- t4 - t3
    begin
        if reset_n = '0' then
            s_SM_PtpParser <= s_Idle;
            byte_counter <= 0;
            send_delay_resp_o <= '0';
            send_delay_req_o <= '0';
            ram_read_address <= (others => '0');
            ptp_calc_valid_o <= '0';
            mean_path_delay_ns_o <= (others => '0');
            offset_from_master_ns_o <= (others => '0');

            active_sequence_id := (others => '0');
            stored_t1_seconds <= (others => '0');
            stored_t1_nanoseconds <= (others => '0');
            stored_t2_seconds <= (others => '0');
            stored_t2_nanoseconds <= (others => '0');
            stored_t3_seconds <= (others => '0');
            stored_t3_nanoseconds <= (others => '0');
            stored_t4_seconds <= (others => '0');
            stored_t4_nanoseconds <= (others => '0');
        elsif rising_edge(clk) then
            send_delay_resp_o <= '0';
            send_delay_req_o <= '0';
            ptp_calc_valid_o <= '0';
            
            -- Capture T3 timestamp when valid
            if (t3_valid_i = '1') then
                stored_t3_seconds <= tx_timestamp_seconds_i;
                stored_t3_nanoseconds <= tx_timestamp_nanoseconds_i;
            end if;

            if (s_SM_PtpParser = s_Idle) then
                -- Detect rising edge on SYNCHRONIZED signal
                if (parse_ptp_packet_prev = '0' and parse_ptp_packet_sync = '1') then
                    -- start parsing PTPv2 packet
                    byte_counter <= 0;
                    ram_read_address <= to_unsigned(0, 11);
                    s_SM_PtpParser <= s_ReadHeader;
                    message_length := 85; -- default length
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
                if (get_message_type(ptp_message_type(3 downto 0)) = t_Delay_Resp) then
                    -- For Sync messages, we can capture T2 timestamp here if needed
                    message_length := 95; -- Delay_Resp messages are longer
                    case byte_counter is
                        when 86 =>
                            -- Capture T2 timestamp seconds
                            requesting_port_identity(79 downto 72) := ram_data;
                        when 87 =>
                            requesting_port_identity(71 downto 64) := ram_data;
                        when 88 =>
                            requesting_port_identity(63 downto 56) := ram_data;
                        when 89 =>
                            requesting_port_identity(55 downto 48) := ram_data;
                        when 90 =>
                            requesting_port_identity(47 downto 40) := ram_data;
                        when 91 =>
                            requesting_port_identity(39 downto 32) := ram_data;
                        when 92 =>
                            requesting_port_identity(31 downto 24) := ram_data;
                        when 93 =>
                            requesting_port_identity(23 downto 16) := ram_data;
                        when 94 =>
                            requesting_port_identity(15 downto 8) := ram_data;
                        when 95 =>
                            requesting_port_identity(7 downto 0) := ram_data;
                        when others =>
                            null;
                    end case;
                end if;
                byte_counter <= byte_counter + 1;
                ram_read_address <= to_unsigned(byte_counter + 1, 11);

                if byte_counter = message_length then
                    s_SM_PtpParser <= s_Interpret_Packet;
                end if;



            elsif (s_SM_PtpParser = s_Interpret_Packet) then
                -- interpret the PTPv2 packet based on message type (lower 4 bits only!)
                -- PTP Byte 0: [ majorSdoId (4 bit) | messageType (4 bit) ]
                case ptp_message_type(3 downto 0) is
                    when x"0" =>
                        -- Sync Message
                        -- Handle Sync message processing here
                        if (is_leader = '0') then
                            active_sequence_id := ptp_sequence_id;
                            -- T2: RX timestamp when Sync was received
                            stored_t2_seconds <= rx_timestamp_seconds_i;
                            stored_t2_nanoseconds <= rx_timestamp_nanoseconds_i;
                        end if;
                    when x"1" =>
                        -- Delay_Req Message
                        if is_leader = '1' then
                            -- Prepare Delay_Resp message
                            rx_follower_identity_o <= ptp_source_port_identity & ptp_source_port_port_number;
                            rx_timestamp_seconds_o <= rx_timestamp_seconds_i;
                            rx_timestamp_nanoseconds_o <= rx_timestamp_nanoseconds_i;
                            sequence_id_o <= ptp_sequence_id;
                            send_delay_resp_o <= '1';
                        end if;
                    when x"2" =>
                        -- Pdelay_Req Message
                        null;
                    when x"3" =>
                        -- Pdelay_Resp Message
                        null;
                    when x"8" =>
                        -- Follow_Up Message
                        if (is_leader = '0') then
                            -- Handle Follow_Up message processing here
                            if ptp_sequence_id = active_sequence_id then
                                -- T1: Origin timestamp from Follow_Up (when Master sent Sync)
                                stored_t1_seconds <= ptp_origin_timestamp_seconds;
                                stored_t1_nanoseconds <= ptp_origin_timestamp_nanoseconds;
                                sequence_id_o <= ptp_sequence_id;
                                send_delay_req_o <= '1';
                            end if;
                        end if;
                        null;
                    when x"9" =>
                        -- Delay_resp Message
                        if (is_leader = '0') then
                            -- Handle Delay_Resp message processing here
                            if ptp_sequence_id = active_sequence_id then -- and requesting_port_identity = my_clock_id 
                                -- T4: Origin timestamp from Delay_Resp (when Master received Delay_Req)
                                stored_t4_seconds <= ptp_origin_timestamp_seconds;
                                stored_t4_nanoseconds <= ptp_origin_timestamp_nanoseconds;
                                
                                -- ============================================
                                -- Calculate Mean Path Delay and Offset
                                -- Formula: 
                                --   mean_path_delay = ((t2 - t1) + (t4 - t3)) / 2
                                --   offset = ((t2 - t1) - (t4 - t3)) / 2
                                -- ============================================
                                
                                -- Convert all timestamps to nanoseconds
                                t1_total := timestamp_to_ns(stored_t1_seconds, stored_t1_nanoseconds);
                                t2_total := timestamp_to_ns(stored_t2_seconds, stored_t2_nanoseconds);
                                t3_total := timestamp_to_ns(stored_t3_seconds, stored_t3_nanoseconds);
                                t4_total := timestamp_to_ns(ptp_origin_timestamp_seconds, ptp_origin_timestamp_nanoseconds);
                                
                                -- Calculate deltas
                                delta_master_to_slave := t2_total - t1_total;
                                delta_slave_to_master := t4_total - t3_total;
                                
                                -- Mean path delay = ((t2-t1) + (t4-t3)) / 2
                                mean_path_delay_ns_o <= shift_right(delta_master_to_slave + delta_slave_to_master, 1);
                                
                                -- Offset from master = ((t2-t1) - (t4-t3)) / 2
                                offset_from_master_ns_o <= shift_right(delta_master_to_slave - delta_slave_to_master, 1);
                                
                                ptp_calc_valid_o <= '1';
                            end if;
                        end if;
                        null;
                    when x"A" =>
                        -- Pdelay_Resp_Follow_Up Message
                        null;
                    when x"B" =>
                        -- Announce Message
                        null;
                    when x"C" =>
                        -- Signaling Message
                        null;
                    when x"D" =>
                        -- Management Message
                        null;
                    when others =>
                        -- Unknown Message Type
                        null;
                end case;
                
                -- WICHTIG: Immer zu s_Done wechseln nach der Interpretation
                s_SM_PtpParser <= s_Done;

            elsif (s_SM_PtpParser = s_Done) then
                -- Wait for parse_ptp_packet to go LOW before returning to Idle
                -- This prevents re-triggering on the same packet
                if (parse_ptp_packet_sync = '0') then
                    s_SM_PtpParser <= s_Idle;
                end if;
            else
                -- Safety: recover from undefined state
                s_SM_PtpParser <= s_Idle;
                byte_counter <= 0;
            end if;
        end if;
    end process main_proc;
end Behavioral;