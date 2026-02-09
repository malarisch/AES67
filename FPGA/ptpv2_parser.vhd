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
        mean_path_delay_ns_o       : out signed(31 downto 0);  -- Mean path delay in nanoseconds (signed)
        offset_from_master_ns_o    : out signed(31 downto 0);  -- Offset from master in nanoseconds (signed)
        ptp_calc_valid_o           : out std_logic;            -- Pulse when calculation is complete
        log_msg_interval_o         : out signed(7 downto 0);   -- PTP logMessageInterval from last Sync/Follow_Up
        log_msg_interval_valid_o   : out std_logic;            -- Pulse when log_msg_interval is updated (from Follow_Up)

        clock_set_o            : out std_logic;
        clock_configured_o   : out std_logic;
        clock_configure_timestamp_seconds_o     : out std_logic_vector(47 downto 0);
        clock_configure_timestamp_nanoseconds_o : out std_logic_vector(31 downto 0)

    );
end entity;

architecture Behavioral of ptpv2_parser is

    -- EUI-64 Clock Identity from MAC address (IEEE 1588)
    -- Format: MAC[47:24] with bit 1 flipped (U/L bit) | 0xFFFE | MAC[23:0] | PortNumber(0x0001)
    -- Total: 80 bits = 64-bit clockIdentity + 16-bit portNumber
    -- NOTE: This must be a signal (not constant) because src_mac_address is a port input
    signal my_clock_id : std_logic_vector(79 downto 0);
    
    -- Constants for seconds-to-nanoseconds conversion (NO DSP multiplication!)
    -- Use lookup table approach since sec_diff is typically -2 to +2
    -- 32-bit signed max is ~2.1e9, so ±2 seconds fits.
    constant ONE_SECOND_NS_POS  : signed(31 downto 0) := to_signed( 1_000_000_000, 32);
    constant TWO_SECONDS_NS_POS : signed(31 downto 0) := to_signed( 2_000_000_000, 32);
    constant ONE_SECOND_NS_NEG  : signed(31 downto 0) := to_signed(-1_000_000_000, 32);
    constant TWO_SECONDS_NS_NEG : signed(31 downto 0) := to_signed(-2_000_000_000, 32);
    
    type t_SM_PtpParser is (s_Idle, s_ReadHeader, s_Interpret_Packet, s_Calc_Stage1, s_Calc_Stage2, s_Done);
    signal s_SM_PtpParser : t_SM_PtpParser := s_Idle;
    signal byte_counter   : integer range 0 to 1500 := 0;

    signal delay_resp_tx_en_reg : std_logic := '0';

    -- ============================================================
    -- PIPELINED PTP Calculation Registers
    -- Pipeline stages:
    --   Stage 1: Calculate delta_m2s = T2 - T1 (registered)
    --   Stage 2: Calculate delta_s2m = T4 - T3 and final results
    -- No calc_t* latches needed: stored_t* are stable during calculation
    -- because no new PTP packet can be parsed while state machine is busy.
    -- ============================================================
    
    -- Intermediate delta result (registered between stages)
    signal delta_m2s_reg : signed(31 downto 0) := (others => '0');  -- T2 - T1

    -- ============================================================
    -- PTP Calculation Function (DSP-FREE, 32-bit VERSION)
    -- Calculate time difference between two timestamps in nanoseconds.
    -- Seconds inputs are only 4 bits wide (lower bits of full timestamp).
    -- Result fits in signed 32 bits (max ±2.1e9 ns ≈ ±2.1 s).
    -- Uses lookup table for sec*1e9 since diff is typically -2..+2.
    -- ============================================================
    function timestamp_diff_ns(
        sec_a  : std_logic_vector(3 downto 0);
        ns_a   : std_logic_vector(31 downto 0);
        sec_b  : std_logic_vector(3 downto 0);
        ns_b   : std_logic_vector(31 downto 0)
    ) return signed is
        variable sec_diff    : signed(4 downto 0);  -- 5 bits for signed 4-bit diff
        variable ns_diff     : signed(32 downto 0);  -- Extra bit for sign
        variable sec_as_ns   : signed(31 downto 0);
        variable result      : signed(31 downto 0);
    begin
        -- Calculate seconds difference (small number, typically -2 to +2)
        sec_diff := resize(signed('0' & sec_a), 5) - resize(signed('0' & sec_b), 5);
        
        -- Calculate nanoseconds difference  
        ns_diff := resize(signed('0' & ns_a), 33) - resize(signed('0' & ns_b), 33);
        
        -- Convert seconds difference to nanoseconds using LOOKUP TABLE
        case to_integer(sec_diff) is
            when  0 => sec_as_ns := (others => '0');
            when  1 => sec_as_ns := ONE_SECOND_NS_POS;
            when  2 => sec_as_ns := TWO_SECONDS_NS_POS;
            when -1 => sec_as_ns := ONE_SECOND_NS_NEG;
            when -2 => sec_as_ns := TWO_SECONDS_NS_NEG;
            when others => 
                -- Clamp to ±2 seconds (should not happen in normal PTP)
                if sec_diff(4) = '1' then  -- negative
                    sec_as_ns := TWO_SECONDS_NS_NEG;
                else
                    sec_as_ns := TWO_SECONDS_NS_POS;
                end if;
        end case;
        
        -- Total difference = seconds_as_ns + nanoseconds_diff
        result := sec_as_ns + resize(ns_diff, 32);
        
        return result;
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

    signal clock_configured: std_logic := '0';


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
    signal ptp_version: std_logic_vector(3 downto 0);  -- PTP version (lower 4 bits of byte 1)
    signal ptp_origin_timestamp_seconds: std_logic_vector(47 downto 0);
    signal ptp_origin_timestamp_nanoseconds: std_logic_vector(31 downto 0);

    signal ptp_announce_current_utc_offset: std_logic_vector(15 downto 0);
    signal ptp_announce_grandmaster_priority1: std_logic_vector(7 downto 0);
    signal ptp_announce_grandmaster_priority2: std_logic_vector(7 downto 0);
    signal ptp_announce_grandmaster_clockClass: std_logic_vector(7 downto 0);
    signal ptp_announce_grandmaster_clockAccuracy: std_logic_vector(7 downto 0);
    signal ptp_announce_grandmaster_offset_scaled_log_variance: std_logic_vector(15 downto 0);
    signal ptp_announce_steps_removed: std_logic_vector(15 downto 0);
    signal ptp_announce_time_source: std_logic_vector(7 downto 0);
    signal ptp_clock_identity: std_logic_vector(63 downto 0);
    

    -- ============================================================
    -- PTP Timestamp Storage
    -- T1: Full 48-bit seconds (needed for initial clock set)
    -- T2, T3, T4: Only lower 4 bits of seconds (enough for ±2s diffs)
    -- All nanoseconds remain full 32 bits
    -- ============================================================
    signal stored_t1_seconds     : std_logic_vector(47 downto 0) := (others => '0');
    signal stored_t1_nanoseconds : std_logic_vector(31 downto 0) := (others => '0');
    signal stored_t2_seconds     : std_logic_vector(3 downto 0) := (others => '0');
    signal stored_t2_nanoseconds : std_logic_vector(31 downto 0) := (others => '0');
    signal stored_t3_seconds     : std_logic_vector(3 downto 0) := (others => '0');
    signal stored_t3_nanoseconds : std_logic_vector(31 downto 0) := (others => '0');
    signal stored_t4_seconds     : std_logic_vector(3 downto 0) := (others => '0');
    signal stored_t4_nanoseconds : std_logic_vector(31 downto 0) := (others => '0');
    
    -- Latched RX timestamp - captured immediately when packet parsing starts
    -- Full seconds kept for delay_resp forwarding; only lower 4 bits used for T2
    signal latched_rx_timestamp_seconds     : std_logic_vector(47 downto 0) := (others => '0');
    signal latched_rx_timestamp_nanoseconds : std_logic_vector(31 downto 0) := (others => '0');


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
    -- Generate Clock Identity from MAC address (concurrent assignment)
    -- EUI-64 format: OUI[23:0] with U/L bit flipped | 0xFFFE | NIC[23:0] | Port 1
    -- ============================================================
    my_clock_id <= (src_mac_address(47 downto 40) xor x"02") & 
                   src_mac_address(39 downto 24) & 
                   x"FFFE" & 
                   src_mac_address(23 downto 0) & 
                   x"0001";

    -- ============================================================
    -- CDC Synchronization Process (separate from main state machine)
    -- This ensures proper timing analysis and prevents race conditions
    -- ============================================================


    clock_configured_o <= clock_configured;
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
        variable delta_s2m : signed(31 downto 0);  -- T4 - T3 (used in Calc_Stage2)
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
            log_msg_interval_o <= (others => '0');
            log_msg_interval_valid_o <= '0';

            active_sequence_id := (others => '0');
            stored_t1_seconds <= (others => '0');
            stored_t1_nanoseconds <= (others => '0');
            stored_t2_seconds <= (others => '0');
            stored_t2_nanoseconds <= (others => '0');
            stored_t3_seconds <= (others => '0');
            stored_t3_nanoseconds <= (others => '0');
            stored_t4_seconds <= (others => '0');
            stored_t4_nanoseconds <= (others => '0');

            -- Pipeline calculation register
            delta_m2s_reg <= (others => '0');

            clock_configure_timestamp_nanoseconds_o <= (others => '0');
            clock_configure_timestamp_seconds_o <= (others => '0');
            clock_configured <= '0';
            clock_set_o <= '0';
        elsif rising_edge(clk) then
            send_delay_resp_o <= '0';
            send_delay_req_o <= '0';
            ptp_calc_valid_o <= '0';
            log_msg_interval_valid_o <= '0';
            clock_set_o <= '0';
            
            -- Capture T3 timestamp when valid
            -- NOTE: t3_valid_i acts as a handshake signal from ptpv2_controller.
            -- It pulses HIGH only AFTER the timestamp has been synchronized
            -- across the clock domain, so we can safely sample here.
            if (t3_valid_i = '1') then
                stored_t3_seconds <= tx_timestamp_seconds_i(3 downto 0);
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
                    
                    -- CRITICAL: Latch RX timestamp immediately when packet arrives!
                    -- This timestamp will be used as T2 if this is a Sync message
                    latched_rx_timestamp_seconds <= rx_timestamp_seconds_i;
                    latched_rx_timestamp_nanoseconds <= rx_timestamp_nanoseconds_i;
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
                        ptp_message_type(7 downto 0) <= ram_data; -- PTP Byte 0: majorSdoId(4) | messageType(4)
                    when 43 =>
                        ptp_version <= ram_data(3 downto 0); -- PTP Byte 1: reserved(4) | versionPTP(4)
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
                if (get_message_type(ptp_message_type(3 downto 0)) = t_Announce) then
                    -- Announce message additional fields
                    message_length := 105; -- Announce messages are longer
                    case byte_counter is
                        when 86 =>
                            ptp_announce_current_utc_offset(15 downto 8) <= ram_data;
                        when 87 =>
                            ptp_announce_current_utc_offset(7 downto 0) <= ram_data;
                        when 89 =>
                            ptp_announce_grandmaster_priority1(7 downto 0) <= ram_data;
                        when 90 =>
                            ptp_announce_grandmaster_clockClass(7 downto 0) <= ram_data;
                        when 91 =>
                            ptp_announce_grandmaster_clockAccuracy(7 downto 0) <= ram_data;
                        when 92 =>
                            ptp_announce_grandmaster_offset_scaled_log_variance(15 downto 8) <= ram_data;
                        when 93 =>
                            ptp_announce_grandmaster_offset_scaled_log_variance(7 downto 0) <= ram_data;
                        when 94 =>
                            ptp_announce_grandmaster_priority2(7 downto 0) <= ram_data;
                        when 95 =>
                            ptp_clock_identity(63 downto 56) <= ram_data;
                        when 96 =>
                            ptp_clock_identity(55 downto 48) <= ram_data;
                        when 97 =>
                            ptp_clock_identity(47 downto 40) <= ram_data;
                        when 98 =>
                            ptp_clock_identity(39 downto 32) <= ram_data;
                        when 99 =>
                            ptp_clock_identity(31 downto 24) <= ram_data;
                        when 100 =>
                            ptp_clock_identity(23 downto 16) <= ram_data;
                        when 101 =>
                            ptp_clock_identity(15 downto 8) <= ram_data;
                        when 102 =>
                            ptp_clock_identity(7 downto 0) <= ram_data;
                        when 103 =>
                            ptp_announce_steps_removed(15 downto 8) <= ram_data;
                        when 104 =>
                            ptp_announce_steps_removed(7 downto 0) <= ram_data;
                        when 105 =>
                            ptp_announce_time_source(7 downto 0) <= ram_data;
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
                -- ============================================
                -- VERSION CHECK: Only accept PTPv2 packets!
                -- PTPv1 uses same ports/multicast but different header format
                -- ============================================
                if ptp_version /= x"2" then
                    -- Not PTPv2 - discard packet silently
                    s_SM_PtpParser <= s_Done;
                else
                    -- interpret the PTPv2 packet based on message type (lower 4 bits only!)
                    -- PTP Byte 0: [ majorSdoId (4 bit) | messageType (4 bit) ]
                    case ptp_message_type(3 downto 0) is
                        when x"0" =>
                            -- Sync Message
                            -- Handle Sync message processing here
                            if (is_leader = '0') then
                                active_sequence_id := ptp_sequence_id;
                                stored_t2_seconds <= latched_rx_timestamp_seconds(3 downto 0);
                                stored_t2_nanoseconds <= latched_rx_timestamp_nanoseconds;
                            end if;
                            s_SM_PtpParser <= s_Done;
                            
                        when x"1" =>
                            -- Delay_Req Message
                            if is_leader = '1' then
                                -- Prepare Delay_Resp message
                                rx_follower_identity_o <= ptp_source_port_identity & ptp_source_port_port_number;
                                rx_timestamp_seconds_o <= latched_rx_timestamp_seconds;
                                rx_timestamp_nanoseconds_o <= latched_rx_timestamp_nanoseconds;
                                sequence_id_o <= ptp_sequence_id;
                                send_delay_resp_o <= '1';
                            end if;
                            s_SM_PtpParser <= s_Done;
                            
                        when x"2" =>
                            -- Pdelay_Req Message
                            s_SM_PtpParser <= s_Done;
                            
                        when x"3" =>
                            -- Pdelay_Resp Message
                            s_SM_PtpParser <= s_Done;
                            
                        when x"8" =>
                            -- Follow_Up Message
                            if (is_leader = '0') then
                                -- Always update log_msg_interval from Follow_Up - this is the master's sync rate
                                -- Do this BEFORE sequence ID check so we track the rate even if we miss packets
                                log_msg_interval_o <= signed(ptp_log_msg_interval);
                                log_msg_interval_valid_o <= '1';
                            
                                -- Handle Follow_Up message processing here
                                if ptp_sequence_id = active_sequence_id then
                                    if (clock_configured = '0') then
                                        -- First sync: Set clock to master time and wait for next full cycle
                                        clock_set_o <= '1';
                                        clock_configured <= '1';
                                        clock_configure_timestamp_seconds_o <= ptp_origin_timestamp_seconds;
                                        clock_configure_timestamp_nanoseconds_o <= ptp_origin_timestamp_nanoseconds;
                                    end if;
                                    -- T1: Origin timestamp from Follow_Up (when Master sent Sync)
                                    stored_t1_seconds <= ptp_origin_timestamp_seconds;
                                    stored_t1_nanoseconds <= ptp_origin_timestamp_nanoseconds;
                                    sequence_id_o <= ptp_sequence_id;
                                    send_delay_req_o <= '1';
                                end if;
                            end if;
                            s_SM_PtpParser <= s_Done;
                            
                        when x"9" =>
                        -- Delay_resp Message
                        if (is_leader = '0') then
                            -- Handle Delay_Resp message processing here
                            -- Check sequence ID AND that this response is for US (our clock identity)
                            if ptp_sequence_id = active_sequence_id and requesting_port_identity = my_clock_id then
                                -- T4: Origin timestamp from Delay_Resp (when Master received Delay_Req)
                                stored_t4_seconds <= ptp_origin_timestamp_seconds(3 downto 0);
                                stored_t4_nanoseconds <= ptp_origin_timestamp_nanoseconds;
                                
                                -- Go to calculation pipeline
                                -- No extra latching needed: stored_t* are stable while
                                -- the state machine is not in s_Idle (no new packets parsed)
                                s_SM_PtpParser <= s_Calc_Stage1;
                            else
                                -- Sequence ID or port identity mismatch - skip calculation
                                s_SM_PtpParser <= s_Done;
                            end if;
                        else
                            -- We are leader, not follower - skip calculation
                            s_SM_PtpParser <= s_Done;
                        end if;
                    when x"A" =>
                        -- Pdelay_Resp_Follow_Up Message
                        s_SM_PtpParser <= s_Done;
                    when x"B" =>
                        -- Announce Message
                        s_SM_PtpParser <= s_Done;
                    when x"C" =>
                        -- Signaling Message
                        s_SM_PtpParser <= s_Done;
                    when x"D" =>
                        -- Management Message
                        s_SM_PtpParser <= s_Done;
                    when others =>
                        -- Unknown Message Type
                        s_SM_PtpParser <= s_Done;
                    end case;
                    
                end if;  -- ptp_version = x"2" check

            elsif (s_SM_PtpParser = s_Calc_Stage1) then
                -- ============================================
                -- PIPELINED CALCULATION - Stage 1
                -- Calculate delta_master_to_slave = T2 - T1
                -- Register result for next stage
                -- ============================================
                delta_m2s_reg <= timestamp_diff_ns(
                    stored_t2_seconds, stored_t2_nanoseconds,
                    stored_t1_seconds(3 downto 0), stored_t1_nanoseconds
                );
                s_SM_PtpParser <= s_Calc_Stage2;
                
            elsif (s_SM_PtpParser = s_Calc_Stage2) then
                -- ============================================
                -- PIPELINED CALCULATION - Stage 2
                -- Calculate delta_slave_to_master = T4 - T3
                -- Then final results using registered delta_m2s_reg
                -- ============================================
                delta_s2m := timestamp_diff_ns(
                    stored_t4_seconds, stored_t4_nanoseconds,
                    stored_t3_seconds, stored_t3_nanoseconds
                );
                
                -- Mean path delay = ((t2-t1) + (t4-t3)) / 2
                mean_path_delay_ns_o <= shift_right(delta_m2s_reg + delta_s2m, 1);
                
                -- Offset from master = ((t2-t1) - (t4-t3)) / 2
                offset_from_master_ns_o <= shift_right(delta_m2s_reg - delta_s2m, 1);
                
                ptp_calc_valid_o <= '1';
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