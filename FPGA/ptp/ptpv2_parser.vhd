library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity ptpv2_parser is
    port(
        clk                 : in std_logic;
        rx_clk_i            : in std_logic;
        rx_data_i           : in STD_LOGIC_VECTOR(7 downto 0);
        rx_byte_received_i  : in std_logic;
        rx_byte_receive_index_i : in unsigned(7 downto 0); -- max length is 105 for ptpv2
        rx_ptp_frame_i      : in std_logic;
        
        is_leader           : in std_logic; -- '1' if this node is PTP leader; will answer to delay_req

        rx_timestamp_seconds_i     : in unsigned(3 downto 0);
        rx_timestamp_nanoseconds_i : in UNSIGNED(29 downto 0);

        rx_timestamp_seconds_o     : out unsigned(3 downto 0);
        rx_timestamp_nanoseconds_o : out unsigned(29 downto 0);
        rx_follower_identity_o      : out std_logic_vector(79 downto 0);

        send_delay_resp_o         : out std_logic;


        sequence_id_o              : out std_logic_vector(15 downto 0);
        reset_n             : in std_logic;

        send_delay_req_o         : out std_logic;

        tx_timestamp_seconds_i     : in unsigned(3 downto 0);
        tx_timestamp_nanoseconds_i : in unsigned(29 downto 0);
        t3_valid_i                 : in std_logic;

        src_mac_address		: in std_logic_vector(47 downto 0);

        -- PTP calculation outputs
        delta_m2s_o       : out signed(31 downto 0);
        delta_m2s_valid_o : out std_logic;
        delta_s2m_o    : out signed(31 downto 0);
        delta_s2m_valid_o : out std_logic;
        ptp_calc_valid_o           : out std_logic;            -- Pulse when calculation is complete
        log_msg_interval_o         : out signed(7 downto 0);   -- PTP logMessageInterval from last Sync/Follow_Up
        log_msg_interval_valid_o   : out std_logic;            -- Pulse when log_msg_interval is updated (from Follow_Up)

        clock_set_o            : out std_logic;
        clock_configured_o   : out std_logic;
        clock_configure_timestamp_seconds_o     : out STD_LOGIC_VECTOR(47 downto 0);
        clock_configure_timestamp_nanoseconds_o : out STD_LOGIC_VECTOR(29 downto 0); 
        ptp_current_leader_id_i : in std_logic_vector(63 downto 0);
        ptp_is_follower_i : in std_logic;

        -- Pulse from servo: offset implausibly large, redo full clock set
        -- on the next Follow_Up.
        clock_reconfigure_req_i : in std_logic;



        
        -- Announce dataset outputs (for BMC)
        announce_valid_o                 : out std_logic;  -- pulse when a valid Announce has been parsed
        announce_clock_identity_o        : out std_logic_vector(63 downto 0);
        announce_priority1_o             : out std_logic_vector(7 downto 0);
        announce_clock_class_o           : out std_logic_vector(7 downto 0);
        announce_clock_accuracy_o        : out std_logic_vector(7 downto 0);
        announce_offset_scaled_log_var_o : out std_logic_vector(15 downto 0);
        announce_priority2_o             : out std_logic_vector(7 downto 0);
        announce_steps_removed_o         : out std_logic_vector(15 downto 0);
        announce_time_source_o           : out std_logic_vector(7 downto 0);
        announce_log_msg_interval_o      : out std_logic_vector(7 downto 0)
    );
end entity;

architecture Behavioral of ptpv2_parser is


    
    -- EUI-64 Clock Identity from MAC address (IEEE 1588)
    signal my_clock_id : std_logic_vector(79 downto 0);
    
    -- Constants for seconds-to-nanoseconds conversion (NO DSP multiplication!)
    constant ONE_SECOND_NS_POS  : signed(31 downto 0) := to_signed( 1_000_000_000, 32);
    constant TWO_SECONDS_NS_POS : signed(31 downto 0) := to_signed( 2_000_000_000, 32);
    constant ONE_SECOND_NS_NEG  : signed(31 downto 0) := to_signed(-1_000_000_000, 32);
    constant TWO_SECONDS_NS_NEG : signed(31 downto 0) := to_signed(-2_000_000_000, 32);
    
    
    -- State machine
    type t_SM_PtpParser is (s_Idle, s_Prefetch, s_ReadHeader, s_Interpret_Packet, s_Calc_m2s, s_Calc_s2m, s_Output, s_Done);
    signal s_SM_PtpParser : t_SM_PtpParser := s_Idle;

    type t_SM_ClockConfigurator is (s_Idle, s_ClockSet_Calc, s_ClockSet_Calc2, s_ClockSet_Apply, s_ClockSet_Apply2);
    signal s_SM_ClockConfigurator : t_SM_ClockConfigurator := s_Idle;
    signal configureClock : STD_LOGIC := '0';
    signal ns_sum : unsigned(31 downto 0);

    -- Elapsed local time since Sync RX, for initial clock set compensation
    signal elapsed_ns : unsigned(31 downto 0) := (others => '0');

    signal ptp_regs_valid : STD_LOGIC := '0';
    signal ptp_regs_valid_sync : STD_LOGIC := '0';
    signal ptp_regs_valid_reg : STD_LOGIC := '0';
    signal ptp_regs_valid_reg_z : STD_LOGIC := '0';

    -- ============================================================
    -- PIPELINED PTP Calculation Registers
    -- ============================================================
    signal delta_m2s_reg : signed(31 downto 0) := (others => '0');  -- T2 - T1
    signal delta_s2m_reg : signed(31 downto 0) := (others => '0');  -- T4 - T3

    function timestamp_diff_ns(
        sec_a  : unsigned(3 downto 0);
        ns_a   : unsigned(29 downto 0);
        sec_b  : unsigned(3 downto 0);
        ns_b   : unsigned(29 downto 0)
    ) return signed is
        variable sec_diff    : signed(4 downto 0);
        variable ns_diff     : signed(30 downto 0);
        variable sec_as_ns   : signed(31 downto 0);
        variable result      : signed(31 downto 0);
    begin
        sec_diff := resize(signed('0' & sec_a), 5) - resize(signed('0' & sec_b), 5);
        ns_diff := resize(signed('0' & ns_a), 31) - resize(signed('0' & ns_b), 31);
        
        case to_integer(sec_diff) is
            when  0 => sec_as_ns := (others => '0');
            when  1 => sec_as_ns := ONE_SECOND_NS_POS;
            when  2 => sec_as_ns := TWO_SECONDS_NS_POS;
            when -1 => sec_as_ns := ONE_SECOND_NS_NEG;
            when -2 => sec_as_ns := TWO_SECONDS_NS_NEG;
            when others => 
                if sec_diff(4) = '1' then
                    sec_as_ns := TWO_SECONDS_NS_NEG;
                else
                    sec_as_ns := TWO_SECONDS_NS_POS;
                end if;
        end case;
        
        result := sec_as_ns + resize(ns_diff, 32);
        return result;
    end function;

    type t_packet_type is (t_Sync, t_Delay_Req, t_Follow_Up, t_Delay_Resp, t_Announce, t_Pdelay_Req, t_Pdelay_Resp, t_Pdelay_Follow_Up, t_Signaling, t_Managment, t_Unknown);
    
    function get_message_type(message_type : std_logic_vector(3 downto 0)) return t_packet_type is
    begin
        case message_type is
            when x"0" => return t_Sync;
            when x"1" => return t_Delay_Req;
            --when x"2" => return t_Pdelay_Req;
            --when x"3" => return t_Pdelay_Resp;
            when x"8" => return t_Follow_Up;
            when x"9" => return t_Delay_Resp;
            --when x"A" => return t_Pdelay_Follow_Up;
            when x"B" => return t_Announce;
            when x"C" => return t_Signaling;
            when x"D" => return t_Managment;
            when others => return t_Unknown;
        end case;
    end function;


    signal clock_configured: std_logic := '0';

    signal rx_packet_length : UNSIGNED(7 downto 0);

    signal ptp_message_type: t_packet_type;
    signal ptp_domain_number: std_logic_vector(7 downto 0);
    signal ptp_flag_field: std_logic_vector(15 downto 0);
    signal ptp_correction_field: std_logic_vector(63 downto 0);
    signal ptp_source_port_identity: std_logic_vector(63 downto 0);
    signal ptp_source_port_port_number: std_logic_vector(15 downto 0);
    signal ptp_sequence_id: std_logic_vector(15 downto 0);
    signal ptp_control_field: std_logic_vector(7 downto 0);
    signal ptp_log_msg_interval: std_logic_vector(7 downto 0);
    signal ptp_version: std_logic_vector(3 downto 0);
    signal ptp_origin_timestamp_seconds: STD_LOGIC_VECTOR(47 downto 0);
    signal ptp_origin_timestamp_nanoseconds: STD_LOGIC_VECTOR(29 downto 0);
    signal ptp_requesting_port_identity : STD_LOGIC_VECTOR(79 downto 0);
    signal active_sequence_id: STD_LOGIC_VECTOR(15 downto 0);
 
    signal ptp_announce_valid : STD_LOGIC;
    signal ptp_announce_valid_sync : STD_LOGIC;
    signal ptp_announce_valid_reg : STD_LOGIC;
    signal ptp_announce_valid_reg_z : STD_LOGIC;

    
    -- PTP Timestamp Storage
    signal stored_t1_seconds     : unsigned(47 downto 0) := (others => '0');
    signal stored_t1_nanoseconds : unsigned(29 downto 0) := (others => '0');
    signal stored_t2_seconds     : unsigned(3 downto 0) := (others => '0');
    signal stored_t2_nanoseconds : unsigned(29 downto 0) := (others => '0');
    signal stored_t3_seconds     : unsigned(3 downto 0) := (others => '0');
    signal stored_t3_nanoseconds : unsigned(29 downto 0) := (others => '0');
    signal stored_t4_seconds     : unsigned(3 downto 0) := (others => '0');
    signal stored_t4_nanoseconds : unsigned(29 downto 0) := (others => '0');
    
    signal latched_rx_timestamp_seconds     : unsigned(3 downto 0) := (others => '0');
    signal latched_rx_timestamp_nanoseconds : unsigned(29 downto 0) := (others => '0');
    -- Throttle for the ClockConfigurator FSM: a free-running 3-bit counter
    -- (wraps 0 -> 7 -> 6 -> ... -> 0) gates state advancement, so the body
    -- runs once every 8 sys_clk cycles. Spaces the adder/compare stages so
    -- they don't share the parser FSM's critical-path window.
    signal cfg_throttle_cnt : unsigned(2 downto 0) := (others => '0');
    
    

begin
    
    -- Generate Clock Identity from MAC address
    my_clock_id <= (src_mac_address(47 downto 40) xor x"02") & 
                   src_mac_address(39 downto 24) & 
                   x"FFFE" & 
                   src_mac_address(23 downto 0) & 
                   x"0001";

    clock_configured_o <= clock_configured;
    




    -- ============================================================
    -- ClockConfigurator: pipelines a one-shot wallclock set from the
    -- master's T1 (Follow_Up origin timestamp) plus the local elapsed
    -- time since Sync RX (= rx_timestamp(Follow_Up) - T2).
    --
    --   s_Idle         : wait for configureClock pulse from main FSM
    --   s_ClockSet_Calc  : elapsed_ns = rx_ts(Follow_Up) - T2
    --   s_ClockSet_Calc2 : ns_sum     = T1.ns + elapsed_ns
    --   s_ClockSet_Apply : split ns_sum into ns/sec with 1e9 carry
    --   s_ClockSet_Apply2: pulse clock_set_o and return to Idle
    --
    -- cfg_throttle_cnt gates state advancement so each stage gets 8 sys_clk
    -- cycles of slack instead of one. See note at the signal declaration.
    -- ============================================================
    clock_config_process: process (clk, reset_n)
    begin
        if reset_n = '0' then
            clock_configure_timestamp_nanoseconds_o <= (others => '0');
            clock_configure_timestamp_seconds_o     <= (others => '0');
            clock_configured                        <= '0';
            clock_set_o                             <= '0';
            cfg_throttle_cnt                        <= (others => '0');
            s_SM_ClockConfigurator                  <= s_Idle;
        elsif rising_edge(clk) then
            clock_set_o      <= '0';
            cfg_throttle_cnt <= cfg_throttle_cnt - 1;

            -- Servo asks for full reconfigure: drop the configured flag so the
            -- next Follow_Up re-arms configureClock and we redo T1+elapsed set.
            if clock_reconfigure_req_i = '1' then
                clock_configured <= '0';
            end if;

            if cfg_throttle_cnt = 0 then
                case s_SM_ClockConfigurator is
                    when s_Idle =>
                        if configureClock = '1' then
                            clock_configured       <= '0';
                            s_SM_ClockConfigurator <= s_ClockSet_Calc;
                        end if;

                    when s_ClockSet_Calc =>
                        -- elapsed_ns = rx_timestamp(Follow_Up) - T2(Sync)
                        elapsed_ns <= unsigned(timestamp_diff_ns(
                            latched_rx_timestamp_seconds(3 downto 0),
                            latched_rx_timestamp_nanoseconds,
                            stored_t2_seconds, stored_t2_nanoseconds));
                        s_SM_ClockConfigurator <= s_ClockSet_Calc2;

                    when s_ClockSet_Calc2 =>
                        ns_sum <= unsigned(stored_t1_nanoseconds) + elapsed_ns;
                        s_SM_ClockConfigurator <= s_ClockSet_Apply;

                    when s_ClockSet_Apply =>
                        -- Split T1 + elapsed into ns/sec with 1e9 carry
                        if ns_sum >= unsigned(ONE_SECOND_NS_POS) then
                            clock_configure_timestamp_nanoseconds_o <= STD_LOGIC_VECTOR(
                                resize(ns_sum - unsigned(ONE_SECOND_NS_POS), 30));
                            clock_configure_timestamp_seconds_o <= STD_LOGIC_VECTOR(
                                unsigned(stored_t1_seconds) + 1);
                        else
                            clock_configure_timestamp_nanoseconds_o <=
                                STD_LOGIC_VECTOR(resize(ns_sum, 30));
                            clock_configure_timestamp_seconds_o <=
                                STD_LOGIC_VECTOR(stored_t1_seconds);
                        end if;
                        s_SM_ClockConfigurator <= s_ClockSet_Apply2;

                    when s_ClockSet_Apply2 =>
                        clock_set_o            <= '1';
                        clock_configured       <= '1';
                        s_SM_ClockConfigurator <= s_Idle;
                end case;
            end if;
        end if;
    end process;




    t3_capture_proc: process (clk, reset_n)

    begin
        if reset_n = '0' then
            stored_t3_seconds <= (others => '0');
            stored_t3_nanoseconds <= (others => '0');
        elsif rising_edge(clk) then
            -- Capture T3 timestamp when valid
            if (t3_valid_i = '1') then
                stored_t3_seconds <= tx_timestamp_seconds_i(3 downto 0);
                stored_t3_nanoseconds <= tx_timestamp_nanoseconds_i;
            end if;

        end if;

    end process;
    -- Main State Machine Process

    rx_packet_length <= x"5F" when ptp_message_type = t_Delay_Resp else x"69" when ptp_message_type = t_Announce else x"55";

    receive_process : process (rx_clk_i, rx_ptp_frame_i)
    begin
        if rising_edge(rx_clk_i) and rx_ptp_frame_i = '1' then
            latched_rx_timestamp_seconds <= rx_timestamp_seconds_i;
            latched_rx_timestamp_nanoseconds <= rx_timestamp_nanoseconds_i;
            if (rx_byte_received_i = '1') then
                case to_integer(rx_byte_receive_index_i) is
                    when 42 =>
                        ptp_message_type <= get_message_type(rx_data_i(3 downto 0));
                        ptp_regs_valid <= '0';
                    when 43 => ptp_version <= rx_data_i(3 downto 0);
                    when 46 => ptp_domain_number(7 downto 0) <= rx_data_i;
                    when 48 => ptp_flag_field(15 downto 8) <= rx_data_i;
                    when 49 => ptp_flag_field(7 downto 0) <= rx_data_i;
                    when 50 => ptp_correction_field(63 downto 56) <= rx_data_i;
                    when 51 => ptp_correction_field(55 downto 48) <= rx_data_i;
                    when 52 => ptp_correction_field(47 downto 40) <= rx_data_i;
                    when 53 => ptp_correction_field(39 downto 32) <= rx_data_i;
                    when 54 => ptp_correction_field(31 downto 24) <= rx_data_i;
                    when 55 => ptp_correction_field(23 downto 16) <= rx_data_i;
                    when 56 => ptp_correction_field(15 downto 8) <= rx_data_i;
                    when 57 => ptp_correction_field(7 downto 0) <= rx_data_i;
                    when 62 => ptp_source_port_identity(63 downto 56) <= rx_data_i;
                    when 63 => ptp_source_port_identity(55 downto 48) <= rx_data_i;
                    when 64 => ptp_source_port_identity(47 downto 40) <= rx_data_i;
                    when 65 => ptp_source_port_identity(39 downto 32) <= rx_data_i;
                    when 66 => ptp_source_port_identity(31 downto 24) <= rx_data_i;
                    when 67 => ptp_source_port_identity(23 downto 16) <= rx_data_i;
                    when 68 => ptp_source_port_identity(15 downto 8) <= rx_data_i;
                    when 69 => ptp_source_port_identity(7 downto 0) <= rx_data_i;
                    when 70 => ptp_source_port_port_number(15 downto 8) <= rx_data_i;
                    when 71 => ptp_source_port_port_number(7 downto 0) <= rx_data_i;
                    when 72 => ptp_sequence_id(15 downto 8) <= rx_data_i;
                    when 73 => ptp_sequence_id(7 downto 0) <= rx_data_i;
                    when 74 => ptp_control_field(7 downto 0) <= rx_data_i;
                    when 75 => ptp_log_msg_interval(7 downto 0) <= rx_data_i;
                    when 76 => ptp_origin_timestamp_seconds(47 downto 40) <= rx_data_i;
                    when 77 => ptp_origin_timestamp_seconds(39 downto 32) <= rx_data_i;
                    when 78 => ptp_origin_timestamp_seconds(31 downto 24) <= rx_data_i;
                    when 79 => ptp_origin_timestamp_seconds(23 downto 16) <= rx_data_i;
                    when 80 => ptp_origin_timestamp_seconds(15 downto 8) <= rx_data_i;
                    when 81 => ptp_origin_timestamp_seconds(7 downto 0) <= rx_data_i;
                    when 82 => ptp_origin_timestamp_nanoseconds(29 downto 24) <= rx_data_i(5 downto 0);
                    when 83 => ptp_origin_timestamp_nanoseconds(23 downto 16) <= rx_data_i;
                    when 84 => ptp_origin_timestamp_nanoseconds(15 downto 8) <= rx_data_i;
                    when 85 => 
                        ptp_origin_timestamp_nanoseconds(7 downto 0) <= rx_data_i;
                        
                    when others => null;
                end case;
                
                if (ptp_message_type = t_Delay_Resp) then
                    case to_integer(rx_byte_receive_index_i) is
                        when 86 => ptp_requesting_port_identity(79 downto 72) <= rx_data_i;
                        when 87 => ptp_requesting_port_identity(71 downto 64) <= rx_data_i;
                        when 88 => ptp_requesting_port_identity(63 downto 56) <= rx_data_i;
                        when 89 => ptp_requesting_port_identity(55 downto 48) <= rx_data_i;
                        when 90 => ptp_requesting_port_identity(47 downto 40) <= rx_data_i;
                        when 91 => ptp_requesting_port_identity(39 downto 32) <= rx_data_i;
                        when 92=> ptp_requesting_port_identity(31 downto 24) <= rx_data_i;
                        when 93 => ptp_requesting_port_identity(23 downto 16) <= rx_data_i;
                        when 94 => ptp_requesting_port_identity(15 downto 8) <= rx_data_i;
                        when 95 => ptp_requesting_port_identity(7 downto 0) <= rx_data_i;
                            
                        when others => null;
                    end case;
                end if;
                
                if (ptp_message_type = t_Announce) then
                    announce_log_msg_interval_o      <= ptp_log_msg_interval;
                    case to_integer(rx_byte_receive_index_i) is
                        --when d"86" => ptp_announce_current_utc_offset(15 downto 8) <= rx_data_i; -- nnot impl
                        --when d"87" => ptp_announce_current_utc_offset(7 downto 0) <= rx_data_i; -- not impl
                        when 89 => announce_priority1_o(7 downto 0) <= rx_data_i;
                        when 90 => announce_clock_class_o(7 downto 0) <= rx_data_i;
                        when 91 => announce_clock_accuracy_o(7 downto 0) <= rx_data_i;
                        when 92 => announce_offset_scaled_log_var_o(15 downto 8) <= rx_data_i;
                        when 93 => announce_offset_scaled_log_var_o(7 downto 0) <= rx_data_i;
                        when 94 => announce_priority2_o(7 downto 0) <= rx_data_i;
                        when 95 => announce_clock_identity_o(63 downto 56) <= rx_data_i;
                        when 96 => announce_clock_identity_o(55 downto 48) <= rx_data_i;
                        when 97 => announce_clock_identity_o(47 downto 40) <= rx_data_i;
                        when 98 => announce_clock_identity_o(39 downto 32) <= rx_data_i;
                        when 99 => announce_clock_identity_o(31 downto 24) <= rx_data_i;
                        when 100 => announce_clock_identity_o(23 downto 16) <= rx_data_i;
                        when 101 => announce_clock_identity_o(15 downto 8) <= rx_data_i;
                        when 102 => announce_clock_identity_o(7 downto 0) <= rx_data_i;
                        when 103 => announce_steps_removed_o(15 downto 8) <= rx_data_i;
                        when 104 => announce_steps_removed_o(7 downto 0) <= rx_data_i;
                        when 105 => 
                            announce_time_source_o(7 downto 0) <= rx_data_i;
                            ptp_announce_valid <= not ptp_announce_valid;
                        when others => null;
                    end case;
                end if;
                if rx_byte_receive_index_i = rx_packet_length then
                    ptp_regs_valid <= '1';
                end if;
            end if;
        end if;
    end process;

    ptp_reg_valid_cdc: process(clk, reset_n) begin

        if (reset_n = '0') then
            ptp_regs_valid_reg <= '0';
            ptp_regs_valid_sync <= '0';
            ptp_regs_valid_reg_z <= '0';
            ptp_announce_valid_reg <= '0';
            ptp_announce_valid_sync <= '0';
        elsif rising_edge(clk) then
            ptp_regs_valid_sync <= ptp_regs_valid;
            ptp_regs_valid_reg <= ptp_regs_valid_sync;
            ptp_regs_valid_reg_z <= ptp_regs_valid_reg;
            ptp_announce_valid_sync <= ptp_announce_valid;
            ptp_announce_valid_reg <= ptp_announce_valid_sync;
            ptp_announce_valid_reg_z <= ptp_announce_valid_reg;
        end if;

    end process;
    announce_valid_o <= ptp_announce_valid_reg xor ptp_announce_valid_reg_z;

    main_proc: process(clk, reset_n)

    begin
        if reset_n = '0' then
            s_SM_PtpParser <= s_Idle;
            send_delay_resp_o <= '0';
            send_delay_req_o <= '0';
            ptp_calc_valid_o <= '0';
            delta_m2s_o <= (others => '0');
            delta_s2m_o <= (others => '0');
            delta_m2s_valid_o <= '0';
            delta_s2m_valid_o <= '0';
            log_msg_interval_o <= (others => '0');
            log_msg_interval_valid_o <= '0';

            stored_t1_seconds <= (others => '0');
            stored_t1_nanoseconds <= (others => '0');
            stored_t2_seconds <= (others => '0');
            stored_t2_nanoseconds <= (others => '0');
            stored_t4_seconds <= (others => '0');
            stored_t4_nanoseconds <= (others => '0');

            delta_m2s_reg <= (others => '0');
            delta_s2m_reg <= (others => '0');

            

        elsif rising_edge(clk) then
            delta_m2s_valid_o <= '0';
            delta_s2m_valid_o <= '0';
            send_delay_resp_o <= '0';
            send_delay_req_o <= '0';
            ptp_calc_valid_o <= '0';
            log_msg_interval_valid_o <= '0';
            if (clock_configured = '1') then
            configureClock <= '0';
            end if;
            
            
            

            if (s_SM_PtpParser = s_Idle) then
                if (ptp_regs_valid_reg_z = '0' and ptp_regs_valid_reg = '1') then
                    s_SM_PtpParser <= s_Interpret_Packet;
                end if;

            elsif (s_SM_PtpParser = s_Interpret_Packet) then
                -- VERSION CHECK: Only accept PTPv2 packets.
                -- Announce messages always pass through so the BMC can see all peers.
                if ptp_version /= x"2" or
                   (ptp_source_port_identity /= ptp_current_leader_id_i
                    and is_leader = '0' and ptp_is_follower_i = '1'
                    and ptp_message_type /= t_Announce) then
                    s_SM_PtpParser <= s_Done;
                else
                    case ptp_message_type is
                        when t_Sync =>
                            -- Sync Message
                            if (is_leader = '0' and ptp_is_follower_i = '1') then
                                active_sequence_id <= ptp_sequence_id;
                                stored_t2_seconds <= latched_rx_timestamp_seconds(3 downto 0);
                                stored_t2_nanoseconds <= latched_rx_timestamp_nanoseconds;
                            end if;
                            s_SM_PtpParser <= s_Done;
                            
                        when t_Delay_Req =>
                            -- Delay_Req Message
                            if is_leader = '1' and ptp_is_follower_i = '0' then
                                rx_follower_identity_o <= ptp_source_port_identity & ptp_source_port_port_number;
                                rx_timestamp_seconds_o <= latched_rx_timestamp_seconds;
                                rx_timestamp_nanoseconds_o <= latched_rx_timestamp_nanoseconds;
                                sequence_id_o <= ptp_sequence_id;
                                send_delay_resp_o <= '1';
                            end if;
                            s_SM_PtpParser <= s_Done;
                            
                        when t_Pdelay_Req => s_SM_PtpParser <= s_Done;  -- Pdelay_Req
                        when t_Pdelay_Resp => s_SM_PtpParser <= s_Done;  -- Pdelay_Resp
                            
                        when t_Follow_Up =>
                            -- Follow_Up Message
                            if (is_leader = '0' and ptp_is_follower_i = '1') then
                                log_msg_interval_o <= signed(ptp_log_msg_interval);
                                log_msg_interval_valid_o <= '1';

                                if ptp_sequence_id = active_sequence_id then
                                    if (clock_configured = '0') then
                                        -- Store T1 for clock set, then compute elapsed time compensation
                                        stored_t1_seconds <= unsigned(ptp_origin_timestamp_seconds);
                                        stored_t1_nanoseconds <= unsigned(ptp_origin_timestamp_nanoseconds);
                                        configureClock <= '1';
                                    else
                                        stored_t1_seconds <=unsigned(ptp_origin_timestamp_seconds);
                                        stored_t1_nanoseconds <= unsigned(ptp_origin_timestamp_nanoseconds);
                                        sequence_id_o <= ptp_sequence_id;
                                        send_delay_req_o <= '1';
                                        s_SM_PtpParser <= s_Calc_m2s;
                                    end if;
                                else
                                    s_SM_PtpParser <= s_Done;
                                end if;
                            else
                                s_SM_PtpParser <= s_Done;
                            end if;
                            
                        when t_Delay_Resp =>
                            -- Delay_resp Message
                            if (is_leader = '0' and ptp_is_follower_i = '1') then
                                if ptp_sequence_id = active_sequence_id and ptp_requesting_port_identity = my_clock_id then
                                    stored_t4_seconds <= unsigned(ptp_origin_timestamp_seconds(3 downto 0));
                                    stored_t4_nanoseconds <= unsigned(ptp_origin_timestamp_nanoseconds);
                                    s_SM_PtpParser <= s_Calc_s2m;
                                else
                                    s_SM_PtpParser <= s_Done;
                                end if;
                            else
                                s_SM_PtpParser <= s_Done;
                            end if;
                            
                        when t_Pdelay_Follow_Up => s_SM_PtpParser <= s_Done;  -- Pdelay_Resp_Follow_Up
                        --when x"C" => s_SM_PtpParser <= s_Done;  -- Signaling
                        --when x"D" => s_SM_PtpParser <= s_Done;  -- Management
                        when others => s_SM_PtpParser <= s_Done;
                    end case;
                end if;

            

            elsif (s_SM_PtpParser = s_Calc_m2s) then

                delta_m2s_o <= timestamp_diff_ns(
                    stored_t2_seconds, stored_t2_nanoseconds,
                    stored_t1_seconds(3 downto 0), stored_t1_nanoseconds
                );
                delta_m2s_valid_o <= '1';



                s_SM_PtpParser <= s_Done;



            elsif (s_SM_PtpParser = s_Calc_s2m) then

                
                delta_s2m_o <= timestamp_diff_ns(
                    stored_t4_seconds, stored_t4_nanoseconds,
                    stored_t3_seconds, stored_t3_nanoseconds
                );
                delta_s2m_valid_o <= '1';

                s_SM_PtpParser <= s_Done;



            elsif (s_SM_PtpParser = s_Done) then
                
                    s_SM_PtpParser <= s_Idle;
                
            else
                s_SM_PtpParser <= s_Idle;
            end if;
        end if;
    end process main_proc;
end Behavioral;
