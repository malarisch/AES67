library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity ptpv2_parser is
    generic(
        -- Maximum min-filter depth (sets the BRAM buffer size).
        -- The actual operating depth is min_filter_active_depth_i (clamped to
        -- this max), and enable is min_filter_enable_i.
        MIN_FILTER_DEPTH : integer := 8
    );
    port(
        clk                 : in std_logic;
        ram_data            : in std_logic_vector(7 downto 0);
        ram_read_address    : out unsigned(10 downto 0);
        parse_ptp_packet_tog    : in std_logic;
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
        clock_configure_timestamp_nanoseconds_o : out std_logic_vector(31 downto 0); 
        ptp_current_leader_id_i : in std_logic_vector(63 downto 0);
        ptp_is_follower_i : in std_logic;
        ptp_locked_i : in std_logic;

        -- Pulse from servo: offset implausibly large, redo full clock set
        -- on the next Follow_Up.
        clock_reconfigure_req_i : in std_logic;

        -- Min filter live-tuning inputs (replace former generic).
        -- min_filter_active_depth_i is clamped internally to MIN_FILTER_DEPTH.
        min_filter_enable_i        : in std_logic := '1';
        min_filter_active_depth_i  : in unsigned(7 downto 0) := to_unsigned(3, 8);
        
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
    
    -- ============================================================
    -- MIN FILTER for path delay measurements
    -- Keeps track of the minimum value over the last MIN_FILTER_DEPTH samples
    -- Separate buffers for Master->Slave (m2s) and Slave->Master (s2m) paths
    --
    -- Buffers are inferred as Block RAM (no reset, no parallel read).
    -- ============================================================
    -- Initialize to maximum positive signed value (not all 1s which = -1!)
    constant MAX_SIGNED_32 : signed(31 downto 0) := to_signed(2_147_483_647, 32);

    -- Combined M2S + S2M buffer sharing a single M9K block.
    -- Layout: addresses 0..DEPTH-1           = M2S samples
    --         addresses DEPTH..2*DEPTH-1     = S2M samples
    type delay_buffer_shared_t is array(0 to 2*MIN_FILTER_DEPTH-1) of signed(31 downto 0);
    signal delay_buffer     : delay_buffer_shared_t;
    -- Registered RAM output (REQUIRED for M9K inference — synchronous read)
    signal ram_rd_data      : signed(31 downto 0) := (others => '0');
    signal ram_rd_addr      : integer range 0 to 2*MIN_FILTER_DEPTH-1 := 0;
    attribute ramstyle : string;
    attribute ramstyle of delay_buffer : signal is "M9K";

    signal m2s_write_idx    : integer range 0 to MIN_FILTER_DEPTH-1 := 0;
    signal m2s_fill_count   : integer range 0 to MIN_FILTER_DEPTH := 0;
    signal s2m_write_idx    : integer range 0 to MIN_FILTER_DEPTH-1 := 0;
    signal s2m_fill_count   : integer range 0 to MIN_FILTER_DEPTH := 0;

    -- Effective min-filter depth: clamp the runtime input against MIN_FILTER_DEPTH
    -- (which sets the BRAM size). 0 is treated as 1.
    signal eff_min_filter_depth : integer range 1 to MIN_FILTER_DEPTH := 1;

    -- Min search state machine
    -- RAM has 1-cycle read latency, so we need an extra FETCH stage before ABS:
    --   ISSUE (set addr) -> FETCH (capture q, abs) -> CMP
    type min_search_state_t is (MS_IDLE,
                                 MS_M2S_ISSUE, MS_M2S_FETCH, MS_M2S_CMP,
                                 MS_S2M_CALC,
                                 MS_S2M_ISSUE, MS_S2M_FETCH, MS_S2M_CMP,
                                 MS_OUTPUT);
    signal min_search_state : min_search_state_t := MS_IDLE;
    signal search_idx       : integer range 0 to MIN_FILTER_DEPTH-1 := 0;
    signal temp_min_m2s     : signed(31 downto 0) := MAX_SIGNED_32;
    signal temp_min_s2m     : signed(31 downto 0) := MAX_SIGNED_32;

    -- Pipeline registers for abs() computation
    signal fetched_val      : signed(31 downto 0) := (others => '0');
    signal fetched_abs      : unsigned(31 downto 0) := (others => '0');
    signal current_min_abs  : unsigned(31 downto 0) := (others => '0');
    
    -- State machine
    type t_SM_PtpParser is (s_Idle, s_ReadHeader, s_Interpret_Packet, s_Calc_Stage1, s_Calc_MinFilter, s_Calc_Bypass, s_Done);
    signal s_SM_PtpParser : t_SM_PtpParser := s_Idle;

    type t_SM_ClockConfigurator is (s_Idle, s_ClockSet_Calc, s_ClockSet_Calc2, s_ClockSet_Apply, s_ClockSet_Apply2);
    signal s_SM_ClockConfigurator : t_SM_ClockConfigurator := s_Idle;
    signal configureClock : STD_LOGIC := '0';
    signal ns_sum : unsigned(31 downto 0);

    -- Elapsed local time since Sync RX, for initial clock set compensation
    signal elapsed_ns : unsigned(31 downto 0) := (others => '0');
    signal byte_counter   : integer range 0 to 1500 := 0;


    -- ============================================================
    -- PIPELINED PTP Calculation Registers
    -- ============================================================
    signal delta_m2s_reg : signed(31 downto 0) := (others => '0');  -- T2 - T1
    signal delta_s2m_reg : signed(31 downto 0) := (others => '0');  -- T4 - T3

    -- ============================================================
    -- PTP Calculation Function (DSP-FREE, 32-bit VERSION)
    -- ============================================================
    function timestamp_diff_ns(
        sec_a  : std_logic_vector(3 downto 0);
        ns_a   : std_logic_vector(31 downto 0);
        sec_b  : std_logic_vector(3 downto 0);
        ns_b   : std_logic_vector(31 downto 0)
    ) return signed is
        variable sec_diff    : signed(4 downto 0);
        variable ns_diff     : signed(32 downto 0);
        variable sec_as_ns   : signed(31 downto 0);
        variable result      : signed(31 downto 0);
    begin
        sec_diff := resize(signed('0' & sec_a), 5) - resize(signed('0' & sec_b), 5);
        ns_diff := resize(signed('0' & ns_a), 33) - resize(signed('0' & ns_b), 33);
        
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

    -- CDC Synchronizers
    signal parse_ptp_packet_meta : std_logic := '0';
    signal parse_ptp_packet_sync : std_logic := '0';
    signal parse_ptp_packet_prev : std_logic := '0';


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
    signal ptp_version: std_logic_vector(3 downto 0);
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
    
    -- PTP Timestamp Storage
    signal stored_t1_seconds     : std_logic_vector(47 downto 0) := (others => '0');
    signal stored_t1_nanoseconds : std_logic_vector(31 downto 0) := (others => '0');
    signal stored_t2_seconds     : std_logic_vector(3 downto 0) := (others => '0');
    signal stored_t2_nanoseconds : std_logic_vector(31 downto 0) := (others => '0');
    signal stored_t3_seconds     : std_logic_vector(3 downto 0) := (others => '0');
    signal stored_t3_nanoseconds : std_logic_vector(31 downto 0) := (others => '0');
    signal stored_t4_seconds     : std_logic_vector(3 downto 0) := (others => '0');
    signal stored_t4_nanoseconds : std_logic_vector(31 downto 0) := (others => '0');
    
    signal latched_rx_timestamp_seconds     : std_logic_vector(47 downto 0) := (others => '0');
    signal latched_rx_timestamp_nanoseconds : std_logic_vector(31 downto 0) := (others => '0');
    signal configure_wait_cycle: unsigned (2 downto 0) := (others => '0') ;
    
    type t_packet_type is (t_Sync, t_Delay_Req, t_Follow_Up, t_Delay_Resp, t_Announce, t_Pdelay_Req, t_Pdelay_Resp, t_Pdelay_Follow_Up);
    
    function get_message_type(message_type : std_logic_vector(3 downto 0)) return t_packet_type is
    begin
        case message_type is
            when x"0" => return t_Sync;
            when x"1" => return t_Delay_Req;
            when x"2" => return t_Pdelay_Req;
            when x"3" => return t_Pdelay_Resp;
            when x"8" => return t_Follow_Up;
            when x"9" => return t_Delay_Resp;
            when x"A" => return t_Pdelay_Follow_Up;
            when x"B" => return t_Announce;
            when others => return t_Sync;
        end case;
    end function;

begin
    
    -- Generate Clock Identity from MAC address
    my_clock_id <= (src_mac_address(47 downto 40) xor x"02") & 
                   src_mac_address(39 downto 24) & 
                   x"FFFE" & 
                   src_mac_address(23 downto 0) & 
                   x"0001";

    clock_configured_o <= clock_configured;
    
    -- CDC Synchronization Process
    cdc_sync_proc: process(clk, reset_n)
    begin
        if reset_n = '0' then
            parse_ptp_packet_meta <= '0';
            parse_ptp_packet_sync <= '0';
            parse_ptp_packet_prev <= '0';
        elsif rising_edge(clk) then
            parse_ptp_packet_meta <= parse_ptp_packet_tog;
            parse_ptp_packet_sync <= parse_ptp_packet_meta;
            parse_ptp_packet_prev <= parse_ptp_packet_sync;
        end if;
    end process cdc_sync_proc;



    clock_config_process: process (clk, reset_n)

    begin
        if (reset_n = '0') then
            clock_configure_timestamp_nanoseconds_o <= (others => '0');
            clock_configure_timestamp_seconds_o <= (others => '0');
            clock_configured <= '0';
            clock_set_o <= '0';
            configure_wait_cycle <= (others => '0');
        elsif (rising_edge(clk)) then
            clock_set_o <= '0';

            -- Servo asks for full reconfigure: drop the configured flag so the
            -- next Follow_Up re-arms configureClock and we redo T1+elapsed set.
            if (clock_reconfigure_req_i = '1') then
                clock_configured <= '0';
            end if;

            configure_wait_cycle <= configure_wait_cycle - 1;
            if (configure_wait_cycle = 0) then
            if (s_SM_ClockConfigurator = s_Idle) then

                if (configureClock = '1') then

                    clock_configured <= '0';
                    s_SM_ClockConfigurator <= s_ClockSet_Calc;
                end if;
            elsif (s_SM_ClockConfigurator = s_ClockSet_Calc) then
                -- Takt 1: Berechne lokal vergangene Zeit seit Sync-Empfang
                -- elapsed_ns = latched_rx_timestamp(FollowUp) - stored_t2(Sync)
                elapsed_ns <= unsigned(timestamp_diff_ns(
                    latched_rx_timestamp_seconds(3 downto 0), latched_rx_timestamp_nanoseconds,
                    stored_t2_seconds, stored_t2_nanoseconds
                ))(31 downto 0);
                s_SM_ClockConfigurator <= s_ClockSet_Calc2;
                
            elsif (s_SM_ClockConfigurator = s_ClockSet_Calc2) then
                ns_sum <= unsigned(unsigned(stored_t1_nanoseconds) + elapsed_ns)(31 downto 0);
                s_SM_ClockConfigurator <= s_ClockSet_Apply;
                
            elsif (s_SM_ClockConfigurator = s_ClockSet_Apply) then
                -- Takt 2: Clock setzen auf T1 + elapsed_ns (mit Sekunden-Carry)
                if (ns_sum >= unsigned(ONE_SECOND_NS_POS)) then
                    clock_configure_timestamp_nanoseconds_o <= std_logic_vector(
                        ns_sum - unsigned(ONE_SECOND_NS_POS));
                    clock_configure_timestamp_seconds_o <=
                        std_logic_vector(unsigned(stored_t1_seconds) + 1);
                else
                    clock_configure_timestamp_nanoseconds_o <=
                        std_logic_vector(ns_sum);
                    clock_configure_timestamp_seconds_o <= stored_t1_seconds;
                end if;
                s_SM_ClockConfigurator <= s_ClockSet_Apply2;
                
            elsif (s_SM_ClockConfigurator = s_ClockSet_Apply2) then
                clock_set_o <= '1';
                clock_configured <= '1';
                s_SM_ClockConfigurator <= s_Idle;
            end if;
            
            end if;
        end if;
    end process;



    -- ============================================================
    -- BRAM READ PORT (synchronous, unconditional — required for M9K inference)
    -- Address is driven by ram_rd_addr; data appears on ram_rd_data one
    -- clock later. M2S samples live at 0..DEPTH-1, S2M at DEPTH..2*DEPTH-1.
    -- ============================================================
    min_filter_ram_rd_proc: process(clk)
    begin
        if rising_edge(clk) then
            ram_rd_data <= delay_buffer(ram_rd_addr);
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
    main_proc: process(clk, reset_n)
        variable active_sequence_id: std_logic_vector(15 downto 0);
        variable requesting_port_identity: std_logic_vector(79 downto 0);
        variable message_length: integer := 85;
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
            announce_valid_o <= '0';
            announce_clock_identity_o <= (others => '0');
            announce_priority1_o <= (others => '0');
            announce_clock_class_o <= (others => '0');
            announce_clock_accuracy_o <= (others => '0');
            announce_offset_scaled_log_var_o <= (others => '0');
            announce_priority2_o <= (others => '0');
            announce_steps_removed_o <= (others => '0');
            announce_time_source_o <= (others => '0');
            announce_log_msg_interval_o <= (others => '0');

            active_sequence_id := (others => '0');
            stored_t1_seconds <= (others => '0');
            stored_t1_nanoseconds <= (others => '0');
            stored_t2_seconds <= (others => '0');
            stored_t2_nanoseconds <= (others => '0');
            stored_t4_seconds <= (others => '0');
            stored_t4_nanoseconds <= (others => '0');

            delta_m2s_reg <= (others => '0');
            delta_s2m_reg <= (others => '0');

            
            
            -- Min filter reset — buffers are BRAM-inferred, NOT reset here
            -- (fill_count gates reads of uninitialized slots).
            m2s_write_idx <= 0;
            m2s_fill_count <= 0;
            s2m_write_idx <= 0;
            s2m_fill_count <= 0;
            min_search_state <= MS_IDLE;
            search_idx <= 0;
            temp_min_m2s <= MAX_SIGNED_32;
            temp_min_s2m <= MAX_SIGNED_32;
            ram_rd_addr <= 0;
            fetched_val <= (others => '0');
            fetched_abs <= (others => '0');
            current_min_abs <= (others => '0');
            
        elsif rising_edge(clk) then
            send_delay_resp_o <= '0';
            send_delay_req_o <= '0';
            ptp_calc_valid_o <= '0';
            log_msg_interval_valid_o <= '0';
            if (clock_configured) then
            configureClock <= '0';
            end if;
            announce_valid_o <= '0';
            
            

            if (s_SM_PtpParser = s_Idle) then
                if (parse_ptp_packet_prev /= parse_ptp_packet_sync) then
                    byte_counter <= 0;
                    ram_read_address <= to_unsigned(0, 11);
                    s_SM_PtpParser <= s_ReadHeader;
                    message_length := 85;
                    
                    latched_rx_timestamp_seconds <= rx_timestamp_seconds_i;
                    latched_rx_timestamp_nanoseconds <= rx_timestamp_nanoseconds_i;
                end if;

            elsif (s_SM_PtpParser = s_ReadHeader) then
                -- read PTPv2 header fields based on byte_counter
                case byte_counter is
                    when 36 => udp_port(15 downto 8) <= ram_data;
                    when 37 => udp_port(7 downto 0) <= ram_data;
                    when 38 => udp_length(15 downto 8) <= ram_data;
                    when 39 => udp_length(7 downto 0) <= ram_data;
                    when 42 => ptp_message_type(7 downto 0) <= ram_data;
                    when 43 => ptp_version <= ram_data(3 downto 0);
                    when 44 => ptp_message_length(15 downto 8) <= ram_data;
                    when 45 => ptp_message_length(7 downto 0) <= ram_data;
                    when 46 => ptp_domain_number(7 downto 0) <= ram_data;
                    when 48 => ptp_flag_field(15 downto 8) <= ram_data;
                    when 49 => ptp_flag_field(7 downto 0) <= ram_data;
                    when 50 => ptp_correction_field(63 downto 56) <= ram_data;
                    when 51 => ptp_correction_field(55 downto 48) <= ram_data;
                    when 52 => ptp_correction_field(47 downto 40) <= ram_data;
                    when 53 => ptp_correction_field(39 downto 32) <= ram_data;
                    when 54 => ptp_correction_field(31 downto 24) <= ram_data;
                    when 55 => ptp_correction_field(23 downto 16) <= ram_data;
                    when 56 => ptp_correction_field(15 downto 8) <= ram_data;
                    when 57 => ptp_correction_field(7 downto 0) <= ram_data;
                    when 62 => ptp_source_port_identity(63 downto 56) <= ram_data;
                    when 63 => ptp_source_port_identity(55 downto 48) <= ram_data;
                    when 64 => ptp_source_port_identity(47 downto 40) <= ram_data;
                    when 65 => ptp_source_port_identity(39 downto 32) <= ram_data;
                    when 66 => ptp_source_port_identity(31 downto 24) <= ram_data;
                    when 67 => ptp_source_port_identity(23 downto 16) <= ram_data;
                    when 68 => ptp_source_port_identity(15 downto 8) <= ram_data;
                    when 69 => ptp_source_port_identity(7 downto 0) <= ram_data;
                    when 70 => ptp_source_port_port_number(15 downto 8) <= ram_data;
                    when 71 => ptp_source_port_port_number(7 downto 0) <= ram_data;
                    when 72 => ptp_sequence_id(15 downto 8) <= ram_data;
                    when 73 => ptp_sequence_id(7 downto 0) <= ram_data;
                    when 74 => ptp_control_field(7 downto 0) <= ram_data;
                    when 75 => ptp_log_msg_interval(7 downto 0) <= ram_data;
                    when 76 => ptp_origin_timestamp_seconds(47 downto 40) <= ram_data;
                    when 77 => ptp_origin_timestamp_seconds(39 downto 32) <= ram_data;
                    when 78 => ptp_origin_timestamp_seconds(31 downto 24) <= ram_data;
                    when 79 => ptp_origin_timestamp_seconds(23 downto 16) <= ram_data;
                    when 80 => ptp_origin_timestamp_seconds(15 downto 8) <= ram_data;
                    when 81 => ptp_origin_timestamp_seconds(7 downto 0) <= ram_data;
                    when 82 => ptp_origin_timestamp_nanoseconds(31 downto 24) <= ram_data;
                    when 83 => ptp_origin_timestamp_nanoseconds(23 downto 16) <= ram_data;
                    when 84 => ptp_origin_timestamp_nanoseconds(15 downto 8) <= ram_data;
                    when 85 => ptp_origin_timestamp_nanoseconds(7 downto 0) <= ram_data;
                    when others => null;
                end case;
                
                if (get_message_type(ptp_message_type(3 downto 0)) = t_Delay_Resp) then
                    message_length := 95;
                    case byte_counter is
                        when 86 => requesting_port_identity(79 downto 72) := ram_data;
                        when 87 => requesting_port_identity(71 downto 64) := ram_data;
                        when 88 => requesting_port_identity(63 downto 56) := ram_data;
                        when 89 => requesting_port_identity(55 downto 48) := ram_data;
                        when 90 => requesting_port_identity(47 downto 40) := ram_data;
                        when 91 => requesting_port_identity(39 downto 32) := ram_data;
                        when 92 => requesting_port_identity(31 downto 24) := ram_data;
                        when 93 => requesting_port_identity(23 downto 16) := ram_data;
                        when 94 => requesting_port_identity(15 downto 8) := ram_data;
                        when 95 => requesting_port_identity(7 downto 0) := ram_data;
                        when others => null;
                    end case;
                end if;
                
                if (get_message_type(ptp_message_type(3 downto 0)) = t_Announce) then
                    message_length := 105;
                    case byte_counter is
                        when 86 => ptp_announce_current_utc_offset(15 downto 8) <= ram_data;
                        when 87 => ptp_announce_current_utc_offset(7 downto 0) <= ram_data;
                        when 89 => ptp_announce_grandmaster_priority1(7 downto 0) <= ram_data;
                        when 90 => ptp_announce_grandmaster_clockClass(7 downto 0) <= ram_data;
                        when 91 => ptp_announce_grandmaster_clockAccuracy(7 downto 0) <= ram_data;
                        when 92 => ptp_announce_grandmaster_offset_scaled_log_variance(15 downto 8) <= ram_data;
                        when 93 => ptp_announce_grandmaster_offset_scaled_log_variance(7 downto 0) <= ram_data;
                        when 94 => ptp_announce_grandmaster_priority2(7 downto 0) <= ram_data;
                        when 95 => ptp_clock_identity(63 downto 56) <= ram_data;
                        when 96 => ptp_clock_identity(55 downto 48) <= ram_data;
                        when 97 => ptp_clock_identity(47 downto 40) <= ram_data;
                        when 98 => ptp_clock_identity(39 downto 32) <= ram_data;
                        when 99 => ptp_clock_identity(31 downto 24) <= ram_data;
                        when 100 => ptp_clock_identity(23 downto 16) <= ram_data;
                        when 101 => ptp_clock_identity(15 downto 8) <= ram_data;
                        when 102 => ptp_clock_identity(7 downto 0) <= ram_data;
                        when 103 => ptp_announce_steps_removed(15 downto 8) <= ram_data;
                        when 104 => ptp_announce_steps_removed(7 downto 0) <= ram_data;
                        when 105 => ptp_announce_time_source(7 downto 0) <= ram_data;
                        when others => null;
                    end case;
                end if;
                
                byte_counter <= byte_counter + 1;
                ram_read_address <= to_unsigned(byte_counter + 1, 11);

                if byte_counter = message_length then
                    s_SM_PtpParser <= s_Interpret_Packet;
                end if;

            elsif (s_SM_PtpParser = s_Interpret_Packet) then
                -- VERSION CHECK: Only accept PTPv2 packets.
                -- Announce messages always pass through so the BMC can see all peers.
                if ptp_version /= x"2" or
                   (ptp_source_port_identity /= ptp_current_leader_id_i
                    and is_leader = '0' and ptp_is_follower_i = '1'
                    and ptp_message_type(3 downto 0) /= x"B") then
                    s_SM_PtpParser <= s_Done;
                else
                    case ptp_message_type(3 downto 0) is
                        when x"0" =>
                            -- Sync Message
                            if (is_leader = '0' and ptp_is_follower_i = '1') then
                                active_sequence_id := ptp_sequence_id;
                                stored_t2_seconds <= latched_rx_timestamp_seconds(3 downto 0);
                                stored_t2_nanoseconds <= latched_rx_timestamp_nanoseconds;
                            end if;
                            s_SM_PtpParser <= s_Done;
                            
                        when x"1" =>
                            -- Delay_Req Message
                            if is_leader = '1' and ptp_is_follower_i = '0' then
                                rx_follower_identity_o <= ptp_source_port_identity & ptp_source_port_port_number;
                                rx_timestamp_seconds_o <= latched_rx_timestamp_seconds;
                                rx_timestamp_nanoseconds_o <= latched_rx_timestamp_nanoseconds;
                                sequence_id_o <= ptp_sequence_id;
                                send_delay_resp_o <= '1';
                            end if;
                            s_SM_PtpParser <= s_Done;
                            
                        when x"2" => s_SM_PtpParser <= s_Done;  -- Pdelay_Req
                        when x"3" => s_SM_PtpParser <= s_Done;  -- Pdelay_Resp
                            
                        when x"8" =>
                            -- Follow_Up Message
                            if (is_leader = '0' and ptp_is_follower_i = '1') then
                                log_msg_interval_o <= signed(ptp_log_msg_interval);
                                log_msg_interval_valid_o <= '1';

                                if ptp_sequence_id = active_sequence_id then
                                    if (clock_configured = '0') then
                                        -- Store T1 for clock set, then compute elapsed time compensation
                                        stored_t1_seconds <= ptp_origin_timestamp_seconds;
                                        stored_t1_nanoseconds <= ptp_origin_timestamp_nanoseconds;
                                        configureClock <= '1';
                                    else
                                        stored_t1_seconds <= ptp_origin_timestamp_seconds;
                                        stored_t1_nanoseconds <= ptp_origin_timestamp_nanoseconds;
                                        sequence_id_o <= ptp_sequence_id;
                                        send_delay_req_o <= '1';
                                        s_SM_PtpParser <= s_Done;
                                    end if;
                                else
                                    s_SM_PtpParser <= s_Done;
                                end if;
                            else
                                s_SM_PtpParser <= s_Done;
                            end if;
                            
                        when x"9" =>
                            -- Delay_resp Message
                            if (is_leader = '0' and ptp_is_follower_i = '1') then
                                if ptp_sequence_id = active_sequence_id and requesting_port_identity = my_clock_id then
                                    stored_t4_seconds <= ptp_origin_timestamp_seconds(3 downto 0);
                                    stored_t4_nanoseconds <= ptp_origin_timestamp_nanoseconds;
                                    s_SM_PtpParser <= s_Calc_Stage1;
                                else
                                    s_SM_PtpParser <= s_Done;
                                end if;
                            else
                                s_SM_PtpParser <= s_Done;
                            end if;
                            
                        when x"A" => s_SM_PtpParser <= s_Done;  -- Pdelay_Resp_Follow_Up
                        when x"B" =>
                            -- Announce: publish dataset to BMC
                            announce_clock_identity_o        <= ptp_clock_identity;
                            announce_priority1_o             <= ptp_announce_grandmaster_priority1;
                            announce_clock_class_o           <= ptp_announce_grandmaster_clockClass;
                            announce_clock_accuracy_o        <= ptp_announce_grandmaster_clockAccuracy;
                            announce_offset_scaled_log_var_o <= ptp_announce_grandmaster_offset_scaled_log_variance;
                            announce_priority2_o             <= ptp_announce_grandmaster_priority2;
                            announce_steps_removed_o         <= ptp_announce_steps_removed;
                            announce_time_source_o           <= ptp_announce_time_source;
                            announce_log_msg_interval_o      <= ptp_log_msg_interval;
                            announce_valid_o                 <= '1';
                            s_SM_PtpParser <= s_Done;
                        when x"C" => s_SM_PtpParser <= s_Done;  -- Signaling
                        when x"D" => s_SM_PtpParser <= s_Done;  -- Management
                        when others => s_SM_PtpParser <= s_Done;
                    end case;
                end if;

            

            elsif (s_SM_PtpParser = s_Calc_Stage1) then
                -- ============================================
                -- CALCULATION Stage 1: Calculate raw deltas
                -- delta_m2s = T2 - T1 (Master to Slave delay)
                -- delta_s2m = T4 - T3 (Slave to Master delay)
                -- ============================================
                delta_m2s_reg <= timestamp_diff_ns(
                    stored_t2_seconds, stored_t2_nanoseconds,
                    stored_t1_seconds(3 downto 0), stored_t1_nanoseconds
                );
                
                delta_s2m_reg <= timestamp_diff_ns(
                    stored_t4_seconds, stored_t4_nanoseconds,
                    stored_t3_seconds, stored_t3_nanoseconds
                );

                -- Initialize min search
                min_search_state <= MS_IDLE;
                -- Latch the active depth on entry, clamped against MIN_FILTER_DEPTH
                if to_integer(min_filter_active_depth_i) = 0 then
                    eff_min_filter_depth <= 1;
                elsif to_integer(min_filter_active_depth_i) > MIN_FILTER_DEPTH then
                    eff_min_filter_depth <= MIN_FILTER_DEPTH;
                else
                    eff_min_filter_depth <= to_integer(min_filter_active_depth_i);
                end if;
                if min_filter_enable_i = '1' then
                    s_SM_PtpParser <= s_Calc_MinFilter;
                else
                    s_SM_PtpParser <= s_Calc_Bypass;
                end if;

            elsif (s_SM_PtpParser = s_Calc_MinFilter) then
                -- ============================================
                -- CALCULATION Stage 2: Min Filter with sequential search
                -- This stage runs multiple clock cycles to find minimum
                -- values in both buffers sequentially.
                -- ============================================
                case min_search_state is
                    when MS_IDLE =>
                        -- Write new m2s sample (address range 0..DEPTH-1)
                        delay_buffer(m2s_write_idx) <= delta_m2s_reg;

                        -- Update write index (circular within active depth)
                        if m2s_write_idx >= eff_min_filter_depth - 1 then
                            m2s_write_idx <= 0;
                        else
                            m2s_write_idx <= m2s_write_idx + 1;
                        end if;

                        -- Update fill count (cap at active depth)
                        if m2s_fill_count < eff_min_filter_depth then
                            m2s_fill_count <= m2s_fill_count + 1;
                        end if;

                        -- Initialize min search for m2s; seed with current sample
                        search_idx <= 0;
                        ram_rd_addr <= 0;  -- M2S region starts at 0
                        temp_min_m2s <= delta_m2s_reg;
                        current_min_abs <= unsigned(abs(delta_m2s_reg));
                        min_search_state <= MS_M2S_ISSUE;

                    when MS_M2S_ISSUE =>
                        -- Address is already on ram_rd_addr, data arrives next cycle.
                        if search_idx < m2s_fill_count then
                            min_search_state <= MS_M2S_FETCH;
                        else
                            -- Buffer not yet filled past this index, skip remaining
                            min_search_state <= MS_S2M_CALC;
                        end if;

                    when MS_M2S_FETCH =>
                        -- Synchronous BRAM output is valid now on ram_rd_data
                        fetched_val <= ram_rd_data;
                        fetched_abs <= unsigned(abs(ram_rd_data));
                        min_search_state <= MS_M2S_CMP;

                    when MS_M2S_CMP =>
                        if fetched_abs < current_min_abs then
                            temp_min_m2s <= fetched_val;
                            current_min_abs <= fetched_abs;
                        end if;

                        if search_idx >= eff_min_filter_depth - 1 then
                            min_search_state <= MS_S2M_CALC;
                        else
                            search_idx <= search_idx + 1;
                            ram_rd_addr <= ram_rd_addr + 1;
                            min_search_state <= MS_M2S_ISSUE;
                        end if;

                    when MS_S2M_CALC =>
                        -- Write new s2m sample (address range DEPTH..2*DEPTH-1)
                        delay_buffer(MIN_FILTER_DEPTH + s2m_write_idx) <= delta_s2m_reg;

                        if s2m_write_idx >= eff_min_filter_depth - 1 then
                            s2m_write_idx <= 0;
                        else
                            s2m_write_idx <= s2m_write_idx + 1;
                        end if;

                        if s2m_fill_count < eff_min_filter_depth then
                            s2m_fill_count <= s2m_fill_count + 1;
                        end if;

                        -- Initialize min search for s2m; seed with current sample
                        search_idx <= 0;
                        ram_rd_addr <= MIN_FILTER_DEPTH;  -- S2M region starts here
                        temp_min_s2m <= delta_s2m_reg;
                        current_min_abs <= unsigned(abs(delta_s2m_reg));
                        min_search_state <= MS_S2M_ISSUE;

                    when MS_S2M_ISSUE =>
                        if search_idx < s2m_fill_count then
                            min_search_state <= MS_S2M_FETCH;
                        else
                            min_search_state <= MS_OUTPUT;
                        end if;

                    when MS_S2M_FETCH =>
                        fetched_val <= ram_rd_data;
                        fetched_abs <= unsigned(abs(ram_rd_data));
                        min_search_state <= MS_S2M_CMP;

                    when MS_S2M_CMP =>
                        if fetched_abs < current_min_abs then
                            temp_min_s2m <= fetched_val;
                            current_min_abs <= fetched_abs;
                        end if;

                        if search_idx >= eff_min_filter_depth - 1 then
                            min_search_state <= MS_OUTPUT;
                        else
                            search_idx <= search_idx + 1;
                            ram_rd_addr <= ram_rd_addr + 1;
                            min_search_state <= MS_S2M_ISSUE;
                        end if;

                    when MS_OUTPUT =>
                        -- mean_path_delay = (min_m2s + min_s2m) / 2
                        -- offset          = (min_m2s - min_s2m) / 2
                        mean_path_delay_ns_o <= shift_right(temp_min_m2s + temp_min_s2m, 1);
                        if (ptp_locked_i = '1') then
                            offset_from_master_ns_o <= shift_right(temp_min_m2s - temp_min_s2m, 1);
                        else
                            offset_from_master_ns_o <= shift_right(delta_m2s_reg - delta_s2m_reg, 1);
                        end if;

                        ptp_calc_valid_o <= '1';
                        min_search_state <= MS_IDLE;
                        s_SM_PtpParser <= s_Done;
                end case;

            elsif (s_SM_PtpParser = s_Calc_Bypass) then
                -- Min filter disabled: use raw deltas directly.
                -- mean_path_delay = (delta_m2s + delta_s2m) / 2
                -- offset          = (delta_m2s - delta_s2m) / 2
                mean_path_delay_ns_o <= shift_right(delta_m2s_reg + delta_s2m_reg, 1);
                offset_from_master_ns_o <= shift_right(delta_m2s_reg - delta_s2m_reg, 1);
                ptp_calc_valid_o <= '1';
                s_SM_PtpParser <= s_Done;

            elsif (s_SM_PtpParser = s_Done) then
                
                    s_SM_PtpParser <= s_Idle;
                
            else
                s_SM_PtpParser <= s_Idle;
                byte_counter <= 0;
            end if;
        end if;
    end process main_proc;
end Behavioral;
