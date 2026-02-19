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
        wallclock_seconds_i     : in  unsigned(47 downto 0);
        wallclock_nanoseconds_i : in  unsigned(31 downto 0);
        timestamp_seconds_o         : out unsigned(47 downto 0);
        timestamp_nanoseconds_o     : out unsigned(31 downto 0);
        rx_timestamp_seconds_i     : in  unsigned(47 downto 0);
        rx_timestamp_nanoseconds_i : in  unsigned(31 downto 0);


        sequence_id_i         : in  unsigned(15 downto 0);
        send_delay_resp_in        : in std_logic;

        second_pulse_i       : in  std_logic;

        is_leader_i   : in  std_logic;
        is_follower_i : in  std_logic;

        tx_en_i: in  std_logic;
        request_port_identity_i : in std_logic_vector(79 downto 0);

        send_delay_req_i: in std_logic;
        ptp_log_message_interval_i : in std_logic_vector(7 downto 0);
        ptp_announce_log_message_interval_i : in std_logic_vector(7 downto 0);
        t3_valid_o: out std_logic;
		ptp_log_interval_o: out std_logic_vector(7 downto 0);

        config_valid_i: in std_logic


    );
    type t_state_leader is (s_Idle,
                            s_Send_Sync, s_Wait_for_Sync_Ack, s_Wait_for_Sync_Done, s_Latch_Sync_Timestamp,
                            s_Send_Follow_Up, s_Wait_for_Follow_Up_Ack, s_Wait_for_Follow_Up_Done,
                            s_Send_Announce, s_Wait_for_Announce_Ack, s_Wait_for_Announce_Done,
                            s_Send_Delay_Resp, s_Wait_for_Delay_Resp_Ack, s_Wait_for_Delay_Resp_Done);

    type t_state_follower is (f_Idle,

                              f_Send_Delay_Req, f_Wait_for_Delay_Req_Ack, f_Wait_for_Delay_Req_Done);

    type t_state_p2p_delay is (p2p_Idle,
                                 p2p_Send_Pdelay_Req, p2p_Wait_for_Pdelay_Req_Ack, p2p_Wait_for_Pdelay_Req_Done,
                                 p2p_Send_Pdelay_Resp, p2p_Wait_for_Pdelay_Resp_Ack, p2p_Wait_for_Pdelay_Resp_Done,
                                 p2p_Send_Pdelay_Follow_Up, p2p_Wait_for_Pdelay_Follow_Up_Ack, p2p_Wait_for_Pdelay_Follow_Up_Done);

    type t_state_controller is (c_Idle, c_Leader, c_Follower, c_Leader_Lost, c_Run_BMC);

    signal leader_state : t_state_leader := s_Idle;

    signal follower_state : t_state_follower := f_Idle;
    signal p2p_state: t_state_p2p_delay := p2p_Idle;
    signal c_state: t_state_controller := c_Idle;

    signal sync_sequence_id  : unsigned(15 downto 0) := (others => '0');
    signal announce_sequence_id : unsigned(15 downto 0) := (others => '0');
    signal tx_started : std_logic := '0';       -- Flag: have we seen tx_en go high?

    signal is_leader : std_logic := '0';
    signal second_pulse_i_reg : std_logic := '0';
    signal send_delay_resp_in_reg : std_logic := '0';  -- Edge detection for same clock domain
    signal sequence_id_i_latched : unsigned(15 downto 0) := (others => '0');
    signal request_port_identity_i_latched : std_logic_vector(79 downto 0) := (others => '0');
    signal send_delay_resp : std_logic := '0';
    signal rx_timestamp_nanoseconds_i_latched : unsigned(31 downto 0) := (others => '0');
    signal rx_timestamp_seconds_i_latched : unsigned(47 downto 0) := (others => '0');

    signal send_delay_req_i_reg : std_logic := '0';  -- Edge detection for same clock domain

    -- ============================================================
    -- CDC (Clock Domain Crossing) Synchronizers
    -- ONLY for signals from TX clock domain (ptpv2_sender runs on mac_tx_clock)
    -- ============================================================
    signal tx_en_i_meta                     : std_logic := '0';
    signal tx_en_i_sync                     : std_logic := '0';
    signal tx_en_i_prev                     : std_logic := '0';  -- for edge detection


    -- ============================================================
    -- Wallclock-based message timing
    -- ============================================================
    -- Sync interval decoded from ptp_log_message_interval_i
    signal sync_interval_sec : unsigned(7 downto 0)  := to_unsigned(1, 8);
    signal sync_interval_ns  : unsigned(31 downto 0) := (others => '0');

    -- Announce interval decoded from ptp_announce_log_message_interval_i
    signal announce_interval_sec : unsigned(7 downto 0)  := to_unsigned(2, 8);
    signal announce_interval_ns  : unsigned(31 downto 0) := (others => '0');

    -- Next-send wallclock targets (4-bit seconds, wraps cleanly)
    signal sync_next_sec     : unsigned(3 downto 0) := (others => '0');
    signal sync_next_ns      : unsigned(31 downto 0) := (others => '0');
    signal announce_next_sec : unsigned(3 downto 0) := (others => '0');
    signal announce_next_ns  : unsigned(31 downto 0) := (others => '0');

    -- Combinational time-reached flags
    signal sync_time_reached     : std_logic;
    signal announce_time_reached : std_logic;

    -- Wrapping difference for modular comparison
    signal sync_sec_diff     : unsigned(3 downto 0);
    signal announce_sec_diff : unsigned(3 downto 0);

    -- Registered wallclock for coherent multi-bit sampling (low 4 bits of seconds)
    signal wc_sec_r  : unsigned(3 downto 0) := (others => '0');
    signal wc_ns_r   : unsigned(31 downto 0) := (others => '0');

    -- Leader mode transition detection
    signal was_leader : std_logic := '0';

    -- Prevent register optimization/merging for metastability registers
    attribute PRESERVE : boolean;
    attribute PRESERVE of tx_en_i_meta : signal is true;
    attribute PRESERVE of tx_en_i_sync : signal is true;
    attribute PRESERVE of is_leader : signal is true;


    end entity;
architecture Behavioral of ptpv2_controller is
    constant ONE_SECOND_NS : unsigned(31 downto 0) := to_unsigned(1000000000, 32);
begin

    -- ============================================================
    -- Decode ptp_log_message_interval_i to seconds + nanoseconds
    -- logMessageInterval is signed: interval = 2^N seconds
    -- Supports -7 (7.8125ms) through +4 (16s)
    -- ============================================================
    interval_decode: process(ptp_log_message_interval_i)
    begin
        -- Default: 1 second
        sync_interval_sec <= to_unsigned(1, 8);
        sync_interval_ns  <= (others => '0');

        case to_integer(signed(ptp_log_message_interval_i)) is
            when -7 =>     sync_interval_sec <= to_unsigned(0, 8);
                           sync_interval_ns  <= to_unsigned(7812500, 32);
            when -6 =>     sync_interval_sec <= to_unsigned(0, 8);
                           sync_interval_ns  <= to_unsigned(15625000, 32);
            when -5 =>     sync_interval_sec <= to_unsigned(0, 8);
                           sync_interval_ns  <= to_unsigned(31250000, 32);
            when -4 =>     sync_interval_sec <= to_unsigned(0, 8);
                           sync_interval_ns  <= to_unsigned(62500000, 32);
            when -3 =>     sync_interval_sec <= to_unsigned(0, 8);
                           sync_interval_ns  <= to_unsigned(125000000, 32);
            when -2 =>     sync_interval_sec <= to_unsigned(0, 8);
                           sync_interval_ns  <= to_unsigned(250000000, 32);
            when -1 =>     sync_interval_sec <= to_unsigned(0, 8);
                           sync_interval_ns  <= to_unsigned(500000000, 32);
            when  0 =>     sync_interval_sec <= to_unsigned(1, 8);
                           sync_interval_ns  <= (others => '0');
            when  1 =>     sync_interval_sec <= to_unsigned(2, 8);
                           sync_interval_ns  <= (others => '0');
            when  2 =>     sync_interval_sec <= to_unsigned(4, 8);
                           sync_interval_ns  <= (others => '0');
            when  3 =>     sync_interval_sec <= to_unsigned(8, 8);
                           sync_interval_ns  <= (others => '0');
            when  4 =>     sync_interval_sec <= to_unsigned(16, 8);
                           sync_interval_ns  <= (others => '0');
            when others => sync_interval_sec <= to_unsigned(1, 8);
                           sync_interval_ns  <= (others => '0');
        end case;
    end process interval_decode;

    -- ============================================================
    -- Decode ptp_announce_log_message_interval_i to seconds + nanoseconds
    -- Same 2^N seconds encoding, supports -7 through +4
    -- ============================================================
    announce_interval_decode: process(ptp_announce_log_message_interval_i)
    begin
        -- Default: 2 seconds (logAnnounceInterval = 1)
        announce_interval_sec <= to_unsigned(2, 8);
        announce_interval_ns  <= (others => '0');

        case to_integer(signed(ptp_announce_log_message_interval_i)) is
            when -7 =>     announce_interval_sec <= to_unsigned(0, 8);
                           announce_interval_ns  <= to_unsigned(7812500, 32);
            when -6 =>     announce_interval_sec <= to_unsigned(0, 8);
                           announce_interval_ns  <= to_unsigned(15625000, 32);
            when -5 =>     announce_interval_sec <= to_unsigned(0, 8);
                           announce_interval_ns  <= to_unsigned(31250000, 32);
            when -4 =>     announce_interval_sec <= to_unsigned(0, 8);
                           announce_interval_ns  <= to_unsigned(62500000, 32);
            when -3 =>     announce_interval_sec <= to_unsigned(0, 8);
                           announce_interval_ns  <= to_unsigned(125000000, 32);
            when -2 =>     announce_interval_sec <= to_unsigned(0, 8);
                           announce_interval_ns  <= to_unsigned(250000000, 32);
            when -1 =>     announce_interval_sec <= to_unsigned(0, 8);
                           announce_interval_ns  <= to_unsigned(500000000, 32);
            when  0 =>     announce_interval_sec <= to_unsigned(1, 8);
                           announce_interval_ns  <= (others => '0');
            when  1 =>     announce_interval_sec <= to_unsigned(2, 8);
                           announce_interval_ns  <= (others => '0');
            when  2 =>     announce_interval_sec <= to_unsigned(4, 8);
                           announce_interval_ns  <= (others => '0');
            when  3 =>     announce_interval_sec <= to_unsigned(8, 8);
                           announce_interval_ns  <= (others => '0');
            when  4 =>     announce_interval_sec <= to_unsigned(16, 8);
                           announce_interval_ns  <= (others => '0');
            when others => announce_interval_sec <= to_unsigned(2, 8);
                           announce_interval_ns  <= (others => '0');
        end case;
    end process announce_interval_decode;

    -- ============================================================
    -- Wallclock time comparisons (modular 4-bit wrapping)
    -- diff = wc - target; reached when diff is in [1..7] (past)
    -- or diff=0 and ns >= target_ns (same second, ns reached)
    -- ============================================================
    sync_sec_diff     <= wc_sec_r - sync_next_sec;
    announce_sec_diff <= wc_sec_r - announce_next_sec;

    sync_time_reached <= '1' when (sync_sec_diff(3) = '0' and sync_sec_diff /= "0000") or
                                  (sync_sec_diff = "0000" and wc_ns_r >= sync_next_ns)
                         else '0';

    announce_time_reached <= '1' when (announce_sec_diff(3) = '0' and announce_sec_diff /= "0000") or
                                      (announce_sec_diff = "0000" and wc_ns_r >= announce_next_ns)
                             else '0';

    cdc_sync_proc: process(clk)
    begin
        if rising_edge(clk) then
            -- TX domain signals (from ptpv2_sender on mac_tx_clock)
            tx_en_i_meta            <= tx_en_i;
            tx_en_i_sync            <= tx_en_i_meta;
            tx_en_i_prev            <= tx_en_i_sync;  -- Stage 3 for edge detection
        end if;
    end process cdc_sync_proc;

    process(clk, reset_n)
        variable ns_sum : unsigned(32 downto 0);  -- 33-bit for overflow detection
    begin
        if reset_n = '0' then
            tx_message_type_o       <= (others => '0');
            sequence_id_o           <= (others => '0');
            frame_start_o           <= '0';
            request_port_identity_o <= (others => '0');
            sync_sequence_id        <= (others => '0');
            announce_sequence_id    <= (others => '0');
            is_leader               <= '0';
            second_pulse_i_reg      <= '0';
            send_delay_resp_in_reg  <= '0';
            leader_state           <= s_Idle;
            tx_started              <= '0';
            send_delay_resp         <= '0';
            timestamp_seconds_o     <= (others => '0');
            timestamp_nanoseconds_o <= (others => '0');
            follower_state         <= f_Idle;
            t3_valid_o              <= '0';
            was_leader              <= '0';
            sync_next_sec           <= (others => '1');
            sync_next_ns            <= (others => '1');
            announce_next_sec       <= (others => '1');
            announce_next_ns        <= (others => '1');
            wc_sec_r                <= (others => '0');
            wc_ns_r                 <= (others => '0');
            ptp_log_interval_o      <= (others => '0');
        elsif rising_edge(clk) then
            -- Register wallclock once per cycle for coherent sampling
            wc_sec_r <= wallclock_seconds_i(3 downto 0);
            wc_ns_r  <= wallclock_nanoseconds_i;

            -- Edge detection registers (same clock domain - no CDC needed)
            second_pulse_i_reg <= second_pulse_i;
            send_delay_resp_in_reg <= send_delay_resp_in;

            -- NOTE: frame_start_o is managed by the state machine only
            -- Do NOT clear it here based on tx_en - the state machine handles this




            case p2p_state is
                when p2p_Idle =>
                    -- P2P delay mechanism not implemented yet
                    p2p_state <= p2p_Idle;
                when p2p_Send_Pdelay_Req =>
                    -- P2P delay mechanism not implemented yet
                    p2p_state <= p2p_Idle;
                when p2p_Wait_for_Pdelay_Req_Ack =>
                    -- P2P delay mechanism not implemented yet
                    p2p_state <= p2p_Idle;
                when p2p_Wait_for_Pdelay_Req_Done =>
                    -- P2P delay mechanism not implemented yet
                    p2p_state <= p2p_Idle;
                when p2p_Send_Pdelay_Resp =>
                    -- P2P delay mechanism not implemented yet
                    p2p_state <= p2p_Wait_for_Pdelay_Resp_Ack;
                when p2p_Wait_for_Pdelay_Resp_Ack =>
                    -- P2P delay mechanism not implemented yet
                    p2p_state <= p2p_Wait_for_Pdelay_Resp_Done;
                when p2p_Wait_for_Pdelay_Resp_Done =>
                    -- P2P delay mechanism not implemented yet
                    p2p_state <= p2p_Send_Pdelay_Follow_Up;
                when p2p_Send_Pdelay_Follow_Up =>
                    -- P2P delay mechanism not implemented yet
                    p2p_state <= p2p_Wait_for_Pdelay_Follow_Up_Ack;
                when p2p_Wait_for_Pdelay_Follow_Up_Ack =>
                    -- P2P delay mechanism not implemented yet
                    p2p_state <= p2p_Wait_for_Pdelay_Follow_Up_Done;
                when p2p_Wait_for_Pdelay_Follow_Up_Done =>
                    -- P2P delay mechanism not implemented yet
                    p2p_state <= p2p_Idle;
                when others =>
                    p2p_state <= p2p_Idle;
            end case;



            if (is_leader_i = '1' and is_follower_i = '0') then

                -- Initialize timers on leader mode entry
                if was_leader = '0' then
                    -- First cycle as leader: schedule first send from registered wallclock
                    -- Do NOT run the state machine this cycle (else branch)
                    -- to prevent it from clobbering these init values.
                    ns_sum := ('0' & wc_ns_r) + ('0' & sync_interval_ns);
                    if ns_sum >= ('0' & ONE_SECOND_NS) then
                        sync_next_ns  <= ns_sum(31 downto 0) - ONE_SECOND_NS;
                        sync_next_sec <= wc_sec_r + resize(sync_interval_sec, 4) + 1;
                    else
                        sync_next_ns  <= ns_sum(31 downto 0);
                        sync_next_sec <= wc_sec_r + resize(sync_interval_sec, 4);
                    end if;
                    ns_sum := ('0' & wc_ns_r) + ('0' & announce_interval_ns);
                    if ns_sum >= ('0' & ONE_SECOND_NS) then
                        announce_next_ns  <= ns_sum(31 downto 0) - ONE_SECOND_NS;
                        announce_next_sec <= wc_sec_r + resize(announce_interval_sec, 4) + 1;
                    else
                        announce_next_ns  <= ns_sum(31 downto 0);
                        announce_next_sec <= wc_sec_r + resize(announce_interval_sec, 4);
                    end if;
                else

                -- Leader logic - simple state machine (runs from 2nd cycle as leader onwards)
                case leader_state is
                    when s_Idle =>
                        -- Priority: delay_resp > sync > announce
                        if (send_delay_resp = '1') then
                            send_delay_resp <= '0';
                            leader_state <= s_Send_Delay_Resp;
                        elsif (sync_time_reached = '1') then
                            leader_state <= s_Send_Sync;
											 							ptp_log_interval_o <= ptp_log_message_interval_i;

                            -- Advance sync timer: next = wallclock + interval
                            ns_sum := ('0' & wc_ns_r) + ('0' & sync_interval_ns);
                            if ns_sum >= ('0' & ONE_SECOND_NS) then
                                sync_next_ns  <= ns_sum(31 downto 0) - ONE_SECOND_NS;
                                sync_next_sec <= wc_sec_r + resize(sync_interval_sec, 4) + 1;
                            else
                                sync_next_ns  <= ns_sum(31 downto 0);
                                sync_next_sec <= wc_sec_r + resize(sync_interval_sec, 4);
                            end if;
                        elsif (announce_time_reached = '1') then
                            leader_state <= s_Send_Announce;
											ptp_log_interval_o <= ptp_announce_log_message_interval_i;

                            -- Advance announce timer: next = wallclock + interval
                            ns_sum := ('0' & wc_ns_r) + ('0' & announce_interval_ns);
                            if ns_sum >= ('0' & ONE_SECOND_NS) then
                                announce_next_ns  <= ns_sum(31 downto 0) - ONE_SECOND_NS;
                                announce_next_sec <= wc_sec_r + resize(announce_interval_sec, 4) + 1;
                            else
                                announce_next_ns  <= ns_sum(31 downto 0);
                                announce_next_sec <= wc_sec_r + resize(announce_interval_sec, 4);
                            end if;
                        end if;

                    -- ========================================
                    -- SYNC MESSAGE SEQUENCE
                    -- ========================================
                    when s_Send_Sync =>
										ptp_log_interval_o <= ptp_log_message_interval_i;
                        tx_message_type_o <= "0000";
                        sync_sequence_id <= sync_sequence_id + 1;
                        sequence_id_o <= sync_sequence_id + 1;
                        timestamp_nanoseconds_o <= wallclock_nanoseconds_i;
                        timestamp_seconds_o <= wallclock_seconds_i;
                        frame_start_o <= '1';
                        tx_started <= '0';
                        leader_state <= s_Wait_for_Sync_Ack;

                    when s_Wait_for_Sync_Ack =>
                        -- Must see tx_en go from 0->1 (rising edge)
                        -- tx_started tracks if we've seen the rising edge
                        if (tx_en_i_prev = '0' and tx_en_i_sync = '1') then
                            -- Rising edge detected - sender has started
                            tx_started <= '1';
                            frame_start_o <= '0';  -- Can deassert now
                        end if;
                        -- Wait for sender to finish (tx_en goes low after rising edge seen)
                        if (tx_started = '1' and tx_en_i_sync = '0') then
                            tx_started <= '0';
                            leader_state <= s_Wait_for_Sync_Done;
                        end if;

                    when s_Wait_for_Sync_Done =>
                        -- Extra cycle for CDC timestamp to settle
                        timestamp_nanoseconds_o <= wallclock_nanoseconds_i;
                        timestamp_seconds_o <= wallclock_seconds_i;
                        leader_state <= s_Latch_Sync_Timestamp;

                    when s_Latch_Sync_Timestamp =>
                        -- Another cycle to ensure timestamp is stable
                        leader_state <= s_Send_Follow_Up;

                    -- ========================================
                    -- FOLLOW-UP MESSAGE SEQUENCE
                    -- ========================================
                    when s_Send_Follow_Up =>
										ptp_log_interval_o <= ptp_log_message_interval_i;
                        tx_message_type_o <= x"8";
                        sequence_id_o <= sync_sequence_id;
                        frame_start_o <= '1';
                        tx_started <= '0';
                        leader_state <= s_Wait_for_Follow_Up_Ack;

                    when s_Wait_for_Follow_Up_Ack =>
                        -- Must see tx_en go from 0->1 (rising edge)
                        if (tx_en_i_prev = '0' and tx_en_i_sync = '1') then
                            tx_started <= '1';
                            frame_start_o <= '0';
                        end if;
                        if (tx_started = '1' and tx_en_i_sync = '0') then
                            tx_started <= '0';
                            leader_state <= s_Wait_for_Follow_Up_Done;
                        end if;

                    when s_Wait_for_Follow_Up_Done =>
                        leader_state <= s_Idle;

                    -- ========================================
                    -- ANNOUNCE MESSAGE SEQUENCE
                    -- ========================================
                    when s_Send_Announce =>
										ptp_log_interval_o <= ptp_announce_log_message_interval_i;
                        tx_message_type_o <= x"B";  -- Announce message type
                        announce_sequence_id <= announce_sequence_id + 1;
                        sequence_id_o <= announce_sequence_id + 1;
                        frame_start_o <= '1';
                        tx_started <= '0';
                        leader_state <= s_Wait_for_Announce_Ack;

                    when s_Wait_for_Announce_Ack =>
                        -- Must see tx_en go from 0->1 (rising edge)
                        if (tx_en_i_prev = '0' and tx_en_i_sync = '1') then
                            tx_started <= '1';
                            frame_start_o <= '0';
                        end if;
                        if (tx_started = '1' and tx_en_i_sync = '0') then
                            tx_started <= '0';
                            leader_state <= s_Wait_for_Announce_Done;
                        end if;

                    when s_Wait_for_Announce_Done =>
                        leader_state <= s_Idle;

                    -- ========================================
                    -- DELAY RESPONSE MESSAGE SEQUENCE
                    -- ========================================
                    when s_Send_Delay_Resp =>
										ptp_log_interval_o <= ptp_log_message_interval_i;
                        timestamp_nanoseconds_o <= rx_timestamp_nanoseconds_i_latched;
                        timestamp_seconds_o <= rx_timestamp_seconds_i_latched;
                        frame_start_o <= '1';
                        tx_started <= '0';
                        leader_state <= s_Wait_for_Delay_Resp_Ack;

                    when s_Wait_for_Delay_Resp_Ack =>
                        -- Must see tx_en go from 0->1 (rising edge)
                        if (tx_en_i_prev = '0' and tx_en_i_sync = '1') then
                            tx_started <= '1';
                            frame_start_o <= '0';
                        end if;
                        if (tx_started = '1' and tx_en_i_sync = '0') then
                            tx_started <= '0';
                            leader_state <= s_Wait_for_Delay_Resp_Done;
                        end if;

                    when s_Wait_for_Delay_Resp_Done =>
                        leader_state <= s_Idle;

                    when others =>
                        leader_state <= s_Idle;

                end case;
                end if; -- was_leader if/else
                was_leader <= '1';

                -- Latch delay_resp request data on rising edge (SAME clock domain - direct signals)
                if (send_delay_resp_in_reg = '0' and send_delay_resp_in = '1') then
                    send_delay_resp <= '1';
                    request_port_identity_i_latched <= request_port_identity_i;
                    sequence_id_i_latched <= sequence_id_i;
                    rx_timestamp_nanoseconds_i_latched <= rx_timestamp_nanoseconds_i;
                    rx_timestamp_seconds_i_latched <= rx_timestamp_seconds_i;
                end if;
                follower_state <= f_Idle;  -- Ensure follower state machine is reset
            elsif (is_leader_i = '0' and is_follower_i = '1') then
                -- Follower logic - reset state machine when not leader
                leader_state <= s_Idle;
                was_leader <= '0';

                send_delay_resp <= '0';
                t3_valid_o <= '0';

                case follower_state is
                    when f_Idle =>
                        -- Check for delay_req request
                        if (send_delay_req_i_reg = '0' and send_delay_req_i = '1') then
                            follower_state <= f_Send_Delay_Req;
                        else
                            send_delay_req_i_reg <= send_delay_req_i;
                        end if;

                    when f_Send_Delay_Req =>
                        tx_message_type_o <= "0001";

                        sequence_id_o <= sequence_id_i;
                        frame_start_o <= '1';
                        tx_started <= '0';
                        follower_state <= f_Wait_for_Delay_Req_Ack;

                    when f_Wait_for_Delay_Req_Ack =>
                        -- Must see tx_en go from 0->1 (rising edge)
                        if (tx_en_i_prev = '0' and tx_en_i_sync = '1') then
                            tx_started <= '1';
                            frame_start_o <= '0';
                        end if;
                        if (tx_started = '1' and tx_en_i_sync = '0') then
                            tx_started <= '0';
                            follower_state <= f_Wait_for_Delay_Req_Done;

                            timestamp_nanoseconds_o <= wallclock_nanoseconds_i;
                            timestamp_seconds_o <= wallclock_seconds_i;
                        end if;

                    when f_Wait_for_Delay_Req_Done =>
                        -- Extra cycle for CDC timestamp to settle
                        follower_state <= f_Idle;
                        t3_valid_o <= '1';

                    when others =>
                        follower_state <= f_Idle;

                end case;
                else
                -- Not a leader or follower - reset state machines and outputs
                leader_state <= s_Idle;
                follower_state <= f_Idle;
                was_leader <= '0';
            end if;

        end if;
    end process;

end architecture;
