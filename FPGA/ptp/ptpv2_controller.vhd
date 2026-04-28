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
        tx_timestamp_seconds_i     : in  unsigned(47 downto 0);
        rx_timestamp_nanoseconds_i : in  unsigned(31 downto 0);
        tx_timestamp_nanoseconds_i      : in unsigned(31 downto 0);


        sequence_id_i         : in  unsigned(15 downto 0);
        send_delay_resp_in        : in std_logic;

        ms_pulse_i            : in  std_logic;

        is_leader_i   : in  std_logic;
        is_follower_i : in  std_logic;

        tx_en_i: in  std_logic;
        request_port_identity_i : in std_logic_vector(79 downto 0);

        send_delay_req_i: in std_logic;
        ptp_log_message_interval_i : in std_logic_vector(7 downto 0);
        ptp_announce_log_message_interval_i : in std_logic_vector(7 downto 0);
        t3_valid_o: out std_logic;
		ptp_log_interval_o: out std_logic_vector(7 downto 0);

        tx_done_sys_i: in STD_LOGIC;

		 parser_log_msg_interval_i : IN STD_LOGIC_VECTOR(7 downto 0)

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
    signal send_delay_resp_in_reg : std_logic := '0';  -- Edge detection for same clock domain
    signal sequence_id_i_latched : unsigned(15 downto 0) := (others => '0');
    signal request_port_identity_i_latched : std_logic_vector(79 downto 0) := (others => '0');
    signal send_delay_resp : std_logic := '0';
    signal timestamp_nanoseconds_i_latched : unsigned(31 downto 0) := (others => '0');
    signal timestamp_seconds_i_latched : unsigned(47 downto 0) := (others => '0');


    signal send_delay_req_i_reg : std_logic := '0';  -- Edge detection for same clock domain

    -- ============================================================
    -- CDC (Clock Domain Crossing) Synchronizers
    -- ONLY for signals from TX clock domain (ptpv2_sender runs on mac_tx_clock)
    -- ============================================================
    signal tx_en_i_meta                     : std_logic := '0';
    signal tx_en_i_sync                     : std_logic := '0';
    signal tx_en_i_prev                     : std_logic := '0';  -- for edge detection


    -- ============================================================
    -- Millisecond-based message scheduling
    -- ============================================================
    -- Wallclock-constrained ms counter. 16 bits wrap every ~65.5 s — far longer than
    -- any supported announce/sync interval (max 16 s), so modular compare
    -- (diff MSB) works cleanly.
    signal ms_counter        : unsigned(15 downto 0) := (others => '0');

    -- Decoded intervals in ms. Max supported interval is 16 s = 16000 ms.
    signal sync_interval_ms     : unsigned(15 downto 0) := to_unsigned(1000, 16);
    signal announce_interval_ms : unsigned(15 downto 0) := to_unsigned(2000, 16);

    -- Next-send targets (ms counter values).
    signal sync_next_ms     : unsigned(15 downto 0) := (others => '0');
    signal announce_next_ms : unsigned(15 downto 0) := (others => '0');

    -- Combinational time-reached flags (modular compare: past if MSB of
    -- (ms_counter - next) is 0 and the diff is non-zero — equivalent to
    -- "ms_counter is within the next half-wrap ahead of target").
    signal sync_time_reached     : std_logic;
    signal announce_time_reached : std_logic;
    signal sync_ms_diff          : unsigned(15 downto 0);
    signal announce_ms_diff      : unsigned(15 downto 0);

    -- Leader mode transition detection
    signal was_leader : std_logic := '0';

    -- Prevent register optimization/merging for metastability registers
    attribute PRESERVE : boolean;
    attribute PRESERVE of tx_en_i_meta : signal is true;
    attribute PRESERVE of tx_en_i_sync : signal is true;
    attribute PRESERVE of is_leader : signal is true;

    -- Registered inputs for interval decode (breaks port -> decode combinatorial path)
    signal ptp_log_msg_int_r     : std_logic_vector(7 downto 0) := (others => '0');
    signal ptp_ann_log_msg_int_r : std_logic_vector(7 downto 0) := (others => '0');

    signal interval_decode_step : std_logic := '0';
    end entity;
architecture Behavioral of ptpv2_controller is

    -- Decode log2(interval seconds) -> milliseconds.
    -- Covers the spec range -7..+4; anything outside falls back to 1000 ms.
    function log_interval_to_ms(log_int : std_logic_vector(7 downto 0))
        return unsigned is
    begin
        case log_int is
            when x"F9" => return to_unsigned(    8, 16); -- -7 ~  7.8 ms
            when x"FA" => return to_unsigned(   16, 16); -- -6 ~ 15.6 ms
            when x"FB" => return to_unsigned(   31, 16); -- -5 ~ 31.25 ms
            when x"FC" => return to_unsigned(   63, 16); -- -4 ~ 62.5 ms
            when x"FD" => return to_unsigned(  125, 16); -- -3  125 ms
            when x"FE" => return to_unsigned(  250, 16); -- -2  250 ms
            when x"FF" => return to_unsigned(  500, 16); -- -1  500 ms
            when x"00" => return to_unsigned( 1000, 16); --  0
            when x"01" => return to_unsigned( 2000, 16); --  1
            when x"02" => return to_unsigned( 4000, 16); --  2
            when x"03" => return to_unsigned( 8000, 16); --  3
            when x"04" => return to_unsigned(16000, 16); --  4
            when others => return to_unsigned(1000, 16);
        end case;
    end function;

begin

    -- Register raw log-interval inputs (1 cycle) and the decoded ms values
    -- (1 more cycle) to break the port -> decode combinatorial path.
    interval_input_reg: process(clk)
    begin
        if rising_edge(clk) then
            if (interval_decode_step = '1') then
            ptp_log_msg_int_r     <= ptp_log_message_interval_i;
            else
            ptp_ann_log_msg_int_r <= ptp_announce_log_message_interval_i;
            end if;
            interval_decode_step <= not interval_decode_step;
        end if;
    end process interval_input_reg;

    interval_decode: process(clk)
    begin
        if rising_edge(clk) then
            sync_interval_ms     <= log_interval_to_ms(ptp_log_msg_int_r);
            announce_interval_ms <= log_interval_to_ms(ptp_ann_log_msg_int_r);
        end if;
    end process interval_decode;

    -- Modular "time reached": target is in the past if (ms_counter - next)
    -- is within the lower half of the wrap window. The intervals are at most
    -- 16 s; the window is 65.5 s, so this comparison is unambiguous.
    sync_ms_diff          <= ms_counter - sync_next_ms;
    announce_ms_diff      <= ms_counter - announce_next_ms;
    sync_time_reached     <= '1' when sync_ms_diff(15) = '0'     else '0';
    announce_time_reached <= '1' when announce_ms_diff(15) = '0' else '0';

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
    begin
        if reset_n = '0' then
            tx_message_type_o       <= (others => '0');
            sequence_id_o           <= (others => '0');
            frame_start_o           <= '0';
            request_port_identity_o <= (others => '0');
            sync_sequence_id        <= (others => '0');
            announce_sequence_id    <= (others => '0');
            is_leader               <= '0';
            send_delay_resp_in_reg  <= '0';
            leader_state           <= s_Idle;
            tx_started              <= '0';
            send_delay_resp         <= '0';
            timestamp_seconds_o     <= (others => '0');
            timestamp_nanoseconds_o <= (others => '0');
            follower_state         <= f_Idle;
            t3_valid_o              <= '0';
            was_leader              <= '0';
            ms_counter              <= (others => '0');
            sync_next_ms            <= (others => '0');
            announce_next_ms        <= (others => '0');
            ptp_log_interval_o      <= (others => '0');
        elsif rising_edge(clk) then
            -- Free-running ms counter
            if ms_pulse_i = '1' then
                ms_counter <= ms_counter + 1;
            end if;

            -- Edge detection registers (same clock domain - no CDC needed)
            send_delay_resp_in_reg <= send_delay_resp_in;
            -- Do NOT clear it here based on tx_en - the state machine handles this




            case p2p_state is
                when p2p_Idle =>
                    -- P2P delay mechanism not implemented yet
                    p2p_state <= p2p_Idle;
                when p2p_Send_Pdelay_Req =>
                    p2p_state <= p2p_Idle;
                when p2p_Wait_for_Pdelay_Req_Ack =>
                    p2p_state <= p2p_Idle;
                when p2p_Wait_for_Pdelay_Req_Done =>
                    p2p_state <= p2p_Idle;
                when p2p_Send_Pdelay_Resp =>
                    p2p_state <= p2p_Wait_for_Pdelay_Resp_Ack;
                when p2p_Wait_for_Pdelay_Resp_Ack =>
                    p2p_state <= p2p_Wait_for_Pdelay_Resp_Done;
                when p2p_Wait_for_Pdelay_Resp_Done =>
                    p2p_state <= p2p_Send_Pdelay_Follow_Up;
                when p2p_Send_Pdelay_Follow_Up =>
                    p2p_state <= p2p_Wait_for_Pdelay_Follow_Up_Ack;
                when p2p_Wait_for_Pdelay_Follow_Up_Ack =>
                    p2p_state <= p2p_Wait_for_Pdelay_Follow_Up_Done;
                when p2p_Wait_for_Pdelay_Follow_Up_Done =>
                    p2p_state <= p2p_Idle;
                when others =>
                    p2p_state <= p2p_Idle;
            end case;



            if (is_leader_i = '1' and is_follower_i = '0') then

                -- Initialize timers on leader mode entry
                if was_leader = '0' then
                    -- First cycle as leader: schedule first send one interval
                    -- from "now". The decoded interval registers are already
                    -- valid (continuously updated).
                    sync_next_ms     <= ms_counter + sync_interval_ms;
                    announce_next_ms <= ms_counter + announce_interval_ms;
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

                            -- Advance sync timer by one interval
                            sync_next_ms <= sync_next_ms + sync_interval_ms;
                        elsif (announce_time_reached = '1') then
                            leader_state <= s_Send_Announce;
                            ptp_log_interval_o <= ptp_announce_log_message_interval_i;

                            -- Advance announce timer by one interval
                            announce_next_ms <= announce_next_ms + announce_interval_ms;
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
                        if (tx_started = '1' and tx_done_sys_i = '1') then
                            tx_started <= '0';
                            leader_state <= s_Wait_for_Sync_Done;
                        end if;

                    when s_Wait_for_Sync_Done =>
                        -- Copy dedicated sync TX timestamp to output
                        timestamp_nanoseconds_o <= tx_timestamp_nanoseconds_i;
                        timestamp_seconds_o <= tx_timestamp_seconds_i;
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
                        if (tx_started = '1' and tx_done_sys_i = '1') then
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
                        if (tx_started = '1' and tx_done_sys_i = '1') then
                            tx_started <= '0';
                            leader_state <= s_Wait_for_Announce_Done;
                        end if;

                    when s_Wait_for_Announce_Done =>
                        leader_state <= s_Idle;

                    -- ========================================
                    -- DELAY RESPONSE MESSAGE SEQUENCE
                    -- ========================================
                    when s_Send_Delay_Resp =>
                        sequence_id_o <= sequence_id_i_latched;
                        request_port_identity_o <= request_port_identity_i_latched;
                        tx_message_type_o <= x"9";
						ptp_log_interval_o <= ptp_log_message_interval_i;
                        timestamp_nanoseconds_o <= timestamp_nanoseconds_i_latched;
                        timestamp_seconds_o <= timestamp_seconds_i_latched;
                        frame_start_o <= '1';
                        tx_started <= '0';
                        leader_state <= s_Wait_for_Delay_Resp_Ack;

                    when s_Wait_for_Delay_Resp_Ack =>
                        -- Must see tx_en go from 0->1 (rising edge)
                        if (tx_en_i_prev = '0' and tx_en_i_sync = '1') then
                            tx_started <= '1';
                            frame_start_o <= '0';
                        end if;
                        if (tx_started = '1' and tx_done_sys_i = '1') then
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

                -- Latch delay_resp request data
                if (send_delay_resp_in_reg = '0' and send_delay_resp_in = '1') then
                    send_delay_resp <= '1';
                    request_port_identity_i_latched <= request_port_identity_i;
                    sequence_id_i_latched <= sequence_id_i;
                    timestamp_nanoseconds_i_latched <= rx_timestamp_nanoseconds_i;
                    timestamp_seconds_i_latched <= rx_timestamp_seconds_i;
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
                        ptp_log_interval_o <= parser_log_msg_interval_i;
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
                        -- Latch T3 
                        if (tx_started = '1') then
                            timestamp_nanoseconds_o <= tx_timestamp_nanoseconds_i;
                            timestamp_seconds_o <= tx_timestamp_seconds_i;
                        end if;
                        -- Wait for TX to finish
                        if (tx_started = '1' and tx_done_sys_i = '1') then
                            tx_started <= '0';
                            follower_state <= f_Wait_for_Delay_Req_Done;
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
