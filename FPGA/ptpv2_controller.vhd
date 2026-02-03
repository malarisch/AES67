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
        tx_ready_timestamp_seconds_i     : in  unsigned(47 downto 0);
        tx_ready_timestamp_nanoseconds_i : in  unsigned(31 downto 0);
        timestamp_seconds_o         : out unsigned(47 downto 0);
        timestamp_nanoseconds_o     : out unsigned(31 downto 0);
        rx_timestamp_seconds_i     : in  unsigned(47 downto 0);
        rx_timestamp_nanoseconds_i : in  unsigned(31 downto 0);


        sequence_id_i         : in  unsigned(15 downto 0);
        send_delay_resp_in        : in std_logic;

        second_pulse_i       : in  std_logic;

        is_leader_override_i   : in  std_logic;

        tx_en_i: in  std_logic;
        request_port_identity_i : in std_logic_vector(79 downto 0);
        
        
        send_delay_req_i: in std_logic;
        t3_valid_o: out std_logic
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

    signal sequence_id_reg : unsigned(15 downto 0) := (others => '0');
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
    
    signal tx_ready_ts_sec_meta             : unsigned(47 downto 0) := (others => '0');
    signal tx_ready_ts_sec_sync             : unsigned(47 downto 0) := (others => '0');
    signal tx_ready_ts_nsec_meta            : unsigned(31 downto 0) := (others => '0');
    signal tx_ready_ts_nsec_sync            : unsigned(31 downto 0) := (others => '0');

    -- Prevent register optimization/merging for metastability registers
    attribute PRESERVE : boolean;
    attribute PRESERVE of tx_en_i_meta : signal is true;
    attribute PRESERVE of tx_en_i_sync : signal is true;
    attribute PRESERVE of tx_ready_ts_sec_meta : signal is true;
    attribute PRESERVE of tx_ready_ts_nsec_meta : signal is true;
    attribute PRESERVE of is_leader : signal is true;


    end entity;
architecture Behavioral of ptpv2_controller is
begin

    
    -- ============================================================
    -- CDC Synchronization Process (2-stage synchronizers)
    -- ONLY for signals crossing from TX clock domain (mac_tx_clock)
    -- ============================================================
    cdc_sync_proc: process(clk)
    begin
        if rising_edge(clk) then
            -- TX domain signals (from ptpv2_sender on mac_tx_clock)
            tx_en_i_meta            <= tx_en_i;
            tx_en_i_sync            <= tx_en_i_meta;
            tx_en_i_prev            <= tx_en_i_sync;  -- Stage 3 for edge detection
            
            tx_ready_ts_sec_meta    <= tx_ready_timestamp_seconds_i;
            tx_ready_ts_sec_sync    <= tx_ready_ts_sec_meta;
            tx_ready_ts_nsec_meta   <= tx_ready_timestamp_nanoseconds_i;
            tx_ready_ts_nsec_sync   <= tx_ready_ts_nsec_meta;
        end if;
    end process cdc_sync_proc;

    process(clk, reset_n)
    begin
        if reset_n = '0' then
            tx_message_type_o       <= (others => '0');
            sequence_id_o           <= (others => '0');
            frame_start_o           <= '0';
            request_port_identity_o <= (others => '0');
            sequence_id_reg         <= (others => '0');
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
        elsif rising_edge(clk) then
            -- Edge detection registers (same clock domain - no CDC needed)
            second_pulse_i_reg <= second_pulse_i;
            send_delay_resp_in_reg <= send_delay_resp_in;

            -- NOTE: frame_start_o is managed by the state machine only
            -- Do NOT clear it here based on tx_en - the state machine handles this

            -- PTPv2 controller logic
            if (is_leader_override_i = '1') then
                is_leader <= '1';
            else
                is_leader <= '0';
            end if;

            case c_state is
                when c_Idle =>
                    if (is_leader_override_i = '1') then
                        c_state <= c_Leader;
                    else 
                        c_state <= c_Run_BMC;
                    end if;

                when c_Leader =>
                    -- Leader state
                    c_state <= c_Leader;  -- Stay in leader state
                    is_leader <= '1';

                when c_Follower =>
                    -- Follower state
                    c_state <= c_Follower;  -- Stay in follower state
                    is_leader <= '0';

                when c_Leader_Lost =>
                    -- Lost leadership - transition to follower
                    c_state <= c_Run_BMC;

                when c_Run_BMC =>
                    -- Run Best Master Clock algorithm (not implemented here)
                    c_state <= c_Idle;

                when others =>
                    c_state <= c_Idle;
            end case;


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



            if (is_leader = '1') then
                -- Leader logic - simple state machine
                case leader_state is
                    when s_Idle =>
                        -- Check for pending delay_resp request first (higher priority)
                        if (send_delay_resp = '1') then
                            send_delay_resp <= '0';
                            leader_state <= s_Send_Delay_Resp;
                        -- Then check for second pulse to send Sync
                        elsif (second_pulse_i_reg = '0' and second_pulse_i = '1') then
                            leader_state <= s_Send_Sync;
                        end if;

                    -- ========================================
                    -- SYNC MESSAGE SEQUENCE
                    -- ========================================
                    when s_Send_Sync =>
                        tx_message_type_o <= "0000";
                        sequence_id_reg <= sequence_id_reg + 1;
                        sequence_id_o <= sequence_id_reg + 1;
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
                        timestamp_nanoseconds_o <= tx_ready_ts_nsec_sync;
                        timestamp_seconds_o <= tx_ready_ts_sec_sync;
                        leader_state <= s_Latch_Sync_Timestamp;

                    when s_Latch_Sync_Timestamp =>
                        -- Another cycle to ensure timestamp is stable
                        leader_state <= s_Send_Follow_Up;

                    -- ========================================
                    -- FOLLOW-UP MESSAGE SEQUENCE
                    -- ========================================
                    when s_Send_Follow_Up =>
                        tx_message_type_o <= x"8";
                        sequence_id_o <= sequence_id_reg;
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
                        leader_state <= s_Send_Announce;

                    -- ========================================
                    -- ANNOUNCE MESSAGE SEQUENCE
                    -- ========================================
                    when s_Send_Announce =>
                        tx_message_type_o <= x"B";  -- Announce message type
                        sequence_id_o <= sequence_id_reg;
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
                        tx_message_type_o <= "1001";
                        sequence_id_o <= sequence_id_i_latched;
                        request_port_identity_o <= request_port_identity_i_latched;
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
                
                -- Latch delay_resp request data on rising edge (SAME clock domain - direct signals)
                if (send_delay_resp_in_reg = '0' and send_delay_resp_in = '1') then
                    send_delay_resp <= '1';
                    request_port_identity_i_latched <= request_port_identity_i;
                    sequence_id_i_latched <= sequence_id_i;
                    rx_timestamp_nanoseconds_i_latched <= rx_timestamp_nanoseconds_i;
                    rx_timestamp_seconds_i_latched <= rx_timestamp_seconds_i;
                end if;
                follower_state <= f_Idle;  -- Ensure follower state machine is reset
            else
                -- Follower logic - reset state machine when not leader
                leader_state <= s_Idle;
                
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
                        timestamp_nanoseconds_o <= wallclock_nanoseconds_i;
                        timestamp_seconds_o <= wallclock_seconds_i;
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

                            timestamp_nanoseconds_o <= tx_ready_ts_nsec_sync;
                            timestamp_seconds_o <= tx_ready_ts_sec_sync;
                        end if;

                    when f_Wait_for_Delay_Req_Done =>
                        -- Extra cycle for CDC timestamp to settle
                        follower_state <= f_Idle;
                        t3_valid_o <= '1';

                    when others =>
                        follower_state <= f_Idle;

                end case;
            end if;

        end if;
    end process;

end architecture;