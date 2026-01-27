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
    type t_state_leader is (s_Idle, 
                            s_Send_Sync, s_Wait_for_Sync_Ack, s_Wait_for_Sync_Done, s_Latch_Sync_Timestamp,
                            s_Send_Follow_Up, s_Wait_for_Follow_Up_Ack, s_Wait_for_Follow_Up_Done, 
                            s_Send_Delay_Resp, s_Wait_for_Delay_Resp_Ack, s_Wait_for_Delay_Resp_Done);

    signal leader_state : t_state_leader := s_Idle;

    signal sequence_id_reg : unsigned(15 downto 0) := (others => '0');
    signal tx_started : std_logic := '0';       -- Flag: have we seen tx_en go high?

    signal is_leader : std_logic := '0';
    signal second_pulse_i_reg : std_logic := '0';
    signal send_delay_resp_in_reg : std_logic := '0';  -- Edge detection for same clock domain
    signal sequence_id_i_latched : unsigned(15 downto 0) := (others => '0');
    signal request_port_identity_i_latched : std_logic_vector(79 downto 0) := (others => '0');
    signal send_delay_resp : std_logic := '0';
    signal rx_timestamp_nanoseconds_i_latched : unsigned(31 downto 0) := (others => '0');
    signal rx_timestamp_seconds_i_latched : unsigned(31 downto 0) := (others => '0');

    -- ============================================================
    -- CDC (Clock Domain Crossing) Synchronizers
    -- ONLY for signals from TX clock domain (ptpv2_sender runs on mac_tx_clock)
    -- Parser signals are in SAME clock domain - no CDC needed!
    -- ============================================================
    signal tx_en_i_meta                     : std_logic := '0';
    signal tx_en_i_sync                     : std_logic := '0';
    signal tx_en_i_prev                     : std_logic := '0';  -- for edge detection
    
    signal tx_ready_ts_sec_meta             : unsigned(31 downto 0) := (others => '0');
    signal tx_ready_ts_sec_sync             : unsigned(31 downto 0) := (others => '0');
    signal tx_ready_ts_nsec_meta            : unsigned(31 downto 0) := (others => '0');
    signal tx_ready_ts_nsec_sync            : unsigned(31 downto 0) := (others => '0');

    -- Prevent register optimization/merging for metastability registers
    attribute PRESERVE : boolean;
    attribute PRESERVE of tx_en_i_meta : signal is true;
    attribute PRESERVE of tx_en_i_sync : signal is true;
    attribute PRESERVE of tx_ready_ts_sec_meta : signal is true;
    attribute PRESERVE of tx_ready_ts_nsec_meta : signal is true;


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
        elsif rising_edge(clk) then
            -- Edge detection registers (same clock domain - no CDC needed)
            second_pulse_i_reg <= second_pulse_i;
            send_delay_resp_in_reg <= send_delay_resp_in;

            -- Detect rising edge on TX_EN (from sender, CDC synchronized)
            -- Clear frame_start when sender acknowledges
            if (tx_en_i_sync = '1' and tx_en_i_prev = '0') then
                frame_start_o <= '0';
            end if;

            -- PTPv2 controller logic
            if (is_leader_override_i = '1') then
                is_leader <= '1';
            else
                is_leader <= '0';
            end if;

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

            else
                -- Follower logic - reset state machine when not leader
                leader_state <= s_Idle;
                frame_start_o <= '0';
                send_delay_resp <= '0';
                tx_started <= '0';
            end if;

        end if;
    end process;

end architecture;