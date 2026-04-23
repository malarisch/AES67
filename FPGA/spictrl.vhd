library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity spictrl is
    generic (
        FPGAVERSIONMSB : integer := 1;
        FPGAVERSIONLSB : integer := 123;
        TXSTREAMS      : integer := 8;
        RXSTREAMS      : integer := 8;
        TXCHANNELS     : integer := 8;
        RXCHANNELS     : integer := 8;
        BITDEPTH       : integer := 24;
        SAMPLERATE     : integer := 48
    );
    port(
        sys_clk_i : IN STD_LOGIC;
        rst_n_i : IN STD_LOGIC;

        -- SPI pins
        spi_clk_i  : IN  STD_LOGIC;
        spi_cs_n_i : IN  STD_LOGIC;
        spi_do_o   : OUT STD_LOGIC;
        spi_di_i   : IN  STD_LOGIC;

        -- Status inputs (reads)
        ptp_path_delay_i           : IN unsigned(31 downto 0);
        ptp_offset_i               : IN unsigned(31 downto 0);
        ptp_gmid_i                 : IN std_logic_vector(63 downto 0);
        ptp_is_follower_i          : IN std_logic;
        ptp_is_leader_i            : IN std_logic;
        wallclock_locked_i         : IN std_logic;
        wallclock_configured_i     : IN std_logic;
        wallclock_ppb_meas_valid_i : IN std_logic;
        wallclock_counter_wc       : IN unsigned(31 downto 0);
        wallclock_counter_pll      : IN unsigned(31 downto 0);
        ethernet_link_up_i         : IN std_logic;
        ethernet_link_speed        : IN std_logic_vector(1 downto 0);

        -- Config outputs (writes, atomic commit at end of transaction)
        mac_address_o              : OUT std_logic_vector(47 downto 0) := (others => '0');
        ip_address_o               : OUT std_logic_vector(31 downto 0) := (others => '0');

        ptp_time_source_o          : OUT std_logic_vector(7 downto 0)  := (others => '0');
        ptp_log_sync_interval_o    : OUT std_logic_vector(7 downto 0)  := (others => '0');
        ptp_log_announce_interval_o: OUT std_logic_vector(7 downto 0)  := (others => '0');
        ptp_priority1_o            : OUT std_logic_vector(7 downto 0)  := (others => '0');
        ptp_priority2_o            : OUT std_logic_vector(7 downto 0)  := (others => '0');
        ptp_clock_class_o          : OUT std_logic_vector(7 downto 0)  := (others => '0');
        ptp_clock_accuracy_o       : OUT std_logic_vector(7 downto 0)  := (others => '0');

        -- Control flag outputs (from register 0x50 write)
        ppb_meter_start_o          : OUT std_logic := '0';
        wallclock_reset_o          : OUT std_logic := '0';
        ptp_reset_o                : OUT std_logic := '0';
        ethernet_reset_o           : OUT std_logic := '0';
        meter_clear_o              : OUT std_logic := '0'; -- 1-cycle pulse
        adda_nrst_o                : OUT std_logic := '0';

        -- Stream config RAM write ports (byte-wise passthrough)
        tx_cfg_wr_en_o   : OUT std_logic := '0';
        tx_cfg_wr_addr_o : OUT std_logic_vector(7 downto 0) := (others => '0');
        tx_cfg_wr_data_o : OUT std_logic_vector(7 downto 0) := (others => '0');
        rx_cfg_wr_en_o   : OUT std_logic := '0';
        rx_cfg_wr_addr_o : OUT std_logic_vector(7 downto 0) := (others => '0');
        rx_cfg_wr_data_o : OUT std_logic_vector(7 downto 0) := (others => '0')
    );
end entity;


architecture rtl of spictrl is

    signal spi_data_to_host          : std_logic_vector(7 downto 0);
    signal spi_data_from_host        : std_logic_vector(7 downto 0);
    signal spi_data_to_host_valid    : std_logic;
    signal spi_data_to_host_valid_reg: std_logic;
    signal spi_data_to_host_ready    : std_logic;
    signal spi_data_to_host_ready_z  : std_logic;
    signal spi_data_from_host_valid  : std_logic := '0';
    signal spi_data_from_host_valid_z: std_logic := '0';
    signal byte_received_pulse       : std_logic; -- one-cycle rising-edge pulse on DOUT_VLD

    -- Transaction tracking (CS_N intentionally ignored)
    signal state_transaction_active : std_logic := '0';
    signal state_reading            : std_logic := '0';
    signal state_writing            : std_logic := '0';
    signal address_received         : std_logic := '0';
    signal active_register          : std_logic_vector(6 downto 0) := (others => '0');
    signal transaction_byte_counter : integer range 0 to 63 := 0;
    signal read_bytes               : integer range 0 to 63 := 0;
    signal write_bytes              : integer range 0 to 63 := 0;
    signal transaction_done         : std_logic;

    -- Shadow registers for atomic writes (committed on transaction_done)
    signal mac_shadow  : std_logic_vector(47 downto 0) := (others => '0');
    signal ip_shadow   : std_logic_vector(31 downto 0) := (others => '0');
    signal ptp_shadow  : std_logic_vector(7*8-1 downto 0) := (others => '0'); -- 0x55, 7 B
    signal flag_shadow : std_logic_vector(7 downto 0) := (others => '0'); -- 0x50

    -- PPB-meter handshake: bit set by host, auto-cleared when measurement valid falls.
    -- Tracks previous pll_meas_valid to catch falling edge.
    signal ppb_meas_valid_z : std_logic := '0';

    -- Stream write: first payload byte is stream_id; subsequent bytes go to
    -- RAM at base = stream_id * 32 + offset. Offset starts at 0x00 for RX and
    -- 0x00 for TX (TX echoes stream_id into RAM[0], then starts payload at
    -- RAM[1], but we write payload byte-wise with an offset that increments
    -- every data byte).
    signal stream_id_latched : unsigned(2 downto 0) := (others => '0');

    signal spi_cs_n_sync : STD_LOGIC;
    signal spi_cs_n_reg : std_logic;
    signal spi_cs_n_reg_z : std_logic := '1';
    -- One-cycle pulse when CS_N transitions low -> high (end of a SPI burst).
    -- Used to flush half-received transactions caused by wire glitches; does
    -- NOT short-circuit transaction_done, so atomic commits still happen.
    signal cs_rise_pulse : std_logic;

begin

    SPI_SLAVE_inst: entity work.SPI_SLAVE
     generic map(
        WORD_SIZE => 8
    )
     port map(
        CLK => sys_clk_i,
        RST => not rst_n_i,
        SCLK => spi_clk_i,
        CS_N => spi_cs_n_i,
        MOSI => spi_di_i,
        MISO => spi_do_o,
        DIN => spi_data_to_host,
        DIN_VLD => spi_data_to_host_valid,
        DIN_RDY => spi_data_to_host_ready,
        DOUT => spi_data_from_host,
        DOUT_VLD => spi_data_from_host_valid
    );
    
    cdc_proc : process (sys_clk_i, rst_n_i)
    begin
        if rst_n_i = '0' then
            spi_cs_n_sync <= '1';
            spi_cs_n_reg  <= '1';
            spi_cs_n_reg_z <= '1';
            cs_rise_pulse <= '0';
        elsif rising_edge(sys_clk_i) then
            spi_cs_n_sync  <= spi_cs_n_i;
            spi_cs_n_reg   <= spi_cs_n_sync;
            spi_cs_n_reg_z <= spi_cs_n_reg;
            cs_rise_pulse <= '0';
            if spi_cs_n_reg = '1' and spi_cs_n_reg_z = '0' then
                cs_rise_pulse <= '1';
            end if;
        end if;
    end process;

    transaction_done <= '1' when (state_transaction_active = '1' and address_received = '1' and
                                  ((state_reading = '1' and read_bytes /= 0
                                    and transaction_byte_counter >= read_bytes)
                                   or
                                   (state_writing = '1' and write_bytes /= 0
                                    and transaction_byte_counter >= write_bytes)))
                            else '0';

    byte_received_pulse <= '1' when (spi_data_from_host_valid = '1'
                                     and spi_data_from_host_valid_z = '0')
                               else '0';

    fsm_controller: process (sys_clk_i, rst_n_i)
    begin
        if rst_n_i = '0' then
            state_transaction_active <= '0';
            state_reading <= '0';
            state_writing <= '0';
            address_received <= '0';
            active_register <= (others => '0');
        elsif rising_edge(sys_clk_i) then
            -- Finish on either the normal end-of-transaction (commit fires
            -- via transaction_done in commit_proc) or on a CS rising edge
            -- that represents an aborted/glitched burst. The CS-rise path
            -- does NOT fire transaction_done, so half-received writes stay
            -- in the shadow registers and never reach the FPGA outputs.
            if transaction_done = '1' or cs_rise_pulse = '1' then
                state_transaction_active <= '0';
                state_reading <= '0';
                state_writing <= '0';
                address_received <= '0';
            elsif (state_transaction_active = '0' and byte_received_pulse = '1') then
                state_transaction_active <= '1';
                address_received <= '1';
                active_register <= spi_data_from_host(6 downto 0);
                if (spi_data_from_host(7) = '1') then
                    state_writing <= '1';
                else
                    state_reading <= '1';
                end if;
            end if;
        end if;
    end process;

    transaction_byte_edge_detector : process (sys_clk_i)
    begin
        if rising_edge(sys_clk_i) then
            spi_data_to_host_ready_z <= spi_data_to_host_ready;
            spi_data_from_host_valid_z <= spi_data_from_host_valid;
        end if;
    end process;

    -- Payload byte counter. The command byte itself does not increment the
    -- counter. Reset on transaction_done.
    transaction_byte_counter_proc : process (sys_clk_i, rst_n_i)
    begin
        if rst_n_i = '0' then
            transaction_byte_counter <= 0;
        elsif rising_edge(sys_clk_i) then
            -- Finish on either the normal end-of-transaction (commit fires
            -- via transaction_done in commit_proc) or on a CS rising edge
            -- that represents an aborted/glitched burst. The CS-rise path
            -- does NOT fire transaction_done, so half-received writes stay
            -- in the shadow registers and never reach the FPGA outputs.
            if transaction_done = '1' or cs_rise_pulse = '1' then
                transaction_byte_counter <= 0;
            elsif (state_transaction_active = '1' and address_received = '1') then
                if (state_writing = '1' and byte_received_pulse = '1') then
                    transaction_byte_counter <= transaction_byte_counter + 1;
                end if;
                if (state_reading = '1' and
                    spi_data_to_host_ready_z = '0' and spi_data_to_host_ready = '1') then
                    transaction_byte_counter <= transaction_byte_counter + 1;
                end if;
            end if;
        end if;
    end process;

    -- =====================================================================
    -- READ path
    -- =====================================================================

    read_register_controller : process (sys_clk_i, rst_n_i)
    begin
        if rst_n_i = '0' then
            spi_data_to_host <= (others => '0');
            spi_data_to_host_valid <= '0';
        elsif rising_edge(sys_clk_i) then
            spi_data_to_host_valid <= '0';
            if (state_transaction_active = '1' and state_reading = '1' and address_received = '1') then
                if (spi_data_to_host_ready_z = '1') then
                spi_data_to_host_valid <= '1';    
                    case active_register is
                        when "0000000" => -- 0x00 FPGA info
                            case transaction_byte_counter is
                                when 0 => spi_data_to_host <= std_logic_vector(to_unsigned(FPGAVERSIONMSB, 8));
                                when 1 => spi_data_to_host <= std_logic_vector(to_unsigned(FPGAVERSIONLSB, 8));
                                when 2 => spi_data_to_host <= std_logic_vector(to_unsigned(TXSTREAMS, 8));
                                when 3 => spi_data_to_host <= std_logic_vector(to_unsigned(RXSTREAMS, 8));
                                when 4 => spi_data_to_host <= std_logic_vector(to_unsigned(TXCHANNELS, 8));
                                when 5 => spi_data_to_host <= std_logic_vector(to_unsigned(RXCHANNELS, 8));
                                when 6 => spi_data_to_host <= std_logic_vector(to_unsigned(BITDEPTH, 8));
                                when 7 => spi_data_to_host <= std_logic_vector(to_unsigned(SAMPLERATE, 8));
                                when others => spi_data_to_host <= x"00";
                            end case;

                        when "1010000" => -- 0x50
                            spi_data_to_host <= wallclock_ppb_meas_valid_i & wallclock_locked_i & wallclock_configured_i
                                                & ptp_is_leader_i & ptp_is_follower_i & "000";
                        when "1010001" => -- 0x51
                            spi_data_to_host <= ethernet_link_up_i & ethernet_link_speed & "00000";
                        when "1010010" => -- 0x52 path delay
                            spi_data_to_host <= std_logic_vector(ptp_path_delay_i((transaction_byte_counter * 8) + 7
                                                                                  downto transaction_byte_counter * 8));
                        when "1010011" => -- 0x53 offset
                            spi_data_to_host <= std_logic_vector(ptp_offset_i((transaction_byte_counter * 8) + 7
                                                                              downto transaction_byte_counter * 8));
                        when "1010100" => -- 0x54 ppb counters
                            if (transaction_byte_counter < 4) then
                                spi_data_to_host <= std_logic_vector(wallclock_counter_pll((transaction_byte_counter * 8) + 7
                                                                                          downto transaction_byte_counter * 8));
                            else
                                spi_data_to_host <= std_logic_vector(wallclock_counter_wc(((transaction_byte_counter - 4) * 8) + 7
                                                                                         downto (transaction_byte_counter - 4) * 8));
                            end if;
                        when "1010101" => -- 0x55 GM id
                            spi_data_to_host <= ptp_gmid_i((transaction_byte_counter * 8) + 7
                                                           downto transaction_byte_counter * 8);
                        when others =>
                            spi_data_to_host <= x"00";
                    end case;
                
                end if;
            end if;
        end if;
    end process;

    read_register_length_controller : process (sys_clk_i, rst_n_i)
    begin
        if rst_n_i = '0' then
            read_bytes <= 0;
        elsif rising_edge(sys_clk_i) then
            if (state_transaction_active = '1' and state_reading = '1') then
                case active_register is
                    when "0000000" => read_bytes <= 8; -- 0x00 FPGA info
                    when "1010000" => read_bytes <= 1; -- 0x50 clocking status
                    when "1010001" => read_bytes <= 1; -- 0x51 eth status
                    when "1010010" => read_bytes <= 4; -- 0x52 path delay
                    when "1010011" => read_bytes <= 4; -- 0x53 offset
                    when "1010100" => read_bytes <= 8; -- 0x54 ppb counters
                    when "1010101" => read_bytes <= 8; -- 0x55 gm id
                    when others => read_bytes <= 0;
                end case;
            end if;
        end if;
    end process;

    -- =====================================================================
    -- WRITE path
    -- =====================================================================

    write_register_length_controller : process (sys_clk_i, rst_n_i)
    begin
        if rst_n_i = '0' then
            write_bytes <= 0;
        elsif rising_edge(sys_clk_i) then
            if (state_transaction_active = '1' and state_writing = '1') then
                case active_register is
                    when "1000000" => write_bytes <= 6;  -- 0x40 MAC
                    when "1000001" => write_bytes <= 4;  -- 0x41 IP
                    when "1010000" => write_bytes <= 1;  -- 0x50 flags
                    when "1010101" => write_bytes <= 7;  -- 0x55 PTP config
                    when "1011000" => write_bytes <= 20; -- 0x58 TX stream
                    when "1011001" => write_bytes <= 18; -- 0x59 RX stream
                    when others    => write_bytes <= 0;
                end case;
            end if;
        end if;
    end process;

    -- Shadow register capture. Each received payload byte goes into the slot
    -- addressed by transaction_byte_counter (0-based; command byte is not
    -- counted). Stream writes do not use shadows — they pass through.
    shadow_capture : process (sys_clk_i, rst_n_i)
        variable byte_idx : integer range 0 to 63;
    begin
        if rst_n_i = '0' then
            mac_shadow  <= (others => '0');
            ip_shadow   <= (others => '0');
            ptp_shadow  <= (others => '0');
            flag_shadow <= (others => '0');
            stream_id_latched <= (others => '0');
        elsif rising_edge(sys_clk_i) then
            if (state_transaction_active = '1' and state_writing = '1'
                and address_received = '1' and byte_received_pulse = '1') then
                byte_idx := transaction_byte_counter; -- 0 = first payload byte
                case active_register is
                    when "1000000" => -- 0x40 MAC: byte 0 = MSB (bits 47..40)
                        if byte_idx < 6 then
                            mac_shadow((5 - byte_idx) * 8 + 7 downto (5 - byte_idx) * 8) <= spi_data_from_host;
                        end if;
                    when "1000001" => -- 0x41 IP: byte 0 = MSB
                        if byte_idx < 4 then
                            ip_shadow((3 - byte_idx) * 8 + 7 downto (3 - byte_idx) * 8) <= spi_data_from_host;
                        end if;
                    when "1010000" => -- 0x50 flags
                        if byte_idx = 0 then
                            flag_shadow <= spi_data_from_host;
                        end if;
                    when "1010101" => -- 0x55 PTP config, 7 bytes, byte 0 = offset 0
                        if byte_idx < 7 then
                            ptp_shadow((6 - byte_idx) * 8 + 7 downto (6 - byte_idx) * 8) <= spi_data_from_host;
                        end if;
                    when "1011000" | "1011001" => -- 0x58/0x59 stream: byte 0 = stream_id
                        if byte_idx = 0 then
                            stream_id_latched <= unsigned(spi_data_from_host(2 downto 0));
                        end if;
                    when others => null;
                end case;
            end if;
        end if;
    end process;

    -- Atomic commit: on the final byte of a write transaction, push shadow
    -- contents to the output ports. PPB-meter bit uses a handshake: host sets
    -- the bit, it stays high until the measurement valid line falls.
    commit_proc : process (sys_clk_i, rst_n_i)
    begin
        if rst_n_i = '0' then
            mac_address_o <= (others => '0');
            ip_address_o  <= (others => '0');
            ptp_time_source_o           <= (others => '0');
            ptp_log_sync_interval_o     <= (others => '0');
            ptp_log_announce_interval_o <= (others => '0');
            ptp_priority1_o             <= (others => '0');
            ptp_priority2_o             <= (others => '0');
            ptp_clock_class_o           <= (others => '0');
            ptp_clock_accuracy_o        <= (others => '0');
            ppb_meter_start_o           <= '0';
            wallclock_reset_o           <= '0';
            ptp_reset_o                 <= '0';
            ethernet_reset_o            <= '0';
            meter_clear_o               <= '0';
            adda_nrst_o                 <= '0';
            ppb_meas_valid_z            <= '0';
        elsif rising_edge(sys_clk_i) then
            -- Meter clear is a single-cycle pulse
            meter_clear_o <= '0';

            -- PPB-meter auto-clear: drop ppb_meter_start_o when valid falls
            ppb_meas_valid_z <= wallclock_ppb_meas_valid_i;
            if (ppb_meas_valid_z = '1' and wallclock_ppb_meas_valid_i = '0') then
                ppb_meter_start_o <= '0';
            end if;

            if transaction_done = '1' and state_writing = '1' then
                case active_register is
                    when "1000000" => -- 0x40 MAC
                        mac_address_o <= mac_shadow;
                    when "1000001" => -- 0x41 IP
                        ip_address_o <= ip_shadow;
                    when "1010000" => -- 0x50 flags
                        if flag_shadow(0) = '1' then
                            ppb_meter_start_o <= '1'; -- only set, cleared by valid-falling
                        end if;
                        wallclock_reset_o <= flag_shadow(1);
                        ptp_reset_o       <= flag_shadow(2);
                        ethernet_reset_o  <= flag_shadow(3);
                        if flag_shadow(4) = '1' then
                            meter_clear_o <= '1';
                        end if;
                        adda_nrst_o       <= flag_shadow(5);
                    when "1010101" => -- 0x55 PTP: order per map
                        ptp_time_source_o           <= ptp_shadow(55 downto 48); -- byte 0
                        ptp_log_sync_interval_o     <= ptp_shadow(47 downto 40); -- byte 1
                        ptp_log_announce_interval_o <= ptp_shadow(39 downto 32); -- byte 2
                        ptp_priority1_o             <= ptp_shadow(31 downto 24); -- byte 3
                        ptp_priority2_o             <= ptp_shadow(23 downto 16); -- byte 4
                        ptp_clock_class_o           <= ptp_shadow(15 downto 8);  -- byte 5
                        ptp_clock_accuracy_o        <= ptp_shadow(7 downto 0);   -- byte 6
                    when others => null;
                end case;
            end if;
        end if;
    end process;

    -- =====================================================================
    -- Stream config RAM passthrough (byte-wise, during write transaction).
    -- Byte 0 (stream_id) is captured into stream_id_latched. Subsequent
    -- bytes are written to RAM at base + offset, where:
    --   TX (0x58): offset 0 = RAM[0]=stream_id echo? No — we follow the map:
    --              byte 0 echoes to RAM offset 0x00, bytes 1..19 -> RAM 0x01..0x13
    --   RX (0x59): stream_id is NOT stored; bytes 1..17 -> RAM 0x00..0x10
    -- =====================================================================

    stream_write_proc : process (sys_clk_i, rst_n_i)
        variable byte_idx : integer range 0 to 63;
        variable base_tx  : unsigned(7 downto 0);
        variable base_rx  : unsigned(7 downto 0);
    begin
        if rst_n_i = '0' then
            tx_cfg_wr_en_o   <= '0';
            tx_cfg_wr_addr_o <= (others => '0');
            tx_cfg_wr_data_o <= (others => '0');
            rx_cfg_wr_en_o   <= '0';
            rx_cfg_wr_addr_o <= (others => '0');
            rx_cfg_wr_data_o <= (others => '0');
        elsif rising_edge(sys_clk_i) then
            tx_cfg_wr_en_o <= '0';
            rx_cfg_wr_en_o <= '0';

            if (state_transaction_active = '1' and state_writing = '1'
                and address_received = '1' and byte_received_pulse = '1') then

                byte_idx := transaction_byte_counter;

                if active_register = "1011000" then -- 0x58 TX
                    -- Base = stream_id * 32. For byte_idx = 0 stream_id is in
                    -- spi_data_from_host itself (not yet latched); write it
                    -- to RAM offset 0. For byte_idx >= 1 use latched value.
                    if byte_idx = 0 then
                        base_tx := resize(unsigned(spi_data_from_host(2 downto 0)) & "00000", 8);
                        tx_cfg_wr_addr_o <= std_logic_vector(base_tx);
                        tx_cfg_wr_data_o <= spi_data_from_host;
                        tx_cfg_wr_en_o   <= '1';
                    elsif byte_idx <= 19 then
                        base_tx := resize(stream_id_latched & "00000", 8);
                        tx_cfg_wr_addr_o <= std_logic_vector(base_tx + to_unsigned(byte_idx, 8));
                        tx_cfg_wr_data_o <= spi_data_from_host;
                        tx_cfg_wr_en_o   <= '1';
                    end if;

                elsif active_register = "1011001" then -- 0x59 RX
                    -- stream_id (byte 0) is not stored; payload bytes 1..17
                    -- go to RAM offsets 0..16.
                    if byte_idx >= 1 and byte_idx <= 17 then
                        base_rx := resize(stream_id_latched & "00000", 8);
                        rx_cfg_wr_addr_o <= std_logic_vector(base_rx + to_unsigned(byte_idx - 1, 8));
                        rx_cfg_wr_data_o <= spi_data_from_host;
                        rx_cfg_wr_en_o   <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process;

end architecture;
