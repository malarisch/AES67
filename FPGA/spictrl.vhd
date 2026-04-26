library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use IEEE.MATH_REAL.ALL;
entity spictrl is
    generic (
        FPGAVERSIONMSB : integer := 1;
        FPGAVERSIONLSB : integer := 123;
        TXSTREAMS      : integer := 8;
        RXSTREAMS      : integer := 8;
        TXCHANNELS     : integer := 8;
        RXCHANNELS     : integer := 8;
        BITDEPTH       : integer := 24;
        SAMPLERATE     : integer := 48;
        -- Some SPI masters (e.g. ESP32) split a logical transfer into
        -- <= 64-byte hardware bursts, briefly de-asserting CS between
        -- bursts. For packet registers (0x20 TX, 0x22 RX data) we
        -- therefore keep the transaction open across short CS-high
        -- gaps and only tear it down once CS has been high for longer
        -- than PACKET_CS_GAP_TIMEOUT sys_clk cycles (genuine abort).
        PACKET_CS_GAP_TIMEOUT : integer := 2048
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
        rx_cfg_wr_data_o : OUT std_logic_vector(7 downto 0) := (others => '0');

        rx_packet_length_i : IN UNSIGNED(10 downto 0);
        rx_packet_tog_i     : IN STD_LOGIC;
        rx_packet_ram_read_addr_o : OUT UNSIGNED(10 downto 0);
        rx_packet_ram_read_data_i : IN STD_LOGIC_VECTOR(7 downto 0);

        tx_clk_i : IN STD_LOGIC;
        tx_allow_i : IN STD_LOGIC;
        tx_byte_sent_i : IN STD_LOGIC;
        tx_req_o : OUT STD_LOGIC;
        tx_en_o : OUT STD_LOGIC;
        tx_data_o : OUT STD_ULOGIC_VECTOR(7 downto 0);

        mcu_irq_o : OUT STD_LOGIC;

        rx_channel_meter_sig_i : IN STD_LOGIC_VECTOR(RXCHANNELS - 1 downto 0);
        rx_channel_meter_clip_i : IN STD_LOGIC_VECTOR(RXCHANNELS - 1 downto 0);
        tx_channel_meter_sig_i : IN STD_LOGIC_VECTOR(TXCHANNELS - 1 downto 0);
        tx_channel_meter_clip_i : IN STD_LOGIC_VECTOR(TXCHANNELS - 1 downto 0)
    );
end entity;


architecture rtl of spictrl is

    constant metering_bits             : integer := rx_channel_meter_sig_i'length + rx_channel_meter_clip_i'length +tx_channel_meter_sig_i'length + tx_channel_meter_clip_i'length;
    constant metering_bytes           : integer := natural(ceil(real((RXCHANNELS / 8) * 2 + (TXCHANNELS / 8) * 2)));
    signal spi_data_to_host          : std_logic_vector(7 downto 0);
    signal spi_data_from_host        : std_logic_vector(7 downto 0);
    signal spi_data_to_host_valid    : std_logic;
    signal spi_data_to_host_ready    : std_logic;
    signal spi_data_to_host_ready_z  : std_logic;
    signal spi_data_from_host_valid  : std_logic := '0';

    -- Transaction tracking (CS_N intentionally ignored)
    signal state_transaction_active : std_logic := '0';
    signal state_reading            : std_logic := '0';
    signal state_writing            : std_logic := '0';
    signal address_received         : std_logic := '0';
    signal active_register          : std_logic_vector(6 downto 0) := (others => '0');
    signal transaction_byte_counter : integer range 0 to 1531 := 0;
    signal read_bytes               : integer range 0 to 1531 := 0;
    signal write_bytes              : integer range 0 to 1531 := 0;
    signal tx_packet_ram_addr : integer range 0 to 1531 := 0;
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
    signal cs_rise_pulse : std_logic;

    -- True while a packet-register transaction (0x20 TX, 0x22 RX data)
    -- is in progress. During those, short CS-high gaps between 64-byte
    -- ESP32 bursts must NOT tear the transaction down.
    signal in_packet_burst : std_logic;
    -- Counts sys_clk cycles CS has been continuously high while the
    -- transaction is active. Exceeds PACKET_CS_GAP_TIMEOUT -> genuine
    -- abort, clear the transaction.
    signal cs_gap_counter : integer range 0 to PACKET_CS_GAP_TIMEOUT := 0;
    signal cs_gap_abort   : std_logic;
    -- Effective CS-rise-triggered reset: only fires when we are *not*
    -- in the middle of a packet-register burst.
    signal cs_reset       : std_logic;

    type t_packet_ram is array (0 to 1531) of std_logic_vector(7 downto 0);
	signal tx_packet_ram : t_packet_ram := (others => (others => '0'));
    signal tx_packet_length : UNSIGNED(10 downto 0);
    signal tx_packet_ready : std_logic := '0';
    signal tx_done : std_logic;
    signal packet_tog_z : std_logic := '0';
    signal packet_available : std_logic := '0';
    signal rx_overflow : std_logic := '0';
    signal metering_shadow : std_logic_vector(metering_bits - 1 downto 0);
    signal rx_packet_length_latch : unsigned(10 downto 0);

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

    


    -- Genuine abort detection: if CS stays high for PACKET_CS_GAP_TIMEOUT
    -- sys_clk cycles while we are in a packet burst, treat it as the
    -- master having aborted and tear the transaction down anyway.
    cs_gap_proc : process (sys_clk_i, rst_n_i)
    begin
        if rst_n_i = '0' then
            cs_gap_counter <= 0;
        elsif rising_edge(sys_clk_i) then
            if (state_transaction_active = '1' and
                                 address_received = '1' and
                                 (active_register = "0100010" or
                                  active_register = "0100000")) then
											 in_packet_burst <= '1';
                           else
									in_packet_burst <= '0';
									end if;
            if spi_cs_n_reg = '0' or in_packet_burst = '0' then
                cs_gap_counter <= 0;
            elsif cs_gap_counter < PACKET_CS_GAP_TIMEOUT then
                cs_gap_counter <= cs_gap_counter + 1;
            end if;
        end if;
    end process;

    cs_gap_abort <= '1' when (in_packet_burst = '1' and
                              cs_gap_counter >= PACKET_CS_GAP_TIMEOUT)
                        else '0';

    -- Effective CS-driven reset: cs_rise_pulse triggers the usual
    -- tear-down for all non-packet registers; during packet bursts we
    -- wait for the timeout before giving up.
    cs_reset <= (cs_rise_pulse and not in_packet_burst) or cs_gap_abort;

    transaction_done <= '1' when (state_transaction_active = '1' and address_received = '1' and
                                  ((state_reading = '1' and read_bytes /= 0
                                    and transaction_byte_counter >= read_bytes)
                                   or
                                   (state_writing = '1' and write_bytes /= 0
                                    and transaction_byte_counter >= write_bytes)))
                            else '0';
    metering_shadow <= rx_channel_meter_sig_i & rx_channel_meter_clip_i & tx_channel_meter_sig_i & tx_channel_meter_clip_i;

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
            if transaction_done = '1' or cs_reset = '1' then
                state_transaction_active <= '0';
                state_reading <= '0';
                state_writing <= '0';
                address_received <= '0';
            elsif (state_transaction_active = '0' and spi_data_from_host_valid = '1') then
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
        end if;
    end process;

    -- Payload byte counter. The command byte itself does not increment the
    -- counter. Reset on transaction_done.
    rx_packet_ram_read_addr_o <= TO_UNSIGNED(transaction_byte_counter, rx_packet_ram_read_addr_o'length);
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
            if transaction_done = '1' or cs_reset = '1' then
                transaction_byte_counter <= 0;
            elsif (state_transaction_active = '1' and address_received = '1') then
                if (state_writing = '1' and spi_data_from_host_valid = '1') then
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
            meter_clear_o <= '0';

        elsif rising_edge(sys_clk_i) then
            spi_data_to_host_valid <= '0';
            meter_clear_o <= '0';
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
                        when "0100001" =>
                            if (transaction_byte_counter = 0) then
                                spi_data_to_host <= std_logic_vector(resize(rx_packet_length_latch(10 downto 8), 8));
                            elsif (transaction_byte_counter = 1) then
                                spi_data_to_host <= std_logic_vector(rx_packet_length_latch(7 downto 0));
                            else
                                spi_data_to_host <= (others => '0');
                            end if;
                        when "0100010" =>
                            spi_data_to_host <= rx_packet_ram_read_data_i;
                        when "0110000" =>
                            spi_data_to_host <= metering_shadow((transaction_byte_counter * 8) + 7
                                                                                  downto transaction_byte_counter * 8);
                            if (transaction_byte_counter >= metering_bytes) then
                                meter_clear_o <= '1';
                            end if;
                        when "1010000" => -- 0x50
                            spi_data_to_host <= wallclock_ppb_meas_valid_i & wallclock_locked_i & wallclock_configured_i
                                                & ptp_is_leader_i & ptp_is_follower_i & packet_available & rx_overflow & "0";
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
                    when "0100001"      => read_bytes <= 2; -- 0x21 RX Packet length
                    when "0100010"      => read_bytes <= to_integer(rx_packet_length_latch - 1); -- rx packet 0x22
                    when "0110000"      => read_bytes <= metering_bytes; -- 0x30
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
                    when "0100000"      => write_bytes <= 1500; -- ethernet packet to fpga 0x20
                    when "1000001" => write_bytes <= 4;  -- 0x41 IP
                    when "1010000" => write_bytes <= 1;  -- 0x50 flags
                    when "1010101" => write_bytes <= 7;  -- 0x55 PTP config
                    when "1011000" => write_bytes <= 20; -- 0x58 TX stream
                    when "1011001" => write_bytes <= 18; -- 0x59 RX stream
                    when others    => write_bytes <= 0;
                end case;
                if (active_register = "0100000" and transaction_byte_counter >= 2) then
                    write_bytes <= to_integer(tx_packet_length + 2);
                end if;
            end if;
        end if;
    end process;


    ethernet_rx_packetcaptrue : process (sys_clk_i, rst_n_i) begin
        if (rst_n_i = '0') then
            packet_tog_z <= '0';
            packet_available <= '0';
        elsif rising_edge(sys_clk_i) then
            packet_tog_z <= rx_packet_tog_i;
            if (packet_tog_z /= rx_packet_tog_i) then
                if (packet_available = '1') then
                    rx_overflow <= '1';
                else
                    rx_packet_length_latch <= rx_packet_length_i;
                    packet_available <= '1';
                end if;

            end if;
            if (active_register = "0100010" and transaction_byte_counter = rx_packet_length_latch - 1) then
                packet_available <= '0';
            end if;
            if (active_register = "1010000" and transaction_done = '1') then
                rx_overflow <= '0';

            end if;
        end if;
    end process;
    mcu_irq_o <= not packet_available;

    -- get packet from mcu and write to blockram
    ethernet_packet_capture_process : process (sys_clk_i, rst_n_i)
    begin
        if rst_n_i = '0' then

            tx_packet_ready <= '0';
        elsif rising_edge(sys_clk_i) then
            if (tx_done = '1') then
            tx_packet_ready <= '0';
            end if;
            if (state_transaction_active = '1' and state_writing = '1' and active_register = "0100000") then
                -- write received data to packet ram
                if (spi_data_from_host_valid = '1') then
                    if (transaction_byte_counter = 0) then
                        tx_packet_length(10 downto 8) <= resize(UNSIGNED(spi_data_from_host), 3);
                    elsif (transaction_byte_counter = 1) then
                        tx_packet_length(7 downto 0) <= UNSIGNED(spi_data_from_host);
                    else
                        if (transaction_byte_counter -2 < tx_packet_length) then
                            tx_packet_ram(transaction_byte_counter - 2) <= spi_data_from_host;
                        end if;
                        
                    end if;
                end if;
                if (transaction_byte_counter = tx_packet_length + 2) then
                    tx_packet_ready <= '1';
                        
                end if;
            end if;
        end if;
    end process;
    tx_req_o <= tx_packet_ready;
    packet_ram_port_b : process(tx_clk_i)
	begin
		if rising_edge(tx_clk_i) then
			if (tx_byte_sent_i = '1') then
					tx_data_o <= std_ulogic_vector(tx_packet_ram(tx_packet_ram_addr + 1));
			else
					tx_data_o <= std_ulogic_vector(tx_packet_ram(tx_packet_ram_addr));
			end if;
            tx_done <= '0';
			if tx_packet_ready = '1' and tx_allow_i = '1' then
				tx_en_o <= '1';
				if tx_packet_ram_addr < tx_packet_length -1  then
				
					if tx_byte_sent_i = '1' then
						tx_packet_ram_addr <= tx_packet_ram_addr + 1;
					end if;
				else 
					tx_en_o <= '0';
					tx_done <= '1';
				end if;
				
			else
				tx_en_o <= '0';
				tx_packet_ram_addr <= 0;
			end if;
			
		end if;
	end process packet_ram_port_b;

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
                and address_received = '1' and spi_data_from_host_valid = '1') then
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
            adda_nrst_o                 <= '0';
            ppb_meas_valid_z            <= '0';
        elsif rising_edge(sys_clk_i) then

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
                and address_received = '1' and spi_data_from_host_valid = '1') then

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
