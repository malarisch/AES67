library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity spictrl is
    generic (
        FPGAVERSIONMSB : integer := 1;
        FPGAVERSIONLSB : integer := 123;
        TXSTREAMS      : integer := 8;
        RXSTREAMS       : INTEGER := 8;
        TXCHANNELS      : INTEGER := 8;
        RXCHANNELS      : INTEGER := 8;
        BITDEPTH        : INTEGER := 24;
        SAMPLERATE      : INTEGER := 48
    );
    port(
        sys_clk_i : IN STD_LOGIC;
        rst_n_i : IN STD_LOGIC;

        spi_clk_i : IN STD_LOGIC;
        spi_cs_n_i : IN STD_LOGIC;
        spi_do_o : OUT STD_LOGIC;
        spi_di_i : IN STD_LOGIC;

        ptp_path_delay_i : IN unsigned(31 downto 0);
        ptp_offset_i : IN unsigned(31 downto 0);
        ptp_gmid_i : IN STD_LOGIC_VECTOR(63 downto 0);
        ptp_is_follower_i : IN std_logic;
        ptp_is_leader_i : IN std_logic;

        wallclock_locked_i : IN std_logic;
        wallclock_configured_i : IN std_logic;
        wallclock_ppb_meas_valid_i : IN STD_LOGIC;
        wallclock_counter_wc : IN UNSIGNED(31 downto 0);
        wallclock_counter_pll : IN UNSIGNED(31 downto 0);



        ethernet_link_up_i : IN std_logic;
        ethernet_link_speed : in STD_LOGIC_VECTOR(1 downto 0)
    );
end entity;


architecture rtl of spictrl is

    SIGNAL spi_data_to_host : STD_LOGIC_VECTOR(7 downto 0); -- data for transmission
    SIGNAL spi_data_from_host : STD_LOGIC_VECTOR(7 downto 0); -- data from transmission
    SIGNAL spi_data_to_host_valid : STD_LOGIC; -- asset when data in is valid
    SIGNAL spi_data_to_host_ready : STD_LOGIC; -- gets asserted once new data can be transmitted
    SIGNAL spi_data_to_host_ready_z : STD_LOGIC; -- gets asserted once new data can be transmitted
    signal spi_data_from_host_valid : STD_LOGIC := '0'; -- received data is valid
    signal spi_data_from_host_valid_z : STD_LOGIC := '0'; -- received data is valid

    -- Transaction tracking. CS_N is NOT used: some masters (e.g. ESP32 DMA)
    -- deassert CS mid-transaction every few bytes. End-of-transaction is
    -- detected solely by matching transaction_byte_counter against read_bytes.
    signal state_transaction_active : std_logic := '0';
    signal state_reading : std_logic := '0';
    signal state_writing : std_logic := '0';
    signal address_received : std_logic := '0'; -- '1' after command byte is latched
    signal active_register : std_logic_vector(6 downto 0) := (others => '0');
    signal transaction_byte_counter : integer range 0 to 63 := 0;
    signal read_bytes : integer range 0 to 63 := 0;
    signal transaction_done : std_logic; -- '1' when counter reached read_bytes during read

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

    -- Transaction end is signalled purely by the byte-count matching the
    -- register's declared read length. CS_N is intentionally ignored.
    transaction_done <= '1' when (state_transaction_active = '1'
                                  and state_reading = '1'
                                  and address_received = '1'
                                  and read_bytes /= 0
                                  and transaction_byte_counter >= read_bytes)
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
            -- End of transaction: full payload has been handed to the master.
            -- Return to IDLE so that the next MOSI byte is interpreted as a
            -- new command/address (back-to-back transfers without CS toggle).
            if transaction_done = '1' then
                state_transaction_active <= '0';
                state_reading <= '0';
                state_writing <= '0';
                address_received <= '0';
            elsif (state_transaction_active = '0' and spi_data_from_host_valid = '1') then
                -- new transaction: first byte is the command/address
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
        if (rising_edge(sys_clk_i)) then
            spi_data_to_host_ready_z <= spi_data_to_host_ready;
            spi_data_from_host_valid_z <= spi_data_from_host_valid;
        end if;
    end process;

    -- Counts payload bytes only. Reset when the transaction completes
    -- (byte count reached the register's declared length).
    -- The command byte itself does not increment the counter.
    transaction_byte_counter_proc : process (sys_clk_i, rst_n_i)
    begin
        if rst_n_i = '0' then
            transaction_byte_counter <= 0;
        elsif rising_edge(sys_clk_i) then
            if transaction_done = '1' then
                transaction_byte_counter <= 0;
            elsif (state_transaction_active = '1' and address_received = '1') then
                -- Write path: count each received payload byte
                if (state_writing = '1' and
                    spi_data_from_host_valid_z = '0' and spi_data_from_host_valid = '1') then
                    transaction_byte_counter <= transaction_byte_counter + 1;
                end if;
                -- Read path: count each byte successfully handed to the shifter
                if (state_reading = '1' and
                    spi_data_to_host_ready_z = '0' and spi_data_to_host_ready = '1') then
                    transaction_byte_counter <= transaction_byte_counter + 1;
                end if;
            end if;
        end if;
    end process;

    read_register_controller : process (sys_clk_i, rst_n_i)
    begin
        if rst_n_i = '0' then
            spi_data_to_host <= (others => '0');
            spi_data_to_host_valid <= '0';
        elsif rising_edge(sys_clk_i) then
            spi_data_to_host_valid <= '0';
            if (state_transaction_active = '1' and state_reading = '1' and address_received = '1') then
                if (spi_data_to_host_ready = '1') then
                    spi_data_to_host_valid <= '1';
                    case active_register is
                        when "0000000" => -- 0x00
                            case transaction_byte_counter is
                                when 0 =>
                                    spi_data_to_host <= STD_LOGIC_VECTOR(TO_UNSIGNED(FPGAVERSIONMSB, 8));
                                when 1 =>
                                    spi_data_to_host <= STD_LOGIC_VECTOR(TO_UNSIGNED(FPGAVERSIONLSB, 8));
                                when 2 =>
                                    spi_data_to_host <= STD_LOGIC_VECTOR(TO_UNSIGNED(TXSTREAMS, 8));
                                when 3 =>
                                    spi_data_to_host <= STD_LOGIC_VECTOR(TO_UNSIGNED(RXSTREAMS, 8));
                                when 4 =>
                                    spi_data_to_host <= STD_LOGIC_VECTOR(TO_UNSIGNED(TXCHANNELS, 8));
                                when 5 =>
                                    spi_data_to_host <= STD_LOGIC_VECTOR(TO_UNSIGNED(RXCHANNELS, 8));
                                when 6 =>
                                    spi_data_to_host <= STD_LOGIC_VECTOR(TO_UNSIGNED(BITDEPTH, 8));
                                when 7 =>
                                    spi_data_to_host <= STD_LOGIC_VECTOR(TO_UNSIGNED(SAMPLERATE, 8));
                                when others =>
                                    spi_data_to_host <= x"00";
                            end case;

                        when "1010000" => -- 0x50
                            spi_data_to_host <= wallclock_ppb_meas_valid_i & wallclock_locked_i & wallclock_configured_i
                                & ptp_is_leader_i & ptp_is_follower_i & "000";
                        when "1010001" => -- 0x51
                            spi_data_to_host <= ethernet_link_up_i & ethernet_link_speed & "00000";
                        when "1010010" => -- 0x52
                            spi_data_to_host <= STD_LOGIC_VECTOR(ptp_path_delay_i((transaction_byte_counter * 8) + 7 downto transaction_byte_counter * 8));
                        when "1010011" => -- 0x53
                            spi_data_to_host <= STD_LOGIC_VECTOR(ptp_offset_i((transaction_byte_counter * 8) + 7 downto transaction_byte_counter * 8));
                        when "1010100" => -- 0x54: ppb counters (pll then wc)
                            if (transaction_byte_counter < 4) then
                                spi_data_to_host <= STD_LOGIC_VECTOR(wallclock_counter_pll((transaction_byte_counter * 8) + 7 downto transaction_byte_counter * 8));
                            else
                                spi_data_to_host <= STD_LOGIC_VECTOR(wallclock_counter_wc(((transaction_byte_counter - 4) * 8) + 7 downto (transaction_byte_counter - 4) * 8));
                            end if;
                        when "1010101" => -- 0x55: current gm id
                            spi_data_to_host <= ptp_gmid_i((transaction_byte_counter * 8) + 7 downto transaction_byte_counter * 8);
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
                        when "0000000" => -- 0x00
                            read_bytes <= 8;

                        when "1010000" => -- 0x50
                            read_bytes <= 1;
                        when "1010001" => -- 0x51
                            read_bytes <= 1;
                        when "1010010" => -- 0x52
                            read_bytes <= 4; -- path delay
                        when "1010011" => -- 0x53
                            read_bytes <= 4; -- leader offset
                        when "1010100" => -- 0x54
                            read_bytes <= 8; -- ppb counters
                        when "1010101" => -- 0x55
                            read_bytes <= 8; -- current gm id

                        when others =>
                            read_bytes <= 0;
                    end case;

            end if;
        end if;
    end process;


end architecture;
