library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity tx_router is
    generic
    (
        samples_per_channel_depth : integer := 48; -- number of samples per channel to buffer
        global_channel_count : integer := 16; -- number of channels to buffer
        bytes_per_sample : integer := 3 -- number of bytes per sample (e.g., 3 for 24-bit audio)
    );
    port
    (
        sys_clk_i                       : in std_logic;
        reset_n                         : in std_logic;
        sample_buffer_wr_ptr_i          : in std_logic_vector(15 downto 0);

        ip_addr_o                       : out std_logic_vector(31 downto 0) := (others => '0');
        channel_count_o                 : out std_logic_vector(7 downto 0) := (others => '0');
        samples_per_packet_per_channel_o : out std_logic_vector(7 downto 0) := (others => '0');
        sample_buffer_tx_start_addr_o   : out std_logic_vector(15 downto 0) := (others => '0');

        packet_time_i                   : in std_logic_vector(31 downto 0);
        packet_time_o                   : out std_logic_vector(31 downto 0) := (others => '0');

        -- max 8 channels per stream
        ch_ids_o                        : out std_logic_vector(63 downto 0) := (others => '0');

        config_wr_addr_i                : in std_logic_vector(7 downto 0);
        config_wr_data_i                : in std_logic_vector(7 downto 0);
        config_wr_en_i                  : in std_logic;

        streams_configured_i            : in std_logic_vector(7 downto 0); -- number of configured streams (unsigned)

        sample_ready_i                  : in std_logic; -- comes from sys clk domain

        audio_packet_tx_start_o         : out std_logic := '0';
        tx_en_i                         : in std_logic
    );
end entity;

architecture Behavioral of tx_router is
    -- config ram layout per stream (16 bytes each, base = stream_index * 16):
    -- 0x00       stream_id (0-7)
    -- 0x01-0x04  IP address (4 bytes)
    -- 0x05       channel count
    -- 0x06       samples per packet per channel
    -- 0x07-0x0E  channel ids (max 8)
    -- 0x0F       reserved
    type t_config_ram is array (0 to 255) of std_logic_vector(7 downto 0);
    signal config_ram : t_config_ram := (others => (others => '0'));

    type t_sample_count is array (0 to 7) of unsigned(7 downto 0);
    signal sample_count : t_sample_count := (others => (others => '0'));

    -- TX ready FIFOs: parallel FIFOs for stream index, write pointer, and packet time
    constant FIFO_DEPTH : integer := 8;
    type t_stream_fifo is array (0 to FIFO_DEPTH - 1) of unsigned(2 downto 0);
    type t_wrptr_fifo is array (0 to FIFO_DEPTH - 1) of std_logic_vector(15 downto 0);
    type t_time_fifo is array (0 to FIFO_DEPTH - 1) of std_logic_vector(31 downto 0);
    signal fifo_stream  : t_stream_fifo := (others => (others => '0'));
    signal fifo_wrptr   : t_wrptr_fifo := (others => (others => '0'));
    signal fifo_time    : t_time_fifo := (others => (others => '0'));
    signal fifo_wr_ptr  : unsigned(2 downto 0) := (others => '0');
    signal fifo_rd_ptr  : unsigned(2 downto 0) := (others => '0');

    signal sample_ready_sync : std_logic := '0';

    type t_tx_state is (IDLE, LOAD_OUTPUTS, ASSERT_START, WAIT_TX_EN_HIGH, WAIT_TX_EN_LOW);
    signal tx_state : t_tx_state := IDLE;

    signal current_stream    : unsigned(2 downto 0) := (others => '0');
    signal current_wr_ptr    : std_logic_vector(15 downto 0) := (others => '0');
    signal current_time      : std_logic_vector(31 downto 0) := (others => '0');

    -- CDC synchronizer for tx_en_i (crosses clock domain)
    signal tx_en_meta        : std_logic := '0';
    signal tx_en_sync        : std_logic := '0';
    attribute PRESERVE : boolean;
    attribute PRESERVE of tx_en_meta : signal is true;
    attribute PRESERVE of tx_en_sync : signal is true;

begin

    -- Sample counting + FIFO write process
    process(sys_clk_i, reset_n)
        variable base : integer;
        variable num_streams : integer;
    begin
        if reset_n = '0' then
            config_ram <= (others => (others => '0'));
            sample_count <= (others => (others => '0'));
            fifo_stream <= (others => (others => '0'));
            fifo_wrptr <= (others => (others => '0'));
            fifo_time <= (others => (others => '0'));
            fifo_wr_ptr <= (others => '0');
            sample_ready_sync <= '0';
        elsif rising_edge(sys_clk_i) then
            sample_ready_sync <= sample_ready_i;

            -- Config RAM write
            if config_wr_en_i = '1' then
                config_ram(to_integer(unsigned(config_wr_addr_i))) <= config_wr_data_i;
            end if;

            -- Rising edge of sample_ready
            if sample_ready_i = '1' and sample_ready_sync = '0' then
                num_streams := to_integer(unsigned(streams_configured_i));
                for i in 0 to 7 loop
                    if i < num_streams then
                        base := i * 16;
                        if unsigned(config_ram(base + 6)) /= 0 then
                            if sample_count(i) >= unsigned(config_ram(base + 6)) - 1 then
                                sample_count(i) <= (others => '0');
                                -- Enqueue into parallel FIFOs with snapshot of current state
                                fifo_stream(to_integer(fifo_wr_ptr)) <= to_unsigned(i, 3);
                                fifo_wrptr(to_integer(fifo_wr_ptr)) <= sample_buffer_wr_ptr_i;
                                fifo_time(to_integer(fifo_wr_ptr)) <= packet_time_i;
                                fifo_wr_ptr <= fifo_wr_ptr + 1;
                            else
                                sample_count(i) <= sample_count(i) + 1;
                            end if;
                        end if;
                    end if;
                end loop;
            end if;

        end if;
    end process;

    -- CDC synchronizer for tx_en_i
    process(sys_clk_i, reset_n)
    begin
        if reset_n = '0' then
            tx_en_meta <= '0';
            tx_en_sync <= '0';
        elsif rising_edge(sys_clk_i) then
            tx_en_meta <= tx_en_i;
            tx_en_sync <= tx_en_meta;
        end if;
    end process;

    -- TX state machine + FIFO read process
    process(sys_clk_i, reset_n)
        variable base : integer;
    begin
        if reset_n = '0' then
            fifo_rd_ptr <= (others => '0');
            tx_state <= IDLE;
            current_stream <= (others => '0');
            current_wr_ptr <= (others => '0');
            current_time <= (others => '0');
            audio_packet_tx_start_o <= '0';
            ip_addr_o <= (others => '0');
            channel_count_o <= (others => '0');
            samples_per_packet_per_channel_o <= (others => '0');
            sample_buffer_tx_start_addr_o <= (others => '0');
            packet_time_o <= (others => '0');
            ch_ids_o <= (others => '0');
        elsif rising_edge(sys_clk_i) then

            case tx_state is
                when IDLE =>
                    if fifo_rd_ptr /= fifo_wr_ptr then
                        current_stream <= fifo_stream(to_integer(fifo_rd_ptr));
                        current_wr_ptr <= fifo_wrptr(to_integer(fifo_rd_ptr));
                        current_time <= fifo_time(to_integer(fifo_rd_ptr));
                        fifo_rd_ptr <= fifo_rd_ptr + 1;
                        tx_state <= LOAD_OUTPUTS;
                    end if;

                when LOAD_OUTPUTS =>
                    base := to_integer(current_stream) * 16;
                    ip_addr_o <= config_ram(base + 1) & config_ram(base + 2) & config_ram(base + 3) & config_ram(base + 4);
                    channel_count_o <= config_ram(base + 5);
                    samples_per_packet_per_channel_o <= config_ram(base + 6);
                    ch_ids_o <= config_ram(base + 7) & config_ram(base + 8) & config_ram(base + 9) & config_ram(base + 10)
                              & config_ram(base + 11) & config_ram(base + 12) & config_ram(base + 13) & config_ram(base + 14);
                    sample_buffer_tx_start_addr_o <= current_wr_ptr;
                    packet_time_o <= current_time;
                    tx_state <= ASSERT_START;

                when ASSERT_START =>
                    audio_packet_tx_start_o <= '1';
                    tx_state <= WAIT_TX_EN_HIGH;

                when WAIT_TX_EN_HIGH =>
                    if tx_en_sync = '1' then
                        audio_packet_tx_start_o <= '0';
                        tx_state <= WAIT_TX_EN_LOW;
                    end if;

                when WAIT_TX_EN_LOW =>
                    if tx_en_sync = '0' then
                        tx_state <= IDLE;
                    end if;

            end case;
        end if;
    end process;

end Behavioral;
