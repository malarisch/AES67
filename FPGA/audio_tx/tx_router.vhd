library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity tx_router is
    generic
    (
        samples_per_channel_depth : integer := 48; -- number of samples per channel to buffer
        global_channel_count : integer := 16; -- number of channels to buffer
        bytes_per_sample : integer := 3; -- number of bytes per sample (e.g., 3 for 24-bit audio)
		  max_streams : integer := 8
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
        
        -- SSRC for RTP header
        ssrc_o                          : out std_logic_vector(31 downto 0) := (others => '0');

        config_wr_addr_i                : in std_logic_vector(7 downto 0);
        config_wr_data_i                : in std_logic_vector(7 downto 0);
        config_wr_en_i                  : in std_logic;



        sample_ready_i                  : in std_logic; -- comes from sys clk domain

        audio_packet_tx_start_o         : out std_logic := '0';
        tx_en_i                         : in std_logic;
        config_wr_clk_i                  : in std_logic
    );
end entity;

architecture Behavioral of tx_router is
    -- config ram layout per stream (20 bytes each, base = stream_index * 32):
    -- 0x00       stream_id (0-7)
    -- 0x01-0x04  IP address (4 bytes)
    -- 0x05       channel count
    -- 0x06       samples per packet per channel
    -- 0x07-0x0E  channel ids (max 8)
    -- 0x0F       reserved
    -- 0x10-0x13  SSRC (4 bytes, big-endian)
    
    -- Block RAM for configuration storage (True Dual Port)
    -- Port A: Write from config_wr_clk_i domain
    -- Port B: Read from sys_clk_i domain (registered output for BRAM inference)
    type t_config_ram is array (0 to 255) of std_logic_vector(7 downto 0);
    signal config_ram : t_config_ram := (others => (others => '0'));
    
    -- Synthesis attributes for Block RAM inference (Intel/Altera)
    attribute ramstyle : string;
    attribute ramstyle of config_ram : signal is "M10K";
    
    -- Registered read address and data for Block RAM (Port B - sys_clk domain)
    signal ram_rd_addr   : unsigned(7 downto 0) := (others => '0');
    signal ram_rd_data   : std_logic_vector(7 downto 0) := (others => '0');
    
    -- Shadow registers for samples_per_packet (offset 0x06 in each stream config)
    -- These are updated on config write and used by sample counting logic
    -- Avoids multi-port RAM access from sample counting process
    type t_samples_per_packet_shadow is array (0 to 7) of std_logic_vector(7 downto 0);
    signal samples_per_packet_shadow : t_samples_per_packet_shadow := (others => (others => '0'));
    
    -- Pre-computed threshold (samples_per_packet - 1) to reduce critical path
    -- Computed in config_wr_clk domain and synchronized
    type t_threshold_shadow is array (0 to 7) of unsigned(7 downto 0);
    signal threshold_shadow : t_threshold_shadow := (others => (others => '0'));
    signal threshold_sync   : t_threshold_shadow := (others => (others => '0'));
    
    -- CDC synchronizer for shadow registers (config_wr_clk -> sys_clk)
    signal samples_per_packet_sync : t_samples_per_packet_shadow := (others => (others => '0'));

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

    -- Extended state machine with sequential RAM read states
    type t_tx_state is (
        IDLE, 
        LOAD_IP1, LOAD_IP2, LOAD_IP3, LOAD_IP4,
        LOAD_CH_COUNT, LOAD_SAMPLES_PER_PKT,
        LOAD_CH0, LOAD_CH1, LOAD_CH2, LOAD_CH3,
        LOAD_CH4, LOAD_CH5, LOAD_CH6, LOAD_CH7,
        LOAD_SSRC1, LOAD_SSRC2, LOAD_SSRC3, LOAD_SSRC4,
        ASSERT_START, WAIT_TX_EN_HIGH, WAIT_TX_EN_LOW
    );
    signal tx_state : t_tx_state := IDLE;

    signal current_stream    : unsigned(2 downto 0) := (others => '0');
    signal current_wr_ptr    : std_logic_vector(15 downto 0) := (others => '0');
    signal current_time      : std_logic_vector(31 downto 0) := (others => '0');
    
    -- Latched configuration outputs (built up sequentially from RAM reads)
    signal ip_addr_latch     : std_logic_vector(31 downto 0) := (others => '0');
    signal ch_ids_latch      : std_logic_vector(63 downto 0) := (others => '0');
    signal ssrc_latch        : std_logic_vector(31 downto 0) := (others => '0');

    -- CDC synchronizer for tx_en_i (crosses clock domain)
    signal tx_en_meta        : std_logic := '0';
    signal tx_en_sync        : std_logic := '0';
    attribute PRESERVE : boolean;
    attribute PRESERVE of tx_en_meta : signal is true;
    attribute PRESERVE of tx_en_sync : signal is true;

begin

    -- ==========================================================================
    -- Config RAM Write Process (Port A - config_wr_clk_i domain)
    -- Also updates shadow registers for samples_per_packet
    -- ==========================================================================
    process (config_wr_clk_i)
        variable stream_idx : integer;
        variable offset     : integer;
    begin
        if rising_edge(config_wr_clk_i) then
            if config_wr_en_i = '1' then
                config_ram(to_integer(unsigned(config_wr_addr_i))) <= config_wr_data_i;
                
                -- Update shadow register when samples_per_packet (offset 0x06) is written
                -- Base address = stream_id * 32, so stream_idx = addr[7:5], offset = addr[4:0]
                stream_idx := to_integer(unsigned(config_wr_addr_i(7 downto 5)));
                offset := to_integer(unsigned(config_wr_addr_i(4 downto 0)));
                if offset = 6 and stream_idx < 8 then
                    samples_per_packet_shadow(stream_idx) <= config_wr_data_i;
                    -- Pre-compute threshold (spp - 1) to reduce critical path in sample counting
                    -- This subtraction is done at config time, not every sample
                    threshold_shadow(stream_idx) <= unsigned(config_wr_data_i) - 1;
                end if;
            end if;
        end if;
    end process;
    
    -- ==========================================================================
    -- Config RAM Read Process (Port B - sys_clk_i domain)
    -- Registered read output for proper Block RAM inference
    -- ==========================================================================
    process (sys_clk_i)
    begin
        if rising_edge(sys_clk_i) then
            ram_rd_data <= config_ram(to_integer(ram_rd_addr));
        end if;
    end process;
    
    -- ==========================================================================
    -- CDC Synchronizer for shadow registers (config_wr_clk -> sys_clk)
    -- Simple register stage - values change slowly (config writes)
    -- ==========================================================================
    process (sys_clk_i, reset_n)
    begin
        if reset_n = '0' then
            samples_per_packet_sync <= (others => (others => '0'));
            threshold_sync <= (others => (others => '0'));
        elsif rising_edge(sys_clk_i) then
            samples_per_packet_sync <= samples_per_packet_shadow;
            threshold_sync <= threshold_shadow;
        end if;
    end process;

    -- ==========================================================================
    -- Sample counting + FIFO write process
    -- Uses shadow registers instead of direct RAM access
    -- ==========================================================================
    process(sys_clk_i, reset_n)
        variable num_streams : integer;
        variable spp : unsigned(7 downto 0);
    begin
        if reset_n = '0' then
            sample_count <= (others => (others => '0'));
            fifo_stream <= (others => (others => '0'));
            fifo_wrptr <= (others => (others => '0'));
            fifo_time <= (others => (others => '0'));
            fifo_wr_ptr <= (others => '0');
            sample_ready_sync <= '0';
        elsif rising_edge(sys_clk_i) then
            sample_ready_sync <= sample_ready_i;

            -- Rising edge of sample_ready
            if sample_ready_i = '1' and sample_ready_sync = '0' then
                for i in 0 to 7 loop
                    if i < max_streams then
                        -- Use pre-computed threshold (spp-1) to reduce critical path
                        -- The subtraction was done at config time, not here
                        spp := unsigned(samples_per_packet_sync(i));
                        if spp /= 0 then
                            if sample_count(i) >= threshold_sync(i) then
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

    -- ==========================================================================
    -- CDC synchronizer for tx_en_i
    -- ==========================================================================
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

    -- ==========================================================================
    -- TX state machine + FIFO read process
    -- Sequential RAM reads (one byte per clock) for Block RAM compatibility
    -- ==========================================================================
    process(sys_clk_i, reset_n)
        variable base : unsigned(7 downto 0);
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
            ssrc_o <= (others => '0');
            ram_rd_addr <= (others => '0');
            ip_addr_latch <= (others => '0');
            ch_ids_latch <= (others => '0');
            ssrc_latch <= (others => '0');
        elsif rising_edge(sys_clk_i) then

            case tx_state is
                when IDLE =>
                    if fifo_rd_ptr /= fifo_wr_ptr then
                        current_stream <= fifo_stream(to_integer(fifo_rd_ptr));
                        current_wr_ptr <= fifo_wrptr(to_integer(fifo_rd_ptr));
                        current_time <= fifo_time(to_integer(fifo_rd_ptr));
                        fifo_rd_ptr <= fifo_rd_ptr + 1;
                        -- Setup first RAM read address (IP byte 1 at offset 1)
                        -- Base address = stream_index * 32 (5 zero bits)
                        ram_rd_addr <= (fifo_stream(to_integer(fifo_rd_ptr)) & "00000") + 1;
                        tx_state <= LOAD_IP1;
                    end if;

                -- Sequential IP address loading (4 bytes)
                when LOAD_IP1 =>
                    ram_rd_addr <= (current_stream & "00000") + 2;
                    tx_state <= LOAD_IP2;
                    
                when LOAD_IP2 =>
                    ip_addr_latch(31 downto 24) <= ram_rd_data;  -- IP byte 1
                    ram_rd_addr <= (current_stream & "00000") + 3;
                    tx_state <= LOAD_IP3;
                    
                when LOAD_IP3 =>
                    ip_addr_latch(23 downto 16) <= ram_rd_data;  -- IP byte 2
                    ram_rd_addr <= (current_stream & "00000") + 4;
                    tx_state <= LOAD_IP4;
                    
                when LOAD_IP4 =>
                    ip_addr_latch(15 downto 8) <= ram_rd_data;   -- IP byte 3
                    ram_rd_addr <= (current_stream & "00000") + 5;
                    tx_state <= LOAD_CH_COUNT;
                    
                -- Channel count
                when LOAD_CH_COUNT =>
                    ip_addr_latch(7 downto 0) <= ram_rd_data;    -- IP byte 4
                    ram_rd_addr <= (current_stream & "00000") + 6;
                    tx_state <= LOAD_SAMPLES_PER_PKT;
                    
                -- Samples per packet
                when LOAD_SAMPLES_PER_PKT =>
                    channel_count_o <= ram_rd_data;
                    ram_rd_addr <= (current_stream & "00000") + 7;
                    tx_state <= LOAD_CH0;
                    
                -- Channel IDs (8 bytes)
                when LOAD_CH0 =>
                    samples_per_packet_per_channel_o <= ram_rd_data;
                    ram_rd_addr <= (current_stream & "00000") + 8;
                    tx_state <= LOAD_CH1;
                    
                when LOAD_CH1 =>
                    ch_ids_latch(63 downto 56) <= ram_rd_data;
                    ram_rd_addr <= (current_stream & "00000") + 9;
                    tx_state <= LOAD_CH2;
                    
                when LOAD_CH2 =>
                    ch_ids_latch(55 downto 48) <= ram_rd_data;
                    ram_rd_addr <= (current_stream & "00000") + 10;
                    tx_state <= LOAD_CH3;
                    
                when LOAD_CH3 =>
                    ch_ids_latch(47 downto 40) <= ram_rd_data;
                    ram_rd_addr <= (current_stream & "00000") + 11;
                    tx_state <= LOAD_CH4;
                    
                when LOAD_CH4 =>
                    ch_ids_latch(39 downto 32) <= ram_rd_data;
                    ram_rd_addr <= (current_stream & "00000") + 12;
                    tx_state <= LOAD_CH5;
                    
                when LOAD_CH5 =>
                    ch_ids_latch(31 downto 24) <= ram_rd_data;
                    ram_rd_addr <= (current_stream & "00000") + 13;
                    tx_state <= LOAD_CH6;
                    
                when LOAD_CH6 =>
                    ch_ids_latch(23 downto 16) <= ram_rd_data;
                    ram_rd_addr <= (current_stream & "00000") + 14;
                    tx_state <= LOAD_CH7;
                    
                when LOAD_CH7 =>
                    ch_ids_latch(15 downto 8) <= ram_rd_data;
                    ram_rd_addr <= (current_stream & "00000") + 16; -- SSRC byte 1 at offset 0x10
                    tx_state <= LOAD_SSRC1;
                    
                -- SSRC (4 bytes, big-endian)
                when LOAD_SSRC1 =>
                    ch_ids_latch(7 downto 0) <= ram_rd_data;
                    ram_rd_addr <= (current_stream & "00000") + 17;
                    tx_state <= LOAD_SSRC2;
                    
                when LOAD_SSRC2 =>
                    ssrc_latch(31 downto 24) <= ram_rd_data;  -- SSRC byte 1
                    ram_rd_addr <= (current_stream & "00000") + 18;
                    tx_state <= LOAD_SSRC3;
                    
                when LOAD_SSRC3 =>
                    ssrc_latch(23 downto 16) <= ram_rd_data;  -- SSRC byte 2
                    ram_rd_addr <= (current_stream & "00000") + 19;
                    tx_state <= LOAD_SSRC4;
                    
                when LOAD_SSRC4 =>
                    ssrc_latch(15 downto 8) <= ram_rd_data;   -- SSRC byte 3
                    tx_state <= ASSERT_START;

                when ASSERT_START =>
                    ssrc_latch(7 downto 0) <= ram_rd_data;    -- SSRC byte 4
                    -- Commit all latched values to outputs
                    ip_addr_o <= ip_addr_latch;
                    ch_ids_o <= ch_ids_latch;
                    ssrc_o <= ssrc_latch(31 downto 8) & ram_rd_data;
                    sample_buffer_tx_start_addr_o <= current_wr_ptr;
                    packet_time_o <= current_time;
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
