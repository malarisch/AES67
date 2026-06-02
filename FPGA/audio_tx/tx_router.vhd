library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity tx_router is
    generic
    (
        samples_per_channel_depth : integer := 64; -- number of samples per channel to buffer (power of two: media-clock-derived pointer)
        global_channel_count : integer := 16; -- number of channels to buffer
        bytes_per_sample : integer := 3; -- number of bytes per sample (e.g., 3 for 24-bit audio)
		  max_streams : integer := 8
    );
    port
    (
        sys_clk_i                       : in std_logic;
        reset_n                         : in std_logic;

        ip_addr_o                       : out std_logic_vector(31 downto 0) := (others => '0');
        channel_count_o                 : out std_logic_vector(7 downto 0) := (others => '0');
        samples_per_packet_per_channel_o : out std_logic_vector(7 downto 0) := (others => '0');

        -- packet_time_o is the media-clock value of the packet's OLDEST sample.
        -- It serves BOTH as the RTP timestamp AND as the read pointer: the
        -- transmitter derives its RAM read base directly from this single value
        -- (no separate start-address port -- they carried the same information).
        packet_time_i                   : in std_logic_vector(31 downto 0);
        packet_time_o                   : out std_logic_vector(31 downto 0) := (others => '0');
        sequence_id_o                   : out unsigned(15 downto 0) := (others => '0');

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
        tx_busy_i                       : in std_logic;
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
    
    
    -- Registered read address and data for Block RAM (Port B - sys_clk domain)
    signal ram_rd_addr   : unsigned(7 downto 0) := (others => '0');
    signal ram_rd_data   : std_logic_vector(7 downto 0) := (others => '0');
    

    -- Pre-computed threshold (samples_per_packet - 1) to reduce critical path
    -- Computed in config_wr_clk domain and synchronized
    type t_threshold_shadow is array (0 to 7) of unsigned(7 downto 0);
    signal threshold_shadow : t_threshold_shadow := (others => (others => '0'));
    

    type t_sample_count is array (0 to 7) of unsigned(7 downto 0);
    signal sample_count : t_sample_count := (others => (others => '0'));

    -- Packet-due queue: holds WHICH stream is due (multiple streams can cross
    -- threshold on the same fs edge, so a queue is still required). The write
    -- pointer is no longer queued -- it is re-derived from the media clock at
    -- dequeue time (see current_start_addr).
    type t_stream_fifo is array (0 to max_streams - 1) of unsigned(2 downto 0);
    type t_seqid_fifo is array (0 to max_streams - 1) of unsigned(15 downto 0);
    signal fifo_stream  : t_stream_fifo := (others => (others => '0'));
    signal fifo_seqid   : t_seqid_fifo := (others => (others => '0'));
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
        ASSERT_START, WAIT_TX_EN_HIGH, WAIT_TX_BUSY_LOW
    );
    signal tx_state : t_tx_state := IDLE;

    signal current_stream    : unsigned(2 downto 0) := (others => '0');

    -- CDC synchronizer for tx_en_i (crosses clock domain)
    signal tx_en_meta        : std_logic := '0';
    
    signal tx_en_sync        : std_logic := '0';

    signal tx_busy_meta        : std_logic := '0';
    signal tx_busy_sync        : std_logic := '0';
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
                    -- Pre-compute the threshold as (spp - 1) here, at config time,
                    -- so the per-sample compare/enqueue path no longer needs an
                    -- 8-wide "threshold - 1" subtract every sample. Streams with
                    -- spp < 5 are inactive (see the > 3 guard below), so clamp
                    -- those to 0 to avoid the spp=0 -> 0xFF underflow wrap.
                    if unsigned(config_wr_data_i) >= 5 then
                        threshold_shadow(stream_idx) <= unsigned(config_wr_data_i) - 1;
                    else
                        threshold_shadow(stream_idx) <= (others => '0');
                    end if;
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
    -- Sample counting + FIFO write process
    -- Uses shadow registers instead of direct RAM access
    -- ==========================================================================
    process(sys_clk_i, reset_n)
    begin
        if reset_n = '0' then
            sample_count <= (others => (others => '0'));
            fifo_stream <= (others => (others => '0'));
            fifo_wr_ptr <= (others => '0');
            sample_ready_sync <= '0';
        elsif rising_edge(sys_clk_i) then
            sample_ready_sync <= sample_ready_i;

            -- Falling edge of sample_ready (= 50%-duty LR clock), where the media
            -- clock is stable. Per-stream sample counting / packet-due detection.
            if sample_ready_i = '0' and sample_ready_sync = '1' then
                for i in 0 to max_streams -1 loop

                    -- threshold_shadow already holds (spp - 1); > 3 keeps the
                    -- original "spp > 4" active-stream guard.
                    if threshold_shadow(i) > 3 then
                        if sample_count(i) >= threshold_shadow(i) then
                            sample_count(i) <= (others => '0');
                            -- Enqueue WHICH stream is due; the start address is
                            -- re-derived from the media clock at dequeue time.
                            fifo_stream(to_integer(fifo_wr_ptr)) <= to_unsigned(i, 3);
                            fifo_wr_ptr <= fifo_wr_ptr + 1;
                        else
                            sample_count(i) <= sample_count(i) + 1;
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
            tx_busy_meta <= '0';
            tx_busy_sync <= '0';
        elsif rising_edge(sys_clk_i) then
            tx_en_meta <= tx_en_i;
            tx_en_sync <= tx_en_meta;
            tx_busy_meta <= tx_busy_i;
            tx_busy_sync <= tx_busy_meta;
        end if;
    end process;

    -- ==========================================================================
    -- TX state machine + FIFO read process
    -- Sequential RAM reads (one byte per clock) for Block RAM compatibility
    -- ==========================================================================
    process(sys_clk_i, reset_n)
        variable v_start_media : unsigned(31 downto 0); -- media clock at packet-start time
    begin
        if reset_n = '0' then
            fifo_rd_ptr <= (others => '0');
            tx_state <= IDLE;
            current_stream <= (others => '0');
            audio_packet_tx_start_o <= '0';
            ip_addr_o <= (others => '0');
            channel_count_o <= (others => '0');
            samples_per_packet_per_channel_o <= (others => '0');
            packet_time_o <= (others => '0');
            ch_ids_o <= (others => '0');
            ssrc_o <= (others => '0');
            ram_rd_addr <= (others => '0');
        elsif rising_edge(sys_clk_i) then

            case tx_state is
                when IDLE =>
                    if fifo_rd_ptr /= fifo_wr_ptr then
                        current_stream <= fifo_stream(to_integer(fifo_rd_ptr));

                        fifo_rd_ptr <= fifo_rd_ptr + 1;
                        -- Setup first RAM read address (IP byte 1 at offset 1)
                        -- Base address = stream_index * 32 (5 zero bits)
                        ram_rd_addr <= (fifo_stream(to_integer(fifo_rd_ptr)) & "00000") + 1;
                        
                        tx_state <= LOAD_IP1;
                    end if;

                -- Sequential RAM walk: the read address advances by one every
                -- state, so increment ram_rd_addr instead of recomputing
                -- (base + N) with a distinct constant adder in each state. Only
                -- LOAD_CH7 jumps by two to skip the reserved byte (offset 0x0F).
                when LOAD_IP1 =>
                    -- Reconstruct the media clock value at the instant this stream's
                    -- packet became due (= the OLDEST sample in the packet). This is
                    -- the RTP timestamp AND the read pointer in one: the transmitter
                    -- derives its RAM read base from packet_time_o directly. Self-
                    -- corrects for queue latency: while the packet waits in the queue,
                    -- sample_count and packet_time_i grow by the same sample count, so
                    -- the difference stays pinned to the packet's oldest sample.
                    v_start_media := unsigned(packet_time_i)
                        - sample_count(to_integer(current_stream))
                        - threshold_shadow(to_integer(current_stream));
                    packet_time_o <= std_logic_vector(v_start_media);

                    sequence_id_o <= fifo_seqid(to_integer(current_stream));
                    ram_rd_addr <= ram_rd_addr + 1;
                    tx_state <= LOAD_IP2;

                when LOAD_IP2 =>
                    ip_addr_o(31 downto 24) <= ram_rd_data;  -- IP byte 1
                    ram_rd_addr <= ram_rd_addr + 1;
                    tx_state <= LOAD_IP3;

                when LOAD_IP3 =>
                    ip_addr_o(23 downto 16) <= ram_rd_data;  -- IP byte 2
                    ram_rd_addr <= ram_rd_addr + 1;
                    tx_state <= LOAD_IP4;

                when LOAD_IP4 =>
                    ip_addr_o(15 downto 8) <= ram_rd_data;   -- IP byte 3
                    ram_rd_addr <= ram_rd_addr + 1;
                    tx_state <= LOAD_CH_COUNT;

                -- Channel count
                when LOAD_CH_COUNT =>
                    ip_addr_o(7 downto 0) <= ram_rd_data;    -- IP byte 4
                    ram_rd_addr <= ram_rd_addr + 1;
                    tx_state <= LOAD_SAMPLES_PER_PKT;

                -- Samples per packet
                when LOAD_SAMPLES_PER_PKT =>
                    channel_count_o <= ram_rd_data;
                    ram_rd_addr <= ram_rd_addr + 1;
                    tx_state <= LOAD_CH0;

                -- Channel IDs (8 bytes)
                when LOAD_CH0 =>
                    samples_per_packet_per_channel_o <= ram_rd_data;
                    ram_rd_addr <= ram_rd_addr + 1;
                    tx_state <= LOAD_CH1;

                when LOAD_CH1 =>
                    ch_ids_o(63 downto 56) <= ram_rd_data;
                    ram_rd_addr <= ram_rd_addr + 1;
                    tx_state <= LOAD_CH2;

                when LOAD_CH2 =>
                    ch_ids_o(55 downto 48) <= ram_rd_data;
                    ram_rd_addr <= ram_rd_addr + 1;
                    tx_state <= LOAD_CH3;

                when LOAD_CH3 =>
                    ch_ids_o(47 downto 40) <= ram_rd_data;
                    ram_rd_addr <= ram_rd_addr + 1;
                    tx_state <= LOAD_CH4;

                when LOAD_CH4 =>
                    ch_ids_o(39 downto 32) <= ram_rd_data;
                    ram_rd_addr <= ram_rd_addr + 1;
                    tx_state <= LOAD_CH5;

                when LOAD_CH5 =>
                    ch_ids_o(31 downto 24) <= ram_rd_data;
                    ram_rd_addr <= ram_rd_addr + 1;
                    tx_state <= LOAD_CH6;

                when LOAD_CH6 =>
                    ch_ids_o(23 downto 16) <= ram_rd_data;
                    ram_rd_addr <= ram_rd_addr + 1;
                    tx_state <= LOAD_CH7;

                when LOAD_CH7 =>
                    ch_ids_o(15 downto 8) <= ram_rd_data;
                    ram_rd_addr <= ram_rd_addr + 2; -- skip reserved 0x0F; SSRC byte 1 at offset 0x10
                    tx_state <= LOAD_SSRC1;

                -- SSRC (4 bytes, big-endian)
                when LOAD_SSRC1 =>
                    ch_ids_o(7 downto 0) <= ram_rd_data;
                    ram_rd_addr <= ram_rd_addr + 1;
                    tx_state <= LOAD_SSRC2;

                when LOAD_SSRC2 =>
                    ssrc_o(31 downto 24) <= ram_rd_data;  -- SSRC byte 1
                    ram_rd_addr <= ram_rd_addr + 1;
                    tx_state <= LOAD_SSRC3;

                when LOAD_SSRC3 =>
                    ssrc_o(23 downto 16) <= ram_rd_data;  -- SSRC byte 2
                    ram_rd_addr <= ram_rd_addr + 1;
                    tx_state <= LOAD_SSRC4;
                    
                when LOAD_SSRC4 =>
                    ssrc_o(15 downto 8) <= ram_rd_data;   -- SSRC byte 3
                    tx_state <= ASSERT_START;

                when ASSERT_START =>
                    ssrc_o(7 downto 0) <= ram_rd_data;    -- SSRC byte 4
                    -- Commit all latched values to outputs. The read pointer is
                    -- carried by packet_time_o (set in LOAD_IP1); the transmitter
                    -- derives the RAM read base from it -- no separate address port.
                    audio_packet_tx_start_o <= '1';
                    tx_state <= WAIT_TX_EN_HIGH;

                when WAIT_TX_EN_HIGH =>
                    if tx_en_sync = '1' then
                        audio_packet_tx_start_o <= '0';
                        tx_state <= WAIT_TX_BUSY_LOW;
                    end if;

                when WAIT_TX_BUSY_LOW =>
                    if tx_busy_sync = '0' then
                        tx_state <= IDLE;
                        fifo_seqid(to_integer(current_stream)) <= fifo_seqid(to_integer(current_stream)) + 1;
                    end if;

            end case;
        end if;
    end process;

end Behavioral;