library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity rx_ringbuffer is
    generic
    (
        audio_buffer_sample_depth : integer := 256; -- must be power of 2
        global_channel_count : integer := 16; -- must be power of 2
        bytes_per_sample : integer := 3;
        max_streams : integer := 8;
        ENABLE_METERING: BOOLEAN := true;
        PARALLEL_OUT: BOOLEAN := true;
        TDM_OUTPUTS:  integer := 2; -- amount of output pins
        TDM_CHANNELS : integer := 8; -- tdm format, eg. 2, 8, 16

        -- Simulation-only backdoor into sample_ram. When false (the default,
        -- and the only synthesizable value) the dbg_* ports are unused and
        -- the extra logic optimizes away. When true a testbench can write
        -- known data into sample_ram and read it back to test the packet
        -- parser and the playout paths independently. See rx_ringbuffer_tb.
        SIM_SAMPLE_RAM_BACKDOOR : boolean := false

    );
	port
	(
        sys_clk                    : in std_logic;
        reset_n                    : in std_logic;

        audio_out :             out std_logic_vector(bytes_per_sample * 8 * global_channel_count - 1 downto 0);
        tdm_out : out std_logic_vector(TDM_OUTPUTS - 1 downto 0);
		fs_clk_sync_i      				: in std_logic;
        bclk_sync_i : in std_logic;

		media_clock_i: in std_logic_vector(31 downto 0);

		eth_read_addr_o		: out unsigned(10 downto 0);
        eth_read_data_i		: in std_logic_vector(7 downto 0);

        packet_ready_i : in std_logic;

        stream_config_wr_clk_i: in std_logic;
        stream_config_wr_en_i: in std_logic;
        stream_config_addr_i: in std_logic_vector(7 downto 0);
        stream_config_data_i: in std_logic_vector(7 downto 0);
        metering_signal_o : out std_logic_vector(global_channel_count - 1 downto 0);
        metering_clip_o : out std_logic_vector(global_channel_count - 1 downto 0);
        metering_clear_i : in std_logic;

        -- ===== Simulation backdoor into sample_ram (see SIM_SAMPLE_RAM_BACKDOOR) =====
        -- Synchronous, byte-wide. Read has 1-cycle latency like the real RAM.
        -- Tie off / leave open in synthesis.
        dbg_wr_en_i   : in  std_logic := '0';
        dbg_wr_addr_i : in  unsigned(13 downto 0) := (others => '0');
        dbg_wr_data_i : in  std_logic_vector(7 downto 0) := (others => '0');
        dbg_rd_addr_i : in  unsigned(13 downto 0) := (others => '0');
        dbg_rd_data_o : out std_logic_vector(7 downto 0)
	);
end entity;

architecture Behavioral of rx_ringbuffer is
    -- ceil(log2(n)) for computing address bits from generics
    function clog2(n : positive) return natural is
        variable result : natural := 0;
        variable val    : natural := n - 1;
    begin
        while val > 0 loop
            result := result + 1;
            val := val / 2;
        end loop;
        return result;
    end function;

    -- Internal slot size: 4 bytes per sample (pad 24-bit to 32-bit) for power-of-2 addressing
    constant SLOT_BYTES : integer := 4;
    -- Buffer size = depth * channels * slot_bytes (all must be powers of 2)
    constant ADDR_BITS : integer := clog2(audio_buffer_sample_depth * global_channel_count * SLOT_BYTES);
    constant AUDIO_BUFFER_LENGTH : integer := 2**ADDR_BITS;
    -- Stride constants (all must be powers of 2)
    constant CHANNEL_STRIDE : integer := SLOT_BYTES; -- 4
    constant SAMPLE_STRIDE  : integer := global_channel_count * SLOT_BYTES;
    constant SAMPLE_SHIFT   : integer := clog2(SAMPLE_STRIDE); -- log2(SAMPLE_STRIDE) for address computation
    constant SAMPLE_IDX_BITS : integer := ADDR_BITS - SAMPLE_SHIFT; -- bits of sample index within buffer (= clog2(audio_buffer_sample_depth))

    type t_s_parsePacket is (s_Idle, s_readHeader, s_prepare, s_readSampleData_wait, s_readSampleData, s_End);
    signal packetParserState : t_s_parsePacket := s_Idle;

    type t_sample_ram is array (0 to AUDIO_BUFFER_LENGTH - 1) of std_logic_vector(7 downto 0);
    type t_stream_ram is array (0 to 255) of std_logic_vector(7 downto 0);

    signal sample_ram : t_sample_ram := (others => (others => '0'));
    signal stream_ram : t_stream_ram;

    -- Synthesis attributes for Block RAM inference (Intel/Altera, Cyclone 10 LP = M9K blocks)
    attribute ramstyle : string;
    attribute ramstyle of stream_ram : signal is "no_rw_check";

    -- Synchronous read port for stream_ram
    -- Two-stage pipeline: stream_rd_data (RAM output reg) -> stream_rd_data_r (user reg)
    -- This breaks the timing path from RAM to comparison logic
    signal stream_rd_addr : unsigned(7 downto 0) := (others => '0');
    signal stream_rd_data : std_logic_vector(7 downto 0) := (others => '0'); -- RAM output register
    signal stream_rd_data_r : std_logic_vector(7 downto 0) := (others => '0'); -- Pipeline register for comparison

    -- Read pointer: derived from media_clock to maintain fixed offset from write pointer
    signal sample_rd_ptr : unsigned(ADDR_BITS - 1 downto 0) := (others => '0');


    -- Synchronous read port for sample_ram
    signal sample_rd_addr : unsigned(ADDR_BITS - 1 downto 0) := (others => '0');
    signal sample_rd_data : std_logic_vector(7 downto 0) := (others => '0');

    -- Write port signals for sample_ram
    signal sample_wr_addr : unsigned(ADDR_BITS - 1 downto 0) := (others => '0');
    signal sample_wr_data : std_logic_vector(7 downto 0) := (others => '0');
    signal sample_wr_en   : std_logic := '0';

    signal packet_ready_i_sync1 : std_logic := '0';
    signal packet_ready_i_sync2 : std_logic := '0';
    signal packet_ready_i_sync3 : std_logic := '0'; -- 3rd stage: toggle change detection
    signal zaudio_sync : std_logic := '0';
    signal zbclk : std_logic := '0';
    signal byte_count_parser : integer range 0 to bytes_per_sample - 1 := 0;
    signal output_next_sample : std_logic := '0';

    -- Playout pipeline
    type t_playout_state is (ps_idle, ps_addr, ps_data);
    signal playout_state : t_playout_state := ps_idle;
    signal playout_channel_id : integer range 0 to global_channel_count - 1 := 0;
    signal playout_byte : integer range 0 to bytes_per_sample - 1 := 0;
    signal playout_data_latch : std_logic_vector(bytes_per_sample * 8 - 1 downto 0) := (others => '0');
    signal playout_rd_base : unsigned(ADDR_BITS - 1 downto 0) := (others => '0');

    signal current_read_channel_index : integer range 0 to 7 := 0;
    signal current_stream_channel_count : integer range 0 to 8 := 0;
    signal media_clock_latch : std_logic_vector(31 downto 0) := (others => '0');
    signal packet_read_index : integer range 0 to 1500 := 0;

    signal current_packet_sample_index : integer range 0 to 255 := 0;
    signal current_packet_length : std_logic_vector(15 downto 0) := (others => '0');
    signal current_packet_dst_ip : std_logic_vector(31 downto 0) := (others => '0');
    signal current_packet_dst_port : std_logic_vector(15 downto 0) := (others => '0');
    signal current_packet_media_clock : std_logic_vector(31 downto 0) := (others => '0');
    signal current_packet_samples_per_channel : integer range 0 to 255 := 0;

    -- Incremental write address: computed once, then incremented
    signal wr_addr_current : unsigned(ADDR_BITS - 1 downto 0) := (others => '0');
    -- Base address for current sample (all channels)
    signal wr_addr_sample_base : unsigned(ADDR_BITS - 1 downto 0) := (others => '0');

    signal prepare_step : integer range 0 to 15 := 0;
    signal ram_read_wait : std_logic := '0'; -- Extra wait cycle for M10K RAM read latency
    signal config_ram_read_ptr : integer range 0 to max_streams*32 := 0;
    signal channel_map_idx : integer range 0 to 7 := 0;
    signal cached_delay : unsigned(7 downto 0) := (others => '0');

    -- Cached channel map from stream_ram (avoids repeated stream_ram reads during parsing)
    type t_channel_map is array (0 to 7) of unsigned(3 downto 0); -- hardcoded 8 channels per stream, as maximum per AES67 spec
    signal channel_map : t_channel_map := (others => (others => '0'));
    signal metering_clear_i_sync1 : std_logic := '0';
    signal metering_clear_i_sync2 : std_logic := '0';
    signal metering_clear_last : std_logic := '0';
    

    -- tdm output signals
    type t_tdm_latch is array (0 to TDM_OUTPUTS - 1) of STD_LOGIC_VECTOR(7 downto 0);
    signal tdm_byte_latch  : t_tdm_latch := (others => (others => '0')); -- driving the serial output
    signal tdm_byte_shadow : t_tdm_latch := (others => (others => '0')); -- pre-fetched next byte
    signal tdm_channel_bit_counter : unsigned(4 downto 0) := (others => '0');
    signal tdm_channel_counter : unsigned(clog2(TDM_CHANNELS)-1 downto 0) := (others => '0');

    -- TDM byte fetch FSM. We prefetch one byte for every TDM_OUTPUTS pin into
    -- tdm_byte_shadow during the previous bclk byte time (8 bclk ~ 32 sys_clk),
    -- then commit shadow -> latch atomically when bit_counter wraps to the
    -- next byte boundary.
    type t_tdm_fetch_state is (tfs_idle, tfs_addr_wait, tfs_capture);
    signal tdm_fetch_state    : t_tdm_fetch_state := tfs_idle;
    signal tdm_fetch_pin_idx  : integer range 0 to TDM_OUTPUTS := 0;
    -- byte_in_slot is also the RAM byte offset: we read offsets 0,1,2,3 of
    -- each channel's 4-byte slot in order. No mapping, no scrambling --
    -- whatever the packet parser wrote into RAM[ch*4 + 0..3] goes straight
    -- out on the wire in that order.
    signal tdm_byte_in_slot   : unsigned(1 downto 0) := (others => '0');
    -- Channel that the prefetch is currently building.
    signal tdm_fetch_ch       : unsigned(clog2(TDM_CHANNELS)-1 downto 0) := (others => '0');
    signal tdm_byte_tick    : std_logic := '0';  -- pulse: start prefetching next byte
    signal tdm_commit_tick  : std_logic := '0';  -- pulse: copy shadow -> latch
    signal tdm_byte_off     : unsigned(1 downto 0) := (others => '0'); -- captured offset for in-flight burst
    
    -- Metering
    constant SAMPLE_BITS : integer := bytes_per_sample * 8;

    -- Thresholds for clip/signal detection (signed audio, magnitude only = SAMPLE_BITS-1 wide)
    -- Clip:   0x640000 ≈ -2 dBFS
    -- Signal: 0x004000 ≈ -42 dBFS
    constant CLIP_THRESHOLD   : unsigned(SAMPLE_BITS - 2 downto 0) := to_unsigned(16#640000#, SAMPLE_BITS - 1);
    constant SIGNAL_THRESHOLD : unsigned(SAMPLE_BITS - 2 downto 0) := to_unsigned(16#004000#, SAMPLE_BITS - 1);
begin

    process(sys_clk, reset_n)
        variable comp_byte : integer range 0 to 32 := 0;
        variable wr_sample_pos : unsigned(SAMPLE_IDX_BITS - 1 downto 0);
        variable v_channel_addr : unsigned(ADDR_BITS - 1 downto 0);
    begin
        if reset_n = '0' then
            eth_read_addr_o <= (others => '0');

            current_stream_channel_count <= 0;
            media_clock_latch <= (others => '0');
            config_ram_read_ptr <= 0;
            packetParserState <= s_Idle;
            wr_addr_current <= (others => '0');
            wr_addr_sample_base <= (others => '0');
            prepare_step <= 0;
            ram_read_wait <= '0';
            channel_map_idx <= 0;
            cached_delay <= (others => '0');
            
        elsif rising_edge(sys_clk) then
            sample_wr_en <= '0';

            -- ======== Packet parser state machine ========
            case packetParserState is
                when s_Idle =>
                    if packet_ready_i_sync2 /= packet_ready_i_sync3 then
                        media_clock_latch <= media_clock_i;
                        packetParserState <= s_readHeader;
                        packet_read_index <= 13;
                        eth_read_addr_o <= to_unsigned(15, 11);
                    end if;

                when s_readHeader =>
                    packet_read_index <= packet_read_index + 1;
                    eth_read_addr_o <= to_unsigned(packet_read_index + 2, 11);
                    case packet_read_index is
                        when 16 =>
                            current_packet_length(15 downto 8) <= eth_read_data_i;
                            --eth_read_addr_o <= to_unsigned(17, 11);
                        when 17 =>
                            current_packet_length(7 downto 0) <= eth_read_data_i;
                            --eth_read_addr_o <= to_unsigned(30, 11);
                        when 30 =>
                            current_packet_dst_ip(31 downto 24) <= eth_read_data_i;
                            --eth_read_addr_o <= to_unsigned(31, 11);
                        when 31 =>
                            current_packet_dst_ip(23 downto 16) <= eth_read_data_i;
                            --eth_read_addr_o <= to_unsigned(32, 11);
                        when 32 =>
                            current_packet_dst_ip(15 downto 8) <= eth_read_data_i;
                            --eth_read_addr_o <= to_unsigned(33, 11);
                        when 33 =>
                            current_packet_dst_ip(7 downto 0) <= eth_read_data_i;
                            --eth_read_addr_o <= to_unsigned(36, 11);
                        when 36 =>
                            current_packet_dst_port(15 downto 8) <= eth_read_data_i;
                            --eth_read_addr_o <= to_unsigned(37, 11);
                        when 37 =>
                            current_packet_dst_port(7 downto 0) <= eth_read_data_i;
                            --eth_read_addr_o <= to_unsigned(46, 11);
                        when 46 =>
                            current_packet_media_clock(31 downto 24) <= eth_read_data_i;
                            --eth_read_addr_o <= to_unsigned(47, 11);
                        when 47 =>
                            current_packet_media_clock(23 downto 16) <= eth_read_data_i;
                            --eth_read_addr_o <= to_unsigned(48, 11);
                        when 48 =>
                            current_packet_media_clock(15 downto 8) <= eth_read_data_i;
                            --eth_read_addr_o <= to_unsigned(49, 11);
                        when 49 =>
                            current_packet_media_clock(7 downto 0) <= eth_read_data_i;
                            packetParserState <= s_prepare;
                        when others =>
                            null;
                    end case;

                when s_prepare =>
                    case prepare_step is
                        when 0 =>
                            config_ram_read_ptr <= 0;
                            comp_byte := 0;
                            -- Present first read address to stream_ram
                            stream_rd_addr <= to_unsigned(0, 8);
                            prepare_step <= 1;

                        when 1 =>
                            -- Wait cycle for RAM read
                            if ram_read_wait = '0' then
                                ram_read_wait <= '1';
                            else
                                ram_read_wait <= '0';
                                prepare_step <= 2;
                            end if;

                        when 2 =>
                            -- Compare stream_rd_data against packet header
                            if config_ram_read_ptr >= max_streams * 32 then
                                packetParserState <= s_End;
                            else
                                if comp_byte < 4 then
                                    if stream_rd_data_r = current_packet_dst_ip(31 - comp_byte * 8 downto 24 - comp_byte * 8) then
                                        comp_byte := comp_byte + 1;
                                        stream_rd_addr <= to_unsigned(config_ram_read_ptr + comp_byte, 8);
                                        prepare_step <= 1;
                                    else
                                        comp_byte := 0;
                                        config_ram_read_ptr <= config_ram_read_ptr + 32;
                                        stream_rd_addr <= to_unsigned(config_ram_read_ptr + 32, 8);
                                        prepare_step <= 1;
                                    end if;
                                else
                                    -- comp_byte 4 or 5: check port
                                    if stream_rd_data_r = current_packet_dst_port(15 - (comp_byte - 4) * 8 downto 8 - (comp_byte - 4) * 8) then
                                        if comp_byte = 5 then
                                            -- Match found! Start caching channel map
                                            channel_map_idx <= 0;
                                            -- Present address for first channel map entry (offset +6)
                                            stream_rd_addr <= to_unsigned(config_ram_read_ptr + 6, 8);
                                            prepare_step <= 3;
                                        else
                                            comp_byte := comp_byte + 1;
                                            stream_rd_addr <= to_unsigned(config_ram_read_ptr + comp_byte, 8);
                                            prepare_step <= 1;
                                        end if;
                                    else
                                        comp_byte := 0;
                                        config_ram_read_ptr <= config_ram_read_ptr + 32;
                                        stream_rd_addr <= to_unsigned(config_ram_read_ptr + 32, 8);
                                        prepare_step <= 1;
                                    end if;
                                end if;
                            end if;

                        when 3 =>
                            -- Wait for stream_rd_data for channel_map entry (2 cycles)
                            if ram_read_wait = '0' then
                                ram_read_wait <= '1';
                            else
                                ram_read_wait <= '0';
                                prepare_step <= 4;
                            end if;

                        when 4 =>
                            -- Latch channel map entry, advance to next
                            channel_map(channel_map_idx) <= unsigned(stream_rd_data_r(3 downto 0));
                            if channel_map_idx = 7 then
                                -- All 8 channel map entries cached, read channel_count (offset +14)
                                stream_rd_addr <= to_unsigned(config_ram_read_ptr + 14, 8);
                                prepare_step <= 5;
                            else
                                channel_map_idx <= channel_map_idx + 1;
                                stream_rd_addr <= to_unsigned(config_ram_read_ptr + 6 + channel_map_idx + 1, 8);
                                prepare_step <= 3;
                            end if;

                        when 5 =>
                            -- Wait for channel_count data (2 cycles)
                            if ram_read_wait = '0' then
                                ram_read_wait <= '1';
                            else
                                ram_read_wait <= '0';
                                prepare_step <= 6;
                            end if;

                        when 6 =>
                            -- Latch channel_count, read delay (offset +15)
                            current_stream_channel_count <= to_integer(unsigned(stream_rd_data_r));
                            stream_rd_addr <= to_unsigned(config_ram_read_ptr + 15, 8);
                            prepare_step <= 7;

                        when 7 =>
                            -- Wait for delay data (2 cycles)
                            if ram_read_wait = '0' then
                                ram_read_wait <= '1';
                            else
                                ram_read_wait <= '0';
                                prepare_step <= 8;
                            end if;

                        when 8 =>
                            -- Latch delay, read samples_per_channel (offset +16)
                            cached_delay <= unsigned(stream_rd_data_r);
                            stream_rd_addr <= to_unsigned(config_ram_read_ptr + 16, 8);
                            prepare_step <= 9;

                        when 9 =>
                            -- Wait for samples_per_channel data (2 cycles)
                            if ram_read_wait = '0' then
                                ram_read_wait <= '1';
                            else
                                ram_read_wait <= '0';
                                prepare_step <= 10;
                            end if;

                        when 10 =>
                            -- Latch samples_per_channel, compute write position
                            current_packet_samples_per_channel <= to_integer(unsigned(stream_rd_data_r));
                            wr_sample_pos := unsigned(current_packet_media_clock(SAMPLE_IDX_BITS - 1 downto 0))
                                           + resize(cached_delay, SAMPLE_IDX_BITS);
                            wr_addr_sample_base <= wr_sample_pos & to_unsigned(0, SAMPLE_SHIFT);
                            prepare_step <= 11;

                        when 11 =>
                            -- Set initial write address: base + channel_map(0) * 4 + 0
                            -- Linear slot layout: MSB@offset0, mid@1, LSB@2, pad@3.
                            -- First byte from ETH is the MSB (RFC 3190 L24 big-endian),
                            -- so we start at offset 0 and increment.
                            v_channel_addr := wr_addr_sample_base
                                + resize(channel_map(0) & "00", ADDR_BITS);
                            wr_addr_current <= v_channel_addr;
                            -- Go through s_readSampleData_wait to absorb the
                            -- 1-cycle eth_ram read latency: addr 54 is being
                            -- registered now, its data is not valid until the
                            -- cycle after next. Skipping the wait state writes
                            -- the first payload byte from a stale address and
                            -- shifts every sample by one byte.
                            packetParserState <= s_readSampleData_wait;
                            packet_read_index <= 54;
                            eth_read_addr_o <= to_unsigned(54, 11);
                            byte_count_parser <= 0;
                            current_read_channel_index <= 0;
                            current_packet_sample_index <= 0;

                        when others =>
                            packetParserState <= s_Idle;
                    end case;
                 when s_readSampleData_wait =>
                    -- Pipeline fill for the 1-cycle eth_ram read latency.
                    -- prepare_step 11 registered the read address (54) last
                    -- cycle, so byte[54] appears at eth_read_data_i next cycle
                    -- (when s_readSampleData first runs). Pre-advance the read
                    -- address to 55 here so the read pointer stays exactly one
                    -- ahead of packet_read_index throughout the loop, keeping
                    -- the invariant "eth_read_data_i == byte[packet_read_index]".
                    eth_read_addr_o <= to_unsigned(packet_read_index + 1, 11);
                    packetParserState <= s_readSampleData;
                when s_readSampleData =>
                    if packet_read_index >= to_integer(unsigned(current_packet_length)) + 14 then
                        packetParserState <= s_End;
                    else
                        -- Write current byte to RAM
                        sample_wr_addr <= wr_addr_current;
                        sample_wr_data <= eth_read_data_i;
                        sample_wr_en <= '1';

                        -- Advance ETH read: keep eth_read_addr_o one ahead of
                        -- packet_read_index (the wait state primed it to +1).
                        eth_read_addr_o <= to_unsigned(packet_read_index + 2, 11);
                        packet_read_index <= packet_read_index + 1;

                        -- Compute next address incrementally
                        if byte_count_parser < bytes_per_sample - 1 then
                            byte_count_parser <= byte_count_parser + 1;
                            -- Next byte in same sample: increment by 1.
                            -- MSB written first at offset 0 → mid@1, LSB@2 (address goes up).
                            wr_addr_current <= wr_addr_current + 1;
                        else
                            byte_count_parser <= 0;
                            if current_read_channel_index < current_stream_channel_count - 1 then
                                current_read_channel_index <= current_read_channel_index + 1;
                                -- Jump to next channel's MSB byte (offset 0) using cached channel_map
                                v_channel_addr := wr_addr_sample_base
                                    + resize(channel_map(current_read_channel_index + 1) & "00", ADDR_BITS);
                                wr_addr_current <= v_channel_addr;
                            else
                                current_read_channel_index <= 0;
                                if current_packet_sample_index < current_packet_samples_per_channel - 1 then
                                    current_packet_sample_index <= current_packet_sample_index + 1;
                                    -- Next sample: advance base by SAMPLE_STRIDE (64)
                                    wr_addr_sample_base <= wr_addr_sample_base + to_unsigned(SAMPLE_STRIDE, ADDR_BITS);
                                    -- Jump to first channel of next sample (MSB byte, offset 0)
                                    v_channel_addr := wr_addr_sample_base + to_unsigned(SAMPLE_STRIDE, ADDR_BITS)
                                        + resize(channel_map(0) & "00", ADDR_BITS);
                                    wr_addr_current <= v_channel_addr;
                                else
                                    current_packet_sample_index <= 0;
                                    packetParserState <= s_End;
                                end if;
                            end if;
                        end if;
                    end if;

                when s_End =>
                    byte_count_parser <= 0;
                    current_read_channel_index <= 0;
                    current_packet_sample_index <= 0;
                    prepare_step <= 0;
                    packetParserState <= s_Idle;

                when others =>
                    packetParserState <= s_End;
            end case;

        end if;
    end process;



    -- cdc process
    process(sys_clk)

    begin
        if (reset_n = '0') then
            zaudio_sync <= '0';
            packet_ready_i_sync1 <= '0';
            packet_ready_i_sync2 <= '0';
            packet_ready_i_sync3 <= '0';

            metering_clear_i_sync1 <= '0';
            metering_clear_i_sync2 <= '0';
            
            zaudio_sync <= '0';
        elsif (rising_edge(sys_clk)) then
            metering_clear_i_sync1 <= metering_clear_i;
            metering_clear_i_sync2 <= metering_clear_i_sync1;

            zaudio_sync <= fs_clk_sync_i;
            zbclk <= bclk_sync_i;
            

            -- CDC synchronizer for toggle signal from mac_rx_clock domain
            packet_ready_i_sync1 <= packet_ready_i;
            packet_ready_i_sync2 <= packet_ready_i_sync1;
            packet_ready_i_sync3 <= packet_ready_i_sync2;
        end if;

    end process;


parellel_out_proc_gen: if (PARALLEL_OUT = true) generate
    -- data playout
    process(sys_clk)

        variable v_sample_abs : unsigned(SAMPLE_BITS - 2 downto 0);
        variable v_playout_sample : std_logic_vector(SAMPLE_BITS - 1 downto 0);
    begin

        if rising_edge(sys_clk) then
            -- ======== Playout logic (pipelined for synchronous RAM read) ========



            if (metering_clear_i_sync2 /= metering_clear_last) then
                metering_clear_last <= metering_clear_i_sync2;
                metering_clip_o <= (others => '0');
                metering_signal_o <= (others => '0');
            end if;


            if zaudio_sync = '0' and fs_clk_sync_i = '1' then
                output_next_sample <= '1';
                -- Derive read pointer directly from media clock (same as write side, but without delay)
                -- This keeps read-write distance constant at exactly the configured delay
                sample_rd_ptr <= unsigned(media_clock_i(SAMPLE_IDX_BITS - 1 downto 0)) & to_unsigned(0, SAMPLE_SHIFT);
            end if;

            case playout_state is
                when ps_idle =>
                    if output_next_sample = '1' then
                        playout_state <= ps_addr;
                        playout_channel_id <= 0;
                        playout_byte <= 0;
                        playout_rd_base <= sample_rd_ptr;
                        playout_data_latch <= (others => '0');
                        -- Present first read address: channel 0, MSB byte
                        -- Linear slot layout: MSB@offset0, mid@1, LSB@2.
                        -- channel 0 * 4 + 0 = 0
                        sample_rd_addr <= sample_rd_ptr;
                    end if;

                when ps_addr =>
                    playout_state <= ps_data;

                when ps_data =>
                    -- Capture data
                    playout_data_latch((bytes_per_sample - 1 - playout_byte)*8 + 7 downto (bytes_per_sample - 1 - playout_byte)*8) <= sample_rd_data;
                    
                    if playout_byte = bytes_per_sample - 1 then
                        -- Full sample assembled: output using last byte directly

                        audio_out(((playout_channel_id + 1) * (bytes_per_sample * 8)) - 1 downto playout_channel_id * (bytes_per_sample * 8)) 
                            <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                        
                        if (ENABLE_METERING = true) then
                        -- Metering: clip & signal detection on assembled sample
                        -- Use variable for combinational abs + compare in same cycle
                        -- Assemble full sample first, then extract magnitude bits
                        v_playout_sample := playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                        v_sample_abs := unsigned(v_playout_sample(SAMPLE_BITS - 2 downto 0));
                        if v_playout_sample(SAMPLE_BITS - 1) = '1' then
                            v_sample_abs := not v_sample_abs; -- one's complement ≈ abs (off by 1 LSB, fine for threshold)
                        end if;
                        -- Set sticky bits (cleared only by metering_clear toggle)
                        if v_sample_abs >= CLIP_THRESHOLD then
                            metering_clip_o(playout_channel_id) <= '1';
                        end if;
                        if v_sample_abs >= SIGNAL_THRESHOLD then
                            metering_signal_o(playout_channel_id) <= '1';
                        end if;

                        else
                            metering_clip_o <= (others => '0');
                            metering_signal_o <= (others => '0');
                        end if;


                        playout_byte <= 0;
                        playout_data_latch <= (others => '0');

                        if playout_channel_id = global_channel_count - 1 then
                            -- All channels done
                            output_next_sample <= '0';
                            playout_state <= ps_idle;
                        else
                            playout_channel_id <= playout_channel_id + 1;
                            -- Next channel MSB byte: base + (ch+1)*4 + 0
                            sample_rd_addr <= playout_rd_base
                                + to_unsigned((playout_channel_id + 1) * CHANNEL_STRIDE, ADDR_BITS);
                            playout_state <= ps_addr;
                        end if;
                    else
                        playout_byte <= playout_byte + 1;
                        -- Next byte in same channel (address increments: MSB@0 → mid@1 → LSB@2)
                        sample_rd_addr <= playout_rd_base
                            + to_unsigned(playout_channel_id * CHANNEL_STRIDE + (playout_byte + 1), ADDR_BITS);
                        playout_state <= ps_addr;
                    end if;
            end case;
        end if;
    
    end process;
end generate;

tdm_out_parallel_proc_gen: if (PARALLEL_OUT = false) generate
    sample_rd_ptr <= unsigned(media_clock_i(SAMPLE_IDX_BITS - 1 downto 0)) & to_unsigned(0, SAMPLE_SHIFT);



    -- Counters and ticks.
    --   tdm_byte_tick at bit_counter(2:0) = 0 -> start prefetch of the byte
    --     that will be shifted out on the next byte boundary.
    --   tdm_commit_tick at bit_counter(2:0) = 7 -> copy shadow -> latch so
    --     the next bclk edge starts shifting the new byte (MSB first).
    tdm_out_ctrl_proc: process (sys_clk)
    begin
        if rising_edge(sys_clk) then
            tdm_byte_tick   <= '0';
            tdm_commit_tick <= '0';
            if (bclk_sync_i = '1' and zbclk = '0') then
                tdm_channel_bit_counter <= tdm_channel_bit_counter + 1;

                if tdm_channel_bit_counter(2 downto 0) = "000" then
                    tdm_byte_tick <= '1';
                end if;

                if tdm_channel_bit_counter(2 downto 0) = "111" then
                    tdm_commit_tick <= '1';
                end if;

                if tdm_channel_bit_counter = to_unsigned(31, tdm_channel_bit_counter'length) then
                    tdm_channel_counter <= tdm_channel_counter + 1;
                end if;
            end if;
            if (fs_clk_sync_i = '1' and zaudio_sync = '0') then
                tdm_channel_bit_counter <= (others => '0');
                tdm_channel_counter <= (others => '0');
            end if;
        end if;
    end process;

    -- Prefetch FSM. On tdm_byte_tick we walk through all TDM_OUTPUTS pins and
    -- read one byte each (RAM offset = tdm_byte_off, same for every pin) into
    -- tdm_byte_shadow. On tdm_commit_tick we copy shadow -> latch atomically.
    --
    -- Pipeline phase: the shifter consumes one byte per 8 bclk cycles. The
    -- byte_tick at bit_counter=0 starts the prefetch for the byte that will
    -- be shifted out starting at the *next* byte boundary (bit_counter=8).
    -- Commit at bit_counter=7 swaps the latch right before that edge.
    --
    -- byte_in_slot is initialised to 3 so that the *first* byte_tick after
    -- frame sync wraps to 0. With the linear RAM slot layout (offset 0=MSB,
    -- 1=mid, 2=LSB, 3=pad) the RAM offset == byte_in_slot, so the wire carries
    -- MSB, mid, LSB, pad in that time order, which matches the datasheet.
    tdm_fetch_proc: process(sys_clk)
        variable v_ch_global : integer range 0 to global_channel_count - 1;
        variable v_next_byte_in_slot : unsigned(1 downto 0);
        variable v_next_ch           : unsigned(clog2(TDM_CHANNELS)-1 downto 0);
        variable v_off               : unsigned(1 downto 0);
    begin
        if rising_edge(sys_clk) then
            case tdm_fetch_state is
                when tfs_idle =>
                    if tdm_byte_tick = '1' then
                        if tdm_byte_in_slot = to_unsigned(3, 2) then
                            v_next_byte_in_slot := (others => '0');
                            v_next_ch := tdm_fetch_ch + 1;
                        else
                            v_next_byte_in_slot := tdm_byte_in_slot + 1;
                            v_next_ch := tdm_fetch_ch;
                        end if;
                        tdm_byte_in_slot <= v_next_byte_in_slot;
                        tdm_fetch_ch     <= v_next_ch;
                        -- RAM byte offset == position in slot: linear 0..3.
                        v_off := v_next_byte_in_slot;
                        tdm_byte_off <= v_off;

                        tdm_fetch_pin_idx <= 0;
                        v_ch_global := 0 * TDM_CHANNELS + to_integer(v_next_ch);
                        -- "+1" is REQUIRED here (verified on hardware): removing it
                        -- shifts the whole TDM stream one byte early, so the wire
                        -- carries [pad ch(N-1)][MSB ch0][mid ch0][LSB ch0]... .
                        -- This branch is NOT latency-symmetric with tfs_capture: it
                        -- only issues the address once tdm_byte_tick arrives, and
                        -- that tick is itself registered one cycle behind the bclk
                        -- edge in tdm_out_ctrl_proc, so pin 0's read needs to point
                        -- one byte further ahead than the tfs_capture pins to land
                        -- in the same wire byte slot. DO NOT remove this +1.
                        sample_rd_addr <= sample_rd_ptr
                            + to_unsigned(v_ch_global * CHANNEL_STRIDE, ADDR_BITS)
                            + resize(v_off, ADDR_BITS) + 1;
                        tdm_fetch_state <= tfs_addr_wait;
                    end if;

                when tfs_addr_wait =>
                    tdm_fetch_state <= tfs_capture;

                when tfs_capture =>
                    tdm_byte_shadow(tdm_fetch_pin_idx) <= sample_rd_data;

                    if tdm_fetch_pin_idx = TDM_OUTPUTS - 1 then
                        tdm_fetch_state <= tfs_idle;
                    else
                        tdm_fetch_pin_idx <= tdm_fetch_pin_idx + 1;
                        v_ch_global := (tdm_fetch_pin_idx + 1) * TDM_CHANNELS + to_integer(tdm_fetch_ch);
                        -- Linear slot layout: RAM offset == position in slot.
                        -- offset 0,1,2,3 = MSB, mid, LSB, pad. Same offset for every pin.
                        sample_rd_addr <= sample_rd_ptr
                            + to_unsigned(v_ch_global * CHANNEL_STRIDE, ADDR_BITS)
                            + resize(tdm_byte_off, ADDR_BITS);
                        tdm_fetch_state <= tfs_addr_wait;
                    end if;
            end case;

            -- Atomic shadow -> latch swap.
            if tdm_commit_tick = '1' then
                tdm_byte_latch <= tdm_byte_shadow;
            end if;

            -- Frame sync: align so the next byte_tick wraps byte_in_slot
            -- from 3 -> 0 (= MSB lane) and fetch_ch from N-1 -> 0.
            if (fs_clk_sync_i = '1' and zaudio_sync = '0') then
                tdm_byte_in_slot <= to_unsigned(3, 2);
                tdm_fetch_ch     <= (others => '1');
                tdm_fetch_state  <= tfs_idle;
            end if;
        end if;
    end process;

    -- Serial output: shift MSB-first out of each latched byte.
    tdmoutgen : for i in 0 to TDM_OUTPUTS - 1 generate

                tdm_out(i) <= tdm_byte_latch(i)(7 - to_integer(tdm_channel_bit_counter(2 downto 0)));

    end generate;
end generate;


    -- Dedicated sample_ram process (simple dual-port: sync write + sync read)
    -- no_rw_check attribute tells Quartus read-during-write result is don't-care,
    -- allowing M9K inference without undefined output on address collision
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            if sample_wr_en = '1' then
                sample_ram(to_integer(sample_wr_addr)) <= sample_wr_data;
            end if;
            -- Simulation backdoor write (testbench only). Takes priority over
            -- the parser write port; disabled / optimized away in synthesis.
            if SIM_SAMPLE_RAM_BACKDOOR and dbg_wr_en_i = '1' then
                sample_ram(to_integer(dbg_wr_addr_i(ADDR_BITS - 1 downto 0))) <= dbg_wr_data_i;
            end if;
            sample_rd_data <= sample_ram(to_integer(sample_rd_addr));
        end if;
    end process;

    -- Simulation backdoor read port (testbench only). Separate 1-cycle-latency
    -- read so a testbench can inspect sample_ram contents without disturbing
    -- the playout read pointer. Disabled / optimized away in synthesis.
    sim_backdoor_rd_gen : if SIM_SAMPLE_RAM_BACKDOOR generate
        process(sys_clk)
        begin
            if rising_edge(sys_clk) then
                dbg_rd_data_o <= sample_ram(to_integer(dbg_rd_addr_i(ADDR_BITS - 1 downto 0)));
            end if;
        end process;
    end generate;
    sim_backdoor_rd_off_gen : if not SIM_SAMPLE_RAM_BACKDOOR generate
        dbg_rd_data_o <= (others => '0');
    end generate;

    -- Stream config RAM Write Process (Port A - stream_config_wr_clk_i domain)
    process(stream_config_wr_clk_i)
    begin
        if rising_edge(stream_config_wr_clk_i) then
            if stream_config_wr_en_i = '1' then
                stream_ram(to_integer(unsigned(stream_config_addr_i))) <= stream_config_data_i;
            end if;
        end if;
    end process;

    -- Stream config RAM Read Process (Port B - sys_clk domain)
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            stream_rd_data <= stream_ram(to_integer(stream_rd_addr));
        end if;
    end process;

    -- Pipeline register (separate process to not interfere with Block RAM inference)
    -- Breaks timing path from RAM output to comparison logic
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            stream_rd_data_r <= stream_rd_data;
        end if;
    end process;

end Behavioral;
