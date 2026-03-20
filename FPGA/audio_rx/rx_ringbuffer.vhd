library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity rx_ringbuffer is
    generic
    (
        audio_buffer_sample_depth : integer := 256; -- must be power of 2
        global_channel_count : integer := 16; -- must be power of 2
        bytes_per_sample : integer := 3;
        max_streams : integer := 8
    );
	port
	(
        sys_clk                    : in std_logic;
        reset_n                    : in std_logic;
		audio_ch0_out			: out std_logic_vector(bytes_per_sample * 8 - 1 downto 0);
		audio_ch1_out			: out std_logic_vector(bytes_per_sample * 8 - 1 downto 0);
		audio_ch2_out			: out std_logic_vector(bytes_per_sample * 8 - 1 downto 0);
		audio_ch3_out			: out std_logic_vector(bytes_per_sample * 8 - 1 downto 0);
		audio_ch4_out			: out std_logic_vector(bytes_per_sample * 8 - 1 downto 0);
		audio_ch5_out			: out std_logic_vector(bytes_per_sample * 8 - 1 downto 0);
		audio_ch6_out			: out std_logic_vector(bytes_per_sample * 8 - 1 downto 0);
		audio_ch7_out			: out std_logic_vector(bytes_per_sample * 8 - 1 downto 0);
		audio_ch8_out			: out std_logic_vector(bytes_per_sample * 8 - 1 downto 0);
		audio_ch9_out			: out std_logic_vector(bytes_per_sample * 8 - 1 downto 0);
		audio_ch10_out			: out std_logic_vector(bytes_per_sample * 8 - 1 downto 0);
		audio_ch11_out			: out std_logic_vector(bytes_per_sample * 8 - 1 downto 0);
		audio_ch12_out			: out std_logic_vector(bytes_per_sample * 8 - 1 downto 0);
		audio_ch13_out			: out std_logic_vector(bytes_per_sample * 8 - 1 downto 0);
		audio_ch14_out			: out std_logic_vector(bytes_per_sample * 8 - 1 downto 0);
		audio_ch15_out			: out std_logic_vector(bytes_per_sample * 8 - 1 downto 0);
		fs_clk_i      				: in std_logic;

		media_clock_i: in std_logic_vector(31 downto 0);

		eth_read_addr_o		: out unsigned(10 downto 0);
        eth_read_data_i		: in std_logic_vector(7 downto 0);

        packet_ready_i : in std_logic;

        stream_config_wr_clk_i: in std_logic;
        stream_config_wr_en_i: in std_logic;
        stream_config_addr_i: in std_logic_vector(7 downto 0);
        stream_config_data_i: in std_logic_vector(7 downto 0)

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

    type t_s_parsePacket is (s_Idle, s_readHeader, s_prepare, s_readSampleData, s_End);
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

    signal fs_clk_i_sync1 : std_logic := '0';
    signal fs_clk_i_sync2 : std_logic := '0';
    signal packet_ready_i_sync1 : std_logic := '0';
    signal packet_ready_i_sync2 : std_logic := '0';
    signal zaudio_sync : std_logic := '0';
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
    type t_channel_map is array (0 to 7) of unsigned(3 downto 0);
    signal channel_map : t_channel_map := (others => (others => '0'));

begin

    process(sys_clk, reset_n)
        variable comp_byte : integer range 0 to 32 := 0;
        variable wr_sample_pos : unsigned(7 downto 0);
        variable v_channel_addr : unsigned(ADDR_BITS - 1 downto 0);
    begin
        if reset_n = '0' then
            eth_read_addr_o <= (others => '0');
            sample_rd_ptr <= (others => '0');
            fs_clk_i_sync1 <= '0';
            fs_clk_i_sync2 <= '0';
            zaudio_sync <= '0';
            packet_ready_i_sync1 <= '0';
            packet_ready_i_sync2 <= '0';
            output_next_sample <= '0';
            current_stream_channel_count <= 0;
            media_clock_latch <= (others => '0');
            config_ram_read_ptr <= 0;
            playout_state <= ps_idle;
            playout_channel_id <= 0;
            playout_byte <= 0;
            playout_data_latch <= (others => '0');
            playout_rd_base <= (others => '0');
            packetParserState <= s_Idle;
            wr_addr_current <= (others => '0');
            wr_addr_sample_base <= (others => '0');
            prepare_step <= 0;
            ram_read_wait <= '0';
            channel_map_idx <= 0;
            cached_delay <= (others => '0');
        elsif rising_edge(sys_clk) then
            sample_wr_en <= '0';

            -- CDC for fs_clk_i
            fs_clk_i_sync1 <= fs_clk_i;
            fs_clk_i_sync2 <= fs_clk_i_sync1;
            zaudio_sync <= fs_clk_i_sync2;

            packet_ready_i_sync1 <= packet_ready_i;
            packet_ready_i_sync2 <= packet_ready_i_sync1;

            if zaudio_sync = '0' and fs_clk_i_sync2 = '1' then
                output_next_sample <= '1';
                -- Derive read pointer directly from media clock (same as write side, but without delay)
                -- This keeps read-write distance constant at exactly the configured delay
                sample_rd_ptr <= unsigned(media_clock_i(7 downto 0)) & to_unsigned(0, SAMPLE_SHIFT);
            end if;

            -- ======== Packet parser state machine ========
            case packetParserState is
                when s_Idle =>
                    if packet_ready_i_sync2 = '0' and packet_ready_i_sync1 = '1' then
                        media_clock_latch <= media_clock_i;
                        packetParserState <= s_readHeader;
                        packet_read_index <= 16;
                        eth_read_addr_o <= to_unsigned(16, 11);
                    end if;

                when s_readHeader =>
                    case packet_read_index is
                        when 16 =>
                            current_packet_length(15 downto 8) <= eth_read_data_i;
                            packet_read_index <= 17;
                            eth_read_addr_o <= to_unsigned(17, 11);
                        when 17 =>
                            current_packet_length(7 downto 0) <= eth_read_data_i;
                            packet_read_index <= 30;
                            eth_read_addr_o <= to_unsigned(30, 11);
                        when 30 =>
                            current_packet_dst_ip(31 downto 24) <= eth_read_data_i;
                            packet_read_index <= 31;
                            eth_read_addr_o <= to_unsigned(31, 11);
                        when 31 =>
                            current_packet_dst_ip(23 downto 16) <= eth_read_data_i;
                            packet_read_index <= 32;
                            eth_read_addr_o <= to_unsigned(32, 11);
                        when 32 =>
                            current_packet_dst_ip(15 downto 8) <= eth_read_data_i;
                            packet_read_index <= 33;
                            eth_read_addr_o <= to_unsigned(33, 11);
                        when 33 =>
                            current_packet_dst_ip(7 downto 0) <= eth_read_data_i;
                            packet_read_index <= 36;
                            eth_read_addr_o <= to_unsigned(36, 11);
                        when 36 =>
                            current_packet_dst_port(15 downto 8) <= eth_read_data_i;
                            packet_read_index <= 37;
                            eth_read_addr_o <= to_unsigned(37, 11);
                        when 37 =>
                            current_packet_dst_port(7 downto 0) <= eth_read_data_i;
                            packet_read_index <= 46;
                            eth_read_addr_o <= to_unsigned(46, 11);
                        when 46 =>
                            current_packet_media_clock(31 downto 24) <= eth_read_data_i;
                            packet_read_index <= 47;
                            eth_read_addr_o <= to_unsigned(47, 11);
                        when 47 =>
                            current_packet_media_clock(23 downto 16) <= eth_read_data_i;
                            packet_read_index <= 48;
                            eth_read_addr_o <= to_unsigned(48, 11);
                        when 48 =>
                            current_packet_media_clock(15 downto 8) <= eth_read_data_i;
                            packet_read_index <= 49;
                            eth_read_addr_o <= to_unsigned(49, 11);
                        when 49 =>
                            current_packet_media_clock(7 downto 0) <= eth_read_data_i;
                            packetParserState <= s_prepare;
                        when others =>
                            packetParserState <= s_End;
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
                            -- Wait cycle for RAM read: M10K has 1 cycle addr reg + 1 cycle data reg
                            -- Stay here for 2 cycles total before moving to comparison
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
                            wr_sample_pos := unsigned(current_packet_media_clock(7 downto 0))
                                           + cached_delay;
                            wr_addr_sample_base <= wr_sample_pos & to_unsigned(0, SAMPLE_SHIFT);
                            prepare_step <= 11;

                        when 11 =>
                            -- Set initial write address: base + channel_map(0) * 4 + (bytes_per_sample - 1)
                            v_channel_addr := wr_addr_sample_base
                                + resize(channel_map(0) & "00", ADDR_BITS)
                                + to_unsigned(bytes_per_sample - 1, ADDR_BITS);
                            wr_addr_current <= v_channel_addr;
                            packetParserState <= s_readSampleData;
                            packet_read_index <= 54;
                            eth_read_addr_o <= to_unsigned(54, 11);
                            byte_count_parser <= 0;
                            current_read_channel_index <= 0;
                            current_packet_sample_index <= 0;

                        when others =>
                            packetParserState <= s_Idle;
                    end case;

                when s_readSampleData =>
                    if packet_read_index >= to_integer(unsigned(current_packet_length)) + 14 then
                        packetParserState <= s_End;
                    else
                        -- Write current byte to RAM
                        sample_wr_addr <= wr_addr_current;
                        sample_wr_data <= eth_read_data_i;
                        sample_wr_en <= '1';

                        -- Advance ETH read
                        eth_read_addr_o <= to_unsigned(packet_read_index + 1, 11);
                        packet_read_index <= packet_read_index + 1;

                        -- Compute next address incrementally
                        if byte_count_parser < bytes_per_sample - 1 then
                            byte_count_parser <= byte_count_parser + 1;
                            -- Next byte in same sample: decrement by 1 (MSB-first → address goes down)
                            wr_addr_current <= wr_addr_current - 1;
                        else
                            byte_count_parser <= 0;
                            if current_read_channel_index < current_stream_channel_count - 1 then
                                current_read_channel_index <= current_read_channel_index + 1;
                                -- Jump to next channel's MSB byte using cached channel_map
                                v_channel_addr := wr_addr_sample_base
                                    + resize(channel_map(current_read_channel_index + 1) & "00", ADDR_BITS)
                                    + to_unsigned(bytes_per_sample - 1, ADDR_BITS);
                                wr_addr_current <= v_channel_addr;
                            else
                                current_read_channel_index <= 0;
                                if current_packet_sample_index < current_packet_samples_per_channel - 1 then
                                    current_packet_sample_index <= current_packet_sample_index + 1;
                                    -- Next sample: advance base by SAMPLE_STRIDE (64)
                                    wr_addr_sample_base <= wr_addr_sample_base + to_unsigned(SAMPLE_STRIDE, ADDR_BITS);
                                    -- Jump to first channel of next sample
                                    v_channel_addr := wr_addr_sample_base + to_unsigned(SAMPLE_STRIDE, ADDR_BITS)
                                        + resize(channel_map(0) & "00", ADDR_BITS)
                                        + to_unsigned(bytes_per_sample - 1, ADDR_BITS);
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

            -- ======== Playout logic (pipelined for synchronous RAM read) ========
            case playout_state is
                when ps_idle =>
                    if output_next_sample = '1' then
                        playout_state <= ps_addr;
                        playout_channel_id <= 0;
                        playout_byte <= 0;
                        playout_rd_base <= sample_rd_ptr;
                        playout_data_latch <= (others => '0');
                        -- Present first read address: channel 0, MSB byte
                        -- channel 0 * 4 + (bytes_per_sample - 1) = 2
                        sample_rd_addr <= sample_rd_ptr + to_unsigned(bytes_per_sample - 1, ADDR_BITS);
                    end if;

                when ps_addr =>
                    playout_state <= ps_data;

                when ps_data =>
                    -- Capture data
                    playout_data_latch((bytes_per_sample - 1 - playout_byte)*8 + 7 downto (bytes_per_sample - 1 - playout_byte)*8) <= sample_rd_data;

                    if playout_byte = bytes_per_sample - 1 then
                        -- Full sample assembled: output using last byte directly
                        case playout_channel_id is
                            when 0 => audio_ch0_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 1 => audio_ch1_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 2 => audio_ch2_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 3 => audio_ch3_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 4 => audio_ch4_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 5 => audio_ch5_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 6 => audio_ch6_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 7 => audio_ch7_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                          --  when 8 => audio_ch8_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                          --  when 9 => audio_ch9_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                          --  when 10 => audio_ch10_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                         --   when 11 => audio_ch11_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                         --   when 12 => audio_ch12_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                         --   when 13 => audio_ch13_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                         --   when 14 => audio_ch14_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                         --   when 15 => audio_ch15_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when others => null;
                        end case;

                        playout_byte <= 0;
                        playout_data_latch <= (others => '0');

                        if playout_channel_id = global_channel_count - 1 then
                            -- All channels done
                            output_next_sample <= '0';
                            playout_state <= ps_idle;
                        else
                            playout_channel_id <= playout_channel_id + 1;
                            -- Next channel MSB byte: base + (ch+1)*4 + (bps-1)
                            sample_rd_addr <= playout_rd_base
                                + to_unsigned((playout_channel_id + 1) * CHANNEL_STRIDE + (bytes_per_sample - 1), ADDR_BITS);
                            playout_state <= ps_addr;
                        end if;
                    else
                        playout_byte <= playout_byte + 1;
                        -- Next byte in same channel (address decrements)
                        sample_rd_addr <= playout_rd_base
                            + to_unsigned(playout_channel_id * CHANNEL_STRIDE + (bytes_per_sample - 1 - (playout_byte + 1)), ADDR_BITS);
                        playout_state <= ps_addr;
                    end if;
            end case;
        end if;
    end process;

    -- Dedicated sample_ram process (simple dual-port: sync write + sync read)
    -- no_rw_check attribute tells Quartus read-during-write result is don't-care,
    -- allowing M9K inference without undefined output on address collision
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            if sample_wr_en = '1' then
                sample_ram(to_integer(sample_wr_addr)) <= sample_wr_data;
            end if;
            sample_rd_data <= sample_ram(to_integer(sample_rd_addr));
        end if;
    end process;

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
    -- IMPORTANT: RAM read must be in its own process with ONLY the registered read.
    -- Adding additional registers in the same process prevents M9K/M10K inference.
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
