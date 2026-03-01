library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity rx_ringbuffer is
    generic
    (
        audio_buffer_sample_depth : integer := 96; -- number of samples per channel to buffer
        global_channel_count : integer := 16; -- number of channels to buffer
        bytes_per_sample : integer := 3; -- number of bytes per sample (e.g., 3 for 24-bit audio)
        max_streams : integer := 4
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

        stream_config_clk_i: in std_logic;
        stream_config_addr_i: in unsigned(7 downto 0);
        stream_config_data_i: in std_logic_vector(7 downto 0)
		
	);
end entity;

architecture Behavioral of rx_ringbuffer is
    constant AUDIO_BUFFER_LENGTH : integer := audio_buffer_sample_depth * global_channel_count * bytes_per_sample * 2; -- total number of bytes in the buffer
    type t_s_parsePacket is (s_Idle, s_readHeader, s_prepare, s_readSampleData, s_End);
    signal packetParserState : t_s_parsePacket := s_Idle;

    type t_sample_ram is array (0 to AUDIO_BUFFER_LENGTH - 1) of std_logic_vector(7 downto 0);
    type t_stream_ram is array (0 to max_streams*32 - 1) of std_logic_vector(7 downto 0);
    -- stream ram layout:

    -- [0] - [3] - destination IP address (4 bytes, big-endian, multicast group)
    -- [4] - [5] - destination UDP port (2 bytes, big-endian)
    -- [6] - [13] - channel output map (8 bytes because max 8 channels per stream, each byte is the output channel id)
    -- [14] - channel count
    -- [15] - output delay in samples
    -- [16] - samples per channel

   	signal sample_ram		: t_sample_ram := (others => (others => '0'));
    signal stream_ram : t_stream_ram := (others => (others => '0'));
    attribute ramstyle : string;
    attribute ramstyle of sample_ram : signal is "M9K";
    signal sample_rd_ptr : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;

    -- Synchronous read port for sample_ram (required for block RAM inference)
    signal sample_rd_addr : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;
    signal sample_rd_data : std_logic_vector(7 downto 0) := (others => '0');

    -- Write port signals for sample_ram
    signal sample_wr_addr : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;
    signal sample_wr_data : std_logic_vector(7 downto 0) := (others => '0');
    signal sample_wr_en   : std_logic := '0';

    signal fs_clk_i_sync1 : std_logic := '0';
    signal fs_clk_i_sync2 : std_logic := '0';
    signal packet_ready_i_sync1 : std_logic := '0';
    signal packet_ready_i_sync2 : std_logic := '0';
    signal zaudio_sync : std_logic := '0';
    signal byte_count_parser : integer range 0 to bytes_per_sample - 1 := 0;
    signal output_next_sample : std_logic := '0';

    -- Playout pipeline states: addr phase -> data phase
    type t_playout_state is (ps_idle, ps_addr, ps_data);
    signal playout_state : t_playout_state := ps_idle;
    signal playout_channel_id : integer range 0 to global_channel_count - 1 := 0;
    signal playout_byte : integer range 0 to bytes_per_sample - 1 := 0;
    signal playout_data_latch : std_logic_vector(bytes_per_sample * 8 - 1 downto 0) := (others => '0');
    signal playout_rd_base : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;

    signal current_read_channel_index : integer range 0 to 7 := 0;
    signal current_stream_channel_count : integer range 0 to 7 := 0;
    signal media_clock_latch : std_logic_vector(31 downto 0) := (others => '0');
    signal packet_read_index : integer range 0 to 1500 := 0;

    signal current_packet_sample_index : integer range 0 to 255 := 0;
    signal current_packet_length: STD_LOGIC_VECTOR(15 downto 0) := (others => '0');
    signal current_packet_dst_ip : std_logic_vector(31 downto 0) := (others => '0');
    signal current_packet_dst_port : std_logic_vector(15 downto 0) := (others => '0');
    signal current_packet_media_clock : std_logic_vector(31 downto 0) := (others => '0');
    signal current_packet_samples_per_channel : integer range 0 to 255 := 0;
    signal wr_ptr_start: integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;
    signal wr_sample_offset : integer range 0 to 511 := 0;
    signal prepare_step: integer range 0 to 10 := 0;
    signal config_ram_read_ptr: integer range 0 to max_streams*32 := 0;
begin
    
    process(sys_clk, reset_n)
        variable comp_byte: integer range 0 to 32 := 0;
        variable raw_addr : integer;
        variable clock_delta : unsigned(8 downto 0); -- 9-bit for delay + small clock diff
    begin
        if reset_n = '0' then
            eth_read_addr_o <= (others => '0');
            sample_rd_ptr <= 0;
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
            playout_rd_base <= 0;
        elsif rising_edge(sys_clk) then
            sample_wr_en <= '0';

            -- cdc for fs_clk_i to sys_clk domain
            fs_clk_i_sync1 <= fs_clk_i;
            fs_clk_i_sync2 <= fs_clk_i_sync1;
            zaudio_sync <= fs_clk_i_sync2;

            packet_ready_i_sync1 <= packet_ready_i;
            packet_ready_i_sync2 <= packet_ready_i_sync1;


            if (zaudio_sync = '0' and fs_clk_i_sync2 = '1') then
                output_next_sample <= '1';
                
                    
            end if;

            case packetParserState is
                when s_Idle =>
                    if (packet_ready_i_sync2 = '0' and packet_ready_i_sync1 = '1') then
                        media_clock_latch <= media_clock_i;
                        packetParserState <= s_readHeader;
                        packet_read_index <= 16;
                        eth_read_addr_o <= to_unsigned(16, 11); -- start at index 16, we don't care for the rest so far
                    end if;
                when s_readHeader =>
                    -- eth_read_data_i has data for address set LAST cycle,
                    -- so we latch based on (packet_read_index - 1) and pre-set
                    -- the address for the NEXT read.
                    case packet_read_index is
                        -- packet_read_index tracks which address we set last cycle
                        -- (i.e. data now available on eth_read_data_i)
                        when 16 =>
                            -- addr 16 was set in s_Idle, data now valid
                            current_packet_length(15 downto 8) <= eth_read_data_i;
                            packet_read_index <= 17;
                            eth_read_addr_o <= to_unsigned(17, 11);
                        when 17 =>
                            current_packet_length(7 downto 0) <= eth_read_data_i;
                            packet_read_index <= 30;
                            eth_read_addr_o <= to_unsigned(30, 11); -- skip to destination IP
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
                            eth_read_addr_o <= to_unsigned(36, 11); -- skip to dest port
                        when 36 =>
                            current_packet_dst_port(15 downto 8) <= eth_read_data_i;
                            packet_read_index <= 37;
                            eth_read_addr_o <= to_unsigned(37, 11);
                        when 37 =>
                            current_packet_dst_port(7 downto 0) <= eth_read_data_i;
                            packet_read_index <= 46;
                            eth_read_addr_o <= to_unsigned(46, 11); -- skip to media clock
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
                                prepare_step <= 1;
                            when 1 =>
                                -- find stream in ram by destination IP [0..3] + dest port [4..5]
                                if (config_ram_read_ptr >= max_streams * 32) then
                                    -- stream not configured for us; skip
                                    packetParserState <= s_End;
                                else
                                    if (comp_byte < 4 and stream_ram(config_ram_read_ptr + comp_byte) = current_packet_dst_ip(31 - comp_byte * 8 downto 24 - comp_byte * 8)) then
                                        comp_byte := comp_byte + 1;
                                    elsif (comp_byte >= 4 and stream_ram(config_ram_read_ptr + comp_byte) = current_packet_dst_port(15 - (comp_byte - 4) * 8 downto 8 - (comp_byte - 4) * 8)) then
                                        if (comp_byte = 5) then
                                            -- this stream is for us!!!
                                            prepare_step <= 2;
                                            comp_byte := 0;
                                        else
                                            comp_byte := comp_byte + 1;
                                        end if;
                                    else
                                        comp_byte := 0;
                                        config_ram_read_ptr <= config_ram_read_ptr + 32;
                                    end if;
                                end if;
                            when 2 =>
                                current_stream_channel_count <= to_integer(unsigned(stream_ram(config_ram_read_ptr + 14)));
                                prepare_step <= 3;
                            when 3 =>
                                current_packet_samples_per_channel <= to_integer(unsigned(stream_ram(config_ram_read_ptr + 16)));
                                prepare_step <= 4;
                            when 4 =>
                                -- media clocks are sample counters; delta is ~0 when synced
                                -- delay (8-bit) determines buffer write position
                                clock_delta := resize(unsigned(current_packet_media_clock(7 downto 0)), 9)
                                             - resize(unsigned(media_clock_latch(7 downto 0)), 9)
                                             + resize(unsigned(stream_ram(config_ram_read_ptr + 15)), 9);
                                wr_sample_offset <= to_integer(clock_delta);
                                prepare_step <= 5;
                            when 5 =>
                                -- iterative modulo: subtract audio_buffer_sample_depth until in range
                                if wr_sample_offset >= audio_buffer_sample_depth then
                                    wr_sample_offset <= wr_sample_offset - audio_buffer_sample_depth;
                                else
                                    wr_ptr_start <= wr_sample_offset * global_channel_count * bytes_per_sample;
                                    packetParserState <= s_readSampleData;
                                    packet_read_index <= 54;
                                    eth_read_addr_o <= to_unsigned(54, 11);
                                    byte_count_parser <= 0;
                                end if;

                            when others =>
                                packetParserState <= s_Idle;

                        end case;
                         
                when s_readSampleData =>
                    if ( packet_read_index >= to_integer(unsigned(current_packet_length))) then
                        packetParserState <= s_End;

                        else
                        -- read sample data



                        if (byte_count_parser < bytes_per_sample - 1 ) then
                                byte_count_parser <= byte_count_parser + 1;
                            else
                                byte_count_parser <= 0;
                                if (current_read_channel_index < current_stream_channel_count - 1) then
                                    current_read_channel_index <= current_read_channel_index + 1;
                                else
                                    current_read_channel_index <= 0;
                                    if (current_packet_sample_index < current_packet_samples_per_channel - 1) then
                                        current_packet_sample_index <= current_packet_sample_index + 1;
                                    else
                                        current_packet_sample_index <= 0;
                                        -- done
                                        packetParserState <= s_End;
                                    end if;
                                end if;
                        end if;


                        eth_read_addr_o <= to_unsigned(packet_read_index + 1, 11);
                        packet_read_index <= packet_read_index + 1;

                        raw_addr := wr_ptr_start + current_packet_sample_index * global_channel_count * bytes_per_sample + to_integer(unsigned(stream_ram(config_ram_read_ptr + 6 + current_read_channel_index))) * bytes_per_sample + (bytes_per_sample - 1 - byte_count_parser);
                        if raw_addr >= AUDIO_BUFFER_LENGTH then
                            raw_addr := raw_addr - AUDIO_BUFFER_LENGTH;
                        end if;
                        sample_wr_addr <= raw_addr;
                        sample_wr_data <= eth_read_data_i;
                        sample_wr_en <= '1';
                        



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
            



            -- playout logic (pipelined for synchronous RAM read)
            case playout_state is
                when ps_idle =>
                    if output_next_sample = '1' then
                        playout_state <= ps_addr;
                        playout_channel_id <= 0;
                        playout_byte <= 0;
                        playout_rd_base <= sample_rd_ptr;
                        playout_data_latch <= (others => '0');
                        -- present first read address
                        raw_addr := sample_rd_ptr + 0 * bytes_per_sample + (bytes_per_sample - 1 - 0);
                        if raw_addr >= AUDIO_BUFFER_LENGTH then
                            raw_addr := raw_addr - AUDIO_BUFFER_LENGTH;
                        end if;
                        sample_rd_addr <= raw_addr;
                    end if;

                when ps_addr =>
                    -- address was presented last cycle, data will be available next cycle
                    playout_state <= ps_data;

                when ps_data =>
                    -- capture data from synchronous read port
                    playout_data_latch((bytes_per_sample - 1 - playout_byte)*8 + 7 downto (bytes_per_sample - 1 - playout_byte)*8) <= sample_rd_data;

                    if playout_byte = bytes_per_sample - 1 then
                        -- full sample assembled, output it
                        -- note: MSB byte was read first (byte index bytes_per_sample-1 down to 0)
                        -- the last byte just read completes the sample
                        case playout_channel_id is
                            when 0 => audio_ch0_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 1 => audio_ch1_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 2 => audio_ch2_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 3 => audio_ch3_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 4 => audio_ch4_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 5 => audio_ch5_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 6 => audio_ch6_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 7 => audio_ch7_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 8 => audio_ch8_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 9 => audio_ch9_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 10 => audio_ch10_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 11 => audio_ch11_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 12 => audio_ch12_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 13 => audio_ch13_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 14 => audio_ch14_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when 15 => audio_ch15_out <= playout_data_latch((bytes_per_sample-1)*8+7 downto 8) & sample_rd_data;
                            when others => null;
                        end case;

                        playout_byte <= 0;
                        playout_data_latch <= (others => '0');

                        if playout_channel_id = global_channel_count - 1 then
                            -- all channels done, advance read pointer
                            if playout_rd_base + global_channel_count * bytes_per_sample >= AUDIO_BUFFER_LENGTH then
                                sample_rd_ptr <= playout_rd_base + global_channel_count * bytes_per_sample - AUDIO_BUFFER_LENGTH;
                            else
                                sample_rd_ptr <= playout_rd_base + global_channel_count * bytes_per_sample;
                            end if;
                            output_next_sample <= '0';
                            playout_state <= ps_idle;
                        else
                            playout_channel_id <= playout_channel_id + 1;
                            -- present next address (first byte of next channel)
                            raw_addr := playout_rd_base + (playout_channel_id + 1) * bytes_per_sample + (bytes_per_sample - 1);
                            if raw_addr >= AUDIO_BUFFER_LENGTH then
                                raw_addr := raw_addr - AUDIO_BUFFER_LENGTH;
                            end if;
                            sample_rd_addr <= raw_addr;
                            playout_state <= ps_addr;
                        end if;
                    else
                        playout_byte <= playout_byte + 1;
                        -- present next byte address
                        raw_addr := playout_rd_base + playout_channel_id * bytes_per_sample + (bytes_per_sample - 1 - (playout_byte + 1));
                        if raw_addr >= AUDIO_BUFFER_LENGTH then
                            raw_addr := raw_addr - AUDIO_BUFFER_LENGTH;
                        end if;
                        sample_rd_addr <= raw_addr;
                        playout_state <= ps_addr;
                    end if;
            end case;
        end if;
    end process;

    -- Dedicated sample_ram process (simple dual-port: sync write + sync read)
    -- This pattern is required for Quartus to infer block RAM (M9K)
    process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            if sample_wr_en = '1' then
                sample_ram(sample_wr_addr) <= sample_wr_data;
            end if;
            sample_rd_data <= sample_ram(sample_rd_addr);
        end if;
    end process;

    -- write process for configuration ram
    process(stream_config_clk_i)
    begin
        if rising_edge(stream_config_clk_i) then
            stream_ram(to_integer(stream_config_addr_i)) <= stream_config_data_i;
        end if;
    end process;

end Behavioral;