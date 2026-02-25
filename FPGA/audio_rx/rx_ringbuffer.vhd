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

    -- [0] - [3] - ssrc
    -- [4] - [11] - channel output map (8 bytes because max 8 channels per stream, each byte is the output channel id)
    -- [12] - channel count
    -- [13] - output delay in samples
    -- [14] - samples per channel

   	signal sample_ram		: t_sample_ram := (others => (others => '0'));
    signal stream_ram : t_stream_ram := (others => (others => '0'));
    signal sample_wr_ptr : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;
    signal sample_rd_ptr : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;

    signal fs_clk_i_sync1 : std_logic := '0';
    signal fs_clk_i_sync2 : std_logic := '0';
    signal packet_ready_i_sync1 : std_logic := '0';
    signal packet_ready_i_sync2 : std_logic := '0';
    signal zaudio_sync : std_logic := '0';
    signal current_read_channel_id : integer range 0 to 7 := 0;
    signal current_playout_channel_id: integer range 0 to global_channel_count - 1 := 0;

    signal byte_count : integer range 0 to bytes_per_sample - 1 := 0;
    signal byte_count_parser : integer range 0 to bytes_per_sample - 1 := 0;
    signal start_read : std_logic := '0';
    signal output_next_sample : std_logic := '0';

    signal current_read_channel_index : integer range 0 to 7 := 0;
    signal current_stream_channel_count : integer range 0 to 7 := 0;
    signal current_stream_playout_offset : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;
    signal media_clock_latch : std_logic_vector(31 downto 0) := (others => '0');
    signal packet_read_index : integer range 0 to 1500 := 0;

    signal current_packet_sample_index : integer range 0 to 255 := 0;
    signal current_packet_length: STD_LOGIC_VECTOR(15 downto 0) := (others => '0');
    signal current_packet_ssrc : std_logic_vector(31 downto 0) := (others => '0');
    signal current_packet_media_clock : std_logic_vector(31 downto 0) := (others => '0');
    signal current_packet_samples_per_channel : integer range 0 to 255 := 0;
    signal wr_ptr_start: integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;
    signal prepare_step: integer range 0 to 10 := 0;
    signal config_ram_read_ptr: integer range 0 to max_streams*32 - 1 := 0;
begin
    
    process(sys_clk, reset_n)
        variable data_latch : std_logic_vector(bytes_per_sample * 8 - 1 downto 0);
        variable comp_byte: integer range 0 to 32 := 0;
        variable raw_addr : integer;
    begin
        if reset_n = '0' then
            eth_read_addr_o <= (others => '0');
            sample_wr_ptr <= 0;
            sample_rd_ptr <= 0;
            fs_clk_i_sync1 <= '0';
            fs_clk_i_sync2 <= '0';
            zaudio_sync <= '0';
            current_read_channel_id <= 0;
            current_playout_channel_id <= 0;
            byte_count <= 0;
            start_read <= '0';
            packet_ready_i_sync1 <= '0';
            packet_ready_i_sync2 <= '0';
            output_next_sample <= '0';

            current_stream_channel_count <= 0;
            current_stream_playout_offset <= 0;
            media_clock_latch <= (others => '0');
            config_ram_read_ptr <= 0;
        elsif rising_edge(sys_clk) then
            

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
                    eth_read_addr_o <= to_unsigned(packet_read_index, 11);
                    packet_read_index <= packet_read_index + 1;
                    case packet_read_index is
                        when 16 => current_packet_length(15 downto 8) <= eth_read_data_i;
                        when 17 => current_packet_length(7 downto 0) <= eth_read_data_i; packet_read_index <= 46; -- skip to media clock, ignoring rest of header for now
                        when 46 => current_packet_media_clock(31 downto 24) <= eth_read_data_i;
                        when 47 => current_packet_media_clock(23 downto 16) <= eth_read_data_i;
                        when 48 => current_packet_media_clock(15 downto 8) <= eth_read_data_i;
                        when 49 => current_packet_media_clock(7 downto 0) <= eth_read_data_i;
                        when 50 => current_packet_ssrc(31 downto 24) <= eth_read_data_i;
                        when 51 => current_packet_ssrc(23 downto 16) <= eth_read_data_i;
                        when 52 => current_packet_ssrc(15 downto 8) <= eth_read_data_i;
                        when 53 => current_packet_ssrc(7 downto 0) <= eth_read_data_i; packetParserState <= s_prepare;
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
                                -- find stream in ram
                                if (config_ram_read_ptr  + 32 > stream_ram'length) then
                                    -- stream not configured for us; skip
                                    packetParserState <= s_End;
                                else
                                    if (stream_ram(config_ram_read_ptr + comp_byte) = current_packet_ssrc(comp_byte * 8 + 7 downto comp_byte * 8)) then
                                        if (comp_byte = 3) then
                                            -- this stream is for us!!!
                                            prepare_step <= 2;
                                            comp_byte := 0;
                                        else 
                                            comp_byte := comp_byte + 1;
                                        end if;
                                        else
                                            config_ram_read_ptr <= config_ram_read_ptr + 32;
                                    end if;
                                end if;
                            when 2 =>
                                current_stream_channel_count <= to_integer(unsigned(stream_ram(config_ram_read_ptr + 12)));
                                prepare_step <= 3;
                            when 3 => 
                                current_packet_samples_per_channel <= to_integer(unsigned(stream_ram(config_ram_read_ptr + 13)));
                                prepare_step <= 4;
                            when 4 =>
                                raw_addr := (to_integer(unsigned(current_packet_media_clock) - unsigned(media_clock_latch)) + to_integer(unsigned(stream_ram(config_ram_read_ptr + 13)))) * global_channel_count * bytes_per_sample;
                                if raw_addr >= AUDIO_BUFFER_LENGTH then
                                    raw_addr := raw_addr - AUDIO_BUFFER_LENGTH;
                                end if;
                                wr_ptr_start <= raw_addr;
                                packetParserState <= s_readSampleData;
                                packet_read_index <= 54;
                                eth_read_addr_o <= to_unsigned(54, 11);
                                byte_count_parser <= 0;

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

                        raw_addr := wr_ptr_start + current_packet_sample_index * global_channel_count * bytes_per_sample + to_integer(unsigned(stream_ram(config_ram_read_ptr + 4 + current_read_channel_index))) * bytes_per_sample + (bytes_per_sample - 1 - byte_count_parser);
                        if raw_addr >= AUDIO_BUFFER_LENGTH then
                            raw_addr := raw_addr - AUDIO_BUFFER_LENGTH;
                        end if;
                        sample_ram(raw_addr) <= eth_read_data_i;
                        



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
            



            -- playout logic
            if output_next_sample = '1' then

                data_latch(byte_count*8 + 7 downto byte_count*8) := sample_ram(sample_rd_ptr + current_playout_channel_id * bytes_per_sample + byte_count);
                byte_count <= byte_count + 1;
                if byte_count = bytes_per_sample - 1 then
                    byte_count <= 0;

                    case current_playout_channel_id is
                        when 0 => audio_ch0_out <= data_latch;
                        when 1 => audio_ch1_out <= data_latch;
                        when 2 => audio_ch2_out <= data_latch;
                        when 3 => audio_ch3_out <= data_latch;
                        when 4 => audio_ch4_out <= data_latch;
                        when 5 => audio_ch5_out <= data_latch;
                        when 6 => audio_ch6_out <= data_latch;
                        when 7 => audio_ch7_out <= data_latch;
                        when 8 => audio_ch8_out <= data_latch;
                        when 9 => audio_ch9_out <= data_latch;
                        when 10 => audio_ch10_out <= data_latch;
                        when 11 => audio_ch11_out <= data_latch;
                        when 12 => audio_ch12_out <= data_latch;
                        when 13 => audio_ch13_out <= data_latch;
                        when 14 => audio_ch14_out <= data_latch;
                        when 15 => audio_ch15_out <= data_latch;
                        when others => null;
                    end case;

                    if current_playout_channel_id = global_channel_count - 1 then
                        -- reset to first channel and move read pointer forward by one sample (all channels)
                        current_playout_channel_id <= 0;
                        if sample_rd_ptr + global_channel_count * bytes_per_sample >= AUDIO_BUFFER_LENGTH then
                            sample_rd_ptr <= sample_rd_ptr + global_channel_count * bytes_per_sample - AUDIO_BUFFER_LENGTH;
                        else
                            sample_rd_ptr <= sample_rd_ptr + global_channel_count * bytes_per_sample;
                        end if;
                        output_next_sample <= '0';
                    else
                        current_playout_channel_id <= current_playout_channel_id + 1;
                    end if;
                end if;
            end if;
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