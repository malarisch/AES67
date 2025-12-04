-- UDP Audio Packet Sender
-- (c) 2025 Dr.-Ing. Christian Noeding
-- christian@noeding-online.de
-- Released under GNU General Public License v3
-- Source: https://www.github.com/xn--nding-jua/AES50_Transmitter
--
-- This file contains an ethernet-packet-generator to send individual bytes to an EthernetMAC directly.

library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity udp_audio_packet is
	port
	(
		src_mac_address		: in std_logic_vector(47 downto 0);
		src_ip_address			: in std_logic_vector(31 downto 0);
		dst_mac_address		: in std_logic_vector(47 downto 0);
		dst_ip_address			: in std_logic_vector(31 downto 0);
		src_udp_port			: in std_logic_vector(15 downto 0);
		dst_udp_port			: in std_logic_vector(15 downto 0);
		tx_clk					: in std_logic;
		tx_busy					: in std_logic;
		tx_byte_sent			: in std_logic;
		audio_ch0_in			: in std_logic_vector(23 downto 0);
		audio_ch1_in			: in std_logic_vector(23 downto 0);
		audio_ch2_in			: in std_logic_vector(23 downto 0);
		audio_ch3_in			: in std_logic_vector(23 downto 0);
		audio_ch4_in			: in std_logic_vector(23 downto 0);
		audio_ch5_in			: in std_logic_vector(23 downto 0);
		audio_ch6_in			: in std_logic_vector(23 downto 0);
		audio_ch7_in			: in std_logic_vector(23 downto 0);
		audio_ch8_in			: in std_logic_vector(23 downto 0);
		audio_ch9_in			: in std_logic_vector(23 downto 0);
		audio_ch10_in			: in std_logic_vector(23 downto 0);
		audio_ch11_in			: in std_logic_vector(23 downto 0);
		audio_ch12_in			: in std_logic_vector(23 downto 0);
		audio_ch13_in			: in std_logic_vector(23 downto 0);
		audio_ch14_in			: in std_logic_vector(23 downto 0);
		audio_ch15_in			: in std_logic_vector(23 downto 0);
		audio_sync				: in std_logic;
		
		sample_counter			: in std_logic_vector(31 downto 0) := (others => '0');
		
		tx_enable				: out std_logic := '0';  -- TX valid
		tx_data					: out std_logic_vector(7 downto 0) := (others => '0') -- data-octet
		
	);
end entity;


architecture Behavioral of udp_audio_packet is
	-- Some general thoughts remain the same as in the original description.  All timing
	-- now funnels through small synchronous RAM helpers so the synthesizer can infer
	-- block RAMs for the heavy buffers.

	-- Constants
	constant BUFFERED_AUDIO_SAMPLES	: integer := 16; -- keep limit of 1460 bytes per UDP-frame in mind
	constant AUDIO_CHANNELS			: integer := 4; -- keep audio-channels factor of two if using 24-bit per sample. Otherwise we will get uneven byte-numbers due to 3-byte-samples
	constant BYTES_PER_SAMPLE		: integer := 3; -- 2 (16-bit), 3 (24-bit) or 4 (32-bit)
	constant AUDIO_START_SIGNAL	: integer := 8;
	constant AUDIO_BUFFER_LENGTH	: integer := BUFFERED_AUDIO_SAMPLES * AUDIO_CHANNELS * BYTES_PER_SAMPLE;

	constant MAC_HEADER_LENGTH			: integer := 14;
	constant IP_HEADER_LENGTH			: integer := 5 * (32 / 8); -- Header length always 20 bytes (5 * 32 bit words)
	
	constant UDP_HEADER_LENGTH		: integer := 12;
	constant UDP_PAYLOAD_LENGTH	: integer := AUDIO_START_SIGNAL + AUDIO_BUFFER_LENGTH; -- 8 start-bytes + x bytes for audio
	constant PACKET_LENGTH			: integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH;
	constant PACKET_HEADER_LENGTH	: integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH + AUDIO_START_SIGNAL;
	constant AUDIO_BYTES_PER_SAMPLE_GROUP	: integer := AUDIO_CHANNELS * BYTES_PER_SAMPLE;
	constant SAMPLE_WRAP_GUARD	: integer := AUDIO_BUFFER_LENGTH - (2 * AUDIO_BYTES_PER_SAMPLE_GROUP);

	-- Types
	type t_SM_Ethernet is (s_Idle, s_PrepFrame, s_CalcUdpChecksum, s_FinalizeChecksum, s_WriteChecksum, s_PrimeTx, s_Transmit, s_End);
	type t_sample_ram is array (0 to AUDIO_BUFFER_LENGTH - 1) of std_logic_vector(7 downto 0);
	type t_packet_ram is array (0 to PACKET_LENGTH - 1) of std_logic_vector(7 downto 0);
	type t_pending_bytes is array (0 to AUDIO_BYTES_PER_SAMPLE_GROUP - 1) of std_logic_vector(7 downto 0);

	

	-- Helper declarations
	function get_header_byte(
		idx				: integer;
		src_mac		: std_logic_vector(47 downto 0);
		dst_mac		: std_logic_vector(47 downto 0);
		src_ip		: std_logic_vector(31 downto 0);
		dst_ip		: std_logic_vector(31 downto 0);
		src_port		: std_logic_vector(15 downto 0);
		dst_port		: std_logic_vector(15 downto 0);
		packet_ctr	: unsigned(15 downto 0);
		sample_ctr	: std_logic_vector(31 downto 0)
	) return std_logic_vector is
		constant total_length	: std_logic_vector(15 downto 0) := std_logic_vector(to_unsigned(IP_HEADER_LENGTH + UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 16));
		constant udp_length	: std_logic_vector(15 downto 0) := std_logic_vector(to_unsigned(UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 16));
		constant pkt_cnt	: std_logic_vector(15 downto 0) := std_logic_vector(packet_ctr);
		constant smpl_cnt : std_logic_vector(31 downto 0) := std_logic_vector(sample_ctr);
	begin
		case idx is
		
			-- ethernet
			when 0	=> return dst_mac(47 downto 40);
			when 1	=> return dst_mac(39 downto 32);
			when 2	=> return dst_mac(31 downto 24);
			when 3	=> return dst_mac(23 downto 16);
			when 4	=> return dst_mac(15 downto 8);
			when 5	=> return dst_mac(7 downto 0);
			when 6	=> return src_mac(47 downto 40);
			when 7	=> return src_mac(39 downto 32);
			when 8	=> return src_mac(31 downto 24);
			when 9	=> return src_mac(23 downto 16);
			when 10=> return src_mac(15 downto 8);
			when 11=> return src_mac(7 downto 0);
			when 12=> return x"08";
			when 13=> return x"00";
			
			-- ipv4
			when 14=> return x"45";
			when 15=> return x"00";
			when 16=> return total_length(15 downto 8);
			when 17=> return total_length(7 downto 0);
			when 18=> return pkt_cnt(15 downto 8);
			when 19=> return pkt_cnt(7 downto 0);
			when 20=> return x"00";
			when 21=> return x"00";
			when 22=> return x"80";
			when 23=> return x"11";
			when 24=> return x"00";
			when 25=> return x"00";
			when 26=> return src_ip(31 downto 24);
			when 27=> return src_ip(23 downto 16);
			when 28=> return src_ip(15 downto 8);
			when 29=> return src_ip(7 downto 0);
			when 30=> return dst_ip(31 downto 24);
			when 31=> return dst_ip(23 downto 16);
			when 32=> return dst_ip(15 downto 8);
			when 33=> return dst_ip(7 downto 0);
			
			-- udp
			when 34=> return src_port(15 downto 8);
			when 35=> return src_port(7 downto 0);
			when 36=> return dst_port(15 downto 8);
			when 37=> return dst_port(7 downto 0);
			when 38=> return udp_length(15 downto 8);
			when 39=> return udp_length(7 downto 0);
			when 40=> return x"00";
			when 41=> return x"00";
			
			-- rdp start
			-- bit 0,1: rdp version = 2 = b10
			-- bit 2: rdp padding = b0
			-- bit 3: rdp extension = b0
			-- bit 4,5,6,7: csrcount = b0
			when 42=> return x"80";

			-- payload type: audio data = 97 = 0x61
			when 43=> return x"61";
			
			-- packet counter (2 bytes, big-endian)
			when 44=> return pkt_cnt(15 downto 8);
			when 45=> return pkt_cnt(7 downto 0);

			-- sample counter (4 bytes, big-endian)
			
			when  46=> return smpl_cnt(31 downto 24);
			when  47=> return smpl_cnt(23 downto 16);
			when  48=> return smpl_cnt(15 downto 8);
			when  49=> return smpl_cnt(7 downto 0);
			-- ssrc
			when 50=> return x"de";
			when 51=> return x"ad";
			when 52=> return x"be";
			when 53=> return x"ef";

			when others => return x"00";
		end case;
	end function;

	procedure feed_checksum(
		signal upper_byte	: inout std_logic_vector(7 downto 0);
		signal byte_phase	: inout std_logic;
		signal accumulator	: inout unsigned(31 downto 0);
		constant data_byte	: std_logic_vector(7 downto 0)
	) is
		variable word16 : unsigned(15 downto 0);
	begin
		if (byte_phase = '0') then
			upper_byte <= data_byte;
			byte_phase <= '1';
		else
			word16 := shift_left(resize(unsigned(upper_byte), 16), 8) + resize(unsigned(data_byte), 16);
			accumulator <= accumulator + resize(word16, accumulator'length);
			byte_phase <= '0';
		end if;
	end procedure;

	function finalize_checksum(sum_in : unsigned(31 downto 0)) return std_logic_vector is
		variable tmp : unsigned(31 downto 0) := sum_in;
	begin
		tmp := resize(tmp(15 downto 0), 32) + resize(tmp(31 downto 16), 32);
		tmp := resize(tmp(15 downto 0), 32) + resize(tmp(31 downto 16), 32);
		return std_logic_vector(not tmp(15 downto 0));
	end function;

	-- RAMs and interface signals
	signal sample_ram		: t_sample_ram := (others => (others => '0'));
	signal packet_ram		: t_packet_ram := (others => (others => '0'));

	signal sample_wr_en		: std_logic := '0';
	signal sample_wr_addr	: integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;
	signal sample_wr_data	: std_logic_vector(7 downto 0) := (others => '0');
	signal sample_rd_addr	: integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;
	signal sample_rd_data	: std_logic_vector(7 downto 0) := (others => '0');

	signal packet_wr_en		: std_logic := '0';
	signal packet_wr_addr	: integer range 0 to PACKET_LENGTH - 1 := 0;
	signal packet_wr_data	: std_logic_vector(7 downto 0) := (others => '0');
	signal packet_rd_addr	: integer range 0 to PACKET_LENGTH - 1 := 0;
	signal packet_rd_data	: std_logic_vector(7 downto 0) := (others => '0');
	signal first_packet_byte	: std_logic_vector(7 downto 0) := (others => '0');
	signal first_tx_byte_pending	: std_logic := '0';

	attribute ram_style : string;
	attribute ram_style of sample_ram : signal is "block";
	attribute ram_style of packet_ram : signal is "block";

	-- General control signals
	signal audio_sync_sync1	: std_logic := '0';
	signal audio_sync_sync2	: std_logic := '0';
	signal zaudio_sync		: std_logic := '0';
	signal frame_start		: std_logic := '0';
	signal buffer_locked		: std_logic := '0';
	signal audio_buffer_ptr	: integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;
	signal next_audio_ptr	: integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;
	signal sample_write_active	: std_logic := '0';
	signal sample_write_index	: integer range 0 to AUDIO_BYTES_PER_SAMPLE_GROUP := 0;
	signal sample_write_base	: integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;
	signal pending_bytes		: t_pending_bytes := (others => (others => '0'));

	signal s_SM_Ethernet	: t_SM_Ethernet := s_Idle;
	signal frame_write_index	: integer range 0 to PACKET_LENGTH := 0;
	signal audio_payload_index	: integer range 0 to AUDIO_BUFFER_LENGTH := 0;
	signal checksum_write_index	: integer range 0 to 4 := 0;
	signal prime_wait		: integer range 0 to 2 := 0;
	signal tx_bytes_remaining	: integer range 0 to PACKET_LENGTH := 0;
	signal tx_read_pointer	: integer range 0 to PACKET_LENGTH := 0;

	signal packet_counter		: unsigned(15 downto 0) := to_unsigned(1, 16);
	signal active_packet_counter	: unsigned(15 downto 0) := to_unsigned(1, 16);

	signal ip_checksum_acc	: unsigned(31 downto 0) := (others => '0');
	signal ip_checksum_upper_byte	: std_logic_vector(7 downto 0) := (others => '0');
	signal ip_checksum_byte_phase	: std_logic := '0';
	signal ip_checksum_value	: std_logic_vector(15 downto 0) := (others => '0');

	signal udp_checksum_acc	: unsigned(31 downto 0) := (others => '0');
	signal udp_checksum_upper_byte	: std_logic_vector(7 downto 0) := (others => '0');
	signal udp_checksum_byte_phase	: std_logic := '0';
	signal udp_checksum_value	: std_logic_vector(15 downto 0) := (others => '0');
	signal udp_pseudo_header_sum	: unsigned(31 downto 0) := (others => '0');
	signal udp_checksum_bytes_remaining	: integer range 0 to UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH := 0;
	signal udp_checksum_request_count	: integer range 0 to UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH := 0;
	signal udp_checksum_data_valid	: std_logic := '0';
begin
	sample_ram_process : process(tx_clk)
	begin
		if falling_edge(tx_clk) then
			if (sample_wr_en = '1') then
				sample_ram(sample_wr_addr) <= sample_wr_data;
			end if;
			sample_rd_data <= sample_ram(sample_rd_addr);
		end if;
	end process sample_ram_process;

	packet_ram_process : process(tx_clk)
	begin
		if falling_edge(tx_clk) then
			if (packet_wr_en = '1') then
				packet_ram(packet_wr_addr) <= packet_wr_data;
			end if;
			packet_rd_data <= packet_ram(packet_rd_addr);
		end if;
	end process packet_ram_process;

	main_proc : process(tx_clk)
		variable header_data		: std_logic_vector(7 downto 0);
		variable pseudo_header_sum	: unsigned(31 downto 0);
	begin
		if falling_edge(tx_clk) then
			sample_wr_en <= '0';
			packet_wr_en <= '0';

			-- synchronize audio_sync into tx_clk domain
			audio_sync_sync1 <= audio_sync;
			audio_sync_sync2 <= audio_sync_sync1;
			zaudio_sync <= audio_sync_sync2;

			-- capture audio words when allowed and push them into the sample RAM sequentially
			if ((audio_sync_sync2 = '1') and (zaudio_sync = '0') and (buffer_locked = '0') and (sample_write_active = '0')) then
				pending_bytes(0) <= audio_ch0_in(7 downto 0);
				pending_bytes(1) <= audio_ch0_in(15 downto 8);
				pending_bytes(2) <= audio_ch0_in(23 downto 16);
				pending_bytes(3) <= audio_ch1_in(7 downto 0);
				pending_bytes(4) <= audio_ch1_in(15 downto 8);
				pending_bytes(5) <= audio_ch1_in(23 downto 16);
				pending_bytes(6) <= audio_ch2_in(7 downto 0);
				pending_bytes(7) <= audio_ch2_in(15 downto 8);
				pending_bytes(8) <= audio_ch2_in(23 downto 16);
				pending_bytes(9) <= audio_ch3_in(7 downto 0);
				pending_bytes(10) <= audio_ch3_in(15 downto 8);
				pending_bytes(11) <= audio_ch3_in(23 downto 16);
				sample_write_base <= audio_buffer_ptr;
				sample_write_index <= 0;
				sample_write_active <= '1';
				if (audio_buffer_ptr < SAMPLE_WRAP_GUARD) then
					next_audio_ptr <= audio_buffer_ptr + AUDIO_BYTES_PER_SAMPLE_GROUP;
				elsif (audio_buffer_ptr = SAMPLE_WRAP_GUARD) then
					next_audio_ptr <= audio_buffer_ptr + AUDIO_BYTES_PER_SAMPLE_GROUP;
				else
					next_audio_ptr <= 0;
					frame_start <= '1';
				end if;
			end if;

			if (sample_write_active = '1') then
				sample_wr_en <= '1';
				sample_wr_addr <= sample_write_base + sample_write_index;
				sample_wr_data <= pending_bytes(sample_write_index);
				if (sample_write_index = AUDIO_BYTES_PER_SAMPLE_GROUP - 1) then
					sample_write_active <= '0';
					audio_buffer_ptr <= next_audio_ptr;
				else
					sample_write_index <= sample_write_index + 1;
				end if;
			end if;

			case s_SM_Ethernet is
				when s_Idle =>
					tx_enable <= '0';
					if (frame_start = '1') then
						frame_start <= '0';
						buffer_locked <= '1';
						frame_write_index <= 0;
						audio_payload_index <= 0;
						sample_rd_addr <= 0;
						checksum_write_index <= 0;
								prime_wait <= 0;
						tx_bytes_remaining <= 0;
						tx_read_pointer <= 0;
						active_packet_counter <= packet_counter;
						packet_counter <= packet_counter + 1;
						ip_checksum_acc <= (others => '0');
						ip_checksum_upper_byte <= (others => '0');
						ip_checksum_byte_phase <= '0';
						udp_checksum_upper_byte <= (others => '0');
						udp_checksum_byte_phase <= '0';
						pseudo_header_sum := resize(unsigned(src_ip_address(31 downto 16)), 32)
							+ resize(unsigned(src_ip_address(15 downto 0)), 32)
							+ resize(unsigned(dst_ip_address(31 downto 16)), 32)
							+ resize(unsigned(dst_ip_address(15 downto 0)), 32)
							+ resize(to_unsigned(16#0011#, 16), 32)
							+ resize(to_unsigned(UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 16), 32);
						udp_checksum_acc <= pseudo_header_sum;
						udp_pseudo_header_sum <= pseudo_header_sum;
							s_SM_Ethernet <= s_PrepFrame;
						end if;

				when s_PrepFrame =>
					packet_wr_en <= '1';
					packet_wr_addr <= frame_write_index;
						if (frame_write_index < PACKET_HEADER_LENGTH) then
							header_data := get_header_byte(frame_write_index, src_mac_address, dst_mac_address, src_ip_address, dst_ip_address, src_udp_port, dst_udp_port, active_packet_counter, sample_counter);
							packet_wr_data <= header_data;
							if (frame_write_index = 0) then
								first_packet_byte <= header_data;
							end if;
						if ((frame_write_index >= MAC_HEADER_LENGTH) and (frame_write_index < MAC_HEADER_LENGTH + IP_HEADER_LENGTH)) then
							feed_checksum(ip_checksum_upper_byte, ip_checksum_byte_phase, ip_checksum_acc, header_data);
						end if;
						if (frame_write_index = PACKET_HEADER_LENGTH - 1) then
							
							--audio_payload_index <= 1;
							sample_rd_addr <= 1;
						end if;
					else
						packet_wr_data <= sample_rd_data;
						sample_rd_addr <= sample_rd_addr + 1;
						
						--audio_payload_index <= audio_payload_index + 1;
					end if;
					if (frame_write_index = PACKET_LENGTH - 1) then
						packet_wr_en <= '0';
						udp_checksum_acc <= udp_pseudo_header_sum;
						udp_checksum_upper_byte <= (others => '0');
						udp_checksum_byte_phase <= '0';
						udp_checksum_bytes_remaining <= UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH;
						if (UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH > 0) then
							udp_checksum_request_count <= 1;
						else
							udp_checksum_request_count <= 0;
						end if;
						udp_checksum_data_valid <= '0';
						packet_rd_addr <= MAC_HEADER_LENGTH + IP_HEADER_LENGTH;
						s_SM_Ethernet <= s_CalcUdpChecksum;
					else
						frame_write_index <= frame_write_index + 1;
					end if;

				when s_CalcUdpChecksum =>
					if (udp_checksum_data_valid = '1') then
						feed_checksum(udp_checksum_upper_byte, udp_checksum_byte_phase, udp_checksum_acc, packet_rd_data);
						if (udp_checksum_bytes_remaining > 0) then
							udp_checksum_bytes_remaining <= udp_checksum_bytes_remaining - 1;
						end if;
					end if;

					if ((udp_checksum_data_valid = '1') and (udp_checksum_bytes_remaining = 1)) then
						udp_checksum_data_valid <= '0';
						s_SM_Ethernet <= s_FinalizeChecksum;
					elsif ((udp_checksum_data_valid = '0') and (udp_checksum_bytes_remaining = 0)) then
						s_SM_Ethernet <= s_FinalizeChecksum;
					else
						if (udp_checksum_data_valid = '0') then
							if (udp_checksum_bytes_remaining > 0) then
								udp_checksum_data_valid <= '1';
							end if;
						end if;
						if (udp_checksum_request_count < (UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH)) then
							packet_rd_addr <= packet_rd_addr + 1;
							udp_checksum_request_count <= udp_checksum_request_count + 1;
						end if;
					end if;

				when s_FinalizeChecksum =>
					ip_checksum_value <= finalize_checksum(ip_checksum_acc);
					udp_checksum_value <= finalize_checksum(udp_checksum_acc);
					s_SM_Ethernet <= s_WriteChecksum;

				when s_WriteChecksum =>
					case checksum_write_index is
						when 0 =>
							packet_wr_en <= '1';
							packet_wr_addr <= MAC_HEADER_LENGTH + 10;
							packet_wr_data <= ip_checksum_value(15 downto 8);
							checksum_write_index <= 1;
						when 1 =>
							packet_wr_en <= '1';
							packet_wr_addr <= MAC_HEADER_LENGTH + 11;
							packet_wr_data <= ip_checksum_value(7 downto 0);
							checksum_write_index <= 2;
						when 2 =>
							packet_wr_en <= '1';
							packet_wr_addr <= MAC_HEADER_LENGTH + IP_HEADER_LENGTH + 6;
							packet_wr_data <= udp_checksum_value(15 downto 8);
							checksum_write_index <= 3;
						when 3 =>
							packet_wr_en <= '1';
							packet_wr_addr <= MAC_HEADER_LENGTH + IP_HEADER_LENGTH + 7;
							packet_wr_data <= udp_checksum_value(7 downto 0);
							checksum_write_index <= 4;
							when others =>
								first_tx_byte_pending <= '1';
								if (PACKET_LENGTH > 0) then
									tx_data <= first_packet_byte;
								else
									tx_data <= (others => '0');
								end if;
								if (PACKET_LENGTH > 1) then
									packet_rd_addr <= 1;
									tx_read_pointer <= 2;
									tx_bytes_remaining <= PACKET_LENGTH - 1;
								else
									packet_rd_addr <= 0;
									tx_read_pointer <= 0;
									tx_bytes_remaining <= 0;
								end if;
								prime_wait <= 0;
								s_SM_Ethernet <= s_PrimeTx;
						end case;

				when s_PrimeTx =>
					tx_enable <= '0';
					if (prime_wait = 0) then
						-- allow one cycle for packet_rd_data to capture the next byte from RAM
						prime_wait <= 1;
							else
								if (tx_busy = '0') then
									tx_enable <= '1';
									s_SM_Ethernet <= s_Transmit;
								end if;
							end if;

				when s_Transmit =>
					tx_enable <= '1';
					if (tx_byte_sent = '1') then
						if (tx_bytes_remaining = 0) then
							s_SM_Ethernet <= s_End;
						elsif (first_tx_byte_pending = '1') then
							first_tx_byte_pending <= '0';
						else
							tx_data <= packet_rd_data;
							if (tx_read_pointer < PACKET_LENGTH) then
								packet_rd_addr <= tx_read_pointer;
								tx_read_pointer <= tx_read_pointer + 1;
							end if;
							tx_bytes_remaining <= tx_bytes_remaining - 1;
						end if;
					end if;

				when s_End =>
					tx_enable <= '0';
					tx_data <= (others => '0');
					buffer_locked <= '0';
					first_tx_byte_pending <= '0';
					s_SM_Ethernet <= s_Idle;
			end case;
		end if;
	end process main_proc;
end Behavioral;
