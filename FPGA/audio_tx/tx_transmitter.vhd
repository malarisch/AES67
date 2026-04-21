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

entity tx_transmitter is
    generic
    (
        samples_per_channel_depth : integer := 48; -- number of samples per channel to buffer
        global_channel_count : integer := 16; -- number of channels to buffer
        bytes_per_sample : integer := 3 -- number of bytes per sample (e.g., 3 for 24-bit audio)
    );
	port
	(
        sys_clk                    : in std_logic;
		src_mac_address		: in std_logic_vector(47 downto 0);
		src_ip_address			: in std_logic_vector(31 downto 0);
		dst_ip_address			: in std_logic_vector(31 downto 0);
		tx_clk					: in std_logic;
		tx_busy					: in std_logic;
		tx_byte_sent			: in std_logic;
		
		
		sample_counter			: in std_logic_vector(31 downto 0) := (others => '0');
		sequence_id_in			: in unsigned(15 downto 0) := (others => '0');
		
		tx_enable				: out std_logic := '0';  -- TX valid
		tx_data					: out std_logic_vector(7 downto 0) := (others => '0'); -- data-octet
		
        channel_count_i                 : in std_logic_vector(7 downto 0) := (others => '0');
        samples_per_packet_per_channel_i : in std_logic_vector(7 downto 0) := (others => '0');
        sample_buffer_tx_start_addr_i   : in std_logic_vector(15 downto 0) := (others => '0');

        sample_ram_read_addr_o         : out std_logic_vector(15 downto 0) := (others => '0');
        sample_ram_data_in_i          : in std_logic_vector(7 downto 0);

        -- max 8 channels per stream
        ch_ids_i                        : in std_logic_vector(63 downto 0) := (others => '0');
        ssrc_i                       : in std_logic_vector(31 downto 0) := (others => '0');

        start_i : in std_logic := '0';
		
		tx_req_o				: out std_logic := '0';
		tx_allow_i				: in std_logic := '0'

	);
end entity;


architecture Behavioral of tx_transmitter is

	constant MAC_HEADER_LENGTH			: integer := 14;
	constant IP_HEADER_LENGTH			: integer := 5 * (32 / 8); -- Header length always 20 bytes (5 * 32 bit words)
	constant AUDIO_START_SIGNAL	: integer := 8; -- 8 bytes at the beginning of the UDP payload reserved for RTP header (version, payload type, packet counter, sample counter, ssrc)
	constant UDP_HEADER_LENGTH		: integer := 12;
	-- calc dynamically constant UDP_PAYLOAD_LENGTH	: integer := AUDIO_START_SIGNAL + AUDIO_BUFFER_LENGTH; -- 8 start-bytes + x bytes for audio
	-- calc dynammically constant PACKET_LENGTH			: integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH;
	constant PACKET_HEADER_LENGTH	: integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH + AUDIO_START_SIGNAL;
	constant AUDIO_BUFFER_LENGTH	: integer := samples_per_channel_depth * global_channel_count * bytes_per_sample * 2;

    signal PACKET_LENGTH : integer := 0;
    signal PACKET_LENGTH_MINUS_1 : integer := 0; -- Pre-computed to break comparison critical path
    signal UDP_PAYLOAD_LENGTH : integer := 0;
    -- Pipeline registers for audio_length multiplication (breaks channel_count_o critical path)
    signal audio_samples_x_channels : integer range 0 to 65535 := 0; -- samples * channels (Stage 1)
    signal audio_data_length : integer range 0 to 65535 := 0;        -- (samples * channels) * bytes (Stage 2)
	-- Types
	type t_SM_Ethernet is (s_Idle, s_waitForAllow, s_Transmit, s_End, s_WaitDeassert);
    type t_SM_AssemblePacket is (s_A_Idle, s_A_CalcAudioLen, s_A_CalcValues, s_A_CalcChecksum, s_A_PrepFrame, s_A_Payload, s_A_WaitAckDone, s_CalcUdpChecksum, s_FinalizeChecksum, s_WriteChecksum);
	type t_packet_ram is array (0 to 1518) of std_logic_vector(7 downto 0);
    
    signal SM_AssemblePacket    : t_SM_AssemblePacket := s_A_Idle;
	
    signal read_base : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;
	-- Helper declarations
	function get_header_byte(
		idx				: integer;
		src_mac		: std_logic_vector(47 downto 0);
		src_ip		: std_logic_vector(31 downto 0);
		dst_ip		: std_logic_vector(31 downto 0);
		packet_ctr	: unsigned(15 downto 0);
		sample_ctr	: std_logic_vector(31 downto 0);
        samples_per_ch_per_pkt : std_logic_vector(7 downto 0);
        channel_count : std_logic_vector(7 downto 0);
        ssrc: std_logic_vector(31 downto 0);
        total_length : std_logic_vector(15 downto 0);
        udp_length : std_logic_vector(15 downto 0)
	) return std_logic_vector is
		--constant total_length	: std_logic_vector(15 downto 0) := std_logic_vector(to_unsigned(IP_HEADER_LENGTH + UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 16));
		--constant udp_length	: std_logic_vector(15 downto 0) := std_logic_vector(to_unsigned(UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 16));

	begin
		case idx is
		
			-- ethernet
			when 0	=> return x"01";
			when 1	=> return x"00";
			when 2	=> return x"5E";
			when 3	=> return b"0" & dst_ip(22 downto 16);
			when 4	=> return dst_ip(15 downto 8);
			when 5	=> return dst_ip(7 downto 0);
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
			when 18=> return STD_LOGIC_VECTOR(packet_ctr(15 downto 8));
			when 19=> return STD_LOGIC_VECTOR(packet_ctr(7 downto 0));
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
			when 34=> return x"13";
			when 35=> return x"8c";
			when 36=> return x"13";
			when 37=> return x"8c";
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
			when 44=> return STD_LOGIC_VECTOR(packet_ctr(15 downto 8));
			when 45=> return STD_LOGIC_VECTOR(packet_ctr(7 downto 0));

			-- sample counter (4 bytes, big-endian)
			
			when  46=> return sample_ctr(31 downto 24);
			when  47=> return sample_ctr(23 downto 16);
			when  48=> return sample_ctr(15 downto 8);
			when  49=> return sample_ctr(7 downto 0);
			-- ssrc
			when 50=> return ssrc(31 downto 24);
			when 51=> return ssrc(23 downto 16);
			when 52=> return ssrc(15 downto 8);
			when 53=> return ssrc(7 downto 0);

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

	-- Double-buffered (ping-pong) sample RAMs
	signal packet_ram		: t_packet_ram := (others => (others => '0'));



	signal packet_wr_en		: std_logic := '0';
	signal packet_wr_addr	: integer range 0 to 1518 - 1 := 0;
	signal packet_wr_data	: std_logic_vector(7 downto 0) := (others => '0');
	signal first_packet_byte	: std_logic_vector(7 downto 0) := (others => '0');

	attribute ram_style : string;

	attribute ram_style of packet_ram : signal is "block";



	-- True dual-port RAM signals: Port A (sys_clk) for write + checksum read, Port B (tx_clk) for TX read
	signal asm_rd_addr		: integer range 0 to 1518 - 1 := 0;
	signal asm_rd_data		: std_logic_vector(7 downto 0) := (others => '0'); -- RAM output register
	signal asm_rd_data_r	: std_logic_vector(7 downto 0) := (others => '0'); -- Pipeline register for timing
	signal tx_rd_addr		: integer range 0 to 1518 - 1 := 0;
	signal tx_rd_data		: std_logic_vector(7 downto 0) := (others => '0');

	signal s_SM_Ethernet	: t_SM_Ethernet := s_Idle;
	signal frame_write_index	: integer range 0 to 1518 := 0;
	signal audio_payload_index	: integer range 0 to samples_per_channel_depth * global_channel_count * bytes_per_sample := 0;
	signal checksum_write_index	: integer range 0 to 4 := 0;
	signal prime_wait		: integer range 0 to 2 := 0;
	signal tx_bytes_remaining	: integer range 0 to 1518 := 0;
	signal tx_read_pointer	: integer range 0 to 1518 := 0;

	
	signal ip_checksum_acc	: unsigned(31 downto 0) := (others => '0');
	signal ip_checksum_upper_byte	: std_logic_vector(7 downto 0) := (others => '0');
	signal ip_checksum_byte_phase	: std_logic := '0';
	signal ip_checksum_value	: std_logic_vector(15 downto 0) := (others => '0');

	signal udp_checksum_acc	: unsigned(31 downto 0) := (others => '0');
	signal udp_checksum_upper_byte	: std_logic_vector(7 downto 0) := (others => '0');
	signal udp_checksum_byte_phase	: std_logic := '0';
	signal udp_checksum_value	: std_logic_vector(15 downto 0) := (others => '0');
	signal udp_pseudo_header_sum	: unsigned(31 downto 0) := (others => '0');
	signal udp_checksum_bytes_remaining	: integer range 0 to 1500 := 0;
	signal udp_checksum_request_count	: integer range 0 to 1500 := 0;
	signal udp_checksum_data_valid	: std_logic := '0';
	signal udp_checksum_data_valid_d	: std_logic := '0'; -- Delayed by 1 cycle for RAM pipeline

    signal start_i_sync1 : std_logic := '0';
    signal channel_counter: integer range 0 to 7 := 0;
    signal byte_counter : integer range 0 to 2 := 0;
    signal sample_index : integer range 0 to samples_per_channel_depth - 1 := 0;

    -- ============================================================
    -- Pipeline registers to break IP checksum critical path
    -- Original: 8-way addition in s_A_CalcValues (~12ns combinational)
    -- Solution: Pre-compute static parts, split into two stages
    -- ============================================================
    signal ip_checksum_partial1 : unsigned(31 downto 0) := (others => '0');  -- src_ip + 0x8011
    signal ip_checksum_partial2 : unsigned(31 downto 0) := (others => '0');  -- dst_ip

    -- ============================================================
    -- Pipeline registers to break sample_ram_read_addr critical path
    -- Original: ch_id mux + 2 multiplies + 3 adds (~10ns)
    -- Solution: Split into 2 stages:
    --   Stage 1: ch_id mux + sample_base computation → ch_id_reg, sample_base_addr
    --   Stage 2: Final address add + wrap → raw_addr_reg
    -- Total: 2 extra cycles latency (compensated by earlier pre-fetch)
    -- ============================================================
    signal ch_id_reg        : integer range 0 to 255 := 0;  -- Selected channel ID (Stage 1)
    signal sample_base_addr : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;  -- sample_index * stride
    signal byte_offset_reg  : integer range 0 to 2 := 0;    -- byte offset within sample
    signal raw_addr_reg     : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;  -- Final address (Stage 2)
    
    -- Look-ahead counters for address pipeline (compute NEXT values one cycle early)
    signal next_channel_counter : integer range 0 to 7 := 0;
    signal next_byte_counter    : integer range 0 to 2 := 0;
    signal next_sample_index    : integer range 0 to samples_per_channel_depth - 1 := 0;

    signal tx_ack: std_logic := '0';
    signal tx_ack_sync1: std_logic := '0';
    signal tx_ack_sync2: std_logic := '0';

    signal tx_frame_start    : std_logic := '0';
    signal tx_frame_start_sync1 : std_logic := '0';
    signal tx_frame_start_sync2 : std_logic := '0';
	signal read_addr : integer range 0 to 1532 := 0;
	signal tx_done_reg : std_logic := '0';
begin

	-- True dual-port packet RAM
	-- Port A (sys_clk): write + checksum read (with pipeline register for timing)
	packet_ram_port_a : process(sys_clk)
	begin
		if rising_edge(sys_clk) then
			if (packet_wr_en = '1') then
				packet_ram(packet_wr_addr) <= packet_wr_data;
			end if;
			asm_rd_data <= packet_ram(asm_rd_addr);
			asm_rd_data_r <= asm_rd_data;
		end if;
	end process packet_ram_port_a;

	-- Port B (tx_clk): TX read
	-- Synchronous read: address register -> RAM -> registered output (tx_data)
	packet_ram_port_b : process(tx_clk)
	begin
		if rising_edge(tx_clk) then
			if (tx_byte_sent = '1') then
					tx_data <= packet_ram(read_addr + 1);
			else 
					tx_data <= packet_ram(read_addr);
			end if;
			if tx_ack = '1' and tx_allow_i = '1' then
				tx_enable <= '1';
				if read_addr < PACKET_LENGTH - 1  then
					tx_done_reg <= '0';
					if tx_byte_sent = '1' then
						read_addr <= read_addr + 1;
					end if;
				else 
					tx_enable <= '0';
					tx_done_reg <= '1';
				end if;
				
			else
				tx_enable <= '0';
				read_addr <= 0;
			end if;
			
		end if;
	end process packet_ram_port_b;

    sys_clock_process : process(sys_clk)
    		variable header_data		: std_logic_vector(7 downto 0);
		variable pseudo_header_sum	: unsigned(31 downto 0);
		variable rd_offset		: integer;
		variable raw_addr		: integer;
		variable ch_id			: integer;
    begin
        if rising_edge(sys_clk) then
            packet_wr_en <= '0'; -- default: no write (overridden when needed)

            -- sync start_i to sys_clk domain
            start_i_sync1 <= start_i;

            tx_ack_sync1 <= tx_ack;
            tx_ack_sync2 <= tx_ack_sync1;
            case SM_AssemblePacket is
                when s_A_Idle =>
                    if (start_i_sync1 = '1') then
                        SM_AssemblePacket <= s_A_CalcAudioLen;
                        -- Pipeline Stage 1: compute samples * channels (breaks critical path)
                        audio_samples_x_channels <= to_integer(unsigned(samples_per_packet_per_channel_i)) * to_integer(unsigned(channel_count_i));
                    end if;
                -- Pipeline Stage 2: multiply by bytes_per_sample and compute final lengths
                when s_A_CalcAudioLen =>
                    audio_data_length <= audio_samples_x_channels * bytes_per_sample;
                    UDP_PAYLOAD_LENGTH <= AUDIO_START_SIGNAL + audio_samples_x_channels * bytes_per_sample;
                    PACKET_LENGTH <= MAC_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH + AUDIO_START_SIGNAL + audio_samples_x_channels * bytes_per_sample;
                    -- Pre-compute PACKET_LENGTH - 1 to remove subtraction from critical comparison path
                    PACKET_LENGTH_MINUS_1 <= MAC_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH + AUDIO_START_SIGNAL + audio_samples_x_channels * bytes_per_sample - 1;
                    SM_AssemblePacket <= s_A_CalcValues;
                when s_A_CalcValues =>
						-- Compute read base: go back samples_per_packet sample periods from write pointer
						rd_offset := to_integer(unsigned(samples_per_packet_per_channel_i)) * global_channel_count * bytes_per_sample;
						if to_integer(unsigned(sample_buffer_tx_start_addr_i)) >= rd_offset then
							read_base <= to_integer(unsigned(sample_buffer_tx_start_addr_i)) - rd_offset;
						else
							read_base <= to_integer(unsigned(sample_buffer_tx_start_addr_i)) - rd_offset + AUDIO_BUFFER_LENGTH;
						end if;

						frame_write_index <= 0;
						audio_payload_index <= 0;
						checksum_write_index <= 0;


						-- ===== IP CHECKSUM PIPELINE STAGE 1 =====
						-- Split 8-way addition into two stages to meet timing
						-- Stage 1: Compute partial sums (src_ip + 0x8011) and (dst_ip)
						ip_checksum_partial1 <= resize(to_unsigned(16#8011#, 16), 32) +
							resize(unsigned(src_ip_address(31 downto 16)), 32) +
							resize(unsigned(src_ip_address(15 downto 0)), 32);
						ip_checksum_partial2 <= 
							resize(unsigned(dst_ip_address(31 downto 16)), 32) +
							resize(unsigned(dst_ip_address(15 downto 0)), 32);

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


                        SM_AssemblePacket <= s_A_CalcChecksum;

				-- ===== IP CHECKSUM PIPELINE STAGE 2 =====
				-- Final sum using registered partial sums from Stage 1
				when s_A_CalcChecksum =>
					ip_checksum_acc <= ip_checksum_partial1 + ip_checksum_partial2 +
						resize(to_unsigned(16#4500#, 16), 32) +
						resize(to_unsigned(IP_HEADER_LENGTH + UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 16), 32) +
						resize(sequence_id_in, 32);  -- already incremented in s_A_CalcValues
					SM_AssemblePacket <= s_A_PrepFrame;
                        
                when s_A_PrepFrame =>

                    packet_wr_en <= '1';
					packet_wr_addr <= frame_write_index;
					if (frame_write_index < PACKET_HEADER_LENGTH) then
							
                        header_data := get_header_byte(frame_write_index, src_mac_address, src_ip_address, 
                            dst_ip_address, sequence_id_in, sample_counter, samples_per_packet_per_channel_i,
                            channel_count_i, ssrc_i, std_logic_vector(to_unsigned(IP_HEADER_LENGTH + UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 16)), 
                            std_logic_vector(to_unsigned(UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 16)));

						packet_wr_data <= header_data;
						if (frame_write_index = 0) then
							first_packet_byte <= header_data;
						end if;
						
						-- ===== ADDRESS PIPELINE PRE-FETCH (4 stages to account for pipeline latency) =====
						-- Need 4 pre-fetch cycles: Stage 1 (ch_id/base), Stage 2 (addr compute),
						-- Stage 3 (addr output to RAM port), Stage 4 (RAM registered read)
						if (frame_write_index = PACKET_HEADER_LENGTH - 4) then
							-- Pipeline Stage 1 setup: Initialize for first byte (MSB of ch0, sample 0)
							ch_id_reg <= to_integer(unsigned(ch_ids_i(63 downto 56)));
							sample_base_addr <= 0;  -- sample_index=0, so base=0
							byte_offset_reg <= bytes_per_sample - 1;  -- MSB first
						end if;
						if (frame_write_index = PACKET_HEADER_LENGTH - 3) then
							-- Pipeline Stage 2: Compute address for first byte
							raw_addr := read_base + sample_base_addr + ch_id_reg * bytes_per_sample + byte_offset_reg;
							if raw_addr >= AUDIO_BUFFER_LENGTH then
								raw_addr := raw_addr - AUDIO_BUFFER_LENGTH;
							end if;
							raw_addr_reg <= raw_addr;
							-- Setup Stage 1 for second byte
							ch_id_reg <= to_integer(unsigned(ch_ids_i(63 downto 56)));
							sample_base_addr <= 0;
							byte_offset_reg <= bytes_per_sample - 2;  -- middle byte
						end if;
						if (frame_write_index = PACKET_HEADER_LENGTH - 2) then
							-- Stage 3: Output first address to RAM
							sample_ram_read_addr_o <= std_logic_vector(to_unsigned(raw_addr_reg, 16));
							-- Pipeline Stage 2: Compute address for second byte
							raw_addr := read_base + sample_base_addr + ch_id_reg * bytes_per_sample + byte_offset_reg;
							if raw_addr >= AUDIO_BUFFER_LENGTH then
								raw_addr := raw_addr - AUDIO_BUFFER_LENGTH;
							end if;
							raw_addr_reg <= raw_addr;
							-- Setup Stage 1 for third byte
							ch_id_reg <= to_integer(unsigned(ch_ids_i(63 downto 56)));
							sample_base_addr <= 0;
							byte_offset_reg <= 0;  -- LSB byte
						end if;
						if (frame_write_index = PACKET_HEADER_LENGTH - 1) then
							-- Stage 4: RAM is now reading first byte; output second address
							sample_ram_read_addr_o <= std_logic_vector(to_unsigned(raw_addr_reg, 16));
							-- Pipeline Stage 2: Compute address for third byte (ch0 LSB)
							raw_addr := read_base + sample_base_addr + ch_id_reg * bytes_per_sample + byte_offset_reg;
							if raw_addr >= AUDIO_BUFFER_LENGTH then
								raw_addr := raw_addr - AUDIO_BUFFER_LENGTH;
							end if;
							raw_addr_reg <= raw_addr;
							-- Setup Stage 1 for fourth byte (ch1 MSB)
							ch_id_reg <= to_integer(unsigned(ch_ids_i(55 downto 48)));
							sample_base_addr <= 0;
							byte_offset_reg <= bytes_per_sample - 1;  -- MSB
							-- next_* at HEADER-1 feeds Stage 1 at HEADER, result appears at PI=4 (ch1 MID)
                            next_byte_counter <= 1;
                            next_channel_counter <= 1;
                            next_sample_index <= 0;
						end if;
					else
                        -- ===== PIPELINED AUDIO PAYLOAD ASSEMBLY =====
                        -- Pipeline: counter → ch_id_reg+base (Stage 1) → raw_addr_reg (Stage 2) → output
                        -- Each cycle: output data from 2 cycles ago, compute address for 2 cycles ahead
                        
						packet_wr_data <= sample_ram_data_in_i;
						
						-- Stage 2 OUTPUT: Address computed last cycle
						sample_ram_read_addr_o <= std_logic_vector(to_unsigned(raw_addr_reg, 16));
						
						-- Stage 2 COMPUTE: Use registered ch_id and base from Stage 1
						raw_addr := read_base + sample_base_addr + ch_id_reg * bytes_per_sample + byte_offset_reg;
						if raw_addr >= AUDIO_BUFFER_LENGTH then
							raw_addr := raw_addr - AUDIO_BUFFER_LENGTH;
						end if;
						raw_addr_reg <= raw_addr;

						-- Stage 1: Compute ch_id and base for look-ahead counters
						case next_channel_counter is
							when 0 => ch_id_reg <= to_integer(unsigned(ch_ids_i(63 downto 56)));
							when 1 => ch_id_reg <= to_integer(unsigned(ch_ids_i(55 downto 48)));
							when 2 => ch_id_reg <= to_integer(unsigned(ch_ids_i(47 downto 40)));
							when 3 => ch_id_reg <= to_integer(unsigned(ch_ids_i(39 downto 32)));
							when 4 => ch_id_reg <= to_integer(unsigned(ch_ids_i(31 downto 24)));
							when 5 => ch_id_reg <= to_integer(unsigned(ch_ids_i(23 downto 16)));
							when 6 => ch_id_reg <= to_integer(unsigned(ch_ids_i(15 downto 8)));
							when 7 => ch_id_reg <= to_integer(unsigned(ch_ids_i(7 downto 0)));
							when others => ch_id_reg <= 0;
						end case;
						sample_base_addr <= next_sample_index * global_channel_count * bytes_per_sample;
						byte_offset_reg <= bytes_per_sample - 1 - next_byte_counter;

						-- Update current counters (lagging behind look-ahead by 1)
						byte_counter <= next_byte_counter;
						channel_counter <= next_channel_counter;
						sample_index <= next_sample_index;
						
						-- Advance look-ahead counters
                        if (next_byte_counter < bytes_per_sample - 1) then
                            next_byte_counter <= next_byte_counter + 1;
                        else
                            next_byte_counter <= 0;
                            if (next_channel_counter < to_integer(unsigned(channel_count_i)) - 1) then
                                next_channel_counter <= next_channel_counter + 1;
                            else
                                next_channel_counter <= 0;
                                if (next_sample_index < to_integer(unsigned(samples_per_packet_per_channel_i)) - 1) then
                                    next_sample_index <= next_sample_index + 1;
                                else
                                    next_sample_index <= 0;
                                end if;
                            end if;
                        end if;

					end if;
					if (frame_write_index = PACKET_LENGTH_MINUS_1) then
						-- Note: packet_wr_en stays '1' this cycle so last byte is written to RAM.
						-- It returns to '0' next cycle via the process default.
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
						udp_checksum_data_valid_d <= '0'; -- Initialize delayed valid signal
						asm_rd_addr <= MAC_HEADER_LENGTH + IP_HEADER_LENGTH;
						SM_AssemblePacket <= s_CalcUdpChecksum;
					else
						frame_write_index <= frame_write_index + 1;
					end if;

                when s_CalcUdpChecksum =>
					-- Pipeline delay register for data_valid (matches asm_rd_data -> asm_rd_data_r latency)
					udp_checksum_data_valid_d <= udp_checksum_data_valid;
					
					-- Note: asm_rd_data_r has 2-cycle latency from asm_rd_addr
					-- udp_checksum_data_valid_d is delayed to match this pipeline
					if (udp_checksum_data_valid_d = '1') then
						feed_checksum(udp_checksum_upper_byte, udp_checksum_byte_phase, udp_checksum_acc, asm_rd_data_r);
						if (udp_checksum_bytes_remaining > 0) then
							udp_checksum_bytes_remaining <= udp_checksum_bytes_remaining - 1;
						end if;
					end if;

					if ((udp_checksum_data_valid_d = '1') and (udp_checksum_bytes_remaining = 1)) then
						udp_checksum_data_valid <= '0';
						SM_AssemblePacket <= s_FinalizeChecksum;
					elsif ((udp_checksum_data_valid = '0') and (udp_checksum_bytes_remaining = 0)) then
						SM_AssemblePacket <= s_FinalizeChecksum;
					else
						if (udp_checksum_data_valid = '0') then
							if (udp_checksum_bytes_remaining > 0) then
								udp_checksum_data_valid <= '1';
							end if;
						end if;
						if (udp_checksum_request_count < (UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH)) then
							asm_rd_addr <= asm_rd_addr + 1;
							udp_checksum_request_count <= udp_checksum_request_count + 1;
						end if;
					end if;

				when s_FinalizeChecksum =>
					ip_checksum_value <= finalize_checksum(ip_checksum_acc);
					udp_checksum_value <= finalize_checksum(udp_checksum_acc);
					SM_AssemblePacket <= s_WriteChecksum;

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
								SM_AssemblePacket <= s_A_Payload;
						end case;
                when s_A_Payload =>
                    tx_frame_start <= '1';
                    -- Phase 3: tx_process saw our request and acked
                    if (tx_ack_sync2 = '1') then
                        tx_frame_start <= '0'; -- deassert request
                        SM_AssemblePacket <= s_A_WaitAckDone;
                    end if;

                when s_A_WaitAckDone =>
                    -- Phase 5: wait for tx_process to confirm deassert by lowering ack
                    if (tx_ack_sync2 = '0') then
                        SM_AssemblePacket <= s_A_Idle;
                    end if;
            end case;
        end if;
    end process sys_clock_process;


	tx_process : process(tx_clk)
	begin
		if rising_edge(tx_clk) then
            tx_frame_start_sync1 <= tx_frame_start;
            tx_frame_start_sync2 <= tx_frame_start_sync1;


			case s_SM_Ethernet is
				when s_Idle =>
                    tx_ack <= '0';
					tx_req_o <= '0';
					if (tx_frame_start_sync2 = '1') then
						tx_ack <= '1';
						s_SM_Ethernet <= s_waitForAllow;
						tx_req_o <= '1'; -- signal packet ready for transmission
						
					end if;
				when s_waitForAllow =>
					if (tx_allow_i = '1') then
						s_SM_Ethernet <= s_Transmit;
					end if;
				when s_Transmit =>
					
					if (tx_done_reg = '1') then
						s_SM_Ethernet <= s_End;
					end if;
					
				when s_End =>
					s_SM_Ethernet <= s_WaitDeassert;
					tx_req_o <= '0'; 
				when s_WaitDeassert =>
					-- Phase 4: wait for sys_clock_process to deassert tx_frame_start
					if (tx_frame_start_sync2 = '0') then
						tx_ack <= '0'; -- signal completion
						s_SM_Ethernet <= s_Idle;
					end if;
			end case;
		end if;
	end process tx_process;
end Behavioral;