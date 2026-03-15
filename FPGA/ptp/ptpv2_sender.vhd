-- PTPv2 Sync/Delay_Resp/Announce UDP Packet Sender
-- Refactored: Dual-Port RAM architecture
-- Packet assembly in sys_clk domain, TX in tx_clk domain
--
-- based on udp_packet.vhd:
-- UDP Packet Sender
-- (c) 2025 Dr.-Ing. Christian Noeding
-- christian@noeding-online.de
-- Released under GNU General Public License v3
-- Source: https://www.github.com/xn--nding-jua/AES50_Transmitter

library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity ptpv2_sender is
	port
	(
		sys_clk					: in std_logic;
		src_mac_address		: in std_logic_vector(47 downto 0);
		src_ip_address			: in std_logic_vector(31 downto 0);
		frame_start				: in std_logic;
		tx_clk					: in std_logic;
		tx_busy					: in std_logic;
		tx_byte_sent			: in std_logic;
		sequence_id			: in unsigned(15 downto 0);

		timestamp_seconds_i : in unsigned(47 downto 0);
		timestamp_nanoseconds_i : in unsigned(31 downto 0);

		request_port_identity : in std_logic_vector(79 downto 0);

		tx_enable				: out std_logic := '0';
		tx_data					: out std_logic_vector(7 downto 0) := (others => '0');

		message_type_i : in std_logic_vector(3 downto 0) := "0000";
		tx_ready_o			: out std_logic := '0';
		ptp_time_source_i : in std_logic_vector(7 downto 0);
		ptp_log_interval_i : in std_logic_vector(7 downto 0);

		tx_allow_req_o : out std_logic := '0';
		tx_allow_i : in std_logic
	);
end entity;

architecture Behavioral of ptpv2_sender is
	-- Constants
	constant MAC_HEADER_LENGTH			: integer := 14;
	constant IP_HEADER_LENGTH			: integer := 20;
	constant UDP_HEADER_LENGTH			: integer := 8;
	constant MAX_UDP_PAYLOAD_LENGTH	: integer := 64;
	constant MAX_PACKET_LENGTH			: integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH + MAX_UDP_PAYLOAD_LENGTH;
	-- RAM depth must be power-of-2 for M9K block RAM inference (106 bytes too small otherwise)
	constant RAM_DEPTH					: integer := 128;

	-- Message type enumeration
	type t_packet_type is (t_Sync, t_Delay_Req, t_Follow_Up, t_Delay_Resp, t_Announce, t_Pdelay_Req, t_Pdelay_Resp, t_Pdelay_Follow_Up);

	function get_message_type(message_type : std_logic_vector(3 downto 0)) return t_packet_type is
	begin
		case message_type is
			when x"0" => return t_Sync;
			when x"1" => return t_Delay_Req;
			when x"2" => return t_Pdelay_Req;
			when x"3" => return t_Pdelay_Resp;
			when x"8" => return t_Follow_Up;
			when x"9" => return t_Delay_Resp;
			when x"A" => return t_Pdelay_Follow_Up;
			when x"B" => return t_Announce;
			when others => return t_Sync;
		end case;
	end function;

	function get_udp_payload_length(msg_type : t_packet_type) return integer is
	begin
		case msg_type is
			when t_Sync           => return 44;
			when t_Delay_Req      => return 44;
			when t_Follow_Up      => return 44;
			when t_Delay_Resp     => return 54;
			when t_Announce        => return 64;
			when t_Pdelay_Req     => return 54;
			when t_Pdelay_Resp    => return 54;
			when t_Pdelay_Follow_Up => return 54;
			when others           => return 44;
		end case;
	end function;

	function is_event_message(msg_type : t_packet_type) return boolean is
	begin
		case msg_type is
			when t_Sync | t_Delay_Req | t_Pdelay_Req | t_Pdelay_Resp => return true;
			when others => return false;
		end case;
	end function;

	function is_pdelay_message(msg_type : t_packet_type) return boolean is
	begin
		case msg_type is
			when t_Pdelay_Req | t_Pdelay_Resp | t_Pdelay_Follow_Up => return true;
			when others => return false;
		end case;
	end function;

	function get_control_field(msg_type : t_packet_type) return std_logic_vector is
	begin
		case msg_type is
			when t_Sync       => return x"00";
			when t_Delay_Req  => return x"01";
			when t_Follow_Up  => return x"02";
			when t_Delay_Resp => return x"03";
			when others       => return x"05";
		end case;
	end function;

	-- Header byte generator function (replaces the huge mux)
	function get_header_byte(
		idx            : integer;
		src_mac        : std_logic_vector(47 downto 0);
		src_ip         : std_logic_vector(31 downto 0);
		msg_type       : t_packet_type;
		msg_type_raw   : std_logic_vector(3 downto 0);
		pkt_cnt        : unsigned(15 downto 0);
		seq_id         : unsigned(15 downto 0);
		ts_sec         : unsigned(47 downto 0);
		ts_nsec        : unsigned(31 downto 0);
		req_port_id    : std_logic_vector(79 downto 0);
		udp_payload_len : integer;
		log_interval   : std_logic_vector(7 downto 0);
		time_source    : std_logic_vector(7 downto 0)
	) return std_logic_vector is
		variable total_length : std_logic_vector(15 downto 0);
		variable udp_length   : std_logic_vector(15 downto 0);
		variable port_byte    : std_logic_vector(7 downto 0);
		variable dst_last     : std_logic_vector(7 downto 0);
		variable clock_id     : std_logic_vector(63 downto 0);
	begin
		total_length := std_logic_vector(to_unsigned(IP_HEADER_LENGTH + UDP_HEADER_LENGTH + udp_payload_len, 16));
		udp_length := std_logic_vector(to_unsigned(UDP_HEADER_LENGTH + udp_payload_len, 16));

		-- Port byte: 0x3f for event (319), 0x40 for general (320)
		if is_event_message(msg_type) then
			port_byte := x"3f";
		else
			port_byte := x"40";
		end if;

		-- Destination MAC/IP last byte: 0x6b for pdelay, 0x81 for PTP
		if is_pdelay_message(msg_type) then
			dst_last := x"6b";
		else
			dst_last := x"81";
		end if;

		-- EUI-64 clock identity from MAC
		clock_id := (src_mac(47 downto 40) xor x"02") & src_mac(39 downto 32) & src_mac(31 downto 24)
		            & x"FF" & x"FE"
		            & src_mac(23 downto 16) & src_mac(15 downto 8) & src_mac(7 downto 0);

		case idx is
			-- MAC header (14 bytes)
			when 0  => return x"01";
			when 1  => return x"00";
			when 2  => return x"5e";
			when 3  => return x"00";
			when 4  => return x"01";
			when 5  => return dst_last;
			when 6  => return src_mac(47 downto 40);
			when 7  => return src_mac(39 downto 32);
			when 8  => return src_mac(31 downto 24);
			when 9  => return src_mac(23 downto 16);
			when 10 => return src_mac(15 downto 8);
			when 11 => return src_mac(7 downto 0);
			when 12 => return x"08";
			when 13 => return x"00";

			-- IP header (20 bytes)
			when 14 => return x"45";
			when 15 => return b"10111000"; -- DSCP=0x2e (Expedited Forwarding)
			when 16 => return total_length(15 downto 8);
			when 17 => return total_length(7 downto 0);
			when 18 => return std_logic_vector(pkt_cnt(15 downto 8));
			when 19 => return std_logic_vector(pkt_cnt(7 downto 0));
			when 20 => return x"00";
			when 21 => return x"00";
			when 22 => return x"80"; -- TTL=128
			when 23 => return x"11"; -- UDP
			when 24 => return x"00"; -- IP checksum placeholder
			when 25 => return x"00";
			when 26 => return src_ip(31 downto 24);
			when 27 => return src_ip(23 downto 16);
			when 28 => return src_ip(15 downto 8);
			when 29 => return src_ip(7 downto 0);
			when 30 => return x"E0"; -- 224
			when 31 => return x"00"; -- 0
			when 32 => return x"01"; -- 1
			when 33 => return dst_last;

			-- UDP header (8 bytes)
			when 34 => return x"01";
			when 35 => return port_byte;
			when 36 => return x"01";
			when 37 => return port_byte;
			when 38 => return udp_length(15 downto 8);
			when 39 => return udp_length(7 downto 0);
			when 40 => return x"00"; -- UDP checksum placeholder
			when 41 => return x"00";

			-- PTP header (34 bytes, offsets 42..75)
			when 42 => return x"0" & msg_type_raw; -- majorSdoId + messageType
			when 43 => return x"02"; -- PTP version 2
			when 44 => return std_logic_vector(to_unsigned(udp_payload_len, 16)(15 downto 8));
			when 45 => return std_logic_vector(to_unsigned(udp_payload_len, 16)(7 downto 0));
			when 46 => return x"00"; -- domainNumber
			when 47 => return x"00"; -- minorSdoId
			when 48 => return x"02"; -- Flags1: TwoStep
			when 49 => return x"00"; -- Flags2
			when 50 => return x"00"; -- correctionField (8 bytes)
			when 51 => return x"00";
			when 52 => return x"00";
			when 53 => return x"00";
			when 54 => return x"00";
			when 55 => return x"00";
			when 56 => return x"00";
			when 57 => return x"00";
			when 58 => return x"00"; -- messageTypeSpecific (4 bytes)
			when 59 => return x"00";
			when 60 => return x"00";
			when 61 => return x"00";
			when 62 => return clock_id(63 downto 56); -- clockIdentity (8 bytes)
			when 63 => return clock_id(55 downto 48);
			when 64 => return clock_id(47 downto 40);
			when 65 => return clock_id(39 downto 32);
			when 66 => return clock_id(31 downto 24);
			when 67 => return clock_id(23 downto 16);
			when 68 => return clock_id(15 downto 8);
			when 69 => return clock_id(7 downto 0);
			when 70 => return x"00"; -- sourcePortId
			when 71 => return x"01";
			when 72 => return std_logic_vector(seq_id(15 downto 8));
			when 73 => return std_logic_vector(seq_id(7 downto 0));
			when 74 => return get_control_field(msg_type);
			when 75 => return log_interval;

			-- Timestamp (10 bytes, offsets 76..85)
			when 76 => return std_logic_vector(ts_sec(47 downto 40));
			when 77 => return std_logic_vector(ts_sec(39 downto 32));
			when 78 => return std_logic_vector(ts_sec(31 downto 24));
			when 79 => return std_logic_vector(ts_sec(23 downto 16));
			when 80 => return std_logic_vector(ts_sec(15 downto 8));
			when 81 => return std_logic_vector(ts_sec(7 downto 0));
			when 82 => return std_logic_vector(ts_nsec(31 downto 24));
			when 83 => return std_logic_vector(ts_nsec(23 downto 16));
			when 84 => return std_logic_vector(ts_nsec(15 downto 8));
			when 85 => return std_logic_vector(ts_nsec(7 downto 0));

			-- Message-type-specific suffix (offsets 86+)
			when 86 =>
				if (msg_type = t_Delay_Resp) or (msg_type = t_Pdelay_Resp) then
					return req_port_id(79 downto 72);
				elsif (msg_type = t_Announce) then
					return x"00"; -- currentUtcOffset MSB
				else
					return x"00";
				end if;
			when 87 =>
				if (msg_type = t_Delay_Resp) or (msg_type = t_Pdelay_Resp) then
					return req_port_id(71 downto 64);
				elsif (msg_type = t_Announce) then
					return x"25"; -- currentUtcOffset LSB
				else
					return x"00";
				end if;
			when 88 =>
				if (msg_type = t_Delay_Resp) or (msg_type = t_Pdelay_Resp) then
					return req_port_id(63 downto 56);
				elsif (msg_type = t_Announce) then
					return x"00"; -- reserved
				else
					return x"00";
				end if;
			when 89 =>
				if (msg_type = t_Delay_Resp) or (msg_type = t_Pdelay_Resp) then
					return req_port_id(55 downto 48);
				elsif (msg_type = t_Announce) then
					return x"80"; -- grandmasterPriority1
				else
					return x"00";
				end if;
			when 90 =>
				if (msg_type = t_Delay_Resp) or (msg_type = t_Pdelay_Resp) then
					return req_port_id(47 downto 40);
				elsif (msg_type = t_Announce) then
					return x"F8"; -- clockQuality.clockClass
				else
					return x"00";
				end if;
			when 91 =>
				if (msg_type = t_Delay_Resp) or (msg_type = t_Pdelay_Resp) then
					return req_port_id(39 downto 32);
				elsif (msg_type = t_Announce) then
					return x"FE"; -- clockQuality.clockAccuracy
				else
					return x"00";
				end if;
			when 92 =>
				if (msg_type = t_Delay_Resp) or (msg_type = t_Pdelay_Resp) then
					return req_port_id(31 downto 24);
				elsif (msg_type = t_Announce) then
					return x"FF"; -- offsetScaledLogVariance MSB
				else
					return x"00";
				end if;
			when 93 =>
				if (msg_type = t_Delay_Resp) or (msg_type = t_Pdelay_Resp) then
					return req_port_id(23 downto 16);
				elsif (msg_type = t_Announce) then
					return x"FF"; -- offsetScaledLogVariance LSB
				else
					return x"00";
				end if;
			when 94 =>
				if (msg_type = t_Delay_Resp) or (msg_type = t_Pdelay_Resp) then
					return req_port_id(15 downto 8);
				elsif (msg_type = t_Announce) then
					return x"80"; -- grandmasterPriority2
				else
					return x"00";
				end if;
			when 95 =>
				if (msg_type = t_Delay_Resp) or (msg_type = t_Pdelay_Resp) then
					return req_port_id(7 downto 0);
				elsif (msg_type = t_Announce) then
					return clock_id(63 downto 56); -- grandmasterIdentity
				else
					return x"00";
				end if;
			-- Announce-only fields (offsets 96..105)
			when 96 =>
				if (msg_type = t_Announce) then return clock_id(55 downto 48);
				else return x"00"; end if;
			when 97 =>
				if (msg_type = t_Announce) then return clock_id(47 downto 40);
				else return x"00"; end if;
			when 98 =>
				if (msg_type = t_Announce) then return clock_id(39 downto 32);
				else return x"00"; end if;
			when 99 =>
				if (msg_type = t_Announce) then return clock_id(31 downto 24);
				else return x"00"; end if;
			when 100 =>
				if (msg_type = t_Announce) then return clock_id(23 downto 16);
				else return x"00"; end if;
			when 101 =>
				if (msg_type = t_Announce) then return clock_id(15 downto 8);
				else return x"00"; end if;
			when 102 =>
				if (msg_type = t_Announce) then return clock_id(7 downto 0);
				else return x"00"; end if;
			when 103 =>
				if (msg_type = t_Announce) then return x"00"; -- stepsRemoved MSB
				else return x"00"; end if;
			when 104 =>
				if (msg_type = t_Announce) then return x"00"; -- stepsRemoved LSB
				else return x"00"; end if;
			when 105 =>
				if (msg_type = t_Announce) then return time_source;
				else return x"00"; end if;

			when others => return x"00";
		end case;
	end function;

	-- Checksum helper procedures/functions (from tx_transmitter pattern)
	procedure feed_checksum(
		signal upper_byte  : inout std_logic_vector(7 downto 0);
		signal byte_phase  : inout std_logic;
		signal accumulator : inout unsigned(31 downto 0);
		constant data_byte : std_logic_vector(7 downto 0)
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

	-- ============================================================
	-- Dual-Port Packet RAM
	-- ============================================================
	type t_packet_ram is array (0 to RAM_DEPTH - 1) of std_logic_vector(7 downto 0);
	signal packet_ram : t_packet_ram := (others => (others => '0'));

	attribute ram_style : string;
	attribute ram_style of packet_ram : signal is "block";

	-- Port A (sys_clk): write + checksum read
	signal packet_wr_en   : std_logic := '0';
	signal packet_wr_addr : integer range 0 to RAM_DEPTH - 1 := 0;
	signal packet_wr_data : std_logic_vector(7 downto 0) := (others => '0');
	signal asm_rd_addr    : integer range 0 to RAM_DEPTH - 1 := 0;
	signal asm_rd_data    : std_logic_vector(7 downto 0) := (others => '0');
	signal asm_rd_data_r  : std_logic_vector(7 downto 0) := (others => '0'); -- Pipeline register

	-- Port B (tx_clk): TX read
	signal tx_rd_addr : integer range 0 to RAM_DEPTH - 1 := 0;
	signal tx_rd_data : std_logic_vector(7 downto 0) := (others => '0');

	-- First byte cache (avoids RAM read latency for byte 0)
	signal first_packet_byte : std_logic_vector(7 downto 0) := (others => '0');

	-- ============================================================
	-- Assembly state machine (sys_clk domain)
	-- ============================================================
	type t_SM_Assemble is (s_A_Idle, s_A_CalcIpChkPartial, s_A_CalcIpChkFinal, s_A_PrepFrame,
	                       s_A_CalcUdpChecksum, s_A_FinalizeChecksum, s_A_WriteChecksum,
	                       s_A_RequestTx, s_A_WaitAckDone);
	signal SM_Assemble : t_SM_Assemble := s_A_Idle;

	signal frame_write_index    : integer range 0 to RAM_DEPTH - 1 := 0;
	signal checksum_write_index : integer range 0 to 4 := 0;
	signal packet_counter       : unsigned(15 downto 0) := (others => '0');

	-- Captured input signals (sampled on frame_start rising edge in sys_clk)
	signal cap_message_type     : t_packet_type := t_Sync;
	signal cap_message_type_raw : std_logic_vector(3 downto 0) := (others => '0');
	signal cap_sequence_id      : unsigned(15 downto 0) := (others => '0');
	signal cap_timestamp_sec    : unsigned(47 downto 0) := (others => '0');
	signal cap_timestamp_nsec   : unsigned(31 downto 0) := (others => '0');
	signal cap_req_port_id      : std_logic_vector(79 downto 0) := (others => '0');
	signal cap_log_interval     : std_logic_vector(7 downto 0) := (others => '0');
	signal cap_time_source      : std_logic_vector(7 downto 0) := (others => '0');
	signal cap_udp_payload_len  : integer := 44;
	signal cap_packet_length    : integer := 0;

	-- IP checksum pipeline
	signal ip_checksum_partial1 : unsigned(31 downto 0) := (others => '0');
	signal ip_checksum_partial2 : unsigned(31 downto 0) := (others => '0');
	signal ip_checksum_acc      : unsigned(31 downto 0) := (others => '0');
	signal ip_checksum_value    : std_logic_vector(15 downto 0) := (others => '0');

	-- UDP checksum
	signal udp_checksum_acc            : unsigned(31 downto 0) := (others => '0');
	signal udp_checksum_upper_byte     : std_logic_vector(7 downto 0) := (others => '0');
	signal udp_checksum_byte_phase     : std_logic := '0';
	signal udp_checksum_value          : std_logic_vector(15 downto 0) := (others => '0');
	signal udp_pseudo_header_sum       : unsigned(31 downto 0) := (others => '0');
	signal udp_checksum_bytes_remaining : integer range 0 to 256 := 0;
	signal udp_checksum_request_count  : integer range 0 to 256 := 0;
	signal udp_checksum_data_valid     : std_logic := '0';
	signal udp_checksum_data_valid_d   : std_logic := '0';

	-- CDC handshake signals
	signal tx_frame_start       : std_logic := '0';
	signal tx_frame_start_sync1 : std_logic := '0';
	signal tx_frame_start_sync2 : std_logic := '0';
	signal tx_ack               : std_logic := '0';
	signal tx_ack_sync1         : std_logic := '0';
	signal tx_ack_sync2         : std_logic := '0';

	-- frame_start edge detection in sys_clk
	signal frame_start_sync1 : std_logic := '0';
	signal frame_start_sync2 : std_logic := '0';
	signal frame_start_prev  : std_logic := '0';

	-- ============================================================
	-- TX state machine (tx_clk domain)
	-- ============================================================
	type t_SM_Tx is (s_Idle, s_WaitForAllow, s_LatchCDC, s_PrimeTx, s_Transmit, s_End, s_WaitDeassert);
	signal SM_Tx : t_SM_Tx := s_Idle;

	signal tx_bytes_remaining : integer range 0 to RAM_DEPTH - 1 := 0;
	signal tx_read_pointer    : integer range 0 to RAM_DEPTH - 1 := 0;
	signal prime_wait         : integer range 0 to 1 := 0;

	-- Prevent register optimization for CDC registers
	attribute PRESERVE : boolean;
	attribute PRESERVE of frame_start_sync1 : signal is true;
	attribute PRESERVE of frame_start_sync2 : signal is true;
	attribute PRESERVE of tx_frame_start_sync1 : signal is true;
	attribute PRESERVE of tx_frame_start_sync2 : signal is true;
	attribute PRESERVE of tx_ack_sync1 : signal is true;
	attribute PRESERVE of tx_ack_sync2 : signal is true;

begin

	-- ============================================================
	-- Packet RAM Port B (tx_clk domain): TX read only
	-- Separate process for true dual-port inference (different clock)
	-- ============================================================
	packet_ram_port_b : process(tx_clk)
	begin
		if falling_edge(tx_clk) then
			tx_rd_data <= packet_ram(tx_rd_addr);
		end if;
	end process packet_ram_port_b;

	-- ============================================================
	-- Packet Assembly Process (sys_clk domain)
	-- Includes RAM Port A (write + read) for proper block RAM inference.
	-- Only ONE ram access (read or write) per clock cycle on this port.
	-- ============================================================
	sys_clock_process : process(sys_clk)
		variable header_data       : std_logic_vector(7 downto 0);
		variable pseudo_header_sum : unsigned(31 downto 0);
		variable udp_pl_len        : integer;
		variable pkt_len           : integer;
	begin
		if rising_edge(sys_clk) then
			-- ---- RAM Port A: one write OR one read per cycle ----
			if (packet_wr_en = '1') then
				packet_ram(packet_wr_addr) <= packet_wr_data;
			else
				-- Read path (used during UDP checksum calculation)
				-- Stage 1: RAM output register (required for M9K inference)
				asm_rd_data <= packet_ram(asm_rd_addr);
			end if;
			-- Stage 2: Pipeline register (breaks timing to checksum logic)
			asm_rd_data_r <= asm_rd_data;

			packet_wr_en <= '0'; -- default: no write next cycle

			-- CDC: sync frame_start from external domain to sys_clk
			frame_start_sync1 <= frame_start;
			frame_start_sync2 <= frame_start_sync1;
			frame_start_prev  <= frame_start_sync2;

			-- CDC: sync tx_ack from tx_clk to sys_clk
			tx_ack_sync1 <= tx_ack;
			tx_ack_sync2 <= tx_ack_sync1;

			case SM_Assemble is
				when s_A_Idle =>
					-- Detect rising edge of frame_start
					if (frame_start_sync2 = '1') and (frame_start_prev = '0') then
						-- Capture all input signals (stable because controller holds them while frame_start=1)
						cap_message_type     <= get_message_type(message_type_i);
						cap_message_type_raw <= message_type_i;
						cap_sequence_id      <= sequence_id;
						cap_timestamp_sec    <= timestamp_seconds_i;
						cap_timestamp_nsec   <= timestamp_nanoseconds_i;
						cap_req_port_id      <= request_port_identity;
						cap_log_interval     <= ptp_log_interval_i;
						cap_time_source      <= ptp_time_source_i;

						-- Compute lengths
						udp_pl_len := get_udp_payload_length(get_message_type(message_type_i));
						cap_udp_payload_len <= udp_pl_len;
						cap_packet_length   <= MAC_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH + udp_pl_len;

						packet_counter <= packet_counter + 1;

						-- IP checksum pipeline stage 1: partial sums
						ip_checksum_partial1 <= resize(to_unsigned(16#8011#, 16), 32) +
							resize(unsigned(src_ip_address(31 downto 16)), 32) +
							resize(unsigned(src_ip_address(15 downto 0)), 32);

						-- Destination IP depends on message type
						if is_pdelay_message(get_message_type(message_type_i)) then
							-- 224.0.0.107 = 0xE000006B
							ip_checksum_partial2 <=
								resize(to_unsigned(16#E000#, 16), 32) +
								resize(to_unsigned(16#006B#, 16), 32);
						else
							-- 224.0.1.129 = 0xE0000181
							ip_checksum_partial2 <=
								resize(to_unsigned(16#E000#, 16), 32) +
								resize(to_unsigned(16#0181#, 16), 32);
						end if;

						-- UDP checksum pseudo-header seed
						if is_pdelay_message(get_message_type(message_type_i)) then
							pseudo_header_sum := resize(unsigned(src_ip_address(31 downto 16)), 32)
								+ resize(unsigned(src_ip_address(15 downto 0)), 32)
								+ resize(to_unsigned(16#E000#, 16), 32)
								+ resize(to_unsigned(16#006B#, 16), 32)
								+ resize(to_unsigned(16#0011#, 16), 32)
								+ resize(to_unsigned(UDP_HEADER_LENGTH + udp_pl_len, 16), 32);
						else
							pseudo_header_sum := resize(unsigned(src_ip_address(31 downto 16)), 32)
								+ resize(unsigned(src_ip_address(15 downto 0)), 32)
								+ resize(to_unsigned(16#E000#, 16), 32)
								+ resize(to_unsigned(16#0181#, 16), 32)
								+ resize(to_unsigned(16#0011#, 16), 32)
								+ resize(to_unsigned(UDP_HEADER_LENGTH + udp_pl_len, 16), 32);
						end if;
						udp_checksum_acc <= pseudo_header_sum;
						udp_pseudo_header_sum <= pseudo_header_sum;

						-- Reset state
						udp_checksum_upper_byte <= (others => '0');
						udp_checksum_byte_phase <= '0';
						checksum_write_index <= 0;

						SM_Assemble <= s_A_CalcIpChkPartial;
					end if;

				-- IP checksum pipeline stage 2
				when s_A_CalcIpChkPartial =>
					ip_checksum_acc <= ip_checksum_partial1 + ip_checksum_partial2 +
						resize(to_unsigned(16#4500# + 16#B800#, 32), 32) +
						resize(to_unsigned(IP_HEADER_LENGTH + UDP_HEADER_LENGTH + cap_udp_payload_len, 16), 32) +
						resize(packet_counter, 32);

					frame_write_index <= 0;
					SM_Assemble <= s_A_CalcIpChkFinal;

				-- One more cycle to let ip_checksum_acc settle, then start writing
				when s_A_CalcIpChkFinal =>
					ip_checksum_value <= finalize_checksum(ip_checksum_acc);
					SM_Assemble <= s_A_PrepFrame;

				-- Write packet bytes into RAM one per cycle
				when s_A_PrepFrame =>
					packet_wr_en <= '1';
					packet_wr_addr <= frame_write_index;

					header_data := get_header_byte(
						frame_write_index, src_mac_address, src_ip_address,
						cap_message_type, cap_message_type_raw, packet_counter,
						cap_sequence_id, cap_timestamp_sec, cap_timestamp_nsec,
						cap_req_port_id, cap_udp_payload_len,
						cap_log_interval, cap_time_source
					);

					-- Patch in computed IP checksum at offsets 24-25
					if (frame_write_index = MAC_HEADER_LENGTH + 10) then
						header_data := ip_checksum_value(15 downto 8);
					elsif (frame_write_index = MAC_HEADER_LENGTH + 11) then
						header_data := ip_checksum_value(7 downto 0);
					end if;

					packet_wr_data <= header_data;

					-- Cache first byte for TX process
					if (frame_write_index = 0) then
						first_packet_byte <= header_data;
					end if;

					if (frame_write_index = cap_packet_length - 1) then
						-- Packet fully written to RAM, start UDP checksum
						udp_checksum_acc <= udp_pseudo_header_sum;
						udp_checksum_upper_byte <= (others => '0');
						udp_checksum_byte_phase <= '0';
						udp_checksum_bytes_remaining <= UDP_HEADER_LENGTH + cap_udp_payload_len;
						if (UDP_HEADER_LENGTH + cap_udp_payload_len > 0) then
							udp_checksum_request_count <= 1;
						else
							udp_checksum_request_count <= 0;
						end if;
						udp_checksum_data_valid <= '0';
						udp_checksum_data_valid_d <= '0';
						asm_rd_addr <= MAC_HEADER_LENGTH + IP_HEADER_LENGTH;
						SM_Assemble <= s_A_CalcUdpChecksum;
					else
						frame_write_index <= frame_write_index + 1;
					end if;

				-- Read back UDP portion from RAM and compute checksum
				when s_A_CalcUdpChecksum =>
					-- Pipeline delay to match asm_rd_data_r latency (2 cycles)
					udp_checksum_data_valid_d <= udp_checksum_data_valid;

					if (udp_checksum_data_valid_d = '1') then
						feed_checksum(udp_checksum_upper_byte, udp_checksum_byte_phase, udp_checksum_acc, asm_rd_data_r);
						if (udp_checksum_bytes_remaining > 0) then
							udp_checksum_bytes_remaining <= udp_checksum_bytes_remaining - 1;
						end if;
					end if;

					if ((udp_checksum_data_valid_d = '1') and (udp_checksum_bytes_remaining = 1)) then
						udp_checksum_data_valid <= '0';
						SM_Assemble <= s_A_FinalizeChecksum;
					elsif ((udp_checksum_data_valid = '0') and (udp_checksum_bytes_remaining = 0)) then
						SM_Assemble <= s_A_FinalizeChecksum;
					else
						if (udp_checksum_data_valid = '0') then
							if (udp_checksum_bytes_remaining > 0) then
								udp_checksum_data_valid <= '1';
							end if;
						end if;
						if (udp_checksum_request_count < (UDP_HEADER_LENGTH + cap_udp_payload_len)) then
							asm_rd_addr <= asm_rd_addr + 1;
							udp_checksum_request_count <= udp_checksum_request_count + 1;
						end if;
					end if;

				when s_A_FinalizeChecksum =>
					udp_checksum_value <= finalize_checksum(udp_checksum_acc);
					SM_Assemble <= s_A_WriteChecksum;

				-- Write UDP checksum back into RAM
				when s_A_WriteChecksum =>
					case checksum_write_index is
						when 0 =>
							packet_wr_en <= '1';
							packet_wr_addr <= MAC_HEADER_LENGTH + IP_HEADER_LENGTH + 6;
							packet_wr_data <= udp_checksum_value(15 downto 8);
							checksum_write_index <= 1;
						when 1 =>
							packet_wr_en <= '1';
							packet_wr_addr <= MAC_HEADER_LENGTH + IP_HEADER_LENGTH + 7;
							packet_wr_data <= udp_checksum_value(7 downto 0);
							checksum_write_index <= 2;
						when others =>
							-- Packet ready, signal TX process
							tx_frame_start <= '1';
							SM_Assemble <= s_A_RequestTx;
					end case;

				-- Wait for TX process to acknowledge
				when s_A_RequestTx =>
					if (tx_ack_sync2 = '1') then
						tx_frame_start <= '0';
						SM_Assemble <= s_A_WaitAckDone;
					end if;

				-- Wait for TX to complete (ack deasserted)
				when s_A_WaitAckDone =>
					if (tx_ack_sync2 = '0') then
						SM_Assemble <= s_A_Idle;
					end if;
			end case;
		end if;
	end process sys_clock_process;

	-- ============================================================
	-- TX Process (tx_clk domain)
	-- Reads packet bytes from RAM and sends them to MAC
	-- ============================================================
	tx_process : process(tx_clk)
	begin
		if rising_edge(tx_clk) then
			-- CDC: sync tx_frame_start from sys_clk
			tx_frame_start_sync1 <= tx_frame_start;
			tx_frame_start_sync2 <= tx_frame_start_sync1;

			tx_ready_o <= '0';

			case SM_Tx is
				when s_Idle =>
					tx_enable <= '0';
					tx_ack <= '0';
					tx_allow_req_o <= '0';
					if (tx_frame_start_sync2 = '1') then
						tx_ack <= '1';
						tx_allow_req_o <= '1';
						SM_Tx <= s_WaitForAllow;
					end if;

				when s_WaitForAllow =>
					if (tx_allow_i = '1') then
						SM_Tx <= s_LatchCDC;
					end if;

				when s_LatchCDC =>
					-- Initialize TX from assembled packet
					tx_data <= first_packet_byte;
					if (cap_packet_length > 1) then
						tx_rd_addr <= 1;
						tx_read_pointer <= 2;
						tx_bytes_remaining <= cap_packet_length - 1;
						SM_Tx <= s_PrimeTx;
					else
						tx_bytes_remaining <= 0;
						SM_Tx <= s_End;
					end if;
					prime_wait <= 0;

				when s_PrimeTx =>
					-- Wait one cycle for tx_rd_data to be valid from RAM
					tx_enable <= '0';
					if (prime_wait = 0) then
						prime_wait <= 1;
					else
						if (tx_busy = '0') then
							tx_enable <= '1';
							SM_Tx <= s_Transmit;
						end if;
					end if;

				when s_Transmit =>
					tx_enable <= '1';
					if (tx_byte_sent = '1') then
						if (tx_bytes_remaining = 0) then
							SM_Tx <= s_End;
						else
							tx_data <= tx_rd_data;
							if (tx_read_pointer < cap_packet_length) then
								tx_rd_addr <= tx_read_pointer;
								tx_read_pointer <= tx_read_pointer + 1;
							end if;
							tx_bytes_remaining <= tx_bytes_remaining - 1;
						end if;
					end if;

				when s_End =>
					tx_enable <= '0';
					tx_data <= (others => '0');
					tx_ready_o <= '1';
					tx_allow_req_o <= '0';
					SM_Tx <= s_WaitDeassert;

				when s_WaitDeassert =>
					-- Wait for sys_clock_process to deassert tx_frame_start
					if (tx_frame_start_sync2 = '0') then
						tx_ack <= '0';
						SM_Tx <= s_Idle;
					end if;
			end case;
		end if;
	end process tx_process;

end Behavioral;
