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
use work.ethernet_types;

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
		timestamp_nanoseconds_i : in unsigned(29 downto 0);

		request_port_identity : in std_logic_vector(79 downto 0);

		tx_enable				: out std_logic := '0';
		tx_data					: out std_logic_vector(7 downto 0) := (others => '0');
		tx_done_sys_o			: out std_logic := '0';

		message_type_i : in std_logic_vector(3 downto 0) := "0000";
		tx_ready_o			: out std_logic := '0';
		ptp_time_source_i : in std_logic_vector(7 downto 0);
		ptp_log_interval_i : in std_logic_vector(7 downto 0);

		tx_allow_req_o : out std_logic := '0';
		tx_allow_i : in std_logic;
		ptp_prioone : std_logic_vector(7 downto 0);
		ptp_priotwo : std_logic_vector(7 downto 0);
		ptp_clockclass : std_logic_vector(7 downto 0);
		ptp_clockaccuracy : std_logic_vector(7 downto 0);
		mac_speed_i : in std_logic_vector(1 downto 0)
		
	);
end entity;

architecture Behavioral of ptpv2_sender is
	-- Constants
	constant MAC_HEADER_LENGTH			: integer := 14;
	constant IP_HEADER_LENGTH			: integer := 20;
	constant UDP_HEADER_LENGTH			: integer := 8;

	-- RAM depth must be power-of-2 for M9K block RAM inference (106 bytes too small otherwise)
	constant RAM_DEPTH					: integer := 256;

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
		ts_nsec        : unsigned(29 downto 0);
		req_port_id    : std_logic_vector(79 downto 0);
		udp_payload_len : integer;
		log_interval   : std_logic_vector(7 downto 0);
		time_source    : std_logic_vector(7 downto 0);
		ptp_prioone_var : std_logic_vector(7 downto 0);
		ptp_priotwo_var : std_logic_vector(7 downto 0);
		ptp_clockclass_var : std_logic_vector(7 downto 0);
		ptp_clockaccuracy_var : std_logic_vector(7 downto 0)
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
			--when 3  => return x"00";
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
			--when 20 => return x"00";
			--when 21 => return x"00";
			when 22 => return x"80"; -- TTL=128
			when 23 => return x"11"; -- UDP
			--when 24 => return x"00"; -- IP checksum placeholder
			--when 25 => return x"00";
			when 26 => return src_ip(31 downto 24);
			when 27 => return src_ip(23 downto 16);
			when 28 => return src_ip(15 downto 8);
			when 29 => return src_ip(7 downto 0);
			when 30 => return x"E0"; -- 224
			--when 31 => return x"00"; -- 0
			when 32 => return x"01"; -- 1
			when 33 => return dst_last;

			-- UDP header (8 bytes)
			when 34 => return x"01";
			when 35 => return port_byte;
			when 36 => return x"01";
			when 37 => return port_byte;
			when 38 => return udp_length(15 downto 8);
			when 39 => return udp_length(7 downto 0);
			--when 40 => return x"00"; -- UDP checksum placeholder
			--when 41 => return x"00";

			-- PTP header (34 bytes, offsets 42..75)
			when 42 => return x"0" & msg_type_raw; -- majorSdoId + messageType
			when 43 => return x"02"; -- PTP version 2
			when 44 => return std_logic_vector(to_unsigned(udp_payload_len, 16)(15 downto 8));
			when 45 => return std_logic_vector(to_unsigned(udp_payload_len, 16)(7 downto 0));
			--when 46 => return x"00"; -- domainNumber
			--when 47 => return x"00"; -- minorSdoId
			when 48 => return x"02"; -- Flags1: TwoStep
			--when 49 => return x"00"; -- Flags2
			--when 50 => return x"00"; -- correctionField (8 bytes)
			--when 51 => return x"00";
			--when 52 => return x"00";
			--when 53 => return x"00";
			--when 54 => return x"00";
			--when 55 => return x"00";
			--when 56 => return x"00";
			--when 57 => return x"00";
			--when 58 => return x"00"; -- messageTypeSpecific (4 bytes)
			--when 59 => return x"00";
			--when 60 => return x"00";
			--when 61 => return x"00";
			when 62 => return clock_id(63 downto 56); -- clockIdentity (8 bytes)
			when 63 => return clock_id(55 downto 48);
			when 64 => return clock_id(47 downto 40);
			when 65 => return clock_id(39 downto 32);
			when 66 => return clock_id(31 downto 24);
			when 67 => return clock_id(23 downto 16);
			when 68 => return clock_id(15 downto 8);
			when 69 => return clock_id(7 downto 0);
			--when 70 => return x"00"; -- sourcePortId
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
			when 82 => return std_logic_vector(resize(ts_nsec(29 downto 24), 8));
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
					return ptp_prioone_var(7 downto 0); -- grandmasterPriority1
				else
					return x"00";
				end if;
			when 90 =>
				if (msg_type = t_Delay_Resp) or (msg_type = t_Pdelay_Resp) then
					return req_port_id(47 downto 40);
				elsif (msg_type = t_Announce) then
					return ptp_clockclass_var(7 downto 0); -- clockQuality.clockClass
				else
					return x"00";
				end if;
			when 91 =>
				if (msg_type = t_Delay_Resp) or (msg_type = t_Pdelay_Resp) then
					return req_port_id(39 downto 32);
				elsif (msg_type = t_Announce) then
					return ptp_clockaccuracy_var(7 downto 0); -- clockQuality.clockAccuracy
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
					return ptp_priotwo_var(7 downto 0); -- grandmasterPriority2
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
	-- Packet RAM (Port A sys_clk write-only, Port B tx_clk read)
	-- The checksums are folded by dedicated accumulator processes that tap the
	-- same Port A write stream, and are muxed into the Port B read at their byte
	-- offsets -- so neither checksum is ever stored in or read back from RAM.
	-- ============================================================
	type t_packet_ram is array (0 to RAM_DEPTH - 1) of std_logic_vector(7 downto 0);
	signal packet_ram : t_packet_ram := (others => (others => '0'));
	signal read_addr : integer range 0 to 200 := 0;
	signal tx_done_reg : std_logic := '0';

	-- Port A (sys_clk): write-only
	signal packet_wr_en   : std_logic := '0';
	signal packet_wr_addr : integer range 0 to RAM_DEPTH - 1 := 0;
	signal packet_wr_data : std_logic_vector(7 downto 0) := (others => '0');

	-- ============================================================
	-- Assembly state machine (sys_clk domain)
	-- ============================================================
	type t_SM_Assemble is (s_A_Idle, s_A_CalcValues, s_A_PrepFrame,
	                       s_A_SettleChecksum, s_A_FinalizeChecksum,
	                       s_A_RequestTx, s_A_WaitAckDone);
	signal SM_Assemble : t_SM_Assemble := s_A_Idle;

	signal frame_write_index    : integer range 0 to RAM_DEPTH - 1 := 0;
	signal packet_counter       : unsigned(15 downto 0) := (others => '0');

	-- Derived / CDC-relevant captures only. The controller (ptpv2_controller,
	-- same sys_clk domain) holds all data inputs stable from frame_start until
	-- tx_done_sys_o, so the plain pass-through inputs (sequence_id, timestamps,
	-- request_port_identity, log_interval, time_source, message_type_i) are read
	-- directly in get_header_byte and need no intermediate register.
	--   cap_message_type    : result of get_message_type() -- computed once instead
	--                         of re-evaluating the decode in every header byte.
	--   cap_udp_payload_len : result of get_udp_payload_length().
	--   cap_packet_length   : read in the tx_clk domain (packet_ram_port_b) -- a
	--                         real CDC crossing, must be a quasi-static register.
	signal cap_message_type     : t_packet_type := t_Sync;
	signal cap_udp_payload_len  : integer := 44;
	signal cap_packet_length    : integer := 0;

	-- ============================================================
	-- Linear checksum accumulators (fed off the packet_wr stream)
	-- ------------------------------------------------------------
	-- Two independent processes fold the byte stream written to the packet RAM
	-- into running ones'-complement sums, mirroring the linear write. The IP
	-- process covers the 20-byte IPv4 header; the UDP process covers the UDP
	-- header + PTP payload. Both checksum *fields* are written as 0x00 by
	-- get_header_byte, so folding them in is a no-op and needs no skip logic.
	-- The finalized 16-bit values stay in registers and are muxed into the TX
	-- byte stream (packet_ram_port_b) -- they are never written back to RAM.
	-- csum_arm pulses for one sys_clk before the first byte is written to seed
	-- both accumulators (IP starts at 0, UDP at the pseudo-header sum).
	constant IP_HDR_FIRST : integer := MAC_HEADER_LENGTH;                        -- 14
	constant IP_HDR_LAST  : integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH - 1; -- 33
	constant UDP_FIRST    : integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH;     -- 34

	-- Packet byte offsets of the four checksum bytes. The TX read substitutes
	-- the held checksum registers at these offsets instead of reading RAM, so
	-- the checksums never need to be written back to the packet RAM.
	constant IP_CSUM_HI_OFF  : integer := MAC_HEADER_LENGTH + 10;                   -- 24
	constant IP_CSUM_LO_OFF  : integer := MAC_HEADER_LENGTH + 11;                   -- 25
	constant UDP_CSUM_HI_OFF : integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH + 6; -- 40
	constant UDP_CSUM_LO_OFF : integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH + 7; -- 41

	signal csum_arm        : std_logic := '0';
	signal udp_pseudo_seed : unsigned(31 downto 0) := (others => '0');

	-- IP checksum accumulator
	signal ip_checksum_acc        : unsigned(31 downto 0) := (others => '0');
	signal ip_checksum_upper_byte : std_logic_vector(7 downto 0) := (others => '0');
	signal ip_checksum_byte_phase : std_logic := '0';
	signal ip_checksum_value      : std_logic_vector(15 downto 0) := (others => '0');

	-- UDP checksum accumulator
	signal udp_checksum_acc        : unsigned(31 downto 0) := (others => '0');
	signal udp_checksum_upper_byte : std_logic_vector(7 downto 0) := (others => '0');
	signal udp_checksum_byte_phase : std_logic := '0';
	signal udp_checksum_value      : std_logic_vector(15 downto 0) := (others => '0');

	-- TX read-port pipeline. The RAM read is kept unconditional so Quartus
	-- infers a real block-RAM read port; csum_sel carries the eff_addr
	-- classification registered in lockstep with ram_q, and tx_data is selected
	-- combinationally outside the process.
	signal ram_q    : std_logic_vector(7 downto 0) := (others => '0');
	signal csum_sel : integer range 0 to 4 := 0; -- 0=RAM, 1=IPhi 2=IPlo 3=UDPhi 4=UDPlo

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
	type t_SM_Tx is (s_Idle, s_WaitForAllow, s_Transmit, s_End, s_WaitDeassert);
	signal SM_Tx : t_SM_Tx := s_Idle;



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
	-- Packet RAM Port A (sys_clk): write-only during assembly.
	-- ============================================================
	packet_ram_port_a : process(sys_clk)
	begin
		if rising_edge(sys_clk) then
			if (packet_wr_en = '1') then
				packet_ram(packet_wr_addr) <= packet_wr_data;
			end if;
		end if;
	end process packet_ram_port_a;

	-- IP checksum accumulator: folds the 20 IPv4 header bytes (write addresses
	-- IP_HDR_FIRST..IP_HDR_LAST) off the same write stream that fills the RAM.
	-- The checksum field (write addr MAC+10/11) carries 0x00 from get_header_byte,
	-- so it folds in as zero -- no skip needed. Armed (cleared) by csum_arm.
	ip_checksum_proc : process(sys_clk)
	begin
		if rising_edge(sys_clk) then
			if (csum_arm = '1') then
				ip_checksum_acc        <= (others => '0');
				ip_checksum_upper_byte <= (others => '0');
				ip_checksum_byte_phase <= '0';
			elsif (packet_wr_en = '1'
			       and packet_wr_addr >= IP_HDR_FIRST
			       and packet_wr_addr <= IP_HDR_LAST) then
				feed_checksum(ip_checksum_upper_byte, ip_checksum_byte_phase, ip_checksum_acc, packet_wr_data);
			end if;
		end if;
	end process ip_checksum_proc;

	-- UDP checksum accumulator: seeded with the UDP/IPv4 pseudo-header sum, then
	-- folds every byte from UDP_FIRST onward (UDP header + PTP payload). The UDP
	-- checksum field (write addr 40/41) carries 0x00, folding in as zero.
	udp_checksum_proc : process(sys_clk)
	begin
		if rising_edge(sys_clk) then
			if (csum_arm = '1') then
				udp_checksum_acc        <= udp_pseudo_seed;
				udp_checksum_upper_byte <= (others => '0');
				udp_checksum_byte_phase <= '0';
			elsif (packet_wr_en = '1' and packet_wr_addr >= UDP_FIRST) then
				feed_checksum(udp_checksum_upper_byte, udp_checksum_byte_phase, udp_checksum_acc, packet_wr_data);
			end if;
		end if;
	end process udp_checksum_proc;

	-- ============================================================
	-- Packet Assembly Process (sys_clk domain)
	-- Writes header bytes linearly into the packet RAM. The dedicated checksum
	-- accumulators above tap the same write stream, so no RAM readback and no
	-- write-back of the checksum bytes is needed.
	-- ============================================================
	sys_clock_process : process(sys_clk)
		variable header_data : std_logic_vector(7 downto 0);
		variable udp_pl_len  : integer;
	begin
		if rising_edge(sys_clk) then
			packet_wr_en <= '0'; -- default: no write next cycle
			csum_arm     <= '0'; -- default: accumulators free-run; pulsed in s_A_CalcValues

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
					if (frame_start_sync2 = '1') then
						-- Only derived / CDC-relevant values are captured. All plain
						-- pass-through inputs (sequence_id, timestamps, request_port_
						-- identity, log_interval, time_source, message_type_i) are held
						-- stable by the controller until tx_done_sys_o, so get_header_byte
						-- reads them directly from the ports.
						tx_done_sys_o    <= '0';
						cap_message_type <= get_message_type(message_type_i);

						-- Compute lengths
						udp_pl_len := get_udp_payload_length(get_message_type(message_type_i));
						cap_udp_payload_len <= udp_pl_len;
						cap_packet_length   <= MAC_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH + udp_pl_len;

						packet_counter <= packet_counter + 1;

						SM_Assemble <= s_A_CalcValues;
					end if;

				-- Seed the UDP accumulator with the UDP/IPv4 pseudo-header sum and
				-- arm both accumulators (clear IP, load UDP seed) before the first
				-- header byte is written in s_A_PrepFrame. The IP accumulator seeds
				-- at 0 and folds the 20 header bytes directly, so no pre-computed IP
				-- partial sum is needed.
				when s_A_CalcValues =>
					if is_pdelay_message(cap_message_type) then
						-- 224.0.0.107 = 0xE000006B
						udp_pseudo_seed <= resize(unsigned(src_ip_address(31 downto 16)), 32)
							+ resize(unsigned(src_ip_address(15 downto 0)), 32)
							+ resize(to_unsigned(16#E000#, 16), 32)
							+ resize(to_unsigned(16#006B#, 16), 32)
							+ resize(to_unsigned(16#0011#, 16), 32)
							+ resize(to_unsigned(UDP_HEADER_LENGTH + cap_udp_payload_len, 16), 32);
					else
						-- 224.0.1.129 = 0xE0000181
						udp_pseudo_seed <= resize(unsigned(src_ip_address(31 downto 16)), 32)
							+ resize(unsigned(src_ip_address(15 downto 0)), 32)
							+ resize(to_unsigned(16#E000#, 16), 32)
							+ resize(to_unsigned(16#0181#, 16), 32)
							+ resize(to_unsigned(16#0011#, 16), 32)
							+ resize(to_unsigned(UDP_HEADER_LENGTH + cap_udp_payload_len, 16), 32);
					end if;

					csum_arm          <= '1';
					frame_write_index <= 0;
					SM_Assemble       <= s_A_PrepFrame;

				-- Write packet bytes into RAM one per cycle
				when s_A_PrepFrame =>
					packet_wr_en <= '1';
					packet_wr_addr <= frame_write_index;

					header_data := get_header_byte(
						frame_write_index, src_mac_address, src_ip_address,
						cap_message_type, message_type_i, packet_counter,
						sequence_id, timestamp_seconds_i, timestamp_nanoseconds_i,
						request_port_identity, cap_udp_payload_len,
						ptp_log_interval_i, ptp_time_source_i, ptp_prioone, ptp_priotwo, ptp_clockclass, ptp_clockaccuracy
					);

					packet_wr_data <= header_data;

					if (frame_write_index = cap_packet_length - 1) then
						-- Last byte's packet_wr_* is registered on this edge; the
						-- dedicated accumulators fold it one edge later.
						-- s_A_SettleChecksum absorbs that one-cycle lag before we
						-- read the final accumulator value.
						SM_Assemble <= s_A_SettleChecksum;
					else
						frame_write_index <= frame_write_index + 1;
					end if;

				when s_A_SettleChecksum =>
					-- Wait one cycle so the final folded byte has landed in the accumulators.
					SM_Assemble <= s_A_FinalizeChecksum;

				when s_A_FinalizeChecksum =>
					-- Fold the carries and one's-complement. Values stay in registers
					-- and are muxed into the TX byte stream (packet_ram_port_b); they
					-- are never written back to the packet RAM.
					ip_checksum_value  <= finalize_checksum(ip_checksum_acc);
					udp_checksum_value <= finalize_checksum(udp_checksum_acc);
					tx_frame_start     <= '1';
					SM_Assemble        <= s_A_RequestTx;

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
						tx_done_sys_o <= '1';
					end if;
			end case;
		end if;
	end process sys_clock_process;

	-- ============================================================
	-- Packet RAM Port B (tx_clk): TX read.
	-- Unconditional RAM read -> registered output. At the four checksum byte
	-- offsets the held checksum registers are muxed in instead of RAM. The
	-- checksum values are computed in sys_clk and stable long before tx_ack
	-- rises, so they cross into tx_clk as quasi-static values.
	-- ============================================================
	packet_ram_port_b : process(tx_clk)
		variable eff_addr : integer range 0 to 200;
	begin
		if rising_edge(tx_clk) then
			-- Effective address of the byte fetched this edge (advances on tx_byte_sent).
			if (tx_byte_sent = '1') then
				eff_addr := read_addr + 1;
			else
				eff_addr := read_addr;
			end if;

			-- Unconditional RAM read -> registered output. Keep this the ONLY thing
			-- that touches packet_ram so Quartus infers a block-RAM read port.
			ram_q <= packet_ram(eff_addr);

			-- Classify the SAME eff_addr and register it alongside ram_q, so the
			-- combinational mux below picks the checksum byte on the cycle ram_q
			-- holds that address's RAM data.
			case eff_addr is
				when IP_CSUM_HI_OFF  => csum_sel <= 1;
				when IP_CSUM_LO_OFF  => csum_sel <= 2;
				when UDP_CSUM_HI_OFF => csum_sel <= 3;
				when UDP_CSUM_LO_OFF => csum_sel <= 4;
				when others          => csum_sel <= 0;
			end case;

			if tx_ack = '1' and tx_allow_i = '1' then
				tx_enable <= '1';
				if (mac_speed_i = "10" and read_addr < cap_packet_length - 1) or (mac_speed_i /= "10" and read_addr < cap_packet_length) then
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

	-- Combinational checksum substitution on the RAM read output. ram_q and
	-- csum_sel are registered together in packet_ram_port_b, so they describe the
	-- same byte; the checksum registers are quasi-static across a frame.
	with csum_sel select tx_data <=
		ip_checksum_value(15 downto 8)  when 1,
		ip_checksum_value(7 downto 0)   when 2,
		udp_checksum_value(15 downto 8) when 3,
		udp_checksum_value(7 downto 0)  when 4,
		ram_q                           when others;
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
					tx_ack <= '0';
					tx_allow_req_o <= '0';
					if (tx_frame_start_sync2 = '1') then
						tx_ack <= '1';
						tx_allow_req_o <= '1';
						SM_Tx <= s_WaitForAllow;
					end if;

				when s_WaitForAllow =>
					if (tx_allow_i = '1') then
						SM_Tx <= s_Transmit;
						
					end if;


				when s_Transmit =>
					if (tx_done_reg = '1') then
						SM_Tx <= s_End;
					end if;

				when s_End =>
					
					if (tx_busy = '0') then -- wait for phy to finish transmission
						tx_ready_o <= '1';
						tx_allow_req_o <= '0';
						SM_Tx <= s_WaitDeassert;
					end if;

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
