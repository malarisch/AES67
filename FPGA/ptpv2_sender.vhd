-- PTPv2 Sync Delay_Resp UDP Packet Sender
-- based on udp_packet.vhd:
-- UDP Packet Sender
-- (c) 2025 Dr.-Ing. Christian Noeding
-- christian@noeding-online.de
-- Released under GNU General Public License v3
-- Source: https://www.github.com/xn--nding-jua/AES50_Transmitter
--
-- This file contains an ethernet-packet-generator to send individual bytes to an EthernetMAC directly.
--
-- when not using ARP, set ARP-entry in Windows manually using
-- netsh interface ipv4 add neighbors "Ethernet 1" 192.168.0.42 00-1c-23-17-4a-cb
-- To delete this entry, use the following command:
-- arp -d 192.168.0.42

library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.ALL;

entity ptpv2_sender is
	port
	(
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

		tx_enable				: out std_logic := '0';  -- TX valid
		tx_data					: out std_logic_vector(7 downto 0) := (others => '0'); -- data-octet


        message_type_i : in std_logic_vector(3 downto 0) := "0000";

        tx_ready_timestamp_seconds_o : out unsigned(47 downto 0);
        tx_ready_timestamp_nanoseconds_o : out unsigned(31 downto 0)
	);
end entity;

architecture Behavioral of ptpv2_sender is
	-- Constants
    constant PTP_EVENT_PORT            : integer := 320;
	constant MAC_HEADER_LENGTH			: integer := 14;
	constant IP_HEADER_LENGTH			: integer := 5 * (32 / 8); -- Header length always 20 bytes (5 * 32 bit words)
	constant UDP_PSEUDO_HEADER_LENGTH: integer := 8;
	constant UDP_HEADER_LENGTH			: integer := 8;
	constant MAX_UDP_PAYLOAD_LENGTH		: integer := 64;
	constant MAX_PACKET_LENGTH				: integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH + MAX_UDP_PAYLOAD_LENGTH;

    signal real_udp_payload_length		: integer := 64;
    signal real_packet_length			: integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH + real_udp_payload_length;
    type t_packet_type is (t_Sync, t_Delay_Req, t_Follow_Up, t_Delay_Resp, t_Announce, t_Pdelay_Req, t_Pdelay_Resp, t_Pdelay_Follow_Up);
    
    function get_message_type(message_type : std_logic_vector(3 downto 0)) return t_packet_type is
    begin
        case message_type is
            when x"0" =>
                return t_Sync;
            when x"1" =>
                return t_Delay_Req;
            when x"2" =>
                return t_Pdelay_Req;
            when x"3" =>
                return t_Pdelay_Resp;
            when x"8" =>
                return t_Follow_Up;
            when x"9" =>
                return t_Delay_Resp;
            when x"A" =>
                return t_Pdelay_Follow_Up;
            when x"B" =>
                return t_Announce;
                
            when others =>
                return t_Sync; -- default
        end case;
    end function;

    signal current_message_type : t_packet_type;
    signal tx_port_319_i : std_logic := '0';
    
	-- Checksum calculation
	signal checksum						: unsigned(15 downto 0) := (others => '0');
	signal checksum_tmp					: unsigned(31 downto 0) := (others => '0');
	signal checksum_byte_count			: integer range 0 to IP_HEADER_LENGTH + 2;
	signal calculating_checksum		: std_logic := '0';
	signal calc_new_checksum			: std_logic := '0';
        

	signal udp_checksum					: unsigned(15 downto 0) := (others => '0');
	signal udp_checksum_tmp				: unsigned(31 downto 0) := (others => '0');
	signal udp_checksum_byte_count	: integer range 0 to UDP_PSEUDO_HEADER_LENGTH + UDP_HEADER_LENGTH + MAX_UDP_PAYLOAD_LENGTH + 2;
	signal udp_calculating_checksum	: std_logic := '0';
	signal udp_calc_new_checksum		: std_logic := '0';

	-- Other signals used in this file
	type t_SM_Ethernet is (s_Idle, s_CalcChecksum, s_WaitChecksum, s_Start, s_Wait, s_Transmit, s_End);
	signal s_SM_Ethernet					: t_SM_Ethernet := s_Idle;
	signal byte_counter					: integer range 0 to 1600 := 0; -- one ethernet-frame cannot take more than 1500 bytes + header
	signal packet_counter				: integer range 0 to 65535 := 0;

	type t_ethernet_frame is array (0 to MAX_PACKET_LENGTH - 1) of std_logic_vector(7 downto 0);
	signal udp_frame		: t_ethernet_frame;

	signal zframe_start	: std_logic;

	-- ============================================================
	-- CDC (Clock Domain Crossing) Synchronizers
	-- All input signals from 250MHz domain need 2-stage sync
	-- ============================================================
	
	-- Frame start CDC (single bit - 2-stage synchronizer)
	signal frame_start_meta    : std_logic := '0';
	signal frame_start_sync    : std_logic := '0';
	
	-- Message type CDC (4-bit bus - synchronized with frame_start)
	signal message_type_meta   : std_logic_vector(3 downto 0) := (others => '0');
	signal message_type_sync   : std_logic_vector(3 downto 0) := (others => '0');
	
	-- Sequence ID CDC (16-bit bus - synchronized with frame_start)
	signal sequence_id_meta    : unsigned(15 downto 0) := (others => '0');
	signal sequence_id_sync    : unsigned(15 downto 0) := (others => '0');
	
	-- Timestamp seconds CDC (32-bit bus - synchronized with frame_start)
	signal timestamp_sec_meta  : unsigned(47 downto 0) := (others => '0');
	signal timestamp_sec_sync  : unsigned(47 downto 0) := (others => '0');
	
	-- Timestamp nanoseconds CDC (32-bit bus - synchronized with frame_start)
	signal timestamp_nsec_meta : unsigned(31 downto 0) := (others => '0');
	signal timestamp_nsec_sync : unsigned(31 downto 0) := (others => '0');
	
	-- Request port identity CDC (80-bit bus - synchronized with frame_start)
	signal req_port_id_meta    : std_logic_vector(79 downto 0) := (others => '0');
	signal req_port_id_sync    : std_logic_vector(79 downto 0) := (others => '0');

	-- Prevent register optimization/merging for metastability registers
	attribute PRESERVE : boolean;
	attribute PRESERVE of frame_start_meta : signal is true;
	attribute PRESERVE of frame_start_sync : signal is true;
	attribute PRESERVE of message_type_meta : signal is true;
	attribute PRESERVE of sequence_id_meta : signal is true;
	attribute PRESERVE of timestamp_sec_meta : signal is true;
	attribute PRESERVE of timestamp_nsec_meta : signal is true;
	attribute PRESERVE of req_port_id_meta : signal is true;

begin
	-- ============================================================
	-- CDC Synchronization Process
	-- Multi-bit signals are safe because they are qualified by frame_start
	-- (held stable when frame_start is asserted in source domain)
	-- ============================================================
	cdc_sync_proc: process(tx_clk)
	begin
		if rising_edge(tx_clk) then
			-- Stage 1 (metastable)
			frame_start_meta    <= frame_start;
			message_type_meta   <= message_type_i;
			sequence_id_meta    <= sequence_id;
			timestamp_sec_meta  <= timestamp_seconds_i;
			timestamp_nsec_meta <= timestamp_nanoseconds_i;
			req_port_id_meta    <= request_port_identity;
			
			-- Stage 2 (stable)
			frame_start_sync    <= frame_start_meta;
			message_type_sync   <= message_type_meta;
			sequence_id_sync    <= sequence_id_meta;
			timestamp_sec_sync  <= timestamp_sec_meta;
			timestamp_nsec_sync <= timestamp_nsec_meta;
			req_port_id_sync    <= req_port_id_meta;
		end if;
	end process cdc_sync_proc;
	-- Combinatorial logic for message type decoding (must be immediate, not clocked)
	-- Uses synchronized signal for CDC safety
	current_message_type <= get_message_type(message_type_sync);
	
	with current_message_type select real_udp_payload_length <=
		44 when t_Sync,
		44 when t_Delay_Req,
		54 when t_Pdelay_Req,
		54 when t_Pdelay_Resp,
		44 when t_Follow_Up,
		54 when t_Delay_Resp,
		64 when t_Announce,
		54 when t_Pdelay_Follow_Up,
		44 when others;
	
	with current_message_type select tx_port_319_i <=
		'1' when t_Sync,
		'1' when t_Delay_Req,
		'1' when t_Pdelay_Req,
		'1' when t_Pdelay_Resp,
		'0' when others;  -- Follow_Up, Delay_Resp, Announce, Pdelay_Follow_Up use port 320
	
	real_packet_length <= MAC_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH + real_udp_payload_length;

	process (tx_clk)
		variable Word: std_logic_vector(15 downto 0);
		variable udpWord: std_logic_vector(15 downto 0);
	begin
		if (falling_edge(tx_clk)) then
			zframe_start <= frame_start_sync;
			if ((frame_start_sync = '1') and (zframe_start = '0') and (s_SM_Ethernet = s_Idle)) then
				-- prepare begin of packet
				packet_counter <= packet_counter + 1; -- increment packet counter
				tx_enable <= '0';
				byte_counter <= 0;

				-- 7 preamble bytes + SFD will be added by Ethernet-MAC
				
				-- MAC HEADER (14 bytes)
				-- fill MAC-Header with desired values
				udp_frame(0) <= x"01"; -- MSB contains typical left side of MAC
				udp_frame(1) <= x"00";
				udp_frame(2) <= x"5e";
				udp_frame(3) <= x"00";
				udp_frame(4) <= x"01";
				udp_frame(5) <= x"81"; -- standard PTP-multicast MAC-address LSB for 224.0.1.129

				udp_frame(6) <= src_mac_address(47 downto 40); -- MSB contains typical left side of MAC
				udp_frame(7) <= src_mac_address(39 downto 32);
				udp_frame(8) <= src_mac_address(31 downto 24);
				udp_frame(9) <= src_mac_address(23 downto 16);
				udp_frame(10) <= src_mac_address(15 downto 8);
				udp_frame(11) <= src_mac_address(7 downto 0);

				-- IP Protocol
				udp_frame(12) <= x"08"; -- type [0x0800 = IP Protocol]
				udp_frame(13) <= x"00";

				-- IP HEADER (20 bytes)
				udp_frame(14) <= x"45"; -- b14 = version (4-bit) | internet header length (4-bit) [Version 4 and header length of 0x05 = 20 bytes]
				udp_frame(15) <= b"10111000"; -- differentiated services (6-bits) | explicit congestion notification (2-bits) -> DSCP=0x2e (Expedited Forwarding)
				udp_frame(16) <= std_logic_vector(to_unsigned(IP_HEADER_LENGTH + UDP_HEADER_LENGTH + real_udp_payload_length, 16)(15 downto 8)); -- total length without MAC-header: entire packet size in bytes, including IP-header and payload-data. The minimum size is 46 bytes of user data (= 0x2e, header without data) and the maximum is 65,535 bytes
				udp_frame(17) <= std_logic_vector(to_unsigned(IP_HEADER_LENGTH + UDP_HEADER_LENGTH + real_udp_payload_length, 16)(7 downto 0)); -- 20 bytes IP-header + 8 bytes UDP-header + 18 bytes UDP-payload = 46 bytes = 0x002e
				udp_frame(18) <= std_logic_vector(to_unsigned(packet_counter, 16)(15 downto 8));
				udp_frame(19) <= std_logic_vector(to_unsigned(packet_counter, 16)(7 downto 0));
				udp_frame(20) <= x"00"; -- flags (3-bits) | fragment offsets (13-bits)
				udp_frame(21) <= x"00";
				udp_frame(22) <= x"80"; -- time to live (0x80 = 128)
				udp_frame(23) <= x"11"; -- b23 = protocol (0x01 = ICMP, 0x06 = TCP, 0x11 = UDP)
				udp_frame(24) <= x"00"; -- header checksum (16-bit ones' complement of the ones' complement sum of all 16-bit words in the header)
				udp_frame(25) <= x"00";

				udp_frame(26) <= src_ip_address(31 downto 24); -- MSB contains typical "192"
				udp_frame(27) <= src_ip_address(23 downto 16);
				udp_frame(28) <= src_ip_address(15 downto 8);
				udp_frame(29) <= src_ip_address(7 downto 0);
	
				udp_frame(30) <= x"E0"; -- 224
				udp_frame(31) <= x"00"; -- 0
				udp_frame(32) <= x"01"; -- 1
				udp_frame(33) <= x"81"; -- 129
				

				-- UDP HEADER (8 bytes)
				udp_frame(34) <= x"01";
                udp_frame(36) <= x"01";

                if (tx_port_319_i = '1') then
                    udp_frame(35) <= x"3f"; -- source port 319 (PTP event messages)
                    udp_frame(37) <= x"3f"; -- source port 319 (PTP event messages)
                    
                else
                    udp_frame(35) <= x"40"; -- source port 320 (PTP general messages)
                    udp_frame(37) <= x"40"; -- source port 320 (PTP general messages)
                    
                end if;
				
				udp_frame(38) <= std_logic_vector(to_unsigned(UDP_HEADER_LENGTH + real_udp_payload_length, 16)(15 downto 8)); -- length (length of this UDP packet including header and data. Minimum 8 bytes)
				udp_frame(39) <= std_logic_vector(to_unsigned(UDP_HEADER_LENGTH + real_udp_payload_length, 16)(7 downto 0));
				udp_frame(40) <= x"00"; -- checksum (0 is a valid CRC-value to ignore it)
				udp_frame(41) <= x"00";

				-- UDP PAYLOAD (44 bytes)
				udp_frame(42) <= x"0" & message_type_sync; -- 0 majorSdoId + MessageType (Mapping is as in protocol specification!!!)
				udp_frame(43) <= x"02"; -- 1 majorVersionPTP = 0000 0010b = version 2
				udp_frame(44) <= std_logic_vector(to_unsigned(real_udp_payload_length, 16)(15 downto 8)); -- 2 messageLength MSB
				udp_frame(45) <= std_logic_vector(to_unsigned(real_udp_payload_length, 16)(7 downto 0)); -- 3 messageLength continued (44 bytes = 0x002c)
				udp_frame(46) <= x"00"; -- 4 domainNumber
				udp_frame(47) <= x"00"; -- 5 minorSdoId (reserved for future use)
				udp_frame(48) <= x"02"; -- 6 Flags1 0x0200 = TwoStep
				udp_frame(49) <= x"00"; -- 7 Flags2
				udp_frame(50) <= x"00"; -- 8 0 CorrectionField NS
				udp_frame(51) <= x"00"; -- 9 1 CorrectionField NS
				udp_frame(52) <= x"00"; -- 10 2 CorrectionField NS
                udp_frame(53) <= x"00"; -- 11 3 CorrectionField NS
				udp_frame(54) <= x"00"; -- 12 4 CorrectionField NS
				udp_frame(55) <= x"00"; -- 13 5 CorrectionField NS
				udp_frame(56) <= x"00"; -- 14 6 CorrectionField SubNS
				udp_frame(57) <= x"00"; -- 15 7 CorrectionField SubNS
				udp_frame(58) <= x"00"; -- 16 MessageType specific
                udp_frame(59) <= x"00"; -- 17 MessageType specific
                udp_frame(60) <= x"00"; -- 18 MessageType specific
                udp_frame(61) <= x"00"; -- 19 MessageType specific
                udp_frame(62) <= src_mac_address(47 downto 40) xor x"02"; -- 20 0 ClockIdentity
                udp_frame(63) <= src_mac_address(39 downto 32); -- 21 1 ClockIdentity
                udp_frame(64) <= src_mac_address(31 downto 24); -- 22 2 ClockIdentity
                udp_frame(65) <= x"FF"; -- 23 3 ClockIdentity
                udp_frame(66) <= x"FE"; -- 24 4 ClockIdentity
                udp_frame(67) <= src_mac_address(23 downto 16); -- 25 5 ClockIdentity
                udp_frame(68) <= src_mac_address(15 downto 8); -- 26 6 ClockIdentity
                udp_frame(69) <= src_mac_address(7 downto 0); -- 27 7 ClockIdentity
                udp_frame(70) <= x"00"; -- 28 sourcePortId
                udp_frame(71) <= x"01"; -- 29 sourcePortId
				udp_frame(72) <= std_logic_vector(sequence_id_sync(15 downto 8)); -- 30 sequenceId
				udp_frame(73) <= std_logic_vector(sequence_id_sync(7 downto 0)); -- 31 sequenceId
                case current_message_type is
                    when t_Sync =>
                        udp_frame(74) <= x"00"; -- 32 controlField
                    when t_Delay_Req =>
                        udp_frame(74) <= x"01"; -- 32 controlField
                    when t_Follow_Up =>
                            udp_frame(74) <= x"02"; -- 32 controlField
                            
                    when t_Delay_Resp =>
                        udp_frame(74) <= x"03";
                    when others =>
                        udp_frame(74) <= x"05"; -- 32 controlField -- 0x02 for Delay_Req, Sync, Follow_Up, Announce

                        -- TODO: Later add 0x04 for management messagess
                end case;
                udp_frame(75) <= x"00"; -- 33 logMessageInterval
                udp_frame(76) <= std_logic_vector(timestamp_sec_sync(47 downto 40)); -- 34 0 originTimestamp seconds
                udp_frame(77) <= std_logic_vector(timestamp_sec_sync(39 downto 32)); -- 35 1 originTimestamp seconds
				udp_frame(78) <= std_logic_vector(timestamp_sec_sync(31 downto 24)); -- 36 2 originTimestamp seconds 
				udp_frame(79) <= std_logic_vector(timestamp_sec_sync(23 downto 16)); -- 37 3 originTimestamp seconds
				udp_frame(80) <= std_logic_vector(timestamp_sec_sync(15 downto 8)); -- 38 4 originTimestamp seconds
				udp_frame(81) <= std_logic_vector(timestamp_sec_sync(7 downto 0)); -- 39 5 originTimestamp seconds
				udp_frame(82) <= std_logic_vector(timestamp_nsec_sync(31 downto 24)); -- 40 0 originTimestamp nanoseconds
				udp_frame(83) <= std_logic_vector(timestamp_nsec_sync(23 downto 16)); -- 41 1 originTimestamp nanoseconds
				udp_frame(84) <= std_logic_vector(timestamp_nsec_sync(15 downto 8)); -- 42 2 originTimestamp nanoseconds
				udp_frame(85) <= std_logic_vector(timestamp_nsec_sync(7 downto 0)); -- 43 3 originTimestamp nanoseconds
                if (current_message_type = t_Delay_Resp) then
                    udp_frame(86) <= std_logic_vector(req_port_id_sync(79 downto 72)); -- 44 0 requestingPortIdentity
                    udp_frame(87) <= std_logic_vector(req_port_id_sync(71 downto 64)); -- 45 1 requestingPortIdentity
                    udp_frame(88) <= std_logic_vector(req_port_id_sync(63 downto 56)); -- 46 2 requestingPortIdentity
                    udp_frame(89) <= std_logic_vector(req_port_id_sync(55 downto 48)); -- 47 3 requestingPortIdentity
                    udp_frame(90) <= std_logic_vector(req_port_id_sync(47 downto 40)); -- 48 4 requestingPortIdentity
                    udp_frame(91) <= std_logic_vector(req_port_id_sync(39 downto 32)); -- 49 5 requestingPortIdentity
                    udp_frame(92) <= std_logic_vector(req_port_id_sync(31 downto 24)); -- 50 6 requestingPortIdentity
                    udp_frame(93) <= std_logic_vector(req_port_id_sync(23 downto 16)); -- 51 7 requestingPortIdentity
                    udp_frame(94) <= std_logic_vector(req_port_id_sync(15 downto 8)); -- 52 8 requestingPortNumber
                    udp_frame(95) <= std_logic_vector(req_port_id_sync(7 downto 0)); -- 53 9 requestingPortNumber
                elsif (current_message_type = t_Announce) then
					-- Announce message has no timestamp, so pad with zeros
					udp_frame(86) <= x"00"; --current UTC Offset
					udp_frame(87) <= x"25"; --current UTC Offset
					udp_frame(88) <= x"00"; -- reserved
					udp_frame(89) <= x"80"; -- grandmasterPriority1
					udp_frame(90) <= x"F8"; --- clockQuality.clockClass
					udp_frame(91) <= x"FE"; -- clockQuality.clockAccuracy
					udp_frame(92) <= x"FF"; -- clockQuality.offsetScaledLogVariance MSB
					udp_frame(93) <= x"FF"; -- clockQuality.offsetScaledLogVariance LSB
					udp_frame(94) <= x"80"; -- grandmasterPriority2

                	udp_frame(95) <= src_mac_address(47 downto 40) xor x"02"; -- 20 0 ClockIdentity
                	udp_frame(96) <= src_mac_address(39 downto 32); -- 21 1 ClockIdentity
                	udp_frame(97) <= src_mac_address(31 downto 24); -- 22 2 ClockIdentity
                	udp_frame(98) <= x"FF"; -- 23 3 ClockIdentity
                	udp_frame(99) <= x"FE"; -- 24 4 ClockIdentity
                	udp_frame(100) <= src_mac_address(23 downto 16); -- 25 5 ClockIdentity
                	udp_frame(101) <= src_mac_address(15 downto 8); -- 26 6 ClockIdentity
                	udp_frame(102) <= src_mac_address(7 downto 0); -- 27 7 ClockIdentity
                	
					udp_frame(103) <= x"00"; -- stepsRemoved MSB
					udp_frame(104) <= x"00"; -- stepsRemoved LSB

					udp_frame(105) <= x"A0"; -- timeSource
				end if;
				
                

				checksum_tmp                <= (others => '0');
				checksum_byte_count         <= 0;
				calculating_checksum        <= '1';
				
				udp_checksum_tmp            <= to_unsigned(17 + UDP_HEADER_LENGTH + real_udp_payload_length, 32); -- 0x11 (protocol) + UDP-LENGTH (header+payload)
				udp_checksum_byte_count     <= 0;
				udp_calculating_checksum    <= '1';
				
				s_SM_Ethernet <= s_CalcChecksum;
				
			elsif (s_SM_Ethernet = s_CalcChecksum) then
				-- calculate checksum for IP-Header
				if (checksum_byte_count < IP_HEADER_LENGTH) then
					Word                    := udp_frame(MAC_HEADER_LENGTH + checksum_byte_count) & udp_frame(MAC_HEADER_LENGTH + checksum_byte_count + 1);
					checksum_tmp            <= checksum_tmp + resize(unsigned(Word), 32);
					checksum_byte_count     <= checksum_byte_count + 2; -- we are reading two bytes at once
				else
					-- checksum is calculated -> make sure that we have only 2-byte checksum and add carryover above 16th bit to 16-bit checksum
					if (checksum_tmp(31 downto 16) > 0) then
						checksum_tmp <= resize(checksum_tmp(15 downto 0), 32) + resize(checksum_tmp(31 downto 16), 32);
					else
						checksum                <= x"ffff" - checksum_tmp(15 downto 0);
						calculating_checksum    <= '0';
					end if;
				end if;

				-- calculate checksum for UDP-Payload
				if (udp_checksum_byte_count < (UDP_PSEUDO_HEADER_LENGTH + UDP_HEADER_LENGTH + real_udp_payload_length)) then
					udpWord                     := udp_frame(MAC_HEADER_LENGTH + IP_HEADER_LENGTH - UDP_PSEUDO_HEADER_LENGTH + udp_checksum_byte_count) & udp_frame(MAC_HEADER_LENGTH + IP_HEADER_LENGTH - UDP_PSEUDO_HEADER_LENGTH + udp_checksum_byte_count + 1);
					udp_checksum_tmp            <= udp_checksum_tmp + resize(unsigned(udpWord), 32);
					udp_checksum_byte_count     <= udp_checksum_byte_count + 2; -- we are reading two bytes at once
				else
					-- checksum is calculated -> make sure that we have only 2-byte checksum and add carryover above 16th bit to 16-bit checksum
					if (udp_checksum_tmp(31 downto 16) > 0) then
						udp_checksum_tmp <= resize(udp_checksum_tmp(15 downto 0), 32) + resize(udp_checksum_tmp(31 downto 16), 32);
					else
						-- calc inversion and stop checksum-calculation
						udp_checksum                <= x"ffff" - udp_checksum_tmp(15 downto 0);
						udp_calculating_checksum    <= '0';
					end if;
				end if;
				
				-- if both checksum are ready, go to next state
				if ((calculating_checksum = '0') and (udp_calculating_checksum = '0')) then
					s_SM_Ethernet <= s_Start;
				end if;
				
			elsif (s_SM_Ethernet = s_Start) then
				-- wait until MAC is ready again
				if (tx_busy = '0') then
					udp_frame(MAC_HEADER_LENGTH + 10) <= std_logic_vector(checksum(15 downto 8)); -- MSB
					udp_frame(MAC_HEADER_LENGTH + 11) <= std_logic_vector(checksum(7 downto 0)); -- LSB
					udp_frame(MAC_HEADER_LENGTH + IP_HEADER_LENGTH + 6) <= std_logic_vector(udp_checksum(15 downto 8)); -- MSB
					udp_frame(MAC_HEADER_LENGTH + IP_HEADER_LENGTH + 7) <= std_logic_vector(udp_checksum(7 downto 0)); -- LSB
					
					tx_enable <= '1';
					byte_counter <= 0; -- preload to first byte again
					tx_data <= udp_frame(0);

					s_SM_Ethernet <= s_Transmit;
				end if;
			
			elsif (s_SM_Ethernet = s_Transmit) then
				-- wait until previous byte is sent
				if (tx_byte_sent = '1') then
					-- send next byte and increment byte_counter
					tx_data <= udp_frame(byte_counter);
					
					if (byte_counter = real_packet_length - 1) then
						-- stop transmitting
						s_SM_Ethernet <= s_End;
					end if;
					
					byte_counter <= byte_counter + 1;
				end if;
				
			elsif (s_SM_Ethernet = s_End) then
				tx_enable <= '0';
				tx_data <= "00000000";
                tx_ready_timestamp_nanoseconds_o <= timestamp_nsec_sync;
                tx_ready_timestamp_seconds_o <= timestamp_sec_sync;
				s_SM_Ethernet <= s_Idle;
			end if;
		end if;
	end process;
end Behavioral;
