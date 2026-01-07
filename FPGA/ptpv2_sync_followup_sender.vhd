-- PTPv2 Sync Follow-Up UDP Packet Sender
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

entity ptpv2_sync_followup_sender is
	port
	(
		src_mac_address		: in std_logic_vector(47 downto 0);
		src_ip_address			: in std_logic_vector(31 downto 0);
		dst_mac_address		: in std_logic_vector(47 downto 0);
		dst_ip_address			: in std_logic_vector(31 downto 0);
		src_udp_port			: in std_logic_vector(15 downto 0);
		dst_udp_port			: in std_logic_vector(15 downto 0);
		frame_start				: in std_logic;
		tx_clk					: in std_logic;
		tx_busy					: in std_logic;
		tx_byte_sent			: in std_logic;

		tx_enable				: out std_logic := '0';  -- TX valid
		tx_data					: out std_logic_vector(7 downto 0) := (others => '0'); -- data-octet

        timestamp_seconds_i : in unsigned(31 downto 0);
        timestamp_nanoseconds_i : in unsigned(31 downto 0)
	);
end entity;

architecture Behavioral of ptpv2_sync_followup_sender is
	-- Constants
    constant PTP_EVENT_PORT            : integer := 319;
	constant MAC_HEADER_LENGTH			: integer := 14;
	constant IP_HEADER_LENGTH			: integer := 5 * (32 / 8); -- Header length always 20 bytes (5 * 32 bit words)
	constant UDP_PSEUDO_HEADER_LENGTH: integer := 8;
	constant UDP_HEADER_LENGTH			: integer := 8;
	constant UDP_PAYLOAD_LENGTH		: integer := 44;
	constant PACKET_LENGTH				: integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH;

	-- Checksum calculation
	signal checksum						: unsigned(15 downto 0) := (others => '0');
	signal checksum_tmp					: unsigned(31 downto 0) := (others => '0');
	signal checksum_byte_count			: integer range 0 to IP_HEADER_LENGTH + 2;
	signal calculating_checksum		: std_logic := '0';
	signal calc_new_checksum			: std_logic := '0';

	signal udp_checksum					: unsigned(15 downto 0) := (others => '0');
	signal udp_checksum_tmp				: unsigned(31 downto 0) := (others => '0');
	signal udp_checksum_byte_count	: integer range 0 to UDP_PSEUDO_HEADER_LENGTH + UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH + 2;
	signal udp_calculating_checksum	: std_logic := '0';
	signal udp_calc_new_checksum		: std_logic := '0';

	-- Other signals used in this file
	type t_SM_Ethernet is (s_Idle, s_CalcChecksum, s_WaitChecksum, s_Start, s_Wait, s_Transmit, s_End);
	signal s_SM_Ethernet					: t_SM_Ethernet := s_Idle;
	signal byte_counter					: integer range 0 to 1600 := 0; -- one ethernet-frame cannot take more than 1500 bytes + header
	signal packet_counter				: integer range 0 to 65535 := 0;

	type t_ethernet_frame is array (0 to PACKET_LENGTH - 1) of std_logic_vector(7 downto 0);
	signal udp_frame		: t_ethernet_frame;

	signal zframe_start	: std_logic;
begin
	process (tx_clk)
		variable Word: std_logic_vector(15 downto 0);
		variable udpWord: std_logic_vector(15 downto 0);
	begin
		if (falling_edge(tx_clk)) then
			zframe_start <= frame_start;

			if ((frame_start = '1') and (zframe_start = '0') and (s_SM_Ethernet = s_Idle)) then
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
				udp_frame(15) <= x"00"; -- differentiated services (6-bits) | explicit congestion notification (2-bits)
				udp_frame(16) <= std_logic_vector(to_unsigned(IP_HEADER_LENGTH + UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 16)(15 downto 8)); -- total length without MAC-header: entire packet size in bytes, including IP-header and payload-data. The minimum size is 46 bytes of user data (= 0x2e, header without data) and the maximum is 65,535 bytes
				udp_frame(17) <= std_logic_vector(to_unsigned(IP_HEADER_LENGTH + UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 16)(7 downto 0)); -- 20 bytes IP-header + 8 bytes UDP-header + 18 bytes UDP-payload = 46 bytes = 0x002e
				udp_frame(18) <= std_logic_vector(to_unsigned(packet_counter, 16))(15 downto 8);
				udp_frame(19) <= std_logic_vector(to_unsigned(packet_counter, 16))(7 downto 0);
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
	
				udp_frame(30) <= x"E0";
				udp_frame(31) <= x"00";
				udp_frame(32) <= x"01";
				udp_frame(33) <= x"81";
				

				-- UDP HEADER (8 bytes)
				udp_frame(34) <= std_logic_vector(to_unsigned(PTP_EVENT_PORT, 16)(15 downto 8));
				udp_frame(35) <= std_logic_vector(to_unsigned(PTP_EVENT_PORT, 16)(7 downto 0));
				udp_frame(36) <= std_logic_vector(to_unsigned(PTP_EVENT_PORT, 16)(15 downto 8));
				udp_frame(37) <= std_logic_vector(to_unsigned(PTP_EVENT_PORT, 16)(7 downto 0));
				udp_frame(38) <= std_logic_vector(to_unsigned(UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 16)(15 downto 8)); -- length (length of this UDP packet including header and data. Minimum 8 bytes)
				udp_frame(39) <= std_logic_vector(to_unsigned(UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 16)(7 downto 0));
				udp_frame(40) <= x"00"; -- checksum (0 is a valid CRC-value to ignore it)
				udp_frame(41) <= x"00";

				-- UDP PAYLOAD (18 bytes)
				udp_frame(42) <= x"00"; -- 0 majorSdoId + SyncMessage
				udp_frame(43) <= b"00010010"; -- 1 minorVersionPTP + majorVersionPTP = 0001 0010b = version 2
				udp_frame(44) <= std_logic_vector(to_unsigned(UDP_PAYLOAD_LENGTH, 16)(15 downto 8)); -- 2 messageLength MSB
				udp_frame(45) <= std_logic_vector(to_unsigned(UDP_PAYLOAD_LENGTH, 16)(7 downto 0)); -- 3 messageLength continued (44 bytes = 0x002c)
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
                udp_frame(72) <= std_logic_vector(to_unsigned(packet_counter, 16))(15 downto 8); -- 30 sequenceId
                udp_frame(73) <= std_logic_vector(to_unsigned(packet_counter, 16))(7 downto 0); -- 31 sequenceId
                udp_frame(74) <= x"00"; -- 32 controlField
                udp_frame(75) <= x"00"; -- 33 logMessageInterval
                udp_frame(76) <= x"00"; -- 34 0 originTimestamp seconds
                udp_frame(77) <= x"00"; -- 35 1 originTimestamp seconds
                udp_frame(78) <= timestamp_seconds_i(31 downto 24); -- 36 2 originTimestamp seconds 
                udp_frame(79) <= timestamp_seconds_i(23 downto 16); -- 37 3 originTimestamp seconds
                udp_frame(80) <= timestamp_seconds_i(15 downto 8); -- 38 4 originTimestamp seconds
                udp_frame(81) <= timestamp_seconds_i(7 downto 0); -- 39 5 originTimestamp seconds
                udp_frame(82) <= timestamp_nanoseconds_i(31 downto 24); -- 40 0 originTimestamp nanoseconds
                udp_frame(83) <= timestamp_nanoseconds_i(23 downto 16); -- 41 1 originTimestamp nanoseconds
                udp_frame(84) <= timestamp_nanoseconds_i(15 downto 8); -- 42 2 originTimestamp nanoseconds
                udp_frame(85) <= timestamp_nanoseconds_i(7 downto 0); -- 43 3 originTimestamp nanoseconds
                

				checksum_tmp                <= (others => '0');
				checksum_byte_count         <= 0;
				calculating_checksum        <= '1';
				
				udp_checksum_tmp            <= to_unsigned(17 + UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 32); -- 0x11 (protocol) + UDP-LENGTH (header+payload)
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
				if (udp_checksum_byte_count < (UDP_PSEUDO_HEADER_LENGTH + UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH)) then
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
					
					if (byte_counter = PACKET_LENGTH - 1) then
						-- stop transmitting
						s_SM_Ethernet <= s_End;
					end if;
					
					byte_counter <= byte_counter + 1;
				end if;
				
			elsif (s_SM_Ethernet = s_End) then
				tx_enable <= '0';
				tx_data <= "00000000";

				s_SM_Ethernet <= s_Idle;
			end if;
		end if;
	end process;
end Behavioral;
