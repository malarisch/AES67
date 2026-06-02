
-- This file was originally based of, but heavily modified:
--
-- UDP Audio Packet Sender
-- (c) 2025 Dr.-Ing. Christian Noeding
-- christian@noeding-online.de
-- Released under GNU General Public License v3
-- Source: https://www.github.com/xn--nding-jua/AES50_Transmitter
--
-- This file contains an ethernet-packet-generator to send individual bytes to an EthernetMAC directly.

library ieee;
use ieee.std_logic_1164.all;
use IEEE.NUMERIC_STD.all;

entity tx_transmitter is
  generic (
    samples_per_channel_depth : integer := 48; -- number of samples per channel to buffer
    global_channel_count      : integer := 16; -- number of channels to buffer
    bytes_per_sample          : integer := 3   -- number of bytes per sample (e.g., 3 for 24-bit audio)
  );
  port (
    sys_clk         : in std_logic;
    src_mac_address : in std_logic_vector(47 downto 0);
    src_ip_address  : in std_logic_vector(31 downto 0);
    dst_ip_address  : in std_logic_vector(31 downto 0);
    tx_clk          : in std_logic;
    tx_byte_sent    : in std_logic;
    sample_counter  : in std_logic_vector(31 downto 0) := (others => '0');
    sequence_id_in  : in unsigned(15 downto 0)         := (others => '0');

    tx_enable : out std_logic                    := '0';             -- TX valid
    tx_data   : out std_logic_vector(7 downto 0) := (others => '0'); -- data-octet

    channel_count_i                  : in std_logic_vector(7 downto 0)  := (others => '0');
    samples_per_packet_per_channel_i : in std_logic_vector(7 downto 0)  := (others => '0');
    sample_buffer_tx_start_addr_i    : in std_logic_vector(15 downto 0) := (others => '0');

    sample_ram_read_addr_o : out std_logic_vector(15 downto 0) := (others => '0');
    sample_ram_data_in_i   : in std_logic_vector(7 downto 0);

    -- max 8 channels per stream
    ch_ids_i : in std_logic_vector(63 downto 0) := (others => '0');
    ssrc_i   : in std_logic_vector(31 downto 0) := (others => '0');

    start_i : in std_logic := '0';

    tx_req_o    : out std_logic := '0';
    tx_allow_i  : in std_logic  := '0';
    mac_speed_i : in std_logic_vector(1 downto 0)

  );
end entity;
architecture Behavioral of tx_transmitter is

  constant MAC_HEADER_LENGTH  : integer := 14;
  constant IP_HEADER_LENGTH   : integer := 5 * (32 / 8); -- Header length always 20 bytes (5 * 32 bit words)
  constant AUDIO_START_SIGNAL : integer := 8;            -- 8 bytes at the beginning of the UDP payload reserved for RTP header (version, payload type, packet counter, sample counter, ssrc)
  constant UDP_HEADER_LENGTH  : integer := 12;
  -- calc dynamically constant UDP_PAYLOAD_LENGTH	: integer := AUDIO_START_SIGNAL + AUDIO_BUFFER_LENGTH; -- 8 start-bytes + x bytes for audio
  -- calc dynammically constant PACKET_LENGTH			: integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH;
  constant PACKET_HEADER_LENGTH : integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH + AUDIO_START_SIGNAL;
  -- Fixed (non-audio) byte count ahead of the L24 payload, reused by the length
  -- calculations so the audio product only appears once.
  constant FRAME_OVERHEAD : integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH + UDP_HEADER_LENGTH + AUDIO_START_SIGNAL;

  -- Two distinct strides (see tx_sample_buffer): the RAM stores 4-byte slots
  -- (power of two -> address shifts), but the RTP wire payload stays L24 = 3
  -- bytes per sample. SLOT_BYTES drives RAM addressing; WIRE_BYTES drives the
  -- payload length and the per-sample byte iteration count.
  constant SLOT_BYTES : integer := 4;
  constant WIRE_BYTES : integer := bytes_per_sample; -- 3 (L24)
  -- Must match tx_sample_buffer's AUDIO_BUFFER_LENGTH exactly.
  constant AUDIO_BUFFER_LENGTH : integer := samples_per_channel_depth * global_channel_count * SLOT_BYTES * 2;

  -- Frame-length signals: range-constrained so Quartus infers ~11-bit registers
  -- and adders/comparators instead of the default 32-bit integer width.
  signal PACKET_LENGTH         : integer range 0 to 1518 := 0;
  signal PACKET_LENGTH_MINUS_1 : integer range 0 to 1518 := 0; -- Pre-computed to break comparison critical path
  signal UDP_PAYLOAD_LENGTH    : integer range 0 to 1518 := 0;
  -- Pipeline register for audio_length multiplication (breaks channel_count_o critical path)
  -- Both inputs are 8-bit CSR values, so the product is bounded by 255*255; the
  -- range stays well under the default 32-bit integer width while never tripping
  -- a range check on a pathological CSR value.
  signal audio_samples_x_channels : integer range 0 to 255 * 255 := 0; -- samples * channels (Stage 1)
  -- Types
  type t_SM_Ethernet is (s_Idle, s_waitForAllow, s_Transmit, s_End, s_WaitDeassert);
  type t_SM_AssemblePacket is (s_A_Idle, s_A_CalcAudioLen, s_A_CalcValues, s_A_PrepFrame, s_A_Payload, s_A_WaitAckDone, s_SettleChecksum, s_FinalizeChecksum);
  type t_packet_ram is array (0 to 1518) of std_logic_vector(7 downto 0);

  signal SM_AssemblePacket : t_SM_AssemblePacket := s_A_Idle;

  signal read_base : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0;
  -- Helper declarations
  function get_header_byte(
    idx                    : integer;
    src_mac                : std_logic_vector(47 downto 0);
    src_ip                 : std_logic_vector(31 downto 0);
    dst_ip                 : std_logic_vector(31 downto 0);
    packet_ctr             : unsigned(15 downto 0);
    sample_ctr             : std_logic_vector(31 downto 0);
    samples_per_ch_per_pkt : std_logic_vector(7 downto 0);
    channel_count          : std_logic_vector(7 downto 0);
    ssrc                   : std_logic_vector(31 downto 0);
    total_length           : std_logic_vector(15 downto 0);
    udp_length             : std_logic_vector(15 downto 0)
  ) return std_logic_vector is
    --constant total_length	: std_logic_vector(15 downto 0) := std_logic_vector(to_unsigned(IP_HEADER_LENGTH + UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 16));
    --constant udp_length	: std_logic_vector(15 downto 0) := std_logic_vector(to_unsigned(UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 16));

  begin
    case idx is

        -- ethernet
      when 0  => return x"01";
      when 1  => return x"00";
      when 2  => return x"5E";
      when 3  => return b"0" & dst_ip(22 downto 16);
      when 4  => return dst_ip(15 downto 8);
      when 5  => return dst_ip(7 downto 0);
      when 6  => return src_mac(47 downto 40);
      when 7  => return src_mac(39 downto 32);
      when 8  => return src_mac(31 downto 24);
      when 9  => return src_mac(23 downto 16);
      when 10 => return src_mac(15 downto 8);
      when 11 => return src_mac(7 downto 0);
      when 12 => return x"08";
      when 13 => return x"00";

        -- ipv4
      when 14 => return x"45";
      when 15 => return x"00";
      when 16 => return total_length(15 downto 8);
      when 17 => return total_length(7 downto 0);
      when 18 => return std_logic_vector(packet_ctr(15 downto 8));
      when 19 => return std_logic_vector(packet_ctr(7 downto 0));
      when 20 => return x"00";
      when 21 => return x"00";
      when 22 => return x"80";
      when 23 => return x"11";
      when 24 => return x"00";
      when 25 => return x"00";
      when 26 => return src_ip(31 downto 24);
      when 27 => return src_ip(23 downto 16);
      when 28 => return src_ip(15 downto 8);
      when 29 => return src_ip(7 downto 0);
      when 30 => return dst_ip(31 downto 24);
      when 31 => return dst_ip(23 downto 16);
      when 32 => return dst_ip(15 downto 8);
      when 33 => return dst_ip(7 downto 0);

        -- udp
      when 34 => return x"13";
      when 35 => return x"8c";
      when 36 => return x"13";
      when 37 => return x"8c";
      when 38 => return udp_length(15 downto 8);
      when 39 => return udp_length(7 downto 0);
      when 40 => return x"00";
      when 41 => return x"00";

        -- rdp start
        -- bit 0,1: rdp version = 2 = b10
        -- bit 2: rdp padding = b0
        -- bit 3: rdp extension = b0
        -- bit 4,5,6,7: csrcount = b0
      when 42 => return x"80";

        -- payload type: audio data = 97 = 0x61
      when 43 => return x"61";

        -- packet counter (2 bytes, big-endian)
      when 44 => return std_logic_vector(packet_ctr(15 downto 8));
      when 45 => return std_logic_vector(packet_ctr(7 downto 0));

        -- sample counter (4 bytes, big-endian)

      when 46 => return sample_ctr(31 downto 24);
      when 47 => return sample_ctr(23 downto 16);
      when 48 => return sample_ctr(15 downto 8);
      when 49 => return sample_ctr(7 downto 0);
        -- ssrc
      when 50 => return ssrc(31 downto 24);
      when 51 => return ssrc(23 downto 16);
      when 52 => return ssrc(15 downto 8);
      when 53 => return ssrc(7 downto 0);

      when others => return x"00";
    end case;
  end function;

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
      byte_phase  <= '0';
    end if;
  end procedure;

  -- Wrapped RAM address for one payload byte. Mirrors the ping-pong buffer's
  -- 4-byte slot stride (SLOT_BYTES) and wraps within AUDIO_BUFFER_LENGTH.
  function wrap_sample_addr(
    base        : integer;
    sample_base : integer;
    ch_id       : integer;
    byte_offset : integer
  ) return integer is
    variable addr : integer;
  begin
    addr := base + sample_base + ch_id * SLOT_BYTES + byte_offset;
    if addr >= AUDIO_BUFFER_LENGTH then
      addr := addr - AUDIO_BUFFER_LENGTH;
    end if;
    return addr;
  end function;

  function finalize_checksum(sum_in : unsigned(31 downto 0)) return std_logic_vector is
    variable tmp                      : unsigned(31 downto 0) := sum_in;
  begin
    tmp := resize(tmp(15 downto 0), 32) + resize(tmp(31 downto 16), 32);
    tmp := resize(tmp(15 downto 0), 32) + resize(tmp(31 downto 16), 32);
    return std_logic_vector(not tmp(15 downto 0));
  end function;

  -- Double-buffered (ping-pong) sample RAMs
  signal packet_ram : t_packet_ram := (others => (others => '0'));

  signal packet_wr_en      : std_logic                    := '0';
  signal packet_wr_addr    : integer range 0 to 1518 - 1  := 0;
  signal packet_wr_data    : std_logic_vector(7 downto 0) := (others => '0');

  attribute ram_style : string;

  attribute ram_style of packet_ram : signal is "block";

  -- Packet RAM: Port A (sys_clk) write-only, Port B (tx_clk) for TX read.
  -- The checksums are folded by dedicated accumulator processes that tap the
  -- same Port A write stream, and are muxed into the Port B read at their byte
  -- offsets -- so neither checksum is ever stored in or read back from RAM.

  signal s_SM_Ethernet          : t_SM_Ethernet                                                                          := s_Idle;
  signal frame_write_index      : integer range 0 to 1518                                                                := 0;
  signal ip_checksum_acc        : unsigned(31 downto 0)                                                                  := (others => '0');
  signal ip_checksum_upper_byte : std_logic_vector(7 downto 0)                                                           := (others => '0');
  signal ip_checksum_byte_phase : std_logic                                                                              := '0';
  signal ip_checksum_value      : std_logic_vector(15 downto 0)                                                          := (others => '0');

  signal udp_checksum_acc        : unsigned(31 downto 0)         := (others => '0');
  signal udp_checksum_upper_byte : std_logic_vector(7 downto 0)  := (others => '0');
  signal udp_checksum_byte_phase : std_logic                     := '0';
  signal udp_checksum_value      : std_logic_vector(15 downto 0) := (others => '0');

  -- ============================================================
  -- Linear checksum accumulators (fed off the packet_wr stream)
  -- ------------------------------------------------------------
  -- Two independent processes fold the byte stream written to the packet RAM
  -- into running ones'-complement sums, exactly mirroring packet_ram_port_a's
  -- linear write. The IP process covers the 20-byte IPv4 header; the UDP
  -- process covers the UDP header + RTP/audio payload. Both checksum *fields*
  -- are written as 0x0000 by get_header_byte, so folding them in is a no-op and
  -- needs no skip logic. The finalized 16-bit values stay in registers and are
  -- muxed into the TX byte stream (packet_ram_port_b) -- they are never written
  -- back to RAM.
  -- csum_arm pulses for one sys_clk at the start of frame assembly to seed both
  -- accumulators (IP starts at 0, UDP at the pseudo-header sum).
  constant IP_HDR_FIRST  : integer := MAC_HEADER_LENGTH;                    -- 14
  constant IP_HDR_LAST   : integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH - 1; -- 33
  constant UDP_FIRST     : integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH; -- 34
  signal csum_arm        : std_logic            := '0';
  signal udp_pseudo_seed : unsigned(31 downto 0) := (others => '0');

  -- Packet byte offsets of the four checksum bytes. The TX read (packet_ram_port_b)
  -- substitutes the held checksum registers at these offsets instead of reading
  -- RAM, so the checksums never need to be written back to the packet RAM.
  constant IP_CSUM_HI_OFF  : integer := MAC_HEADER_LENGTH + 10;                -- 24
  constant IP_CSUM_LO_OFF  : integer := MAC_HEADER_LENGTH + 11;                -- 25
  constant UDP_CSUM_HI_OFF : integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH + 6; -- 40
  constant UDP_CSUM_LO_OFF : integer := MAC_HEADER_LENGTH + IP_HEADER_LENGTH + 7; -- 41

  signal start_i_sync1   : std_logic                                        := '0';

  -- ============================================================
  -- Pipeline registers to break sample_ram_read_addr critical path
  -- Original: ch_id mux + 2 multiplies + 3 adds (~10ns)
  -- Solution: Split into 2 stages:
  --   Stage 1: ch_id mux + sample_base computation → ch_id_reg, sample_base_addr
  --   Stage 2: Final address add + wrap → raw_addr_reg
  -- Total: 2 extra cycles latency (compensated by earlier pre-fetch)
  -- ============================================================
  signal ch_id_reg        : integer range 0 to 255                     := 0; -- Selected channel ID (Stage 1)
  signal sample_base_addr : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0; -- sample_index * stride
  signal byte_offset_reg  : integer range 0 to 2                       := 0; -- byte offset within sample
  signal raw_addr_reg     : integer range 0 to AUDIO_BUFFER_LENGTH - 1 := 0; -- Final address (Stage 2)

  -- Look-ahead counters for address pipeline (compute NEXT values one cycle early)
  signal next_channel_counter : integer range 0 to 7                             := 0;
  signal next_byte_counter    : integer range 0 to 2                             := 0;
  signal next_sample_index    : integer range 0 to samples_per_channel_depth - 1 := 0;

  -- Per-sample slot stride. sample_base_addr = sample_index * SAMPLE_STRIDE used
  -- to need a runtime multiply; instead next_sample_base accumulates this
  -- constant stride as next_sample_index increments, turning the multiplier into
  -- a constant-increment adder (saves LEs on the DSP-poor Cyclone 10LP).
  constant SAMPLE_STRIDE   : integer                                          := global_channel_count * SLOT_BYTES;
  signal   next_sample_base : integer range 0 to AUDIO_BUFFER_LENGTH - 1       := 0;

  signal tx_ack       : std_logic := '0';
  signal tx_ack_sync1 : std_logic := '0';
  signal tx_ack_sync2 : std_logic := '0';

  signal tx_frame_start       : std_logic               := '0';
  signal tx_frame_start_sync1 : std_logic               := '0';
  signal tx_frame_start_sync2 : std_logic               := '0';
  signal read_addr            : integer range 0 to 1532 := 0;
  signal tx_done_reg          : std_logic               := '0';

  -- TX read-port pipeline. The RAM read is kept unconditional (ram_q <=
  -- packet_ram(eff_addr)) so Quartus infers a real block-RAM read port -- the
  -- checksum mux must NOT sit on the RAM output inside the clocked process or
  -- inference breaks. Instead, csum_sel carries the eff_addr classification
  -- registered in lockstep with ram_q, and tx_data is selected combinationally
  -- outside the process. Latency is unchanged vs. the old direct-registered
  -- read (one cycle: address -> ram_q/csum_sel -> combinational tx_data).
  signal ram_q    : std_logic_vector(7 downto 0) := (others => '0');
  signal csum_sel : integer range 0 to 4         := 0; -- 0=RAM, 1=IPhi 2=IPlo 3=UDPhi 4=UDPlo
begin

  -- Packet RAM Port A (sys_clk): write-only during assembly.
  packet_ram_port_a : process (sys_clk)
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
  ip_checksum_proc : process (sys_clk)
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
  -- folds every byte from UDP_FIRST onward (UDP header + RTP + L24 payload).
  -- The UDP checksum field (write addr 40/41) carries 0x00, folding in as zero.
  udp_checksum_proc : process (sys_clk)
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

  -- Port B (tx_clk): TX read
  -- Synchronous read: address register -> RAM -> registered output (tx_data)
  -- At the four checksum byte offsets the held checksum registers are muxed in
  -- instead of the RAM contents. ip_/udp_checksum_value are computed in sys_clk
  -- and are stable long before tx_ack rises (assembly finishes first), so they
  -- cross into tx_clk as quasi-static values without an extra synchronizer.
  packet_ram_port_b : process (tx_clk)
    variable eff_addr : integer range 0 to 1532;
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
      -- combinational mux below picks the checksum byte on the cycle ram_q holds
      -- that address's RAM data.
      case eff_addr is
        when IP_CSUM_HI_OFF  => csum_sel <= 1;
        when IP_CSUM_LO_OFF  => csum_sel <= 2;
        when UDP_CSUM_HI_OFF => csum_sel <= 3;
        when UDP_CSUM_LO_OFF => csum_sel <= 4;
        when others          => csum_sel <= 0;
      end case;

      if tx_ack = '1' and tx_allow_i = '1' then
        tx_enable <= '1';
        if (mac_speed_i = "10" and read_addr < PACKET_LENGTH - 1) or (mac_speed_i /= "10" and read_addr < PACKET_LENGTH) then
          tx_done_reg <= '0';
          if tx_byte_sent = '1' then
            read_addr <= read_addr + 1;
          end if;
        else
          tx_enable   <= '0';
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
  -- same byte; the checksum registers (ip_/udp_checksum_value) are quasi-static
  -- across a frame, so this mux carries no extra CDC concern.
  with csum_sel select tx_data <=
    ip_checksum_value(15 downto 8)  when 1,
    ip_checksum_value(7 downto 0)   when 2,
    udp_checksum_value(15 downto 8) when 3,
    udp_checksum_value(7 downto 0)  when 4,
    ram_q                           when others;

  sys_clock_process : process (sys_clk)
    variable header_data : std_logic_vector(7 downto 0);
    -- Bounds cover the worst-case 8-bit CSR inputs so a pathological value can
    -- never trip a range error; real values are far smaller. rd_offset uses the
    -- fixed global_channel_count generic, audio_len follows audio_samples_x_channels.
    variable rd_offset   : integer range 0 to 255 * global_channel_count * SLOT_BYTES;
    variable audio_len   : integer range 0 to 255 * 255 * WIRE_BYTES;
  begin
    if rising_edge(sys_clk) then
      packet_wr_en <= '0'; -- default: no write (overridden when needed)
      csum_arm     <= '0'; -- default: accumulators free-run; pulsed in s_A_CalcValues

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
          -- Pipeline Stage 2: multiply by WIRE_BYTES (L24=3) and compute final
          -- lengths. The payload is on the wire, so it uses WIRE_BYTES, not the
          -- 4-byte RAM slot stride.
        when s_A_CalcAudioLen =>
          -- Single L24 payload product; the rest are constant offsets.
          audio_len := audio_samples_x_channels * WIRE_BYTES;
          UDP_PAYLOAD_LENGTH <= AUDIO_START_SIGNAL + audio_len;
          PACKET_LENGTH      <= FRAME_OVERHEAD + audio_len;
          -- Pre-compute PACKET_LENGTH - 1 to remove subtraction from critical comparison path
          PACKET_LENGTH_MINUS_1 <= FRAME_OVERHEAD + audio_len - 1;
          SM_AssemblePacket     <= s_A_CalcValues;
        when s_A_CalcValues =>
          -- Compute read base: go back samples_per_packet sample periods from write pointer.
          -- RAM addressing uses the 4-byte slot stride.
          rd_offset := to_integer(unsigned(samples_per_packet_per_channel_i)) * global_channel_count * SLOT_BYTES;
          if to_integer(unsigned(sample_buffer_tx_start_addr_i)) >= rd_offset then
            read_base <= to_integer(unsigned(sample_buffer_tx_start_addr_i)) - rd_offset;
          else
            read_base <= to_integer(unsigned(sample_buffer_tx_start_addr_i)) - rd_offset + AUDIO_BUFFER_LENGTH;
          end if;

          frame_write_index <= 0;

          -- Seed the UDP accumulator with the UDP/IPv4 pseudo-header sum. The
          -- dedicated udp_checksum_proc loads this on the next edge (csum_arm)
          -- and then folds every byte written from UDP_FIRST onward. The IP
          -- accumulator seeds at 0 and folds the 20 header bytes directly, so
          -- no pre-computed IP partial sum is needed anymore.
          udp_pseudo_seed <= resize(unsigned(src_ip_address(31 downto 16)), 32)
            + resize(unsigned(src_ip_address(15 downto 0)), 32)
            + resize(unsigned(dst_ip_address(31 downto 16)), 32)
            + resize(unsigned(dst_ip_address(15 downto 0)), 32)
            + resize(to_unsigned(16#0011#, 16), 32)
            + resize(to_unsigned(UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 16), 32);
          -- Arm both accumulators (clear IP, load UDP seed) before the first
          -- header byte is written in s_A_PrepFrame.
          csum_arm          <= '1';
          SM_AssemblePacket <= s_A_PrepFrame;

        when s_A_PrepFrame =>

          packet_wr_en   <= '1';
          packet_wr_addr <= frame_write_index;
          if (frame_write_index < PACKET_HEADER_LENGTH) then

            header_data := get_header_byte(frame_write_index, src_mac_address, src_ip_address,
              dst_ip_address, sequence_id_in, sample_counter, samples_per_packet_per_channel_i,
              channel_count_i, ssrc_i, std_logic_vector(to_unsigned(IP_HEADER_LENGTH + UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 16)),
              std_logic_vector(to_unsigned(UDP_HEADER_LENGTH + UDP_PAYLOAD_LENGTH, 16)));

            packet_wr_data <= header_data;

            -- ===== ADDRESS PIPELINE PRE-FETCH (4 stages to account for pipeline latency) =====
            -- Need 4 pre-fetch cycles: Stage 1 (ch_id/base), Stage 2 (addr compute),
            -- Stage 3 (addr output to RAM port), Stage 4 (RAM registered read)
            if (frame_write_index = PACKET_HEADER_LENGTH - 4) then
              -- Pipeline Stage 1 setup: Initialize for first byte (MSB of ch0, sample 0)
              ch_id_reg        <= to_integer(unsigned(ch_ids_i(63 downto 56)));
              sample_base_addr <= 0; -- sample_index=0, so base=0
              byte_offset_reg  <= 0; -- MSB first @ slot offset 0 (linear layout)
            end if;
            if (frame_write_index = PACKET_HEADER_LENGTH - 3) then
              -- Pipeline Stage 2: Compute address for first byte
              raw_addr_reg <= wrap_sample_addr(read_base, sample_base_addr, ch_id_reg, byte_offset_reg);
              -- Setup Stage 1 for second byte
              ch_id_reg        <= to_integer(unsigned(ch_ids_i(63 downto 56)));
              sample_base_addr <= 0;
              byte_offset_reg  <= 1; -- middle byte @ +1
            end if;
            if (frame_write_index = PACKET_HEADER_LENGTH - 2) then
              -- Stage 3: Output first address to RAM
              sample_ram_read_addr_o <= std_logic_vector(to_unsigned(raw_addr_reg, 16));
              -- Pipeline Stage 2: Compute address for second byte
              raw_addr_reg <= wrap_sample_addr(read_base, sample_base_addr, ch_id_reg, byte_offset_reg);
              -- Setup Stage 1 for third byte
              ch_id_reg        <= to_integer(unsigned(ch_ids_i(63 downto 56)));
              sample_base_addr <= 0;
              byte_offset_reg  <= 2; -- LSB byte @ +2
            end if;
            if (frame_write_index = PACKET_HEADER_LENGTH - 1) then
              -- Stage 4: RAM is now reading first byte; output second address
              sample_ram_read_addr_o <= std_logic_vector(to_unsigned(raw_addr_reg, 16));
              -- Pipeline Stage 2: Compute address for third byte (ch0 LSB)
              raw_addr_reg <= wrap_sample_addr(read_base, sample_base_addr, ch_id_reg, byte_offset_reg);
              -- Setup Stage 1 for fourth byte (ch1 MSB)
              ch_id_reg        <= to_integer(unsigned(ch_ids_i(55 downto 48)));
              sample_base_addr <= 0;
              byte_offset_reg  <= 0; -- MSB @ offset 0 (next channel)
              -- next_* at HEADER-1 feeds Stage 1 at HEADER, result appears at PI=4 (ch1 MID)
              next_byte_counter    <= 1;
              next_channel_counter <= 1;
              next_sample_index    <= 0;
              next_sample_base     <= 0; -- sample 0 -> base 0; accumulates +SAMPLE_STRIDE per sample
            end if;
          else
            -- ===== PIPELINED AUDIO PAYLOAD ASSEMBLY =====
            -- Pipeline: counter → ch_id_reg+base (Stage 1) → raw_addr_reg (Stage 2) → output
            -- Each cycle: output data from 2 cycles ago, compute address for 2 cycles ahead

            packet_wr_data <= sample_ram_data_in_i;

            -- Stage 2 OUTPUT: Address computed last cycle
            sample_ram_read_addr_o <= std_logic_vector(to_unsigned(raw_addr_reg, 16));

            -- Stage 2 COMPUTE: Use registered ch_id and base from Stage 1
            raw_addr_reg <= wrap_sample_addr(read_base, sample_base_addr, ch_id_reg, byte_offset_reg);

            -- Stage 1: Compute ch_id and base for look-ahead counters
            case next_channel_counter is
              when 0      => ch_id_reg      <= to_integer(unsigned(ch_ids_i(63 downto 56)));
              when 1      => ch_id_reg      <= to_integer(unsigned(ch_ids_i(55 downto 48)));
              when 2      => ch_id_reg      <= to_integer(unsigned(ch_ids_i(47 downto 40)));
              when 3      => ch_id_reg      <= to_integer(unsigned(ch_ids_i(39 downto 32)));
              when 4      => ch_id_reg      <= to_integer(unsigned(ch_ids_i(31 downto 24)));
              when 5      => ch_id_reg      <= to_integer(unsigned(ch_ids_i(23 downto 16)));
              when 6      => ch_id_reg      <= to_integer(unsigned(ch_ids_i(15 downto 8)));
              when 7      => ch_id_reg      <= to_integer(unsigned(ch_ids_i(7 downto 0)));
              when others => ch_id_reg <= 0;
            end case;
            -- sample_base = sample_index * SAMPLE_STRIDE, tracked by the
            -- next_sample_base accumulator instead of a runtime multiply.
            sample_base_addr <= next_sample_base;
            -- Linear slot layout: byte 0=MSB@+0, 1=mid@+1, 2=LSB@+2.
            byte_offset_reg <= next_byte_counter;

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
                  -- Constant-increment adder mirrors next_sample_index * SAMPLE_STRIDE.
                  next_sample_base <= next_sample_base + SAMPLE_STRIDE;
                else
                  next_sample_index <= 0;
                  next_sample_base  <= 0;
                end if;
              end if;
            end if;

          end if;

          -- Checksums are now folded by the dedicated ip_checksum_proc /
          -- udp_checksum_proc, which listen to this same packet_wr stream. No
          -- inline fold here.
          if (frame_write_index = PACKET_LENGTH_MINUS_1) then
            -- Last byte's packet_wr_* is registered on this edge; the dedicated
            -- accumulators fold it one edge later. s_SettleChecksum absorbs that
            -- one-cycle lag before we read the final accumulator value.
            SM_AssemblePacket <= s_SettleChecksum;
          else
            frame_write_index <= frame_write_index + 1;
          end if;

        when s_SettleChecksum =>
          -- Wait one cycle so the final folded byte has landed in the accumulators.
          SM_AssemblePacket <= s_FinalizeChecksum;

        when s_FinalizeChecksum =>
          -- Fold the carries and one's-complement. Values stay in registers and
          -- are muxed into the TX byte stream (packet_ram_port_b); they are no
          -- longer written back to the packet RAM.
          ip_checksum_value  <= finalize_checksum(ip_checksum_acc);
          udp_checksum_value <= finalize_checksum(udp_checksum_acc);
          SM_AssemblePacket  <= s_A_Payload;

        when s_A_Payload =>
          tx_frame_start <= '1';
          -- Phase 3: tx_process saw our request and acked
          if (tx_ack_sync2 = '1') then
            tx_frame_start    <= '0'; -- deassert request
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
  tx_process : process (tx_clk)
  begin
    if rising_edge(tx_clk) then
      tx_frame_start_sync1 <= tx_frame_start;
      tx_frame_start_sync2 <= tx_frame_start_sync1;
      case s_SM_Ethernet is
        when s_Idle =>
          tx_ack   <= '0';
          tx_req_o <= '0';
          if (tx_frame_start_sync2 = '1') then
            tx_ack        <= '1';
            s_SM_Ethernet <= s_waitForAllow;
            tx_req_o      <= '1'; -- signal packet ready for transmission

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
          tx_req_o      <= '0';
        when s_WaitDeassert =>
          -- Phase 4: wait for sys_clock_process to deassert tx_frame_start
          if (tx_frame_start_sync2 = '0') then
            tx_ack        <= '0'; -- signal completion
            s_SM_Ethernet <= s_Idle;
          end if;
      end case;
    end if;
  end process tx_process;
end Behavioral;