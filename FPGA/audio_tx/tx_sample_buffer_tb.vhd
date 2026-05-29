-- Testbench for tx_sample_buffer.vhd + tx_transmitter.vhd
--
-- Mirrors the rx_ringbuffer testbench philosophy: independent tests using the
-- SIM_SAMPLE_RAM_BACKDOOR ports so the input frontends and the transmitter read
-- path can be checked in isolation.
--
--   Test A -- TDM serial demux (TDM_INPUT=true):
--     Drive serial TDM frames (known data) into the integrated demux, then read
--     sample_ram back via the backdoor. Expect linear slot layout:
--     MSB@+0, mid@+1, LSB@+2, pad@+3.
--
--   Test B -- parallel capture (TDM_INPUT=false):
--     Drive the parallel audio_in bus with known data, fs pulse, read RAM back.
--     Expect the same linear layout (so both frontends agree).
--
--   Test C -- transmitter read path:
--     Backdoor-fill a buffer DUT's RAM, configure tx_router for one stream, let
--     tx_transmitter assemble one packet, and check the RTP audio payload bytes
--     are L24 (3 bytes/sample, pad skipped) in MSB,mid,LSB order.
--
-- Known data: channel ch -> MSB=0x10+ch, mid=0xA0+ch, LSB=0x0C.
--
-- Run:
--   ghdl -a --std=08 tx_sample_buffer.vhd tx_router.vhd tx_transmitter.vhd tx_sample_buffer_tb.vhd
--   ghdl -e --std=08 tx_sample_buffer_tb
--   ghdl -r --std=08 tx_sample_buffer_tb --stop-time=2ms --ieee-asserts=disable

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tx_sample_buffer_tb is
end entity;

architecture sim of tx_sample_buffer_tb is

    constant CHANNELS_C    : integer := 8;   -- global_channel_count (one TDM frame)
    constant DEPTH_C       : integer := 8;   -- samples_per_channel_depth
    constant BPS_C         : integer := 3;   -- wire bytes per sample (L24)
    constant SLOT_BYTES_C  : integer := 4;
    constant CHANNEL_STRIDE_C : integer := SLOT_BYTES_C;
    constant SAMPLE_STRIDE_C  : integer := CHANNELS_C * SLOT_BYTES_C;
    constant TDM_INPUTS_C  : integer := 1;
    constant TDM_CHANNELS_C: integer := CHANNELS_C;
    constant SAMPLE_BITS_C : integer := BPS_C * 8;  -- 24

    constant CLK_PERIOD : time := 8 ns;   -- 125 MHz

    -- ===== shared =====
    signal sys_clk : std_logic := '0';
    signal reset_n : std_logic := '0';
    signal sim_done : boolean := false;

    -- ===== Test A: TDM demux DUT =====
    signal a_fs   : std_logic := '0';
    signal a_bclk : std_logic := '0';
    signal a_tdm  : std_logic_vector(TDM_INPUTS_C - 1 downto 0) := (others => '0');
    signal a_wr_ready : std_logic;
    signal a_wr_ptr   : std_logic_vector(15 downto 0);
    signal a_bd_rd_addr : unsigned(15 downto 0) := (others => '0');
    signal a_bd_rd_data : std_logic_vector(7 downto 0);
    signal a_data0 : std_logic_vector(7 downto 0);

    -- ===== Test B: parallel DUT =====
    signal b_fs    : std_logic := '0';
    signal b_audio : std_logic_vector(SAMPLE_BITS_C*CHANNELS_C - 1 downto 0) := (others => '0');
    signal b_wr_ready : std_logic;
    signal b_wr_ptr   : std_logic_vector(15 downto 0);
    signal b_bd_rd_addr : unsigned(15 downto 0) := (others => '0');
    signal b_bd_rd_data : std_logic_vector(7 downto 0);
    signal b_data0 : std_logic_vector(7 downto 0);

    -- ===== Test C: transmitter read path =====
    -- A backdoor-filled buffer feeds its data0_out into a tx_transmitter; we
    -- collect the emitted packet bytes and check the RTP audio payload.
    signal c_bd_wr_en   : std_logic := '0';
    signal c_bd_wr_addr : unsigned(15 downto 0) := (others => '0');
    signal c_bd_wr_data : std_logic_vector(7 downto 0) := (others => '0');
    signal c_rd_addr    : std_logic_vector(15 downto 0);
    signal c_rd_data    : std_logic_vector(7 downto 0);

    signal c_tx_clk      : std_logic := '0';
    signal c_tx_byte_sent: std_logic := '0';
    signal c_tx_enable   : std_logic;
    signal c_tx_data     : std_logic_vector(7 downto 0);
    signal c_tx_req      : std_logic;
    signal c_tx_allow    : std_logic := '0';
    signal c_start       : std_logic := '0';

    constant C_SPP    : integer := 2;  -- samples per packet per channel
    constant C_CHCNT  : integer := CHANNELS_C;
    -- ch_ids identity: ch_ids_i(63 downto 56)=ch0, ... 8 channels
    constant C_CHIDS  : std_logic_vector(63 downto 0) := x"0001020304050607";
    -- start address so that read_base = 0: rd_offset = SPP*CH*SLOT_BYTES
    constant C_START_ADDR : integer := C_SPP * C_CHCNT * SLOT_BYTES_C;

    signal c_done : boolean := false;

    signal a_done, b_done : boolean := false;

    -- expected bytes per channel: 0=MSB,1=mid,2=LSB
    type t_byte is array (0 to 2) of std_logic_vector(7 downto 0);
    type t_chbytes is array (0 to CHANNELS_C-1) of t_byte;
    function make_expected return t_chbytes is
        variable v : t_chbytes;
    begin
        for ch in 0 to CHANNELS_C-1 loop
            v(ch)(0) := std_logic_vector(to_unsigned(16#10# + ch, 8));
            v(ch)(1) := std_logic_vector(to_unsigned(16#A0# + ch, 8));
            v(ch)(2) := std_logic_vector(to_unsigned(16#0C#,      8));
        end loop;
        return v;
    end function;
    constant EXP : t_chbytes := make_expected;

begin

    clk_gen : process
    begin
        while not sim_done loop
            sys_clk <= '0'; wait for CLK_PERIOD/2;
            sys_clk <= '1'; wait for CLK_PERIOD/2;
        end loop;
        wait;
    end process;

    rst_gen : process
    begin
        reset_n <= '0';
        wait for 10*CLK_PERIOD;
        reset_n <= '1';
        wait;
    end process;

    -- ===================== Test A DUT: TDM demux =====================
    a_dut : entity work.tx_sample_buffer
        generic map (
            samples_per_channel_depth => DEPTH_C, global_channel_count => CHANNELS_C,
            bytes_per_sample => BPS_C, ENABLE_METERING => false,
            TDM_INPUT => true, TDM_INPUTS => TDM_INPUTS_C, TDM_CHANNELS => TDM_CHANNELS_C,
            SIM_SAMPLE_RAM_BACKDOOR => true
        )
        port map (
            sys_clk => sys_clk, reset_n => reset_n,
            audio_in => (others => '0'),
            fs_clk_i => a_fs, bclk_sync_i => a_bclk, tdm_in => a_tdm,
            wr_ptr_o => a_wr_ptr, wr_ready_o => a_wr_ready,
            read0Addr => (others => '0'), data0_out => a_data0,
            metering_signal_o => open, metering_clip_o => open, metering_clear_i => '0',
            dbg_wr_en_i => '0', dbg_wr_addr_i => (others=>'0'), dbg_wr_data_i => (others=>'0'),
            dbg_rd_addr_i => a_bd_rd_addr, dbg_rd_data_o => a_bd_rd_data
        );

    -- ===================== Test B DUT: parallel =====================
    b_dut : entity work.tx_sample_buffer
        generic map (
            samples_per_channel_depth => DEPTH_C, global_channel_count => CHANNELS_C,
            bytes_per_sample => BPS_C, ENABLE_METERING => false,
            TDM_INPUT => false, TDM_INPUTS => TDM_INPUTS_C, TDM_CHANNELS => TDM_CHANNELS_C,
            SIM_SAMPLE_RAM_BACKDOOR => true
        )
        port map (
            sys_clk => sys_clk, reset_n => reset_n,
            audio_in => b_audio,
            fs_clk_i => b_fs, bclk_sync_i => '0', tdm_in => (others=>'0'),
            wr_ptr_o => b_wr_ptr, wr_ready_o => b_wr_ready,
            read0Addr => (others => '0'), data0_out => b_data0,
            metering_signal_o => open, metering_clip_o => open, metering_clear_i => '0',
            dbg_wr_en_i => '0', dbg_wr_addr_i => (others=>'0'), dbg_wr_data_i => (others=>'0'),
            dbg_rd_addr_i => b_bd_rd_addr, dbg_rd_data_o => b_bd_rd_data
        );

    -- ===================== Test C DUTs =====================
    -- RAM holder: backdoor-written, data0_out driven by transmitter's read addr.
    c_buf : entity work.tx_sample_buffer
        generic map (
            samples_per_channel_depth => DEPTH_C, global_channel_count => CHANNELS_C,
            bytes_per_sample => BPS_C, ENABLE_METERING => false,
            TDM_INPUT => false, TDM_INPUTS => TDM_INPUTS_C, TDM_CHANNELS => TDM_CHANNELS_C,
            SIM_SAMPLE_RAM_BACKDOOR => true
        )
        port map (
            sys_clk => sys_clk, reset_n => reset_n,
            audio_in => (others => '0'),
            fs_clk_i => '0', bclk_sync_i => '0', tdm_in => (others=>'0'),
            wr_ptr_o => open, wr_ready_o => open,
            read0Addr => unsigned(c_rd_addr), data0_out => c_rd_data,
            metering_signal_o => open, metering_clip_o => open, metering_clear_i => '0',
            dbg_wr_en_i => c_bd_wr_en, dbg_wr_addr_i => c_bd_wr_addr, dbg_wr_data_i => c_bd_wr_data,
            dbg_rd_addr_i => (others=>'0'), dbg_rd_data_o => open
        );

    c_tx : entity work.tx_transmitter
        generic map (
            samples_per_channel_depth => DEPTH_C, global_channel_count => CHANNELS_C,
            bytes_per_sample => BPS_C
        )
        port map (
            sys_clk => sys_clk,
            src_mac_address => x"001122334455",
            src_ip_address  => x"C0A80101",
            dst_ip_address  => x"EF010203",
            tx_clk => c_tx_clk, tx_byte_sent => c_tx_byte_sent,
            sample_counter => (others=>'0'), sequence_id_in => (others=>'0'),
            tx_enable => c_tx_enable, tx_data => c_tx_data,
            channel_count_i => std_logic_vector(to_unsigned(C_CHCNT, 8)),
            samples_per_packet_per_channel_i => std_logic_vector(to_unsigned(C_SPP, 8)),
            sample_buffer_tx_start_addr_i => std_logic_vector(to_unsigned(C_START_ADDR, 16)),
            sample_ram_read_addr_o => c_rd_addr, sample_ram_data_in_i => c_rd_data,
            ch_ids_i => C_CHIDS, ssrc_i => x"DEADBEEF",
            start_i => c_start, tx_req_o => c_tx_req, tx_allow_i => c_tx_allow,
            mac_speed_i => "01"
        );

    -- transmitter tx_clk (use a slower clock so tx_byte_sent pacing is clean)
    c_tx_clk_gen : process
    begin
        while not sim_done loop
            c_tx_clk <= '0'; wait for CLK_PERIOD;
            c_tx_clk <= '1'; wait for CLK_PERIOD;
        end loop;
        wait;
    end process;

    -- ===================== Test A stimulus =====================
    -- Generate one TDM frame: fs pulse, then CHANNELS_C slots of 32 bclk each.
    -- Each slot carries 24 data bits MSB-first then 8 pad bits.
    test_a : process
        -- emit one bclk cycle with DIN held to `bit`
        procedure bclk_bit(bit : std_logic) is
        begin
            a_tdm(0) <= bit;
            a_bclk <= '0'; wait for 2*CLK_PERIOD;
            a_bclk <= '1'; wait for 2*CLK_PERIOD;
        end procedure;
        variable word : std_logic_vector(SAMPLE_BITS_C - 1 downto 0);
        variable got  : std_logic_vector(7 downto 0);
    begin
        wait until reset_n = '1';
        wait for 4*CLK_PERIOD;

        -- fs pulse (one bclk-ish wide) to align frame at channel 0
        a_fs <= '1';
        wait for 4*CLK_PERIOD;
        a_fs <= '0';
        wait for 4*CLK_PERIOD;

        -- stream all channels
        for ch in 0 to CHANNELS_C-1 loop
            word := EXP(ch)(0) & EXP(ch)(1) & EXP(ch)(2);  -- MSB..LSB, 24 bits
            -- 24 data bits MSB-first
            for b in SAMPLE_BITS_C - 1 downto 0 loop
                bclk_bit(word(b));
            end loop;
            -- 8 pad bits
            for b in 0 to 7 loop
                bclk_bit('0');
            end loop;
        end loop;

        -- let the write FSM drain
        wait for 200*CLK_PERIOD;

        -- read back RAM via backdoor (sample 0 region = addr 0.. for ch*4+b)
        for ch in 0 to CHANNELS_C-1 loop
            for b in 0 to 2 loop
                a_bd_rd_addr <= to_unsigned(ch*CHANNEL_STRIDE_C + b, 16);
                wait until rising_edge(sys_clk);
                wait until rising_edge(sys_clk);
                got := a_bd_rd_data;
                assert got = EXP(ch)(b)
                    report "A/DEMUX ch " & integer'image(ch) & " slot+" & integer'image(b) &
                           " got 0x" & to_hstring(got) & " expected 0x" & to_hstring(EXP(ch)(b))
                    severity error;
            end loop;
        end loop;
        report "Test A: TDM demux wrote linear MSB@+0, mid@+1, LSB@+2." severity note;
        a_done <= true;
        wait;
    end process;

    -- ===================== Test B stimulus =====================
    test_b : process
        variable got : std_logic_vector(7 downto 0);
    begin
        wait until reset_n = '1';
        wait for 4*CLK_PERIOD;

        -- Build parallel bus: per channel slice [SAMPLE_BITS-1:0] = MSB&mid&LSB
        for ch in 0 to CHANNELS_C-1 loop
            b_audio((ch+1)*SAMPLE_BITS_C - 1 downto ch*SAMPLE_BITS_C) <=
                EXP(ch)(0) & EXP(ch)(1) & EXP(ch)(2);
        end loop;
        wait for 4*CLK_PERIOD;

        -- fs pulse triggers the capture FSM
        b_fs <= '1';
        wait for 4*CLK_PERIOD;
        b_fs <= '0';
        wait for 200*CLK_PERIOD;

        for ch in 0 to CHANNELS_C-1 loop
            for b in 0 to 2 loop
                b_bd_rd_addr <= to_unsigned(ch*CHANNEL_STRIDE_C + b, 16);
                wait until rising_edge(sys_clk);
                wait until rising_edge(sys_clk);
                got := b_bd_rd_data;
                assert got = EXP(ch)(b)
                    report "B/PARALLEL ch " & integer'image(ch) & " slot+" & integer'image(b) &
                           " got 0x" & to_hstring(got) & " expected 0x" & to_hstring(EXP(ch)(b))
                    severity error;
            end loop;
        end loop;
        report "Test B: parallel capture wrote linear MSB@+0, mid@+1, LSB@+2." severity note;
        b_done <= true;
        wait;
    end process;

    -- ===================== Test C stimulus =====================
    test_c : process
        constant PKT_HEADER_LEN : integer := 14 + 20 + 12 + 8; -- 54
        type t_pkt is array (0 to 255) of std_logic_vector(7 downto 0);
        -- backdoor write one byte into the c_buf RAM
        procedure cpoke(addr : integer; val : std_logic_vector(7 downto 0)) is
        begin
            wait until rising_edge(sys_clk);
            c_bd_wr_en   <= '1';
            c_bd_wr_addr <= to_unsigned(addr, 16);
            c_bd_wr_data <= val;
            wait until rising_edge(sys_clk);
            c_bd_wr_en   <= '0';
        end procedure;
        variable byte_idx : integer;
        variable expv : std_logic_vector(7 downto 0);
        variable pkt  : t_pkt := (others => (others => '0'));
        variable idx  : integer := 0;
    begin
        wait until reset_n = '1';
        wait for 4*CLK_PERIOD;

        -- Fill RAM: C_SPP samples x CHANNELS_C channels, linear slots.
        -- sample s at base s*SAMPLE_STRIDE, channel ch at +ch*SLOT, MSB@+0..LSB@+2.
        for smp in 0 to C_SPP-1 loop
            for ch in 0 to CHANNELS_C-1 loop
                cpoke(smp*SAMPLE_STRIDE_C + ch*CHANNEL_STRIDE_C + 0, EXP(ch)(0)); -- MSB
                cpoke(smp*SAMPLE_STRIDE_C + ch*CHANNEL_STRIDE_C + 1, EXP(ch)(1)); -- mid
                cpoke(smp*SAMPLE_STRIDE_C + ch*CHANNEL_STRIDE_C + 2, EXP(ch)(2)); -- LSB
                -- pad @ +3 left 0
            end loop;
        end loop;

        wait for 10*CLK_PERIOD;

        -- Kick the transmitter assembly. Hold tx_allow so it streams out.
        c_tx_allow <= '1';
        c_start <= '1';
        wait for 4*CLK_PERIOD;
        c_start <= '0';

        -- Wait for the transmitter to request, then drive tx_byte_sent each
        -- tx_clk so read_addr advances and tx_data walks the packet RAM.
        wait until c_tx_req = '1';
        -- The first tx_data corresponds to read_addr=0. Each tx_clk with
        -- tx_byte_sent='1' advances by one. Capture a full header+payload.
        for n in 0 to PKT_HEADER_LEN + C_SPP*CHANNELS_C*BPS_C + 4 loop
            wait until rising_edge(c_tx_clk);
            if idx <= 255 then
                pkt(idx) := c_tx_data;
            end if;
            idx := idx + 1;
            c_tx_byte_sent <= '1';
            wait until rising_edge(c_tx_clk);
            c_tx_byte_sent <= '0';
        end loop;

        -- Verify audio payload: starts at PKT_HEADER_LEN, L24, sample-major then
        -- channel-major: sample0(ch0 MSB,mid,LSB; ch1 ...); sample1(...).
        byte_idx := PKT_HEADER_LEN;
        for smp in 0 to C_SPP-1 loop
            for ch in 0 to CHANNELS_C-1 loop
                for b in 0 to 2 loop
                    expv := EXP(ch)(b);
                    assert pkt(byte_idx) = expv
                        report "C/TX smp " & integer'image(smp) & " ch " & integer'image(ch) &
                               " byte " & integer'image(b) &
                               " got 0x" & to_hstring(pkt(byte_idx)) &
                               " expected 0x" & to_hstring(expv)
                        severity error;
                    byte_idx := byte_idx + 1;
                end loop;
            end loop;
        end loop;
        report "Test C: transmitter emitted L24 payload (3 bytes/sample, pad skipped)." severity note;

        c_done <= true;
        wait;
    end process;

    finish : process
    begin
        wait until a_done and b_done and c_done;
        wait for 10*CLK_PERIOD;
        report "tx_sample_buffer_tb (A+B+C) finished." severity note;
        sim_done <= true;
        wait;
    end process;

end architecture;
