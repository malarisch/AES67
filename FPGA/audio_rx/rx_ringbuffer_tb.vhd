-- Testbench for rx_ringbuffer.vhd
--
-- Scope: byte-order verification of the RX ring buffer, split into two
-- INDEPENDENT tests using the SIM_SAMPLE_RAM_BACKDOOR sample_ram ports:
--
--   Test A -- Playout paths (parser NOT involved):
--     The backdoor write port loads a known sample into sample_ram for every
--     channel (linear slot layout: MSB@+0, mid@+1, LSB@+2, pad@+3). Then a
--     frame-sync pulse + bit clock drive the playout. We check:
--       * dut_par.audio_out  -> 24-bit word per channel == known value
--       * dut_tdm.tdm_out    -> serial byte order == MSB, mid, LSB, pad
--
--   Test B -- Packet parser (playout NOT relied upon):
--     A synthetic RTP frame is fed through the parser via a modelled eth RAM
--     (synchronous, 1-cycle read latency, exactly like FPGA/.../eth_ram.vhd).
--     Afterwards the backdoor read port walks sample_ram and checks the parser
--     stored each channel's bytes in linear order MSB@+0, mid@+1, LSB@+2.
--
-- Known data: channel ch -> MSB=0x10+ch, mid=0xA0+ch, LSB=0x0C. Distinct per
-- channel and per byte position, so any byte swap is impossible to miss.
--
-- Run:
--   ghdl -a --std=08 rx_ringbuffer.vhd rx_ringbuffer_tb.vhd
--   ghdl -e --std=08 rx_ringbuffer_tb
--   ghdl -r --std=08 rx_ringbuffer_tb --stop-time=300us --ieee-asserts=disable

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity rx_ringbuffer_tb is
end entity;

architecture sim of rx_ringbuffer_tb is

    -- Reduced geometry for fast simulation. bytes_per_sample=3 and the 4-byte
    -- slot (the byte-order-relevant part) match the real design.
    constant CHANNELS_C    : integer := 4;   -- global_channel_count
    constant DEPTH_C       : integer := 8;   -- audio_buffer_sample_depth
    constant BPS_C         : integer := 3;   -- bytes_per_sample (L24)
    constant MAX_STREAMS_C : integer := 1;
    constant TDM_OUTPUTS_C : integer := 1;
    constant TDM_CHANNELS_C: integer := CHANNELS_C;

    -- Derived RAM geometry (mirrors the DUT's internal constants).
    constant SLOT_BYTES_C    : integer := 4;
    constant CHANNEL_STRIDE_C: integer := SLOT_BYTES_C;          -- 4

    constant CLK_PERIOD : time := 8 ns;   -- 125 MHz sys_clk

    -- Frame layout offsets the parser expects (byte index inside eth RAM).
    constant OFF_LEN_HI  : integer := 16;
    constant OFF_LEN_LO  : integer := 17;
    constant OFF_DSTIP   : integer := 30;  -- 4 bytes 30..33
    constant OFF_DSTPORT : integer := 36;  -- 2 bytes 36..37
    constant OFF_MCLOCK  : integer := 46;  -- 4 bytes 46..49
    constant OFF_PAYLOAD : integer := 54;

    constant STREAM_IP   : std_logic_vector(31 downto 0) := x"EF010203";
    constant STREAM_PORT : std_logic_vector(15 downto 0) := x"1388";
    constant SAMPLES_PER_CH : integer := 1;

    type t_eth_ram is array (0 to 255) of std_logic_vector(7 downto 0);
    signal eth_ram : t_eth_ram := (others => (others => '0'));

    -- ===== shared stimulus =====
    signal sys_clk : std_logic := '0';
    signal reset_n : std_logic := '0';

    signal media_clock : std_logic_vector(31 downto 0) := (others => '0');
    signal fs_clk      : std_logic := '0';
    signal bclk        : std_logic := '0';
    signal packet_ready: std_logic := '0';

    signal cfg_wr_clk : std_logic := '0';
    signal cfg_wr_en  : std_logic := '0';
    signal cfg_addr   : std_logic_vector(7 downto 0) := (others => '0');
    signal cfg_data   : std_logic_vector(7 downto 0) := (others => '0');
    signal metering_clear : std_logic := '0';

    -- ===== Test A (playout) DUTs: filled via backdoor, parser idle =====
    signal a_par_rd_addr : unsigned(10 downto 0);
    signal a_par_audio   : std_logic_vector(BPS_C*8*CHANNELS_C - 1 downto 0);
    signal a_par_tdm     : std_logic_vector(TDM_OUTPUTS_C - 1 downto 0);
    signal a_par_msig, a_par_mclip : std_logic_vector(CHANNELS_C - 1 downto 0);

    signal a_tdm_rd_addr : unsigned(10 downto 0);
    signal a_tdm_audio   : std_logic_vector(BPS_C*8*CHANNELS_C - 1 downto 0);
    signal a_tdm_tdm     : std_logic_vector(TDM_OUTPUTS_C - 1 downto 0);
    signal a_tdm_msig, a_tdm_mclip : std_logic_vector(CHANNELS_C - 1 downto 0);

    -- backdoor write bus (shared by both Test-A DUTs)
    signal bd_wr_en   : std_logic := '0';
    signal bd_wr_addr : unsigned(13 downto 0) := (others => '0');
    signal bd_wr_data : std_logic_vector(7 downto 0) := (others => '0');

    -- ===== Test B (parser) DUT: filled via parser, read back via backdoor =====
    signal b_rd_addr_eth : unsigned(10 downto 0);
    signal b_eth_data    : std_logic_vector(7 downto 0);
    signal b_audio       : std_logic_vector(BPS_C*8*CHANNELS_C - 1 downto 0);
    signal b_tdm         : std_logic_vector(TDM_OUTPUTS_C - 1 downto 0);
    signal b_msig, b_mclip : std_logic_vector(CHANNELS_C - 1 downto 0);

    signal bd_rd_addr : unsigned(13 downto 0) := (others => '0');
    signal bd_rd_data : std_logic_vector(7 downto 0);

    signal a_done : boolean := false;
    signal a_tdm_done : boolean := false;
    signal b_done : boolean := false;
    signal sim_done : boolean := false;

    -- Expected per-channel bytes. index 0 = MSB, 1 = mid, 2 = LSB.
    type t_byte is array (0 to 2) of std_logic_vector(7 downto 0);
    type t_chbytes is array (0 to CHANNELS_C-1) of t_byte;
    function make_expected return t_chbytes is
        variable v : t_chbytes;
    begin
        for ch in 0 to CHANNELS_C-1 loop
            v(ch)(0) := std_logic_vector(to_unsigned(16#10# + ch, 8)); -- MSB
            v(ch)(1) := std_logic_vector(to_unsigned(16#A0# + ch, 8)); -- mid
            v(ch)(2) := std_logic_vector(to_unsigned(16#0C#,      8)); -- LSB
        end loop;
        return v;
    end function;
    constant EXP : t_chbytes := make_expected;
    function exp_word(ch : integer) return std_logic_vector is
    begin
        return EXP(ch)(0) & EXP(ch)(1) & EXP(ch)(2);
    end function;

begin

    clk_gen : process
    begin
        while not sim_done loop
            sys_clk <= '0'; wait for CLK_PERIOD/2;
            sys_clk <= '1'; wait for CLK_PERIOD/2;
        end loop;
        wait;
    end process;

    -- Reset release
    rst_gen : process
    begin
        reset_n <= '0';
        wait for 10*CLK_PERIOD;
        reset_n <= '1';
        wait;
    end process;

    -- modelled eth frame RAM for Test B: synchronous 1-cycle read latency
    eth_b_rd : process(sys_clk)
    begin
        if rising_edge(sys_clk) then
            b_eth_data <= eth_ram(to_integer(b_rd_addr_eth));
        end if;
    end process;

    -- ============================================================
    -- Test-A DUTs: backdoor-filled, parser kept idle (packet_ready=0)
    -- ============================================================
    a_dut_par : entity work.rx_ringbuffer
        generic map (
            audio_buffer_sample_depth => DEPTH_C, global_channel_count => CHANNELS_C,
            bytes_per_sample => BPS_C, max_streams => MAX_STREAMS_C,
            ENABLE_METERING => true, PARALLEL_OUT => true,
            TDM_OUTPUTS => TDM_OUTPUTS_C, TDM_CHANNELS => TDM_CHANNELS_C,
            SIM_SAMPLE_RAM_BACKDOOR => true
        )
        port map (
            sys_clk => sys_clk, reset_n => reset_n,
            audio_out => a_par_audio, tdm_out => a_par_tdm,
            fs_clk_sync_i => fs_clk, bclk_sync_i => bclk, media_clock_i => media_clock,
            eth_read_addr_o => a_par_rd_addr, eth_read_data_i => (others => '0'),
            packet_ready_i => '0',
            stream_config_wr_clk_i => '0', stream_config_wr_en_i => '0',
            stream_config_addr_i => (others=>'0'), stream_config_data_i => (others=>'0'),
            metering_signal_o => a_par_msig, metering_clip_o => a_par_mclip,
            metering_clear_i => '0',
            dbg_wr_en_i => bd_wr_en, dbg_wr_addr_i => bd_wr_addr, dbg_wr_data_i => bd_wr_data,
            dbg_rd_addr_i => (others=>'0'), dbg_rd_data_o => open
        );

    a_dut_tdm : entity work.rx_ringbuffer
        generic map (
            audio_buffer_sample_depth => DEPTH_C, global_channel_count => CHANNELS_C,
            bytes_per_sample => BPS_C, max_streams => MAX_STREAMS_C,
            ENABLE_METERING => false, PARALLEL_OUT => false,
            TDM_OUTPUTS => TDM_OUTPUTS_C, TDM_CHANNELS => TDM_CHANNELS_C,
            SIM_SAMPLE_RAM_BACKDOOR => true
        )
        port map (
            sys_clk => sys_clk, reset_n => reset_n,
            audio_out => a_tdm_audio, tdm_out => a_tdm_tdm,
            fs_clk_sync_i => fs_clk, bclk_sync_i => bclk, media_clock_i => media_clock,
            eth_read_addr_o => a_tdm_rd_addr, eth_read_data_i => (others => '0'),
            packet_ready_i => '0',
            stream_config_wr_clk_i => '0', stream_config_wr_en_i => '0',
            stream_config_addr_i => (others=>'0'), stream_config_data_i => (others=>'0'),
            metering_signal_o => a_tdm_msig, metering_clip_o => a_tdm_mclip,
            metering_clear_i => '0',
            dbg_wr_en_i => bd_wr_en, dbg_wr_addr_i => bd_wr_addr, dbg_wr_data_i => bd_wr_data,
            dbg_rd_addr_i => (others=>'0'), dbg_rd_data_o => open
        );

    -- ============================================================
    -- Test-B DUT: parser-filled, read back via backdoor
    -- ============================================================
    b_dut : entity work.rx_ringbuffer
        generic map (
            audio_buffer_sample_depth => DEPTH_C, global_channel_count => CHANNELS_C,
            bytes_per_sample => BPS_C, max_streams => MAX_STREAMS_C,
            ENABLE_METERING => true, PARALLEL_OUT => true,
            TDM_OUTPUTS => TDM_OUTPUTS_C, TDM_CHANNELS => TDM_CHANNELS_C,
            SIM_SAMPLE_RAM_BACKDOOR => true
        )
        port map (
            sys_clk => sys_clk, reset_n => reset_n,
            audio_out => b_audio, tdm_out => b_tdm,
            fs_clk_sync_i => '0', bclk_sync_i => '0', media_clock_i => media_clock,
            eth_read_addr_o => b_rd_addr_eth, eth_read_data_i => b_eth_data,
            packet_ready_i => packet_ready,
            stream_config_wr_clk_i => cfg_wr_clk, stream_config_wr_en_i => cfg_wr_en,
            stream_config_addr_i => cfg_addr, stream_config_data_i => cfg_data,
            metering_signal_o => b_msig, metering_clip_o => b_mclip,
            metering_clear_i => '0',
            dbg_wr_en_i => '0', dbg_wr_addr_i => (others=>'0'), dbg_wr_data_i => (others=>'0'),
            dbg_rd_addr_i => bd_rd_addr, dbg_rd_data_o => bd_rd_data
        );

    -- ============================================================
    -- Test A: drive the playout from backdoor-loaded RAM
    -- ============================================================
    test_a : process
        -- backdoor write of one byte
        procedure bd_poke(addr : integer; val : std_logic_vector(7 downto 0)) is
        begin
            wait until rising_edge(sys_clk);
            bd_wr_en   <= '1';
            bd_wr_addr <= to_unsigned(addr, 14);
            bd_wr_data <= val;
            wait until rising_edge(sys_clk);
            bd_wr_en   <= '0';
        end procedure;
    begin
        wait until reset_n = '1';
        wait for 4*CLK_PERIOD;
        media_clock <= (others => '0');   -- read pointer base = 0

        -- Load known data: channel ch -> MSB@ch*4+0, mid@+1, LSB@+2, pad@+3=0
        for ch in 0 to CHANNELS_C-1 loop
            bd_poke(ch*CHANNEL_STRIDE_C + 0, EXP(ch)(0));
            bd_poke(ch*CHANNEL_STRIDE_C + 1, EXP(ch)(1));
            bd_poke(ch*CHANNEL_STRIDE_C + 2, EXP(ch)(2));
            bd_poke(ch*CHANNEL_STRIDE_C + 3, x"00");
        end loop;
        wait for 4*CLK_PERIOD;

        -- Trigger one playout frame: fs rising edge arms both paths.
        fs_clk <= '1';
        wait for 2*CLK_PERIOD;
        fs_clk <= '0';

        -- Let the parallel scan complete (a few cycles per channel byte).
        wait for 200*CLK_PERIOD;

        -- ---- Parallel path check ----
        for ch in 0 to CHANNELS_C-1 loop
            assert a_par_audio((ch+1)*BPS_C*8 - 1 downto ch*BPS_C*8) = exp_word(ch)
                report "A/PAR ch " & integer'image(ch) & " got 0x" &
                       to_hstring(a_par_audio((ch+1)*BPS_C*8 - 1 downto ch*BPS_C*8)) &
                       " expected 0x" & to_hstring(exp_word(ch)) severity error;
        end loop;
        report "Test A: PARALLEL path byte order checked." severity note;

        a_done <= true;
        wait;
    end process;

    -- TDM serial collector for Test A. Runs bclk and gathers bits.
    test_a_tdm : process
        variable bitcnt  : integer := 0;
        variable curb    : std_logic_vector(7 downto 0) := (others => '0');
        type t_cap is array (0 to CHANNELS_C*4 - 1) of std_logic_vector(7 downto 0);
        variable cap     : t_cap := (others => (others => '0'));
        variable bytecnt : integer := 0;
        variable expv    : std_logic_vector(7 downto 0);
        variable ok      : boolean := true;
    begin
        wait until reset_n = '1';
        wait until fs_clk = '1';   -- the Test-A playout trigger
        -- Skip one byte of pipeline latency (commit happens at bit 7 of the
        -- previous byte), landing the collector on a byte boundary.
        for s in 0 to 7 loop
            wait until rising_edge(bclk);
        end loop;

        while bytecnt < CHANNELS_C*4 loop
            wait until rising_edge(bclk);
            curb := curb(6 downto 0) & a_tdm_tdm(0);  -- MSB-first shift in
            bitcnt := bitcnt + 1;
            if bitcnt = 8 then
                cap(bytecnt) := curb;
                bytecnt := bytecnt + 1;
                bitcnt := 0;
            end if;
        end loop;

        for ch in 0 to CHANNELS_C-1 loop
            for b in 0 to 3 loop
                if b < 3 then expv := EXP(ch)(b); else expv := x"00"; end if;
                if cap(ch*4 + b) /= expv then
                    ok := false;
                    report "A/TDM ch " & integer'image(ch) & " byte " & integer'image(b) &
                           " got 0x" & to_hstring(cap(ch*4 + b)) &
                           " expected 0x" & to_hstring(expv) severity error;
                end if;
            end loop;
        end loop;
        if ok then
            report "Test A: TDM path byte order OK (MSB, mid, LSB, pad)." severity note;
        end if;
        a_tdm_done <= true;
        wait;
    end process;

    -- bclk generator (free running until sim_done)
    bclk_gen : process
    begin
        wait until reset_n = '1';
        loop
            exit when sim_done;
            bclk <= '0'; wait for 2*CLK_PERIOD;
            bclk <= '1'; wait for 2*CLK_PERIOD;
        end loop;
        wait;
    end process;

    -- ============================================================
    -- Test B: drive a frame through the parser, read RAM back
    -- ============================================================
    test_b : process
        procedure cfg_write(addr : integer; val : std_logic_vector(7 downto 0)) is
        begin
            cfg_wr_clk <= '0';
            cfg_addr <= std_logic_vector(to_unsigned(addr, 8));
            cfg_data <= val; cfg_wr_en <= '1';
            wait for 2 ns; cfg_wr_clk <= '1';
            wait for 2 ns; cfg_wr_clk <= '0'; cfg_wr_en <= '0';
        end procedure;
        procedure set_eth(idx : integer; val : std_logic_vector(7 downto 0)) is
        begin
            eth_ram(idx) <= val; wait for 0 ns;
        end procedure;
        variable payload_idx : integer;
        variable frame_len   : integer;
        variable got         : std_logic_vector(7 downto 0);
        variable expv        : std_logic_vector(7 downto 0);
    begin
        wait until reset_n = '1';
        wait for 4*CLK_PERIOD;

        -- stream config: ip/port/channel-map/count/delay/samples
        cfg_write(0, STREAM_IP(31 downto 24)); cfg_write(1, STREAM_IP(23 downto 16));
        cfg_write(2, STREAM_IP(15 downto 8));  cfg_write(3, STREAM_IP(7 downto 0));
        cfg_write(4, STREAM_PORT(15 downto 8)); cfg_write(5, STREAM_PORT(7 downto 0));
        for ch in 0 to CHANNELS_C-1 loop
            cfg_write(6+ch, std_logic_vector(to_unsigned(ch, 8)));  -- identity map
        end loop;
        cfg_write(14, std_logic_vector(to_unsigned(CHANNELS_C, 8)));
        cfg_write(15, x"00");                                       -- delay 0
        cfg_write(16, std_logic_vector(to_unsigned(SAMPLES_PER_CH, 8)));

        -- synthetic frame payload: MSB,mid,LSB per channel (RFC 3190 order)
        payload_idx := OFF_PAYLOAD;
        for s in 0 to SAMPLES_PER_CH-1 loop
            for ch in 0 to CHANNELS_C-1 loop
                set_eth(payload_idx+0, EXP(ch)(0));
                set_eth(payload_idx+1, EXP(ch)(1));
                set_eth(payload_idx+2, EXP(ch)(2));
                payload_idx := payload_idx + BPS_C;
            end loop;
        end loop;
        frame_len := payload_idx - 14;          -- parser stops at len+14
        set_eth(OFF_LEN_HI, std_logic_vector(to_unsigned(frame_len/256, 8)));
        set_eth(OFF_LEN_LO, std_logic_vector(to_unsigned(frame_len mod 256, 8)));
        set_eth(OFF_DSTIP+0, STREAM_IP(31 downto 24)); set_eth(OFF_DSTIP+1, STREAM_IP(23 downto 16));
        set_eth(OFF_DSTIP+2, STREAM_IP(15 downto 8));  set_eth(OFF_DSTIP+3, STREAM_IP(7 downto 0));
        set_eth(OFF_DSTPORT+0, STREAM_PORT(15 downto 8)); set_eth(OFF_DSTPORT+1, STREAM_PORT(7 downto 0));
        set_eth(OFF_MCLOCK+0, x"00"); set_eth(OFF_MCLOCK+1, x"00");
        set_eth(OFF_MCLOCK+2, x"00"); set_eth(OFF_MCLOCK+3, x"00");
        media_clock <= (others => '0');

        wait for 4*CLK_PERIOD;
        packet_ready <= not packet_ready;       -- kick parser
        wait for 400*CLK_PERIOD;                 -- let it fill RAM

        -- read back RAM via backdoor (1-cycle latency: present addr, wait, sample)
        for ch in 0 to CHANNELS_C-1 loop
            for b in 0 to 2 loop
                bd_rd_addr <= to_unsigned(ch*CHANNEL_STRIDE_C + b, 14);
                wait until rising_edge(sys_clk);
                wait until rising_edge(sys_clk);  -- data valid now
                got  := bd_rd_data;
                expv := EXP(ch)(b);
                assert got = expv
                    report "B/PARSER ch " & integer'image(ch) &
                           " slot+" & integer'image(b) &
                           " got 0x" & to_hstring(got) &
                           " expected 0x" & to_hstring(expv) severity error;
            end loop;
        end loop;
        report "Test B: parser wrote linear byte order MSB@+0, mid@+1, LSB@+2." severity note;

        b_done <= true;
        wait;
    end process;

    -- ============================================================
    -- End-of-sim coordinator
    -- ============================================================
    finish : process
    begin
        wait until a_done and a_tdm_done and b_done;
        wait for 10*CLK_PERIOD;
        report "rx_ringbuffer_tb finished." severity note;
        sim_done <= true;
        wait;
    end process;

end architecture;
