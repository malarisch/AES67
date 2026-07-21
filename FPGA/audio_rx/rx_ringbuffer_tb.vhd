-- Testbench for rx_ringbuffer.vhd -- TDM serial playout path (PARALLEL_OUT=false)
--
-- Uses the SIM_SAMPLE_RAM_BACKDOOR to fill the first 16 sample slots with
-- known data (byte value == RAM address mod 256), then captures the serial
-- TDM frames on all TDM_OUTPUTS pins like a TDM receiver (data is launched
-- on rising bclk, so the receiver samples on falling bclk; frame aligned to
-- fsclk_i2s_tdm) and checks every byte of every channel slot on every pin:
--
--   pin i, channel c, byte k  ==  RAM[smp*64 + (i*TDM_CHANNELS + c)*4 + k]
--
-- where smp is the media clock value at the frame's fs edge. This checks:
--
--  1. Prefetch byte alignment across pins: the multi-pin fetch in
--     tfs_capture needs the same +1 as the pin-0 fetch in tfs_idle
--     (prefetch bookkeeping lags one byte behind the wire). Without it
--     every pin after the first carries all bytes one byte late.
--
--  2. Frame-consistent sample selection: media_clock_i steps once per
--     sample right AFTER the fs edge (worst case, matching PTP lock where
--     the NCO is pulled onto the wallclock-derived media clock). The read
--     pointer must be latched mid-frame (fsclk_i2s_50 falling edge) --
--     using media_clock_i combinationally tears channel 0 apart (MSB
--     primed from sample N, mid/LSB fetched from N+1).
--
-- bclk / fs clocks are generated the same way wallclock.vhd generates the
-- 256fs set: sys_clk-synchronous from an mclk counter (bclk = not mclk_cnt(0),
-- fsclk_i2s_tdm high for the last bclk period of the frame (mclk_cnt 510/511),
-- fsclk_i2s_50 high for the first half). media_clock increments at the
-- mclk_cnt 511->0 wrap, two mclk ticks after the fs edge.
--
-- Run:
--   ghdl -a --std=08 ../packages/audioclks_pkg.vhd rx_ringbuffer.vhd rx_ringbuffer_tb.vhd
--   ghdl -e --std=08 rx_ringbuffer_tb
--   ghdl -r --std=08 rx_ringbuffer_tb --ieee-asserts=disable

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.audioclks_pkg.all;

entity rx_ringbuffer_tb is
end entity;

architecture sim of rx_ringbuffer_tb is

    constant TDM_OUTPUTS_C  : integer := 2;
    constant TDM_CHANNELS_C : integer := 8;
    constant GLOBAL_CH_C    : integer := TDM_OUTPUTS_C * TDM_CHANNELS_C;
    constant SLOT_BYTES_C   : integer := 4;
    constant FRAME_BITS_C   : integer := TDM_CHANNELS_C * 32; -- 256

    constant CLK_PERIOD : time    := 8 ns; -- 125 MHz sys_clk
    constant MCLK_DIV   : integer := 5;    -- sys_clk ticks per mclk tick (~24.6 MHz)

    -- Frames 0..1 settle the pipeline; check these full frames afterwards.
    constant FIRST_CHECKED_FRAME : integer := 2;
    constant LAST_CHECKED_FRAME  : integer := 6;

    signal sys_clk : std_logic := '0';
    signal reset_n : std_logic := '0';

    signal audioclks   : t_audio_clocks_selected := AUDIO_CLOCKS_RESET_SELECTED;
    signal mclk_cnt    : unsigned(8 downto 0) := (others => '0');
    signal mclk_divcnt    : integer range 0 to MCLK_DIV - 1 := 0;

    signal media_clock : std_logic_vector(31 downto 0) := (others => '0');
    signal tdm_out     : std_logic_vector(TDM_OUTPUTS_C - 1 downto 0);

    signal dbg_wr_en   : std_logic := '0';
    signal dbg_wr_addr : unsigned(13 downto 0) := (others => '0');
    signal dbg_wr_data : std_logic_vector(7 downto 0) := (others => '0');

begin

    sys_clk <= not sys_clk after CLK_PERIOD / 2;
    reset_n <= '1' after 10 * CLK_PERIOD;

    dut : entity work.rx_ringbuffer
        generic map (
            audio_buffer_sample_depth => 256,
            global_channel_count      => GLOBAL_CH_C,
            bytes_per_sample          => 3,
            max_streams               => 8,
            ENABLE_METERING           => false,
            PARALLEL_OUT              => false,
            TDM_OUTPUTS               => TDM_OUTPUTS_C,
            TDM_CHANNELS              => TDM_CHANNELS_C,
            SIM_SAMPLE_RAM_BACKDOOR   => true
        )
        port map (
            sys_clk                => sys_clk,
            reset_n                => reset_n,
            audio_out              => open,
            tdm_out                => tdm_out,
            audioclocks_i          => audioclks,
            media_clock_i          => media_clock,
            eth_read_addr_o        => open,
            eth_read_data_i        => (others => '0'),
            packet_ready_i         => '0',
            stream_config_wr_clk_i => '0',
            stream_config_wr_en_i  => '0',
            stream_config_addr_i   => (others => '0'),
            stream_config_data_i   => (others => '0'),
            metering_signal_o      => open,
            metering_clip_o        => open,
            metering_clear_i       => '0',
            dbg_wr_en_i            => dbg_wr_en,
            dbg_wr_addr_i          => dbg_wr_addr,
            dbg_wr_data_i          => dbg_wr_data,
            dbg_rd_addr_i          => (others => '0'),
            dbg_rd_data_o          => open
        );

    -- Audio clock generation, mirroring wallclock.vhd's 256fs set:
    -- mclk_cnt advances every MCLK_DIV sys_clks; bclk = not mclk_cnt(0);
    -- fsclk_i2s_tdm high while (mclk_cnt + 2) < 2, i.e. cnt 510 and 511.
    clkgen_proc : process (sys_clk)
        variable cntsub : unsigned(8 downto 0);
    begin
        if rising_edge(sys_clk) then
            if mclk_divcnt = MCLK_DIV - 1 then
                mclk_divcnt <= 0;
                mclk_cnt <= mclk_cnt + 1;
                -- Sample boundary: media clock steps two mclk ticks after
                -- the fs edge, i.e. right between the DUT's channel-0 MSB
                -- prime and its mid/LSB fetches (worst case for tearing).
                if mclk_cnt = 511 then
                    media_clock <= std_logic_vector(unsigned(media_clock) + 1);
                end if;
            else
                mclk_divcnt <= mclk_divcnt + 1;
            end if;

            audioclks.bclk <= not mclk_cnt(0);

            cntsub := mclk_cnt + 2;
            if cntsub = 0 then
                audioclks.fsclk_i2s_tdm <= '1';
            elsif cntsub = 2 then
                audioclks.fsclk_i2s_tdm <= '0';
            end if;
            audioclks.fsclk_i2s_50 <= not cntsub(8);
        end if;
    end process;

    -- Backdoor fill: sample slots 0..15 with byte value == RAM address mod
    -- 256, so every (channel, byte-in-slot) position is unique on the wire
    -- and adjacent sample slots hold different data (tear detection).
    fill_proc : process
    begin
        wait until reset_n = '1';
        wait until rising_edge(sys_clk);
        for a in 0 to 16 * GLOBAL_CH_C * SLOT_BYTES_C - 1 loop
            dbg_wr_en   <= '1';
            dbg_wr_addr <= to_unsigned(a, dbg_wr_addr'length);
            dbg_wr_data <= std_logic_vector(to_unsigned(a mod 256, 8));
            wait until rising_edge(sys_clk);
        end loop;
        dbg_wr_en <= '0';
        wait;
    end process;

    -- TDM receiver + checker. Data is launched on rising bclk, so the
    -- receiver samples on falling bclk. The fs rising edge (seen on a
    -- falling bclk edge) marks the frame boundary: the *next* falling edge
    -- samples bit 0, and the fs edge itself samples bit 255 of the frame
    -- just finished. The expected sample index is the media clock value at
    -- the frame's fs edge (the DUT latched it mid-way through the previous
    -- frame; media_clock steps again only after this edge).
    check_proc : process (audioclks.bclk)
        type t_frame_bits is array (0 to TDM_OUTPUTS_C - 1)
            of std_logic_vector(0 to FRAME_BITS_C - 1);
        variable v_bits    : t_frame_bits;
        variable v_fs_d    : std_logic := '0';
        variable v_bitpos  : integer := 0;
        variable v_started : boolean := false;
        variable v_frame   : integer := 0;
        variable v_smp     : integer := 0;
        variable v_byte    : std_logic_vector(7 downto 0);
        variable v_exp     : integer;
        variable v_errors  : integer := 0;
    begin
        if falling_edge(audioclks.bclk) then
            if audioclks.fsclk_i2s_tdm = '1' and v_fs_d = '0' then
                if v_started then
                    for i in 0 to TDM_OUTPUTS_C - 1 loop
                        v_bits(i)(FRAME_BITS_C - 1) := tdm_out(i);
                    end loop;

                    if v_frame >= FIRST_CHECKED_FRAME then
                        for i in 0 to TDM_OUTPUTS_C - 1 loop
                            for b in 0 to TDM_CHANNELS_C * SLOT_BYTES_C - 1 loop
                                for j in 0 to 7 loop
                                    v_byte(7 - j) := v_bits(i)(b * 8 + j);
                                end loop;
                                v_exp := (v_smp * GLOBAL_CH_C * SLOT_BYTES_C
                                          + i * TDM_CHANNELS_C * SLOT_BYTES_C + b) mod 256;
                                if v_byte /= std_logic_vector(to_unsigned(v_exp, 8)) then
                                    v_errors := v_errors + 1;
                                    report "frame " & integer'image(v_frame)
                                        & " (sample " & integer'image(v_smp) & ")"
                                        & " pin " & integer'image(i)
                                        & " ch " & integer'image(b / SLOT_BYTES_C)
                                        & " byte " & integer'image(b mod SLOT_BYTES_C)
                                        & ": expected " & integer'image(v_exp)
                                        & " got " & integer'image(
                                            to_integer(unsigned(v_byte)))
                                        severity error;
                                end if;
                            end loop;
                        end loop;
                    end if;

                    if v_frame = LAST_CHECKED_FRAME then
                        if v_errors = 0 then
                            report "rx_ringbuffer_tb PASS: TDM playout, "
                                & integer'image(TDM_OUTPUTS_C) & " pins, frames "
                                & integer'image(FIRST_CHECKED_FRAME) & ".."
                                & integer'image(LAST_CHECKED_FRAME) & " clean"
                                severity note;
                            finish;
                        else
                            report "rx_ringbuffer_tb FAIL: "
                                & integer'image(v_errors) & " byte errors"
                                severity failure;
                        end if;
                    end if;
                    v_frame := v_frame + 1;
                end if;
                v_started := true;
                v_bitpos  := 0;
                -- Sample index this new frame must play: media clock at the
                -- fs edge (it steps two mclk ticks later).
                v_smp := to_integer(unsigned(media_clock(7 downto 0)));
            elsif v_started and v_bitpos < FRAME_BITS_C - 1 then
                for i in 0 to TDM_OUTPUTS_C - 1 loop
                    v_bits(i)(v_bitpos) := tdm_out(i);
                end loop;
                v_bitpos := v_bitpos + 1;
            end if;
            v_fs_d := audioclks.fsclk_i2s_tdm;
        end if;
    end process;

end architecture;
