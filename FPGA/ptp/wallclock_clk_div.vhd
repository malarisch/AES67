-- ============================================================
-- wallclock_clk_div: audio clock tree from the MCLK sample counter
--
-- mclk_cnt_i counts MCLK cycles (0..511) within one sample. Derived:
--   clk_256fs/128fs/64fs.bclk = inverted cnt bit 0/1/2
--   fsclk_50                  = inverted cnt bit 8 (LRCK, 50 % duty)
--   *.fsclk_tdm               = TDM frame sync, one bit clock wide
--   *.fsclk_i2s_50/_tdm       = same, advanced by one bit clock (I2S)
--   *.fsclk_i2s_50_dac        = I2S LRCK, advanced by 1.5 bit clocks
-- All outputs are registered on clk; mclk is passed through.
--
-- Used twice: in the sys_clk domain (clocks_o) and, with
-- VCXO_DOMAIN_CLOCKS, clocked by the VCXO itself for the pins.
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.audioclks_pkg.all;

entity wallclock_clk_div is
    port(
        clk        : in  std_logic;
        mclk_i     : in  std_logic;
        mclk_cnt_i : in  unsigned(8 downto 0);
        clocks_o   : out t_audio_clocks := AUDIO_CLOCKS_RESET
    );
end entity wallclock_clk_div;

architecture rtl of wallclock_clk_div is
begin

    clocks_o.mclk <= mclk_i;

    bclk_proc: process (clk)
    begin
        if rising_edge(clk) then
            clocks_o.clk_256fs.bclk <= not mclk_cnt_i(0);
            clocks_o.clk_128fs.bclk <= not mclk_cnt_i(1);
            clocks_o.clk_64fs.bclk  <= not mclk_cnt_i(2);
            clocks_o.fsclk_50       <= not mclk_cnt_i(8);
        end if;
    end process;

    tdm_proc: process (clk)
    begin
        if rising_edge(clk) then
            if (mclk_cnt_i = 0) then
                clocks_o.clk_256fs.fsclk_tdm <= '1';
                clocks_o.clk_128fs.fsclk_tdm <= '1';
                clocks_o.clk_64fs.fsclk_tdm  <= '1';
            elsif (mclk_cnt_i = 2) then
                clocks_o.clk_256fs.fsclk_tdm <= '0';
            elsif (mclk_cnt_i = 4) then
                clocks_o.clk_128fs.fsclk_tdm <= '0';
            elsif (mclk_cnt_i = 8) then
                clocks_o.clk_64fs.fsclk_tdm  <= '0';
            end if;
        end if;
    end process;

    i2s_256fs_proc: process (clk)
        variable cntsub : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            cntsub := mclk_cnt_i + 2;
            clocks_o.clk_256fs.fsclk_i2s_50 <= not cntsub(8);
            if (cntsub = 0) then
                clocks_o.clk_256fs.fsclk_i2s_tdm <= '1';
            elsif (cntsub = 2) then
                clocks_o.clk_256fs.fsclk_i2s_tdm <= '0';
            end if;
            cntsub := mclk_cnt_i + 3;
            clocks_o.clk_256fs.fsclk_i2s_50_dac <= not cntsub(8);
        end if;
    end process;

    i2s_128fs_proc: process (clk)
        variable cntsub : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            cntsub := mclk_cnt_i + 4;
            clocks_o.clk_128fs.fsclk_i2s_50 <= not cntsub(8);
            if (cntsub = 0) then
                clocks_o.clk_128fs.fsclk_i2s_tdm <= '1';
            elsif (cntsub = 4) then
                clocks_o.clk_128fs.fsclk_i2s_tdm <= '0';
            end if;
            cntsub := mclk_cnt_i + 6;
            clocks_o.clk_128fs.fsclk_i2s_50_dac <= not cntsub(8);
        end if;
    end process;

    i2s_64fs_proc: process (clk)
        variable cntsub : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            cntsub := mclk_cnt_i + 8;
            clocks_o.clk_64fs.fsclk_i2s_50 <= not cntsub(8);
            if (cntsub = 0) then
                clocks_o.clk_64fs.fsclk_i2s_tdm <= '1';
            elsif (cntsub = 8) then
                clocks_o.clk_64fs.fsclk_i2s_tdm <= '0';
            end if;
            cntsub := mclk_cnt_i + 12;
            clocks_o.clk_64fs.fsclk_i2s_50_dac <= not cntsub(8);
        end if;
    end process;

end architecture rtl;
