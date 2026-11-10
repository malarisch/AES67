

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.numeric_std.all;
USE work.system_cfg_pkg.all;
ENTITY sysclk_pll_gen IS
	generic (
		platform : t_platforms := ALTERA; -- "ALTERA" or "GOWIN"
		clk_in_speed : natural := 50; -- input clock speed in mhz (for now only 12, 27, 50)
		-- sys_clk period [ns]. The PLLs below all make 125 MHz (8 ns); if
		-- the input clock already has the sys_clk frequency it is used
		-- directly (no PLL), also as the SoC clock.
		sys_clk_ns_per_tick : natural := 8
	);
	PORT
	(
        clock_i : IN STD_LOGIC;
        ext_mcu_clk_i : IN STD_LOGIC;
        ext_locked_i : IN STD_LOGIC;
        rst_n_i : IN STD_LOGIC;
        sys_clk_125MHz_o : OUT STD_LOGIC;
        mcu_clk_o : OUT STD_LOGIC;
        mcu_clk2_o : OUT STD_LOGIC;
        locked_o : OUT STD_LOGIC

    );
    end sysclk_pll_gen;

architecture rtl of sysclk_pll_gen is
    component gowin_pll_50i is
    port (
        clkin: in std_logic;
        clkout0: out std_logic;
        clkout1: out std_logic;
        lock: out std_logic;
        mdclk: in std_logic
    );
end component;
component lattice_pll_16m
  port (
    CLKI : in std_logic;
    CLKOP : out std_logic;
    CLKOS : out std_logic;
    LOCK : out std_logic
  );
end component;
component sysclks_altpll_50m_in
  port (
    areset : in STD_LOGIC;
    inclk0 : in STD_LOGIC;
    c0 : out STD_LOGIC;
    c1 : out STD_LOGIC;
    c2 : out STD_LOGIC;
    locked : out STD_LOGIC
  );
end component;
component sysclks_altpll_12m_in
  port (
    areset : in STD_LOGIC;
    inclk0 : in STD_LOGIC;
    c0 : out STD_LOGIC;
    c1 : out STD_LOGIC;
    c2 : out STD_LOGIC;
    locked : out STD_LOGIC
  );
end component;
component gowin_pll_27i_125o
  port (
    clkout : out std_logic;
    reset : in std_logic;
    clkin : in std_logic;
    lock_o : out std_logic
  );
  
end component;
component altpll_25m
  port (
    inclk0 : in STD_LOGIC;
    c0 : out STD_LOGIC;
    c1 : out STD_LOGIC;
    locked : out STD_LOGIC
  );
end component;
signal sys_clk_locked : std_logic;
constant CLK_IN_IS_SYS_CLK : boolean := clk_in_speed * sys_clk_ns_per_tick = 1000;
begin
assert CLK_IN_IS_SYS_CLK or sys_clk_ns_per_tick = 8
	report "sysclk_pll_gen: the PLLs only make 125 MHz; feed the sys_clk directly for other rates"
	severity failure;

-- input clock is the sys_clk: no PLL. The SoC runs on it too.
sysclk_direct: if CLK_IN_IS_SYS_CLK generate
	sys_clk_125MHz_o <= clock_i;
	mcu_clk_o        <= ext_mcu_clk_i;
	mcu_clk2_o       <= ext_mcu_clk_i;   -- no phase-shifted copy (SDRAM SoC needs a PLL)
	sys_clk_locked   <= ext_locked_i;
end generate;

-- system clocks
sysclkgen50: if (platform = ALTERA and clk_in_speed = 50 and not CLK_IN_IS_SYS_CLK) generate
sysclks_altpll_50m_in_inst : sysclks_altpll_50m_in PORT MAP (
		areset	 => not rst_n_i,
		inclk0	 => clock_i,
		c0	 => sys_clk_125MHz_o,
		c1 	 => mcu_clk_o,
		c2 	 => mcu_clk2_o,
		locked	 => sys_clk_locked
	);

end generate;
sysclkgen25: if (platform = ALTERA and clk_in_speed = 25 and not CLK_IN_IS_SYS_CLK) generate
sysclks_altpll_25m_in_inst : altpll_25m
  port map (
    inclk0 => clock_i,
    c0 => sys_clk_125MHz_o,
    c1 => mcu_clk_o,
    locked => sys_clk_locked
  );


end generate;
sysclkgen12: if (platform = ALTERA and clk_in_speed = 12 and not CLK_IN_IS_SYS_CLK) generate
sysclks_altpll_12m_in_inst : sysclks_altpll_12m_in PORT MAP (
		areset	 => not rst_n_i,
		inclk0	 => clock_i,
		c0	 => sys_clk_125MHz_o,
		c1 	 => mcu_clk_o,
		c2 	 => mcu_clk2_o,
		locked	 => sys_clk_locked
	);

end generate;
sysclkgen27: if (platform = GOWIN and clk_in_speed = 27 and not CLK_IN_IS_SYS_CLK) generate
gowin_pll_27i_125o_inst: gowin_pll_27i_125o
 port map(
	clkout => sys_clk_125MHz_o,
	reset => not rst_n_i,
	clkin => clock_i,
	lock_o => sys_clk_locked
);
end generate;


sysclkgen50_gw: if (platform = GOWIN and clk_in_speed = 50 and not CLK_IN_IS_SYS_CLK) generate
	gowin_pll_50i_inst: gowin_pll_50i
	 port map(
		clkin => clock_i,
		clkout0 => sys_clk_125MHz_o,
		lock => sys_clk_locked,
		mdclk => clock_i
	);
end generate;
sysclkgen16_lat : if (platform = LATTICE and clk_in_speed = 16 and not CLK_IN_IS_SYS_CLK) generate
	lattice_pll_16m_inst: lattice_pll_16m
	 port map(
		CLKI => clock_i,
		CLKOP => sys_clk_125MHz_o,
		CLKOS => mcu_clk_o,
		LOCK => sys_clk_locked
	);

end generate;
    
locked_o <= sys_clk_locked;
end architecture;
