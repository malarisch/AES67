--Copyright (C)2014-2025 Gowin Semiconductor Corporation.
--All rights reserved.
--File Title: Template file for instantiation
--Tool Version: V1.9.12
--Part Number: GW5A-LV25MG121NC1/I0
--Device: GW5A-25
--Device Version: A
--Created Time: Sat May  2 15:16:03 2026

--Change the instance name and port connections to the signal names
----------Copy here to design--------

component gowin_pll_50i_MOD
    port (
        lock: out std_logic;
        clkout0: out std_logic;
        clkout1: out std_logic;
        mdrdo: out std_logic_vector(7 downto 0);
        clkin: in std_logic;
        reset: in std_logic;
        mdclk: in std_logic;
        mdopc: in std_logic_vector(1 downto 0);
        mdainc: in std_logic;
        mdwdi: in std_logic_vector(7 downto 0)
    );
end component;

your_instance_name: gowin_pll_50i_MOD
    port map (
        lock => lock,
        clkout0 => clkout0,
        clkout1 => clkout1,
        mdrdo => mdrdo,
        clkin => clkin,
        reset => reset,
        mdclk => mdclk,
        mdopc => mdopc,
        mdainc => mdainc,
        mdwdi => mdwdi
    );

----------Copy end-------------------
