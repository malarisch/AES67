--Copyright (C)2014-2025 Gowin Semiconductor Corporation.
--All rights reserved.
--File Title: Template file for instantiation
--Part Number: GW5A-LV25MG121NC1/I0
--Device: GW5A-25
--Device Version: A


--Change the instance name and port connections to the signal names
----------Copy here to design--------
component gowin_pll_50i
    port (
        clkin: in std_logic;
        clkout0: out std_logic;
        clkout1: out std_logic;
        lock: out std_logic;
        mdclk: in std_logic
    );
end component;


your_instance_name: gowin_pll_50i
    port map (
        clkin => clkin,
        clkout0 => clkout0,
        clkout1 => clkout1,
        lock => lock,
        mdclk => mdclk
    );


----------Copy end-------------------
