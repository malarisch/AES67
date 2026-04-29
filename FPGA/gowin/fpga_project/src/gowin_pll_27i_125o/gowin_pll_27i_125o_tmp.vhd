--Copyright (C)2014-2025 Gowin Semiconductor Corporation.
--All rights reserved.
--File Title: Template file for instantiation
--Tool Version: V1.9.12
--Part Number: GW2A-LV55PG484C8/I7
--Device: GW2A-55
--Device Version: C
--Created Time: Wed Apr 29 19:03:48 2026

--Change the instance name and port connections to the signal names
----------Copy here to design--------

component Gowin_rPLL
    port (
        clkout: out std_logic;
        reset: in std_logic;
        clkin: in std_logic
    );
end component;

your_instance_name: Gowin_rPLL
    port map (
        clkout => clkout,
        reset => reset,
        clkin => clkin
    );

----------Copy end-------------------
