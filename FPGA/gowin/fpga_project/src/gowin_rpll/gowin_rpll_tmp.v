//Copyright (C)2014-2025 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//Tool Version: V1.9.12
//Part Number: GW2A-LV18PG256C8/I7
//Device: GW2A-18
//Created Time: Sat Apr  4 19:06:55 2026

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

    Gowin_rPLL your_instance_name(
        .clkout(clkout), //output clkout
        .lock(lock), //output lock
        .clkoutd(clkoutd), //output clkoutd
        .reset(reset), //input reset
        .clkin(clkin) //input clkin
    );

//--------Copy end-------------------
