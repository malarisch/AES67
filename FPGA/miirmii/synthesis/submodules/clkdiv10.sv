// (C) 2001-2025 Altera Corporation. All rights reserved.
// Your use of Altera Corporation's design tools, logic functions and other 
// software and tools, and its AMPP partner logic functions, and any output 
// files from any of the foregoing (including device programming or simulation 
// files), and any associated documentation or information are expressly subject 
// to the terms and conditions of the Altera Program License Subscription 
// Agreement, Altera IP License Agreement, or other applicable 
// license agreement, including, without limitation, that your use is for the 
// sole purpose of programming logic devices manufactured by Altera and sold by 
// Altera or its authorized distributors.  Please refer to the applicable 
// agreement for further details.


`timescale 1 ns / 100 ps
module clkdiv10 (
 input  clock_i,
 input  Rstn_i,
 input  start_tick_i,
 output clock_o,
 output tck_o
);

logic [9:0] Shift10;
logic [4:0] Shift5;
logic temp_clk;

 always_ff @(posedge clock_i)
   if (~Rstn_i)
      Shift5 <= 10'b1;
   else
      Shift5 <= {Shift5[3:0],Shift5[4]};

 always_ff @(posedge clock_i)
   if (~Rstn_i)
      Shift10 <= 10'b1;
   else if (start_tick_i)
      Shift10 <= {Shift10[8:0],Shift10[9]};
/*
assign clock_o =  Shift10[0] | 
                  Shift10[1] | 
                  Shift10[2] | 
                  Shift10[3] | 
                  Shift10[4] ; 
*/

assign tck_o  =   Shift10[0] ;

 always_ff @(posedge clock_i)
   if (~Rstn_i)
      temp_clk <= 1'b0;
   else 
      temp_clk <=  Shift5[0] ^ temp_clk;


assign clock_o = temp_clk;

endmodule 
