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
module intel_fpga_mii2rmii  
#(
  parameter RX_FIXED_THROUGHPUT=1'b0,
  parameter MBPS=1'b1,
  parameter MAC_SPEED=1'b1)
(
input  logic       RefClk,
input  logic       Rstn,
//mii(Mac) <-> rmii Tx
output logic       tx_clk,
input logic        m_tx_en,
input logic [3:0]  m_tx_d,
input logic        m_tx_err,
//rmii <-> Mac Rx
output             rx_clk,
output logic       m_rx_en,
output logic [3:0] m_rx_d,
output logic       m_rx_err,
output logic       m_rx_crs,
output logic       m_rx_col,
//rmii <-> PHY
input  logic       rmii_crs_dv, //Carrier Sense/Receive Data Valid
input  logic [1:0] rmii_rx_d,   //Receive Data
input  logic       rmii_rx_err,


output logic       rmii_tx_en,  // 
output logic [1:0] rmii_tx_d,
//
input logic       ena_10
//
// MDIO interface
/*               
input logic        MiiClkI_i,
input logic        MiiClkO_i,
input logic        MiiClkT_i,

input logic        MiiDataI_i,
input logic        MiiDataO_i,
input logic        MiiDataT_i,
//
output logic       PhyMDC_o,
output logic       PhyMDIO_o
*/

);
//Signal Declarations
logic       PhyClkBy10Tck;
logic       PhyClkBy2Tck;
logic [3:0][1:0] SfdAccum100Mbps; 
logic       RxIn100MbpsMode1 ;
logic       RxIn10MbpsMode1  ;
logic       RxSpeedDetected1 ;
logic       RxSpeedDetectedFixed;
logic       Shift100Mbps1;
logic       Shift10Mbps1 ;
logic       SamplingTick;
//
logic       Phy2RmiiCrsDv0   ;
logic [1:0] Phy2RmiiRxd0     ;
logic       Phy2RmiiRxErr0   ;
//
logic       Phy2RmiiCrsDv1   ;
logic [1:0] Phy2RmiiRxd1     ;
logic       Phy2RmiiRxErr1   ;
//
logic       Phy2RmiiCrsDv2   ;
logic [1:0] Phy2RmiiRxd2     ;
logic       Phy2RmiiRxErr2   ;
//
logic       Phy2RmiiCrsDv3   ;
logic [1:0] Phy2RmiiRxd3     ;
logic       Phy2RmiiRxErr3   ;
//
logic       Phy2RmiiCrsDv4   ;
logic [1:0] Phy2RmiiRxd4     ;
logic       Phy2RmiiRxErr4   ;
//
logic [3:0][1:0] SfdAccum10Mbps;
//
logic       EndOfData2;
logic       EndOfData3;
logic       EndOfData4;
logic       EndOfData5; 
logic       EndOfData6;
logic       RxEnd0;
logic       RxEnd1;
logic       RxEnd2;
//
logic       RxDataVal;
logic [3:0] RxNibble;
logic [3:0] RxNibbleInt4;
logic [1:0] LowDibit;
//logic [1:0] HighDibit;
logic [1:0] LowDibit4;
logic [1:0] HighDibit4;
logic       NibbleVal;
logic       HighDibitVal;
logic [3:0] ShiftReg100;
logic [3:0] ShiftReg10;

logic       SampleNibble;
logic       Clk25Mhz;
logic       Clk2p5MHz;
logic       Clk5MHz;
logic       RxClk;
logic       EndOfCrs;
logic       StartOfRx;
logic       Rmii2MiiCrs;
logic [1:0] RxdSfd;
logic [3:0] ShiftSfd;
logic       TransmitSfd;
logic [1:0] Phy2RmiiRxdInt4;

//Start of code
 //Clock generation 
 generate 
  if (RX_FIXED_THROUGHPUT == 0) begin
   clkdiv10 u0_clkddiv10 ( .clock_i  (RefClk) 
                           ,.Rstn_i  (Rstn) 
                           ,.clock_o (Clk5MHz) 
                           ,.start_tick_i(NibbleVal || Phy2RmiiCrsDv0)
                           ,.tck_o   (PhyClkBy10Tck) 
                         ) ;
  end 
  else begin   
   clkdiv10 u0_clkddiv10 ( .clock_i  (RefClk) 
                           ,.Rstn_i  (Rstn) 
                           ,.clock_o (Clk5MHz) 
                           ,.tck_o   (PhyClkBy10Tck) 
                           ,.start_tick_i(1'b1)
                         ) ;
 end 
 endgenerate 
 
  
   clkdiv2 u0_clkdiv     (.clk       (RefClk)
                          ,.rstn     (Rstn)
                          ,.clkby2   (Clk25Mhz)
                         );

   clkdiv2 u1_clkdiv     (.clk       (Clk5MHz)
                          ,.rstn     (Rstn)
                          ,.clkby2   (Clk2p5MHz)
                        );
  //RX speed selection

  
 

  // Input pipe stage
  always_ff @(posedge RefClk) begin
    Phy2RmiiCrsDv0 <= rmii_crs_dv;
    Phy2RmiiRxd0   <= rmii_rx_d;
    Phy2RmiiRxErr0 <= rmii_rx_err; 
  end
  //
  //SFD(0xAB) detection in 100Mbps mode and 10 Mbps mode
  //Shift register to shift in RX dibit
  assign RxSpeedDetected1 = (RX_FIXED_THROUGHPUT==0) ?  (RxIn100MbpsMode1 | RxIn10MbpsMode1) : RxSpeedDetectedFixed;
  assign Shift100Mbps1    = Phy2RmiiCrsDv0 & ~RxSpeedDetected1 & (Phy2RmiiRxd0[1:0] != 2'b00);
  //assign Shift10Mbps1     = Phy2RmiiCrsDv0 & PhyClkBy10Tck & ~RxIn10MbpsMode1 & (Phy2RmiiRxd0[1:0] != 2'b00);
  assign Shift10Mbps1     = Phy2RmiiCrsDv0 & PhyClkBy10Tck & ~RxSpeedDetected1 & (Phy2RmiiRxd0[1:0] != 2'b00);
 
  always_ff @(posedge RefClk) 
    if (!Rstn)
       ShiftReg100 <= 4'b0001;
    else if (RxEnd2 && SamplingTick)
       ShiftReg100 <= 4'b0001;
    else if (Shift100Mbps1)
       ShiftReg100 <= {ShiftReg100[2:0],ShiftReg100[3]};
 
  always_ff @(posedge RefClk) 
    if (!Rstn)
       ShiftReg10 <= 4'b0001;
    else if (RxEnd2 && SamplingTick)
       ShiftReg10 <= 4'b0001;
    else if (Shift10Mbps1)
       ShiftReg10 <= {ShiftReg10[2:0],ShiftReg10[3]};

  always_ff @(posedge RefClk)
    if (!Rstn)
       RxSpeedDetectedFixed <= 1'b0;
    else if (EndOfData4 & RxSpeedDetectedFixed & SamplingTick)
       RxSpeedDetectedFixed <= 1'b0;
    else if (Phy2RmiiCrsDv3 & |Phy2RmiiRxd3 & ~RxSpeedDetectedFixed & SamplingTick)
       RxSpeedDetectedFixed <= 1'b1;

  genvar i;
  generate 
  begin
  	for (i=0; i < 4 ; i = i+1 ) begin : SFD_SHIFTER
            if (i==0) 
            begin
               always_ff @(posedge RefClk)
                 if (~Rstn)
                    SfdAccum100Mbps[i] <= 2'b0;
                 else if (RxEnd2 && SamplingTick)
                    SfdAccum100Mbps[0] <= 2'b0;
                 else if (Shift100Mbps1)
                    SfdAccum100Mbps[0] <= Phy2RmiiRxd0[1:0];

               always_ff @(posedge RefClk)
                 if (~Rstn)
                    SfdAccum10Mbps[i] <= 2'b0;
                 else if (RxEnd2 && SamplingTick)
                    SfdAccum10Mbps[0] <= 2'b0;
                 else if (Shift10Mbps1)
                    SfdAccum10Mbps[0] <= Phy2RmiiRxd0[1:0];
            end
            else 
            begin
               always_ff @(posedge RefClk)
                 if (RxEnd2 && SamplingTick)
                    SfdAccum100Mbps[i] <= 2'b0;
                 else if (Shift100Mbps1)
                    SfdAccum100Mbps[i] <= SfdAccum100Mbps[i-1];

               always_ff @(posedge RefClk)
                 if (RxEnd2 && SamplingTick)
                    SfdAccum10Mbps[i] <= 2'b0;
                 else if (Shift10Mbps1)
                    SfdAccum10Mbps[i] <= SfdAccum10Mbps[i-1];
            end
        end
  end
  endgenerate

  assign RxIn100MbpsMode1 =(MAC_SPEED == 1 ) ? ~ena_10 :  (RX_FIXED_THROUGHPUT == 1) ? (MBPS == 1) ? 1'b1 : 1'b0 :
                            ({SfdAccum100Mbps[0],SfdAccum100Mbps[1],SfdAccum100Mbps[2],SfdAccum100Mbps[3]} == 8'hD5) && ~RxIn10MbpsMode1 && ShiftReg100[0];
  assign RxIn10MbpsMode1  =(MAC_SPEED == 1 ) ? ena_10 : (RX_FIXED_THROUGHPUT == 1) ? (MBPS == 0) ? 1'b1 : 1'b0 :
                            ({SfdAccum10Mbps[0],SfdAccum10Mbps[1],SfdAccum10Mbps[2],SfdAccum10Mbps[3]}     == 8'hD5)  && ShiftReg10[0];
      
 //--------------------- 
 // RX_DV generation  
 //--------------------- 
 //assign SamplingTick =     RxIn10MbpsMode1 ? PhyClkBy10Tck : 1'b1;
 assign SamplingTick =  RxIn100MbpsMode1 ? 1'b1 :   RxIn10MbpsMode1 ? PhyClkBy10Tck : 1'b0;
 
 always_ff @(posedge RefClk) begin
   if (~Rstn) begin
      Phy2RmiiCrsDv1 <='b0; 
      Phy2RmiiCrsDv2 <='b0;
      Phy2RmiiCrsDv3 <='b0;
      Phy2RmiiCrsDv4 <='b0;
      //
      Phy2RmiiRxErr1 <= 1'b0;
      Phy2RmiiRxErr2 <= 1'b0;
      Phy2RmiiRxErr3 <= 1'b0;
      Phy2RmiiRxErr4 <= 1'b0;

   end
   else if (SamplingTick) begin
      Phy2RmiiCrsDv1 <= Phy2RmiiCrsDv0;
      Phy2RmiiCrsDv2 <= Phy2RmiiCrsDv1;
      Phy2RmiiCrsDv3 <= Phy2RmiiCrsDv2;
      Phy2RmiiCrsDv4 <= Phy2RmiiCrsDv3;
      //
      Phy2RmiiRxErr1 <= Phy2RmiiRxErr0; 
      Phy2RmiiRxErr2 <= Phy2RmiiRxErr1;
      Phy2RmiiRxErr3 <= Phy2RmiiRxErr2;
      Phy2RmiiRxErr4 <= Phy2RmiiRxErr3;
   end
 end
 always_ff @(posedge RefClk) begin
   if (SamplingTick) begin
     
      Phy2RmiiRxd1 <= Phy2RmiiRxd0;
      Phy2RmiiRxd2 <= Phy2RmiiRxd1;
      Phy2RmiiRxd3 <= Phy2RmiiRxd2;
      Phy2RmiiRxd4 <= Phy2RmiiRxd3;

    
      EndOfData3     <= EndOfData2;
      EndOfData4     <= EndOfData3;
      EndOfData5     <= EndOfData4;
      EndOfData6     <= EndOfData5;
   end
 end
 
 assign EndOfData2 =  Phy2RmiiCrsDv2 &&  ~Phy2RmiiCrsDv1 && ~Phy2RmiiCrsDv0;
 
 assign TransmitSfd = RxSpeedDetected1 && ~Phy2RmiiCrsDv4 && ~EndOfCrs && ~RxEnd0 && ~RxEnd1 && ~RxEnd2;

 always_ff @(posedge RefClk)
   if (~Rstn)
     ShiftSfd <= 4'b0001;
   else if (RxEnd2)
     ShiftSfd <= 4'b0001;
   else if (RxSpeedDetected1 & ~ShiftSfd[3] & SamplingTick)
     ShiftSfd <=  {ShiftSfd[2:0],ShiftSfd[3]};
    
 always_ff @(posedge RefClk) 
   if (~Rstn)
      RxDataVal <= 1'b0;
   //else if (EndOfData4 && RxDataVal)
   else if (EndOfData4 && RxDataVal && SamplingTick)
      RxDataVal <= ~RxDataVal;
   else if (RxSpeedDetected1 && ~RxDataVal && SamplingTick && ~RxEnd0 && ~RxEnd1)
      RxDataVal <=RxSpeedDetected1; 

 always_ff @(posedge RefClk)
    if (RxSpeedDetected1 && SamplingTick) begin
	LowDibit[1:0] <=  LowDibit4[1:0]; 
     end

 always_comb
   case (1'b1)
     ShiftSfd[0]:RxdSfd = 2'b01; 
     ShiftSfd[1]:RxdSfd = 2'b01;
     ShiftSfd[2]:RxdSfd = 2'b01;
     ShiftSfd[3]:RxdSfd = 2'b11;
     default:RxdSfd = 2'b00; 
   endcase
 
 assign Phy2RmiiRxdInt4 = TransmitSfd ? RxdSfd :Phy2RmiiRxd4 ;
 assign	LowDibit4[1:0]  = ~HighDibitVal ? Phy2RmiiRxdInt4 : LowDibit[1:0]; 
 assign HighDibit4[1:0] = HighDibitVal ? Phy2RmiiRxdInt4 : RxNibble[3:2]; 

 always_ff @(posedge RefClk)
    if (~Rstn)
       HighDibitVal <= 1'b0;
    else if (EndOfData4  && SamplingTick)
       HighDibitVal <= 1'b0;
    else if (RxSpeedDetected1 && SamplingTick && !RxEnd0)  
       HighDibitVal = ~HighDibitVal; 
 
 assign RxNibbleInt4    = {HighDibit4, LowDibit4};
// assign RxClk           = RxIn10MbpsMode1 ?  Clk2p5MHz : Clk25Mhz; 
 assign #0 SampleNibble = RxDataVal & HighDibitVal;
 assign #0 ClearRxEnd   = RxEnd0 && SamplingTick;
 always_ff @(posedge RefClk)
    if (~Rstn)
       RxEnd0 <= 1'b0;
    else if (ClearRxEnd)
       RxEnd0 <= 1'b0;
    else if (SampleNibble && EndOfData4)
       RxEnd0 <= 1'b1;

 always_ff @(posedge RefClk)
    if (~Rstn) begin
       RxEnd1 <= 1'b0;
    end
    else if ( SamplingTick) begin
       RxEnd1 <= RxEnd0;
    end
 
 always_ff @(posedge RefClk)
   if (~Rstn)
       RxEnd2 <= 1'b0;
   else if (RxEnd1 & SamplingTick )
       RxEnd2 <= 1'b1;
   else if (RxEnd2 & SamplingTick) 
       RxEnd2 <= 1'b0;

 always_ff @(posedge RefClk)
    if (~Rstn)
      RxNibble <= 'h0;
    else if (RxEnd1 &&  SamplingTick)
      RxNibble <= 'h0;
    else if (SampleNibble)
      RxNibble <= RxNibbleInt4;
 
 always_ff @(posedge RefClk)
    if (!Rstn)
       NibbleVal = 1'b0;
    else if (RxEnd1 && SamplingTick) 
       NibbleVal = 1'b0;
    else if (SampleNibble )
       NibbleVal = 1'b1; 

//Carrier Sense  

assign StartOfRx = Phy2RmiiCrsDv3 & ~Phy2RmiiCrsDv4;
always_ff @(posedge RefClk)
   if (!Rstn)
      EndOfCrs <= 1'b0;
   else if (EndOfData5 & SamplingTick)
      EndOfCrs <= 1'b0;
   else if (~Phy2RmiiCrsDv3 & Phy2RmiiCrsDv4 & SamplingTick & Rmii2MiiCrs)  
      EndOfCrs <= 1'b1;

always_ff @(posedge RefClk)
   if (!Rstn)
      Rmii2MiiCrs <= 1'b0;
   else if (EndOfCrs && Rmii2MiiCrs && SamplingTick)
      Rmii2MiiCrs <= 1'b0;
   else if (StartOfRx && ~EndOfCrs && SamplingTick)
      Rmii2MiiCrs <= 1'b1;

always_ff @(posedge RefClk) 
   if (!Rstn)
      m_rx_crs <= 1'b0;
   else if (SamplingTick)
      m_rx_crs <= Rmii2MiiCrs;

 
 assign rx_clk   = RxIn10MbpsMode1 ? Clk2p5MHz:Clk25Mhz  ;
 assign m_rx_d   = RxNibble; 
 assign m_rx_en  = NibbleVal ;
 assign m_rx_err = Phy2RmiiRxErr3;
 assign m_rx_col = m_rx_crs & m_tx_en;


//Tx side
 logic [3:0] TxNibble;
 logic       TxNibbleVal;
 logic       TxHigh;
 logic       TxErr;

 assign tx_clk = (MBPS == 1'b1)  ? Clk25Mhz : Clk2p5MHz;
 assign TxSamplingTck   = (MBPS == 1'b1)  ? 1'b1 : PhyClkBy10Tck ;
 assign TxClk = tx_clk ;

 always_ff @(posedge RefClk)
   if (m_tx_en && TxClk)
      TxNibble <=  m_tx_d;
 
 always_ff @(posedge RefClk)
   if (TxClk)
      TxNibbleVal <= m_tx_en;

 always_ff @(posedge RefClk)
   if (~Rstn)
      TxHigh <= 1'b0;
   else if (TxNibbleVal & TxSamplingTck) 
      TxHigh <= ~TxHigh;

 always_ff @(posedge RefClk)
   if (TxSamplingTck)
      rmii_tx_en <= TxNibbleVal;

 always_ff @(posedge RefClk)
   if (~TxNibbleVal & TxSamplingTck)
      rmii_tx_d <= 'b0;
   else if (TxHigh & TxSamplingTck)
      rmii_tx_d <=  TxErr ? 2'b01 : TxNibble[3:2];
   else if (~TxHigh & TxSamplingTck)
      rmii_tx_d <=  TxErr ? 2'b01 : TxNibble[1:0];

 always_ff @(posedge RefClk)
   if (~Rstn)
     TxErr <= 1'b0;
   else if (m_tx_en && TxClk && m_tx_err) 
     TxErr <= 1'b1;
   else if (~m_tx_en && TxClk && TxErr) 
     TxErr <= 1'b0;

endmodule 
