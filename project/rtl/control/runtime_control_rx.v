`timescale 1ns/1ps
// Competition firmware accepts the existing fixed-size VCT1 control packets.
// Full image upload and the old PC replay stream stay in the archived firmware.
module runtime_control_rx #(parameter integer CONTROL_TIMEOUT_CYCLES=144000000)(
 input wire clk,rst_n,rx_valid_i,framing_error_i,input wire[7:0]rx_byte_i,
 output reg cmd_valid_o,input wire cmd_ready_i,
 output reg[7:0]cmd_opcode_o,cmd_flags_o,output wire[7:0]pending_opcode_o,output reg[15:0]cmd_threshold_o,
 output reg[31:0]cmd_request_id_o
);
 function[31:0]crc_step;
  input[31:0]c;input[7:0]b;reg[31:0]w;integer k;
  begin w=c^{24'd0,b};for(k=0;k<8;k=k+1)w=w[0]?(w>>1)^32'hedb88320:w>>1;crc_step=w;end
 endfunction
 localparam TW=$clog2(CONTROL_TIMEOUT_CYCLES)>0?$clog2(CONTROL_TIMEOUT_CYCLES):1;
 reg active,damaged;reg[31:0]magic_shift,rx_crc;reg[127:0]packet;
 reg[3:0]received;reg[TW-1:0]waiting;
 assign pending_opcode_o=packet[39:32];
 wire[31:0]next_magic={rx_byte_i,magic_shift[31:8]};
 always@(posedge clk)begin
  if(!rst_n)begin
   active<=0;damaged<=0;magic_shift<=0;rx_crc<=0;packet<=0;received<=0;waiting<=0;
   cmd_valid_o<=0;cmd_opcode_o<=0;cmd_flags_o<=0;cmd_threshold_o<=0;cmd_request_id_o<=0;
  end else begin
   cmd_valid_o<=0;
   if(!active)begin
    if(framing_error_i)magic_shift<=0;
    else if(rx_valid_i)begin
     magic_shift<=next_magic;
     if(next_magic==32'h31544356)begin
      active<=1;damaged<=0;received<=4;waiting<=0;packet<={96'd0,next_magic};magic_shift<=0;
      rx_crc<=crc_step(crc_step(crc_step(crc_step(32'hffffffff,8'h56),8'h43),8'h54),8'h31);
     end
    end
   end else if(rx_valid_i||framing_error_i)begin
    waiting<=0;packet[received*8+:8]<=rx_byte_i;
    if(framing_error_i)damaged<=1;
    if(received<12)rx_crc<=crc_step(rx_crc,rx_byte_i);
    if(received==15)begin
     active<=0;magic_shift<=0;
     if(!damaged&&!framing_error_i&&{rx_byte_i,packet[119:96]}==~rx_crc&&cmd_ready_i)begin
      cmd_valid_o<=1;cmd_opcode_o<=packet[39:32];cmd_flags_o<=packet[47:40];
      cmd_threshold_o<=packet[63:48];cmd_request_id_o<=packet[95:64];
     end
    end else received<=received+1'b1;
   end else if(waiting==CONTROL_TIMEOUT_CYCLES-1)begin active<=0;magic_shift<=0;waiting<=0;end
   else waiting<=waiting+1'b1;
  end
 end
endmodule
