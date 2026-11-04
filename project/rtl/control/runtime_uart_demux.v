`timescale 1ns/1ps
// A recognized 16-byte packet owns framing through its entire body. CAP1 is
// forwarded byte-for-byte, including bad CRC and partial packets, to its
// original parser. VCT1 errors never become capture protocol errors.
module runtime_uart_demux #(parameter integer CONTROL_TIMEOUT_CYCLES=144000000)(
 input wire clk,rst_n,rx_valid_i,framing_error_i,input wire[7:0] rx_byte_i,
 output reg persistence_start_o,persistence_rx_valid_o,persistence_error_o,
 output reg[7:0] persistence_rx_byte_o,output wire persistence_rx_active_o,
 output reg capture_rx_valid_o,output reg[7:0] capture_rx_byte_o,
 output wire capture_rx_active_o,capture_framing_error_o,output reg cmd_valid_o,input wire cmd_ready_i,
 output reg[7:0] cmd_opcode_o,cmd_flags_o,output reg[15:0] cmd_threshold_o,
 output reg[31:0] cmd_request_id_o,output reg dropped_o
);
 function[31:0] crc_step;
  input[31:0] c;input[7:0] b;reg[31:0] w;integer k;
  begin w=c^{24'd0,b};for(k=0;k<8;k=k+1)w=w[0]?(w>>1)^32'hedb88320:w>>1;crc_step=w;end
 endfunction
 localparam TW=$clog2(CONTROL_TIMEOUT_CYCLES)>0?$clog2(CONTROL_TIMEOUT_CYCLES):1;
 localparam SEEK=0,CAP=1,RUNTIME=2,DRAIN=3,GUARD=4,PHEAD=5,PSTREAM=6,PBAD=7;
 reg[16:0] persistence_remaining;
 reg[2:0] state;reg runtime_damaged;reg[31:0] magic_shift,rx_crc;reg[127:0] packet;
 reg[4:0] received,forwarded;reg[2:0] guard_count;reg[TW-1:0] waiting;
 wire[31:0] next_magic={rx_byte_i,magic_shift[31:8]};
 assign capture_rx_active_o=state==CAP||state==DRAIN||state==GUARD;
 // Only an already recognized VCT1 body owns a physical framing error.
 // Before magic recognition and inside CAP1, retain the original global error.
 assign persistence_rx_active_o=state==PHEAD||state==PSTREAM||state==PBAD;
 assign capture_framing_error_o=framing_error_i&&(state!=RUNTIME)&&!persistence_rx_active_o;
 always @(posedge clk)begin
  if(!rst_n)begin
   persistence_start_o<=0;persistence_rx_valid_o<=0;persistence_rx_byte_o<=0;persistence_error_o<=0;persistence_remaining<=0;
   state<=SEEK;runtime_damaged<=0;magic_shift<=0;rx_crc<=32'hffffffff;packet<=0;received<=0;forwarded<=0;guard_count<=0;waiting<=0;
   capture_rx_valid_o<=0;capture_rx_byte_o<=0;cmd_valid_o<=0;cmd_opcode_o<=0;cmd_flags_o<=0;
   cmd_threshold_o<=0;cmd_request_id_o<=0;dropped_o<=0;
  end else begin
   capture_rx_valid_o<=0;cmd_valid_o<=0;dropped_o<=0;
   persistence_start_o<=0;persistence_rx_valid_o<=0;persistence_error_o<=0;
   // A framing error consumes one UART byte. Retain packet ownership through
   // the remaining body, even if those bytes spell CAP1; then discard it.
   if(framing_error_i&&persistence_rx_active_o)begin
    state<=PBAD;waiting<=0;persistence_error_o<=1;
   end
   else if(framing_error_i&&state==RUNTIME)begin
    runtime_damaged<=1;waiting<=0;
    if(received==15)begin state<=SEEK;magic_shift<=0;end else received<=received+1'b1;
   end
   else case(state)
    SEEK:if(rx_valid_i)begin
     magic_shift<=next_magic;
     if(next_magic==32'h31504143||next_magic==32'h31544356||next_magic==32'h31545350)begin
      state<=(next_magic==32'h31504143)?CAP:(next_magic==32'h31544356)?RUNTIME:PHEAD;
      persistence_start_o<=next_magic==32'h31545350;
      received<=4;forwarded<=0;waiting<=0;
      packet<={96'd0,next_magic};magic_shift<=0;runtime_damaged<=0;
      rx_crc<=crc_step(crc_step(crc_step(crc_step(32'hffffffff,next_magic[7:0]),next_magic[15:8]),next_magic[23:16]),next_magic[31:24]);
     end
    end
    CAP:begin
     if(forwarded<received)begin
      capture_rx_valid_o<=1;capture_rx_byte_o<=packet[forwarded*8+:8];forwarded<=forwarded+1'b1;
     end
     if(rx_valid_i)begin
      packet[received*8+:8]<=rx_byte_i;received<=received+1'b1;waiting<=0;
      if(received==15)state<=DRAIN;
     end else if(waiting==CONTROL_TIMEOUT_CYCLES-1)begin state<=SEEK;magic_shift<=0;waiting<=0;end
     else waiting<=waiting+1'b1;
    end
    DRAIN:if(forwarded<16)begin
     capture_rx_valid_o<=1;capture_rx_byte_o<=packet[forwarded*8+:8];forwarded<=forwarded+1'b1;
     if(forwarded==15)begin state<=GUARD;guard_count<=3;end
    end
    GUARD:if(guard_count==0)begin state<=SEEK;magic_shift<=0;end else guard_count<=guard_count-1'b1;
    RUNTIME:if(rx_valid_i)begin
     packet[received*8+:8]<=rx_byte_i;waiting<=0;
     if(received<12)rx_crc<=crc_step(rx_crc,rx_byte_i);
     if(received==15)begin
      state<=SEEK;magic_shift<=0;
      if(!runtime_damaged&&{rx_byte_i,packet[119:96]}==~rx_crc)begin
       if(cmd_ready_i)begin cmd_valid_o<=1;cmd_opcode_o<=packet[39:32];cmd_flags_o<=packet[47:40];
        cmd_threshold_o<=packet[63:48];cmd_request_id_o<=packet[95:64];end
       else dropped_o<=1;
      end
     end else received<=received+1'b1;
    end else if(waiting==CONTROL_TIMEOUT_CYCLES-1)begin state<=SEEK;magic_shift<=0;waiting<=0;end
    else waiting<=waiting+1'b1;
    PHEAD:begin
     if(forwarded<received)begin
      persistence_rx_valid_o<=1;persistence_rx_byte_o<=packet[forwarded*8+:8];forwarded<=forwarded+1'b1;
     end
     if(rx_valid_i)begin
      packet[received*8+:8]<=rx_byte_i;received<=received+1'b1;waiting<=0;
      if(received==7)persistence_remaining<={1'b0,rx_byte_i,packet[55:48]}+17'd4;
      if(received==15)state<=PSTREAM;
     end else if(waiting==CONTROL_TIMEOUT_CYCLES-1)begin
      state<=SEEK;magic_shift<=0;waiting<=0;persistence_error_o<=1;
     end else waiting<=waiting+1'b1;
    end
    PSTREAM:begin
     if(forwarded<16)begin
      persistence_rx_valid_o<=1;persistence_rx_byte_o<=packet[forwarded*8+:8];forwarded<=forwarded+1'b1;
     end
     if(rx_valid_i)begin
      waiting<=0;
      if(forwarded<16)begin state<=PBAD;persistence_error_o<=1;end
      else begin
       persistence_rx_valid_o<=1;persistence_rx_byte_o<=rx_byte_i;
       if(persistence_remaining==1)begin state<=SEEK;magic_shift<=0;end
       else persistence_remaining<=persistence_remaining-1'b1;
      end
     end else if(waiting==CONTROL_TIMEOUT_CYCLES-1)begin
      state<=SEEK;magic_shift<=0;waiting<=0;persistence_error_o<=1;
     end else waiting<=waiting+1'b1;
    end
    // A damaged variable-length packet retains ownership until the sender is
    // silent for a complete timeout. Slow fragments must not expose body bytes
    // to CAP1/VCT1 magic detection after a wall-clock timeout from the error.
    PBAD:if(rx_valid_i)waiting<=0;
    else if(waiting==CONTROL_TIMEOUT_CYCLES-1)begin state<=SEEK;magic_shift<=0;waiting<=0;end
    else waiting<=waiting+1'b1;
    default:state<=SEEK;
   endcase
  end
 end
endmodule
