`timescale 1ns/1ps
// Capture owns UART for its whole transaction. A runtime response owns UART
// through the final physical stop bit; capture commands can queue outside it.
module runtime_uart_response(
 input wire clk,rst_n,capture_busy_i,capture_tx_valid_i,input wire[7:0] capture_tx_byte_i,
 output wire capture_tx_ready_o,capture_tx_idle_o,output wire uart_tx_valid_o,output wire[7:0] uart_tx_byte_o,
 input wire uart_tx_ready_i,uart_tx_idle_i,output wire runtime_active_o,
 input wire response_valid_i,output wire response_ready_o,
 input wire[7:0] response_status_i,response_opcode_i,response_committed_flags_i,response_pending_flags_i,
 input wire[15:0] response_committed_threshold_i,response_pending_threshold_i,
 input wire[31:0] response_request_id_i,response_commit_counter_i
);
 function[31:0] crc_step;
  input[31:0] c;input[7:0] b;reg[31:0] w;integer k;
  begin w=c^{24'd0,b};for(k=0;k<8;k=k+1)w=w[0]?(w>>1)^32'hedb88320:w>>1;crc_step=w;end
 endfunction
 localparam IDLE=0,CRC=1,SEND=2,WAIT_IDLE=3;
 reg[1:0] state;reg[191:0] packet;reg[31:0] crc;reg[4:0] index;
 assign runtime_active_o=state!=IDLE;
 assign response_ready_o=state==IDLE&&!capture_busy_i&&!capture_tx_valid_i&&uart_tx_idle_i;
 assign capture_tx_ready_o=state==IDLE&&uart_tx_ready_i;
 assign capture_tx_idle_o=state==IDLE&&uart_tx_idle_i;
 assign uart_tx_valid_o=state==IDLE?capture_tx_valid_i:state==SEND;
 assign uart_tx_byte_o=state==IDLE?capture_tx_byte_i:packet[index*8+:8];
 always @(posedge clk)begin
  if(!rst_n)begin state<=IDLE;packet<=0;crc<=32'hffffffff;index<=0;end
  else case(state)
   IDLE:if(response_valid_i&&response_ready_o)begin
    packet<={32'd0,response_commit_counter_i,response_request_id_i,response_pending_threshold_i,
     response_committed_threshold_i,response_pending_flags_i,response_committed_flags_i,response_opcode_i,response_status_i,32'h31534356};
    crc<=32'hffffffff;index<=0;state<=CRC;
   end
   CRC:begin
    crc<=crc_step(crc,packet[index*8+:8]);
    if(index==19)begin packet[191:160]<=~crc_step(crc,packet[index*8+:8]);index<=0;state<=SEND;end
    else index<=index+1'b1;
   end
   SEND:if(uart_tx_ready_i)begin
    if(index==23)state<=WAIT_IDLE;else index<=index+1'b1;
   end
   WAIT_IDLE:if(uart_tx_idle_i)begin state<=IDLE;index<=0;end
   default:state<=IDLE;
  endcase
 end
endmodule
