`timescale 1ns/1ps
// One outstanding transfer. Bundled payloads remain stable across the two-flop
// request/ack synchronizers until the consumer accepts the response. No RAM.
// Task cancellation must drain this mailbox; only the common system reset clears it.
module part_ddr_mailbox(
 input wire s_clk,s_rst_n,d_clk,d_rst_n,
 input wire s_req_valid,output wire s_req_ready,input wire s_req_write,
 input wire[31:0]s_req_addr,input wire[127:0]s_req_data,
 output reg s_rsp_valid,input wire s_rsp_ready,output wire s_rsp_error,output wire[127:0]s_rsp_data,
 output wire d_req_valid,input wire d_req_ready,output wire d_req_write,
 output wire[31:0]d_req_addr,output wire[127:0]d_req_data,
 input wire d_rsp_valid,output wire d_rsp_ready,input wire d_rsp_error,input wire[127:0]d_rsp_data
);
 reg req_toggle,ack_toggle,s_busy;
 reg req_write_hold;reg[31:0]req_addr_hold;reg[127:0]req_data_hold,rsp_data_hold;reg rsp_error_hold;
 (* async_reg="true" *) reg[1:0]req_sync,ack_sync;
 reg req_seen,ack_seen;reg[1:0]d_state;
 assign s_req_ready=s_rst_n&&!s_busy&&!s_rsp_valid;
 assign s_rsp_error=rsp_error_hold;assign s_rsp_data=rsp_data_hold;
 assign d_req_valid=d_rst_n&&d_state==1;
 assign d_req_write=req_write_hold;assign d_req_addr=req_addr_hold;assign d_req_data=req_data_hold;
 assign d_rsp_ready=d_state==2;
 always@(posedge s_clk)begin
  if(!s_rst_n)begin
   req_toggle<=0;s_busy<=0;req_write_hold<=0;req_addr_hold<=0;req_data_hold<=0;
   ack_sync<=0;ack_seen<=0;s_rsp_valid<=0;
  end else begin
   ack_sync<={ack_sync[0],ack_toggle};
   if(s_req_valid&&s_req_ready)begin
    req_write_hold<=s_req_write;req_addr_hold<=s_req_addr;req_data_hold<=s_req_data;
    req_toggle<=!req_toggle;s_busy<=1;
   end
   if(ack_sync[1]!=ack_seen)begin ack_seen<=ack_sync[1];s_rsp_valid<=1;end
   if(s_rsp_valid&&s_rsp_ready)begin s_rsp_valid<=0;s_busy<=0;end
  end
 end
 always@(posedge d_clk)begin
  if(!d_rst_n)begin
   req_sync<=0;req_seen<=0;ack_toggle<=0;d_state<=0;rsp_data_hold<=0;rsp_error_hold<=0;
  end else begin
   req_sync<={req_sync[0],req_toggle};
   case(d_state)
    0:if(req_sync[1]!=req_seen)begin req_seen<=req_sync[1];d_state<=1;end
    1:if(d_req_valid&&d_req_ready)d_state<=2;
    2:if(d_rsp_valid&&d_rsp_ready)begin
     rsp_data_hold<=d_rsp_data;rsp_error_hold<=d_rsp_error;ack_toggle<=!ack_toggle;d_state<=0;
    end
   endcase
  end
 end
endmodule
