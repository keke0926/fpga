`timescale 1ns/1ps
// Small VCT1 adapter for Canny / same-frame auxiliary control, without image TX.
// 50=freeze, 51=resume, 52=query. Values return CRC low/high, counter=frame ID.
// Flags: committed[0]=locked,[1]=restored; pending[0]=local reference valid.
module frame_freeze_control(
 input wire clk,rst_n,part_mode_i,locked_i,restored_i,freezer_error_i,reference_valid_i,
 input wire[31:0]frame_i,crc_i,
 input wire[63:0]local_diagnostic_i,input wire[3:0]local_status_i,input wire local_active_i,local_locked_i,
 input wire cmd_valid_i,output wire cmd_ready_o,input wire[7:0]cmd_opcode_i,cmd_flags_i,
 input wire[15:0]cmd_data_i,input wire[31:0]cmd_request_i,
 output reg response_valid_o,input wire response_ready_i,
 output reg[7:0]response_status_o,response_opcode_o,response_flags_o,response_reference_o,
 output reg[15:0]response_crc_low_o,response_crc_high_o,
 output reg[31:0]response_request_o,response_frame_o,
 output reg snapshot_o,resume_o,output wire busy_o
);
 localparam IDLE=0,SHOT=1,WAIT_LOCK=2,HELD=3,RELEASE=4,WAIT_RELEASE=5;
 reg[2:0]state;reg reply_pending,cancelled;
 assign cmd_ready_o=(state==IDLE||state==HELD)&&!response_valid_o;
 assign busy_o=(state!=IDLE&&state!=HELD)||response_valid_o;
 task reply;
  input[7:0]status;
  begin
   response_valid_o<=1;response_status_o<=status;
   response_flags_o<={6'd0,restored_i,locked_i};response_reference_o<={7'd0,reference_valid_i};
   response_crc_low_o<=crc_i[15:0];response_crc_high_o<=crc_i[31:16];response_frame_o<=frame_i;
   reply_pending<=0;
  end
 endtask
 always@(posedge clk)begin
  if(!rst_n)begin
   state<=IDLE;reply_pending<=0;cancelled<=0;snapshot_o<=0;resume_o<=0;
   response_valid_o<=0;response_status_o<=0;response_opcode_o<=0;response_flags_o<=0;response_reference_o<=0;
   response_crc_low_o<=0;response_crc_high_o<=0;response_request_o<=0;response_frame_o<=0;
  end else begin
   snapshot_o<=0;resume_o<=0;
   if(response_valid_o&&response_ready_i)response_valid_o<=0;
   case(state)
    IDLE,HELD:begin
     if(state==HELD&&!part_mode_i)begin resume_o<=1;state<=RELEASE;end
     else if(cmd_valid_i&&cmd_ready_o)begin
      response_opcode_o<=cmd_opcode_i;response_request_o<=cmd_request_i;cancelled<=0;
      if(cmd_flags_i!=0||cmd_data_i!=0||cmd_request_i==0)reply(8'd1);
      else case(cmd_opcode_i)
       8'h50:if(!part_mode_i)reply(8'd2);
        else if(locked_i)reply(8'd0);
        else begin snapshot_o<=1;reply_pending<=1;state<=SHOT;end
       8'h51:if(state==HELD)begin resume_o<=1;reply_pending<=1;state<=RELEASE;end
        else reply(8'd0);
       8'h52:reply(8'd0);
       // 53 is read-only and never changes freeze ownership or reference state.
       8'h53:begin
        reply(8'd0);
        response_flags_o<={1'b0,reference_valid_i,local_locked_i,local_active_i,local_status_i};
        response_reference_o<=8'd1;
        response_crc_low_o<=local_diagnostic_i[15:0];
        response_crc_high_o<=local_diagnostic_i[31:16];
        response_frame_o<=local_diagnostic_i[63:32];
       end
       default:reply(8'd1);
      endcase
     end
    end
    // One cycle allows the freezer to consume the request and clear old events.
    SHOT:state<=WAIT_LOCK;
    WAIT_LOCK:begin
     if(!part_mode_i)begin resume_o<=1;cancelled<=1;state<=RELEASE;end
     else if(freezer_error_i)begin if(reply_pending)reply(8'd4);state<=IDLE;end
     else if(locked_i)begin if(reply_pending)reply(8'd0);state<=HELD;end
    end
    RELEASE:state<=WAIT_RELEASE;
    WAIT_RELEASE:if(restored_i||freezer_error_i)begin
     if(reply_pending)reply(freezer_error_i?8'd4:cancelled?8'd3:8'd0);
     state<=IDLE;
    end
   endcase
  end
 end
endmodule
