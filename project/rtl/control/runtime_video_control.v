`timescale 1ns/1ps
// One runtime request in flight. Unsupported algorithms never change state.
module runtime_video_control(
 input wire clk,rst_n,lcd_vs_i,capture_busy_i,cmd_valid_i,
 output wire cmd_ready_o,input wire[7:0] cmd_opcode_i,cmd_flags_i,
 input wire[15:0] cmd_threshold_i,input wire[31:0] cmd_request_id_i,
 output reg response_valid_o,input wire response_ready_i,
 output reg[7:0] response_status_o,response_opcode_o,response_committed_flags_o,response_pending_flags_o,
 output reg[15:0] response_committed_threshold_o,response_pending_threshold_o,
 output reg[31:0] response_request_id_o,response_commit_counter_o,
 output wire median_enable_o,output wire[1:0] view_mode_o,output wire[10:0] threshold_o
);
 reg previous_vs,waiting_commit;
 reg[7:0] committed_flags,pending_flags;
 reg[15:0] committed_threshold,pending_threshold;
 reg[31:0] commit_counter,waiting_request;
 wire boundary=previous_vs&&!lcd_vs_i;
 assign cmd_ready_o=!waiting_commit&&!response_valid_o;
 assign median_enable_o=committed_flags[0];
 assign view_mode_o=committed_flags[2:1];
 assign threshold_o=committed_threshold[10:0];
 task reply;
  input[7:0] status,opcode,cflags,pflags;
  input[15:0] cthreshold,pthreshold;input[31:0] request,count;
  begin response_valid_o<=1;response_status_o<=status;response_opcode_o<=opcode;
   response_committed_flags_o<=cflags;response_pending_flags_o<=pflags;
   response_committed_threshold_o<=cthreshold;response_pending_threshold_o<=pthreshold;
   response_request_id_o<=request;response_commit_counter_o<=count;end
 endtask
 always @(posedge clk)begin
  if(!rst_n)begin
   previous_vs<=1;waiting_commit<=0;committed_flags<=5;pending_flags<=5;
   committed_threshold<=128;pending_threshold<=128;commit_counter<=0;waiting_request<=0;
   response_valid_o<=0;response_status_o<=0;response_opcode_o<=0;
   response_committed_flags_o<=5;response_pending_flags_o<=5;
   response_committed_threshold_o<=128;response_pending_threshold_o<=128;
   response_request_id_o<=0;response_commit_counter_o<=0;
  end else begin
   previous_vs<=lcd_vs_i;
   if(response_valid_o&&response_ready_i)response_valid_o<=0;
   if(boundary&&waiting_commit)begin
    committed_flags<=pending_flags;committed_threshold<=pending_threshold;
    commit_counter<=commit_counter+1'b1;waiting_commit<=0;
    reply(0,1,pending_flags,pending_flags,pending_threshold,pending_threshold,waiting_request,commit_counter+1'b1);
   end
   if(cmd_valid_i&&cmd_ready_o)begin
    if(cmd_request_id_i==0 || (cmd_opcode_i!=1&&cmd_opcode_i!=2) ||
       (cmd_opcode_i==2&&(cmd_flags_i!=0||cmd_threshold_i!=0)) ||
       (cmd_opcode_i==1&&(cmd_flags_i[7:5]!=0||cmd_flags_i[2:1]==3||cmd_threshold_i>2047)))
     reply(1,cmd_opcode_i,committed_flags,pending_flags,committed_threshold,pending_threshold,cmd_request_id_i,commit_counter);
    else if(cmd_opcode_i==1&&cmd_flags_i[4:3]!=0)
     reply(3,cmd_opcode_i,committed_flags,pending_flags,committed_threshold,pending_threshold,cmd_request_id_i,commit_counter);
    else if(cmd_opcode_i==1&&capture_busy_i)
     reply(2,cmd_opcode_i,committed_flags,pending_flags,committed_threshold,pending_threshold,cmd_request_id_i,commit_counter);
    else if(cmd_opcode_i==2)
     reply(0,2,committed_flags,pending_flags,committed_threshold,pending_threshold,cmd_request_id_i,commit_counter);
    else begin
     pending_flags<=cmd_flags_i;pending_threshold<=cmd_threshold_i;
     waiting_request<=cmd_request_id_i;waiting_commit<=1;
    end
   end
  end
 end
endmodule
