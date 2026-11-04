`timescale 1ns/1ps
// VCT1-compatible unified runtime control. Physical keys are already qualified
// one-clock pulses in clk's domain. No raw GPIO synchronization lives here.
module runtime_video_control_keys #(parameter ENABLE_LOCAL_CANCEL=0)(
 input wire local_mode_cancel_i,output wire[31:0]configuration_epoch_o,
 input wire clk,rst_n,lcd_vs_i,capture_busy_i,cmd_valid_i,
 output wire cmd_ready_o,input wire[7:0] cmd_opcode_i,cmd_flags_i,
 input wire[15:0] cmd_threshold_i,input wire[31:0] cmd_request_id_i,
 output reg response_valid_o,input wire response_ready_i,
 output reg[7:0] response_status_o,response_opcode_o,response_committed_flags_o,response_pending_flags_o,
 output reg[15:0] response_committed_threshold_o,response_pending_threshold_o,
 output reg[31:0] response_request_id_o,response_commit_counter_o,
 output wire auto_enable_o,output wire median_enable_o,output wire[1:0] view_mode_o,output wire[10:0] threshold_o,
 input wire[4:0] key_press_i,output wire app_mode_o
);
 reg previous_vs,waiting_commit;
 reg committed_app,pending_app;
 reg[7:0] committed_flags,pending_flags;
 reg[10:0] committed_generic_manual,committed_part_manual;
 reg[10:0] pending_generic_manual,pending_part_manual;
 reg[31:0] commit_counter,waiting_request;
 reg next_app;
 reg[7:0] next_flags;
 reg[10:0] next_generic_manual,next_part_manual;

 wire boundary=previous_vs&&!lcd_vs_i;
 wire accepted=cmd_valid_i&&cmd_ready_o;
 wire invalid_request=cmd_request_id_i==0 || (cmd_opcode_i!=1&&cmd_opcode_i!=2) ||
  (cmd_opcode_i==2&&(cmd_flags_i!=0||cmd_threshold_i!=0)) ||
  (cmd_opcode_i==1&&(cmd_flags_i[7:5]!=0||cmd_flags_i[2:1]==3||cmd_threshold_i>2047));
 wire unsupported_request=cmd_opcode_i==1&&cmd_flags_i[4];
 wire busy_request=cmd_opcode_i==1&&capture_busy_i;
 wire valid_set=accepted&&!invalid_request&&!unsupported_request&&!busy_request&&cmd_opcode_i==1;
 wire keys_allowed=!capture_busy_i&&!waiting_commit&&!response_valid_o&&!accepted;
 wire[10:0] committed_threshold=committed_app?committed_part_manual:committed_generic_manual;
 wire[10:0] pending_threshold=pending_app?pending_part_manual:pending_generic_manual;
 // Hidden-bank changes are saved at a boundary but do not increment the
 // counter unless the published app/flags/active manual value also changes.
 wire published_change=(committed_app!=pending_app)||(committed_flags!=pending_flags)||
  (committed_threshold!=pending_threshold);
 wire[31:0] boundary_counter=commit_counter+((waiting_commit||published_change)?32'd1:32'd0);
 // A request accepted on the commit edge observes that edge's publication.
 // This also makes same-edge SET target the app that has just become visible.
 wire visible_app=boundary?pending_app:committed_app;
 wire[7:0] visible_flags=boundary?pending_flags:committed_flags;
 wire[10:0] visible_generic_manual=boundary?pending_generic_manual:committed_generic_manual;
 wire[10:0] visible_part_manual=boundary?pending_part_manual:committed_part_manual;
 wire[10:0] visible_threshold=visible_app?visible_part_manual:visible_generic_manual;
 wire[31:0] visible_counter=boundary?boundary_counter:commit_counter;

 assign cmd_ready_o=!waiting_commit&&!response_valid_o;
 assign app_mode_o=committed_app;
 assign configuration_epoch_o=commit_counter;
 // Keep the proven guarded shape: protocol rejects bit4, physical app is separate.
 assign auto_enable_o=committed_flags[3] && !committed_flags[4];
 assign median_enable_o=committed_flags[0];
 assign view_mode_o=committed_flags[2:1];
 assign threshold_o=committed_threshold;

 function[10:0] increased;
  input[10:0] value;
  reg[11:0] sum;
  begin
   sum={1'b0,value}+12'd8;
   increased=(sum>12'd2047)?11'd2047:sum[10:0];
  end
 endfunction
 function[10:0] decreased;
  input[10:0] value;
  begin decreased=(value<11'd8)?11'd0:value-11'd8;end
 endfunction

 always @* begin
  next_app=pending_app;next_flags=pending_flags;
  next_generic_manual=pending_generic_manual;next_part_manual=pending_part_manual;
  if(keys_allowed)begin
   // Simultaneous toggles are resolved before adjustment, as vision_control.
   if(key_press_i[0])next_app=!pending_app;
   if(key_press_i[1])next_flags[3]=!pending_flags[3];
   if(key_press_i[2])next_flags[2:1]=(pending_flags[2:1]==2)?2'd0:pending_flags[2:1]+2'd1;
   if(!next_flags[3]&&(key_press_i[3]^key_press_i[4]))begin
    if(next_app)next_part_manual=key_press_i[3]?increased(pending_part_manual):decreased(pending_part_manual);
    else next_generic_manual=key_press_i[3]?increased(pending_generic_manual):decreased(pending_generic_manual);
   end
  end
  if(ENABLE_LOCAL_CANCEL&&local_mode_cancel_i&&!waiting_commit&&!accepted)next_app=!pending_app;
  if(valid_set)begin
   // UART SET owns the next commit. Cancel uncommitted physical edits and
   // restore the other app's last published bank; bit4 never selects app.
   next_app=visible_app;next_flags=cmd_flags_i;
   next_generic_manual=visible_generic_manual;next_part_manual=visible_part_manual;
   if(visible_app)next_part_manual=cmd_threshold_i[10:0];
   else next_generic_manual=cmd_threshold_i[10:0];
  end
 end

 task reply;
  input[7:0] status,opcode,cflags,pflags;
  input[15:0] cthreshold,pthreshold;
  input[31:0] request,count;
  begin
   response_valid_o<=1;response_status_o<=status;response_opcode_o<=opcode;
   response_committed_flags_o<=cflags;response_pending_flags_o<=pflags;
   response_committed_threshold_o<=cthreshold;response_pending_threshold_o<=pthreshold;
   response_request_id_o<=request;response_commit_counter_o<=count;
  end
 endtask

 always @(posedge clk)begin
  if(!rst_n)begin
   previous_vs<=1;waiting_commit<=0;waiting_request<=0;commit_counter<=0;
   committed_app<=1;pending_app<=1;committed_flags<=13;pending_flags<=13;
   committed_generic_manual<=128;committed_part_manual<=128;
   pending_generic_manual<=128;pending_part_manual<=128;
   response_valid_o<=0;response_status_o<=0;response_opcode_o<=0;
   response_committed_flags_o<=13;response_pending_flags_o<=13;
   response_committed_threshold_o<=128;response_pending_threshold_o<=128;
   response_request_id_o<=0;response_commit_counter_o<=0;
  end else begin
   previous_vs<=lcd_vs_i;
   pending_app<=next_app;pending_flags<=next_flags;
   pending_generic_manual<=next_generic_manual;pending_part_manual<=next_part_manual;
   if(response_valid_o&&response_ready_i)response_valid_o<=0;
   if(boundary)begin
    // Publish OLD pending atomically. A same-edge key remains next pending.
    committed_app<=pending_app;committed_flags<=pending_flags;
    committed_generic_manual<=pending_generic_manual;committed_part_manual<=pending_part_manual;
    commit_counter<=boundary_counter;
    if(waiting_commit)begin
     waiting_commit<=0;
     reply(0,1,pending_flags,pending_flags,{5'd0,pending_threshold},{5'd0,pending_threshold},waiting_request,boundary_counter);
    end
   end
   if(accepted)begin
    if(invalid_request)
     reply(1,cmd_opcode_i,visible_flags,pending_flags,{5'd0,visible_threshold},{5'd0,pending_threshold},cmd_request_id_i,visible_counter);
    else if(unsupported_request)
     reply(3,cmd_opcode_i,visible_flags,pending_flags,{5'd0,visible_threshold},{5'd0,pending_threshold},cmd_request_id_i,visible_counter);
    else if(busy_request)
     reply(2,cmd_opcode_i,visible_flags,pending_flags,{5'd0,visible_threshold},{5'd0,pending_threshold},cmd_request_id_i,visible_counter);
    else if(cmd_opcode_i==2)
     reply(0,2,visible_flags,pending_flags,{5'd0,visible_threshold},{5'd0,pending_threshold},cmd_request_id_i,visible_counter);
    else begin
     waiting_request<=cmd_request_id_i;waiting_commit<=1;
    end
   end
  end
 end
endmodule
