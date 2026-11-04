`timescale 1ns/1ps
// VCT1 0x40..0x44. RUN acknowledges configuration acceptance only.
// Snapshot data maps onto VCS1's two sixteen-bit payload fields.
module canny_roi_control #(parameter TRANSACTION_TIMEOUT_CYCLES=96000000)(
 input wire clk,rst_n,part_mode_i,locked_i,video_error_i,
 input wire [31:0] locked_frame_i,locked_crc_i,
 input wire [15:0] frontend_i,input wire invalidate_i,
 input wire cmd_valid_i,output wire cmd_ready_o,
 input wire [7:0] cmd_opcode_i,cmd_flags_i,input wire [15:0] cmd_data_i,
 input wire [31:0] cmd_request_i,
 output reg response_valid_o,input wire response_ready_i,
 output reg [7:0] response_status_o,response_opcode_o,response_committed_flags_o,response_pending_flags_o,
 output reg [15:0] response_data_low_o,response_data_high_o,
 output reg [31:0] response_request_o,response_counter_o,
 input wire [3:0] vision_state_i,vision_error_i,
 input wire [14:0] sample_count_i,edge_count_i,
 input wire [31:0] cycles_i,display_commits_i,
 output wire enable_o,output reg [9:0] roi_x_o,roi_y_o,
 output reg [7:0] low_o,high_o,
 output reg [31:0] frame_o,crc_o,reference_o,tag_o
);
 reg active,staged;
 reg [31:0] transaction_id,timer,configuration_commits;
 reg [3:0] word_index;
 reg [15:0] words[0:11];
 reg [15:0] committed_frontend;
 reg snapshot_valid;
 reg [31:0] snapshot_request,snapshot_generation,snapshot[0:12];
 reg [7:0] snapshot_flags,snapshot_pending;
 wire allowed=part_mode_i&&locked_i&&!video_error_i&&!invalidate_i;
 assign enable_o=active&&allowed&&frame_o==locked_frame_i&&crc_o==locked_crc_i&&committed_frontend==frontend_i;
 assign cmd_ready_o=!response_valid_o;
 wire accepted=cmd_valid_i&&cmd_ready_o;
 wire expired=staged&&timer>=TRANSACTION_TIMEOUT_CYCLES-1;
 wire owned=staged&&!expired&&cmd_request_i==transaction_id;
 wire [7:0] live_flags={4'b0,(enable_o&&vision_state_i==4'd5),locked_i,part_mode_i,enable_o};
 wire [7:0] live_pending={7'b0,(staged&&!expired&&allowed)};
 task reply;
 input [7:0] status;input[31:0] value;
 begin
  response_valid_o<=1;response_status_o<=status;response_opcode_o<=cmd_opcode_i;
  response_request_o<=cmd_request_i;response_data_low_o<=value[15:0];response_data_high_o<=value[31:16];
  response_counter_o<=configuration_commits;
  response_committed_flags_o<=live_flags;response_pending_flags_o<=live_pending;
 end endtask
 integer i;
 always @(posedge clk)begin
  if(!rst_n)begin
   active<=0;staged<=0;transaction_id<=0;timer<=0;word_index<=0;configuration_commits<=0;
   roi_x_o<=0;roi_y_o<=0;low_o<=0;high_o<=0;frame_o<=0;crc_o<=0;reference_o<=0;tag_o<=0;committed_frontend<=0;
   snapshot_valid<=0;snapshot_request<=0;snapshot_generation<=0;snapshot_flags<=0;snapshot_pending<=0;
   response_valid_o<=0;response_status_o<=0;response_opcode_o<=0;response_request_o<=0;
   response_data_low_o<=0;response_data_high_o<=0;response_counter_o<=0;response_committed_flags_o<=0;response_pending_flags_o<=0;
   for(i=0;i<12;i=i+1)words[i]<=0;
   for(i=0;i<13;i=i+1)snapshot[i]<=0;
  end else begin
   if(response_valid_o&&response_ready_i)response_valid_o<=0;
   if(!enable_o)active<=0;
   if(!allowed||expired)begin staged<=0;timer<=0;end
   else if(staged)timer<=timer+1'b1;
   else timer<=0;
   if(accepted)begin
    if(cmd_request_i==0)reply(1,0);
    else case(cmd_opcode_i)
     8'h40:begin
      if(cmd_flags_i!=0||cmd_data_i!=0)reply(1,0);
      else if(!allowed)reply(2,0);
      else begin
       active<=0;staged<=1;transaction_id<=cmd_request_i;timer<=0;word_index<=0;
       for(i=0;i<12;i=i+1)words[i]<=0;
       reply(0,0);response_committed_flags_o<=live_flags&8'h06;response_pending_flags_o<=1;
      end
     end
     8'h41:begin
      if(!owned||!allowed)reply(2,0);
      else if(word_index>=12||cmd_flags_i!={4'b0,word_index}||(word_index==11&&cmd_data_i!=0))reply(1,0);
      else begin words[word_index]<=cmd_data_i;word_index<=word_index+1'b1;reply(0,{28'b0,word_index}+1'b1);end
     end
     8'h42:begin
      if(cmd_flags_i!=0||cmd_data_i!=0)reply(1,0);
      else if(!owned||!allowed||word_index!=12)reply(2,0);
      else if({words[1],words[0]}!=locked_frame_i||{words[3],words[2]}!=locked_crc_i)reply(2,0);
      else if({words[5],words[4]}==0||{words[7],words[6]}==0||words[8]>896||words[9]>472||words[10][7:0]==0||words[10][15:8]<words[10][7:0])reply(4,0);
      else begin
       active<=1;staged<=0;timer<=0;roi_x_o<=words[8][9:0];roi_y_o<=words[9][9:0];
       low_o<=words[10][7:0];high_o<=words[10][15:8];frame_o<={words[1],words[0]};crc_o<={words[3],words[2]};
       reference_o<={words[5],words[4]};tag_o<={words[7],words[6]};committed_frontend<=frontend_i;
       configuration_commits<=configuration_commits+1'b1;reply(0,0);response_counter_o<=configuration_commits+1'b1;
       response_committed_flags_o<={5'b0,locked_i,part_mode_i,1'b1};response_pending_flags_o<=0;
      end
     end
     8'h43:begin
      if(cmd_flags_i>12||cmd_data_i!=0)reply(1,0);
      else if(cmd_flags_i==0)begin
       snapshot_valid<=1;snapshot_request<=cmd_request_i;snapshot_generation<=snapshot_generation+1'b1;
       snapshot_flags<=live_flags;snapshot_pending<=live_pending;
       snapshot[0]<=32'h00030080;snapshot[1]<=frame_o;snapshot[2]<=crc_o;
       snapshot[3]<=reference_o;snapshot[4]<=tag_o;snapshot[5]<={6'b0,roi_y_o,6'b0,roi_x_o};
       snapshot[6]<={frontend_i,high_o,low_o};snapshot[7]<={16'b0,vision_error_i,vision_state_i,8'b0};
       snapshot[8]<={1'b0,edge_count_i,1'b0,sample_count_i};snapshot[9]<=cycles_i;snapshot[10]<=display_commits_i;
       snapshot[11]<={16'b0,committed_frontend};snapshot[12]<=configuration_commits;
       reply(0,32'h00030080);response_counter_o<=snapshot_generation+1'b1;
      end else if(!snapshot_valid||cmd_request_i!=snapshot_request)reply(2,0);
      else begin
       reply(0,snapshot[cmd_flags_i]);response_counter_o<=snapshot_generation;
       response_committed_flags_o<=snapshot_flags;response_pending_flags_o<=snapshot_pending;
      end
     end
     8'h44:begin
      if(cmd_flags_i!=0||cmd_data_i!=0)reply(1,0);
      else begin active<=0;staged<=0;timer<=0;word_index<=0;reply(0,0);response_committed_flags_o<=live_flags&8'h06;response_pending_flags_o<=0;end
     end
     default:reply(1,0);
    endcase
   end
  end
 end
endmodule
