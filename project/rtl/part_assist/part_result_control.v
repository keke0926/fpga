`timescale 1ns/1ps
// Host/FPGA detector-neutral result transaction. Coordinates on the wire are
// half-open; RAM packs inclusive endpoints so full-width1024 fits ten bits.
// 512 records per bank, eight cached rows per display page. No defect verdict.
module part_result_control #(parameter PAGE_CYCLES=96000000,COMMIT_TIMEOUT_CYCLES=96000000)(
 input wire clk,rst_n,sof_i,part_mode_i,locked_i,video_error_i,
 input wire [31:0] locked_frame_i,locked_crc_i,
 input wire cmd_valid_i,output wire cmd_ready_o,
 input wire [7:0] cmd_opcode_i,cmd_flags_i,input wire [15:0] cmd_data_i,
 input wire [31:0] cmd_request_i,
 output reg response_valid_o,input wire response_ready_i,
 output reg [7:0] response_status_o,response_opcode_o,response_committed_flags_o,response_pending_flags_o,
 output reg [15:0] response_committed_count_o,response_pending_count_o,
 output reg [31:0] response_request_o,response_counter_o,
 output wire display_valid_o,output wire [319:0] display_boxes_o,
 output wire [3:0] display_rows_o,output wire [9:0] display_count_o,
 output wire [5:0] display_page_o,output wire [6:0] display_pages_o,
 output wire [31:0] display_reference_o
);
 (* ram_style="block" *) reg [39:0] records[0:1023];
 reg [39:0] read_data;
 reg [39:0] active_boxes[0:7],pending_boxes[0:7];
 reg transaction,back_bank,active_bank,active_valid,streaming;
 reg [31:0] transaction_id,word_count;
 reg [9:0] expected_count,written;
 reg [2:0] meta_index;reg [1:0] lane;
 reg [95:0] meta;
 reg [29:0] partial_box;
 reg [31:0] active_frame,active_crc,active_reference;
 reg [9:0] active_count;reg [3:0] active_rows;reg [5:0] active_page;
 reg [6:0] active_pages;
 reg pending,pending_ready,pending_ack,pending_bank;
 reg [31:0] pending_frame,pending_crc,pending_reference,pending_request;
 reg [9:0] pending_count;reg [3:0] pending_rows;reg [5:0] pending_page;
 reg [6:0] pending_pages;
 reg loading,read_wait;reg [2:0] read_index;
 reg [31:0] page_timer,pending_timer;
 wire [9:0] read_address={pending_bank,pending_page,3'b000}+{7'd0,read_index};
 wire allowed=part_mode_i&&locked_i&&!video_error_i;
 wire active_matches=allowed&&active_frame==locked_frame_i&&active_crc==locked_crc_i;
 wire pending_matches=allowed&&pending_frame==locked_frame_i&&pending_crc==locked_crc_i;
 wire publish=sof_i&&pending&&pending_ready&&pending_matches;
 wire active_now=publish ? 1'b1 : active_valid&&active_matches;
 assign display_valid_o=active_now;
 assign display_count_o=publish?pending_count:active_count;
 assign display_rows_o=publish?pending_rows:active_rows;
 assign display_page_o=publish?pending_page:active_page;
 assign display_pages_o=publish?pending_pages:active_pages;
 assign display_reference_o=publish?pending_reference:active_reference;
 genvar g;generate for(g=0;g<8;g=g+1)begin:display_mux
  assign display_boxes_o[g*40+:40]=publish?pending_boxes[g]:active_boxes[g];
 end endgenerate
 // Background pagination cannot apply UART backpressure: demux has no queue.
 // Commit transactions block until their one SOF ACK; auto-pages accept queries
 // and CLEAR, while a new BEGIN cancels their speculative read/publication.
 assign cmd_ready_o=!response_valid_o&&(!pending||!pending_ack);
 wire accepted=cmd_valid_i&&cmd_ready_o;
 wire owned=transaction&&cmd_request_i==transaction_id;
 wire [9:0] commit_count=streaming?written:expected_count;
 wire [9:0] end_minus_one=cmd_data_i[9:0]-10'd1;
 function [3:0] page_rows;
 input [9:0] count;input[5:0] page;reg[9:0] remaining;
 begin remaining=count-{1'b0,page,3'b000};page_rows=remaining>8?4'd8:remaining[3:0];end
 endfunction
 task reply;
 input[7:0] status,opcode;input[31:0] request;
 begin
  response_valid_o<=1;response_status_o<=status;response_opcode_o<=opcode;response_request_o<=request;
  response_committed_flags_o<={5'b0,locked_i,part_mode_i,active_now};response_pending_flags_o<={7'b0,transaction};
  response_committed_count_o<={6'b0,display_count_o};response_pending_count_o<={6'b0,written};response_counter_o<=word_count;
 end endtask
 integer i;
 always @(posedge clk)begin
  read_data<=records[read_address];
  if(rst_n&&allowed&&accepted&&owned&&cmd_opcode_i==8'h32&&meta_index==6&&
     cmd_flags_i==3&&lane==3&&written<expected_count&&cmd_data_i>partial_box[19:10]&&cmd_data_i<=600)
   records[{back_bank,written[8:0]}]<={end_minus_one,partial_box};
 end
 always @(posedge clk)begin
  if(!rst_n)begin
   transaction<=0;back_bank<=1;active_bank<=0;active_valid<=0;streaming<=0;transaction_id<=0;word_count<=0;
   expected_count<=0;written<=0;meta_index<=0;lane<=0;meta<=0;partial_box<=0;
   active_frame<=0;active_crc<=0;active_reference<=0;active_count<=0;active_rows<=0;active_page<=0;active_pages<=0;
   pending<=0;pending_ready<=0;pending_ack<=0;pending_bank<=0;pending_frame<=0;pending_crc<=0;
   pending_reference<=0;pending_request<=0;pending_count<=0;pending_rows<=0;pending_page<=0;pending_pages<=0;
   loading<=0;read_wait<=0;read_index<=0;page_timer<=0;pending_timer<=0;
   response_valid_o<=0;response_status_o<=0;response_opcode_o<=0;response_committed_flags_o<=0;response_pending_flags_o<=0;
   response_committed_count_o<=0;response_pending_count_o<=0;response_request_o<=0;response_counter_o<=0;
   for(i=0;i<8;i=i+1)begin active_boxes[i]<=0;pending_boxes[i]<=0;end
  end else begin
   if(response_valid_o&&response_ready_i)response_valid_o<=0;
   if(!pending)pending_timer<=0;else pending_timer<=pending_timer+1'b1;
   if(!active_matches)begin active_valid<=0;page_timer<=0;end
   if(!allowed)begin
    active_valid<=0;transaction<=0;pending<=0;loading<=0;pending_ready<=0;page_timer<=0;
    if(pending&&pending_ack)reply(2,8'h33,pending_request);
   end else begin
    if(pending&&(!pending_matches||pending_timer>=COMMIT_TIMEOUT_CYCLES-1))begin
     pending<=0;loading<=0;pending_ready<=0;
     if(pending_ack)reply(pending_matches?8'd3:8'd2,8'h33,pending_request);
    end else if(loading)begin
     if(read_wait)read_wait<=0;
     else begin
      pending_boxes[read_index]<=read_data;
      if({1'b0,read_index}+4'd1==pending_rows)begin loading<=0;pending_ready<=1;end
      else begin read_index<=read_index+1'b1;read_wait<=1;end
     end
    end
    if(publish)begin
     active_valid<=1;active_bank<=pending_bank;active_count<=pending_count;active_rows<=pending_rows;
     active_page<=pending_page;active_pages<=pending_pages;active_frame<=pending_frame;active_crc<=pending_crc;
     active_reference<=pending_reference;pending<=0;pending_ready<=0;page_timer<=0;
     for(i=0;i<8;i=i+1)active_boxes[i]<=pending_boxes[i];
     if(pending_ack)begin
      reply(0,8'h33,pending_request);response_committed_count_o<={6'b0,pending_count};
      response_committed_flags_o<={5'b0,locked_i,part_mode_i,1'b1};response_pending_flags_o<=0;
     end
    end else if(active_valid&&active_matches&&active_pages>1&&!transaction&&!pending&&!accepted)begin
     if(page_timer==PAGE_CYCLES-1)begin
      page_timer<=0;pending<=1;pending_ready<=0;pending_ack<=0;pending_bank<=active_bank;
      pending_frame<=active_frame;pending_crc<=active_crc;pending_reference<=active_reference;
      pending_count<=active_count;pending_pages<=active_pages;
      pending_page<=({1'b0,active_page}+7'd1<active_pages)?active_page+6'd1:6'd0;
      pending_rows<=page_rows(active_count,({1'b0,active_page}+7'd1<active_pages)?active_page+6'd1:6'd0);
      loading<=1;read_wait<=1;read_index<=0;
     end else page_timer<=page_timer+1'b1;
    end
   end
   if(accepted)begin
    if(cmd_request_i==0)reply(1,cmd_opcode_i,cmd_request_i);
    else if(cmd_opcode_i==8'h34)begin
     transaction<=0;active_valid<=0;pending<=0;loading<=0;page_timer<=0;reply(0,cmd_opcode_i,cmd_request_i);
    end else if(cmd_opcode_i==8'h35)reply(0,cmd_opcode_i,cmd_request_i);
    else if(!allowed)reply(2,cmd_opcode_i,cmd_request_i);
    else case(cmd_opcode_i)
     // 0x36 is the internal detector streaming BEGIN. It reserves existing
     // back-bank capacity; actual count is checked on COMMIT after extraction.
     // Legacy 0x30 and UART packet formats are unchanged.
     8'h30,8'h36:if(cmd_flags_i!=0||(cmd_opcode_i==8'h36&&cmd_data_i!=0))reply(1,cmd_opcode_i,cmd_request_i);
      else if(cmd_data_i>512)reply(4,cmd_opcode_i,cmd_request_i);
      else begin
       transaction<=1;transaction_id<=cmd_request_i;streaming<=cmd_opcode_i==8'h36;
       expected_count<=cmd_opcode_i==8'h36 ? 10'd512 : cmd_data_i[9:0];written<=0;
       meta_index<=0;meta<=0;lane<=0;word_count<=0;back_bank<=!active_bank;
       pending<=0;pending_ready<=0;loading<=0;page_timer<=0;
       reply(0,cmd_opcode_i,cmd_request_i);response_counter_o<=0;response_pending_count_o<=0;
      end
     8'h31:if(!owned||meta_index>=6||cmd_flags_i!={5'b0,meta_index})reply(1,cmd_opcode_i,cmd_request_i);
      else begin
       meta[meta_index*16+:16]<=cmd_data_i;meta_index<=meta_index+1'b1;word_count<=word_count+1'b1;
       reply(0,cmd_opcode_i,cmd_request_i);response_counter_o<=word_count+1'b1;
      end
     8'h32:if(!owned||meta_index!=6||cmd_flags_i!={6'b0,lane}||written>=expected_count||
         (lane==0&&cmd_data_i>=1024)||(lane==1&&cmd_data_i>=600)||
         (lane==2&&(cmd_data_i<=partial_box[9:0]||cmd_data_i>1024))||
         (lane==3&&(cmd_data_i<=partial_box[19:10]||cmd_data_i>600)))reply(1,cmd_opcode_i,cmd_request_i);
      else begin
       if(lane==0)partial_box[9:0]<=cmd_data_i[9:0];
       if(lane==1)partial_box[19:10]<=cmd_data_i[9:0];
       if(lane==2)partial_box[29:20]<=end_minus_one;
       if(lane==3)written<=written+1'b1;
       lane<=lane+1'b1;word_count<=word_count+1'b1;reply(0,cmd_opcode_i,cmd_request_i);
       response_counter_o<=word_count+1'b1;if(lane==3)response_pending_count_o<={6'b0,written+10'd1};
      end
     8'h33:if(!owned||meta_index!=6||lane!=0||cmd_flags_i!=0||
        (streaming ? cmd_data_i!={6'd0,written} : written!=expected_count||cmd_data_i!=0))
        reply(1,cmd_opcode_i,cmd_request_i);
      else if(meta[31:0]!=locked_frame_i||meta[63:32]!=locked_crc_i||meta[95:64]==0)
        reply(2,cmd_opcode_i,cmd_request_i);
      else begin
       transaction<=0;pending<=1;pending_ready<=commit_count==0;pending_ack<=1;pending_bank<=back_bank;
       pending_frame<=meta[31:0];pending_crc<=meta[63:32];pending_reference<=meta[95:64];pending_request<=cmd_request_i;
       pending_count<=commit_count;pending_page<=0;pending_pages<=(commit_count+10'd7)>>3;
       pending_rows<=page_rows(commit_count,0);loading<=commit_count!=0;read_wait<=1;read_index<=0;
       for(i=0;i<8;i=i+1)pending_boxes[i]<=0;
      end
     default:reply(1,cmd_opcode_i,cmd_request_i);
    endcase
   end
  end
 end
endmodule
