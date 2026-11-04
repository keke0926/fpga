`timescale 1ns/1ps
`include "capture_protocol_defs.vh"
module capture_stream_controller #(parameter BAUD=460800,WATCHDOG_CYCLES=144000000)(
 input wire clk,rst_n,cmd_valid_i,input wire[7:0] cmd_opcode_i,input wire[15:0] cmd_sequence_i,
 input wire[31:0] cmd_request_id_i,input wire protocol_error_i,record_ready_i,record_sent_i,
 output wire record_valid_o,output reg[7:0] record_kind_o,
 output reg[31:0] record_request_id_o,record_frame_id_o,
 output reg[15:0] record_sequence_o,record_first_row_o,record_row_count_o,record_payload_bytes_o,
 input wire[13:0] payload_addr_i,output wire[7:0] payload_byte_o,
 input wire lcd_vs_i,lcd_de_i,input wire[7:0] lcd_gray_i,
 input wire locked_i,hold_cancel_i,input wire[31:0] locked_frame_id_i,locked_crc32_i,
 input wire restored_i,freezer_error_i,input wire[31:0] freezer_error_code_i,
 output reg snapshot_req_o,resume_req_o,busy_rejected_o,output wire busy_o);
 localparam IDLE=0,FREEZE=1,SEND=2,SENT=3,ACK_LOCK=4,START_TILE=5,
   FILL=6,ACK_DATA=7,WAIT_RESTORE=8,FAIL_START=9,FAIL_WAIT=10,HELD_RESTORE=11,AUTO_RESTORE=12;
 reg hold_requested,held;
 reg[3:0] state,after_sent;
 reg source_kind,cancel_pending,terminal_cancel,terminal_restored,recovery_started,terminal_record;
 reg[15:0] block_number;reg[31:0] request_id,expected_crc,frame_counter,failure,watchdog;
 reg tile_start,tile_release,crc_clear;
 // Bounded, explicit replay window; HELLO must not erase capture history.
 reg[31:0] recent_requests[0:15];reg[3:0] recent_index;
 reg replayed;integer scan_index,reset_index;
 always @*begin
  replayed=0;
  for(scan_index=0;scan_index<16;scan_index=scan_index+1)
   if(recent_requests[scan_index]==cmd_request_id_i)replayed=1;
 end
 wire tile_ready,tile_error,tile_accepted;wire[7:0] tile_byte,tile_accepted_byte;
 wire[15:0] tile_bytes;wire[31:0] frame_crc;
 reg[10:0] pattern_x;reg[9:0] pattern_y;
 wire pattern_vs=pattern_y<600,pattern_de=pattern_vs&&pattern_x<1024;
 wire[7:0] pattern_gray=(pattern_x*3+pattern_y*5)&255;
 wire matched=cmd_valid_i&&cmd_request_id_i==request_id;
 assign record_valid_o=state==SEND;
 wire[15:0] block_first_row=block_number<<4;
 localparam[31:0] BAUD_WORD=BAUD;
 capture_tile_buffer tile(.clk(clk),.rst_n(rst_n),.start_i(tile_start),
   .first_row_i(block_first_row),.row_count_i(block_number==37?16'd8:16'd16),
   .in_vs_i(source_kind?pattern_vs:lcd_vs_i),.in_de_i(source_kind?pattern_de:lcd_de_i),
   .in_gray_i(source_kind?pattern_gray:lcd_gray_i),.release_i(tile_release),
   .ready_o(tile_ready),.payload_bytes_o(tile_bytes),.error_o(tile_error),
   .read_addr_i(payload_addr_i),.read_byte_o(tile_byte),
   .accepted_valid_o(tile_accepted),.accepted_byte_o(tile_accepted_byte));
 capture_crc32 crc(.clk(clk),.rst_n(rst_n),.clear_i(crc_clear),
   .valid_i(tile_accepted),.byte_i(tile_accepted_byte),.crc_o(frame_crc));
 reg[7:0] control_byte;
 assign payload_byte_o=record_kind_o==2?tile_byte:control_byte;
 function[7:0] control_payload;
   input[13:0] address;reg[95:0] payload;
   begin
    payload=0;
    case(record_kind_o)
     1:payload={16'b0,32'd614400,7'b0,source_kind,8'd1,16'd600,16'd1024};
     3:payload={32'd1,frame_crc,32'd614400};
     7:payload={32'd0,frame_crc,32'd614400};
     8:payload={32'd0,31'd0,terminal_restored,failure};
     4,5:payload={32'b0,31'b0,terminal_restored,failure};
     6:payload={BAUD_WORD,16'd16,8'd1,8'd1,16'd600,16'd1024};
    endcase
    control_payload=address<12?(payload>>(address*8)):0;
   end
 endfunction
 always @(posedge clk)control_byte<=control_payload(payload_addr_i);
 task control_record;
  input[7:0] kind;input[15:0] bytes;input[3:0] next_state;
  begin record_kind_o<=kind;terminal_record<=kind>=3;record_sequence_o<=16'hffff;record_first_row_o<=0;
   record_row_count_o<=0;record_payload_bytes_o<=bytes;after_sent<=next_state;state<=SEND;end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;after_sent<=IDLE;source_kind<=0;cancel_pending<=0;terminal_cancel<=0;terminal_restored<=0;
   hold_requested<=0;held<=0;
   recovery_started<=0;terminal_record<=0;
   recent_index<=0;for(reset_index=0;reset_index<16;reset_index=reset_index+1)recent_requests[reset_index]<=0;
   block_number<=0;request_id<=0;expected_crc<=0;frame_counter<=0;failure<=0;watchdog<=0;
   snapshot_req_o<=0;resume_req_o<=0;busy_rejected_o<=0;tile_start<=0;tile_release<=0;crc_clear<=0;
   record_kind_o<=0;record_request_id_o<=0;record_frame_id_o<=0;record_sequence_o<=16'hffff;
   record_first_row_o<=0;record_row_count_o<=0;record_payload_bytes_o<=0;pattern_x<=0;pattern_y<=0;
  end else begin
   snapshot_req_o<=0;resume_req_o<=0;busy_rejected_o<=0;tile_start<=0;tile_release<=0;crc_clear<=0;
   if(source_kind&&state!=IDLE)begin
    if(pattern_x==1025)begin pattern_x<=0;pattern_y<=pattern_y==600?0:pattern_y+1'b1;end
    else pattern_x<=pattern_x+1'b1;
   end else begin pattern_x<=0;pattern_y<=0;end
   if(state==IDLE)watchdog<=0;else watchdog<=watchdog+1'b1;
   if(record_sent_i)watchdog<=0;
   case(state)
    IDLE:if(held&&(hold_cancel_i||freezer_error_i))begin
     resume_req_o<=1;held<=0;state<=AUTO_RESTORE;terminal_record<=0;
    end else if(cmd_valid_i&&cmd_request_id_i!=0)begin
     if(cmd_opcode_i==7||(cmd_opcode_i==3&&held))begin
      request_id<=cmd_request_id_i;record_request_id_o<=cmd_request_id_i;
      record_frame_id_o<=locked_frame_id_i;held<=0;failure<=0;terminal_record<=0;
      if(locked_i)begin resume_req_o<=1;state<=HELD_RESTORE;end
      else begin terminal_restored<=restored_i;control_record(8,8,IDLE);end
     end else if((cmd_opcode_i==1||cmd_opcode_i==5||cmd_opcode_i==6)&&(replayed||locked_i||held))busy_rejected_o<=1;
     else if(cmd_opcode_i==1||cmd_opcode_i==5||cmd_opcode_i==4||cmd_opcode_i==6)begin
      if(cmd_opcode_i==1||cmd_opcode_i==5||cmd_opcode_i==6)begin
       recent_requests[recent_index]<=cmd_request_id_i;recent_index<=recent_index+1'b1;
      end
      request_id<=cmd_request_id_i;record_request_id_o<=cmd_request_id_i;
      record_frame_id_o<=0;block_number<=0;failure<=0;cancel_pending<=0;terminal_cancel<=0;recovery_started<=0;terminal_record<=0;
      source_kind<=cmd_opcode_i==5;hold_requested<=cmd_opcode_i==6;crc_clear<=1;tile_release<=1;terminal_restored<=0;
      if(cmd_opcode_i==4)control_record(6,12,IDLE);
      else if(cmd_opcode_i==5)begin
       frame_counter<=frame_counter+1'b1;record_frame_id_o<=frame_counter+1'b1;
       control_record(1,10,ACK_LOCK);
      end else begin snapshot_req_o<=1;state<=FREEZE;end
     end
    end
    FREEZE:if(locked_i)begin
     record_frame_id_o<=locked_frame_id_i;expected_crc<=locked_crc32_i;
     control_record(1,10,ACK_LOCK);watchdog<=0;
    end
    SEND:if(record_ready_i)state<=SENT;
    SENT:if(record_sent_i)begin
     if(cancel_pending)begin cancel_pending<=0;state<=FAIL_START;end else state<=after_sent;
    end
    ACK_LOCK:if(matched&&cmd_opcode_i==2&&cmd_sequence_i==16'hffff)begin state<=START_TILE;watchdog<=0;end
    START_TILE:begin tile_start<=1;state<=FILL;end
    FILL:begin
     if(tile_error)begin failure<=3;state<=FAIL_START;end
     else if(tile_ready)begin
      record_kind_o<=2;terminal_record<=0;record_sequence_o<=block_number;record_first_row_o<=block_number*16;
      record_row_count_o<=block_number==37?8:16;record_payload_bytes_o<=tile_bytes;
      after_sent<=ACK_DATA;state<=SEND;watchdog<=0;
     end
    end
    ACK_DATA:if(matched&&cmd_opcode_i==2&&cmd_sequence_i==block_number)begin
     tile_release<=1;watchdog<=0;
     if(block_number==37)begin
      if(!source_kind&&frame_crc!=expected_crc)begin failure<=6;state<=FAIL_START;end
      else if(source_kind)control_record(3,12,IDLE);
      else if(hold_requested)begin held<=1;control_record(7,12,IDLE);end
      else begin resume_req_o<=1;state<=WAIT_RESTORE;end
     end else begin block_number<=block_number+1'b1;state<=START_TILE;end
    end
    WAIT_RESTORE:if(restored_i&&!freezer_error_i)control_record(3,12,IDLE);
    HELD_RESTORE:if(restored_i||freezer_error_i)begin
     terminal_restored<=restored_i;failure<=freezer_error_i?freezer_error_code_i:0;control_record(8,8,IDLE);
    end
    AUTO_RESTORE:if(restored_i||freezer_error_i)state<=IDLE;
    FAIL_START:begin
     tile_release<=1;watchdog<=0;
     if(source_kind)begin terminal_restored<=1;control_record(terminal_cancel?5:4,8,IDLE);end
     else begin if(!recovery_started)resume_req_o<=1;recovery_started<=1;state<=FAIL_WAIT;end
    end
    FAIL_WAIT:if(restored_i||freezer_error_i)begin
     terminal_restored<=restored_i;control_record(terminal_cancel?5:4,8,IDLE);
    end
   endcase
   if(state!=IDLE&&cmd_valid_i&&(cmd_opcode_i==1||cmd_opcode_i==5||cmd_opcode_i==4||cmd_opcode_i==6||cmd_opcode_i==7))busy_rejected_o<=1;
   // Never cut a physical record halfway through; finish it before terminal
   // error/cancel. Wrong request IDs and stale ACKs do not reset the watchdog.
   if(state!=IDLE&&state!=FAIL_START&&state!=FAIL_WAIT&&state!=AUTO_RESTORE&&state!=HELD_RESTORE&&!terminal_record&&
      ((matched&&cmd_opcode_i==3)||protocol_error_i||
       (!source_kind&&freezer_error_i&&(state!=FREEZE||watchdog>2))))begin
    failure<=(matched&&cmd_opcode_i==3)?0:(protocol_error_i?2:freezer_error_code_i);
    terminal_cancel<=matched&&cmd_opcode_i==3;
    if(state==SEND||state==SENT)begin
     cancel_pending<=1;
     if(!source_kind&&!recovery_started)begin resume_req_o<=1;recovery_started<=1;end
    end else state<=FAIL_START;
   end
   if(state!=IDLE&&!terminal_record&&watchdog>=WATCHDOG_CYCLES-1)begin
    watchdog<=0;
    if(state==FAIL_WAIT)begin terminal_restored<=0;control_record(terminal_cancel?5:4,8,IDLE);end
    else if(state!=FAIL_START)begin
     failure<=7;terminal_cancel<=0;
     if(state==SEND||state==SENT)begin
      cancel_pending<=1;
      if(!source_kind&&!recovery_started)begin resume_req_o<=1;recovery_started<=1;end
     end else state<=FAIL_START;
    end
   end
  end
 end
 assign busy_o=state!=IDLE;
endmodule
