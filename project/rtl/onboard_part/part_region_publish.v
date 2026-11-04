`timescale 1ns/1ps
// Local connected-region stream -> existing atomic same-frame Chinese result
// store. No private RAM, UART or host acknowledgement. All boxes need review.
// STATIC_CONTEXT reuses parent-held task fields through completion/drain.
// The region ready/valid contract keeps its coordinates stable through ACK.
module part_region_publish #(parameter STATIC_CONTEXT=1)(
 input wire clk_i,rst_i,abort_i,start_i,output wire ready_o,busy_o,
 input wire[31:0]task_id_i,frame_id_i,frame_crc_i,reference_id_i,
 input wire region_valid_i,output wire region_ready_o,region_abort_o,
 input wire[8:0]x0_i,x1_i,input wire[7:0]y0_i,y1_i,
 input wire regions_done_i,input wire[11:0]regions_count_i,input wire[3:0]regions_error_i,
 output wire cmd_valid_o,input wire cmd_ready_i,
 output reg[7:0]cmd_opcode_o,cmd_flags_o,output reg[15:0]cmd_data_o,output wire[31:0]cmd_request_o,
 input wire response_valid_i,output wire response_ready_o,input wire[7:0]response_status_i,
 output wire result_valid_o,input wire out_ready_i,output reg[3:0]error_o,output reg[9:0]count_o
);
 localparam IDLE=0,BEGIN_SEND=1,BEGIN_ACK=2,META_SEND=3,META_ACK=4,
  REGION_WAIT=5,BOX_SEND=6,BOX_ACK=7,COMMIT_SEND=8,COMMIT_ACK=9,
  CLEAR_SEND=10,CLEAR_ACK=11,RESULT=12;
 reg[3:0]state;reg[2:0]meta_index;reg[1:0]lane;
 reg[31:0]task_held;reg[95:0]meta_held;
 wire[31:0]task_id=STATIC_CONTEXT?task_id_i:task_held;
 wire[95:0]meta=STATIC_CONTEXT?{reference_id_i,frame_crc_i,frame_id_i}:meta_held;
 wire[33:0]box_data={y1_i,x1_i,y0_i,x0_i};
 reg completed;reg[11:0]completed_count;reg[3:0]completed_error;
 wire context_lost=!STATIC_CONTEXT&&(task_id_i!=task_id||{reference_id_i,frame_crc_i,frame_id_i}!=meta);
 wire cancelling=abort_i||(state!=IDLE&&context_lost);
 wire sending=state==BEGIN_SEND||state==META_SEND||state==BOX_SEND||state==COMMIT_SEND||state==CLEAR_SEND;
 wire ack=state==BEGIN_ACK||state==META_ACK||state==BOX_ACK||state==COMMIT_ACK||state==CLEAR_ACK;
 assign ready_o=state==IDLE&&!rst_i&&!abort_i;
 assign busy_o=state!=IDLE;
 assign cmd_valid_o=sending&&!rst_i&&(!cancelling||state==CLEAR_SEND);
 assign cmd_request_o=task_id;
 // Cancellation can arrive with an earlier command ACK pending. Drain it
 // while waiting to send CLEAR, otherwise response_valid blocks cmd_ready.
 assign response_ready_o=(ack||state==CLEAR_SEND)&&!rst_i;
 assign region_ready_o=state==BOX_ACK&&lane==3&&response_valid_i&&response_status_i==0&&!cancelling&&!rst_i;
 assign region_abort_o=state==CLEAR_SEND||state==CLEAR_ACK||(state==RESULT&&error_o!=0);
 assign result_valid_o=state==RESULT&&!rst_i&&!cancelling;
 always@*begin
  cmd_opcode_o=0;cmd_flags_o=0;cmd_data_o=0;
  case(state)
   BEGIN_SEND:cmd_opcode_o=8'h36;
   META_SEND:begin cmd_opcode_o=8'h31;cmd_flags_o={5'd0,meta_index};cmd_data_o=meta[meta_index*16+:16];end
   BOX_SEND:begin
    cmd_opcode_o=8'h32;cmd_flags_o={6'd0,lane};
    case(lane)
     0:cmd_data_o={5'd0,box_data[8:0],2'b00};
     1:cmd_data_o={6'd0,box_data[16:9],2'b00};
     2:cmd_data_o={5'd0,box_data[25:17],2'b00};
     3:cmd_data_o={6'd0,box_data[33:26],2'b00};
    endcase
   end
   COMMIT_SEND:begin cmd_opcode_o=8'h33;cmd_data_o={6'd0,count_o};end
   CLEAR_SEND:cmd_opcode_o=8'h34;
  endcase
 end
 always@(posedge clk_i)begin
  if(rst_i)begin
   state<=IDLE;task_held<=0;meta_held<=0;meta_index<=0;lane<=0;
   completed<=0;completed_count<=0;completed_error<=0;error_o<=0;count_o<=0;
  end else begin
   if(regions_done_i&&state!=IDLE)begin completed<=1;completed_count<=regions_count_i;completed_error<=regions_error_i;end
   if(state!=IDLE&&state!=RESULT&&state!=CLEAR_SEND&&state!=CLEAR_ACK&&cancelling)begin
    state<=CLEAR_SEND;error_o<=3;
   end else case(state)
    IDLE:if(start_i&&ready_o)begin
     task_held<=task_id_i;meta_held<={reference_id_i,frame_crc_i,frame_id_i};count_o<=0;error_o<=0;
     meta_index<=0;lane<=0;completed<=0;completed_count<=0;completed_error<=0;
     if(task_id_i==0||reference_id_i==0)begin error_o<=1;state<=RESULT;end else state<=BEGIN_SEND;
    end
    BEGIN_SEND:if(cmd_ready_i)state<=BEGIN_ACK;
    BEGIN_ACK:if(response_valid_i)begin
     if(response_status_i!=0)begin error_o<=2;state<=CLEAR_SEND;end else state<=META_SEND;
    end
    META_SEND:if(cmd_ready_i)state<=META_ACK;
    META_ACK:if(response_valid_i)begin
     if(response_status_i!=0)begin error_o<=2;state<=CLEAR_SEND;end
     else if(meta_index==5)state<=REGION_WAIT;
     else begin meta_index<=meta_index+1'b1;state<=META_SEND;end
    end
    REGION_WAIT:begin
     if(region_valid_i)begin
      if(count_o==512)begin error_o<=4;state<=CLEAR_SEND;end
      else begin lane<=0;state<=BOX_SEND;end
     end else if(completed)begin
      if(completed_error!=0||completed_count!={2'd0,count_o})begin error_o<=5;state<=CLEAR_SEND;end
      else state<=COMMIT_SEND;
     end
    end
    BOX_SEND:if(cmd_ready_i)state<=BOX_ACK;
    BOX_ACK:if(response_valid_i)begin
     if(response_status_i!=0)begin error_o<=2;state<=CLEAR_SEND;end
     else if(lane==3)begin count_o<=count_o+1'b1;state<=REGION_WAIT;end
     else begin lane<=lane+1'b1;state<=BOX_SEND;end
    end
    COMMIT_SEND:if(cmd_ready_i)state<=COMMIT_ACK;
    COMMIT_ACK:if(response_valid_i)begin
     if(response_status_i!=0)begin error_o<=2;state<=CLEAR_SEND;end else state<=RESULT;
    end
    CLEAR_SEND:if(cmd_ready_i)state<=CLEAR_ACK;
    CLEAR_ACK:if(response_valid_i)state<=cancelling?IDLE:RESULT;
    RESULT:if(out_ready_i||cancelling)state<=IDLE;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
