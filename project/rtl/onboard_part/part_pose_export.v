`timescale 1ns/1ps
// Two-pass admission and compact pose export before scratch is reused.
// Evidence gates only reject insufficient overlap/signal; accepted poses are
// alternative registration hypotheses, not proof of a good or defective part.
module part_pose_export #(parameter POSE_BASE=32'h00f70000)(
 input wire clk_i,rst_i,abort_i,start_i,output wire ready_o,busy_o,valid_o,input wire out_ready_i,
 input wire[5:0]record_count_i,input wire[31:0]task_id_i,frame_id_i,reference_id_i,config_epoch_i,
 output reg[3:0]error_o,output reg[5:0]count_o,
 output wire[15:0]gray_addr_o,input wire[7:0]gray_data_i,
 output wire req_valid_o,input wire req_ready_i,output wire req_write_o,
 output wire[31:0]req_addr_o,output wire[127:0]req_data_o,
 input wire rsp_valid_i,output wire rsp_ready_o,input wire rsp_error_i
);
 localparam IDLE=0,READ=1,TAKE=2,WORD=3,CHECK=4,NEXT=5,SEND0=6,WAIT0=7,SEND1=8,WAIT1=9,RESULT=10,DRAIN=11;
 reg[3:0]state;reg second,pending,eligible;reg[5:0]record_index;reg[4:0]word_index;reg[1:0]byte_index;
 reg[31:0]word_data,cost,best;reg[127:0]pose;reg[16:0]invscale;
 reg[19:0]attempted,joint;reg[31:0]admitted;
 wire[35:0]cost10=({4'd0,cost}<<3)+({4'd0,cost}<<1);
 wire[35:0]best11=({4'd0,best}<<3)+({4'd0,best}<<1)+{4'd0,best};
 assign ready_o=state==IDLE&&!rst_i&&!abort_i;
 assign busy_o=state!=IDLE&&state!=RESULT;
 assign valid_o=state==RESULT&&!rst_i&&!abort_i;
 assign gray_addr_o=16'd1024+{4'd0,record_index[4:0],word_index,byte_index};
 assign req_valid_o=(state==SEND0||state==SEND1)&&!pending&&!rst_i&&!abort_i;
 assign req_write_o=1'b1;
 assign req_addr_o=POSE_BASE+{21'd0,count_o,5'd0}+(state==SEND1?32'd16:32'd0);
 assign req_data_o=state==SEND1?{15'd0,invscale,96'd0}:pose;
 assign rsp_ready_o=pending;
 always@(posedge clk_i)begin
  if(rst_i)begin
   state<=IDLE;second<=0;pending<=0;eligible<=0;record_index<=0;word_index<=0;byte_index<=0;
   word_data<=0;cost<=0;best<=32'hffffffff;pose<=0;invscale<=0;attempted<=0;joint<=0;admitted<=0;error_o<=0;count_o<=0;
  end else begin
   if(req_valid_o&&req_ready_i)pending<=1;
   if(rsp_valid_i&&rsp_ready_o)pending<=0;
   if(abort_i)state<=pending&&!(rsp_valid_i&&rsp_ready_o)?DRAIN:IDLE;
   else if(rsp_valid_i&&rsp_ready_o&&rsp_error_i)begin error_o<=2;state<=RESULT;end
   else case(state)
    IDLE:if(start_i)begin
     second<=0;record_index<=0;word_index<=0;byte_index<=0;eligible<=1;best<=32'hffffffff;admitted<=0;count_o<=0;error_o<=0;
     if(record_count_i==0||record_count_i>32)begin error_o<=5;state<=RESULT;end else state<=READ;
    end
    READ:state<=TAKE;
    TAKE:begin word_data<={gray_data_i,word_data[31:8]};if(byte_index==3)state<=WORD;else begin byte_index<=byte_index+1'b1;state<=READ;end end
    WORD:begin
     case(word_index)
      0:eligible<=eligible&&word_data[31:24]==1&&word_data[3:0]==0;
      1:eligible<=eligible&&word_data==task_id_i;
      2:eligible<=eligible&&word_data==frame_id_i;
      3:eligible<=eligible&&word_data==reference_id_i;
      4:eligible<=eligible&&word_data==config_epoch_i;
      10:pose[127:96]<=word_data;11:pose[95:64]<=word_data;
      12:pose[63:32]<=word_data;13:pose[31:0]<=word_data;
      14:cost<=word_data;
      15:begin attempted<=word_data[19:0];eligible<=eligible&&word_data!=0;end
      16:eligible<=eligible&&{word_data[19:0],1'b0}>={1'b0,attempted};
      17,19:eligible<=eligible&&word_data>=16;
      21:begin joint<=word_data[19:0];eligible<=eligible&&word_data>=8;end
      22:eligible<=eligible&&word_data>=2&&{word_data[19:0],3'b0}>={3'd0,joint};
      29:begin invscale<=word_data[25:9];eligible<=eligible&&word_data[8]&&word_data[25:9]>=52429&&word_data[25:9]<=87381;end
     endcase
     byte_index<=0;
     if(word_index==29)state<=CHECK;else begin word_index<=word_index+1'b1;state<=READ;end
    end
    CHECK:begin
     if(!second)begin admitted[record_index]<=eligible;if(eligible&&cost<best)best<=cost;state<=NEXT;end
     else state<=admitted[record_index]&&eligible&&cost10<=best11?SEND0:NEXT;
    end
    NEXT:begin
     word_index<=0;byte_index<=0;eligible<=1;
     if(record_index+1==record_count_i)begin
      if(second)begin if(count_o==0)error_o<=5;state<=RESULT;end
      else if(admitted==0)begin error_o<=5;state<=RESULT;end
      else begin second<=1;record_index<=0;state<=READ;end
     end else begin record_index<=record_index+1'b1;state<=READ;end
    end
    SEND0:if(req_valid_o&&req_ready_i)state<=WAIT0;
    WAIT0:if(rsp_valid_i&&pending)state<=SEND1;
    SEND1:if(req_valid_o&&req_ready_i)state<=WAIT1;
    WAIT1:if(rsp_valid_i&&pending)begin count_o<=count_o+1'b1;state<=NEXT;end
    RESULT:if(out_ready_i)state<=IDLE;
    DRAIN:if(!pending||rsp_valid_i)state<=IDLE;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
