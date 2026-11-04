`timescale 1ns/1ps
// Autonomous coarse search: 72 angles * 2 image-derived anchors * 9 scales * 25 translations.
// One best record per angle in reused scratch, then greedy 20-degree NMS.
// Outputs are seeds for native refinement, not accepted geometry/defects.
module part_coarse_search #(parameter integer MAX_CANDIDATES=32)(
 input wire clk_i,rst_i,abort_i,start_i,output wire ready_o,busy_o,
 input wire[11:0] ref_count_i,obs_count_i,
 input wire[15:0] ref_cx_q8_i,ref_cy_q8_i,obs_cx_q8_i,obs_cy_q8_i,
 input wire[7:0] cap_i,input wire[15:0] ref_distance_base_i,obs_distance_base_i,record_base_i,
 output wire done_o,result_valid_o,input wire out_ready_i,output reg[3:0] error_o,
 output wire[15:0] mem_addr_o,output wire mem_we_o,output wire[7:0] mem_data_o,input wire[7:0] mem_data_i,
 output wire queue_read_o,output wire[11:0] queue_addr_o,input wire[15:0] queue_data_i,
 output wire candidate_valid_o,input wire candidate_ready_i,
 output wire[6:0] angle_index_o,output wire[3:0] scale_index_o,
 output wire signed[15:0] dx_q8_o,dy_q8_o,a_q14_o,b_q14_o,ia_q14_o,ib_q14_o,
 output wire anchor_o,output wire[31:0] cost_o,output reg[5:0] candidate_count_o,output reg candidates_truncated_o,
 output reg[14:0] poses_evaluated_o,output reg[31:0] cycles_o,
 output wire sc_start_o,sc_accept_o,output wire[95:0]sc_pose_o,
 input wire sc_ready_i,sc_valid_i,input wire[31:0]sc_cost_i,input wire[3:0]sc_error_i,
 input wire[15:0]sc_gray_addr_i,input wire[11:0]sc_queue_addr_i

);
 localparam IDLE=0,COEFF_START=1,COEFF_WAIT=2,SCORE_START=3,SCORE_WAIT=4,ADVANCE=5,
 STORE=6,NMS_INIT=7,READ_REQ=8,READ_WAIT=9,PICK=10,OUT_COEFF_START=11,OUT_COEFF_WAIT=12,
 OUTPUT_CANDIDATE=13,MARK=14,RESULT=15;
 reg[3:0] state;
 reg[6:0] angle,scan_angle,pick_angle;reg[3:0] scale,best_scale,pick_scale;
 reg[2:0] dx,dy,best_dx,best_dy,pick_dx,pick_dy,byte_index,mark_index;
 reg[31:0] best_cost,pick_cost;
 reg search_anchor,best_anchor,pick_anchor;
 reg[63:0] record;
 reg[15:0] record_base;
 wire coeff_ready,coeff_valid;
 wire signed[15:0] ca,cb,cia,cib;
 wire out_phase=state==OUT_COEFF_START||state==OUT_COEFF_WAIT||state==OUTPUT_CANDIDATE;
 wire signed[15:0] search_dx=($signed({1'b0,dx})-16'sd2)*16'sd512;
 wire signed[15:0] search_dy=($signed({1'b0,dy})-16'sd2)*16'sd512;
 wire score_ready,score_busy,score_valid,score_done,gray_read;
 wire[31:0] score_cost;wire[3:0] score_error;wire[15:0] gray_addr;
 // Cost, scale/dx/dy; final byte bit 0 = NMS blocked, bit 1 = anchor.
 reg[7:0] record_byte;
 always@*begin case(byte_index)
  0:record_byte=best_cost[7:0];1:record_byte=best_cost[15:8];2:record_byte=best_cost[23:16];3:record_byte=best_cost[31:24];
  4:record_byte={4'd0,best_scale};5:record_byte={5'd0,best_dx};6:record_byte={5'd0,best_dy};7:record_byte={6'd0,best_anchor,1'b0};
 endcase end
 // Bytes are always consumed 0..7, so a fixed byte shift replaces two barrel shifters.
 wire[63:0] received={mem_data_i,record[63:8]};
 wire signed[8:0] marked=$signed({2'd0,pick_angle})+$signed({6'd0,mark_index})-9'sd3;
 wire[6:0] mark_angle=marked<0?marked+9'sd72:(marked>=72?marked-9'sd72:marked);
 assign queue_read_o=state==SCORE_WAIT;
 assign ready_o=state==IDLE&&!rst_i&&!abort_i;
 assign busy_o=state!=IDLE;
 assign result_valid_o=state==RESULT&&!rst_i&&!abort_i;
 assign done_o=result_valid_o;
 assign candidate_valid_o=state==OUTPUT_CANDIDATE&&!rst_i&&!abort_i;
 assign anchor_o=out_phase?pick_anchor:search_anchor;
 assign angle_index_o=pick_angle;assign scale_index_o=pick_scale;
 assign dx_q8_o=($signed({1'b0,pick_dx})-16'sd2)*16'sd512;
 assign dy_q8_o=($signed({1'b0,pick_dy})-16'sd2)*16'sd512;
 assign a_q14_o=ca;assign b_q14_o=cb;assign ia_q14_o=cia;assign ib_q14_o=cib;assign cost_o=pick_cost;
 assign mem_addr_o=state==SCORE_START||state==SCORE_WAIT?gray_addr:
                    state==MARK?record_base+{mark_angle,3'b0}+16'd7:
                    record_base+{(state==READ_REQ||state==READ_WAIT?scan_angle:angle),3'b0}+byte_index;
 assign mem_we_o=(state==STORE||state==MARK)&&!rst_i&&!abort_i;
 assign mem_data_o=state==MARK?8'd1:record_byte;
 part_coarse_coeff coefficients(.clk_i(clk_i),.rst_i(rst_i),.abort_i(abort_i),
  .start_i(state==COEFF_START||state==OUT_COEFF_START),.ready_o(coeff_ready),
  .angle_i(out_phase?pick_angle:angle),.scale_i(out_phase?pick_scale:scale),
  .valid_o(coeff_valid),.out_ready_i(state==COEFF_WAIT||state==OUT_COEFF_WAIT),
  .a_o(ca),.b_o(cb),.ia_o(cia),.ib_o(cib));
 assign sc_start_o=state==SCORE_START;
 assign sc_accept_o=state==SCORE_WAIT;assign sc_pose_o={ca,cb,cia,cib,search_dx,search_dy};
 assign score_ready=sc_ready_i;assign score_valid=sc_valid_i;assign score_cost=sc_cost_i;assign score_error=sc_error_i;
 assign queue_addr_o=sc_queue_addr_i;assign gray_addr=sc_gray_addr_i;

 always@(posedge clk_i)begin
  if(rst_i)begin
   state<=IDLE;angle<=0;scale<=0;dx<=0;dy<=0;byte_index<=0;mark_index<=0;scan_angle<=0;record<=0;
   best_cost<=32'hffffffff;best_scale<=0;best_dx<=0;best_dy<=0;pick_cost<=32'hffffffff;pick_angle<=0;pick_scale<=0;pick_dx<=0;pick_dy<=0;
   search_anchor<=0;best_anchor<=0;pick_anchor<=0;
   record_base<=0;error_o<=0;candidate_count_o<=0;candidates_truncated_o<=0;poses_evaluated_o<=0;cycles_o<=0;
  end else if(abort_i)begin state<=IDLE;error_o<=3;end
  else begin
   if(state!=IDLE&&state!=RESULT)cycles_o<=cycles_o+1'b1;
   case(state)
    IDLE:if(start_i&&ready_o)begin
     angle<=0;scale<=0;dx<=0;dy<=0;best_cost<=32'hffffffff;record_base<=record_base_i;search_anchor<=0;best_anchor<=0;pick_anchor<=0;
     error_o<=0;candidate_count_o<=0;candidates_truncated_o<=0;poses_evaluated_o<=0;cycles_o<=0;
     if(ref_count_i==0||obs_count_i==0||ref_count_i>2048||obs_count_i>2048||cap_i==0||
        record_base_i>38400-576||MAX_CANDIDATES<1||MAX_CANDIDATES>32)begin error_o<=1;state<=RESULT;end
     else state<=COEFF_START;
    end
    COEFF_START:if(coeff_ready)state<=COEFF_WAIT;
    COEFF_WAIT:if(coeff_valid)state<=SCORE_START;
    SCORE_START:if(score_ready)state<=SCORE_WAIT;
    SCORE_WAIT:if(score_valid)begin
     if(score_error!=0)begin error_o<=2;state<=RESULT;end
     else begin
      poses_evaluated_o<=poses_evaluated_o+1'b1;
      if(score_cost<best_cost)begin best_cost<=score_cost;best_scale<=scale;best_dx<=dx;best_dy<=dy;best_anchor<=search_anchor;end
      state<=ADVANCE;
     end
    end
    ADVANCE:begin
     if(dx<4)begin dx<=dx+1'b1;state<=SCORE_START;end
     else begin dx<=0;
      if(dy<4)begin dy<=dy+1'b1;state<=SCORE_START;end
      else begin dy<=0;
       if(scale<8)begin scale<=scale+1'b1;state<=COEFF_START;end
       else begin scale<=0;
        if(!search_anchor)begin search_anchor<=1;state<=COEFF_START;end
        else begin search_anchor<=0;byte_index<=0;state<=STORE;end
       end
      end
     end
    end
    STORE:if(byte_index==7)begin
     if(angle==71)state<=NMS_INIT;
     else begin angle<=angle+1'b1;best_cost<=32'hffffffff;state<=COEFF_START;end
    end else byte_index<=byte_index+1'b1;
    NMS_INIT:begin scan_angle<=0;byte_index<=0;record<=0;pick_cost<=32'hffffffff;state<=READ_REQ;end
    READ_REQ:state<=READ_WAIT;
    READ_WAIT:begin
     record<=received;
     if(byte_index==7)begin
      if(!received[56]&&received[31:0]<pick_cost)begin
       pick_cost<=received[31:0];pick_scale<=received[35:32];pick_dx<=received[42:40];pick_dy<=received[50:48];pick_angle<=scan_angle;pick_anchor<=received[57];
      end
      if(scan_angle==71)state<=PICK;
      else begin scan_angle<=scan_angle+1'b1;byte_index<=0;record<=0;state<=READ_REQ;end
     end else begin byte_index<=byte_index+1'b1;state<=READ_REQ;end
    end
    PICK:if(pick_cost==32'hffffffff)state<=RESULT;
     else if(candidate_count_o>=MAX_CANDIDATES)begin candidates_truncated_o<=1;state<=RESULT;end
     else state<=OUT_COEFF_START;
    OUT_COEFF_START:if(coeff_ready)state<=OUT_COEFF_WAIT;
    OUT_COEFF_WAIT:if(coeff_valid)state<=OUTPUT_CANDIDATE;
    OUTPUT_CANDIDATE:if(candidate_ready_i)begin candidate_count_o<=candidate_count_o+1'b1;mark_index<=0;state<=MARK;end
    MARK:begin if(mark_index==6)state<=NMS_INIT;else mark_index<=mark_index+1'b1;end
    RESULT:if(out_ready_i)state<=IDLE;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
