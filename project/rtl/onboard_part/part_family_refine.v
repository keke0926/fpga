`timescale 1ns/1ps
// One coarse pose family -> locally refined seed. Native weak features unchanged.
// Caller holds maps, point lists, centers/counts and lease stable through busy.
module part_family_refine(
 input wire clk_i,rst_i,abort_i,start_i,output wire ready_o,busy_o,
 input wire[8:0]angle_i,scale_q8_i,input wire signed[15:0]dx_q8_i,dy_q8_i,
 input wire[11:0]ref_count_i,obs_count_i,
 input wire[15:0]ref_cx_q8_i,ref_cy_q8_i,obs_cx_q8_i,obs_cy_q8_i,
 output wire[15:0]gray_addr_o,input wire[7:0]gray_data_i,
 output wire[11:0]queue_addr_o,input wire[15:0]queue_data_i,
 output wire result_valid_o,input wire out_ready_i,output reg[3:0]error_o,
 output reg[8:0]angle_o,scale_q8_o,output reg signed[15:0]dx_q8_o,dy_q8_o,
 output wire signed[17:0]c_q16_o,d_q16_o,
 output reg[31:0]cost_o,output reg[7:0]evaluations_o,
 output wire sc_start_o,sc_accept_o,output wire[95:0]sc_pose_o,
 input wire sc_ready_i,sc_valid_i,input wire[31:0]sc_cost_i,input wire[3:0]sc_error_i,
 input wire[15:0]sc_gray_addr_i,input wire[11:0]sc_queue_addr_i

);
 localparam IDLE=0,CONFIG=1,COEFF_START=2,COEFF_WAIT=3,SCORE_START=4,SCORE_WAIT=5,
 ADVANCE=6,OUTPUT_COEFF_START=7,OUTPUT_COEFF_WAIT=8,RESULT=9;
 reg[3:0]state;reg initial_seed;
 reg[8:0]seed_angle,seed_scale,eval_angle,eval_scale;
 reg signed[15:0]seed_dx,seed_dy,eval_dx,eval_dy;
 reg[2:0]ai;reg[1:0]si,xi,yi;
 wire signed[10:0]trial_angle=$signed({2'b0,seed_angle})+($signed({1'b0,ai})-11'sd2)*11'sd2;
 wire signed[10:0]trial_scale=$signed({2'b0,seed_scale})+($signed({1'b0,si})-11'sd1)*11'sd8;
 wire coeff_ready,coeff_valid,score_ready,score_valid;
 wire signed[15:0]a,b,ia,ib;wire[3:0]score_error;wire[31:0]score_cost;
 wire output_phase=state==OUTPUT_COEFF_START||state==OUTPUT_COEFF_WAIT||state==RESULT;
 assign ready_o=state==IDLE&&!rst_i&&!abort_i;assign busy_o=state!=IDLE;
 assign result_valid_o=state==RESULT&&!rst_i&&!abort_i;
 part_refine_coeff coeff(.clk_i(clk_i),.rst_i(rst_i),.abort_i(abort_i),
 .start_i(state==COEFF_START||state==OUTPUT_COEFF_START),.ready_o(coeff_ready),
 .angle_i(output_phase?angle_o:eval_angle),.scale_q8_i(output_phase?scale_q8_o:eval_scale),
 .valid_o(coeff_valid),.out_ready_i(state==COEFF_WAIT||state==OUTPUT_COEFF_WAIT),
 .a_o(a),.b_o(b),.ia_o(ia),.ib_o(ib),.c_o(c_q16_o),.d_o(d_q16_o));
 assign sc_start_o=state==SCORE_START;
 assign sc_accept_o=state==SCORE_WAIT;assign sc_pose_o={a,b,ia,ib,eval_dx,eval_dy};
 assign score_ready=sc_ready_i;assign score_valid=sc_valid_i;assign score_cost=sc_cost_i;assign score_error=sc_error_i;
 assign queue_addr_o=sc_queue_addr_i;assign gray_addr_o=sc_gray_addr_i;

 always@(posedge clk_i)begin
  if(rst_i||abort_i)begin
   state<=IDLE;error_o<=0;initial_seed<=1;seed_angle<=0;seed_scale<=0;seed_dx<=0;seed_dy<=0;
   eval_angle<=0;eval_scale<=0;eval_dx<=0;eval_dy<=0;ai<=0;si<=0;xi<=0;yi<=0;
   angle_o<=0;scale_q8_o<=0;dx_q8_o<=0;dy_q8_o<=0;cost_o<=32'hffffffff;evaluations_o<=0;
  end else case(state)
   IDLE:if(start_i)begin
    seed_angle<=angle_i;seed_scale<=scale_q8_i;seed_dx<=dx_q8_i;seed_dy<=dy_q8_i;
    initial_seed<=1;ai<=0;si<=0;xi<=0;yi<=0;cost_o<=32'hffffffff;evaluations_o<=0;error_o<=0;
    if(angle_i>=360||scale_q8_i<192||scale_q8_i>320||scale_q8_i[2:0]!=0||ref_count_i==0||obs_count_i==0||ref_count_i>2048||obs_count_i>2048)begin error_o<=1;state<=RESULT;end
    else state<=CONFIG;
   end
   CONFIG:begin
    if(initial_seed)begin eval_angle<=seed_angle;eval_scale<=seed_scale;eval_dx<=seed_dx;eval_dy<=seed_dy;state<=COEFF_START;end
    else if(trial_scale<192||trial_scale>320)state<=ADVANCE;
    else begin
     eval_angle<=trial_angle<0?trial_angle+11'sd360:trial_angle>=360?trial_angle-11'sd360:trial_angle;
     eval_scale<=trial_scale;eval_dx<=seed_dx+($signed({1'b0,xi})-16'sd1)*16'sd256;
     eval_dy<=seed_dy+($signed({1'b0,yi})-16'sd1)*16'sd256;state<=COEFF_START;
    end
   end
   COEFF_START:if(coeff_ready)state<=COEFF_WAIT;
   COEFF_WAIT:if(coeff_valid)state<=SCORE_START;
   SCORE_START:if(score_ready)state<=SCORE_WAIT;
   SCORE_WAIT:if(score_valid)begin
    if(score_error!=0)begin error_o<=2;state<=RESULT;end
    else begin
     evaluations_o<=evaluations_o+1'b1;
     if(score_cost<cost_o)begin cost_o<=score_cost;angle_o<=eval_angle;scale_q8_o<=eval_scale;dx_q8_o<=eval_dx;dy_q8_o<=eval_dy;end
     state<=ADVANCE;
    end
   end
   ADVANCE:begin
    if(initial_seed)begin initial_seed<=0;state<=CONFIG;end
    else if(xi<2)begin xi<=xi+1'b1;state<=CONFIG;end
    else begin xi<=0;
     if(yi<2)begin yi<=yi+1'b1;state<=CONFIG;end
     else begin yi<=0;
      if(si<2)begin si<=si+1'b1;state<=CONFIG;end
      else begin si<=0;if(ai<4)begin ai<=ai+1'b1;state<=CONFIG;end else state<=OUTPUT_COEFF_START;end
     end
    end
   end
   OUTPUT_COEFF_START:if(coeff_ready)state<=OUTPUT_COEFF_WAIT;
   OUTPUT_COEFF_WAIT:if(coeff_valid)state<=RESULT;
   RESULT:if(out_ready_i)state<=IDLE;
   default:state<=IDLE;
  endcase
 end
endmodule
