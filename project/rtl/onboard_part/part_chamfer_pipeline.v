`timescale 1ns/1ps
// One point per cycle, both directions, exact cap-2 Chamfer cost.
// Parent holds coefficients/counts/centers until result consumption. Shared
// scratch and queue have one synchronous read cycle, as in the serial scorer.
module part_chamfer_pipeline(
 input wire clk_i,rst_i,abort_i,start_i,
 output wire ready_o,busy_o,result_valid_o,input wire out_ready_i,
 input wire signed[15:0]a_q14_i,b_q14_i,ia_q14_i,ib_q14_i,dx_q8_i,dy_q8_i,
 input wire[15:0]ref_cx_q8_i,ref_cy_q8_i,obs_cx_q8_i,obs_cy_q8_i,
 input wire[11:0]ref_count_i,obs_count_i,
 output wire[11:0]queue_addr_o,input wire[15:0]queue_data_i,
 output wire[15:0]gray_addr_o,input wire[7:0]gray_data_i,
 output reg[31:0]cost_o,output reg[3:0]error_o
);
 localparam IDLE=0,RUN=1,RESULT=2;
 reg[1:0]state;
 reg issue,issue_reverse;
 reg[10:0]issue_index;
 reg[12:0]committed;
 reg v0,v1,v2,v3,v4,rev0,rev1,rev2,rev3,rev4,inside4;
 reg signed[17:0]px,py;
 reg signed[35:0]ax,by,bx,ay;
 reg signed[20:0]sample_x,sample_y;
 wire signed[17:0]point_x=$signed({3'd0,queue_data_i[6:0],8'd0});
 wire signed[17:0]point_y=$signed({3'd0,queue_data_i[13:7],8'd0});
 wire signed[17:0]source_x=$signed({2'd0,rev0?obs_cx_q8_i:ref_cx_q8_i});
 wire signed[17:0]source_y=$signed({2'd0,rev0?obs_cy_q8_i:ref_cy_q8_i});
 wire signed[17:0]delta_x=rev0?{{2{dx_q8_i[15]}},dx_q8_i}:18'sd0;
 wire signed[17:0]delta_y=rev0?{{2{dy_q8_i[15]}},dy_q8_i}:18'sd0;
 wire signed[17:0]ma=rev1?$signed({{2{ia_q14_i[15]}},ia_q14_i}):$signed({{2{a_q14_i[15]}},a_q14_i});
 wire signed[17:0]mb=rev1?$signed({{2{ib_q14_i[15]}},ib_q14_i}):$signed({{2{b_q14_i[15]}},b_q14_i});
 wire signed[36:0]mx=$signed({ax[35],ax})-$signed({by[35],by});
 wire signed[36:0]my=$signed({bx[35],bx})+$signed({ay[35],ay});
 wire signed[36:0]dest_x=$signed({21'd0,rev2?ref_cx_q8_i:obs_cx_q8_i});
 wire signed[36:0]dest_y=$signed({21'd0,rev2?ref_cy_q8_i:obs_cy_q8_i});
 wire signed[36:0]cor_x=rev2?37'sd0:{{21{dx_q8_i[15]}},dx_q8_i};
 wire signed[36:0]cor_y=rev2?37'sd0:{{21{dy_q8_i[15]}},dy_q8_i};
 wire signed[36:0]index_x=((mx>>>14)+dest_x+cor_x+37'sd128)>>>8;
 wire signed[36:0]index_y=((my>>>14)+dest_y+cor_y+37'sd128)>>>8;
 wire in_domain=sample_x>=0&&sample_x<128&&sample_y>=0&&sample_y<75;
 wire[1:0]distance_value=inside4?gray_data_i[1:0]:2'd2;
 wire[13:0]weight={2'd0,rev4?ref_count_i:obs_count_i};
 wire[13:0]increment=(distance_value[0]?weight:14'd0)+(distance_value[1]?{weight[12:0],1'b0}:14'd0);
 assign ready_o=state==IDLE&&!rst_i&&!abort_i;
 assign busy_o=state!=IDLE;
 assign result_valid_o=state==RESULT&&!rst_i&&!abort_i;
 assign queue_addr_o={issue_reverse,issue_index};
 assign gray_addr_o=(rev3?16'd19200:16'd28800)+(in_domain?{2'b0,sample_y[6:0],sample_x[6:0]}:16'd0);
 always@(posedge clk_i)begin
  if(rst_i||abort_i)begin
   state<=IDLE;issue<=0;issue_reverse<=0;issue_index<=0;committed<=0;cost_o<=0;error_o<=0;
   v0<=0;v1<=0;v2<=0;v3<=0;v4<=0;rev0<=0;rev1<=0;rev2<=0;rev3<=0;rev4<=0;inside4<=0;
   px<=0;py<=0;ax<=0;by<=0;bx<=0;ay<=0;sample_x<=0;sample_y<=0;
  end else begin
   v0<=state==RUN&&issue;v1<=v0;v2<=v1;v3<=v2;v4<=v3;
   if(state==RUN&&issue)begin
    rev0<=issue_reverse;
    if({1'b0,issue_index}+12'd1<(issue_reverse?obs_count_i:ref_count_i))issue_index<=issue_index+1'b1;
    else if(!issue_reverse&&obs_count_i!=0)begin issue_index<=0;issue_reverse<=1;end
    else issue<=0;
   end
   if(v0)begin px<=point_x-source_x-delta_x;py<=point_y-source_y-delta_y;rev1<=rev0;end
   if(v1)begin ax<=ma*px;by<=mb*py;bx<=mb*px;ay<=ma*py;rev2<=rev1;end
   if(v2)begin sample_x<=index_x[20:0];sample_y<=index_y[20:0];rev3<=rev2;end
   if(v3)begin inside4<=in_domain;rev4<=rev3;end
   if(state==RUN&&v4)begin
    cost_o<=cost_o+{18'd0,increment};committed<=committed+1'b1;
    if(committed+13'd1=={1'b0,ref_count_i}+{1'b0,obs_count_i})state<=RESULT;
   end
   case(state)
    IDLE:if(start_i)begin
     cost_o<=0;error_o<=0;committed<=0;issue_index<=0;issue_reverse<=ref_count_i==0;
     if(ref_count_i>2048||obs_count_i>2048)begin error_o<=1;state<=RESULT;end
     else if(ref_count_i==0&&obs_count_i==0)state<=RESULT;
     else begin issue<=1;state<=RUN;end
    end
    RESULT:if(out_ready_i)state<=IDLE;
    default:begin end
   endcase
  end
 end
endmodule
