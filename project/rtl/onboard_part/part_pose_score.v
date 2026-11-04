`timescale 1ns/1ps
// Serial evidence accumulation for one supplied pose. One shared signed
// multiplier, no private image RAM. Flags are counted, never used as masks.
module part_pose_score #(parameter integer WIDTH=1024,HEIGHT=600,COUNTER_WIDTH=20)(
 input wire clk,rst,abort_i,start_i,output wire ready_o,
 input wire [31:0] task_id_i,frame_id_i,reference_id_i,config_epoch_i,
 output reg [31:0] task_id_o,frame_id_o,reference_id_o,config_epoch_o,
 input wire sample_valid_i,output wire sample_ready_o,
 input wire in_domain_i,last_i,input wire [9:0] obs_x_i,obs_y_i,
 input wire signed [12:0] ref_gx_i,ref_gy_i,
 input wire [9:0] ref_raw_i,input wire [1:0] ref_flags_i,
 input wire signed [10:0] obs_gx_i,obs_gy_i,
 input wire [7:0] obs_raw_i,input wire [1:0] obs_flags_i,
 output wire done_o,result_valid_o,busy_o,input wire out_ready_i,
 output wire [19:0] attempted_count_o,valid_count_o,
 output wire [19:0] ref_strong_count_o,ref_weak_count_o,obs_strong_count_o,obs_weak_count_o,
 output wire [19:0] joint_strong_count_o,direction_count_o,
 output wire [19:0] ref_clipped_count_o,ref_low_contrast_count_o,
 output wire [19:0] obs_clipped_count_o,obs_low_contrast_count_o,
 output reg bbox_valid_o,output reg [9:0] min_x_o,min_y_o,max_x_o,max_y_o,
 output reg [15:0] grid_o
);
 // Bounded scan schedule sets counter width; public outputs stay 20 bits.
 reg[COUNTER_WIDTH-1:0]attempted_count_o_held;assign attempted_count_o=attempted_count_o_held;
 reg[COUNTER_WIDTH-1:0]valid_count_o_held;assign valid_count_o=valid_count_o_held;
 reg[COUNTER_WIDTH-1:0]ref_strong_count_o_held;assign ref_strong_count_o=ref_strong_count_o_held;
 reg[COUNTER_WIDTH-1:0]ref_weak_count_o_held;assign ref_weak_count_o=ref_weak_count_o_held;
 reg[COUNTER_WIDTH-1:0]obs_strong_count_o_held;assign obs_strong_count_o=obs_strong_count_o_held;
 reg[COUNTER_WIDTH-1:0]obs_weak_count_o_held;assign obs_weak_count_o=obs_weak_count_o_held;
 reg[COUNTER_WIDTH-1:0]joint_strong_count_o_held;assign joint_strong_count_o=joint_strong_count_o_held;
 reg[COUNTER_WIDTH-1:0]direction_count_o_held;assign direction_count_o=direction_count_o_held;
 reg[COUNTER_WIDTH-1:0]ref_clipped_count_o_held;assign ref_clipped_count_o=ref_clipped_count_o_held;
 reg[COUNTER_WIDTH-1:0]ref_low_contrast_count_o_held;assign ref_low_contrast_count_o=ref_low_contrast_count_o_held;
 reg[COUNTER_WIDTH-1:0]obs_clipped_count_o_held;assign obs_clipped_count_o=obs_clipped_count_o_held;
 reg[COUNTER_WIDTH-1:0]obs_low_contrast_count_o_held;assign obs_low_contrast_count_o=obs_low_contrast_count_o_held;

 localparam IDLE=0,WAIT_SAMPLE=1,REF_X=2,REF_Y=3,OBS_X=4,OBS_Y=5,
  DOT_X=6,DOT_Y=7,CROSS_X=8,CROSS_Y=9,COMMIT=10,RESULT=11;
 reg [3:0] state;
 reg signed [12:0] rx,ry;
 reg signed [10:0] ox,oy;
 reg [9:0] raw_r,x,y;reg [7:0] raw_o;
 reg [1:0] flags_r,flags_o;reg last;
 reg strong_r,weak_r,strong_o,weak_o,direction_ok;
 reg signed [12:0] mul_a,mul_b;
 wire signed [25:0] product=mul_a*mul_b;
 wire signed [26:0] product_extended={product[25],product};
 reg signed [26:0] partial;
 wire signed [26:0] combined=state==CROSS_Y ? partial-product_extended : partial+product_extended;
 // max 25*(4096^2+4096^2)=838860800: these shifts retain all bits.
 wire [31:0] magnitude25=({5'b0,combined}<<4)+({5'b0,combined}<<3)+{5'b0,combined};
 wire [26:0] combined_abs=combined[26] ? (~combined+27'd1) : combined;
 reg [26:0] dot_abs;
 wire [28:0] cross_times4={combined_abs,2'b00};
 wire [28:0] dot_extended={2'b00,dot_abs};
 wire [11:0] x4={x,2'b00},y4={y,2'b00};
 wire [1:0] grid_x=x4>=3*WIDTH?2'd3:x4>=2*WIDTH?2'd2:x4>=WIDTH?2'd1:2'd0;
 wire [1:0] grid_y=y4>=3*HEIGHT?2'd3:y4>=2*HEIGHT?2'd2:y4>=HEIGHT?2'd1:2'd0;
 assign ready_o=!rst&&!abort_i&&state==IDLE;
 assign sample_ready_o=!rst&&!abort_i&&state==WAIT_SAMPLE;
 assign done_o=!rst&&!abort_i&&state==RESULT;
 assign result_valid_o=done_o;
 assign busy_o=!rst&&state!=IDLE;
 always @* begin
  mul_a=0;mul_b=0;
  case(state)
   REF_X:begin mul_a=rx;mul_b=rx;end
   REF_Y:begin mul_a=ry;mul_b=ry;end
   OBS_X:begin mul_a={{2{ox[10]}},ox};mul_b={{2{ox[10]}},ox};end
   OBS_Y:begin mul_a={{2{oy[10]}},oy};mul_b={{2{oy[10]}},oy};end
   DOT_X:begin mul_a=rx;mul_b={{2{ox[10]}},ox};end
   DOT_Y:begin mul_a=ry;mul_b={{2{oy[10]}},oy};end
   CROSS_X:begin mul_a=rx;mul_b={{2{oy[10]}},oy};end
   CROSS_Y:begin mul_a=ry;mul_b={{2{ox[10]}},ox};end
  endcase
 end
 task clear_evidence;
 begin
  attempted_count_o_held<=0;valid_count_o_held<=0;
  ref_strong_count_o_held<=0;ref_weak_count_o_held<=0;obs_strong_count_o_held<=0;obs_weak_count_o_held<=0;
  joint_strong_count_o_held<=0;direction_count_o_held<=0;
  ref_clipped_count_o_held<=0;ref_low_contrast_count_o_held<=0;obs_clipped_count_o_held<=0;obs_low_contrast_count_o_held<=0;
  bbox_valid_o<=0;min_x_o<=0;min_y_o<=0;max_x_o<=0;max_y_o<=0;grid_o<=0;
 end endtask
 always @(posedge clk)begin
  if(rst)begin
   state<=IDLE;task_id_o<=0;frame_id_o<=0;reference_id_o<=0;config_epoch_o<=0;
   rx<=0;ry<=0;ox<=0;oy<=0;raw_r<=0;raw_o<=0;flags_r<=0;flags_o<=0;x<=0;y<=0;last<=0;
   partial<=0;dot_abs<=0;strong_r<=0;weak_r<=0;strong_o<=0;weak_o<=0;direction_ok<=0;
   clear_evidence();
  end else if(abort_i)state<=IDLE;
  else case(state)
   IDLE:if(start_i)begin
    task_id_o<=task_id_i;frame_id_o<=frame_id_i;reference_id_o<=reference_id_i;config_epoch_o<=config_epoch_i;
    clear_evidence();state<=WAIT_SAMPLE;
   end
   WAIT_SAMPLE:if(sample_valid_i)begin
    attempted_count_o_held<=attempted_count_o+1'b1;
    if(in_domain_i)begin
     rx<=ref_gx_i;ry<=ref_gy_i;ox<=obs_gx_i;oy<=obs_gy_i;
     raw_r<=ref_raw_i;raw_o<=obs_raw_i;flags_r<=ref_flags_i;flags_o<=obs_flags_i;
     x<=obs_x_i;y<=obs_y_i;last<=last_i;state<=REF_X;
    end else if(last_i)state<=RESULT;
   end
   REF_X:begin partial<=product_extended;state<=REF_Y;end
   REF_Y:begin
    strong_r<=magnitude25>=32'd4096&&raw_r>=10'd48;
    weak_r<=magnitude25>=32'd784&&raw_r>=10'd24;state<=OBS_X;
   end
   OBS_X:begin partial<=product_extended;state<=OBS_Y;end
   OBS_Y:begin
    strong_o<=magnitude25>=32'd4096&&raw_o>=8'd48;
    weak_o<=magnitude25>=32'd784&&raw_o>=8'd24;state<=DOT_X;
   end
   DOT_X:begin partial<=product_extended;state<=DOT_Y;end
   DOT_Y:begin dot_abs<=combined_abs;state<=CROSS_X;end
   CROSS_X:begin partial<=product_extended;state<=CROSS_Y;end
   CROSS_Y:begin direction_ok<=cross_times4<=dot_extended;state<=COMMIT;end
   COMMIT:begin
    valid_count_o_held<=valid_count_o+1'b1;
    ref_strong_count_o_held<=ref_strong_count_o+strong_r;ref_weak_count_o_held<=ref_weak_count_o+weak_r;
    obs_strong_count_o_held<=obs_strong_count_o+strong_o;obs_weak_count_o_held<=obs_weak_count_o+weak_o;
    joint_strong_count_o_held<=joint_strong_count_o+(strong_r&&strong_o);
    direction_count_o_held<=direction_count_o+(strong_r&&strong_o&&direction_ok);
    ref_clipped_count_o_held<=ref_clipped_count_o+flags_r[0];ref_low_contrast_count_o_held<=ref_low_contrast_count_o+flags_r[1];
    obs_clipped_count_o_held<=obs_clipped_count_o+flags_o[0];obs_low_contrast_count_o_held<=obs_low_contrast_count_o+flags_o[1];
    if(!bbox_valid_o)begin min_x_o<=x;max_x_o<=x;min_y_o<=y;max_y_o<=y;bbox_valid_o<=1;end
    else begin
     if(x<min_x_o)min_x_o<=x;if(x>max_x_o)max_x_o<=x;
     if(y<min_y_o)min_y_o<=y;if(y>max_y_o)max_y_o<=y;
    end
    grid_o<=grid_o|(16'h0001<<{grid_y,grid_x});
    state<=last?RESULT:WAIT_SAMPLE;
   end
   RESULT:if(out_ready_i)state<=IDLE;
   default:state<=IDLE;
  endcase
 end
endmodule
