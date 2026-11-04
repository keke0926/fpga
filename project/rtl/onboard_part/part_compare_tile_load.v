`timescale 1ns/1ps
// Fill a 42x42 halo tile through the shared native sampler and scalar direction
// service. No DDR cache or private RAM here; parent holds pose/context until done.
// Descriptor [19]=normalization-domain-valid, [18]=clip, [17]=weak, [16]=strong,
// [15:9]=weight, [8:0]=angle. Reference vector is two signed 16-bit components.
module part_compare_tile_load #(
 parameter WIDTH=1024,HEIGHT=600,DESC_BASE=0,VECTOR_BASE=5292,CENTER_BASE=12348
)(
 input wire clk_i,rst_i,abort_i,start_i,reference_i,
 input wire[9:0]tile_x_i,tile_y_i,
 output wire ready_o,busy_o,valid_o,input wire out_ready_i,output reg[3:0]error_o,output reg center_strong_o,
 output wire sample_start_o,sample_accept_o,input wire sample_ready_i,sample_valid_i,
 input wire[3:0]sample_error_i,input wire sample_domain_i,sample_interior_i,
 output wire[9:0]sample_x_o,sample_y_o,
 input wire signed[10:0]obs_gx_i,obs_gy_i,input wire[7:0]obs_raw_i,input wire[1:0]obs_flags_i,
 input wire signed[12:0]ref_gx_i,ref_gy_i,input wire[9:0]ref_raw_i,input wire[1:0]ref_flags_i,
 output wire direction_start_o,direction_accept_o,input wire direction_ready_i,direction_valid_i,
 output wire signed[12:0]direction_gx_o,direction_gy_o,output wire[9:0]direction_raw_o,
 output wire[1:0]direction_flags_o,input wire[8:0]direction_angle_i,input wire[6:0]direction_weight_i,
 input wire direction_strong_i,direction_weak_i,
 output wire[15:0]gray_addr_o,output wire gray_we_o,output wire[7:0]gray_data_o,
 output wire[15:0]state_addr_o,output wire[1:0]state_we_o,state_data_o,input wire[1:0]state_data_i
);
 localparam IDLE=0,SAMPLE=1,WAIT_SAMPLE=2,DIRECTION=3,WAIT_DIRECTION=4,
 DESC=5,EXTRA=6,NEXT=7,RESULT=8;
 reg[3:0]state;reg reference_mode,padding,interior;
 reg[5:0]x,y;reg[10:0]index;reg[2:0]lane;
 wire signed[11:0]global_x=$signed({2'd0,tile_x_i})+$signed({6'd0,x})-12'sd11;
 wire signed[11:0]global_y=$signed({2'd0,tile_y_i})+$signed({6'd0,y})-12'sd11;
 wire outside=global_x<0||global_y<0||global_x>=WIDTH||global_y>=HEIGHT;
 wire center=x>=11&&x<31&&y>=11&&y<31;
 wire[8:0]center_index=(y-6'd11)*20+(x-6'd11);
 wire[8:0]angle=padding?9'd0:direction_angle_i;
 wire[6:0]weight=padding?7'd0:direction_weight_i;
 wire strong_edge=!padding&&direction_strong_i&&(!reference_mode||interior);
 wire weak_edge=!padding&&direction_weak_i&&(!reference_mode||interior);
 wire[1:0]flags=padding?2'd0:reference_mode?ref_flags_i:obs_flags_i;
 wire[23:0]descriptor={4'd0,interior,flags[0],weak_edge,strong_edge,weight,angle};
 wire signed[15:0]ogx={{5{obs_gx_i[10]}},obs_gx_i},ogy={{5{obs_gy_i[10]}},obs_gy_i};
 wire signed[15:0]rgx={{3{ref_gx_i[12]}},ref_gx_i},rgy={{3{ref_gy_i[12]}},ref_gy_i};
 wire[31:0]vector=padding?32'd0:{rgy,rgx};
 wire[55:0]center_word=padding?56'd0:{5'd0,flags[0],weak_edge,strong_edge,7'd0,angle,ogy,ogx};
 assign ready_o=state==IDLE&&!rst_i&&!abort_i;
 assign busy_o=state!=IDLE&&state!=RESULT;
 assign valid_o=state==RESULT&&!rst_i&&!abort_i;
 assign sample_start_o=state==SAMPLE&&!outside&&!rst_i&&!abort_i;
 assign sample_accept_o=state==NEXT&&!padding&&!rst_i&&!abort_i;
 assign sample_x_o=global_x[9:0];assign sample_y_o=global_y[9:0];
 assign direction_start_o=state==DIRECTION&&!rst_i&&!abort_i;
 assign direction_accept_o=state==NEXT&&!padding&&!rst_i&&!abort_i;
 assign direction_gx_o=reference_mode?ref_gx_i:{{2{obs_gx_i[10]}},obs_gx_i};
 assign direction_gy_o=reference_mode?ref_gy_i:{{2{obs_gy_i[10]}},obs_gy_i};
 assign direction_raw_o=reference_mode?ref_raw_i:{2'd0,obs_raw_i};
 assign direction_flags_o=reference_mode?ref_flags_i:obs_flags_i;
 assign gray_addr_o=state==DESC?DESC_BASE+index*3+lane:
                    reference_mode?VECTOR_BASE+index*4+lane:CENTER_BASE+center_index*7+lane;
 assign gray_we_o=(state==DESC||state==EXTRA)&&!rst_i&&!abort_i;
 assign gray_data_o=state==DESC?(descriptor>>(lane*8)):reference_mode?(vector>>(lane*8)):(center_word>>(lane*8));
 assign state_addr_o={5'd0,index};
 assign state_we_o=state==NEXT&&!rst_i&&!abort_i?(reference_mode?2'b10:2'b11):2'd0;
 assign state_data_o=reference_mode?{state_data_i[1]|flags[0],1'b0}:{1'b0,flags[0]};
 always@(posedge clk_i)begin
  if(rst_i||abort_i)begin state<=IDLE;reference_mode<=0;padding<=0;interior<=0;x<=0;y<=0;index<=0;lane<=0;error_o<=0;center_strong_o<=0;end
  else case(state)
   IDLE:if(start_i)begin reference_mode<=reference_i;x<=0;y<=0;index<=0;error_o<=0;center_strong_o<=0;state<=SAMPLE;end
   SAMPLE:begin
    padding<=outside;interior<=0;
    if(outside)begin lane<=0;state<=DESC;end
    else if(sample_ready_i)state<=WAIT_SAMPLE;
   end
   WAIT_SAMPLE:if(sample_valid_i)begin
    if(sample_error_i!=0)begin error_o<=sample_error_i;state<=RESULT;end
    else begin interior<=sample_domain_i&&(!reference_mode||sample_interior_i);state<=DIRECTION;end
   end
   DIRECTION:if(direction_ready_i)state<=WAIT_DIRECTION;
   WAIT_DIRECTION:if(direction_valid_i)begin lane<=0;state<=DESC;end
   DESC:if(lane==2)begin lane<=0;state<=reference_mode||center?EXTRA:NEXT;end else lane<=lane+1'b1;
   EXTRA:if(lane==(reference_mode?3'd3:3'd6))state<=NEXT;else lane<=lane+1'b1;
   NEXT:begin
    if(center&&strong_edge)center_strong_o<=1;
    if(index==1763)state<=RESULT;
    else begin index<=index+1'b1;if(x==41)begin x<=0;y<=y+1'b1;end else x<=x+1'b1;state<=SAMPLE;end
   end
   RESULT:if(out_ready_i)state<=IDLE;
   default:state<=IDLE;
  endcase
 end
endmodule
