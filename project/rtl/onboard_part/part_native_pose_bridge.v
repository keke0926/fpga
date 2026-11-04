`timescale 1ns/1ps
// Convert cell-centered coordinates to the native sampler's inverse Q16 map.
// One shared 18x18 multiplier. Centers include +3.5 native pixels exactly.
module part_native_pose_bridge(
 input wire clk_i,rst_i,abort_i,start_i,output wire ready_o,
 input wire signed[17:0]c_q16_i,d_q16_i,
 input wire[15:0]ref_cx_q8_i,ref_cy_q8_i,obs_cx_q8_i,obs_cy_q8_i,
 input wire signed[15:0]dx_q8_i,dy_q8_i,
 output wire valid_o,input wire out_ready_i,
 output reg signed[17:0]c_q16_o,d_q16_o,
 output reg signed[28:0]offset_x_q16_o,offset_y_q16_o
);
 reg[2:0]state;reg signed[17:0]rx,ry,qx,qy;reg signed[35:0]first;
 wire signed[17:0]ma=state==1||state==4?c_q16_o:d_q16_o;
 wire signed[17:0]mb=state==1||state==3?qx:qy;
 wire signed[35:0]product=ma*mb;
 wire signed[36:0]sum_x=$signed({first[35],first})+$signed({product[35],product});
 wire signed[36:0]sum_y=$signed({product[35],product})-$signed({first[35],first});
 wire signed[36:0]rx_native=$signed({{19{rx[17]}},rx})<<<11;
 wire signed[36:0]ry_native=$signed({{19{ry[17]}},ry})<<<11;
 assign ready_o=state==0&&!rst_i&&!abort_i;assign valid_o=state==5&&!rst_i&&!abort_i;
 always@(posedge clk_i)begin
  if(rst_i||abort_i)begin state<=0;rx<=0;ry<=0;qx<=0;qy<=0;first<=0;c_q16_o<=0;d_q16_o<=0;offset_x_q16_o<=0;offset_y_q16_o<=0;end
  else case(state)
   0:if(start_i)begin
    c_q16_o<=c_q16_i;d_q16_o<=d_q16_i;
    rx<=$signed({2'b0,ref_cx_q8_i})+18'sd112;ry<=$signed({2'b0,ref_cy_q8_i})+18'sd112;
    qx<=$signed({2'b0,obs_cx_q8_i})+$signed({{2{dx_q8_i[15]}},dx_q8_i})+18'sd112;
    qy<=$signed({2'b0,obs_cy_q8_i})+$signed({{2{dy_q8_i[15]}},dy_q8_i})+18'sd112;state<=1;
   end
   1:begin first<=product;state<=2;end
   2:begin offset_x_q16_o<=rx_native-(sum_x>>>5);state<=3;end
   3:begin first<=product;state<=4;end
   4:begin offset_y_q16_o<=ry_native-(sum_y>>>5);state<=5;end
   5:if(out_ready_i)state<=0;
   default:state<=0;
  endcase
 end
endmodule
