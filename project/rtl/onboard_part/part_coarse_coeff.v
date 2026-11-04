`timescale 1ns/1ps
// Exact original coefficients. One quarter-wave lookup is read twice; unsigned
// magnitudes avoid wide absolute-value and sign-restoration carry chains.
module part_coarse_coeff(
 input wire clk_i,rst_i,abort_i,start_i,output wire ready_o,
 input wire[6:0]angle_i,input wire[3:0]scale_i,
 output wire valid_o,input wire out_ready_i,
 output reg signed[15:0]a_o,b_o,ia_o,ib_o
);
 reg[2:0]state;reg[14:0]sn,cs;reg sign_s,sign_c,swap;
 reg[8:0]scale;reg[15:0]inv;reg[4:0]cos_address;
 wire[1:0]quad=angle_i>=54?3:angle_i>=36?2:angle_i>=18?1:0;
 wire[4:0]phase=angle_i-(quad==3?54:quad==2?36:quad==1?18:0);
 wire[4:0]lookup_address=state==0?phase:cos_address;
 function[14:0]sine;input[4:0]i;begin case(i)
 5'd0:sine=15'd0;
 5'd1:sine=15'd1428;
 5'd2:sine=15'd2845;
 5'd3:sine=15'd4240;
 5'd4:sine=15'd5604;
 5'd5:sine=15'd6924;
 5'd6:sine=15'd8192;
 5'd7:sine=15'd9397;
 5'd8:sine=15'd10531;
 5'd9:sine=15'd11585;
 5'd10:sine=15'd12551;
 5'd11:sine=15'd13421;
 5'd12:sine=15'd14189;
 5'd13:sine=15'd14849;
 5'd14:sine=15'd15396;
 5'd15:sine=15'd15826;
 5'd16:sine=15'd16135;
 5'd17:sine=15'd16322;
 5'd18:sine=15'd16384;
 default:sine=0;endcase end endfunction
 reg[15:0]reciprocal;
 always@*begin case(scale_i)
 4'd0:reciprocal=16'd21845;
 4'd1:reciprocal=16'd20165;
 4'd2:reciprocal=16'd18725;
 4'd3:reciprocal=16'd17476;
 4'd4:reciprocal=16'd16384;
 4'd5:reciprocal=16'd15420;
 4'd6:reciprocal=16'd14564;
 4'd7:reciprocal=16'd13797;
 4'd8:reciprocal=16'd13107;
 default:reciprocal=0;endcase end
 wire[14:0]lookup_value=sine(lookup_address);
 wire select_s=state==3||state==5;
 wire[14:0]ma=select_s?sn:cs;
 wire[15:0]mb=state==2||state==3?scale:inv;
 wire[30:0]product=ma*mb;
 wire[15:0]magnitude14=state==2||state==3?
  (product+31'd128)>>8:
  (product+31'd8192)>>14;
 wire negative=select_s?sign_s:sign_c;
 wire[15:0]signed14=negative?-magnitude14:magnitude14;
 assign ready_o=state==0&&!rst_i&&!abort_i;
 assign valid_o=state==6&&!rst_i&&!abort_i;
 always@(posedge clk_i)begin
  if(rst_i||abort_i)begin
   state<=0;sn<=0;cs<=0;sign_s<=0;sign_c<=0;swap<=0;scale<=0;inv<=0;cos_address<=0;
   a_o<=0;b_o<=0;ia_o<=0;ib_o<=0;
  end else case(state)
   0:if(start_i)begin
    if(quad[0])cs<=lookup_value;else sn<=lookup_value;
    swap<=quad[0];sign_s<=quad[1];sign_c<=quad[0]^quad[1];
    cos_address<=5'd18-phase;scale<=9'd192+{scale_i,4'b0};inv<=reciprocal;state<=1;
   end
   1:begin if(swap)sn<=lookup_value;else cs<=lookup_value;state<=2;end
   2:begin a_o<=signed14;state<=3;end
   3:begin b_o<=signed14;state<=4;end
   4:begin ia_o<=signed14;state<=5;end
   5:begin ib_o<=-signed14;state<=6;end
   6:if(out_ready_i)state<=0;
   default:state<=0;
  endcase
 end
endmodule
