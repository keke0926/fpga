`timescale 1ns/1ps
// Exact original coefficients. One quarter-wave lookup is read twice; unsigned
// magnitudes avoid wide absolute-value and sign-restoration carry chains.
module part_refine_coeff(
 input wire clk_i,rst_i,abort_i,start_i,output wire ready_o,
 input wire[8:0]angle_i,scale_q8_i,
 output wire valid_o,input wire out_ready_i,
 output reg signed[15:0]a_o,b_o,ia_o,ib_o,output reg signed[17:0]c_o,d_o
);
 reg[2:0]state;reg[16:0]sn,cs;reg sign_s,sign_c,swap;
 reg[8:0]scale;reg[16:0]inv;reg[6:0]cos_address;
 wire[1:0]quad=angle_i>=270?3:angle_i>=180?2:angle_i>=90?1:0;
 wire[6:0]phase=angle_i-(quad==3?270:quad==2?180:quad==1?90:0);
 wire[6:0]lookup_address=state==0?phase:cos_address;
 function[16:0]sine;input[6:0]i;begin case(i)
 7'd0:sine=17'd0;
 7'd1:sine=17'd1144;
 7'd2:sine=17'd2287;
 7'd3:sine=17'd3430;
 7'd4:sine=17'd4572;
 7'd5:sine=17'd5712;
 7'd6:sine=17'd6850;
 7'd7:sine=17'd7987;
 7'd8:sine=17'd9121;
 7'd9:sine=17'd10252;
 7'd10:sine=17'd11380;
 7'd11:sine=17'd12505;
 7'd12:sine=17'd13626;
 7'd13:sine=17'd14742;
 7'd14:sine=17'd15855;
 7'd15:sine=17'd16962;
 7'd16:sine=17'd18064;
 7'd17:sine=17'd19161;
 7'd18:sine=17'd20252;
 7'd19:sine=17'd21336;
 7'd20:sine=17'd22415;
 7'd21:sine=17'd23486;
 7'd22:sine=17'd24550;
 7'd23:sine=17'd25607;
 7'd24:sine=17'd26656;
 7'd25:sine=17'd27697;
 7'd26:sine=17'd28729;
 7'd27:sine=17'd29753;
 7'd28:sine=17'd30767;
 7'd29:sine=17'd31772;
 7'd30:sine=17'd32768;
 7'd31:sine=17'd33754;
 7'd32:sine=17'd34729;
 7'd33:sine=17'd35693;
 7'd34:sine=17'd36647;
 7'd35:sine=17'd37590;
 7'd36:sine=17'd38521;
 7'd37:sine=17'd39441;
 7'd38:sine=17'd40348;
 7'd39:sine=17'd41243;
 7'd40:sine=17'd42126;
 7'd41:sine=17'd42995;
 7'd42:sine=17'd43852;
 7'd43:sine=17'd44695;
 7'd44:sine=17'd45525;
 7'd45:sine=17'd46341;
 7'd46:sine=17'd47143;
 7'd47:sine=17'd47930;
 7'd48:sine=17'd48703;
 7'd49:sine=17'd49461;
 7'd50:sine=17'd50203;
 7'd51:sine=17'd50931;
 7'd52:sine=17'd51643;
 7'd53:sine=17'd52339;
 7'd54:sine=17'd53020;
 7'd55:sine=17'd53684;
 7'd56:sine=17'd54332;
 7'd57:sine=17'd54963;
 7'd58:sine=17'd55578;
 7'd59:sine=17'd56175;
 7'd60:sine=17'd56756;
 7'd61:sine=17'd57319;
 7'd62:sine=17'd57865;
 7'd63:sine=17'd58393;
 7'd64:sine=17'd58903;
 7'd65:sine=17'd59396;
 7'd66:sine=17'd59870;
 7'd67:sine=17'd60326;
 7'd68:sine=17'd60764;
 7'd69:sine=17'd61183;
 7'd70:sine=17'd61584;
 7'd71:sine=17'd61966;
 7'd72:sine=17'd62328;
 7'd73:sine=17'd62672;
 7'd74:sine=17'd62997;
 7'd75:sine=17'd63303;
 7'd76:sine=17'd63589;
 7'd77:sine=17'd63856;
 7'd78:sine=17'd64104;
 7'd79:sine=17'd64332;
 7'd80:sine=17'd64540;
 7'd81:sine=17'd64729;
 7'd82:sine=17'd64898;
 7'd83:sine=17'd65048;
 7'd84:sine=17'd65177;
 7'd85:sine=17'd65287;
 7'd86:sine=17'd65376;
 7'd87:sine=17'd65446;
 7'd88:sine=17'd65496;
 7'd89:sine=17'd65526;
 7'd90:sine=17'd65536;
 default:sine=0;endcase end endfunction
 reg[16:0]reciprocal;
 always@*begin case(scale_q8_i)
 9'd192:reciprocal=17'd87381;
 9'd200:reciprocal=17'd83886;
 9'd208:reciprocal=17'd80660;
 9'd216:reciprocal=17'd77672;
 9'd224:reciprocal=17'd74898;
 9'd232:reciprocal=17'd72316;
 9'd240:reciprocal=17'd69905;
 9'd248:reciprocal=17'd67650;
 9'd256:reciprocal=17'd65536;
 9'd264:reciprocal=17'd63550;
 9'd272:reciprocal=17'd61681;
 9'd280:reciprocal=17'd59919;
 9'd288:reciprocal=17'd58254;
 9'd296:reciprocal=17'd56680;
 9'd304:reciprocal=17'd55188;
 9'd312:reciprocal=17'd53773;
 9'd320:reciprocal=17'd52429;
 default:reciprocal=0;endcase end
 wire[16:0]lookup_value=sine(lookup_address);
 wire select_s=state==3||state==5;
 wire[16:0]ma=select_s?sn:cs;
 wire[16:0]mb=state==2||state==3?scale:inv;
 wire[33:0]product=ma*mb;
 wire[15:0]magnitude14=state==2||state==3?
  (product+34'd512)>>10:
  (product+34'd131072)>>18;
 wire negative=select_s?sign_s:sign_c;
 wire[15:0]signed14=negative?-magnitude14:magnitude14;
 wire[17:0]magnitude16=(product+34'd32768)>>16;
 wire[17:0]signed16=negative?-magnitude16:magnitude16;
 assign ready_o=state==0&&!rst_i&&!abort_i;
 assign valid_o=state==6&&!rst_i&&!abort_i;
 always@(posedge clk_i)begin
  if(rst_i||abort_i)begin
   state<=0;sn<=0;cs<=0;sign_s<=0;sign_c<=0;swap<=0;scale<=0;inv<=0;cos_address<=0;
   a_o<=0;b_o<=0;ia_o<=0;ib_o<=0;c_o<=0;d_o<=0;
  end else case(state)
   0:if(start_i)begin
    if(quad[0])cs<=lookup_value;else sn<=lookup_value;
    swap<=quad[0];sign_s<=quad[1];sign_c<=quad[0]^quad[1];
    cos_address<=7'd90-phase;scale<=scale_q8_i;inv<=reciprocal;state<=1;
   end
   1:begin if(swap)sn<=lookup_value;else cs<=lookup_value;state<=2;end
   2:begin a_o<=signed14;state<=3;end
   3:begin b_o<=signed14;state<=4;end
   4:begin ia_o<=signed14;c_o<=signed16;state<=5;end
   5:begin ib_o<=-signed14;d_o<=signed16;state<=6;end
   6:if(out_ready_i)state<=0;
   default:state<=0;
  endcase
 end
endmodule
