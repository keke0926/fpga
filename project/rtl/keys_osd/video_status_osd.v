`timescale 1ns/1ps
// Chinese status strip; synchronous font ROM, two matched pixel cycles.
// Automatic mode does not invent or display a global numeric local threshold.
module video_status_osd #(parameter COORD_WIDTH=12,ENABLE_FPS=0,ENABLE_LOCAL=0)(
 input wire clk,rst_n,in_hs,in_vs,in_de,in_sof,in_eof,
 input wire[COORD_WIDTH-1:0]in_x,in_y,input wire[23:0]in_rgb,
 input wire cfg_app_mode,cfg_auto_mode,cfg_median_enable,
 input wire[1:0]cfg_view_mode,input wire[10:0]cfg_manual_threshold,
 input wire[11:0]fps_bcd_i,input wire[3:0]local_status_i,input wire local_reference_valid_i,
 output reg out_hs,out_vs,out_de,out_sof,out_eof,
 output reg[COORD_WIDTH-1:0]out_x,out_y,output reg[23:0]out_rgb);
 reg app_q,auto_q,median_q;reg[1:0]view_q;reg[15:0]bcd_q;
 reg[11:0]fps_q;reg[3:0]local_status_q;
 wire[3:0]local_status_now=in_sof?((local_status_i==0&&local_reference_valid_i)?4'd7:local_status_i):local_status_q;
 function[15:0]binary_to_bcd;input[10:0]v;integer i;reg[15:0]b;begin
  b=0;for(i=10;i>=0;i=i-1)begin
   if(b[3:0]>=5)b[3:0]=b[3:0]+3;if(b[7:4]>=5)b[7:4]=b[7:4]+3;
   if(b[11:8]>=5)b[11:8]=b[11:8]+3;if(b[15:12]>=5)b[15:12]=b[15:12]+3;
   b={b[14:0],v[i]};
  end binary_to_bcd=b;
 end endfunction
 wire app_now=in_sof?cfg_app_mode:app_q,auto_now=in_sof?cfg_auto_mode:auto_q;
 wire median_now=in_sof?cfg_median_enable:median_q;
 wire[1:0]view_now=in_sof?cfg_view_mode:view_q;
 wire[15:0]bcd_now=in_sof?binary_to_bcd(cfg_manual_threshold):bcd_q;
 wire in_fps_bar=ENABLE_FPS&&in_de&&in_x>=784&&in_x<1024&&in_y>=8&&in_y<32;
 wire in_local_bar=ENABLE_LOCAL&&app_now&&in_de&&in_x>=432&&in_x<560&&in_y>=8&&in_y<32;
 wire in_bar=(in_de&&in_x>=8&&in_x<408&&in_y>=8&&in_y<32)||in_fps_bar||in_local_bar;
 wire[COORD_WIDTH-1:0]bx=in_fps_bar?in_x-792:in_x-8,by=in_y-12;
 wire[11:0]fps_now=in_sof?fps_bcd_i:fps_q;
 wire[4:0]slot=bx>>4;reg[6:0]glyph;
 always@*begin
  glyph=0;
  if(in_fps_bar)begin
   case(slot)
    0:glyph=64;1:glyph=65;2:glyph=66;3:glyph=67;4:glyph=68;5:glyph=69;
    7:glyph=(fps_now[11:8]!=0)?({3'b0,fps_now[11:8]}+7'd1):0;
    8:glyph=(fps_now[11:4]!=0)?({3'b0,fps_now[7:4]}+7'd1):0;
    9:glyph={3'b0,fps_now[3:0]}+7'd1;
    11:glyph=68;12:glyph=52;13:glyph=70;
   endcase
  end else case(slot)
   0:glyph=app_now?13:11;1:glyph=app_now?14:12;
   3:glyph=auto_now?15:17;4:glyph=16;
   6:glyph=auto_now?0:44;7:glyph=auto_now?0:25;
   8:glyph=auto_now?0:({2'b0,bcd_now[15:12]}+6'd1);
   9:glyph=auto_now?0:({2'b0,bcd_now[11:8]}+6'd1);
   10:glyph=auto_now?0:({2'b0,bcd_now[7:4]}+6'd1);
   11:glyph=auto_now?0:({2'b0,bcd_now[3:0]}+6'd1);
   14:case(view_now)0:glyph=18;1:glyph=20;default:glyph=22;endcase
   15:case(view_now)0:glyph=19;1:glyph=21;default:glyph=23;endcase
   18:glyph=24;19:glyph=25;20:glyph=median_now?26:27;
  endcase
 end
 wire[15:0]font_bits;cn_font_rom u_font(.clk(clk),.glyph(glyph[5:0]),.row(by[3:0]),.bits(font_bits));
 wire new_font_ink;
 generate if(ENABLE_FPS)begin:Gen_FPS_Font
  fps_chinese_ink u_ink(.glyph(glyph[2:0]),.row(by[3:0]),.col(bx[3:0]),.ink(new_font_ink));
 end else begin:Gen_No_FPS_Font
  assign new_font_ink=1'b0;
 end endgenerate
 wire local_ink;
 wire[6:0]local_col=in_x-432;
 generate if(ENABLE_LOCAL)begin:Gen_Local_Font
  part_local_status_ink u_ink(.status(local_status_now),.row(by[3:0]),.col(local_col),.ink(local_ink));
 end else begin:Gen_No_Local_Font assign local_ink=0;end endgenerate
 reg plocal,plocalink;reg pextended,pnewink;
 reg ph,pv,pd,ps,pe,pbar,pglyph;reg[COORD_WIDTH-1:0]px,py;reg[23:0]prgb;reg[3:0]pcol;
 always@(posedge clk)begin
  if(!rst_n)begin
   app_q<=0;auto_q<=0;median_q<=0;view_q<=0;bcd_q<=0;fps_q<=0;local_status_q<=0;plocal<=0;plocalink<=0;pextended<=0;pnewink<=0;
   ph<=0;pv<=0;pd<=0;ps<=0;pe<=0;px<=0;py<=0;prgb<=0;pbar<=0;pglyph<=0;pcol<=0;
   out_hs<=0;out_vs<=0;out_de<=0;out_sof<=0;out_eof<=0;out_x<=0;out_y<=0;out_rgb<=0;
  end else begin
   if(in_sof)begin app_q<=cfg_app_mode;auto_q<=cfg_auto_mode;median_q<=cfg_median_enable;view_q<=cfg_view_mode;bcd_q<=binary_to_bcd(cfg_manual_threshold);if(ENABLE_FPS)fps_q<=fps_bcd_i;if(ENABLE_LOCAL)local_status_q<=(local_status_i==0&&local_reference_valid_i)?4'd7:local_status_i;end
   ph<=in_hs;pv<=in_vs;pd<=in_de;ps<=in_sof;pe<=in_eof;px<=in_x;py<=in_y;prgb<=in_rgb;
   pbar<=in_bar;pglyph<=in_y>=12&&in_y<28;pcol<=bx[3:0];
   pextended<=glyph[6];pnewink<=new_font_ink;plocal<=in_local_bar;plocalink<=local_ink;
   out_hs<=ph;out_vs<=pv;out_de<=pd;out_sof<=ps;out_eof<=pe;out_x<=px;out_y<=py;
   out_rgb<=pbar?((pglyph&&(plocal?plocalink:(pextended?pnewink:font_bits[15-pcol])))?24'hffffff:24'h000000):prgb;
  end
 end
endmodule
