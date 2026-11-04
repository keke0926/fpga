`timescale 1ns/1ps
// Two matched clocks for every pixel; labels belong only to the frozen CRC-bound frame.
module part_assist_overlay #(parameter EXTERNAL_FONT=0)(
 input wire[15:0]font_bits_i,output wire[9:0]font_address_o,
 input wire clk,rst_n,in_hs,in_vs,in_de,in_sof,in_eof,
 input wire[11:0] in_x,in_y,input wire[23:0] in_rgb,
 input wire display_valid,input wire[319:0] display_boxes,input wire[3:0] display_rows,
 input wire[9:0] display_count,input wire[5:0] display_page,input wire[6:0] display_pages,
 output reg out_hs,out_vs,out_de,out_sof,out_eof,
 output reg[11:0] out_x,out_y,output reg[23:0] out_rgb);
 localparam [23:0] REVIEW_RGB=24'hff44cc;
 // Eight cached labels per page; the result RAM still holds 512 records.
 // Sequential placement on 16-pixel bands avoids pixel-path all-pairs logic.
 reg [9:0] marker_x[0:7],marker_y[0:7],candidate_x,candidate_y;
 reg [3:0] work_i,work_j;
 reg [1:0] layout_state;
 reg layout_ready;
 wire [9:0] next_x=display_boxes[work_i*40+:10];
 wire [9:0] next_y=display_boxes[work_i*40+10+:10];
 integer j;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   candidate_x<=0;candidate_y<=0;work_i<=0;work_j<=0;
   layout_state<=0;layout_ready<=0;
   for(j=0;j<8;j=j+1)begin marker_x[j]<=0;marker_y[j]<=0;end
  end else if(!display_valid)begin layout_state<=0;layout_ready<=0;end
  else if(in_sof)begin
   work_i<=0;work_j<=0;layout_ready<=display_rows==0;
   layout_state<=display_rows==0?0:1;
  end else case(layout_state)
   1:begin
    candidate_x<=next_x<112?10'd112:(next_x>1000?10'd1000:next_x);
    candidate_y<=next_y<64?10'd64:(next_y>=576?10'd576:{next_y[9:4],4'b0});
    work_j<=0;layout_state<=2;
   end
   2:begin
    if(work_j>=work_i)begin
     marker_x[work_i]<=candidate_x;marker_y[work_i]<=candidate_y;
     if(work_i+1>=display_rows)begin layout_ready<=1;layout_state<=0;end
     else begin work_i<=work_i+1'b1;layout_state<=1;end
    end else if(candidate_y==marker_y[work_j]&&
        candidate_x<marker_x[work_j]+11'd24&&marker_x[work_j]<candidate_x+11'd24)begin
     candidate_y<=candidate_y==576?10'd64:candidate_y+10'd16;work_j<=0;
    end else work_j<=work_j+1'b1;
   end
   default:layout_state<=0;
  endcase
 end
 reg border,text_region,chosen,digit_region;
 reg[9:0] number_value;reg[1:0] digit_cell;
 reg page_tens;
 // Shared pixel offsets keep the two-pixel stroke inside each box.
 wire [11:0] x_prev=in_x-12'd1,x_next=in_x+12'd1;
 wire [11:0] y_prev=in_y-12'd1,y_next=in_y+12'd1;
 reg [5:0] glyph;reg[3:0] row,col;
 reg[9:0] x0,y0,x1,y1,tag;
 integer i,cellx;
 wire[6:0] page_number=display_pages==0?7'd0:{1'b0,display_page}+1'b1;
 // Select the active number first so the title, page and row labels share
 // one combinational binary-to-decimal converter on the existing pixel path.
 function [12:0] to_decimal;
  input [9:0] number;
  reg [12:0] bcd;
  integer bit_index,digit_index;
  begin
   bcd=0;
   for(bit_index=9;bit_index>=0;bit_index=bit_index-1)begin
    for(digit_index=0;digit_index<3;digit_index=digit_index+1)
     if(bcd[digit_index*4+:4]>=5)bcd[digit_index*4+:4]=bcd[digit_index*4+:4]+4'd3;
    bcd={bcd[11:0],number[bit_index]};
   end
   to_decimal=bcd;
  end
 endfunction
 reg [12:0] digits;
 always @*begin
  border=0;text_region=0;chosen=0;
  glyph=0;row=0;col=0;digit_region=0;number_value=0;digit_cell=0;page_tens=0;
  digits=0;x0=0;y0=0;x1=0;y1=0;tag=0;cellx=0;
  if(display_valid&&in_de)begin
   for(i=0;i<8;i=i+1)begin
    x0=display_boxes[i*40+:10];y0=display_boxes[i*40+10+:10];
    x1=display_boxes[i*40+20+:10];y1=display_boxes[i*40+30+:10];
    if(i<display_rows)begin
     if(!border&&in_x>=x0&&in_x<=x1&&in_y>=y0&&in_y<=y1&&
       (in_x==x0||x_prev==x0||in_x==x1||x_next==x1||
        in_y==y0||y_prev==y0||in_y==y1||y_next==y1))border=1;
     if(layout_ready&&!chosen&&in_y>=marker_y[i]&&in_y<marker_y[i]+16&&
       in_x>=marker_x[i]&&in_x<marker_x[i]+24)begin
      text_region=1;chosen=1;row=in_y-marker_y[i];
      col=((in_x-marker_x[i])&7)+4;cellx=(in_x-marker_x[i])>>3;
      tag={display_page,3'b0}+i+1;
      digit_region=1;digit_cell=cellx;number_value=tag;
     end
     // Dedicated Chinese list is disjoint from every numeric-marker rectangle.
     if(in_y>=64+i*20&&in_y<80+i*20)begin
      tag={display_page,3'b0}+i+1;
      if(in_x>=8&&in_x<32)begin
       text_region=1;row=in_y-(64+i*20);
       col=((in_x-8)&7)+4;cellx=(in_x-8)>>3;
       digit_region=1;digit_cell=cellx;number_value=tag;
      end
      if(in_x>=40&&in_x<88)begin
       text_region=1;digit_region=0;row=in_y-(64+i*20);col=in_x-40;cellx=(in_x-40)>>4;
       case(cellx)0:glyph=60;1:glyph=61;2:glyph=62;default:glyph=0;endcase
      end
     end
    end
   end
   if(in_y>=40&&in_y<56&&in_x>=8&&in_x<328)begin
    text_region=1;digit_region=0;row=in_y-40;col=in_x-8;cellx=(in_x-8)>>4;
    case(cellx)
     0:glyph=58;1:glyph=59;3:glyph=60;4:glyph=61;5:glyph=62;
     7,8,9:begin digit_region=1;number_value=display_count;digit_cell=cellx-7;end
     11,12:begin digit_region=1;number_value=page_number;digit_cell=cellx-10;page_tens=cellx==11;end
     13:glyph=52;
     14,15:begin digit_region=1;number_value=display_pages;digit_cell=cellx-13;page_tens=cellx==14;end
     16:glyph=53;
     default:glyph=0;
    endcase
   end
  end
  if(digit_region)begin
   digits=to_decimal(number_value);
   case(digit_cell)
    0:glyph={2'd0,digits[11:8]}+(digits[12]?6'd11:6'd1);
    1:glyph={2'd0,digits[7:4]}+((page_tens&&digits[8])?6'd11:6'd1);
    2:glyph={2'd0,digits[3:0]}+6'd1;
    default:glyph=0;
   endcase
  end
 end
 wire[15:0] font_bits;
 assign font_address_o={glyph,row};
 generate if(EXTERNAL_FONT)begin:SharedFont assign font_bits=font_bits_i;end
 else begin:PrivateFont cn_font_rom font(.clk(clk),.glyph(glyph),.row(row),.bits(font_bits));end endgenerate
 reg h1,v1,d1,s1,e1,b1,t1;reg[11:0] x_d,y_d;reg[23:0] rgb_d;reg[3:0] col_d;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   h1<=0;v1<=0;d1<=0;s1<=0;e1<=0;b1<=0;t1<=0;x_d<=0;y_d<=0;rgb_d<=0;col_d<=0;
   out_hs<=0;out_vs<=0;out_de<=0;out_sof<=0;out_eof<=0;out_x<=0;out_y<=0;out_rgb<=0;
  end else begin
   h1<=in_hs;v1<=in_vs;d1<=in_de;s1<=in_sof;e1<=in_eof;x_d<=in_x;y_d<=in_y;rgb_d<=in_rgb;
   b1<=border;t1<=text_region;col_d<=col;
   out_hs<=h1;out_vs<=v1;out_de<=d1;out_sof<=s1;out_eof<=e1;out_x<=x_d;out_y<=y_d;
   if(display_valid&&d1&&t1&&font_bits[15-col_d])out_rgb<=REVIEW_RGB;
   else if(display_valid&&d1&&b1)out_rgb<=REVIEW_RGB;
   else out_rgb<=rgb_d;
  end
 end
endmodule
