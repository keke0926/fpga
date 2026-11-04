`timescale 1ns/1ps
// 128x128 raw tile -> <=112x112 median core, offset (1,1).
// Reuses the caller's gray RAM; nine sorting phases share four comparators.
module part_median_tile(
 input wire clk,rst_n,abort_i,start_i,input wire[7:0]width_i,height_i,
 output reg[15:0]gray_addr_o,input wire[7:0]gray_data_i,
 output wire out_valid_o,input wire out_ready_i,
 output reg[6:0]out_x_o,out_y_o,output wire[7:0]out_gray_o
);
 localparam IDLE=0,READ=1,WAIT=2,SORT=3,OUT=4;
 reg[2:0]state;reg[3:0]tap,layer;reg[7:0]width,height;
 reg[71:0]values;reg[7:0]tx,ty;
 wire[31:0]lower,upper;
 genvar k;
 generate for(k=0;k<4;k=k+1)begin:compare
  wire[7:0]a=layer[0]?values[(2*k+1)*8+:8]:values[(2*k)*8+:8];
  wire[7:0]b=layer[0]?values[(2*k+2)*8+:8]:values[(2*k+1)*8+:8];
  assign lower[k*8+:8]=a<b?a:b;
  assign upper[k*8+:8]=a<b?b:a;
 end endgenerate
 assign out_valid_o=rst_n&&!abort_i&&state==OUT;
 assign out_gray_o=values[39:32];
 always@*begin
  tx={1'b0,out_x_o}+1'b1;ty={1'b0,out_y_o}+1'b1;
  case(tap)
   0:begin tx=tx-1'b1;ty=ty-1'b1;end
   1:ty=ty-1'b1;
   2:begin tx=tx+1'b1;ty=ty-1'b1;end
   3:tx=tx-1'b1;5:tx=tx+1'b1;
   6:begin tx=tx-1'b1;ty=ty+1'b1;end
   7:ty=ty+1'b1;
   8:begin tx=tx+1'b1;ty=ty+1'b1;end
  endcase
  gray_addr_o={2'd0,ty[6:0],tx[6:0]};
 end
 integer i;
 always@(posedge clk)begin
  if(!rst_n||abort_i)begin state<=IDLE;tap<=0;layer<=0;values<=0;width<=0;height<=0;out_x_o<=0;out_y_o<=0;end
  else case(state)
   IDLE:if(start_i)begin width<=width_i;height<=height_i;out_x_o<=0;out_y_o<=0;tap<=0;state<=READ;end
   READ:state<=WAIT;
   WAIT:begin
    values[tap*8+:8]<=gray_data_i;
    if(tap==8)begin layer<=0;state<=SORT;end else begin tap<=tap+1'b1;state<=READ;end
   end
   SORT:begin
    for(i=0;i<4;i=i+1)begin
     if(layer[0])begin values[(2*i+1)*8+:8]<=lower[i*8+:8];values[(2*i+2)*8+:8]<=upper[i*8+:8];end
     else begin values[(2*i)*8+:8]<=lower[i*8+:8];values[(2*i+1)*8+:8]<=upper[i*8+:8];end
    end
    if(layer==8)state<=OUT;else layer<=layer+1'b1;
   end
   OUT:if(out_ready_i)begin
    if(out_x_o+1==width&&out_y_o+1==height)state<=IDLE;
    else begin
     if(out_x_o+1==width)begin out_x_o<=0;out_y_o<=out_y_o+1'b1;end else out_x_o<=out_x_o+1'b1;
     tap<=0;state<=READ;
    end
   end
  endcase
 end
endmodule
