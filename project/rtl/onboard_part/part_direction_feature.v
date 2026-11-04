`timescale 1ns/1ps
// Native/transformed Q4 gradient -> fine unsigned direction + soft-splat weight.
// One serial multiplier; no image/line RAM. Caller owns the frozen task context.
module part_direction_feature(
 input wire clk_i,rst_i,abort_i,start_i,output wire ready_o,
 input wire signed[12:0]gx_i,gy_i,input wire[9:0]raw_i,input wire[1:0]flags_i,
 output wire valid_o,input wire out_ready_i,
 output reg[8:0]angle_o,output reg[6:0]weight_o,
 output reg strong_o,weak_o,output reg[1:0]flags_o
);
 localparam IDLE=0,SQUARE_X=1,SQUARE_Y=2,ROOT_INIT=3,ROOT_STEP=4,
  DIV_INIT=5,DIV_STEP=6,ANGLE=7,RESULT=8;
 reg[3:0]state;reg[12:0]ax,ay,lo,hi,rem_q;
 reg negative,y_major;reg[9:0]raw;reg[25:0]norm;
 reg[5:0]root;reg[2:0]step;reg[8:0]ratio;
 wire[5:0]trial=root|(6'd1<<step);
 reg[12:0]operand;
 always@*begin
  operand={7'd0,trial};
  if(state==SQUARE_X)operand=ax;
  else if(state==SQUARE_Y)operand=ay;
 end
 wire[25:0]product=operand*operand;
 wire[25:0]sum=norm+product;
 wire[30:0]sum25=({5'd0,sum}<<4)+({5'd0,sum}<<3)+{5'd0,sum};
 wire[13:0]shifted={rem_q,1'b0};wire subtract=shifted>={1'b0,hi};
 wire[13:0]reduced=shifted-{1'b0,hi};
 `include "part_direction_lut.vh"
 wire[7:0]angle_small=angle_rom[ratio];
 wire[8:0]base_angle=y_major?9'd256-{1'b0,angle_small}:{1'b0,angle_small};
 assign ready_o=state==IDLE&&!rst_i&&!abort_i;
 assign valid_o=state==RESULT&&!rst_i&&!abort_i;
 always@(posedge clk_i)begin
  if(rst_i||abort_i)begin
   state<=IDLE;ax<=0;ay<=0;lo<=0;hi<=0;rem_q<=0;negative<=0;y_major<=0;raw<=0;norm<=0;
   root<=0;step<=0;ratio<=0;angle_o<=0;weight_o<=0;strong_o<=0;weak_o<=0;flags_o<=0;
  end else case(state)
   IDLE:if(start_i&&ready_o)begin
    ax<=gx_i[12]?$unsigned(-gx_i):$unsigned(gx_i);
    ay<=gy_i[12]?$unsigned(-gy_i):$unsigned(gy_i);
    negative<=gx_i[12]^gy_i[12];raw<=raw_i;flags_o<=flags_i;state<=SQUARE_X;
   end
   SQUARE_X:begin
    norm<=product;lo<=ax<ay?ax:ay;hi<=ax<ay?ay:ax;y_major<=ay>ax;state<=SQUARE_Y;
   end
   SQUARE_Y:begin
    norm<=sum;strong_o<=sum25>=4096&&raw>=48;weak_o<=sum25>=784&&raw>=24;state<=ROOT_INIT;
   end
   ROOT_INIT:begin
    if(norm>=4096)begin weight_o<=64;state<=DIV_INIT;end
    else begin root<=0;step<=5;state<=ROOT_STEP;end
   end
   ROOT_STEP:begin
    if(product<=norm)root<=trial;
    if(step==0)begin weight_o<={1'b0,(product<=norm?trial:root)};state<=DIV_INIT;end
    else step<=step-1'b1;
   end
   DIV_INIT:begin
    ratio<=0;rem_q<=lo;step<=7;
    if(hi==0)state<=ANGLE;
    else if(lo==hi)begin ratio<=256;state<=ANGLE;end
    else state<=DIV_STEP;
   end
   DIV_STEP:begin
    rem_q<=subtract?reduced[12:0]:shifted[12:0];ratio<={ratio[7:0],subtract};
    if(step==0)state<=ANGLE;else step<=step-1'b1;
   end
   ANGLE:begin angle_o<=negative?9'd0-base_angle:base_angle;state<=RESULT;end
   RESULT:if(out_ready_i)state<=IDLE;
   default:state<=IDLE;
  endcase
 end
endmodule
