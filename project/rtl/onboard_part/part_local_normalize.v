`timescale 1ns/1ps
// Scalar Q4 local normalization, no private RAM and no UART dependency.
// Inputs are the center of a median3 image and exact 31x31 sums around it.
// Windows/DDR/ownership are supplied by the future shared tile scheduler.
module part_local_normalize(
 input wire clk, rst_n, abort_i,
 input wire in_valid_i, output wire in_ready_o,
 input wire [7:0] center_i,
 input wire magnitude_i,
 input wire [17:0] sum_i,
 input wire [25:0] square_sum_i,
 output wire out_valid_o, input wire out_ready_i,
 output reg signed [7:0] normalized_o,
 output reg saturated_o, low_contrast_o
);
 localparam IDLE=0, VARIANCE=1, ROOT=2, DIVIDE=3, OUTPUT=4;
 reg [2:0] state;
 reg magnitude;
 reg [35:0] moment_product, sum_square, radicand;
 reg signed [18:0] centered;
 reg [37:0] root_remainder;
 reg [17:0] root, denominator;
 reg [4:0] step;
 reg [22:0] dividend, quotient;
 reg [18:0] divide_remainder;
 wire [17:0] weighted_center = center_i * 10'd961;
 wire [35:0] variance_value = moment_product >= sum_square ? moment_product-sum_square : 36'd0;
 wire [37:0] root_shift = {root_remainder[35:0],radicand[35:34]};
 wire [37:0] root_trial = {18'd0,root,2'b01};
 wire root_bit = root_shift >= root_trial;
 wire [17:0] root_next = {root[16:0],root_bit};
 wire [17:0] denominator_next = root_next < 18'd7688 ? 18'd7688 : root_next;
 wire [18:0] centered_abs = centered[18] ? -centered : centered;
 wire [22:0] rounded_numerator = {centered_abs,4'b0} + (denominator_next >> 1);
 wire [18:0] divide_shift = {divide_remainder[17:0],dividend[22]};
 wire divide_bit = divide_shift >= {1'b0,denominator};
 wire [22:0] quotient_next = {quotient[21:0],divide_bit};
 assign in_ready_o = rst_n && !abort_i && state==IDLE;
 assign out_valid_o = rst_n && !abort_i && state==OUTPUT;
 always @(posedge clk) begin
  if(!rst_n || abort_i) begin
   state<=IDLE;normalized_o<=0;saturated_o<=0;low_contrast_o<=0;magnitude<=0;
   moment_product<=0;sum_square<=0;radicand<=0;centered<=0;
   root_remainder<=0;root<=0;denominator<=0;step<=0;
   dividend<=0;quotient<=0;divide_remainder<=0;
  end else begin
   case(state)
    IDLE:if(in_valid_i)begin
     magnitude<=magnitude_i;
     moment_product<=magnitude_i ? {10'd0,square_sum_i} : square_sum_i*10'd961;
     sum_square<=magnitude_i ? 36'd0 : sum_i*sum_i;
     centered<=$signed({1'b0,weighted_center})-$signed({1'b0,sum_i});
     state<=VARIANCE;
    end
    VARIANCE:begin
     radicand<=variance_value;root_remainder<=0;root<=0;step<=18;
     low_contrast_o<=variance_value<36'd59105344;
     state<=ROOT;
    end
    ROOT:begin
     root_remainder<=root_bit ? root_shift-root_trial : root_shift;
     root<=root_next;radicand<=radicand<<2;step<=step-1'b1;
     if(step==1)begin
      denominator<=denominator_next;dividend<=rounded_numerator;
      quotient<=0;divide_remainder<=0;step<=23;
      if(magnitude)begin
       normalized_o<=root_next>255 ? 8'hff : root_next[7:0];
       saturated_o<=root_next>255;state<=OUTPUT;
      end else state<=DIVIDE;
     end
    end
    DIVIDE:begin
     divide_remainder<=divide_bit ? divide_shift-{1'b0,denominator} : divide_shift;
     quotient<=quotient_next;dividend<=dividend<<1;step<=step-1'b1;
     if(step==1)begin
      saturated_o<=quotient_next>127;
      if(quotient_next>127)normalized_o<=centered[18]?8'h81:8'h7f;
      else normalized_o<=centered[18]?(8'd0-{1'b0,quotient_next[6:0]}):{1'b0,quotient_next[6:0]};
      state<=OUTPUT;
     end
    end
    OUTPUT:if(out_ready_i)state<=IDLE;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
