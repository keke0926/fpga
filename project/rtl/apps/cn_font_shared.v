`timescale 1ns/1ps
// Two independent synchronous readers, one immutable font storage instance.
module cn_font_shared #(parameter FONT_FILE="rtl/apps/cn_font16.mem")(
 input wire clk,input wire[9:0]address_a,address_b,
 output reg[15:0]bits_a,bits_b
);
 reg[15:0]memory[0:1023];
 initial $readmemh(FONT_FILE,memory);
 always@(posedge clk)begin bits_a<=memory[address_a];bits_b<=memory[address_b];end
endmodule
