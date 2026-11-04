`timescale 1ns/1ps
// Synchronous 1024x16 ROM. No reset and no large combinational glyph cases.
module cn_font_rom #(parameter FONT_FILE="rtl/apps/cn_font16.mem")(
 input wire clk,input wire[5:0] glyph,input wire[3:0] row,
 output reg[15:0] bits);
 reg[15:0] memory[0:1023];
 initial $readmemh(FONT_FILE,memory);
 always @(posedge clk) bits<=memory[{glyph,row}];
endmodule
