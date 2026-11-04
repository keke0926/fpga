`timescale 1ns/1ps
// Full-word variant of the proven two-row synchronous-read window. A window
// centered on (x,y) appears H_TOTAL+2 clocks after that center was accepted,
// assuming a fixed raster. Metadata may be included in WORD_WIDTH.
module window3x3_word #(
    parameter integer IMAGE_WIDTH = 1024,
    parameter integer IMAGE_HEIGHT = 600,
    parameter integer COORD_WIDTH = 12,
    parameter integer WORD_WIDTH = 8
)(
    input wire clk, rst_n, in_valid, in_sof,
    input wire [COORD_WIDTH-1:0] in_x, in_y,
    input wire [WORD_WIDTH-1:0] in_word,
    output reg out_valid,
    output reg [COORD_WIDTH-1:0] out_x, out_y,
    output reg [WORD_WIDTH-1:0] p00,p01,p02,p10,p11,p12,p20,p21,p22
);
    // W,H >=3. Explicit coordinates belong to accepted raster tokens. No
    // backpressure: consecutive rows/frames are supported with zero blanking.
    reg active;
    wire accept = in_valid && (active || in_sof) &&
                  (in_x < IMAGE_WIDTH) && (in_y < IMAGE_HEIGHT);
    reg [2*WORD_WIDTH-1:0] rows [0:IMAGE_WIDTH-1];
    reg [2*WORD_WIDTH-1:0] old_rows_q;
    reg [WORD_WIDTH-1:0] word_q;
    reg read_valid;
    reg [COORD_WIDTH-1:0] read_x, read_y;
    // No RAM reset. Rows 0 and 1 overwrite stale frame/reset data before any
    // window is exposed. At W>=3 the next column differs from read_x, including
    // zero-blanking row/frame wrap, so simultaneous read/write never collides.
    always @(posedge clk) begin
        if (rst_n) begin
            if (accept) old_rows_q <= rows[in_x];
            if (read_valid) rows[read_x] <= {old_rows_q[WORD_WIDTH-1:0],word_q};
        end
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            active <= 1'b0;
            read_valid <= 1'b0;
            read_x <= 0; read_y <= 0; word_q <= 0;
        end else begin
            read_valid <= accept;
            if (accept) begin
                read_x <= in_x; read_y <= in_y; word_q <= in_word;
                active <= !((in_x == IMAGE_WIDTH-1) && (in_y == IMAGE_HEIGHT-1));
            end
        end
    end
    reg [WORD_WIDTH-1:0] top_left,top_mid,mid_left,mid_mid,bot_left,bot_mid;
    always @(posedge clk) begin
        if (!rst_n) begin
            out_valid <= 1'b0; out_x <= 0; out_y <= 0;
            {p00,p01,p02,p10,p11,p12,p20,p21,p22} <= 0;
            {top_left,top_mid,mid_left,mid_mid,bot_left,bot_mid} <= 0;
        end else begin
            out_valid <= 1'b0;
            if (read_valid) begin
                top_left <= (read_x == 0) ? {WORD_WIDTH{1'b0}} : top_mid;
                mid_left <= (read_x == 0) ? {WORD_WIDTH{1'b0}} : mid_mid;
                bot_left <= (read_x == 0) ? {WORD_WIDTH{1'b0}} : bot_mid;
                top_mid <= old_rows_q[2*WORD_WIDTH-1:WORD_WIDTH];
                mid_mid <= old_rows_q[WORD_WIDTH-1:0];
                bot_mid <= word_q;
                if ((read_x >= 2) && (read_y >= 2)) begin
                    out_valid <= 1'b1;
                    out_x <= read_x-1'b1; out_y <= read_y-1'b1;
                    {p00,p01,p02} <= {top_left,top_mid,old_rows_q[2*WORD_WIDTH-1:WORD_WIDTH]};
                    {p10,p11,p12} <= {mid_left,mid_mid,old_rows_q[WORD_WIDTH-1:0]};
                    {p20,p21,p22} <= {bot_left,bot_mid,word_q};
                end
            end
        end
    end
endmodule
