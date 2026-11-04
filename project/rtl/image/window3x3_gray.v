`timescale 1ns/1ps
module window3x3_gray #(
    parameter integer IMAGE_WIDTH = 1280,
    parameter integer IMAGE_HEIGHT = 720,
    parameter integer COORD_WIDTH = 12
)(
    input wire clk, rst_n, in_valid, in_sof,
    input wire [7:0] in_gray,
    output reg out_valid, out_sof, out_eof,
    output reg [COORD_WIDTH-1:0] out_x, out_y,
    output reg [7:0] p00,p01,p02,p10,p11,p12,p20,p21,p22
);
    // Contract: dimensions >= 3 and representable by COORD_WIDTH. A valid
    // SOF starts/restarts a fixed-size raster. No backpressure is required.
    reg active;
    reg [COORD_WIDTH-1:0] next_x, next_y;
    wire accept = in_valid && (active || in_sof);
    wire [COORD_WIDTH-1:0] sample_x = in_sof ? {COORD_WIDTH{1'b0}} : next_x;
    wire [COORD_WIDTH-1:0] sample_y = in_sof ? {COORD_WIDTH{1'b0}} : next_y;

    // At a column, high byte is two rows ago; low byte is one row ago.
    // Synchronous read, followed by write-back on the next clock. Avoid a
    // RAM reset: the first two incoming rows overwrite stale/unknown data.
    reg [15:0] rows [0:IMAGE_WIDTH-1];
    reg [15:0] old_rows_q;
    reg [7:0] gray_q;
    reg read_valid;
    reg [COORD_WIDTH-1:0] read_x, read_y;
    always @(posedge clk) begin
        if (rst_n) begin
            if (accept) old_rows_q <= rows[sample_x];
            if (read_valid) rows[read_x] <= {old_rows_q[7:0], gray_q};
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            active <= 1'b0;
            next_x <= 0;
            next_y <= 0;
            read_valid <= 1'b0;
            read_x <= 0;
            read_y <= 0;
            gray_q <= 0;
        end else begin
            read_valid <= accept;
            if (accept) begin
                read_x <= sample_x;
                read_y <= sample_y;
                gray_q <= in_gray;
                if (sample_x == IMAGE_WIDTH-1) begin
                    next_x <= 0;
                    if (sample_y == IMAGE_HEIGHT-1) begin
                        next_y <= 0;
                        active <= 1'b0;
                    end else begin
                        next_y <= sample_y + 1'b1;
                        active <= 1'b1;
                    end
                end else begin
                    next_x <= sample_x + 1'b1;
                    next_y <= sample_y;
                    active <= 1'b1;
                end
            end
        end
    end

    reg [7:0] top_left, top_mid, mid_left, mid_mid, bot_left, bot_mid;
    always @(posedge clk) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            out_sof <= 1'b0;
            out_eof <= 1'b0;
            out_x <= 0;
            out_y <= 0;
            {p00,p01,p02,p10,p11,p12,p20,p21,p22} <= 0;
            {top_left,top_mid,mid_left,mid_mid,bot_left,bot_mid} <= 0;
        end else begin
            out_valid <= 1'b0;
            out_sof <= 1'b0;
            out_eof <= 1'b0;
            if (read_valid) begin
                // Drain accepted tokens even when the current input stalls
                // or starts a new frame. Only reset cancels the pipeline.
                top_left <= (read_x == 0) ? 8'd0 : top_mid;
                mid_left <= (read_x == 0) ? 8'd0 : mid_mid;
                bot_left <= (read_x == 0) ? 8'd0 : bot_mid;
                top_mid <= old_rows_q[15:8];
                mid_mid <= old_rows_q[7:0];
                bot_mid <= gray_q;
                if (read_x >= 2 && read_y >= 2) begin
                    out_valid <= 1'b1;
                    out_sof <= (read_x == 2 && read_y == 2);
                    out_eof <= (read_x == IMAGE_WIDTH-1 && read_y == IMAGE_HEIGHT-1);
                    out_x <= read_x - 1'b1;
                    out_y <= read_y - 1'b1;
                    {p00,p01,p02} <= {top_left,top_mid,old_rows_q[15:8]};
                    {p10,p11,p12} <= {mid_left,mid_mid,old_rows_q[7:0]};
                    {p20,p21,p22} <= {bot_left,bot_mid,gray_q};
                end
            end
        end
    end
endmodule
