`timescale 1ns/1ps
module sobel3x3_core #(
    parameter integer COORD_WIDTH = 12
)(
    input wire clk, rst_n, in_valid, in_sof, in_eof,
    input wire [COORD_WIDTH-1:0] in_x, in_y,
    input wire [10:0] in_threshold,
    input wire [7:0] p00,p01,p02,p10,p11,p12,p20,p21,p22,
    output reg out_valid, out_sof, out_eof,
    output reg [COORD_WIDTH-1:0] out_x, out_y,
    output reg [7:0] out_gray,
    output reg signed [10:0] out_gx, out_gy,
    output reg [10:0] out_magnitude,
    output reg out_edge
);
    // Extend BEFORE arithmetic: every weighted side is in 0..1020,
    // differences in -1020..1020. Explicit widths avoid 8-bit truncation.
    wire [10:0] right_sum = {3'b0,p02} + {2'b0,p12,1'b0} + {3'b0,p22};
    wire [10:0] left_sum  = {3'b0,p00} + {2'b0,p10,1'b0} + {3'b0,p20};
    wire [10:0] bottom_sum = {3'b0,p20} + {2'b0,p21,1'b0} + {3'b0,p22};
    wire [10:0] top_sum    = {3'b0,p00} + {2'b0,p01,1'b0} + {3'b0,p02};
    reg signed [10:0] gx_q, gy_q;
    reg valid_q, sof_q, eof_q;
    reg [COORD_WIDTH-1:0] x_q, y_q;
    reg [7:0] gray_q;
    reg [10:0] threshold_q;
    wire [10:0] abs_gx = gx_q[10] ? (~gx_q + 11'd1) : gx_q;
    wire [10:0] abs_gy = gy_q[10] ? (~gy_q + 11'd1) : gy_q;
    wire [10:0] magnitude = abs_gx + abs_gy;

    always @(posedge clk) begin
        if (!rst_n) begin
            valid_q <= 1'b0;
            sof_q <= 1'b0;
            eof_q <= 1'b0;
            gx_q <= 0;
            gy_q <= 0;
            x_q <= 0;
            y_q <= 0;
            gray_q <= 0;
            threshold_q <= 0;
            out_valid <= 1'b0;
            out_sof <= 1'b0;
            out_eof <= 1'b0;
            out_x <= 0;
            out_y <= 0;
            out_gray <= 0;
            out_gx <= 0;
            out_gy <= 0;
            out_magnitude <= 0;
            out_edge <= 1'b0;
        end else begin
            valid_q <= in_valid;
            if (in_valid) begin
                gx_q <= $signed(right_sum) - $signed(left_sum);
                gy_q <= $signed(bottom_sum) - $signed(top_sum);
                x_q <= in_x;
                y_q <= in_y;
                gray_q <= p11;
                sof_q <= in_sof;
                eof_q <= in_eof;
                threshold_q <= in_threshold;
            end
            out_valid <= valid_q;
            out_sof <= valid_q && sof_q;
            out_eof <= valid_q && eof_q;
            if (valid_q) begin
                out_x <= x_q;
                out_y <= y_q;
                out_gray <= gray_q;
                out_gx <= gx_q;
                out_gy <= gy_q;
                out_magnitude <= magnitude;
                out_edge <= (magnitude >= threshold_q);
            end
        end
    end
endmodule
