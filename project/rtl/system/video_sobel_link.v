`timescale 1ns/1ps
module video_sobel_link #(
    parameter integer IMAGE_WIDTH = 1280,
    parameter integer IMAGE_HEIGHT = 720,
    parameter integer H_TOTAL = 1650,
    parameter integer COORD_WIDTH = 12,
    parameter VS_ACTIVE = 1'b0,
    parameter SYNC_IDLE = 1'b1
)(
    input wire clk, rst_n, in_hs, in_vs, in_de,
    input wire [7:0] in_gray,
    input wire [10:0] in_threshold,
    input wire [1:0] in_mode,
    output reg out_ready, out_hs, out_vs, out_de, out_sof, out_eof,
    output reg [COORD_WIDTH-1:0] out_x, out_y,
    output reg [7:0] out_gray,
    output reg out_edge, out_border,
    output reg [23:0] out_rgb,
    output reg out_align_error
);
    // Asynchronous assertion, two complete clock edges for release. The
    // attribute keeps the two synchronization registers distinct and close.
    (* async_reg = "true" *) reg [1:0] reset_pipe;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) reset_pipe <= 2'b00;
        else reset_pipe <= {reset_pipe[0],1'b1};
    end
    wire local_rst_n = reset_pipe[1];
    wire ready, hs, vs, de, sof, eof, edge_bit, border, align_error;
    wire [COORD_WIDTH-1:0] x, y;
    wire [7:0] gray;
    wire [23:0] rgb;
    video_sobel_display #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT), .H_TOTAL(H_TOTAL),
        .COORD_WIDTH(COORD_WIDTH), .VS_ACTIVE(VS_ACTIVE), .SYNC_IDLE(SYNC_IDLE)
    ) u_video (
        .clk(clk), .rst_n(local_rst_n), .in_hs(in_hs), .in_vs(in_vs), .in_de(in_de),
        .in_gray(in_gray), .in_threshold(in_threshold), .in_mode(in_mode),
        .out_ready(ready), .out_hs(hs), .out_vs(vs), .out_de(de), .out_sof(sof), .out_eof(eof),
        .out_x(x), .out_y(y), .out_gray(gray), .out_edge(edge_bit), .out_border(border),
        .out_rgb(rgb), .out_align_error(align_error)
    );

    // Keep all control, data and diagnostics on the same output edge.
    // In steady operation this adds one clock: center N -> N+H_TOTAL+5.
    always @(posedge clk or negedge local_rst_n) begin
        if (!local_rst_n) begin
            out_ready <= 1'b0;
            out_hs <= SYNC_IDLE;
            out_vs <= SYNC_IDLE;
            out_de <= 1'b0;
            out_sof <= 1'b0;
            out_eof <= 1'b0;
            out_x <= 0;
            out_y <= 0;
            out_gray <= 0;
            out_edge <= 1'b0;
            out_border <= 1'b0;
            out_rgb <= 0;
            out_align_error <= 1'b0;
        end else begin
            out_ready <= ready;
            out_hs <= hs;
            out_vs <= vs;
            out_de <= de;
            out_sof <= sof;
            out_eof <= eof;
            out_x <= x;
            out_y <= y;
            out_gray <= gray;
            out_edge <= edge_bit;
            out_border <= border;
            out_rgb <= rgb;
            out_align_error <= align_error;
        end
    end
endmodule
