`timescale 1ns/1ps
module edge_detect_gray #(
    parameter integer IMAGE_WIDTH = 1280,
    parameter integer IMAGE_HEIGHT = 720,
    parameter integer COORD_WIDTH = 12
)(
    input wire clk, rst_n, in_valid, in_sof,
    input wire [7:0] in_gray,
    input wire [10:0] in_threshold,
    output wire out_valid, out_sof, out_eof,
    output wire [COORD_WIDTH-1:0] out_x, out_y,
    output wire [7:0] out_gray,
    output wire signed [10:0] out_gx, out_gy,
    output wire [10:0] out_magnitude,
    output wire out_edge
);
    // Output coordinates refer to the CENTER, not the currently arriving pixel.
    // Only interior pixels are emitted: display borders/timing belong outside.
    wire win_valid, win_sof, win_eof;
    wire [COORD_WIDTH-1:0] win_x, win_y;
    wire [7:0] p00,p01,p02,p10,p11,p12,p20,p21,p22;
    window3x3_gray #(
        .IMAGE_WIDTH(IMAGE_WIDTH), .IMAGE_HEIGHT(IMAGE_HEIGHT), .COORD_WIDTH(COORD_WIDTH)
    ) window_inst (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid), .in_sof(in_sof), .in_gray(in_gray),
        .out_valid(win_valid), .out_sof(win_sof), .out_eof(win_eof), .out_x(win_x), .out_y(win_y),
        .p00(p00),.p01(p01),.p02(p02),.p10(p10),.p11(p11),.p12(p12),.p20(p20),.p21(p21),.p22(p22)
    );

    // Input accepted at N -> window registered at N+1 -> core samples at N+2.
    // Therefore threshold needs TWO physical registers before the core. Shift
    // every clock, including invalid bubbles; valid masks the associated data.
    reg [10:0] threshold_d1, threshold_d2;
    always @(posedge clk) begin
        if (!rst_n) begin
            threshold_d1 <= 0;
            threshold_d2 <= 0;
        end else begin
            threshold_d1 <= in_threshold;
            threshold_d2 <= threshold_d1;
        end
    end
    sobel3x3_core #(.COORD_WIDTH(COORD_WIDTH)) sobel_inst (
        .clk(clk), .rst_n(rst_n), .in_valid(win_valid), .in_sof(win_sof), .in_eof(win_eof),
        .in_x(win_x), .in_y(win_y), .in_threshold(threshold_d2),
        .p00(p00),.p01(p01),.p02(p02),.p10(p10),.p11(p11),.p12(p12),.p20(p20),.p21(p21),.p22(p22),
        .out_valid(out_valid), .out_sof(out_sof), .out_eof(out_eof), .out_x(out_x), .out_y(out_y),
        .out_gray(out_gray), .out_gx(out_gx), .out_gy(out_gy),
        .out_magnitude(out_magnitude), .out_edge(out_edge)
    );
endmodule
