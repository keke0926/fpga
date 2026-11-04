`timescale 1ns/1ps
module video_median_preprocess #(
    parameter integer IMAGE_WIDTH = 1280,
    parameter integer IMAGE_HEIGHT = 720,
    parameter integer H_TOTAL = 1650,
    parameter integer COORD_WIDTH = 12,
    parameter VS_ACTIVE = 1'b0,
    parameter SYNC_IDLE = 1'b1
)(
    input wire clk, rst_n, in_hs, in_vs, in_de,
    input wire [7:0] in_gray,
    input wire in_enable,
    output wire out_ready, out_hs, out_vs, out_de, out_sof, out_eof,
    output wire [COORD_WIDTH-1:0] out_x, out_y,
    output wire [7:0] out_gray,
    output wire out_border, out_align_error
);
    // Fixed raster: W,H >= 3, H_TOTAL >= W, coordinates must fit COORD_WIDTH.
    // Let a center pixel be accepted on edge N. Its bottom-right neighbor is
    // accepted at N+H_TOTAL+1; window3x3 registers that window one edge later;
    // median9 captures it at N+H_TOTAL+3 and emits after nine more periods.
    // Thus ALL raster signals and pixels (including bypass/borders) emerge
    // immediately after edge N+H_TOTAL+12. Blanking is delayed identically.
    // Synchronous low reset cancels pending data; VS must arm a new frame.
    localparam integer DELAY = H_TOTAL + 12;
    localparam integer BUFFER_DEPTH = DELAY + 1;
    function integer address_bits;
        input integer depth;
        integer value;
        begin
            address_bits = 0;
            value = depth-1;
            while (value > 0) begin
                address_bits = address_bits + 1;
                value = value >> 1;
            end
        end
    endfunction
    localparam integer ADDR_WIDTH = address_bits(BUFFER_DEPTH);

    reg frame_armed, capture_active, enable_locked;
    reg [COORD_WIDTH-1:0] capture_x, capture_y;
    wire source_sof = frame_armed && in_de && (in_vs != VS_ACTIVE);
    wire source_de = in_de && (capture_active || source_sof);
    wire [COORD_WIDTH-1:0] sample_x = source_sof ? {COORD_WIDTH{1'b0}} : capture_x;
    wire [COORD_WIDTH-1:0] sample_y = source_sof ? {COORD_WIDTH{1'b0}} : capture_y;
    wire source_enable = source_sof ? in_enable : enable_locked;
    always @(posedge clk) begin
        if (!rst_n) begin
            frame_armed <= 1'b0;
            capture_active <= 1'b0;
            capture_x <= 0;
            capture_y <= 0;
            enable_locked <= 1'b0;
        end else begin
            if (in_vs == VS_ACTIVE) frame_armed <= 1'b1;
            else if (in_de) frame_armed <= 1'b0;
            if (source_sof) enable_locked <= in_enable;
            if (source_de) begin
                if (sample_x == IMAGE_WIDTH-1) begin
                    capture_x <= 0;
                    if (sample_y == IMAGE_HEIGHT-1) begin
                        capture_y <= 0;
                        capture_active <= 1'b0;
                    end else begin
                        capture_y <= sample_y + 1'b1;
                        capture_active <= 1'b1;
                    end
                end else begin
                    capture_x <= sample_x + 1'b1;
                    capture_y <= sample_y;
                    capture_active <= 1'b1;
                end
            end
        end
    end

    // 13-bit packet: HS, VS, normalized DE, SOF, frame-locked enable, raw gray.
    // Include enable in every packet so a new frame cannot change a pending
    // previous frame. Read and write addresses differ, avoiding RAM collisions.
    reg [12:0] video_ram [0:BUFFER_DEPTH-1];
    reg [12:0] delayed;
    reg [ADDR_WIDTH-1:0] write_ptr, filled;
    reg ready_q;
    wire [ADDR_WIDTH-1:0] read_ptr = (write_ptr == BUFFER_DEPTH-1) ?
                                    {ADDR_WIDTH{1'b0}} : write_ptr + 1'b1;
    always @(posedge clk) begin
        if (rst_n) begin
            video_ram[write_ptr] <= {in_hs,in_vs,source_de,source_sof,source_enable,in_gray};
            delayed <= video_ram[read_ptr];
        end
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            write_ptr <= 0;
            filled <= 0;
            ready_q <= 1'b0;
        end else begin
            write_ptr <= read_ptr;
            if (filled < DELAY) filled <= filled + 1'b1;
            ready_q <= (filled == DELAY);
        end
    end
    assign out_ready = ready_q;
    assign out_hs = ready_q ? delayed[12] : SYNC_IDLE;
    assign out_vs = ready_q ? delayed[11] : SYNC_IDLE;
    assign out_de = ready_q && delayed[10];
    assign out_sof = out_de && delayed[9];

    reg [COORD_WIDTH-1:0] display_x, display_y;
    wire [COORD_WIDTH-1:0] pixel_x = delayed[9] ? {COORD_WIDTH{1'b0}} : display_x;
    wire [COORD_WIDTH-1:0] pixel_y = delayed[9] ? {COORD_WIDTH{1'b0}} : display_y;
    always @(posedge clk) begin
        if (!rst_n) begin
            display_x <= 0;
            display_y <= 0;
        end else if (out_de) begin
            if (pixel_x == IMAGE_WIDTH-1) begin
                display_x <= 0;
                display_y <= (pixel_y == IMAGE_HEIGHT-1) ? {COORD_WIDTH{1'b0}} : pixel_y + 1'b1;
            end else begin
                display_x <= pixel_x + 1'b1;
                display_y <= pixel_y;
            end
        end
    end
    assign out_x = out_de ? pixel_x : {COORD_WIDTH{1'b0}};
    assign out_y = out_de ? pixel_y : {COORD_WIDTH{1'b0}};
    assign out_eof = out_de && (pixel_x == IMAGE_WIDTH-1) && (pixel_y == IMAGE_HEIGHT-1);
    assign out_border = out_de && ((pixel_x == 0) || (pixel_y == 0) ||
                                  (pixel_x == IMAGE_WIDTH-1) || (pixel_y == IMAGE_HEIGHT-1));

    wire window_valid, window_sof, window_eof;
    wire [COORD_WIDTH-1:0] window_x, window_y;
    wire [7:0] p00,p01,p02,p10,p11,p12,p20,p21,p22;
    window3x3_gray #(.IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),.COORD_WIDTH(COORD_WIDTH)) window_inst (
        .clk(clk),.rst_n(rst_n),.in_valid(source_de),.in_sof(source_sof),.in_gray(in_gray),
        .out_valid(window_valid),.out_sof(window_sof),.out_eof(window_eof),.out_x(window_x),.out_y(window_y),
        .p00(p00),.p01(p01),.p02(p02),.p10(p10),.p11(p11),.p12(p12),.p20(p20),.p21(p21),.p22(p22));
    wire median_valid;
    wire [COORD_WIDTH-1:0] median_x, median_y;
    wire [7:0] median_gray;
    median9_core #(.COORD_WIDTH(COORD_WIDTH)) median_inst (
        .clk(clk),.rst_n(rst_n),.in_valid(window_valid),.in_sof(window_sof),.in_eof(window_eof),
        .in_x(window_x),.in_y(window_y),.p00(p00),.p01(p01),.p02(p02),.p10(p10),.p11(p11),.p12(p12),
        .p20(p20),.p21(p21),.p22(p22),.out_valid(median_valid),.out_sof(),.out_eof(),
        .out_x(median_x),.out_y(median_y),.out_gray(median_gray));
    wire interior = out_de && !out_border;
    wire aligned = median_valid && (median_x == pixel_x) && (median_y == pixel_y);
    assign out_align_error = ready_q && ((interior && !aligned) || (median_valid && !interior));
    // Preserve every input pixel even on malformed raster timing. An alignment
    // failure is visible and falls back to delayed raw, never stale kernel data.
    assign out_gray = !out_de ? 8'd0 :
                      (delayed[8] && interior && aligned) ? median_gray : delayed[7:0];
endmodule
