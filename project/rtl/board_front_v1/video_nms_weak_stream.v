`timescale 1ns/1ps
module video_nms_weak_stream #(
    parameter integer IMAGE_WIDTH = 1024,
    parameter integer IMAGE_HEIGHT = 600,
    parameter integer H_TOTAL = 1344,
    parameter integer COORD_WIDTH = 12,
    parameter VS_ACTIVE = 1'b0,
    parameter SYNC_IDLE = 1'b1
)(
    input wire clk, rst_n, in_ready, in_hs, in_vs, in_de, in_sof, in_eof,
    input wire [COORD_WIDTH-1:0] in_x, in_y,
    input wire [7:0] in_gray,
    input wire [10:0] in_magnitude,
    input wire [1:0] in_direction,
    input wire [10:0] in_threshold,
    input wire [1:0] in_mode,
    input wire in_median_enable,in_auto_enable,in_border,in_align_error,in_table_applied_valid,
    output wire out_ready,out_hs,out_vs,out_de,out_sof,out_eof,
    output wire [COORD_WIDTH-1:0] out_x,out_y,
    output wire [7:0] out_gray,
    output wire [10:0] out_magnitude,out_threshold,
    output wire [1:0] out_mode,
    output wire out_median_enable,out_auto_enable,out_border,out_align_error,out_table_applied_valid,
    output wire out_strong,out_weak,out_edge,
    output wire [23:0] out_rgb
);
    // NMS and two local support windows preserve a fixed raster delay.
    // Reset cancels pending tokens without resetting the line memories.
    localparam integer FIRST_DELAY = H_TOTAL+2;
    // Packet: ready,HS,VS,DE,SOF,EOF,x,y,gray,magnitude,threshold,mode,
    // median,auto,border,error,table. Every control is owned by its center.
    localparam integer PACKET_WIDTH = 43+2*COORD_WIDTH;
    wire [PACKET_WIDTH-1:0] source_packet = {
        in_ready,in_hs,in_vs,in_de,in_sof,in_eof,in_x,in_y,in_gray,
        in_magnitude,in_threshold,in_mode,in_median_enable,in_auto_enable,
        in_border,in_align_error,in_table_applied_valid};
    wire [PACKET_WIDTH-1:0] first_packet,support_packet;
    wire first_filled,support_filled;
    wire support_delay_filled,first_delayed_filled;
    assign support_filled = support_delay_filled && first_delayed_filled;
    nms_fixed_word_delay #(.WIDTH(PACKET_WIDTH),.DELAY(FIRST_DELAY)) first_delay_inst (
        .clk(clk),.rst_n(rst_n),.in_word(source_packet),.out_filled(first_filled),.out_word(first_packet));
    // Forward the first delay's packet instead of duplicating its storage.
    nms_fixed_word_delay #(.WIDTH(PACKET_WIDTH+1),.DELAY(FIRST_DELAY)) support_delay_inst (
        .clk(clk),.rst_n(rst_n),.in_word({first_filled,first_packet}),
        .out_filled(support_delay_filled),.out_word({first_delayed_filled,support_packet}));
    wire f_ready,f_hs,f_vs,f_de,f_sof,f_eof,f_med,f_auto,f_border,f_error,f_table;
    wire [COORD_WIDTH-1:0] f_x,f_y;
    wire [7:0] f_gray;
    wire [10:0] f_mag,f_threshold;
    wire [1:0] f_mode;
    assign {f_ready,f_hs,f_vs,f_de,f_sof,f_eof,f_x,f_y,f_gray,f_mag,f_threshold,
            f_mode,f_med,f_auto,f_border,f_error,f_table} = first_packet;
    wire s_ready,s_hs,s_vs,s_de,s_sof,s_eof,s_med,s_auto,s_border,s_error,s_table;
    wire [COORD_WIDTH-1:0] s_x,s_y;
    wire [7:0] s_gray;
    wire [10:0] s_mag,s_threshold;
    wire [1:0] s_mode;
    assign {s_ready,s_hs,s_vs,s_de,s_sof,s_eof,s_x,s_y,s_gray,s_mag,s_threshold,
            s_mode,s_med,s_auto,s_border,s_error,s_table} = support_packet;

    // Upstream Sobel requires radius1, median+Sobel requires radius2. The
    // supplied border/error flags may invalidate additional observed contexts.
    wire radius1 = (in_x >= 1) && (in_y >= 1) &&
                   (in_x < IMAGE_WIDTH-1) && (in_y < IMAGE_HEIGHT-1);
    wire radius2 = (in_x >= 2) && (in_y >= 2) &&
                   (in_x < IMAGE_WIDTH-2) && (in_y < IMAGE_HEIGHT-2);
    wire gradient_valid = in_ready && in_de && !in_border && !in_align_error &&
                          (in_median_enable ? radius2 : radius1);
    wire nms_window_valid;
    wire [COORD_WIDTH-1:0] nms_x,nms_y;
    wire [24:0] a00,a01,a02,a10,a11,a12,a20,a21,a22;
    window3x3_word #(.IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),
        .COORD_WIDTH(COORD_WIDTH),.WORD_WIDTH(25)) nms_window_inst (
        .clk(clk),.rst_n(rst_n),.in_valid(in_ready && in_de),.in_sof(in_sof),
        .in_x(in_x),.in_y(in_y),.in_word({gradient_valid,in_threshold,in_direction,in_magnitude}),
        .out_valid(nms_window_valid),.out_x(nms_x),.out_y(nms_y),
        .p00(a00),.p01(a01),.p02(a02),.p10(a10),.p11(a11),.p12(a12),.p20(a20),.p21(a21),.p22(a22));
    wire first_active = first_filled && f_ready && f_de;
    wire first_interior = first_active && (f_x>=1) && (f_y>=1) &&
                          (f_x<IMAGE_WIDTH-1) && (f_y<IMAGE_HEIGHT-1);
    wire nms_aligned = nms_window_valid && (nms_x==f_x) && (nms_y==f_y);
    wire first_error = first_interior && !nms_aligned;
    wire nms_context_valid = nms_aligned && a00[24] && a01[24] && a02[24] &&
        a10[24] && a11[24] && a12[24] && a20[24] && a21[24] && a22[24];
    reg [10:0] negative_mag,positive_mag;
    always @* begin
        case (a11[12:11])
            2'd0: begin negative_mag=a10[10:0]; positive_mag=a12[10:0]; end
            2'd1: begin negative_mag=a00[10:0]; positive_mag=a22[10:0]; end
            2'd2: begin negative_mag=a01[10:0]; positive_mag=a21[10:0]; end
            default: begin negative_mag=a02[10:0]; positive_mag=a20[10:0]; end
        endcase
    end
    wire nms_kept = nms_context_valid && (a11[10:0]!=0) &&
                    (a11[10:0]>=negative_mag) && (a11[10:0]>positive_mag);
    // W includes S. The 12-bit addition protects T=2047 from 11-bit overflow.
    wire [11:0] threshold_plus_one = {1'b0,a11[23:13]}+12'd1;
    wire [10:0] low_threshold = (a11[23:13]==0) ? 11'd1 : threshold_plus_one[11:1];
    wire nms_strong = nms_kept && (a11[10:0]>=a11[23:13]);
    wire nms_weak_candidate = nms_kept && (a11[10:0]>=low_threshold);
    // Crucial: store PRE-final-guard W/S here. A neighbor just outside the
    // final radius3/4 border remains eligible evidence for an interior center.
    wire support_window_valid;
    wire [COORD_WIDTH-1:0] support_x,support_y;
    wire [3:0] b00,b01,b02,b10,b11,b12,b20,b21,b22;
    window3x3_word #(.IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),
        .COORD_WIDTH(COORD_WIDTH),.WORD_WIDTH(4)) support_window_inst (
        .clk(clk),.rst_n(rst_n),.in_valid(first_active),.in_sof(f_sof),
        .in_x(f_x),.in_y(f_y),.in_word({first_error,nms_context_valid,nms_weak_candidate,nms_strong}),
        .out_valid(support_window_valid),.out_x(support_x),.out_y(support_y),
        .p00(b00),.p01(b01),.p02(b02),.p10(b10),.p11(b11),.p12(b12),.p20(b20),.p21(b21),.p22(b22));
    wire support_aligned = support_window_valid && (support_x==s_x) && (support_y==s_y);
    wire support_context_valid = support_aligned && b00[2] && b01[2] && b02[2] &&
        b10[2] && b11[2] && b12[2] && b20[2] && b21[2] && b22[2];
    wire [3:0] weak_neighbor_count = {3'd0,b00[1]} + {3'd0,b01[1]} +
        {3'd0,b02[1]} + {3'd0,b10[1]} + {3'd0,b12[1]} +
        {3'd0,b20[1]} + {3'd0,b21[1]} + {3'd0,b22[1]};
    wire neighbor_strong = b00[0] || b01[0] || b02[0] || b10[0] || b12[0] ||
                           b20[0] || b21[0] || b22[0];
    wire neighbor_weak = (weak_neighbor_count >= 4'd2) || neighbor_strong;
    wire s_active = support_filled && s_ready && s_de;
    wire s_interior = s_active && (s_x>=1) && (s_y>=1) &&
                      (s_x<IMAGE_WIDTH-1) && (s_y<IMAGE_HEIGHT-1);
    wire s_align_error = s_error || (s_interior && !support_aligned) ||
                         (s_active && support_aligned && b11[3]);
    wire s_strong = s_active && support_context_valid && b11[0];
    wire s_weak = s_active && support_context_valid && b11[1] && !b11[0] && neighbor_weak;

    // Seed weak contours with support at least 135 degrees apart. A one-pixel
    // neighborhood around each seed retains weak endpoints and shallow curves.
    // Strong pixels also seed support; isolated strong decisions are unchanged.
    wire [7:0] ring = {b00[1],b10[1],b20[1],b21[1],b22[1],b12[1],b02[1],b01[1]};
    wire [7:0] turn_three = {ring[2:0],ring[7:3]};
    wire [7:0] turn_four = {ring[3:0],ring[7:4]};
    wire seed = s_strong || (s_weak && |(ring & (turn_three | turn_four)));
    wire seed_valid;
    wire [COORD_WIDTH-1:0] seed_x,seed_y;
    wire [8:0] seeds;
    window3x3_word #(.IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),
        .COORD_WIDTH(COORD_WIDTH),.WORD_WIDTH(1)) seed_window (
        .clk(clk),.rst_n(rst_n),.in_valid(s_active),.in_sof(s_sof),
        .in_x(s_x),.in_y(s_y),.in_word(seed),
        .out_valid(seed_valid),.out_x(seed_x),.out_y(seed_y),
        .p00(seeds[0]),.p01(seeds[1]),.p02(seeds[2]),.p10(seeds[3]),.p11(seeds[4]),
        .p12(seeds[5]),.p20(seeds[6]),.p21(seeds[7]),.p22(seeds[8]));

    // Center flags travel with all raster fields through the added window.
    wire [PACKET_WIDTH-1:0] final_packet;
    wire delayed_filled,final_filled,final_context,final_error,final_strong,final_weak;
    wire support_delayed_filled;
    assign final_filled = delayed_filled && support_delayed_filled;
    nms_fixed_word_delay #(.WIDTH(PACKET_WIDTH+5),.DELAY(FIRST_DELAY)) final_delay_inst (
        .clk(clk),.rst_n(rst_n),
        .in_word({support_filled,support_packet,support_context_valid,s_align_error,s_strong,s_weak}),
        .out_filled(delayed_filled),
        .out_word({support_delayed_filled,final_packet,final_context,final_error,final_strong,final_weak}));
    wire d_ready,d_hs,d_vs,d_de,d_sof,d_eof,d_med,d_auto,d_border,d_error,d_table;
    wire [COORD_WIDTH-1:0] d_x,d_y;
    wire [7:0] d_gray;
    wire [10:0] d_mag,d_threshold;
    wire [1:0] d_mode;
    assign {d_ready,d_hs,d_vs,d_de,d_sof,d_eof,d_x,d_y,d_gray,d_mag,d_threshold,
            d_mode,d_med,d_auto,d_border,d_error,d_table} = final_packet;

    assign out_ready = final_filled && d_ready;
    assign out_hs = final_filled ? d_hs : SYNC_IDLE;
    assign out_vs = final_filled ? d_vs : SYNC_IDLE;
    assign out_de = out_ready && d_de;
    assign out_sof = out_de && d_sof;
    assign out_eof = out_de && d_eof;
    assign out_x = final_filled ? d_x : {COORD_WIDTH{1'b0}};
    assign out_y = final_filled ? d_y : {COORD_WIDTH{1'b0}};
    assign out_gray = final_filled ? d_gray : 8'd0;
    assign out_magnitude = final_filled ? d_mag : 11'd0;
    assign out_threshold = final_filled ? d_threshold : 11'd0;
    assign out_mode = final_filled ? d_mode : 2'd0;
    assign out_median_enable = final_filled && d_med;
    assign out_auto_enable = final_filled && d_auto;
    assign out_table_applied_valid = final_filled && d_table;
    assign out_border = out_de && !final_context;
    wire final_interior = out_de && (d_x>=1) && (d_y>=1) &&
                          (d_x<IMAGE_WIDTH-1) && (d_y<IMAGE_HEIGHT-1);
    wire seed_aligned = seed_valid && (seed_x==d_x) && (seed_y==d_y);
    assign out_align_error = final_filled && (final_error || (final_interior && !seed_aligned));
    assign out_strong = out_de && final_strong;
    assign out_weak = out_de && final_weak && seed_aligned && |seeds;
    assign out_edge = out_strong || out_weak;
    wire [23:0] gray_rgb = {out_gray,out_gray,out_gray};
    assign out_rgb = !out_de ? 24'd0 : (out_mode==0) ? gray_rgb :
                     out_strong ? ((out_mode==1) ? 24'hffffff : 24'h00ff00) :
                     out_weak ? 24'hffb000 : (out_mode==1) ? 24'd0 : gray_rgb;
endmodule

// A synchronous circular packet delay with distinct read/write addresses.
// No RAM reset. The fill counter masks stale RAM contents until DELAY clocks
// of fresh data have passed, independently of input DE/ready and blanking.
module nms_fixed_word_delay #(
    parameter integer WIDTH=8,
    parameter integer DELAY=8
)(
    input wire clk,rst_n,
    input wire [WIDTH-1:0] in_word,
    output reg out_filled,
    output reg [WIDTH-1:0] out_word
);
    function integer address_bits;
        input integer depth;
        integer value;
        begin
            address_bits=0; value=depth-1;
            while (value>0) begin address_bits=address_bits+1; value=value>>1; end
        end
    endfunction
    localparam integer DEPTH=DELAY+1;
    localparam integer ADDR_WIDTH=address_bits(DEPTH);
    reg [WIDTH-1:0] packets[0:DEPTH-1];
    reg [ADDR_WIDTH-1:0] write_ptr,filled;
    wire [ADDR_WIDTH-1:0] read_ptr=(write_ptr==DEPTH-1) ? {ADDR_WIDTH{1'b0}} : write_ptr+1'b1;
    always @(posedge clk) begin
        if (rst_n) begin
            packets[write_ptr] <= in_word;
            out_word <= packets[read_ptr];
        end
    end
    always @(posedge clk) begin
        if (!rst_n) begin write_ptr<=0; filled<=0; out_filled<=1'b0; end
        else begin
            write_ptr<=read_ptr;
            if (filled<DELAY) filled<=filled+1'b1;
            out_filled<=(filled==DELAY);
        end
    end
endmodule
