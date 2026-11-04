`timescale 1ns/1ps
// Fixed-raster median -> Sobel integration. A runtime bypass is sampled at
// original SOF by the preprocessor; threshold/mode travel with that same SOF.
// W,H >= 3; H_TOTAL >= W. Accepted source edge N -> final edge N+2*H_TOTAL+18.
module video_median_sobel_link #(
    parameter integer IMAGE_WIDTH=1280,
    parameter integer IMAGE_HEIGHT=720,
    parameter integer H_TOTAL=1650,
    parameter integer COORD_WIDTH=12,
    parameter VS_ACTIVE=1'b0,
    parameter SYNC_IDLE=1'b1
)(
    input wire clk,rst_n,in_hs,in_vs,in_de,
    input wire [7:0] in_gray,
    input wire in_median_enable,
    input wire [10:0] in_threshold,
    input wire [1:0] in_mode,
    output reg out_ready,out_hs,out_vs,out_de,out_sof,out_eof,
    output reg [COORD_WIDTH-1:0] out_x,out_y,
    output reg [7:0] out_gray,
    output reg out_edge,out_border,
    output reg [23:0] out_rgb,
    output reg out_align_error
);
    // Match the existing video_sobel_link reset assertion/release discipline.
    (* async_reg="true" *) reg [1:0] reset_pipe;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) reset_pipe<=2'b00;
        else reset_pipe<={reset_pipe[0],1'b1};
    end
    wire local_rst_n=reset_pipe[1];
    function integer address_bits;
        input integer depth;
        integer value;
        begin
            address_bits=0;value=depth-1;
            while(value>0)begin address_bits=address_bits+1;value=value>>1;end
        end
    endfunction
    localparam integer MEDIAN_DELAY=H_TOTAL+12;
    localparam integer CONTROL_DEPTH=MEDIAN_DELAY+1;
    localparam integer CONTROL_AW=address_bits(CONTROL_DEPTH);
    reg [12:0] control_ram [0:CONTROL_DEPTH-1];
    reg [12:0] delayed_control;
    reg [CONTROL_AW-1:0] control_wr;
    wire [CONTROL_AW-1:0] control_rd=(control_wr==CONTROL_DEPTH-1)?0:control_wr+1'b1;
    // Delay the complete stream, rather than one global frame register: the
    // next frame can arrive while the previous tiny frame is still in flight.
    always @(posedge clk)begin
        if(local_rst_n)begin
            control_ram[control_wr]<={in_threshold,in_mode};
            delayed_control<=control_ram[control_rd];
        end
    end
    always @(posedge clk)begin
        if(!local_rst_n)control_wr<=0;
        else control_wr<=control_rd;
    end
    wire median_ready,median_hs,median_vs,median_de,median_error;
    wire [7:0] median_gray;
    video_median_preprocess #(
        .IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),.H_TOTAL(H_TOTAL),
        .COORD_WIDTH(COORD_WIDTH),.VS_ACTIVE(VS_ACTIVE),.SYNC_IDLE(SYNC_IDLE)
    ) median_inst(
        .clk(clk),.rst_n(local_rst_n),.in_hs(in_hs),.in_vs(in_vs),.in_de(in_de),
        .in_gray(in_gray),.in_enable(in_median_enable),.out_ready(median_ready),
        .out_hs(median_hs),.out_vs(median_vs),.out_de(median_de),.out_sof(),.out_eof(),
        .out_x(),.out_y(),.out_gray(median_gray),.out_border(),.out_align_error(median_error));
    wire ready,hs,vs,de,sof,eof,edge_bit,border,align_error;
    wire [COORD_WIDTH-1:0] x,y;
    wire [7:0] gray;
    wire [23:0] rgb;
    video_sobel_display #(
        .IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),.H_TOTAL(H_TOTAL),
        .COORD_WIDTH(COORD_WIDTH),.VS_ACTIVE(VS_ACTIVE),.SYNC_IDLE(SYNC_IDLE)
    ) sobel_inst(
        .clk(clk),.rst_n(local_rst_n),.in_hs(median_hs),.in_vs(median_vs),.in_de(median_de),
        .in_gray(median_gray),.in_threshold(delayed_control[12:2]),.in_mode(delayed_control[1:0]),
        .out_ready(ready),.out_hs(hs),.out_vs(vs),.out_de(de),.out_sof(sof),.out_eof(eof),
        .out_x(x),.out_y(y),.out_gray(gray),.out_edge(edge_bit),.out_border(border),
        .out_rgb(rgb),.out_align_error(align_error));
    // Carry median diagnostics through the same delay as the Sobel raster.
    localparam integer ERROR_DEPTH=H_TOTAL+5;
    localparam integer ERROR_AW=address_bits(ERROR_DEPTH);
    reg error_ram [0:ERROR_DEPTH-1];
    reg delayed_error;
    reg [ERROR_AW-1:0] error_wr;
    wire [ERROR_AW-1:0] error_rd=(error_wr==ERROR_DEPTH-1)?0:error_wr+1'b1;
    always @(posedge clk)begin
        if(local_rst_n)begin
            error_ram[error_wr]<=median_error;
            delayed_error<=error_ram[error_rd];
        end
    end
    always @(posedge clk)begin
        if(!local_rst_n)error_wr<=0;
        else error_wr<=error_rd;
    end
    localparam integer TOTAL_DELAY=2*H_TOTAL+18;
    localparam integer FILL_AW=address_bits(TOTAL_DELAY+1);
    reg [FILL_AW-1:0] filled;
    wire pipeline_ready=(filled==TOTAL_DELAY);
    always @(posedge clk or negedge local_rst_n)begin
        if(!local_rst_n)begin
            filled<=0;out_ready<=0;out_hs<=SYNC_IDLE;out_vs<=SYNC_IDLE;
            out_de<=0;out_sof<=0;out_eof<=0;out_x<=0;out_y<=0;out_gray<=0;
            out_edge<=0;out_border<=0;out_rgb<=0;out_align_error<=0;
        end else begin
            if(!pipeline_ready)filled<=filled+1'b1;
            out_ready<=pipeline_ready;
            out_hs<=pipeline_ready?hs:SYNC_IDLE;
            out_vs<=pipeline_ready?vs:SYNC_IDLE;
            out_de<=pipeline_ready&&de;
            out_sof<=pipeline_ready&&sof;
            out_eof<=pipeline_ready&&eof;
            out_x<=pipeline_ready?x:0;out_y<=pipeline_ready?y:0;
            out_gray<=pipeline_ready?gray:0;
            out_edge<=pipeline_ready&&edge_bit;
            out_border<=pipeline_ready&&border;
            out_rgb<=pipeline_ready?rgb:0;
            out_align_error<=pipeline_ready&&(align_error||delayed_error);
        end
    end
endmodule
