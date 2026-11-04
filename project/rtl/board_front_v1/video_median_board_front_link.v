`timescale 1ns/1ps
// Board-test fixed frontend: original causal threshold/median stream followed
// by finite NMS+local weak support. Original sources and protocol unchanged.
// Base delay2*H_TOTAL+19, frontend3*H_TOTAL+8 plus one module boundary clock.
// Full source input N -> final N+5*H_TOTAL+28.
module video_median_board_front_link #(
    parameter integer IMAGE_WIDTH=1024, IMAGE_HEIGHT=600, H_TOTAL=1344,
    parameter integer TILE_WIDTH=128,TILE_HEIGHT=64,COORD_WIDTH=12,
    parameter VS_ACTIVE=1'b0,SYNC_IDLE=1'b1
)(
    input wire clk,rst_n,in_hs,in_vs,in_de,
    input wire [7:0] in_gray,
    input wire in_median_enable,in_auto_enable,
    input wire [10:0] in_threshold,
    input wire [1:0] in_mode,
    output wire out_ready,out_hs,out_vs,out_de,out_sof,out_eof,
    output wire [COORD_WIDTH-1:0] out_x,out_y,
    output wire [7:0] out_gray,
    output wire [10:0] out_magnitude,out_threshold,
    output wire out_auto_enable,out_edge,out_border,
    output wire out_strong,out_weak,
    output wire [23:0] out_rgb,
    output wire out_align_error,out_table_applied_valid,
    output wire out_statistics_busy,out_statistics_late,out_statistics_ready
);
    (* async_reg="true" *) reg [1:0] reset_pipe;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)reset_pipe<=0;
        else reset_pipe<={reset_pipe[0],1'b1};
    end
    wire local_rst_n=reset_pipe[1];
    function integer address_bits;
        input integer depth;integer value;
        begin address_bits=0;value=depth-1;
            while(value>0)begin address_bits=address_bits+1;value=value>>1;end
        end
    endfunction

    // Original-SOF capture independently mirrors the proven fixed-raster
    // median input acceptance. Midframe input-control changes have no effect.
    reg frame_armed,capture_active;
    reg [COORD_WIDTH-1:0] capture_x,capture_y;
    reg [14:0] locked_control;
    wire source_sof=frame_armed&&in_de&&(in_vs!=VS_ACTIVE);
    wire source_de=in_de&&(capture_active||source_sof);
    wire [COORD_WIDTH-1:0] sample_x=source_sof?{COORD_WIDTH{1'b0}}:capture_x;
    wire [COORD_WIDTH-1:0] sample_y=source_sof?{COORD_WIDTH{1'b0}}:capture_y;
    wire [14:0] source_control=source_sof?{in_median_enable,in_auto_enable,in_threshold,in_mode}:locked_control;
    always @(posedge clk)begin
        if(!local_rst_n)begin frame_armed<=0;capture_active<=0;capture_x<=0;capture_y<=0;locked_control<=15'd514;end
        else begin
            if(in_vs==VS_ACTIVE)frame_armed<=1;else if(in_de)frame_armed<=0;
            if(source_sof)locked_control<=source_control;
            if(source_de)begin
                if(sample_x==IMAGE_WIDTH-1)begin capture_x<=0;
                    if(sample_y==IMAGE_HEIGHT-1)begin capture_y<=0;capture_active<=0;end
                    else begin capture_y<=sample_y+1'b1;capture_active<=1;end
                end else begin capture_x<=sample_x+1'b1;capture_y<=sample_y;capture_active<=1;end
            end
        end
    end
    localparam integer GRADIENT_DELAY=2*H_TOTAL+17;
    localparam integer CONTROL_DEPTH=GRADIENT_DELAY+1;
    localparam integer CONTROL_AW=address_bits(CONTROL_DEPTH);
    reg [14:0] control_ram[0:CONTROL_DEPTH-1];
    reg [14:0] delayed_control;
    reg [CONTROL_AW-1:0] control_wr;
    wire [CONTROL_AW-1:0] control_rd=(control_wr==CONTROL_DEPTH-1)?{CONTROL_AW{1'b0}}:control_wr+1'b1;
    always @(posedge clk)begin
        if(local_rst_n)begin control_ram[control_wr]<=source_control;delayed_control<=control_ram[control_rd];end
    end
    always @(posedge clk)begin if(!local_rst_n)control_wr<=0;else control_wr<=control_rd;end

    wire median_hs,median_vs,median_de,median_error;
    wire [7:0] median_gray;
    video_median_preprocess #(.IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),.H_TOTAL(H_TOTAL),
      .COORD_WIDTH(COORD_WIDTH),.VS_ACTIVE(VS_ACTIVE),.SYNC_IDLE(SYNC_IDLE)) median_inst(
        .clk(clk),.rst_n(local_rst_n),.in_hs(in_hs),.in_vs(in_vs),.in_de(in_de),.in_gray(in_gray),
        .in_enable(in_median_enable),.out_ready(),.out_hs(median_hs),.out_vs(median_vs),.out_de(median_de),
        .out_sof(),.out_eof(),.out_x(),.out_y(),.out_gray(median_gray),.out_border(),.out_align_error(median_error));
    wire gradient_ready,hs,vs,de,sof,eof,border,gradient_error;
    wire [COORD_WIDTH-1:0] x,y;
    wire [7:0] gray;
    wire [10:0] magnitude;
    wire [1:0] direction;
    video_sobel_gradient_direction_stream #(.IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),.H_TOTAL(H_TOTAL),
      .COORD_WIDTH(COORD_WIDTH),.VS_ACTIVE(VS_ACTIVE),.SYNC_IDLE(SYNC_IDLE)) gradient_inst(
        .clk(clk),.rst_n(local_rst_n),.in_hs(median_hs),.in_vs(median_vs),.in_de(median_de),.in_gray(median_gray),
        .out_ready(gradient_ready),.out_hs(hs),.out_vs(vs),.out_de(de),.out_sof(sof),.out_eof(eof),
        .out_x(x),.out_y(y),.out_gray(gray),.out_magnitude(magnitude),.out_direction(direction),.out_border(border),.out_align_error(gradient_error));
    localparam integer ERROR_DEPTH=H_TOTAL+5;
    localparam integer ERROR_AW=address_bits(ERROR_DEPTH);
    reg error_ram[0:ERROR_DEPTH-1];reg delayed_error;
    reg [ERROR_AW-1:0] error_wr;
    wire [ERROR_AW-1:0] error_rd=(error_wr==ERROR_DEPTH-1)?{ERROR_AW{1'b0}}:error_wr+1'b1;
    always @(posedge clk)begin if(local_rst_n)begin error_ram[error_wr]<=median_error;delayed_error<=error_ram[error_rd];end end
    always @(posedge clk)begin if(!local_rst_n)error_wr<=0;else error_wr<=error_rd;end

    reg [31:0] frame_counter;
    reg have_median,previous_median;
    wire median_changed=de&&sof&&have_median&&(delayed_control[14]!=previous_median);
    always @(posedge clk)begin
        if(!local_rst_n)begin frame_counter<=0;have_median<=0;previous_median<=0;end
        else if(de&&sof)begin frame_counter<=frame_counter+1'b1;have_median<=1;previous_median<=delayed_control[14];end
    end
    reg q_hs,q_vs,q_de,q_sof,q_eof,q_border,q_error;
    reg [COORD_WIDTH-1:0] q_x,q_y;
    reg [7:0] q_gray;
    reg [10:0] q_magnitude;
    reg [1:0] q_direction;
    reg [14:0] q_control;
    wire [8:0] read_threshold;
    wire table_valid;
    dynamic_threshold_raster #(.IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),.TILE_WIDTH(TILE_WIDTH),
      .TILE_HEIGHT(TILE_HEIGHT),.COORD_WIDTH(COORD_WIDTH)) dynamic_inst(
        .clk(clk),.rst(!local_rst_n||median_changed),.frame_start(de&&sof),.frame_end(de&&eof),.frame_id(frame_counter),
        .in_valid(de),.in_x(x),.in_y(y),.in_gray(gray),.in_magnitude(magnitude),.read_x(x),.read_y(y),
        .read_threshold(read_threshold),.table_applied_valid(table_valid),.statistics_busy(out_statistics_busy),
        .statistics_late(out_statistics_late),.statistics_ready(out_statistics_ready));
    // Synchronous table read and q_* capture the same source coordinate.
    // The table selects the committing bank at SOF, preserving first-pixel timing.
    always @(posedge clk or negedge local_rst_n)begin
        if(!local_rst_n)begin
            q_hs<=SYNC_IDLE;q_vs<=SYNC_IDLE;q_de<=0;q_sof<=0;q_eof<=0;q_border<=0;q_error<=0;
            q_x<=0;q_y<=0;q_gray<=0;q_magnitude<=0;q_direction<=0;q_control<=15'd514;
        end else begin
            q_hs<=hs;q_vs<=vs;q_de<=de;q_sof<=sof;q_eof<=eof;q_border<=border;
            q_error<=gradient_error||delayed_error;q_x<=x;q_y<=y;q_gray<=gray;q_magnitude<=magnitude;q_direction<=direction;q_control<=delayed_control;
        end
    end
    wire [10:0] chosen_threshold=q_control[13]?{2'b00,read_threshold}:q_control[12:2];
    wire chosen_edge=q_de&&!q_border&&(q_magnitude>=chosen_threshold);
    wire [23:0] gray_rgb={q_gray,q_gray,q_gray};
    wire [23:0] chosen_rgb=(q_control[1:0]==2'd1)?(chosen_edge?24'hffffff:24'd0):
        ((q_control[1:0]==2'd2&&chosen_edge)?24'h00ff00:gray_rgb);
    localparam integer TOTAL_DELAY=2*H_TOTAL+19;
    localparam integer FILL_AW=address_bits(TOTAL_DELAY+1);
    reg [FILL_AW-1:0] filled;
    wire pipeline_ready=(filled==TOTAL_DELAY);
    reg base_ready,base_hs,base_vs,base_de,base_sof,base_eof;
    reg [COORD_WIDTH-1:0] base_x,base_y;
    reg [7:0] base_gray;
    reg [10:0] base_magnitude,base_threshold;
    reg [1:0] base_direction,base_mode;
    reg base_median,base_auto_enable,base_edge,base_border;
    reg [23:0] base_rgb;
    reg base_align_error,base_table_applied_valid;
    always @(posedge clk or negedge local_rst_n)begin
        if(!local_rst_n)begin
            filled<=0;base_ready<=0;base_hs<=SYNC_IDLE;base_vs<=SYNC_IDLE;base_de<=0;base_sof<=0;base_eof<=0;
            base_x<=0;base_y<=0;base_gray<=0;base_magnitude<=0;base_direction<=0;base_mode<=0;base_median<=0;base_threshold<=128;base_auto_enable<=0;
            base_edge<=0;base_border<=0;base_rgb<=0;base_align_error<=0;base_table_applied_valid<=0;
        end else begin
            if(!pipeline_ready)filled<=filled+1'b1;
            base_ready<=pipeline_ready;base_hs<=pipeline_ready?q_hs:SYNC_IDLE;base_vs<=pipeline_ready?q_vs:SYNC_IDLE;
            base_de<=pipeline_ready&&q_de;base_sof<=pipeline_ready&&q_sof;base_eof<=pipeline_ready&&q_eof;
            base_x<=(pipeline_ready&&q_de)?q_x:0;base_y<=(pipeline_ready&&q_de)?q_y:0;
            base_gray<=(pipeline_ready&&q_de)?q_gray:0;
            base_direction<=(pipeline_ready&&q_de)?q_direction:0;
            base_mode<=(pipeline_ready&&q_de)?q_control[1:0]:0;
            base_median<=pipeline_ready&&q_de&&q_control[14];base_magnitude<=(pipeline_ready&&q_de)?q_magnitude:0;
            base_threshold<=(pipeline_ready&&q_de)?chosen_threshold:11'd128;
            base_auto_enable<=pipeline_ready&&q_de&&q_control[13];base_edge<=pipeline_ready&&chosen_edge;
            base_border<=pipeline_ready&&q_border;base_rgb<=(pipeline_ready&&q_de)?chosen_rgb:24'd0;
            base_align_error<=pipeline_ready&&q_error;base_table_applied_valid<=pipeline_ready&&q_de&&table_valid;
        end
    end

    video_nms_weak_stream #(.IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),.H_TOTAL(H_TOTAL),
      .COORD_WIDTH(COORD_WIDTH),.VS_ACTIVE(VS_ACTIVE),.SYNC_IDLE(SYNC_IDLE)) front_inst(
        .clk(clk),.rst_n(local_rst_n),.in_ready(base_ready),
        .in_hs(base_hs),.in_vs(base_vs),.in_de(base_de),.in_sof(base_sof),.in_eof(base_eof),
        .in_x(base_x),.in_y(base_y),.in_gray(base_gray),.in_magnitude(base_magnitude),
        .in_direction(base_direction),.in_threshold(base_threshold),.in_mode(base_mode),
        .in_median_enable(base_median),.in_auto_enable(base_auto_enable),.in_border(base_border),
        .in_align_error(base_align_error),.in_table_applied_valid(base_table_applied_valid),
        .out_ready(out_ready),.out_hs(out_hs),.out_vs(out_vs),.out_de(out_de),.out_sof(out_sof),.out_eof(out_eof),
        .out_x(out_x),.out_y(out_y),.out_gray(out_gray),.out_magnitude(out_magnitude),.out_threshold(out_threshold),
        .out_mode(),.out_median_enable(),.out_auto_enable(out_auto_enable),.out_border(out_border),
        .out_align_error(out_align_error),.out_table_applied_valid(out_table_applied_valid),
        .out_strong(out_strong),.out_weak(out_weak),.out_edge(out_edge),.out_rgb(out_rgb));
endmodule
