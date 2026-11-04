`timescale 1ns/1ps
// Fixed-raster grayscale and unthresholded Sobel L1 magnitude, all pixels.
// Borders retain their grayscale value and emit magnitude0. Accepted edge N
// emerges on N+H_TOTAL+4. Every signal, including blanking, has equal delay.
module video_sobel_gradient_stream #(
    parameter integer IMAGE_WIDTH=1024, IMAGE_HEIGHT=600, H_TOTAL=1344,
    parameter integer COORD_WIDTH=12,
    parameter VS_ACTIVE=1'b0, SYNC_IDLE=1'b1
)(
    input wire clk,rst_n,in_hs,in_vs,in_de,
    input wire [7:0] in_gray,
    output wire out_ready,out_hs,out_vs,out_de,out_sof,out_eof,
    output wire [COORD_WIDTH-1:0] out_x,out_y,
    output wire [7:0] out_gray,
    output wire [10:0] out_magnitude,
    output wire out_border,out_align_error
);
    function integer address_bits;
        input integer depth;integer value;
        begin
            address_bits=0;value=depth-1;
            while(value>0)begin address_bits=address_bits+1;value=value>>1;end
        end
    endfunction
    localparam integer DELAY=H_TOTAL+4,BUFFER_DEPTH=DELAY+1;
    localparam integer AW=address_bits(BUFFER_DEPTH);
    reg frame_armed,capture_active;
    reg [COORD_WIDTH-1:0] capture_x,capture_y;
    wire source_sof=frame_armed&&in_de&&(in_vs!=VS_ACTIVE);
    wire source_de=in_de&&(capture_active||source_sof);
    wire [COORD_WIDTH-1:0] sample_x=source_sof?{COORD_WIDTH{1'b0}}:capture_x;
    wire [COORD_WIDTH-1:0] sample_y=source_sof?{COORD_WIDTH{1'b0}}:capture_y;
    always @(posedge clk)begin
        if(!rst_n)begin frame_armed<=0;capture_active<=0;capture_x<=0;capture_y<=0;end
        else begin
            if(in_vs==VS_ACTIVE)frame_armed<=1;
            else if(in_de)frame_armed<=0;
            if(source_de)begin
                if(sample_x==IMAGE_WIDTH-1)begin
                    capture_x<=0;
                    if(sample_y==IMAGE_HEIGHT-1)begin capture_y<=0;capture_active<=0;end
                    else begin capture_y<=sample_y+1'b1;capture_active<=1;end
                end else begin capture_x<=sample_x+1'b1;capture_y<=sample_y;capture_active<=1;end
            end
        end
    end
    reg [11:0] video_ram[0:BUFFER_DEPTH-1];
    reg [11:0] delayed;
    reg [AW-1:0] write_ptr,filled;
    reg ready_q;
    wire [AW-1:0] read_ptr=(write_ptr==BUFFER_DEPTH-1)?{AW{1'b0}}:write_ptr+1'b1;
    always @(posedge clk)begin
        if(rst_n)begin
            video_ram[write_ptr]<={in_hs,in_vs,source_de,source_sof,in_gray};
            delayed<=video_ram[read_ptr];
        end
    end
    always @(posedge clk)begin
        if(!rst_n)begin write_ptr<=0;filled<=0;ready_q<=0;end
        else begin write_ptr<=read_ptr;if(filled<DELAY)filled<=filled+1'b1;ready_q<=(filled==DELAY);end
    end
    assign out_ready=ready_q;
    assign out_hs=ready_q?delayed[11]:SYNC_IDLE;
    assign out_vs=ready_q?delayed[10]:SYNC_IDLE;
    assign out_de=ready_q&&delayed[9];
    assign out_sof=out_de&&delayed[8];
    reg [COORD_WIDTH-1:0] display_x,display_y;
    wire [COORD_WIDTH-1:0] pixel_x=delayed[8]?{COORD_WIDTH{1'b0}}:display_x;
    wire [COORD_WIDTH-1:0] pixel_y=delayed[8]?{COORD_WIDTH{1'b0}}:display_y;
    always @(posedge clk)begin
        if(!rst_n)begin display_x<=0;display_y<=0;end
        else if(out_de)begin
            if(pixel_x==IMAGE_WIDTH-1)begin
                display_x<=0;display_y<=(pixel_y==IMAGE_HEIGHT-1)?{COORD_WIDTH{1'b0}}:pixel_y+1'b1;
            end else begin display_x<=pixel_x+1'b1;display_y<=pixel_y;end
        end
    end
    assign out_x=out_de?pixel_x:{COORD_WIDTH{1'b0}};
    assign out_y=out_de?pixel_y:{COORD_WIDTH{1'b0}};
    assign out_gray=out_de?delayed[7:0]:8'd0;
    assign out_eof=out_de&&(pixel_x==IMAGE_WIDTH-1)&&(pixel_y==IMAGE_HEIGHT-1);
    assign out_border=out_de&&((pixel_x==0)||(pixel_y==0)||(pixel_x==IMAGE_WIDTH-1)||(pixel_y==IMAGE_HEIGHT-1));
    wire edge_valid;
    wire [COORD_WIDTH-1:0] edge_x,edge_y;
    wire [10:0] edge_magnitude;
    edge_detect_gray #(.IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),.COORD_WIDTH(COORD_WIDTH)) edge_inst(
        .clk(clk),.rst_n(rst_n),.in_valid(source_de),.in_sof(source_sof),.in_gray(in_gray),.in_threshold(11'd128),
        .out_valid(edge_valid),.out_sof(),.out_eof(),.out_x(edge_x),.out_y(edge_y),.out_gray(),
        .out_gx(),.out_gy(),.out_magnitude(edge_magnitude),.out_edge());
    wire interior=out_de&&!out_border;
    wire aligned=edge_valid&&(edge_x==pixel_x)&&(edge_y==pixel_y);
    assign out_magnitude=(interior&&aligned)?edge_magnitude:11'd0;
    assign out_align_error=ready_q&&((interior&&!aligned)||(edge_valid&&!interior));
endmodule
