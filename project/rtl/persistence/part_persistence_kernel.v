// Active host persistence kernel, radius=2 (5x5), single clock.
// Host retains affine resampling, all admitted poses and their common domain.
// One packed four-row ring is read sequentially: no full reference-frame RAM.
module part_persistence_kernel (
    input wire clk, input wire rst, input wire cancel,
    input wire start, input wire [10:0] width, input wire [9:0] height,
    input wire [31:0] job_tag,
    output reg busy, output reg done, output reg error,
    input wire in_valid, output wire in_ready, input wire [4:0] in_data,
    output reg out_valid, input wire out_ready,
    output reg [5:0] out_data, output reg [10:0] out_x,
    output reg [9:0] out_y, output reg [31:0] out_tag, output reg out_last
);
localparam IDLE=0, TAKE=1, READ0=2, READ1=3, READ2=4, READ3=5, STORE=6, EMIT=7, HOLD=8;
reg [3:0] state;
reg [10:0] frame_width,x;
reg [9:0] frame_height,y;
reg [31:0] tag;
reg [4:0] pixel,old0,old1,old2;
reg [24:0] window_rows[0:4];
// Keep the synchronous memory port outside the reset/cancel branch. Each
// frame's first four rows explicitly suppress stale ring contents.
(* ram_style = "block" *) reg [4:0] row_ring[0:4095];
reg [4:0] read_data;
wire [1:0] read_bank = y[1:0] + (state-READ0);
wire [11:0] read_address = {read_bank,x[9:0]};
wire [11:0] write_address = {y[1:0],x[9:0]};
wire reading = busy && state>=READ0 && state<=READ3 && x<frame_width;
wire writing = busy && state==STORE && x<frame_width;
always @(posedge clk) begin
    if (reading) read_data <= row_ring[read_address];
    if (writing) row_ring[write_address] <= pixel;
end
assign in_ready = busy && state==TAKE && x<frame_width && y<frame_height && !rst && !cancel;
integer r,c;
reg domain,any_support,any_source,any_target;
reg source_query,target_query;
reg [5:0] result;
always @* begin
    domain=1;any_support=0;any_source=0;any_target=0;
    for (r=0;r<5;r=r+1) begin
        for (c=0;c<5;c=c+1) begin
            domain=domain & window_rows[r][c*5];
            any_support=any_support | window_rows[r][c*5+1];
            any_source=any_source | window_rows[r][c*5+2];
            any_target=any_target | window_rows[r][c*5+3];
        end
    end
    source_query=domain & window_rows[2][12];
    target_query=domain & window_rows[2][13];
    result={target_query & any_source,source_query & any_target,
            target_query,source_query,domain & window_rows[2][14] & any_support,domain};
end

integer k;
task advance_pixel;
begin
    if (x==frame_width+1) begin
        x<=0;y<=y+1;
        for (k=0;k<5;k=k+1) window_rows[k]<=0;
    end else x<=x+1;
    state<=TAKE;
end
endtask

always @(posedge clk) begin
    if (rst || cancel) begin
        state<=IDLE;busy<=0;done<=0;error<=0;out_valid<=0;out_last<=0;
        frame_width<=0;frame_height<=0;x<=0;y<=0;tag<=0;
        pixel<=0;old0<=0;old1<=0;old2<=0;
        out_data<=0;out_x<=0;out_y<=0;out_tag<=0;
        for(k=0;k<5;k=k+1) window_rows[k]<=0;
    end else begin
        done<=0;
        case(state)
          IDLE: if(start) begin
            error<=0;out_valid<=0;out_last<=0;
            if(width==0 || width>1024 || height==0 || height>600) begin
                error<=1;done<=1;
            end else begin
                frame_width<=width;frame_height<=height;tag<=job_tag;
                x<=0;y<=0;busy<=1;state<=TAKE;
                for(k=0;k<5;k=k+1) window_rows[k]<=0;
            end
          end
          TAKE: if(x>=frame_width || y>=frame_height || in_valid) begin
            pixel <= (x<frame_width && y<frame_height) ? in_data : 5'b0;
            state<=READ0;
          end
          READ0: state<=READ1;
          READ1: begin old0<=(y>=4 && x<frame_width)?read_data:5'b0;state<=READ2;end
          READ2: begin old1<=(y>=3 && x<frame_width)?read_data:5'b0;state<=READ3;end
          READ3: begin old2<=(y>=2 && x<frame_width)?read_data:5'b0;state<=STORE;end
          STORE: begin
            window_rows[0]<={window_rows[0][19:0],old0};
            window_rows[1]<={window_rows[1][19:0],old1};
            window_rows[2]<={window_rows[2][19:0],old2};
            window_rows[3]<={window_rows[3][19:0],(y>=1 && x<frame_width)?read_data:5'b0};
            window_rows[4]<={window_rows[4][19:0],pixel};
            state<=EMIT;
          end
          EMIT: begin
            if(x>=2 && y>=2) begin
                out_valid<=1;out_data<=result;out_x<=x-2;out_y<=y-2;out_tag<=tag;
                out_last<=x==frame_width+1 && y==frame_height+1;
                state<=HOLD;
            end else advance_pixel();
          end
          HOLD: if(out_ready) begin
            out_valid<=0;
            if(out_last) begin busy<=0;done<=1;state<=IDLE;end
            else advance_pixel();
          end
          default: begin state<=IDLE;busy<=0;out_valid<=0;error<=1;end
        endcase
    end
end
endmodule
