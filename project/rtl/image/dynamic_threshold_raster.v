// No-stall raster statistics for frozen dynamic_20261017_v2 arithmetic.
// Input gray and Sobel-L1 must already refer to the same pixel/coordinate.
// Only the global onepixel Sobel border is excluded. All64 bins are scrubbed
// before reuse; reset never creates a histogram RAM reset mux.
module dynamic_threshold_raster #(
    parameter integer IMAGE_WIDTH=1024,
    parameter integer IMAGE_HEIGHT=600,
    parameter integer TILE_WIDTH=128,
    parameter integer TILE_HEIGHT=64,
    parameter integer COORD_WIDTH=12
) (
    input wire clk,
    input wire rst,
    input wire frame_start,
    input wire frame_end,
    input wire [31:0] frame_id,
    input wire in_valid,
    input wire [COORD_WIDTH-1:0] in_x,
    input wire [COORD_WIDTH-1:0] in_y,
    input wire [7:0] in_gray,
    input wire [10:0] in_magnitude,
    input wire [COORD_WIDTH-1:0] read_x,
    input wire [COORD_WIDTH-1:0] read_y,
    output wire [8:0] read_threshold,
    output wire table_applied_valid,
    output wire statistics_busy,
    output reg statistics_late,
    output wire statistics_ready
);
    function integer width_for;
        input integer value;
        integer n;
        begin n=value-1;width_for=0;
            while(n>0)begin width_for=width_for+1;n=n>>1;end
            if(width_for<1)width_for=1;
        end
    endfunction
    localparam integer TILES_X=(IMAGE_WIDTH+TILE_WIDTH-1)/TILE_WIDTH;
    localparam integer TILES_Y=(IMAGE_HEIGHT+TILE_HEIGHT-1)/TILE_HEIGHT;
    localparam integer TILE_COUNT=TILES_X*TILES_Y;
    localparam integer HIST_WORDS=TILE_COUNT*64;
    localparam integer ADDR_WIDTH=width_for(HIST_WORDS);
    localparam [3:0] INIT=0, IDLE=1, COLLECT=2, DRAIN=3, BEGIN_TABLE=4,
        PREPARE=5, SCAN_REQUEST=6, SCAN_USE=7, DIVIDE=8, UPDATE=9, EMIT=10, COUNT_REQUEST=11, COUNT_USE=12, COUNT_DONE=13;
    reg [3:0] state;
    reg [ADDR_WIDTH-1:0] clear_address;
    reg [13:0] histogram [0:HIST_WORDS-1];
    reg [13:0] hist_q;
    reg [ADDR_WIDTH-1:0] hist_read_address;
    reg hist_write_enable;
    reg [ADDR_WIDTH-1:0] hist_write_address;
    reg [13:0] hist_write_data;
    reg pending_valid;
    reg [ADDR_WIDTH-1:0] pending_address;
    reg previous_write_valid;
    reg [ADDR_WIDTH-1:0] previous_write_address;
    reg [13:0] previous_write_data;
    // Finalization always visits tile 0..TILE_COUNT-1. A serial history ring
    // removes the indexed register file and its write/read muxes. No reset is
    // needed: history_valid is asserted only after an entire ring is replaced.
    // A late SOF in finalization still drains all tiles; a reset invalidates it.
    (* syn_srlstyle = "srl" *) reg [26*TILE_COUNT-1:0] history_ring;
    wire [25:0] history_head=history_ring[26*TILE_COUNT-1-:26];
    wire previous_tile_valid=history_head[25];
    wire [7:0] previous_brightness=history_head[24:17];
    wire [7:0] previous_proxy=history_head[16:9];
    wire [8:0] previous_threshold=history_head[8:0];
    reg history_valid;
    reg [31:0] history_frame_id;
    reg use_previous;
    reg [31:0] collection_frame_id;
    reg cancelled;
    reg invalidate_table;
    reg [15:0] final_tile;
    reg [5:0] bin_index;
    reg [13:0] final_count, rank;
    reg [14:0] cumulative;
    reg median_found;
    reg [21:0] remainder;
    reg [8:0] quotient;
    reg [7:0] brightness, proxy;
    reg [8:0] proposed_threshold;
    integer i;

    wire late_event=frame_start && state!=IDLE && state!=INIT;
    wire accept_frame=frame_start && state==IDLE && !rst;
    wire collecting_pixel=(state==COLLECT || accept_frame) && !late_event && !rst;
    wire interior=in_valid && in_x>0 && in_x<IMAGE_WIDTH-1 && in_y>0 && in_y<IMAGE_HEIGHT-1;
    wire sample_accept=collecting_pixel && interior;
    wire [15:0] input_tile=(in_y/TILE_HEIGHT)*TILES_X+(in_x/TILE_WIDTH);
    wire [5:0] input_bucket=(in_magnitude>=11'd255)?6'd63:in_magnitude[7:2];
    wire [ADDR_WIDTH-1:0] input_address=input_tile*64+input_bucket;
    wire [ADDR_WIDTH-1:0] scan_address=final_tile*64+bin_index;
    wire [13:0] increment_base=(previous_write_valid && previous_write_address==pending_address)?
        previous_write_data:hist_q;
    wire [14:0] scan_total=cumulative+{1'b0,hist_q};
    wire [8:0] brightness_delta=(brightness>=previous_brightness)?
        {1'b0,brightness}-{1'b0,previous_brightness}:
        {1'b0,previous_brightness}-{1'b0,brightness};
    wire [8:0] proxy_delta=(proxy>=previous_proxy)?
        {1'b0,proxy}-{1'b0,previous_proxy}:
        {1'b0,previous_proxy}-{1'b0,proxy};
    wire [7:0] relative_base=(previous_brightness>=8'd32)?previous_brightness:8'd32;
    wire jumped=brightness_delta>=9'd24 || proxy_delta>=9'd8 ||
        {brightness_delta,3'b000}>={4'b0000,relative_base};
    wire [10:0] proxy_target={1'b0,proxy,2'b00}+11'd8;
    wire [10:0] raw_target=(proxy_target>={3'b000,brightness})?proxy_target:{3'b000,brightness};
    wire [8:0] clamped_target=(raw_target<11'd32)?9'd32:((raw_target>11'd256)?9'd256:raw_target[8:0]);
    wire [10:0] blend_sum={2'b00,previous_threshold}+
        {1'b0,previous_threshold,1'b0}+{2'b00,clamped_target}+11'd2;

    wire table_start_ready,table_tile_ready;
    // Register invalidation: a normal SOF changes IDLE->COLLECT on this edge;
    // its still-high input pulse must not mask the newly committed bank after
    // that NBA. A truly late SOF already rejects an incomplete bank in V1;
    // the following pulse clears its transaction provenance as well.
    wire table_geometry_reset=invalidate_table;
    wire table_start_valid=state==BEGIN_TABLE && !cancelled && !late_event;
    wire table_tile_valid=state==EMIT && !cancelled && !late_event;
    wire [15:0] read_tile=(read_y<IMAGE_HEIGHT && read_x<IMAGE_WIDTH)?
        (read_y/TILE_HEIGHT)*TILES_X+(read_x/TILE_WIDTH):16'hffff;
    wire ignored_read_range,ignored_staging_complete,ignored_transaction_error;
    wire [31:0] ignored_active_id;
    dynamic_threshold_frame_table #(.TILE_COUNT(TILE_COUNT)) frame_table(
        .clk(clk),.rst(rst),.geometry_reset(table_geometry_reset),.frame_start(frame_start),.frame_id(frame_id),
        .update_start_valid(table_start_valid),.update_start_ready(table_start_ready),.update_frame_id(collection_frame_id),
        .tile_update_valid(table_tile_valid),.tile_update_ready(table_tile_ready),.tile_frame_id(collection_frame_id),
        .tile_index(final_tile),.tile_threshold(proposed_threshold),.read_tile_index(read_tile),
        .read_threshold(read_threshold),.read_in_range(ignored_read_range),.applied_valid(table_applied_valid),
        .staging_complete(ignored_staging_complete),.transaction_error(ignored_transaction_error),.active_frame_id(ignored_active_id));
    // Raster visits each horizontal tile in order. Keep only a running sum
    // plus one previous-row sum per column; completed tiles form a serial ring.
    // First interior row seeds zero, so the SRL storage needs no reset network.
    wire [20:0] final_gray_sum;
    wire sum_emit=state==EMIT && (cancelled || late_event || table_tile_ready);
    generate if(TILE_WIDTH>1 && TILE_HEIGHT>1 && IMAGE_WIDTH>2 && IMAGE_HEIGHT>2 &&
                IMAGE_WIDTH%TILE_WIDTH!=1 && IMAGE_HEIGHT%TILE_HEIGHT!=1)begin:SerialGray
        (* syn_srlstyle="srl" *) reg [21*TILES_X-1:0] columns;
        (* syn_srlstyle="srl" *) reg [21*TILE_COUNT-1:0] completed;
        reg [20:0] run_sum;
        wire first_column_pixel=in_x==1 || in_x%TILE_WIDTH==0;
        wire last_column_pixel=in_x==IMAGE_WIDTH-2 || in_x%TILE_WIDTH==TILE_WIDTH-1;
        wire first_tile_row=in_y==1 || in_y%TILE_HEIGHT==0;
        wire last_tile_row=in_y==IMAGE_HEIGHT-2 || in_y%TILE_HEIGHT==TILE_HEIGHT-1;
        wire [20:0] previous_column=first_tile_row?21'd0:columns[21*TILES_X-1-:21];
        wire [20:0] next_sum=(first_column_pixel?previous_column:run_sum)+{13'd0,in_gray};
        assign final_gray_sum=completed[21*TILE_COUNT-1-:21];
        always@(posedge clk)begin
            if(sample_accept)begin
                run_sum<=next_sum;
                if(last_column_pixel)begin
                    columns<=(columns<<21)|next_sum;
                end
            end
            // Consumed tail values are immaterial. One uniform shift assignment
            // allows Efinity to infer SRLs instead of 1680 individual FFs.
            if((sample_accept && last_column_pixel && last_tile_row) || (!rst && sum_emit))
                completed<=(completed<<21)|next_sum;
        end
    end else begin:BoundaryOnlyTiles
        // Keep the parameterized behavior for a geometry containing empty
        // border-only tiles. This branch is absent from the 1024x600 product.
        reg[20:0] sums[0:TILE_COUNT-1];integer si;
        assign final_gray_sum=sums[final_tile];
        always@(posedge clk)begin
            if(rst || (state==COLLECT && late_event))begin
                for(si=0;si<TILE_COUNT;si=si+1)sums[si]<=0;
            end else begin
                if(sample_accept)sums[input_tile]<=sums[input_tile]+{13'd0,in_gray};
                if(sum_emit)sums[final_tile]<=0;
            end
        end
    end endgenerate

    assign statistics_busy=state!=IDLE && state!=COLLECT;
    assign statistics_ready=state==IDLE && !rst;

    always @(posedge clk)begin
        if(!rst && state==EMIT && (cancelled || late_event || table_tile_ready))
            history_ring<=(history_ring<<26)|{(final_count!=0),brightness,proxy,proposed_threshold};
    end

    // Explicit synchronous read+write memory, with a single ownership point.
    // Forward only the immediately preceding physical RAM write: read-during-
    // write mode is thus irrelevant for consecutive same-bin RMW samples.
    always @* begin
        hist_read_address=input_address;
        hist_write_enable=1'b0;
        hist_write_address=pending_address;
        hist_write_data=increment_base+14'd1;
        if(state==INIT)begin
            hist_read_address=clear_address;
            hist_write_enable=!rst;
            hist_write_address=clear_address;
            hist_write_data=14'd0;
        end else if(state==SCAN_REQUEST || state==SCAN_USE || state==COUNT_REQUEST || state==COUNT_USE)begin
            hist_read_address=scan_address;
            if(state==SCAN_USE)begin
                hist_write_enable=!rst;
                hist_write_address=scan_address;
                hist_write_data=14'd0;
            end
        end else if(pending_valid && !rst && !late_event)begin
            hist_write_enable=1'b1;
        end
    end
    always @(posedge clk)begin
        hist_q<=histogram[hist_read_address];
        if(hist_write_enable)histogram[hist_write_address]<=hist_write_data;
    end

    always @(posedge clk)begin
        if(rst)begin
            state<=INIT;clear_address<=0;pending_valid<=0;previous_write_valid<=0;
            pending_address<=0;previous_write_address<=0;previous_write_data<=0;
            history_valid<=0;history_frame_id<=0;use_previous<=0;collection_frame_id<=0;
            cancelled<=0;invalidate_table<=0;statistics_late<=0;final_tile<=0;bin_index<=0;
            final_count<=0;rank<=0;cumulative<=0;median_found<=0;remainder<=0;quotient<=0;
            brightness<=0;proxy<=0;proposed_threshold<=9'd128;
        end else begin
            invalidate_table<=late_event || (frame_start && state==INIT);
            pending_valid<=sample_accept;
            if(sample_accept)begin
                pending_address<=input_address;
            end
            previous_write_valid<=hist_write_enable;
            if(hist_write_enable)begin
                previous_write_address<=hist_write_address;
                previous_write_data<=hist_write_data;
            end
            if(late_event)begin
                statistics_late<=1'b1;
                cancelled<=1'b1;
                history_valid<=1'b0;
                use_previous<=1'b0;
            end
            case(state)
                INIT:begin
                    pending_valid<=0;previous_write_valid<=0;
                    if(clear_address==HIST_WORDS-1)begin
                        state<=IDLE;clear_address<=0;cancelled<=0;
                    end else clear_address<=clear_address+1'b1;
                end
                IDLE:if(frame_start)begin
                    collection_frame_id<=frame_id;
                    use_previous<=history_valid && frame_id==history_frame_id+32'd1;
                    if(!(history_valid && frame_id==history_frame_id+32'd1))history_valid<=0;
                    cancelled<=0;
                    if(frame_end)state<=DRAIN;
                    else state<=COLLECT;
                end
                COLLECT:begin
                    if(late_event)begin
                        state<=INIT;clear_address<=0;pending_valid<=0;previous_write_valid<=0;
                    end else if(frame_end)state<=DRAIN;
                end
                DRAIN:begin
                    pending_valid<=0;previous_write_valid<=0;final_tile<=0;
                    state<=BEGIN_TABLE;
                end
                BEGIN_TABLE:if(cancelled || late_event || table_start_ready)state<=PREPARE;
                // Reuse histogram counts instead of an 80x14 indexed register file.
                // First pass sums without clearing; second pass keeps the original
                // median scan and clear order. No changes to the threshold formula.
                PREPARE:begin
                    final_count<=0;bin_index<=0;state<=COUNT_REQUEST;
                end
                COUNT_REQUEST:state<=COUNT_USE;
                COUNT_USE:begin
                    final_count<=final_count+hist_q;
                    if(bin_index==6'd63)state<=COUNT_DONE;
                    else begin bin_index<=bin_index+6'd1;state<=COUNT_REQUEST;end
                end
                COUNT_DONE:begin
                    remainder<={1'b0,final_gray_sum}+{8'd0,final_count}-22'd1;
                    quotient<=0;brightness<=0;proxy<=0;
                    rank<=(final_count+14'd1)>>1;
                    cumulative<=0;bin_index<=0;median_found<=0;
                    state<=SCAN_REQUEST;
                end
                SCAN_REQUEST:state<=SCAN_USE;
                SCAN_USE:begin
                    cumulative<=scan_total;
                    if(!median_found && final_count!=0 && scan_total>={1'b0,rank})begin
                        proxy<={bin_index,2'b11};median_found<=1;
                    end
                    if(bin_index==6'd63)state<=DIVIDE;
                    else begin bin_index<=bin_index+6'd1;state<=SCAN_REQUEST;end
                end
                DIVIDE:begin
                    if(final_count==0)state<=UPDATE;
                    else if(remainder>={8'd0,final_count})begin
                        remainder<=remainder-{8'd0,final_count};quotient<=quotient+9'd1;
                    end else begin brightness<=quotient[7:0];state<=UPDATE;end
                end
                UPDATE:begin
                    if(final_count==0)proposed_threshold<=9'd128;
                    else if(!use_previous || !previous_tile_valid || jumped)proposed_threshold<=clamped_target;
                    else proposed_threshold<=blend_sum[10:2];
                    state<=EMIT;
                end
                EMIT:if(cancelled || late_event || table_tile_ready)begin
                    if(final_tile==TILE_COUNT-1)begin
                        if(!cancelled && !late_event)begin history_valid<=1;history_frame_id<=collection_frame_id;end
                        state<=IDLE;
                    end else begin final_tile<=final_tile+16'd1;state<=PREPARE;end
                end
                default:begin state<=INIT;clear_address<=0;history_valid<=0;pending_valid<=0;end
            endcase
        end
    end
endmodule
