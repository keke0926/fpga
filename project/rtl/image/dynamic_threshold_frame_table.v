// Complete immediately-previous-frame dynamic threshold table only.
// Storage/commit core, not raster statistics scheduling or a video pipeline.
// Dynamic values32..256 do not change the separate manual UART0..2047 path.
module dynamic_threshold_frame_table #(
    parameter integer TILE_COUNT = 80
) (
    input wire clk,
    input wire rst,
    input wire geometry_reset,
    input wire frame_start,
    input wire [31:0] frame_id,
    input wire update_start_valid,
    output wire update_start_ready,
    input wire [31:0] update_frame_id,
    input wire tile_update_valid,
    output wire tile_update_ready,
    input wire [31:0] tile_frame_id,
    input wire [15:0] tile_index,
    input wire [8:0] tile_threshold,
    input wire [15:0] read_tile_index,
    output wire [8:0] read_threshold,
    output wire read_in_range,
    output reg applied_valid,
    output reg staging_complete,
    output reg transaction_error,
    output reg [31:0] active_frame_id
);
    // Synchronous read fits the existing q-pixel stage in both frontends.
    // A single RAM contains both banks. No pixel or threshold arithmetic changes.
    localparam AW=(TILE_COUNT<2)?1:$clog2(TILE_COUNT);
    reg[8:0] memory[0:(1<<(AW+1))-1];
    reg[8:0] read_word;reg read_available;
    reg applied_bank, staging_bank;
    reg have_frame, collecting;
    reg [31:0] staging_frame_id;
    reg [TILE_COUNT-1:0] seen;
    reg [16:0] accepted_count;
    wire [31:0] expected_frame_id = active_frame_id + 32'd1;
    assign update_start_ready = have_frame && !frame_start && !rst && !geometry_reset;
    assign tile_update_ready = collecting && !staging_complete && !update_start_valid &&
        !frame_start && !rst && !geometry_reset;
    assign read_in_range = (read_tile_index < TILE_COUNT);
    wire commit=have_frame && frame_id==expected_frame_id && staging_complete &&
        staging_frame_id==active_frame_id && !transaction_error;
    wire read_bank=frame_start?staging_bank:applied_bank;
    wire read_valid=frame_start?commit:applied_valid;
    wire accepted_write=tile_update_valid && tile_update_ready &&
        tile_frame_id==active_frame_id && tile_frame_id==staging_frame_id &&
        tile_index<TILE_COUNT && tile_threshold>=32 && tile_threshold<=256 && !seen[tile_index];
    always@(posedge clk)begin
        read_word<=memory[{read_bank,read_tile_index[AW-1:0]}];
        read_available<=!rst&&!geometry_reset&&read_valid&&read_in_range;
        if(accepted_write)memory[{staging_bank,tile_index[AW-1:0]}]<=tile_threshold;
    end
    assign read_threshold=read_available&&!rst&&!geometry_reset?read_word:9'd128;
    always @(posedge clk) begin
        if (rst || geometry_reset) begin
            applied_bank <= 1'b0;
            staging_bank <= 1'b1;
            have_frame <= 1'b0;
            active_frame_id <= 32'd0;
            staging_frame_id <= 32'd0;
            collecting <= 1'b0;
            staging_complete <= 1'b0;
            transaction_error <= 1'b0;
            accepted_count <= 17'd0;
            seen <= {TILE_COUNT{1'b0}};
            applied_valid <= 1'b0;
        end else if (frame_start) begin
            // Only state already complete before this edge may be committed.
            if (have_frame && frame_id == expected_frame_id && staging_complete &&
                staging_frame_id == active_frame_id && !transaction_error) begin
                applied_bank <= staging_bank;
                staging_bank <= !staging_bank;
                applied_valid <= 1'b1;
            end else begin
                applied_valid <= 1'b0;
            end
            have_frame <= 1'b1;
            active_frame_id <= frame_id;
            collecting <= 1'b0;
            staging_complete <= 1'b0;
            transaction_error <= 1'b0;
            accepted_count <= 17'd0;
            seen <= {TILE_COUNT{1'b0}};
        end else if (update_start_valid && update_start_ready) begin
            staging_frame_id <= update_frame_id;
            seen <= {TILE_COUNT{1'b0}};
            accepted_count <= 17'd0;
            staging_complete <= 1'b0;
            if (update_frame_id == active_frame_id) begin
                collecting <= 1'b1;
                transaction_error <= 1'b0;
            end else begin
                collecting <= 1'b0;
                transaction_error <= 1'b1;
            end
        end else if (tile_update_valid && tile_update_ready) begin
            if (tile_frame_id != active_frame_id || tile_frame_id != staging_frame_id ||
                tile_index >= TILE_COUNT || tile_threshold < 9'd32 || tile_threshold > 9'd256) begin
                collecting <= 1'b0;
                staging_complete <= 1'b0;
                transaction_error <= 1'b1;
            end else if (seen[tile_index]) begin
                collecting <= 1'b0;
                staging_complete <= 1'b0;
                transaction_error <= 1'b1;
            end else begin
                // Physical RAM write is gated by accepted_write above.
                seen[tile_index] <= 1'b1;
                accepted_count <= accepted_count + 17'd1;
                if (accepted_count == TILE_COUNT-1) begin
                    collecting <= 1'b0;
                    staging_complete <= 1'b1;
                end
            end
        end
    end
endmodule
