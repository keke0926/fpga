`timescale 1ns/1ps
module median9_core #(
    parameter integer COORD_WIDTH = 12
)(
    input wire clk, rst_n, in_valid, in_sof, in_eof,
    input wire [COORD_WIDTH-1:0] in_x, in_y,
    input wire [7:0] p00,p01,p02,p10,p11,p12,p20,p21,p22,
    output wire out_valid, out_sof, out_eof,
    output wire [COORD_WIDTH-1:0] out_x, out_y,
    output wire [7:0] out_gray
);
    // Pure nine-pixel kernel: no line RAM, raster reconstruction or bypass.
    // One input capture register plus nine registered odd/even sorting layers.
    // Input accepted on rising edge N -> output after edge N+9 (9 periods).
    // All 36 compare/exchange nodes operate concurrently across pipeline
    // layers; initiation interval is one clock, including valid bubbles.
    // Synchronous active-low reset cancels every pending token.
    reg [71:0] values [0:9];
    reg [9:0] valid_pipe, sof_pipe, eof_pipe;
    reg [COORD_WIDTH-1:0] x_pipe [0:9], y_pipe [0:9];
    integer i;

    always @(posedge clk) begin
        if (!rst_n) begin
            values[0] <= 72'd0;
            valid_pipe <= 10'd0;
            sof_pipe <= 10'd0;
            eof_pipe <= 10'd0;
            for (i=0; i<=9; i=i+1) begin
                x_pipe[i] <= 0;
                y_pipe[i] <= 0;
            end
        end else begin
            valid_pipe <= {valid_pipe[8:0], in_valid};
            if (in_valid) begin
                values[0] <= {p22,p21,p20,p12,p11,p10,p02,p01,p00};
                x_pipe[0] <= in_x;
                y_pipe[0] <= in_y;
                sof_pipe[0] <= in_sof;
                eof_pipe[0] <= in_eof;
            end
            for (i=1; i<=9; i=i+1) begin
                if (valid_pipe[i-1]) begin
                    x_pipe[i] <= x_pipe[i-1];
                    y_pipe[i] <= y_pipe[i-1];
                    sof_pipe[i] <= sof_pipe[i-1];
                    eof_pipe[i] <= eof_pipe[i-1];
                end
            end
        end
    end

    genvar layer, pair_index;
    generate
        for (layer=0; layer<9; layer=layer+1) begin: Sort_Layer
            // Even layers: (0,1),(2,3),(4,5),(6,7), pass 8.
            // Odd layers:  (1,2),(3,4),(5,6),(7,8), pass 0.
            for (pair_index=0; pair_index<4; pair_index=pair_index+1) begin: Compare_Pair
                localparam integer LEFT = pair_index*2 + (layer%2);
                wire [7:0] a = values[layer][LEFT*8 +: 8];
                wire [7:0] b = values[layer][(LEFT+1)*8 +: 8];
                wire swap = a > b;
                always @(posedge clk) begin
                    if (!rst_n) begin
                        values[layer+1][LEFT*8 +: 8] <= 0;
                        values[layer+1][(LEFT+1)*8 +: 8] <= 0;
                    end else if (valid_pipe[layer]) begin
                        values[layer+1][LEFT*8 +: 8] <= swap ? b : a;
                        values[layer+1][(LEFT+1)*8 +: 8] <= swap ? a : b;
                    end
                end
            end
            localparam integer UNPAIRED = (layer%2) ? 0 : 8;
            always @(posedge clk) begin
                if (!rst_n)
                    values[layer+1][UNPAIRED*8 +: 8] <= 0;
                else if (valid_pipe[layer])
                    values[layer+1][UNPAIRED*8 +: 8] <= values[layer][UNPAIRED*8 +: 8];
            end
        end
    endgenerate

    assign out_valid = valid_pipe[9];
    assign out_sof = out_valid && sof_pipe[9];
    assign out_eof = out_valid && eof_pipe[9];
    assign out_x = x_pipe[9];
    assign out_y = y_pipe[9];
    assign out_gray = values[9][4*8 +: 8];
endmodule
