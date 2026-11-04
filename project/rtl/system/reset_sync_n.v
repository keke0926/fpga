`timescale 1ns/1ps
module reset_sync_n(input wire clk, arst_n, output wire rst_n);
    // Initialization preserves the vendor design's FPGA startup reset value.
    // async_reg requests preservation and synchronization-aware placement;
    // physical resolution time/MTBF still require implementation evidence.
    (* async_reg = "true" *) reg [1:0] release_pipe = 2'b00;
    always @(posedge clk or negedge arst_n) begin
        if (!arst_n) release_pipe <= 2'b00;
        else release_pipe <= {release_pipe[0],1'b1};
    end
    assign rst_n = release_pipe[1];
endmodule
