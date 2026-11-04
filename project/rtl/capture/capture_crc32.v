`timescale 1ns/1ps
module capture_crc32(input wire clk,rst_n,clear_i,valid_i,input wire[7:0] byte_i,
                     output wire[31:0] crc_o);
    reg[31:0] state;
    reg[31:0] next_state;
    integer i;
    always @* begin
        next_state=state^{24'b0,byte_i};
        for(i=0;i<8;i=i+1)
            next_state=next_state[0] ? ((next_state>>1)^32'hedb88320) : (next_state>>1);
    end
    always @(posedge clk or negedge rst_n)
        if(!rst_n)state<=32'hffffffff;
        else if(clear_i)state<=32'hffffffff;
        else if(valid_i)state<=next_state;
    assign crc_o=~state;
endmodule
