`timescale 1ns/1ps
module capture_uart_tx #(parameter CLOCK_HZ=48000000,BAUD=460800)(
    input wire clk,rst_n,byte_valid_i,input wire[7:0] byte_i,
    output wire byte_ready_o,output reg tx_o,output wire idle_o);
    localparam CLKS=(CLOCK_HZ+BAUD/2)/BAUD;
    localparam CW=$clog2(CLKS)>0?$clog2(CLKS):1;
    reg[CW-1:0] countdown;
    reg[9:0] frame;
    reg[3:0] bit_index;
    reg busy;
    assign byte_ready_o=!busy;
    assign idle_o=!busy;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin tx_o<=1;busy<=0;countdown<=0;frame<=10'h3ff;bit_index<=0;end
        else if(!busy)begin
            tx_o<=1;
            if(byte_valid_i)begin
                frame<={1'b1,byte_i,1'b0};tx_o<=0;busy<=1;bit_index<=0;countdown<=CLKS-1;
            end
        end else if(countdown!=0)countdown<=countdown-1'b1;
        else begin
            countdown<=CLKS-1;
            if(bit_index==9)begin busy<=0;tx_o<=1;end
            else begin bit_index<=bit_index+1'b1;tx_o<=frame[bit_index+1'b1];end
        end
    end
endmodule
