`timescale 1ns/1ps
module capture_uart_rx #(parameter CLOCK_HZ=48000000,BAUD=460800)(
    input wire clk,rst_n,rx_i,output reg byte_valid_o,output reg[7:0] byte_o,
    output reg framing_error_o);
    localparam CLKS=(CLOCK_HZ+BAUD/2)/BAUD;
    localparam CW=$clog2(CLKS)>0?$clog2(CLKS):1;
    (* async_reg="true" *) reg[1:0] rx_sync;
    reg[2:0] state;
    reg[CW-1:0] countdown;
    reg[2:0] bit_index;
    reg[7:0] shift;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)rx_sync<=2'b11;else rx_sync<={rx_sync[0],rx_i};
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin state<=0;countdown<=0;bit_index<=0;shift<=0;byte_o<=0;byte_valid_o<=0;framing_error_o<=0;end
        else begin
            byte_valid_o<=0;framing_error_o<=0;
            case(state)
                0:if(!rx_sync[1])begin state<=1;countdown<=CLKS/2-1;end
                1:if(countdown!=0)countdown<=countdown-1'b1;
                  else if(!rx_sync[1])begin state<=2;countdown<=CLKS-1;bit_index<=0;end
                  else state<=0;
                2:if(countdown!=0)countdown<=countdown-1'b1;
                  else begin
                      shift[bit_index]<=rx_sync[1];countdown<=CLKS-1;
                      if(bit_index==7)state<=3;else bit_index<=bit_index+1'b1;
                  end
                3:if(countdown!=0)countdown<=countdown-1'b1;
                  else if(rx_sync[1])begin byte_o<=shift;byte_valid_o<=1;state<=0;end
                  else begin framing_error_o<=1;state<=4;end
                4:if(rx_sync[1])state<=0;
                default:state<=0;
            endcase
        end
    end
endmodule
