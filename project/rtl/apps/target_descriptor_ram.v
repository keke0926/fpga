`timescale 1ns/1ps
// Two snapshot banks, each 2048 x 48 bits, implemented as 12 true dual-port
// RAM10 blocks. Port A belongs to discovery/classification; port B reads pages.
// No memory reset. The owner publishes a bank only after every valid record
// has become a result, and never writes that bank while it is displayed.
module target_descriptor_ram(
    input wire clk,
    input wire engine_bank_i,engine_we_i,
    input wire [10:0] engine_addr_i,
    input wire [47:0] engine_data_i,
    output wire [47:0] engine_data_o,
    input wire page_bank_i,
    input wire [10:0] page_addr_i,
    output wire [47:0] page_data_o
);
    wire [47:0] engine0,engine1,page0,page1;
    reg engine_bank_q,page_bank_q;
    always @(posedge clk) begin
        engine_bank_q<=engine_bank_i;
        page_bank_q<=page_bank_i;
    end
    assign engine_data_o=engine_bank_q?engine1:engine0;
    assign page_data_o=page_bank_q?page1:page0;
    genvar lane;
    generate for(lane=0;lane<12;lane=lane+1) begin:gen_lane
        EFX_DPRAM10 #(
            .READ_WIDTH_A(4),.WRITE_WIDTH_A(4),
            .READ_WIDTH_B(4),.WRITE_WIDTH_B(4),
            .OUTPUT_REG_A(1'b0),.OUTPUT_REG_B(1'b0),
            .WRITE_MODE_A("READ_FIRST"),.WRITE_MODE_B("READ_FIRST")
        ) bank0_ram (
            .CLKA(clk),.CLKEA(1'b1),.WEA(engine_we_i&&!engine_bank_i),
            .RSTA(1'b0),.ADDRENA(1'b1),.ADDRA(engine_addr_i),
            .WDATAA(engine_data_i[lane*4+:4]),.RDATAA(engine0[lane*4+:4]),
            .CLKB(clk),.CLKEB(1'b1),.WEB(1'b0),.RSTB(1'b0),
            .ADDRENB(1'b1),.ADDRB(page_addr_i),.WDATAB(4'b0),.RDATAB(page0[lane*4+:4])
        );
        EFX_DPRAM10 #(
            .READ_WIDTH_A(4),.WRITE_WIDTH_A(4),
            .READ_WIDTH_B(4),.WRITE_WIDTH_B(4),
            .OUTPUT_REG_A(1'b0),.OUTPUT_REG_B(1'b0),
            .WRITE_MODE_A("READ_FIRST"),.WRITE_MODE_B("READ_FIRST")
        ) bank1_ram (
            .CLKA(clk),.CLKEA(1'b1),.WEA(engine_we_i&&engine_bank_i),
            .RSTA(1'b0),.ADDRENA(1'b1),.ADDRA(engine_addr_i),
            .WDATAA(engine_data_i[lane*4+:4]),.RDATAA(engine1[lane*4+:4]),
            .CLKB(clk),.CLKEB(1'b1),.WEB(1'b0),.RSTB(1'b0),
            .ADDRENB(1'b1),.ADDRB(page_addr_i),.WDATAB(4'b0),.RDATAB(page1[lane*4+:4])
        );
    end endgenerate
endmodule
