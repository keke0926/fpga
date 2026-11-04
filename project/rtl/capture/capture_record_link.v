`timescale 1ns/1ps
`include "capture_protocol_defs.vh"
module capture_record_link #(parameter CONTROL_TIMEOUT_CYCLES=144000000)(
    input wire clk,rst_n,rx_valid_i,input wire[7:0] rx_byte_i,
    input wire tx_ready_i,tx_idle_i,output reg tx_valid_o,output reg[7:0] tx_byte_o,
    output reg cmd_valid_o,output reg[7:0] cmd_opcode_o,output reg[15:0] cmd_sequence_o,
    output reg[31:0] cmd_request_id_o,output reg protocol_error_o,
    input wire record_valid_i,output wire record_ready_o,input wire[7:0] record_kind_i,
    input wire[31:0] record_request_id_i,record_frame_id_i,
    input wire[15:0] record_sequence_i,record_first_row_i,record_row_count_i,record_payload_bytes_i,
    output wire[13:0] payload_addr_o,input wire[7:0] payload_byte_i,output reg record_sent_o);
    function[31:0] crc_step;
        input[31:0] value;input[7:0] data;reg[31:0] work;integer k;
        begin work=value^{24'b0,data};for(k=0;k<8;k=k+1)work=work[0]?((work>>1)^32'hedb88320):(work>>1);crc_step=work;end
    endfunction
    localparam TW=$clog2(CONTROL_TIMEOUT_CYCLES)>0?$clog2(CONTROL_TIMEOUT_CYCLES):1;
    reg[31:0] magic_shift,rx_crc;
    reg[127:0] rx_packet;
    reg[4:0] rx_count;
    reg receiving,rx_error;
    reg[TW-1:0] rx_wait;
    wire[7:0] parsed_opcode=rx_packet[39:32];
    wire[15:0] parsed_sequence=rx_packet[63:48];
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            magic_shift<=0;rx_crc<=32'hffffffff;rx_packet<=0;rx_count<=0;receiving<=0;rx_wait<=0;
            cmd_valid_o<=0;cmd_opcode_o<=0;cmd_sequence_o<=0;cmd_request_id_o<=0;rx_error<=0;
        end else begin
            cmd_valid_o<=0;rx_error<=0;
            if(rx_valid_i)begin
                rx_wait<=0;
                if(!receiving)begin
                    magic_shift<={rx_byte_i,magic_shift[31:8]};
                    if({rx_byte_i,magic_shift[31:8]}==32'h31504143)begin
                        receiving<=1;rx_count<=4;rx_packet<=128'h31504143;
                        rx_crc<=crc_step(crc_step(crc_step(crc_step(32'hffffffff,8'h43),8'h41),8'h50),8'h31);
                    end
                end else begin
                    rx_packet[rx_count*8+:8]<=rx_byte_i;
                    if(rx_count<12)rx_crc<=crc_step(rx_crc,rx_byte_i);
                    if(rx_count==15)begin
                        receiving<=0;magic_shift<=0;
                        if(({rx_byte_i,rx_packet[119:96]}==~rx_crc) && rx_packet[47:40]==0 &&
                           rx_packet[95:64]!=0 && parsed_opcode>=1 && parsed_opcode<=7 &&
                           (parsed_sequence==16'hffff || (parsed_opcode==2 && parsed_sequence<38)))begin
                            cmd_valid_o<=1;cmd_opcode_o<=parsed_opcode;cmd_sequence_o<=parsed_sequence;
                            cmd_request_id_o<=rx_packet[95:64];
                        end else rx_error<=1;
                    end else rx_count<=rx_count+1'b1;
                end
            end else if(receiving)begin
                if(rx_wait==CONTROL_TIMEOUT_CYCLES-1)begin receiving<=0;magic_shift<=0;rx_wait<=0;rx_error<=1;end
                else rx_wait<=rx_wait+1'b1;
            end
        end
    end
    reg[3:0] state;
    localparam IDLE=0,HEADER_CRC=1,SEND_HEADER=2,ADDR=3,WAIT_RAM=4,SEND_PAYLOAD=5,SEND_TRAILER=6,WAIT_IDLE=7;
    reg[255:0] header;
    reg[31:0] hcrc,body_crc,trailer;
    reg[15:0] payload_len,payload_index;
    reg[5:0] index;
    reg[7:0] payload_latched;
    reg tx_error;
    wire descriptor_ok = record_request_id_i!=0 && record_payload_bytes_i<=16384 &&
        ((record_kind_i==2 && record_sequence_i<38 && record_first_row_i==record_sequence_i*16 &&
          record_row_count_i==(record_sequence_i==37?8:16) &&
          record_payload_bytes_i==(record_sequence_i==37?8192:16384)) ||
         (record_sequence_i==16'hffff && record_first_row_i==0 && record_row_count_i==0 &&
          ((record_kind_i==1 && record_payload_bytes_i==10) ||
           ((record_kind_i==3 || record_kind_i==6 || record_kind_i==7) && record_payload_bytes_i==12) ||
           ((record_kind_i==4 || record_kind_i==5 || record_kind_i==8) && record_payload_bytes_i==8))));
    assign record_ready_o=state==IDLE;
    assign payload_addr_o=payload_index[13:0];
    always @* begin
        tx_valid_o=0;tx_byte_o=0;protocol_error_o=rx_error||tx_error;
        case(state)
            SEND_HEADER:begin tx_valid_o=1;tx_byte_o=header[index*8+:8];end
            SEND_PAYLOAD:begin tx_valid_o=1;tx_byte_o=payload_latched;end
            SEND_TRAILER:begin tx_valid_o=1;tx_byte_o=trailer[index*8+:8];end
            default:begin end
        endcase
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin state<=IDLE;header<=0;hcrc<=32'hffffffff;body_crc<=32'hffffffff;
            trailer<=0;payload_len<=0;payload_index<=0;index<=0;payload_latched<=0;record_sent_o<=0;tx_error<=0;end
        else begin
            record_sent_o<=0;tx_error<=0;
            case(state)
                IDLE:if(record_valid_i)begin
                    if(!descriptor_ok)tx_error<=1;
                    else begin
                        header[63:0]<=64'h3156353331305241;header[71:64]<=record_kind_i;
                        header[79:72]<=8'd1;header[95:80]<=16'd32;
                        header[127:96]<=record_request_id_i;header[159:128]<=record_frame_id_i;
                        header[175:160]<=record_sequence_i;header[191:176]<=record_first_row_i;
                        header[207:192]<=record_row_count_i;header[223:208]<=record_payload_bytes_i;
                        header[255:224]<=0;payload_len<=record_payload_bytes_i;payload_index<=0;
                        hcrc<=32'hffffffff;body_crc<=32'hffffffff;index<=0;state<=HEADER_CRC;
                    end
                end
                HEADER_CRC:begin
                    hcrc<=crc_step(hcrc,header[index*8+:8]);
                    if(index==27)begin header[255:224]<=~crc_step(hcrc,header[index*8+:8]);index<=0;state<=SEND_HEADER;end
                    else index<=index+1'b1;
                end
                SEND_HEADER:if(tx_ready_i)begin
                    body_crc<=crc_step(body_crc,tx_byte_o);
                    if(index==31)begin index<=0;state<=ADDR;end else index<=index+1'b1;
                end
                ADDR:state<=WAIT_RAM;
                WAIT_RAM:begin payload_latched<=payload_byte_i;state<=SEND_PAYLOAD;end
                SEND_PAYLOAD:if(tx_ready_i)begin
                    body_crc<=crc_step(body_crc,payload_latched);
                    if(payload_index+1==payload_len)begin
                        trailer<=~crc_step(body_crc,payload_latched);index<=0;state<=SEND_TRAILER;
                    end else begin payload_index<=payload_index+1'b1;state<=ADDR;end
                end
                SEND_TRAILER:if(tx_ready_i)begin
                    if(index==3)state<=WAIT_IDLE;else index<=index+1'b1;
                end
                WAIT_IDLE:if(tx_idle_i)begin record_sent_o<=1;state<=IDLE;end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
