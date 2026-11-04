`timescale 1ns/1ps
// Writer controls only; source gray is never altered. Commands/results are
// bundled-data toggle handshakes: opcode stays stable until ack returns.
module capture_source_gate #(parameter WIDTH=1024,HEIGHT=600)(
    input wire clk,rst_n,src_vs_i,src_de_i,input wire[7:0] src_gray_i,
    input wire cmd_toggle_i,input wire[1:0] cmd_opcode_i,
    output reg ack_toggle_o,output wire writer_vs_o,writer_de_o,held_o,
    output reg[31:0] source_tag_o,output reg[19:0] pixel_count_o,
    output reg[31:0] source_crc32_o,output reg geometry_error_o);
    localparam NORMAL=0,STOP=1,HELD=2,SINGLE_WAIT=3,SINGLE=4,RESUME_WAIT=5,RESUME_FRAME=6;
    reg[2:0] state;
    (* async_reg="true" *) reg[1:0] command_sync;
    reg active_toggle,vs_d,de_d;
    reg[31:0] frame_counter,line_pixels,line_count,crc_state;
    wire sof=src_vs_i&&!vs_d,eof=!src_vs_i&&vs_d;
    wire starting=(state==SINGLE_WAIT || state==RESUME_WAIT)&&sof;
    wire tracking=state==SINGLE || state==RESUME_FRAME;
    wire pass=state==NORMAL || state==STOP || tracking || starting;
    assign writer_vs_o=pass&&src_vs_i;
    assign writer_de_o=pass&&src_de_i;
    assign held_o=state==HELD || state==SINGLE_WAIT || state==RESUME_WAIT;
    function[31:0] crc_step;
        input[31:0] value;input[7:0] data;reg[31:0] work;integer k;
        begin work=value^{24'b0,data};for(k=0;k<8;k=k+1)work=work[0]?((work>>1)^32'hedb88320):(work>>1);crc_step=work;end
    endfunction
    always @(posedge clk or negedge rst_n)
        if(!rst_n)command_sync<=0;else command_sync<={command_sync[0],cmd_toggle_i};
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=NORMAL;ack_toggle_o<=0;active_toggle<=0;vs_d<=0;de_d<=0;
            frame_counter<=0;source_tag_o<=0;pixel_count_o<=0;source_crc32_o<=0;
            geometry_error_o<=0;line_pixels<=0;line_count<=0;crc_state<=32'hffffffff;
        end else begin
            vs_d<=src_vs_i;de_d<=src_de_i;
            if(sof)frame_counter<=frame_counter+1'b1;
            if((state==NORMAL || state==HELD) && command_sync[1]!=ack_toggle_o)begin
                active_toggle<=command_sync[1];
                case(cmd_opcode_i)
                    0:if(!src_vs_i)begin state<=HELD;ack_toggle_o<=command_sync[1];end else state<=STOP;
                    1:if(state==HELD)state<=SINGLE_WAIT;
                      else begin geometry_error_o<=1;ack_toggle_o<=command_sync[1];end
                    2:if(state==HELD)state<=RESUME_WAIT;
                      else begin ack_toggle_o<=command_sync[1];end
                    default:begin geometry_error_o<=1;ack_toggle_o<=command_sync[1];end
                endcase
            end
            if(state==STOP && eof)begin state<=HELD;ack_toggle_o<=active_toggle;end
            if(starting)begin
                state<=state==SINGLE_WAIT?SINGLE:RESUME_FRAME;
                source_tag_o<=frame_counter+1'b1;
                pixel_count_o<=src_de_i?1:0;line_pixels<=src_de_i?1:0;line_count<=0;
                crc_state<=src_de_i?crc_step(32'hffffffff,src_gray_i):32'hffffffff;
                geometry_error_o<=0;
            end else if(tracking)begin
                if(src_de_i)begin
                    if(!src_vs_i)geometry_error_o<=1;
                    else begin
                        pixel_count_o<=pixel_count_o+1'b1;line_pixels<=line_pixels+1'b1;
                        crc_state<=crc_step(crc_state,src_gray_i);
                        if(line_pixels>=WIDTH || pixel_count_o>=WIDTH*HEIGHT)geometry_error_o<=1;
                    end
                end
                if(de_d&&!src_de_i)begin
                    if(line_pixels!=WIDTH)geometry_error_o<=1;
                    line_count<=line_count+1'b1;line_pixels<=0;
                end
                if(eof)begin
                    if(pixel_count_o!=WIDTH*HEIGHT || line_count+(de_d&&!src_de_i)!=HEIGHT ||
                       (de_d&&!src_de_i&&line_pixels!=WIDTH) || src_de_i)geometry_error_o<=1;
                    source_crc32_o<=~crc_state;
                    state<=state==SINGLE?HELD:NORMAL;
                    ack_toggle_o<=active_toggle;
                end
            end
        end
    end
endmodule
