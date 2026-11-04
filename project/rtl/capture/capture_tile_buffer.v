`timescale 1ns/1ps
module capture_tile_buffer #(parameter WIDTH=1024,HEIGHT=600)(
 input wire clk,rst_n,start_i,input wire[15:0] first_row_i,row_count_i,
 input wire in_vs_i,in_de_i,input wire[7:0] in_gray_i,input wire release_i,
 output reg ready_o,output reg[15:0] payload_bytes_o,output reg error_o,
 input wire[13:0] read_addr_i,output reg[7:0] read_byte_o,
 output reg accepted_valid_o,output reg[7:0] accepted_byte_o);
 localparam IDLE=0,WAIT_FRAME=1,CAPTURE=2,HOLD=3;
 reg[1:0] state;reg vs_d,de_d;
 reg[15:0] first_row,end_row,row,column,count;
 (* ram_style="block" *) reg[7:0] memory[0:16383];
 // Unconditional synchronous read infers RAM; no reset of the RAM array.
 always @(posedge clk)read_byte_o<=memory[read_addr_i];
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;vs_d<=1;de_d<=0;ready_o<=0;payload_bytes_o<=0;error_o<=0;
   first_row<=0;end_row<=0;row<=0;column<=0;count<=0;accepted_valid_o<=0;accepted_byte_o<=0;
  end else begin
   vs_d<=in_vs_i;de_d<=in_de_i;accepted_valid_o<=0;
   if(release_i)begin state<=IDLE;ready_o<=0;error_o<=0;end
   else case(state)
    IDLE:if(start_i)begin
     ready_o<=0;error_o<=0;count<=0;row<=0;column<=0;
     if(row_count_i==0||row_count_i>16||first_row_i>=HEIGHT||
       {1'b0,first_row_i}+{1'b0,row_count_i}>HEIGHT||row_count_i*WIDTH>16384)begin error_o<=1;state<=HOLD;end
     else begin first_row<=first_row_i;end_row<=first_row_i+row_count_i;
       payload_bytes_o<=row_count_i*WIDTH;state<=WAIT_FRAME;end
    end
    WAIT_FRAME:if(vs_d&&!in_vs_i)begin row<=0;column<=0;count<=0;state<=CAPTURE;end
    CAPTURE:begin
     if(vs_d&&!in_vs_i)begin error_o<=1;state<=HOLD;end
     if(in_de_i)begin
      column<=column+1'b1;
      if(!in_vs_i||column>=WIDTH)begin error_o<=1;state<=HOLD;end
      else if(row>=first_row&&row<end_row)begin
       memory[count[13:0]]<=in_gray_i;count<=count+1'b1;
       accepted_valid_o<=1;accepted_byte_o<=in_gray_i;
      end
     end
     if(de_d&&!in_de_i)begin
      row<=row+1'b1;column<=0;
      if(column!=WIDTH)begin error_o<=1;state<=HOLD;end
      else if(row+1==end_row)begin
       if(count!=payload_bytes_o)error_o<=1;else ready_o<=1;
       state<=HOLD;
      end
     end
    end
   endcase
  end
 end
endmodule
