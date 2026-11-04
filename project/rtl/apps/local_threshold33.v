// 256x150 uint8, replicate-border 33x33 box sum, no full-frame copy.
// External gray_data_i is a one-clock synchronous read of gray_addr_o.
module local_threshold33(
 input wire clk,input wire rst_n,input wire abort_i,input wire start_i,
 output wire ready_o,output wire busy_o,
 output reg[15:0]gray_addr_o,input wire[7:0]gray_data_i,
 output wire mask_valid_o,output reg[15:0]mask_addr_o,output reg mask_bit_o,
 output wire done_o,output reg[31:0]cycles_o
);
 localparam IDLE=0,INIT_READ=1,INIT_USE=2,HINIT_READ=3,HINIT_USE=4,
  PIX_READ=5,PIX_USE=6,HOUT_READ=7,HOUT_USE=8,HIN_READ=9,HIN_USE=10,
  COL_READ=11,COL_USE=12,OUT_READ=13,OUT_USE=14,IN_READ=15,IN_USE=16,FINISH=17;
 reg[4:0]state;
 reg valid_reg,done_reg;
 assign ready_o=(state==IDLE)&&rst_n&&!abort_i;
 assign busy_o=(state!=IDLE);
 assign mask_valid_o=valid_reg&&rst_n&&!abort_i;
 assign done_o=done_reg&&rst_n&&!abort_i;

 // 33*255 = 8415, fits 14 bits. Contents are rebuilt at every START.
 (* ram_style="block" *) reg[13:0]column_sums[0:255];
 reg[7:0]column_addr;reg column_write;reg[13:0]column_write_data,column_data;
 always @(posedge clk)begin
  column_data<=column_sums[column_addr];
  if(rst_n&&!abort_i&&column_write)column_sums[column_addr]<=column_write_data;
 end
 reg[7:0]init_x,update_x,px,row_y;
 reg[4:0]init_y,hindex;
 reg[13:0]col_acc,old_column;
 reg[7:0]out_pixel;
 // 1089*255 = 277695, requires 19 bits (18 would overflow).
 reg[18:0]window_sum,window_minus;
 wire[7:0]remove_y=row_y<17?8'd0:row_y-8'd17;
 wire[7:0]add_y=row_y>133?8'd149:row_y+8'd16;
 wire[7:0]remove_x=px<17?8'd0:px-8'd17;
 wire[7:0]add_x=px>239?8'd255:px+8'd16;
 wire[21:0]pixel_scaled={14'd0,gray_data_i}*22'd8712;
 wire[21:0]sum_scaled={3'd0,window_sum}*22'd7;
 always @*begin
  gray_addr_o=0;column_addr=0;column_write=0;column_write_data=0;
  case(state)
   INIT_READ,INIT_USE:begin
    gray_addr_o={3'd0,init_y,init_x};column_addr=init_x;
    if(state==INIT_USE&&init_y==16)begin column_write=1;column_write_data=col_acc+{6'd0,gray_data_i};end
   end
   HINIT_READ,HINIT_USE:column_addr={3'd0,hindex};
   PIX_READ,PIX_USE:gray_addr_o={row_y,px};
   HOUT_READ,HOUT_USE:column_addr=remove_x;
   HIN_READ,HIN_USE:column_addr=add_x;
   COL_READ,COL_USE:column_addr=update_x;
   OUT_READ,OUT_USE:gray_addr_o={remove_y,update_x};
   IN_READ,IN_USE:begin
    gray_addr_o={add_y,update_x};column_addr=update_x;
    if(state==IN_USE)begin column_write=1;column_write_data=old_column-{6'd0,out_pixel}+{6'd0,gray_data_i};end
   end
   default:begin end
  endcase
 end
 always @(posedge clk)begin
  if(!rst_n||abort_i)begin
   state<=IDLE;valid_reg<=0;done_reg<=0;mask_addr_o<=0;mask_bit_o<=0;cycles_o<=0;
   init_x<=0;init_y<=0;update_x<=0;px<=0;row_y<=0;hindex<=0;
   col_acc<=0;old_column<=0;out_pixel<=0;window_sum<=0;window_minus<=0;
  end else begin
   valid_reg<=0;done_reg<=0;
   if(state!=IDLE)cycles_o<=cycles_o+1'b1;
   case(state)
    IDLE:if(start_i)begin
     cycles_o<=0;init_x<=0;init_y<=0;col_acc<=0;row_y<=0;px<=0;state<=INIT_READ;
    end
    INIT_READ:state<=INIT_USE;
    INIT_USE:begin
     if(init_y==0)col_acc<=({6'd0,gray_data_i}<<4)+{6'd0,gray_data_i};
     else col_acc<=col_acc+{6'd0,gray_data_i};
     if(init_y==16)begin
      init_y<=0;col_acc<=0;
      if(init_x==255)begin hindex<=0;window_sum<=0;state<=HINIT_READ;end
      else begin init_x<=init_x+1'b1;state<=INIT_READ;end
     end else begin init_y<=init_y+1'b1;state<=INIT_READ;end
    end
    HINIT_READ:state<=HINIT_USE;
    HINIT_USE:begin
     if(hindex==0)window_sum<=({5'd0,column_data}<<4)+{5'd0,column_data};
     else window_sum<=window_sum+{5'd0,column_data};
     if(hindex==16)begin px<=0;state<=PIX_READ;end
     else begin hindex<=hindex+1'b1;state<=HINIT_READ;end
    end
    PIX_READ:state<=PIX_USE;
    PIX_USE:begin
     valid_reg<=1;mask_addr_o<={row_y,px};mask_bit_o<=window_sum>=19'd8712&&pixel_scaled<=sum_scaled;
     if(px==255)begin
      if(row_y==149)state<=FINISH;
      else begin row_y<=row_y+1'b1;update_x<=0;state<=COL_READ;end
     end else begin px<=px+1'b1;state<=HOUT_READ;end
    end
    HOUT_READ:state<=HOUT_USE;
    HOUT_USE:begin window_minus<=window_sum-{5'd0,column_data};state<=HIN_READ;end
    HIN_READ:state<=HIN_USE;
    HIN_USE:begin window_sum<=window_minus+{5'd0,column_data};state<=PIX_READ;end
    COL_READ:state<=COL_USE;
    COL_USE:begin old_column<=column_data;state<=OUT_READ;end
    OUT_READ:state<=OUT_USE;
    OUT_USE:begin out_pixel<=gray_data_i;state<=IN_READ;end
    IN_READ:state<=IN_USE;
    IN_USE:begin
     if(update_x==255)begin hindex<=0;window_sum<=0;state<=HINIT_READ;end
     else begin update_x<=update_x+1'b1;state<=COL_READ;end
    end
    FINISH:begin done_reg<=1;state<=IDLE;end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
