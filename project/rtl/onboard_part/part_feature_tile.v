`timescale 1ns/1ps
// Native-resolution feature tile. External 1-cycle synchronous memories only.
// gray[0:16383] = reflected median 128x128; gray[16384:25987] = Q4 98x98.
// queue[0:383] = 128 column records: sum16, square-sum low16, high5.
// state[0:9603].bit0 = original clipping (caller); bit1 = low local contrast.
// Input halo is already formed in GLOBAL coordinates, never at tile seams.
// Output core begins at median (16,16), at most 96x96 pixels.
module part_feature_tile(
 input wire clk,rst_n,abort_i,start_i,input wire[7:0]width_i,height_i,
 output wire busy_o,output reg done_o,
 output reg[15:0]gray_addr_o,output reg gray_we_o,output reg[7:0]gray_data_o,input wire[7:0]gray_data_i,
 output reg[15:0]state_addr_o,output reg[1:0]state_we_o,state_data_o,input wire[1:0]state_data_i,
 output reg[11:0]queue_addr_o,output reg queue_we_o,output reg[15:0]queue_data_o,input wire[15:0]queue_data_i,
 output wire out_valid_o,input wire out_ready_i,output reg[6:0]out_x_o,out_y_o,output reg[31:0]out_word_o
);
 localparam IDLE=0,INIT_REQ=1,INIT_WAIT=2,CWRITE=3,CREQ=4,CWAIT=5,
 OLD_REQ=6,OLD_WAIT=7,NEW_REQ=8,NEW_WAIT=9,CENTER_REQ=10,CENTER_WAIT=11,
 NSTART=12,NWAIT=13,NWRITE=14,GRAD_REQ=15,GRAD_WAIT=16,MSTART=17,MWAIT=18,
 FLAG_REQ=19,FLAG_WAIT=20,OUT=21;
 reg[4:0]state;
 reg[7:0]width,height;
 reg[6:0]col,row,nx,ny;
 reg[1:0]word_index,read_kind; // 0 row start, 1 departing, 2 arriving, 3 update
 reg updating;
 reg[15:0]column_sum;
 reg[20:0]column_square;
 reg[17:0]window_sum;
 reg[25:0]window_square;
 reg[7:0]old_pixel,center;
 reg[7:0]normalized;
 reg low;
 reg[3:0]tap;
 reg raw_read;
 reg signed[11:0]gx,gy,rx,ry;
 reg[7:0]raw_mag;
 wire[15:0]pixel_square=gray_data_i*gray_data_i;
 wire[15:0]old_square=old_pixel*old_pixel;
 wire[20:0]loaded_square={queue_data_i[4:0],column_square[15:0]};
 wire[13:0]norm_index=ny*14'd98+nx;
 wire[13:0]out_index=(out_y_o+14'd1)*14'd98+out_x_o+14'd1;
 wire[11:0]queue_base={5'd0,col}*12'd3;
 reg[13:0]tap_addr;
 reg[7:0]tx,ty;
 wire signed[11:0]pixel=raw_read ? $signed({4'd0,gray_data_i}) : $signed({{4{gray_data_i[7]}},gray_data_i});
 reg signed[11:0]xterm,yterm;
 wire[23:0]rx_square=rx*rx,ry_square=ry*ry;
 wire[25:0]mag_squared_q4=({2'b0,rx_square}+{2'b0,ry_square})<<4;
 wire normal_ready,normal_valid;wire signed[7:0]normal_value;wire normal_sat,normal_low;
 wire magnitude_request=state==MSTART;
 part_local_normalize arithmetic(.clk(clk),.rst_n(rst_n),.abort_i(abort_i),
 .in_valid_i(state==NSTART||state==MSTART),.in_ready_o(normal_ready),
 .center_i(center),.magnitude_i(magnitude_request),.sum_i(window_sum),
 .square_sum_i(magnitude_request?mag_squared_q4:window_square),
 .out_valid_o(normal_valid),.out_ready_i(state==NWAIT||state==MWAIT),
 .normalized_o(normal_value),.saturated_o(normal_sat),.low_contrast_o(normal_low));
 assign busy_o=state!=IDLE;
 assign out_valid_o=rst_n&&!abort_i&&state==OUT;
 always@*begin
  tx={1'b0,out_x_o}+(raw_read?8'd16:8'd1);
  ty={1'b0,out_y_o}+(raw_read?8'd16:8'd1);
  case(tap)
   0:begin tx=tx-1'b1;ty=ty-1'b1;end
   1:ty=ty-1'b1;
   2:begin tx=tx+1'b1;ty=ty-1'b1;end
   3:tx=tx-1'b1;
   5:tx=tx+1'b1;
   6:begin tx=tx-1'b1;ty=ty+1'b1;end
   7:ty=ty+1'b1;
   8:begin tx=tx+1'b1;ty=ty+1'b1;end
  endcase
  tap_addr=raw_read ? {ty[6:0],tx[6:0]} : ty*14'd98+tx;
  xterm=0;yterm=0;
  case(tap)0,6:xterm=-pixel;3:xterm=-(pixel<<<1);2,8:xterm=pixel;5:xterm=pixel<<<1;endcase
  case(tap)0,2:yterm=-pixel;1:yterm=-(pixel<<<1);6,8:yterm=pixel;7:yterm=pixel<<<1;endcase
 end
 always@*begin
  gray_addr_o=0;gray_we_o=0;gray_data_o=normalized;
  state_addr_o=0;state_we_o=0;state_data_o={low,1'b0};
  queue_addr_o=queue_base+word_index;queue_we_o=0;
  queue_data_o=word_index==0 ? column_sum : word_index==1 ? column_square[15:0] : {11'd0,column_square[20:16]};
  if(rst_n&&!abort_i)case(state)
   INIT_REQ,INIT_WAIT:gray_addr_o={2'b0,row,col};
   CWRITE:queue_we_o=1;
   OLD_REQ,OLD_WAIT:gray_addr_o={2'b0,ny,col};
   NEW_REQ,NEW_WAIT:gray_addr_o=({9'd0,ny}+16'd31)*16'd128+col;
   CENTER_REQ,CENTER_WAIT:gray_addr_o=({9'd0,ny}+16'd15)*16'd128+nx+16'd15;
   NWRITE:begin
    gray_addr_o=16'd16384+norm_index;gray_we_o=1;
    state_addr_o={2'd0,norm_index};state_we_o=2'b10;
   end
   GRAD_REQ,GRAD_WAIT:gray_addr_o=raw_read ? {2'd0,tap_addr} : 16'd16384+tap_addr;
   FLAG_REQ,FLAG_WAIT:state_addr_o={2'd0,out_index};
  endcase
 end
 always@(posedge clk)begin
  if(!rst_n||abort_i)begin
   state<=IDLE;done_o<=0;width<=0;height<=0;col<=0;row<=0;nx<=0;ny<=0;
   word_index<=0;read_kind<=0;updating<=0;column_sum<=0;column_square<=0;
   window_sum<=0;window_square<=0;old_pixel<=0;center<=0;normalized<=0;low<=0;
   tap<=0;raw_read<=0;gx<=0;gy<=0;rx<=0;ry<=0;raw_mag<=0;
   out_x_o<=0;out_y_o<=0;out_word_o<=0;
  end else begin
   done_o<=0;
   case(state)
    IDLE:if(start_i)begin
     width<=width_i;height<=height_i;col<=0;row<=0;nx<=0;ny<=0;
     column_sum<=0;column_square<=0;window_sum<=0;window_square<=0;
     updating<=0;word_index<=0;state<=INIT_REQ;
    end
    INIT_REQ:state<=INIT_WAIT;
    INIT_WAIT:begin
     column_sum<=column_sum+gray_data_i;column_square<=column_square+pixel_square;
     if(row==30)begin word_index<=0;state<=CWRITE;end
     else begin row<=row+1'b1;state<=INIT_REQ;end
    end
    CWRITE:if(word_index!=2)word_index<=word_index+1'b1;
     else begin
      word_index<=0;
      if(col==127)begin
       if(updating)ny<=ny+1'b1;
       updating<=0;col<=0;nx<=0;window_sum<=0;window_square<=0;read_kind<=0;state<=CREQ;
      end else begin
       col<=col+1'b1;row<=0;column_sum<=0;column_square<=0;
       if(updating)begin read_kind<=3;state<=CREQ;end else state<=INIT_REQ;
      end
     end
    CREQ:state<=CWAIT;
    CWAIT:begin
     if(word_index==0)column_sum<=queue_data_i;
     if(word_index==1)column_square[15:0]<=queue_data_i;
     if(word_index!=2)begin word_index<=word_index+1'b1;state<=CREQ;end
     else begin
      column_square[20:16]<=queue_data_i[4:0];word_index<=0;
      if(read_kind==3)state<=OLD_REQ;
      else if(read_kind==1)begin
       window_sum<=window_sum-column_sum;window_square<=window_square-loaded_square;
       col<=nx+7'd31;read_kind<=2;state<=CREQ;
      end else begin
       window_sum<=window_sum+column_sum;window_square<=window_square+loaded_square;
       if(read_kind==0&&col!=30)begin col<=col+1'b1;state<=CREQ;end
       else begin if(read_kind==2)nx<=nx+1'b1;state<=CENTER_REQ;end
      end
     end
    end
    OLD_REQ:state<=OLD_WAIT;
    OLD_WAIT:begin old_pixel<=gray_data_i;state<=NEW_REQ;end
    NEW_REQ:state<=NEW_WAIT;
    NEW_WAIT:begin
     column_sum<=column_sum-old_pixel+gray_data_i;
     column_square<=column_square-old_square+pixel_square;
     word_index<=0;state<=CWRITE;
    end
    CENTER_REQ:state<=CENTER_WAIT;
    CENTER_WAIT:begin center<=gray_data_i;state<=NSTART;end
    NSTART:if(normal_ready)state<=NWAIT;
    NWAIT:if(normal_valid)begin normalized<=normal_value;low<=normal_low;state<=NWRITE;end
    NWRITE:begin
     if(nx!=97)begin col<=nx;word_index<=0;read_kind<=1;state<=CREQ;end
     else if(ny!=97)begin col<=0;word_index<=0;read_kind<=3;updating<=1;state<=CREQ;end
     else begin out_x_o<=0;out_y_o<=0;tap<=0;raw_read<=0;gx<=0;gy<=0;rx<=0;ry<=0;state<=GRAD_REQ;end
    end
    GRAD_REQ:state<=GRAD_WAIT;
    GRAD_WAIT:begin
     if(raw_read)begin rx<=rx+xterm;ry<=ry+yterm;end
     else begin gx<=gx+xterm;gy<=gy+yterm;end
     if(!raw_read)begin raw_read<=1;state<=GRAD_REQ;end
     else begin
      raw_read<=0;
      if(tap==8)state<=MSTART;
      else begin tap<=tap+1'b1;state<=GRAD_REQ;end
     end
    end
    MSTART:if(normal_ready)state<=MWAIT;
    MWAIT:if(normal_valid)begin raw_mag<=normal_value;state<=FLAG_REQ;end
    FLAG_REQ:state<=FLAG_WAIT;
    FLAG_WAIT:begin out_word_o<={state_data_i[1],state_data_i[0],raw_mag,gy[10:0],gx[10:0]};state<=OUT;end
    OUT:if(out_ready_i)begin
     if(out_x_o+1==width&&out_y_o+1==height)begin done_o<=1;state<=IDLE;end
     else begin
      if(out_x_o+1==width)begin out_x_o<=0;out_y_o<=out_y_o+1'b1;end
      else out_x_o<=out_x_o+1'b1;
      tap<=0;raw_read<=0;gx<=0;gy<=0;rx<=0;ry<=0;state<=GRAD_REQ;
     end
    end
   endcase
  end
 end
endmodule
