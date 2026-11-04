`timescale 1ns/1ps
// 42x42 soft-direction descriptors -> 26x26 exact 17x17 Q12 histograms.
// The caller leases a byte scratch port. No private image/line RAM.
// Descriptor: angle[8:0], weight[15:9], strong[16], weak[17].
// Histograms use 21-bit sums (289*64*64 = 1183744), not 16-bit sums.
module part_histogram_tile #(
 parameter DESC_BASE=0,COLUMN_BASE=32364,OUTPUT_BASE=21548
)(
 input wire clk_i,rst_i,abort_i,start_i,weak_i,
 output wire ready_o,busy_o,valid_o,input wire out_ready_i,
 output wire[15:0]gray_addr_o,output wire gray_we_o,
 output wire[7:0]gray_data_o,input wire[7:0]gray_data_i,
 output reg[31:0]cycles_o
);
 localparam IDLE=0,CLEAR=1,V_DESC=2,V_DECODE=3,V_COL=4,V_SUM=5,V_WRITE=6,V_NEXT_BIN=7,V_NEXT=8,
 H_INIT=9,H_READ=10,H_ADD=11,H_NEXT=12,ENERGY=13,NORM_INIT=14,NORM_STEP=15,NORM_WRITE=16,NORM_NEXT=17,
 NEXT_X=18,NEXT_Y=19,RESULT=20,M_READ=21,M_TAKE=22,M_WRITE=23;
 reg[4:0]state,ret;
 reg weak_mode,subtract_row,second_bin,slide;
 reg[5:0]vy,vx,hy,hx,cx;reg[2:0]k;
 reg[9:0]clear_index;
 reg[15:0]ma;reg[1:0]mlen,mi;reg[23:0]mdata,read_data;
 reg[8:0]angle;reg[6:0]weight;reg[12:0]amount;
 reg[20:0]sums[0:7];
 reg[23:0]energy,remainder;reg[12:0]share;reg[3:0]step;
 wire[2:0]bin=angle[8:6]+{2'd0,second_bin};
 wire[6:0]fraction=second_bin?{1'b0,angle[5:0]}:7'd64-{1'b0,angle[5:0]};
 wire[13:0]splat=weight*fraction;
 wire[24:0]shifted={remainder,1'b0};
 wire take_bit=shifted>={1'b0,energy};
 wire[24:0]remaining=shifted-{1'b0,energy};
 wire[15:0]column_address=COLUMN_BASE+({7'd0,vx,3'd0}+bin)*3;
 assign ready_o=state==IDLE&&!rst_i&&!abort_i;
 assign busy_o=state!=IDLE&&state!=RESULT;
 assign valid_o=state==RESULT&&!rst_i&&!abort_i;
 assign gray_addr_o=state==CLEAR?COLUMN_BASE+clear_index:ma+mi;
 assign gray_we_o=!rst_i&&!abort_i&&(state==CLEAR||state==M_WRITE);
 assign gray_data_o=state==CLEAR?8'd0:mdata[7:0];
 integer j;
 always@(posedge clk_i)begin
  if(rst_i||abort_i)begin
   state<=IDLE;ret<=IDLE;weak_mode<=0;subtract_row<=0;second_bin<=0;slide<=0;
   vy<=0;vx<=0;hy<=0;hx<=0;cx<=0;k<=0;clear_index<=0;
   ma<=0;mlen<=0;mi<=0;mdata<=0;read_data<=0;angle<=0;weight<=0;amount<=0;
   energy<=0;remainder<=0;share<=0;step<=0;cycles_o<=0;
   for(j=0;j<8;j=j+1)sums[j]<=0;
  end else begin
   if(busy_o)cycles_o<=cycles_o+1'b1;
   case(state)
    IDLE:if(start_i)begin weak_mode<=weak_i;clear_index<=0;cycles_o<=0;state<=CLEAR;hy<=0;end
    CLEAR:if(clear_index==1007)begin vy<=0;vx<=0;subtract_row<=0;state<=V_DESC;end
     else clear_index<=clear_index+1'b1;
    V_DESC:begin ma<=DESC_BASE+(vy*42+vx)*3;mi<=0;mlen<=3;ret<=V_DECODE;state<=M_READ;end
    V_DECODE:begin
     angle<=read_data[8:0];weight<=read_data[15:9];second_bin<=0;
     state<=(weak_mode?read_data[17]:read_data[16])&&read_data[15:9]!=0?V_COL:V_NEXT;
    end
    V_COL:begin ma<=column_address;mi<=0;mlen<=3;ret<=V_SUM;amount<=splat[12:0];state<=M_READ;end
    V_SUM:begin mdata<=subtract_row?read_data-amount:read_data+amount;state<=V_WRITE;end
    V_WRITE:begin mi<=0;mlen<=3;ret<=V_NEXT_BIN;state<=M_WRITE;end
    V_NEXT_BIN:if(!second_bin)begin second_bin<=1;state<=V_COL;end else state<=V_NEXT;
    V_NEXT:begin
     if(vx!=41)begin vx<=vx+1'b1;state<=V_DESC;end
     else if(hy==0&&vy!=16)begin vx<=0;vy<=vy+1'b1;state<=V_DESC;end
     else if(subtract_row)begin vx<=0;vy<=hy+6'd16;subtract_row<=0;state<=V_DESC;end
     else state<=H_INIT;
    end
    H_INIT:begin
     for(j=0;j<8;j=j+1)sums[j]<=0;
     hx<=0;cx<=0;k<=0;slide<=0;subtract_row<=0;state<=H_READ;
    end
    H_READ:begin ma<=COLUMN_BASE+({7'd0,cx,3'd0}+k)*3;mi<=0;mlen<=3;ret<=H_ADD;state<=M_READ;end
    H_ADD:begin sums[k]<=subtract_row?sums[k]-read_data[20:0]:sums[k]+read_data[20:0];state<=H_NEXT;end
    H_NEXT:begin
     if(k!=7)begin k<=k+1'b1;state<=H_READ;end
     else if(!slide&&cx!=16)begin cx<=cx+1'b1;k<=0;state<=H_READ;end
     else if(subtract_row)begin cx<=hx+6'd16;k<=0;subtract_row<=0;state<=H_READ;end
     else begin k<=0;energy<=0;state<=ENERGY;end
    end
    ENERGY:begin
     energy<=energy+{3'd0,sums[k]};
     if(k==7)begin k<=0;state<=NORM_INIT;end else k<=k+1'b1;
    end
    NORM_INIT:begin
     remainder<={3'd0,sums[k]};share<=0;step<=12;
     if(energy==0)state<=NORM_WRITE;
     else if({3'd0,sums[k]}==energy)begin share<=4096;state<=NORM_WRITE;end
     else state<=NORM_STEP;
    end
    NORM_STEP:begin
     remainder<=take_bit?remaining[23:0]:shifted[23:0];share<={share[11:0],take_bit};step<=step-1'b1;
     if(step==1)state<=NORM_WRITE;
    end
    NORM_WRITE:begin
     ma<=OUTPUT_BASE+((hy*26+hx)*8+k)*2;mi<=0;mlen<=2;
     mdata<={11'd0,share};ret<=NORM_NEXT;state<=M_WRITE;
    end
    NORM_NEXT:if(k!=7)begin k<=k+1'b1;state<=NORM_INIT;end else state<=NEXT_X;
    NEXT_X:if(hx==25)state<=NEXT_Y;
     else begin hx<=hx+1'b1;cx<=hx;k<=0;slide<=1;subtract_row<=1;state<=H_READ;end
    NEXT_Y:if(hy==25)state<=RESULT;
     else begin hy<=hy+1'b1;vy<=hy;vx<=0;subtract_row<=1;state<=V_DESC;end
    M_READ:state<=M_TAKE;
    M_TAKE:begin
     read_data[mi*8+:8]<=gray_data_i;
     if(mi+1==mlen)state<=ret;else begin mi<=mi+1'b1;state<=M_READ;end
    end
    M_WRITE:begin
     mdata<={8'd0,mdata[23:8]};
     if(mi+1==mlen)state<=ret;else mi<=mi+1'b1;
    end
    RESULT:if(out_ready_i)state<=IDLE;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
