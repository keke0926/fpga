`timescale 1ns/1ps
// Fixed-point finite-ROI complete Canny, with external synchronous scratch RAM.
// Input: 16384 grayscale bytes at gray[0..16383]. Work: gray[0..32767]
// and state[0..16383] (2 bits). No RAM is declared or reset in this controller.
// Gaussian uses gray's second half; Sobel magnitude overwrites its first half.
// After NMS, all 32768 bytes become a 16384-entry two-byte FIFO. Each pixel is
// marked before enqueue, so queue capacity covers even a whole connected ROI.
// Only completed hysteresis is streamed; abort publishes no completion.
// A caller must own these memories exclusively for the whole transaction.
module canny_shared_scratch(
 input wire clk,rst_n,abort_i,start_i,
 input wire hysteresis_only_i,
 input wire [7:0] low_i,high_i,
 input wire [31:0] frame_id_i,config_tag_i,
 output wire busy_o,
 output wire done_o,valid_o,
 output reg [3:0] error_o,
 output reg [3:0] phase_o,
 output reg [31:0] frame_id_o,config_tag_o,cycles_o,
 output reg [14:0] gray_addr_o,
 output reg gray_we_o,
 output reg [7:0] gray_data_o,
 input wire [7:0] gray_data_i,
 output reg [13:0] state_addr_o,
 output reg state_we_o,
 output reg [1:0] state_data_o,
 input wire [1:0] state_data_i,
 output wire edge_valid_o,
 output reg [13:0] edge_addr_o,
 output reg edge_bit_o
);
 localparam IDLE=0,G_REQ=1,G_WAIT=2,G_WRITE=3,D_REQ=4,D_WAIT=5,D_WRITE=6,
 N_DIR_REQ=7,N_DIR_WAIT=8,N_CENTER_REQ=9,N_CENTER_WAIT=10,N_A_REQ=11,N_A_WAIT=12,
 N_B_REQ=13,N_B_WAIT=14,N_WRITE=15,SEED_REQ=16,SEED_WAIT=17,SEED_MARK=18,
 PUSH_LOW=19,PUSH_HIGH=20,POP_LOW_REQ=21,POP_LOW_WAIT=22,POP_HIGH_REQ=23,
 POP_HIGH_WAIT=24,NEIGH_REQ=25,NEIGH_WAIT=26,NEIGH_MARK=27,NEIGH_NEXT=28,
 SEED_NEXT=29,OUT_REQ=30,OUT_WAIT=31,FINISH=32;
 reg [5:0] state;
 reg done_pending,valid_pending,edge_pending;
 assign done_o=done_pending && rst_n && !abort_i;
 assign valid_o=valid_pending && rst_n && !abort_i;
 assign edge_valid_o=edge_pending && rst_n && !abort_i;
 assign busy_o=state!=IDLE;
 reg [13:0] index,scan,node,push_node;
 reg [3:0] tap;
 reg [2:0] neighbor;
 reg [7:0] low,high,center_mag,side_a,side_b,pop_low;
 reg [1:0] direction;
 reg [11:0] gauss_sum;
 reg signed [12:0] gx,gy;
 reg [13:0] qhead,qtail;
 reg [14:0] qcount;
 reg push_seed;
 wire [12:0] ax=gx[12] ? -gx : gx;
 wire [12:0] ay=gy[12] ? -gy : gy;
 wire [13:0] magnitude_sum={1'b0,ax}+{1'b0,ay}+14'd4;
 wire [7:0] magnitude=magnitude_sum[10:3];
 wire [16:0] ax5=ax*5,ay5=ay*5,ax12=ax*12,ay12=ay*12;
 wire [1:0] gradient_dir=ay12<=ax5 ? 2'd0 : ax12<=ay5 ? 2'd2 :
                                (gx[12]==gy[12] ? 2'd1 : 2'd3);
 wire border=index[6:0]==0 || index[6:0]==127 || index[13:7]==0 || index[13:7]==127;
 wire [1:0] nms_state=(!border && center_mag>=low && center_mag>=side_a && center_mag>=side_b)
                          ? (center_mag>=high ? 2'd2 : 2'd1) : 2'd0;
 reg [13:0] tap_address,side_address,neighbor_address;
 reg neighbor_ok;
 integer tx,ty,sx,sy,nx,ny,dx,dy;
 reg [1:0] weight_shift;
 wire [11:0] tap_weighted={4'b0,gray_data_i} << weight_shift;

 function signed [12:0] x_term;
 input [3:0] t;input [7:0] p;
 reg signed [12:0] v;
 begin
 v=$signed({5'b0,p});
 case(t)
  0,6:x_term=-v;3:x_term=-(v<<<1);
  2,8:x_term=v;5:x_term=v<<<1;default:x_term=0;
 endcase
 end
 endfunction
 function signed [12:0] y_term;
 input [3:0] t;input [7:0] p;
 reg signed [12:0] v;
 begin
 v=$signed({5'b0,p});
 case(t)
  0,2:y_term=-v;1:y_term=-(v<<<1);
  6,8:y_term=v;7:y_term=v<<<1;default:y_term=0;
 endcase
 end
 endfunction

 always @*begin
  tx={25'b0,index[6:0]};ty={25'b0,index[13:7]};
  case(tap)
   0:begin tx=tx-1;ty=ty-1;end
   1:ty=ty-1;
   2:begin tx=tx+1;ty=ty-1;end
   3:tx=tx-1;
   5:tx=tx+1;
   6:begin tx=tx-1;ty=ty+1;end
   7:ty=ty+1;
   8:begin tx=tx+1;ty=ty+1;end
  endcase
  if(tx<0)tx=0;else if(tx>127)tx=127;
  if(ty<0)ty=0;else if(ty>127)ty=127;
  tap_address={ty[6:0],tx[6:0]};
  case(tap)
   4:weight_shift=2;
   1,3,5,7:weight_shift=1;
   default:weight_shift=0;
  endcase
  dx=1;dy=0;
  case(direction)
   1:dy=1;
   2:begin dx=0;dy=1;end
   3:dy=-1;
  endcase
  sx={25'b0,index[6:0]};sy={25'b0,index[13:7]};
  if(state==N_A_REQ || state==N_A_WAIT)begin sx=sx-dx;sy=sy-dy;end
  else begin sx=sx+dx;sy=sy+dy;end
  // NMS discards this border; clamping prevents unrelated scratch reads.
  if(sx<0)sx=0;else if(sx>127)sx=127;
  if(sy<0)sy=0;else if(sy>127)sy=127;
  side_address={sy[6:0],sx[6:0]};
  nx={25'b0,node[6:0]};ny={25'b0,node[13:7]};
  case(neighbor)
   0:begin nx=nx-1;ny=ny-1;end
   1:ny=ny-1;
   2:begin nx=nx+1;ny=ny-1;end
   3:nx=nx-1;
   4:nx=nx+1;
   5:begin nx=nx-1;ny=ny+1;end
   6:ny=ny+1;
   7:begin nx=nx+1;ny=ny+1;end
  endcase
  neighbor_ok=nx>=0 && nx<128 && ny>=0 && ny<128;
  neighbor_address=neighbor_ok ? {ny[6:0],nx[6:0]} : 14'd0;
 end

 always @*begin
  gray_addr_o=0;gray_we_o=0;gray_data_o=0;
  state_addr_o=0;state_we_o=0;state_data_o=0;phase_o=0;
  if(rst_n && !abort_i)case(state)
   G_REQ,G_WAIT:begin phase_o=1;gray_addr_o={1'b0,tap_address};end
   G_WRITE:begin phase_o=1;gray_addr_o={1'b1,index};gray_we_o=1;
      gray_data_o=(gauss_sum+12'd8)>>4;end
   D_REQ,D_WAIT:begin phase_o=2;gray_addr_o={1'b1,tap_address};end
   D_WRITE:begin phase_o=2;gray_addr_o={1'b0,index};gray_we_o=1;gray_data_o=magnitude;
      state_addr_o=index;state_we_o=1;state_data_o=gradient_dir;end
   N_DIR_REQ,N_DIR_WAIT:begin phase_o=3;state_addr_o=index;end
   N_CENTER_REQ,N_CENTER_WAIT:begin phase_o=3;gray_addr_o={1'b0,index};end
   N_A_REQ,N_A_WAIT,N_B_REQ,N_B_WAIT:begin phase_o=3;gray_addr_o={1'b0,side_address};end
   N_WRITE:begin phase_o=3;state_addr_o=index;state_we_o=1;state_data_o=nms_state;end
   SEED_REQ,SEED_WAIT:begin phase_o=4;state_addr_o=scan;end
   SEED_MARK:begin phase_o=4;state_addr_o=scan;state_we_o=1;state_data_o=3;end
   PUSH_LOW:begin phase_o=4;gray_addr_o={qtail,1'b0};gray_we_o=1;gray_data_o=push_node[7:0];end
   PUSH_HIGH:begin phase_o=4;gray_addr_o={qtail,1'b1};gray_we_o=1;gray_data_o={2'b0,push_node[13:8]};end
   POP_LOW_REQ,POP_LOW_WAIT:begin phase_o=4;gray_addr_o={qhead,1'b0};end
   POP_HIGH_REQ,POP_HIGH_WAIT:begin phase_o=4;gray_addr_o={qhead,1'b1};end
   NEIGH_REQ,NEIGH_WAIT:begin phase_o=4;state_addr_o=neighbor_address;end
   NEIGH_MARK:begin phase_o=4;state_addr_o=push_node;state_we_o=1;state_data_o=3;end
   NEIGH_NEXT,SEED_NEXT:phase_o=4;
   OUT_REQ,OUT_WAIT:begin phase_o=5;state_addr_o=index;end
  endcase
 end

 always @(posedge clk)begin
  if(!rst_n || abort_i)begin
   state<=IDLE;done_pending<=0;valid_pending<=0;error_o<=0;frame_id_o<=0;config_tag_o<=0;cycles_o<=0;
   edge_pending<=0;edge_addr_o<=0;edge_bit_o<=0;
   index<=0;scan<=0;node<=0;push_node<=0;tap<=0;neighbor<=0;
   low<=0;high<=0;center_mag<=0;side_a<=0;side_b<=0;pop_low<=0;direction<=0;
   gauss_sum<=0;gx<=0;gy<=0;qhead<=0;qtail<=0;qcount<=0;push_seed<=0;
  end else begin
   done_pending<=0;valid_pending<=0;edge_pending<=0;
   if(state!=IDLE)cycles_o<=cycles_o+1'b1;
   case(state)
    IDLE:if(start_i)begin
     cycles_o<=0;error_o<=0;frame_id_o<=frame_id_i;config_tag_o<=config_tag_i;
     low<=low_i;high<=high_i;index<=0;scan<=0;tap<=0;gauss_sum<=0;gx<=0;gy<=0;
     qhead<=0;qtail<=0;qcount<=0;
     if(low_i==0 || high_i<low_i)begin done_pending<=1;error_o<=1;end
     else state<=hysteresis_only_i ? SEED_REQ : G_REQ;
    end
    G_REQ:state<=G_WAIT;
    G_WAIT:begin
     gauss_sum<=gauss_sum+tap_weighted;
     if(tap==8)state<=G_WRITE;else begin tap<=tap+1'b1;state<=G_REQ;end
    end
    G_WRITE:begin
     gauss_sum<=0;tap<=0;
     if(index==16383)begin index<=0;gx<=0;gy<=0;state<=D_REQ;end
     else begin index<=index+1'b1;state<=G_REQ;end
    end
    D_REQ:state<=D_WAIT;
    D_WAIT:begin
     gx<=gx+x_term(tap,gray_data_i);gy<=gy+y_term(tap,gray_data_i);
     if(tap==8)state<=D_WRITE;else begin tap<=tap+1'b1;state<=D_REQ;end
    end
    D_WRITE:begin
     tap<=0;gx<=0;gy<=0;
     if(index==16383)begin index<=0;state<=N_DIR_REQ;end
     else begin index<=index+1'b1;state<=D_REQ;end
    end
    N_DIR_REQ:state<=N_DIR_WAIT;
    N_DIR_WAIT:begin direction<=state_data_i;state<=N_CENTER_REQ;end
    N_CENTER_REQ:state<=N_CENTER_WAIT;
    N_CENTER_WAIT:begin center_mag<=gray_data_i;state<=N_A_REQ;end
    N_A_REQ:state<=N_A_WAIT;
    N_A_WAIT:begin side_a<=gray_data_i;state<=N_B_REQ;end
    N_B_REQ:state<=N_B_WAIT;
    N_B_WAIT:begin side_b<=gray_data_i;state<=N_WRITE;end
    N_WRITE:if(index==16383)begin scan<=0;state<=SEED_REQ;end
            else begin index<=index+1'b1;state<=N_DIR_REQ;end
    SEED_REQ:state<=SEED_WAIT;
    SEED_WAIT:if(state_data_i==2)begin
     qhead<=0;qtail<=0;qcount<=0;push_node<=scan;push_seed<=1;state<=SEED_MARK;
    end else state<=SEED_NEXT;
    SEED_MARK:state<=PUSH_LOW;
    PUSH_LOW:state<=PUSH_HIGH;
    PUSH_HIGH:begin
     qtail<=qtail+1'b1;qcount<=qcount+1'b1;
     state<=push_seed ? POP_LOW_REQ : NEIGH_NEXT;
    end
    POP_LOW_REQ:state<=POP_LOW_WAIT;
    POP_LOW_WAIT:begin pop_low<=gray_data_i;state<=POP_HIGH_REQ;end
    POP_HIGH_REQ:state<=POP_HIGH_WAIT;
    POP_HIGH_WAIT:begin
     node<={gray_data_i[5:0],pop_low};qhead<=qhead+1'b1;qcount<=qcount-1'b1;
     neighbor<=0;state<=NEIGH_REQ;
    end
    NEIGH_REQ:state<=neighbor_ok ? NEIGH_WAIT : NEIGH_NEXT;
    NEIGH_WAIT:if(state_data_i==1 || state_data_i==2)begin
     if(qcount==16384)begin state<=IDLE;done_pending<=1;error_o<=2;end
     else begin push_node<=neighbor_address;push_seed<=0;state<=NEIGH_MARK;end
    end else state<=NEIGH_NEXT;
    NEIGH_MARK:state<=PUSH_LOW;
    NEIGH_NEXT:if(neighbor==7)state<=qcount!=0 ? POP_LOW_REQ : SEED_NEXT;
               else begin neighbor<=neighbor+1'b1;state<=NEIGH_REQ;end
    SEED_NEXT:if(scan==16383)begin index<=0;state<=OUT_REQ;end
              else begin scan<=scan+1'b1;state<=SEED_REQ;end
    OUT_REQ:state<=OUT_WAIT;
    OUT_WAIT:begin
     edge_pending<=1;edge_addr_o<=index;edge_bit_o<=state_data_i==3;
     if(index==16383)state<=FINISH;else begin index<=index+1'b1;state<=OUT_REQ;end
    end
    FINISH:begin done_pending<=1;valid_pending<=1;state<=IDLE;end
    default:begin state<=IDLE;done_pending<=1;error_o<=3;end
   endcase
  end
 end
endmodule
