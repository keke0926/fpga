`timescale 1ns/1ps
// Exact S10 comparison on a leased tile. Operations: 0 initialize, 1 merge
// strong reference, 2 explain weak reference, 3 emit final native candidates.
// Caller runs 1 then 2 for every admitted pose. A reference explanation excludes
// a point, so remaining histogram work for that point can stop immediately.
module part_compare_tile_reduce #(
 parameter DESC_BASE=0,VECTOR_BASE=5292,CENTER_BASE=12348,OBS_HIST_BASE=15148,
 REF_HIST_BASE=21548,FLAGS_BASE=33372
)(
 input wire clk_i,rst_i,abort_i,start_i,input wire[1:0]operation_i,
 input wire[9:0]tile_x_i,tile_y_i,domain_x0_i,domain_y0_i,domain_x1_i,domain_y1_i,
 output wire ready_o,busy_o,valid_o,input wire out_ready_i,
 output wire[15:0]gray_addr_o,output wire gray_we_o,output wire[7:0]gray_data_o,input wire[7:0]gray_data_i,
 output wire[11:0]queue_addr_o,output wire queue_we_o,output wire[15:0]queue_data_o,input wire[15:0]queue_data_i,
 output wire[15:0]state_addr_o,input wire[1:0]state_data_i,
 output wire pixel_valid_o,input wire pixel_ready_i,output wire[4:0]pixel_x_o,pixel_y_o,
 output reg pixel_candidate_o,pixel_clipped_o,output reg[31:0]cycles_o,output reg[8:0]unexplained_o
);
 localparam IDLE=0,INIT=1,P_CENTER=2,P_GOT=3,P_FLAGS=4,P_BRANCH=5,DENOM=6,
 CORNER=7,CORNER_GOT=8,STRONG_READ=9,STRONG_GOT=10,STRONG_NEXT=11,
 QUERY_A=12,QUERY_B=13,QUERY_TEST=14,NEIGHBOR_NEXT=15,FLAG_WRITE=16,POINT_NEXT=17,
 WEAK_OBS=18,WEAK_OBS_GOT=19,WEAK_DESC=20,WEAK_DESC_GOT=21,WEAK_VEC=22,WEAK_VEC_GOT=23,
 DOT_X=24,DOT_Y=25,CROSS_X=26,CROSS_Y=27,WEAK_ANGLE=28,WEAK_SHARE_A=29,WEAK_SHARE_A_GOT=30,
 WEAK_SHARE_B=31,WEAK_SHARE_B_GOT=32,FINAL_READ=33,FINAL_GOT=34,FINAL_NEXT=35,
 DIV_INIT=36,DIV_STEP=37,DIV_DONE=38,CLIP_READ=39,CLIP_TAKE=40,EMIT=41,RESULT=42,
 M_READ=43,M_TAKE=44,M_WRITE=45;
 reg[5:0]state,ret;reg[1:0]op;reg[11:0]init_index;
 reg[4:0]x,y;reg[8:0]point;reg[2:0]nx,ny,k;reg[1:0]corner;
 reg[8:0]angle;reg observation_strong;reg[2:0]flags;
 reg which_div;
 reg[15:0]ma;reg[2:0]mi,mlen;reg[31:0]read_data,mdata;
 reg[12:0]qa,qb,denominator;reg[23:0]query_sum;
 reg signed[15:0]ogx,ogy,rgx,rgy;
 reg signed[32:0]dot,cross_value;reg signed[31:0]term;
 reg[15:0]energy,lo_residual,hi_residual,remainder;reg[12:0]share;reg[3:0]step;
 reg[3:0]clip_x,clip_y;
 wire[2:0]lo_bin=angle[8:6],hi_bin=angle[8:6]+3'd1;
 wire[6:0]frac={1'b0,angle[5:0]},inverse_frac=7'd64-frac;
 wire[9:0]hist_index=(y+{2'd0,ny})*26+x+{2'd0,nx};
 wire[10:0]near_index=(y+{2'd0,ny}+6'd8)*42+x+{2'd0,nx}+6'd8;
 wire[10:0]corner_index=(y+(corner[1]?6'd22:6'd0))*42+x+(corner[0]?6'd22:6'd0);
 wire[10:0]global_x={1'b0,tile_x_i}+x,global_y={1'b0,tile_y_i}+y;
 wire in_domain=global_x>=domain_x0_i&&global_x<domain_x1_i&&global_y>=domain_y0_i&&global_y<domain_y1_i;
 wire[15:0]residual=read_data[15:0]>queue_data_i?read_data[15:0]-queue_data_i:16'd0;
 wire[16:0]shifted={remainder,1'b0};wire div_bit=shifted>={1'b0,energy};
 wire[16:0]reduced=shifted-{1'b0,energy};
 wire[32:0]abs_dot=dot[32]?-dot:dot,abs_cross=cross_value[32]?-cross_value:cross_value;
 wire[36:0]cross10=({4'd0,abs_cross}<<3)+({4'd0,abs_cross}<<1);
 wire[35:0]dot7=({3'd0,abs_dot}<<3)-{3'd0,abs_dot};
 reg signed[17:0]mul_a,mul_b;
 wire signed[35:0]product=mul_a*mul_b;
 always@*begin
  mul_a=$signed({5'd0,qa});mul_b=$signed({11'd0,inverse_frac});
  case(state)
   DENOM:begin mul_a=$signed({11'd0,frac});mul_b=$signed({11'd0,frac});end
   QUERY_B:begin mul_a=$signed({5'd0,qb});mul_b=$signed({11'd0,frac});end
   DOT_X:begin mul_a=ogx;mul_b=rgx;end
   DOT_Y:begin mul_a=ogy;mul_b=rgy;end
   CROSS_X:begin mul_a=ogx;mul_b=rgy;end
   CROSS_Y:begin mul_a=ogy;mul_b=rgx;end
   default:begin end
  endcase
 end
 wire[29:0]query_numerator={query_sum,6'd0};
 wire[29:0]query_limit=op==3?(denominator*30'd738):(denominator*30'd246);
 assign ready_o=state==IDLE&&!rst_i&&!abort_i;
 assign busy_o=state!=IDLE&&state!=RESULT;
 assign valid_o=state==RESULT&&!rst_i&&!abort_i;
 assign gray_addr_o=state==INIT?FLAGS_BASE+init_index:ma+mi;
 assign gray_we_o=!rst_i&&!abort_i&&((state==INIT&&init_index<400)||state==M_WRITE);
 assign gray_data_o=state==INIT?8'd4:mdata[7:0];
 assign queue_addr_o=state==INIT?init_index:{point,3'b000}+k;
 assign queue_we_o=!rst_i&&!abort_i&&(state==INIT||(state==STRONG_GOT&&read_data[15:0]>queue_data_i));
 assign queue_data_o=state==INIT?16'd0:read_data[15:0];
 assign state_addr_o=(y+{1'b0,clip_y}+6'd7)*42+x+{1'b0,clip_x}+6'd7;
 assign pixel_valid_o=state==EMIT&&!rst_i&&!abort_i;
 assign pixel_x_o=x;assign pixel_y_o=y;
 always@(posedge clk_i)begin
  if(rst_i||abort_i)begin
   state<=IDLE;ret<=IDLE;op<=0;init_index<=0;x<=0;y<=0;point<=0;nx<=0;ny<=0;k<=0;corner<=0;
   angle<=0;observation_strong<=0;flags<=0;which_div<=0;
   ma<=0;mi<=0;mlen<=0;read_data<=0;mdata<=0;qa<=0;qb<=0;denominator<=0;query_sum<=0;
   ogx<=0;ogy<=0;rgx<=0;rgy<=0;dot<=0;cross_value<=0;term<=0;energy<=0;lo_residual<=0;hi_residual<=0;
   remainder<=0;share<=0;step<=0;clip_x<=0;clip_y<=0;pixel_candidate_o<=0;pixel_clipped_o<=0;cycles_o<=0;unexplained_o<=0;
  end else begin
   if(busy_o)cycles_o<=cycles_o+1'b1;
   case(state)
    IDLE:if(start_i)begin op<=operation_i;init_index<=0;cycles_o<=0;unexplained_o<=0;x<=0;y<=0;point<=0;state<=operation_i==0?INIT:P_CENTER;end
    INIT:if(init_index==3199)state<=RESULT;else init_index<=init_index+1'b1;
    P_CENTER:begin ma<=CENTER_BASE+point*7+4;mi<=0;mlen<=3;ret<=P_GOT;state<=M_READ;end
    P_GOT:begin angle<=read_data[8:0];observation_strong<=read_data[16];state<=P_FLAGS;end
    P_FLAGS:begin ma<=FLAGS_BASE+point;mi<=0;mlen<=1;ret<=P_BRANCH;state<=M_READ;end
    P_BRANCH:begin
     flags<=read_data[2:0];nx<=0;ny<=0;k<=0;corner<=0;
     pixel_candidate_o<=0;pixel_clipped_o<=0;
     if(!observation_strong||!in_domain||!read_data[2]||read_data[0]||(op!=1&&read_data[1]))state<=op==3?EMIT:POINT_NEXT;
     else state<=DENOM;
    end
    DENOM:begin
     denominator<=13'd4096-({6'd0,frac}<<7)+(product[12:0]<<1);
     if(op==1)state<=CORNER;
     else if(op==2)state<=WEAK_OBS;
     else begin energy<=0;lo_residual<=0;hi_residual<=0;state<=FINAL_READ;end
    end
    CORNER:begin ma<=DESC_BASE+corner_index*3+2;mi<=0;mlen<=1;ret<=CORNER_GOT;state<=M_READ;end
    CORNER_GOT:if(!read_data[3])begin flags[2]<=0;state<=FLAG_WRITE;end
     else if(corner==3)state<=STRONG_READ;else begin corner<=corner+1'b1;state<=CORNER;end
    STRONG_READ:begin ma<=REF_HIST_BASE+({3'd0,hist_index,3'd0}+k)*2;mi<=0;mlen<=2;ret<=STRONG_GOT;state<=M_READ;end
    STRONG_GOT:begin if(k==lo_bin)qa<=read_data[12:0];if(k==hi_bin)qb<=read_data[12:0];state<=STRONG_NEXT;end
    STRONG_NEXT:if(k==7)state<=QUERY_A;else begin k<=k+1'b1;state<=STRONG_READ;end
    QUERY_A:begin query_sum<=product[23:0];state<=QUERY_B;end
    QUERY_B:begin query_sum<=query_sum+product[23:0];state<=QUERY_TEST;end
    QUERY_TEST:begin
     if(op==3)begin
      pixel_candidate_o<=query_numerator>=query_limit;clip_x<=0;clip_y<=0;
      state<=query_numerator>=query_limit?CLIP_READ:EMIT;
     end else if(op==1)begin
      if(query_numerator>=query_limit)begin flags[0]<=1;state<=FLAG_WRITE;end else state<=NEIGHBOR_NEXT;
     end else begin
      if(query_numerator>=query_limit)begin flags[1]<=1;state<=FLAG_WRITE;end
      else state<=NEIGHBOR_NEXT;
     end
    end
    NEIGHBOR_NEXT:begin
     k<=0;
     if(nx!=6)begin nx<=nx+1'b1;state<=op==1?STRONG_READ:WEAK_DESC;end
     else if(ny!=6)begin nx<=0;ny<=ny+1'b1;state<=op==1?STRONG_READ:WEAK_DESC;end
     else state<=FLAG_WRITE;
    end
    FLAG_WRITE:begin ma<=FLAGS_BASE+point;mi<=0;mlen<=1;mdata<={29'd0,flags};ret<=POINT_NEXT;state<=M_WRITE;end
    POINT_NEXT:begin
     if(observation_strong&&in_domain&&flags[2]&&!flags[0]&&!flags[1])unexplained_o<=unexplained_o+1'b1;
     if(point==399)state<=RESULT;
     else begin point<=point+1'b1;if(x==19)begin x<=0;y<=y+1'b1;end else x<=x+1'b1;state<=P_CENTER;end
    end
    WEAK_OBS:begin ma<=CENTER_BASE+point*7;mi<=0;mlen<=4;ret<=WEAK_OBS_GOT;state<=M_READ;end
    WEAK_OBS_GOT:begin ogx<=read_data[15:0];ogy<=read_data[31:16];state<=WEAK_DESC;end
    // Require both weak-reference tests at the same sampled neighbor.
    WEAK_DESC:begin ma<=DESC_BASE+near_index*3+2;mi<=0;mlen<=1;ret<=WEAK_DESC_GOT;state<=M_READ;end
    WEAK_DESC_GOT:state<=read_data[1]?WEAK_VEC:NEIGHBOR_NEXT;
    WEAK_VEC:begin ma<=VECTOR_BASE+near_index*4;mi<=0;mlen<=4;ret<=WEAK_VEC_GOT;state<=M_READ;end
    WEAK_VEC_GOT:begin rgx<=read_data[15:0];rgy<=read_data[31:16];state<=DOT_X;end
    DOT_X:begin term<=product[31:0];state<=DOT_Y;end
    DOT_Y:begin dot<=$signed({term[31],term})+product;state<=CROSS_X;end
    CROSS_X:begin term<=product[31:0];state<=CROSS_Y;end
    CROSS_Y:begin cross_value<=$signed({term[31],term})-product;state<=WEAK_ANGLE;end
    WEAK_ANGLE:state<=cross10<=dot7?WEAK_SHARE_A:NEIGHBOR_NEXT;
    WEAK_SHARE_A:begin ma<=REF_HIST_BASE+({3'd0,hist_index,3'd0}+lo_bin)*2;mi<=0;mlen<=2;ret<=WEAK_SHARE_A_GOT;state<=M_READ;end
    WEAK_SHARE_A_GOT:begin qa<=read_data[12:0];state<=WEAK_SHARE_B;end
    WEAK_SHARE_B:begin ma<=REF_HIST_BASE+({3'd0,hist_index,3'd0}+hi_bin)*2;mi<=0;mlen<=2;ret<=WEAK_SHARE_B_GOT;state<=M_READ;end
    WEAK_SHARE_B_GOT:begin qb<=read_data[12:0];state<=QUERY_A;end
    FINAL_READ:begin ma<=OBS_HIST_BASE+({4'd0,point,3'd0}+k)*2;mi<=0;mlen<=2;ret<=FINAL_GOT;state<=M_READ;end
    FINAL_GOT:begin
     energy<=energy+residual;if(k==lo_bin)lo_residual<=residual;if(k==hi_bin)hi_residual<=residual;state<=FINAL_NEXT;
    end
    FINAL_NEXT:if(k==7)begin which_div<=0;state<=DIV_INIT;end else begin k<=k+1'b1;state<=FINAL_READ;end
    DIV_INIT:begin
     remainder<=which_div?hi_residual:lo_residual;share<=0;step<=12;
     if(energy==0)state<=DIV_DONE;
     else if((which_div?hi_residual:lo_residual)==energy)begin share<=4096;state<=DIV_DONE;end
     else state<=DIV_STEP;
    end
    DIV_STEP:begin
     remainder<=div_bit?reduced[15:0]:shifted[15:0];share<={share[11:0],div_bit};step<=step-1'b1;
     if(step==1)state<=DIV_DONE;
    end
    DIV_DONE:if(!which_div)begin qa<=share;which_div<=1;state<=DIV_INIT;end else begin qb<=share;state<=QUERY_A;end
    CLIP_READ:state<=CLIP_TAKE;
    CLIP_TAKE:if(state_data_i[0]&&!state_data_i[1])begin pixel_clipped_o<=1;state<=EMIT;end
     else if(clip_x==8&&clip_y==8)state<=EMIT;
     else begin if(clip_x==8)begin clip_x<=0;clip_y<=clip_y+1'b1;end else clip_x<=clip_x+1'b1;state<=CLIP_READ;end
    EMIT:if(pixel_ready_i)state<=POINT_NEXT;
    M_READ:state<=M_TAKE;
    M_TAKE:begin read_data[mi*8+:8]<=gray_data_i;if(mi+1==mlen)state<=ret;else begin mi<=mi+1'b1;state<=M_READ;end end
    M_WRITE:begin mdata<={8'd0,mdata[31:8]};if(mi+1==mlen)state<=ret;else mi<=mi+1'b1;end
    RESULT:if(out_ready_i)state<=IDLE;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
