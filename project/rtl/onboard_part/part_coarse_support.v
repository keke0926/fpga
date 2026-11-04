`timescale 1ns/1ps
// Shared-scratch count/label BFS; queue marks precede enqueue. No private RAM.
module part_coarse_support #(
 parameter integer GRID_WIDTH=128,GRID_HEIGHT=75,QUEUE_DEPTH=4096
)(
 input wire clk_i,rst_i,abort_i,start_i,
 output wire ready_o,busy_o,done_o,result_valid_o,input wire out_ready_i,
 input wire[15:0] count_base_i,label_base_i,input wire[7:0] threshold_i,
 output reg[3:0] error_o,
 output reg[15:0] scratch_addr_o,output reg scratch_we_o,output reg[7:0] scratch_data_o,input wire[7:0] scratch_data_i,
 output reg[11:0] queue_addr_o,output reg queue_we_o,output reg[15:0] queue_data_o,input wire[15:0] queue_data_i,
 output reg found_o,ambiguous_o,
 output reg[13:0] best_count_o,second_count_o,
 output reg[7:0] best_label_o,component_count_o,
 output reg[31:0] cycles_o,output reg[12:0] queue_peak_o
);
 localparam integer CELLS=GRID_WIDTH*GRID_HEIGHT;
 localparam[5:0] IDLE=0,INIT_LABELS=1,SCAN_COUNT_REQ=2,SCAN_COUNT_WAIT=3,
   SCAN_LABEL_REQ=4,SCAN_LABEL_WAIT=5,SCAN_NEXT=6,MARK=7,ENQUEUE=8,
   BFS_CHECK=9,DEQUEUE_REQ=10,DEQUEUE_WAIT=11,NEIGHBOR_CHECK=12,
   NEIGHBOR_COUNT_REQ=13,NEIGHBOR_COUNT_WAIT=14,NEIGHBOR_LABEL_REQ=15,NEIGHBOR_LABEL_WAIT=16,
   NEIGHBOR_NEXT=17,COMPONENT_DONE=18,MASK_LABEL_REQ=19,MASK_LABEL_WAIT=20,
   MASK_ZERO=21,MASK_NEXT=22,FINISH=23,RESULT=24;
 reg[5:0] state;
 reg[15:0] count_base,label_base;
 reg[7:0] threshold,label_value;
 reg[13:0] index,scan_index,target_index,component_size;
 reg[6:0] scan_x,scan_y,target_x,target_y,center_x,center_y;
 reg signed[4:0] dx,dy;
 reg seed_enqueue;
 reg[11:0] queue_head,queue_tail;
 reg[12:0] queue_count;
 wire[16:0] count_end={1'b0,count_base_i}+CELLS;
 wire[16:0] label_end={1'b0,label_base_i}+CELLS;
 wire signed[8:0] nx=$signed({2'b0,center_x})+{{4{dx[4]}},dx};
 wire signed[8:0] ny=$signed({2'b0,center_y})+{{4{dy[4]}},dy};
 wire neighbor_in_domain=nx>=0&&ny>=0&&nx<GRID_WIDTH&&ny<GRID_HEIGHT;
 wire[13:0] neighbor_index=$unsigned(ny)*GRID_WIDTH+$unsigned(nx);
 // Shift/add comparisons retain up to 9600*100=960000 exactly.
 wire[20:0] best_extended={7'd0,best_count_o},second_extended={7'd0,second_count_o};
 wire[20:0] best65=(best_extended<<6)+best_extended;
 wire[20:0] second100=(second_extended<<6)+(second_extended<<5)+(second_extended<<2);
 assign ready_o=state==IDLE&&!rst_i&&!abort_i;
 assign busy_o=state!=IDLE;
 assign result_valid_o=state==RESULT&&!rst_i&&!abort_i;
 assign done_o=result_valid_o;
 always @*begin
  scratch_addr_o=count_base;scratch_we_o=0;scratch_data_o=0;
  queue_addr_o=queue_head;queue_we_o=0;queue_data_o={2'b0,target_y,target_x};
  case(state)
   INIT_LABELS:begin scratch_addr_o=label_base+index;scratch_we_o=1;end
   SCAN_COUNT_REQ,SCAN_COUNT_WAIT:scratch_addr_o=count_base+scan_index;
   SCAN_LABEL_REQ,SCAN_LABEL_WAIT:scratch_addr_o=label_base+scan_index;
   MARK:begin scratch_addr_o=label_base+target_index;scratch_we_o=1;scratch_data_o=label_value;end
   ENQUEUE:begin queue_addr_o=queue_tail;queue_we_o=1;end
   NEIGHBOR_COUNT_REQ,NEIGHBOR_COUNT_WAIT:scratch_addr_o=count_base+target_index;
   NEIGHBOR_LABEL_REQ,NEIGHBOR_LABEL_WAIT:scratch_addr_o=label_base+target_index;
   MASK_LABEL_REQ,MASK_LABEL_WAIT:scratch_addr_o=label_base+index;
   MASK_ZERO:begin scratch_addr_o=count_base+index;scratch_we_o=1;end
   default:begin end
  endcase
  if(rst_i||abort_i)begin scratch_we_o=0;queue_we_o=0;end
 end
 always @(posedge clk_i)begin
  if(rst_i)begin
   state<=IDLE;error_o<=0;found_o<=0;ambiguous_o<=0;best_count_o<=0;second_count_o<=0;
   best_label_o<=0;component_count_o<=0;cycles_o<=0;queue_peak_o<=0;
   count_base<=0;label_base<=0;threshold<=0;label_value<=0;
   index<=0;scan_index<=0;target_index<=0;component_size<=0;
   scan_x<=0;scan_y<=0;target_x<=0;target_y<=0;center_x<=0;center_y<=0;dx<=0;dy<=0;
   seed_enqueue<=0;queue_head<=0;queue_tail<=0;queue_count<=0;
  end else if(abort_i)begin
   state<=IDLE;error_o<=0;found_o<=0;ambiguous_o<=0;queue_count<=0;
  end else begin
   if(state!=IDLE&&state!=RESULT)cycles_o<=cycles_o+1'b1;
   case(state)
    IDLE:if(start_i&&ready_o)begin
     count_base<=count_base_i;label_base<=label_base_i;threshold<=threshold_i;
     error_o<=0;found_o<=0;ambiguous_o<=0;best_count_o<=0;second_count_o<=0;best_label_o<=0;
     component_count_o<=0;cycles_o<=0;queue_peak_o<=0;index<=0;scan_index<=0;scan_x<=0;scan_y<=0;
     queue_head<=0;queue_tail<=0;queue_count<=0;component_size<=0;label_value<=0;
     if(GRID_WIDTH<1||GRID_WIDTH>128||GRID_HEIGHT<1||GRID_HEIGHT>128||CELLS>9600||QUEUE_DEPTH<1||QUEUE_DEPTH>4096
        ||threshold_i<1||threshold_i>64||count_end>17'd65536||label_end>17'd65536
        ||!((count_end<={1'b0,label_base_i})||(label_end<={1'b0,count_base_i})))begin
      error_o<=1;state<=FINISH;
     end else state<=INIT_LABELS;
    end
    INIT_LABELS:if(index+1==CELLS)state<=SCAN_COUNT_REQ;else index<=index+1'b1;
    SCAN_COUNT_REQ:state<=SCAN_COUNT_WAIT;
    SCAN_COUNT_WAIT:state<=scratch_data_i>=threshold?SCAN_LABEL_REQ:SCAN_NEXT;
    SCAN_LABEL_REQ:state<=SCAN_LABEL_WAIT;
    SCAN_LABEL_WAIT:if(scratch_data_i!=0)state<=SCAN_NEXT;
    else if(component_count_o==254)begin error_o<=2;state<=FINISH;end
    else begin
     label_value<=component_count_o+1'b1;component_count_o<=component_count_o+1'b1;
     component_size<=0;queue_count<=0;queue_head<=0;queue_tail<=0;
     target_index<=scan_index;target_x<=scan_x;target_y<=scan_y;seed_enqueue<=1;state<=MARK;
    end
    SCAN_NEXT:if(scan_index+1==CELLS)begin index<=0;state<=MASK_LABEL_REQ;end
    else begin
     scan_index<=scan_index+1'b1;
     if(scan_x+1==GRID_WIDTH)begin scan_x<=0;scan_y<=scan_y+1'b1;end else scan_x<=scan_x+1'b1;
     state<=SCAN_COUNT_REQ;
    end
    MARK:state<=ENQUEUE;
    ENQUEUE:begin
     queue_tail<=queue_tail+1==QUEUE_DEPTH?12'd0:queue_tail+1'b1;
     queue_count<=queue_count+1'b1;component_size<=component_size+1'b1;
     if(queue_count+1>queue_peak_o)queue_peak_o<=queue_count+1'b1;
     state<=seed_enqueue?BFS_CHECK:NEIGHBOR_NEXT;
    end
    BFS_CHECK:state<=queue_count==0?COMPONENT_DONE:DEQUEUE_REQ;
    DEQUEUE_REQ:state<=DEQUEUE_WAIT;
    DEQUEUE_WAIT:begin
     center_x<=queue_data_i[6:0];center_y<=queue_data_i[13:7];
     queue_head<=queue_head+1==QUEUE_DEPTH?12'd0:queue_head+1'b1;queue_count<=queue_count-1'b1;
     dx<=-5;dy<=-5;state<=NEIGHBOR_CHECK;
    end
    NEIGHBOR_CHECK:if(neighbor_in_domain)begin
     target_x<=nx[6:0];target_y<=ny[6:0];target_index<=neighbor_index;state<=NEIGHBOR_COUNT_REQ;
    end else state<=NEIGHBOR_NEXT;
    NEIGHBOR_COUNT_REQ:state<=NEIGHBOR_COUNT_WAIT;
    NEIGHBOR_COUNT_WAIT:state<=scratch_data_i>=threshold?NEIGHBOR_LABEL_REQ:NEIGHBOR_NEXT;
    NEIGHBOR_LABEL_REQ:state<=NEIGHBOR_LABEL_WAIT;
    NEIGHBOR_LABEL_WAIT:if(scratch_data_i!=0)state<=NEIGHBOR_NEXT;
    else if(queue_count>=QUEUE_DEPTH)begin error_o<=3;state<=FINISH;end
    else begin seed_enqueue<=0;state<=MARK;end
    NEIGHBOR_NEXT:if(dx==5)begin
     dx<=-5;
     if(dy==5)state<=BFS_CHECK;
     else begin dy<=dy+1'b1;state<=NEIGHBOR_CHECK;end
    end else begin dx<=dx+1'b1;state<=NEIGHBOR_CHECK;end
    COMPONENT_DONE:begin
     if(component_size>best_count_o)begin
      second_count_o<=best_count_o;best_count_o<=component_size;best_label_o<=label_value;
     end else if(component_size>second_count_o)second_count_o<=component_size;
     state<=SCAN_NEXT;
    end
    MASK_LABEL_REQ:state<=MASK_LABEL_WAIT;
    MASK_LABEL_WAIT:state<=best_label_o!=0&&scratch_data_i==best_label_o?MASK_NEXT:MASK_ZERO;
    MASK_ZERO:state<=MASK_NEXT;
    MASK_NEXT:if(index+1==CELLS)state<=FINISH;else begin index<=index+1'b1;state<=MASK_LABEL_REQ;end
    FINISH:begin
     found_o<=error_o==0&&best_count_o!=0;
     ambiguous_o<=error_o==0&&best_count_o!=0&&second100>=best65;
     state<=RESULT;
    end
    RESULT:if(out_ready_i)state<=IDLE;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
