`timescale 1ns/1ps
// Exact native-feature strong counts for 8x8 cells. One multiplier, no RAM.
module part_coarse_builder #(
 parameter integer WIDTH=1024,HEIGHT=600,STATIC_CONTEXT=0
)(
 input wire clk_i,rst_i,abort_i,start_i,
 output wire ready_o,busy_o,done_o,result_valid_o,input wire out_ready_i,
 input wire[31:0] feature_base_i,task_id_i,frame_id_i,reference_id_i,config_epoch_i,
 input wire[7:0] raw_floor_i,
 output wire[31:0] task_id_o,frame_id_o,reference_id_o,config_epoch_o,
 output reg[3:0] error_o,
 output wire grid_valid_o,input wire grid_ready_i,
 output reg[13:0] grid_addr_o,output wire[7:0] grid_data_o,
 output reg[31:0] read_requests_o,cycles_o,
 output reg[13:0] cells_written_o,
 output wire req_valid_o,input wire req_ready_i,output wire req_write_o,
 output wire[31:0] req_addr_o,output wire[127:0] req_data_o,
 input wire rsp_valid_i,output wire rsp_ready_o,input wire rsp_error_i,input wire[127:0] rsp_data_i
);
 // Parent keeps all identity buses stable until result consumption or drain.
 reg[31:0]task_id_held;assign task_id_o=STATIC_CONTEXT?task_id_i:task_id_held;
 reg[31:0]frame_id_held;assign frame_id_o=STATIC_CONTEXT?frame_id_i:frame_id_held;
 reg[31:0]reference_id_held;assign reference_id_o=STATIC_CONTEXT?reference_id_i:reference_id_held;
 reg[31:0]config_epoch_held;assign config_epoch_o=STATIC_CONTEXT?config_epoch_i:config_epoch_held;

 localparam [3:0] IDLE=0,REQUEST=1,RESPONSE=2,SQUARE_X=3,SQUARE_Y=4,
   NEXT_BEAT=5,OUTPUT_CELL=6,FINISH=7,RESULT=8,DRAIN=9;
 localparam integer GRID_WIDTH=WIDTH/8,GRID_HEIGHT=HEIGHT/8;
 localparam [32:0] IMAGE_BYTES=WIDTH*HEIGHT*4;
 reg[3:0] state;
 reg[6:0] cell_x,cell_y;
 reg[2:0] row;
 reg group;
 reg[1:0] lane;
 reg[31:0] request_addr,cell_start_addr;
 reg[127:0] payload;
 reg[7:0] cell_count;
 reg[7:0] raw_floor;
 reg[21:0] square_x;
 wire context_lost=(task_id_i!=task_id_o)||(frame_id_i!=frame_id_o)||
                   (reference_id_i!=reference_id_o)||(config_epoch_i!=config_epoch_o);
 wire cancel=abort_i||(state!=IDLE&&state!=DRAIN&&context_lost);
 wire[31:0] word_value=payload>>((3-lane)*32);
 wire signed[10:0] operand=state==SQUARE_X?$signed(word_value[10:0]):$signed(word_value[21:11]);
 wire signed[21:0] product=operand*operand;
 // Each square <=2^20; sum <=2^21; 25*sum <=52428800.
 wire[22:0] magnitude={1'b0,square_x}+{1'b0,$unsigned(product)};
 wire[27:0] magnitude_extended={5'd0,magnitude};
 wire[27:0] magnitude25=(magnitude_extended<<4)+(magnitude_extended<<3)+magnitude_extended;
 wire strong_pixel=magnitude25>=28'd4096&&word_value[29:22]>=raw_floor;
 assign ready_o=state==IDLE&&!rst_i&&!abort_i;
 assign busy_o=state!=IDLE;
 assign result_valid_o=state==RESULT&&!rst_i&&!cancel;
 assign done_o=result_valid_o;
 assign grid_valid_o=state==OUTPUT_CELL&&!rst_i&&!cancel;
 assign grid_data_o=cell_count;
 assign req_valid_o=state==REQUEST&&!rst_i&&!cancel;
 assign req_write_o=1'b0;
 assign req_addr_o=request_addr;
 assign req_data_o=128'd0;
 assign rsp_ready_o=!rst_i&&(state==RESPONSE||state==DRAIN);
 always @(posedge clk_i)begin
  if(rst_i)begin
   state<=IDLE;task_id_held<=0;frame_id_held<=0;reference_id_held<=0;config_epoch_held<=0;error_o<=0;
   grid_addr_o<=0;read_requests_o<=0;cycles_o<=0;cells_written_o<=0;
   cell_x<=0;cell_y<=0;row<=0;group<=0;lane<=0;
   request_addr<=0;cell_start_addr<=0;payload<=0;cell_count<=0;square_x<=0;raw_floor<=48;
  end else begin
   if(state!=IDLE&&state!=RESULT&&state!=DRAIN)cycles_o<=cycles_o+1'b1;
   if(state!=IDLE&&state!=DRAIN&&cancel)begin
    error_o<=3;
    if(state==RESPONSE&&!rsp_valid_i)state<=DRAIN;
    else state<=IDLE;
   end else case(state)
    IDLE:if(start_i&&ready_o)begin
     task_id_held<=task_id_i;frame_id_held<=frame_id_i;reference_id_held<=reference_id_i;config_epoch_held<=config_epoch_i;
     error_o<=0;grid_addr_o<=0;read_requests_o<=0;cycles_o<=0;cells_written_o<=0;
     raw_floor<=raw_floor_i;
     cell_x<=0;cell_y<=0;row<=0;group<=0;lane<=0;cell_count<=0;square_x<=0;
     request_addr<=feature_base_i;cell_start_addr<=feature_base_i;
     if(WIDTH<8||WIDTH>1024||WIDTH%8!=0||HEIGHT<8||HEIGHT>600||HEIGHT%8!=0||feature_base_i[3:0]!=0||raw_floor_i<48
       ||({1'b0,feature_base_i}+IMAGE_BYTES)>33'h100000000)begin error_o<=1;state<=FINISH;end
     else state<=REQUEST;
    end
    REQUEST:if(req_ready_i)begin read_requests_o<=read_requests_o+1'b1;state<=RESPONSE;end
    RESPONSE:if(rsp_valid_i)begin
     if(rsp_error_i)begin error_o<=2;state<=FINISH;end
     else begin payload<=rsp_data_i;lane<=0;state<=SQUARE_X;end
    end
    SQUARE_X:begin square_x<=$unsigned(product);state<=SQUARE_Y;end
    SQUARE_Y:begin
     cell_count<=cell_count+strong_pixel;
     if(lane==3)state<=NEXT_BEAT;
     else begin lane<=lane+1'b1;state<=SQUARE_X;end
    end
    NEXT_BEAT:if(!group)begin
     group<=1;request_addr<=request_addr+32'd16;state<=REQUEST;
    end else if(row==7)state<=OUTPUT_CELL;
    else begin
     group<=0;row<=row+1'b1;request_addr<=request_addr+WIDTH*4-16;state<=REQUEST;
    end
    OUTPUT_CELL:if(grid_ready_i)begin
     cells_written_o<=cells_written_o+1'b1;
     if(cell_x+1==GRID_WIDTH&&cell_y+1==GRID_HEIGHT)state<=FINISH;
     else begin
      grid_addr_o<=grid_addr_o+1'b1;row<=0;group<=0;cell_count<=0;
      if(cell_x+1==GRID_WIDTH)begin
       cell_x<=0;cell_y<=cell_y+1'b1;
       cell_start_addr<=cell_start_addr+(WIDTH*7+8)*4;
       request_addr<=cell_start_addr+(WIDTH*7+8)*4;
      end else begin
       cell_x<=cell_x+1'b1;cell_start_addr<=cell_start_addr+32'd32;request_addr<=cell_start_addr+32'd32;
      end
      state<=REQUEST;
     end
    end
    FINISH:state<=RESULT;
    RESULT:if(out_ready_i)state<=IDLE;
    DRAIN:if(rsp_valid_i)state<=IDLE;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
