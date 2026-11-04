`timescale 1ns/1ps
// Bounded raster tile scheduler, DDR count map and final scratch publication.
// Pose table: 32 bytes/pose, native-word order, words c,d,ox,oy,invscale,0,0,0.
// This input list contains already retained hypotheses; admission/record export
// belongs to the registration owner. The frozen context stays stable until drain.
module part_difference_frame #(
 parameter COUNT_BASE=32'h00f60000,POSE_BASE=32'h00f70000
)(

 output wire sample_start_o,sample_accept_o,sample_abort_o,output wire[194:0]sample_query_o,
 input wire sample_ready_i,sample_busy_i,sample_valid_i,input wire[75:0]sample_reply_i,
 input wire clk_i,rst_i,abort_i,start_i,output wire ready_o,busy_o,valid_o,input wire out_ready_i,
 input wire[5:0]pose_count_i,
 input wire[9:0]domain_x0_i,domain_y0_i,domain_x1_i,domain_y1_i,
 input wire[31:0]ref_base_i,obs_base_i,task_id_i,frame_id_i,reference_id_i,config_epoch_i,
 output reg[3:0]error_o,output reg[31:0]cycles_o,output reg[11:0]tiles_o,
 output reg[19:0]candidate_points_o,clean_points_o,
 output wire[15:0]gray_addr_o,state_addr_o,output wire gray_we_o,output wire[7:0]gray_data_o,input wire[7:0]gray_data_i,
 output wire[1:0]state_we_o,state_data_o,input wire[1:0]state_data_i,
 output wire[11:0]queue_addr_o,output wire queue_we_o,output wire[15:0]queue_data_o,input wire[15:0]queue_data_i,
 output wire req_valid_o,input wire req_ready_i,output wire req_write_o,output wire[31:0]req_addr_o,output wire[127:0]req_data_o,
 input wire rsp_valid_i,output wire rsp_ready_o,input wire rsp_error_i,input wire[127:0]rsp_data_i
);
 localparam IDLE=0,ZERO=1,ZERO_WAIT=2,P0=3,P0_WAIT=4,P1=5,P1_WAIT=6,TILE_START=7,RUN=8,
 COUNT_READ=9,COUNT_READ_WAIT=10,COUNT_EDIT=11,COUNT_WRITE=12,COUNT_WRITE_WAIT=13,CELL_ACK=14,NEXT_TILE=15,
 COPY=16,COPY_WAIT=17,COPY_BYTES=18,RESULT=19,DRAIN_OWN=20,DRAIN_TILE=21;
 reg[4:0]state;reg[15:0]map_offset;reg[3:0]lane;
 reg[9:0]tx,ty;reg[5:0]pose_index;reg cached,initial_pose,failed;
 reg[2:0]cell_x,cell_y;reg[7:0]cell_byte;reg[15:0]cell_offset;
 reg[127:0]word_data;reg pending,abort_core;
 reg signed[17:0]c,d;reg signed[28:0]ox,oy;reg[16:0]invscale;
 wire tr,tb,tv,tp,cv;wire[4:0]ci;wire[7:0]cd;wire[3:0]te;
 wire[15:0]tga;wire tgwe;wire[7:0]tgd;
 wire treq,trwrite,trsp;wire[31:0]taddr;wire[127:0]tdata;
 wire tile_owner=state==RUN||state==DRAIN_TILE;
 wire own_request=state==ZERO||state==P0||state==P1||state==COUNT_READ||state==COUNT_WRITE||state==COPY;
 wire[10:0]cx={1'b0,tx[9:2]}+cell_x,cy={1'b0,ty[9:2]}+cell_y;
 wire[15:0]next_cell_offset={cy[7:0],8'd0}+cx[7:0];
 wire pose_needed; // A speculative next-pose read is unnecessary after early explanation.
 assign ready_o=state==IDLE&&!rst_i&&!abort_i;
 assign busy_o=state!=IDLE&&state!=RESULT;
 assign valid_o=state==RESULT&&!rst_i&&!abort_i;
 assign gray_addr_o=state==COPY_BYTES?map_offset+lane:tga;
 assign gray_we_o=!rst_i&&!abort_i&&(state==COPY_BYTES||tgwe);
 assign gray_data_o=state==COPY_BYTES?word_data[127:120]:tgd;
 assign req_valid_o=!rst_i&&!abort_i&&(tile_owner?treq:(own_request&&!pending));
 assign req_write_o=tile_owner?trwrite:(state==ZERO||state==COUNT_WRITE);
 assign req_addr_o=tile_owner?taddr:(state==P0||state==P1)?POSE_BASE+{21'd0,pose_index,5'd0}+(state==P1?32'd16:32'd0):
   (state==COUNT_READ||state==COUNT_WRITE)?COUNT_BASE+{16'd0,cell_offset[15:4],4'd0}:COUNT_BASE+map_offset;
 assign req_data_o=tile_owner?tdata:state==ZERO?128'd0:word_data;
 assign rsp_ready_o=tile_owner?trsp:pending;
 part_compare_tile_pipeline core(.sample_start_o(sample_start_o),.sample_accept_o(sample_accept_o),.sample_abort_o(sample_abort_o),.sample_query_o(sample_query_o),
 .sample_ready_i(sample_ready_i),.sample_busy_i(sample_busy_i),.sample_valid_i(sample_valid_i),.sample_reply_i(sample_reply_i),.clk_i(clk_i),.rst_i(rst_i),.abort_i(abort_i||abort_core),.start_i(state==TILE_START),.ready_o(tr),.busy_o(tb),.valid_o(tv),.out_ready_i(state==RUN),
 .tile_x_i(tx),.tile_y_i(ty),.domain_x0_i(domain_x0_i),.domain_y0_i(domain_y0_i),.domain_x1_i(domain_x1_i),.domain_y1_i(domain_y1_i),
 .ref_base_i(ref_base_i),.obs_base_i(obs_base_i),.task_id_i(task_id_i),.frame_id_i(frame_id_i),.reference_id_i(reference_id_i),.config_epoch_i(config_epoch_i),
 .pose_valid_i(cached),.pose_last_i(pose_index==pose_count_i-1'b1),.pose_ready_o(tp),.pose_needed_o(pose_needed),.c_q16_i(c),.d_q16_i(d),.offset_x_q16_i(ox),.offset_y_q16_i(oy),.invscale_q16_i(invscale),
 .error_o(te),.cell_valid_o(cv),.cell_ready_i(state==CELL_ACK),.cell_index_o(ci),.cell_data_o(cd),
 .gray_addr_o(tga),.gray_we_o(tgwe),.gray_data_o(tgd),.gray_data_i(gray_data_i),.state_addr_o(state_addr_o),.state_we_o(state_we_o),.state_data_o(state_data_o),.state_data_i(state_data_i),
 .queue_addr_o(queue_addr_o),.queue_we_o(queue_we_o),.queue_data_o(queue_data_o),.queue_data_i(queue_data_i),
 .req_valid_o(treq),.req_ready_i(tile_owner&&req_ready_i),.req_write_o(trwrite),.req_addr_o(taddr),.req_data_o(tdata),
 .rsp_valid_i(tile_owner&&rsp_valid_i),.rsp_ready_o(trsp),.rsp_error_i(rsp_error_i),.rsp_data_i(rsp_data_i));
 integer byte_lane;
 always@(posedge clk_i)begin
  if(rst_i)begin
   state<=IDLE;map_offset<=0;lane<=0;tx<=0;ty<=0;pose_index<=0;cached<=0;initial_pose<=0;failed<=0;
   cell_x<=0;cell_y<=0;cell_byte<=0;cell_offset<=0;word_data<=0;pending<=0;abort_core<=0;c<=0;d<=0;ox<=0;oy<=0;invscale<=0;
   error_o<=0;cycles_o<=0;tiles_o<=0;candidate_points_o<=0;clean_points_o<=0;
  end else begin
   abort_core<=0;
   if(busy_o)cycles_o<=cycles_o+1'b1;
   if(!tile_owner&&req_valid_o&&req_ready_i)pending<=1;
   if(!tile_owner&&rsp_valid_i&&rsp_ready_o)pending<=0;
   if(tp)begin cached<=0;if(pose_index<pose_count_i-1'b1)pose_index<=pose_index+1'b1;end
   if(abort_i)begin
    failed<=0;abort_core<=1;
    if(tile_owner)state<=tb?DRAIN_TILE:IDLE;
    else state<=pending&&!(rsp_valid_i&&rsp_ready_o)?DRAIN_OWN:(tb?DRAIN_TILE:IDLE);
   end else if(!tile_owner&&rsp_valid_i&&rsp_ready_o&&rsp_error_i)begin error_o<=2;failed<=1;abort_core<=1;state<=DRAIN_TILE;end
   else case(state)
    IDLE:if(start_i)begin
     error_o<=0;failed<=0;map_offset<=0;cycles_o<=0;tiles_o<=0;candidate_points_o<=0;clean_points_o<=0;
     tx<={domain_x0_i[9:2],2'd0};ty<={domain_y0_i[9:2],2'd0};state<=ZERO;
     if(pose_count_i==0||pose_count_i>32||domain_x0_i>=domain_x1_i||domain_y0_i>=domain_y1_i||domain_y1_i>600)begin error_o<=1;state<=RESULT;end
    end
    ZERO:if(req_valid_o&&req_ready_i)state<=ZERO_WAIT;
    ZERO_WAIT:if(rsp_valid_i&&pending)begin
     if(map_offset==38384)begin pose_index<=0;cached<=0;initial_pose<=1;state<=P0;end
     else begin map_offset<=map_offset+16;state<=ZERO;end
    end
    P0:if(req_valid_o&&req_ready_i)state<=P0_WAIT;
    P0_WAIT:if(rsp_valid_i&&pending)begin c<=rsp_data_i[113:96];d<=rsp_data_i[81:64];ox<=rsp_data_i[60:32];oy<=rsp_data_i[28:0];state<=P1;end
    P1:if(req_valid_o&&req_ready_i)state<=P1_WAIT;
    P1_WAIT:if(rsp_valid_i&&pending)begin invscale<=rsp_data_i[112:96];cached<=1;state<=initial_pose?TILE_START:RUN;end
    TILE_START:if(tr)begin cell_x<=0;cell_y<=0;state<=RUN;end
    RUN:begin
     if(tv)begin
      if(te!=0)begin error_o<=te;state<=RESULT;end else begin tiles_o<=tiles_o+1'b1;state<=NEXT_TILE;end
     end else if(cv)begin
      cell_byte<=cd;cell_offset<=next_cell_offset;
      state<=cx>=256||cy>=150||cd==0?CELL_ACK:COUNT_READ;
     end else if(!cached&&pose_needed)begin initial_pose<=0;state<=P0;end
    end
    COUNT_READ:if(req_valid_o&&req_ready_i)state<=COUNT_READ_WAIT;
    COUNT_READ_WAIT:if(rsp_valid_i&&pending)begin word_data<=rsp_data_i;state<=COUNT_EDIT;end
    COUNT_EDIT:begin
     // Static byte enables avoid a variable-width 128-bit insertion shifter.
     for(byte_lane=0;byte_lane<16;byte_lane=byte_lane+1)
      if(cell_offset[3:0]==byte_lane)word_data[127-byte_lane*8-:8]<=cell_byte;
     state<=COUNT_WRITE;
    end
    COUNT_WRITE:if(req_valid_o&&req_ready_i)state<=COUNT_WRITE_WAIT;
    COUNT_WRITE_WAIT:if(rsp_valid_i&&pending)state<=CELL_ACK;
    CELL_ACK:begin
     if(cell_x==4)begin cell_x<=0;cell_y<=cell_y+1'b1;end else cell_x<=cell_x+1'b1;
     state<=RUN;
    end
    NEXT_TILE:begin
     pose_index<=0;cached<=0;initial_pose<=1;
     if({1'b0,tx}+11'd20<domain_x1_i)begin tx<=tx+20;state<=P0;end
     else if({1'b0,ty}+11'd20<domain_y1_i)begin tx<={domain_x0_i[9:2],2'd0};ty<=ty+20;state<=P0;end
     else begin map_offset<=0;state<=COPY;end
    end
    COPY:if(req_valid_o&&req_ready_i)state<=COPY_WAIT;
    COPY_WAIT:if(rsp_valid_i&&pending)begin word_data<=rsp_data_i;lane<=0;state<=COPY_BYTES;end
    COPY_BYTES:begin
     candidate_points_o<=candidate_points_o+{15'd0,word_data[124:120]};
     if(!word_data[127])clean_points_o<=clean_points_o+{15'd0,word_data[124:120]};
     word_data<={word_data[119:0],8'd0};
     if(lane!=15)lane<=lane+1'b1;
     else if(map_offset==38384)state<=RESULT;
     else begin map_offset<=map_offset+16;state<=COPY;end
    end
    RESULT:if(out_ready_i)state<=IDLE;
    DRAIN_OWN:if(!pending||rsp_valid_i)state<=tb?DRAIN_TILE:(failed?RESULT:IDLE);
    DRAIN_TILE:if(!tb)state<=failed?RESULT:IDLE;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
