`timescale 1ns/1ps
// Native DDR -> aligned features -> soft histogram comparison -> 25 cell counts.
// External gray/state/frontier memories are borrowed from shared_vision_engine.
// Pose service is ready/valid: every pose stays stable until pose_ready_o.
// Whole-frame registration/DDR record scheduling is the parent responsibility.
module part_compare_tile_pipeline #(
 parameter WIDTH=1024,HEIGHT=600,CELL_BASE=34000
)(

 output wire sample_start_o,sample_accept_o,sample_abort_o,output wire[194:0]sample_query_o,
 input wire sample_ready_i,sample_busy_i,sample_valid_i,input wire[75:0]sample_reply_i,
 input wire clk_i,rst_i,abort_i,start_i,output wire ready_o,busy_o,valid_o,input wire out_ready_i,
 input wire[9:0]tile_x_i,tile_y_i,domain_x0_i,domain_y0_i,domain_x1_i,domain_y1_i,
 input wire[31:0]ref_base_i,obs_base_i,task_id_i,frame_id_i,reference_id_i,config_epoch_i,
 input wire pose_valid_i,pose_last_i,output wire pose_ready_o,pose_needed_o,
 input wire signed[17:0]c_q16_i,d_q16_i,input wire signed[28:0]offset_x_q16_i,offset_y_q16_i,
 input wire[16:0]invscale_q16_i,
 output reg[3:0]error_o,output reg[31:0]cycles_o,
 output wire cell_valid_o,input wire cell_ready_i,output wire[4:0]cell_index_o,output reg[7:0]cell_data_o,
 output wire[15:0]gray_addr_o,state_addr_o,output wire gray_we_o,output wire[7:0]gray_data_o,input wire[7:0]gray_data_i,
 output wire[1:0]state_we_o,state_data_o,input wire[1:0]state_data_i,
 output wire[11:0]queue_addr_o,output wire queue_we_o,output wire[15:0]queue_data_o,input wire[15:0]queue_data_i,
 output wire req_valid_o,input wire req_ready_i,output wire req_write_o,output wire[31:0]req_addr_o,output wire[127:0]req_data_o,
 input wire rsp_valid_i,output wire rsp_ready_o,input wire rsp_error_i,input wire[127:0]rsp_data_i
);
 localparam IDLE=0,CLEAR=1,INIT_START=2,INIT_WAIT=3,OLOAD_START=4,OLOAD_WAIT=5,OHIST_START=6,OHIST_WAIT=7,
 COPY_READ=8,COPY_TAKE=9,COPY_WRITE=10,POSE_WAIT=11,RLOAD_START=12,RLOAD_WAIT=13,
 RHIST_START=14,RHIST_WAIT=15,MERGE_START=16,MERGE_WAIT=17,WHIST_START=18,WHIST_WAIT=19,
 WEAK_START=20,WEAK_WAIT=21,POSE_ACCEPT=22,FINAL_START=23,FINAL_WAIT=24,CELL_READ=25,CELL_TAKE=26,CELL_EMIT=27,
 RESULT=28,DRAIN=29;
 reg[4:0]state;reg[4:0]cell_index,x,y;reg[3:0]lane;reg[7:0]copy_data;
 reg[1:0]collector;reg[7:0]cell_previous;reg failed;
 wire loader_owner=state==OLOAD_START||state==OLOAD_WAIT||state==RLOAD_START||state==RLOAD_WAIT||state==DRAIN;
 wire obs_phase=state==OLOAD_START||state==OLOAD_WAIT;
 wire hist_owner=state==OHIST_START||state==OHIST_WAIT||state==RHIST_START||state==RHIST_WAIT||state==WHIST_START||state==WHIST_WAIT;
 wire reducer_owner=state==INIT_START||state==INIT_WAIT||state==MERGE_START||state==MERGE_WAIT||state==WEAK_START||state==WEAK_WAIT||state==FINAL_START||state==FINAL_WAIT;
 wire stop=abort_i||state==DRAIN;
 wire lready,lbusy,lvalid,lstrong;wire[3:0]lerror;wire[15:0]lga,lsa;wire lgwe;wire[7:0]lgd;wire[1:0]lswe,lsd;
 wire ss,sa,sready,sbusy,sv,sdomain,sinterior;wire[3:0]serror;wire[9:0]sx,sy;
 wire signed[10:0]ogx,ogy;wire[7:0]oraw;wire[1:0]oflags,rflags;wire signed[12:0]rgx,rgy;wire[9:0]rraw;
 wire ds,da,dready,dv;wire signed[12:0]dgx,dgy;wire[9:0]draw;wire[1:0]dflags;wire[8:0]angle;wire[6:0]weight;wire strong_edge,weak_edge;
 wire hready,hvalid;wire[15:0]hga;wire hgwe;wire[7:0]hgd;
 wire rready,rvalid;wire[8:0]unexplained;wire[15:0]rga,rsa;wire rgwe;wire[7:0]rgd;
 wire pixel_valid,pixel_candidate,pixel_clipped;wire[4:0]px,py;
 wire[4:0]pixel_cell=(py>>2)*5+(px>>2);
 wire collecting=state==FINAL_WAIT&&pixel_valid&&pixel_candidate;
 wire pixel_accept=!pixel_candidate||collector==2;
 wire[15:0]private_addr=state==COPY_READ||state==COPY_TAKE?16'd21548+((y+5'd3)*26+x+5'd3)*16+lane:
   state==COPY_WRITE?16'd15148+(y*20+x)*16+lane:collecting?CELL_BASE+pixel_cell:CELL_BASE+cell_index;
 wire[7:0]count_next={cell_previous[7]|pixel_clipped,2'b00,cell_previous[4:0]+5'd1};
 assign ready_o=state==IDLE&&sready&&!rst_i&&!abort_i;
 assign busy_o=state!=IDLE&&state!=RESULT;
 assign valid_o=state==RESULT&&!rst_i&&!abort_i;
 assign pose_ready_o=state==POSE_ACCEPT&&!rst_i&&!abort_i;
 assign pose_needed_o=state==POSE_WAIT&&!rst_i&&!abort_i;
 assign cell_valid_o=state==CELL_EMIT&&!rst_i&&!abort_i;
 assign cell_index_o=cell_index;
 assign gray_addr_o=collecting?private_addr:loader_owner?lga:hist_owner?hga:reducer_owner?rga:private_addr;
 assign gray_we_o=!rst_i&&!abort_i&&(collecting?collector==2:loader_owner?lgwe:hist_owner?hgwe:reducer_owner?rgwe:(state==CLEAR||state==COPY_WRITE));
 assign gray_data_o=collecting?count_next:loader_owner?lgd:hist_owner?hgd:reducer_owner?rgd:state==CLEAR?8'd0:copy_data;
 assign state_addr_o=loader_owner?lsa:rsa;
 assign state_we_o=loader_owner?lswe:2'd0;assign state_data_o=lsd;
 part_compare_tile_load #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) loader(
 .clk_i(clk_i),.rst_i(rst_i),.abort_i(stop),.start_i(state==OLOAD_START||state==RLOAD_START),.reference_i(!obs_phase),
 .tile_x_i(tile_x_i),.tile_y_i(tile_y_i),.ready_o(lready),.busy_o(lbusy),.valid_o(lvalid),.out_ready_i(1'b1),.error_o(lerror),.center_strong_o(lstrong),
 .sample_start_o(ss),.sample_accept_o(sa),.sample_ready_i(sready),.sample_valid_i(sv),.sample_error_i(serror),.sample_domain_i(sdomain),.sample_interior_i(sinterior),
 .sample_x_o(sx),.sample_y_o(sy),.obs_gx_i(ogx),.obs_gy_i(ogy),.obs_raw_i(oraw),.obs_flags_i(oflags),.ref_gx_i(rgx),.ref_gy_i(rgy),.ref_raw_i(rraw),.ref_flags_i(rflags),
 .direction_start_o(ds),.direction_accept_o(da),.direction_ready_i(dready),.direction_valid_i(dv),.direction_gx_o(dgx),.direction_gy_o(dgy),.direction_raw_o(draw),.direction_flags_o(dflags),
 .direction_angle_i(angle),.direction_weight_i(weight),.direction_strong_i(strong_edge),.direction_weak_i(weak_edge),
 .gray_addr_o(lga),.gray_we_o(lgwe),.gray_data_o(lgd),.state_addr_o(lsa),.state_we_o(lswe),.state_data_o(lsd),.state_data_i(state_data_i));
 // Borrow the registration sampler. The parent retains this lease through drain.
 assign sample_start_o=ss;assign sample_accept_o=sa;assign sample_abort_o=stop;
 assign sample_query_o={obs_phase?obs_base_i:ref_base_i,obs_base_i,
   obs_phase?18'd65536:c_q16_i,obs_phase?18'd0:d_q16_i,
   obs_phase?29'd0:offset_x_q16_i,obs_phase?29'd0:offset_y_q16_i,
   obs_phase?17'd65536:invscale_q16_i,sx,sy};
 assign sready=sample_ready_i;assign sbusy=sample_busy_i;assign sv=sample_valid_i;
 assign {serror,sdomain,sinterior,ogx,ogy,oraw,oflags,rgx,rgy,rraw,rflags}=sample_reply_i;
 assign req_valid_o=0;assign req_write_o=0;assign req_addr_o=0;assign req_data_o=0;assign rsp_ready_o=0;
 part_direction_feature direction(.clk_i(clk_i),.rst_i(rst_i),.abort_i(stop),.start_i(ds),.ready_o(dready),.gx_i(dgx),.gy_i(dgy),.raw_i(draw),.flags_i(dflags),
 .valid_o(dv),.out_ready_i(da),.angle_o(angle),.weight_o(weight),.strong_o(strong_edge),.weak_o(weak_edge));
 part_histogram_tile histogram(.clk_i(clk_i),.rst_i(rst_i),.abort_i(stop),.start_i(state==OHIST_START||state==RHIST_START||state==WHIST_START),.weak_i(state==WHIST_START),
 .ready_o(hready),.valid_o(hvalid),.out_ready_i(1'b1),.gray_addr_o(hga),.gray_we_o(hgwe),.gray_data_o(hgd),.gray_data_i(gray_data_i));
 part_compare_tile_reduce reducer(.clk_i(clk_i),.rst_i(rst_i),.abort_i(stop),
 .start_i(state==INIT_START||state==MERGE_START||state==WEAK_START||state==FINAL_START),
 .operation_i(state==INIT_START?2'd0:state==MERGE_START?2'd1:state==WEAK_START?2'd2:2'd3),
 .tile_x_i(tile_x_i),.tile_y_i(tile_y_i),.domain_x0_i(domain_x0_i),.domain_y0_i(domain_y0_i),.domain_x1_i(domain_x1_i),.domain_y1_i(domain_y1_i),
 .ready_o(rready),.valid_o(rvalid),.out_ready_i(1'b1),.unexplained_o(unexplained),.gray_addr_o(rga),.gray_we_o(rgwe),.gray_data_o(rgd),.gray_data_i(gray_data_i),
 .queue_addr_o(queue_addr_o),.queue_we_o(queue_we_o),.queue_data_o(queue_data_o),.queue_data_i(queue_data_i),.state_addr_o(rsa),.state_data_i(state_data_i),
 .pixel_valid_o(pixel_valid),.pixel_ready_i(pixel_accept),.pixel_x_o(px),.pixel_y_o(py),.pixel_candidate_o(pixel_candidate),.pixel_clipped_o(pixel_clipped));
 always@(posedge clk_i)begin
  if(rst_i)begin state<=IDLE;cell_index<=0;x<=0;y<=0;lane<=0;copy_data<=0;collector<=0;cell_previous<=0;failed<=0;error_o<=0;cycles_o<=0;cell_data_o<=0;end
  else begin
   if(busy_o)cycles_o<=cycles_o+1'b1;
   if(!collecting)collector<=0;
   else if(collector==0)collector<=1;
   else if(collector==1)begin cell_previous<=gray_data_i;collector<=2;end
   else collector<=0;
   if(abort_i)begin state<=sbusy?DRAIN:IDLE;failed<=0;collector<=0;end
   else case(state)
    IDLE:if(start_i&&ready_o)begin state<=CLEAR;cell_index<=0;error_o<=0;failed<=0;cycles_o<=0;end
    CLEAR:if(cell_index==24)state<=INIT_START;else cell_index<=cell_index+1'b1;
    INIT_START:if(rready)state<=INIT_WAIT;
    INIT_WAIT:if(rvalid)state<=OLOAD_START;
    OLOAD_START:if(lready)state<=OLOAD_WAIT;
    OLOAD_WAIT:if(lvalid)begin
     if(lerror!=0)begin error_o<=lerror;failed<=1;state<=DRAIN;end else state<=lstrong?OHIST_START:FINAL_START;
    end
    OHIST_START:if(hready)state<=OHIST_WAIT;
    OHIST_WAIT:if(hvalid)begin x<=0;y<=0;lane<=0;state<=COPY_READ;end
    COPY_READ:state<=COPY_TAKE;
    COPY_TAKE:begin copy_data<=gray_data_i;state<=COPY_WRITE;end
    COPY_WRITE:begin
     if(lane!=15)begin lane<=lane+1'b1;state<=COPY_READ;end
     else if(x!=19)begin lane<=0;x<=x+1'b1;state<=COPY_READ;end
     else if(y!=19)begin lane<=0;x<=0;y<=y+1'b1;state<=COPY_READ;end else state<=POSE_WAIT;
    end
    POSE_WAIT:if(pose_valid_i)state<=RLOAD_START;
    RLOAD_START:if(lready)state<=RLOAD_WAIT;
    RLOAD_WAIT:if(lvalid)begin
     if(lerror!=0)begin error_o<=lerror;failed<=1;state<=DRAIN;end else state<=RHIST_START;
    end
    RHIST_START:if(hready)state<=RHIST_WAIT;
    RHIST_WAIT:if(hvalid)state<=MERGE_START;
    MERGE_START:if(rready)state<=MERGE_WAIT;
    MERGE_WAIT:if(rvalid)state<=unexplained==0?POSE_ACCEPT:WHIST_START;
    WHIST_START:if(hready)state<=WHIST_WAIT;
    WHIST_WAIT:if(hvalid)state<=WEAK_START;
    WEAK_START:if(rready)state<=WEAK_WAIT;
    WEAK_WAIT:if(rvalid)state<=POSE_ACCEPT;
    POSE_ACCEPT:state<=pose_last_i||unexplained==0?FINAL_START:POSE_WAIT;
    FINAL_START:if(rready)state<=FINAL_WAIT;
    FINAL_WAIT:if(rvalid)begin cell_index<=0;state<=CELL_READ;end
    CELL_READ:state<=CELL_TAKE;
    CELL_TAKE:begin cell_data_o<=gray_data_i;state<=CELL_EMIT;end
    CELL_EMIT:if(cell_ready_i)begin if(cell_index==24)state<=RESULT;else begin cell_index<=cell_index+1'b1;state<=CELL_READ;end end
    RESULT:if(out_ready_i)state<=IDLE;
    DRAIN:if(!sbusy)state<=failed?RESULT:IDLE;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
