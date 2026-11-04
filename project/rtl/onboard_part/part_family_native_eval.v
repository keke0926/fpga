`timescale 1ns/1ps
// Local geometric refinement -> native inverse pose -> actual native evidence.
// Stable parent-owned identity/geometry/memory context; no verdict or private RAM.
module part_family_native_eval(

 input wire sample_external_i,sample_start_i,sample_accept_i,sample_abort_i,
 input wire[194:0]sample_query_i,
 output wire sample_ready_o,sample_busy_o,sample_valid_o,output wire[75:0]sample_reply_o,
 output wire[16:0]invscale_q16_o,
 input wire clk_i,rst_i,abort_i,start_i,output wire ready_o,busy_o,
 input wire[8:0]angle_i,scale_q8_i,input wire signed[15:0]dx_q8_i,dy_q8_i,
 input wire[11:0]ref_count_i,obs_count_i,
 input wire[15:0]ref_cx_q8_i,ref_cy_q8_i,obs_cx_q8_i,obs_cy_q8_i,
 input wire[31:0]ref_base_i,obs_base_i,task_id_i,frame_id_i,reference_id_i,config_epoch_i,
 output wire[15:0]gray_addr_o,input wire[7:0]gray_data_i,
 output wire[11:0]queue_addr_o,input wire[15:0]queue_data_i,
 output wire result_valid_o,input wire out_ready_i,output reg[3:0]error_o,
 output wire[8:0]angle_o,scale_q8_o,output wire signed[15:0]dx_q8_o,dy_q8_o,
 output wire signed[17:0]c_q16_o,d_q16_o,
 output wire signed[28:0]offset_x_q16_o,offset_y_q16_o,
 output wire[31:0]cost_o,output wire[7:0]evaluations_o,
 output wire[239:0]evidence_o,output wire[15:0]coverage_grid_o,
 output wire[40:0]coverage_bbox_o,output wire[31:0]native_cycles_o,native_reads_o,
 output wire req_valid_o,input wire req_ready_i,output wire req_write_o,
 output wire[31:0]req_addr_o,output wire[127:0]req_data_o,
 input wire rsp_valid_i,output wire rsp_ready_o,input wire rsp_error_i,input wire[127:0]rsp_data_i,
 output wire sc_start_o,sc_accept_o,output wire[95:0]sc_pose_o,
 input wire sc_ready_i,sc_valid_i,input wire[31:0]sc_cost_i,input wire[3:0]sc_error_i,
 input wire[15:0]sc_gray_addr_i,input wire[11:0]sc_queue_addr_i

);
 localparam IDLE=0,FSTART=1,FWAIT=2,BSTART=3,BWAIT=4,NSTART=5,NWAIT=6,RESULT=7,DRAIN=8;
 reg[3:0]state;wire fready,fvalid,bready,bvalid,nready,nvalid,nbusy;
 wire[3:0]ferror,nerror;wire signed[17:0]fc,fd;
 wire stop=abort_i||state==DRAIN;
 assign ready_o=state==IDLE&&!rst_i&&!abort_i;assign busy_o=state!=IDLE;
 assign result_valid_o=state==RESULT&&!rst_i&&!abort_i;
 part_family_refine refine(.clk_i(clk_i),.rst_i(rst_i),.abort_i(stop),.start_i(state==FSTART),.ready_o(fready),.busy_o(),
 .angle_i(angle_i),.scale_q8_i(scale_q8_i),.dx_q8_i(dx_q8_i),.dy_q8_i(dy_q8_i),
 .ref_count_i(ref_count_i),.obs_count_i(obs_count_i),.ref_cx_q8_i(ref_cx_q8_i),.ref_cy_q8_i(ref_cy_q8_i),.obs_cx_q8_i(obs_cx_q8_i),.obs_cy_q8_i(obs_cy_q8_i),
 .gray_addr_o(gray_addr_o),.gray_data_i(gray_data_i),.queue_addr_o(queue_addr_o),.queue_data_i(queue_data_i),
 .result_valid_o(fvalid),.out_ready_i(state==FWAIT),.error_o(ferror),.angle_o(angle_o),.scale_q8_o(scale_q8_o),.dx_q8_o(dx_q8_o),.dy_q8_o(dy_q8_o),
 .c_q16_o(fc),.d_q16_o(fd),.cost_o(cost_o),.evaluations_o(evaluations_o),.sc_start_o(sc_start_o),.sc_accept_o(sc_accept_o),.sc_pose_o(sc_pose_o),
 .sc_ready_i(sc_ready_i),.sc_valid_i(sc_valid_i),.sc_cost_i(sc_cost_i),.sc_error_i(sc_error_i),
 .sc_gray_addr_i(sc_gray_addr_i),.sc_queue_addr_i(sc_queue_addr_i));
 part_native_pose_bridge bridge(.clk_i(clk_i),.rst_i(rst_i),.abort_i(stop),.start_i(state==BSTART),.ready_o(bready),
 .c_q16_i(fc),.d_q16_i(fd),.ref_cx_q8_i(ref_cx_q8_i),.ref_cy_q8_i(ref_cy_q8_i),.obs_cx_q8_i(obs_cx_q8_i),.obs_cy_q8_i(obs_cy_q8_i),
 .dx_q8_i(dx_q8_o),.dy_q8_i(dy_q8_o),.valid_o(bvalid),.out_ready_i(state==BWAIT),
 .c_q16_o(c_q16_o),.d_q16_o(d_q16_o),.offset_x_q16_o(offset_x_q16_o),.offset_y_q16_o(offset_y_q16_o));
 part_pose_scan_score #(.STATIC_CONTEXT(1),.MIN_STRIDE_LOG2(4)) native_score(.sample_external_i(sample_external_i),.sample_start_i(sample_start_i),.sample_accept_i(sample_accept_i),.sample_abort_i(sample_abort_i),
 .sample_query_i(sample_query_i),.sample_ready_o(sample_ready_o),.sample_busy_o(sample_busy_o),.sample_valid_o(sample_valid_o),.sample_reply_o(sample_reply_o),.invscale_q16_o(invscale_q16_o),.clk_i(clk_i),.rst_i(rst_i),.abort_i(stop),.start_i(state==NSTART),.ready_o(nready),.busy_o(nbusy),
 .c_q16_i(c_q16_o),.d_q16_i(d_q16_o),.offset_x_q16_i(offset_x_q16_o),.offset_y_q16_i(offset_y_q16_o),
 .ref_base_i(ref_base_i),.obs_base_i(obs_base_i),.stride_log2_i(3'd4),
 .task_id_i(task_id_i),.frame_id_i(frame_id_i),.reference_id_i(reference_id_i),.config_epoch_i(config_epoch_i),
 .done_o(),.result_valid_o(nvalid),.out_ready_i(state==RESULT&&out_ready_i),.error_o(nerror),
 .task_id_o(),.frame_id_o(),.reference_id_o(),.config_epoch_o(),
 .attempted_count_o(evidence_o[0+:20]),.valid_count_o(evidence_o[20+:20]),
 .ref_strong_count_o(evidence_o[40+:20]),.ref_weak_count_o(evidence_o[60+:20]),
 .obs_strong_count_o(evidence_o[80+:20]),.obs_weak_count_o(evidence_o[100+:20]),
 .joint_strong_count_o(evidence_o[120+:20]),.direction_count_o(evidence_o[140+:20]),
 .ref_clipped_count_o(evidence_o[160+:20]),.ref_low_contrast_count_o(evidence_o[180+:20]),
 .obs_clipped_count_o(evidence_o[200+:20]),.obs_low_contrast_count_o(evidence_o[220+:20]),
 .bbox_valid_o(coverage_bbox_o[40]),.min_x_o(coverage_bbox_o[0+:10]),.min_y_o(coverage_bbox_o[10+:10]),
 .max_x_o(coverage_bbox_o[20+:10]),.max_y_o(coverage_bbox_o[30+:10]),.grid_o(coverage_grid_o),
 .cycles_o(native_cycles_o),.read_requests_o(native_reads_o),
 .req_valid_o(req_valid_o),.req_ready_i(req_ready_i),.req_write_o(req_write_o),.req_addr_o(req_addr_o),.req_data_o(req_data_o),
 .rsp_valid_i(rsp_valid_i),.rsp_ready_o(rsp_ready_o),.rsp_error_i(rsp_error_i),.rsp_data_i(rsp_data_i));
 always@(posedge clk_i)begin
  if(rst_i)begin state<=IDLE;error_o<=0;end
  // An idle evaluator owns no DDR request. In particular a coarse-phase
  // cancellation must not acquire the shared response channel just to drain it.
  else if(abort_i&&state!=DRAIN)begin if(state!=IDLE)state<=DRAIN;error_o<=3;end
  else case(state)
   IDLE:if(start_i)begin state<=FSTART;error_o<=0;end
   FSTART:if(fready)state<=FWAIT;
   FWAIT:if(fvalid)begin if(ferror!=0)begin error_o<=ferror;state<=RESULT;end else state<=BSTART;end
   BSTART:if(bready)state<=BWAIT;
   BWAIT:if(bvalid)state<=NSTART;
   NSTART:if(nready)state<=NWAIT;
   NWAIT:if(nvalid)begin error_o<=nerror;state<=RESULT;end
   RESULT:if(out_ready_i)state<=IDLE;
   DRAIN:if(!nbusy)state<=IDLE;
   default:state<=IDLE;
  endcase
 end
endmodule
