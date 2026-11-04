`timescale 1ns/1ps
// Autonomous coarse candidates -> local refinement -> native evidence.
// Candidate-valid is now the held evaluated record; coarse metadata remains visible.
module part_auto_native_pipeline #(parameter integer MAX_CANDIDATES=32)(

 input wire sample_external_i,sample_start_i,sample_accept_i,sample_abort_i,
 input wire[194:0]sample_query_i,
 output wire sample_ready_o,sample_busy_o,sample_valid_o,output wire[75:0]sample_reply_o,
 output wire[16:0]invscale_q16_o,
 output wire[9:0]domain_x0_o,domain_y0_o,domain_x1_o,domain_y1_o,
 input wire clk_i,rst_i,abort_i,start_i,
 output wire ready_o,busy_o,done_o,result_valid_o,input wire out_ready_i,
 input wire[31:0] ref_feature_base_i,obs_feature_base_i,
 input wire[31:0] task_id_i,frame_id_i,reference_id_i,config_epoch_i,
 output wire[31:0] task_id_o,frame_id_o,reference_id_o,config_epoch_o,
 output wire[3:0] error_o,stage_error_o,
 output wire support_ambiguous_o,point_budget_exceeded_o,
 output wire[13:0] ref_support_count_o,obs_support_count_o,
 output wire[15:0] ref_cx_q8_o,ref_cy_q8_o,obs_cx_q8_o,obs_cy_q8_o,
 output wire[5:0] candidate_count_o,output wire candidates_truncated_o,
 output wire[14:0] poses_evaluated_o,output wire[31:0] cycles_o,
 output wire candidate_valid_o,input wire candidate_ready_i,
 output wire[6:0] angle_index_o,output wire[3:0] scale_index_o,
 output wire signed[15:0] dx_q8_o,dy_q8_o,a_q14_o,b_q14_o,ia_q14_o,ib_q14_o,
 output wire[31:0] cost_o,
 output wire[15:0] gray_addr_o,output wire gray_we_o,output wire[7:0] gray_data_o,input wire[7:0] gray_data_i,
 output wire[11:0] queue_addr_o,output wire queue_we_o,output wire[15:0] queue_data_o,input wire[15:0] queue_data_i,
 output wire req_valid_o,input wire req_ready_i,output wire req_write_o,
 output wire[31:0] req_addr_o,output wire[127:0] req_data_o,
 input wire rsp_valid_i,output wire rsp_ready_o,input wire rsp_error_i,input wire[127:0] rsp_data_i
,
 output wire[3:0]candidate_error_o,
 output wire[8:0]refined_angle_o,refined_scale_q8_o,
 output wire signed[15:0]refined_dx_q8_o,refined_dy_q8_o,
 output wire signed[17:0]native_c_q16_o,native_d_q16_o,
 output wire signed[28:0]native_offset_x_q16_o,native_offset_y_q16_o,
 output wire[31:0]refined_cost_o,native_cycles_o,native_reads_o,
 output wire[7:0]refinement_evaluations_o,
 output wire[239:0]native_evidence_o,output wire[15:0]native_coverage_grid_o,
 output wire[40:0]native_coverage_bbox_o
);
wire  coarse_ready_o;
wire  coarse_busy_o;
wire  coarse_candidate_valid_o;
wire [15:0] coarse_gray_addr_o;
wire  coarse_gray_we_o;
wire [7:0] coarse_gray_data_o;
wire [11:0] coarse_queue_addr_o;
wire  coarse_queue_we_o;
wire [15:0] coarse_queue_data_o;
wire  coarse_req_valid_o;
wire  coarse_req_write_o;
wire [31:0] coarse_req_addr_o;
wire [127:0] coarse_req_data_o;
wire  coarse_rsp_ready_o;
 wire eval_ready,eval_busy,eval_valid;
 wire c_sc_start,c_sc_accept,e_sc_start,e_sc_accept;
 wire[95:0]c_sc_pose,e_sc_pose;
 wire sc_ready,sc_valid;wire[31:0]sc_cost;wire[3:0]sc_error;
 wire[15:0]sc_gray_addr;wire[11:0]sc_queue_addr;
 wire[95:0]sc_pose=eval_busy?e_sc_pose:c_sc_pose;
 wire[15:0]eval_gray_addr;wire[11:0]eval_queue_addr;
 wire eval_req,eval_write,eval_rsp_ready;wire[31:0]eval_addr;wire[127:0]eval_data;
 reg[31:0]ref_base,obs_base;
 wire cancel=abort_i||(coarse_busy_o&&(task_id_i!=task_id_o||frame_id_i!=frame_id_o||reference_id_i!=reference_id_o||config_epoch_i!=config_epoch_o));
 assign ready_o=coarse_ready_o&&eval_ready&&!abort_i;
 assign busy_o=coarse_busy_o||eval_busy;
 assign candidate_valid_o=eval_valid&&!cancel&&!rst_i;
 always@(posedge clk_i)begin
  if(rst_i)begin ref_base<=0;obs_base<=0;end
  else if(start_i&&ready_o)begin ref_base<=ref_feature_base_i;obs_base<=obs_feature_base_i;end
 end
 part_auto_coarse_pipeline #(.MAX_CANDIDATES(MAX_CANDIDATES)) coarse(
.clk_i(clk_i),
.rst_i(rst_i),
.abort_i(abort_i),
.start_i(start_i&&ready_o),
.ready_o(coarse_ready_o),
.busy_o(coarse_busy_o),
.done_o(done_o),
.result_valid_o(result_valid_o),
.out_ready_i(out_ready_i),
.ref_feature_base_i(ref_feature_base_i),
.obs_feature_base_i(obs_feature_base_i),
.task_id_i(task_id_i),
.frame_id_i(frame_id_i),
.reference_id_i(reference_id_i),
.config_epoch_i(config_epoch_i),
.task_id_o(task_id_o),
.frame_id_o(frame_id_o),
.reference_id_o(reference_id_o),
.config_epoch_o(config_epoch_o),
.error_o(error_o),
.stage_error_o(stage_error_o),
.support_ambiguous_o(support_ambiguous_o),
.point_budget_exceeded_o(point_budget_exceeded_o),
.ref_support_count_o(ref_support_count_o),
.obs_support_count_o(obs_support_count_o),
.ref_cx_q8_o(ref_cx_q8_o),
.ref_cy_q8_o(ref_cy_q8_o),
.obs_cx_q8_o(obs_cx_q8_o),
.obs_cy_q8_o(obs_cy_q8_o),.domain_x0_o(domain_x0_o),.domain_y0_o(domain_y0_o),.domain_x1_o(domain_x1_o),.domain_y1_o(domain_y1_o),
.candidate_count_o(candidate_count_o),
.candidates_truncated_o(candidates_truncated_o),
.poses_evaluated_o(poses_evaluated_o),
.cycles_o(cycles_o),
.candidate_valid_o(coarse_candidate_valid_o),
.candidate_ready_i(eval_valid&&candidate_ready_i&&!cancel),
.angle_index_o(angle_index_o),
.scale_index_o(scale_index_o),
.dx_q8_o(dx_q8_o),
.dy_q8_o(dy_q8_o),
.a_q14_o(a_q14_o),
.b_q14_o(b_q14_o),
.ia_q14_o(ia_q14_o),
.ib_q14_o(ib_q14_o),
.cost_o(cost_o),
.gray_addr_o(coarse_gray_addr_o),
.gray_we_o(coarse_gray_we_o),
.gray_data_o(coarse_gray_data_o),
.gray_data_i(gray_data_i),
.queue_addr_o(coarse_queue_addr_o),
.queue_we_o(coarse_queue_we_o),
.queue_data_o(coarse_queue_data_o),
.queue_data_i(queue_data_i),
.req_valid_o(coarse_req_valid_o),
.req_ready_i(req_ready_i&&!(eval_busy||sample_external_i)),
.req_write_o(coarse_req_write_o),
.req_addr_o(coarse_req_addr_o),
.req_data_o(coarse_req_data_o),
.rsp_valid_i(rsp_valid_i&&!(eval_busy||sample_external_i)),
.rsp_ready_o(coarse_rsp_ready_o),
.rsp_error_i(rsp_error_i),
.rsp_data_i(rsp_data_i),.sc_start_o(c_sc_start),.sc_accept_o(c_sc_accept),.sc_pose_o(c_sc_pose),
 .sc_ready_i(sc_ready),.sc_valid_i(sc_valid),.sc_cost_i(sc_cost),.sc_error_i(sc_error),
 .sc_gray_addr_i(sc_gray_addr),.sc_queue_addr_i(sc_queue_addr));
 part_family_native_eval evaluator(.sample_external_i(sample_external_i),.sample_start_i(sample_start_i),.sample_accept_i(sample_accept_i),.sample_abort_i(sample_abort_i),
 .sample_query_i(sample_query_i),.sample_ready_o(sample_ready_o),.sample_busy_o(sample_busy_o),.sample_valid_o(sample_valid_o),.sample_reply_o(sample_reply_o),.invscale_q16_o(invscale_q16_o),
 .clk_i(clk_i),.rst_i(rst_i),.abort_i(cancel),.start_i(coarse_candidate_valid_o&&eval_ready&&!cancel),.ready_o(eval_ready),.busy_o(eval_busy),
 .angle_i({2'd0,angle_index_o}*9'd5),.scale_q8_i(9'd192+{1'b0,scale_index_o,4'b0}),.dx_q8_i(dx_q8_o),.dy_q8_i(dy_q8_o),
 .ref_count_i(ref_support_count_o[11:0]),.obs_count_i(obs_support_count_o[11:0]),
 .ref_cx_q8_i(ref_cx_q8_o),.ref_cy_q8_i(ref_cy_q8_o),.obs_cx_q8_i(obs_cx_q8_o),.obs_cy_q8_i(obs_cy_q8_o),
 .ref_base_i(ref_base),.obs_base_i(obs_base),.task_id_i(task_id_o),.frame_id_i(frame_id_o),.reference_id_i(reference_id_o),.config_epoch_i(config_epoch_o),
 .gray_addr_o(eval_gray_addr),.gray_data_i(gray_data_i),.queue_addr_o(eval_queue_addr),.queue_data_i(queue_data_i),
 .result_valid_o(eval_valid),.out_ready_i(candidate_ready_i&&!cancel),.error_o(candidate_error_o),
 .angle_o(refined_angle_o),.scale_q8_o(refined_scale_q8_o),.dx_q8_o(refined_dx_q8_o),.dy_q8_o(refined_dy_q8_o),
 .c_q16_o(native_c_q16_o),.d_q16_o(native_d_q16_o),.offset_x_q16_o(native_offset_x_q16_o),.offset_y_q16_o(native_offset_y_q16_o),
 .cost_o(refined_cost_o),.evaluations_o(refinement_evaluations_o),.evidence_o(native_evidence_o),.coverage_grid_o(native_coverage_grid_o),
 .coverage_bbox_o(native_coverage_bbox_o),.native_cycles_o(native_cycles_o),.native_reads_o(native_reads_o),
 .req_valid_o(eval_req),.req_ready_i(req_ready_i&&(eval_busy||sample_external_i)),.req_write_o(eval_write),.req_addr_o(eval_addr),.req_data_o(eval_data),
 .rsp_valid_i(rsp_valid_i&&(eval_busy||sample_external_i)),.rsp_ready_o(eval_rsp_ready),.rsp_error_i(rsp_error_i),.rsp_data_i(rsp_data_i),.sc_start_o(e_sc_start),.sc_accept_o(e_sc_accept),.sc_pose_o(e_sc_pose),
 .sc_ready_i(sc_ready),.sc_valid_i(sc_valid),.sc_cost_i(sc_cost),.sc_error_i(sc_error),
 .sc_gray_addr_i(sc_gray_addr),.sc_queue_addr_i(sc_queue_addr));
 // Coarse DDR reads finish before its first candidate. A held coarse candidate
 // grants scratch/queue/DDR to the evaluator until its result is consumed.
 assign gray_addr_o=eval_busy?eval_gray_addr:coarse_gray_addr_o;
 assign gray_we_o=coarse_gray_we_o&&!(eval_busy||sample_external_i);assign gray_data_o=coarse_gray_data_o;
 assign queue_addr_o=eval_busy?eval_queue_addr:coarse_queue_addr_o;
 assign queue_we_o=coarse_queue_we_o&&!(eval_busy||sample_external_i);assign queue_data_o=coarse_queue_data_o;
 assign req_valid_o=(eval_busy||sample_external_i)?eval_req:coarse_req_valid_o;
 assign req_write_o=(eval_busy||sample_external_i)?eval_write:coarse_req_write_o;
 assign req_addr_o=(eval_busy||sample_external_i)?eval_addr:coarse_req_addr_o;
 assign req_data_o=(eval_busy||sample_external_i)?eval_data:coarse_req_data_o;
 assign rsp_ready_o=(eval_busy||sample_external_i)?eval_rsp_ready:coarse_rsp_ready_o;
 part_chamfer_pipeline scorer(.clk_i(clk_i),.rst_i(rst_i),.abort_i(cancel),
 .start_i(eval_busy?e_sc_start:c_sc_start),.ready_o(sc_ready),.busy_o(),.result_valid_o(sc_valid),
 .out_ready_i(eval_busy?e_sc_accept:c_sc_accept),
 .a_q14_i(sc_pose[95:80]),.b_q14_i(sc_pose[79:64]),.ia_q14_i(sc_pose[63:48]),.ib_q14_i(sc_pose[47:32]),
 .dx_q8_i(sc_pose[31:16]),.dy_q8_i(sc_pose[15:0]),
 .ref_cx_q8_i(ref_cx_q8_o),.ref_cy_q8_i(ref_cy_q8_o),.obs_cx_q8_i(obs_cx_q8_o),.obs_cy_q8_i(obs_cy_q8_o),
 .ref_count_i(ref_support_count_o[11:0]),.obs_count_i(obs_support_count_o[11:0]),
 .queue_addr_o(sc_queue_addr),.queue_data_i(queue_data_i),.gray_addr_o(sc_gray_addr),.gray_data_i(gray_data_i),
 .cost_o(sc_cost),.error_o(sc_error));
endmodule
