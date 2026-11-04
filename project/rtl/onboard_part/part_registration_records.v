`timescale 1ns/1ps
// Complete native registration candidates in shared scratch [1024,5120).
// 32 x 128-byte records, little-endian words; count becomes usable only with
// result_valid and error==0. Records are hypotheses, never defect boxes.
module part_registration_records(

 input wire sample_external_i,sample_start_i,sample_accept_i,sample_abort_i,
 input wire[194:0]sample_query_i,
 output wire sample_ready_o,sample_busy_o,sample_valid_o,output wire[75:0]sample_reply_o,
 output wire[16:0]invscale_q16_o,
 output wire[9:0]domain_x0_o,domain_y0_o,domain_x1_o,domain_y1_o,
 input wire clk_i,rst_i,abort_i,start_i,output wire ready_o,busy_o,result_valid_o,input wire out_ready_i,
 input wire[31:0]ref_base_i,obs_base_i,task_id_i,frame_id_i,reference_id_i,config_epoch_i,
 output wire[3:0]error_o,output reg[5:0]count_o,output wire ambiguous_o,truncated_o,
 output wire[31:0]cycles_o,
 output wire[15:0]gray_addr_o,output wire gray_we_o,output wire[7:0]gray_data_o,input wire[7:0]gray_data_i,
 output wire[11:0]queue_addr_o,output wire queue_we_o,output wire[15:0]queue_data_o,input wire[15:0]queue_data_i,
 output wire req_valid_o,input wire req_ready_i,output wire req_write_o,output wire[31:0]req_addr_o,output wire[127:0]req_data_o,
 input wire rsp_valid_i,output wire rsp_ready_o,input wire rsp_error_i,input wire[127:0]rsp_data_i
);
 localparam WAIT_RECORD=0,LOAD_WORD=1,WRITE_BYTE=2,ACCEPT_RECORD=3;
 reg[1:0]writer;reg[6:0]byte_index;reg[31:0]word_data;reg record_error;
 wire ar,av,ab,cv;wire[3:0]ae,ce;wire[5:0]ac;
 wire[15:0]ga;wire gw;wire[7:0]gd;
 wire[31:0]tid,fid,rid,epoch;
 wire[6:0]angle;wire[3:0]scale;wire signed[15:0]dx,dy;
 wire[31:0]coarse_cost,fine_cost,ncycles,nreads;
 wire[8:0]fa,fs;wire signed[15:0]fdx,fdy;wire signed[17:0]nc,nd;wire signed[28:0]ox,oy;
 wire[7:0]evaluations;wire[239:0]evidence;wire[40:0]box;wire[15:0]grid;
 wire cancel=abort_i||(ab&&(task_id_i!=tid||frame_id_i!=fid||reference_id_i!=rid||config_epoch_i!=epoch));
 assign ready_o=ar&&writer==WAIT_RECORD;
 assign busy_o=ab||writer!=WAIT_RECORD;
 assign result_valid_o=av&&writer==WAIT_RECORD&&!cancel&&!rst_i;
 assign error_o=ae!=0?ae:record_error?4'd8:4'd0;
 assign gray_addr_o=writer==WRITE_BYTE?16'd1024+{4'd0,count_o[4:0],byte_index}:ga;
 assign gray_we_o=!cancel&&!rst_i&&(writer==WRITE_BYTE||gw);
 assign gray_data_o=writer==WRITE_BYTE?word_data[7:0]:gd;
 part_auto_native_pipeline core(.sample_external_i(sample_external_i),.sample_start_i(sample_start_i),.sample_accept_i(sample_accept_i),.sample_abort_i(sample_abort_i),
 .sample_query_i(sample_query_i),.sample_ready_o(sample_ready_o),.sample_busy_o(sample_busy_o),.sample_valid_o(sample_valid_o),.sample_reply_o(sample_reply_o),.invscale_q16_o(invscale_q16_o),.clk_i(clk_i),.rst_i(rst_i),.abort_i(abort_i),.start_i(start_i&&ready_o),
 .ready_o(ar),.busy_o(ab),.done_o(),.result_valid_o(av),.out_ready_i(out_ready_i&&writer==WAIT_RECORD),
 .ref_feature_base_i(ref_base_i),.obs_feature_base_i(obs_base_i),.task_id_i(task_id_i),.frame_id_i(frame_id_i),.reference_id_i(reference_id_i),.config_epoch_i(config_epoch_i),
 .task_id_o(tid),.frame_id_o(fid),.reference_id_o(rid),.config_epoch_o(epoch),.error_o(ae),.stage_error_o(),
 .support_ambiguous_o(ambiguous_o),.point_budget_exceeded_o(),.ref_support_count_o(),.obs_support_count_o(),
 .ref_cx_q8_o(),.ref_cy_q8_o(),.obs_cx_q8_o(),.obs_cy_q8_o(),.domain_x0_o(domain_x0_o),.domain_y0_o(domain_y0_o),.domain_x1_o(domain_x1_o),.domain_y1_o(domain_y1_o),
 .candidate_count_o(ac),.candidates_truncated_o(truncated_o),.poses_evaluated_o(),.cycles_o(cycles_o),
 .candidate_valid_o(cv),.candidate_ready_i(writer==ACCEPT_RECORD&&!cancel),
 .angle_index_o(angle),.scale_index_o(scale),.dx_q8_o(dx),.dy_q8_o(dy),.a_q14_o(),.b_q14_o(),.ia_q14_o(),.ib_q14_o(),.cost_o(coarse_cost),
 .gray_addr_o(ga),.gray_we_o(gw),.gray_data_o(gd),.gray_data_i(gray_data_i),
 .queue_addr_o(queue_addr_o),.queue_we_o(queue_we_o),.queue_data_o(queue_data_o),.queue_data_i(queue_data_i),
 .req_valid_o(req_valid_o),.req_ready_i(req_ready_i),.req_write_o(req_write_o),.req_addr_o(req_addr_o),.req_data_o(req_data_o),
 .rsp_valid_i(rsp_valid_i),.rsp_ready_o(rsp_ready_o),.rsp_error_i(rsp_error_i),.rsp_data_i(rsp_data_i),
 .candidate_error_o(ce),.refined_angle_o(fa),.refined_scale_q8_o(fs),.refined_dx_q8_o(fdx),.refined_dy_q8_o(fdy),
 .native_c_q16_o(nc),.native_d_q16_o(nd),.native_offset_x_q16_o(ox),.native_offset_y_q16_o(oy),
 .refined_cost_o(fine_cost),.native_cycles_o(ncycles),.native_reads_o(nreads),.refinement_evaluations_o(evaluations),
 .native_evidence_o(evidence),.native_coverage_grid_o(grid),.native_coverage_bbox_o(box));
 reg[31:0]selected_word;
 always@*begin
  selected_word=0;
  case(byte_index[6:2])
   0:selected_word={8'h01,14'd0,truncated_o,ambiguous_o,4'd0,ce};
   1:selected_word=tid;2:selected_word=fid;3:selected_word=rid;4:selected_word=epoch;
   5:selected_word={21'd0,scale,angle};6:selected_word={dy,dx};7:selected_word=coarse_cost;
   8:selected_word={6'd0,evaluations,fs,fa};9:selected_word={fdy,fdx};
   10:selected_word={{14{nc[17]}},nc};11:selected_word={{14{nd[17]}},nd};
   12:selected_word={{3{ox[28]}},ox};13:selected_word={{3{oy[28]}},oy};14:selected_word=fine_cost;
   15:selected_word={12'd0,evidence[0+:20]};16:selected_word={12'd0,evidence[20+:20]};
   17:selected_word={12'd0,evidence[40+:20]};18:selected_word={12'd0,evidence[60+:20]};
   19:selected_word={12'd0,evidence[80+:20]};20:selected_word={12'd0,evidence[100+:20]};
   21:selected_word={12'd0,evidence[120+:20]};22:selected_word={12'd0,evidence[140+:20]};
   23:selected_word={12'd0,evidence[160+:20]};24:selected_word={12'd0,evidence[180+:20]};
   25:selected_word={12'd0,evidence[200+:20]};26:selected_word={12'd0,evidence[220+:20]};
   27:selected_word={16'd0,grid};28:selected_word=box[31:0];29:selected_word={6'd0,invscale_q16_o,box[40:32]};
   30:selected_word=ncycles;31:selected_word=nreads;
  endcase
 end
 always@(posedge clk_i)begin
  if(rst_i||cancel)begin writer<=WAIT_RECORD;byte_index<=0;word_data<=0;count_o<=0;record_error<=0;end
  else begin
   if(start_i&&ready_o)begin count_o<=0;record_error<=0;end
   case(writer)
    WAIT_RECORD:if(cv)begin writer<=LOAD_WORD;byte_index<=0;record_error<=record_error||(ce!=0);end
    LOAD_WORD:begin word_data<=selected_word;writer<=WRITE_BYTE;end
    WRITE_BYTE:begin
     word_data<={8'd0,word_data[31:8]};byte_index<=byte_index+1'b1;
     if(byte_index==127)writer<=ACCEPT_RECORD;
     else if(byte_index[1:0]==3)writer<=LOAD_WORD;
    end
    ACCEPT_RECORD:begin count_o<=count_o+1'b1;writer<=WAIT_RECORD;end
   endcase
  end
 end
endmodule
