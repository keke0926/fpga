`timescale 1ns/1ps
// Native fixed-feature frames -> coarse pose candidates. Caller owns frozen
// frame and scratch/queue leases; no private RAM and no final defect verdict.
module part_auto_coarse_pipeline #(
 parameter integer MAX_CANDIDATES=32
)(
 output reg[9:0]domain_x0_o,domain_y0_o,domain_x1_o,domain_y1_o,
 input wire clk_i,rst_i,abort_i,start_i,
 output wire ready_o,busy_o,done_o,result_valid_o,input wire out_ready_i,
 input wire[31:0] ref_feature_base_i,obs_feature_base_i,
 input wire[31:0] task_id_i,frame_id_i,reference_id_i,config_epoch_i,
 output reg[31:0] task_id_o,frame_id_o,reference_id_o,config_epoch_o,
 output reg[3:0] error_o,stage_error_o,
 output reg support_ambiguous_o,point_budget_exceeded_o,
 output reg[13:0] ref_support_count_o,obs_support_count_o,
 output wire[15:0] ref_cx_q8_o,ref_cy_q8_o,obs_cx_q8_o,obs_cy_q8_o,
 output reg[5:0] candidate_count_o,output reg candidates_truncated_o,
 output reg[14:0] poses_evaluated_o,output reg[31:0] cycles_o,
 output wire candidate_valid_o,input wire candidate_ready_i,
 output wire[6:0] angle_index_o,output wire[3:0] scale_index_o,
 output wire signed[15:0] dx_q8_o,dy_q8_o,a_q14_o,b_q14_o,ia_q14_o,ib_q14_o,
 output wire[31:0] cost_o,
 output reg[15:0] gray_addr_o,output wire gray_we_o,output reg[7:0] gray_data_o,input wire[7:0] gray_data_i,
 output reg[11:0] queue_addr_o,output wire queue_we_o,output reg[15:0] queue_data_o,input wire[15:0] queue_data_i,
 output wire req_valid_o,input wire req_ready_i,output wire req_write_o,
 output wire[31:0] req_addr_o,output wire[127:0] req_data_o,
 input wire rsp_valid_i,output wire rsp_ready_o,input wire rsp_error_i,input wire[127:0] rsp_data_i,
 output wire sc_start_o,sc_accept_o,output wire[95:0]sc_pose_o,
 input wire sc_ready_i,sc_valid_i,input wire[31:0]sc_cost_i,input wire[3:0]sc_error_i,
 input wire[15:0]sc_gray_addr_i,input wire[11:0]sc_queue_addr_i

);
 localparam IDLE=0,BUILD_START=1,BUILD_WAIT=2,SUPPORT_START=3,SUPPORT_WAIT=4,
  DIST_START=5,DIST_WAIT=6,POINT_REQ=7,POINT_WAIT=8,POINT_WRITE=9,
  DIV_INIT=10,DIV_STEP=11,SEARCH_START=12,SEARCH_WAIT=13,RESULT=14,CANCEL=15;
 localparam[15:0] REF_COUNT_BASE=0,OBS_COUNT_BASE=9600,REF_DIST_BASE=19200,OBS_DIST_BASE=28800;
 reg[3:0] state;
 reg phase;
 reg[15:0] ref_cx_centroid,ref_cy_centroid,obs_cx_centroid,obs_cy_centroid;
 reg[15:0] ref_cx_box,ref_cy_box,obs_cx_box,obs_cy_box;
 wire search_anchor;
 assign ref_cx_q8_o=search_anchor?ref_cx_box:ref_cx_centroid;
 assign ref_cy_q8_o=search_anchor?ref_cy_box:ref_cy_centroid;
 assign obs_cx_q8_o=search_anchor?obs_cx_box:obs_cx_centroid;
 assign obs_cy_q8_o=search_anchor?obs_cy_box:obs_cy_centroid;
 reg[31:0] ref_base,obs_base;
 reg[20:0] ref_sum_x,obs_sum_x;
 reg[19:0] ref_sum_y,obs_sum_y;
 reg[13:0] point_scan_index;
 reg[6:0] point_x,point_y;
 reg[10:0] point_write_index;
 reg[1:0] div_axis;
 reg[4:0] div_step;
 reg[28:0] dividend,quotient;
 reg[14:0] remainder;
 reg[11:0] denominator;
 reg gray_we,queue_we;
 wire context_lost=state!=IDLE&&state!=CANCEL&&
  (task_id_i!=task_id_o||frame_id_i!=frame_id_o||reference_id_i!=reference_id_o||config_epoch_i!=config_epoch_o);
 wire cancel=abort_i||context_lost;
 wire core_abort=cancel||state==CANCEL;
 wire[15:0] count_base=phase?OBS_COUNT_BASE:REF_COUNT_BASE;
 wire[15:0] distance_base=phase?OBS_DIST_BASE:REF_DIST_BASE;
 wire[14:0] remainder_shift={remainder[13:0],dividend[28]};
 wire quotient_bit=remainder_shift>={3'd0,denominator};
 wire[28:0] quotient_next={quotient[27:0],quotient_bit};
 wire[14:0] remainder_next=quotient_bit?remainder_shift-{3'd0,denominator}:remainder_shift;
 assign ready_o=state==IDLE&&!rst_i&&!abort_i;
 assign busy_o=state!=IDLE;
 assign result_valid_o=state==RESULT&&!rst_i&&!cancel;
 assign done_o=result_valid_o;
 assign gray_we_o=gray_we&&!rst_i&&!core_abort;
 assign queue_we_o=queue_we&&!rst_i&&!core_abort;

 wire builder_ready,builder_busy,builder_valid,builder_grid_valid,builder_req_valid;
 wire[3:0] builder_error;
 wire[13:0] builder_grid_addr;
 wire[7:0] builder_grid_data;
 part_coarse_builder #(.STATIC_CONTEXT(1)) builder(
  .clk_i(clk_i),.rst_i(rst_i),.abort_i(core_abort),.start_i(state==BUILD_START),
  .ready_o(builder_ready),.busy_o(builder_busy),.done_o(),.result_valid_o(builder_valid),
  .out_ready_i(state==BUILD_WAIT&&!cancel),.feature_base_i(phase?obs_base:ref_base),.raw_floor_i(8'd96),
  .task_id_i(task_id_i),.frame_id_i(frame_id_i),.reference_id_i(reference_id_i),.config_epoch_i(config_epoch_i),
  .task_id_o(),.frame_id_o(),.reference_id_o(),.config_epoch_o(),.error_o(builder_error),
  .grid_valid_o(builder_grid_valid),.grid_ready_i(state==BUILD_WAIT&&!cancel),.grid_addr_o(builder_grid_addr),.grid_data_o(builder_grid_data),
  .read_requests_o(),.cycles_o(),.cells_written_o(),
  .req_valid_o(builder_req_valid),.req_ready_i(req_ready_i),.req_write_o(req_write_o),.req_addr_o(req_addr_o),.req_data_o(req_data_o),
  .rsp_valid_i(rsp_valid_i),.rsp_ready_o(rsp_ready_o),.rsp_error_i(rsp_error_i),.rsp_data_i(rsp_data_i));
 assign req_valid_o=builder_req_valid&&state==BUILD_WAIT&&!rst_i&&!cancel;

 wire support_ready,support_valid,support_found,support_ambiguous,support_gray_we,support_queue_we;
 wire[3:0] support_error;
 wire[15:0] support_gray_addr,support_queue_data;
 wire[7:0] support_gray_data;
 wire[11:0] support_queue_addr;
 wire[13:0] support_best_count;
 part_coarse_support support(
  .clk_i(clk_i),.rst_i(rst_i),.abort_i(core_abort),.start_i(state==SUPPORT_START),
  .ready_o(support_ready),.busy_o(),.done_o(),.result_valid_o(support_valid),.out_ready_i(state==SUPPORT_WAIT&&!cancel),
  .count_base_i(count_base),.label_base_i(distance_base),.threshold_i(8'd8),.error_o(support_error),
  .scratch_addr_o(support_gray_addr),.scratch_we_o(support_gray_we),.scratch_data_o(support_gray_data),.scratch_data_i(gray_data_i),
  .queue_addr_o(support_queue_addr),.queue_we_o(support_queue_we),.queue_data_o(support_queue_data),.queue_data_i(queue_data_i),
  .found_o(support_found),.ambiguous_o(support_ambiguous),.best_count_o(support_best_count),.second_count_o(),.best_label_o(),.component_count_o(),.cycles_o(),.queue_peak_o());

 wire distance_ready,distance_valid,distance_gray_we;
 wire[3:0] distance_error;
 wire[15:0] distance_gray_addr;
 wire[7:0] distance_gray_data;
 wire[13:0] distance_count;
 wire[20:0] distance_sum_x;
 wire[19:0] distance_sum_y;
 wire[6:0] distance_min_x,distance_max_x,distance_min_y,distance_max_y;
 wire[7:0] box_sum_x={1'b0,distance_min_x}+{1'b0,distance_max_x};
 wire[7:0] box_sum_y={1'b0,distance_min_y}+{1'b0,distance_max_y};
 part_coarse_distance distance(
  .clk_i(clk_i),.rst_i(rst_i),.abort_i(core_abort),.start_i(state==DIST_START),
  .ready_o(distance_ready),.busy_o(),.done_o(),.result_valid_o(distance_valid),.out_ready_i(state==DIST_WAIT&&!cancel),
  .count_base_i(count_base),.distance_base_i(distance_base),.threshold_i(8'd8),.cap_i(8'd2),.error_o(distance_error),
  .mem_addr_o(distance_gray_addr),.mem_we_o(distance_gray_we),.mem_data_o(distance_gray_data),.mem_data_i(gray_data_i),
  .seed_count_o(distance_count),.sum_x_o(distance_sum_x),.sum_y_o(distance_sum_y),
  .min_x_o(distance_min_x),.max_x_o(distance_max_x),.min_y_o(distance_min_y),.max_y_o(distance_max_y),.cycles_o());

 wire search_ready,search_valid,search_gray_we,search_candidate_valid;
 wire[3:0] search_error;
 wire[15:0] search_gray_addr;
 wire[7:0] search_gray_data;
 wire[11:0] search_queue_addr;
 wire[5:0] search_candidate_count;
 wire search_truncated;
 wire[14:0] search_poses;
 part_coarse_search #(.MAX_CANDIDATES(MAX_CANDIDATES)) search(
  .clk_i(clk_i),.rst_i(rst_i),.abort_i(core_abort),.start_i(state==SEARCH_START),
  .ready_o(search_ready),.busy_o(),.done_o(),.result_valid_o(search_valid),.out_ready_i(state==SEARCH_WAIT&&!cancel),.error_o(search_error),
  .ref_count_i(ref_support_count_o[11:0]),.obs_count_i(obs_support_count_o[11:0]),
  .ref_cx_q8_i(ref_cx_q8_o),.ref_cy_q8_i(ref_cy_q8_o),.obs_cx_q8_i(obs_cx_q8_o),.obs_cy_q8_i(obs_cy_q8_o),
  .cap_i(8'd2),.ref_distance_base_i(REF_DIST_BASE),.obs_distance_base_i(OBS_DIST_BASE),.record_base_i(16'd0),
  .mem_addr_o(search_gray_addr),.mem_we_o(search_gray_we),.mem_data_o(search_gray_data),.mem_data_i(gray_data_i),
  .queue_read_o(),.queue_addr_o(search_queue_addr),.queue_data_i(queue_data_i),
  .candidate_valid_o(search_candidate_valid),.candidate_ready_i(candidate_ready_i&&state==SEARCH_WAIT&&!cancel),
  .anchor_o(search_anchor),.angle_index_o(angle_index_o),.scale_index_o(scale_index_o),.dx_q8_o(dx_q8_o),.dy_q8_o(dy_q8_o),
  .a_q14_o(a_q14_o),.b_q14_o(b_q14_o),.ia_q14_o(ia_q14_o),.ib_q14_o(ib_q14_o),.cost_o(cost_o),
  .candidate_count_o(search_candidate_count),.candidates_truncated_o(search_truncated),.poses_evaluated_o(search_poses),.cycles_o(),.sc_start_o(sc_start_o),.sc_accept_o(sc_accept_o),.sc_pose_o(sc_pose_o),
 .sc_ready_i(sc_ready_i),.sc_valid_i(sc_valid_i),.sc_cost_i(sc_cost_i),.sc_error_i(sc_error_i),
 .sc_gray_addr_i(sc_gray_addr_i),.sc_queue_addr_i(sc_queue_addr_i));
 assign candidate_valid_o=search_candidate_valid&&state==SEARCH_WAIT&&!rst_i&&!cancel;

 always@*begin
  gray_addr_o=0;gray_data_o=0;gray_we=0;queue_addr_o=0;queue_data_o=0;queue_we=0;
  case(state)
   BUILD_START,BUILD_WAIT:begin gray_addr_o=count_base+builder_grid_addr;gray_data_o=builder_grid_data;gray_we=builder_grid_valid&&state==BUILD_WAIT;end
   SUPPORT_START,SUPPORT_WAIT:begin
    gray_addr_o=support_gray_addr;gray_data_o=support_gray_data;gray_we=support_gray_we;
    queue_addr_o=support_queue_addr;queue_data_o=support_queue_data;queue_we=support_queue_we;
   end
   DIST_START,DIST_WAIT:begin gray_addr_o=distance_gray_addr;gray_data_o=distance_gray_data;gray_we=distance_gray_we;end
   POINT_REQ,POINT_WAIT,POINT_WRITE:begin
    gray_addr_o=count_base+point_scan_index;
    queue_addr_o={phase,point_write_index};queue_data_o={2'b0,point_y,point_x};queue_we=state==POINT_WRITE;
   end
   SEARCH_START,SEARCH_WAIT:begin gray_addr_o=search_gray_addr;gray_data_o=search_gray_data;gray_we=search_gray_we;queue_addr_o=search_queue_addr;end
   default:begin end
  endcase
 end

 task advance_point;
  begin
   if(point_scan_index==14'd9599)begin
    point_scan_index<=0;point_x<=0;point_y<=0;point_write_index<=0;
    if(!phase)begin phase<=1;state<=POINT_REQ;end
    else begin div_axis<=0;state<=DIV_INIT;end
   end else begin
    point_scan_index<=point_scan_index+1'b1;
    if(point_x==127)begin point_x<=0;point_y<=point_y+1'b1;end else point_x<=point_x+1'b1;
    state<=POINT_REQ;
   end
  end
 endtask

 always@(posedge clk_i)begin
  if(rst_i)begin
   state<=IDLE;phase<=0;ref_base<=0;obs_base<=0;
   task_id_o<=0;frame_id_o<=0;reference_id_o<=0;config_epoch_o<=0;error_o<=0;stage_error_o<=0;
   support_ambiguous_o<=0;point_budget_exceeded_o<=0;ref_support_count_o<=0;obs_support_count_o<=0;
   ref_sum_x<=0;ref_sum_y<=0;obs_sum_x<=0;obs_sum_y<=0;domain_x0_o<=0;domain_x1_o<=0;domain_y0_o<=0;domain_y1_o<=0;
   ref_cx_box<=0;ref_cy_box<=0;obs_cx_box<=0;obs_cy_box<=0;
   ref_cx_centroid<=0;ref_cy_centroid<=0;obs_cx_centroid<=0;obs_cy_centroid<=0;
   candidate_count_o<=0;candidates_truncated_o<=0;poses_evaluated_o<=0;cycles_o<=0;
   point_scan_index<=0;point_x<=0;point_y<=0;point_write_index<=0;
   div_axis<=0;div_step<=0;dividend<=0;quotient<=0;remainder<=0;denominator<=0;
  end else if(cancel&&state!=IDLE&&state!=CANCEL)begin
   state<=CANCEL;candidate_count_o<=0;candidates_truncated_o<=0;poses_evaluated_o<=0;
  end else begin
   if(state!=IDLE&&state!=RESULT&&state!=CANCEL)cycles_o<=cycles_o+1'b1;
   case(state)
    IDLE:if(start_i&&ready_o)begin
     task_id_o<=task_id_i;frame_id_o<=frame_id_i;reference_id_o<=reference_id_i;config_epoch_o<=config_epoch_i;
     ref_base<=ref_feature_base_i;obs_base<=obs_feature_base_i;phase<=0;error_o<=0;stage_error_o<=0;
     support_ambiguous_o<=0;point_budget_exceeded_o<=0;ref_support_count_o<=0;obs_support_count_o<=0;
     ref_cx_centroid<=0;ref_cy_centroid<=0;obs_cx_centroid<=0;obs_cy_centroid<=0;
     candidate_count_o<=0;candidates_truncated_o<=0;poses_evaluated_o<=0;cycles_o<=0;
     state<=BUILD_START;
    end
    BUILD_START:if(builder_ready)state<=BUILD_WAIT;
    BUILD_WAIT:if(builder_valid)begin
     if(builder_error!=0)begin error_o<=builder_error;stage_error_o<=builder_error;state<=RESULT;end
     else if(!phase)begin phase<=1;state<=BUILD_START;end
     else begin phase<=0;state<=SUPPORT_START;end
    end
    SUPPORT_START:if(support_ready)state<=SUPPORT_WAIT;
    SUPPORT_WAIT:if(support_valid)begin
     support_ambiguous_o<=support_ambiguous_o||support_ambiguous;
     if(phase)obs_support_count_o<=support_best_count;else ref_support_count_o<=support_best_count;
     if(support_error!=0)begin error_o<=3;stage_error_o<=support_error;state<=RESULT;end
     else if(!support_found)begin error_o<=5;state<=RESULT;end
     else if(support_best_count>2048)begin error_o<=6;point_budget_exceeded_o<=1;state<=RESULT;end
     else if(!phase)begin phase<=1;state<=SUPPORT_START;end
     else begin phase<=0;state<=DIST_START;end
    end
    DIST_START:if(distance_ready)state<=DIST_WAIT;
    DIST_WAIT:if(distance_valid)begin
     if(distance_error!=0)begin error_o<=4;stage_error_o<=distance_error;state<=RESULT;end
     else begin
      if(!phase)begin ref_sum_x<=distance_sum_x;ref_sum_y<=distance_sum_y;
       ref_cx_box<={1'b0,box_sum_x,7'd0};ref_cy_box<={1'b0,box_sum_y,7'd0};phase<=1;state<=DIST_START;end
      else begin
       obs_sum_x<=distance_sum_x;obs_sum_y<=distance_sum_y;phase<=0;
        domain_x0_o<=distance_min_x<6?10'd28:{distance_min_x,3'b0}-10'd16;
        domain_y0_o<=distance_min_y<6?10'd28:{distance_min_y,3'b0}-10'd16;
        domain_x1_o<=distance_max_x>=121?10'd996:{distance_max_x,3'b0}+10'd24;
        domain_y1_o<=distance_max_y>=69?10'd572:{distance_max_y,3'b0}+10'd24;
       obs_cx_box<={1'b0,box_sum_x,7'd0};obs_cy_box<={1'b0,box_sum_y,7'd0};
       point_scan_index<=0;point_x<=0;point_y<=0;point_write_index<=0;state<=POINT_REQ;
      end
     end
    end
    POINT_REQ:state<=POINT_WAIT;
    POINT_WAIT:if(gray_data_i>=8)state<=POINT_WRITE;else advance_point;
    POINT_WRITE:begin point_write_index<=point_write_index+1'b1;advance_point;end
    DIV_INIT:begin
     case(div_axis)
      0:begin dividend<={ref_sum_x,8'd0};denominator<=ref_support_count_o[11:0];end
      1:begin dividend<={1'b0,ref_sum_y,8'd0};denominator<=ref_support_count_o[11:0];end
      2:begin dividend<={obs_sum_x,8'd0};denominator<=obs_support_count_o[11:0];end
      3:begin dividend<={1'b0,obs_sum_y,8'd0};denominator<=obs_support_count_o[11:0];end
     endcase
     remainder<=0;quotient<=0;div_step<=0;state<=DIV_STEP;
    end
    DIV_STEP:begin
     dividend<={dividend[27:0],1'b0};quotient<=quotient_next;remainder<=remainder_next;
     if(div_step==28)begin
      case(div_axis)
       0:ref_cx_centroid<=quotient_next[15:0];1:ref_cy_centroid<=quotient_next[15:0];
       2:obs_cx_centroid<=quotient_next[15:0];3:obs_cy_centroid<=quotient_next[15:0];
      endcase
      if(div_axis==3)state<=SEARCH_START;else begin div_axis<=div_axis+1'b1;state<=DIV_INIT;end
     end else div_step<=div_step+1'b1;
    end
    SEARCH_START:if(search_ready)state<=SEARCH_WAIT;
    SEARCH_WAIT:if(search_valid)begin
     candidate_count_o<=search_candidate_count;candidates_truncated_o<=search_truncated;poses_evaluated_o<=search_poses;
     if(search_error!=0)begin error_o<=7;stage_error_o<=search_error;end
     state<=RESULT;
    end
    RESULT:if(out_ready_i)state<=IDLE;
    CANCEL:if(!builder_busy)state<=IDLE;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
