`timescale 1ns/1ps
// Full sampled display-frame transaction; independent video is never stalled.
// Exact 8-connected components, bounded circular pixel frontier, RAM record queue.
// All qualified components are queued in raster order and use one classifier.
// Record RAM has 2048 entries: floor(256*150/20)=1920 is a conservative bound.
// Only counters and small classifier maps reset; pixel/record memories do not.
module shared_vision_engine #(
    parameter TEMPLATE_FILE="rtl/apps/digit_variants_14.mem",
    parameter SHAPE_FILE="rtl/apps/geometry_templates.mem",
    parameter DIGIT_MIRROR_X=1,parameter ENABLE_CANNY=1,parameter ENABLE_LOCAL=0
)(

 input wire local_req_i,output wire local_grant_o,
 input wire local_region_start_i,local_region_ready_i,
 output wire local_region_idle_o,local_region_valid_o,local_region_done_o,
 output wire[8:0]local_region_x0_o,local_region_x1_o,
 output wire[7:0]local_region_y0_o,local_region_y1_o,
 output wire[19:0]local_region_support_o,output wire local_region_uncertain_o,
 output wire[11:0]local_region_count_o,output wire[3:0]local_region_error_o,
 input wire[15:0]local_gray_addr_i,local_state_addr_i,
 input wire local_gray_we_i,input wire[1:0]local_state_we_i,
 input wire[7:0]local_gray_data_i,input wire[1:0]local_state_data_i,
 output wire[7:0]local_gray_data_o,output wire[1:0]local_state_data_o,
 input wire[11:0]local_queue_addr_i,input wire local_queue_we_i,input wire[15:0]local_queue_data_i,
 output wire[15:0]local_queue_data_o,
    input wire clk,rst_n,abort_i,start_i,
    input wire [31:0] frame_id_i,config_tag_i,
    input wire sample_valid_i,input wire [15:0] sample_addr_i,
    input wire [7:0] sample_gray_i,input wire end_i,capture_error_i,
    output wire ready_o,busy_o,
    output wire object_valid_o,output wire [10:0] object_index_o,
    output wire [4:0] object_class_o,output wire object_accepted_o,
    output wire [3:0] object_reason_o,
    output wire [8:0] object_x0_o,object_x1_o,
    output wire [7:0] object_y0_o,object_y1_o,
    output wire [15:0] object_area_o,output wire [13:0] object_score_o,
    output wire preview_write_o,output wire [15:0] preview_addr_o,output wire preview_bit_o,
    output wire descriptor_we_o,output wire [10:0] descriptor_addr_o,
    output wire [47:0] descriptor_data_o,input wire [47:0] descriptor_data_i,
    output wire annotation_write_o,output wire [15:0] annotation_addr_o,
    output wire result_valid_o,
    output wire [31:0] result_frame_id_o,result_config_tag_o,
    output wire [11:0] result_count_o,
    output wire [15:0] result_candidate_count_o,result_declined_count_o,result_omitted_count_o,
    output wire result_overflow_o,output wire [3:0] result_reason_o,
    output wire [7:0] threshold_o,output wire [31:0] processing_cycles_o,
 input wire part_start_i,part_sample_valid_i,part_end_i,part_capture_error_i,
 input wire[13:0] part_sample_addr_i,input wire[7:0]part_sample_gray_i,part_low_i,part_high_i,
 input wire[31:0]part_frame_id_i,part_config_tag_i,
 output wire part_ready_o,part_busy_o,part_done_o,part_valid_o,
 output wire[3:0]part_error_o,part_phase_o,
 output wire[31:0]part_frame_id_o,part_config_tag_o,part_cycles_o,
 output wire part_edge_valid_o,part_edge_bit_o,output wire[13:0]part_edge_addr_o
);

 localparam FREE=0,GEN=1,PART_CAPTURE=2,PART_RUN=3,LOCAL=4;
 reg[2:0]owner;
 reg region_running;
 wire local_request=ENABLE_LOCAL&&local_req_i;
 wire local_owner=owner==LOCAL;
 assign local_grant_o=rst_n&&!abort_i&&local_owner;
 wire part_owner=owner==PART_CAPTURE||owner==PART_RUN;
 wire gready,gbusy;
 wire gstart=rst_n&&!abort_i&&!part_owner&&!local_owner&&!local_request&&gready&&start_i;
 wire region_start=local_owner&&gready&&!region_running&&local_region_start_i;
 wire local_scratch=local_owner&&!region_running&&!region_start;
 wire gabort=abort_i||part_owner||(region_running&&!local_request)||
   (local_owner&&!region_running&&!region_start);
 wire core_result;
 assign local_region_idle_o=local_owner&&gready&&!region_running;
 assign local_region_done_o=region_running&&core_result;
 assign result_valid_o=!region_running&&core_result;
 assign local_region_count_o=result_count_o;
 assign local_region_error_o=result_reason_o;
 assign local_region_x0_o=object_x0_o;assign local_region_x1_o=object_x1_o;
 assign local_region_y0_o=object_y0_o;assign local_region_y1_o=object_y1_o;
 assign ready_o=rst_n&&!abort_i&&!part_owner&&!local_owner&&!local_request&&gready;
 assign busy_o=gbusy;
 assign part_ready_o=ENABLE_CANNY&&rst_n&&!abort_i&&owner==FREE&&gready&&!start_i&&!local_request;
 assign part_busy_o=part_owner;
 reg[14:0]captured;reg capture_bad,issued;
 reg[7:0]low,high;reg[31:0]frame,tag;
 reg done_q,valid_q;reg[3:0]error_q;reg[31:0]result_frame,result_tag,result_cycles;
 assign part_done_o=done_q&&rst_n&&!abort_i;
 assign part_valid_o=valid_q&&rst_n&&!abort_i;
 assign part_error_o=error_q;
 assign part_frame_id_o=result_frame;assign part_config_tag_o=result_tag;assign part_cycles_o=result_cycles;
 wire good_pixel=part_sample_valid_i&&captured<16384&&{1'b0,part_sample_addr_i}==captured;
 wire[15:0]gaddr,saddr;wire gwe;wire[1:0]swe;
 wire[7:0]gdata,gr;wire[1:0]sdata,sr;
 assign local_gray_data_o=gr;assign local_state_data_o=sr;
 wire alease,adone,avalid,aev,aeb;wire[13:0]aea;wire[3:0]aerr,aphase;
 wire[31:0]af,at,acycles;
 assign part_phase_o=part_owner?aphase:4'd0;
 assign part_edge_valid_o=aev&&owner==PART_RUN&&rst_n&&!abort_i;
 assign part_edge_bit_o=aeb;assign part_edge_addr_o=aea;
 wire request=owner==PART_RUN&&!issued;
 always @(posedge clk)begin
  if(!rst_n||abort_i)begin
   owner<=FREE;region_running<=0;captured<=0;capture_bad<=0;issued<=0;low<=0;high<=0;frame<=0;tag<=0;
   done_q<=0;valid_q<=0;error_q<=0;result_frame<=0;result_tag<=0;result_cycles<=0;
  end else begin
   done_q<=0;valid_q<=0;
   if(region_start)region_running<=1;
   if(region_running&&core_result)region_running<=0;
   case(owner)
    FREE:if(local_request&&gready&&!alease)owner<=LOCAL;
     else if(gstart)owner<=GEN;
     else if(part_start_i&&part_ready_o)begin
      owner<=PART_CAPTURE;captured<=0;capture_bad<=part_capture_error_i;issued<=0;
      low<=part_low_i;high<=part_high_i;frame<=part_frame_id_i;tag<=part_config_tag_i;error_q<=0;
     end
    LOCAL:if(!local_request)begin owner<=FREE;region_running<=0;end
    GEN:if(result_valid_o&&!gstart)owner<=FREE;
    PART_CAPTURE:begin
     if(good_pixel)captured<=captured+1'b1;
     if(part_capture_error_i||(part_sample_valid_i&&!good_pixel))capture_bad<=1;
     if(part_end_i)begin
      if(capture_bad||part_capture_error_i||(part_sample_valid_i&&!good_pixel)||
         ({1'b0,captured}+{15'd0,good_pixel})!=16'd16384)begin
       owner<=FREE;done_q<=1;valid_q<=0;error_q<=4;result_frame<=frame;result_tag<=tag;result_cycles<=0;
      end else owner<=PART_RUN;
     end
    end
    PART_RUN:begin
     if(request)issued<=1;
     if(adone)begin
      owner<=FREE;done_q<=1;valid_q<=avalid;error_q<=aerr;result_frame<=af;result_tag<=at;result_cycles<=acycles;
     end
    end
   endcase
  end
 end
 canny_scratch_adapter scratch(
 .clk(clk),.rst_n(rst_n),.abort_i(abort_i),.request_i(request),.low_i(low),.high_i(high),.frame_id_i(frame),.config_tag_i(tag),
 .host_gray_addr_i(local_scratch?local_gray_addr_i:(owner==PART_CAPTURE?{2'b0,part_sample_addr_i}:gaddr)),.host_state_addr_i(local_scratch?local_state_addr_i:saddr),
 .host_gray_we_i(local_scratch?local_gray_we_i:(owner==PART_CAPTURE?good_pixel:(!part_owner&&gwe))),
 .host_state_we_i(local_scratch?local_state_we_i:(part_owner?2'b00:swe)),
 .host_gray_data_i(local_scratch?local_gray_data_i:(owner==PART_CAPTURE?part_sample_gray_i:gdata)),.host_state_data_i(local_scratch?local_state_data_i:sdata),
 .host_gray_data_o(gr),.host_state_data_o(sr),.leased_o(alease),.done_o(adone),.valid_o(avalid),
 .error_o(aerr),.phase_o(aphase),.frame_id_o(af),.config_tag_o(at),.cycles_o(acycles),
 .edge_valid_o(aev),.edge_bit_o(aeb),.edge_addr_o(aea));
 multi_target_core #(.ENABLE_LOCAL(ENABLE_LOCAL),.TEMPLATE_FILE(TEMPLATE_FILE),.SHAPE_FILE(SHAPE_FILE),.DIGIT_MIRROR_X(DIGIT_MIRROR_X)) core(.clk(clk),
.rst_n(rst_n),
.local_grant_i(local_scratch),.local_queue_addr_i(local_queue_addr_i),.local_queue_we_i(local_queue_we_i),
.local_queue_data_i(local_queue_data_i),.local_queue_data_o(local_queue_data_o),
.abort_i(gabort),
.start_i(gstart||region_start),
.regions_i(region_start),.region_ready_i(local_region_ready_i),.region_valid_o(local_region_valid_o),
.region_support_o(local_region_support_o),.region_uncertain_o(local_region_uncertain_o),
.frame_id_i(frame_id_i),
.config_tag_i(config_tag_i),
.sample_valid_i(sample_valid_i),
.sample_addr_i(sample_addr_i),
.sample_gray_i(sample_gray_i),
.end_i(end_i),
.capture_error_i(capture_error_i),
.ready_o(gready),
.busy_o(gbusy),
.object_valid_o(object_valid_o),
.object_index_o(object_index_o),
.object_class_o(object_class_o),
.object_accepted_o(object_accepted_o),
.object_reason_o(object_reason_o),
.object_x0_o(object_x0_o),
.object_x1_o(object_x1_o),
.object_y0_o(object_y0_o),
.object_y1_o(object_y1_o),
.object_area_o(object_area_o),
.object_score_o(object_score_o),
.preview_write_o(preview_write_o),
.preview_addr_o(preview_addr_o),
.preview_bit_o(preview_bit_o),
.descriptor_we_o(descriptor_we_o),
.descriptor_addr_o(descriptor_addr_o),
.descriptor_data_o(descriptor_data_o),
.descriptor_data_i(descriptor_data_i),
.annotation_write_o(annotation_write_o),
.annotation_addr_o(annotation_addr_o),
.result_valid_o(core_result),
.result_frame_id_o(result_frame_id_o),
.result_config_tag_o(result_config_tag_o),
.result_count_o(result_count_o),
.result_candidate_count_o(result_candidate_count_o),
.result_declined_count_o(result_declined_count_o),
.result_omitted_count_o(result_omitted_count_o),
.result_overflow_o(result_overflow_o),
.result_reason_o(result_reason_o),
.threshold_o(threshold_o),
.processing_cycles_o(processing_cycles_o),
 .scratch_gray_addr_o(gaddr),.scratch_state_addr_o(saddr),.scratch_gray_we_o(gwe),.scratch_state_we_o(swe),
 .scratch_gray_data_o(gdata),.scratch_state_data_o(sdata),.scratch_gray_data_i(gr),.scratch_state_data_i(sr));
endmodule
