`timescale 1ns/1ps
// Full-scene queue display. Eight rows are a page cache, never an object limit.
// Descriptor/annotated overview banks and cache metadata publish atomically.
module video_roi_digit_app #(parameter AUTO_PAGE_CYCLES=96000000,parameter CANNY_TIMEOUT_CYCLES=96000000,parameter ENABLE_LOCAL=0,parameter EXTERNAL_FONT=0)(

 input wire local_region_start_i,local_region_ready_i,
 output wire local_region_idle_o,local_region_valid_o,local_region_done_o,
 output wire[8:0]local_region_x0_o,local_region_x1_o,
 output wire[7:0]local_region_y0_o,local_region_y1_o,
 output wire[11:0]local_region_count_o,output wire[3:0]local_region_error_o,
 input wire local_req_i,output wire local_grant_o,
 input wire[15:0]local_gray_addr_i,local_state_addr_i,
 input wire local_gray_we_i,input wire[1:0]local_state_we_i,
 input wire[7:0]local_gray_data_i,input wire[1:0]local_state_data_i,
 output wire[7:0]local_gray_data_o,output wire[1:0]local_state_data_o,
 input wire[11:0]local_queue_addr_i,input wire local_queue_we_i,input wire[15:0]local_queue_data_i,
 output wire[15:0]local_queue_data_o,
 input wire[15:0]font_bits_i,output wire[9:0]font_address_o,
 input wire clk,rst_n,in_hs,in_vs,in_de,in_sof,in_eof,
 input wire[11:0]in_x,in_y,input wire[7:0]in_gray,
 input wire[23:0]in_rgb,input wire in_align_error,
 input wire cfg_app_mode,cfg_auto_mode,cfg_median_enable,
 input wire[1:0]cfg_view_mode,input wire[10:0]cfg_manual_threshold,
 input wire canny_enable_i,source_locked_i,
 input wire[31:0]source_frame_i,source_crc_i,
 input wire[9:0]canny_x_i,canny_y_i,input wire[7:0]canny_low_i,canny_high_i,
 input wire[31:0]canny_frame_i,canny_crc_i,canny_reference_i,canny_tag_i,
 output wire[3:0]canny_state_o,canny_error_o,output wire[14:0]canny_samples_o,canny_edges_o,
 output wire[31:0]canny_cycles_o,canny_display_commits_o,
 output reg out_hs,out_vs,out_de,out_sof,out_eof,
 output reg[11:0]out_x,out_y,output reg[23:0]out_rgb);
 wire[15:0]configuration={cfg_app_mode,cfg_auto_mode,cfg_median_enable,cfg_view_mode,cfg_manual_threshold};
 reg configuration_seen;reg[15:0]frame_configuration;
 reg[31:0]display_frame_id,config_generation;
 wire source_sof=in_de&&in_sof;
 wire good_sof=source_sof&&in_x==0&&in_y==0&&!in_align_error;
 wire configuration_changed=source_sof&&(!configuration_seen||configuration!=frame_configuration);
 wire[31:0]next_frame=display_frame_id+32'd1;
 reg capture_active,capture_bad;reg[11:0]expected_x,expected_y;
 reg[19:0]capture_pixels;reg work_active,back_bank,front_bank;
 reg[31:0]work_frame,work_tag;
 reg pending_valid,pending_ready,pending_usable,pending_snapshot,pending_bank;
 reg[31:0]pending_frame,pending_tag;reg[11:0]pending_count;
 reg[7:0]pending_page;reg[8:0]pending_page_count;reg[3:0]pending_row_count;
 reg pending_overflow;reg[3:0]pending_reason;
 reg preview_valid;reg[31:0]preview_frame,preview_tag;
 reg[11:0]preview_count;reg[7:0]preview_page;reg[8:0]preview_page_count;
 reg[3:0]preview_row_count;reg preview_overflow;reg[3:0]preview_reason;
 reg[4:0]back_class[0:7],preview_class[0:7];
 reg[8:0]back_x0[0:7],back_x1[0:7],preview_x0[0:7],preview_x1[0:7];
 reg[7:0]back_y0[0:7],back_y1[0:7],preview_y0[0:7],preview_y1[0:7];
 reg[15:0]preview_write_count;reg[11:0]object_write_count;reg stream_bad;
 reg[31:0]page_timer;reg page_turn_due;reg page_read_wait;reg[2:0]page_read_index;
 wire engine_ready,engine_busy,engine_result_valid;
 wire[31:0]engine_result_frame,engine_result_tag;
 wire[11:0]engine_count;wire engine_overflow;wire[3:0]engine_reason;
 wire object_write,object_accepted;wire[10:0]object_index;
 wire[4:0]object_class;wire[3:0]object_reason;wire[8:0]object_x0,object_x1;
 wire[7:0]object_y0,object_y1;wire plane_write,plane_bit;wire[15:0]plane_addr;
 wire annotation_write;wire[15:0]annotation_addr;
 wire descriptor_we;wire[10:0]descriptor_addr;wire[47:0]descriptor_data,descriptor_q,page_data;
 wire[10:0]page_addr={pending_page,3'b000}+{8'd0,page_read_index};
 wire part_abort;
 wire engine_abort=(!cfg_app_mode&&configuration_changed)||(source_sof&&capture_active)||(cfg_app_mode&&work_active)||part_abort;
 wire engine_start=good_sof&&!configuration_changed&&!capture_active&&!work_active&&
                    (!pending_valid||!pending_snapshot)&&!cfg_app_mode&&engine_ready;
 wire source_bad=in_x!=expected_x||in_y!=expected_y;
 wire capture_bad_now=capture_bad||in_align_error||(in_de&&(source_bad||in_sof));
 reg sample_valid_q;reg[15:0]sample_addr_q;reg[7:0]sample_gray_q;
 wire sample_now=(engine_start||(capture_active&&!source_sof))&&in_de&&
                 in_x<1024&&in_y<600&&in_x[1:0]==0&&in_y[1:0]==0;
 wire engine_end=capture_active&&in_de&&in_eof&&!source_sof;
 wire engine_capture_error=capture_active&&(capture_bad_now||
      (engine_end&&(in_x!=1023||in_y!=599||capture_pixels!=20'd614399)));
 wire part_start,part_ready,part_done,part_valid,part_sample_valid,part_end;
 wire[13:0]part_sample_addr,part_edge_addr;wire[7:0]part_sample_gray,part_low,part_high;
 wire[31:0]part_frame,part_tag,part_result_frame,part_result_tag,part_cycles;
 wire[3:0]part_error;wire part_edge_valid,part_edge_bit,part_write,part_write_bank;
 wire part_display_valid,part_display_bank;wire[9:0]part_display_x,part_display_y;
 canny_roi_scheduler #(.TIMEOUT_CYCLES(CANNY_TIMEOUT_CYCLES)) u_canny_scheduler(
 .clk(clk),.rst_n(rst_n),.enable_i(canny_enable_i),.part_mode_i(cfg_app_mode),.locked_i(source_locked_i),
 .source_frame_i(source_frame_i),.source_crc_i(source_crc_i),.frame_i(canny_frame_i),.crc_i(canny_crc_i),.reference_i(canny_reference_i),.tag_i(canny_tag_i),
 .roi_x_i(canny_x_i),.roi_y_i(canny_y_i),.low_i(canny_low_i),.high_i(canny_high_i),.frontend_i(configuration),
 .de_i(in_de),.sof_i(in_sof),.eof_i(in_eof),.align_error_i(in_align_error),.x_i(in_x),.y_i(in_y),.gray_i(in_gray),
 .ready_i(part_ready),.done_i(part_done),.valid_i(part_valid),.edge_valid_i(part_edge_valid),.edge_bit_i(part_edge_bit),.edge_addr_i(part_edge_addr),
 .error_i(part_error),.result_frame_i(part_result_frame),.result_tag_i(part_result_tag),.cycles_i(part_cycles),
 .start_o(part_start),.abort_o(part_abort),.sample_valid_o(part_sample_valid),.end_o(part_end),.sample_addr_o(part_sample_addr),.sample_gray_o(part_sample_gray),
 .low_o(part_low),.high_o(part_high),.frame_o(part_frame),.tag_o(part_tag),
 .write_o(part_write),.write_bank_o(part_write_bank),.display_valid_o(part_display_valid),.display_bank_o(part_display_bank),.display_x_o(part_display_x),.display_y_o(part_display_y),
 .state_o(canny_state_o),.error_o(canny_error_o),.samples_o(canny_samples_o),.edges_o(canny_edges_o),.cycles_o(canny_cycles_o),.display_commits_o(canny_display_commits_o));
 shared_vision_engine #(.ENABLE_CANNY(1),.ENABLE_LOCAL(ENABLE_LOCAL)) u_engine(
 .local_region_start_i(local_region_start_i),.local_region_ready_i(local_region_ready_i),.local_region_idle_o(local_region_idle_o),.local_region_valid_o(local_region_valid_o),.local_region_done_o(local_region_done_o),.local_region_x0_o(local_region_x0_o),.local_region_x1_o(local_region_x1_o),.local_region_y0_o(local_region_y0_o),.local_region_y1_o(local_region_y1_o),.local_region_count_o(local_region_count_o),.local_region_error_o(local_region_error_o),.local_req_i(local_req_i),.local_grant_o(local_grant_o),
 .local_gray_addr_i(local_gray_addr_i),.local_state_addr_i(local_state_addr_i),
 .local_gray_we_i(local_gray_we_i),.local_state_we_i(local_state_we_i),
 .local_gray_data_i(local_gray_data_i),.local_state_data_i(local_state_data_i),
 .local_gray_data_o(local_gray_data_o),.local_state_data_o(local_state_data_o),
 .local_queue_addr_i(local_queue_addr_i),.local_queue_we_i(local_queue_we_i),
 .local_queue_data_i(local_queue_data_i),.local_queue_data_o(local_queue_data_o),
 .clk(clk),.rst_n(rst_n),.abort_i(engine_abort),.start_i(engine_start),
 .frame_id_i(next_frame),.config_tag_i(config_generation),
 .sample_valid_i(sample_valid_q),.sample_addr_i(sample_addr_q),.sample_gray_i(sample_gray_q),
 .end_i(engine_end),.capture_error_i(engine_capture_error),.ready_o(engine_ready),.busy_o(engine_busy),
 .object_valid_o(object_write),.object_index_o(object_index),.object_class_o(object_class),
 .object_accepted_o(object_accepted),.object_reason_o(object_reason),
 .object_x0_o(object_x0),.object_x1_o(object_x1),.object_y0_o(object_y0),.object_y1_o(object_y1),
 .object_area_o(),.object_score_o(),.preview_write_o(plane_write),.preview_addr_o(plane_addr),.preview_bit_o(plane_bit),
 .annotation_write_o(annotation_write),.annotation_addr_o(annotation_addr),
 .descriptor_we_o(descriptor_we),.descriptor_addr_o(descriptor_addr),.descriptor_data_o(descriptor_data),.descriptor_data_i(descriptor_q),
 .result_valid_o(engine_result_valid),.result_frame_id_o(engine_result_frame),.result_config_tag_o(engine_result_tag),
 .result_count_o(engine_count),.result_candidate_count_o(),.result_declined_count_o(),.result_omitted_count_o(),
 .result_overflow_o(engine_overflow),.result_reason_o(engine_reason),.threshold_o(),.processing_cycles_o(),
 .part_start_i(part_start),.part_sample_valid_i(part_sample_valid),.part_sample_addr_i(part_sample_addr),.part_sample_gray_i(part_sample_gray),.part_end_i(part_end),.part_capture_error_i(1'b0),
 .part_low_i(part_low),.part_high_i(part_high),.part_frame_id_i(part_frame),.part_config_tag_i(part_tag),
 .part_ready_o(part_ready),.part_busy_o(),.part_done_o(part_done),.part_valid_o(part_valid),.part_error_o(part_error),.part_phase_o(),
 .part_frame_id_o(part_result_frame),.part_config_tag_o(part_result_tag),.part_cycles_o(part_cycles),.part_edge_valid_o(part_edge_valid),.part_edge_bit_o(part_edge_bit),.part_edge_addr_o(part_edge_addr));
 target_descriptor_ram u_descriptors(.clk(clk),.engine_bank_i(back_bank),
 .engine_we_i(rst_n&&work_active&&!engine_abort&&descriptor_we),.engine_addr_i(descriptor_addr),
 .engine_data_i(descriptor_data),.engine_data_o(descriptor_q),
 .page_bank_i(pending_bank),.page_addr_i(page_addr),.page_data_o(page_data));
 always@(posedge clk)begin
  if(!rst_n)begin
   configuration_seen<=0;frame_configuration<=16'hf080;display_frame_id<=0;config_generation<=0;
   capture_active<=0;capture_bad<=0;expected_x<=0;expected_y<=0;capture_pixels<=0;
   sample_valid_q<=0;sample_addr_q<=0;sample_gray_q<=0;
  end else begin
   sample_valid_q<=sample_now&&!engine_abort;sample_addr_q<={in_y[9:2],in_x[9:2]};sample_gray_q<=in_gray;
   if(source_sof)begin
    configuration_seen<=1;frame_configuration<=configuration;
    if(configuration_changed)config_generation<=config_generation+32'd1;
    if(good_sof)display_frame_id<=next_frame;
   end
   if(engine_abort)begin capture_active<=0;capture_bad<=0;capture_pixels<=0;end
   else if(engine_start)begin
    capture_active<=1;capture_bad<=in_eof;capture_pixels<=1;expected_x<=1;expected_y<=0;
   end else if(capture_active)begin
    if(in_align_error)capture_bad<=1;
    if(in_de)begin
     capture_bad<=capture_bad_now;
     if(capture_pixels<20'hfffff)capture_pixels<=capture_pixels+1'b1;else capture_bad<=1;
     if(expected_x==1023)begin expected_x<=0;expected_y<=expected_y+1'b1;end
     else expected_x<=expected_x+1'b1;
     if(in_eof)capture_active<=0;
    end
   end
  end
 end
 function[8:0]pages_for;input[11:0]n;begin pages_for=(n+12'd7)>>3;end endfunction
 function[3:0]rows_for;input[11:0]n;input[7:0]p;reg[11:0]remaining;begin
  remaining=n-{1'b0,p,3'b000};rows_for=remaining>8?4'd8:remaining[3:0];
 end endfunction
 wire result_identity=work_active&&engine_result_frame==work_frame&&engine_result_tag==work_tag&&engine_result_tag==config_generation;
 wire result_complete=(engine_reason==0||engine_reason==2)&&!engine_overflow&&!stream_bad&&
        preview_write_count==16'd38400&&object_write_count==engine_count&&engine_count<=12'd2048;
 wire[8:0]incoming_pages=pages_for(engine_count);
 wire page_request_pending=pending_valid&&!pending_snapshot;
 // A result may replace a staged page before SOF. Keep that already-consumed
 // page turn, otherwise frequent result completion can repeatedly cancel it.
 wire[7:0]requested_page=page_request_pending?pending_page:preview_page;
 wire[7:0]clamped_page=!preview_valid||incoming_pages==0?8'd0:
        ({1'b0,requested_page}<incoming_pages?requested_page:incoming_pages[7:0]-8'd1);
 wire take_due_with_result=page_turn_due&&!page_request_pending&&preview_valid&&preview_page_count>1&&incoming_pages>1&&{1'b0,preview_page}<incoming_pages;
 wire[7:0]incoming_page=take_due_with_result?({1'b0,clamped_page}+9'd1<incoming_pages?clamped_page+8'd1:8'd0):clamped_page;
 wire[7:0]next_page={1'b0,preview_page}+9'd1<preview_page_count?preview_page+8'd1:8'd0;
 integer k;
 always@(posedge clk)begin
  if(!rst_n)begin
   work_active<=0;back_bank<=1;front_bank<=0;work_frame<=0;work_tag<=0;
   pending_valid<=0;pending_ready<=0;pending_usable<=0;pending_snapshot<=0;pending_bank<=0;
   pending_frame<=0;pending_tag<=0;pending_count<=0;pending_page<=0;pending_page_count<=0;pending_row_count<=0;pending_overflow<=0;pending_reason<=0;
   preview_valid<=0;preview_frame<=0;preview_tag<=0;preview_count<=0;preview_page<=0;preview_page_count<=0;preview_row_count<=0;preview_overflow<=0;preview_reason<=0;
   preview_write_count<=0;object_write_count<=0;stream_bad<=0;page_timer<=0;page_turn_due<=0;page_read_wait<=0;page_read_index<=0;
   for(k=0;k<8;k=k+1)begin
    back_class[k]<=14;back_x0[k]<=0;back_x1[k]<=0;back_y0[k]<=0;back_y1[k]<=0;
    preview_class[k]<=14;preview_x0[k]<=0;preview_x1[k]<=0;preview_y0[k]<=0;preview_y1[k]<=0;
   end
  end else if(engine_abort||(source_sof&&cfg_app_mode))begin
   work_active<=0;pending_valid<=0;pending_ready<=0;pending_usable<=0;preview_valid<=0;preview_count<=0;
   pending_count<=0;pending_row_count<=0;preview_page<=0;preview_page_count<=0;preview_row_count<=0;preview_overflow<=0;preview_reason<=0;
   preview_write_count<=0;object_write_count<=0;stream_bad<=0;page_timer<=0;page_turn_due<=0;page_read_wait<=0;page_read_index<=0;
   for(k=0;k<8;k=k+1)begin
    back_class[k]<=14;back_x0[k]<=0;back_x1[k]<=0;back_y0[k]<=0;back_y1[k]<=0;
    preview_class[k]<=14;preview_x0[k]<=0;preview_x1[k]<=0;preview_y0[k]<=0;preview_y1[k]<=0;
   end
  end else begin
   // Result refresh deliberately never resets the wall-clock page timer.
   if(configuration_seen&&!frame_configuration[15]&&preview_valid&&preview_page_count>1)begin
    if(page_timer>=AUTO_PAGE_CYCLES-1)begin page_timer<=0;page_turn_due<=1;end
    else page_timer<=page_timer+32'd1;
   end else begin page_timer<=0;page_turn_due<=0;end
   // Staged rows and their mask are never made visible before a frame boundary.
   if(good_sof&&pending_valid&&pending_ready)begin
    pending_valid<=0;pending_ready<=0;
    if(pending_tag==config_generation)begin
     preview_valid<=pending_usable;preview_frame<=pending_frame;preview_tag<=pending_tag;
     preview_count<=pending_usable?pending_count:12'd0;preview_page<=pending_usable?pending_page:8'd0;
     preview_page_count<=pending_usable?pending_page_count:9'd0;preview_row_count<=pending_usable?pending_row_count:4'd0;
     preview_overflow<=pending_overflow;preview_reason<=pending_reason;
     if(pending_usable)front_bank<=pending_bank;
     for(k=0;k<8;k=k+1)begin
      preview_class[k]<=pending_usable&&k<pending_row_count?back_class[k]:5'd14;
      preview_x0[k]<=pending_usable&&k<pending_row_count?back_x0[k]:9'd0;
      preview_x1[k]<=pending_usable&&k<pending_row_count?back_x1[k]:9'd0;
      preview_y0[k]<=pending_usable&&k<pending_row_count?back_y0[k]:8'd0;
      preview_y1[k]<=pending_usable&&k<pending_row_count?back_y1[k]:8'd0;
     end
    end
   end
   if(pending_valid&&!pending_ready)begin
    if(page_read_wait)page_read_wait<=0;
    else begin
     back_x0[page_read_index]<={1'b0,page_data[7:0]};back_x1[page_read_index]<={1'b0,page_data[15:8]};
     back_y0[page_read_index]<=page_data[23:16];back_y1[page_read_index]<=page_data[31:24];
     back_class[page_read_index]<=page_data[37]?page_data[36:32]:(page_data[36:32]==15?5'd15:5'd14);
     if({1'b0,page_read_index}+4'd1==pending_row_count)pending_ready<=1;
     else begin page_read_index<=page_read_index+3'd1;page_read_wait<=1;end
    end
   end
   if(page_turn_due&&preview_valid&&preview_page_count>1&&!pending_valid&&!engine_result_valid)begin
    pending_valid<=1;pending_ready<=0;pending_usable<=1;pending_snapshot<=0;pending_bank<=front_bank;
    pending_frame<=preview_frame;pending_tag<=preview_tag;pending_count<=preview_count;
    pending_page<=next_page;pending_page_count<=preview_page_count;pending_row_count<=rows_for(preview_count,next_page);
    pending_overflow<=preview_overflow;pending_reason<=preview_reason;
    page_read_index<=0;page_read_wait<=1;page_turn_due<=0;
    for(k=0;k<8;k=k+1)begin back_class[k]<=14;back_x0[k]<=0;back_x1[k]<=0;back_y0[k]<=0;back_y1[k]<=0;end
   end
   if(engine_start)begin
    work_active<=1;back_bank<=!front_bank;work_frame<=next_frame;work_tag<=config_generation;
    preview_write_count<=0;object_write_count<=0;stream_bad<=0;
   end
   if(work_active&&plane_write)begin
    if(plane_addr!=preview_write_count||preview_write_count>=38400||annotation_write)stream_bad<=1;
    if(preview_write_count<16'hffff)preview_write_count<=preview_write_count+1'b1;
   end
   if(work_active&&annotation_write&&annotation_addr>=38400)stream_bad<=1;
   if(work_active&&object_write)begin
    if({1'b0,object_index}!=object_write_count||object_write_count>=2048||object_x0>=object_x1||object_x1>=256||object_y0>=object_y1||object_y1>150)stream_bad<=1;
    if(object_write_count<12'hfff)object_write_count<=object_write_count+1'b1;
   end
   // Full result takes precedence over a page read of the preceding snapshot.
   if(engine_result_valid&&work_active)begin
    work_active<=0;
    if(result_identity)begin
     pending_valid<=1;pending_ready<=!result_complete||engine_count==0;pending_usable<=result_complete;pending_snapshot<=1;pending_bank<=back_bank;
     pending_frame<=engine_result_frame;pending_tag<=engine_result_tag;
     pending_count<=engine_count;pending_page<=incoming_page;pending_page_count<=incoming_pages;
     pending_row_count<=result_complete?rows_for(engine_count,incoming_page):4'd0;
     pending_overflow<=engine_overflow;pending_reason<=engine_reason;
     page_read_index<=0;page_read_wait<=1;
     if(take_due_with_result)page_turn_due<=0;
     for(k=0;k<8;k=k+1)begin back_class[k]<=14;back_x0[k]<=0;back_x1[k]<=0;back_y0[k]<=0;back_y1[k]<=0;end
    end
   end
  end
 end
 // Both annotated snapshot planes are physical memories, never reset.
 reg overview0[0:38399],overview1[0:38399];reg plane0_q,plane1_q;
 wire in_overview=in_x>=752&&in_x<1008&&in_y>=64&&in_y<214;
 wire[11:0]overview_x=in_x-12'd752,overview_y=in_y-12'd64;
 wire part_in_roi=in_x>={2'b0,part_display_x}&&in_x<{2'b0,part_display_x}+12'd128&&in_y>={2'b0,part_display_y}&&in_y<{2'b0,part_display_y}+12'd128;
 wire[11:0]part_dx=in_x-{2'b0,part_display_x},part_dy=in_y-{2'b0,part_display_y};
 wire part_border=part_in_roi&&(part_dx==0||part_dx==127||part_dy==0||part_dy==127);
 wire[15:0]read_addr=cfg_app_mode?{2'b0,part_dy[6:0],part_dx[6:0]}:in_overview?{overview_y[7:0],overview_x[7:0]}:16'd0;
 wire[15:0]overview_write_addr=part_write?{2'b0,part_edge_addr}:(annotation_write?annotation_addr:plane_addr);
 wire overview_write_bit=part_write?part_edge_bit:(annotation_write?1'b1:plane_bit);
 wire overview_write_bank=part_write?part_write_bank:back_bank;
 always@(posedge clk)begin
  if(rst_n&&(part_write||(work_active&&!engine_abort&&(plane_write||annotation_write)))&&overview_write_addr<38400)begin
   if(overview_write_bank)overview1[overview_write_addr]<=overview_write_bit;else overview0[overview_write_addr]<=overview_write_bit;
  end
  plane0_q<=overview0[read_addr];plane1_q<=overview1[read_addr];
 end
 function[23:0]object_color;input[2:0]i;begin
  case(i)
   0:object_color=24'h00ffff;1:object_color=24'hff66ff;2:object_color=24'h66ff66;3:object_color=24'hffbb55;
   4:object_color=24'hffff66;5:object_color=24'h6699ff;6:object_color=24'hff6666;default:object_color=24'h66ffcc;
  endcase
 end endfunction
 function[5:0]class_glyph;input[4:0]c;input[3:0]slot;begin
  class_glyph=0;
  if(c<=9)begin if(slot==0)class_glyph={1'b0,c}+6'd1;end
  else case(c)
   10:if(slot==0)class_glyph=38;
   11:case(slot)0:class_glyph=39;1:class_glyph=40;endcase
   12:case(slot)0:class_glyph=41;1:class_glyph=42;endcase
   13:case(slot)0:class_glyph=43;1:class_glyph=42;endcase
   16:case(slot)0:class_glyph=50;1:class_glyph=51;endcase
   17:case(slot)0:class_glyph=1;1:class_glyph=52;2:class_glyph=38;3:class_glyph=49;endcase
   default:case(slot)0:class_glyph=34;1:class_glyph=35;endcase
  endcase
 end endfunction
 function[5:0]decimal_glyph;input[13:0]n;input[1:0]place;reg[13:0]v;begin
  case(place)0:v=n;1:v=n/10;2:v=n/100;default:v=n/1000;endcase
  decimal_glyph=(v%10)+1;
 end endfunction
 wire app_now=source_sof?cfg_app_mode:frame_configuration[15];
 wire panel=in_x>=752&&in_x<1008&&in_y>=40&&in_y<456;
 wire[31:0]snapshot_age=display_frame_id-preview_frame;
 wire[13:0]visible_age=snapshot_age>9999?14'd9999:snapshot_age[13:0];
 reg text_on,box_on;reg[5:0]glyph_code;reg[3:0]glyph_row,glyph_col;
 reg[23:0]mark_color;integer i;reg[11:0]tx,ty;reg[3:0]slot;
 reg number_on;reg[13:0]number_value;reg[1:0]number_place;
 reg[13:0]object_number;reg[2:0]object_digits;
 always@*begin
  text_on=0;box_on=0;glyph_code=0;glyph_row=0;glyph_col=0;mark_color=24'hffffff;tx=0;ty=0;slot=0;
  number_on=0;number_value=0;number_place=0;
  object_number=0;object_digits=1;
  if(in_x>=752&&in_x<1008&&in_y>=40&&in_y<56)begin
   text_on=1;tx=in_x-752;glyph_col=tx[3:0];glyph_row=in_y-40;slot=tx[7:4];
   case(slot)0:glyph_code=28;1:glyph_code=29;2:glyph_code=30;3:glyph_code=31;
    5:glyph_code=work_active||pending_valid?55:(preview_valid?0:32);
    6:glyph_code=work_active||pending_valid?56:(preview_valid?0:33);
    8:glyph_code=preview_overflow?36:0;9:glyph_code=preview_overflow?37:0;
   endcase
  end
  if(in_x>=752&&in_x<1008&&in_y>=216&&in_y<232)begin
   text_on=1;tx=in_x-752;glyph_col=tx[3:0];glyph_row=in_y-216;slot=tx[7:4];
   case(slot)0:glyph_code=46;1:glyph_code=47;2:glyph_code=48;11:glyph_code=52;15:glyph_code=53;default:glyph_code=0;endcase
   if(slot>=3&&slot<=6)begin number_on=1;number_value={2'd0,preview_count};number_place=6-slot;end
   if(slot>=8&&slot<=10)begin number_on=1;number_value=preview_page_count==0?14'd0:({6'd0,preview_page}+14'd1);number_place=10-slot;end
   if(slot>=12&&slot<=14)begin number_on=1;number_value={5'd0,preview_page_count};number_place=14-slot;end
  end
  for(i=0;i<8;i=i+1)begin
   if(preview_valid&&i<preview_row_count)begin
    object_number={3'd0,preview_page,3'b000}+i+1;
    object_digits=object_number>=1000?3'd4:(object_number>=100?3'd3:(object_number>=10?3'd2:3'd1));
    if(in_overview&&overview_x>=preview_x0[i]&&overview_x<preview_x1[i]&&overview_y>=preview_y0[i]&&overview_y<preview_y1[i]&&
      (overview_x==preview_x0[i]||overview_x+1'b1==preview_x1[i]||overview_y==preview_y0[i]||overview_y+1'b1==preview_y1[i]))begin
      box_on=1;mark_color=object_color(i);
    end
    // Every digit lies in font columns4..11. An8px advance keeps compact
    // variable-width global IDs readable without four leading-zero cells.
    if(in_overview&&overview_x>=preview_x0[i]&&overview_x<preview_x0[i]+{6'd0,object_digits,3'b000}&&overview_y>=preview_y0[i]&&overview_y<preview_y0[i]+16)begin
     text_on=1;tx=overview_x-preview_x0[i];glyph_col={1'b0,tx[2:0]}+4'd4;glyph_row=overview_y-preview_y0[i];mark_color=object_color(i);
     number_on=1;number_value=object_number;number_place=object_digits-3'd1-{1'b0,tx[4:3]};
    end
    if(in_x>=752&&in_x<1008&&in_y>=232+i*24&&in_y<248+i*24)begin
     text_on=1;tx=in_x-752;ty=in_y-(232+i*24);slot=tx[7:4];glyph_col=tx[3:0];glyph_row=ty[3:0];mark_color=object_color(i);
     number_on=slot<4;number_value={3'd0,preview_page,3'b000}+i+1;number_place=3-slot;
     if(slot==4)glyph_code=48;else if(slot>=5)glyph_code=class_glyph(preview_class[i],slot-5);else glyph_code=0;
    end
   end
  end
  if(preview_valid&&preview_count==0&&in_x>=752&&in_x<816&&in_y>=232&&in_y<248)begin
   text_on=1;tx=in_x-752;glyph_col=tx[3:0];glyph_row=in_y-232;
   case(tx[5:4])0:glyph_code=45;1:glyph_code=46;2:glyph_code=47;default:glyph_code=0;endcase
  end
  if(preview_valid&&in_x>=752&&in_x<1008&&in_y>=432&&in_y<448)begin
   text_on=1;tx=in_x-752;glyph_col=tx[3:0];glyph_row=in_y-432;slot=tx[7:4];
   case(slot)0:glyph_code=54;1:glyph_code=57;2:glyph_code=48;default:glyph_code=0;endcase
   if(slot>=3&&slot<=6)begin number_on=1;number_value=visible_age;number_place=6-slot;end
  end
  if(number_on)glyph_code=decimal_glyph(number_value,number_place);
 end
 wire[15:0]font_bits;
 assign font_address_o={glyph_code,glyph_row};
 generate if(EXTERNAL_FONT)begin:SharedFont assign font_bits=font_bits_i;end
 else begin:PrivateFont cn_font_rom u_font(.clk(clk),.glyph(glyph_code),.row(glyph_row),.bits(font_bits));end endgenerate
 wire part_title=in_x>=336&&in_x<400&&in_y>=40&&in_y<56;
 wire[11:0]part_title_x=in_x-12'd336,part_title_y=in_y-12'd40;
 wire[15:0]part_font_bits;
 canny_title_rom u_canny_title(.clk(clk),.glyph(part_title_x[5:4]),.row(part_title_y[3:0]),.bits(part_font_bits));
 reg p_part,p_part_roi,p_part_border,p_part_title;reg[3:0]p_part_col;
 reg p_hs,p_vs,p_de,p_sof,p_eof;reg[11:0]p_x,p_y;reg[23:0]p_rgb,p_color;
 reg p_panel,p_plane,p_valid,p_bank,p_text,p_box;reg[3:0]p_col;
 always@(posedge clk)begin
  if(!rst_n)begin
   p_hs<=0;p_vs<=0;p_de<=0;p_sof<=0;p_eof<=0;p_x<=0;p_y<=0;p_rgb<=0;p_color<=0;
   p_part<=0;p_part_roi<=0;p_part_border<=0;p_part_title<=0;p_part_col<=0;
   p_panel<=0;p_plane<=0;p_valid<=0;p_bank<=0;p_text<=0;p_box<=0;p_col<=0;
   out_hs<=0;out_vs<=0;out_de<=0;out_sof<=0;out_eof<=0;out_x<=0;out_y<=0;out_rgb<=0;
  end else begin
   p_hs<=in_hs;p_vs<=in_vs;p_de<=in_de;p_sof<=in_sof;p_eof<=in_eof;p_x<=in_x;p_y<=in_y;p_rgb<=in_rgb;
   p_part<=in_de&&cfg_app_mode&&part_display_valid;p_part_roi<=part_in_roi;p_part_border<=part_border;p_part_title<=part_title;p_part_col<=part_title_x[3:0];
   p_panel<=in_de&&!app_now&&panel;p_plane<=in_overview;p_valid<=cfg_app_mode?part_display_valid:preview_valid;p_bank<=cfg_app_mode?part_display_bank:front_bank;
   p_text<=text_on;p_box<=box_on;p_col<=glyph_col;p_color<=mark_color;
   out_hs<=p_hs;out_vs<=p_vs;out_de<=p_de;out_sof<=p_sof;out_eof<=p_eof;out_x<=p_x;out_y<=p_y;
   if(p_part&&part_display_valid&&p_valid)begin
    if(p_part_border||(p_part_roi&&(p_bank?plane1_q:plane0_q))||(p_part_title&&part_font_bits[15-p_part_col]))out_rgb<=24'h00ffff;
    else out_rgb<=p_rgb;
   end else if(p_panel)begin
    if(p_text&&font_bits[15-p_col])out_rgb<=p_color;
    else if(p_box)out_rgb<=p_color;
    else if(p_plane&&p_valid)out_rgb<=(p_bank?plane1_q:plane0_q)?24'hffffff:24'h101018;
    else out_rgb<=24'h101018;
   end else out_rgb<=p_rgb;
  end
 end
endmodule
