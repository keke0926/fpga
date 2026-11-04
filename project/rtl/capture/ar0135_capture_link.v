`timescale 1ns/1ps
module ar0135_capture_link #(parameter BAUD=460800, parameter integer KEY_DEBOUNCE_CYCLES=480000, parameter ENABLE_FPS=0,parameter ENABLE_LOCAL=0,parameter LOCAL_HOLD_CYCLES=96000000)(


 output wire local_region_start_o,local_region_ready_o,input wire local_region_idle_i,local_region_valid_i,local_region_done_i,
 input wire[8:0]local_region_x0_i,local_region_x1_i,input wire[7:0]local_region_y0_i,local_region_y1_i,
 input wire[11:0]local_region_count_i,input wire[3:0]local_region_error_i,
 output wire local_active_o,output wire[3:0]local_status_o,
 output wire local_reference_valid_o,
 output wire local_lease_req_o,input wire local_lease_grant_i,
 output wire[15:0]local_scratch_addr_o,output wire local_scratch_we_o,output wire[7:0]local_scratch_data_o,input wire[7:0]local_scratch_data_i,
 output wire[15:0]local_state_addr_o,output wire[1:0]local_state_we_o,local_state_data_o,input wire[1:0]local_state_data_i,
 output wire[11:0]local_queue_addr_o,output wire local_queue_we_o,output wire[15:0]local_queue_data_o,input wire[15:0]local_queue_data_i,
 output wire analysis_req_valid_o,input wire analysis_req_ready_i,output wire analysis_req_write_o,
 output wire[31:0]analysis_req_addr_o,output wire[127:0]analysis_req_data_o,
 input wire analysis_rsp_valid_i,output wire analysis_rsp_ready_o,input wire analysis_rsp_error_i,input wire[127:0]analysis_rsp_data_i,
 output wire held_axi_o,output wire[1:0]held_bank_axi_o,
 input wire clk_cmos_i,rst_cmos_n_i,src_vs_i,src_de_i,input wire[7:0] src_gray_i,
 input wire clk_axi_i,rst_axi_n_i,ddr_ready_i,awvalid_i,awready_i,arvalid_i,arready_i,
 input wire[31:0] raw_araddr_i,
 input wire cmd_valid_i,cmd_ready_i,cmd_write_i,input wire[31:0] cmd_addr_i,input wire[7:0] cmd_len_i,
 input wire wvalid_i,wready_i,wlast_i,input wire[127:0] wdata_i,input wire[15:0] wstrb_i,
 input wire bvalid_i,bready_i,input wire[1:0] bresp_i,input wire[3:0] bid_i,
 input wire rvalid_i,rready_i,rlast_i,input wire[1:0] rresp_i,input wire[3:0] rid_i,
 input wire[1:0] writer_state_i,
 input wire clk_lcd_i,rst_lcd_n_i,lcd_vs_i,lcd_de_i,input wire[7:0] lcd_gray_i,
 input wire[6:0] fps_axi_observation_i,input wire fps_lcd_underflow_i,output wire[11:0] fps_bcd_o,
 input wire[4:0] keys_n_i,input wire key_detect_n_i,output wire runtime_app_mode_o,
 input wire assist_sof_i,assist_error_i,
 output wire assist_valid_o,output wire[319:0] assist_boxes_o,output wire[3:0] assist_rows_o,
 output wire[9:0] assist_count_o,output wire[5:0] assist_page_o,output wire[6:0] assist_pages_o,
 input wire[3:0] canny_state_i,canny_error_i,
 input wire[14:0] canny_samples_i,canny_edges_i,
 input wire[31:0] canny_cycles_i,canny_display_commits_i,
 output wire source_locked_o,output wire[31:0] source_frame_o,source_crc_o,
 output wire canny_enable_o,output wire[9:0] canny_x_o,canny_y_o,
 output wire[7:0] canny_low_o,canny_high_o,
 output wire[31:0] canny_frame_o,canny_crc_o,canny_reference_o,canny_tag_o,
 input wire uart_rx_i,output wire uart_tx_o,
 output wire writer_vs_o,writer_de_o,output wire[31:0] capture_araddr_o,
 output wire runtime_auto_enable_o,output wire runtime_median_enable_o,output wire[1:0] runtime_view_mode_o,output wire[10:0] runtime_threshold_o);
 wire lr;
 reset_sync_n link_reset(.clk(clk_lcd_i),.arst_n(rst_cmos_n_i&&rst_axi_n_i&&rst_lcd_n_i),.rst_n(lr));
 wire[4:0] key_press;wire detect_pressed;
 button_debounce #(.DEBOUNCE_CYCLES(KEY_DEBOUNCE_CYCLES)) u_key_app(.clk(clk_lcd_i),.rst_n(lr),.button_n(keys_n_i[0]),.press_pulse(key_press[0]));
 button_debounce #(.DEBOUNCE_CYCLES(KEY_DEBOUNCE_CYCLES)) u_key_auto(.clk(clk_lcd_i),.rst_n(lr),.button_n(keys_n_i[1]),.press_pulse(key_press[1]));
 button_debounce #(.DEBOUNCE_CYCLES(KEY_DEBOUNCE_CYCLES)) u_key_view(.clk(clk_lcd_i),.rst_n(lr),.button_n(keys_n_i[2]),.press_pulse(key_press[2]));
 button_debounce #(.DEBOUNCE_CYCLES(KEY_DEBOUNCE_CYCLES)) u_key_plus(.clk(clk_lcd_i),.rst_n(lr),.button_n(keys_n_i[3]),.press_pulse(key_press[3]));
 button_debounce #(.DEBOUNCE_CYCLES(KEY_DEBOUNCE_CYCLES)) u_key_minus(.clk(clk_lcd_i),.rst_n(lr),.button_n(keys_n_i[4]),.press_pulse(key_press[4]));
 button_debounce #(.DEBOUNCE_CYCLES(KEY_DEBOUNCE_CYCLES),.RESET_PRESSED(1)) u_key_detect(.clk(clk_lcd_i),.rst_n(lr),.button_n(key_detect_n_i),.pressed_o(detect_pressed),.press_pulse());
 wire rx_valid,framing_error;wire[7:0]rx_byte;
 wire snapshot_req,resume_req,locked,restored,freezer_error;
 wire[31:0] locked_frame,locked_crc,freezer_code;
 assign source_locked_o=locked;assign source_frame_o=locked_frame;assign source_crc_o=locked_crc;

 // BEGIN RUNTIME VIDEO CONTROL: existing VCT1 plus physical-key events.
 wire runtime_uart_tx_valid,runtime_uart_tx_ready,runtime_uart_tx_idle,runtime_tx_active;
 wire physical_tx_ready,physical_tx_idle;
 assign runtime_uart_tx_ready=physical_tx_ready;
 assign runtime_uart_tx_idle=physical_tx_idle;
 wire[7:0]runtime_uart_tx_byte;
 wire freeze_busy,freeze_ready;
 wire runtime_command_valid,runtime_command_ready,runtime_response_valid,runtime_response_ready;
 wire[7:0] runtime_opcode,runtime_flags,runtime_status,runtime_response_opcode,runtime_committed_flags,runtime_pending_flags;
 wire[15:0] runtime_command_threshold,runtime_committed_threshold,runtime_pending_threshold;
 wire[31:0] runtime_request,runtime_response_request,runtime_commit_counter;
 wire[7:0]runtime_rx_opcode;
 runtime_control_rx runtime_demux(.clk(clk_lcd_i),.rst_n(lr),.rx_valid_i(rx_valid),.rx_byte_i(rx_byte),
  .framing_error_i(framing_error),.cmd_valid_o(runtime_command_valid),
  .cmd_ready_i((local_active_o?(runtime_rx_opcode==8'h53&&freeze_ready):runtime_command_ready)&&!runtime_tx_active&&!runtime_response_valid),
  .pending_opcode_o(runtime_rx_opcode),.cmd_opcode_o(runtime_opcode),
  .cmd_flags_o(runtime_flags),.cmd_threshold_o(runtime_command_threshold),.cmd_request_id_o(runtime_request));
 // Conservative ready gate: demux checks ready before publishing its new opcode.
 wire part_command=runtime_opcode>=8'h30&&runtime_opcode<=8'h35;
 wire video_command_ready,part_command_ready,video_response_valid,part_response_valid;
 wire video_response_ready,part_response_ready;
 wire[7:0] video_status,part_status,video_response_opcode,part_response_opcode;
 wire[7:0] video_committed_flags,part_committed_flags,video_pending_flags,part_pending_flags;
 wire[15:0] video_committed_threshold,part_committed_threshold,video_pending_threshold,part_pending_threshold;
 wire[31:0] video_response_request,part_response_request,video_commit_counter,part_commit_counter;
 wire canny_command=runtime_opcode>=8'h40&&runtime_opcode<=8'h44;
 wire canny_command_ready,canny_response_valid,canny_response_ready;
 wire[7:0] canny_status,canny_response_opcode,canny_committed_flags,canny_pending_flags;
 wire[15:0] canny_data_low,canny_data_high;
 wire[31:0] canny_response_request,canny_response_counter;
 wire freeze_command=runtime_opcode>=8'h50&&runtime_opcode<=8'h53;
 wire local_query=runtime_opcode==8'h53;
 wire[63:0]local_diagnostic;
 wire freeze_response_valid,freeze_response_ready;
 wire[7:0]freeze_status,freeze_opcode,freeze_flags,freeze_reference;
 wire[15:0]freeze_crc_low,freeze_crc_high;
 wire[31:0]freeze_request,freeze_frame;
 frame_freeze_control freeze_control(.clk(clk_lcd_i),.rst_n(lr),.part_mode_i(runtime_app_mode_o),
  .locked_i(locked&&!local_active_o),.restored_i(restored),.freezer_error_i(freezer_error&&!local_active_o),
  .reference_valid_i(local_reference_valid_o),.frame_i(locked_frame),.crc_i(locked_crc),
  .local_diagnostic_i(local_diagnostic),.local_status_i(local_status_o),.local_active_i(local_active_o),.local_locked_i(locked),
  .cmd_valid_i(runtime_command_valid&&freeze_command&&(!local_active_o||local_query)),.cmd_ready_o(freeze_ready),
  .cmd_opcode_i(runtime_opcode),.cmd_flags_i(runtime_flags),.cmd_data_i(runtime_command_threshold),.cmd_request_i(runtime_request),
  .response_valid_o(freeze_response_valid),.response_ready_i(freeze_response_ready),.response_status_o(freeze_status),
  .response_opcode_o(freeze_opcode),.response_flags_o(freeze_flags),.response_reference_o(freeze_reference),
  .response_crc_low_o(freeze_crc_low),.response_crc_high_o(freeze_crc_high),
  .response_request_o(freeze_request),.response_frame_o(freeze_frame),
  .snapshot_o(snapshot_req),.resume_o(resume_req),.busy_o(freeze_busy));
 assign runtime_command_ready=local_query?freeze_ready:!local_active_o&&video_command_ready&&part_command_ready&&canny_command_ready&&freeze_ready;
 assign runtime_response_valid=video_response_valid||part_response_valid||canny_response_valid||freeze_response_valid;
 assign freeze_response_ready=runtime_response_ready&&freeze_response_valid;
 assign canny_response_ready=runtime_response_ready&&!freeze_response_valid&&canny_response_valid;
 assign part_response_ready=runtime_response_ready&&!freeze_response_valid&&!canny_response_valid&&part_response_valid;
 assign video_response_ready=runtime_response_ready&&!freeze_response_valid&&!canny_response_valid&&!part_response_valid;
 assign runtime_status=freeze_response_valid?freeze_status:canny_response_valid?canny_status:part_response_valid?part_status:video_status;
 assign runtime_response_opcode=freeze_response_valid?freeze_opcode:canny_response_valid?canny_response_opcode:part_response_valid?part_response_opcode:video_response_opcode;
 assign runtime_committed_flags=freeze_response_valid?freeze_flags:canny_response_valid?canny_committed_flags:part_response_valid?part_committed_flags:video_committed_flags;
 assign runtime_pending_flags=freeze_response_valid?freeze_reference:canny_response_valid?canny_pending_flags:part_response_valid?part_pending_flags:video_pending_flags;
 assign runtime_committed_threshold=freeze_response_valid?freeze_crc_low:canny_response_valid?canny_data_low:part_response_valid?part_committed_threshold:video_committed_threshold;
 assign runtime_pending_threshold=freeze_response_valid?freeze_crc_high:canny_response_valid?canny_data_high:part_response_valid?part_pending_threshold:video_pending_threshold;
 assign runtime_response_request=freeze_response_valid?freeze_request:canny_response_valid?canny_response_request:part_response_valid?part_response_request:video_response_request;
 assign runtime_commit_counter=freeze_response_valid?freeze_frame:canny_response_valid?canny_response_counter:part_response_valid?part_commit_counter:video_commit_counter;
 wire canny_invalidate=runtime_command_valid&&part_command_ready&&runtime_request!=0&&
   (runtime_opcode==8'h34||(runtime_opcode==8'h30&&runtime_flags==0&&runtime_command_threshold<=512));
 canny_roi_control canny_control(.clk(clk_lcd_i),.rst_n(lr),.part_mode_i(runtime_app_mode_o),
  .locked_i(locked&&!local_active_o),.video_error_i(assist_error_i),.locked_frame_i(locked_frame),.locked_crc_i(locked_crc),
  .frontend_i({runtime_app_mode_o,runtime_auto_enable_o,runtime_median_enable_o,runtime_view_mode_o,runtime_threshold_o}),
  .invalidate_i(canny_invalidate),
  .cmd_valid_i(runtime_command_valid&&canny_command),.cmd_ready_o(canny_command_ready),.cmd_opcode_i(runtime_opcode),
  .cmd_flags_i(runtime_flags),.cmd_data_i(runtime_command_threshold),.cmd_request_i(runtime_request),
  .response_valid_o(canny_response_valid),.response_ready_i(canny_response_ready),.response_status_o(canny_status),
  .response_opcode_o(canny_response_opcode),.response_committed_flags_o(canny_committed_flags),.response_pending_flags_o(canny_pending_flags),
  .response_data_low_o(canny_data_low),.response_data_high_o(canny_data_high),.response_request_o(canny_response_request),.response_counter_o(canny_response_counter),
  .vision_state_i(canny_state_i),.vision_error_i(canny_error_i),.sample_count_i(canny_samples_i),.edge_count_i(canny_edges_i),
  .cycles_i(canny_cycles_i),.display_commits_i(canny_display_commits_i),
  .enable_o(canny_enable_o),.roi_x_o(canny_x_o),.roi_y_o(canny_y_o),.low_o(canny_low_o),.high_o(canny_high_o),
  .frame_o(canny_frame_o),.crc_o(canny_crc_o),.reference_o(canny_reference_o),.tag_o(canny_tag_o));
 wire[31:0]local_config_epoch;
 runtime_video_control_keys #(.ENABLE_LOCAL_CANCEL(ENABLE_LOCAL)) runtime_control(
 .local_mode_cancel_i(local_active_o&&key_press[0]),.configuration_epoch_o(local_config_epoch),
 .key_press_i(local_active_o?5'd0:key_press),.app_mode_o(runtime_app_mode_o),.clk(clk_lcd_i),.rst_n(lr),.lcd_vs_i(lcd_vs_i),.capture_busy_i(freeze_busy||local_active_o),
  .cmd_valid_i(runtime_command_valid&&!part_command&&!canny_command&&!freeze_command),.cmd_ready_o(video_command_ready),.cmd_opcode_i(runtime_opcode),
  .cmd_flags_i(runtime_flags),.cmd_threshold_i(runtime_command_threshold),.cmd_request_id_i(runtime_request),
  .response_valid_o(video_response_valid),.response_ready_i(video_response_ready),.response_status_o(video_status),
  .response_opcode_o(video_response_opcode),.response_committed_flags_o(video_committed_flags),
  .response_pending_flags_o(video_pending_flags),.response_committed_threshold_o(video_committed_threshold),
  .response_pending_threshold_o(video_pending_threshold),.response_request_id_o(video_response_request),
  .response_commit_counter_o(video_commit_counter),.auto_enable_o(runtime_auto_enable_o),.median_enable_o(runtime_median_enable_o),
  .view_mode_o(runtime_view_mode_o),.threshold_o(runtime_threshold_o));
 wire local_result_context,local_cmd_valid,local_cmd_ready,local_response_valid,local_response_ready;
 wire[7:0]local_cmd_opcode,local_cmd_flags;wire[15:0]local_cmd_data;wire[31:0]local_cmd_request;
 wire part_store_ready,part_store_response;
 assign local_cmd_ready=local_active_o&&part_store_ready;
 assign part_command_ready=!local_active_o&&part_store_ready;
 assign local_response_valid=local_active_o&&part_store_response;
 assign part_response_valid=!local_active_o&&part_store_response;
 part_result_control part_results(.clk(clk_lcd_i),.rst_n(lr),.sof_i(assist_sof_i),
  .part_mode_i(runtime_app_mode_o),.locked_i(locked&&(!local_active_o||local_result_context)),.video_error_i(assist_error_i),
  .locked_frame_i(locked_frame),.locked_crc_i(locked_crc),
  .cmd_valid_i(local_active_o?local_cmd_valid:runtime_command_valid&&part_command),.cmd_ready_o(part_store_ready),
  .cmd_opcode_i(local_active_o?local_cmd_opcode:runtime_opcode),.cmd_flags_i(local_active_o?local_cmd_flags:runtime_flags),.cmd_data_i(local_active_o?local_cmd_data:runtime_command_threshold),.cmd_request_i(local_active_o?local_cmd_request:runtime_request),
  .response_valid_o(part_store_response),.response_ready_i(local_active_o?local_response_ready:part_response_ready),.response_status_o(part_status),
  .response_opcode_o(part_response_opcode),.response_committed_flags_o(part_committed_flags),.response_pending_flags_o(part_pending_flags),
  .response_committed_count_o(part_committed_threshold),.response_pending_count_o(part_pending_threshold),
  .response_request_o(part_response_request),.response_counter_o(part_commit_counter),
  .display_valid_o(assist_valid_o),.display_boxes_o(assist_boxes_o),.display_rows_o(assist_rows_o),
  .display_count_o(assist_count_o),.display_page_o(assist_page_o),.display_pages_o(assist_pages_o),.display_reference_o());
 runtime_uart_response runtime_response(.clk(clk_lcd_i),.rst_n(lr),.capture_busy_i(1'b0),
  .capture_tx_valid_i(1'b0),.capture_tx_byte_i(8'd0),.capture_tx_ready_o(),.capture_tx_idle_o(),
  .uart_tx_valid_o(runtime_uart_tx_valid),.uart_tx_byte_o(runtime_uart_tx_byte),
  .uart_tx_ready_i(runtime_uart_tx_ready),.uart_tx_idle_i(runtime_uart_tx_idle),.runtime_active_o(runtime_tx_active),
  .response_valid_i(runtime_response_valid),.response_ready_o(runtime_response_ready),.response_status_i(runtime_status),
  .response_opcode_i(runtime_response_opcode),.response_committed_flags_i(runtime_committed_flags),
  .response_pending_flags_i(runtime_pending_flags),.response_committed_threshold_i(runtime_committed_threshold),
  .response_pending_threshold_i(runtime_pending_threshold),.response_request_id_i(runtime_response_request),
  .response_commit_counter_i(runtime_commit_counter));
 // END RUNTIME VIDEO CONTROL V1
 capture_uart_rx #(.BAUD(BAUD)) rx(.clk(clk_lcd_i),.rst_n(lr),.rx_i(uart_rx_i),
  .byte_valid_o(rx_valid),.byte_o(rx_byte),.framing_error_o(framing_error));
 capture_uart_tx #(.BAUD(BAUD)) tx(.clk(clk_lcd_i),.rst_n(lr),.byte_valid_i(runtime_uart_tx_valid),
  .byte_i(runtime_uart_tx_byte),.byte_ready_o(physical_tx_ready),.tx_o(uart_tx_o),.idle_o(physical_tx_idle));
 wire local_snapshot,local_resume;wire[1:0]locked_bank;
 generate if(ENABLE_LOCAL)begin:Gen_Local
  wire teach,action;wire ar;
  wire available=!freeze_busy&&!locked&&!canny_enable_o&&
      !runtime_tx_active&&!runtime_response_valid&&!runtime_command_valid;
  wire[1:0]key_context=!runtime_app_mode_o ? 2'd3 :
      !local_active_o ? (available ? 2'd0 : 2'd3) :
      local_status_o==4'd1 ? (available ? 2'd1 : 2'd3) :
      (local_status_o==4'd6||local_status_o==4'd11||local_status_o==4'd13||local_status_o==4'd14) ? 2'd2 : 2'd3;
  wire req_valid,req_ready,req_write,rsp_valid,rsp_ready,rsp_error;
  wire[31:0]req_addr;wire[127:0]req_data,rsp_data;
  reset_sync_n local_axi_reset(.clk(clk_axi_i),.arst_n(rst_cmos_n_i&&rst_axi_n_i&&rst_lcd_n_i),.rst_n(ar));
  part_local_keys #(.HOLD_CYCLES(LOCAL_HOLD_CYCLES)) keys(.clk(clk_lcd_i),.rst_n(lr),
   .context_i(key_context),.cancel_i(key_press[0]),.pressed_i(detect_pressed),.teach_o(teach),.action_o(action));
  part_local_reference local_reference(.diagnostic_o(local_diagnostic),.region_start_o(local_region_start_o),.region_ready_o(local_region_ready_o),.region_idle_i(local_region_idle_i),.region_valid_i(local_region_valid_i),.region_done_i(local_region_done_i),.region_x0_i(local_region_x0_i),.region_x1_i(local_region_x1_i),.region_y0_i(local_region_y0_i),.region_y1_i(local_region_y1_i),.region_count_i(local_region_count_i),.region_error_i(local_region_error_i),.cmd_valid_o(local_cmd_valid),.cmd_ready_i(local_cmd_ready),.cmd_opcode_o(local_cmd_opcode),.cmd_flags_o(local_cmd_flags),.cmd_data_o(local_cmd_data),.cmd_request_o(local_cmd_request),
 .response_valid_i(local_response_valid),.response_ready_o(local_response_ready),.response_status_i(part_status),.result_context_o(local_result_context),.clk(clk_lcd_i),.rst_n(lr),.part_mode_i(runtime_app_mode_o),
   .config_epoch_i(local_config_epoch),.teach_i(teach),.detect_i(action),.confirm_i(action),.cancel_i(key_press[0]),
   .available_i(available),
   .snapshot_o(local_snapshot),.resume_o(local_resume),.locked_i(locked),.restored_i(restored),
   .freeze_error_i(freezer_error),.frame_i(locked_frame),.crc_i(locked_crc),.bank_i(locked_bank),
   .lease_req_o(local_lease_req_o),.lease_grant_i(local_lease_grant_i),
   .scratch_addr_o(local_scratch_addr_o),.scratch_we_o(local_scratch_we_o),.scratch_data_o(local_scratch_data_o),.scratch_data_i(local_scratch_data_i),
   .state_addr_o(local_state_addr_o),.state_we_o(local_state_we_o),.state_data_o(local_state_data_o),.state_data_i(local_state_data_i),
   .queue_addr_o(local_queue_addr_o),.queue_we_o(local_queue_we_o),.queue_data_o(local_queue_data_o),.queue_data_i(local_queue_data_i),
   .req_valid_o(req_valid),.req_ready_i(req_ready),.req_write_o(req_write),.req_addr_o(req_addr),.req_data_o(req_data),
   .rsp_valid_i(rsp_valid),.rsp_ready_o(rsp_ready),.rsp_error_i(rsp_error),.rsp_data_i(rsp_data),
   .active_o(local_active_o),.status_o(local_status_o),.reference_valid_o(local_reference_valid_o),
   .reference_slot_o(),.reference_id_o(),.reference_frame_o(),.reference_crc_o(),.task_id_o());
  part_ddr_mailbox mailbox(.s_clk(clk_lcd_i),.s_rst_n(lr),.d_clk(clk_axi_i),.d_rst_n(ar),
   .s_req_valid(req_valid),.s_req_ready(req_ready),.s_req_write(req_write),.s_req_addr(req_addr),.s_req_data(req_data),
   .s_rsp_valid(rsp_valid),.s_rsp_ready(rsp_ready),.s_rsp_error(rsp_error),.s_rsp_data(rsp_data),
   .d_req_valid(analysis_req_valid_o),.d_req_ready(analysis_req_ready_i),.d_req_write(analysis_req_write_o),
   .d_req_addr(analysis_req_addr_o),.d_req_data(analysis_req_data_o),
   .d_rsp_valid(analysis_rsp_valid_i),.d_rsp_ready(analysis_rsp_ready_o),.d_rsp_error(analysis_rsp_error_i),.d_rsp_data(analysis_rsp_data_i));
 end else begin:Gen_No_Local
  assign local_region_start_o=0;assign local_region_ready_o=0;
  assign local_cmd_valid=0;assign local_cmd_opcode=0;assign local_cmd_flags=0;assign local_cmd_data=0;assign local_cmd_request=0;assign local_response_ready=0;assign local_result_context=0;
  assign local_active_o=0;assign local_status_o=0;assign local_reference_valid_o=0;assign local_diagnostic=64'd0;
  assign local_lease_req_o=0;assign local_scratch_addr_o=0;assign local_scratch_we_o=0;assign local_scratch_data_o=0;
  assign local_state_addr_o=0;assign local_state_we_o=0;assign local_state_data_o=0;
  assign local_queue_addr_o=0;assign local_queue_we_o=0;assign local_queue_data_o=0;
  assign analysis_req_valid_o=0;assign analysis_req_write_o=0;assign analysis_req_addr_o=0;assign analysis_req_data_o=0;
  assign analysis_rsp_ready_o=0;assign local_snapshot=0;assign local_resume=0;
 end endgenerate
 capture_frame_freezer #(.ENABLE_FPS(ENABLE_FPS)) freezer(
  .clk_cmos_i(clk_cmos_i),.rst_cmos_n_i(rst_cmos_n_i),.src_vs_i(src_vs_i),.src_de_i(src_de_i),.src_gray_i(src_gray_i),
  .clk_axi_i(clk_axi_i),.rst_axi_n_i(rst_axi_n_i),.ddr_ready_i(ddr_ready_i),
  .awvalid_i(awvalid_i),.awready_i(awready_i),.arvalid_i(arvalid_i),.arready_i(arready_i),.raw_araddr_i(raw_araddr_i),
  .cmd_valid_i(cmd_valid_i),.cmd_ready_i(cmd_ready_i),.cmd_write_i(cmd_write_i),.cmd_addr_i(cmd_addr_i),.cmd_len_i(cmd_len_i),
  .wvalid_i(wvalid_i),.wready_i(wready_i),.wlast_i(wlast_i),.wdata_i(wdata_i),.wstrb_i(wstrb_i),
  .bvalid_i(bvalid_i),.bready_i(bready_i),.bresp_i(bresp_i),.bid_i(bid_i),
  .rvalid_i(rvalid_i),.rready_i(rready_i),.rlast_i(rlast_i),.rresp_i(rresp_i),.rid_i(rid_i),.writer_state_i(writer_state_i),
  .clk_lcd_i(clk_lcd_i),.rst_lcd_n_i(rst_lcd_n_i),.lcd_vs_i(lcd_vs_i),.lcd_de_i(lcd_de_i),.lcd_gray_i(lcd_gray_i),
  .snapshot_req_i(local_active_o?local_snapshot:snapshot_req),.resume_req_i(local_active_o?local_resume:resume_req),.writer_vs_o(writer_vs_o),.writer_de_o(writer_de_o),
  .fps_axi_observation_i(fps_axi_observation_i),.fps_lcd_underflow_i(fps_lcd_underflow_i),.fps_bcd_o(fps_bcd_o),
  .held_axi_o(held_axi_o),.locked_bank_axi_o(held_bank_axi_o),.locked_bank_o(locked_bank),
  .capture_araddr_o(capture_araddr_o),.locked_o(locked),.locked_frame_id_o(locked_frame),.locked_crc32_o(locked_crc),
  .restored_o(restored),.error_o(freezer_error),.error_code_o(freezer_code));
endmodule
