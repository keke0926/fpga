`timescale 1ns/1ps
// Local raw+feature reference transaction. A/B pointers are published together
// only after raw readback and full native feature generation have completed.
// Native registration records follow observation features; comparison remains a separate consumer.
module part_local_reference #(
 parameter integer WIDTH=1024,HEIGHT=600,FRAME_BYTES=WIDTH*HEIGHT
)(

 output wire region_start_o,region_ready_o,input wire region_idle_i,region_valid_i,region_done_i,
 input wire[8:0]region_x0_i,region_x1_i,input wire[7:0]region_y0_i,region_y1_i,
 input wire[11:0]region_count_i,input wire[3:0]region_error_i,

 output wire cmd_valid_o,input wire cmd_ready_i,output wire[7:0]cmd_opcode_o,cmd_flags_o,
 output wire[15:0]cmd_data_o,output wire[31:0]cmd_request_o,
 input wire response_valid_i,output wire response_ready_o,input wire[7:0]response_status_i,
 output wire result_context_o,output wire[63:0]diagnostic_o,
 input wire clk,rst_n,part_mode_i,available_i,
 input wire[31:0]config_epoch_i,
 input wire teach_i,confirm_i,detect_i,cancel_i,
 output reg snapshot_o,resume_o,
 input wire locked_i,restored_i,freeze_error_i,
 input wire[31:0]frame_i,crc_i,input wire[1:0]bank_i,
 output wire lease_req_o,input wire lease_grant_i,
 output wire[15:0]scratch_addr_o,output wire scratch_we_o,output wire[7:0]scratch_data_o,input wire[7:0]scratch_data_i,
 output wire[15:0]state_addr_o,output wire[1:0]state_we_o,state_data_o,input wire[1:0]state_data_i,
 output wire[11:0]queue_addr_o,output wire queue_we_o,output wire[15:0]queue_data_o,input wire[15:0]queue_data_i,
 output wire req_valid_o,input wire req_ready_i,output wire req_write_o,
 output wire[31:0]req_addr_o,output wire[127:0]req_data_o,
 input wire rsp_valid_i,output wire rsp_ready_o,input wire rsp_error_i,input wire[127:0]rsp_data_i,
 output wire active_o,output reg[3:0]status_o,
 output reg reference_valid_o,reference_slot_o,
 output reg[31:0]reference_id_o,reference_frame_o,reference_crc_o,task_id_o
);
 localparam IDLE=0,CONFIRM=1,FREEZE=2,LEASE=3,READ=4,READ_WAIT=5,
 WRITE=6,WRITE_WAIT=7,VERIFY=8,VERIFY_WAIT=9,NEXT=10,
 FEATURE_START=11,FEATURE_RUN=12,FEATURE_DRAIN=13,HELD=14,DRAIN=15,RESUME=16,RESTORE_WAIT=17,REG_START=18,REG_RUN=19,REG_DRAIN=20;
 reg[4:0]state;reg teaching,staging_slot,pending;
 reg[31:0]task_epoch,task_frame,task_crc;reg[1:0]task_bank;
 reg[19:0]offset;reg[127:0]word_data;
 wire invalid_task=!part_mode_i||cancel_i||
   (state!=IDLE&&state!=CONFIRM&&config_epoch_i!=task_epoch);
 wire registration_owner=state==REG_START||state==REG_RUN||state==REG_DRAIN||state==HELD;
 wire running=(state>=LEASE&&state<=HELD)||registration_owner;
 wire identity_lost=frame_i!=task_frame||crc_i!=task_crc||bank_i!=task_bank;
 wire cancel_now=invalid_task||(state!=IDLE&&state!=CONFIRM&&freeze_error_i&&!(state==FREEZE&&snapshot_o))||
   (((state>LEASE&&state<=HELD)||registration_owner)&&(!locked_i||!lease_grant_i||identity_lost));
 wire feature_owner=state==FEATURE_START||state==FEATURE_RUN||state==FEATURE_DRAIN;
 wire feature_busy,feature_done,feature_valid;wire[3:0]feature_error;
 wire f_req,f_write,f_rsp_ready;wire[31:0]f_addr;wire[127:0]f_data;
 wire r_ready,r_busy,r_valid,r_req,r_write,r_rsp_ready;wire[3:0]r_phase;
 wire[3:0]r_error;wire[9:0]r_count;wire[31:0]r_addr,r_cycles;wire[127:0]r_data;
 wire[15:0]f_sa,r_sa;wire[1:0]f_sw,r_sw,f_sd,r_sd;
 assign state_addr_o=registration_owner?r_sa:f_sa;assign state_we_o=registration_owner?r_sw:f_sw;assign state_data_o=registration_owner?r_sd:f_sd;
 assign result_context_o=registration_owner&&!cancel_now;
 wire[15:0]f_ga,r_ga;wire f_gw,r_gw;wire[7:0]f_gd,r_gd;
 wire[11:0]f_qa,r_qa;wire f_qw,r_qw;wire[15:0]f_qd,r_qd;
 assign scratch_addr_o=registration_owner?r_ga:f_ga;assign scratch_we_o=registration_owner?r_gw:f_gw;
 assign scratch_data_o=registration_owner?r_gd:f_gd;
 assign queue_addr_o=registration_owner?r_qa:f_qa;assign queue_we_o=registration_owner?r_qw:f_qw;
 assign queue_data_o=registration_owner?r_qd:f_qd;
 wire raw_req=(state==READ||state==WRITE||state==VERIFY)&&!pending&&!cancel_now&&lease_grant_i;
 assign active_o=state!=IDLE;
 assign lease_req_o=running||state==DRAIN;
 assign req_valid_o=registration_owner?r_req:feature_owner?f_req:raw_req;
 assign req_write_o=registration_owner?r_write:feature_owner?f_write:state==WRITE;
 assign req_addr_o=registration_owner?r_addr:feature_owner?f_addr:
   (state==READ?{8'd0,task_bank,22'd0}:(staging_slot?32'h00600000:32'h00500000))+{12'd0,offset};
 assign req_data_o=registration_owner?r_data:feature_owner?f_data:word_data;
 assign rsp_ready_o=registration_owner?r_rsp_ready:feature_owner?f_rsp_ready:pending;
 part_feature_frame #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) features(
  .clk(clk),.rst_n(rst_n),.start_i(state==FEATURE_START&&!cancel_now),
  .abort_i(cancel_now||state==FEATURE_DRAIN||!feature_owner),
  .raw_base_i(teaching?(staging_slot?32'h00600000:32'h00500000):{8'd0,task_bank,22'd0}),
  .feature_base_i(teaching?(staging_slot?32'h00900000:32'h00100000):32'h00d00000),
  .task_id_i(task_id_o),.frame_id_i(task_frame),.reference_id_i(reference_id_o),.config_epoch_i(task_epoch),
  .busy_o(feature_busy),.done_o(feature_done),.valid_o(feature_valid),.error_o(feature_error),
  .task_id_o(),.frame_id_o(),.reference_id_o(),.config_epoch_o(),
  .gray_addr_o(f_ga),.gray_we_o(f_gw),.gray_data_o(f_gd),.gray_data_i(scratch_data_i),
  .state_addr_o(f_sa),.state_we_o(f_sw),.state_data_o(f_sd),.state_data_i(state_data_i),
  .queue_addr_o(f_qa),.queue_we_o(f_qw),.queue_data_o(f_qd),.queue_data_i(queue_data_i),
  .req_valid_o(f_req),.req_ready_i(feature_owner&&req_ready_i),.req_write_o(f_write),.req_addr_o(f_addr),.req_data_o(f_data),
  .rsp_valid_i(feature_owner&&rsp_valid_i),.rsp_ready_o(f_rsp_ready),.rsp_error_i(rsp_error_i),.rsp_data_i(rsp_data_i));
 part_local_inspection registration(.diagnostic_o(diagnostic_o),.region_start_o(region_start_o),.region_ready_o(region_ready_o),.region_idle_i(region_idle_i),.region_valid_i(region_valid_i),.region_done_i(region_done_i),.region_x0_i(region_x0_i),.region_x1_i(region_x1_i),.region_y0_i(region_y0_i),.region_y1_i(region_y1_i),.region_count_i(region_count_i),.region_error_i(region_error_i),.cmd_valid_o(cmd_valid_o),.cmd_ready_i(cmd_ready_i),.cmd_opcode_o(cmd_opcode_o),.cmd_flags_o(cmd_flags_o),.cmd_data_o(cmd_data_o),.cmd_request_o(cmd_request_o),.response_valid_i(response_valid_i),.response_ready_o(response_ready_o),.response_status_i(response_status_i),.frame_crc_i(task_crc),.phase_o(r_phase),.state_addr_o(r_sa),.state_we_o(r_sw),.state_data_o(r_sd),.state_data_i(state_data_i),.clk_i(clk),.rst_i(!rst_n),
 .abort_i(cancel_now||state==REG_DRAIN||!registration_owner),.start_i(state==REG_START&&!cancel_now),
 .ready_o(r_ready),.busy_o(r_busy),.result_valid_o(r_valid),.out_ready_i(state==REG_RUN&&!cancel_now),
 .ref_base_i(reference_slot_o?32'h00900000:32'h00100000),.obs_base_i(32'h00d00000),
 .task_id_i(task_id_o),.frame_id_i(task_frame),.reference_id_i(reference_id_o),.config_epoch_i(task_epoch),
 .error_o(r_error),.count_o(r_count),
 .gray_addr_o(r_ga),.gray_we_o(r_gw),.gray_data_o(r_gd),.gray_data_i(scratch_data_i),
 .queue_addr_o(r_qa),.queue_we_o(r_qw),.queue_data_o(r_qd),.queue_data_i(queue_data_i),
 .req_valid_o(r_req),.req_ready_i(registration_owner&&req_ready_i),.req_write_o(r_write),.req_addr_o(r_addr),.req_data_o(r_data),
 .rsp_valid_i(registration_owner&&rsp_valid_i),.rsp_ready_o(r_rsp_ready),.rsp_error_i(rsp_error_i),.rsp_data_i(rsp_data_i));
 always@(posedge clk)begin
  if(!rst_n)begin
   state<=IDLE;teaching<=0;staging_slot<=0;pending<=0;task_epoch<=0;task_frame<=0;task_crc<=0;task_bank<=0;
   offset<=0;word_data<=0;snapshot_o<=0;resume_o<=0;status_o<=0;
   reference_valid_o<=0;reference_slot_o<=0;reference_id_o<=0;reference_frame_o<=0;reference_crc_o<=0;task_id_o<=0;
  end else begin
   snapshot_o<=0;resume_o<=0;
   if(!feature_owner&&!registration_owner&&raw_req&&req_ready_i)pending<=1;
   if(!feature_owner&&!registration_owner&&rsp_valid_i&&pending)pending<=0;
   if(state!=IDLE&&state!=DRAIN&&state!=FEATURE_DRAIN&&state!=REG_DRAIN&&state!=RESUME&&state!=RESTORE_WAIT&&cancel_now)begin
    if(state==CONFIRM)begin state<=IDLE;status_o<=0;end
    else begin
     state<=registration_owner?REG_DRAIN:(state==FEATURE_RUN&&feature_busy&&!feature_done)?FEATURE_DRAIN:DRAIN;
     status_o<=freeze_error_i?4'd9:4'd8;
    end
   end else case(state)
    IDLE:if(part_mode_i&&available_i&&(!freeze_error_i||restored_i))begin
     if(teach_i)begin state<=CONFIRM;status_o<=1;end
     else if(detect_i)begin
      if(!reference_valid_o)status_o<=2;
      else begin teaching<=0;task_epoch<=config_epoch_i;task_id_o<=task_id_o+1'b1;snapshot_o<=1;state<=FREEZE;status_o<=3;end
     end
    end
    CONFIRM:if(confirm_i&&available_i)begin
     teaching<=1;staging_slot<=reference_valid_o?!reference_slot_o:1'b0;
     task_epoch<=config_epoch_i;task_id_o<=task_id_o+1'b1;snapshot_o<=1;state<=FREEZE;status_o<=3;
    end
    FREEZE:if(locked_i)begin task_frame<=frame_i;task_crc<=crc_i;task_bank<=bank_i;offset<=0;state<=LEASE;end
    LEASE:if(lease_grant_i)begin state<=teaching?READ:FEATURE_START;status_o<=teaching?4'd4:4'd5;end
    READ:if(raw_req&&req_ready_i)state<=READ_WAIT;
    READ_WAIT:if(rsp_valid_i&&pending)begin
     if(rsp_error_i)begin state<=RESUME;status_o<=9;end
     else begin word_data<=rsp_data_i;state<=WRITE;end
    end
    WRITE:if(raw_req&&req_ready_i)state<=WRITE_WAIT;
    WRITE_WAIT:if(rsp_valid_i&&pending)begin
     if(rsp_error_i)begin state<=RESUME;status_o<=9;end else state<=VERIFY;
    end
    VERIFY:if(raw_req&&req_ready_i)state<=VERIFY_WAIT;
    VERIFY_WAIT:if(rsp_valid_i&&pending)begin
     if(rsp_error_i||rsp_data_i!=word_data)begin state<=RESUME;status_o<=9;end else state<=NEXT;
    end
    NEXT:if(offset==FRAME_BYTES-16)begin status_o<=5;state<=FEATURE_START;end
     else begin offset<=offset+20'd16;state<=READ;end
    FEATURE_START:state<=FEATURE_RUN;
    FEATURE_RUN:if(feature_done)begin
     if(feature_valid&&feature_error==0)begin
      if(teaching)begin
       reference_valid_o<=1;reference_slot_o<=staging_slot;reference_id_o<=reference_id_o+1'b1;
       reference_frame_o<=task_frame;reference_crc_o<=task_crc;status_o<=7;state<=RESUME;
      end else begin status_o<=10;state<=REG_START;end
     end else begin status_o<=feature_error==3?4'd8:4'd9;state<=RESUME;end
    end
    FEATURE_DRAIN:if(feature_done||!feature_busy)state<=RESUME;
    REG_START:if(r_ready)state<=REG_RUN;
    REG_RUN:begin status_o<=r_phase;if(r_valid)begin
     if(r_error==0)begin status_o<=r_phase;state<=HELD;end
     else if(r_error==5||r_error==6)begin status_o<=11;state<=HELD;end
     else begin status_o<=9;state<=RESUME;end
    end
    end
    REG_DRAIN:if(!r_busy)state<=RESUME;
    HELD:if(confirm_i)begin state<=RESUME;status_o<=0;end
    DRAIN:if(!pending||rsp_valid_i)state<=RESUME;
    RESUME:begin
     if(restored_i||freeze_error_i)begin state<=IDLE;if(freeze_error_i)status_o<=9;end
     else begin resume_o<=1;state<=RESTORE_WAIT;end
    end
    RESTORE_WAIT:if(restored_i||freeze_error_i)begin state<=IDLE;if(freeze_error_i)status_o<=9;end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
