`timescale 1ns/1ps
// Observes REAL DDR transactions; never uses the vendor controller's fake B.
// Bundled CDC fields remain stable until the corresponding toggle is received.
module capture_frame_freezer #(
 parameter WIDTH=1024,HEIGHT=600,BURST_BEATS=128,WATCHDOG_CYCLES=288000000,ENABLE_FPS=0)(
 input wire clk_cmos_i,rst_cmos_n_i,src_vs_i,src_de_i,input wire[7:0] src_gray_i,
 input wire clk_axi_i,rst_axi_n_i,ddr_ready_i,awvalid_i,awready_i,arvalid_i,arready_i,
 input wire[31:0] raw_araddr_i,
 input wire cmd_valid_i,cmd_ready_i,cmd_write_i,input wire[31:0] cmd_addr_i,input wire[7:0] cmd_len_i,
 input wire wvalid_i,wready_i,wlast_i,input wire[127:0] wdata_i,input wire[15:0] wstrb_i,
 input wire bvalid_i,bready_i,input wire[1:0] bresp_i,input wire[3:0] bid_i,
 input wire rvalid_i,rready_i,rlast_i,input wire[1:0] rresp_i,input wire[3:0] rid_i,
 input wire[1:0] writer_state_i,
 input wire clk_lcd_i,rst_lcd_n_i,lcd_vs_i,lcd_de_i,input wire[7:0] lcd_gray_i,
 input wire snapshot_req_i,resume_req_i,
 input wire[6:0] fps_axi_observation_i,input wire fps_lcd_underflow_i,
 output wire[11:0] fps_bcd_o,
 output wire writer_vs_o,writer_de_o,output wire[31:0] capture_araddr_o,
 output wire held_axi_o,output wire[1:0]locked_bank_axi_o,output reg[1:0]locked_bank_o,
 output reg locked_o,output reg[31:0] locked_frame_id_o,locked_crc32_o,
 output reg restored_o,error_o,output reg[31:0] error_code_o);
 localparam WORDS=WIDTH*HEIGHT/16,BURSTS=WORDS/BURST_BEATS;
 localparam IDLE=0,STOP_ACK=1,PREP_DRAIN=2,SHOT_ACK=3,SHOT_DRAIN=4,
   PIN=5,VERIFY=6,LOCKED=7,REC_ACK=8,REC_DRAIN=9,REC_SOURCE=10,
   REC_WRITE=11,REC_VERIFY=12,FAILED=13;
 wire common_rst=rst_cmos_n_i&&rst_axi_n_i&&rst_lcd_n_i;
 wire cr,ar,lr;
 reset_sync_n c_reset(.clk(clk_cmos_i),.arst_n(common_rst),.rst_n(cr));
 reset_sync_n a_reset(.clk(clk_axi_i),.arst_n(common_rst),.rst_n(ar));
 reset_sync_n l_reset(.clk(clk_lcd_i),.arst_n(common_rst),.rst_n(lr));
 generate if(ENABLE_FPS)begin:Gen_FPS
  effective_display_fps #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) u_effective_fps(
   .clk_cmos_i(clk_cmos_i),.rst_cmos_n_i(cr),.src_vs_i(writer_vs_o),.src_de_i(writer_de_o),
   .clk_axi_i(clk_axi_i),.rst_axi_n_i(ar),.ddr_ready_i(ddr_ready_i),.axi_observation_i(fps_axi_observation_i),
   .clk_lcd_i(clk_lcd_i),.rst_lcd_n_i(lr),.lcd_vs_i(lcd_vs_i),.lcd_de_i(lcd_de_i),
   .lcd_underflow_i(fps_lcd_underflow_i),.hold_begin_i(snapshot_req_i),.hold_end_i(restored_o),.fps_bcd_o(fps_bcd_o));
 end else begin:Gen_No_FPS
  assign fps_bcd_o=12'b0;
 end endgenerate
 reg gate_command;reg[1:0] gate_opcode;wire gate_ack,gate_held,gate_bad;
 wire[31:0] gate_tag,gate_crc;wire[19:0] gate_pixels;
 capture_source_gate #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) source_gate(
   .clk(clk_cmos_i),.rst_n(cr),.src_vs_i(src_vs_i),.src_de_i(src_de_i),.src_gray_i(src_gray_i),
   .cmd_toggle_i(gate_command),.cmd_opcode_i(gate_opcode),.ack_toggle_o(gate_ack),
   .writer_vs_o(writer_vs_o),.writer_de_o(writer_de_o),.held_o(gate_held),
   .source_tag_o(gate_tag),.pixel_count_o(gate_pixels),.source_crc32_o(gate_crc),.geometry_error_o(gate_bad));
 reg snap_toggle,resume_toggle;
 (* async_reg="true" *) reg[1:0] snap_sync,resume_sync,ack_sync,blank_sync,wvs_sync;
 reg snap_seen,resume_seen,wvs_d,blank_d;reg[4:0] blank_age;
 reg[3:0] state;reg[31:0] watchdog,failure,target_tag,target_crc;
 reg pin_bank;reg[1:0] bank;
 assign held_axi_o=state==LOCKED&&pin_bank;assign locked_bank_axi_o=bank;
 assign capture_araddr_o=pin_bank?{raw_araddr_i[31:24],bank,raw_araddr_i[21:0]}:raw_araddr_i;
 wire accepted=cmd_valid_i&&cmd_ready_i&&ddr_ready_i;
 wire write_cmd=accepted&&cmd_write_i,read_cmd=accepted&&!cmd_write_i;
 wire wbeat=wvalid_i&&wready_i,bbeat=bvalid_i&&bready_i;
 wire rbeat=rvalid_i&&rready_i,read_end=rbeat&&rlast_i;
 reg[15:0] write_outstanding,read_outstanding;
 reg[31:0] total_commands,total_words,shot_commands,shot_words,shot_responses;
 reg eof_held,bank_bound;
 wire drained=eof_held&&write_outstanding==0&&total_words==total_commands*BURST_BEATS&&
   !awvalid_i&&!awready_i&&!(cmd_valid_i&&cmd_write_i)&&!wvalid_i;
 // Only the beginning of natural VS blank is eligible: leave ample time before
 // the vendor's VS-rising FIFO reset. Never change a bank with upstream pending.
 wire read_barrier=blank_sync[1]&&blank_age<8&&read_outstanding==0&&
   !arvalid_i&&!arready_i&&!cmd_valid_i&&!rvalid_i;
 reg check_toggle,check_match;reg[31:0] check_crc;
 wire check_ack;wire check_bad;
 (* async_reg="true" *) reg[1:0] check_ack_sync;
 reg[1:0]event_bank;reg event_toggle,event_locked,event_restored;reg[31:0] event_tag,event_crc,event_error;
 (* async_reg="true" *) reg[1:0] event_sync;
 reg event_seen,cancelling;
 // LCD requests and result delivery. Event buses are held by AXI until its next
 // state transition, at least a complete frame later; toggle synchronization
 // gives the receiving domain two full settling clocks before sampling.
 always @(posedge clk_lcd_i or negedge lr)begin
   if(!lr)begin
     snap_toggle<=0;resume_toggle<=0;event_sync<=0;event_seen<=0;cancelling<=0;
     locked_o<=0;locked_frame_id_o<=0;locked_crc32_o<=0;locked_bank_o<=0;
     restored_o<=0;error_o<=0;error_code_o<=0;
   end else begin
     event_sync<={event_sync[0],event_toggle};
     if(snapshot_req_i)begin snap_toggle<=~snap_toggle;locked_o<=0;restored_o<=0;error_o<=0;error_code_o<=0;cancelling<=0;end
     if(resume_req_i)begin resume_toggle<=~resume_toggle;locked_o<=0;restored_o<=0;cancelling<=1;end
     if(event_sync[1]!=event_seen)begin
       event_seen<=event_sync[1];locked_o<=event_locked&&!cancelling&&!resume_req_i;restored_o<=event_restored;
       if(event_restored||event_error!=0)cancelling<=0;
       locked_frame_id_o<=event_tag;locked_crc32_o<=event_crc;locked_bank_o<=event_bank;
       error_code_o<=event_error;error_o<=event_error!=0;
     end
   end
 end
 always @(posedge clk_axi_i or negedge ar)begin
   if(!ar)begin
     snap_sync<=0;resume_sync<=0;ack_sync<=0;blank_sync<=0;wvs_sync<=0;check_ack_sync<=0;
   end else begin
     snap_sync<={snap_sync[0],snap_toggle};resume_sync<={resume_sync[0],resume_toggle};
     ack_sync<={ack_sync[0],gate_ack};blank_sync<={blank_sync[0],!lcd_vs_i};
     wvs_sync<={wvs_sync[0],writer_vs_o};check_ack_sync<={check_ack_sync[0],check_ack};
   end
 end
 always @(posedge clk_axi_i or negedge ar)begin
   if(!ar)begin
     state<=IDLE;gate_command<=0;gate_opcode<=0;snap_seen<=0;resume_seen<=0;
     pin_bank<=0;bank<=0;watchdog<=0;failure<=0;target_tag<=0;target_crc<=0;
     write_outstanding<=0;read_outstanding<=0;total_commands<=0;total_words<=0;
     shot_commands<=0;shot_words<=0;shot_responses<=0;bank_bound<=0;eof_held<=0;
     blank_age<=31;blank_d<=0;wvs_d<=0;check_toggle<=0;check_match<=0;check_crc<=0;
     event_bank<=0;event_toggle<=0;event_locked<=0;event_restored<=0;event_tag<=0;event_crc<=0;event_error<=0;
   end else begin
     blank_d<=blank_sync[1];wvs_d<=wvs_sync[1];
     if(blank_sync[1]&&!blank_d)blank_age<=0;else if(blank_age<31)blank_age<=blank_age+1'b1;
     case({write_cmd,bbeat})2'b10:write_outstanding<=write_outstanding+1'b1;2'b01:write_outstanding<=write_outstanding-1'b1;endcase
     case({read_cmd,read_end})2'b10:read_outstanding<=read_outstanding+1'b1;2'b01:read_outstanding<=read_outstanding-1'b1;endcase
     if(write_cmd)total_commands<=total_commands+1'b1;
     if(wbeat)total_words<=total_words+1'b1;
     if(writer_state_i==3)eof_held<=1;
     if(write_cmd||wbeat||(wvs_sync[1]&&!wvs_d))eof_held<=0;
     if(state==IDLE||state==LOCKED||state==FAILED)watchdog<=0;else watchdog<=watchdog+1'b1;
     case(state)
       IDLE:if(snap_sync[1]!=snap_seen)begin
         snap_seen<=snap_sync[1];failure<=0;event_error<=0;event_restored<=0;
         gate_opcode<=0;gate_command<=~gate_command;state<=STOP_ACK;
       end
       STOP_ACK:if(ack_sync[1]==gate_command)state<=PREP_DRAIN;
       PREP_DRAIN:if(ddr_ready_i&&drained)begin
         shot_commands<=0;shot_words<=0;shot_responses<=0;bank_bound<=0;
         gate_opcode<=1;gate_command<=~gate_command;state<=SHOT_ACK;
       end
       SHOT_ACK:if(ack_sync[1]==gate_command)begin
         if(gate_bad||gate_pixels!=WIDTH*HEIGHT)begin failure<=3;state<=REC_ACK;watchdog<=0;end
         else begin target_tag<=gate_tag;target_crc<=gate_crc;state<=SHOT_DRAIN;end
       end
       SHOT_DRAIN:if(drained)begin
         if(shot_commands!=BURSTS||shot_words!=WORDS||shot_responses!=BURSTS||!bank_bound)begin failure<=4;state<=REC_ACK;watchdog<=0;end
         else state<=PIN;
       end
       PIN:if(read_barrier)begin
         pin_bank<=1;check_crc<=target_crc;check_match<=1;check_toggle<=~check_toggle;state<=VERIFY;
       end
       VERIFY:if(check_ack_sync[1]==check_toggle)begin
         if(check_bad)begin failure<=6;state<=REC_ACK;watchdog<=0;end
         else begin
           event_bank<=bank;event_tag<=target_tag;event_crc<=target_crc;event_locked<=1;event_restored<=0;event_error<=0;
           event_toggle<=~event_toggle;state<=LOCKED;
         end
       end
       REC_ACK:if(ack_sync[1]==gate_command)state<=REC_DRAIN;
       REC_DRAIN:if(ddr_ready_i&&drained&&read_barrier&&check_ack_sync[1]==check_toggle)begin
         shot_commands<=0;shot_words<=0;shot_responses<=0;bank_bound<=0;
         pin_bank<=0;gate_opcode<=2;gate_command<=~gate_command;state<=REC_SOURCE;
       end
       REC_SOURCE:if(ack_sync[1]==gate_command)begin
         if(gate_bad||gate_pixels!=WIDTH*HEIGHT)begin failure<=9;state<=FAILED;
           event_locked<=0;event_restored<=0;event_error<=9;event_toggle<=~event_toggle;end
         else state<=REC_WRITE;
       end
       // A cancelled lock validation can still be in flight in the LCD domain.
       // Keep its bundle stable and await ack before reusing the toggle; else
       // the old ack could equal the new toggle and falsely confirm restore.
       REC_WRITE:if(drained&&check_ack_sync[1]==check_toggle)begin
         if(shot_commands!=BURSTS||shot_words!=WORDS||shot_responses!=BURSTS||!bank_bound)begin
           failure<=9;state<=FAILED;event_locked<=0;event_restored<=0;event_error<=9;event_toggle<=~event_toggle;
         end else begin check_match<=0;check_toggle<=~check_toggle;state<=REC_VERIFY;end
       end
       REC_VERIFY:if(check_ack_sync[1]==check_toggle)begin
         event_locked<=0;event_restored<=!check_bad;event_error<=check_bad?9:failure;
         event_toggle<=~event_toggle;state<=check_bad?FAILED:IDLE;
       end
     endcase
     if(state==SHOT_ACK||state==SHOT_DRAIN||state==REC_SOURCE||state==REC_WRITE)begin
       if(write_cmd)begin
         if(!bank_bound)begin bank<=cmd_addr_i[23:22];bank_bound<=1;end
         shot_commands<=shot_commands+1'b1;
         if(cmd_len_i!=BURST_BEATS-1||cmd_addr_i[21:0]!=shot_commands*BURST_BEATS*16||
           cmd_addr_i[31:24]!=0||(bank_bound&&cmd_addr_i[23:22]!=bank)||shot_commands>=BURSTS)
           begin
             watchdog<=0;
             if(state>=REC_ACK)begin failure<=9;state<=FAILED;event_locked<=0;event_restored<=0;event_error<=9;event_toggle<=~event_toggle;end
             else begin failure<=4;state<=REC_ACK;end
           end
       end
       if(wbeat)begin
         shot_words<=shot_words+1'b1;
         if(wstrb_i!=16'hffff||wlast_i!=(shot_words%BURST_BEATS==BURST_BEATS-1)||shot_words>=WORDS)
           begin
             watchdog<=0;
             if(state>=REC_ACK)begin failure<=9;state<=FAILED;event_locked<=0;event_restored<=0;event_error<=9;event_toggle<=~event_toggle;end
             else begin failure<=4;state<=REC_ACK;end
           end
       end
       if(bbeat)shot_responses<=shot_responses+1'b1;
     end
     // Faults override nominal state progress; a late R/B cannot publish LOCKED.
     if(state!=IDLE&&state!=FAILED&&
       ((!ddr_ready_i)||(bbeat&&(bresp_i!=0||bid_i!=0||(!write_outstanding&&!write_cmd)))||
        (rbeat&&(rresp_i!=0||rid_i!=0||(!read_outstanding&&!read_cmd)))))begin
       watchdog<=0;
       if(state>=REC_ACK)begin
         failure<=9;state<=FAILED;event_locked<=0;event_restored<=0;event_error<=9;event_toggle<=~event_toggle;
       end else begin failure<=!ddr_ready_i?8:(bbeat?4:5);state<=REC_ACK;end
     end
     if(resume_sync[1]!=resume_seen)begin
       resume_seen<=resume_sync[1];
       if(state!=IDLE&&state!=FAILED&&state<REC_ACK)begin state<=REC_ACK;watchdog<=0;end
     end
     if(state!=IDLE&&state!=LOCKED&&state!=FAILED&&watchdog>=WATCHDOG_CYCLES-1)begin
       watchdog<=0;
       if(state>=REC_ACK)begin
         state<=FAILED;event_locked<=0;event_restored<=0;event_error<=failure==0?7:failure;event_toggle<=~event_toggle;
       end else begin failure<=7;state<=REC_ACK;end
     end
   end
 end
 // Validate a fresh, complete LCD raster after the next natural frame boundary.
 (* async_reg="true" *) reg[1:0] check_sync;
 reg check_active,check_started,check_seen,check_ack_reg,check_bad_reg;
 reg lvs_d,lde_d;reg[31:0] display_crc,display_pixels,line_pixels,lines;
 assign check_ack=check_ack_reg;assign check_bad=check_bad_reg;
 function[31:0] crc_step;
   input[31:0] value;input[7:0] data;reg[31:0] work;integer k;
   begin work=value^data;for(k=0;k<8;k=k+1)work=work[0]?((work>>1)^32'hedb88320):(work>>1);crc_step=work;end
 endfunction
 always @(posedge clk_lcd_i or negedge lr)begin
   if(!lr)begin
     check_sync<=0;check_seen<=0;check_ack_reg<=0;check_bad_reg<=0;check_active<=0;check_started<=0;
     lvs_d<=1;lde_d<=0;display_crc<=32'hffffffff;display_pixels<=0;line_pixels<=0;lines<=0;
   end else begin
     check_sync<={check_sync[0],check_toggle};lvs_d<=lcd_vs_i;lde_d<=lcd_de_i;
     if(check_sync[1]!=check_seen)begin
       check_seen<=check_sync[1];check_active<=1;check_started<=0;check_bad_reg<=0;
     end
     if(check_active&&lvs_d&&!lcd_vs_i)begin
       if(!check_started)begin
         check_started<=1;display_crc<=32'hffffffff;display_pixels<=0;line_pixels<=0;lines<=0;
       end else begin
         if(display_pixels!=WIDTH*HEIGHT||lines!=HEIGHT||line_pixels!=0||
           (check_match&&~display_crc!=check_crc))check_bad_reg<=1;
         check_ack_reg<=check_seen;check_active<=0;check_started<=0;
       end
     end
     if(check_active&&check_started)begin
       if(lcd_de_i)begin
         display_crc<=crc_step(display_crc,lcd_gray_i);display_pixels<=display_pixels+1'b1;line_pixels<=line_pixels+1'b1;
         if(!lcd_vs_i||line_pixels>=WIDTH||display_pixels>=WIDTH*HEIGHT)check_bad_reg<=1;
       end
       if(lde_d&&!lcd_de_i)begin
         if(line_pixels!=WIDTH)check_bad_reg<=1;
         line_pixels<=0;lines<=lines+1'b1;
       end
     end
   end
 end
endmodule
