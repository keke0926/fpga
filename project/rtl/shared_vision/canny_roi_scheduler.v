`timescale 1ns/1ps
// PART transaction owner. No image storage: result writes use app overview RAM.
module canny_roi_scheduler #(parameter TIMEOUT_CYCLES=96000000)(
 input wire clk,rst_n,enable_i,part_mode_i,locked_i,
 input wire[31:0]source_frame_i,source_crc_i,frame_i,crc_i,reference_i,tag_i,
 input wire[9:0]roi_x_i,roi_y_i,input wire[7:0]low_i,high_i,input wire[15:0]frontend_i,
 input wire de_i,sof_i,eof_i,align_error_i,input wire[11:0]x_i,y_i,input wire[7:0]gray_i,
 input wire ready_i,done_i,valid_i,edge_valid_i,edge_bit_i,input wire[13:0]edge_addr_i,
 input wire[3:0]error_i,input wire[31:0]result_frame_i,result_tag_i,cycles_i,
 output wire start_o,abort_o,output reg sample_valid_o,end_o,
 output reg[13:0]sample_addr_o,output reg[7:0]sample_gray_o,
 output wire[7:0]low_o,high_o,output wire[31:0]frame_o,tag_o,
 output wire write_o,write_bank_o,display_valid_o,display_bank_o,
 output wire[9:0]display_x_o,display_y_o,
 output reg[3:0]state_o,error_o,output reg[14:0]samples_o,edges_o,
 output reg[31:0]cycles_o,display_commits_o
);
 localparam OFF=0,QUEUED=1,CAPTURE=2,COMPUTE=3,PENDING=4,DISPLAY=5,ERROR=6;
 reg[179:0]configuration;reg configured;reg[31:0]timer;
 wire[179:0]incoming={frame_i,crc_i,reference_i,tag_i,roi_x_i,roi_y_i,low_i,high_i,frontend_i};
 wire[31:0]saved_crc,saved_reference;wire[9:0]rx,ry;wire[15:0]frontend;
 assign {frame_o,saved_crc,saved_reference,tag_o,rx,ry,low_o,high_o,frontend}=configuration;
 wire changed=configured&&incoming[179:16]!=configuration[179:16];
 wire frontend_changed=configured&&frontend_i!=frontend;
 wire allowed=enable_i&&part_mode_i&&locked_i;
 wire identity=source_frame_i==frame_o&&source_crc_i==saved_crc;
 wire legal=rx<=896&&ry<=472&&low_o!=0&&low_o<=high_o&&saved_reference!=0&&tag_o!=0;
 wire source_sof=de_i&&sof_i;
 wire good_sof=source_sof&&x_i==0&&y_i==0&&!align_error_i;
 reg[11:0]expected_x,expected_y;reg[19:0]pixels;reg[14:0]outputs;
 reg back_bank,front_bank;
 wire capture_fault=(state_o==QUEUED&&good_sof&&eof_i)||(state_o==CAPTURE&&(align_error_i||(de_i&&(source_sof||x_i!=expected_x||y_i!=expected_y||x_i>=1024||y_i>=600||
   (eof_i&&(x_i!=1023||y_i!=599||pixels!=614399))||(!eof_i&&pixels==614399)))));
 wire active=state_o==QUEUED||state_o==CAPTURE||state_o==COMPUTE||state_o==PENDING;
 wire timeout=active&&timer>=TIMEOUT_CYCLES-1;
 wire stream_fault=state_o==COMPUTE&&edge_valid_i&&(outputs>=16384||{1'b0,edge_addr_i}!=outputs);
 wire stale=!allowed||changed||frontend_changed||!identity;
 wire abort_fault=configured&&(!legal||capture_fault||timeout||align_error_i);
 wire fault=abort_fault||stream_fault;
 // edge_valid_i is itself gated by the core's abort input. A bad address
 // masks RAM writes immediately, then ERROR latches the abort at the clock;
 // never feed combinational edge_valid back into its own abort mask.
 assign abort_o=rst_n&&configured&&(stale||abort_fault||state_o==ERROR);
 assign start_o=rst_n&&state_o==QUEUED&&configured&&!stale&&!fault&&good_sof&&ready_i;
 wire capture_pixel=(start_o||state_o==CAPTURE)&&de_i&&!abort_o;
 wire roi_sample=capture_pixel&&x_i>=rx&&x_i<{2'b0,rx}+12'd128&&y_i>=ry&&y_i<{2'b0,ry}+12'd128;
 wire[11:0]dx=x_i-{2'b0,rx},dy=y_i-{2'b0,ry};
 wire publish=state_o==PENDING&&good_sof&&!stale&&!fault;
 assign display_valid_o=rst_n&&configured&&!stale&&!fault&&(state_o==DISPLAY||publish);
 assign display_bank_o=publish?back_bank:front_bank;
 assign display_x_o=rx;assign display_y_o=ry;
 assign write_o=rst_n&&state_o==COMPUTE&&!stale&&!fault&&edge_valid_i;
 assign write_bank_o=back_bank;
 always @(posedge clk)begin
  if(!rst_n)begin
   configuration<=0;configured<=0;state_o<=OFF;error_o<=0;timer<=0;expected_x<=0;expected_y<=0;pixels<=0;
   samples_o<=0;edges_o<=0;outputs<=0;cycles_o<=0;display_commits_o<=0;back_bank<=1;front_bank<=0;
   sample_valid_o<=0;sample_addr_o<=0;sample_gray_o<=0;end_o<=0;
  end else begin
   sample_valid_o<=roi_sample;sample_addr_o<={dy[6:0],dx[6:0]};sample_gray_o<=gray_i;
   // END travels with the registered last sample, including ROI (896,472).
   end_o<=state_o==CAPTURE&&de_i&&eof_i&&!abort_o;
   if(!enable_i||!part_mode_i)begin
    configured<=0;state_o<=OFF;error_o<=0;timer<=0;sample_valid_o<=0;end_o<=0;samples_o<=0;edges_o<=0;outputs<=0;
   end else if(!configured||changed)begin
    configuration<=incoming;configured<=1;state_o<=QUEUED;error_o<=0;timer<=0;sample_valid_o<=0;end_o<=0;samples_o<=0;edges_o<=0;outputs<=0;cycles_o<=0;
   end else if(state_o!=ERROR)begin
    if(!locked_i||!identity||frontend_changed||fault)begin
     state_o<=ERROR;sample_valid_o<=0;end_o<=0;
     if(frontend_changed)error_o<=7;
     else if(!locked_i||!identity)error_o<=2;
     else if(!legal)error_o<=1;
     else if(capture_fault||align_error_i)error_o<=3;
     else if(stream_fault)error_o<=5;
     else error_o<=6;
    end else begin
     if(active)timer<=timer+1'b1;
     if(start_o)begin state_o<=CAPTURE;pixels<=1;expected_x<=1;expected_y<=0;back_bank<=!front_bank;samples_o<=roi_sample?1:0;end
     else if(state_o==CAPTURE&&de_i)begin
      pixels<=pixels+1'b1;
      if(expected_x==1023)begin expected_x<=0;expected_y<=expected_y+1'b1;end else expected_x<=expected_x+1'b1;
      if(roi_sample)samples_o<=samples_o+1'b1;
      if(eof_i)state_o<=COMPUTE;
     end
     if(state_o==COMPUTE)begin
      if(edge_valid_i)begin outputs<=outputs+1'b1;if(edge_bit_i)edges_o<=edges_o+1'b1;end
      if(done_i)begin
       cycles_o<=cycles_i;
       if(!valid_i||error_i!=0)begin state_o<=ERROR;error_o<=4;end
       else if(outputs!=16384||samples_o!=16384)begin state_o<=ERROR;error_o<=5;end
       else if(result_frame_i!=frame_o||result_tag_i!=tag_o)begin state_o<=ERROR;error_o<=2;end
       else begin state_o<=PENDING;timer<=0;end
      end
     end
     if(publish)begin state_o<=DISPLAY;front_bank<=back_bank;display_commits_o<=display_commits_o+1'b1;end
    end
   end
  end
 end
endmodule
