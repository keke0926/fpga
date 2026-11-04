`timescale 1ns/1ps
// Frozen native grayscale DDR -> median DDR -> native 32-bit feature DDR.
// Caller holds the frozen source and all shared-memory leases while busy.
// This controller uses the existing held-payload DDR mailbox, no private RAM.
// done/valid are coincident pulses; only a successful uncancelled task validates
// a staging feature image, never an inspection verdict. WIDTH is a multiple of
// 16 in [128,1024], HEIGHT in [128,600]; production geometry is 1024x600.
module part_feature_frame #(
 parameter integer WIDTH=1024,HEIGHT=600,
 parameter[31:0]MEDIAN_BASE=32'h00700000
)(
 input wire clk,rst_n,start_i,abort_i,
 input wire[31:0]raw_base_i,feature_base_i,task_id_i,frame_id_i,reference_id_i,config_epoch_i,
 output wire busy_o,output reg done_o,valid_o,output reg[3:0]error_o,
 output reg[31:0]task_id_o,frame_id_o,reference_id_o,config_epoch_o,
 output reg[15:0]gray_addr_o,output reg gray_we_o,output reg[7:0]gray_data_o,input wire[7:0]gray_data_i,
 output reg[15:0]state_addr_o,output reg[1:0]state_we_o,state_data_o,input wire[1:0]state_data_i,
 output wire[11:0]queue_addr_o,output wire queue_we_o,output wire[15:0]queue_data_o,input wire[15:0]queue_data_i,
 output wire req_valid_o,input wire req_ready_i,output wire req_write_o,
 output wire[31:0]req_addr_o,output wire[127:0]req_data_o,
 input wire rsp_valid_i,output wire rsp_ready_o,input wire rsp_error_i,input wire[127:0]rsp_data_i
);
 localparam IDLE=0,TILE=1,LOAD=2,RREQ=3,RWAIT=4,PUT=5,CORE_START=6,CORE=7,
 WREQ=8,WWAIT=9,NEXT=10,DRAIN=11,FINISH=12;
 reg[3:0]state;
 reg feature_phase,clip_load;
 reg[10:0]tile_x,tile_y;
 reg[7:0]tile_width,tile_height;
 reg[6:0]lx,ly;
 reg[31:0]raw_base,feature_base,transfer_addr;
 reg[127:0]transfer_data;
 // Loading and output packing are exclusive phases; reuse one payload/address.
 wire[31:0]cache_addr=transfer_addr,write_addr=transfer_addr;
 wire[127:0]cache_data=transfer_data,packed_data=transfer_data;
 reg cache_valid,tile_last_word;
 reg[3:0]pack_count;
 reg[7:0]load_byte;
 wire core_abort=abort_i||state==DRAIN||state==FINISH;
 wire[15:0]mga,fga,fsa;wire fgwe;wire[7:0]fgwd;wire[1:0]fswe,fswd;
 wire mv,fv;wire[6:0]mx,my,fx,fy;wire[7:0]mg;wire[31:0]fw;
 wire feature_busy,feature_done;
 wire core_ready=rst_n&&!abort_i&&state==CORE;
 wire output_valid=feature_phase?fv:mv;
 wire[6:0]output_x=feature_phase?fx:mx,output_y=feature_phase?fy:my;
 wire[31:0]output_index=({21'd0,tile_y}+output_y)*WIDTH+tile_x+output_x;
 wire[127:0]next_packed=feature_phase?{packed_data[95:0],fw}:{packed_data[119:0],mg};
 wire group_full=feature_phase?pack_count==3:pack_count==15;
 wire output_last=output_x+1==tile_width&&output_y+1==tile_height;
 wire[7:0]tile_step=feature_phase?8'd96:8'd112;
 wire[10:0]remain_x=WIDTH-tile_x,remain_y=HEIGHT-tile_y;
 reg signed[12:0]global_x,global_y;
 reg[10:0]source_x,source_y;
 reg[31:0]pixel_addr;
 always@*begin
  global_x=$signed({2'd0,tile_x})+$signed({6'd0,lx});
  global_y=$signed({2'd0,tile_y})+$signed({6'd0,ly});
  if(!clip_load)begin global_x=global_x-(feature_phase?13'sd16:13'sd1);global_y=global_y-(feature_phase?13'sd16:13'sd1);end
  if(global_x<0)source_x=feature_phase?-global_x:0;
  else if(global_x>=WIDTH)source_x=feature_phase?(2*WIDTH-2-global_x):(WIDTH-1);
  else source_x=global_x[10:0];
  if(global_y<0)source_y=feature_phase?-global_y:0;
  else if(global_y>=HEIGHT)source_y=feature_phase?(2*HEIGHT-2-global_y):(HEIGHT-1);
  else source_y=global_y[10:0];
  pixel_addr=(feature_phase&&!clip_load?MEDIAN_BASE:raw_base)+source_y*WIDTH+source_x;
 end
 assign busy_o=state!=IDLE;
 assign req_valid_o=rst_n&&!abort_i&&(state==RREQ||state==WREQ);
 assign req_write_o=state==WREQ;
 assign req_addr_o=state==WREQ?write_addr:{pixel_addr[31:4],4'd0};
 assign req_data_o=packed_data;
 assign rsp_ready_o=rst_n&&(state==RWAIT||state==WWAIT||state==DRAIN);
 part_median_tile median(.clk(clk),.rst_n(rst_n),.abort_i(core_abort),
 .start_i(state==CORE_START&&!feature_phase),.width_i(tile_width),.height_i(tile_height),
 .gray_addr_o(mga),.gray_data_i(gray_data_i),.out_valid_o(mv),.out_ready_i(core_ready&&!feature_phase),
 .out_x_o(mx),.out_y_o(my),.out_gray_o(mg));
 part_feature_tile features(.clk(clk),.rst_n(rst_n),.abort_i(core_abort),
 .start_i(state==CORE_START&&feature_phase),.width_i(tile_width),.height_i(tile_height),
 .busy_o(feature_busy),.done_o(feature_done),
 .gray_addr_o(fga),.gray_we_o(fgwe),.gray_data_o(fgwd),.gray_data_i(gray_data_i),
 .state_addr_o(fsa),.state_we_o(fswe),.state_data_o(fswd),.state_data_i(state_data_i),
 .queue_addr_o(queue_addr_o),.queue_we_o(queue_we_o),.queue_data_o(queue_data_o),.queue_data_i(queue_data_i),
 .out_valid_o(fv),.out_ready_i(core_ready&&feature_phase),.out_x_o(fx),.out_y_o(fy),.out_word_o(fw));
 always@*begin
  gray_addr_o=feature_phase?fga:mga;gray_we_o=feature_phase&&fgwe;gray_data_o=fgwd;
  state_addr_o=fsa;state_we_o=fswe;state_data_o=fswd;
  if(state==PUT)begin
   if(clip_load)begin
    state_addr_o=({9'd0,ly}+16'd1)*16'd98+lx+16'd1;
    state_we_o=2'b01;state_data_o={1'b0,(load_byte<=4||load_byte>=251)};
   end else begin gray_addr_o={2'd0,ly,lx};gray_we_o=1;gray_data_o=load_byte;end
  end
  if(!rst_n||abort_i)begin gray_we_o=0;state_we_o=0;end
 end
 always@(posedge clk)begin
  if(!rst_n)begin
   state<=IDLE;done_o<=0;valid_o<=0;error_o<=0;task_id_o<=0;frame_id_o<=0;reference_id_o<=0;config_epoch_o<=0;
   feature_phase<=0;clip_load<=0;tile_x<=0;tile_y<=0;tile_width<=0;tile_height<=0;lx<=0;ly<=0;
   raw_base<=0;feature_base<=0;transfer_addr<=0;transfer_data<=0;
   cache_valid<=0;tile_last_word<=0;pack_count<=0;load_byte<=0;
  end else begin
   done_o<=0;valid_o<=0;
   if(abort_i&&state!=IDLE&&state!=FINISH&&state!=DRAIN)begin
    valid_o<=0;error_o<=3;
    if((state==RWAIT||state==WWAIT)&&!rsp_valid_i)state<=DRAIN;
    else state<=FINISH;
   end else case(state)
    IDLE:if(start_i&&!abort_i)begin
     valid_o<=0;error_o<=0;raw_base<=raw_base_i;feature_base<=feature_base_i;
     task_id_o<=task_id_i;frame_id_o<=frame_id_i;reference_id_o<=reference_id_i;config_epoch_o<=config_epoch_i;
     feature_phase<=0;clip_load<=0;tile_x<=0;tile_y<=0;cache_valid<=0;state<=TILE;
     // Source extent and the staging-bank choice are validated once per task.
     if(WIDTH<128||WIDTH>1024||WIDTH%16!=0||HEIGHT<128||HEIGHT>600||
        (raw_base_i!=0&&raw_base_i!=32'h400000&&raw_base_i!=32'h800000&&raw_base_i!=32'hc00000&&raw_base_i!=32'h500000&&raw_base_i!=32'h600000)||
        (feature_base_i!=32'h100000&&feature_base_i!=32'h900000&&feature_base_i!=32'hd00000))begin error_o<=1;state<=FINISH;end
    end
    TILE:begin
     tile_width<=remain_x<tile_step?remain_x[7:0]:tile_step;
     tile_height<=remain_y<tile_step?remain_y[7:0]:tile_step;
     lx<=0;ly<=0;clip_load<=0;cache_valid<=0;pack_count<=0;transfer_data<=0;state<=LOAD;
    end
    LOAD:if(cache_valid&&cache_addr=={pixel_addr[31:4],4'd0})begin
     load_byte<=cache_data >> ((15-pixel_addr[3:0])*8);state<=PUT;
    end else state<=RREQ;
    RREQ:if(req_ready_i)state<=RWAIT;
    RWAIT:if(rsp_valid_i)begin
     if(rsp_error_i)begin error_o<=2;state<=FINISH;end
     else begin transfer_data<=rsp_data_i;transfer_addr<={pixel_addr[31:4],4'd0};cache_valid<=1;state<=LOAD;end
    end
    PUT:begin
     if(lx+1==(clip_load?tile_width:8'd128))begin
      lx<=0;
      if(ly+1==(clip_load?tile_height:8'd128))begin
       ly<=0;
       if(feature_phase&&!clip_load)begin clip_load<=1;cache_valid<=0;state<=LOAD;end
       else state<=CORE_START;
      end else begin ly<=ly+1'b1;state<=LOAD;end
     end else begin lx<=lx+1'b1;state<=LOAD;end
    end
    CORE_START:begin transfer_data<=0;cache_valid<=0;state<=CORE;end
    CORE:if(output_valid)begin
     transfer_data<=next_packed;
     if(group_full)begin
      transfer_addr<=feature_phase?feature_base+(output_index<<2)-32'd12:MEDIAN_BASE+output_index-32'd15;
      tile_last_word<=output_last;pack_count<=0;state<=WREQ;
     end else pack_count<=pack_count+1'b1;
    end
    WREQ:if(req_ready_i)state<=WWAIT;
    WWAIT:if(rsp_valid_i)begin
     if(rsp_error_i)begin error_o<=2;state<=FINISH;end
     else begin transfer_data<=0;state<=tile_last_word?NEXT:CORE;end
    end
    NEXT:begin
     if(tile_x+tile_width==WIDTH)begin
      tile_x<=0;
      if(tile_y+tile_height==HEIGHT)begin
       tile_y<=0;
       if(feature_phase)state<=FINISH;
       else begin feature_phase<=1;state<=TILE;end
      end else begin tile_y<=tile_y+tile_step;state<=TILE;end
     end else begin tile_x<=tile_x+tile_step;state<=TILE;end
    end
    DRAIN:if(rsp_valid_i)state<=FINISH;
    FINISH:begin
     done_o<=1;valid_o<=error_o==0&&!abort_i;
     if(abort_i)error_o<=3;
     state<=IDLE;
    end
   endcase
  end
 end
endmodule
