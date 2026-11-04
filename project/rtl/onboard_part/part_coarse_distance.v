`timescale 1ns/1ps
// Exact capped L1 distance on a count grid, two raster passes through the
// already leased byte scratch RAM. Read latency: one clock. No private RAM.
module part_coarse_distance #(parameter integer GW=128,GH=75,DEPTH=38400)(
 input wire clk_i,rst_i,abort_i,start_i,output wire ready_o,busy_o,
 input wire[15:0] count_base_i,distance_base_i,input wire[7:0] threshold_i,cap_i,
 output wire done_o,result_valid_o,input wire out_ready_i,output reg[3:0] error_o,
 output reg[15:0] mem_addr_o,output wire mem_we_o,output wire[7:0] mem_data_o,input wire[7:0] mem_data_i,
 output reg[13:0] seed_count_o,output reg[20:0] sum_x_o,output reg[19:0] sum_y_o,
 output reg[6:0] min_x_o,max_x_o,min_y_o,max_y_o,output reg[31:0] cycles_o
);
 localparam IDLE=0,COUNT_REQ=1,COUNT_WAIT=2,NORTH_REQ=3,NORTH_WAIT=4,FWRITE=5,
 CURRENT_REQ=6,CURRENT_WAIT=7,SOUTH_REQ=8,SOUTH_WAIT=9,BWRITE=10,RESULT=11;
 reg[3:0] state;
 reg[15:0] count_base,distance_base;
 reg[7:0] threshold,cap,value,neighbor;
 reg[13:0] index;
 reg[6:0] x,y;
 function[7:0] increment;
  input[7:0] v,limit;
  begin increment=(v>=limit)?limit:v+1'b1;end
 endfunction
 wire[7:0] adjacent=increment(neighbor,cap),vertical=increment(mem_data_i,cap);
 wire last=index==GW*GH-1;
 assign ready_o=state==IDLE&&!rst_i&&!abort_i;
 assign busy_o=state!=IDLE;
 assign result_valid_o=state==RESULT&&!rst_i&&!abort_i;
 assign done_o=result_valid_o;
 assign mem_we_o=(state==FWRITE||state==BWRITE)&&!rst_i&&!abort_i;
 assign mem_data_o=value;
 always@*begin
  mem_addr_o=distance_base+index;
  if(state==COUNT_REQ||state==COUNT_WAIT)mem_addr_o=count_base+index;
  else if(state==NORTH_REQ||state==NORTH_WAIT)mem_addr_o=distance_base+index-GW;
  else if(state==SOUTH_REQ||state==SOUTH_WAIT)mem_addr_o=distance_base+index+GW;
 end
 always@(posedge clk_i)begin
  if(rst_i)begin
   state<=IDLE;count_base<=0;distance_base<=0;threshold<=0;cap<=0;value<=0;neighbor<=0;
   index<=0;x<=0;y<=0;error_o<=0;seed_count_o<=0;sum_x_o<=0;sum_y_o<=0;
   min_x_o<=0;max_x_o<=0;min_y_o<=0;max_y_o<=0;cycles_o<=0;
  end else if(abort_i)state<=IDLE;
  else begin
   if(state!=IDLE&&state!=RESULT)cycles_o<=cycles_o+1'b1;
   case(state)
    IDLE:if(start_i&&ready_o)begin
     count_base<=count_base_i;distance_base<=distance_base_i;threshold<=threshold_i;cap<=cap_i;
     value<=cap_i;neighbor<=cap_i;index<=0;x<=0;y<=0;error_o<=0;seed_count_o<=0;sum_x_o<=0;sum_y_o<=0;
     min_x_o<=0;max_x_o<=0;min_y_o<=0;max_y_o<=0;cycles_o<=0;
     if(GW<1||GW>128||GH<1||GH>75||threshold_i==0||cap_i==0||
        {1'b0,count_base_i}+GW*GH>DEPTH||{1'b0,distance_base_i}+GW*GH>DEPTH||
        ((count_base_i<distance_base_i+GW*GH)&&(distance_base_i<count_base_i+GW*GH)))begin
       error_o<=1;state<=RESULT;
     end else state<=COUNT_REQ;
    end
    COUNT_REQ:state<=COUNT_WAIT;
    COUNT_WAIT:begin
     if(mem_data_i>=threshold)begin
      value<=0;state<=FWRITE;seed_count_o<=seed_count_o+1'b1;sum_x_o<=sum_x_o+x;sum_y_o<=sum_y_o+y;
      if(seed_count_o==0)begin min_x_o<=x;max_x_o<=x;min_y_o<=y;max_y_o<=y;end
      else begin if(x<min_x_o)min_x_o<=x;if(x>max_x_o)max_x_o<=x;if(y<min_y_o)min_y_o<=y;if(y>max_y_o)max_y_o<=y;end
     end else begin value<=x==0?cap:adjacent;state<=y==0?FWRITE:NORTH_REQ;end
    end
    NORTH_REQ:state<=NORTH_WAIT;
    NORTH_WAIT:begin if(vertical<value)value<=vertical;state<=FWRITE;end
    FWRITE:begin
     neighbor<=value;
     if(last)begin neighbor<=cap;state<=CURRENT_REQ;end
     else begin
      index<=index+1'b1;
      if(x==GW-1)begin x<=0;y<=y+1'b1;neighbor<=cap;end else x<=x+1'b1;
      state<=COUNT_REQ;
     end
    end
    CURRENT_REQ:state<=CURRENT_WAIT;
    CURRENT_WAIT:begin value<=(x!=GW-1&&adjacent<mem_data_i)?adjacent:mem_data_i;state<=y==GH-1?BWRITE:SOUTH_REQ;end
    SOUTH_REQ:state<=SOUTH_WAIT;
    SOUTH_WAIT:begin if(vertical<value)value<=vertical;state<=BWRITE;end
    BWRITE:begin
     neighbor<=value;
     if(index==0)state<=RESULT;
     else begin
      index<=index-1'b1;
      if(x==0)begin x<=GW-1;y<=y-1'b1;neighbor<=cap;end else x<=x-1'b1;
      state<=CURRENT_REQ;
     end
    end
    RESULT:if(out_ready_i)state<=IDLE;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
