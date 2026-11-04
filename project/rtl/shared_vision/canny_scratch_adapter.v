// Candidate extraction boundary for the general engine's existing gray,
// visited-mask and foreground memories. This adapter is NOT yet substituted
// into the released general engine. A single lease selects every memory port.
// Host has one-cycle synchronous reads; writes during a Canny lease are ignored.
module canny_scratch_adapter(
 input wire clk,rst_n,abort_i,request_i,
 input wire [7:0] low_i,high_i,
 input wire [31:0] frame_id_i,config_tag_i,
 input wire [15:0] host_gray_addr_i,host_state_addr_i,
 input wire host_gray_we_i,
 input wire [1:0] host_state_we_i,
 input wire [7:0] host_gray_data_i,
 input wire [1:0] host_state_data_i,
 output reg [7:0] host_gray_data_o,
 output reg [1:0] host_state_data_o,
 output wire leased_o,done_o,valid_o,
 output wire [3:0] error_o,phase_o,
 output wire [31:0] frame_id_o,config_tag_o,cycles_o,
 output wire edge_valid_o,edge_bit_o,
 output wire [13:0] edge_addr_o
);
 (* ram_style="block" *) reg [7:0] gray_memory[0:38399];
 (* ram_style="block" *) reg mask_memory[0:38399];
 (* ram_style="block" *) reg foreground_memory[0:38399];
 reg leased,start;
 reg [7:0] low,high;
 reg [31:0] frame,config_tag;
 wire busy;
 wire [14:0] ga;wire gwe;wire [7:0] gwd;
 wire [13:0] sa;wire swe;wire [1:0] swd;
 wire [15:0] gray_addr=leased ? {1'b0,ga} : host_gray_addr_i;
 wire [15:0] mask_addr=leased ? {2'b0,sa} : host_state_addr_i;
 wire gray_we=rst_n && !abort_i && (leased ? gwe : host_gray_we_i);
 wire [1:0] mask_we=(rst_n && !abort_i) ? (leased ? {2{swe}} : host_state_we_i) : 2'b00;
 wire [7:0] gray_data=leased ? gwd : host_gray_data_i;
 wire [1:0] mask_data=leased ? swd : host_state_data_i;
 assign leased_o=leased;
 always @(posedge clk)begin
  host_gray_data_o<=gray_memory[gray_addr];
  host_state_data_o<={foreground_memory[mask_addr],mask_memory[mask_addr]};
  if(gray_we && gray_addr<38400)gray_memory[gray_addr]<=gray_data;
  if(mask_we[0] && mask_addr<38400)mask_memory[mask_addr]<=mask_data[0];
  if(mask_we[1] && mask_addr<38400)foreground_memory[mask_addr]<=mask_data[1];
 end
 always @(posedge clk)begin
  if(!rst_n || abort_i)begin leased<=0;start<=0;low<=0;high<=0;frame<=0;config_tag<=0;end
  else begin
   start<=0;
   if(!leased && request_i)begin
    leased<=1;start<=1;low<=low_i;high<=high_i;frame<=frame_id_i;config_tag<=config_tag_i;
   end else if(leased && done_o)leased<=0;
  end
 end
 canny_shared_scratch u_canny(
 .clk(clk),.rst_n(rst_n),.abort_i(abort_i || !leased),.start_i(start),.hysteresis_only_i(1'b0),
 .low_i(low),.high_i(high),.frame_id_i(frame),.config_tag_i(config_tag),
 .busy_o(busy),.done_o(done_o),.valid_o(valid_o),.error_o(error_o),.phase_o(phase_o),
 .frame_id_o(frame_id_o),.config_tag_o(config_tag_o),.cycles_o(cycles_o),
 .gray_addr_o(ga),.gray_we_o(gwe),.gray_data_o(gwd),.gray_data_i(host_gray_data_o),
 .state_addr_o(sa),.state_we_o(swe),.state_data_o(swd),.state_data_i(host_state_data_o),
 .edge_valid_o(edge_valid_o),.edge_addr_o(edge_addr_o),.edge_bit_o(edge_bit_o));
endmodule
