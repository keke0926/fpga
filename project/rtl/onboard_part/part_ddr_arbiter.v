`timescale 1ns/1ps
// Vendor combined AW/AR interface. Video ID 0; local analysis ID 1.
// Analysis single-word transfers run only with a complete pinned frame. Video
// commands win each fresh arbitration; an already offered command cannot change.
module part_ddr_arbiter # (parameter integer FRAME_BYTES=614400)(
 input wire clk,rst_n,ddr_ready_i,held_i,input wire[1:0]bank_i,
 input wire v_avalid,output wire v_aready,input wire[31:0]v_aaddr,input wire[3:0]v_aid,
 input wire[7:0]v_alen,input wire v_atype,
 input wire v_wvalid,output wire v_wready,input wire[127:0]v_wdata,input wire[15:0]v_wstrb,
 input wire v_wlast,input wire[3:0]v_wid,
 output wire v_bvalid,input wire v_bready,output wire[3:0]v_bid,output wire[1:0]v_bresp,
 output wire v_rvalid,input wire v_rready,output wire[3:0]v_rid,output wire[1:0]v_rresp,
 output wire[127:0]v_rdata,output wire v_rlast,
 input wire a_req_valid,output wire a_req_ready,input wire a_req_write,
 input wire[31:0]a_req_addr,input wire[127:0]a_req_data,
 output reg a_rsp_valid,input wire a_rsp_ready,output reg a_rsp_error,output reg[127:0]a_rsp_data,
 output wire p_avalid,input wire p_aready,output wire[31:0]p_aaddr,output wire[3:0]p_aid,
 output wire[7:0]p_alen,output wire p_atype,
 output wire p_wvalid,input wire p_wready,output wire[127:0]p_wdata,output wire[15:0]p_wstrb,
 output wire p_wlast,output wire[3:0]p_wid,
 input wire p_bvalid,output wire p_bready,input wire[3:0]p_bid,input wire[1:0]p_bresp,
 input wire p_rvalid,output wire p_rready,input wire[3:0]p_rid,input wire[1:0]p_rresp,
 input wire[127:0]p_rdata,input wire p_rlast,
 output reg video_range_fault_o
);
 reg[1:0]selected;reg a_busy,a_writing,a_wsent,a_read_bad;
 // The current DDR core registers the length FIFO head one cycle late.
 // After accepting RLAST, let that register load the next burst's length
 // before accepting its first beat (video LEN127 and local LEN0 can differ).
 reg r_turnaround;
 reg[9:0]video_writes;
 wire[22:0]video_end={1'b0,v_aaddr[21:0]}+(({15'd0,v_alen}+23'd1)<<4);
 wire video_address_ok=v_aaddr[31:24]==0&&v_aaddr[3:0]==0&&video_end<=FRAME_BYTES;
 wire analysis_reserved=a_req_addr[21:0]>=22'h100000;
 wire analysis_source=a_req_addr[23:22]==bank_i&&
                         ({1'b0,a_req_addr[21:0]}+23'd16)<=FRAME_BYTES;
 wire analysis_address_ok=held_i&&a_req_addr[31:24]==0&&a_req_addr[3:0]==0&&
                          (analysis_reserved||(!a_req_write&&analysis_source));
 wire reject_request=selected==0&&!a_busy&&a_req_valid&&!analysis_address_ok;
 wire analysis_write_owner=a_writing||(selected==2&&a_req_write);
 wire video_eligible=v_avalid&&(!v_atype||!analysis_write_owner);
 assign p_avalid=rst_n&&ddr_ready_i&&((selected==1&&video_eligible)||(selected==2&&a_req_valid));
 assign p_atype=selected==2?a_req_write:v_atype;
 assign p_aid=selected==2?4'd1:4'd0;
 assign p_alen=selected==2?8'd0:v_alen;
 // Faulted video addresses still complete to avoid wedging the vendor writer;
 // redirect inside its image slot so an invalid video burst cannot touch reference storage.
 assign p_aaddr=selected==2?a_req_addr:(video_address_ok?v_aaddr:{8'd0,v_aaddr[23:22],22'd0});
 assign v_aready=selected==1&&p_avalid&&p_aready;
 assign a_req_ready=rst_n&&ddr_ready_i&&(reject_request||(selected==2&&p_avalid&&p_aready));
 // AW and W must be offered independently: the controller may wait for WVALID
 // before accepting AW. Keep an early accepted W beat recorded until B completes.
 assign p_wvalid=ddr_ready_i&&(analysis_write_owner?!a_wsent:v_wvalid);
 assign p_wdata=analysis_write_owner?a_req_data:v_wdata;
 assign p_wstrb=analysis_write_owner?16'hffff:v_wstrb;
 assign p_wlast=analysis_write_owner?1'b1:v_wlast;
 assign p_wid=analysis_write_owner?4'd1:4'd0;
 assign v_wready=ddr_ready_i&&!analysis_write_owner&&p_wready;
 assign v_bvalid=p_bvalid&&p_bid!=1;assign v_bid=p_bid;assign v_bresp=p_bresp;
 assign v_rvalid=p_rvalid&&p_rid!=1&&!r_turnaround;assign v_rid=p_rid;assign v_rresp=p_rresp;
 assign v_rdata=p_rdata;assign v_rlast=p_rlast;
 assign p_bready=p_bid==1?(a_busy&&a_writing&&a_wsent&&!a_rsp_valid):v_bready;
 assign p_rready=!r_turnaround&&(p_rid==1?(a_busy&&!a_writing&&!a_rsp_valid):v_rready);
 wire vwrite_command=v_aready&&v_atype;
 wire vwrite_response=v_bvalid&&v_bready;
 always@(posedge clk)begin
  if(!rst_n)begin
   selected<=0;a_busy<=0;a_writing<=0;a_wsent<=0;a_read_bad<=0;video_writes<=0;
   r_turnaround<=0;
   a_rsp_valid<=0;a_rsp_error<=0;a_rsp_data<=0;video_range_fault_o<=0;
  end else begin
   r_turnaround<=p_rvalid&&p_rready&&p_rlast;
   case({vwrite_command,vwrite_response})
    2'b10:video_writes<=video_writes+1'b1;
    2'b01:if(video_writes!=0)video_writes<=video_writes-1'b1;
   endcase
   if((v_aready&&(!video_address_ok||v_aid!=0))||(v_wvalid&&v_wready&&v_wid!=0))video_range_fault_o<=1;
   if(selected==0&&ddr_ready_i)begin
    if(video_eligible)selected<=1;
    else if(a_req_valid&&!a_busy&&analysis_address_ok&&video_writes==0&&!v_wvalid)selected<=2;
   end else if(p_avalid&&p_aready)selected<=0;
   if(a_req_valid&&a_req_ready)begin
    a_busy<=1;a_read_bad<=0;a_rsp_error<=0;
    if(reject_request)begin a_rsp_valid<=1;a_rsp_error<=1;a_rsp_data<=0;a_writing<=0;end
    else a_writing<=a_req_write;
   end
   if(analysis_write_owner&&p_wvalid&&p_wready)a_wsent<=1;
   if(p_bvalid&&p_bready&&p_bid==1)begin
    a_rsp_valid<=1;a_rsp_error<=p_bresp!=0;a_rsp_data<=0;
   end
   if(p_rvalid&&p_rready&&p_rid==1)begin
    a_rsp_data<=p_rdata;
    if(!p_rlast)a_read_bad<=1;
    else begin a_rsp_valid<=1;a_rsp_error<=a_read_bad||p_rresp!=0;end
    if(p_rresp!=0)a_read_bad<=1;
   end
   if(a_rsp_valid&&a_rsp_ready)begin a_rsp_valid<=0;a_busy<=0;a_writing<=0;a_wsent<=0;end
  end
 end
endmodule
