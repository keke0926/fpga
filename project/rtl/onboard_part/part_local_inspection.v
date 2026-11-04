`timescale 1ns/1ps
// One frozen-frame transaction: autonomous registration, pose export, native
// difference, shared region extraction and atomic same-frame publication.
// The parent holds task/frame/reference/config and all memory leases until drain.
module part_local_inspection(
 input wire clk_i,rst_i,abort_i,start_i,output wire ready_o,busy_o,result_valid_o,input wire out_ready_i,
 input wire[31:0]ref_base_i,obs_base_i,task_id_i,frame_id_i,frame_crc_i,reference_id_i,config_epoch_i,
 output reg[3:0]error_o,output wire[9:0]count_o,output reg[3:0]phase_o,
 output wire[63:0]diagnostic_o,
 output wire[15:0]gray_addr_o,state_addr_o,output wire gray_we_o,output wire[7:0]gray_data_o,input wire[7:0]gray_data_i,
 output wire[1:0]state_we_o,state_data_o,input wire[1:0]state_data_i,
 output wire[11:0]queue_addr_o,output wire queue_we_o,output wire[15:0]queue_data_o,input wire[15:0]queue_data_i,
 output wire req_valid_o,input wire req_ready_i,output wire req_write_o,output wire[31:0]req_addr_o,output wire[127:0]req_data_o,
 input wire rsp_valid_i,output wire rsp_ready_o,input wire rsp_error_i,input wire[127:0]rsp_data_i,
 output wire region_start_o,region_ready_o,input wire region_idle_i,region_valid_i,region_done_i,
 input wire[8:0]region_x0_i,region_x1_i,input wire[7:0]region_y0_i,region_y1_i,
 input wire[11:0]region_count_i,input wire[3:0]region_error_i,
 output wire cmd_valid_o,input wire cmd_ready_i,output wire[7:0]cmd_opcode_o,cmd_flags_o,
 output wire[15:0]cmd_data_o,output wire[31:0]cmd_request_o,
 input wire response_valid_i,output wire response_ready_o,input wire[7:0]response_status_i
);
 localparam IDLE=0,RSTART=1,RWAIT=2,ESTART=3,EWAIT=4,DSTART=5,DWAIT=6,PSTART=7,PWAIT=8,RESULT=9,DRAIN=10;
 reg[3:0]state;reg[1:0]owner;
 wire stop=abort_i||state==DRAIN;
 wire rr,rb,rv,ra,rt,er,eb,ev,dr,db,dv,pr,pb,pv,pa;
 wire[3:0]re,ee,de,pe;wire[5:0]records,poses;
 wire[9:0]x0,y0,x1,y1;
 wire[19:0]candidate_points,clean_points;
 assign diagnostic_o={error_o,phase_o,poses,count_o,clean_points,candidate_points};
 wire[15:0]rga,ega,dga;wire rgw,dgw;wire[7:0]rgd,dgd;
 wire[11:0]rqa,dqa;wire rqw,dqw;wire[15:0]rqd,dqd;
 wire rq,eq,dq,rw,ew,dw,rs,es,ds;wire[31:0]raddr,eaddr,daddr;wire[127:0]rdata,edata,ddata;
 wire ss,sa,sabort,sready,sbusy,sv;wire[194:0]sq;wire[75:0]reply;
 wire sample_owner=owner==2&&(sbusy||ss);
 wire registration_ddr=owner==0||sample_owner;
 assign ready_o=state==IDLE&&rr&&!rst_i&&!abort_i;
 assign busy_o=state!=IDLE&&state!=RESULT;
 assign result_valid_o=state==RESULT&&!rst_i&&!abort_i;
 assign gray_addr_o=owner==0?rga:owner==1?ega:dga;
 assign gray_we_o=!stop&&(owner==0?rgw:owner==2?dgw:1'b0);
 assign gray_data_o=owner==0?rgd:dgd;
 assign queue_addr_o=owner==0?rqa:dqa;assign queue_we_o=!stop&&(owner==0?rqw:owner==2?dqw:1'b0);assign queue_data_o=owner==0?rqd:dqd;
 assign req_valid_o=registration_ddr?rq:owner==1?eq:owner==2?dq:1'b0;
 assign req_write_o=registration_ddr?rw:owner==1?ew:dw;
 assign req_addr_o=registration_ddr?raddr:owner==1?eaddr:daddr;
 assign req_data_o=registration_ddr?rdata:owner==1?edata:ddata;
 assign rsp_ready_o=registration_ddr?rs:owner==1?es:owner==2?ds:1'b0;
 assign region_start_o=state==PSTART&&pr&&region_idle_i&&!stop;
 part_registration_records registration(
 .clk_i(clk_i),.rst_i(rst_i),.abort_i(stop),.start_i(state==RSTART&&!stop),.ready_o(rr),.busy_o(rb),.result_valid_o(rv),.out_ready_i(state==RWAIT),
 .ref_base_i(ref_base_i),.obs_base_i(obs_base_i),.task_id_i(task_id_i),.frame_id_i(frame_id_i),.reference_id_i(reference_id_i),.config_epoch_i(config_epoch_i),
 .error_o(re),.count_o(records),.ambiguous_o(ra),.truncated_o(rt),.cycles_o(),
 .domain_x0_o(x0),.domain_y0_o(y0),.domain_x1_o(x1),.domain_y1_o(y1),
 .gray_addr_o(rga),.gray_we_o(rgw),.gray_data_o(rgd),.gray_data_i(gray_data_i),
 .queue_addr_o(rqa),.queue_we_o(rqw),.queue_data_o(rqd),.queue_data_i(queue_data_i),
 .sample_external_i(owner!=0),.sample_start_i(ss&&owner==2),.sample_accept_i(sa),.sample_abort_i(stop||sabort||owner!=2),.sample_query_i(sq),
 .sample_ready_o(sready),.sample_busy_o(sbusy),.sample_valid_o(sv),.sample_reply_o(reply),.invscale_q16_o(),
 .req_valid_o(rq),.req_ready_i(registration_ddr&&req_ready_i),.req_write_o(rw),.req_addr_o(raddr),.req_data_o(rdata),
 .rsp_valid_i(registration_ddr&&rsp_valid_i),.rsp_ready_o(rs),.rsp_error_i(rsp_error_i),.rsp_data_i(rsp_data_i));
 part_pose_export exporter(.clk_i(clk_i),.rst_i(rst_i),.abort_i(stop),.start_i(state==ESTART&&!stop),.ready_o(er),.busy_o(eb),.valid_o(ev),.out_ready_i(state==EWAIT),
 .record_count_i(records),.task_id_i(task_id_i),.frame_id_i(frame_id_i),.reference_id_i(reference_id_i),.config_epoch_i(config_epoch_i),.error_o(ee),.count_o(poses),
 .gray_addr_o(ega),.gray_data_i(gray_data_i),
 .req_valid_o(eq),.req_ready_i(owner==1&&req_ready_i),.req_write_o(ew),.req_addr_o(eaddr),.req_data_o(edata),
 .rsp_valid_i(owner==1&&rsp_valid_i),.rsp_ready_o(es),.rsp_error_i(rsp_error_i));
 part_difference_frame difference(.clk_i(clk_i),.rst_i(rst_i),.abort_i(stop),.start_i(state==DSTART&&!stop),.ready_o(dr),.busy_o(db),.valid_o(dv),.out_ready_i(state==DWAIT),
 .pose_count_i(poses),.domain_x0_i(x0),.domain_y0_i(y0),.domain_x1_i(x1),.domain_y1_i(y1),
 .ref_base_i(ref_base_i),.obs_base_i(obs_base_i),.task_id_i(task_id_i),.frame_id_i(frame_id_i),.reference_id_i(reference_id_i),.config_epoch_i(config_epoch_i),
 .error_o(de),.cycles_o(),.tiles_o(),.candidate_points_o(candidate_points),.clean_points_o(clean_points),
 .gray_addr_o(dga),.gray_we_o(dgw),.gray_data_o(dgd),.gray_data_i(gray_data_i),.state_addr_o(state_addr_o),.state_we_o(state_we_o),.state_data_o(state_data_o),.state_data_i(state_data_i),
 .queue_addr_o(dqa),.queue_we_o(dqw),.queue_data_o(dqd),.queue_data_i(queue_data_i),
 .sample_start_o(ss),.sample_accept_o(sa),.sample_abort_o(sabort),.sample_query_o(sq),.sample_ready_i(sready),.sample_busy_i(sbusy),.sample_valid_i(sv),.sample_reply_i(reply),
 .req_valid_o(dq),.req_ready_i(owner==2&&!sample_owner&&req_ready_i),.req_write_o(dw),.req_addr_o(daddr),.req_data_o(ddata),
 .rsp_valid_i(owner==2&&!sample_owner&&rsp_valid_i),.rsp_ready_o(ds),.rsp_error_i(rsp_error_i),.rsp_data_i(rsp_data_i));
 part_region_publish publisher(.clk_i(clk_i),.rst_i(rst_i),.abort_i(stop),.start_i(region_start_o),.ready_o(pr),.busy_o(pb),
 .task_id_i(task_id_i),.frame_id_i(frame_id_i),.frame_crc_i(frame_crc_i),.reference_id_i(reference_id_i),
 .region_valid_i(region_valid_i),.region_ready_o(region_ready_o),.region_abort_o(pa),
 .x0_i(region_x0_i),.x1_i(region_x1_i),.y0_i(region_y0_i),.y1_i(region_y1_i),
 .regions_done_i(region_done_i),.regions_count_i(region_count_i),.regions_error_i(region_error_i),
 .cmd_valid_o(cmd_valid_o),.cmd_ready_i(cmd_ready_i),.cmd_opcode_o(cmd_opcode_o),.cmd_flags_o(cmd_flags_o),.cmd_data_o(cmd_data_o),.cmd_request_o(cmd_request_o),
 .response_valid_i(response_valid_i),.response_ready_o(response_ready_o),.response_status_i(response_status_i),
 .result_valid_o(pv),.out_ready_i(state==PWAIT),.error_o(pe),.count_o(count_o));
 always@(posedge clk_i)begin
  if(rst_i)begin state<=IDLE;owner<=0;error_o<=0;phase_o<=10;end
  else if(abort_i&&state!=IDLE&&state!=DRAIN)begin state<=DRAIN;error_o<=3;end
  else case(state)
   IDLE:if(start_i&&ready_o)begin owner<=0;error_o<=0;phase_o<=10;state<=RSTART;end
   RSTART:if(rr)state<=RWAIT;
   RWAIT:if(rv)begin
    if(re!=0||ra||rt||records==0)begin error_o<=re!=0?re:4'd5;state<=RESULT;end
    else begin owner<=1;state<=ESTART;end
   end
   ESTART:if(er)state<=EWAIT;
   EWAIT:if(ev)begin
    if(ee!=0)begin error_o<=ee;state<=RESULT;end
    else begin owner<=2;phase_o<=12;state<=DSTART;end
   end
   DSTART:if(dr)state<=DWAIT;
   DWAIT:if(dv)begin
    if(de!=0)begin error_o<=de;state<=RESULT;end
    else begin owner<=3;phase_o<=15;state<=PSTART;end
   end
   PSTART:if(region_start_o)state<=PWAIT;
   PWAIT:if(pv)begin error_o<=pe;phase_o<=count_o==0?4'd14:4'd13;state<=RESULT;end
   RESULT:if(out_ready_i)state<=IDLE;
   DRAIN:if(!rb&&!eb&&!db&&!pb&&!sbusy)begin state<=IDLE;owner<=0;end
   default:state<=IDLE;
  endcase
 end
endmodule
