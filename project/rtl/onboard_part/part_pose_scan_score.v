`timescale 1ns/1ps
// C3a candidate backend. Candidate generation/selection is not implemented here.
// One owner, one DDR request at a time, no private frame/line memory.
module part_pose_scan_score #(parameter integer WIDTH=1024, HEIGHT=600, STATIC_CONTEXT=0, MIN_STRIDE_LOG2=0)(

 input wire sample_external_i,sample_start_i,sample_accept_i,sample_abort_i,
 input wire[194:0]sample_query_i,
 output wire sample_ready_o,sample_busy_o,sample_valid_o,output wire[75:0]sample_reply_o,
 output wire[16:0]invscale_q16_o,
 input wire clk_i,rst_i,abort_i,start_i, output wire ready_o,busy_o,
 input wire signed[17:0] c_q16_i,d_q16_i,
 input wire signed[28:0] offset_x_q16_i,offset_y_q16_i,
 input wire[31:0] ref_base_i,obs_base_i,
 input wire[2:0] stride_log2_i,
 input wire[31:0] task_id_i,frame_id_i,reference_id_i,config_epoch_i,
 output wire done_o,result_valid_o,input wire out_ready_i,output reg[3:0] error_o,
 output wire[31:0] task_id_o,frame_id_o,reference_id_o,config_epoch_o,
 output wire[19:0] attempted_count_o,valid_count_o,ref_strong_count_o,ref_weak_count_o,
 output wire[19:0] obs_strong_count_o,obs_weak_count_o,joint_strong_count_o,direction_count_o,
 output wire[19:0] ref_clipped_count_o,ref_low_contrast_count_o,obs_clipped_count_o,obs_low_contrast_count_o,
 output wire bbox_valid_o,output wire[9:0] min_x_o,min_y_o,max_x_o,max_y_o,output wire[15:0] grid_o,
 output reg[31:0] cycles_o,read_requests_o,
 output wire req_valid_o,input wire req_ready_i,output wire req_write_o,
 output wire[31:0] req_addr_o,output wire[127:0] req_data_o,
 input wire rsp_valid_i,output wire rsp_ready_o,input wire rsp_error_i,input wire[127:0] rsp_data_i
);
 // Parent keeps all identity buses stable until result consumption or drain.
 reg[31:0]task_id_held;assign task_id_o=STATIC_CONTEXT?task_id_i:task_id_held;
 reg[31:0]frame_id_held;assign frame_id_o=STATIC_CONTEXT?frame_id_i:frame_id_held;
 reg[31:0]reference_id_held;assign reference_id_o=STATIC_CONTEXT?reference_id_i:reference_id_held;
 reg[31:0]config_epoch_held;assign config_epoch_o=STATIC_CONTEXT?config_epoch_i:config_epoch_held;

 localparam IDLE=0,NORM_C=1,NORM_D=2,ROOT_INIT=3,ROOT_STEP=4,NORM_CHECK=5,
 SCORE_START=6,SAMPLE_START=7,SAMPLE_WAIT=8,SCORE_WAIT=9,RESULT_WAIT=10,RESULT=11,DRAIN=12;
 reg[3:0] state;
 reg signed[17:0] c_held,d_held;
 wire signed[17:0] c=STATIC_CONTEXT?c_q16_i:c_held;
 wire signed[17:0] d=STATIC_CONTEXT?d_q16_i:d_held;
 reg signed[28:0] ox_held,oy_held;
 wire signed[28:0] ox=STATIC_CONTEXT?offset_x_q16_i:ox_held;
 wire signed[28:0] oy=STATIC_CONTEXT?offset_y_q16_i:oy_held;
 reg [31:0] rb_held,ob_held;
 wire [31:0] rb=STATIC_CONTEXT?ref_base_i:rb_held;
 wire [31:0] ob=STATIC_CONTEXT?obs_base_i:ob_held;
 reg[5:0] stride;
 reg[9:0] x,y;
 reg[35:0] norm;
 reg[16:0] root,invscale;
 reg[4:0] root_bit;
 wire[16:0] trial=root|(17'd1<<root_bit);
 reg signed[17:0] norm_operand;
 always@*begin
  norm_operand=$signed({1'b0,root});
  if(state==NORM_C)norm_operand=c;
  else if(state==NORM_D)norm_operand=d;
  else if(state==ROOT_STEP)norm_operand=$signed({1'b0,trial});
 end
 wire signed[35:0] norm_product=norm_operand*norm_operand;
 wire[17:0] nearest_root={1'b0,root}+((norm-$unsigned(norm_product))>{19'd0,root});
 wire context_lost=(task_id_i!=task_id_o)||(frame_id_i!=frame_id_o)||
                   (reference_id_i!=reference_id_o)||(config_epoch_i!=config_epoch_o);
 wire cancel=abort_i||((state!=IDLE)&&(state!=DRAIN)&&context_lost);
 wire module_abort=cancel||state==DRAIN;
 wire sample_interior;
 assign invscale_q16_o=invscale;
 wire sample_ready,sample_busy,sample_done,sample_out_valid,sample_in_domain;
 wire[3:0] sample_error;
 wire signed[10:0] ogx,ogy;wire[7:0] oraw;wire[1:0] oflags,rflags;
 wire signed[12:0] rgx,rgy;wire[9:0] rraw;
 wire[9:0] sx,sy;wire[31:0] sid,sframe,sref,sepoch;
 assign sample_ready_o=sample_ready;assign sample_busy_o=sample_busy;assign sample_valid_o=sample_out_valid;
 assign sample_reply_o={sample_error,sample_in_domain,sample_interior,ogx,ogy,oraw,oflags,rgx,rgy,rraw,rflags};
 wire score_ready,score_sample_ready,score_result,score_done,score_busy;
 wire[10:0] next_x={1'b0,x}+stride,next_y={1'b0,y}+stride;
 wire last=(next_x>=WIDTH)&&(next_y>=HEIGHT);
 wire id_ok=sid==task_id_o&&sframe==frame_id_o&&sref==reference_id_o&&sepoch==config_epoch_o;
 wire send_sample=state==SAMPLE_WAIT&&sample_out_valid&&sample_error==0&&id_ok&&!cancel;
 wire consume_sample=state==SAMPLE_WAIT&&
     ((sample_error!=0||!id_ok)||score_sample_ready)&&!cancel;
 assign ready_o=state==IDLE&&!rst_i&&!abort_i;
 assign busy_o=state!=IDLE;
 assign result_valid_o=state==RESULT&&!rst_i&&!cancel;
 assign done_o=result_valid_o;
 part_pose_sample #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.STATIC_CONTEXT(1)) sample(
  .clk_i(clk_i),.rst_i(rst_i),.abort_i(sample_external_i?sample_abort_i:module_abort),
  .start_i(sample_external_i?sample_start_i:state==SAMPLE_START&&!cancel),.ready_o(sample_ready),.busy_o(sample_busy),
  .done_o(sample_done),.valid_o(sample_out_valid),.out_ready_i(sample_external_i?sample_accept_i:consume_sample),
  .sample_valid_o(sample_in_domain),.interior_o(sample_interior),.error_o(sample_error),
  .c_q16_i(sample_external_i?sample_query_i[130:113]:c),.d_q16_i(sample_external_i?sample_query_i[112:95]:d),.offset_x_q16_i(sample_external_i?sample_query_i[94:66]:ox),.offset_y_q16_i(sample_external_i?sample_query_i[65:37]:oy),.invscale_q16_i(sample_external_i?sample_query_i[36:20]:invscale),
  .ref_base_i(sample_external_i?sample_query_i[194:163]:rb),.obs_base_i(sample_external_i?sample_query_i[162:131]:ob),.obs_x_i(sample_external_i?sample_query_i[19:10]:x),.obs_y_i(sample_external_i?sample_query_i[9:0]:y),
  .task_id_i(task_id_o),.frame_id_i(frame_id_o),.reference_id_i(reference_id_o),.config_epoch_i(config_epoch_o),
  .task_id_o(sid),.frame_id_o(sframe),.reference_id_o(sref),.config_epoch_o(sepoch),
  .obs_x_o(sx),.obs_y_o(sy),.obs_gx_q4_o(ogx),.obs_gy_q4_o(ogy),.obs_raw_q2_o(oraw),.obs_flags_o(oflags),
  .ref_gx_q4_o(rgx),.ref_gy_q4_o(rgy),.ref_raw_q2_o(rraw),.ref_flags_o(rflags),
  .ref_x_q10_o(),.ref_y_q10_o(),.read_count_o(),
  .req_valid_o(req_valid_o),.req_ready_i(req_ready_i),.req_write_o(req_write_o),.req_addr_o(req_addr_o),.req_data_o(req_data_o),
  .rsp_valid_i(rsp_valid_i),.rsp_ready_o(rsp_ready_o),.rsp_error_i(rsp_error_i),.rsp_data_i(rsp_data_i));
 part_pose_score #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.COUNTER_WIDTH($clog2(((WIDTH+(1<<MIN_STRIDE_LOG2)-1)>>MIN_STRIDE_LOG2)*((HEIGHT+(1<<MIN_STRIDE_LOG2)-1)>>MIN_STRIDE_LOG2)+1))) score(
  .clk(clk_i),.rst(rst_i),.abort_i(module_abort),.start_i(state==SCORE_START&&!cancel),.ready_o(score_ready),
  .task_id_i(task_id_o),.frame_id_i(frame_id_o),.reference_id_i(reference_id_o),.config_epoch_i(config_epoch_o),
  .task_id_o(),.frame_id_o(),.reference_id_o(),.config_epoch_o(),
  .sample_valid_i(send_sample),.sample_ready_o(score_sample_ready),.in_domain_i(sample_in_domain),
  .obs_x_i(sx),.obs_y_i(sy),.ref_gx_i(rgx),.ref_gy_i(rgy),.ref_raw_i(rraw),.ref_flags_i(rflags),
  .obs_gx_i(ogx),.obs_gy_i(ogy),.obs_raw_i(oraw),.obs_flags_i(oflags),.last_i(last),
  .done_o(score_done),.result_valid_o(score_result),.out_ready_i(state==RESULT&&out_ready_i&&!cancel),.busy_o(score_busy),
  .attempted_count_o(attempted_count_o),.valid_count_o(valid_count_o),
  .ref_strong_count_o(ref_strong_count_o),.ref_weak_count_o(ref_weak_count_o),
  .obs_strong_count_o(obs_strong_count_o),.obs_weak_count_o(obs_weak_count_o),
  .joint_strong_count_o(joint_strong_count_o),.direction_count_o(direction_count_o),
  .ref_clipped_count_o(ref_clipped_count_o),.ref_low_contrast_count_o(ref_low_contrast_count_o),
  .obs_clipped_count_o(obs_clipped_count_o),.obs_low_contrast_count_o(obs_low_contrast_count_o),
  .bbox_valid_o(bbox_valid_o),.min_x_o(min_x_o),.min_y_o(min_y_o),.max_x_o(max_x_o),.max_y_o(max_y_o),.grid_o(grid_o));
 always@(posedge clk_i)begin
  if(rst_i)begin
   state<=IDLE;c_held<=0;d_held<=0;ox_held<=0;oy_held<=0;rb_held<=0;ob_held<=0;stride<=1;x<=0;y<=0;norm<=0;root<=0;root_bit<=0;invscale<=0;
   task_id_held<=0;frame_id_held<=0;reference_id_held<=0;config_epoch_held<=0;error_o<=0;cycles_o<=0;read_requests_o<=0;
  end else begin
   if(state!=IDLE&&state!=RESULT&&state!=DRAIN)cycles_o<=cycles_o+1'b1;
   if(!sample_external_i&&req_valid_o&&req_ready_i)read_requests_o<=read_requests_o+1'b1;
   if(state!=IDLE&&state!=DRAIN&&cancel)begin state<=DRAIN;error_o<=3;end
   else case(state)
    IDLE:if(start_i&&ready_o)begin
     c_held<=c_q16_i;d_held<=d_q16_i;ox_held<=offset_x_q16_i;oy_held<=offset_y_q16_i;
     rb_held<=ref_base_i;ob_held<=obs_base_i;stride<=6'd1<<stride_log2_i;x<=0;y<=0;
     task_id_held<=task_id_i;frame_id_held<=frame_id_i;reference_id_held<=reference_id_i;config_epoch_held<=config_epoch_i;
     error_o<=0;cycles_o<=0;read_requests_o<=0;
     // Quantized pose domain plus address alignment. Full extents are checked by sampler.
     if(stride_log2_i<MIN_STRIDE_LOG2||stride_log2_i>5||ref_base_i[1:0]!=0||obs_base_i[1:0]!=0||
        c_q16_i>18'sd87383||c_q16_i< -18'sd87383||d_q16_i>18'sd87383||d_q16_i< -18'sd87383)
       begin error_o<=1;state<=RESULT;end
     else state<=NORM_C;
    end
    NORM_C:begin norm<=$unsigned(norm_product);state<=NORM_D;end
    NORM_D:begin norm<=norm+$unsigned(norm_product);state<=ROOT_INIT;end
    ROOT_INIT:begin root<=0;root_bit<=16;state<=ROOT_STEP;end
    ROOT_STEP:begin
     if($unsigned(norm_product)<=norm)root<=trial;
     if(root_bit==0)state<=NORM_CHECK;else root_bit<=root_bit-1'b1;
    end
    NORM_CHECK:begin
     // +/-1 encoded Q16 norm tolerance admits coefficient rounding at endpoint rotations.
     if(nearest_root<52428||nearest_root>87382)begin error_o<=1;state<=RESULT;end
     else begin
      invscale<=nearest_root<52429?17'd52429:(nearest_root>87381?17'd87381:nearest_root[16:0]);
      state<=SCORE_START;
     end
    end
    SCORE_START:if(score_ready)state<=SAMPLE_START;
    SAMPLE_START:if(sample_ready)state<=SAMPLE_WAIT;
    SAMPLE_WAIT:if(sample_out_valid&&consume_sample)begin
     if(sample_error!=0||!id_ok)begin error_o<=sample_error!=0?sample_error:4'd4;state<=DRAIN;end
     else state<=last?RESULT_WAIT:SCORE_WAIT;
    end
    SCORE_WAIT:if(score_sample_ready)begin
     if(next_x>=WIDTH)begin x<=0;y<=next_y[9:0];end else x<=next_x[9:0];
     state<=SAMPLE_START;
    end
    RESULT_WAIT:if(score_result)state<=RESULT;
    RESULT:if(out_ready_i)state<=IDLE;
    DRAIN:if(!sample_busy)begin
     // External abort/context loss has no publishable result. DDR error is reported after drain.
     if(error_o==3)state<=IDLE;else state<=RESULT;
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
