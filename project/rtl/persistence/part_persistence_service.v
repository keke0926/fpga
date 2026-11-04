`timescale 1ns/1ps
// PST1 service. All ports use the LCD clock. The demux owns packet framing;
// this service commits DATA only after its whole CRC has been verified.
module part_persistence_service #(parameter integer IDLE_TIMEOUT=480000000)(
 input wire clk,rst_n,rx_start_i,rx_valid_i,rx_error_i,input wire[7:0]rx_byte_i,
 input wire context_valid_i,begin_allowed_i,invalidate_i,
 input wire[31:0]locked_frame_i,locked_crc_i,input wire[15:0]frontend_i,
 input wire tx_grant_i,tx_ready_i,tx_idle_i,
 output wire tx_claim_o,tx_active_o,tx_valid_o,output wire[7:0]tx_byte_o
);
 function[31:0]crc_step;
 input[31:0]c;input[7:0]b;reg[31:0]v;integer i;
 begin v=c^{24'b0,b};for(i=0;i<8;i=i+1)v=v[0]?(v>>1)^32'hedb88320:v>>1;crc_step=v;end
 endfunction
 localparam TX_IDLE=0,TX_HEAD=1,TX_BODY=2,TX_CRC=3,TX_DRAIN=4;
 reg active,finished,input_pending,run_valid,flush_run;
 reg[7:0]fault;
 reg[31:0]job,frame,frame_crc,reference_id,tag,input_pixels,output_pixels,input_crc,output_crc;
 reg[10:0]image_width;reg[9:0]image_height;reg[31:0]total_pixels;
 reg[15:0]bound_frontend,next_sequence;
 reg[28:0]idle_timer;
 reg[6:0]fifo_count;
 reg[5:0]write_ptr,read_ptr;
 // Input packets and result FIFO share unused capacity of the original block.
 // Input words [0:127], result words [128:191]. One shared read and write
 // port, explicitly scheduled: packet reception wins writes; input decoding
 // wins reads. UART prefetch/kernel backpressure absorb the short conflicts.
 reg[5:0]run_value;reg[10:0]run_length;
 reg[1:0]decoder_state;reg[6:0]input_ptr;reg[7:0]words_left;
 reg[11:0]input_run;reg[4:0]input_value;

 reg[1:0]rx_state;reg[15:0]rx_index,rx_length;
 reg[127:0]rx_header;reg[159:0]config_payload;
 reg[31:0]rx_crc,rx_trailer;reg[7:0]rx_low;
 reg rx_store,rx_complete,rx_crc_ok;
 wire[7:0]opcode=rx_header[39:32];wire[31:0]request_job=rx_header[95:64];
 wire[15:0]packet_sequence=rx_header[111:96];
 wire envelope_ok=rx_header[31:0]==32'h31545350 && rx_header[47:40]==0 && rx_header[127:112]==0 &&
                  rx_length<=256 && request_job!=0 && opcode>=1 && opcode<=5;
 (* ram_style="block" *) reg[15:0]packet_memory[0:255];
 reg[15:0]packet_q;
 wire[15:0]input_q=packet_q;
 wire input_write=rx_valid_i && rx_state==1 && opcode==2 && rx_store && rx_length<=256 && rx_index[0];

 reg response_pending;reg[2:0]tx_state;reg[8:0]tx_index;
 reg[127:0]response_header;reg[319:0]response_data;
 reg[5:0]response_base;reg[31:0]tx_crc,tx_trailer;
 wire done_flag=finished&&!run_valid&&!input_pending;
 reg response_word_valid,response_fetch_pending;
 reg[15:0]response_word;
 wire[7:0]live_flags={3'b0,(fault!=0),(fifo_count!=0),done_flag,input_pending,active};
 assign tx_claim_o=response_pending||tx_state!=TX_IDLE;
 assign tx_active_o=tx_state!=TX_IDLE;
 assign tx_valid_o=tx_state==TX_HEAD||(tx_state==TX_BODY&&(response_header[39:32]!=3||response_word_valid))||tx_state==TX_CRC;
 wire[5:0]response_address=response_base+tx_index[7:1];
 assign tx_byte_o=tx_state==TX_HEAD?response_header[tx_index*8+:8]:
   tx_state==TX_BODY?(response_header[39:32]==3?(tx_index[0]?response_word[15:8]:response_word[7:0]):response_data[tx_index*8+:8]):
   tx_state==TX_CRC?tx_trailer[tx_index*8+:8]:8'b0;
 wire word_sent=tx_state==TX_BODY&&tx_valid_o&&tx_ready_i&&response_header[39:32]==3&&tx_index[0];
 wire pop=word_sent&&fifo_count!=0;

 reg kernel_start,kernel_cancel;
 wire kernel_busy,kernel_done,kernel_error,ki_ready,ko_valid,ko_last;
 wire[5:0]ko_data;wire[10:0]ko_x;wire[9:0]ko_y;wire[31:0]ko_tag;
 wire ki_valid=active&&input_pending&&decoder_state==2;
 wire ko_ready=active&&!input_write&&!flush_run&&(!run_valid||(ko_data==run_value&&run_length<1024)||fifo_count<64);
 wire ko_accept=ko_valid&&ko_ready;
 wire changed_run=run_valid&&(ko_data!=run_value||run_length==1024);
 wire push=!input_write&&((active&&flush_run&&run_valid&&fifo_count<64)||(ko_accept&&changed_run));
 wire[9:0]encoded_length=run_length-1'b1;
 wire input_read=input_pending&&decoder_state==0;
 wire[7:0]read_address=input_read?{1'b0,input_ptr}:{2'b10,response_address};
 wire[7:0]write_address=input_write?{1'b0,rx_index[7:1]}:{2'b10,write_ptr};
 wire[15:0]write_data=input_write?{rx_byte_i,rx_low}:{run_value,encoded_length};
 wire response_fetch=tx_state==TX_BODY&&response_header[39:32]==3&&!response_word_valid&&!response_fetch_pending&&!input_read;
 always @(posedge clk)begin
  if(input_read||response_fetch)packet_q<=packet_memory[read_address];
  if(input_write||push)packet_memory[write_address]<=write_data;
 end
 part_persistence_kernel kernel(.clk(clk),.rst(!rst_n),.cancel(kernel_cancel),.start(kernel_start),
  .width(image_width),.height(image_height),.job_tag(tag),.busy(kernel_busy),.done(kernel_done),.error(kernel_error),
  .in_valid(ki_valid),.in_ready(ki_ready),.in_data(input_value),.out_valid(ko_valid),.out_ready(ko_ready),
  .out_data(ko_data),.out_x(ko_x),.out_y(ko_y),.out_tag(ko_tag),.out_last(ko_last));

 always @(posedge clk)begin
  if(!rst_n||rx_start_i||rx_error_i)begin
   rx_state<=0;rx_index<=0;rx_length<=0;rx_header<=0;config_payload<=0;rx_crc<=32'hffffffff;
   rx_trailer<=0;rx_low<=0;rx_store<=0;rx_complete<=0;rx_crc_ok<=0;
  end else begin
   rx_complete<=0;
   if(rx_valid_i)case(rx_state)
    0:begin
     rx_header[rx_index*8+:8]<=rx_byte_i;rx_crc<=crc_step(rx_crc,rx_byte_i);
     if(rx_index==15)begin
      rx_length<=rx_header[63:48];rx_store<=!input_pending&&!response_pending&&tx_state==TX_IDLE;
      rx_index<=0;rx_state<=rx_header[63:48]==0?2:1;
     end else rx_index<=rx_index+1'b1;
    end
    1:begin
     rx_crc<=crc_step(rx_crc,rx_byte_i);rx_low<=rx_byte_i;
     if(opcode==1&&rx_index<20)config_payload[rx_index*8+:8]<=rx_byte_i;
     if(rx_index+1==rx_length)begin rx_index<=0;rx_state<=2;end else rx_index<=rx_index+1'b1;
    end
    2:begin
     rx_trailer[rx_index*8+:8]<=rx_byte_i;
     if(rx_index==3)begin rx_complete<=1;rx_crc_ok<={rx_byte_i,rx_trailer[23:0]}==~rx_crc;rx_state<=3;end
     else rx_index<=rx_index+1'b1;
    end
    default:begin end
   endcase
  end
 end

 task reply;
 input[7:0]status;input[15:0]length;
 begin response_pending<=1;response_header<={8'd0,live_flags,packet_sequence,request_job,length,status,opcode,32'h31525350};
  response_data<=0;response_base<=read_ptr;response_word_valid<=0;response_fetch_pending<=0;
 end endtask
 task stop_job;
 input[7:0]reason;
 begin
  active<=0;finished<=0;input_pending<=0;decoder_state<=0;kernel_cancel<=1;
  fault<=reason;run_valid<=0;flush_run<=0;fifo_count<=0;write_ptr<=0;read_ptr<=0;idle_timer<=0;
 end endtask

 always @(posedge clk)begin
  if(!rst_n)begin
   active<=0;finished<=0;input_pending<=0;run_valid<=0;flush_run<=0;fault<=0;
   job<=0;frame<=0;frame_crc<=0;reference_id<=0;tag<=0;image_width<=0;image_height<=0;total_pixels<=0;
   bound_frontend<=0;next_sequence<=1;input_pixels<=0;output_pixels<=0;input_crc<=32'hffffffff;output_crc<=32'hffffffff;
   fifo_count<=0;write_ptr<=0;read_ptr<=0;run_value<=0;run_length<=0;idle_timer<=0;
   decoder_state<=0;input_ptr<=0;words_left<=0;input_run<=0;input_value<=0;
   kernel_start<=0;kernel_cancel<=0;
   response_pending<=0;response_header<=0;response_data<=0;response_base<=0;
   response_word<=0;response_word_valid<=0;response_fetch_pending<=0;
   tx_state<=TX_IDLE;tx_index<=0;tx_crc<=32'hffffffff;tx_trailer<=0;
  end else begin
   kernel_start<=0;kernel_cancel<=0;
   if(response_fetch)response_fetch_pending<=1;
   if(response_fetch_pending)begin response_word<=packet_q;response_word_valid<=1;response_fetch_pending<=0;end
   if(word_sent)response_word_valid<=0;
   if(active)idle_timer<=idle_timer+1'b1;else idle_timer<=0;
   case({push,pop})
    2'b10:fifo_count<=fifo_count+1'b1;
    2'b01:fifo_count<=fifo_count-1'b1;
    default:begin end
   endcase
   if(push)write_ptr<=write_ptr+1'b1;
   if(pop)read_ptr<=read_ptr+1'b1;
   if(active&&flush_run&&push)begin run_valid<=0;flush_run<=0;end
   if(ko_accept)begin
    output_pixels<=output_pixels+1'b1;output_crc<=crc_step(output_crc,{2'b0,ko_data});idle_timer<=0;
    if(!run_valid||changed_run)begin run_value<=ko_data;run_length<=1;run_valid<=1;end
    else run_length<=run_length+1'b1;
    if(ko_last)flush_run<=1;
   end
   if(kernel_done&&active)finished<=1;
   if(active&&input_pending)case(decoder_state)
    0:decoder_state<=1;
    1:begin
     if(input_pixels+{21'b0,input_q[10:0]}+1>total_pixels)stop_job(4);
     else begin input_run<={1'b0,input_q[10:0]}+1'b1;input_value<=input_q[15:11];decoder_state<=2;end
    end
    2:if(ki_ready)begin
     input_pixels<=input_pixels+1'b1;input_crc<=crc_step(input_crc,{3'b0,input_value});idle_timer<=0;
     if(input_run==1)begin
      decoder_state<=0;
      if(words_left==1)input_pending<=0;
      else begin words_left<=words_left-1'b1;input_ptr<=input_ptr+1'b1;end
     end else input_run<=input_run-1'b1;
    end
    default:stop_job(4);
   endcase
   case(tx_state)
    TX_IDLE:if(response_pending&&tx_grant_i&&tx_idle_i)begin response_pending<=0;tx_state<=TX_HEAD;tx_index<=0;tx_crc<=32'hffffffff;end
    TX_HEAD:if(tx_ready_i)begin
     tx_crc<=crc_step(tx_crc,tx_byte_o);
     if(tx_index==15)begin tx_index<=0;
      if(response_header[63:48]==0)begin tx_trailer<=~crc_step(tx_crc,tx_byte_o);tx_state<=TX_CRC;end
      else tx_state<=TX_BODY;
     end else tx_index<=tx_index+1'b1;
    end
    TX_BODY:if(tx_valid_o&&tx_ready_i)begin
     tx_crc<=crc_step(tx_crc,tx_byte_o);
     if(tx_index+1==response_header[63:48])begin tx_index<=0;tx_trailer<=~crc_step(tx_crc,tx_byte_o);tx_state<=TX_CRC;end
     else tx_index<=tx_index+1'b1;
    end
    TX_CRC:if(tx_ready_i)begin if(tx_index==3)tx_state<=TX_DRAIN;else tx_index<=tx_index+1'b1;end
    TX_DRAIN:if(tx_idle_i)begin tx_state<=TX_IDLE;tx_index<=0;end
    default:tx_state<=TX_IDLE;
   endcase
   if(rx_complete&&!response_pending&&tx_state==TX_IDLE)begin
    if(!rx_crc_ok)reply(3,0);
    else if(!envelope_ok)reply(1,0);
    else begin
     idle_timer<=0;
     if(opcode==1)begin
      if(rx_length!=20||packet_sequence!=0)reply(1,0);
      else if(!context_valid_i||!begin_allowed_i||invalidate_i||
        (active&&(!done_flag||fifo_count!=0))||
        config_payload[63:32]!=locked_frame_i||config_payload[95:64]!=locked_crc_i)reply(2,0);
      else if(config_payload[15:0]==0||config_payload[15:0]>1024||config_payload[31:16]==0||config_payload[31:16]>600||
              config_payload[63:32]==0||config_payload[127:96]==0||config_payload[159:128]==0)reply(1,0);
      else begin
       active<=1;finished<=0;fault<=0;job<=request_job;frame<=locked_frame_i;frame_crc<=locked_crc_i;
       reference_id<=config_payload[127:96];tag<=config_payload[159:128];bound_frontend<=frontend_i;
       image_width<=config_payload[10:0];image_height<=config_payload[25:16];
       total_pixels<=config_payload[15:0]*config_payload[31:16];next_sequence<=1;
       input_pixels<=0;output_pixels<=0;input_crc<=32'hffffffff;output_crc<=32'hffffffff;
       input_pending<=0;decoder_state<=0;run_valid<=0;flush_run<=0;fifo_count<=0;write_ptr<=0;read_ptr<=0;
       kernel_start<=1;reply(0,0);response_header[119:112]<=1;
      end
     end else if(request_job!=job)reply(2,0);
     else case(opcode)
      2:begin
       if(!active||fault!=0||input_pending||!rx_store||input_pixels>=total_pixels)reply(2,0);
       else if(rx_length==0||rx_length[0]||packet_sequence!=next_sequence)reply(1,0);
       else begin input_pending<=1;input_ptr<=0;words_left<=rx_length[8:1];decoder_state<=0;
        next_sequence<=next_sequence+1'b1;reply(0,0);response_header[119:112]<=live_flags|2;
       end
      end
      3:if(rx_length!=0)reply(1,0);else if(fault!=0)reply(fault,0);else reply(0,{8'b0,fifo_count,1'b0});
      4:if(rx_length!=0)reply(1,0);else begin
       reply(fault,40);response_data<={{6'b0,image_height,5'b0,image_width},{9'b0,fifo_count,next_sequence},
        ~output_crc,~input_crc,output_pixels,input_pixels,tag,reference_id,frame_crc,frame};
      end
      5:if(rx_length!=0)reply(1,0);else begin stop_job(0);reply(0,0);response_header[119:112]<=0;end
      default:reply(1,0);
     endcase
    end
   end
   if(active&&(!context_valid_i||invalidate_i||frame!=locked_frame_i||frame_crc!=locked_crc_i||bound_frontend!=frontend_i))stop_job(8);
   else if(active&&idle_timer>=IDLE_TIMEOUT-1)stop_job(9);
  end
 end
endmodule
