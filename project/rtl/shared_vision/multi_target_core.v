`timescale 1ns/1ps
// Full sampled display-frame transaction; independent video is never stalled.
// Exact 8-connected components, bounded circular pixel frontier, RAM record queue.
// All qualified components are queued in raster order and use one classifier.
// Record RAM has 2048 entries: floor(256*150/20)=1920 is a conservative bound.
// Only counters and small classifier maps reset; pixel/record memories do not.
module multi_target_core #(
    parameter TEMPLATE_FILE="rtl/apps/digit_variants_14.mem",
    parameter SHAPE_FILE="rtl/apps/geometry_templates.mem",
    parameter DIGIT_MIRROR_X=1,parameter ENABLE_LOCAL=0,parameter PART_MIN_SUPPORT=32
 )(
    input wire local_grant_i,local_queue_we_i,input wire[11:0]local_queue_addr_i,
    input wire[15:0]local_queue_data_i,output wire[15:0]local_queue_data_o,
    input wire clk,rst_n,abort_i,start_i,
    // Reuse this BFS for local comparison counts already in shared gray RAM.
    // Each byte: [4:0] native support (0..16), [7] uncertain evidence.
    input wire regions_i,region_ready_i,
    output wire region_valid_o,output reg [19:0] region_support_o,
    output reg region_uncertain_o,
    input wire [31:0] frame_id_i,config_tag_i,
    input wire sample_valid_i,input wire [15:0] sample_addr_i,
    input wire [7:0] sample_gray_i,input wire end_i,capture_error_i,
    output wire ready_o,busy_o,
    output reg object_valid_o,output reg [10:0] object_index_o,
    output reg [4:0] object_class_o,output reg object_accepted_o,
    output reg [3:0] object_reason_o,
    output reg [8:0] object_x0_o,object_x1_o,
    output reg [7:0] object_y0_o,object_y1_o,
    output reg [15:0] object_area_o,output reg [13:0] object_score_o,
    output reg preview_write_o,output reg [15:0] preview_addr_o,output reg preview_bit_o,
    output reg descriptor_we_o,output reg [10:0] descriptor_addr_o,
    output reg [47:0] descriptor_data_o,input wire [47:0] descriptor_data_i,
    output reg annotation_write_o,output reg [15:0] annotation_addr_o,
    output reg result_valid_o,
    output reg [31:0] result_frame_id_o,result_config_tag_o,
    output reg [11:0] result_count_o,
    output reg [15:0] result_candidate_count_o,result_declined_count_o,result_omitted_count_o,
    output reg result_overflow_o,output reg [3:0] result_reason_o,
    output reg [7:0] threshold_o,output reg [31:0] processing_cycles_o,
    output wire [15:0] scratch_gray_addr_o,scratch_state_addr_o,
    output wire scratch_gray_we_o,output wire [1:0] scratch_state_we_o,
    output wire [7:0] scratch_gray_data_o,output wire [1:0] scratch_state_data_o,
    input wire [7:0] scratch_gray_data_i,input wire [1:0] scratch_state_data_i
);
    localparam IDLE=0,CAPTURE=1,THRESHOLD_READ=2,THRESHOLD_WRITE=3,
      SCAN_READ=4,SCAN_CHECK=5,SEED=6,POP_READ=7,POP_LOAD=8,
      NEIGHBOR_READ=9,NEIGHBOR_CHECK=10,COMPONENT_DONE=11,
      TARGET_CHECK=12,FIT_START=14,FIT_DONE=15,XMAP_START=16,XMAP_DONE=17,
      YMAP_START=18,YMAP_DONE=19,NORMAL_START=20,NORMAL_READ=21,NORMAL_ACCUMULATE=22,
      SCORE_START=23,SCORE_DONE=24,CLASSIFY=25,PUBLISH=26,DIVIDE=27,DIVIDE_DONE=28,
      SELECT_OBJECT=29,CLEAR_MASK=30,GEOM_START=31,GEOM_READ=32,GEOM_CHECK=33,
      SHAPE_MAP=34,SHAPE_START=35,SHAPE_READ=36,SHAPE_ACCUMULATE=37,
      SHAPE_SCORE_START=38,SHAPE_SCORE_DONE=39,EMIT_OBJECT=40,DIGIT_RANK=41,
      LOAD_OBJECT=42,ANNOTATE_TOP=43,ANNOTATE_BOTTOM=44,ANNOTATE_LEFT=45,
      ANNOTATE_RIGHT=46,ANNOTATE_DRAIN=47,REGION_READ=48,REGION_MASK=49,REGION_EMIT=50;
    reg [5:0] state,divide_return;
    reg region_mode;
    assign region_valid_o=rst_n&&!abort_i&&region_mode&&state==REGION_EMIT;
    assign ready_o=state==IDLE;
    assign busy_o=state!=IDLE;

    (* ram_style="block" *) reg [15:0] queue_memory[0:4095];
    (* ram_style="block" *) reg [13:0] template_memory[0:4607];
    initial $readmemh(TEMPLATE_FILE,template_memory);
    reg [15:0] gray_address,mask_address;
    reg [11:0] queue_address;reg [12:0] template_address;
    reg gray_write,mask_write,queue_write,mask_write_data;
    reg [15:0] queue_write_data;wire [7:0] gray_read_data=scratch_gray_data_i;wire mask_read_data=scratch_state_data_i[0];
    reg [15:0] queue_read_data;reg [13:0] template_read_data;
    wire foreground_read_data=scratch_state_data_i[1];
    wire segment_busy,segment_ready,segment_done,segment_valid,segment_bit;
    wire [15:0] segment_gray_address,segment_mask_address;
    wire [31:0] segment_cycles;
    local_threshold33 u_local_threshold(
      .clk(clk),.rst_n(rst_n),.abort_i(abort_i),.start_i(state==THRESHOLD_READ),
      .ready_o(segment_ready),.busy_o(segment_busy),.gray_addr_o(segment_gray_address),
      .gray_data_i(gray_read_data),.mask_valid_o(segment_valid),.mask_addr_o(segment_mask_address),
      .mask_bit_o(segment_bit),.done_o(segment_done),.cycles_o(segment_cycles));
    // In THRESHOLD_WRITE mask_address is exactly segment_mask_address;
    // the immutable foreground and visited bit share an address but have
    // independent write enables, preserving the old synchronous RAM semantics.
    assign scratch_gray_addr_o=gray_address;
    assign scratch_state_addr_o=mask_address;
    assign scratch_gray_we_o=gray_write;
    assign scratch_state_we_o={rst_n && !abort_i && ((state==THRESHOLD_WRITE && segment_valid)||state==REGION_MASK),mask_write};
    assign scratch_gray_data_o=sample_gray_i;
    wire region_cell_valid=gray_read_data[4:0]!=0 && !gray_read_data[7];
    assign scratch_state_data_o={state==REGION_MASK?region_cell_valid:segment_bit,mask_write_data};
    wire local_owner=ENABLE_LOCAL&&local_grant_i;
    wire[11:0]leased_queue_addr=local_owner?local_queue_addr_i:queue_address;
    assign local_queue_data_o=queue_read_data;
    always @(posedge clk)begin
        queue_read_data<=queue_memory[leased_queue_addr];template_read_data<=template_memory[template_address];
        if(rst_n&&(local_owner?local_queue_we_i:(!abort_i&&queue_write)))
          queue_memory[leased_queue_addr]<=local_owner?local_queue_data_i:queue_write_data;
    end

    reg [15:0] capture_count,scan_address,component_seed;
    reg capture_bad,rebuild;reg [7:0] gray_min,gray_max;
    wire [8:0] minmax_sum={1'b0,gray_min}+{1'b0,gray_max};
    reg [31:0] cycle_count;
    reg [11:0] queue_head,queue_tail;reg [12:0] queue_count;
    reg [15:0] node;reg [2:0] neighbor_index;
    reg [8:0] neighbor_x;reg [8:0] neighbor_y;reg neighbor_valid;
    wire [15:0] neighbor_address={neighbor_y[7:0],neighbor_x[7:0]};
    wire neighbor_foreground=rebuild?(!mask_read_data && foreground_read_data):mask_read_data;
    reg [15:0] component_area;
    reg [4:0] region_seed_support;
    reg region_seed_uncertain;
    reg [8:0] component_x0,component_x1;reg [7:0] component_y0,component_y1;
    reg [11:0] selected_count,processed_count;
    reg [8:0] annotation_x;reg [7:0] annotation_y;
    wire component_qualified=component_area>=20 && component_x0!=0 && component_y0!=0 &&
      component_x1!=256 && component_y1!=150 && component_y1-component_y0>=12;
    // An appearance hint needs accumulated evidence, not isolated bright dots.
    // Larger/clipped changes stay unresolved; this is never a normal verdict.
    // No fixed maximum extent: a longer coherent slot is still evidence.
    // Suppress sparse scattering by requiring mean support >= 3.5 pixels/cell.
    // Count-map cells are 4x4; this development setting is not a defect verdict.
    wire [20:0] region_support_twice={region_support_o,1'b0};
    wire [20:0] region_cell_evidence=({5'd0,component_area}<<3)-{5'd0,component_area};
    wire region_qualified=region_support_o>=PART_MIN_SUPPORT && !region_uncertain_o &&
      region_support_twice>=region_cell_evidence &&
      (component_x1-component_x0>=3 || component_y1-component_y0>=3);
    reg [8:0] selected_x0,selected_x1;reg [7:0] selected_y0,selected_y1;
    integer t;
    always @*begin
        neighbor_x={1'b0,node[7:0]};neighbor_y={1'b0,node[15:8]};
        case(neighbor_index)
          0:begin neighbor_x=neighbor_x-1'b1;neighbor_y=neighbor_y-1'b1;end
          1:neighbor_y=neighbor_y-1'b1;
          2:begin neighbor_x=neighbor_x+1'b1;neighbor_y=neighbor_y-1'b1;end
          3:neighbor_x=neighbor_x-1'b1;
          4:neighbor_x=neighbor_x+1'b1;
          5:begin neighbor_x=neighbor_x-1'b1;neighbor_y=neighbor_y+1'b1;end
          6:neighbor_y=neighbor_y+1'b1;
          7:begin neighbor_x=neighbor_x+1'b1;neighbor_y=neighbor_y+1'b1;end
        endcase
        neighbor_valid=neighbor_x<256 && neighbor_y<150;
    end

    reg [8:0] source_width;reg [7:0] source_height;
    reg [5:0] fitted_width,fitted_height,left_padding,top_padding,map_index;
    reg fit_width_limited;reg [7:0] x_map[0:13],y_map[0:21];
    reg [10:0] normal_index;
    wire [5:0] normal_x={2'b0,normal_index[3:0]},normal_y={1'b0,normal_index[8:4]};
    wire normal_inside=normal_x>=left_padding && normal_x<left_padding+fitted_width &&
                       normal_y>=top_padding && normal_y<top_padding+fitted_height;
    wire [5:0] normal_local_x=normal_x-left_padding,normal_local_y=normal_y-top_padding;
    // Fitted coordinates are bounded by 14x22. Padding never consumes a source pixel.
    wire [3:0] digit_map_x=normal_inside?normal_local_x[3:0]:4'd0;
    wire [4:0] digit_map_y=normal_inside?normal_local_y[4:0]:5'd0;
    wire [8:0] digit_local_x=DIGIT_MIRROR_X ? source_width-1'b1-{1'b0,x_map[digit_map_x]} : {1'b0,x_map[digit_map_x]};
    wire [8:0] source_x=selected_x0+digit_local_x;
    wire [7:0] source_y=selected_y0+y_map[digit_map_y];
    wire normal_bit=normal_inside && mask_read_data;
    reg [8:0] intersections[0:13],unions[0:13];
    reg [13:0] class_best[0:13],negative_score;
    reg [3:0] variant_index;
    reg [13:0] best_score,second_score;
    reg [3:0] best_digit,score_index;
    wire digit_ok=best_score>=6500 && best_score-second_score>=700 && {1'b0,best_score}>={1'b0,negative_score}+15'd300;
    wire shape_ok,shape_done,shape_busy;wire [4:0] shape_label;wire [13:0] shape_score;
    wire [8:0] shape_mask_x;wire [7:0] shape_mask_y;
    wire [8:0] shape_source_x=selected_x0+shape_mask_x;
    wire [7:0] shape_source_y=selected_y0+shape_mask_y;
    wire [3:0] shape_reason;wire [15:0] shape_hole_pixels;wire [31:0] shape_cycles;
    shape_geometry_engine #(.TEMPLATE_FILE(SHAPE_FILE)) u_shape(
      .clk(clk),.rst_n(rst_n),.abort_i(abort_i),.start_i(state==GEOM_START),
      .width_i(source_width),.height_i(source_height),.mask_x_o(shape_mask_x),.mask_y_o(shape_mask_y),
      .mask_data_i(mask_read_data),.done_o(shape_done),.busy_o(shape_busy),.accepted_o(shape_ok),
      .label_o(shape_label),.score_o(shape_score),.reason_o(shape_reason),
      .hole_pixels_o(shape_hole_pixels),.cycles_o(shape_cycles));

    // One bounded restoring divider shared by fitting, maps and scores.
    reg [23:0] divide_numerator,divide_denominator,divide_quotient;
    reg [24:0] divide_remainder;reg [4:0] divide_steps;
    wire [24:0] divide_shifted={divide_remainder[23:0],divide_numerator[23]};
    wire divide_subtract=divide_shifted>={1'b0,divide_denominator};

    always @*begin
        gray_address=0;mask_address=0;queue_address=0;template_address=0;
        gray_write=0;mask_write=0;queue_write=0;mask_write_data=0;queue_write_data=0;
        descriptor_we_o=0;descriptor_addr_o=processed_count[10:0];descriptor_data_o=0;
        annotation_write_o=0;annotation_addr_o=0;
        if(rst_n && !abort_i)case(state)
          CAPTURE:begin gray_address=sample_addr_i;gray_write=sample_valid_i && capture_count<38400 && sample_addr_i==capture_count;end
          THRESHOLD_READ:gray_address=segment_gray_address;
          THRESHOLD_WRITE:begin
              gray_address=segment_gray_address;mask_address=segment_mask_address;
              mask_write=segment_valid;mask_write_data=segment_bit;
          end
          REGION_READ:gray_address=scan_address;
          REGION_MASK:begin
              gray_address=scan_address;mask_address=scan_address;mask_write=1;
              mask_write_data=region_cell_valid;
          end
          SCAN_READ:begin mask_address=scan_address;gray_address=scan_address;end
          SEED:begin mask_address=component_seed;mask_write=1;mask_write_data=rebuild;queue_address=0;queue_write=1;queue_write_data=component_seed;end
          POP_READ:queue_address=queue_head;
          NEIGHBOR_READ:begin mask_address=neighbor_address;gray_address=neighbor_address;end
          NEIGHBOR_CHECK:if(neighbor_foreground && queue_count<4096)begin
              mask_address=neighbor_address;mask_write=1;mask_write_data=rebuild;
              queue_address=queue_tail;queue_write=1;queue_write_data=neighbor_address;end
          COMPONENT_DONE:if(!region_mode && !rebuild && component_qualified && selected_count<12'd2048)begin
              descriptor_we_o=1;descriptor_addr_o=selected_count[10:0];
              descriptor_data_o={component_seed,component_y1,component_y0,component_x1[7:0],component_x0[7:0]};
          end
          EMIT_OBJECT:begin
              descriptor_we_o=1;
              descriptor_data_o={6'b0,object_reason_o,object_accepted_o,object_class_o,
                selected_y1,selected_y0,selected_x1[7:0],selected_x0[7:0]};
          end
          ANNOTATE_TOP:begin annotation_write_o=1;annotation_addr_o={selected_y0,annotation_x[7:0]};end
          ANNOTATE_BOTTOM:begin annotation_write_o=1;annotation_addr_o={(selected_y1-8'd1),annotation_x[7:0]};end
          ANNOTATE_LEFT:begin annotation_write_o=1;annotation_addr_o={annotation_y,selected_x0[7:0]};end
          ANNOTATE_RIGHT:begin annotation_write_o=1;annotation_addr_o={annotation_y,(selected_x1[7:0]-8'd1)};end
          CLEAR_MASK:begin mask_address=scan_address;mask_write=1;mask_write_data=0;end
          NORMAL_READ:begin mask_address={source_y,source_x[7:0]};template_address=variant_index*384+normal_index;end
          GEOM_START,GEOM_READ:mask_address={shape_source_y,shape_source_x[7:0]};
        endcase
    end

    always @(posedge clk)begin
        if(!rst_n || abort_i)begin
            state<=IDLE;result_valid_o<=0;object_valid_o<=0;preview_write_o<=0;
            region_mode<=0;region_support_o<=0;region_uncertain_o<=0;
            region_seed_support<=0;region_seed_uncertain<=0;
            object_index_o<=0;object_class_o<=14;object_accepted_o<=0;object_reason_o<=0;
            object_x0_o<=0;object_y0_o<=0;object_x1_o<=0;object_y1_o<=0;object_area_o<=0;object_score_o<=0;
            preview_addr_o<=0;preview_bit_o<=0;result_frame_id_o<=0;result_config_tag_o<=0;result_count_o<=0;
            result_candidate_count_o<=0;result_declined_count_o<=0;result_omitted_count_o<=0;result_overflow_o<=0;result_reason_o<=0;
            threshold_o<=0;processing_cycles_o<=0;capture_count<=0;capture_bad<=0;gray_min<=255;gray_max<=0;
            cycle_count<=0;scan_address<=0;component_seed<=0;rebuild<=0;queue_head<=0;queue_tail<=0;queue_count<=0;
            node<=0;neighbor_index<=0;component_area<=0;component_x0<=0;component_y0<=0;component_x1<=0;component_y1<=0;
            selected_count<=0;processed_count<=0;annotation_x<=0;annotation_y<=0;
            selected_x0<=0;selected_y0<=0;selected_x1<=0;selected_y1<=0;
            source_width<=0;source_height<=0;fitted_width<=0;fitted_height<=0;left_padding<=0;top_padding<=0;map_index<=0;
            fit_width_limited<=0;normal_index<=0;score_index<=0;best_score<=0;second_score<=0;best_digit<=0;
            variant_index<=0;negative_score<=0;
            divide_return<=IDLE;divide_numerator<=0;divide_denominator<=1;divide_quotient<=0;divide_remainder<=0;divide_steps<=0;
            for(t=0;t<14;t=t+1)begin intersections[t]<=0;unions[t]<=0;class_best[t]<=0;end
        end else begin
            result_valid_o<=0;object_valid_o<=0;preview_write_o<=0;
            if(state!=IDLE && state!=CAPTURE)cycle_count<=cycle_count+1'b1;
            case(state)
              IDLE:if(start_i)begin
                  region_mode<=0;state<=CAPTURE;scan_address<=0;
                  if(ENABLE_LOCAL && regions_i)begin region_mode<=1;state<=REGION_READ;end
                  capture_count<=0;capture_bad<=capture_error_i;gray_min<=255;gray_max<=0;cycle_count<=0;
                  result_frame_id_o<=frame_id_i;result_config_tag_o<=config_tag_i;result_count_o<=0;
                  result_candidate_count_o<=0;result_declined_count_o<=0;result_omitted_count_o<=0;result_overflow_o<=0;result_reason_o<=0;
                  selected_count<=0;processed_count<=0;rebuild<=0;threshold_o<=0;processing_cycles_o<=0;
              end
              CAPTURE:begin
                  if(capture_error_i)capture_bad<=1;
                  if(sample_valid_i)begin
                      if(capture_count>=38400 || sample_addr_i!=capture_count)capture_bad<=1;
                      else begin capture_count<=capture_count+1'b1;if(sample_gray_i<gray_min)gray_min<=sample_gray_i;if(sample_gray_i>gray_max)gray_max<=sample_gray_i;end
                  end
                  if(end_i)begin
                      if(capture_bad || capture_error_i || sample_valid_i || capture_count!=38400)begin result_reason_o<=8;state<=PUBLISH;end
                      else begin threshold_o<=minmax_sum[8:1];scan_address<=0;
                          if(gray_max-gray_min<8)begin result_reason_o<=1;state<=PUBLISH;end else state<=THRESHOLD_READ;end
                  end
              end
              THRESHOLD_READ:state<=THRESHOLD_WRITE;
              REGION_READ:state<=REGION_MASK;
              REGION_MASK:begin
                  if(scan_address==38399)begin scan_address<=0;state<=SCAN_READ;end
                  else begin scan_address<=scan_address+1'b1;state<=REGION_READ;end
              end
              THRESHOLD_WRITE:begin
                  preview_write_o<=segment_valid;preview_addr_o<=segment_mask_address;preview_bit_o<=segment_bit;
                  if(segment_done)begin scan_address<=0;state<=SCAN_READ;end
              end
              SCAN_READ:state<=SCAN_CHECK;
              SCAN_CHECK:begin
                  if(mask_read_data)begin
                      component_seed<=scan_address;state<=SEED;
                      region_seed_support<=gray_read_data[4:0];region_seed_uncertain<=gray_read_data[7];
                  end
                  else if(scan_address==38399)state<=TARGET_CHECK;
                  else begin scan_address<=scan_address+1'b1;state<=SCAN_READ;end
              end
              SEED:begin
                  queue_head<=0;queue_tail<=1;queue_count<=1;component_area<=1;
                  region_support_o<={15'd0,region_seed_support};region_uncertain_o<=region_seed_uncertain;
                  component_x0<={1'b0,component_seed[7:0]};component_x1<={1'b0,component_seed[7:0]}+1'b1;
                  component_y0<=component_seed[15:8];component_y1<=component_seed[15:8]+1'b1;state<=POP_READ;
              end
              POP_READ:if(queue_count==0)state<=COMPONENT_DONE;else state<=POP_LOAD;
              POP_LOAD:begin node<=queue_read_data;queue_head<=queue_head+1'b1;queue_count<=queue_count-1'b1;neighbor_index<=0;state<=NEIGHBOR_READ;end
              NEIGHBOR_READ:begin
                  if(neighbor_valid)state<=NEIGHBOR_CHECK;
                  else if(neighbor_index==7)state<=POP_READ;else neighbor_index<=neighbor_index+1'b1;
              end
              NEIGHBOR_CHECK:begin
                  if(neighbor_foreground && queue_count==4096)begin result_reason_o<=9;result_count_o<=0;state<=PUBLISH;end
                  else begin
                      if(neighbor_foreground)begin
                          region_support_o<=region_support_o+{15'd0,gray_read_data[4:0]};
                          region_uncertain_o<=region_uncertain_o||gray_read_data[7];
                          queue_tail<=queue_tail+1'b1;queue_count<=queue_count+1'b1;component_area<=component_area+1'b1;
                          if(neighbor_x<component_x0)component_x0<=neighbor_x;
                          if(neighbor_x+1'b1>component_x1)component_x1<=neighbor_x+1'b1;
                          if(neighbor_y[7:0]<component_y0)component_y0<=neighbor_y[7:0];
                          if(neighbor_y[7:0]+1'b1>component_y1)component_y1<=neighbor_y[7:0]+1'b1;
                      end
                      if(neighbor_index==7)state<=POP_READ;else begin neighbor_index<=neighbor_index+1'b1;state<=NEIGHBOR_READ;end
                  end
              end
              COMPONENT_DONE:begin
                  if(region_mode)begin
                      if(region_qualified && selected_count>=512)begin
                          result_count_o<=0;result_overflow_o<=1;result_reason_o<=10;state<=PUBLISH;
                      end else if(region_qualified)begin
                          object_x0_o<=component_x0;object_x1_o<=component_x1;
                          object_y0_o<=component_y0;object_y1_o<=component_y1;object_area_o<=component_area;
                          // Extent alone no longer changes the region's evidence status.
                          state<=REGION_EMIT;
                      end else if(scan_address==38399)begin result_count_o<=selected_count;state<=PUBLISH;end
                      else begin scan_address<=scan_address+1'b1;state<=SCAN_READ;end
                  end else if(rebuild)begin source_width<=selected_x1-selected_x0;source_height<=selected_y1-selected_y0;state<=FIT_START;end
                  else begin
                      if(component_area>=20)begin
                          result_candidate_count_o<=result_candidate_count_o+1'b1;
                          if(!component_qualified)result_declined_count_o<=result_declined_count_o+1'b1;
                          else if(selected_count<12'd2048)selected_count<=selected_count+1'b1;
                      end
                      // A corrupt or exhausted record count fails closed instead of
                      // wrapping the write address or publishing a partial snapshot.
                      if(component_qualified && selected_count>=12'd2048)begin
                          result_count_o<=0;result_overflow_o<=1;result_omitted_count_o<=1;
                          result_reason_o<=10;state<=PUBLISH;
                      end else if(scan_address==38399)state<=TARGET_CHECK;
                      else begin scan_address<=scan_address+1'b1;state<=SCAN_READ;end
                  end
              end
              TARGET_CHECK:begin
                  if(region_mode)begin result_count_o<=selected_count;state<=PUBLISH;end
                  else if(selected_count==0)begin result_reason_o<=2;state<=PUBLISH;end else state<=SELECT_OBJECT;
              end
              REGION_EMIT:if(region_ready_i)begin
                  selected_count<=selected_count+1'b1;
                  if(scan_address==38399)begin result_count_o<=selected_count+1'b1;state<=PUBLISH;end
                  else begin scan_address<=scan_address+1'b1;state<=SCAN_READ;end
              end
              SELECT_OBJECT:begin
                  if(processed_count==selected_count)begin result_count_o<=selected_count;state<=PUBLISH;end
                  else state<=LOAD_OBJECT;
              end
              LOAD_OBJECT:begin
                  // SELECT_OBJECT presented this address for one complete RAM edge.
                  selected_x0<={1'b0,descriptor_data_i[7:0]};selected_x1<={1'b0,descriptor_data_i[15:8]};
                  selected_y0<=descriptor_data_i[23:16];selected_y1<=descriptor_data_i[31:24];
                  component_seed<=descriptor_data_i[47:32];
                  scan_address<=0;rebuild<=1;best_score<=0;second_score<=0;best_digit<=0;state<=CLEAR_MASK;
                  variant_index<=0;negative_score<=0;for(t=0;t<14;t=t+1)class_best[t]<=0;
              end
              CLEAR_MASK:if(scan_address==38399)state<=SEED;else scan_address<=scan_address+1'b1;
              FIT_START:begin
                  fit_width_limited<=source_width*22>=source_height*14;
                  if(source_width*22>=source_height*14)begin fitted_width<=14;divide_numerator<=source_height*14+(source_width>>1);divide_denominator<={15'd0,source_width};end
                  else begin fitted_height<=22;divide_numerator<=source_width*22+(source_height>>1);divide_denominator<={16'd0,source_height};end
                  divide_quotient<=0;divide_remainder<=0;divide_steps<=24;divide_return<=FIT_DONE;state<=DIVIDE;
              end
              FIT_DONE:begin
                  if(fit_width_limited)begin fitted_height<=divide_quotient==0?6'd1:divide_quotient[5:0];left_padding<=1;top_padding<=(24-(divide_quotient==0?24'd1:divide_quotient))>>1;end
                  else begin fitted_width<=divide_quotient==0?6'd1:divide_quotient[5:0];top_padding<=1;left_padding<=(16-(divide_quotient==0?24'd1:divide_quotient))>>1;end
                  map_index<=0;state<=XMAP_START;
              end
              XMAP_START:begin divide_numerator<=(2*map_index+1)*source_width;divide_denominator<=2*fitted_width;divide_quotient<=0;divide_remainder<=0;divide_steps<=24;divide_return<=XMAP_DONE;state<=DIVIDE;end
              XMAP_DONE:begin x_map[map_index[3:0]]<=divide_quotient[7:0];if(map_index+1==fitted_width)begin map_index<=0;state<=YMAP_START;end else begin map_index<=map_index+1'b1;state<=XMAP_START;end end
              YMAP_START:begin divide_numerator<=(2*map_index+1)*source_height;divide_denominator<=2*fitted_height;divide_quotient<=0;divide_remainder<=0;divide_steps<=24;divide_return<=YMAP_DONE;state<=DIVIDE;end
              YMAP_DONE:begin y_map[map_index[4:0]]<=divide_quotient[7:0];if(map_index+1==fitted_height)state<=NORMAL_START;else begin map_index<=map_index+1'b1;state<=YMAP_START;end end
              NORMAL_START:begin normal_index<=0;for(t=0;t<14;t=t+1)begin intersections[t]<=0;unions[t]<=0;end state<=NORMAL_READ;end
              NORMAL_READ:state<=NORMAL_ACCUMULATE;
              NORMAL_ACCUMULATE:begin
                  for(t=0;t<14;t=t+1)begin if(normal_bit && template_read_data[t])intersections[t]<=intersections[t]+1'b1;if(normal_bit || template_read_data[t])unions[t]<=unions[t]+1'b1;end
                  if(normal_index==383)begin score_index<=0;state<=SCORE_START;end else begin normal_index<=normal_index+1'b1;state<=NORMAL_READ;end
              end
              SCORE_START:begin divide_numerator<=intersections[score_index]*10000;divide_denominator<=unions[score_index]==0?24'd1:{15'd0,unions[score_index]};divide_quotient<=0;divide_remainder<=0;divide_steps<=24;divide_return<=SCORE_DONE;state<=DIVIDE;end
              SCORE_DONE:begin
                  if(divide_quotient>class_best[score_index])class_best[score_index]<=divide_quotient[13:0];
                  if(score_index==13)begin
                      if(variant_index==11)begin score_index<=0;state<=DIGIT_RANK;end
                      else begin variant_index<=variant_index+1'b1;state<=NORMAL_START;end
                  end else begin score_index<=score_index+1'b1;state<=SCORE_START;end
              end
              DIGIT_RANK:begin
                  if(score_index<10)begin
                      if(class_best[score_index]>best_score)begin second_score<=best_score;best_score<=class_best[score_index];best_digit<=score_index;end
                      else if(class_best[score_index]>second_score)second_score<=class_best[score_index];
                  end else if(class_best[score_index]>negative_score)negative_score<=class_best[score_index];
                  if(score_index==13)state<=GEOM_START;else score_index<=score_index+1'b1;
              end
              GEOM_START:state<=GEOM_READ;
              GEOM_READ:if(shape_done)state<=CLASSIFY;
              CLASSIFY:begin
                  object_x0_o<=selected_x0;object_x1_o<=selected_x1;object_y0_o<=selected_y0;object_y1_o<=selected_y1;object_area_o<=component_area;
                  object_index_o<=processed_count[10:0];object_score_o<=shape_ok && shape_score>best_score?shape_score:best_score;
                  if(digit_ok && best_digit==0 && shape_ok && shape_label==17)begin object_class_o<=17;object_accepted_o<=1;object_reason_o<=0;end
                  else if(digit_ok)begin object_class_o<=best_digit;object_accepted_o<=1;object_reason_o<=0;end
                  else if(shape_ok)begin object_class_o<=shape_label;object_accepted_o<=1;object_reason_o<=0;end
                  else begin object_class_o<=14;object_accepted_o<=0;object_reason_o<=best_score>=6500 && best_score-second_score<700?7:6;end
                  state<=EMIT_OBJECT;
              end
              EMIT_OBJECT:begin
                  object_valid_o<=1;annotation_x<=selected_x0;state<=ANNOTATE_TOP;
              end
              ANNOTATE_TOP:begin
                  if(annotation_x+1'b1==selected_x1)begin annotation_x<=selected_x0;state<=ANNOTATE_BOTTOM;end
                  else annotation_x<=annotation_x+1'b1;
              end
              ANNOTATE_BOTTOM:begin
                  if(annotation_x+1'b1==selected_x1)begin annotation_y<=selected_y0+1'b1;state<=ANNOTATE_LEFT;end
                  else annotation_x<=annotation_x+1'b1;
              end
              ANNOTATE_LEFT:begin
                  if(annotation_y+1'b1==selected_y1-1'b1)begin annotation_y<=selected_y0+1'b1;state<=ANNOTATE_RIGHT;end
                  else annotation_y<=annotation_y+1'b1;
              end
              ANNOTATE_RIGHT:begin
                  if(annotation_y+1'b1==selected_y1-1'b1)state<=ANNOTATE_DRAIN;
                  else annotation_y<=annotation_y+1'b1;
              end
              ANNOTATE_DRAIN:begin
                  // The final annotation write has committed before publication.
                  processed_count<=processed_count+1'b1;state<=SELECT_OBJECT;
              end
              DIVIDE:begin
                  divide_numerator<={divide_numerator[22:0],1'b0};divide_remainder<=divide_subtract?divide_shifted-{1'b0,divide_denominator}:divide_shifted;
                  divide_quotient<={divide_quotient[22:0],divide_subtract};divide_steps<=divide_steps-1'b1;if(divide_steps==1)state<=DIVIDE_DONE;
              end
              DIVIDE_DONE:state<=divide_return;
              PUBLISH:begin result_valid_o<=1;processing_cycles_o<=cycle_count+1'b1;state<=IDLE;end
              default:begin result_reason_o<=8;result_count_o<=0;state<=PUBLISH;end
            endcase
        end
    end
endmodule
