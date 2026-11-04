`timescale 1ns/1ps
// Fixed bounded grayscale transaction. No backpressure on an external video
// stream: the adapter owns sample4 capture and skips frames while busy.
// Two exact 8-connected traversals retain only the selected component mask.
// All large arrays have synchronous reads; RAM contents are never reset.
module roi_digit_engine #(
    parameter TEMPLATE_FILE = "rtl/apps/digit_templates.mem"
)(
    input wire clk, rst_n, abort_i,
    input wire start_i,
    input wire [31:0] frame_id_i, config_tag_i,
    input wire sample_valid_i,
    input wire [13:0] sample_addr_i,
    input wire [7:0] sample_gray_i,
    input wire end_i, capture_error_i,
    output wire ready_o, busy_o,
    output reg result_valid_o,
    output reg [31:0] result_frame_id_o, result_config_tag_o,
    output reg accepted_o,
    output reg [3:0] digit_o, reason_o,
    output reg [13:0] best_score_o, second_score_o,
    output reg [7:0] bbox_x0_o, bbox_y0_o, bbox_x1_o, bbox_y1_o,
    output reg [7:0] threshold_o,
    output reg [14:0] foreground_area_o,
    output reg [1535:0] normalized_o,
    output reg [31:0] processing_cycles_o
);
    localparam IDLE=0, CAPTURE=1, THRESHOLD_READ=2, THRESHOLD_WRITE=3,
      SCAN_READ=4, SCAN_CHECK=5, SEED=6, POP_READ=7, POP_LOAD=8,
      NEIGHBOR_READ=9, NEIGHBOR_CHECK=10, COMPONENT_DONE=11,
      TARGET_CHECK=12, REBUILD_START=13, FIT_START=14, FIT_DONE=15,
      XMAP_START=16, XMAP_DONE=17, YMAP_START=18, YMAP_DONE=19,
      NORMAL_START=20, NORMAL_READ=21, NORMAL_ACCUMULATE=22,
      SCORE_START=23, SCORE_DONE=24, CLASSIFY=25, PUBLISH=26,
      DIVIDE=27, DIVIDE_DONE=28;
    reg [5:0] state, divide_return;
    assign ready_o = state == IDLE;
    assign busy_o = state != IDLE;

    (* ram_style="block" *) reg [7:0] gray_memory [0:16383];
    (* ram_style="block" *) reg mask_memory [0:16383];
    (* ram_style="block" *) reg [13:0] queue_memory [0:16383];
    // Bit d of ROM word p is template digit d, pixel p (row-major).
    (* ram_style="block" *) reg [9:0] template_memory [0:1535];
    initial $readmemh(TEMPLATE_FILE, template_memory);
    reg [13:0] gray_address, mask_address, queue_address;
    reg [10:0] template_address;
    reg gray_write, mask_write, queue_write;
    reg mask_write_data;
    reg [13:0] queue_write_data;
    reg [7:0] gray_read_data;
    reg mask_read_data;
    reg [13:0] queue_read_data;
    reg [9:0] template_read_data;
    always @(posedge clk) begin
        gray_read_data <= gray_memory[gray_address];
        mask_read_data <= mask_memory[mask_address];
        queue_read_data <= queue_memory[queue_address];
        template_read_data <= template_memory[template_address];
        if (gray_write) gray_memory[gray_address] <= sample_gray_i;
        if (mask_write) mask_memory[mask_address] <= mask_write_data;
        if (queue_write) queue_memory[queue_address] <= queue_write_data;
    end

    reg [14:0] capture_count;
    reg capture_bad;
    reg [7:0] gray_min, gray_max;
    wire [8:0] minmax_sum = {1'b0,gray_min}+{1'b0,gray_max};
    reg [31:0] cycle_count;
    reg [13:0] scan_address, component_seed, selected_seed;
    reg rebuild;
    reg [14:0] queue_head, queue_tail;
    reg [13:0] node;
    reg [2:0] neighbor_index;
    reg [14:0] component_area, largest_area, runner_area;
    reg [7:0] component_x0, component_y0, component_x1, component_y1;
    reg [7:0] selected_x0, selected_y0, selected_x1, selected_y1;
    reg [7:0] neighbor_x, neighbor_y;
    reg neighbor_valid;
    wire [13:0] neighbor_address = {neighbor_y[6:0],neighbor_x[6:0]};
    wire neighbor_foreground = rebuild ?
        (!mask_read_data && gray_read_data <= threshold_o) : mask_read_data;

    always @* begin
        neighbor_x={1'b0,node[6:0]};
        neighbor_y={1'b0,node[13:7]};
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
        neighbor_valid = neighbor_x < 128 && neighbor_y < 128;
    end

    reg [7:0] source_width, source_height;
    reg [5:0] fitted_width, fitted_height, left_padding, top_padding, map_index;
    reg fit_width_limited;
    reg [6:0] x_map[0:31];
    reg [6:0] y_map[0:47];
    reg [10:0] normal_index;
    wire [5:0] normal_x={1'b0,normal_index[4:0]};
    wire [5:0] normal_y=normal_index[10:5];
    wire normal_inside = normal_x>=left_padding && normal_x<left_padding+fitted_width &&
                         normal_y>=top_padding && normal_y<top_padding+fitted_height;
    wire [5:0] normal_local_x=normal_x-left_padding;
    wire [5:0] normal_local_y=normal_y-top_padding;
    wire [7:0] source_x=selected_x0+{1'b0,x_map[normal_local_x[4:0]]};
    wire [7:0] source_y=selected_y0+{1'b0,y_map[normal_local_y]};
    wire normal_bit = normal_inside && mask_read_data;
    reg [10:0] intersections[0:9], unions[0:9];
    reg [3:0] score_index;
    integer t;

    // Shared restoring unsigned divider: fixed 24 steps, no variable / operator.
    reg [23:0] divide_numerator, divide_denominator, divide_quotient;
    reg [24:0] divide_remainder;
    reg [4:0] divide_steps;
    wire [24:0] divide_shifted={divide_remainder[23:0],divide_numerator[23]};
    wire divide_subtract=divide_shifted>={1'b0,divide_denominator};

    // Combinational port selection; READ state -> synchronous RAM -> CHECK state.
    // Enqueue and visited marking happen together so each traversal enqueues
    // each pixel at most once. Linear 15-bit pointers represent 16384 exactly.
    always @* begin
        gray_address=0;mask_address=0;queue_address=0;template_address=0;
        gray_write=0;mask_write=0;queue_write=0;
        mask_write_data=0;queue_write_data=0;
        if(rst_n && !abort_i)begin
            case(state)
                CAPTURE:begin
                    gray_address=sample_addr_i;
                    gray_write=sample_valid_i && capture_count<16384 &&
                               sample_addr_i==capture_count[13:0];
                end
                THRESHOLD_READ:gray_address=scan_address;
                THRESHOLD_WRITE:begin
                    mask_address=scan_address;mask_write=1;
                    mask_write_data=gray_read_data<=threshold_o;
                end
                SCAN_READ:mask_address=scan_address;
                SEED:begin
                    mask_address=component_seed;mask_write=1;mask_write_data=rebuild;
                    queue_address=0;queue_write=1;queue_write_data=component_seed;
                end
                POP_READ:queue_address=queue_head[13:0];
                NEIGHBOR_READ:begin mask_address=neighbor_address;gray_address=neighbor_address;end
                NEIGHBOR_CHECK:begin
                    if(neighbor_foreground && queue_tail<16384)begin
                        mask_address=neighbor_address;mask_write=1;mask_write_data=rebuild;
                        queue_address=queue_tail[13:0];queue_write=1;queue_write_data=neighbor_address;
                    end
                end
                NORMAL_READ:begin
                    mask_address={source_y[6:0],source_x[6:0]};
                    template_address=normal_index;
                end
            endcase
        end
    end

    always @(posedge clk)begin
        if(!rst_n || abort_i)begin
            state<=IDLE;result_valid_o<=0;accepted_o<=0;digit_o<=0;reason_o<=0;
            result_frame_id_o<=0;result_config_tag_o<=0;
            best_score_o<=0;second_score_o<=0;
            bbox_x0_o<=0;bbox_y0_o<=0;bbox_x1_o<=0;bbox_y1_o<=0;
            threshold_o<=0;foreground_area_o<=0;normalized_o<=0;processing_cycles_o<=0;
            capture_count<=0;capture_bad<=0;gray_min<=255;gray_max<=0;
            cycle_count<=0;scan_address<=0;component_seed<=0;selected_seed<=0;rebuild<=0;
            queue_head<=0;queue_tail<=0;node<=0;neighbor_index<=0;
            component_area<=0;largest_area<=0;runner_area<=0;
            component_x0<=0;component_y0<=0;component_x1<=0;component_y1<=0;
            selected_x0<=0;selected_y0<=0;selected_x1<=0;selected_y1<=0;
            source_width<=0;source_height<=0;fitted_width<=0;fitted_height<=0;
            left_padding<=0;top_padding<=0;map_index<=0;fit_width_limited<=0;
            normal_index<=0;score_index<=0;
            divide_return<=IDLE;divide_numerator<=0;divide_denominator<=1;
            divide_quotient<=0;divide_remainder<=0;divide_steps<=0;
            for(t=0;t<10;t=t+1)begin intersections[t]<=0;unions[t]<=0;end
        end else begin
            result_valid_o<=0;
            if(state!=IDLE && state!=CAPTURE)cycle_count<=cycle_count+1'b1;
            case(state)
                IDLE:if(start_i)begin
                    state<=CAPTURE;capture_count<=0;capture_bad<=capture_error_i;
                    gray_min<=255;gray_max<=0;cycle_count<=0;
                    result_frame_id_o<=frame_id_i;result_config_tag_o<=config_tag_i;
                    accepted_o<=0;digit_o<=0;reason_o<=0;best_score_o<=0;second_score_o<=0;
                    bbox_x0_o<=0;bbox_y0_o<=0;bbox_x1_o<=0;bbox_y1_o<=0;
                    threshold_o<=0;foreground_area_o<=0;normalized_o<=0;processing_cycles_o<=0;
                    largest_area<=0;runner_area<=0;queue_head<=0;queue_tail<=0;rebuild<=0;
                end
                CAPTURE:begin
                    if(capture_error_i)capture_bad<=1;
                    if(sample_valid_i)begin
                        if(capture_count>=16384 || sample_addr_i!=capture_count[13:0])capture_bad<=1;
                        else begin
                            capture_count<=capture_count+1'b1;
                            if(sample_gray_i<gray_min)gray_min<=sample_gray_i;
                            if(sample_gray_i>gray_max)gray_max<=sample_gray_i;
                        end
                    end
                    if(end_i)begin
                        if(capture_bad || capture_error_i || sample_valid_i || capture_count!=16384)begin
                            reason_o<=8;state<=PUBLISH;
                        end else begin
                            threshold_o<=minmax_sum[8:1];scan_address<=0;
                            if(gray_max-gray_min<40)begin reason_o<=1;state<=PUBLISH;end
                            else state<=THRESHOLD_READ;
                        end
                    end
                end
                THRESHOLD_READ:state<=THRESHOLD_WRITE;
                THRESHOLD_WRITE:begin
                    if(scan_address==16383)begin scan_address<=0;state<=SCAN_READ;end
                    else begin scan_address<=scan_address+1'b1;state<=THRESHOLD_READ;end
                end
                SCAN_READ:state<=SCAN_CHECK;
                SCAN_CHECK:begin
                    if(mask_read_data)begin component_seed<=scan_address;state<=SEED;end
                    else if(scan_address==16383)state<=TARGET_CHECK;
                    else begin scan_address<=scan_address+1'b1;state<=SCAN_READ;end
                end
                SEED:begin
                    queue_head<=0;queue_tail<=1;component_area<=1;
                    component_x0<={1'b0,component_seed[6:0]};
                    component_y0<={1'b0,component_seed[13:7]};
                    component_x1<={1'b0,component_seed[6:0]}+1'b1;
                    component_y1<={1'b0,component_seed[13:7]}+1'b1;
                    state<=POP_READ;
                end
                POP_READ:begin
                    if(queue_head==queue_tail)state<=COMPONENT_DONE;
                    else state<=POP_LOAD;
                end
                POP_LOAD:begin
                    node<=queue_read_data;queue_head<=queue_head+1'b1;
                    neighbor_index<=0;state<=NEIGHBOR_READ;
                end
                NEIGHBOR_READ:begin
                    if(neighbor_valid)state<=NEIGHBOR_CHECK;
                    else if(neighbor_index==7)state<=POP_READ;
                    else neighbor_index<=neighbor_index+1'b1;
                end
                NEIGHBOR_CHECK:begin
                    if(neighbor_foreground)begin
                        if(queue_tail==16384)begin
                            // Unreachable with atomic visited marking; fail closed.
                            reason_o<=8;normalized_o<=0;state<=PUBLISH;
                        end else begin
                            queue_tail<=queue_tail+1'b1;component_area<=component_area+1'b1;
                            if(neighbor_x<component_x0)component_x0<=neighbor_x;
                            if(neighbor_y<component_y0)component_y0<=neighbor_y;
                            if(neighbor_x+1'b1>component_x1)component_x1<=neighbor_x+1'b1;
                            if(neighbor_y+1'b1>component_y1)component_y1<=neighbor_y+1'b1;
                            if(neighbor_index==7)state<=POP_READ;
                            else begin neighbor_index<=neighbor_index+1'b1;state<=NEIGHBOR_READ;end
                        end
                    end else if(neighbor_index==7)state<=POP_READ;
                    else begin neighbor_index<=neighbor_index+1'b1;state<=NEIGHBOR_READ;end
                end
                COMPONENT_DONE:begin
                    if(rebuild)begin
                        source_width<=selected_x1-selected_x0;source_height<=selected_y1-selected_y0;
                        state<=FIT_START;
                    end else begin
                        if(component_area>largest_area)begin
                            runner_area<=largest_area;largest_area<=component_area;selected_seed<=component_seed;
                            selected_x0<=component_x0;selected_y0<=component_y0;
                            selected_x1<=component_x1;selected_y1<=component_y1;
                        end else if(component_area>runner_area)runner_area<=component_area;
                        if(scan_address==16383)state<=TARGET_CHECK;
                        else begin scan_address<=scan_address+1'b1;state<=SCAN_READ;end
                    end
                end
                TARGET_CHECK:begin
                    foreground_area_o<=largest_area;
                    bbox_x0_o<=selected_x0;bbox_y0_o<=selected_y0;
                    bbox_x1_o<=selected_x1;bbox_y1_o<=selected_y1;
                    if(largest_area<20)begin reason_o<=2;state<=PUBLISH;end
                    else if(selected_x0==0 || selected_y0==0 || selected_x1==128 || selected_y1==128)begin
                        reason_o<=3;state<=PUBLISH;
                    end else if(selected_y1-selected_y0<18)begin reason_o<=4;state<=PUBLISH;end
                    else if(runner_area>=20 && {runner_area,2'b00}>={2'b00,largest_area})begin
                        reason_o<=5;state<=PUBLISH;
                    end else state<=REBUILD_START;
                end
                REBUILD_START:begin
                    // Full first pass consumed every foreground bit. Reuse zeros
                    // as visited flags; grayscale retains membership information.
                    rebuild<=1;component_seed<=selected_seed;state<=SEED;
                end
                FIT_START:begin
                    fit_width_limited<=source_width*44>=source_height*28;
                    if(source_width*44>=source_height*28)begin
                        fitted_width<=28;
                        divide_numerator<=source_height*28+(source_width>>1);
                        divide_denominator<={16'd0,source_width};
                    end else begin
                        fitted_height<=44;
                        divide_numerator<=source_width*44+(source_height>>1);
                        divide_denominator<={16'd0,source_height};
                    end
                    divide_quotient<=0;divide_remainder<=0;divide_steps<=24;divide_return<=FIT_DONE;state<=DIVIDE;
                end
                FIT_DONE:begin
                    if(fit_width_limited)begin
                        fitted_height<=divide_quotient==0?6'd1:divide_quotient[5:0];
                        left_padding<=2;
                        top_padding<=(48-(divide_quotient==0?24'd1:divide_quotient))>>1;
                    end else begin
                        fitted_width<=divide_quotient==0?6'd1:divide_quotient[5:0];
                        top_padding<=2;
                        left_padding<=(32-(divide_quotient==0?24'd1:divide_quotient))>>1;
                    end
                    map_index<=0;state<=XMAP_START;
                end
                XMAP_START:begin
                    divide_numerator<=(2*map_index+1)*source_width;
                    divide_denominator<=2*fitted_width;
                    divide_quotient<=0;divide_remainder<=0;divide_steps<=24;divide_return<=XMAP_DONE;state<=DIVIDE;
                end
                XMAP_DONE:begin
                    x_map[map_index[4:0]]<=divide_quotient[6:0];
                    if(map_index+1==fitted_width)begin map_index<=0;state<=YMAP_START;end
                    else begin map_index<=map_index+1'b1;state<=XMAP_START;end
                end
                YMAP_START:begin
                    divide_numerator<=(2*map_index+1)*source_height;
                    divide_denominator<=2*fitted_height;
                    divide_quotient<=0;divide_remainder<=0;divide_steps<=24;divide_return<=YMAP_DONE;state<=DIVIDE;
                end
                YMAP_DONE:begin
                    y_map[map_index]<=divide_quotient[6:0];
                    if(map_index+1==fitted_height)state<=NORMAL_START;
                    else begin map_index<=map_index+1'b1;state<=YMAP_START;end
                end
                NORMAL_START:begin
                    normal_index<=0;normalized_o<=0;
                    for(t=0;t<10;t=t+1)begin intersections[t]<=0;unions[t]<=0;end
                    state<=NORMAL_READ;
                end
                NORMAL_READ:state<=NORMAL_ACCUMULATE;
                NORMAL_ACCUMULATE:begin
                    normalized_o[normal_index]<=normal_bit;
                    for(t=0;t<10;t=t+1)begin
                        if(normal_bit && template_read_data[t])intersections[t]<=intersections[t]+1'b1;
                        if(normal_bit || template_read_data[t])unions[t]<=unions[t]+1'b1;
                    end
                    if(normal_index==1535)begin score_index<=0;state<=SCORE_START;end
                    else begin normal_index<=normal_index+1'b1;state<=NORMAL_READ;end
                end
                SCORE_START:begin
                    divide_numerator<=intersections[score_index]*10000;
                    divide_denominator<=unions[score_index]==0?24'd1:{13'd0,unions[score_index]};
                    divide_quotient<=0;divide_remainder<=0;divide_steps<=24;divide_return<=SCORE_DONE;state<=DIVIDE;
                end
                SCORE_DONE:begin
                    // Ascending labels and strictly-greater replacement preserve
                    // lower-label tie policy; equal best is still runner-up.
                    if(divide_quotient>best_score_o)begin
                        second_score_o<=best_score_o;best_score_o<=divide_quotient[13:0];digit_o<=score_index;
                    end else if(divide_quotient>second_score_o)second_score_o<=divide_quotient[13:0];
                    if(score_index==9)state<=CLASSIFY;
                    else begin score_index<=score_index+1'b1;state<=SCORE_START;end
                end
                CLASSIFY:begin
                    if(best_score_o<6500)reason_o<=6;
                    else if(best_score_o-second_score_o<700)reason_o<=7;
                    else begin reason_o<=0;accepted_o<=1;end
                    state<=PUBLISH;
                end
                DIVIDE:begin
                    divide_numerator<={divide_numerator[22:0],1'b0};
                    divide_remainder<=divide_subtract?divide_shifted-{1'b0,divide_denominator}:divide_shifted;
                    divide_quotient<={divide_quotient[22:0],divide_subtract};
                    divide_steps<=divide_steps-1'b1;
                    if(divide_steps==1)state<=DIVIDE_DONE;
                end
                DIVIDE_DONE:state<=divide_return;
                PUBLISH:begin result_valid_o<=1;processing_cycles_o<=cycle_count+1'b1;state<=IDLE;end
                default:begin reason_o<=8;normalized_o<=0;state<=PUBLISH;end
            endcase
        end
    end
endmodule
