`timescale 1ns/1ps
// One observation pixel and bilinear reference sample through the held DDR
// mailbox. Native layout and transaction contract are in ../INTERFACE.md.
module part_pose_sample #(
    parameter integer WIDTH=1024, HEIGHT=600, STATIC_CONTEXT=0
)(
    input wire clk_i,rst_i,abort_i,start_i,
    output wire ready_o,busy_o,done_o,valid_o,
    input wire out_ready_i,
    output reg sample_valid_o,output reg interior_o,
    output reg [3:0] error_o,
    input wire signed [17:0] c_q16_i,d_q16_i,
    input wire signed [28:0] offset_x_q16_i,offset_y_q16_i,
    input wire [16:0] invscale_q16_i,
    input wire [31:0] ref_base_i,obs_base_i,
    input wire [9:0] obs_x_i,obs_y_i,
    input wire [31:0] task_id_i,frame_id_i,reference_id_i,config_epoch_i,
    output wire [9:0] obs_x_o,obs_y_o,
    output wire [31:0] task_id_o,frame_id_o,reference_id_o,config_epoch_o,
    output reg signed [10:0] obs_gx_q4_o,obs_gy_q4_o,
    output reg [7:0] obs_raw_q2_o,
    output reg [1:0] obs_flags_o,
    output reg signed [12:0] ref_gx_q4_o,ref_gy_q4_o,
    output reg [9:0] ref_raw_q2_o,
    output reg [1:0] ref_flags_o,
    output reg signed [23:0] ref_x_q10_o,ref_y_q10_o,
    output reg [2:0] read_count_o,
    output wire req_valid_o,input wire req_ready_i,
    output wire req_write_o,output wire [31:0] req_addr_o,
    output wire [127:0] req_data_o,
    input wire rsp_valid_i,output wire rsp_ready_o,
    input wire rsp_error_i,input wire [127:0] rsp_data_i
);
    localparam [4:0] IDLE=0,COORD_START=1,COORD_WAIT=2,CHECK=3,
        FETCH=4,REQUEST=5,RESPONSE=6,WORD=7,WEIGHT=8,
        MAC_GX=9,MAC_GY=10,MAC_RAW=11,NEXT_TAP=12,ROUND=13,
        FEATURE_START=14,FEATURE_WAIT=15,PUBLISH=16,HOLD=17,DRAIN=18;
    localparam [32:0] IMAGE_BYTES=WIDTH*HEIGHT*4;
    reg [4:0] state;
    reg [9:0] obs_x_o_held;
    reg [9:0] obs_y_o_held;
    reg [31:0] task_id_o_held;
    reg [31:0] frame_id_o_held;
    reg [31:0] reference_id_o_held;
    reg [31:0] config_epoch_o_held;
    reg signed [17:0] c_q_held,d_q_held;
    reg signed [28:0] offset_x_q_held,offset_y_q_held;
    reg [16:0] invscale_q_held;
    reg [31:0] ref_base_q_held,obs_base_q_held;
    // STATIC_CONTEXT requires caller-held operand/identity buses through result consumption.
    wire signed [17:0] c_q = STATIC_CONTEXT ? c_q16_i : c_q_held;
    wire signed [17:0] d_q = STATIC_CONTEXT ? d_q16_i : d_q_held;
    wire signed [28:0] offset_x_q = STATIC_CONTEXT ? offset_x_q16_i : offset_x_q_held;
    wire signed [28:0] offset_y_q = STATIC_CONTEXT ? offset_y_q16_i : offset_y_q_held;
    wire [16:0] invscale_q = STATIC_CONTEXT ? invscale_q16_i : invscale_q_held;
    wire [31:0] ref_base_q = STATIC_CONTEXT ? ref_base_i : ref_base_q_held;
    wire [31:0] obs_base_q = STATIC_CONTEXT ? obs_base_i : obs_base_q_held;
    assign obs_x_o = STATIC_CONTEXT ? obs_x_i : obs_x_o_held;
    assign obs_y_o = STATIC_CONTEXT ? obs_y_i : obs_y_o_held;
    assign task_id_o = STATIC_CONTEXT ? task_id_i : task_id_o_held;
    assign frame_id_o = STATIC_CONTEXT ? frame_id_i : frame_id_o_held;
    assign reference_id_o = STATIC_CONTEXT ? reference_id_i : reference_id_o_held;
    assign config_epoch_o = STATIC_CONTEXT ? config_epoch_i : config_epoch_o_held;
    reg result_valid,done_q;
    // One cache beat, no array or private RAM. Never persists across samples.
    reg [127:0] cache_data;
    reg [31:0] cache_addr,request_addr;
    reg cache_valid,reference_fetch;
    reg [1:0] lane_q,tap_q;
    reg [31:0] word_q;
    reg signed [13:0] x0_q,y0_q;
    reg [9:0] fx_q,fy_q;
    reg [20:0] weight_q;
    reg signed [31:0] gx_sum,gy_sum,raw_sum;
    reg signed [10:0] interpolated_gx,interpolated_gy;
    reg [7:0] interpolated_raw;

    assign ready_o=!rst_i&&!abort_i&&state==IDLE;
    assign busy_o=state!=IDLE&&state!=HOLD;
    assign valid_o=result_valid&&!rst_i&&!abort_i;
    assign done_o=done_q&&!rst_i&&!abort_i;
    assign req_valid_o=!rst_i&&!abort_i&&state==REQUEST;
    assign req_write_o=1'b0;
    assign req_addr_o=request_addr;
    assign req_data_o=128'd0;
    // An abort may consume a response already present on its edge.
    assign rsp_ready_o=!rst_i&&(state==RESPONSE||state==DRAIN);

    wire core_ready,core_valid,core_interior;
    wire signed [23:0] core_x,core_y;
    wire signed [12:0] core_gx,core_gy;
    wire [9:0] core_raw;
    wire core_start=(state==COORD_START||state==FEATURE_START)&&!abort_i;
    wire signed [17:0] minus_d=-d_q;
    part_feature_transport #(.STATIC_CONTEXT(STATIC_CONTEXT),.WIDTH(WIDTH),.HEIGHT(HEIGHT)) transport(
        .clk_i(clk_i),.rst_i(rst_i),.abort_i(abort_i||state==DRAIN),
        .start_i(core_start),.ready_o(core_ready),.feature_mode_i(state==FEATURE_START),
        .b00_q16_i(c_q),.b01_q16_i(d_q),.b10_q16_i(minus_d),.b11_q16_i(c_q),
        .offset_x_q16_i(offset_x_q),.offset_y_q16_i(offset_y_q),
        .obs_x_i(obs_x_o),.obs_y_i(obs_y_o),
        .native_gx_q4_i(interpolated_gx),.native_gy_q4_i(interpolated_gy),
        .native_raw_q2_i(interpolated_raw),.invscale_q16_i(invscale_q),
        .busy_o(),.done_o(),.valid_o(core_valid),.out_ready_i(1'b1),.feature_mode_o(),
        .ref_x_q10_o(core_x),.ref_y_q10_o(core_y),
        .warped_gx_q4_o(core_gx),.warped_gy_q4_o(core_gy),.warped_raw_q2_o(core_raw),.interior_o(core_interior));

    // Arithmetic shift gives floor, including negative coordinates. A zero
    // fractional part suppresses the otherwise outside right/bottom tap.
    wire signed [13:0] floor_x=ref_x_q10_o[23:10];
    wire signed [13:0] floor_y=ref_y_q10_o[23:10];
    wire reference_in_domain=floor_x>=0&&floor_y>=0&&floor_x<WIDTH&&floor_y<HEIGHT
        &&(ref_x_q10_o[9:0]==0||floor_x<WIDTH-1)
        &&(ref_y_q10_o[9:0]==0||floor_y<HEIGHT-1);
    wire [10:0] tap_x=$unsigned(x0_q)+tap_q[0];
    wire [10:0] tap_y=$unsigned(y0_q)+tap_q[1];
    wire [31:0] pixel_index=reference_fetch ? tap_y*WIDTH+tap_x
                                                         : obs_y_o*WIDTH+obs_x_o;
    wire [31:0] word_addr=(reference_fetch?ref_base_q:obs_base_q)+(pixel_index<<2);
    wire [10:0] wx=tap_q[0]?{1'b0,fx_q}:11'd1024-{1'b0,fx_q};
    wire [10:0] wy=tap_q[1]?{1'b0,fy_q}:11'd1024-{1'b0,fy_q};
    // Weight <= 2^20; the other operand is a signed native11 component or
    // positive weight axis <= 1024. Nonnegative tap weights sum to 2^20,
    // so every partial gradient sum has magnitude <= 1024*2^20 = 2^30.
    reg signed [21:0] mul_a;
    reg signed [11:0] mul_b;
    wire signed [33:0] product=mul_a*mul_b;
    always @* begin
        mul_a=$signed({1'b0,weight_q});
        mul_b=12'sd0;
        case(state)
            WEIGHT:begin mul_a=$signed({11'd0,wx});mul_b=$signed({1'b0,wy});end
            MAC_GX:mul_b={word_q[10],word_q[10:0]};
            MAC_GY:mul_b={word_q[21],word_q[21:11]};
            MAC_RAW:mul_b=$signed({4'd0,word_q[29:22]});
            default:begin end
        endcase
    end
    function signed [31:0] round20;
        input signed [31:0] value;
        reg [31:0] magnitude;
        begin
            magnitude=value[31]?-value:value;
            magnitude=(magnitude+32'd524288)>>20;
            round20=value[31]?-$signed(magnitude):$signed(magnitude);
        end
    endfunction

    always @(posedge clk_i) begin
        if(rst_i)begin
            state<=IDLE;result_valid<=0;done_q<=0;sample_valid_o<=0;interior_o<=0;error_o<=0;
            obs_x_o_held<=0;obs_y_o_held<=0;task_id_o_held<=0;frame_id_o_held<=0;reference_id_o_held<=0;config_epoch_o_held<=0;
            obs_gx_q4_o<=0;obs_gy_q4_o<=0;obs_raw_q2_o<=0;obs_flags_o<=0;
            ref_gx_q4_o<=0;ref_gy_q4_o<=0;ref_raw_q2_o<=0;ref_flags_o<=0;
            ref_x_q10_o<=0;ref_y_q10_o<=0;read_count_o<=0;
            c_q_held<=0;d_q_held<=0;offset_x_q_held<=0;offset_y_q_held<=0;invscale_q_held<=0;ref_base_q_held<=0;obs_base_q_held<=0;
            cache_data<=0;cache_addr<=0;request_addr<=0;cache_valid<=0;reference_fetch<=0;
            lane_q<=0;tap_q<=0;word_q<=0;x0_q<=0;y0_q<=0;fx_q<=0;fy_q<=0;weight_q<=0;
            gx_sum<=0;gy_sum<=0;raw_sum<=0;interpolated_gx<=0;interpolated_gy<=0;interpolated_raw<=0;
        end else begin
            done_q<=0;
            if(abort_i)begin
                result_valid<=0;sample_valid_o<=0;interior_o<=0;error_o<=0;cache_valid<=0;
                obs_gx_q4_o<=0;obs_gy_q4_o<=0;obs_raw_q2_o<=0;obs_flags_o<=0;
                ref_gx_q4_o<=0;ref_gy_q4_o<=0;ref_raw_q2_o<=0;ref_flags_o<=0;
                if((state==RESPONSE||state==DRAIN)&&!rsp_valid_i)state<=DRAIN;
                else state<=IDLE;
            end else case(state)
                IDLE:if(start_i)begin
                    c_q_held<=c_q16_i;d_q_held<=d_q16_i;offset_x_q_held<=offset_x_q16_i;offset_y_q_held<=offset_y_q16_i;
                    invscale_q_held<=invscale_q16_i;ref_base_q_held<=ref_base_i;obs_base_q_held<=obs_base_i;
                    obs_x_o_held<=obs_x_i;obs_y_o_held<=obs_y_i;
                    task_id_o_held<=task_id_i;frame_id_o_held<=frame_id_i;reference_id_o_held<=reference_id_i;config_epoch_o_held<=config_epoch_i;
                    result_valid<=0;sample_valid_o<=0;interior_o<=0;error_o<=0;read_count_o<=0;cache_valid<=0;
                    obs_gx_q4_o<=0;obs_gy_q4_o<=0;obs_raw_q2_o<=0;obs_flags_o<=0;
                    ref_gx_q4_o<=0;ref_gy_q4_o<=0;ref_raw_q2_o<=0;ref_flags_o<=0;
                    ref_x_q10_o<=0;ref_y_q10_o<=0;
                    gx_sum<=0;gy_sum<=0;raw_sum<=0;reference_fetch<=0;tap_q<=0;
                    if(WIDTH<1||WIDTH>1024||HEIGHT<1||HEIGHT>600||ref_base_i[1:0]!=0||obs_base_i[1:0]!=0
                       ||({1'b0,ref_base_i}+IMAGE_BYTES)>33'h100000000
                       ||({1'b0,obs_base_i}+IMAGE_BYTES)>33'h100000000)begin
                        error_o<=1;state<=PUBLISH;
                    end else if(obs_x_i>=WIDTH||obs_y_i>=HEIGHT)state<=PUBLISH;
                    else state<=COORD_START;
                end
                COORD_START:if(core_ready)state<=COORD_WAIT;
                COORD_WAIT:if(core_valid)begin ref_x_q10_o<=core_x;ref_y_q10_o<=core_y;interior_o<=core_interior;state<=CHECK;end
                CHECK:if(reference_in_domain)begin
                    x0_q<=floor_x;y0_q<=floor_y;fx_q<=ref_x_q10_o[9:0];fy_q<=ref_y_q10_o[9:0];state<=FETCH;
                end else state<=PUBLISH;
                FETCH:if(cache_valid&&cache_addr=={word_addr[31:4],4'd0})begin
                    word_q<=cache_data>>((3-word_addr[3:2])*32);state<=WORD;
                end else begin request_addr<={word_addr[31:4],4'd0};lane_q<=word_addr[3:2];state<=REQUEST;end
                REQUEST:if(req_ready_i)begin read_count_o<=read_count_o+1'b1;state<=RESPONSE;end
                RESPONSE:if(rsp_valid_i)begin
                    if(rsp_error_i)begin
                        error_o<=2;sample_valid_o<=0;cache_valid<=0;
                        obs_gx_q4_o<=0;obs_gy_q4_o<=0;obs_raw_q2_o<=0;obs_flags_o<=0;
                        ref_gx_q4_o<=0;ref_gy_q4_o<=0;ref_raw_q2_o<=0;ref_flags_o<=0;state<=PUBLISH;
                    end else begin
                        cache_data<=rsp_data_i;cache_addr<=request_addr;cache_valid<=1;
                        word_q<=rsp_data_i>>((3-lane_q)*32);state<=WORD;
                    end
                end
                WORD:if(!reference_fetch)begin
                    obs_gx_q4_o<=word_q[10:0];obs_gy_q4_o<=word_q[21:11];obs_raw_q2_o<=word_q[29:22];obs_flags_o<=word_q[31:30];
                    reference_fetch<=1;tap_q<=0;state<=WEIGHT;
                end else begin ref_flags_o<=ref_flags_o|word_q[31:30];state<=MAC_GX;end
                WEIGHT:begin weight_q<=product[20:0];state<=product==0?NEXT_TAP:FETCH;end
                MAC_GX:begin gx_sum<=gx_sum+product;state<=MAC_GY;end
                MAC_GY:begin gy_sum<=gy_sum+product;state<=MAC_RAW;end
                MAC_RAW:begin raw_sum<=raw_sum+product;state<=NEXT_TAP;end
                NEXT_TAP:if(tap_q==3)state<=ROUND;else begin tap_q<=tap_q+1'b1;state<=WEIGHT;end
                ROUND:begin
                    interpolated_gx<=round20(gx_sum);interpolated_gy<=round20(gy_sum);interpolated_raw<=round20(raw_sum);
                    state<=FEATURE_START;
                end
                FEATURE_START:if(core_ready)state<=FEATURE_WAIT;
                FEATURE_WAIT:if(core_valid)begin
                    ref_gx_q4_o<=core_gx;ref_gy_q4_o<=core_gy;ref_raw_q2_o<=core_raw;sample_valid_o<=1;state<=PUBLISH;
                end
                PUBLISH:begin result_valid<=1;done_q<=1;state<=HOLD;end
                HOLD:if(out_ready_i)begin result_valid<=0;state<=IDLE;end
                DRAIN:if(rsp_valid_i)begin cache_valid<=0;state<=IDLE;end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
