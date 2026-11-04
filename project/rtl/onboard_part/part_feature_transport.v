`timescale 1ns/1ps
// Integer arithmetic only: inverse-coordinate and native-feature transport.
// See ../INTERFACE.md. One syntactic multiplier, no private memory.
module part_feature_transport #(parameter integer STATIC_CONTEXT=0, WIDTH=1024, HEIGHT=600) (
    input  wire                 clk_i,
    input  wire                 rst_i,
    input  wire                 abort_i,
    input  wire                 start_i,
    output wire                 ready_o,
    input  wire                 feature_mode_i,
    input  wire signed [17:0]   b00_q16_i,
    input  wire signed [17:0]   b01_q16_i,
    input  wire signed [17:0]   b10_q16_i,
    input  wire signed [17:0]   b11_q16_i,
    input  wire signed [28:0]   offset_x_q16_i,
    input  wire signed [28:0]   offset_y_q16_i,
    input  wire        [9:0]    obs_x_i,
    input  wire        [9:0]    obs_y_i,
    input  wire signed [10:0]   native_gx_q4_i,
    input  wire signed [10:0]   native_gy_q4_i,
    input  wire        [7:0]    native_raw_q2_i,
    input  wire        [16:0]   invscale_q16_i,
    output reg                  busy_o,
    output reg                  done_o,
    output reg                  valid_o,
    input  wire                 out_ready_i,
    output reg                  feature_mode_o,
    output reg signed [23:0]    ref_x_q10_o,
    output reg signed [23:0]    ref_y_q10_o,
    output reg signed [12:0]    warped_gx_q4_o,
    output reg signed [12:0]    warped_gy_q4_o,
    output reg        [9:0]     warped_raw_q2_o,
    output reg interior_o
);
    reg signed [17:0] b00_q_held, b01_q_held, b10_q_held, b11_q_held;
    reg signed [28:0] offset_y_q_held;
    reg [9:0] x_q_held, y_q_held;
    reg signed [10:0] gx_q_held, gy_q_held;
    reg [7:0] raw_q_held;
    reg [16:0] invscale_q_held;
    // STATIC_CONTEXT requires caller-held operand/identity buses through result consumption.
    wire signed [17:0] b00_q = STATIC_CONTEXT ? b00_q16_i : b00_q_held;
    wire signed [17:0] b01_q = STATIC_CONTEXT ? b01_q16_i : b01_q_held;
    wire signed [17:0] b10_q = STATIC_CONTEXT ? b10_q16_i : b10_q_held;
    wire signed [17:0] b11_q = STATIC_CONTEXT ? b11_q16_i : b11_q_held;
    wire signed [28:0] offset_y_q = STATIC_CONTEXT ? offset_y_q16_i : offset_y_q_held;
    wire [9:0] x_q = STATIC_CONTEXT ? obs_x_i : x_q_held;
    wire [9:0] y_q = STATIC_CONTEXT ? obs_y_i : y_q_held;
    wire signed [10:0] gx_q = STATIC_CONTEXT ? native_gx_q4_i : gx_q_held;
    wire signed [10:0] gy_q = STATIC_CONTEXT ? native_gy_q4_i : gy_q_held;
    wire [7:0] raw_q = STATIC_CONTEXT ? native_raw_q2_i : raw_q_held;
    wire [16:0] invscale_q = STATIC_CONTEXT ? invscale_q16_i : invscale_q_held;
    reg mode_q;
    reg x_interior;
    reg [2:0] step_q;
    reg signed [36:0] acc_q;
    reg signed [17:0] mul_a, mul_b;

    assign ready_o = !rst_i && !abort_i && !busy_o && !valid_o;

    // Only one multiply expression. Coordinate and feature calls use the same
    // selected operands, accumulator and final rounding path.
    wire signed [35:0] product = mul_a * mul_b;
    wire signed [36:0] sum_value = acc_q + {product[35], product};
    wire coordinate_interior = sum_value >= 37'sd1114112 &&
        sum_value < (step_q==1 ? (WIDTH-18)*37'sd65536 : (HEIGHT-18)*37'sd65536);
    wire [36:0] abs_sum = sum_value[36] ? -sum_value : sum_value;
    wire [36:0] biased = abs_sum + (mode_q ? 37'd32768 : 37'd32);
    wire [36:0] rounded_magnitude = mode_q ? (biased >> 16) : (biased >> 6);
    wire signed [36:0] rounded_value = sum_value[36]
        ? -$signed(rounded_magnitude) : $signed(rounded_magnitude);

    always @* begin
        mul_a = 18'sd0;
        mul_b = 18'sd0;
        if (!mode_q) begin
            case (step_q)
                3'd0: begin mul_a=b00_q; mul_b=$signed({8'b0,x_q}); end
                3'd1: begin mul_a=b01_q; mul_b=$signed({8'b0,y_q}); end
                3'd2: begin mul_a=b10_q; mul_b=$signed({8'b0,x_q}); end
                3'd3: begin mul_a=b11_q; mul_b=$signed({8'b0,y_q}); end
                default: begin end
            endcase
        end else begin
            case (step_q)
                // B transposed, because A maps reference to observation.
                3'd0: begin mul_a=b00_q; mul_b={{7{gx_q[10]}},gx_q}; end
                3'd1: begin mul_a=b10_q; mul_b={{7{gy_q[10]}},gy_q}; end
                3'd2: begin mul_a=b01_q; mul_b={{7{gx_q[10]}},gx_q}; end
                3'd3: begin mul_a=b11_q; mul_b={{7{gy_q[10]}},gy_q}; end
                3'd4: begin mul_a=$signed({1'b0,invscale_q});
                            mul_b=$signed({10'b0,raw_q}); end
                default: begin end
            endcase
        end
    end

    always @(posedge clk_i) begin
        if (rst_i || abort_i) begin
            busy_o <= 1'b0;
            done_o <= 1'b0;
            valid_o <= 1'b0;
            feature_mode_o <= 1'b0;
            ref_x_q10_o <= 24'sd0;
            ref_y_q10_o <= 24'sd0;
            warped_gx_q4_o <= 13'sd0;
            warped_gy_q4_o <= 13'sd0;
            warped_raw_q2_o <= 10'd0;interior_o<=0;x_interior<=0;
            mode_q <= 1'b0;
            step_q <= 3'd0;
            b00_q_held <= 18'sd0; b01_q_held <= 18'sd0;
            b10_q_held <= 18'sd0; b11_q_held <= 18'sd0;
            offset_y_q_held <= 29'sd0;
            x_q_held <= 10'd0; y_q_held <= 10'd0;
            gx_q_held <= 11'sd0; gy_q_held <= 11'sd0;
            raw_q_held <= 8'd0; invscale_q_held <= 17'd0;
            acc_q <= 37'sd0;
        end else begin
            done_o <= 1'b0;
            if (valid_o && out_ready_i)
                valid_o <= 1'b0;

            if (start_i && ready_o) begin
                busy_o <= 1'b1;
                mode_q <= feature_mode_i;
                step_q <= 3'd0;
                b00_q_held <= b00_q16_i; b01_q_held <= b01_q16_i;
                b10_q_held <= b10_q16_i; b11_q_held <= b11_q16_i;
                offset_y_q_held <= offset_y_q16_i;
                x_q_held <= obs_x_i; y_q_held <= obs_y_i;
                gx_q_held <= native_gx_q4_i; gy_q_held <= native_gy_q4_i;
                raw_q_held <= native_raw_q2_i; invscale_q_held <= invscale_q16_i;
                // X offset is captured directly into the shared accumulator.
                acc_q <= feature_mode_i ? 37'sd0
                         : {{8{offset_x_q16_i[28]}},offset_x_q16_i};
                ref_x_q10_o <= 24'sd0; ref_y_q10_o <= 24'sd0;
                warped_gx_q4_o <= 13'sd0; warped_gy_q4_o <= 13'sd0;
                warped_raw_q2_o <= 10'd0;
            end else if (busy_o) begin
                step_q <= step_q + 3'd1;
                case (step_q)
                    3'd0, 3'd2: acc_q <= sum_value;
                    3'd1: begin
                        if (mode_q) begin
                            warped_gx_q4_o <= rounded_value[12:0];
                            acc_q <= 37'sd0;
                        end else begin
                            ref_x_q10_o <= rounded_value[23:0];x_interior<=coordinate_interior;
                            acc_q <= {{8{offset_y_q[28]}},offset_y_q};
                        end
                    end
                    3'd3: begin
                        acc_q <= 37'sd0;
                        if (mode_q) begin
                            warped_gy_q4_o <= rounded_value[12:0];
                        end else begin
                            ref_y_q10_o <= rounded_value[23:0];interior_o<=x_interior&&coordinate_interior;
                            feature_mode_o <= 1'b0;
                            busy_o <= 1'b0;
                            valid_o <= 1'b1;
                            done_o <= 1'b1;
                        end
                    end
                    3'd4: begin
                        warped_raw_q2_o <= rounded_value[9:0];
                        feature_mode_o <= 1'b1;
                        busy_o <= 1'b0;
                        valid_o <= 1'b1;
                        done_o <= 1'b1;
                    end
                    default: begin end
                endcase
            end
        end
    end
endmodule
