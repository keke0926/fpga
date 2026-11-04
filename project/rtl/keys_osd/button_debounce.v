`timescale 1ns/1ps
module button_debounce #(
    parameter integer CLOCK_HZ = 48000000,
    parameter integer DEBOUNCE_CYCLES = CLOCK_HZ / 100,
    parameter integer RESET_PRESSED = 0
)(
    input wire clk, rst_n,
    input wire button_n,
    output wire pressed_o,
    output reg press_pulse
);
    // Physical active-low input, double synchronized before qualification.
    // DEBOUNCE_CYCLES must be >= 1; default is 10 ms at configurable CLOCK_HZ.
    // The clock frequency is a configuration default, not a board measurement.
    // A new level requires exactly DEBOUNCE_CYCLES consecutive synchronized
    // samples. Holding low never repeats; a qualified release rearms it.
    // Default reset is released. The dedicated detection key starts pressed
    // so its gesture controller waits for a real qualified release after reset.
    function integer counter_width;
        input integer value;
        integer remaining;
        begin
            remaining = value - 1;
            counter_width = 0;
            while (remaining > 0) begin
                counter_width = counter_width + 1;
                remaining = remaining >> 1;
            end
        end
    endfunction
    localparam integer COUNT_WIDTH = (DEBOUNCE_CYCLES <= 1) ? 1 : counter_width(DEBOUNCE_CYCLES);
    (* ASYNC_REG = "TRUE" *) reg sync_meta, sync_button_n;
    reg debounced_n;
    assign pressed_o=!debounced_n;
    reg [COUNT_WIDTH-1:0] stable_count;

    always @(posedge clk) begin
        if (!rst_n) begin
            sync_meta <= !RESET_PRESSED;
            sync_button_n <= !RESET_PRESSED;
            debounced_n <= !RESET_PRESSED;
            stable_count <= 0;
            press_pulse <= 1'b0;
        end else begin
            sync_meta <= button_n;
            sync_button_n <= sync_meta;
            press_pulse <= 1'b0;
            if (sync_button_n == debounced_n)
                stable_count <= 0;
            else if (stable_count == DEBOUNCE_CYCLES-1) begin
                stable_count <= 0;
                debounced_n <= sync_button_n;
                if (!sync_button_n) press_pulse <= 1'b1;
            end else
                stable_count <= stable_count + 1'b1;
        end
    end
endmodule
