`timescale 1ns / 1ps

// BT.601 full-range RGB888 luminance approximation.
// The coefficients are powers-of-two friendly and synthesize to adders.
module rgb_luma (
    input  [23:0] rgb_in,
    output [7:0]  y_out
);
    wire [15:0] y_sum;

    assign y_sum = ({8'd0, rgb_in[23:16]} * 16'd77) +
                   ({8'd0, rgb_in[15:8]}  * 16'd150) +
                   ({8'd0, rgb_in[7:0]}   * 16'd29);
    assign y_out = y_sum[15:8];
endmodule
