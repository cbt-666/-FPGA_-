// RGB888转8位灰度，采用整数加权近似，便于综合并保持教程中的时序结构。
`timescale 1ns / 1ps

// RGB888 to 8-bit BT.601-style luminance.
// The three arithmetic stages deliberately match the example project's
// 77/150/29 implementation: Y = (77R + 150G + 29B) >> 8.
module rgb888_gray(
    input        clk,
    input        rst_n,
    input        de_in,
    input        hs_in,
    input        vs_in,
    input [23:0] data_in,
    output       de,
    output       hs,
    output       vs,
    output [7:0] gray
);

reg [15:0] r_mul;
reg [15:0] g_mul;
reg [15:0] b_mul;
reg [15:0] y_sum;
reg [7:0]  gray_r;
reg [2:0]  de_r;
reg [2:0]  hs_r;
reg [2:0]  vs_r;

wire [7:0] red   = data_in[23:16];
wire [7:0] green = data_in[15:8];
wire [7:0] blue  = data_in[7:0];

assign de   = de_r[2];
assign hs   = hs_r[2];
assign vs   = vs_r[2];
assign gray = de ? gray_r : 8'd0;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_mul  <= 16'd0;
        g_mul  <= 16'd0;
        b_mul  <= 16'd0;
        y_sum  <= 16'd0;
        gray_r <= 8'd0;
        de_r   <= 3'd0;
        hs_r   <= 3'd0;
        vs_r   <= 3'd0;
    end else begin
        r_mul  <= red   * 8'd77;
        g_mul  <= green * 8'd150;
        b_mul  <= blue  * 8'd29;
        y_sum  <= r_mul + g_mul + b_mul;
        gray_r <= y_sum[15:8];
        de_r   <= {de_r[1:0], de_in};
        hs_r   <= {hs_r[1:0], hs_in};
        vs_r   <= {vs_r[1:0], vs_in};
    end
end

endmodule
