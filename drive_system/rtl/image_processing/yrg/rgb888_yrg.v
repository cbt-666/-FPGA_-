// RGB888 红/绿目标保留模块。
// 红色像素输出纯红，绿色像素输出纯绿，其余有效像素输出灰度；
// DE/HS/VS 与数据保持固定三拍延迟，便于后级叠加和 HDMI 时序对齐。
module rgb888_yrg #(
    parameter [7:0] RED_MIN      = 8'd96,
    parameter [7:0] GREEN_MIN    = 8'd56,
    parameter [7:0] GREEN_MARGIN = 8'd16
)(
    input        clk,
    input        rst_n,
    input        de_in,
    input        vs_in,
    input        hs_in,
    input [23:0] data_in,
    output       de,
    output       hs,
    output       vs,
    output [23:0] data
);

wire [7:0] r;
wire [7:0] g;
wire [7:0] b;
assign r = data_in[23:16];
assign g = data_in[15:8];
assign b = data_in[7:0];

wire red_keep_in;
wire green_keep_in;
assign red_keep_in = (r >= RED_MIN) &&
                     ({1'b0, r} > ({1'b0, g} << 1)) &&
                     ({1'b0, r} > ({1'b0, b} << 1));
assign green_keep_in = (g >= GREEN_MIN) &&
                       ({1'b0, g} > ({1'b0, r} + {1'b0, GREEN_MARGIN})) &&
                       ({1'b0, g} > ({1'b0, b} + {1'b0, GREEN_MARGIN}));

reg [2:0] de_d;
reg [2:0] hs_d;
reg [2:0] vs_d;
reg [2:0] red_d;
reg [2:0] green_d;
reg [15:0] r_mul;
reg [15:0] g_mul;
reg [15:0] b_mul;
reg [15:0] y_sum;
reg [7:0]  gray;

assign de = de_d[2];
assign hs = hs_d[2];
assign vs = vs_d[2];
assign data = de ? (red_d[2] ? 24'hff0000 :
                    green_d[2] ? 24'h00ff00 : {gray, gray, gray}) : 24'd0;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        de_d    <= 3'd0;
        hs_d    <= 3'd0;
        vs_d    <= 3'd0;
        red_d   <= 3'd0;
        green_d <= 3'd0;
        r_mul   <= 16'd0;
        g_mul   <= 16'd0;
        b_mul   <= 16'd0;
        y_sum   <= 16'd0;
        gray    <= 8'd0;
    end else begin
        de_d    <= {de_d[1:0], de_in};
        hs_d    <= {hs_d[1:0], hs_in};
        vs_d    <= {vs_d[1:0], vs_in};
        red_d   <= {red_d[1:0], red_keep_in};
        green_d <= {green_d[1:0], green_keep_in};
        r_mul   <= r * 8'd77;
        g_mul   <= g * 8'd150;
        b_mul   <= b * 8'd29;
        y_sum   <= r_mul + g_mul + b_mul;
        gray    <= y_sum[15:8];
    end
end

endmodule
