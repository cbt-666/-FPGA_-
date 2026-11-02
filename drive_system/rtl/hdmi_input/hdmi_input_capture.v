// HDMI输入缩放流：1280x720 RGB888 -> 640x360 RGB565。
// 该路径用于BACK_LEFT可选HDMI源，输入数据应接夜间增强后的RGB直通支路，
// 不受YRG/二值化全屏选择影响。out_frame保持输入VS电平，便于DDR侧换页。
module hdmi_input_capture #(
    parameter integer IMAGE_WIDTH  = 640,
    parameter integer IMAGE_HEIGHT = 360
)(
    input                   in_clk,
    input                   in_rst_n,
    input                   in_de,
    input                   in_vs,
    input                   in_hs,
    input       [23:0]      in_rgb,
    output reg  [15:0]      out_data,
    output reg              out_valid,
    output reg              out_frame
);

reg        in_de_d;
reg        in_vs_d;
reg [11:0] in_x;
reg [10:0] in_y;
reg        line_seen;
reg [1:0]  rst_sync;

wire rst_n_sync = rst_sync[1];
wire de_rise = in_de & ~in_de_d;
wire vs_rise = in_vs & ~in_vs_d;
wire [11:0] sample_x = de_rise ? 12'd0 : in_x;
wire [10:0] sample_y = de_rise ? (line_seen ? (in_y + 1'b1) : 11'd0) : in_y;

always @(posedge in_clk or negedge in_rst_n) begin
    if (!in_rst_n) begin
        rst_sync  <= 2'b00;
        in_de_d   <= 1'b0;
        in_vs_d   <= 1'b0;
        in_x      <= 12'd0;
        in_y      <= 11'd0;
        line_seen <= 1'b0;
        out_data  <= 16'd0;
        out_valid <= 1'b0;
        out_frame <= 1'b0;
    end else begin
        rst_sync  <= {rst_sync[0],1'b1};
        out_valid <= 1'b0;
        out_frame <= in_vs;
        in_de_d   <= in_de;
        in_vs_d   <= in_vs;

        if (vs_rise) begin
            in_x      <= 12'd0;
            in_y      <= 11'd0;
            line_seen <= 1'b0;
        end else if (de_rise) begin
            in_x <= 12'd1;
            if (line_seen)
                in_y <= in_y + 1'b1;
            line_seen <= 1'b1;
        end else if (in_de && (in_x < 12'd2047)) begin
            in_x <= in_x + 1'b1;
        end

        // 2:1最近邻抽样，保留夜间增强后的RGB颜色。
        if (rst_n_sync && in_de && !vs_rise &&
            !sample_x[0] && !sample_y[0] &&
            (sample_x < IMAGE_WIDTH*2) &&
            (sample_y < IMAGE_HEIGHT*2)) begin
            out_data  <= {in_rgb[23:19], in_rgb[15:10], in_rgb[7:3]};
            out_valid <= 1'b1;
        end
    end
end

wire unused_in_hs = in_hs;

endmodule
