// HDMI输入与三模式图像处理选择。
// mode_select=0：HDMI保持RGB888直通，网络端随后转换为RGB565；
// mode_select=1：YRG彩色灰度化，保留红色/绿色，其余像素转灰度；
// mode_select=2：灰度化 -> 3x3中值滤波 -> 二值化。
// 三条路径均对齐到12级处理延迟，模式只在输入帧边界锁存，避免半帧撕裂。
`timescale 1ns / 1ps

module hdmi_rx(
    input               hdmi_clk_in,
    input               rst_n,
    input               hdmi_de_in,
    input               hdmi_vs_in,
    input               hdmi_hs_in,
    input       [23:0]  hdmi_data_in,
    input       [1:0]   mode_select,
    output              pclk,
    output              hsync,
    output              vsync,
    output              de,
    output      [23:0]  rgb_data,
    output      [1:0]   mode_active,
    // 三条等延迟支路同时导出，本地显示与网络发送可独立选模。
    output      [23:0]  rgb_pass,
    output      [23:0]  rgb_yrg,
    output      [23:0]  rgb_binary,
    output              stream_hsync,
    output              stream_vsync,
    output              stream_de,
    output              night_active
);

reg         hdmi_hs_in_d0;
reg         hdmi_vs_in_d0;
reg         hdmi_de_in_d0;
reg  [23:0] hdmi_data_in_d0;
reg         hdmi_hs_in_d1;
reg         hdmi_vs_in_d1;
reg         hdmi_de_in_d1;
reg  [23:0] hdmi_data_in_d1;
// gamma输出后的共同隔离寄存器：同时给三条HDMI处理支路提供一个稳定
// 的时序起点，避免夜间增强改变布局后造成输入时钟同域hold裕量不足。
reg         gamma_hs_d;
reg         gamma_vs_d;
reg         gamma_de_d;
reg  [23:0] gamma_data_d;
reg         vs_in_d;
reg  [1:0]  mode_frame;

// 直通路径补偿完整处理链的12级延迟；YRG路径自身3级，再补9级。
reg [23:0] pass_data_d [0:11];
reg        pass_de_d   [0:11];
reg        pass_hs_d   [0:11];
reg        pass_vs_d   [0:11];
reg [23:0] yrg_data_d  [0:8];
reg        yrg_de_d    [0:8];
reg        yrg_hs_d    [0:8];
reg        yrg_vs_d    [0:8];
integer    delay_i;

// HDMI输入先经过夜间增强，再进入本工程原有的三种处理支路。
// 增强模块保持1像素/时钟吞吐，DE/HS/VS与RGB同步延迟。
wire [23:0] gamma_rgb;
wire        gamma_de;
wire        gamma_hs;
wire        gamma_vs;
wire        gamma_night_active;
wire [23:0] data_in = gamma_data_d;
wire        de_in   = gamma_de_d;
wire        hs_in   = gamma_hs_d;
wire        vs_in   = gamma_vs_d;
wire [7:0]  gray_data;
wire [23:0] yrg_data;
wire [7:0]  median_data;
wire [7:0]  binary_data;
wire        gray_de, gray_hs, gray_vs;
wire        yrg_de, yrg_hs, yrg_vs;
wire        median_de, median_hs, median_vs;
wire        binary_de, binary_hs, binary_vs;

assign pclk        = hdmi_clk_in;
assign mode_active = mode_frame;
assign rgb_pass    = pass_de_d[11] ? pass_data_d[11] : 24'd0;
assign rgb_yrg     = yrg_de_d[8] ? yrg_data_d[8] : 24'd0;
assign rgb_binary  = binary_de ? {binary_data, binary_data, binary_data} : 24'd0;
assign stream_hsync = pass_hs_d[11];
assign stream_vsync = pass_vs_d[11];
assign stream_de    = pass_de_d[11];
assign night_active = gamma_night_active;

always @(posedge pclk or negedge rst_n) begin
    if (!rst_n) begin
        hdmi_hs_in_d0   <= 1'b0;
        hdmi_vs_in_d0   <= 1'b0;
        hdmi_de_in_d0   <= 1'b0;
        hdmi_data_in_d0 <= 24'd0;
        hdmi_hs_in_d1   <= 1'b0;
        hdmi_vs_in_d1   <= 1'b0;
        hdmi_de_in_d1   <= 1'b0;
        hdmi_data_in_d1 <= 24'd0;
        gamma_hs_d      <= 1'b0;
        gamma_vs_d      <= 1'b0;
        gamma_de_d      <= 1'b0;
        gamma_data_d    <= 24'd0;
        vs_in_d         <= 1'b0;
        mode_frame      <= 2'd0;
        for (delay_i = 0; delay_i < 12; delay_i = delay_i + 1) begin
            pass_data_d[delay_i] <= 24'd0;
            pass_de_d[delay_i]   <= 1'b0;
            pass_hs_d[delay_i]   <= 1'b0;
            pass_vs_d[delay_i]   <= 1'b0;
        end
        for (delay_i = 0; delay_i < 9; delay_i = delay_i + 1) begin
            yrg_data_d[delay_i] <= 24'd0;
            yrg_de_d[delay_i]   <= 1'b0;
            yrg_hs_d[delay_i]   <= 1'b0;
            yrg_vs_d[delay_i]   <= 1'b0;
        end
    end else begin
        hdmi_hs_in_d0   <= hdmi_hs_in;
        hdmi_vs_in_d0   <= hdmi_vs_in;
        hdmi_de_in_d0   <= hdmi_de_in;
        hdmi_data_in_d0 <= hdmi_data_in;
        hdmi_hs_in_d1   <= hdmi_hs_in_d0;
        hdmi_vs_in_d1   <= hdmi_vs_in_d0;
        hdmi_de_in_d1   <= hdmi_de_in_d0;
        hdmi_data_in_d1 <= hdmi_data_in_d0;
        gamma_hs_d      <= gamma_hs;
        gamma_vs_d      <= gamma_vs;
        gamma_de_d      <= gamma_de;
        gamma_data_d    <= gamma_rgb;
        vs_in_d         <= hdmi_vs_in_d1;

        // mode_select在帧间保持稳定；只在VS上升沿更新本帧工作模式。
        if (hdmi_vs_in_d1 && !vs_in_d) begin
            if (mode_select <= 2'd2)
                mode_frame <= mode_select;
            else
                mode_frame <= 2'd0;
        end

        pass_data_d[0] <= data_in;
        pass_de_d[0]   <= de_in;
        pass_hs_d[0]   <= hs_in;
        pass_vs_d[0]   <= vs_in;
        for (delay_i = 1; delay_i < 12; delay_i = delay_i + 1) begin
            pass_data_d[delay_i] <= pass_data_d[delay_i-1];
            pass_de_d[delay_i]   <= pass_de_d[delay_i-1];
            pass_hs_d[delay_i]   <= pass_hs_d[delay_i-1];
            pass_vs_d[delay_i]   <= pass_vs_d[delay_i-1];
        end

        yrg_data_d[0] <= yrg_data;
        yrg_de_d[0]   <= yrg_de;
        yrg_hs_d[0]   <= yrg_hs;
        yrg_vs_d[0]   <= yrg_vs;
        for (delay_i = 1; delay_i < 9; delay_i = delay_i + 1) begin
            yrg_data_d[delay_i] <= yrg_data_d[delay_i-1];
            yrg_de_d[delay_i]   <= yrg_de_d[delay_i-1];
            yrg_hs_d[delay_i]   <= yrg_hs_d[delay_i-1];
            yrg_vs_d[delay_i]   <= yrg_vs_d[delay_i-1];
        end
    end
end

// 夜间增强仅作用于HDMI输入；摄像头/DDR显示链路保持原有数据路径。
gamma_lut_rgb u_gamma_lut_rgb (
    .pclk    (pclk),
    .rst_n   (rst_n),
    .de_in   (hdmi_de_in_d1),
    .hs_in   (hdmi_hs_in_d1),
    .vs_in   (hdmi_vs_in_d1),
    .rgb_in  (hdmi_data_in_d1),
    .de_out  (gamma_de),
    .hs_out  (gamma_hs),
    .vs_out  (gamma_vs),
    .rgb_out (gamma_rgb),
    .night_active (gamma_night_active)
);

rgb888_gray u_rgb888_gray(
    .clk(pclk), .rst_n(rst_n), .de_in(de_in), .hs_in(hs_in), .vs_in(vs_in),
    .data_in(data_in), .de(gray_de), .hs(gray_hs), .vs(gray_vs), .gray(gray_data)
);

// 保持原工程YRG判色规则：红、绿目标保留彩色，其余区域灰度化。
rgb888_yrg u_rgb888_yrg(
    .clk(pclk), .rst_n(rst_n), .de_in(de_in), .hs_in(hs_in), .vs_in(vs_in),
    .data_in(data_in), .de(yrg_de), .hs(yrg_hs), .vs(yrg_vs), .data(yrg_data)
);

gray_median_filter u_gray_median_filter(
    .clk(pclk), .rst_n(rst_n), .in_vsync(gray_vs), .in_hsync(gray_hs),
    .in_de(gray_de), .in_gray(gray_data), .out_vsync(median_vs),
    .out_hsync(median_hs), .out_de(median_de), .out_gray(median_data)
);

gray_binarization #(
    .THRESHOLD       (8'd125),
    .ADAPTIVE_OFFSET (8'd32),
    .MIN_THRESHOLD   (8'd64),
    .MAX_THRESHOLD   (8'd220),
    .FRAME_AVG_SHIFT (20)
) u_gray_binarization(
    .clk(pclk), .rst_n(rst_n), .in_vsync(median_vs), .in_hsync(median_hs),
    .in_de(median_de), .in_gray(median_data), .out_vsync(binary_vs),
    .out_hsync(binary_hs), .out_de(binary_de), .out_gray(binary_data)
);

assign de = (mode_frame == 2'd0) ? pass_de_d[11] :
            (mode_frame == 2'd1) ? yrg_de_d[8]  : binary_de;
assign hsync = (mode_frame == 2'd0) ? pass_hs_d[11] :
               (mode_frame == 2'd1) ? yrg_hs_d[8]  : binary_hs;
assign vsync = (mode_frame == 2'd0) ? pass_vs_d[11] :
               (mode_frame == 2'd1) ? yrg_vs_d[8]  : binary_vs;
assign rgb_data = (mode_frame == 2'd0) ? rgb_pass :
                  (mode_frame == 2'd1) ? rgb_yrg : rgb_binary;

endmodule
