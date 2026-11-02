// 灰度自适应二值化：像素大于当前帧阈值输出白色，否则输出黑色。
//
// 阈值按帧更新：统计当前帧有效像素的平均亮度和最大亮度，在下一帧
// 的 VSYNC 边界锁存新的阈值。这样一帧内阈值保持不变，不会出现画面
// 撕裂或同一帧黑白区域随统计量变化而闪烁的问题。
`timescale 1ns / 1ps

// 适用于本工程固定的 1280x720@60Hz HDMI 输入。
// 平均值采用移位加法近似，避免引入大位宽除法器/IP。
module gray_binarization #(
    // 复位后、第一帧统计完成前使用的回退阈值。
    parameter [7:0] THRESHOLD       = 8'd125,
    // 自适应阈值 = 帧平均亮度 + 偏移量。
    parameter [7:0] ADAPTIVE_OFFSET = 8'd32,
    // 限制阈值范围，避免整幅全黑或对暗场过度放大。
    parameter [7:0] MIN_THRESHOLD   = 8'd64,
    parameter [7:0] MAX_THRESHOLD   = 8'd220,
    // 1280x720=921600；移位近似对应 1/921600。
    parameter integer FRAME_AVG_SHIFT = 20
)(
    input        clk,
    input        rst_n,
    input        in_vsync,
    input        in_hsync,
    input        in_de,
    input [7:0]  in_gray,
    output reg   out_vsync,
    output reg   out_hsync,
    output reg   out_de,
    output reg [7:0] out_gray
);

localparam integer SUM_WIDTH = 32;

reg [SUM_WIDTH-1:0] frame_sum;
reg                  frame_seen;
reg [7:0]            frame_max;
reg                  in_vsync_d;
reg [7:0]            adaptive_threshold;

// 输入 VSYNC 上升沿表示新帧开始；此时上一帧统计已经完整结束。
wire frame_start = in_vsync & ~in_vsync_d;

// 1/921600 ≈ 1/2^20 + 1/2^23 + 1/2^26。
wire [SUM_WIDTH-1:0] average_term0 = frame_sum >> FRAME_AVG_SHIFT;
wire [SUM_WIDTH-1:0] average_term1 = frame_sum >> (FRAME_AVG_SHIFT + 3);
wire [SUM_WIDTH-1:0] average_term2 = frame_sum >> (FRAME_AVG_SHIFT + 6);
wire [SUM_WIDTH-1:0] frame_average_approx =
    average_term0 + average_term1 + average_term2;

wire [8:0] threshold_sum =
    {1'b0, frame_average_approx[7:0]} + {1'b0, ADAPTIVE_OFFSET};

wire [7:0] mean_threshold =
    (threshold_sum < {1'b0, MIN_THRESHOLD}) ? MIN_THRESHOLD :
    (threshold_sum > {1'b0, MAX_THRESHOLD}) ? MAX_THRESHOLD :
    threshold_sum[7:0];

// 若平均值阈值不高于本帧最大值，则采用平均值阈值；否则保留一个
// 低于最大值的边界，避免高亮场景下二值结果全部变黑。
wire [7:0] max_threshold =
    (frame_max > (MIN_THRESHOLD + 8'd8)) ?
        (frame_max - 8'd8) : MIN_THRESHOLD;
wire [7:0] threshold_from_frame =
    (frame_max <= mean_threshold) ? max_threshold : mean_threshold;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        out_vsync <= 1'b0;
        out_hsync <= 1'b0;
        out_de    <= 1'b0;
        out_gray  <= 8'd0;
        frame_sum <= {SUM_WIDTH{1'b0}};
        frame_seen <= 1'b0;
        frame_max <= 8'd0;
        in_vsync_d <= 1'b0;
        adaptive_threshold <= THRESHOLD;
    end else begin
        out_vsync <= in_vsync;
        out_hsync <= in_hsync;
        out_de    <= in_de;
        if (in_de)
            out_gray <= (in_gray > adaptive_threshold) ? 8'hFF : 8'h00;
        else
            out_gray <= 8'd0;

        in_vsync_d <= in_vsync;

        // 新帧开始时锁存上一帧统计结果，然后清零统计寄存器。
        if (frame_start) begin
            if (frame_seen)
                adaptive_threshold <= threshold_from_frame;
            frame_sum  <= {SUM_WIDTH{1'b0}};
            frame_seen <= 1'b0;
            frame_max  <= 8'd0;
        end else if (in_de) begin
            frame_sum  <= frame_sum + {{(SUM_WIDTH-8){1'b0}}, in_gray};
            frame_seen <= 1'b1;
            if (in_gray > frame_max)
                frame_max <= in_gray;
        end
    end
end

endmodule
