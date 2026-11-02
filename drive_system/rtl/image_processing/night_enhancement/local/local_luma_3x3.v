`timescale 1ns / 1ps

// Two-line streaming luminance window.
//
// The window is causal in the horizontal direction (x-2..x) so it does not
// need a right-edge flush cycle. Two line memories provide the previous two
// rows. This is intentionally a small local statistic block, not a frame
// buffer; it can normally infer two BRAMs on the target device.
module local_luma_3x3 #(
    parameter integer LINE_PIXELS = 2048
) (
    input              pclk,
    input              rst_n,
    input              de_in,
    input              hs_in,
    input              vs_in,
    input      [23:0]  rgb_in,
    output reg         de_out,
    output reg         hs_out,
    output reg         vs_out,
    output reg [23:0]  rgb_out,
    output reg [7:0]   local_mean_out,
    output reg [7:0]   local_max_out,
    output reg [7:0]   local_range_out,
    output reg         stats_valid_out
);
    reg [7:0] line_prev [0:LINE_PIXELS-1];
    reg [7:0] line_prev2[0:LINE_PIXELS-1];

    reg [10:0] x_count;
    reg [1:0]  line_count;
    reg        de_d;
    reg        vs_d;

    reg [7:0] row0_d2;
    reg [7:0] row0_d1;
    reg [7:0] row1_d2;
    reg [7:0] row1_d1;
    reg [7:0] row2_d2;
    reg [7:0] row2_d1;

    wire [7:0] prev1_y;
    wire [7:0] prev2_y;
    reg [7:0] row0_max;
    reg [7:0] row1_max;
    reg [7:0] row2_max;
    reg [7:0] row0_min;
    reg [7:0] row1_min;
    reg [7:0] row2_min;
    reg [11:0] row0_sum;
    reg [11:0] row1_sum;
    reg [11:0] row2_sum;
    reg [13:0] window_sum;
    reg [7:0] window_max;
    reg [7:0] window_min;
    reg [16:0] mean_product;
    wire [7:0] rgb_luma_value;

    // At the first two rows, replicate the current row rather than using
    // uninitialized memory. line_count is updated at the first DE pixel of a
    // line and therefore becomes available for the following pixel.
    assign prev1_y = (line_count == 2'd0) ? rgb_luma_value :
                     line_prev[x_count];
    assign prev2_y = (line_count < 2'd2) ? rgb_luma_value :
                     line_prev2[x_count];

    rgb_luma u_local_rgb_luma (
        .rgb_in (rgb_in),
        .y_out  (rgb_luma_value)
    );

    function [7:0] max3;
        input [7:0] a;
        input [7:0] b;
        input [7:0] c;
        reg [7:0] t;
        begin
            t = (a > b) ? a : b;
            max3 = (t > c) ? t : c;
        end
    endfunction

    function [7:0] min3;
        input [7:0] a;
        input [7:0] b;
        input [7:0] c;
        reg [7:0] t;
        begin
            t = (a < b) ? a : b;
            min3 = (t < c) ? t : c;
        end
    endfunction

    always @* begin
        row0_max = max3(row0_d2, row0_d1, prev2_y);
        row1_max = max3(row1_d2, row1_d1, prev1_y);
        row2_max = max3(row2_d2, row2_d1, rgb_luma_value);
        row0_min = min3(row0_d2, row0_d1, prev2_y);
        row1_min = min3(row1_d2, row1_d1, prev1_y);
        row2_min = min3(row2_d2, row2_d1, rgb_luma_value);
        row0_sum = row0_d2 + row0_d1 + prev2_y;
        row1_sum = row1_d2 + row1_d1 + prev1_y;
        row2_sum = row2_d2 + row2_d1 + rgb_luma_value;
        window_sum = row0_sum + row1_sum + row2_sum;

        window_max = max3(row0_max, row1_max, row2_max);
        window_min = min3(row0_min, row1_min, row2_min);

        // 29/256 is a hardware-friendly approximation of 1/9.
        mean_product = window_sum * 17'd29;
    end

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            x_count          <= 11'd0;
            line_count       <= 2'd0;
            de_d             <= 1'b0;
            vs_d             <= 1'b0;
            row0_d2          <= 8'd0;
            row0_d1          <= 8'd0;
            row1_d2          <= 8'd0;
            row1_d1          <= 8'd0;
            row2_d2          <= 8'd0;
            row2_d1          <= 8'd0;
            de_out           <= 1'b0;
            hs_out           <= 1'b0;
            vs_out           <= 1'b0;
            rgb_out          <= 24'h000000;
            local_mean_out   <= 8'd0;
            local_max_out    <= 8'd0;
            local_range_out  <= 8'd0;
            stats_valid_out  <= 1'b0;
        end else begin
            de_d <= de_in;
            vs_d <= vs_in;

            // 每个VS帧边界重新开始行统计，避免下一帧顶部两行误用上一帧
            // 底部行缓存。缓存内容无需清零，前两行由当前行复制填充。
            if (vs_in && !vs_d) begin
                x_count    <= 11'd0;
                line_count <= 2'd0;
            end

            if (!de_in) begin
                x_count <= 11'd0;
                if (de_d && (line_count != 2'd3))
                    line_count <= line_count + 1'b1;
                row0_d2 <= 8'd0;
                row0_d1 <= 8'd0;
                row1_d2 <= 8'd0;
                row1_d1 <= 8'd0;
                row2_d2 <= 8'd0;
                row2_d1 <= 8'd0;
                de_out  <= 1'b0;
                hs_out  <= hs_in;
                vs_out  <= vs_in;
                rgb_out <= 24'h000000;
                local_mean_out  <= 8'd0;
                local_max_out   <= 8'd0;
                local_range_out <= 8'd0;
                stats_valid_out <= 1'b0;
            end else begin
                if (!de_d) begin
                    row0_d2 <= prev2_y;
                    row0_d1 <= prev2_y;
                    row1_d2 <= prev1_y;
                    row1_d1 <= prev1_y;
                    row2_d2 <= rgb_luma_value;
                    row2_d1 <= rgb_luma_value;
                end else begin
                    row0_d2 <= row0_d1;
                    row0_d1 <= prev2_y;
                    row1_d2 <= row1_d1;
                    row1_d1 <= prev1_y;
                    row2_d2 <= row2_d1;
                    row2_d1 <= rgb_luma_value;
                end

                // The first two columns use a one-pixel neighborhood; from
                // column two onward this is a valid causal 3x3 statistic.
                de_out          <= 1'b1;
                hs_out          <= hs_in;
                vs_out          <= vs_in;
                rgb_out         <= rgb_in;
                if (x_count < 11'd2) begin
                    local_mean_out  <= rgb_luma_value;
                    local_max_out   <= rgb_luma_value;
                    local_range_out <= 8'd0;
                end else begin
                    if (mean_product[16:8] > 9'd255)
                        local_mean_out <= 8'hff;
                    else
                        local_mean_out <= mean_product[15:8];
                    local_max_out   <= window_max;
                    local_range_out <= window_max - window_min;
                end
                stats_valid_out <= (line_count >= 2'd2) && (x_count >= 11'd2);

                line_prev[x_count]  <= rgb_luma_value;
                line_prev2[x_count] <= prev1_y;
                if (x_count == LINE_PIXELS-1)
                    x_count <= 11'd0;
                else
                    x_count <= x_count + 1'b1;
            end
        end
    end
endmodule
