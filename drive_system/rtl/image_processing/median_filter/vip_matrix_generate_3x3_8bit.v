`timescale 1ns / 1ps

// Tutorial-style 3x3 window generator using reset-safe line buffers.
module vip_matrix_generate_3x3_8bit(
    input clk, input rst_n,
    input pre_frame_vsync, input pre_frame_href, input pre_frame_clken,
    input [7:0] pre_img_y,
    output matrix_frame_vsync, output matrix_frame_href,
    output matrix_frame_clken,
    output reg [7:0] matrix_p11, matrix_p12, matrix_p13,
    output reg [7:0] matrix_p21, matrix_p22, matrix_p23,
    output reg [7:0] matrix_p31, matrix_p32, matrix_p33
);
wire [7:0] row1_data, row2_data;
wire read_frame_href, read_frame_clken;
reg [7:0] pre_img_y_d[2:0];
reg [7:0] row3_data;
reg [7:0] row1_d[0:2], row2_d[0:2];
integer tap_delay_i;
reg vsync_d, href_d;
reg [1:0] valid_lines;
reg row1_valid, row2_valid;
reg [4:0] pre_frame_vsync_r, pre_frame_href_r, pre_frame_clken_r;

assign read_frame_href = pre_frame_href_r[3];
assign read_frame_clken = pre_frame_clken_r[3];
assign matrix_frame_vsync = pre_frame_vsync_r[4];
assign matrix_frame_href = pre_frame_href_r[4];
assign matrix_frame_clken = pre_frame_clken_r[4];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        row3_data <= 8'd0;
        pre_img_y_d[0] <= 8'd0;
        pre_img_y_d[1] <= 8'd0;
        pre_img_y_d[2] <= 8'd0;
        vsync_d <= 1'b0;
        href_d <= 1'b0;
        valid_lines <= 2'd0;
        row1_valid <= 1'b0;
        row2_valid <= 1'b0;
        for (tap_delay_i=0; tap_delay_i<3; tap_delay_i=tap_delay_i+1) begin
            row1_d[tap_delay_i] <= 8'd0;
            row2_d[tap_delay_i] <= 8'd0;
        end
    end else begin
        pre_img_y_d[0] <= pre_img_y;
        pre_img_y_d[1] <= pre_img_y_d[0];
        pre_img_y_d[2] <= pre_img_y_d[1];
        row3_data <= pre_img_y_d[2];
        vsync_d <= pre_frame_vsync;
        href_d <= pre_frame_href;
        if (pre_frame_vsync && !vsync_d)
            valid_lines <= 2'd0;
        else if (!pre_frame_href && href_d && valid_lines != 2'd2)
            valid_lines <= valid_lines + 1'b1;
        // RAM contents are not cleared on VS. Mask the previous frame's
        // rows until enough lines of this frame have filled the buffers.
        row1_valid <= (valid_lines == 2'd2);
        row2_valid <= (valid_lines != 2'd0);
        // The RAM taps arrive after one clock; delay them three more clocks
        // to match row3_data and the four-clock window input controls.
        row1_d[0] <= row1_valid ? row1_data : 8'd0;
        row2_d[0] <= row2_valid ? row2_data : 8'd0;
        for (tap_delay_i=1; tap_delay_i<3; tap_delay_i=tap_delay_i+1) begin
            row1_d[tap_delay_i] <= row1_d[tap_delay_i-1];
            row2_d[tap_delay_i] <= row2_d[tap_delay_i-1];
        end
    end
end

line_shift_ram_8bit u_line_shift_ram_8bit(
    .clock(clk), .rst_n(rst_n), .clken(pre_frame_clken),
    .pre_frame_href(pre_frame_href), .shiftin(pre_img_y),
    .taps0x(row2_data), .taps1x(row1_data)
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pre_frame_vsync_r <= 5'd0;
        pre_frame_href_r <= 5'd0;
        pre_frame_clken_r <= 5'd0;
    end else begin
        pre_frame_vsync_r <= {pre_frame_vsync_r[3:0], pre_frame_vsync};
        pre_frame_href_r <= {pre_frame_href_r[3:0], pre_frame_href};
        pre_frame_clken_r <= {pre_frame_clken_r[3:0], pre_frame_clken};
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        {matrix_p11, matrix_p12, matrix_p13} <= 24'h0;
        {matrix_p21, matrix_p22, matrix_p23} <= 24'h0;
        {matrix_p31, matrix_p32, matrix_p33} <= 24'h0;
    end else if (read_frame_href) begin
        if (read_frame_clken) begin
            {matrix_p11, matrix_p12, matrix_p13} <= {matrix_p12, matrix_p13, row1_d[2]};
            {matrix_p21, matrix_p22, matrix_p23} <= {matrix_p22, matrix_p23, row2_d[2]};
            {matrix_p31, matrix_p32, matrix_p33} <= {matrix_p32, matrix_p33, row3_data};
        end
    end else begin
        {matrix_p11, matrix_p12, matrix_p13} <= 24'h0;
        {matrix_p21, matrix_p22, matrix_p23} <= 24'h0;
        {matrix_p31, matrix_p32, matrix_p33} <= 24'h0;
    end
end
endmodule
