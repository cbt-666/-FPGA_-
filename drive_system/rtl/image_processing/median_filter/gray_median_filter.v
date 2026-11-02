// 3×3中值滤波：先生成邻域窗口，再对9个像素排序并输出中值。
`timescale 1ns / 1ps

// Stand-alone grayscale 3x3 median-filter stage.
// The implementation follows the tutorial structure:
//   1) two 2048x8 dual-port RAMs form the two line delays;
//   2) a 3x3 matrix generator forms the neighbourhood;
//   3) seven sort3 instances in three pipeline levels select the median.
module gray_median_filter(
    input        clk,
    input        rst_n,
    input        in_vsync,
    input        in_hsync,
    input        in_de,
    input [7:0]  in_gray,
    output       out_vsync,
    output       out_hsync,
    output       out_de,
    output [7:0] out_gray
);

wire        matrix_vsync;
wire        matrix_href;
wire        matrix_de;
wire [7:0]  matrix_p11;
wire [7:0]  matrix_p12;
wire [7:0]  matrix_p13;
wire [7:0]  matrix_p21;
wire [7:0]  matrix_p22;
wire [7:0]  matrix_p23;
wire [7:0]  matrix_p31;
wire [7:0]  matrix_p32;
wire [7:0]  matrix_p33;
reg  [4:0]  hsync_matrix_delay;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        hsync_matrix_delay <= 5'd0;
    end else begin
        // Match the matrix generator's five-stage control pipeline.
        hsync_matrix_delay <= {hsync_matrix_delay[3:0], in_hsync};
    end
end

vip_matrix_generate_3x3_8bit u_matrix_generate(
    .clk                (clk),
    .rst_n              (rst_n),
    .pre_frame_vsync    (in_vsync),
    // href in the tutorial means active pixels; HDMI DE is that signal.
    .pre_frame_href     (in_de),
    .pre_frame_clken    (in_de),
    .pre_img_y          (in_gray),
    .matrix_frame_vsync (matrix_vsync),
    .matrix_frame_href  (matrix_href),
    .matrix_frame_clken (matrix_de),
    .matrix_p11         (matrix_p11),
    .matrix_p12         (matrix_p12),
    .matrix_p13         (matrix_p13),
    .matrix_p21         (matrix_p21),
    .matrix_p22         (matrix_p22),
    .matrix_p23         (matrix_p23),
    .matrix_p31         (matrix_p31),
    .matrix_p32         (matrix_p32),
    .matrix_p33         (matrix_p33)
);

median_filter_3x3 u_median_filter_3x3(
    .clk                (clk),
    .rst_n              (rst_n),
    .median_frame_vsync (matrix_vsync),
    .median_frame_href  (hsync_matrix_delay[4]),
    .median_frame_clken (matrix_de),
    .data11             (matrix_p11),
    .data12             (matrix_p12),
    .data13             (matrix_p13),
    .data21             (matrix_p21),
    .data22             (matrix_p22),
    .data23             (matrix_p23),
    .data31             (matrix_p31),
    .data32             (matrix_p32),
    .data33             (matrix_p33),
    .target_data        (out_gray),
    .pos_median_vsync   (out_vsync),
    .pos_median_href    (out_hsync),
    .pos_median_clken   (out_de)
);

endmodule
