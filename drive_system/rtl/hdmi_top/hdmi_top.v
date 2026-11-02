// HDMI显示顶层。
// 输入时序始终作为输出时序基准；支持 HDMI 全屏、四宫格和近车伪鸟瞰三种显示模式。
module hdmi_top(
    input          stream_clk,
    input          tx_pixel_clk,
    input          tx_pixel_clk_5x,
    input          sys_rst_n,

    input          quad_mode_req,
    input          bev_mode_req,
    input          zoom_fullscreen_req,
    input  [1:0]   zoom_source_req,
    input  [15:0]  bev_rgb565,
    input          bev_rd_valid,
    input          bev_rd_fault,
    input          bev_frame_ready,
    input  [1:0]   ultrasonic_alert_level,
    input  [23:0]  hdmi_rgb_in,
    input          hdmi_hs_in,
    input          hdmi_vs_in,
    input          hdmi_de_in,

    output         tmds_clk_p,
    output         tmds_clk_n,
    output [2:0]   tmds_data_p,
    output [2:0]   tmds_data_n,

    input  [15:0]  rd_data,
    input          eth_frame_valid,
    output         rd_en,
    output         video_vs,
    output [10:0]  pixel_xpos,
    output [10:0]  pixel_ypos,
    output [1:0]   zoom_source_active,
    output         zoom_fullscreen,
    output         quad_mode_active_out,
    output         bev_mode_active_out,
    output         bev_rd_en
);

localparam [10:0] H_DISP       = 11'd1280;
localparam [10:0] IMAGE_HEIGHT = 11'd360;

(* ASYNC_REG = "TRUE" *) reg quad_mode_meta;
(* ASYNC_REG = "TRUE" *) reg quad_mode_sync;
(* ASYNC_REG = "TRUE" *) reg bev_mode_meta;
(* ASYNC_REG = "TRUE" *) reg bev_mode_sync;
(* ASYNC_REG = "TRUE" *) reg zoom_fullscreen_meta;
(* ASYNC_REG = "TRUE" *) reg zoom_fullscreen_sync;
(* ASYNC_REG = "TRUE" *) reg [1:0] zoom_source_meta;
(* ASYNC_REG = "TRUE" *) reg [1:0] zoom_source_sync;
reg quad_mode_active;
reg bev_mode_active;
reg zoom_fullscreen_active;
reg [1:0] zoom_source_active_reg;
reg hdmi_vs_d;
reg hdmi_de_d;
reg [10:0] active_x;
reg [10:0] active_y;

// 输入RGB及控制信号在ADV7611时钟域延迟两拍，与DDR四宫格像素对齐。
// PLL的1倍输出相对输入时钟固定移相，TMDS编码器在数据稳定区采样。
reg [23:0] hdmi_rgb_d0;
reg [23:0] hdmi_rgb_d1;
reg        hdmi_hs_d0;
reg        hdmi_hs_d1;
reg        hdmi_vs_d0;
reg        hdmi_vs_d1;
reg        hdmi_de_d0;
reg        hdmi_de_d1;
reg [10:0] display_x;
reg [10:0] display_y;
(* ASYNC_REG = "TRUE" *) reg bev_ready_meta, bev_ready_sync;
reg [15:0] bev_rgb_d;
reg bev_valid_d, bev_valid_q;
(* ASYNC_REG = "TRUE" *) reg bev_fault_meta, bev_fault_sync;
reg bev_line_active;

wire vs_rise = hdmi_vs_in & ~hdmi_vs_d;
wire line_start = hdmi_de_in & ~hdmi_de_d;
wire [10:0] pixel_x_now = !hdmi_de_in ? 11'd0 :
                             (line_start ? 11'd1 : active_x + 1'b1);
wire [10:0] pixel_y_now = !hdmi_de_in ? 11'd0 :
                             (line_start ? active_y + 1'b1 : active_y);

wire [15:0] quad_rgb565;
wire [23:0] quad_rgb888 = {quad_rgb565[15:11], 3'b000,
                           quad_rgb565[10:5],  2'b00,
                           quad_rgb565[4:0],   3'b000};
// 放大使能与源编号独立：编号2现在是第三路摄像头，不再代表直播HDMI。
wire zoom_ddr_active = !quad_mode_active && !bev_mode_active &&
                       zoom_fullscreen_active;
wire [23:0] bev_rgb888 = {bev_rgb_d[15:11], 3'b000,
                          bev_rgb_d[10:5],  2'b00,
                          bev_rgb_d[4:0],   3'b000};
wire [23:0] video_rgb = bev_mode_active ?
                        ((!bev_fault_sync && bev_valid_q) ? bev_rgb888 : 24'h000000) :
                        quad_mode_active ? quad_rgb888 :
                        zoom_ddr_active ? quad_rgb888 : hdmi_rgb_d1;
wire [23:0] warning_video_rgb;
wire warning_video_hs;
wire warning_video_vs;
wire warning_video_de;

assign zoom_source_active = zoom_source_active_reg;
assign zoom_fullscreen    = zoom_ddr_active;
assign quad_mode_active_out = quad_mode_active;
assign bev_mode_active_out = bev_mode_active;
assign bev_rd_en = bev_mode_active && bev_ready_sync && hdmi_de_in &&
                   bev_line_active &&
                   !bev_fault_sync &&
                   (pixel_x_now >= 11'd280) && (pixel_x_now < 11'd1000);

// 四宫格DDR读链始终按统一的HDMI输出时序运行；模式切换只选择最终像素源。
assign rd_en       = hdmi_de_in;
assign video_vs    = hdmi_vs_in;
assign pixel_xpos  = pixel_x_now;
assign pixel_ypos  = pixel_y_now;

// 模式请求跨到ADV7611像素时钟域，并只在场同步边界生效，避免半帧切换。
always @(posedge stream_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        quad_mode_meta   <= 1'b0;
        quad_mode_sync   <= 1'b0;
        bev_mode_meta    <= 1'b0;
        bev_mode_sync    <= 1'b0;
        zoom_fullscreen_meta <= 1'b0;
        zoom_fullscreen_sync <= 1'b0;
        zoom_source_meta <= 2'b10;
        zoom_source_sync <= 2'b10;
        quad_mode_active <= 1'b0;
        bev_mode_active  <= 1'b0;
        zoom_fullscreen_active <= 1'b0;
        zoom_source_active_reg <= 2'b10;
        hdmi_vs_d        <= 1'b0;
    end else begin
        quad_mode_meta <= quad_mode_req;
        quad_mode_sync <= quad_mode_meta;
        bev_mode_meta  <= bev_mode_req;
        bev_mode_sync  <= bev_mode_meta;
        zoom_fullscreen_meta <= zoom_fullscreen_req;
        zoom_fullscreen_sync <= zoom_fullscreen_meta;
        zoom_source_meta <= zoom_source_req;
        zoom_source_sync <= zoom_source_meta;
        hdmi_vs_d      <= hdmi_vs_in;
        if (vs_rise) begin
            // 鸟瞰模式与四宫格互斥，且只在场同步边界切换。
            bev_mode_active  <= bev_mode_sync;
            quad_mode_active <= bev_mode_sync ? 1'b0 : quad_mode_sync;
            zoom_fullscreen_active <= zoom_fullscreen_sync;
            zoom_source_active_reg <= zoom_source_sync;
        end
    end
end

// The DDR width-converting FIFO returns one RGB565 pixel one stream clock
// after the request.  Register data once and delay validity twice so it aligns
// with the existing two-stage HDMI control pipeline.
always @(posedge stream_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        bev_ready_meta<=1'b0; bev_ready_sync<=1'b0;
        bev_rgb_d<=16'd0; bev_valid_d<=1'b0; bev_valid_q<=1'b0;
        bev_fault_meta<=1'b0; bev_fault_sync<=1'b0;
    end else begin
        bev_ready_meta<=bev_frame_ready;
        bev_ready_sync<=bev_ready_meta;
        bev_rgb_d<=bev_rgb565;
        // Use the FIFO's accepted-read indication, not the request.  When
        // the display FIFO is empty, bev_rd_en can remain high while no
        // address advances; marking that cycle valid would repeat stale data
        // and create the striped/torn BEV image seen on hardware.
        bev_valid_d<=bev_rd_valid;
        bev_valid_q<=bev_valid_d;
        bev_fault_meta<=bev_rd_fault;
        bev_fault_sync<=bev_fault_meta;
    end
end

// 从输入DE生成1基有效区坐标；场同步到来时重新从第一行计数。
always @(posedge stream_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        hdmi_de_d <= 1'b0;
        active_x  <= 11'd0;
        active_y  <= 11'd0;
        bev_line_active <= 1'b0;
    end else begin
        hdmi_de_d <= hdmi_de_in;
        if (vs_rise) begin
            active_x <= 11'd0;
            active_y <= 11'd0;
            bev_line_active <= 1'b0;
        end else if (hdmi_de_in) begin
            active_x <= pixel_x_now;
            if (line_start) begin
                active_y <= active_y + 1'b1;
                // The BEV window starts at x=280, so this registered line
                // qualifier has settled long before its first FIFO read.
                // Registering it also removes the y-counter carry/comparator
                // chain from the asynchronous FIFO read-enable timing path.
                bev_line_active <= (pixel_y_now >= 11'd1) &&
                                   (pixel_y_now <= 11'd720);
            end
        end else begin
            active_x <= 11'd0;
        end
    end
end

always @(posedge stream_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        hdmi_rgb_d0 <= 24'd0;
        hdmi_rgb_d1 <= 24'd0;
        hdmi_hs_d0  <= 1'b0;
        hdmi_hs_d1  <= 1'b0;
        hdmi_vs_d0  <= 1'b0;
        hdmi_vs_d1  <= 1'b0;
        hdmi_de_d0  <= 1'b0;
        hdmi_de_d1  <= 1'b0;
        display_x   <= 11'd0;
        display_y   <= 11'd0;
    end else begin
        hdmi_rgb_d0 <= hdmi_rgb_in;
        hdmi_rgb_d1 <= hdmi_rgb_d0;
        hdmi_hs_d0  <= hdmi_hs_in;
        hdmi_hs_d1  <= hdmi_hs_d0;
        hdmi_vs_d0  <= hdmi_vs_in;
        hdmi_vs_d1  <= hdmi_vs_d0;
        hdmi_de_d0  <= hdmi_de_in;
        hdmi_de_d1  <= hdmi_de_d0;
        display_x   <= pixel_x_now;
        display_y   <= pixel_y_now;
    end
end

hdmi_disply #(
    .IMAGE_HEIGHT(IMAGE_HEIGHT)
) u_hdmi_disply (
    .hdmi_clk        (stream_clk),
    .sys_rst_n       (sys_rst_n),
    .pixel_xpos      (display_x),
    .pixel_ypos      (display_y),
    .rd_data         (rd_data),
    .rd_lower_mode   (display_y > IMAGE_HEIGHT),
    .eth_frame_valid (eth_frame_valid),
    .rd_h_pixel      (H_DISP),
    .zoom_fullscreen (zoom_ddr_active),
    .zoom_source     (zoom_source_active_reg),
    .pixel_data      (quad_rgb565)
);

// 超声波告警在最终画面选择之后叠加；告警期间上层状态机会强制四宫格。
ultrasonic_warning_overlay u_ultrasonic_warning_overlay (
    .clk              (stream_clk),
    .rst_n            (sys_rst_n),
    .de_in            (hdmi_de_d1),
    .hs_in            (hdmi_hs_d1),
    .vs_in            (hdmi_vs_d1),
    .data_in          (video_rgb),
    .quad_mode_active (quad_mode_active),
    .alert_level      (ultrasonic_alert_level),
    .de               (warning_video_de),
    .hs               (warning_video_hs),
    .vs               (warning_video_vs),
    .data             (warning_video_rgb)
);

dvi_transmitter_top u_rgb2dvi (
    .pclk        (tx_pixel_clk),
    .pclk_x5     (tx_pixel_clk_5x),
    .reset_n     (sys_rst_n),
    .video_din   (warning_video_rgb),
    .video_hsync (warning_video_hs),
    .video_vsync (warning_video_vs),
    .video_de    (warning_video_de),
    .tmds_clk_p  (tmds_clk_p),
    .tmds_clk_n  (tmds_clk_n),
    .tmds_data_p (tmds_data_p),
    .tmds_data_n (tmds_data_n)
);

endmodule
