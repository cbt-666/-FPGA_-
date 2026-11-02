module ddr3_fifo_ctrl_top(
    input rst_n, input rd_clk, input wire ui_clk,
    input wd_clk_1, input wd_en_1, input [15:0] wd_data_1, input wd_load_1,
    input rfifo_wd_en_1, input [255:0] rfifo_wd_data_1, input wfifo_rd_en_1,
    output [255:0] wfifo_rd_data_1, output [10:0] wfifo_rcount_1, output [10:0] rfifo_wcount_1,
    input wd_clk_2, input wd_en_2, input [15:0] wd_data_2, input wd_load_2,
    input rfifo_wd_en_2, input [255:0] rfifo_wd_data_2, input wfifo_rd_en_2,
    output [255:0] wfifo_rd_data_2, output [10:0] wfifo_rcount_2, output [10:0] rfifo_wcount_2,
    // 第三路BACK_LEFT当前摄像头RGB565
    input wd_clk_3, input wd_en_3, input [15:0] wd_data_3, input wd_load_3,
    input rfifo_wd_en_3, input [255:0] rfifo_wd_data_3, input wfifo_rd_en_3,
    output [255:0] wfifo_rd_data_3, output [10:0] wfifo_rcount_3, output [10:0] rfifo_wcount_3,
    // BACK_LEFT备用HDMI写FIFO；与摄像头3共用DDR第三路地址空间，读侧由调度器择一取数
    input wd_clk_5, input wd_en_5, input [15:0] wd_data_5, input wd_load_5,
    input wfifo_rd_en_5,
    output [255:0] wfifo_rd_data_5, output [10:0] wfifo_rcount_5,
    // 以太网RGB565第四路；网络流连续写入，不在帧边界复位写FIFO
    input wd_clk_4, input wd_en_4, input [15:0] wd_data_4,
    input rfifo_wd_en_4, input [255:0] rfifo_wd_data_4, input wfifo_rd_en_4,
    output [255:0] wfifo_rd_data_4, output [10:0] wfifo_rcount_4, output [10:0] rfifo_wcount_4,
    input [12:0] h_disp, input rd_load, input rd_en,
    input rd_lower_mode,
    input rd_fullscreen,
    input rd_row_active,
    input [1:0] rd_source_sel,
    output [15:0] rd_data
);
reg [12:0] rd_cnt;
// 全屏2倍放大：奇数输出行读取源行，偶数输出行复用行缓存。
reg [15:0] fs_line_buf [0:639];
reg        fs_wr_en_d;
reg [9:0]  fs_wr_addr_d;
wire rd_fs_req = rd_fullscreen && rd_en && rd_row_active && !rd_cnt[0];
wire rd_en_1 = rd_fullscreen ? ((rd_source_sel == 2'd0) ? rd_fs_req : 1'b0) :
               (!rd_lower_mode && (rd_cnt < h_disp[12:1]) ? rd_en : 1'b0);
wire rd_en_2 = rd_fullscreen ? ((rd_source_sel == 2'd1) ? rd_fs_req : 1'b0) :
               (!rd_lower_mode && (rd_cnt >= h_disp[12:1]) ? rd_en : 1'b0);
wire rd_en_3 = rd_fullscreen ? ((rd_source_sel == 2'd2) ? rd_fs_req : 1'b0) :
               ( rd_lower_mode && (rd_cnt < h_disp[12:1]) ? rd_en : 1'b0);
wire rd_en_4 = rd_fullscreen ? ((rd_source_sel == 2'd3) ? rd_fs_req : 1'b0) :
               ( rd_lower_mode && (rd_cnt >= h_disp[12:1]) ? rd_en : 1'b0);
wire [15:0] rd_data_1, rd_data_2, rd_data_3, rd_data_4;
wire [15:0] rd_fs_fifo_data = (rd_source_sel == 2'd0) ? rd_data_1 :
                               (rd_source_sel == 2'd1) ? rd_data_2 :
                               (rd_source_sel == 2'd2) ? rd_data_3 : rd_data_4;
wire [9:0] fs_line_addr = rd_cnt[10:1];

always @(posedge rd_clk or negedge rst_n) begin
    if (!rst_n) begin
        fs_wr_en_d   <= 1'b0;
        fs_wr_addr_d <= 10'd0;
    end else begin
        fs_wr_en_d   <= rd_fs_req;
        fs_wr_addr_d <= fs_line_addr;
        if (fs_wr_en_d)
            fs_line_buf[fs_wr_addr_d] <= rd_fs_fifo_data;
    end
end

// 读FIFO为同步读出，rd_cnt在返回数据有效时已比请求计数前进一拍。
// 返回通道在rd_cnt超过半屏边界后切换，确保第640个像素仍取左侧FIFO。
assign rd_data = rd_fullscreen ?
                 (rd_row_active ? rd_fs_fifo_data : fs_line_buf[fs_line_addr]) :
                 (rd_lower_mode ?
                  ((rd_cnt <= h_disp[12:1]) ? rd_data_3 : rd_data_4) :
                  ((rd_cnt <= h_disp[12:1]) ? rd_data_1 : rd_data_2));

always @(posedge rd_clk or negedge rst_n) begin
    if (!rst_n) rd_cnt <= 13'd0;
    else if (rd_en) rd_cnt <= rd_cnt + 1'b1;
    else rd_cnt <= 13'd0;
end

ctrl_fifo u_ctrl_fifo_1(
    .rst_n(rst_n), .wd_clk(wd_clk_1), .rd_clk(rd_clk), .clk_100(ui_clk),
    .rd_load(rd_load), .wr_load(wd_load_1), .wfifo_wr_en(wd_en_1), .wfifo_wr_data(wd_data_1),
    .wfifo_rd_en(wfifo_rd_en_1), .wfifo_rd_data(wfifo_rd_data_1), .wfifo_rcount(wfifo_rcount_1),
    .rfifo_wr_en(rfifo_wd_en_1), .rfifo_wr_data(rfifo_wd_data_1), .rfifo_wcount(rfifo_wcount_1),
    .rfifo_rd_en(rd_en_1), .rfifo_rd_data(rd_data_1));
ctrl_fifo u_ctrl_fifo_2(
    .rst_n(rst_n), .wd_clk(wd_clk_2), .rd_clk(rd_clk), .clk_100(ui_clk),
    .rd_load(rd_load), .wr_load(wd_load_2), .wfifo_wr_en(wd_en_2), .wfifo_wr_data(wd_data_2),
    .wfifo_rd_en(wfifo_rd_en_2), .wfifo_rd_data(wfifo_rd_data_2), .wfifo_rcount(wfifo_rcount_2),
    .rfifo_wr_en(rfifo_wd_en_2), .rfifo_wr_data(rfifo_wd_data_2), .rfifo_wcount(rfifo_wcount_2),
    .rfifo_rd_en(rd_en_2), .rfifo_rd_data(rd_data_2));
ctrl_fifo u_ctrl_fifo_3(
    .rst_n(rst_n), .wd_clk(wd_clk_3), .rd_clk(rd_clk), .clk_100(ui_clk),
    .rd_load(rd_load), .wr_load(wd_load_3), .wfifo_wr_en(wd_en_3), .wfifo_wr_data(wd_data_3),
    .wfifo_rd_en(wfifo_rd_en_3), .wfifo_rd_data(wfifo_rd_data_3), .wfifo_rcount(wfifo_rcount_3),
    .rfifo_wr_en(rfifo_wd_en_3), .rfifo_wr_data(rfifo_wd_data_3), .rfifo_wcount(rfifo_wcount_3),
    .rfifo_rd_en(rd_en_3), .rfifo_rd_data(rd_data_3));
// HDMI备用写FIFO。读FIFO接口在本工程中不使用，但复用ctrl_fifo可保持
// 与摄像头写FIFO完全一致的异步复位和水位计数行为。
ctrl_fifo u_ctrl_fifo_5(
    .rst_n(rst_n), .wd_clk(wd_clk_5), .rd_clk(rd_clk), .clk_100(ui_clk),
    .rd_load(rd_load), .wr_load(wd_load_5), .wfifo_wr_en(wd_en_5), .wfifo_wr_data(wd_data_5),
    .wfifo_rd_en(wfifo_rd_en_5), .wfifo_rd_data(wfifo_rd_data_5), .wfifo_rcount(wfifo_rcount_5),
    .rfifo_wr_en(1'b0), .rfifo_wr_data(256'd0), .rfifo_wcount(),
    .rfifo_rd_en(1'b0), .rfifo_rd_data());
ctrl_fifo u_ctrl_fifo_4(
    .rst_n(rst_n), .wd_clk(wd_clk_4), .rd_clk(rd_clk), .clk_100(ui_clk),
    .rd_load(rd_load), .wr_load(1'b0), .wfifo_wr_en(wd_en_4), .wfifo_wr_data(wd_data_4),
    .wfifo_rd_en(wfifo_rd_en_4), .wfifo_rd_data(wfifo_rd_data_4), .wfifo_rcount(wfifo_rcount_4),
    .rfifo_wr_en(rfifo_wd_en_4), .rfifo_wr_data(rfifo_wd_data_4), .rfifo_wcount(rfifo_wcount_4),
    .rfifo_rd_en(rd_en_4), .rfifo_rd_data(rd_data_4));
endmodule
