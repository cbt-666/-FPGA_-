// 复用工程已有Pango异步FIFO底层，配置为8192x16位图像跨时钟FIFO。
// 深度与参考RGB565发送工程一致，可吸收像素时钟与千兆网时钟的突发速率差。
`timescale 1ns / 1ps

module async_fifo_8192x16b (
    input             wr_clk,
    input             wr_rst,
    input             wr_en,
    input      [15:0] wr_data,
    output            wr_full,
    output            almost_full,
    output     [13:0] wr_water_level,
    input             rd_clk,
    input             rd_rst,
    input             rd_en,
    output     [15:0] rd_data,
    output            rd_empty,
    output            almost_empty,
    output     [13:0] rd_water_level
);

ipm2l_fifo_v1_7_async_fifo_2048x8b #(
    .c_CAS_MODE         ("18K"),
    .c_WR_DEPTH_WIDTH   (13),
    .c_WR_DATA_WIDTH    (16),
    .c_RD_DEPTH_WIDTH   (13),
    .c_RD_DATA_WIDTH    (16),
    .c_OUTPUT_REG       (0),
    .c_RD_OCE_EN        (0),
    .c_FAB_REG          (0),
    .c_RESET_TYPE       ("ASYNC"),
    .c_POWER_OPT        (0),
    .c_RD_CLK_OR_POL_INV(0),
    .c_WR_BYTE_EN       (0),
    .c_BE_WIDTH         (1),
    .c_FIFO_TYPE        ("ASYN"),
    .c_ALMOST_FULL_NUM  (8184),
    .c_ALMOST_EMPTY_NUM (4)
) u_fifo_16b (
    .wr_data       (wr_data),
    .wr_en         (wr_en),
    .wr_clk        (wr_clk),
    .wr_full       (wr_full),
    .wr_rst        (wr_rst),
    .wr_byte_en    (1'b1),
    .almost_full   (almost_full),
    .wr_water_level(wr_water_level),
    .rd_data       (rd_data),
    .rd_en         (rd_en),
    .rd_clk        (rd_clk),
    .rd_empty      (rd_empty),
    .rd_rst        (rd_rst),
    .rd_oce        (1'b0),
    .almost_empty  (almost_empty),
    .rd_water_level(rd_water_level)
);

endmodule
