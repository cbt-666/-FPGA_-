`timescale 1ns/1ps

module tb_ddr_read_channel_alignment;
    reg         clk = 1'b0;
    reg         rst_n = 1'b0;
    reg         rd_en = 1'b0;
    reg         rd_lower_mode = 1'b1;
    wire [15:0] rd_data;

    always #5 clk = ~clk;

    ddr3_fifo_ctrl_top dut (
        .rst_n(rst_n), .rd_clk(clk), .ui_clk(clk),
        .wd_clk_1(clk), .wd_en_1(1'b0), .wd_data_1(16'd0), .wd_load_1(1'b0),
        .rfifo_wd_en_1(1'b0), .rfifo_wd_data_1(256'd0), .wfifo_rd_en_1(1'b0),
        .wfifo_rd_data_1(), .wfifo_rcount_1(), .rfifo_wcount_1(),
        .wd_clk_2(clk), .wd_en_2(1'b0), .wd_data_2(16'd0), .wd_load_2(1'b0),
        .rfifo_wd_en_2(1'b0), .rfifo_wd_data_2(256'd0), .wfifo_rd_en_2(1'b0),
        .wfifo_rd_data_2(), .wfifo_rcount_2(), .rfifo_wcount_2(),
        .wd_clk_3(clk), .wd_en_3(1'b0), .wd_data_3(16'd0), .wd_load_3(1'b0),
        .rfifo_wd_en_3(1'b0), .rfifo_wd_data_3(256'd0), .wfifo_rd_en_3(1'b0),
        .wfifo_rd_data_3(), .wfifo_rcount_3(), .rfifo_wcount_3(),
        .wd_clk_4(clk), .wd_en_4(1'b0), .wd_data_4(16'd0),
        .rfifo_wd_en_4(1'b0), .rfifo_wd_data_4(256'd0), .wfifo_rd_en_4(1'b0),
        .wfifo_rd_data_4(), .wfifo_rcount_4(), .rfifo_wcount_4(),
        .h_disp(13'd1280), .rd_load(1'b0), .rd_en(rd_en),
        .rd_lower_mode(rd_lower_mode), .rd_data(rd_data)
    );

    initial begin
        force dut.rd_data_1 = 16'h1111;
        force dut.rd_data_2 = 16'h2222;
        force dut.rd_data_3 = 16'h3333;
        force dut.rd_data_4 = 16'h4444;

        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        rd_en = 1'b1;

        // 完成左下640个像素的读请求。
        repeat (640) @(posedge clk);
        #1;
        if (dut.rd_cnt !== 13'd640 || rd_data !== 16'h3333)
            $fatal(1, "Lower-left data changed before the FIFO return boundary");

        // 下一拍发出右下首像素请求；返回多路器此后才切到第四路。
        @(negedge clk);
        if (!dut.rd_en_4 || rd_data !== 16'h3333)
            $fatal(1, "Ethernet request/data channel boundary is not aligned");
        @(posedge clk);
        #1;
        if (rd_data !== 16'h4444)
            $fatal(1, "Ethernet return channel did not switch after one read cycle");

        $display("PASS: DDR read channel selection is aligned at pixel 640/641");
        $finish;
    end
endmodule

module ctrl_fifo(
    input rst_n, input wd_clk, input rd_clk, input clk_100,
    input rd_load, input wr_load, input wfifo_wr_en, input [15:0] wfifo_wr_data,
    input wfifo_rd_en, output [255:0] wfifo_rd_data, output [10:0] wfifo_rcount,
    input rfifo_wr_en, input [255:0] rfifo_wr_data, output [10:0] rfifo_wcount,
    input rfifo_rd_en, output [15:0] rfifo_rd_data
);
    assign wfifo_rd_data = 256'd0;
    assign wfifo_rcount = 11'd0;
    assign rfifo_wcount = 11'd0;
    assign rfifo_rd_data = 16'd0;
endmodule
