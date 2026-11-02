`timescale 1ns/1ps

// DDR三通道帧边界回归测试：帧沿刻意插入正在进行的读/写事务，
// 检查待处理机制能否在安全边界复位地址，并阻止读FIFO复位期间提前读取。
module tb_ddr3_rw_ctrl_frame_reset;
localparam [3:0] DONE    = 4'd1;
localparam [3:0] WRITE_1 = 4'd2;
localparam [3:0] READ_1  = 4'd3;
localparam [3:0] WRITE_4 = 4'd8;
localparam [9:0] BURST   = 10'd16;
localparam integer ETH_FRAME_BURSTS = 14400 / BURST;

reg clk = 1'b0;
reg rst_n = 1'b0;
reg ddr3_init_done = 1'b0;
reg rd_load = 1'b0;
reg wd_load_1 = 1'b0;
reg wd_load_2 = 1'b0;
reg wd_load_3 = 1'b0;
reg [10:0] rfifo_wcount_1 = 11'd1000;
reg [10:0] rfifo_wcount_2 = 11'd1000;
reg [10:0] rfifo_wcount_3 = 11'd1000;
reg [10:0] rfifo_wcount_4 = 11'd1000;
reg [10:0] wfifo_rcount_1 = 11'd0;
reg [10:0] wfifo_rcount_2 = 11'd0;
reg [10:0] wfifo_rcount_3 = 11'd0;
reg [10:0] wfifo_rcount_4 = 11'd0;
reg wd_fifo_re = 1'b0;
reg wd_finish = 1'b0;
reg rd_fifo_we = 1'b0;
reg rd_finish = 1'b0;
integer i;

wire wfifo_rd_en_1, wfifo_rd_en_2, wfifo_rd_en_3, wfifo_rd_en_4;
wire rfifo_wd_en_1, rfifo_wd_en_2, rfifo_wd_en_3, rfifo_wd_en_4;
wire [255:0] rfifo_wd_data_1, rfifo_wd_data_2, rfifo_wd_data_3, rfifo_wd_data_4;
wire wd_req, rd_req;
wire [23:0] wd_addr, rd_addr;
wire [9:0] wd_len, rd_len;
wire [255:0] wd_ddr3_data;
wire eth_frame_ready;

always #5 clk = ~clk;

ddr3_rw_ctrl dut(
    .rst_n(rst_n), .clk(clk), .ddr3_init_done(ddr3_init_done),
    .addr_rd_min(27'd0), .rd_burst_len(BURST),
    .addr_wd_min(27'd0), .wd_burst_len(BURST), .ddr3_pingpang_en(1'b1),
    .rd_load(rd_load), .wd_load_1(wd_load_1),
    .wd_load_2(wd_load_2), .wd_load_3(wd_load_3),
    .rfifo_wcount_1(rfifo_wcount_1), .wfifo_rcount_1(wfifo_rcount_1),
    .wfifo_rd_en_1(wfifo_rd_en_1), .wfifo_rd_data_1(256'd0),
    .rfifo_wd_en_1(rfifo_wd_en_1), .rfifo_wd_data_1(rfifo_wd_data_1),
    .rfifo_wcount_2(rfifo_wcount_2), .wfifo_rcount_2(wfifo_rcount_2),
    .wfifo_rd_en_2(wfifo_rd_en_2), .wfifo_rd_data_2(256'd0),
    .rfifo_wd_en_2(rfifo_wd_en_2), .rfifo_wd_data_2(rfifo_wd_data_2),
    .rfifo_wcount_3(rfifo_wcount_3), .wfifo_rcount_3(wfifo_rcount_3),
    .wfifo_rd_en_3(wfifo_rd_en_3), .wfifo_rd_data_3(256'd0),
    .rfifo_wd_en_3(rfifo_wd_en_3), .rfifo_wd_data_3(rfifo_wd_data_3),
    .rfifo_wcount_4(rfifo_wcount_4), .wfifo_rcount_4(wfifo_rcount_4),
    .wfifo_rd_en_4(wfifo_rd_en_4), .wfifo_rd_data_4(256'd0),
    .rfifo_wd_en_4(rfifo_wd_en_4), .rfifo_wd_data_4(rfifo_wd_data_4),
    .wd_fifo_re(wd_fifo_re), .wd_finish(wd_finish),
    .wd_req(wd_req), .wd_addr(wd_addr), .wd_len(wd_len),
    .rd_fifo_we(rd_fifo_we), .rd_finish(rd_finish),
    .rd_req(rd_req), .rd_addr(rd_addr), .rd_len(rd_len),
    .wd_ddr3_data(wd_ddr3_data), .rd_ddr3_data(256'd0),
    .eth_frame_ready(eth_frame_ready)
);

task fail;
    input [8*80-1:0] message;
    begin
        $fatal(1, "FAIL: %0s", message);
    end
endtask

task pulse_wd_finish;
    begin
        @(negedge clk); wd_finish = 1'b1;
        @(negedge clk); wd_finish = 1'b0;
    end
endtask

task pulse_rd_finish;
    begin
        @(negedge clk); rd_finish = 1'b1;
        @(negedge clk); rd_finish = 1'b0;
    end
endtask

initial begin
    repeat(4) @(posedge clk);
    rst_n = 1'b1;
    ddr3_init_done = 1'b1;
    wait(dut.state == DONE);

    // 先完成一个写突发，使通道1地址离开页首。
    wfifo_rcount_1 = 11'd32;
    wait(dut.state == WRITE_1);
    wait(wd_req);
    pulse_wd_finish();
    wfifo_rcount_1 = 11'd0;
    wait(dut.state == DONE);
    @(posedge clk);
    if(dut.wd_addr_1 != BURST)
        fail("write address did not advance before frame-reset test");

    // 第二个写事务中插入输入帧沿；旧实现会漏掉这次地址复位。
    wfifo_rcount_1 = 11'd32;
    wait(dut.state == WRITE_1);
    wait(wd_req);
    @(negedge clk); wd_load_1 = 1'b1;
    wait(dut.wd_rst_1);
    @(negedge clk); wd_load_1 = 1'b0;
    if(dut.state != WRITE_1)
        fail("write-frame edge was not injected during WRITE_1");
    pulse_wd_finish();
    wfifo_rcount_1 = 11'd0;
    wait(dut.state == DONE);
    repeat(2) @(posedge clk);
    if(dut.wd_addr_1 != 20'd0 || dut.wd_addr_reset_pending_1)
        fail("write-frame reset was lost across an active DDR transaction");

    // 先完成一个读突发，使读地址离开页首。
    rfifo_wcount_1 = 11'd0;
    wait(dut.state == READ_1);
    wait(rd_req);
    pulse_rd_finish();
    rfifo_wcount_1 = 11'd1000;
    wait(dut.state == DONE);
    @(posedge clk);
    if(dut.rd_addr_1 != BURST)
        fail("read address did not advance before frame-reset test");

    // 第二个读事务中插入输出帧沿。
    rfifo_wcount_1 = 11'd0;
    wait(dut.state == READ_1);
    wait(rd_req);
    @(negedge clk); rd_load = 1'b1;
    wait(dut.rd_rst);
    @(negedge clk); rd_load = 1'b0;
    if(dut.state != READ_1)
        fail("output-frame edge was not injected during READ_1");
    pulse_rd_finish();
    rfifo_wcount_1 = 11'd1000;
    wait(dut.state == DONE);
    repeat(2) @(posedge clk);
    if(dut.rd_addr_1 != 20'd0 || dut.rd_addr_reset_pending)
        fail("read-frame reset was lost across an active DDR transaction");

    // 复位等待的1000拍内即使FIFO报告缺数，也不能发起普通读事务。
    rfifo_wcount_1 = 11'd0;
    for(i=0; i<995; i=i+1) begin
        @(posedge clk);
        if(dut.state == READ_1)
            fail("read transaction started before frame-reset hold expired");
    end
    wait(dut.state == READ_1);
    if(dut.rd_addr_1 != 20'd0)
        fail("first read after frame reset did not start at page offset zero");

    // 完成当前读事务，再连续写满以太网通道的一帧，检查自动换页。
    pulse_rd_finish();
    rfifo_wcount_1 = 11'd1000;
    // 少于一次事务的数据不能启动第四路，否则AXI主控会继续读取并读空FIFO。
    wfifo_rcount_4 = (BURST >> 2);
    repeat(8) begin
        @(posedge clk);
        if(dut.state == WRITE_4)
            fail("Ethernet write started with an underfilled FIFO");
    end

    wfifo_rcount_4 = BURST - 1'b1;
    for(i=0; i<ETH_FRAME_BURSTS; i=i+1) begin
        wait(dut.state == WRITE_4);
        wait(wd_req);
        pulse_wd_finish();
    end
    wfifo_rcount_4 = 11'd0;
    wait(dut.state == DONE);
    @(posedge clk);
    if(dut.wd_addr_4 != 20'd0 || dut.wpage4 != 2'd1 || !dut.eth_frame_ready)
        fail("Ethernet channel did not roll over after one 640x360 frame");

    $display("PASS: frame-reset requests survive active DDR transactions");
    $finish;
end

initial begin
    #1000000;
    fail("simulation timeout");
end
endmodule
