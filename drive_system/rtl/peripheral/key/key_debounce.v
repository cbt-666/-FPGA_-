// 按键消抖模块。开发板按键为低电平有效，输出保持同样的有效电平。
module key_debounce #(
    parameter integer CNT_MAX = 20'd1_000_000  // 50 MHz 时钟下约 20 ms
)(
    input  sys_clk,
    input  sys_rst_n,
    input  key,
    output reg key_filter
);

reg [19:0] cnt;
reg        key_d0;
reg        key_d1;

// 两级同步，避免按键异步输入直接进入计数逻辑。
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        key_d0 <= 1'b1;
        key_d1 <= 1'b1;
    end else begin
        key_d0 <= key;
        key_d1 <= key_d0;
    end
end

// 输入变化后重新装载消抖计数器，稳定 CNT_MAX 个周期后更新输出。
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        cnt <= 20'd0;
    end else if (key_d1 != key_d0) begin
        cnt <= CNT_MAX;
    end else if (cnt != 20'd0) begin
        cnt <= cnt - 1'b1;
    end
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n)
        key_filter <= 1'b1;
    else if (cnt == 20'd1)
        key_filter <= key_d1;
end

endmodule
