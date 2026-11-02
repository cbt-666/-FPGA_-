// 超声波触发脉冲：50 MHz下每约70 ms输出一次15 us高电平。
module ultrasonic_trigger #(
    parameter integer PERIOD_CYCLES = 3_500_750,
    parameter integer PULSE_CYCLES  = 750
)(
    input  wire sys_clk,
    input  wire sys_rst_n,
    output wire trig
);

reg [21:0] period_cnt;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n)
        period_cnt <= 22'd0;
    else if (period_cnt >= PERIOD_CYCLES - 1)
        period_cnt <= 22'd0;
    else
        period_cnt <= period_cnt + 1'b1;
end

// 计数0保持低电平，随后完整输出PULSE_CYCLES个高电平周期。
assign trig = (period_cnt != 0) && (period_cnt <= PULSE_CYCLES);

endmodule
