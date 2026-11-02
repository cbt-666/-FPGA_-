// 超声波测距顶层：生成触发脉冲并将回波宽度转换为告警等级。
module ultrasonic_ranging(
    input  wire       sys_clk,
    input  wire       sys_rst_n,
    input  wire       echo,
    output wire       trig,
    output wire [1:0] alert_level
);

ultrasonic_trigger u_ultrasonic_trigger (
    .sys_clk   (sys_clk),
    .sys_rst_n (sys_rst_n),
    .trig      (trig)
);

ultrasonic_echo u_ultrasonic_echo (
    .sys_clk     (sys_clk),
    .sys_rst_n   (sys_rst_n),
    .echo        (echo),
    .alert_level (alert_level)
);

endmodule
