// 超声波三级告警蜂鸣器：15~20 cm为1 Hz，10~15 cm为2 Hz，10 cm内为5 Hz。
module warning_buzzer(
    input  wire       sys_clk,
    input  wire       sys_rst_n,
    input  wire [1:0] alert_level,
    output reg        beep
);

localparam [25:0] HALF_1HZ = 26'd25_000_000;
localparam [25:0] HALF_2HZ = 26'd12_500_000;
localparam [25:0] HALF_5HZ = 26'd5_000_000;

reg [25:0] count;
reg [1:0]  level_d;
wire [25:0] half_period = (alert_level == 2'd1) ? HALF_1HZ :
                          (alert_level == 2'd2) ? HALF_2HZ : HALF_5HZ;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        count   <= 26'd0;
        level_d <= 2'd0;
        beep    <= 1'b0;
    end else if (alert_level == 2'd0) begin
        count   <= 26'd0;
        level_d <= 2'd0;
        beep    <= 1'b0;
    end else if (level_d != alert_level) begin
        count   <= 26'd0;
        level_d <= alert_level;
        beep    <= 1'b0;
    end else if (count >= half_period - 1'b1) begin
        count <= 26'd0;
        beep  <= ~beep;
    end else begin
        count <= count + 1'b1;
    end
end

endmodule
