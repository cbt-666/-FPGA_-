// 超声波回波计时与三级告警判定。
// 50 MHz下约2900个时钟对应1 cm，分级保持参考工程的10/15/20 cm阈值。
module ultrasonic_echo #(
    parameter integer LEVEL3_MAX_CYCLES = 29_000,
    parameter integer LEVEL2_MAX_CYCLES = 43_500,
    parameter integer LEVEL1_MAX_CYCLES = 58_000,
    parameter integer ECHO_TIMEOUT_CYCLES = 3_000_000,
    parameter integer STALE_CLEAR_CYCLES  = 5_000_000
)(
    input  wire       sys_clk,
    input  wire       sys_rst_n,
    input  wire       echo,
    output reg  [1:0] alert_level
);

localparam [1:0] WAIT_RISE = 2'd0;
localparam [1:0] COUNT_HIGH = 2'd1;
localparam [1:0] WAIT_LOW = 2'd2;

(* ASYNC_REG = "TRUE" *) reg echo_meta;
(* ASYNC_REG = "TRUE" *) reg echo_sync;
reg [1:0]  state;
reg [31:0] echo_cnt;
reg [31:0] stale_cnt;
reg        sample_valid;
reg [1:0]  sample_level;

// echo与系统时钟异步，进入计数状态机前先做双触发同步。
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        echo_meta <= 1'b0;
        echo_sync <= 1'b0;
    end else begin
        echo_meta <= echo;
        echo_sync <= echo_meta;
    end
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        state        <= WAIT_RISE;
        echo_cnt     <= 32'd0;
        sample_valid <= 1'b0;
        sample_level <= 2'd0;
    end else begin
        sample_valid <= 1'b0;
        case (state)
            WAIT_RISE: begin
                echo_cnt <= 32'd0;
                if (echo_sync) begin
                    echo_cnt <= 32'd1;
                    state    <= COUNT_HIGH;
                end
            end

            COUNT_HIGH: begin
                if (!echo_sync) begin
                    sample_valid <= 1'b1;
                    if (echo_cnt < LEVEL3_MAX_CYCLES)
                        sample_level <= 2'd3;
                    else if (echo_cnt < LEVEL2_MAX_CYCLES)
                        sample_level <= 2'd2;
                    else if (echo_cnt < LEVEL1_MAX_CYCLES)
                        sample_level <= 2'd1;
                    else
                        sample_level <= 2'd0;
                    echo_cnt <= 32'd0;
                    state    <= WAIT_RISE;
                end else if (echo_cnt >= ECHO_TIMEOUT_CYCLES - 1) begin
                    // echo卡高时立即撤销旧告警，等待信号恢复低电平。
                    sample_valid <= 1'b1;
                    sample_level <= 2'd0;
                    echo_cnt     <= 32'd0;
                    state        <= WAIT_LOW;
                end else begin
                    echo_cnt <= echo_cnt + 1'b1;
                end
            end

            WAIT_LOW: begin
                echo_cnt <= 32'd0;
                if (!echo_sync)
                    state <= WAIT_RISE;
            end

            default: begin
                state    <= WAIT_RISE;
                echo_cnt <= 32'd0;
            end
        endcase
    end
end

// 连续没有新回波时清除旧告警，避免目标移走或传感器断开后画面一直报警。
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        stale_cnt   <= 32'd0;
        alert_level <= 2'd0;
    end else if (sample_valid) begin
        stale_cnt   <= 32'd0;
        alert_level <= sample_level;
    end else if (stale_cnt >= STALE_CLEAR_CYCLES - 1) begin
        stale_cnt   <= stale_cnt;
        alert_level <= 2'd0;
    end else begin
        stale_cnt <= stale_cnt + 1'b1;
    end
end

endmodule
