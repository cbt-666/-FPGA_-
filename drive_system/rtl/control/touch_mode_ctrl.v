// 显示状态与触摸键控制。
// 单击：保留原有全屏/四宫格/DDR放大全屏切换，但延迟一个双击窗口确认。
// 双击：进入或退出鸟瞰模式。鸟瞰、四宫格和DDR放大全屏彼此互斥。
// 长按：HDMI全屏时配合机械键选择原始/YRG/二值化显示；
//       四宫格时切换BACK_LEFT输入（摄像头3/HDMI）。
module touch_mode_ctrl #(
    parameter integer FILTER_CYCLES = 500000,
    parameter integer LONG_CYCLES   = 50000000,
    parameter integer DOUBLE_CYCLES = 20000000
)(
    input  wire       sys_clk,
    input  wire       sys_rst_n,
    input  wire       touch_key,
    input  wire [3:0] key_filtered,
    input  wire       warning_active,
    output reg        quad_mode_req,
    output reg        zoom_fullscreen_req,
    output reg  [1:0] zoom_source_req,
    output reg  [1:0] local_mode_req,
    output reg        bev_mode_req,
    output reg        back_left_hdmi_req,
    output wire       touch_active
);

(* ASYNC_REG = "TRUE" *) reg touch_meta;
(* ASYNC_REG = "TRUE" *) reg touch_sync;
reg touch_filtered;
reg touch_filtered_d;
reg [19:0] filter_cnt;
reg [25:0] hold_cnt;
reg [24:0] double_cnt;
reg long_seen;
reg back_left_toggle_done;
reg pending_click;
reg [3:0] key_filtered_d;
reg control_lockout;
reg warning_restore_quad;
reg warning_restore_bev;

wire touch_rise = touch_filtered && !touch_filtered_d;
wire touch_fall = !touch_filtered && touch_filtered_d;
wire hdmi_fullscreen_req = !quad_mode_req && !zoom_fullscreen_req && !bev_mode_req;
wire one_mode_key = ((!key_filtered[0] && key_filtered[1] && key_filtered[2]) ||
                     (key_filtered[0] && !key_filtered[1] && key_filtered[2]) ||
                     (key_filtered[0] && key_filtered[1] && !key_filtered[2])) &&
                    key_filtered[3];
wire [3:0] key_press = key_filtered_d & ~key_filtered;

assign touch_active = touch_filtered;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        touch_meta       <= 1'b0;
        touch_sync       <= 1'b0;
        touch_filtered   <= 1'b0;
        touch_filtered_d <= 1'b0;
        filter_cnt       <= 20'd0;
    end else begin
        touch_meta       <= touch_key;
        touch_sync       <= touch_meta;
        touch_filtered_d <= touch_filtered;
        if (touch_sync == touch_filtered)
            filter_cnt <= 20'd0;
        else if (filter_cnt >= FILTER_CYCLES - 1) begin
            touch_filtered <= touch_sync;
            filter_cnt <= 20'd0;
        end else
            filter_cnt <= filter_cnt + 1'b1;
    end
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        quad_mode_req       <= 1'b0;
        zoom_fullscreen_req <= 1'b0;
        zoom_source_req     <= 2'd2;
        local_mode_req      <= 2'd0;
        bev_mode_req        <= 1'b0;
        back_left_hdmi_req  <= 1'b0;
        hold_cnt             <= 26'd0;
        double_cnt           <= 25'd0;
        long_seen            <= 1'b0;
        back_left_toggle_done<= 1'b0;
        pending_click        <= 1'b0;
        key_filtered_d       <= 4'b1111;
        control_lockout      <= 1'b0;
        warning_restore_quad <= 1'b0;
        warning_restore_bev  <= 1'b0;
    end else begin
        key_filtered_d <= key_filtered;

        if (warning_active) begin
            if (!control_lockout) begin
                warning_restore_quad <= quad_mode_req;
                warning_restore_bev  <= bev_mode_req;
            end
            quad_mode_req       <= 1'b1;
            zoom_fullscreen_req <= 1'b0;
            zoom_source_req     <= 2'd2;
            local_mode_req      <= 2'd0;
            bev_mode_req        <= 1'b0;
            hold_cnt             <= 26'd0;
            double_cnt           <= 25'd0;
            long_seen            <= 1'b0;
            pending_click        <= 1'b0;
            control_lockout      <= 1'b1;
        end else if (control_lockout) begin
            quad_mode_req       <= warning_restore_bev ? 1'b0 : warning_restore_quad;
            zoom_fullscreen_req <= 1'b0;
            zoom_source_req     <= 2'd2;
            local_mode_req      <= 2'd0;
            bev_mode_req        <= warning_restore_bev;
            hold_cnt             <= 26'd0;
            double_cnt           <= 25'd0;
            long_seen            <= 1'b0;
            pending_click        <= 1'b0;
            if (!touch_filtered && (&key_filtered))
                control_lockout <= 1'b0;
        end else begin
            if (touch_rise) begin
                hold_cnt  <= 26'd0;
                long_seen <= 1'b0;
                back_left_toggle_done <= 1'b0;
            end else if (touch_filtered && !long_seen) begin
                if (hold_cnt >= LONG_CYCLES - 1) begin
                    long_seen     <= 1'b1;
                    pending_click <= 1'b0;
                    double_cnt    <= 25'd0;
                    // 四宫格下长按只切换BACK_LEFT输入，达到长按阈值时
                    // 立即执行一次；鸟瞰图和全屏模式不改变该选择。
                    if (quad_mode_req && !bev_mode_req && !zoom_fullscreen_req &&
                        !back_left_toggle_done) begin
                        back_left_hdmi_req   <= ~back_left_hdmi_req;
                        back_left_toggle_done <= 1'b1;
                    end
                end else
                    hold_cnt <= hold_cnt + 1'b1;
            end

            if (touch_filtered &&
                (long_seen || (hold_cnt >= LONG_CYCLES - 1)) &&
                hdmi_fullscreen_req && one_mode_key) begin
                if (!key_filtered[0])
                    local_mode_req <= 2'd0;
                else if (!key_filtered[1])
                    local_mode_req <= 2'd1;
                else
                    local_mode_req <= 2'd2;
            end

            if (touch_fall) begin
                hold_cnt <= 26'd0;
                if (!long_seen) begin
                    if (pending_click && (double_cnt != 0)) begin
                        pending_click       <= 1'b0;
                        double_cnt          <= 25'd0;
                        bev_mode_req        <= ~bev_mode_req;
                        quad_mode_req       <= 1'b0;
                        zoom_fullscreen_req <= 1'b0;
                        zoom_source_req     <= 2'd2;
                        local_mode_req      <= 2'd0;
                    end else begin
                        pending_click <= 1'b1;
                        double_cnt    <= DOUBLE_CYCLES;
                    end
                end
            end

            if (pending_click && !touch_filtered && !touch_fall) begin
                if (double_cnt > 0)
                    double_cnt <= double_cnt - 1'b1;
                else begin
                    pending_click <= 1'b0;
                    if (!bev_mode_req) begin
                        if (quad_mode_req) begin
                            quad_mode_req       <= 1'b0;
                            zoom_fullscreen_req <= 1'b0;
                            zoom_source_req     <= 2'd2;
                            local_mode_req      <= 2'd0;
                        end else if (zoom_fullscreen_req) begin
                            quad_mode_req       <= 1'b1;
                            zoom_fullscreen_req <= 1'b0;
                        end else begin
                            quad_mode_req       <= 1'b1;
                            zoom_fullscreen_req <= 1'b0;
                        end
                    end
                end
            end

            if (quad_mode_req && !touch_filtered && !touch_fall && !pending_click) begin
                if (key_press == 4'b0001 && !key_filtered[0] && key_filtered[3:1] == 3'b111) begin
                    quad_mode_req       <= 1'b0;
                    zoom_fullscreen_req <= 1'b1;
                    zoom_source_req     <= 2'd0;
                end else if (key_press == 4'b0010 && key_filtered[0] && !key_filtered[1] && key_filtered[3:2] == 2'b11) begin
                    quad_mode_req       <= 1'b0;
                    zoom_fullscreen_req <= 1'b1;
                    zoom_source_req     <= 2'd1;
                end else if (key_press == 4'b0100 && key_filtered[1:0] == 2'b11 && !key_filtered[2] && key_filtered[3]) begin
                    quad_mode_req       <= 1'b0;
                    zoom_fullscreen_req <= 1'b1;
                    zoom_source_req     <= 2'd2;
                end else if (key_press == 4'b1000 && key_filtered[2:0] == 3'b111 && !key_filtered[3]) begin
                    quad_mode_req       <= 1'b0;
                    zoom_fullscreen_req <= 1'b1;
                    zoom_source_req     <= 2'd3;
                end
            end
        end
    end
end

endmodule
