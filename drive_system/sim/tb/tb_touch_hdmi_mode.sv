`timescale 1ns/1ps

module tb_touch_hdmi_mode;
    reg sys_clk = 1'b0;
    reg stream_clk = 1'b0;
    reg rst_n = 1'b0;
    reg touch_key = 1'b0;
    reg [3:0] key_filtered = 4'b1111;
    reg warning_active = 1'b0;
    reg hdmi_vs_in = 1'b0;
    wire quad_mode_req;
    wire zoom_fullscreen_req;
    wire [1:0] zoom_source_req;
    wire [1:0] local_mode_req;
    wire bev_mode_req;
    wire back_left_hdmi_req;
    wire zoom_fullscreen;
    wire [1:0] zoom_source_active;
    wire quad_mode_active;

    always #10 sys_clk = ~sys_clk;
    always #7 stream_clk = ~stream_clk;

    touch_mode_ctrl #(
        .FILTER_CYCLES(2),
        .LONG_CYCLES(8),
        .DOUBLE_CYCLES(16)
    ) u_ctrl (
        .sys_clk(sys_clk),
        .sys_rst_n(rst_n),
        .touch_key(touch_key),
        .key_filtered(key_filtered),
        .warning_active(warning_active),
        .quad_mode_req(quad_mode_req),
        .zoom_fullscreen_req(zoom_fullscreen_req),
        .zoom_source_req(zoom_source_req),
        .local_mode_req(local_mode_req),
        .bev_mode_req(bev_mode_req),
        .back_left_hdmi_req(back_left_hdmi_req),
        .touch_active()
    );

    hdmi_top u_hdmi (
        .stream_clk(stream_clk),
        .tx_pixel_clk(stream_clk),
        .tx_pixel_clk_5x(stream_clk),
        .sys_rst_n(rst_n),
        .quad_mode_req(quad_mode_req),
        .bev_mode_req(bev_mode_req),
        .zoom_fullscreen_req(zoom_fullscreen_req),
        .zoom_source_req(zoom_source_req),
        .bev_rgb565(16'd0),
        .bev_rd_valid(1'b0),
        .bev_rd_fault(1'b0),
        .bev_frame_ready(1'b0),
        .ultrasonic_alert_level(warning_active ? 2'd1 : 2'd0),
        .hdmi_rgb_in(24'h123456),
        .hdmi_hs_in(1'b0),
        .hdmi_vs_in(hdmi_vs_in),
        .hdmi_de_in(1'b1),
        .tmds_clk_p(), .tmds_clk_n(), .tmds_data_p(), .tmds_data_n(),
        .rd_data(16'hf800),
        .eth_frame_valid(1'b1),
        .rd_en(), .video_vs(), .pixel_xpos(), .pixel_ypos(),
        .zoom_source_active(zoom_source_active),
        .quad_mode_active_out(quad_mode_active),
        .bev_mode_active_out(),
        .bev_rd_en(),
        .zoom_fullscreen(zoom_fullscreen)
    );

    task short_touch;
        begin
            @(negedge sys_clk); touch_key = 1'b1;
            repeat (5) @(negedge sys_clk);
            touch_key = 1'b0;
            // 单击须等双击窗口结束，才提交显示切换。
            repeat (25) @(negedge sys_clk);
        end
    endtask

    task long_touch_with_key;
        input [3:0] keys;
        begin
            @(negedge sys_clk); key_filtered = keys; touch_key = 1'b1;
            repeat (14) @(negedge sys_clk);
            touch_key = 1'b0;
            repeat (5) @(negedge sys_clk);
            key_filtered = 4'b1111;
            repeat (2) @(negedge sys_clk);
        end
    endtask

    task long_touch_only;
        begin
            @(negedge sys_clk); touch_key = 1'b1;
            repeat (14) @(negedge sys_clk);
            touch_key = 1'b0;
            repeat (5) @(negedge sys_clk);
        end
    endtask

    task long_touch_then_key;
        input [3:0] keys;
        begin
            @(negedge sys_clk); touch_key = 1'b1;
            // 先达到长按阈值，再按模式键，覆盖板上实际操作顺序。
            repeat (14) @(negedge sys_clk);
            key_filtered = keys;
            repeat (3) @(negedge sys_clk);
            touch_key = 1'b0;
            repeat (5) @(negedge sys_clk);
            key_filtered = 4'b1111;
            repeat (2) @(negedge sys_clk);
        end
    endtask

    task press_key;
        input integer index;
        begin
            @(negedge sys_clk); key_filtered[index] = 1'b0;
            repeat (3) @(negedge sys_clk);
            key_filtered[index] = 1'b1;
            repeat (2) @(negedge sys_clk);
        end
    endtask

    task apply_frame_boundary;
        begin
            repeat (5) @(negedge stream_clk);
            hdmi_vs_in = 1'b1;
            repeat (2) @(negedge stream_clk);
            hdmi_vs_in = 1'b0;
            repeat (2) @(negedge stream_clk);
        end
    endtask

    task expect_zoom_source;
        input [1:0] source;
        begin
            press_key(source);
            if (quad_mode_req || !zoom_fullscreen_req || zoom_source_req != source)
                $fatal(1, "DDR zoom request failed for source %0d", source);
            if (!quad_mode_active || zoom_fullscreen)
                $fatal(1, "Display must remain quad until the frame boundary");
            apply_frame_boundary();
            if (!zoom_fullscreen || zoom_source_active != source || !u_hdmi.zoom_ddr_active)
                $fatal(1, "DDR zoom was not frame-latched for source %0d", source);
            if (u_hdmi.u_hdmi_disply.zoom_source !== source)
                $fatal(1, "Zoom label source must match active DDR source %0d", source);
            if (u_hdmi.video_rgb !== 24'hf80000)
                $fatal(1, "DDR zoom source %0d must display DDR pixels, not live HDMI", source);
            press_key((source + 1) % 4);
            apply_frame_boundary();
            if (!zoom_fullscreen || zoom_source_active != source)
                $fatal(1, "DDR zoom must stay on source %0d until returning to quad", source);
            short_touch();
            if (!quad_mode_req || zoom_fullscreen_req)
                $fatal(1, "DDR zoom source %0d did not return to quad", source);
            apply_frame_boundary();
            if (zoom_fullscreen || !quad_mode_active)
                $fatal(1, "DDR zoom source %0d did not return to quad at frame boundary", source);
        end
    endtask

    initial begin
        repeat (4) @(negedge sys_clk);
        rst_n = 1'b1;
        repeat (4) @(negedge sys_clk);

        if (quad_mode_req || zoom_fullscreen_req || zoom_source_req != 2'd2 || local_mode_req != 2'd0)
            $fatal(1, "Reset must select original live HDMI");

        short_touch();
        if (!quad_mode_req || zoom_fullscreen_req)
            $fatal(1, "Live HDMI short touch must enter quad mode");
        apply_frame_boundary();

        // 四宫格长按触摸键只切换BACK_LEFT输入，且一次长按只切换一次。
        long_touch_only();
        if (!back_left_hdmi_req)
            $fatal(1, "Quad long touch must select HDMI for BACK_LEFT");
        long_touch_only();
        if (back_left_hdmi_req)
            $fatal(1, "Second quad long touch must restore camera3 for BACK_LEFT");

        // 四宫格的四个机械键分别选择四路DDR；source 2也必须走DDR，而非直播HDMI。
        expect_zoom_source(2'd0);
        expect_zoom_source(2'd1);
        expect_zoom_source(2'd2);
        expect_zoom_source(2'd3);

        // 四宫格短按固定返回原始直播HDMI。
        short_touch();
        if (quad_mode_req || zoom_fullscreen_req || zoom_source_req != 2'd2 || local_mode_req != 2'd0)
            $fatal(1, "Quad short touch must return to original live HDMI");
        apply_frame_boundary();

        if (zoom_fullscreen || u_hdmi.video_rgb !== 24'h123456)
            $fatal(1, "Quad exit must display live HDMI, not DDR camera 3");

        long_touch_with_key(4'b1101); // key1 -> YRG
        if (local_mode_req != 2'd1 || quad_mode_req || zoom_fullscreen_req)
            $fatal(1, "Live HDMI long touch + key1 must select YRG");

        long_touch_with_key(4'b1011); // key2 -> binary
        if (local_mode_req != 2'd2)
            $fatal(1, "Live HDMI long touch + key2 must select binary");

        long_touch_then_key(4'b1101); // 长按成立后再按key1 -> YRG
        if (local_mode_req != 2'd1)
            $fatal(1, "Key pressed after long-touch threshold must select YRG");

        // 多键同时按下不允许改变模式。
        long_touch_with_key(4'b1001);
        if (local_mode_req != 2'd1)
            $fatal(1, "Multiple keys must not change local mode");

        short_touch();
        apply_frame_boundary();
        press_key(2);
        apply_frame_boundary();
        long_touch_with_key(4'b1101);
        if (local_mode_req != 2'd1 || !zoom_fullscreen_req)
            $fatal(1, "Long touch must be ignored during DDR fullscreen");

        // 任意显示状态下告警均覆盖为四宫格，期间按键无效，结束固定回原始HDMI。
        warning_active = 1'b1;
        repeat (2) @(negedge sys_clk);
        if (!quad_mode_req || zoom_fullscreen_req || local_mode_req != 2'd0)
            $fatal(1, "Warning must force quad and clear local processing mode");
        key_filtered = 4'b1110;
        touch_key = 1'b1;
        repeat (12) @(negedge sys_clk);
        if (!quad_mode_req || zoom_fullscreen_req || local_mode_req != 2'd0)
            $fatal(1, "Display controls must be locked during warning");
        warning_active = 1'b0;
        repeat (3) @(negedge sys_clk);
        if (quad_mode_req || zoom_fullscreen_req || zoom_source_req != 2'd2 || local_mode_req != 2'd0)
            $fatal(1, "Warning clear must restore original live HDMI");
        touch_key = 1'b0;
        key_filtered = 4'b1111;
        repeat (5) @(negedge sys_clk);

        // 告警前若已处于四宫格，告警结束后应继续保留四宫格。
        short_touch();
        if (!quad_mode_req || zoom_fullscreen_req)
            $fatal(1, "Setup for quad warning preservation failed");
        warning_active = 1'b1;
        repeat (3) @(negedge sys_clk);
        if (!quad_mode_req || zoom_fullscreen_req || local_mode_req != 2'd0)
            $fatal(1, "Warning must force quad from pre-existing quad");
        warning_active = 1'b0;
        repeat (3) @(negedge sys_clk);
        if (!quad_mode_req || zoom_fullscreen_req || zoom_source_req != 2'd2 || local_mode_req != 2'd0)
            $fatal(1, "Warning clear must preserve pre-existing quad");
        repeat (2) @(negedge sys_clk);

        $display("PASS: touch modes, DDR zoom and ultrasonic warning override");
        $finish;
    end
endmodule

module hdmi_disply #(
    parameter [10:0] IMAGE_HEIGHT = 11'd360
)(
    input hdmi_clk, input sys_rst_n,
    input [10:0] pixel_xpos, input [10:0] pixel_ypos,
    input [15:0] rd_data, input rd_lower_mode, input eth_frame_valid,
    input [10:0] rd_h_pixel, input zoom_fullscreen, input [1:0] zoom_source,
    output [15:0] pixel_data
);
    assign pixel_data = rd_data;
endmodule

module dvi_transmitter_top(
    input pclk, input pclk_x5, input reset_n,
    input [23:0] video_din, input video_hsync, input video_vsync, input video_de,
    output tmds_clk_p, output tmds_clk_n,
    output [2:0] tmds_data_p, output [2:0] tmds_data_n
);
    assign tmds_clk_p = 1'b0;
    assign tmds_clk_n = 1'b0;
    assign tmds_data_p = 3'b0;
    assign tmds_data_n = 3'b0;
endmodule

