`timescale 1ns/1ps

module tb_image_overlay;
    localparam [23:0] BASE_RGB = 24'h204060;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg de_in = 1'b0;
    reg hs_in = 1'b0;
    reg vs_in = 1'b0;
    wire de;
    wire [23:0] data;
    integer changed_pixels = 0;
    integer y;
    integer x;

    always #5 clk = ~clk;

    image_overlay u_dut (
        .clk(clk), .rst_n(rst_n),
        .de_in(de_in), .hs_in(hs_in), .vs_in(vs_in), .data_in(BASE_RGB),
        .overlay_enable(1'b1),
        .left_enable(1'b1), .right_enable(1'b0),
        .up_enable(1'b0), .warning_enable(1'b0),
        .left_color(2'b00), .right_color(2'b00), .up_color(2'b00),
        .left_blink(1'b1), .right_blink(1'b0), .up_blink(1'b0),
        .warning_blink(1'b0), .blink_visible(1'b1),
        .de(de), .hs(), .vs(), .data(data)
    );

    always @(posedge clk) begin
        if (de && (data != BASE_RGB))
            changed_pixels <= changed_pixels + 1;
    end

    initial begin
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk); vs_in = 1'b1;
        @(negedge clk); vs_in = 1'b0;

        // 扫描完整720p有效区，确认机械键临时红色左箭头确实改变输出像素。
        for (y = 0; y < 720; y = y + 1) begin
            de_in = 1'b1;
            for (x = 0; x < 1280; x = x + 1)
                @(negedge clk);
            de_in = 1'b0;
            repeat (8) @(negedge clk);
        end
        repeat (8) @(negedge clk);

        if (changed_pixels == 0)
            $fatal(1, "Enabled left arrow did not alter any output pixel");
        $display("PASS: image overlay changed %0d pixels", changed_pixels);
        $finish;
    end
endmodule
