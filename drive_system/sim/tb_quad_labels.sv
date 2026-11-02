`timescale 1ns/1ps
// 检查四格标签、四路放大名称，以及以太网未收首帧时的黑底标签。
module tb_quad_labels;
    reg clk = 0;
    always #5 clk = ~clk;
    reg [10:0] x = 0, y = 0;
    reg zoom = 0, eth_ready = 1;
    reg [1:0] zoom_source = 0;
    wire [15:0] pixel;
    localparam [15:0] BACKGROUND = 16'h4208;
    integer px, py, fd, region, source, ready, label_index;
    integer ink[0:3];
    reg label_mask[0:3][0:4095];
    reg [15:0] expected_pixel;
    hdmi_disply dut (
        .hdmi_clk(clk), .sys_rst_n(1'b1),
        .pixel_xpos(x), .pixel_ypos(y), .rd_data(BACKGROUND),
        .rd_lower_mode(y > 360), .eth_frame_valid(eth_ready),
        .rd_h_pixel(11'd1280), .zoom_fullscreen(zoom),
        .zoom_source(zoom_source), .pixel_data(pixel)
    );
    initial begin
        for (region = 0; region < 4; region = region + 1) ink[region] = 0;
        for (region = 0; region < 4; region = region + 1)
            for (label_index = 0; label_index < 4096; label_index = label_index + 1)
                label_mask[region][label_index] = 0;
        repeat (3) @(negedge clk);
        fd = $fopen("tmp/quad_labels_preview.ppm", "w");
        if (!fd) $fatal(1, "Cannot create pixel preview");
        $fwrite(fd, "P3\n1280 720\n255\n");
        for (py = 1; py <= 720; py = py + 1) begin
            for (px = 1; px <= 1280; px = px + 1) begin
                x = px; y = py;
                @(posedge clk); #1;
                if ($isunknown(pixel)) $fatal(1, "Unknown pixel at %0d,%0d", px, py);
                if (pixel == 16'h001f) begin
                    if (!(((px >= 256 && px < 384) || (px >= 896 && px < 1024)) &&
                          ((py <= 32) || (py > 360 && py <= 392))))
                        $fatal(1, "Label outside its window at %0d,%0d", px, py);
                    region = (py > 360 ? 2 : 0) + (px > 640 ? 1 : 0);
                    ink[region] = ink[region] + 1;
                    label_index = ((py - 1) % 360) * 128 + px - (px > 640 ? 896 : 256);
                    label_mask[region][label_index] = 1;
                    $fwrite(fd, "0 0 255\n");
                end else begin
                    if (pixel !== BACKGROUND) $fatal(1, "Background changed at %0d,%0d", px, py);
                    $fwrite(fd, "64 64 64\n");
                end
                @(negedge clk);
            end
        end
        $fclose(fd);
        for (region = 0; region < 4; region = region + 1) begin
            if (ink[region] < 200) $fatal(1, "Missing label in quadrant %0d", region);
        end
        // 四路放大逐像素核对：顶部居中标签应与四宫格对应名称完全一致。
        // eth_ready同时取0/1，摄像头画面不受它影响，以太网无首帧时只显示黑底名称。
        zoom = 1;
        for (source = 0; source < 4; source = source + 1) begin
            zoom_source = source;
            for (ready = 0; ready < 2; ready = ready + 1) begin
                eth_ready = ready;
                for (py = 1; py <= 720; py = py + 1) begin
                    for (px = 1; px <= 1280; px = px + 1) begin
                        x = px; y = py;
                        @(posedge clk); #1;
                        expected_pixel = source == 3 && !ready ? 16'h0000 : BACKGROUND;
                        if (px >= 576 && px < 704 && py <= 32)
                            if (label_mask[source][(py-1)*128+px-576]) expected_pixel = 16'h001f;
                        if (pixel !== expected_pixel)
                            $fatal(1, "Zoom source=%0d ready=%0d mismatch at %0d,%0d", source, ready, px, py);
                        @(negedge clk);
                    end
                end
            end
        end
        // 四宫格的以太网无首帧：标签照常显示，区域内其他像素保持黑色。
        zoom = 0; eth_ready = 0;
        for (py = 361; py <= 720; py = py + 1) begin
            for (px = 641; px <= 1280; px = px + 1) begin
                x = px; y = py;
                @(posedge clk); #1;
                expected_pixel = 16'h0000;
                if (px >= 896 && px < 1024 && py <= 392)
                    if (label_mask[3][(py-361)*128+px-896]) expected_pixel = 16'h001f;
                if (pixel !== expected_pixel) $fatal(1, "Quad Ethernet not-ready mismatch at %0d,%0d", px, py);
                @(negedge clk);
            end
        end
        $display("PASS: quad labels, all four zoom labels, Ethernet labels before/after first frame");
        $finish;
    end
endmodule
