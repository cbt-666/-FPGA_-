// 三模式图像UDP发送器。
// 模式0：每行两个包，每包8字节头+640个RGB565像素（1288字节）。
// 模式1/2：每行一个包，每包8字节头+1280个8位编码像素（1288字节）。
// 模式切换、FIFO溢出或异常行宽时停止采集；完成已开始的UDP包后排空
// 旧数据，再从后续完整帧恢复，避免两种协议或不同图像模式发生拼接。
`timescale 1ns / 1ps

module send_image #(
    parameter [47:0] DES_MAC = 48'hff_ff_ff_ff_ff_ff,
    parameter [31:0] DES_IP  = {8'd192,8'd168,8'd1,8'd102},
    parameter integer SEND_FRAME_DIV_MODE0  = 3,
    parameter integer SEND_FRAME_DIV_MODE12 = 2
)(
    input             eth_clk,
    input             pclk,
    input             rst_n,
    input             tx_req,
    input             tx_done,
    output reg [7:0]  tx_data,
    output reg        tx_start_en,
    output reg [15:0] tx_byte_num,
    output reg [47:0] des_mac,
    output reg [31:0] des_ip,
    input             hs,
    input             vs,
    input             de,
    input      [1:0]  image_mode,
    input      [23:0] image_rgb
);

    localparam integer LINE_PIXELS = 1280;
    localparam integer HALF_PIXELS = 640;
    localparam integer TOTAL_LINE = 720;
    localparam integer HDR_LEN = 8;
    localparam integer PKT_LEN = HDR_LEN + 1280;
    localparam integer START_LEVEL = 1900;
    localparam integer INTER_PACKET_GAP = 32;

    // image_rgb为标准RGB888顺序：R[23:16]、G[15:8]、B[7:0]。
    // RGB565按R5/G6/B5打包，避免参考工程旧公式造成红蓝通道互换。
    wire [15:0] rgb565 = {image_rgb[23:19], image_rgb[15:10], image_rgb[7:3]};
    wire [7:0] coded_pixel;
    image_pixel_encode u_image_pixel_encode(
        .rgb   (image_rgb),
        .pixel (coded_pixel)
    );

    // ---------------------------- 像素时钟域 ----------------------------
    reg vs_d, de_d;
    reg frame_pending, capture_frame;
    reg [15:0] frame_skip_cnt;
    reg [10:0] pixel_cnt;
    reg [9:0] line_cnt_wr;
    reg [7:0] packed_pixel_high;
    reg frame_start_toggle_wr, frame_done_toggle_wr;
    reg [1:0] frame_mode_wr;
    reg [1:0] image_mode_d;
    reg mode_epoch_wr, mode_epoch_target;
    reg flush_wait_wr;
    reg flush_done_meta_wr, flush_done_sync_wr;
    reg flush_done_toggle_rd;
    reg frame_busy_wr;
    reg frame_sent_toggle_rd;
    reg frame_sent_meta_wr, frame_sent_sync_wr, frame_sent_seen_wr;
    wire mode_change_p = (image_mode != image_mode_d);
    wire video_frame_start = de && !de_d && frame_pending;
    wire selected_frame_start = video_frame_start &&
                                (frame_skip_cnt == 16'd0) && !frame_busy_wr;
    wire [15:0] selected_frame_div = (image_mode == 2'd0) ?
                                     SEND_FRAME_DIV_MODE0 :
                                     SEND_FRAME_DIV_MODE12;
    wire capture_active = capture_frame || selected_frame_start;
    // 8位模式把相邻两个像素打包成一个16位FIFO字，以维持GMII逐字节吞吐率。
    wire processed_second_pixel = (image_mode != 2'd0) && pixel_cnt[0];
    wire fifo_write_slot = (image_mode == 2'd0) || processed_second_pixel;
    wire [15:0] fifo_wr_data = (image_mode == 2'd0) ? rgb565 :
                               {packed_pixel_high, coded_pixel};
    wire fifo_full;
    wire capture_fault = capture_frame &&
                         ((de && fifo_write_slot && fifo_full) ||
                          (!de && de_d && (pixel_cnt != 0)) ||
                          (de && de_d && (pixel_cnt == 0)) ||
                          (vs && !vs_d));
    wire fifo_wr_en = de && capture_active && fifo_write_slot &&
                      !flush_wait_wr && !mode_change_p &&
                      !capture_fault && !fifo_full;

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            vs_d <= 1'b0;
            de_d <= 1'b0;
            frame_pending <= 1'b0;
            capture_frame <= 1'b0;
            frame_skip_cnt <= 16'd0;
            pixel_cnt <= 11'd0;
            line_cnt_wr <= 10'd0;
            packed_pixel_high <= 8'd0;
            frame_start_toggle_wr <= 1'b0;
            frame_done_toggle_wr <= 1'b0;
            frame_mode_wr <= 2'd0;
            image_mode_d <= 2'd0;
            mode_epoch_wr <= 1'b0;
            mode_epoch_target <= 1'b0;
            flush_wait_wr <= 1'b0;
            flush_done_meta_wr <= 1'b0;
            flush_done_sync_wr <= 1'b0;
            frame_busy_wr <= 1'b0;
            frame_sent_meta_wr <= 1'b0;
            frame_sent_sync_wr <= 1'b0;
            frame_sent_seen_wr <= 1'b0;
        end else begin
            vs_d <= vs;
            de_d <= de;
            image_mode_d <= image_mode;
            flush_done_meta_wr <= flush_done_toggle_rd;
            flush_done_sync_wr <= flush_done_meta_wr;
            frame_sent_meta_wr <= frame_sent_toggle_rd;
            frame_sent_sync_wr <= frame_sent_meta_wr;

            if (frame_sent_sync_wr != frame_sent_seen_wr) begin
                frame_sent_seen_wr <= frame_sent_sync_wr;
                frame_busy_wr <= 1'b0;
            end

            if (mode_change_p || (capture_fault && !flush_wait_wr)) begin
                mode_epoch_wr <= ~mode_epoch_wr;
                mode_epoch_target <= ~mode_epoch_wr;
                flush_wait_wr <= 1'b1;
                capture_frame <= 1'b0;
                frame_pending <= 1'b0;
                frame_skip_cnt <= 16'd0;
                pixel_cnt <= 11'd0;
                line_cnt_wr <= 10'd0;
            end else if (flush_wait_wr &&
                         (flush_done_sync_wr == mode_epoch_target)) begin
                flush_wait_wr <= 1'b0;
                frame_busy_wr <= 1'b0;
            end

            if (!flush_wait_wr && !mode_change_p && !capture_fault) begin
                if (vs && !vs_d)
                    frame_pending <= 1'b1;

                if (video_frame_start) begin
                    frame_pending <= 1'b0;
                    pixel_cnt <= 11'd0;
                    line_cnt_wr <= 10'd0;
                    if ((frame_skip_cnt == 16'd0) && !frame_busy_wr) begin
                        capture_frame <= 1'b1;
                        frame_busy_wr <= 1'b1;
                        frame_mode_wr <= image_mode;
                        pixel_cnt <= 11'd1;
                        if (image_mode != 2'd0)
                            packed_pixel_high <= coded_pixel;
                        frame_start_toggle_wr <= ~frame_start_toggle_wr;
                        if (selected_frame_div <= 1)
                            frame_skip_cnt <= 16'd0;
                        else
                            frame_skip_cnt <= selected_frame_div - 1'b1;
                    end else begin
                        capture_frame <= 1'b0;
                        if (frame_skip_cnt != 0)
                            frame_skip_cnt <= frame_skip_cnt - 1'b1;
                    end
                end else if (capture_frame && de) begin
                    if ((image_mode != 2'd0) && !pixel_cnt[0])
                        packed_pixel_high <= coded_pixel;
                    if (pixel_cnt == LINE_PIXELS-1) begin
                        pixel_cnt <= 11'd0;
                        if (line_cnt_wr == TOTAL_LINE-1) begin
                            line_cnt_wr <= 10'd0;
                            capture_frame <= 1'b0;
                            frame_done_toggle_wr <= ~frame_done_toggle_wr;
                        end else begin
                            line_cnt_wr <= line_cnt_wr + 1'b1;
                        end
                    end else begin
                        pixel_cnt <= pixel_cnt + 1'b1;
                    end
                end
            end
        end
    end

    // ---------------------------- 异步 FIFO ----------------------------
    wire [15:0] fifo_rd_data;
    wire fifo_empty;
    wire [13:0] fifo_rd_count;
    reg fifo_rd_en;

    async_fifo_8192x16b u_image_fifo(
        .wr_clk(pclk), .wr_rst(~rst_n), .wr_en(fifo_wr_en),
        .wr_data(fifo_wr_data), .wr_full(fifo_full), .almost_full(),
        .wr_water_level(), .rd_clk(eth_clk), .rd_rst(~rst_n),
        .rd_en(fifo_rd_en), .rd_data(fifo_rd_data), .rd_empty(fifo_empty),
        .almost_empty(), .rd_water_level(fifo_rd_count)
    );

    // ---------------------------- 以太网时钟域 ----------------------------
    localparam [2:0] ST_IDLE = 3'd0;
    localparam [2:0] ST_PREFETCH = 3'd1;
    localparam [2:0] ST_WAIT_REQ = 3'd2;
    localparam [2:0] ST_SEND = 3'd3;
    localparam [2:0] ST_WAIT_DONE = 3'd4;
    localparam [2:0] ST_FLUSH = 3'd5;
    localparam [2:0] ST_GAP = 3'd6;

    reg [2:0] state;
    reg [15:0] payload_cnt, line_cnt;
    reg [31:0] frame_cnt;
    reg packet_half;
    reg [1:0] frame_mode_meta, frame_mode_sync, frame_mode_rd;
    reg [7:0] hdr [0:7];
    reg [15:0] pixel_word;
    reg pixel_low_next;
    reg first_packet;
    reg rd_pipe0, tx_req_d0;
    reg [7:0] gap_cnt;
    reg frame_start_meta, frame_start_sync, frame_start_seen;
    reg frame_done_meta, frame_done_sync, frame_done_seen;
    reg frame_in_progress_rd;
    reg mode_epoch_meta, mode_epoch_sync, mode_epoch_seen;
    reg abort_after_packet;
    reg [1:0] flush_empty_cnt;
    wire mode_epoch_event = (mode_epoch_sync != mode_epoch_seen);
    wire packet_active_state = (state == ST_WAIT_REQ) ||
                               (state == ST_SEND) ||
                               (state == ST_WAIT_DONE);

    always @(posedge eth_clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_start_meta <= 1'b0;
            frame_start_sync <= 1'b0;
            frame_done_meta <= 1'b0;
            frame_done_sync <= 1'b0;
            mode_epoch_meta <= 1'b0;
            mode_epoch_sync <= 1'b0;
            frame_mode_meta <= 2'd0;
            frame_mode_sync <= 2'd0;
        end else begin
            frame_start_meta <= frame_start_toggle_wr;
            frame_start_sync <= frame_start_meta;
            frame_done_meta <= frame_done_toggle_wr;
            frame_done_sync <= frame_done_meta;
            mode_epoch_meta <= mode_epoch_wr;
            mode_epoch_sync <= mode_epoch_meta;
            frame_mode_meta <= frame_mode_wr;
            frame_mode_sync <= frame_mode_meta;
        end
    end

    always @(*) begin
        if (frame_mode_rd == 2'd0) begin
            // RGB565：00 00 frame16 line16 00 half。
            hdr[0] = 8'd0;
            hdr[1] = 8'd0;
            hdr[2] = frame_cnt[15:8];
            hdr[3] = frame_cnt[7:0];
            hdr[4] = line_cnt[15:8];
            hdr[5] = line_cnt[7:0];
            hdr[6] = 8'd0;
            hdr[7] = {7'd0, packet_half};
        end else begin
            // YRG/二值：frame32 line16 height16，height固定为720。
            hdr[0] = frame_cnt[31:24];
            hdr[1] = frame_cnt[23:16];
            hdr[2] = frame_cnt[15:8];
            hdr[3] = frame_cnt[7:0];
            hdr[4] = line_cnt[15:8];
            hdr[5] = line_cnt[7:0];
            hdr[6] = TOTAL_LINE[15:8];
            hdr[7] = TOTAL_LINE[7:0];
        end
    end

    always @(posedge eth_clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            payload_cnt <= 16'd0;
            line_cnt <= 16'd0;
            frame_cnt <= 32'd0;
            packet_half <= 1'b0;
            frame_mode_rd <= 2'd0;
            pixel_word <= 16'd0;
            pixel_low_next <= 1'b0;
            first_packet <= 1'b1;
            fifo_rd_en <= 1'b0;
            flush_done_toggle_rd <= 1'b0;
            rd_pipe0 <= 1'b0;
            tx_req_d0 <= 1'b0;
            gap_cnt <= 8'd0;
            frame_start_seen <= 1'b0;
            frame_done_seen <= 1'b0;
            frame_in_progress_rd <= 1'b0;
            frame_sent_toggle_rd <= 1'b0;
            mode_epoch_seen <= 1'b0;
            abort_after_packet <= 1'b0;
            flush_empty_cnt <= 2'd0;
            tx_data <= 8'd0;
            tx_start_en <= 1'b0;
            tx_byte_num <= PKT_LEN;
            des_mac <= DES_MAC;
            des_ip <= DES_IP;
        end else begin
            tx_start_en <= 1'b0;
            fifo_rd_en <= 1'b0;
            rd_pipe0 <= fifo_rd_en;
            tx_req_d0 <= tx_req;

            if (frame_start_sync != frame_start_seen) begin
                frame_start_seen <= frame_start_sync;
                frame_mode_rd <= frame_mode_sync;
                frame_in_progress_rd <= 1'b1;
                line_cnt <= 16'd0;
                packet_half <= 1'b0;
                first_packet <= 1'b1;
            end
            if (frame_done_sync != frame_done_seen)
                frame_done_seen <= frame_done_sync;

            if (mode_epoch_event) begin
                mode_epoch_seen <= mode_epoch_sync;
                if (packet_active_state && !(state == ST_WAIT_DONE && tx_done))
                    abort_after_packet <= 1'b1;
                else begin
                    state <= ST_FLUSH;
                    flush_empty_cnt <= 2'd0;
                end
            end

            if (!mode_epoch_event || packet_active_state) begin
            case (state)
                ST_IDLE: begin
                    tx_req_d0 <= 1'b0;
                    if (frame_in_progress_rd && !fifo_empty &&
                        ((!first_packet && (fifo_rd_count >= HALF_PIXELS)) ||
                         (first_packet && (fifo_rd_count >= START_LEVEL)))) begin
                        fifo_rd_en <= 1'b1;
                        state <= ST_PREFETCH;
                    end
                end

                ST_PREFETCH: begin
                    if (rd_pipe0) begin
                        pixel_word <= fifo_rd_data;
                        pixel_low_next <= 1'b0;
                        tx_byte_num <= PKT_LEN;
                        tx_start_en <= 1'b1;
                        tx_data <= hdr[0];
                        payload_cnt <= 16'd0;
                        state <= ST_WAIT_REQ;
                    end
                end

                ST_WAIT_REQ: begin
                    if (tx_req_d0) begin
                        tx_data <= hdr[1];
                        payload_cnt <= 16'd1;
                        state <= ST_SEND;
                    end
                end

                ST_SEND: begin
                    if (tx_req) begin
                        if (payload_cnt == PKT_LEN-2) begin
                            tx_data <= pixel_word[7:0];
                            payload_cnt <= PKT_LEN-1;
                            state <= ST_WAIT_DONE;
                        end else begin
                            payload_cnt <= payload_cnt + 1'b1;
                            if ((payload_cnt + 1'b1) < HDR_LEN) begin
                                tx_data <= hdr[payload_cnt[2:0] + 1'b1];
                            end else if (payload_cnt == HDR_LEN-1) begin
                                tx_data <= pixel_word[15:8];
                                pixel_low_next <= 1'b1;
                                if (!fifo_empty)
                                    fifo_rd_en <= 1'b1;
                            end else if (pixel_low_next) begin
                                tx_data <= pixel_word[7:0];
                                pixel_low_next <= 1'b0;
                            end else if (rd_pipe0) begin
                                pixel_word <= fifo_rd_data;
                                tx_data <= fifo_rd_data[15:8];
                                pixel_low_next <= 1'b1;
                                if ((payload_cnt < PKT_LEN-3) && !fifo_empty)
                                    fifo_rd_en <= 1'b1;
                            end
                        end
                    end
                end

                ST_WAIT_DONE: begin
                    if (tx_done) begin
                        if (abort_after_packet || mode_epoch_event) begin
                            abort_after_packet <= 1'b0;
                            state <= ST_FLUSH;
                            flush_empty_cnt <= 2'd0;
                        end else begin
                            first_packet <= 1'b0;
                            if ((frame_mode_rd == 2'd0) && !packet_half) begin
                                packet_half <= 1'b1;
                            end else begin
                                packet_half <= 1'b0;
                                if (line_cnt == TOTAL_LINE-1) begin
                                    line_cnt <= 16'd0;
                                    frame_cnt <= frame_cnt + 1'b1;
                                    frame_in_progress_rd <= 1'b0;
                                    frame_sent_toggle_rd <= ~frame_sent_toggle_rd;
                                end else begin
                                    line_cnt <= line_cnt + 1'b1;
                                end
                            end
                            gap_cnt <= INTER_PACKET_GAP;
                            state <= ST_GAP;
                        end
                    end
                end

                ST_GAP: begin
                    if (gap_cnt == 0)
                        state <= ST_IDLE;
                    else
                        gap_cnt <= gap_cnt - 1'b1;
                end

                ST_FLUSH: begin
                    if (!fifo_empty) begin
                        fifo_rd_en <= 1'b1;
                        flush_empty_cnt <= 2'd0;
                    end else if (flush_empty_cnt != 2'd3) begin
                        flush_empty_cnt <= flush_empty_cnt + 1'b1;
                    end else begin
                        flush_done_toggle_rd <= mode_epoch_seen;
                        state <= ST_IDLE;
                        flush_empty_cnt <= 2'd0;
                        line_cnt <= 16'd0;
                        packet_half <= 1'b0;
                        frame_in_progress_rd <= 1'b0;
                        // 取消帧也推进帧号，防止接收端与下一完整帧拼接。
                        frame_cnt <= frame_cnt + 1'b1;
                        abort_after_packet <= 1'b0;
                        first_packet <= 1'b1;
                        pixel_low_next <= 1'b0;
                        rd_pipe0 <= 1'b0;
                        tx_req_d0 <= 1'b0;
                    end
                end

                default: state <= ST_IDLE;
            endcase
            end
        end
    end

    wire unused_hs = hs;

endmodule
