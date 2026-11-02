// DDR3图像读写调度器。
// 通道1/2为摄像头1/2，通道3为BACK_LEFT共享地址空间：
// 根据back_left_source_req择一接收摄像头3或HDMI；通道4为以太网RGB565流。
// 鸟瞰映射固定为：摄像头1左前、摄像头2右前、BACK_LEFT左后、以太网右后。
module ddr3_rw_ctrl(
    input rst_n, input clk, input ddr3_init_done,
    input [26:0] addr_rd_min,
    input [9:0] rd_burst_len, input [26:0] addr_wd_min,
    input [9:0] wd_burst_len, input ddr3_pingpang_en,
    input rd_load, input wd_load_1, input wd_load_2, input wd_load_3,
    input wd_load_5, input back_left_source_req,
    input [10:0] rfifo_wcount_1, input [10:0] wfifo_rcount_1,
    output reg wfifo_rd_en_1, input [255:0] wfifo_rd_data_1,
    output reg rfifo_wd_en_1, output reg [255:0] rfifo_wd_data_1,
    input [10:0] rfifo_wcount_2, input [10:0] wfifo_rcount_2,
    output reg wfifo_rd_en_2, input [255:0] wfifo_rd_data_2,
    output reg rfifo_wd_en_2, output reg [255:0] rfifo_wd_data_2,
    input [10:0] rfifo_wcount_3, input [10:0] wfifo_rcount_3,
    output reg wfifo_rd_en_3, input [255:0] wfifo_rd_data_3,
    output reg rfifo_wd_en_3, output reg [255:0] rfifo_wd_data_3,
    input [10:0] wfifo_rcount_5,
    input [255:0] wfifo_rd_data_5,
    output reg wfifo_rd_en_5,
    input [10:0] rfifo_wcount_4, input [10:0] wfifo_rcount_4,
    output reg wfifo_rd_en_4, input [255:0] wfifo_rd_data_4,
    output reg rfifo_wd_en_4, output reg [255:0] rfifo_wd_data_4,
    input wd_fifo_re, input wd_finish, output reg wd_req, output reg [23:0] wd_addr,
    output reg [9:0] wd_len, input rd_fifo_we, input rd_finish, output reg rd_req,
    output reg [23:0] rd_addr, output reg [9:0] rd_len,
    output reg [255:0] wd_ddr3_data, input [255:0] rd_ddr3_data,
    output reg eth_frame_ready,

    // Background BEV renderer random block access.
    input bev_source_req, input [23:0] bev_source_addr,
    output reg bev_source_valid, output reg [255:0] bev_source_data,
    input bev_write_req, input [23:0] bev_write_addr,
    input [255:0] bev_write_data, output reg bev_write_done,
    output [1:0] bev_source_page0, output [1:0] bev_source_page1,
    output [1:0] bev_source_page2, output [1:0] bev_source_page3,
    output [3:0] bev_source_ready,
    // Renderer page lease. While asserted, do not rotate a source writer
    // onto the page captured by the renderer.
    input bev_source_lock,

    // Sequential readout of the completed 720x720 BEV frame.
    input bev_display_active, input bev_frame_ready, input [1:0] bev_frame_page,
    input [10:0] bev_fifo_wcount,
    output reg bev_fifo_wr_en, output reg [255:0] bev_fifo_wr_data,
    output [1:0] bev_display_page_in_use
);
localparam IDLE=4'd0, DONE=4'd1, WRITE_1=4'd2, READ_1=4'd3,
           WRITE_2=4'd4, READ_2=4'd5, WRITE_3=4'd6, READ_3=4'd7,
           WRITE_4=4'd8, READ_4=4'd9, BEV_SOURCE_READ=4'd10,
           BEV_FRAME_WRITE=4'd11, BEV_DISPLAY_READ=4'd12;
// 本工程AXI包装层把wd_burst_len先左移3位，再按[10:3]产生256-bit FIFO读数，
// 因而调度器中的一个地址步进对应一个256-bit FIFO字（16个RGB565像素）。
// 640x360一帧共230400像素，即14400个调度单位。
localparam [19:0] ETH_FRAME_ADDR_UNITS = 20'd14400;
localparam [19:0] BEV_BASE_WORD = 20'h20000;
localparam [15:0] BEV_FRAME_WORDS = 16'd32400; // 720*720/16
// 80 words = 1280 RGB565 pixels.  32400 is exactly divisible by 80,
// so every display transaction remains inside the completed BEV frame.
localparam [9:0] BEV_DISPLAY_BURST = 10'd80;
reg [3:0] state;
// 页内偏移最多使用20位；通道号和页号在输出地址处统一拼接。
reg [19:0] wd_addr_1,wd_addr_2,wd_addr_3,wd_addr_4;
reg [19:0] rd_addr_1,rd_addr_2,rd_addr_3,rd_addr_4;
reg [1:0] wpage1,wpage2,wpage3,wpage4,rpage1,rpage2,rpage3,rpage4;
reg [2:0] source_frames1,source_frames2,source_frames3;
reg source_ready1,source_ready2,source_ready3;
(* ASYNC_REG = "TRUE" *) reg back_left_source_meta, back_left_source_sync;
reg back_left_source_active;
reg back_left_source_wait;
reg [1:0] bev_read_page;
reg [15:0] bev_read_word;
reg bev_source_lock_d;
reg bev_source_lock_active;
reg [1:0] bev_lock_page1, bev_lock_page2, bev_lock_page3, bev_lock_page4;
(* ASYNC_REG = "TRUE" *) reg bev_active_meta, bev_active_sync;
reg rd_load_d0,rd_load_d1,rd_load_d2,wd1_d0,wd1_d1,wd1_d2,wd2_d0,wd2_d1,wd2_d2,
    wd3_d0,wd3_d1,wd3_d2,wd5_d0,wd5_d1,wd5_d2;
// 各输入帧标志均为源时钟域的稳定电平，在 UI 时钟域同步后检测上升沿。
// 上升沿表示新帧到达，推进对应写页并复位该路写地址。
wire rd_rst = rd_load_d1 & ~rd_load_d2;
wire wd_rst_1 = wd1_d1 & ~wd1_d2;
wire wd_rst_2 = wd2_d1 & ~wd2_d2;
wire wd_rst_3 = wd3_d1 & ~wd3_d2;
wire wd_rst_5 = wd5_d1 & ~wd5_d2;
// 帧沿可能落在DDR突发事务期间，不能只在DONE状态采样一个时钟宽的脉冲。
// 将事件保持到事务安全边界，再统一复位地址，避免跨帧地址继续累加。
reg rd_addr_reset_pending;
reg wd_addr_reset_pending_1;
reg wd_addr_reset_pending_2;
reg wd_addr_reset_pending_3;
reg wd_addr_reset_pending_5;
wire rd_reset_request = rd_addr_reset_pending | rd_rst;
wire wd_reset_request_1 = wd_addr_reset_pending_1 | wd_rst_1;
wire wd_reset_request_2 = wd_addr_reset_pending_2 | wd_rst_2;
wire wd_reset_request_3 = wd_addr_reset_pending_3 | wd_rst_3;
wire wd_reset_request_5 = wd_addr_reset_pending_5 | wd_rst_5;
wire wd_reset_request_3_sel = back_left_source_active ? wd_reset_request_5 : wd_reset_request_3;
wire wd_rst_3_sel = back_left_source_active ? wd_rst_5 : wd_rst_3;
wire eth_frame_done = (state == WRITE_4) && wd_finish &&
                      (wd_addr_4 + wd_burst_len >= ETH_FRAME_ADDR_UNITS);
wire any_reset_request = rd_reset_request | wd_reset_request_1 |
                         wd_reset_request_2 | wd_reset_request_3_sel;
wire [10:0] wfifo_rcount_3_sel = back_left_source_active ? wfifo_rcount_5 : wfifo_rcount_3;
wire [255:0] wfifo_rd_data_3_sel = back_left_source_active ? wfifo_rd_data_5 : wfifo_rd_data_3;
reg [10:0] rd_rst_cnt;
reg rd_reset_hold;

assign bev_source_page0 = rpage1;
assign bev_source_page1 = rpage2;
assign bev_source_page2 = rpage3;
assign bev_source_page3 = rpage4;
assign bev_source_ready = {eth_frame_ready,source_ready3,
                           source_ready2,source_ready1};
assign bev_display_page_in_use = bev_read_page;

always @(*) begin
    wfifo_rd_en_1=0; wfifo_rd_en_2=0; wfifo_rd_en_3=0; wfifo_rd_en_4=0;
    wfifo_rd_en_5=0;
    rfifo_wd_en_1=0; rfifo_wd_en_2=0; rfifo_wd_en_3=0; rfifo_wd_en_4=0;
    rfifo_wd_data_1=rd_ddr3_data; rfifo_wd_data_2=rd_ddr3_data;
    rfifo_wd_data_3=rd_ddr3_data; rfifo_wd_data_4=rd_ddr3_data;
    bev_source_valid=0; bev_source_data=rd_ddr3_data;
    bev_write_done=0;
    bev_fifo_wr_en=0; bev_fifo_wr_data=rd_ddr3_data;
    wd_ddr3_data=0;
    if (state==WRITE_1) begin wfifo_rd_en_1=wd_fifo_re; wd_ddr3_data=wfifo_rd_data_1; end
    if (state==WRITE_2) begin wfifo_rd_en_2=wd_fifo_re; wd_ddr3_data=wfifo_rd_data_2; end
    if (state==WRITE_3) begin
        if (back_left_source_active) begin
            wfifo_rd_en_5=wd_fifo_re;
            wd_ddr3_data=wfifo_rd_data_5;
        end else begin
            wfifo_rd_en_3=wd_fifo_re;
            wd_ddr3_data=wfifo_rd_data_3;
        end
    end
    if (state==WRITE_4) begin wfifo_rd_en_4=wd_fifo_re; wd_ddr3_data=wfifo_rd_data_4; end
    if (state==BEV_FRAME_WRITE) begin
        wd_ddr3_data=bev_write_data;
        bev_write_done=wd_finish;
    end
    if (state==READ_1) rfifo_wd_en_1=rd_fifo_we;
    if (state==READ_2) rfifo_wd_en_2=rd_fifo_we;
    if (state==READ_3) rfifo_wd_en_3=rd_fifo_we;
    if (state==READ_4) rfifo_wd_en_4=rd_fifo_we;
    if (state==BEV_SOURCE_READ) begin
        bev_source_valid=rd_fifo_we;
        bev_source_data=rd_ddr3_data;
    end
    if (state==BEV_DISPLAY_READ) begin
        bev_fifo_wr_en=rd_fifo_we;
        bev_fifo_wr_data=rd_ddr3_data;
    end
end

always @(*) begin
    wd_addr=24'd0; rd_addr=24'd0;
    if (ddr3_pingpang_en) begin
        // AXI 地址在上层左移3位（每个地址单位为8字节），因此这里只能使用
        // 24位字地址。按“通道2位 + 乒乓页2位 + 页内地址20位”划分：
        // 通道1=00、通道2=01、HDMI=10、以太网=11，四个区域互不重叠。
        case (state)
            WRITE_1: wd_addr={2'b00,wpage1,wd_addr_1[19:0]};
            READ_1 : rd_addr={2'b00,rpage1,rd_addr_1[19:0]};
            WRITE_2: wd_addr={2'b01,wpage2,wd_addr_2[19:0]};
            READ_2 : rd_addr={2'b01,rpage2,rd_addr_2[19:0]};
            WRITE_3: wd_addr={2'b10,wpage3,wd_addr_3[19:0]};
            READ_3 : rd_addr={2'b10,rpage3,rd_addr_3[19:0]};
            WRITE_4: wd_addr={2'b11,wpage4,wd_addr_4[19:0]};
            READ_4 : rd_addr={2'b11,rpage4,rd_addr_4[19:0]};
            BEV_SOURCE_READ: rd_addr=bev_source_addr;
            BEV_FRAME_WRITE: wd_addr=bev_write_addr;
            BEV_DISPLAY_READ: rd_addr={2'b00,bev_read_page,
                                       BEV_BASE_WORD + {4'd0,bev_read_word}};
            default: ;
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        rd_load_d0<=0; rd_load_d1<=0; rd_load_d2<=0;
        wd1_d0<=0; wd1_d1<=0; wd1_d2<=0; wd2_d0<=0; wd2_d1<=0; wd2_d2<=0;
        wd3_d0<=0; wd3_d1<=0; wd3_d2<=0;
        wd5_d0<=0; wd5_d1<=0; wd5_d2<=0;
    end else begin
        rd_load_d0<=rd_load; rd_load_d1<=rd_load_d0; rd_load_d2<=rd_load_d1;
        wd1_d0<=wd_load_1; wd1_d1<=wd1_d0; wd1_d2<=wd1_d1;
        wd2_d0<=wd_load_2; wd2_d1<=wd2_d0; wd2_d2<=wd2_d1;
        wd3_d0<=wd_load_3; wd3_d1<=wd3_d0; wd3_d2<=wd3_d1;
        wd5_d0<=wd_load_5; wd5_d1<=wd5_d0; wd5_d2<=wd5_d1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        bev_active_meta<=1'b0;
        bev_active_sync<=1'b0;
    end else begin
        bev_active_meta<=bev_display_active;
        bev_active_sync<=bev_active_meta;
    end
end

// 触摸控制在50MHz域产生BACK_LEFT选择；在DDR UI时钟域双触发同步，
// 实际选择只在事务安全边界更新，避免同一DDR突发中途切换输入FIFO。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        back_left_source_meta   <= 1'b0;
        back_left_source_sync   <= 1'b0;
        back_left_source_active <= 1'b0;
        back_left_source_wait   <= 1'b0;
    end else begin
        back_left_source_meta <= back_left_source_req;
        back_left_source_sync <= back_left_source_meta;
        if (state == DONE && back_left_source_active != back_left_source_sync) begin
            back_left_source_active <= back_left_source_sync;
            back_left_source_wait   <= 1'b1;
        end else if (wd_rst_3_sel) begin
            // 新输入的帧沿到达后，允许该路FIFO参与DDR写入。
            back_left_source_wait   <= 1'b0;
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        rpage1<=0; rpage2<=0; rpage3<=0; rpage4<=0;
        wpage1<=0; wpage2<=0; wpage3<=0; wpage4<=0;
        eth_frame_ready<=1'b0;
        source_frames1<=0; source_frames2<=0; source_frames3<=0;
        source_ready1<=0; source_ready2<=0; source_ready3<=0;
        rd_reset_hold<=0; rd_rst_cnt<=0;
        bev_read_page<=0;
        bev_source_lock_d<=1'b0;
        bev_source_lock_active<=1'b0;
        bev_lock_page1<=0; bev_lock_page2<=0;
        bev_lock_page3<=0; bev_lock_page4<=0;
    end else begin
        if (wd_rst_1) begin
            if (!(bev_source_lock_active && ((wpage1 + 2'd1) == bev_lock_page1)))
                wpage1<=wpage1+1'b1;
            if(source_frames1<3'd4) source_frames1<=source_frames1+1'b1;
            if(source_frames1>=3'd3) source_ready1<=1'b1;
        end
        if (wd_rst_2) begin
            if (!(bev_source_lock_active && ((wpage2 + 2'd1) == bev_lock_page2)))
                wpage2<=wpage2+1'b1;
            if(source_frames2<3'd4) source_frames2<=source_frames2+1'b1;
            if(source_frames2>=3'd3) source_ready2<=1'b1;
        end
        if (back_left_source_active != back_left_source_sync) begin
            // 切换输入时先丢弃旧源的帧计数，并切到下一写页；
            // 新源完成完整帧后才重新声明BACK_LEFT有效。
            wpage3<=wpage3+1'b1;
            source_frames3<=0;
            source_ready3<=1'b0;
        end else if (wd_rst_3_sel) begin
            if (!(bev_source_lock_active && ((wpage3 + 2'd1) == bev_lock_page3)))
                wpage3<=wpage3+1'b1;
            if(source_frames3<3'd4) source_frames3<=source_frames3+1'b1;
            if(source_frames3>=3'd3) source_ready3<=1'b1;
        end
        if (eth_frame_done) begin
            if (!(bev_source_lock_active && ((wpage4 + 2'd1) == bev_lock_page4)))
                wpage4<=wpage4+1'b1;
            eth_frame_ready<=1'b1;
        end
        // Capture the same page set the renderer latched at its start. When
        // the next page would collide, the writer reuses its current safe
        // page after the frame boundary instead of overwriting the lease.
        bev_source_lock_d<=bev_source_lock;
        if (bev_source_lock && !bev_source_lock_d) begin
            bev_source_lock_active<=1'b1;
            bev_lock_page1<=rpage1; bev_lock_page2<=rpage2;
            bev_lock_page3<=rpage3; bev_lock_page4<=rpage4;
        end else if (!bev_source_lock) begin
            bev_source_lock_active<=1'b0;
        end
        if (rd_rst) begin
            rpage1<=wpage1-2'd2; rpage2<=wpage2-2'd2; rpage3<=wpage3-2'd2;
            // 以太网写页在整帧写完后推进，上一页就是最新完整帧。
            rpage4<=eth_frame_done ? wpage4 : (wpage4-2'd1);
            rd_reset_hold<=1'b1; rd_rst_cnt<=0;
            bev_read_page<=bev_frame_page;
        end else if (rd_reset_hold && rd_rst_cnt<11'd1000)
            rd_rst_cnt<=rd_rst_cnt+1'b1;
        // 旧帧中尚未结束的读事务不能提前解除等待；必须先在DONE状态
        // 消化地址复位请求，再由新帧的首次读事务解除保持。
        else if (rd_reset_hold && !rd_addr_reset_pending &&
                 (state==READ_1 || state==READ_2 || state==READ_3 ||
                  state==READ_4 || state==BEV_DISPLAY_READ))
            rd_reset_hold<=1'b0;
    end
end

// 跨事务保持各路帧地址复位请求。在DONE状态完成处理后才清除。
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        rd_addr_reset_pending   <= 1'b0;
        wd_addr_reset_pending_1 <= 1'b0;
        wd_addr_reset_pending_2 <= 1'b0;
        wd_addr_reset_pending_3 <= 1'b0;
        wd_addr_reset_pending_5 <= 1'b0;
    end else begin
        if (rd_rst)   rd_addr_reset_pending   <= 1'b1;
        if (wd_rst_1) wd_addr_reset_pending_1 <= 1'b1;
        if (wd_rst_2) wd_addr_reset_pending_2 <= 1'b1;
        if (wd_rst_3) wd_addr_reset_pending_3 <= 1'b1;
        if (wd_rst_5) wd_addr_reset_pending_5 <= 1'b1;

        // 输入源切换后丢弃切换前已经到达的帧沿，等待新输入的下一帧，
        // 防止把半帧数据当作完整BACK_LEFT图像写入DDR。
        if (state == DONE && back_left_source_active != back_left_source_sync) begin
            wd_addr_reset_pending_3 <= 1'b0;
            wd_addr_reset_pending_5 <= 1'b0;
        end

        // 同一拍新到达的帧沿也可直接在DONE状态完成处理。
        if (state==DONE && rd_reset_request)
            rd_addr_reset_pending <= 1'b0;
        if (state==DONE && wd_reset_request_1)
            wd_addr_reset_pending_1 <= 1'b0;
        if (state==DONE && wd_reset_request_2)
            wd_addr_reset_pending_2 <= 1'b0;
        if (state==DONE && wd_reset_request_3)
            wd_addr_reset_pending_3 <= 1'b0;
        if (state==DONE && wd_reset_request_5)
            wd_addr_reset_pending_5 <= 1'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state<=IDLE; wd_addr_1<=addr_wd_min[19:0]; wd_addr_2<=addr_wd_min[19:0];
        wd_addr_3<=addr_wd_min[19:0]; wd_addr_4<=addr_wd_min[19:0];
        rd_addr_1<=addr_rd_min[19:0]; rd_addr_2<=addr_rd_min[19:0];
        rd_addr_3<=addr_rd_min[19:0]; rd_addr_4<=addr_rd_min[19:0];
        bev_read_word<=0;
        wd_req<=0; rd_req<=0; wd_len<=0; rd_len<=0;
    end else begin
        wd_req<=0; rd_req<=0;
        case(state)
            IDLE: if(ddr3_init_done) state<=DONE;
            DONE: begin
                // 帧边界处理优先于新的DDR事务，确保下一事务从新页首地址开始。
                if (back_left_source_active != back_left_source_sync) begin
                    state<=DONE;
                    wd_addr_3<=addr_wd_min[19:0];
                end else if(any_reset_request) begin
                    state<=DONE;
                    if(rd_reset_request) begin
                        rd_addr_1<=addr_rd_min[19:0];
                        rd_addr_2<=addr_rd_min[19:0];
                        rd_addr_3<=addr_rd_min[19:0];
                        rd_addr_4<=addr_rd_min[19:0];
                        bev_read_word<=0;
                    end
                    if(wd_reset_request_1) wd_addr_1<=addr_wd_min[19:0];
                    if(wd_reset_request_2) wd_addr_2<=addr_wd_min[19:0];
                    if(wd_reset_request_3_sel) wd_addr_3<=addr_wd_min[19:0];
                end else if(wfifo_rcount_1 >= wd_burst_len-1) state<=WRITE_1;
                else if(wfifo_rcount_2 >= wd_burst_len-1) state<=WRITE_2;
                else if(!back_left_source_wait &&
                        (wfifo_rcount_3_sel >= wd_burst_len-1)) state<=WRITE_3;
                // 与已验证的前三路保持相同水位语义，避免未攒满一次事务就读空FIFO。
                else if(wfifo_rcount_4 >= wd_burst_len-1'b1) state<=WRITE_4;
                // Frame reset also clears the asynchronous read FIFO.  Do not
                // issue a DDR read until its reset has safely propagated.
                else if(rd_reset_hold) begin
                    if(rd_rst_cnt>=11'd1000) begin
                        if(bev_active_sync && bev_frame_ready &&
                           bev_fifo_wcount < 11'd640 &&
                           bev_read_word < BEV_FRAME_WORDS)
                            state <= BEV_DISPLAY_READ;
                        else if(!bev_active_sync)
                            state <= READ_1;
                    end
                end
                // Keep the visible BEV FIFO ahead of the raster.  Source
                // writes remain higher priority so no input frame is lost.
                else if(bev_active_sync && bev_frame_ready &&
                        bev_fifo_wcount < 11'd640 &&
                        bev_read_word < BEV_FRAME_WORDS) state<=BEV_DISPLAY_READ;
                else if(!bev_active_sync && rfifo_wcount_1 < rd_burst_len-1) state<=READ_1;
                else if(!bev_active_sync && rfifo_wcount_2 < rd_burst_len-1) state<=READ_2;
                else if(!bev_active_sync && rfifo_wcount_3 < rd_burst_len-1) state<=READ_3;
                else if(!bev_active_sync && rfifo_wcount_4 < rd_burst_len-1) state<=READ_4;
                // 后台渲染只使用当前显示链路不需要的空闲带宽，避免常规
                // 四宫格/单路模式被逐像素随机读抢占。鸟瞰模式下普通读已
                // 关闭，因此渲染器仍可持续更新下一页完整帧。
                else if(bev_write_req) state<=BEV_FRAME_WRITE;
                else if(bev_source_req) state<=BEV_SOURCE_READ;
            end
            WRITE_1: begin
                if(wd_finish) begin state<=DONE; wd_addr_1<=wd_addr_1+wd_burst_len; end
                else if(wfifo_rcount_1 >= wd_burst_len-1) begin wd_len<=wd_burst_len; wd_req<=1'b1; end
            end
            WRITE_2: begin
                if(wd_finish) begin state<=DONE; wd_addr_2<=wd_addr_2+wd_burst_len; end
                else if(wfifo_rcount_2 >= wd_burst_len-1) begin wd_len<=wd_burst_len; wd_req<=1'b1; end
            end
            WRITE_3: begin
                if(wd_finish) begin state<=DONE; wd_addr_3<=wd_addr_3+wd_burst_len; end
                else if(wfifo_rcount_3_sel >= wd_burst_len-1) begin wd_len<=wd_burst_len; wd_req<=1'b1; end
            end
            WRITE_4: begin
                if(wd_finish) begin
                    state<=DONE;
                    // 网络流没有场消隐，不复位异步FIFO；按固定帧长度自动换页。
                    if(wd_addr_4 + wd_burst_len >= ETH_FRAME_ADDR_UNITS) begin
                        wd_addr_4<=addr_wd_min[19:0];
                    end else
                        wd_addr_4<=wd_addr_4+wd_burst_len;
                end else if(wfifo_rcount_4 >= wd_burst_len-1'b1) begin
                    wd_len<=wd_burst_len; wd_req<=1'b1;
                end
            end
            READ_1: begin
                if(rd_finish) begin state<=DONE; rd_addr_1<=rd_addr_1+rd_burst_len; end
                else if(rfifo_wcount_1 <= rd_burst_len-1) begin rd_len<=rd_burst_len; rd_req<=1'b1; end
            end
            READ_2: begin
                if(rd_finish) begin state<=DONE; rd_addr_2<=rd_addr_2+rd_burst_len; end
                else if(rfifo_wcount_2 <= rd_burst_len-1) begin rd_len<=rd_burst_len; rd_req<=1'b1; end
            end
            READ_3: begin
                if(rd_finish) begin state<=DONE; rd_addr_3<=rd_addr_3+rd_burst_len; end
                else if(rfifo_wcount_3 <= rd_burst_len-1) begin rd_len<=rd_burst_len; rd_req<=1'b1; end
            end
            READ_4: begin
                if(rd_finish) begin state<=DONE; rd_addr_4<=rd_addr_4+rd_burst_len; end
                else if(rfifo_wcount_4 <= rd_burst_len-1) begin rd_len<=rd_burst_len; rd_req<=1'b1; end
            end
            BEV_SOURCE_READ: begin
                if(rd_finish) state<=DONE;
                else begin rd_len<=10'd1; rd_req<=1'b1; end
            end
            BEV_FRAME_WRITE: begin
                if(wd_finish) state<=DONE;
                else begin wd_len<=10'd1; wd_req<=1'b1; end
            end
            BEV_DISPLAY_READ: begin
                if(rd_finish) begin
                    state<=DONE;
                    if(bev_read_word + BEV_DISPLAY_BURST >= BEV_FRAME_WORDS)
                        bev_read_word<=BEV_FRAME_WORDS;
                    else
                        bev_read_word<=bev_read_word+BEV_DISPLAY_BURST;
                end else begin
                    rd_len<=BEV_DISPLAY_BURST;
                    rd_req<=1'b1;
                end
            end
            default: state<=IDLE;
        endcase
    end
end
endmodule
