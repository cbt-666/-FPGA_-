//****************************************Copyright (c)***********************************//
//原子哥在线教学平台：www.yuanzige.com
//技术支持：www.openedv.com
//淘宝店铺：http://openedv.taobao.com
//关注微信公众平台微信号："正点原子"，免费获取ZYNQ & FPGA & STM32 & LINUX资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2018-2028
//All rights reserved
//----------------------------------------------------------------------------------------
// File name:           top_dual_ov5640_hdmi
// Last modified Date:  2020/05/04 9:19:08
// Last Version:        V1.0
// Descriptions:        三路OV5640、HDMI和以太网五路输入显示
//                      
//----------------------------------------------------------------------------------------
// Created by:          正点原子
// Created date:        2019/05/04 9:19:08
// Version:             V1.0
// Descriptions:        The original version
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module top_dual_ov5640_hdmi(    
    input                 sys_clk        ,  //系统时钟
    input                 sys_rst_n      ,  //系统复位，低电平有效
    input                 touch_key      ,  //触摸按键，高电平有效
    input       [0:3]     key            ,  //机械按键，低电平有效
    output                led0           ,  //状态指示灯，仅保留LED0
    output                beep           ,  //蜂鸣器
    output                voice_uart_txd ,  //JQ8900-16P语音模块串口发送
    input                 ultrasonic_echo,  //超声波回波，3.3 V输入
    output                ultrasonic_trig,  //超声波触发，3.3 V输出
    //摄像头1接口                       
    input                 cam_pclk_1     ,  //cmos 数据像素时钟
    input                 cam_vsync_1    ,  //cmos 场同步信号
    input                 cam_href_1     ,  //cmos 行同步信号
    input   [7:0]         cam_data_1     ,  //cmos 数据
    output                cam_rst_n_1    ,  //cmos 复位信号，低电平有效
    output                cam_pwdn_1     ,  //电源休眠模式选择 0：正常模式 1：电源休眠模式
    output                cam_scl_1      ,  //cmos SCCB_SCL线
    inout                 cam_sda_1      ,  //cmos SCCB_SDA线
    //摄像头2接口     
    input                 cam_pclk_2     ,  //cmos 数据像素时钟
    input                 cam_vsync_2    ,  //cmos 场同步信号
    input                 cam_href_2     ,  //cmos 行同步信号
    input   [7:0]         cam_data_2     ,  //cmos 数据
    output                cam_rst_n_2    ,  //cmos 复位信号，低电平有效
    output                cam_pwdn_2     ,  //电源休眠模式选择 0：正常模式 1：电源休眠模式
    output                cam_scl_2      ,  //cmos SCCB_SCL线
    inout                 cam_sda_2      ,  //cmos SCCB_SDA线
    //单目摄像头接口
    input                 cam_pclk_3     ,  //cmos 数据像素时钟
    input                 cam_vsync_3    ,  //cmos 场同步信号
    input                 cam_href_3     ,  //cmos 行同步信号
    input   [7:0]         cam_data_3     ,  //cmos 数据
    output                cam_rst_n_3    ,  //cmos 复位信号，低电平有效
    output                cam_pwdn_3     ,  //电源休眠模式选择 0：正常模式 1：电源休眠模式
    output                cam_scl_3      ,  //cmos SCCB_SCL线
    inout                 cam_sda_3      ,  //cmos SCCB_SDA线
    //HDMI输入（ADV7611并行RGB接口）
    input                 hdmi_clk_in    ,
    input                 hdmi_de_in     ,
    input                 hdmi_vs_in     ,
    input                 hdmi_hs_in     ,
    input   [23:0]        hdmi_data_in   ,
    output                hdmi_rst_n_out ,
    output                hdmi_scl       ,
    inout                 hdmi_sda       ,
    //千兆以太网RGMII接口
    input                 eth_rxc        ,
    input                 eth_rx_ctl     ,
    input   [3:0]         eth_rxd        ,
    output                eth_txc        ,
    output                eth_tx_ctl     ,
    output  [3:0]         eth_txd        ,
    output                eth_rst_n      ,
    //第二路千兆以太网RGMII接口（GE1）
    input                 eth1_rxc       ,
    input                 eth1_rx_ctl    ,
    input       [3:0]     eth1_rxd       ,
    output                eth1_txc       ,
    output                eth1_tx_ctl    ,
    output      [3:0]     eth1_txd       ,
    // DDR3                            
    output                mem_rst_n      ,                       
    output                mem_ck         ,
    output                mem_ck_n       ,
    output                mem_cke        ,
    output                mem_ras_n      ,
    output                mem_cas_n      ,
    output                mem_we_n       , 
    output                mem_odt        ,
    output                mem_cs_n       ,
    output  [14:0]        mem_a          ,   
    output  [2:0]         mem_ba         ,   
    inout   [3:0]         mem_dqs        ,
    inout   [3:0]         mem_dqs_n      ,
    inout   [31:0]        mem_dq         ,
    output  [3:0]         mem_dm         ,									   
    //hdmi接口                            
    output                tmds_clk_p     ,  // TMDS 时钟通道
    output                tmds_clk_n     ,
    output  [2:0]         tmds_data_p    ,  // TMDS 数据通道
    output  [2:0]         tmds_data_n     
);                                 
//parameter define
parameter  V_CMOS_DISP = 13'd360        ;   //单路摄像头画面高度（四宫格上半屏）
parameter  H_CMOS_DISP = 13'd1280       ;   //CMOS分辨率--列	
parameter  TOTAL_H_PIXEL = 13'd2570     ; 
parameter  TOTAL_V_PIXEL = 13'd980      ;   
parameter  H_CMOS_DISP_HALF = 13'd640   ;   //CMOS分辨率--行的一半


//wire define                          
wire         hdmi_pll_locked              ;  //HDMI输入时钟PLL锁定信号
wire         rst_n                        ;  //采集、网络和DDR复位
wire         video_rst_n                  ;  //HDMI显示链复位
wire         rdata_req                   ;  //DDR3控制器模块读使能
wire  [15:0] rddata                      ;  //DDR3控制器模块读数据
wire         cmos_frame_valid_1          ;  //数据1有效使能信号
wire  [15:0] wr_data_1                   ;  //DDR3控制器模块写数据1
wire         cmos_frame_valid_2          ;  //数据2有效使能信号
wire  [15:0] wr_data_2                   ;  //DDR3控制器模块写数据2
wire         cmos_frame_valid_3          ;  //单目摄像头数据有效使能信号
wire  [15:0] wr_data_3                   ;  //DDR3控制器模块写数据3
wire         init_calib_complete         ;  //DDR3初始化完成init_calib_complete
wire         sys_init_done               ;  //系统初始化完成(DDR初始化+摄像头初始化)
wire         cmos_frame_vsync_1          ;  //输出帧1有效场同步信号
wire         cmos_frame_vsync_2          ;  //输出帧2有效场同步信号
wire         cmos_frame_vsync_3          ;  //输出帧3有效场同步信号
//PLL
wire         pixel_clk                   ;  //74.25 MHz HDMI pixel clock
wire         pixel_clk_5x                ;  //371.25 MHz HDMI 5x clock
wire         clk_50m                     ;  //50 MHz board oscillator domain
wire         quad_mode_req               ;  //0: 全屏，1: 四宫格
wire         zoom_fullscreen_req         ;  //1: 显示选中一路DDR缓存的放大全屏
wire  [1:0]  zoom_source_req             ;  //0/1/2/3：双目1/双目2/单目/以太网
wire  [1:0]  zoom_source_active          ;
wire         quad_mode_active            ;
wire         zoom_fullscreen             ;
wire  [1:0]  local_mode_req              ;
wire         bev_mode_req                ;
wire         back_left_hdmi_req          ; // BACK_LEFT输入：0摄像头3，1 HDMI
wire         bev_mode_active             ;
wire         bev_rd_en                   ;
wire [15:0]  bev_rd_data                 ;
wire         bev_rd_valid                ;
wire         bev_rd_fault                ;
wire         bev_frame_ready             ;
wire         touch_active_sys            ;
wire  [1:0]  ultrasonic_alert_level      ;
wire         ultrasonic_warning_active   ;
wire         ultrasonic_beep             ;
wire         mechanical_beep             ;
wire [23:0]  processed_rgb;
wire         processed_hs, processed_vs, processed_de;
wire [23:0]  pass_rgb, yrg_rgb, binary_rgb;
wire         stream_hs, stream_vs, stream_de;
wire [23:0]  network_rgb;
wire [23:0]  overlay_rgb;
wire         overlay_hs, overlay_vs, overlay_de;
reg  [23:0]  overlay_rgb_px;
reg          overlay_hs_px, overlay_vs_px, overlay_de_px;
wire [1:0]   process_mode_active;
wire         night_active_hdmi;
wire [15:0]  eth1_command_word;
wire         eth1_command_toggle;
wire         eth1_single_arrow_active;
wire [15:0]  hdmi_back_left_data;
wire         hdmi_back_left_valid;
wire         hdmi_back_left_frame;
(* ASYNC_REG = "TRUE" *) reg back_left_hdmi_meta_cam3, back_left_hdmi_sync_cam3;
(* ASYNC_REG = "TRUE" *) reg back_left_hdmi_meta_hdmi, back_left_hdmi_sync_hdmi;

//HDMI
wire         video_vs                    ;  //场同步信号
wire  [10:0] pixel_ypos                  ;  //HDMI当前有效行坐标
wire  [10:0] pixel_xpos                  ;  //HDMI当前有效列坐标
wire         hdmi_input_init_done        ;  //ADV7611初始化完成
// ADV7611复位保持10 ms（50 MHz时钟），给接收器内部电源和PLL足够稳定时间。
reg   [23:0] hdmi_reset_cnt              ;
localparam [23:0] HDMI_RESET_HOLD_CYCLES = 24'd500000;
// ADV7611配置完成后若恢复像素时钟仍未锁定，自动重新复位并配置。
// 这样可处理上电时ADV7611残留配置或输入源启动较慢的情况。
reg   [23:0] hdmi_lock_watchdog_cnt      ;
reg   [1:0]  hdmi_reset_state            ;
localparam [23:0] HDMI_LOCK_TIMEOUT_CYCLES = 24'd12500000; // 250 ms @ 50 MHz
localparam [1:0] HDMI_ST_RESET = 2'd0,
                 HDMI_ST_START = 2'd1,
                 HDMI_ST_WAIT  = 2'd2,
                 HDMI_ST_RETRY = 2'd3;
(* ASYNC_REG = "TRUE" *) reg command_toggle_meta, command_toggle_sync;
reg command_toggle_seen;
(* ASYNC_REG = "TRUE" *) reg [15:0] command_word_meta, command_word_sync;
reg  [15:0]  command_word_active;
(* ASYNC_REG = "TRUE" *) reg [1:0] local_mode_meta, local_mode_sync;
(* ASYNC_REG = "TRUE" *) reg touch_active_meta, touch_active_sync;
(* ASYNC_REG = "TRUE" *) reg ultrasonic_warning_meta_hdmi;
(* ASYNC_REG = "TRUE" *) reg ultrasonic_warning_sync_hdmi;
(* ASYNC_REG = "TRUE" *) reg night_active_meta_sys, night_active_sync_sys;
(* ASYNC_REG = "TRUE" *) reg mechanical_beep_meta_sys, mechanical_beep_sync_sys;
(* ASYNC_REG = "TRUE" *) reg zebra_active_meta_sys, zebra_active_sync_sys;
(* ASYNC_REG = "TRUE" *) reg blue_arrow_meta_sys, blue_arrow_sync_sys;
(* ASYNC_REG = "TRUE" *) reg single_arrow_meta_sys, single_arrow_sync_sys;
// 显示布局状态由pixel_clk域产生，回传HDMI输入域后再参与叠加控制。
(* ASYNC_REG = "TRUE" *) reg quad_active_meta_hdmi, quad_active_sync_hdmi;
(* ASYNC_REG = "TRUE" *) reg zoom_active_meta_hdmi, zoom_active_sync_hdmi;
(* ASYNC_REG = "TRUE" *) reg [1:0] zoom_source_meta_hdmi, zoom_source_sync_hdmi;
(* ASYNC_REG = "TRUE" *) reg bev_active_meta_hdmi, bev_active_sync_hdmi;
// 语音事件先在HDMI输入时钟域完成判断并寄存，再以单比特跨域到50 MHz。
// 这样不会在50 MHz域直接读取command_word_active或多路显示状态。
wire night_voice_event_hdmi;
voice_night_event u_voice_night_event (
    .clk(hdmi_clk_in), .rst_n(video_rst_n),
    .frame_vs(hdmi_vs_in), .frame_de(hdmi_de_in),
    .night_active(night_active_hdmi),
    .hdmi_visible(!quad_active_sync_hdmi && !zoom_active_sync_hdmi &&
                  !bev_active_sync_hdmi),
    .event_active(night_voice_event_hdmi)
);
wire zebra_voice_event_hdmi = command_word_active[3] &&
                              !quad_active_sync_hdmi &&
                              !zoom_active_sync_hdmi &&
                              !bev_active_sync_hdmi &&
                              (process_mode_active == 2'd0) &&
                              !ultrasonic_warning_sync_hdmi;
// 三个方向中任意一个蓝色箭头显示时产生统一的蓝箭头语音事件。
// 仅在HDMI原始全屏叠加界面有效，避免四宫格/DDR放大时误播。
wire blue_arrow_voice_event_hdmi = overlay_allowed &&
                                   (process_mode_active == 2'd0) &&
                                   ((command_word_active[0] && (command_word_active[5:4] == 2'b10)) ||
                                    (command_word_active[1] && (command_word_active[7:6] == 2'b10)) ||
                                    (command_word_active[2] && (command_word_active[9:8] == 2'b10)));
reg night_voice_level_hdmi;
reg zebra_voice_level_hdmi;
reg blue_arrow_level_hdmi;
reg mechanical_voice_level_hdmi;
//Ethernet RGB565
wire         gmii_rx_clk                  ;
wire         udp_rec_en                   ;
wire  [7:0]  udp_rec_data                 ;
wire  [15:0] eth_ddr_data                 ;
wire         eth_ddr_valid                ;
wire         eth_frame_valid              ;
wire         eth_frame_ready              ;
(* ASYNC_REG = "TRUE" *) reg eth_frame_ready_q1;
(* ASYNC_REG = "TRUE" *) reg eth_frame_ready_q2;
// key_sync0由按键消抖模块驱动，必须使用wire；key_sync1为HDMI时钟域同步寄存器。
wire [3:0] key_sync0;
(* ASYNC_REG = "TRUE" *) reg [3:0] key_sync_meta_hdmi;
(* ASYNC_REG = "TRUE" *) reg [3:0] key_sync1;
reg [24:0] blink_cnt;
// 每次机械键按下先显示图形，再进入闪烁，避免短按刚好落在隐藏相位而无反馈。
wire blink_visible = ~blink_cnt[24];

//*****************************************************
//**                    main code
//*****************************************************

// HDMI输入失锁时不再复位显示链；摄像头、以太网和DDR写侧继续工作。
assign  rst_n = sys_rst_n;
// HDMI显示链复位只跟随系统复位，沿用参考工程的启动方式。
// ADV7611初始化完成和恢复时钟PLL锁定用于状态指示/采集有效性判断，
// 不能直接串到显示链复位上，否则任一条件暂时未满足时TMDS发送器会
// 一直保持复位，烧录后显示器表现为无信号。
assign  video_rst_n = rst_n;

// Keep the camera/SCCB and DDR control domain on the board oscillator.
// The PLL is used only for the HDMI pixel and 5x TMDS clocks.
assign  clk_50m = sys_clk;

//系统初始化完成：DDR3初始化完成
assign  sys_init_done = init_calib_complete;
assign  hdmi_rst_n_out = (hdmi_reset_state != HDMI_ST_RESET);

// ADV7611复位释放由板载50MHz时钟产生，避免依赖尚未建立的恢复像素时钟。
// 配置完成后等待PLL锁定；超时则再次给ADV7611一个完整复位脉冲。
always @(posedge clk_50m or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        hdmi_reset_cnt         <= 24'd0;
        hdmi_lock_watchdog_cnt <= 24'd0;
        hdmi_reset_state       <= HDMI_ST_RESET;
    end else begin
        case (hdmi_reset_state)
            HDMI_ST_RESET: begin
                hdmi_lock_watchdog_cnt <= 24'd0;
                if (hdmi_reset_cnt < HDMI_RESET_HOLD_CYCLES)
                    hdmi_reset_cnt <= hdmi_reset_cnt + 1'b1;
                else begin
                    hdmi_reset_cnt   <= 24'd0;
                    hdmi_reset_state <= HDMI_ST_START;
                end
            end
            HDMI_ST_START: begin
                hdmi_reset_cnt         <= 24'd0;
                hdmi_lock_watchdog_cnt <= 24'd0;
                hdmi_reset_state       <= HDMI_ST_WAIT;
            end
            HDMI_ST_WAIT: begin
                hdmi_reset_cnt <= 24'd0;
                // 初始化未完成或PLL未锁定都视为本轮启动失败，统一计时重试。
                // 仅在PLL锁定后清零，避免I2C无应答时永久卡在等待状态。
                if (hdmi_pll_locked) begin
                    hdmi_lock_watchdog_cnt <= 24'd0;
                end else if (hdmi_lock_watchdog_cnt < HDMI_LOCK_TIMEOUT_CYCLES) begin
                    hdmi_lock_watchdog_cnt <= hdmi_lock_watchdog_cnt + 1'b1;
                end else begin
                    hdmi_lock_watchdog_cnt <= 24'd0;
                    hdmi_reset_state       <= HDMI_ST_RETRY;
                end
            end
            HDMI_ST_RETRY: begin
                hdmi_lock_watchdog_cnt <= 24'd0;
                if (hdmi_reset_cnt < HDMI_RESET_HOLD_CYCLES)
                    hdmi_reset_cnt <= hdmi_reset_cnt + 1'b1;
                else begin
                    hdmi_reset_cnt   <= 24'd0;
                    hdmi_reset_state <= HDMI_ST_START;
                end
            end
            default: begin
                hdmi_reset_cnt         <= 24'd0;
                hdmi_lock_watchdog_cnt <= 24'd0;
                hdmi_reset_state       <= HDMI_ST_RESET;
            end
        endcase
    end
end

// 触摸短按切换显示布局，长按配合机械键选择本地HDMI处理模式。
touch_mode_ctrl u_touch_mode_ctrl(
    .sys_clk       (clk_50m),
    .sys_rst_n     (rst_n),
    .touch_key     (touch_key),
    .key_filtered  (key_sync0),
    .warning_active(ultrasonic_warning_active),
    .quad_mode_req (quad_mode_req),
    .zoom_fullscreen_req(zoom_fullscreen_req),
    .zoom_source_req(zoom_source_req),
    .local_mode_req(local_mode_req),
    .bev_mode_req  (bev_mode_req),
    .back_left_hdmi_req(back_left_hdmi_req),
    .touch_active  (touch_active_sys)
);

// 超声波告警链路独立运行于50 MHz系统时钟域。
ultrasonic_ranging u_ultrasonic_ranging (
    .sys_clk     (clk_50m),
    .sys_rst_n   (rst_n),
    .echo        (ultrasonic_echo),
    .trig        (ultrasonic_trig),
    .alert_level (ultrasonic_alert_level)
);

assign ultrasonic_warning_active = (ultrasonic_alert_level != 2'd0);

warning_buzzer u_warning_buzzer (
    .sys_clk     (clk_50m),
    .sys_rst_n   (rst_n),
    .alert_level (ultrasonic_alert_level),
    .beep        (ultrasonic_beep)
);

// 机械按键仅用于原有箭头/蜂鸣器逻辑。
genvar key_i;
generate
    for (key_i = 0; key_i < 4; key_i = key_i + 1) begin : g_key_debounce
        key_debounce #(.CNT_MAX(20'd1_000_000)) u_key_debounce(
            .sys_clk    (clk_50m),
            .sys_rst_n  (rst_n),
            .key        (key[key_i]),
            .key_filter (key_sync0[key_i])
        );
    end
endgenerate

always @(posedge hdmi_clk_in or negedge rst_n) begin
    if (!rst_n) begin
        key_sync_meta_hdmi <= 4'b1111;
        key_sync1 <= 4'b1111;
        blink_cnt <= 25'd0;
    end else begin
        // 按键消抖输出来自50 MHz域，进入HDMI像素域采用两级同步。
        key_sync_meta_hdmi <= key_sync0;
        key_sync1 <= key_sync_meta_hdmi;
        if (&key_sync1)
            blink_cnt <= 25'd0;
        else
            blink_cnt <= blink_cnt + 1'b1;
    end
end

// BACK_LEFT输入选择跨到两路写入时钟域。两路输入均保持独立时钟，
// 只在各自域内门控写使能，避免直接组合切换cam_pclk_3和hdmi_clk_in。
always @(posedge cam_pclk_3 or negedge rst_n) begin
    if (!rst_n) begin
        back_left_hdmi_meta_cam3 <= 1'b0;
        back_left_hdmi_sync_cam3 <= 1'b0;
    end else begin
        back_left_hdmi_meta_cam3 <= back_left_hdmi_req;
        back_left_hdmi_sync_cam3 <= back_left_hdmi_meta_cam3;
    end
end

always @(posedge hdmi_clk_in or negedge rst_n) begin
    if (!rst_n) begin
        back_left_hdmi_meta_hdmi <= 1'b0;
        back_left_hdmi_sync_hdmi <= 1'b0;
    end else begin
        back_left_hdmi_meta_hdmi <= back_left_hdmi_req;
        back_left_hdmi_sync_hdmi <= back_left_hdmi_meta_hdmi;
    end
end

// 布局状态变化只发生在场边界；双触发同步避免pixel_clk域组合量直接进入叠加关键路径。
always @(posedge hdmi_clk_in or negedge rst_n) begin
    if (!rst_n) begin
        quad_active_meta_hdmi  <= 1'b0;
        quad_active_sync_hdmi  <= 1'b0;
        zoom_active_meta_hdmi  <= 1'b0;
        zoom_active_sync_hdmi  <= 1'b0;
        zoom_source_meta_hdmi  <= 2'd2;
        zoom_source_sync_hdmi  <= 2'd2;
        bev_active_meta_hdmi   <= 1'b0;
        bev_active_sync_hdmi   <= 1'b0;
    end else begin
        quad_active_meta_hdmi  <= quad_mode_active;
        quad_active_sync_hdmi  <= quad_active_meta_hdmi;
        zoom_active_meta_hdmi  <= zoom_fullscreen;
        zoom_active_sync_hdmi  <= zoom_active_meta_hdmi;
        zoom_source_meta_hdmi  <= zoom_source_active;
        zoom_source_sync_hdmi  <= zoom_source_meta_hdmi;
        bev_active_meta_hdmi   <= bev_mode_active;
        bev_active_sync_hdmi   <= bev_active_meta_hdmi;
    end
end

assign led0 = hdmi_input_init_done;

// HDMI域的语音源状态先锁存一个像素时钟周期，消除多路组合条件在
// 跨到50 MHz域时可能产生的窄毛刺。夜间状态不依赖处理模式，因而
// RGB888、YRG和二值化全屏均可产生night事件。
always @(posedge hdmi_clk_in or negedge rst_n) begin
    if (!rst_n) begin
        night_voice_level_hdmi     <= 1'b0;
        zebra_voice_level_hdmi     <= 1'b0;
        blue_arrow_level_hdmi      <= 1'b0;
        mechanical_voice_level_hdmi <= 1'b0;
    end else begin
        night_voice_level_hdmi      <= night_voice_event_hdmi;
        zebra_voice_level_hdmi      <= zebra_voice_event_hdmi;
        blue_arrow_level_hdmi       <= blue_arrow_voice_event_hdmi;
        mechanical_voice_level_hdmi <= mechanical_beep;
    end
end

// 语音事件同步到50 MHz控制域；语音模块不进入HDMI像素关键路径。
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        night_active_meta_sys <= 1'b0;
        night_active_sync_sys <= 1'b0;
        mechanical_beep_meta_sys <= 1'b0;
        mechanical_beep_sync_sys <= 1'b0;
        zebra_active_meta_sys <= 1'b0;
        zebra_active_sync_sys <= 1'b0;
        blue_arrow_meta_sys <= 1'b0;
        blue_arrow_sync_sys <= 1'b0;
        single_arrow_meta_sys <= 1'b0;
        single_arrow_sync_sys <= 1'b0;
    end else begin
        // 夜间增强位于HDMI输入和YRG/二值化支路之前，因此YRG、二值化
        // 全屏时仍允许播报night；仅在非HDMI显示布局时屏蔽。
        night_active_meta_sys <= night_voice_level_hdmi;
        night_active_sync_sys <= night_active_meta_sys;
        mechanical_beep_meta_sys <= mechanical_voice_level_hdmi;
        mechanical_beep_sync_sys <= mechanical_beep_meta_sys;
        // bit3就是黄色边缘警告显示位，与三个箭头颜色无关。
        zebra_active_meta_sys <= zebra_voice_level_hdmi;
        zebra_active_sync_sys <= zebra_active_meta_sys;
        blue_arrow_meta_sys <= blue_arrow_level_hdmi;
        blue_arrow_sync_sys <= blue_arrow_meta_sys;
        // 在以太网接收域依据合法整包寄存，再单比特两级同步。
        // 按收到的指令触发，不依赖当前显示布局或机械按键临时叠加效果。
        single_arrow_meta_sys <= eth1_single_arrow_active;
        single_arrow_sync_sys <= single_arrow_meta_sys;
    end
end

// 语音事件位定义：bit4红色超声波，bit3黄色超声波，bit2错误方向，
// bit1黄色边缘警告，bit0夜间增强，bit5蓝色箭头，bit6红/绿/黄单箭头指令。
// 橙色超声波不产生事件；单箭头播报优先级最低。
wire [6:0] voice_source_active = {
    single_arrow_sync_sys,
    blue_arrow_sync_sys,
    (ultrasonic_alert_level == 2'd3),
    (ultrasonic_alert_level == 2'd1),
    mechanical_beep_sync_sys,
    zebra_active_sync_sys,
    night_active_sync_sys
};
wire [6:0] voice_source_valid = voice_source_active;

voice_event_ctrl u_voice_event_ctrl (
    .clk              (clk_50m),
    .rst_n            (rst_n),
    .source_active    (voice_source_active),
    .source_valid     (voice_source_valid),
    .uart_txd         (voice_uart_txd)
);


// HDMI发送时钟由ADV7611恢复时钟同源生成。输入采集使用hdmi_clk_in，
// DDR显示读侧和鸟瞰显示使用pixel_clk；PLL的1倍输出带固定半周期相移，
// 供TMDS编码安全采样，5倍输出供串行器使用。
pll u_hdmi_video_pll(
    .clkin1  (hdmi_clk_in),
    .clkout0 (pixel_clk),
    .clkout1 (pixel_clk_5x),
    .lock    (hdmi_pll_locked)
);

// ADV7611寄存器及EDID初始化，参考hdmi_yrg_hdmi工程，输出制式为720p60 RGB888。
hdmi_adv7611_ctrl u_hdmi_adv7611_ctrl(
    .clk       (clk_50m),
    .rst_n     (rst_n & hdmi_rst_n_out),
    .scl       (hdmi_scl),
    .sda       (hdmi_sda),
    .init_done (hdmi_input_init_done)
);

// 以太网协议栈：本机192.168.1.10，接收UDP载荷中的640x360 RGB565图像。
eth_top #(
    .BOARD_MAC    (48'h00_11_22_33_44_55),
    .BOARD_IP     ({8'd192,8'd168,8'd1,8'd10}),
    .DES_MAC      (48'hff_ff_ff_ff_ff_ff),
    .DES_IP       ({8'd192,8'd168,8'd1,8'd102}),
    .UDP_LOOPBACK (1'b0)
) u_eth_top (
    .sys_rst_n    (rst_n),
    .eth_rxc      (eth_rxc),
    .eth_rx_ctl   (eth_rx_ctl),
    .eth_rxd      (eth_rxd),
    .eth_txc      (eth_txc),
    .eth_tx_ctl   (eth_tx_ctl),
    .eth_txd      (eth_txd),
    .eth_rst_n    (eth_rst_n),
    .udp_rec_en   (udp_rec_en),
    .udp_rec_data (udp_rec_data),
    .gmii_rx_clk  (gmii_rx_clk)
);

// UDP载荷按高字节在前组装为RGB565，字节相位跨UDP包连续保持。
udp_data u_udp_data (
    .clk              (gmii_rx_clk),
    .rst_n            (rst_n),
    .rec_en           (udp_rec_en),
    .rec_data         (udp_rec_data),
    .picture_data_vld (eth_ddr_valid),
    .frame_valid      (eth_frame_valid),
    .picture_data     (eth_ddr_data)
);

// GE1：独立的UDP命令接收和处理后HDMI图像发送通道；PHY复位与GE0共用N20。
eth_udp_loop #(
    // 两个以太网物理网络彼此独立，GE1沿用参考工程MAC地址。
    .BOARD_MAC    (48'h00_11_22_33_44_55),
    // GE1所在物理网络独立，因此沿用参考工程的地址配置。
    .BOARD_IP     ({8'd192,8'd168,8'd1,8'd10}),
    .DES_MAC      (48'hff_ff_ff_ff_ff_ff),
    .DES_IP       ({8'd192,8'd168,8'd1,8'd102})
) u_eth_udp_loop (
    .sys_clk       (clk_50m),
    .sys_rst_n     (rst_n),
    .eth_rxc       (eth1_rxc),
    .eth_rx_ctl    (eth1_rx_ctl),
    .eth_rxd       (eth1_rxd),
    .eth_txc       (eth1_txc),
    .eth_tx_ctl    (eth1_tx_ctl),
    .eth_txd       (eth1_txd),
    .eth_rst_n     (eth_rst_n),
    .image_pclk    (hdmi_clk_in),
    .image_hs      (stream_hs),
    .image_vs      (stream_vs),
    .image_de      (stream_de),
    .image_rgb     (network_rgb),
    .image_mode    (command_word_active[11:10]),
    .command_word  (eth1_command_word),
    .command_toggle(eth1_command_toggle),
    .single_arrow_active(eth1_single_arrow_active)
);

// GE1两字节命令和本地显示模式分别跨到HDMI像素域。
always @(posedge hdmi_clk_in or negedge rst_n) begin
    if (!rst_n) begin
        command_toggle_meta <= 1'b0;
        command_toggle_sync <= 1'b0;
        command_toggle_seen <= 1'b0;
        command_word_meta   <= 16'd0;
        command_word_sync   <= 16'd0;
        command_word_active <= 16'd0;
        local_mode_meta     <= 2'd0;
        local_mode_sync     <= 2'd0;
        touch_active_meta   <= 1'b0;
        touch_active_sync   <= 1'b0;
        ultrasonic_warning_meta_hdmi <= 1'b0;
        ultrasonic_warning_sync_hdmi <= 1'b0;
    end else begin
        command_toggle_meta <= eth1_command_toggle;
        command_toggle_sync <= command_toggle_meta;
        command_word_meta   <= eth1_command_word;
        command_word_sync   <= command_word_meta;
        local_mode_meta     <= local_mode_req;
        local_mode_sync     <= local_mode_meta;
        touch_active_meta   <= touch_active_sys;
        touch_active_sync   <= touch_active_meta;
        ultrasonic_warning_meta_hdmi <= ultrasonic_warning_active;
        ultrasonic_warning_sync_hdmi <= ultrasonic_warning_meta_hdmi;
        if (command_toggle_sync != command_toggle_seen) begin
            command_toggle_seen <= command_toggle_sync;
            command_word_active <= command_word_sync;
        end
    end
end

hdmi_rx u_hdmi_rx(
    .hdmi_clk_in  (hdmi_clk_in),
    .rst_n        (video_rst_n),
    .hdmi_de_in   (hdmi_de_in),
    .hdmi_vs_in   (hdmi_vs_in),
    .hdmi_hs_in   (hdmi_hs_in),
    .hdmi_data_in (hdmi_data_in),
    .mode_select  (local_mode_sync),
    .pclk         (),
    .hsync        (processed_hs),
    .vsync        (processed_vs),
    .de           (processed_de),
    .rgb_data     (processed_rgb),
    .mode_active  (process_mode_active),
    .rgb_pass     (pass_rgb),
    .rgb_yrg      (yrg_rgb),
    .rgb_binary   (binary_rgb),
    .stream_hsync (stream_hs),
    .stream_vsync (stream_vs),
    .stream_de    (stream_de),
    .night_active (night_active_hdmi)
);

// BACK_LEFT的HDMI候选源：使用夜间增强后的RGB直通支路，
// 不随全屏YRG/二值化模式改变；缩放后进入独立HDMI写FIFO。
hdmi_input_capture u_hdmi_back_left_capture(
    .in_clk    (hdmi_clk_in),
    .in_rst_n  (video_rst_n),
    .in_de     (stream_de),
    .in_vs     (stream_vs),
    .in_hs     (stream_hs),
    .in_rgb    (pass_rgb),
    .out_data  (hdmi_back_left_data),
    .out_valid (hdmi_back_left_valid),
    .out_frame (hdmi_back_left_frame)
);

assign network_rgb = (command_word_active[11:10] == 2'd0) ? pass_rgb :
                     (command_word_active[11:10] == 2'd1) ? yrg_rgb : binary_rgb;

// 图形叠加只允许出现在“原始HDMI RGB888全屏”状态。
wire overlay_allowed = !quad_active_sync_hdmi && !zoom_active_sync_hdmi &&
                       !bev_active_sync_hdmi &&
                       (zoom_source_sync_hdmi == 2'd2) &&
                       (process_mode_active == 2'd0) &&
                       !ultrasonic_warning_sync_hdmi;
wire key0_only = overlay_allowed && !touch_active_sync &&
                 !key_sync1[0] && key_sync1[1] && key_sync1[2] && key_sync1[3];
wire key1_only = overlay_allowed && !touch_active_sync &&
                 key_sync1[0] && !key_sync1[1] && key_sync1[2] && key_sync1[3];
wire key2_only = overlay_allowed && !touch_active_sync &&
                 key_sync1[0] && key_sync1[1] && !key_sync1[2] && key_sync1[3];
wire key3_only = overlay_allowed && !touch_active_sync &&
                 key_sync1[0] && key_sync1[1] && key_sync1[2] && !key_sync1[3];

wire cmd_left_enable = command_word_active[0];
wire cmd_right_enable = command_word_active[1];
wire cmd_up_enable = command_word_active[2];
wire cmd_warning_enable = command_word_active[3];
wire [1:0] cmd_left_color = command_word_active[5:4];
wire [1:0] cmd_right_color = command_word_active[7:6];
wire [1:0] cmd_up_color = command_word_active[9:8];
// 蓝色箭头表示允许通行，与绿色/黄色一样不触发错误方向报警。
wire left_safe = cmd_left_enable && (cmd_left_color != 2'b00);
wire right_safe = cmd_right_enable && (cmd_right_color != 2'b00);
wire up_safe = cmd_up_enable && (cmd_up_color != 2'b00);
wire left_exclusive = key0_only && left_safe;
wire right_exclusive = key1_only && right_safe;
wire up_exclusive = key2_only && up_safe;
wire any_exclusive = left_exclusive || right_exclusive || up_exclusive;
wire left_alarm = key0_only && !left_safe;
wire right_alarm = key1_only && !right_safe;
wire up_alarm = key2_only && !up_safe;

wire overlay_left_enable = any_exclusive ? left_exclusive :
                           (cmd_left_enable || left_alarm);
wire overlay_right_enable = any_exclusive ? right_exclusive :
                            (cmd_right_enable || right_alarm);
wire overlay_up_enable = any_exclusive ? up_exclusive :
                         (cmd_up_enable || up_alarm);
wire overlay_warning_enable = any_exclusive ? 1'b0 : cmd_warning_enable;
wire [1:0] overlay_left_color = (!cmd_left_enable && left_alarm) ? 2'b00 : cmd_left_color;
wire [1:0] overlay_right_color = (!cmd_right_enable && right_alarm) ? 2'b00 : cmd_right_color;
wire [1:0] overlay_up_color = (!cmd_up_enable && up_alarm) ? 2'b00 : cmd_up_color;
wire left_blink = key0_only;
wire right_blink = key1_only;
wire up_blink = key2_only || (key3_only && cmd_up_enable && (cmd_up_color == 2'b00));
wire warning_blink = key3_only && cmd_warning_enable;

assign mechanical_beep = overlay_allowed && (left_alarm || right_alarm || up_alarm);
// 超声波告警具有最高优先级；告警期间机械按键报警逻辑已由overlay_allowed屏蔽。
assign beep = ultrasonic_warning_active ? ultrasonic_beep : mechanical_beep;

image_overlay u_image_overlay(
    .clk(hdmi_clk_in), .rst_n(video_rst_n), .de_in(processed_de), .hs_in(processed_hs), .vs_in(processed_vs), .data_in(processed_rgb),
    .overlay_enable(overlay_allowed),
    .left_enable(overlay_left_enable), .right_enable(overlay_right_enable),
    .up_enable(overlay_up_enable), .warning_enable(overlay_warning_enable),
    .left_color(overlay_left_color), .right_color(overlay_right_color), .up_color(overlay_up_color),
    .left_blink(left_blink), .right_blink(right_blink), .up_blink(up_blink),
    .warning_blink(warning_blink), .blink_visible(blink_visible),
    .de(overlay_de), .hs(overlay_hs), .vs(overlay_vs), .data(overlay_rgb)
);

// PLL输出相对恢复时钟移相半周期；先在pixel_clk域捕获，后级只走完整周期路径。
always @(posedge pixel_clk or negedge video_rst_n) begin
    if (!video_rst_n) begin
        overlay_rgb_px <= 24'd0;
        overlay_hs_px  <= 1'b0;
        overlay_vs_px  <= 1'b0;
        overlay_de_px  <= 1'b0;
    end else begin
        overlay_rgb_px <= overlay_rgb;
        overlay_hs_px  <= overlay_hs;
        overlay_vs_px  <= overlay_vs;
        overlay_de_px  <= overlay_de;
    end
end

// DDR完整帧标志为保持电平，双触发同步到HDMI发送像素域后用于右下格显示门控。
always @(posedge pixel_clk or negedge rst_n) begin
    if (!rst_n) begin
        eth_frame_ready_q1 <= 1'b0;
        eth_frame_ready_q2 <= 1'b0;
    end else begin
        eth_frame_ready_q1 <= eth_frame_ready;
        eth_frame_ready_q2 <= eth_frame_ready_q1;
    end
end

// 双目OV5640摄像头1，物理接口由J3迁移到J4。
ov5640_dri u_ov5640_dri_1(
    .clk               (clk_50m         ),
    .rst_n             (rst_n           ),

    .cam_pclk          (cam_pclk_1      ),
    .cam_vsync         (cam_vsync_1     ),
    .cam_href          (cam_href_1      ),
    .cam_data          (cam_data_1      ),
    .cam_rst_n         (cam_rst_n_1     ),
    .cam_pwdn          (cam_pwdn_1      ),
    .cam_scl           (cam_scl_1       ),
    .cam_sda           (cam_sda_1       ),

    .capture_start     (init_calib_complete),
    .cmos_h_pixel      (H_CMOS_DISP_HALF),
    .cmos_v_pixel      (V_CMOS_DISP     ),
    .total_h_pixel     (TOTAL_H_PIXEL   ),
    .total_v_pixel     (TOTAL_V_PIXEL   ),
    .cmos_frame_vsync  (cmos_frame_vsync_1),
    .cmos_frame_href   (),
    .cmos_frame_valid  (cmos_frame_valid_1),
    .cmos_frame_data   (wr_data_1       )
);   

// 双目OV5640摄像头2，物理接口由J3迁移到J4。
ov5640_dri u_ov5640_dri_2(
    .clk               (clk_50m),
    .rst_n             (rst_n),

    .cam_pclk          (cam_pclk_2      ),
    .cam_vsync         (cam_vsync_2     ),
    .cam_href          (cam_href_2      ),
    .cam_data          (cam_data_2      ),
    .cam_rst_n         (cam_rst_n_2     ),
    .cam_pwdn          (cam_pwdn_2      ),
    .cam_scl           (cam_scl_2       ),
    .cam_sda           (cam_sda_2       ),
    
    .capture_start     (init_calib_complete),
    .cmos_h_pixel      (H_CMOS_DISP_HALF),
    .cmos_v_pixel      (V_CMOS_DISP     ),
    .total_h_pixel     (TOTAL_H_PIXEL   ),
    .total_v_pixel     (TOTAL_V_PIXEL   ),
    .cmos_frame_vsync  (cmos_frame_vsync_2),
    .cmos_frame_href   (),
    .cmos_frame_valid  (cmos_frame_valid_2),
    .cmos_frame_data   (wr_data_2       )
);    

// 单目OV5640沿用参考工程J3接口，输出640x360 RGB565至DDR第三路。
ov5640_dri u_ov5640_dri_3(
    .clk               (clk_50m),
    .rst_n             (rst_n),

    .cam_pclk          (cam_pclk_3),
    .cam_vsync         (cam_vsync_3),
    .cam_href          (cam_href_3),
    .cam_data          (cam_data_3),
    .cam_rst_n         (cam_rst_n_3),
    .cam_pwdn          (cam_pwdn_3),
    .cam_scl           (cam_scl_3),
    .cam_sda           (cam_sda_3),

    .capture_start     (init_calib_complete),
    .cmos_h_pixel      (H_CMOS_DISP_HALF),
    .cmos_v_pixel      (V_CMOS_DISP),
    .total_h_pixel     (TOTAL_H_PIXEL),
    .total_v_pixel     (TOTAL_V_PIXEL),
    .cmos_frame_vsync  (cmos_frame_vsync_3),
    .cmos_frame_href   (),
    .cmos_frame_valid  (cmos_frame_valid_3),
    .cmos_frame_data   (wr_data_3)
);

//DDR3
ddr3_ctrl_top u_ddr3_ctrl_top(
    .clk              (clk_50m          ),
    // DDR读FIFO与显示排版统一使用PLL像素时钟，避免高扇出的输入时钟跨区驱动块RAM。
    .rd_clk           (pixel_clk        ),
    .rst_n            (rst_n            ),
    .ddr3_init_done   (init_calib_complete),
    //用户接口
    .wd_en_1          (cmos_frame_valid_1),
    .wd_en_2          (cmos_frame_valid_2),
    .wd_data_1        (wr_data_1        ),
    .wd_data_2        (wr_data_2        ),
    .wd_clk_1         (cam_pclk_1        ),
    .wd_clk_2         (cam_pclk_2        ),
    .wd_load_1        (cmos_frame_vsync_1),
    .wd_load_2        (cmos_frame_vsync_2),
    // BACK_LEFT两路输入分别使用自己的时钟域门控，DDR侧再择一读FIFO。
    .wd_en_3          (cmos_frame_valid_3 && !back_left_hdmi_sync_cam3),
    .wd_data_3        (wr_data_3        ),
    .wd_clk_3         (cam_pclk_3       ),
    .wd_load_3        (cmos_frame_vsync_3),
    .wd_en_5          (hdmi_back_left_valid && back_left_hdmi_sync_hdmi),
    .wd_data_5        (hdmi_back_left_data),
    .wd_clk_5         (hdmi_clk_in       ),
    .wd_load_5        (hdmi_back_left_frame),
    .back_left_source_req(back_left_hdmi_req),
    .wd_en_4          (eth_ddr_valid),
    .wd_data_4        (eth_ddr_data),
    .wd_clk_4         (gmii_rx_clk),

    .rd_en            (rdata_req        ),
    .rd_data          (rddata           ),
    .eth_frame_ready  (eth_frame_ready  ),
    .rd_load          (video_vs         ),
    .ddr3_pingpang_en (1'b1             ),
    .rd_lower_mode    (pixel_ypos > V_CMOS_DISP),
    .rd_fullscreen    (zoom_fullscreen  ),
    .rd_row_active    (pixel_ypos[0]    ),
    .rd_source_sel    (zoom_source_active),
    .h_disp           (H_CMOS_DISP      ),
    //地址接口
    .addr_rd_min      (27'b0            ),
    .rd_burst_len     ({2'b0,H_CMOS_DISP[10:3]}),
    .addr_wd_min      (27'b0            ),
    .wd_burst_len     ({2'b0,H_CMOS_DISP[10:3]}),
    // 鸟瞰图使用已有四路DDR输入，结果帧存放在保留的DDR地址区。
    .bev_display_active(bev_mode_active),
    .bev_rd_en        (bev_rd_en),
    .bev_rd_data      (bev_rd_data),
    .bev_rd_valid     (bev_rd_valid),
    .bev_rd_fault     (bev_rd_fault),
    .bev_frame_ready  (bev_frame_ready),

    //DDR3 IO接口
    .mem_rst_n        (mem_rst_n        ),
    .mem_ck           (mem_ck           ),
    .mem_ck_n         (mem_ck_n         ),
    .mem_cke          (mem_cke          ),
    .mem_ras_n        (mem_ras_n        ),
    .mem_cas_n        (mem_cas_n        ),
    .mem_we_n         (mem_we_n         ),
    .mem_odt          (mem_odt          ),
    .mem_cs_n         (mem_cs_n         ),
    .mem_a            (mem_a            ),
    .mem_ba           (mem_ba           ),
    .mem_dqs          (mem_dqs          ),
    .mem_dqs_n        (mem_dqs_n        ),
    .mem_dq           (mem_dq           ),
    .mem_dm           (mem_dm           )
);

//HDMI顶层模块
hdmi_top u_hdmi_top(
    .stream_clk       (pixel_clk        ),
    .tx_pixel_clk     (pixel_clk        ),
    .tx_pixel_clk_5x  (pixel_clk_5x     ),
    .sys_rst_n        (video_rst_n      ),
    .quad_mode_req    (quad_mode_req    ),
    .bev_mode_req     (bev_mode_req     ),
    .zoom_fullscreen_req(zoom_fullscreen_req),
    .zoom_source_req  (zoom_source_req  ),
    .bev_rgb565       (bev_rd_data      ),
    .bev_rd_valid     (bev_rd_valid     ),
    .bev_rd_fault     (bev_rd_fault     ),
    .bev_frame_ready  (bev_frame_ready  ),
    .ultrasonic_alert_level(ultrasonic_alert_level),
    .hdmi_rgb_in      (overlay_rgb_px   ),
    .hdmi_hs_in       (overlay_hs_px    ),
    .hdmi_vs_in       (overlay_vs_px    ),
    .hdmi_de_in       (overlay_de_px    ),
    //HDMI interface
   .tmds_clk_p       (tmds_clk_p       ),
   .tmds_clk_n       (tmds_clk_n       ),
   .tmds_data_p      (tmds_data_p      ),
   .tmds_data_n      (tmds_data_n      ),
   //user interface
   .rd_data          (rddata           ),
   .eth_frame_valid  (eth_frame_ready_q2),
   .rd_en            (rdata_req        ),
   .video_vs         (video_vs         ),
   .pixel_xpos       (pixel_xpos       ),
   .pixel_ypos       (pixel_ypos       ),
   .zoom_source_active(zoom_source_active),
   .quad_mode_active_out(quad_mode_active),
   .zoom_fullscreen  (zoom_fullscreen  ),
   .bev_mode_active_out(bev_mode_active),
   .bev_rd_en        (bev_rd_en)
);   

endmodule
