//****************************************Copyright (c)***********************************//
//原子哥在线教学平台：www.yuanzige.com
//技术支持：http://www.openedv.com/forum.php
//淘宝店铺：https://zhengdianyuanzi.tmall.com
//关注微信公众平台微信号："正点原子"，免费获取ZYNQ & FPGA & STM32 & LINUX资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2023-2033
//All rights reserved                                  
//----------------------------------------------------------------------------------------
// File name:           ddr3_ctrl_top
// Created by:          正点原子
// Created date:        2023年2月3日14:17:02
// Version:             V1.0
// Descriptions:        ddr3_ctrl_top
//
//----------------------------------------------------------------------------------------
//****************************************************************************************///

module ddr3_ctrl_top(
    input                   clk             ,
    input                   rd_clk          ,
    input                   rst_n           ,
    // 外部数据接口   
    input                   wd_en_1         ,   //摄像头1写使能
    input                   wd_en_2         ,   //摄像头2写使能
    input       [15:0]      wd_data_1       ,   //摄像头1数据
    input       [15:0]      wd_data_2       ,   //摄像头1数据
    input                   wd_clk_1        ,   //摄像头1写时钟
    input                   wd_clk_2        ,   //摄像头2写时钟
    input                   wd_load_1       ,   //摄像头1输入源更新信号
    input                   wd_load_2       ,   //摄像头2输入源更新信号
    input                   wd_en_3         ,   //BACK_LEFT摄像头3 RGB565写使能
    input       [15:0]      wd_data_3       ,   //BACK_LEFT摄像头3 RGB565数据
    input                   wd_clk_3        ,   //摄像头3像素时钟
    input                   wd_load_3       ,   //摄像头3输入帧脉冲
    input                   wd_en_5         ,   //BACK_LEFT HDMI RGB565写使能
    input       [15:0]      wd_data_5       ,   //BACK_LEFT HDMI RGB565数据
    input                   wd_clk_5        ,   //HDMI输入时钟
    input                   wd_load_5       ,   //HDMI输入帧脉冲
    input                   wd_en_4         ,   //以太网RGB565写使能
    input       [15:0]      wd_data_4       ,   //以太网RGB565数据
    input                   wd_clk_4        ,   //RGMII接收时钟

    input                   rd_en           ,   //读使能
    output      [15:0]      rd_data         ,   //读数据
    output                  eth_frame_ready ,   //以太网完整帧已写入DDR
    input                   rd_load         ,   //输出源更新信号
    input                   ddr3_pingpang_en,   //DDR3乒乓操作
    input                   rd_lower_mode   ,   //四宫格下半屏选择HDMI/以太网两路
    input                   rd_fullscreen   ,   //1：指定DDR源按2倍像素放大全屏
    input                   rd_row_active   ,   //全屏放大时当前输出行是否为源图像新行
    input       [1:0]       rd_source_sel   ,   //0:摄像头1，1:摄像头2，2:BACK_LEFT，3:以太网
    input                   back_left_source_req, //0:摄像头3，1:HDMI
    input       [12:0]      h_disp          ,   //摄像头水平分辨率
    input                   bev_display_active,
    input                   bev_rd_en       ,
    output      [15:0]      bev_rd_data     ,
    output                  bev_rd_valid    ,
    output                  bev_rd_fault    ,
    output                  bev_frame_ready ,
    // ddr地址长度控制接口                  
    input       [26:0]      addr_rd_min     ,
    input       [9:0]       rd_burst_len    ,
    input       [26:0]      addr_wd_min     ,
    input       [9:0]       wd_burst_len    ,
    output  reg             ddr3_init_done  ,   //ddr3初始化完成标志
    //DDR控制器接口
    output                  mem_rst_n       ,                       
    output                  mem_ck          ,
    output                  mem_ck_n        ,
    output                  mem_cke         ,
    output                  mem_ras_n       ,
    output                  mem_cas_n       ,
    output                  mem_we_n        , 
    output                  mem_odt         ,
    output                  mem_cs_n        ,
    output 	    [14:0]      mem_a           ,   
    output 	    [2:0]       mem_ba          ,   
    inout 	    [3:0]       mem_dqs         ,
    inout 	    [3:0]       mem_dqs_n       ,
    inout 	    [31:0]      mem_dq          ,
    output 	    [3:0]       mem_dm               
);
//wire define
wire            ui_clk          ;
// DDR调度器使用24位、8字节对齐的字地址；左移3位后正好形成27位AXI字节地址。
wire [23:0]     wd_addr         ;
wire [23:0]     rd_addr         ;
wire [9:0]      wd_len          ;
wire [9:0]      rd_len          ;
wire [255:0]    rd_ddr3_data    ;
wire [255:0]    wd_ddr3_data    ;
wire            wd_fifo_re      ;
wire            rd_fifo_we      ;
wire            rd_finish       ;
wire            wd_finish       ;
wire            rd_req          ;
wire            wd_req          ;
// DDR-backed BEV renderer and display FIFO.
wire            bev_source_req;
wire [23:0]     bev_source_addr;
wire            bev_source_valid;
wire [255:0]    bev_source_data;
wire            bev_write_req;
wire [23:0]     bev_write_addr;
wire [255:0]    bev_write_data;
wire            bev_write_done;
wire [1:0]      bev_source_page0, bev_source_page1;
wire [1:0]      bev_source_page2, bev_source_page3;
wire [3:0]      bev_source_ready;
wire            bev_source_lock;
wire [1:0]      bev_frame_page;
wire [1:0]      bev_display_page_in_use;
wire            bev_fifo_wr_en;
wire [255:0]    bev_fifo_wr_data;
wire [10:0]     bev_fifo_wcount;
wire            bev_fifo_empty;
// BEV mode is generated in the recovered HDMI clock domain.  Synchronize it
// before using it to start the UI-clock-domain renderer.
(* ASYNC_REG = "TRUE" *) reg bev_enable_meta;
(* ASYNC_REG = "TRUE" *) reg bev_enable_sync;
//fifo1
wire            wfifo_rd_en_1   ;
wire [255:0]    wfifo_rd_data_1 ;
wire [10:0]     wfifo_rcount_1  ;
wire            rfifo_wd_en_1   ;
wire [255:0]    rfifo_wd_data_1 ;
wire [10:0]     rfifo_wcount_1  ;
//fifo2
wire            wfifo_rd_en_2   ;
wire [255:0]    wfifo_rd_data_2 ;
wire [10:0]     wfifo_rcount_2  ;
wire            rfifo_wd_en_2   ;
wire [255:0]    rfifo_wd_data_2 ;
wire [10:0]     rfifo_wcount_2  ;
// fifo3：HDMI RGB565
wire            wfifo_rd_en_3   ;
wire [255:0]    wfifo_rd_data_3 ;
wire [10:0]     wfifo_rcount_3  ;
wire            rfifo_wd_en_3   ;
wire [255:0]    rfifo_wd_data_3 ;
wire [10:0]     rfifo_wcount_3  ;
// BACK_LEFT备用HDMI写FIFO，与摄像头3共用DDR第三路地址空间
wire            wfifo_rd_en_5   ;
wire [255:0]    wfifo_rd_data_5 ;
wire [10:0]     wfifo_rcount_5  ;
// fifo4：以太网RGB565
wire            wfifo_rd_en_4   ;
wire [255:0]    wfifo_rd_data_4 ;
wire [10:0]     wfifo_rcount_4  ;
wire            rfifo_wd_en_4   ;
wire [255:0]    rfifo_wd_data_4 ;
wire [10:0]     rfifo_wcount_4  ;
// Write Address                
wire [3:0]     axi_awid         ;
wire [26:0]    axi_awaddr       ;
wire [3:0]     axi_awlen        ;
wire           axi_awvalid      ;
wire           axi_awready      ;
// write data
wire [255:0]   axi_wdata        ;
wire [31:0]    axi_wstrb        ;
wire           axi_wlast        ;
wire           axi_wready       ;
// read address                 
wire [3:0]     axi_arid         ;
wire [26:0]    axi_araddr       ;
wire [3:0]     axi_arlen        ;
wire           axi_arvalid      ;
wire           axi_arready      ;
wire           ddr_init_done    ;
// read data                    
wire [3:0]     axi_rid          ;
wire [255:0]   axi_rdata        ;
wire           axi_rlast        ;
wire           axi_rvalid       ;
wire [13:0]    mem_a_ip         ;

reg            ddr3_init_done_c0;
reg            ddr3_init_done_c1;

// 当前DDR3 IP配置使用14位行地址；板级接口多出的A14固定为0。
assign mem_a = {1'b0, mem_a_ip};

//*****************************************************
//**                    main code
//*****************************************************

// 稳定初始化信号
always@ (posedge ui_clk or negedge rst_n ) begin
    if(!rst_n)begin
        ddr3_init_done_c0 <= 1'b0;
        ddr3_init_done_c1 <= 1'b0;
    end
	else begin
        ddr3_init_done_c0 <= ddr_init_done;
        ddr3_init_done_c1 <= ddr3_init_done_c0;
    end
end

always @(posedge ui_clk or negedge rst_n) begin
    if (!rst_n) begin
        bev_enable_meta <= 1'b0;
        bev_enable_sync  <= 1'b0;
    end else begin
        bev_enable_meta <= bev_display_active;
        bev_enable_sync  <= bev_enable_meta;
    end
end
always@(posedge ui_clk or negedge rst_n)begin
    if(!rst_n)
        ddr3_init_done <= 1'b0;
    else if(ddr3_init_done_c0 && ~ddr3_init_done_c1)	
		ddr3_init_done <= 1'b1;
    else
        ddr3_init_done <= ddr3_init_done;
end

// DDR3读写控制模块 
ddr3_rw_ctrl u_ddr3_rw_ctrl(
    .rst_n            (rst_n            ),
    .clk              (ui_clk           ),
    .ddr3_init_done   (ddr3_init_done   ),
    .addr_rd_min      (addr_rd_min      ),
    .rd_burst_len     (rd_burst_len     ),
    .addr_wd_min      (addr_wd_min      ),
    .wd_burst_len     (wd_burst_len     ),
    .ddr3_pingpang_en (ddr3_pingpang_en ),

    .rd_load          (rd_load          ),
    .wd_load_1        (wd_load_1        ),
    .wd_load_2        (wd_load_2        ),
    .wd_load_3        (wd_load_3        ),
    .wd_load_5        (wd_load_5        ),
    .back_left_source_req(back_left_source_req),

    .rfifo_wcount_1   (rfifo_wcount_1   ),
    .wfifo_rcount_1   (wfifo_rcount_1   ),

    .wfifo_rd_en_1    (wfifo_rd_en_1    ),
    .wfifo_rd_data_1  (wfifo_rd_data_1  ),
    .rfifo_wd_en_1    (rfifo_wd_en_1    ),
    .rfifo_wd_data_1  (rfifo_wd_data_1  ),

    .rfifo_wcount_2   (rfifo_wcount_2   ),
    .wfifo_rcount_2   (wfifo_rcount_2   ),

    .wfifo_rd_en_2    (wfifo_rd_en_2    ),
    .wfifo_rd_data_2  (wfifo_rd_data_2  ),
    .rfifo_wd_en_2    (rfifo_wd_en_2    ),
    .rfifo_wd_data_2  (rfifo_wd_data_2  ),

    .rfifo_wcount_3   (rfifo_wcount_3   ),
    .wfifo_rcount_3   (wfifo_rcount_3   ),
    .wfifo_rd_en_3    (wfifo_rd_en_3    ),
    .wfifo_rd_data_3  (wfifo_rd_data_3  ),
    .rfifo_wd_en_3    (rfifo_wd_en_3    ),
    .rfifo_wd_data_3  (rfifo_wd_data_3  ),
    .wfifo_rcount_5   (wfifo_rcount_5   ),
    .wfifo_rd_data_5  (wfifo_rd_data_5  ),
    .wfifo_rd_en_5    (wfifo_rd_en_5    ),

    .rfifo_wcount_4   (rfifo_wcount_4   ),
    .wfifo_rcount_4   (wfifo_rcount_4   ),
    .wfifo_rd_en_4    (wfifo_rd_en_4    ),
    .wfifo_rd_data_4  (wfifo_rd_data_4  ),
    .rfifo_wd_en_4    (rfifo_wd_en_4    ),
    .rfifo_wd_data_4  (rfifo_wd_data_4  ),

    .wd_fifo_re       (wd_fifo_re       ),
    .wd_finish        (wd_finish        ),
    .wd_req           (wd_req           ),
    .wd_addr          (wd_addr          ),
    .wd_len           (wd_len           ),

    .rd_fifo_we       (rd_fifo_we       ),
    .rd_finish        (rd_finish        ),
    .rd_req           (rd_req           ),
    .rd_addr          (rd_addr          ),
    .rd_len           (rd_len           ),

    .wd_ddr3_data     (wd_ddr3_data     ),
    .rd_ddr3_data     (rd_ddr3_data     ),
    .eth_frame_ready  (eth_frame_ready   ),

    .bev_source_req   (bev_source_req    ),
    .bev_source_addr  (bev_source_addr   ),
    .bev_source_valid (bev_source_valid  ),
    .bev_source_data  (bev_source_data   ),
    .bev_write_req    (bev_write_req     ),
    .bev_write_addr   (bev_write_addr    ),
    .bev_write_data   (bev_write_data    ),
    .bev_write_done   (bev_write_done    ),
    .bev_source_page0 (bev_source_page0  ),
    .bev_source_page1 (bev_source_page1  ),
    .bev_source_page2 (bev_source_page2  ),
    .bev_source_page3 (bev_source_page3  ),
    .bev_source_ready (bev_source_ready  ),
    .bev_source_lock  (bev_source_lock   ),
    .bev_display_active(bev_display_active),
    .bev_frame_ready  (bev_frame_ready   ),
    .bev_frame_page   (bev_frame_page    ),
    .bev_fifo_wcount  (bev_fifo_wcount   ),
    .bev_fifo_wr_en   (bev_fifo_wr_en    ),
    .bev_fifo_wr_data (bev_fifo_wr_data  ),
    .bev_display_page_in_use(bev_display_page_in_use)
);

bev_ddr_renderer u_bev_ddr_renderer(
    .clk               (ui_clk),
    .rst_n             (rst_n),
    // Do not render BEV frames in the normal HDMI/four-way modes.  The
    // dynamic blind-area path performs a DDR source read for every projected
    // donor pixel; running it continuously would consume the same DDR
    // scheduler used by the ordinary display FIFOs and can prevent a clean
    // video start.  Start the renderer only after the HDMI-domain mode
    // request has reached the DDR/UI clock domain.
    .enable            (ddr3_init_done && bev_enable_sync),
    .source_page0      (bev_source_page0),
    .source_page1      (bev_source_page1),
    .source_page2      (bev_source_page2),
    .source_page3      (bev_source_page3),
    .source_ready      (bev_source_ready),
    .display_in_use_page(bev_display_page_in_use),
    .source_req        (bev_source_req),
    .source_addr       (bev_source_addr),
    .source_data_valid (bev_source_valid),
    .source_data       (bev_source_data),
    .write_req         (bev_write_req),
    .write_addr        (bev_write_addr),
    .write_data        (bev_write_data),
    .write_done        (bev_write_done),
    .display_page      (bev_frame_page),
    .frame_ready       (bev_frame_ready),
    .source_lock       (bev_source_lock)
);

bev_read_fifo u_bev_read_fifo(
    .rst_n       (rst_n),
    .ui_clk      (ui_clk),
    .pixel_clk   (rd_clk),
    .frame_reset (rd_load),
    .wr_en       (bev_fifo_wr_en),
    .wr_data     (bev_fifo_wr_data),
    .wr_count    (bev_fifo_wcount),
    .rd_en       (bev_rd_en),
    .rd_data     (bev_rd_data),
    .rd_empty    (bev_fifo_empty),
    .rd_valid    (bev_rd_valid),
    .rd_fault    (bev_rd_fault)
);

// axi 控制模块
aq_axi_master u_aq_axi_master
	(
	.rst_n                  (rst_n  && ddr3_init_done),
	.clk                    (ui_clk        ),
    // Write address channel
	.m_axi_awid             (axi_awid      ),                             
	.m_axi_awaddr           (axi_awaddr    ),                             
	.m_axi_awlen            (axi_awlen     ),                                                         
	.m_axi_awvalid          (axi_awvalid   ),                             
	.m_axi_awready          (axi_awready   ),
    // Write data channel
	.m_axi_wdata            (axi_wdata     ),//O                             
	.m_axi_wstrb            (axi_wstrb     ),                             
	.m_axi_wlast            (axi_wlast     ),                                                         
	.m_axi_wready           (axi_wready    ),
    // Read address channel
	.m_axi_arid             (axi_arid      ),                             
	.m_axi_araddr           (axi_araddr    ),                             
	.m_axi_arlen            (axi_arlen     ),                                                     
	.m_axi_arvalid          (axi_arvalid   ),                             
	.m_axi_arready          (axi_arready   ),
    // Read data channel
	.m_axi_rdata            (axi_rdata     ), //I                                                        
	.m_axi_rlast            (axi_rlast     ),                                                          
	.m_axi_rvalid           (axi_rvalid    ),
    // User control interface
	.wr_start               (wd_req        ),      
	.wr_adrs                ({wd_addr,3'd0}),         
	.wr_len                 ({wd_len[7:0],3'd0}),
	.wr_ready               ( ),         
	.wr_fifo_re             (wd_fifo_re     ),         
	.wr_fifo_empty          (1'b0          ),         
	.wr_fifo_aempty         (1'b0          ),
	.wr_fifo_data           (wd_ddr3_data   ),//I
	.wr_done                (wd_finish     ),
	.rd_start               (rd_req        ),	  
	.rd_adrs                ({rd_addr,3'd0}),
	.rd_len                 ({rd_len[7:0],3'd0}),
	.rd_ready               ( ),
	.rd_fifo_we             (rd_fifo_we     ),
	.rd_fifo_full           (1'b0          ),
	.rd_fifo_afull          (1'b0          ),
	.rd_fifo_data           (rd_ddr3_data   ),//O
	.rd_done                (rd_finish     )
);

// DDR3 IP core
ddr3_ip u_ddr3_ip (
    .ref_clk               	(clk),                  // input
    .resetn                	(rst_n),                // input
    .core_clk              	(ui_clk),               // output
    .pll_lock              	( ),                    // output
    .phy_pll_lock          	( ),                    // output
    .gpll_lock             	( ),                    // output
    .rst_gpll_lock         	( ),                    // output
    .ddrphy_cpd_lock       	( ),                    // output
    .ddr_init_done         	(ddr_init_done),        // output
	
    .axi_awaddr            	(axi_awaddr),           // input [26:0]
    .axi_awuser_ap         	(1'b0),                 // input
    .axi_awuser_id         	(4'd0),                 // input [3:0]
    .axi_awlen             	(axi_awlen),            // input [3:0]
    .axi_awready           	(axi_awready),          // output
    .axi_awvalid           	(axi_awvalid),          // input
		
    .axi_wdata             	(axi_wdata),            // input [255:0]
    .axi_wstrb             	(axi_wstrb),            // input [31:0]
    .axi_wready            	(axi_wready),           // output
    .axi_wusero_id         	( ),                    // output [3:0]
    .axi_wusero_last       	(axi_wlast),            // output
    
    .axi_araddr            	(axi_araddr),           // input [26:0]
    .axi_aruser_ap         	(1'b0),                 // input
    .axi_aruser_id         	(4'd0),                 // input [3:0]
    .axi_arlen             	(axi_arlen),            // input [3:0]
    .axi_arready           	(axi_arready),          // output
    .axi_arvalid           	(axi_arvalid),          // input
		
    .axi_rdata             	(axi_rdata),            // output [255:0]
    .axi_rid               	( ),                    // output [3:0]
    .axi_rlast             	(axi_rlast),            // output
    .axi_rvalid            	(axi_rvalid),           // output
		
    .apb_clk               	(1'b0),                 // input
    .apb_rst_n             	(1'b0),                 // input
    .apb_sel               	(1'b0),                 // input
    .apb_enable            	(1'b0),                 // input
    .apb_addr              	(8'd0),                 // input [7:0]
    .apb_write             	(1'b0),                 // input
    .apb_ready             	( ),                    // output
    .apb_wdata             	(16'd0),                // input [15:0]
    .apb_rdata             	( ),                    // output [15:0]
		
    .mem_cs_n              	(mem_cs_n),             // output
    .mem_rst_n             	(mem_rst_n),            // output
    .mem_ck                	(mem_ck),               // output
    .mem_ck_n              	(mem_ck_n),             // output
    .mem_cke               	(mem_cke),              // output
    .mem_ras_n             	(mem_ras_n),            // output
    .mem_cas_n             	(mem_cas_n),            // output
    .mem_we_n              	(mem_we_n),             // output
    .mem_odt               	(mem_odt),              // output
    .mem_a                 	(mem_a_ip),             // output [13:0]
    .mem_ba                	(mem_ba),               // output [2:0]
    .mem_dqs               	(mem_dqs),              // inout [3:0]
    .mem_dqs_n             	(mem_dqs_n),            // inout [3:0]
    .mem_dq                	(mem_dq),               // inout [31:0]
    .mem_dm                	(mem_dm),               // output [3:0]
  
    //.dfi_address(0),                              // input [59:0]
    //.dfi_bank(0),                                 // input [11:0]
    //.dfi_cs_n(0),                                 // input [3:0]
    //.dfi_cas_n(0),                                // input [3:0]
    //.dfi_ras_n(0),                                // input [3:0]
    //.dfi_we_n(0),                                 // input [3:0]
    //.dfi_cke(0),                                  // input [3:0]
    //.dfi_odt(0),                                  // input [3:0]
    //.dfi_wrdata_en(0),                            // input [3:0]
    //.dfi_wrdata(0),                               // input [255:0]
    //.dfi_wrdata_mask(0),                          // input [31:0]
    //.dfi_rddata( ),                               // output [255:0]
    //.dfi_rddata_valid( ),                         // output
    //.dfi_reset_n(1),                              // input
    //.dfi_phyupd_req( ),                           // output
    //.dfi_phyupd_ack(0),                           // input
    //.dfi_error( ),                                // output
    .dbg_gate_start         	(1'b0),            	// input
    .dbg_cpd_start          	(1'b1),            	// input
    .dbg_ddrphy_rst_n       	(1'b1),            	// input
    .dbg_gpll_scan_rst      	(1'b0),            	// input
    .samp_position_dyn_adj  	(1'b0),            	// input
    .init_samp_position_even	(32'd0),           	// input [31:0]
    .init_samp_position_odd 	(32'd0),           	// input [31:0]
    .wrcal_position_dyn_adj 	(1'b0),            	// input
    .init_wrcal_position    	(32'd0),           	// input [31:0]
    .force_read_clk_ctrl    	(1'b0),            	// input
    .init_slip_step         	(16'd0),           	// input [15:0]
    .init_read_clk_ctrl     	(12'd0),           	// input [11:0]
    .debug_calib_ctrl       	( ),               	// output [33:0]
    .dbg_slice_state        	( ),               	// output [87:0]
    .debug_data             	( ),               	// output [275:0]
    .dbg_dll_upd_state      	( ),               	// output [1:0]
    .debug_gpll_dps_phase       ( ),               	// output [8:0]
    .dbg_rst_dps_state          ( ),               	// output [2:0]
    .dbg_tran_err_rst_cnt       ( ),               	// output [5:0]
    .dbg_ddrphy_init_fail       ( ),               	// output
    .debug_cpd_offset_adj       (1'b0),            	// input
    .debug_cpd_offset_dir       (1'b0),            	// input
    .debug_cpd_offset           (10'd0),           	// input [9:0]
    .debug_dps_cnt_dir0         ( ),               	// output [9:0]
    .debug_dps_cnt_dir1         ( ),               	// output [9:0]
    .ck_dly_en                  (1'b1),            	// input
    .init_ck_dly_step           (8'd0),            	// input [7:0]
    .ck_dly_set_bin             ( ),               	// output [7:0]
    .align_error                ( ),               	// output
    .debug_rst_state            ( ),               	// output [3:0]
    .debug_cpd_state            ( )                	// output [3:0]
);

// FIFO控制模块
ddr3_fifo_ctrl_top u_ddr3_fifo_ctrl_top(
    .rst_n           (rst_n           ),
    .rd_clk          (rd_clk          ),
    .ui_clk          (ui_clk          ),
    //通道一
    .wd_clk_1        (wd_clk_1        ),
    .wd_en_1         (wd_en_1         ),
    .wd_data_1       (wd_data_1       ),
    .wd_load_1       (wd_load_1       ),

    .rfifo_wd_en_1   (rfifo_wd_en_1   ),
    .rfifo_wd_data_1 (rfifo_wd_data_1 ),
    .wfifo_rd_en_1   (wfifo_rd_en_1   ),
    .wfifo_rd_data_1 (wfifo_rd_data_1 ),
    .wfifo_rcount_1  (wfifo_rcount_1  ),
    .rfifo_wcount_1  (rfifo_wcount_1  ),
    //通道二
    .wd_clk_2        (wd_clk_2        ),
    .wd_clk_3        (wd_clk_3        ),
    .wd_en_2         (wd_en_2         ),
    .wd_data_2       (wd_data_2       ),
    .wd_load_2       (wd_load_2       ),
    .wd_load_3       (wd_load_3       ),

    .rfifo_wd_en_2   (rfifo_wd_en_2   ),
    .rfifo_wd_data_2 (rfifo_wd_data_2 ),
    .wfifo_rd_en_2   (wfifo_rd_en_2   ),
    .wfifo_rd_data_2 (wfifo_rd_data_2 ),
    .wfifo_rcount_2  (wfifo_rcount_2  ),
    .rfifo_wcount_2  (rfifo_wcount_2  ),
    .wd_en_3         (wd_en_3         ),
    .wd_data_3       (wd_data_3       ),
    .rfifo_wd_en_3   (rfifo_wd_en_3   ),
    .rfifo_wd_data_3 (rfifo_wd_data_3 ),
    .wfifo_rd_en_3   (wfifo_rd_en_3   ),
    .wfifo_rd_data_3 (wfifo_rd_data_3 ),
    .wfifo_rcount_3  (wfifo_rcount_3  ),
    .rfifo_wcount_3  (rfifo_wcount_3  ),
    // BACK_LEFT备用HDMI写FIFO
    .wd_clk_5        (wd_clk_5        ),
    .wd_en_5         (wd_en_5         ),
    .wd_data_5       (wd_data_5       ),
    .wd_load_5       (wd_load_5       ),
    .wfifo_rd_en_5   (wfifo_rd_en_5   ),
    .wfifo_rd_data_5 (wfifo_rd_data_5 ),
    .wfifo_rcount_5  (wfifo_rcount_5  ),
    //通道四：以太网
    .wd_clk_4        (wd_clk_4        ),
    .wd_en_4         (wd_en_4         ),
    .wd_data_4       (wd_data_4       ),
    .rfifo_wd_en_4   (rfifo_wd_en_4   ),
    .rfifo_wd_data_4 (rfifo_wd_data_4 ),
    .wfifo_rd_en_4   (wfifo_rd_en_4   ),
    .wfifo_rd_data_4 (wfifo_rd_data_4 ),
    .wfifo_rcount_4  (wfifo_rcount_4  ),
    .rfifo_wcount_4  (rfifo_wcount_4  ),
    
    .h_disp          (h_disp          ),
    .rd_load         (rd_load         ),
    .rd_en           (rd_en           ),
    .rd_lower_mode   (rd_lower_mode   ),
    .rd_fullscreen   (rd_fullscreen   ),
    .rd_row_active   (rd_row_active   ),
    .rd_source_sel   (rd_source_sel   ),
    .rd_data         (rd_data         )
);

endmodule
