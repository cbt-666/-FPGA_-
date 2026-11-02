//****************************************Copyright (c)***********************************//
//ԭ�Ӹ����߽�ѧƽ̨��www.yuanzige.com
//����֧�֣�www.openedv.com
//�Ա����̣�http://openedv.taobao.com 
//��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡZYNQ & FPGA & STM32 & LINUX���ϡ�
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2018-2028
//All rights reserved                                  
//----------------------------------------------------------------------------------------
// File name:           eth_udp_loop
// Last modified Date:  2020/2/18 9:20:14
// Last Version:        V1.0
// Descriptions:        ��̫��ͨ��UDPͨ�Ż��ض���ģ��
//----------------------------------------------------------------------------------------
// Created by:          ����ԭ��
// Created date:        2020/2/18 9:20:14
// Version:             V1.0
// Descriptions:        The original version
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module eth_udp_loop(
    input              sys_clk   , //ϵͳʱ��
    input              sys_rst_n , //ϵͳ��λ�źţ��͵�ƽ��Ч 
    //��̫��RGMII�ӿ�   
    input              eth_rxc   , //RGMII��������ʱ��
    input              eth_rx_ctl, //RGMII����������Ч�ź�
    input       [3:0]  eth_rxd   , //RGMII��������
    output             eth_txc   , //RGMII��������ʱ��    
    output             eth_tx_ctl, //RGMII���������Ч�ź�
    output      [3:0]  eth_txd   , //RGMII�������          
    input              eth_rst_n   , //与GE0共用的PHY复位，低电平有效
    // HDMI 处理后图像流（像素时钟域），由 send_image 跨域并发送。
    input              image_pclk,
    input              image_hs,
    input              image_vs,
    input              image_de,
    input       [23:0] image_rgb,
    input       [1:0]  image_mode,
    // 两字节小端命令：先收[7:0]，再收[15:8]。仅在整包长度为2且
    // 保留位、图像模式均合法时更新，command_toggle作为跨时钟域事件标志。
    output     [15:0]  command_word,
    output             command_toggle,
    output             single_arrow_active
    );

//parameter define
//������MAC��ַ 00-11-22-33-44-55
parameter  BOARD_MAC = 48'h00_11_22_33_44_55;     
//������IP��ַ 192.168.1.10
parameter  BOARD_IP  = {8'd192,8'd168,8'd1,8'd10};  
//Ŀ��MAC��ַ ff_ff_ff_ff_ff_ff
parameter  DES_MAC   = 48'hff_ff_ff_ff_ff_ff;    
//Ŀ��IP��ַ 192.168.1.102     
parameter  DES_IP    = {8'd192,8'd168,8'd1,8'd102};  

//wire define					  
wire          gmii_rx_clk		  ; //GMII����ʱ��
wire          gmii_rx_dv 		  ; //GMII����������Ч�ź�
wire  [7:0]   gmii_rxd   		  ; //GMII��������
wire          gmii_tx_clk		  ; //GMII����ʱ��
wire          gmii_tx_en 		  ; //GMII��������ʹ���ź�
wire  [7:0]   gmii_txd   		  ; //GMII��������     

wire          arp_gmii_tx_en	  ; //ARP GMII���������Ч�ź� 
wire  [7:0]   arp_gmii_txd  	  ; //ARP GMII�������
wire          arp_rx_done   	  ; //ARP��������ź�
wire          arp_rx_type   	  ; //ARP�������� 0:����  1:Ӧ��
wire  [47:0]  src_mac       	  ; //���յ�Ŀ��MAC��ַ
wire  [31:0]  src_ip        	  ; //���յ�Ŀ��IP��ַ    
wire          arp_tx_en     	  ; //ARP����ʹ���ź�
wire          arp_tx_type   	  ; //ARP�������� 0:����  1:Ӧ��
wire  [47:0]  des_mac       	  ; //���͵�Ŀ��MAC��ַ
wire  [31:0]  des_ip        	  ; //���͵�Ŀ��IP��ַ   
wire          arp_tx_done   	  ; //ARP��������ź�

wire          icmp_gmii_tx_en	  ; //ICMP GMII���������Ч�ź� 
wire  [7:0]   icmp_gmii_txd  	  ; //ICMP GMII�������
wire          icmp_rec_pkt_done   ; //ICMP�������ݽ�������ź�
wire          icmp_rec_en         ; //ICMP���յ�����ʹ���ź�
wire  [ 7:0]  icmp_rec_data       ; //ICMP���յ�����
wire  [15:0]  icmp_rec_byte_num   ; //ICMP���յ���Ч�ֽ��� ��λ:byte 
wire  [15:0]  icmp_tx_byte_num    ; //ICMP���͵���Ч�ֽ��� ��λ:byte 
wire          icmp_tx_done   	  ; //ICMP��������ź�
wire          icmp_tx_req         ; //ICMP�����������ź�
wire  [ 7:0]  icmp_tx_data        ; //ICMP����������
wire          icmp_tx_start_en    ; //ICMP���Ϳ�ʼʹ���ź�

wire          udp_gmii_tx_en	  ; //UDP GMII���������Ч�ź� 
wire  [7:0]   udp_gmii_txd  	  ; //UDP GMII�������
wire          rec_pkt_done  	  ; //UDP�������ݽ�������ź�
wire          udp_rec_en    	  ; //UDP���յ�����ʹ���ź�
wire  [ 7:0]  udp_rec_data  	  ; //UDP���յ�����
wire  [15:0]  rec_byte_num  	  ; //UDP���յ���Ч�ֽ��� ��λ:byte 
wire  [15:0]  tx_byte_num   	  ; //UDP���͵���Ч�ֽ��� ��λ:byte 
wire          udp_tx_done   	  ; //UDP��������ź�
wire          udp_tx_req    	  ; //UDP�����������ź�
wire  [ 7:0]  udp_tx_data   	  ; //UDP����������
wire          tx_start_en   	  ; //UDP���Ϳ�ʼʹ���ź�
								  
wire  [7:0]	  rec_data			  ;
wire		  rec_en			  ;
wire		  tx_req			  ;
wire  [7:0]	  tx_data	    	  ;
wire          image_tx_start_en;
wire [15:0]   image_tx_byte_num;
wire [47:0]   image_des_mac;
wire [31:0]   image_des_ip;
wire [7:0]    cmd_tx_data;

//*****************************************************
//**                    main code
//*****************************************************

assign icmp_tx_start_en = icmp_rec_pkt_done;
assign icmp_tx_byte_num = icmp_rec_byte_num;

// UDP TX 发送处理后的图像；UDP RX 仍用于解析显示命令。
assign tx_start_en = image_tx_start_en;
assign tx_byte_num = image_tx_byte_num;
assign des_mac     = image_des_mac;
assign des_ip      = image_des_ip;
udp_command_decoder u_udp_command_decoder(
    .clk            (gmii_rx_clk),
    .rst_n          (sys_rst_n),
    .rec_pkt_done   (rec_pkt_done),
    .rec_en         (udp_rec_en),
    .rec_data       (udp_rec_data),
    .rec_byte_num   (rec_byte_num),
    .command_word   (command_word),
    .command_toggle (command_toggle),
    .single_arrow_active(single_arrow_active)
);


//GMII�ӿ�תRGMII�ӿ�
gmii_to_rgmii u_gmii_to_rgmii(
    .gmii_rx_clk   (gmii_rx_clk ),
    .gmii_rx_dv    (gmii_rx_dv  ),
    .gmii_rxd      (gmii_rxd    ),
    .gmii_tx_clk   (gmii_tx_clk ),
    .gmii_tx_en    (gmii_tx_en  ),
    .gmii_txd      (gmii_txd    ),
    
    .rgmii_rxc     (eth_rxc     ),
    .rgmii_rx_ctl  (eth_rx_ctl  ),
    .rgmii_rxd     (eth_rxd     ),
    .rgmii_txc     (eth_txc     ),
    .rgmii_tx_ctl  (eth_tx_ctl  ),
    .rgmii_txd     (eth_txd     )
    );

//ARPͨ��
arp                                             
   #(
    .BOARD_MAC     (BOARD_MAC),      //��������
    .BOARD_IP      (BOARD_IP ),
    .DES_MAC       (DES_MAC  ),
    .DES_IP        (DES_IP   )
    )
   u_arp(
    .rst_n         (sys_rst_n  	  ),
                    
    .gmii_rx_clk   (gmii_rx_clk	  ),
    .gmii_rx_dv    (gmii_rx_dv 	  ),
    .gmii_rxd      (gmii_rxd   	  ),
    .gmii_tx_clk   (gmii_tx_clk	  ),
    .gmii_tx_en    (arp_gmii_tx_en),
    .gmii_txd      (arp_gmii_txd  ),
                    
    .arp_rx_done   (arp_rx_done	  ),
    .arp_rx_type   (arp_rx_type	  ),
    .src_mac       (src_mac    	  ),
    .src_ip        (src_ip     	  ),
    .arp_tx_en     (arp_tx_en  	  ),
    .arp_tx_type   (arp_tx_type	  ),
    .des_mac       (des_mac    	  ),
    .des_ip        (des_ip     	  ),
    .tx_done       (arp_tx_done	  )
    );

//ICMPͨ��
icmp                                             
   #(
    .BOARD_MAC     (BOARD_MAC),      //��������
    .BOARD_IP      (BOARD_IP ),
    .DES_MAC       (DES_MAC  ),
    .DES_IP        (DES_IP   )
    )
   u_icmp(
    .rst_n         (sys_rst_n   	 ),  
		 
    .gmii_rx_clk   (gmii_rx_clk 	 ),           
    .gmii_rx_dv    (gmii_rx_dv  	 ),         
    .gmii_rxd      (gmii_rxd    	 ),                   
    .gmii_tx_clk   (gmii_tx_clk 	 ), 
    .gmii_tx_en    (icmp_gmii_tx_en	 ),         
    .gmii_txd      (icmp_gmii_txd	 ),  

    .rec_pkt_done  (icmp_rec_pkt_done),    
    .rec_en        (icmp_rec_en      ), 		  
    .rec_data      (icmp_rec_data    ),   	    
    .rec_byte_num  (icmp_rec_byte_num),      
    .tx_start_en   (icmp_tx_start_en ),        
    .tx_data       (icmp_tx_data     ),       
    .tx_byte_num   (icmp_tx_byte_num ),  
    .des_mac       (des_mac     	 ),
    .des_ip        (des_ip      	 ),    
    .tx_done       (icmp_tx_done	 ),        
    .tx_req        (icmp_tx_req      )        
    ); 

//UDPͨ��
udp                                             
   #(
    .BOARD_MAC     (BOARD_MAC),      //��������
    .BOARD_IP      (BOARD_IP ),
    .DES_MAC       (DES_MAC  ),
    .DES_IP        (DES_IP   )
    )
   u_udp(
    .rst_n         (sys_rst_n   ),  
    
    .gmii_rx_clk   (gmii_rx_clk ),           
    .gmii_rx_dv    (gmii_rx_dv  ),         
    .gmii_rxd      (gmii_rxd    ),                   
    .gmii_tx_clk   (gmii_tx_clk ), 
    .gmii_tx_en    (udp_gmii_tx_en),         
    .gmii_txd      (udp_gmii_txd),  

    .rec_pkt_done  (rec_pkt_done),    
    .rec_en        (udp_rec_en  ),     
    .rec_data      (udp_rec_data),         
    .rec_byte_num  (rec_byte_num),      
    .tx_start_en   (tx_start_en ),        
    .tx_data       (udp_tx_data ),         
    .tx_byte_num   (tx_byte_num ),  
    .des_mac       (des_mac     ),
    .des_ip        (des_ip      ),    
    .tx_done       (udp_tx_done ),        
    .tx_req        (udp_tx_req  )           
    ); 

//�첽FIFO
async_fifo_2048x8b async_fifo_2048x8b_inst (
  .wr_clk(gmii_rx_clk),            // input
  .wr_rst(~sys_rst_n),             // input
  .wr_en(rec_en),                  // input
  .wr_data(rec_data),              // input [7:0]
  .wr_full(),                      // output
  .almost_full(),                  // output
  .rd_clk(gmii_rx_clk),            // input
  .rd_rst(~sys_rst_n),             // input
  .rd_en(tx_req),                  // input
  .rd_data(cmd_tx_data),           // output [7:0]
  .rd_empty(),                     // output
  .almost_empty()                  // output
);

// 在像素时钟域采集处理后图像，在 GMII TX 时钟域提供 UDP 负载。
send_image #(
    .DES_MAC        (DES_MAC),
    .DES_IP         (DES_IP),
    .SEND_FRAME_DIV_MODE0  (3),
    .SEND_FRAME_DIV_MODE12 (2)
) u_send_image(
    .eth_clk        (gmii_tx_clk),
    .pclk           (image_pclk),
    .rst_n          (sys_rst_n),
    .tx_req         (udp_tx_req),
    .tx_done        (udp_tx_done),
    .tx_data        (tx_data),
    .tx_start_en    (image_tx_start_en),
    .tx_byte_num    (image_tx_byte_num),
    .des_mac        (image_des_mac),
    .des_ip         (image_des_ip),
    .hs             (image_hs),
    .vs             (image_vs),
    .de             (image_de),
    .image_mode     (image_mode),
    .image_rgb      (image_rgb)
);

//��̫������ģ��
eth_ctrl u_eth_ctrl(
    .clk            	(gmii_rx_clk	 ),
    .rst_n          	(sys_rst_n		 ),
										 
    .arp_rx_done    	(arp_rx_done   	 ),
    .arp_rx_type    	(arp_rx_type   	 ),
    .arp_tx_en      	(arp_tx_en     	 ),
    .arp_tx_type    	(arp_tx_type   	 ),
    .arp_tx_done    	(arp_tx_done   	 ),
    .arp_gmii_tx_en 	(arp_gmii_tx_en	 ),
    .arp_gmii_txd   	(arp_gmii_txd  	 ),
										 
    .icmp_tx_start_en	(icmp_tx_start_en),
    .icmp_tx_done		(icmp_tx_done	 ),
    .icmp_gmii_tx_en	(icmp_gmii_tx_en ),
    .icmp_gmii_txd		(icmp_gmii_txd	 ),
										 
	.icmp_rec_en       	(icmp_rec_en     ),
	.icmp_rec_data     	(icmp_rec_data   ),
	.icmp_tx_req       	(icmp_tx_req     ),
	.icmp_tx_data      	(icmp_tx_data    ),
										 
    .udp_tx_start_en	(tx_start_en   	 ),
    .udp_tx_done    	(udp_tx_done   	 ),    
    .udp_gmii_tx_en 	(udp_gmii_tx_en	 ),
    .udp_gmii_txd   	(udp_gmii_txd  	 ),
										 
	.udp_rec_data		(udp_rec_data	 ),		
	.udp_rec_en			(udp_rec_en		 ),    
	.udp_tx_req			(udp_tx_req		 ),    
	.udp_tx_data		(udp_tx_data	 ),		
										 
	.rec_data			(rec_data	     ),	
	.rec_en	        	(rec_en	         ),
    .tx_req	        	(tx_req	         ),
	.tx_data	    	(tx_data	     ),
										 
    .gmii_tx_en     	(gmii_tx_en    	 ),
    .gmii_txd       	(gmii_txd      	 )
    );
	
endmodule
