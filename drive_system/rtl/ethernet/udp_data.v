//****************************************Copyright (c)***********************************//
//原子哥在线教学平台：www.yuanzige.com
//技术支持：www.openedv.com
//淘宝店铺：http://openedv.taobao.com
//关注微信公众平台微信号："正点原子"，免费获取ZYNQ & FPGA & STM32 & LINUX资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2018-2028
//All rights reserved
//----------------------------------------------------------------------------------------
// File name:           udp_data
// Last modified Date:  2020/05/04 9:19:08
// Last Version:        V1.0
// Descriptions:        udp数据处理模块
//                      
//----------------------------------------------------------------------------------------
// Created by:          正点原子
// Created date:        2019/05/04 9:19:08
// Version:             V1.0
// Descriptions:        The original version
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module udp_data(
    input               clk,                    //时钟信号
    input               rst_n,                  //复位信号，低电平有效
    
    input               rec_en,                 //UDP接收的数据使能信号
    input       [7:0]   rec_data,               //以太网图片像素16数据
    output              picture_data_vld,       //以太网640x360图片像素数据有效信号
    output              frame_valid,            //收到首个完整像素后的保持标志
    output reg  [15:0]  picture_data            //以太网图片像素16数据
    );
  
//reg define
reg rec_en_cnt  ;  
reg picture_data_vld_r;
reg frame_valid_r;
reg [17:0] pixel_count;

//wire define
wire rec_en_t   ;
//*****************************************************
//**                    main code
//*****************************************************
//产生以太网16位图片像素数据有效信号
assign rec_en_t = (rec_en & rec_en_cnt)? 1'b1:1'b0;
// Delay the write strobe by one clock. The low byte is captured with a
// nonblocking assignment, so the complete RGB565 word is only stable after
// that edge.
assign picture_data_vld = picture_data_vld_r;
assign frame_valid = frame_valid_r;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin  
        rec_en_cnt <= 0;
        picture_data_vld_r <= 1'b0;
        frame_valid_r <= 1'b0;
        pixel_count <= 18'd0;
    end
    else begin
        picture_data_vld_r <= rec_en_t;
        // UDP包之间允许存在空隙，字节奇偶状态和像素计数均连续保持。
        // 收满640x360后再开放显示，避免首帧尚未写完就显示半帧。
        if (picture_data_vld_r) begin
            if (pixel_count == 18'd230399) begin
                pixel_count <= 18'd0;
                frame_valid_r <= 1'b1;
            end else
                pixel_count <= pixel_count + 1'b1;
        end
        if(rec_en)
            rec_en_cnt <= rec_en_cnt + 1'b1;
        else
            rec_en_cnt <= rec_en_cnt;
    end
end
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin  
        picture_data <= 0;
    end
    else if(rec_en & rec_en_cnt)
        picture_data[7:0] <= rec_data ;
    else if(rec_en) 
        picture_data[15:8] <= rec_data ;
    else
        picture_data <= picture_data ;
end
    
endmodule
