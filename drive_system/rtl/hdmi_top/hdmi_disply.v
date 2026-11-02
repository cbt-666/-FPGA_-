//****************************************Copyright (c)***********************************//
//原子哥在线教学平台：www.yuanzige.com
//技术支持：www.openedv.com
//淘宝店铺：http://openedv.taobao.com
//关注微信公众平台微信号："正点原子"，免费获取ZYNQ & FPGA & STM32 & LINUX资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2018-2028
//All rights reserved
//----------------------------------------------------------------------------------------
// File name:           lcd_disply
// Last modified Date:  2020/05/04 9:19:08
// Last Version:        V1.0
// Descriptions:        lcd显示模块
//                      
//----------------------------------------------------------------------------------------
// Created by:          正点原子
// Created date:        2019/05/04 9:19:08
// Version:             V1.0
// Descriptions:        The original version
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module hdmi_disply #(
    parameter [10:0] IMAGE_HEIGHT = 11'd360
)(
    input              hdmi_clk,     //模块驱动时钟
    input              sys_rst_n,    //复位信号
    //HDMI接口                             
    input      [10:0]  pixel_xpos,   //像素点横坐标
    input      [10:0]  pixel_ypos,   //像素点纵坐标 
    input      [15:0]  rd_data,      //图像像素值
    input              rd_lower_mode,   //下半屏选择DDR第三、四路
    input              eth_frame_valid, //以太网完整帧已就绪
    input      [10:0]  rd_h_pixel,   //输出水平方向分辨率（1280）
    input              zoom_fullscreen,
    input      [1:0]   zoom_source, //DDR放大源：左前、右前、左后、右后
    output reg [15:0]  pixel_data    //像素点数据,
); 
//颜色定义
localparam RED    = 16'b11111_000000_00000;     //字符颜色
localparam BLUE   = 16'b00000_000000_11111;     //字符区域背景色
localparam BLACK  = 16'b00000_000000_00000;     //屏幕背景色  
//reg define                                    
// 四宫格四路输入标签。统一使用128x32字模，避免沿用旧的OV5640编号。
reg [127:0] char0[31:0]; // 左上：FRONT_LEFT
reg [127:0] char1[31:0]; // 右上：FRONT_RIGHT
reg [127:0] char2[31:0]; // 左下：BACK_LEFT
reg [127:0] char3[31:0]; // 右下：BACK_RIGHT

// char0: 四宫格FRONT_LEFT标签（128x32，蓝色字模）
always @(posedge hdmi_clk) begin
    char0[0] <= 128'h00000000000000000000000000000000;
    char0[1] <= 128'h00000000000000000000000000000000;
    char0[2] <= 128'h00000000000000000000000000000000;
    char0[3] <= 128'h00000000000000000000000000000000;
    char0[4] <= 128'h00000000000000000000000000000000;
    char0[5] <= 128'h007FEFF07E3037FE001803FF7FEFFC00;
    char0[6] <= 128'h007FEFF07E3037FE001803FF7FEFFC00;
    char0[7] <= 128'h007FEFF07E3037FE001803FF7FEFFC00;
    char0[8] <= 128'h00600C0D81BC3060001803006000C000;
    char0[9] <= 128'h00600C0D81BC3060001803006000C000;
    char0[10] <= 128'h00600C0D81BC3060001803006000C000;
    char0[11] <= 128'h00600C0D81B33060001803006000C000;
    char0[12] <= 128'h00600C0D81B33060001803006000C000;
    char0[13] <= 128'h00600C0D81B33060001803006000C000;
    char0[14] <= 128'h007F8FF181B0F060001803FC7F80C000;
    char0[15] <= 128'h007F8FF181B0F060001803FC7F80C000;
    char0[16] <= 128'h007F8FF181B0F060001803FC7F80C000;
    char0[17] <= 128'h00600CC181B03060001803006000C000;
    char0[18] <= 128'h00600CC181B03060001803006000C000;
    char0[19] <= 128'h00600CC181B03060001803006000C000;
    char0[20] <= 128'h00600C3181B03060001803006000C000;
    char0[21] <= 128'h00600C3181B03060001803006000C000;
    char0[22] <= 128'h00600C3181B03060001803006000C000;
    char0[23] <= 128'h00600C0C7E303060FFDFFBFF6000C000;
    char0[24] <= 128'h00600C0C7E303060FFDFFBFF6000C000;
    char0[25] <= 128'h00600C0C7E303060FFDFFBFF6000C000;
    char0[26] <= 128'h00000000000000000000000000000000;
    char0[27] <= 128'h00000000000000000000000000000000;
    char0[28] <= 128'h00000000000000000000000000000000;
    char0[29] <= 128'h00000000000000000000000000000000;
    char0[30] <= 128'h00000000000000000000000000000000;
    char0[31] <= 128'h00000000000000000000000000000000;
end

// char1: 四宫格FRONT_RIGHT标签（128x32，蓝色字模）
always @(posedge hdmi_clk) begin
    char1[0] <= 128'h00000000000000000000000000000000;
    char1[1] <= 128'h00000000000000000000000000000000;
    char1[2] <= 128'h00000000000000000000000000000000;
    char1[3] <= 128'h00000000000000000000000000000000;
    char1[4] <= 128'h00000000000000000000000000000000;
    char1[5] <= 128'h0FFDFE0FC606FFC003FC7FE3FD81BFF0;
    char1[6] <= 128'h0FFDFE0FC606FFC003FC7FE3FD81BFF0;
    char1[7] <= 128'h0FFDFE0FC606FFC003FC7FE3FD81BFF0;
    char1[8] <= 128'h0C0181B037860C000303060C01818300;
    char1[9] <= 128'h0C0181B037860C000303060C01818300;
    char1[10] <= 128'h0C0181B037860C000303060C01818300;
    char1[11] <= 128'h0C0181B036660C000303060C01818300;
    char1[12] <= 128'h0C0181B036660C000303060C01818300;
    char1[13] <= 128'h0C0181B036660C000303060C01818300;
    char1[14] <= 128'h0FF1FE30361E0C0003FC060CFDFF8300;
    char1[15] <= 128'h0FF1FE30361E0C0003FC060CFDFF8300;
    char1[16] <= 128'h0FF1FE30361E0C0003FC060CFDFF8300;
    char1[17] <= 128'h0C01983036060C000330060C0D818300;
    char1[18] <= 128'h0C01983036060C000330060C0D818300;
    char1[19] <= 128'h0C01983036060C000330060C0D818300;
    char1[20] <= 128'h0C01863036060C00030C060C0D818300;
    char1[21] <= 128'h0C01863036060C00030C060C0D818300;
    char1[22] <= 128'h0C01863036060C00030C060C0D818300;
    char1[23] <= 128'h0C01818FC6060C1FFB037FE3FD818300;
    char1[24] <= 128'h0C01818FC6060C1FFB037FE3FD818300;
    char1[25] <= 128'h0C01818FC6060C1FFB037FE3FD818300;
    char1[26] <= 128'h00000000000000000000000000000000;
    char1[27] <= 128'h00000000000000000000000000000000;
    char1[28] <= 128'h00000000000000000000000000000000;
    char1[29] <= 128'h00000000000000000000000000000000;
    char1[30] <= 128'h00000000000000000000000000000000;
    char1[31] <= 128'h00000000000000000000000000000000;
end

// char2: 四宫格BACK_LEFT标签（128x32，蓝色字模）
always @(posedge hdmi_clk) begin
    char2[0] <= 128'h00000000000000000000000000000000;
    char2[1] <= 128'h00000000000000000000000000000000;
    char2[2] <= 128'h00000000000000000000000000000000;
    char2[3] <= 128'h00000000000000000000000000000000;
    char2[4] <= 128'h00000000000000000000000000000000;
    char2[5] <= 128'h0001FE0FC1FEC0C003007FEFFDFF8000;
    char2[6] <= 128'h0001FE0FC1FEC0C003007FEFFDFF8000;
    char2[7] <= 128'h0001FE0FC1FEC0C003007FEFFDFF8000;
    char2[8] <= 128'h000181B03600C3000300600C00180000;
    char2[9] <= 128'h000181B03600C3000300600C00180000;
    char2[10] <= 128'h000181B03600C3000300600C00180000;
    char2[11] <= 128'h000181B03600CC000300600C00180000;
    char2[12] <= 128'h000181B03600CC000300600C00180000;
    char2[13] <= 128'h000181B03600CC000300600C00180000;
    char2[14] <= 128'h0001FE3FF600F00003007F8FF0180000;
    char2[15] <= 128'h0001FE3FF600F00003007F8FF0180000;
    char2[16] <= 128'h0001FE3FF600F00003007F8FF0180000;
    char2[17] <= 128'h000181B03600CC000300600C00180000;
    char2[18] <= 128'h000181B03600CC000300600C00180000;
    char2[19] <= 128'h000181B03600CC000300600C00180000;
    char2[20] <= 128'h000181B03600C3000300600C00180000;
    char2[21] <= 128'h000181B03600C3000300600C00180000;
    char2[22] <= 128'h000181B03600C3000300600C00180000;
    char2[23] <= 128'h0001FE3031FEC0DFFBFF7FEC00180000;
    char2[24] <= 128'h0001FE3031FEC0DFFBFF7FEC00180000;
    char2[25] <= 128'h0001FE3031FEC0DFFBFF7FEC00180000;
    char2[26] <= 128'h00000000000000000000000000000000;
    char2[27] <= 128'h00000000000000000000000000000000;
    char2[28] <= 128'h00000000000000000000000000000000;
    char2[29] <= 128'h00000000000000000000000000000000;
    char2[30] <= 128'h00000000000000000000000000000000;
    char2[31] <= 128'h00000000000000000000000000000000;
end

// char3: 四宫格BACK_RIGHT标签（128x32，蓝色字模）
always @(posedge hdmi_clk) begin
    char3[0] <= 128'h00000000000000000000000000000000;
    char3[1] <= 128'h00000000000000000000000000000000;
    char3[2] <= 128'h00000000000000000000000000000000;
    char3[3] <= 128'h00000000000000000000000000000000;
    char3[4] <= 128'h00000000000000000000000000000000;
    char3[5] <= 128'h007F83F07FB03000FF1FF8FF606FFC00;
    char3[6] <= 128'h007F83F07FB03000FF1FF8FF606FFC00;
    char3[7] <= 128'h007F83F07FB03000FF1FF8FF606FFC00;
    char3[8] <= 128'h00606C0D8030C000C0C183006060C000;
    char3[9] <= 128'h00606C0D8030C000C0C183006060C000;
    char3[10] <= 128'h00606C0D8030C000C0C183006060C000;
    char3[11] <= 128'h00606C0D80330000C0C183006060C000;
    char3[12] <= 128'h00606C0D80330000C0C183006060C000;
    char3[13] <= 128'h00606C0D80330000C0C183006060C000;
    char3[14] <= 128'h007F8FFD803C0000FF01833F7FE0C000;
    char3[15] <= 128'h007F8FFD803C0000FF01833F7FE0C000;
    char3[16] <= 128'h007F8FFD803C0000FF01833F7FE0C000;
    char3[17] <= 128'h00606C0D80330000CC0183036060C000;
    char3[18] <= 128'h00606C0D80330000CC0183036060C000;
    char3[19] <= 128'h00606C0D80330000CC0183036060C000;
    char3[20] <= 128'h00606C0D8030C000C30183036060C000;
    char3[21] <= 128'h00606C0D8030C000C30183036060C000;
    char3[22] <= 128'h00606C0D8030C000C30183036060C000;
    char3[23] <= 128'h007F8C0C7FB037FEC0DFF8FF6060C000;
    char3[24] <= 128'h007F8C0C7FB037FEC0DFF8FF6060C000;
    char3[25] <= 128'h007F8C0C7FB037FEC0DFF8FF6060C000;
    char3[26] <= 128'h00000000000000000000000000000000;
    char3[27] <= 128'h00000000000000000000000000000000;
    char3[28] <= 128'h00000000000000000000000000000000;
    char3[29] <= 128'h00000000000000000000000000000000;
    char3[30] <= 128'h00000000000000000000000000000000;
    char3[31] <= 128'h00000000000000000000000000000000;
end

//显示逻辑判断
always@(posedge hdmi_clk) begin
    if (zoom_fullscreen) begin
        // 单路放大时保留该路名称，字号与四宫格一致，放在全屏顶部中央。
        // 以太网未收到首帧时背景为黑色，名称独立显示。
        if ((pixel_xpos >= 1'b1) && (pixel_xpos <= rd_h_pixel) &&
            (pixel_ypos >= 1'b1) && (pixel_ypos <= (IMAGE_HEIGHT << 1))) begin
            pixel_data = ((zoom_source == 2'd3) && !eth_frame_valid) ? BLACK : rd_data;
            if ((pixel_ypos <= 11'd32) &&
                (pixel_xpos >= (rd_h_pixel[10:1]-64)) &&
                (pixel_xpos < (rd_h_pixel[10:1]+64))) begin
                case (zoom_source)
                    2'd0: if (char0[pixel_ypos-11'd1][127-(pixel_xpos-rd_h_pixel[10:1]+64)])
                              pixel_data = BLUE;
                    2'd1: if (char1[pixel_ypos-11'd1][127-(pixel_xpos-rd_h_pixel[10:1]+64)])
                              pixel_data = BLUE;
                    2'd2: if (char2[pixel_ypos-11'd1][127-(pixel_xpos-rd_h_pixel[10:1]+64)])
                              pixel_data = BLUE;
                    2'd3: if (char3[pixel_ypos-11'd1][127-(pixel_xpos-rd_h_pixel[10:1]+64)])
                              pixel_data = BLUE;
                endcase
            end
        end
        else
            pixel_data = BLACK;
    end else if((pixel_ypos >= 1'b1) && (pixel_ypos <= IMAGE_HEIGHT)) begin
      if(pixel_ypos < 33)begin
        //判断像素坐标是否在左半边屏幕中间 
        if(pixel_xpos < (rd_h_pixel[10:2]+64) 
        && pixel_xpos >= (rd_h_pixel[10:2]-64) )begin
            //左上格：FRONT_LEFT
            if(char0[pixel_ypos-11'd1][127-(pixel_xpos-rd_h_pixel[10:2]+64)])
                pixel_data =BLUE;  //字模数组中的"1"显示蓝色
            else                   //字模数组中的"0"显示图像像素值
               pixel_data = rd_data;
        end    //判断像素坐标是否在右半边屏幕中间
        else if(pixel_xpos < (rd_h_pixel[10:2]*3+64)
        && pixel_xpos >= (rd_h_pixel[10:2]*3-64))begin
            //右上格：FRONT_RIGHT
            if(char1[pixel_ypos-11'd1][63-pixel_xpos+(rd_h_pixel[10:2])*3])
                pixel_data =BLUE;  //字模数组中的"1"显示蓝色
            else                    //字模数组中的"0"显示图像像素值 
                pixel_data = rd_data; 
        end
        else            //纵坐标位于字符区域内的字符两边区域显示黑色
            pixel_data = rd_data;
      end
      else                //上半屏字符区以外显示摄像头图像
          pixel_data = rd_data;
    end
    else if((pixel_ypos > IMAGE_HEIGHT) &&
            (pixel_ypos <= (IMAGE_HEIGHT + 11'd32)) &&
            (pixel_xpos >= (rd_h_pixel[10:2]-64)) &&
            (pixel_xpos < (rd_h_pixel[10:2]+64)) &&
            rd_lower_mode) begin
        //左下格：BACK_LEFT
        if(char2[pixel_ypos-IMAGE_HEIGHT-11'd1][127-(pixel_xpos-rd_h_pixel[10:2]+64)])
            pixel_data = BLUE;
        else
            pixel_data = rd_data;
    end
    else if((pixel_ypos > IMAGE_HEIGHT) &&
            (pixel_ypos <= (IMAGE_HEIGHT + 11'd32)) &&
            (pixel_xpos >= (rd_h_pixel[10:2]*3-64)) &&
            (pixel_xpos < (rd_h_pixel[10:2]*3+64)) &&
            rd_lower_mode) begin
        //右下格：BACK_RIGHT，首帧是否就绪只控制背景，不屏蔽标签。
        if(char3[pixel_ypos-IMAGE_HEIGHT-11'd1][63-pixel_xpos+(rd_h_pixel[10:2])*3])
            pixel_data = BLUE;
        else
            pixel_data = eth_frame_valid ? rd_data : BLACK;
    end
    else if((pixel_ypos > IMAGE_HEIGHT) &&
            (pixel_ypos <= (IMAGE_HEIGHT << 1)) &&
            (pixel_xpos >= 1'b1) && (pixel_xpos <= 11'd640) &&
            rd_lower_mode)
        pixel_data = rd_data;
    else if((pixel_ypos > IMAGE_HEIGHT) &&
            (pixel_ypos <= (IMAGE_HEIGHT << 1)) &&
            (pixel_xpos > 11'd640) && (pixel_xpos <= 11'd1280) &&
            rd_lower_mode && eth_frame_valid)
        pixel_data = rd_data;
    else
        pixel_data = BLACK; //以太网尚未收满首帧时右下格背景保持黑色
end

endmodule
