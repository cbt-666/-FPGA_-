module i2c_adv7611_cfg(  
    input                clk      ,  //时钟信号
    input                rst_n    ,  //复位信号，低电平有效
    
    input                i2c_done ,  //I2C寄存器配置完成信号
    output  reg          i2c_exec ,  //I2C触发执行信号   
    output  reg  [23:0]  i2c_data ,  //I2C要配置的地址与数据
    output  reg          init_done   //初始化完成信号
    );

parameter  REG_NUM = 9'd181   ;       //总共需要配置的寄存器个数

reg    [16:0]   start_init_cnt;       //等待延时计数器
reg    [8:0]   init_reg_cnt  ;       //寄存器配置个数计数器

//寄存器延时配置
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        start_init_cnt <= 16'b0;
    else if(start_init_cnt < 17'd100000) begin
        start_init_cnt <= start_init_cnt + 1'b1;                    
    end
end 

//寄存器配置个数计数    
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        init_reg_cnt <= 9'd0;
    else if(i2c_exec)   
        init_reg_cnt <= init_reg_cnt + 1'b1;
end         

//i2c触发执行信号   
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        i2c_exec <= 1'b0;
    else if(start_init_cnt == 17'd99999)
        i2c_exec <= 1'b1;
    else if(i2c_done && (init_reg_cnt < REG_NUM))
        i2c_exec <= 1'b1;  
    else
        i2c_exec <= 1'b0;
end 

//初始化完成信号
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        init_done <= 1'b0;
    else if((init_reg_cnt == REG_NUM) && i2c_done)  
        init_done <= 1'b1;  
end        
   
//配置寄存器地址与数据
always @(*) begin    
        case(init_reg_cnt)    
        	//IO Map       
            9'd0  : i2c_data <= {8'h98,8'hF4, 8'h80}; //CEC MAP
            9'd1  : i2c_data <= {8'h98,8'hF5, 8'h7c}; //INFOFRAME MAP
            9'd2  : i2c_data <= {8'h98,8'hF8, 8'h4c}; //DPLL MAP
            9'd3  : i2c_data <= {8'h98,8'hF9, 8'h64}; //REPEATER MAP(KSV Map I2C address)
            9'd4  : i2c_data <= {8'h98,8'hFA, 8'h6c}; //EDID MAP
            9'd5  : i2c_data <= {8'h98,8'hFB, 8'h68}; //HDMI MAP
            9'd6  : i2c_data <= {8'h98,8'hFD, 8'h44}; //CP MAP
            
            //配合设置分辨率(1280*720)
            // 720p60 free-run/input expectation. These registers do not scale
            // an arbitrary HDMI input; the source mode is selected through EDID.
            9'd7  : i2c_data <= {8'h98,8'h01, 8'h05}; //V_FREQ=60 Hz, PRIM_MODE=HDMI-Comp
            9'd8  : i2c_data <= {8'h98,8'h00, 8'h13}; //VID_STD=HD 1x1, 1280x720
            
            
            //bit[7:4]设置输入视频的颜色空间(RGB (range 0 to 255) )
            //bit[3]择YPbPr颜色空间转换的类型(不转换)
            //bit[2]设置数字数据的输出范围(0-255)
            //bit[1]RGB color space output
            //bit[0]根据bit[2]设置启用或禁用Data saturator。
            9'd9  : i2c_data <= {8'h98,8'h02, 8'h12}; 
            
            //选择数据格式和像素总线配置(24-bit 4:4:4 SDR mode)
            9'd10 : i2c_data <= {8'h98,8'h03, 8'h40}; 
            
            //bit[7:5]在像素引脚上选择像素数据总线的配置,BGR顺序。
            9'd11 : i2c_data <= {8'h98,8'h04, 8'h42}; 

            //bit[4]选择输出DE信号
            9'd12 : i2c_data <= {8'h98,8'h05, 8'h20}; 
            //   
            //bit[7]输出场同步信号
            //bit[3:1]正极性行、场、DE同步信号,bit[0]不插入LLC
            9'd13 : i2c_data <= {8'h98,8'h06, 8'ha6}; 
            //  
            //Powers up CP and digital sections of HDMI block
            //Powers up XTAL buffer to the digital core.               
            9'd14 : i2c_data <= {8'h98,8'h0b, 8'h44};
            
            // Put the RX section into power down mode
            //启用IIC控制和启动CP核心的时钟
            //启用行、场、DE同步信号。
            9'd15 : i2c_data <= {8'h98,8'h0C, 8'h42};
            //Audio,Sync,LLC,Pixel bus激活
            9'd16 : i2c_data <= {8'h98,8'h15, 8'h80}; 
            //使能LLC DLL ，正常频率，默认值。
            9'd17 : i2c_data <= {8'h98,8'h19, 8'h80}; 
            9'd18 : i2c_data <= {8'h98,8'h33, 8'h40}; 
            
            //设置输出驱动的驱动强度（默认即可）。
            9'd19 : i2c_data <= {8'h98,8'h14, 8'h4c}; 
            
            //CP MAP
            //启用自由运行模式
            9'd20 : i2c_data <= {8'h44,8'hba, 8'h01}; 
            9'd21 : i2c_data <= {8'h44,8'h7c, 8'h01}; 
            
            //REPEATER MAP
            9'd22 : i2c_data <= {8'h64,8'h40, 8'h81}; 

            //HDMI MAP
            9'd23 : i2c_data <= {8'h68,8'h9b, 8'h03};
            //官方推荐
            9'd24 : i2c_data <= {8'h68,8'hc1, 8'h01}; 
            9'd25 : i2c_data <= {8'h68,8'hc2, 8'h01}; 
            9'd26 : i2c_data <= {8'h68,8'hc3, 8'h01}; 
            9'd27 : i2c_data <= {8'h68,8'hc4, 8'h01};  
            9'd28 : i2c_data <= {8'h68,8'hc5, 8'h01}; 
            9'd29 : i2c_data <= {8'h68,8'hc6, 8'h01};  
            9'd30 : i2c_data <= {8'h68,8'hc7, 8'h01};      
            9'd31 : i2c_data <= {8'h68,8'hc8, 8'h01}; 
            9'd32 : i2c_data <= {8'h68,8'hc9, 8'h01};             
            9'd33 : i2c_data <= {8'h68,8'hca, 8'h01}; 
            9'd34 : i2c_data <= {8'h68,8'hcb, 8'h01}; 
            9'd35 : i2c_data <= {8'h68,8'hcc, 8'h01}; 
            
            9'd36 : i2c_data <= {8'h68,8'h00, 8'h00}; 
            9'd37 : i2c_data <= {8'h68,8'h83, 8'hfe}; 
            9'd38 : i2c_data <= {8'h68,8'h6f, 8'h08}; 
            9'd39 : i2c_data <= {8'h68,8'h85, 8'h1f}; 
            9'd40 : i2c_data <= {8'h68,8'h87, 8'h70}; 
            //Device can support video modes above 480p/576
            9'd41 : i2c_data <= {8'h68,8'h8d, 8'h04}; 
            9'd42 : i2c_data <= {8'h68,8'h8e, 8'h1e}; 
            
            9'd43 : i2c_data <= {8'h68,8'h1a, 8'h8a}; 
            9'd44 : i2c_data <= {8'h68,8'h57, 8'hda}; 
            9'd45 : i2c_data <= {8'h68,8'h58, 8'h01}; 
            9'd46 : i2c_data <= {8'h68,8'h75, 8'h10};
            9'd47 : i2c_data <= {8'h68,8'h6c ,8'ha3}; 

            //IO MAP
            9'd48 : i2c_data <= {8'h98,8'h20 ,8'h70}; 

            //REPEATER MAP
            9'd49 : i2c_data <= {8'h64,8'h74 ,8'h00}; 

			//EDID MAP
            9'd50  : i2c_data <= {8'h6c,8'd0  , 8'h00}; 
            9'd51  : i2c_data <= {8'h6c,8'd1  , 8'hFF};     
            9'd52  : i2c_data <= {8'h6c,8'd2  , 8'hFF};
            9'd53  : i2c_data <= {8'h6c,8'd3  , 8'hFF}; 
            9'd54  : i2c_data <= {8'h6c,8'd4  , 8'hFF}; 
            9'd55  : i2c_data <= {8'h6c,8'd5  , 8'hFF}; 
            9'd56  : i2c_data <= {8'h6c,8'd6  , 8'hFF}; 
            9'd57  : i2c_data <= {8'h6c,8'd7  , 8'h00}; 
            9'd58  : i2c_data <= {8'h6c,8'd8  , 8'h20}; 
            9'd59  : i2c_data <= {8'h6c,8'd9  , 8'hA3};   
            9'd60  : i2c_data <= {8'h6c,8'd10 , 8'h29}; 
            9'd61  : i2c_data <= {8'h6c,8'd11 , 8'h00}; 
            9'd62  : i2c_data <= {8'h6c,8'd12 , 8'h01}; 
            9'd63  : i2c_data <= {8'h6c,8'd13 , 8'h00}; 
            9'd64  : i2c_data <= {8'h6c,8'd14 , 8'h00}; 
            9'd65  : i2c_data <= {8'h6c,8'd15 , 8'h00}; 
            9'd66  : i2c_data <= {8'h6c,8'd16 , 8'h23}; 
            9'd67  : i2c_data <= {8'h6c,8'd17 , 8'h12};             
            9'd68  : i2c_data <= {8'h6c,8'd18 , 8'h01}; 
            9'd69  : i2c_data <= {8'h6c,8'd19 , 8'h03}; 
			9'd70  : i2c_data <= {8'h6c,8'd20 , 8'h80};
			9'd71  : i2c_data <= {8'h6c,8'd21 , 8'h73};
			9'd72  : i2c_data <= {8'h6c,8'd22 , 8'h41};
			9'd73  : i2c_data <= {8'h6c,8'd23 , 8'h78};
			9'd74  : i2c_data <= {8'h6c,8'd24 , 8'h0A};
			9'd75  : i2c_data <= {8'h6c,8'd25 , 8'hF3};
			9'd76  : i2c_data <= {8'h6c,8'd26 , 8'h30};
			9'd77  : i2c_data <= {8'h6c,8'd27 , 8'hA7};
			9'd78  : i2c_data <= {8'h6c,8'd28 , 8'h54};
			9'd79  : i2c_data <= {8'h6c,8'd29 , 8'h42};
			9'd80  : i2c_data <= {8'h6c,8'd30 , 8'hAA};
			9'd81  : i2c_data <= {8'h6c,8'd31 , 8'h26};
			9'd82  : i2c_data <= {8'h6c,8'd32 , 8'h0F};
			9'd83  : i2c_data <= {8'h6c,8'd33 , 8'h50};
			9'd84  : i2c_data <= {8'h6c,8'd34 , 8'h54};
			9'd85  : i2c_data <= {8'h6c,8'd35 , 8'h25};
			9'd86  : i2c_data <= {8'h6c,8'd36 , 8'hC8};
			9'd87  : i2c_data <= {8'h6c,8'd37 , 8'h00};
			9'd88  : i2c_data <= {8'h6c,8'd38 , 8'h61};
			9'd89  : i2c_data <= {8'h6c,8'd39 , 8'h4F};
			9'd90  : i2c_data <= {8'h6c,8'd40 , 8'h01};
			9'd91  : i2c_data <= {8'h6c,8'd41 , 8'h01};
			9'd92  : i2c_data <= {8'h6c,8'd42 , 8'h01};
			9'd93  : i2c_data <= {8'h6c,8'd43 , 8'h01};
			9'd94  : i2c_data <= {8'h6c,8'd44 , 8'h01};
			9'd95  : i2c_data <= {8'h6c,8'd45 , 8'h01};
			9'd96  : i2c_data <= {8'h6c,8'd46 , 8'h01};
			9'd97  : i2c_data <= {8'h6c,8'd47 , 8'h01};
			9'd98  : i2c_data <= {8'h6c,8'd48 , 8'h01};
			9'd99  : i2c_data <= {8'h6c,8'd49 , 8'h01};
			9'd100 : i2c_data <= {8'h6c,8'd50 , 8'h01};
			9'd101 : i2c_data <= {8'h6c,8'd51 , 8'h01};
			9'd102 : i2c_data <= {8'h6c,8'd52 , 8'h01};
			9'd103 : i2c_data <= {8'h6c,8'd53 , 8'h01};
			// Preferred timing: CTA-861 1280x720p60, 74.25 MHz pixel clock.
			9'd104 : i2c_data <= {8'h6c,8'd54 , 8'h01}; //Pixel clock LSB: 7425 * 10 kHz
			9'd105 : i2c_data <= {8'h6c,8'd55 , 8'h1D}; //Pixel clock MSB
			9'd106 : i2c_data <= {8'h6c,8'd56 , 8'h00}; //H active LSB: 1280
			9'd107 : i2c_data <= {8'h6c,8'd57 , 8'h72}; //H blanking LSB: 370
			9'd108 : i2c_data <= {8'h6c,8'd58 , 8'h51}; //H active/blanking MSBs
			9'd109 : i2c_data <= {8'h6c,8'd59 , 8'hD0}; //V active LSB: 720
			9'd110 : i2c_data <= {8'h6c,8'd60 , 8'h1E}; //V blanking LSB: 30
			9'd111 : i2c_data <= {8'h6c,8'd61 , 8'h20}; //V active/blanking MSBs
			9'd112 : i2c_data <= {8'h6c,8'd62 , 8'h6E}; //H sync offset: 110
			9'd113 : i2c_data <= {8'h6c,8'd63 , 8'h28}; //H sync width: 40
			9'd114 : i2c_data <= {8'h6c,8'd64 , 8'h55}; //V sync offset/width: 5/5
			9'd115 : i2c_data <= {8'h6c,8'd65 , 8'h00}; //Sync offset/width MSBs
			9'd116 : i2c_data <= {8'h6c,8'd66 , 8'hC4}; //Image size H LSB
			9'd117 : i2c_data <= {8'h6c,8'd67 , 8'h8E}; //Image size V LSB
			9'd118 : i2c_data <= {8'h6c,8'd68 , 8'h21}; //Image size MSBs
			9'd119 : i2c_data <= {8'h6c,8'd69 , 8'h00}; //H border
			9'd120 : i2c_data <= {8'h6c,8'd70 , 8'h00}; //V border
			9'd121 : i2c_data <= {8'h6c,8'd71 , 8'h1E}; //Progressive, positive H/V sync
			9'd122 : i2c_data <= {8'h6c,8'd72 , 8'h8C};
			9'd123 : i2c_data <= {8'h6c,8'd73 , 8'h0A};
			9'd124 : i2c_data <= {8'h6c,8'd74 , 8'hD0};
			9'd125 : i2c_data <= {8'h6c,8'd75 , 8'h8A};
			9'd126 : i2c_data <= {8'h6c,8'd76 , 8'h20};
			9'd127 : i2c_data <= {8'h6c,8'd77 , 8'hE0};
			9'd128 : i2c_data <= {8'h6c,8'd78 , 8'h2D};
			9'd129 : i2c_data <= {8'h6c,8'd79 , 8'h10};
			9'd130 : i2c_data <= {8'h6c,8'd80 , 8'h10};
			9'd131 : i2c_data <= {8'h6c,8'd81 , 8'h3E};
			9'd132 : i2c_data <= {8'h6c,8'd82 , 8'h96};
			9'd133 : i2c_data <= {8'h6c,8'd83 , 8'h00};
			9'd134 : i2c_data <= {8'h6c,8'd84 , 8'h80};
			9'd135 : i2c_data <= {8'h6c,8'd85 , 8'h88};
			9'd136 : i2c_data <= {8'h6c,8'd86 , 8'h42};
			9'd137 : i2c_data <= {8'h6c,8'd87 , 8'h00};
			9'd138 : i2c_data <= {8'h6c,8'd88 , 8'h00};
			9'd139 : i2c_data <= {8'h6c,8'd89 , 8'h18};
			9'd140 : i2c_data <= {8'h6c,8'd90 , 8'h00};
			9'd141 : i2c_data <= {8'h6c,8'd91 , 8'h00};
			9'd142 : i2c_data <= {8'h6c,8'd92 , 8'h00};
			9'd143 : i2c_data <= {8'h6c,8'd93 , 8'hFC};
			9'd144 : i2c_data <= {8'h6c,8'd94 , 8'h00};
			9'd145 : i2c_data <= {8'h6c,8'd95 , 8'h48};
			9'd146 : i2c_data <= {8'h6c,8'd96 , 8'h44};
			9'd147 : i2c_data <= {8'h6c,8'd97 , 8'h4D};
			9'd148 : i2c_data <= {8'h6c,8'd98 , 8'h49};
			9'd149 : i2c_data <= {8'h6c,8'd99 , 8'h20};
			9'd150 : i2c_data <= {8'h6c,8'd100 , 8'h20};
			9'd151 : i2c_data <= {8'h6c,8'd101 , 8'h20};
			9'd152 : i2c_data <= {8'h6c,8'd102 , 8'h20};
			9'd153 : i2c_data <= {8'h6c,8'd103 , 8'h0A};
			9'd154 : i2c_data <= {8'h6c,8'd104 , 8'h20};
			9'd155 : i2c_data <= {8'h6c,8'd105 , 8'h20};
			9'd156 : i2c_data <= {8'h6c,8'd106 , 8'h20};
			9'd157 : i2c_data <= {8'h6c,8'd107 , 8'h20};
			9'd158 : i2c_data <= {8'h6c,8'd108 , 8'h00};
			9'd159 : i2c_data <= {8'h6c,8'd109 , 8'h00};
			9'd160 : i2c_data <= {8'h6c,8'd110 , 8'h00};
			9'd161 : i2c_data <= {8'h6c,8'd111 , 8'hFD};
			9'd162 : i2c_data <= {8'h6c,8'd112 , 8'h00};
			9'd163 : i2c_data <= {8'h6c,8'd113 , 8'h32};
			9'd164 : i2c_data <= {8'h6c,8'd114 , 8'h55};
			9'd165 : i2c_data <= {8'h6c,8'd115 , 8'h1F};
			9'd166 : i2c_data <= {8'h6c,8'd116 , 8'h45};
			9'd167 : i2c_data <= {8'h6c,8'd117 , 8'h0F};
			9'd168 : i2c_data <= {8'h6c,8'd118 , 8'h00};
			9'd169 : i2c_data <= {8'h6c,8'd119 , 8'h0A};
			9'd170 : i2c_data <= {8'h6c,8'd120 , 8'h20};
			9'd171 : i2c_data <= {8'h6c,8'd121 , 8'h20};
			9'd172 : i2c_data <= {8'h6c,8'd122 , 8'h20};
			9'd173 : i2c_data <= {8'h6c,8'd123 , 8'h20};
			9'd174 : i2c_data <= {8'h6c,8'd124 , 8'h20};
			9'd175 : i2c_data <= {8'h6c,8'd125 , 8'h20};
			9'd176 : i2c_data <= {8'h6c,8'd126 , 8'h00}; //No extension block is programmed
			9'd177 : i2c_data <= {8'h6c,8'd127 , 8'hD5}; //Base-block checksum

			//REPEATER MAP
			9'd178 : i2c_data <= {8'h64,8'h74 ,8'h01};

			//IO Map
			9'd179 : i2c_data <= {8'h98,8'h20 ,8'hf0};

			///EDID MAP
			9'd180 : i2c_data <= {8'h68,8'h6c ,8'ha2}; 

            default:i2c_data <=	0; 
        endcase
    end
    
endmodule
