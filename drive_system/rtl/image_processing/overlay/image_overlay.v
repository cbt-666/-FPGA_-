// 原始HDMI全屏叠加器：三个128x128柔边箭头和黄色四周渐隐警告。
// 箭头颜色编码：00红、01绿、10蓝、11黄；主体在对应色系内由上亮到下深渐变。
// 各图形可独立闪烁并同时叠加。
module image_overlay #(
    parameter integer ACTIVE_WIDTH  = 1280,
    parameter integer ACTIVE_HEIGHT = 720
)(
    input              clk,
    input              rst_n,
    input              de_in,
    input              hs_in,
    input              vs_in,
    input      [23:0]  data_in,
    input              overlay_enable,
    input              left_enable,
    input              right_enable,
    input              up_enable,
    input              warning_enable,
    input      [1:0]   left_color,
    input      [1:0]   right_color,
    input      [1:0]   up_color,
    input              left_blink,
    input              right_blink,
    input              up_blink,
    input              warning_blink,
    input              blink_visible,
    output reg         de,
    output reg         hs,
    output reg         vs,
    output reg [23:0]  data
);

localparam integer ROM_W = 32;
localparam integer ROM_H = 32;
localparam integer SCALE_SHIFT = 2;
localparam integer PATTERN_W = ROM_W << SCALE_SHIFT;
localparam integer PATTERN_H = ROM_H << SCALE_SHIFT;
localparam integer ARROW_Y = 520;
localparam integer LEFT_X = 48;
localparam integer RIGHT_X = ACTIVE_WIDTH - 48 - PATTERN_W;
localparam integer UP_X = (ACTIVE_WIDTH - PATTERN_W) / 2;

reg [10:0] x_pos;
reg [10:0] y_pos;
reg de_d;
reg vs_d;
wire line_end = de_d && !de_in;
wire frame_edge = vs_d ^ vs_in;

reg [31:0] arrow_rom [0:31];
reg [31:0] up_rom [0:31];
initial begin
    arrow_rom[0]=32'h00000000; arrow_rom[1]=32'h00000000;
    arrow_rom[2]=32'h00000000; arrow_rom[3]=32'h00000000;
    arrow_rom[4]=32'h00000100; arrow_rom[5]=32'h00000300;
    arrow_rom[6]=32'h00000700; arrow_rom[7]=32'h00000f00;
    arrow_rom[8]=32'h00001f00; arrow_rom[9]=32'h00003f00;
    arrow_rom[10]=32'h00007f00; arrow_rom[11]=32'h0000ffff;
    arrow_rom[12]=32'h0001ffff; arrow_rom[13]=32'h0000ffff;
    arrow_rom[14]=32'h00007f00; arrow_rom[15]=32'h00003f00;
    arrow_rom[16]=32'h00001f00; arrow_rom[17]=32'h00000f00;
    arrow_rom[18]=32'h00000700; arrow_rom[19]=32'h00000300;
    arrow_rom[20]=32'h00000100; arrow_rom[21]=32'h00000000;
    arrow_rom[22]=32'h00000000; arrow_rom[23]=32'h00000000;
    arrow_rom[24]=32'h00000000; arrow_rom[25]=32'h00000000;
    arrow_rom[26]=32'h00000000; arrow_rom[27]=32'h00000000;
    arrow_rom[28]=32'h00000000; arrow_rom[29]=32'h00000000;
    arrow_rom[30]=32'h00000000; arrow_rom[31]=32'h00000000;

    up_rom[0]=32'h00000000;  up_rom[1]=32'h00000000;
    up_rom[2]=32'h00018000;  up_rom[3]=32'h00018000;
    up_rom[4]=32'h0003c000;  up_rom[5]=32'h0003c000;
    up_rom[6]=32'h0007e000;  up_rom[7]=32'h0007e000;
    up_rom[8]=32'h000ff000;  up_rom[9]=32'h000ff000;
    up_rom[10]=32'h001ff800; up_rom[11]=32'h001ff800;
    up_rom[12]=32'h003ffc00; up_rom[13]=32'h003ffc00;
    up_rom[14]=32'h007ffe00; up_rom[15]=32'h007ffe00;
    up_rom[16]=32'h00ffff00; up_rom[17]=32'h01ffff80;
    up_rom[18]=32'h000ff000; up_rom[19]=32'h000ff000;
    up_rom[20]=32'h000ff000; up_rom[21]=32'h000ff000;
    up_rom[22]=32'h000ff000; up_rom[23]=32'h000ff000;
    up_rom[24]=32'h000ff000; up_rom[25]=32'h000ff000;
    up_rom[26]=32'h000ff000; up_rom[27]=32'h000ff000;
    up_rom[28]=32'h00000000; up_rom[29]=32'h00000000;
    up_rom[30]=32'h00000000; up_rom[31]=32'h00000000;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        x_pos <= 11'd0;
        y_pos <= 11'd0;
        de_d <= 1'b0;
        vs_d <= 1'b0;
    end else begin
        de_d <= de_in;
        vs_d <= vs_in;
        if (frame_edge) begin
            x_pos <= 11'd0;
            y_pos <= 11'd0;
        end else if (de_in) begin
            x_pos <= x_pos + 1'b1;
        end else begin
            x_pos <= 11'd0;
            if (line_end)
                y_pos <= y_pos + 1'b1;
        end
    end
end

function arrow_bit_at;
    input [10:0] px;
    input [10:0] py;
    input [10:0] sx;
    input reverse_x;
    integer ix;
    integer iy;
    begin
        arrow_bit_at = 1'b0;
        if ((px >= sx) && (px < sx + PATTERN_W) &&
            (py >= ARROW_Y) && (py < ARROW_Y + PATTERN_H)) begin
            ix = (px - sx) >> SCALE_SHIFT;
            iy = (py - ARROW_Y) >> SCALE_SHIFT;
            if (reverse_x)
                arrow_bit_at = arrow_rom[iy][ix];
            else
                arrow_bit_at = arrow_rom[iy][ROM_W-1-ix];
        end
    end
endfunction

function up_bit_at;
    input [10:0] px;
    input [10:0] py;
    integer ix;
    integer iy;
    begin
        up_bit_at = 1'b0;
        if ((px >= UP_X) && (px < UP_X + PATTERN_W) &&
            (py >= ARROW_Y) && (py < ARROW_Y + PATTERN_H)) begin
            ix = (px - UP_X) >> SCALE_SHIFT;
            iy = (py - ARROW_Y) >> SCALE_SHIFT;
            up_bit_at = up_rom[iy][ROM_W-1-ix];
        end
    end
endfunction

// 将箭头内部纵坐标限制在0~127，柔光区域沿用最近的主体渐变色。
wire [6:0] arrow_gradient_y = (y_pos <= ARROW_Y) ? 7'd0 :
                              (y_pos >= ARROW_Y + PATTERN_H - 1) ? 7'd127 :
                              y_pos - ARROW_Y;

// 保留协议指定的红/绿/蓝/黄色相，以移位减法生成由上亮到下深的渐变。
// 只使用固定移位和加减法，避免在像素时钟路径中推断乘法器。
function [23:0] color_rgb;
    input [1:0] code;
    input [6:0] gradient_y;
    reg [7:0] half_y;
    reg [7:0] quarter_y;
    begin
        half_y = {1'b0, gradient_y} >> 1;
        quarter_y = {1'b0, gradient_y} >> 2;
        case (code)
            // 顶部颜色略亮，底部提高主色纯度并适当压暗。
            2'b00: color_rgb = {8'd255 - quarter_y,
                                8'd112 - half_y,
                                8'd96  - half_y};
            2'b01: color_rgb = {8'd96  - half_y,
                                8'd255 - quarter_y,
                                8'd112 - half_y};
            2'b10: color_rgb = {8'd80  - half_y,
                                8'd144 - half_y,
                                8'd255 - quarter_y};
            default: color_rgb = {8'd255 - quarter_y,
                                  8'd240 - half_y,
                                  8'd80  - half_y};
        endcase
    end
endfunction

function [7:0] blend_3_4;
    input [7:0] base;
    input [7:0] over;
    reg [9:0] sum;
    begin
        sum = {2'b0,base} + {2'b0,over} + ({2'b0,over} << 1);
        blend_3_4 = sum[9:2];
    end
endfunction

function [7:0] blend_1_4;
    input [7:0] base;
    input [7:0] over;
    reg [9:0] sum;
    begin
        sum = {2'b0,over} + {2'b0,base} + ({2'b0,base} << 1);
        blend_1_4 = sum[9:2];
    end
endfunction

wire left_show = overlay_enable && left_enable && (!left_blink || blink_visible);
wire right_show = overlay_enable && right_enable && (!right_blink || blink_visible);
wire up_show = overlay_enable && up_enable && (!up_blink || blink_visible);
wire warning_show = overlay_enable && warning_enable &&
                    (!warning_blink || blink_visible);

wire l0 = arrow_bit_at(x_pos, y_pos, LEFT_X, 1'b0);
wire r0 = arrow_bit_at(x_pos, y_pos, RIGHT_X, 1'b1);
wire u0 = up_bit_at(x_pos, y_pos);
wire lg = arrow_bit_at(x_pos-11'd3,y_pos,LEFT_X,1'b0) |
          arrow_bit_at(x_pos+11'd3,y_pos,LEFT_X,1'b0) |
          arrow_bit_at(x_pos,y_pos-11'd3,LEFT_X,1'b0) |
          arrow_bit_at(x_pos,y_pos+11'd3,LEFT_X,1'b0);
wire rg = arrow_bit_at(x_pos-11'd3,y_pos,RIGHT_X,1'b1) |
          arrow_bit_at(x_pos+11'd3,y_pos,RIGHT_X,1'b1) |
          arrow_bit_at(x_pos,y_pos-11'd3,RIGHT_X,1'b1) |
          arrow_bit_at(x_pos,y_pos+11'd3,RIGHT_X,1'b1);
wire ug = up_bit_at(x_pos-11'd3,y_pos) | up_bit_at(x_pos+11'd3,y_pos) |
          up_bit_at(x_pos,y_pos-11'd3) | up_bit_at(x_pos,y_pos+11'd3);
wire ls = arrow_bit_at(x_pos-11'd6,y_pos-11'd6,LEFT_X,1'b0);
wire rs = arrow_bit_at(x_pos-11'd6,y_pos-11'd6,RIGHT_X,1'b1);
wire us = up_bit_at(x_pos-11'd6,y_pos-11'd6);

wire [23:0] left_rgb = color_rgb(left_color, arrow_gradient_y);
wire [23:0] right_rgb = color_rgb(right_color, arrow_gradient_y);
wire [23:0] up_rgb = color_rgb(up_color, arrow_gradient_y);

// warning工程中的四边96像素渐隐，去掉中央警告图标。
wire [10:0] edge_x_dist = (x_pos < ACTIVE_WIDTH/2) ? x_pos : ACTIVE_WIDTH-1-x_pos;
wire [10:0] edge_y_dist = (y_pos < ACTIVE_HEIGHT/2) ? y_pos : ACTIVE_HEIGHT-1-y_pos;
wire [10:0] edge_dist = (edge_x_dist < edge_y_dist) ? edge_x_dist : edge_y_dist;
wire [3:0] edge_alpha = (edge_dist < 12) ? 4'd8 :
                        (edge_dist < 24) ? 4'd7 :
                        (edge_dist < 36) ? 4'd6 :
                        (edge_dist < 48) ? 4'd5 :
                        (edge_dist < 60) ? 4'd4 :
                        (edge_dist < 72) ? 4'd3 :
                        (edge_dist < 84) ? 4'd2 :
                        (edge_dist < 96) ? 4'd1 : 4'd0;

function [7:0] blend_16;
    input [7:0] base;
    input [7:0] over;
    input [3:0] alpha;
    reg [12:0] b;
    reg [12:0] o;
    reg [12:0] sum;
    begin
        b = {5'd0, base};
        o = {5'd0, over};
        // alpha/16常系数混色，避免像素关键路径推断可变乘法器。
        case (alpha)
            4'd1:  sum = (b << 3) + (b << 2) + (b << 1) + b + o;
            4'd2:  sum = (b << 3) + (b << 2) + (b << 1) + (o << 1);
            4'd3:  sum = (b << 3) + (b << 2) + b + (o << 1) + o;
            4'd4:  sum = (b << 3) + (b << 2) + (o << 2);
            4'd5:  sum = (b << 3) + (b << 1) + b + (o << 2) + o;
            4'd6:  sum = (b << 3) + (b << 1) + (o << 2) + (o << 1);
            4'd7:  sum = (b << 3) + b + (o << 2) + (o << 1) + o;
            4'd8:  sum = (b << 3) + (o << 3);
            4'd9:  sum = (b << 2) + (b << 1) + b + (o << 3) + o;
            4'd10: sum = (b << 2) + (b << 1) + (o << 3) + (o << 1);
            4'd11: sum = (b << 2) + b + (o << 3) + (o << 1) + o;
            4'd12: sum = (b << 2) + (o << 3) + (o << 2);
            4'd13: sum = (b << 1) + b + (o << 3) + (o << 2) + o;
            4'd14: sum = (b << 1) + (o << 3) + (o << 2) + (o << 1);
            4'd15: sum = b + (o << 3) + (o << 2) + (o << 1) + o;
            default: sum = b << 4;
        endcase
        blend_16 = (sum + 8) >> 4;
    end
endfunction

function [23:0] apply_arrow;
    input [23:0] base;
    input [23:0] color;
    input core;
    input glow;
    input shadow;
    input show;
    reg [23:0] value;
    begin
        value = base;
        if (show && shadow && !glow)
            value = {base[23:16] >> 1, base[15:8] >> 1, base[7:0] >> 1};
        if (show && glow)
            value = core ?
                    {blend_3_4(base[23:16],color[23:16]),
                     blend_3_4(base[15:8], color[15:8]),
                     blend_3_4(base[7:0],  color[7:0])} :
                    {blend_1_4(base[23:16],color[23:16]),
                     blend_1_4(base[15:8], color[15:8]),
                     blend_1_4(base[7:0],  color[7:0])};
        apply_arrow = value;
    end
endfunction

// 将坐标译码、边缘混色和三个箭头分级，避免一拍内串联所有叠加运算。
reg [23:0] s0_data;
reg [3:0]  s0_edge_alpha;
reg        s0_warning_show;
reg        s0_l0, s0_r0, s0_u0;
reg        s0_lg, s0_rg, s0_ug;
reg        s0_ls, s0_rs, s0_us;
reg        s0_left_show, s0_right_show, s0_up_show;
reg [23:0] s0_left_rgb, s0_right_rgb, s0_up_rgb;
reg        s0_de, s0_hs, s0_vs;

reg [23:0] s1_data;
reg        s1_l0, s1_r0, s1_u0;
reg        s1_lg, s1_rg, s1_ug;
reg        s1_ls, s1_rs, s1_us;
reg        s1_left_show, s1_right_show, s1_up_show;
reg [23:0] s1_left_rgb, s1_right_rgb, s1_up_rgb;
reg        s1_de, s1_hs, s1_vs;

reg [23:0] s2_data;
reg        s2_r0, s2_u0, s2_rg, s2_ug, s2_rs, s2_us;
reg        s2_right_show, s2_up_show;
reg [23:0] s2_right_rgb, s2_up_rgb;
reg        s2_de, s2_hs, s2_vs;

reg [23:0] s3_data;
reg        s3_u0, s3_ug, s3_us, s3_up_show;
reg [23:0] s3_up_rgb;
reg        s3_de, s3_hs, s3_vs;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        s0_data <= 24'd0; s0_edge_alpha <= 4'd0; s0_warning_show <= 1'b0;
        s0_l0 <= 1'b0; s0_r0 <= 1'b0; s0_u0 <= 1'b0;
        s0_lg <= 1'b0; s0_rg <= 1'b0; s0_ug <= 1'b0;
        s0_ls <= 1'b0; s0_rs <= 1'b0; s0_us <= 1'b0;
        s0_left_show <= 1'b0; s0_right_show <= 1'b0; s0_up_show <= 1'b0;
        s0_left_rgb <= 24'd0; s0_right_rgb <= 24'd0; s0_up_rgb <= 24'd0;
        s0_de <= 1'b0; s0_hs <= 1'b0; s0_vs <= 1'b0;
        s1_data <= 24'd0;
        s1_l0 <= 1'b0; s1_r0 <= 1'b0; s1_u0 <= 1'b0;
        s1_lg <= 1'b0; s1_rg <= 1'b0; s1_ug <= 1'b0;
        s1_ls <= 1'b0; s1_rs <= 1'b0; s1_us <= 1'b0;
        s1_left_show <= 1'b0; s1_right_show <= 1'b0; s1_up_show <= 1'b0;
        s1_left_rgb <= 24'd0; s1_right_rgb <= 24'd0; s1_up_rgb <= 24'd0;
        s1_de <= 1'b0; s1_hs <= 1'b0; s1_vs <= 1'b0;
        s2_data <= 24'd0;
        s2_r0 <= 1'b0; s2_u0 <= 1'b0; s2_rg <= 1'b0; s2_ug <= 1'b0;
        s2_rs <= 1'b0; s2_us <= 1'b0; s2_right_show <= 1'b0; s2_up_show <= 1'b0;
        s2_right_rgb <= 24'd0; s2_up_rgb <= 24'd0;
        s2_de <= 1'b0; s2_hs <= 1'b0; s2_vs <= 1'b0;
        s3_data <= 24'd0; s3_u0 <= 1'b0; s3_ug <= 1'b0; s3_us <= 1'b0;
        s3_up_show <= 1'b0; s3_up_rgb <= 24'd0;
        s3_de <= 1'b0; s3_hs <= 1'b0; s3_vs <= 1'b0;
        de <= 1'b0;
        hs <= 1'b0;
        vs <= 1'b0;
        data <= 24'd0;
    end else begin
        // 第0级：锁存当前像素、坐标译码结果和控制量。
        s0_data <= data_in;
        s0_edge_alpha <= edge_alpha;
        s0_warning_show <= warning_show;
        s0_l0 <= l0; s0_r0 <= r0; s0_u0 <= u0;
        s0_lg <= lg; s0_rg <= rg; s0_ug <= ug;
        s0_ls <= ls; s0_rs <= rs; s0_us <= us;
        s0_left_show <= left_show; s0_right_show <= right_show; s0_up_show <= up_show;
        s0_left_rgb <= left_rgb; s0_right_rgb <= right_rgb; s0_up_rgb <= up_rgb;
        s0_de <= de_in; s0_hs <= hs_in; s0_vs <= vs_in;

        // 第1级：黄色边缘警告混色。
        s1_data <= (s0_warning_show && (s0_edge_alpha != 0)) ?
                   {blend_16(s0_data[23:16],8'hff,s0_edge_alpha),
                    blend_16(s0_data[15:8],8'hd0,s0_edge_alpha),
                    blend_16(s0_data[7:0],8'h00,s0_edge_alpha)} : s0_data;
        s1_l0 <= s0_l0; s1_r0 <= s0_r0; s1_u0 <= s0_u0;
        s1_lg <= s0_lg; s1_rg <= s0_rg; s1_ug <= s0_ug;
        s1_ls <= s0_ls; s1_rs <= s0_rs; s1_us <= s0_us;
        s1_left_show <= s0_left_show; s1_right_show <= s0_right_show; s1_up_show <= s0_up_show;
        s1_left_rgb <= s0_left_rgb; s1_right_rgb <= s0_right_rgb; s1_up_rgb <= s0_up_rgb;
        s1_de <= s0_de; s1_hs <= s0_hs; s1_vs <= s0_vs;

        // 第2~4级：每级只处理一个方向箭头。
        s2_data <= apply_arrow(s1_data, s1_left_rgb, s1_l0, s1_lg, s1_ls, s1_left_show);
        s2_r0 <= s1_r0; s2_u0 <= s1_u0; s2_rg <= s1_rg; s2_ug <= s1_ug;
        s2_rs <= s1_rs; s2_us <= s1_us;
        s2_right_show <= s1_right_show; s2_up_show <= s1_up_show;
        s2_right_rgb <= s1_right_rgb; s2_up_rgb <= s1_up_rgb;
        s2_de <= s1_de; s2_hs <= s1_hs; s2_vs <= s1_vs;

        s3_data <= apply_arrow(s2_data, s2_right_rgb, s2_r0, s2_rg, s2_rs, s2_right_show);
        s3_u0 <= s2_u0; s3_ug <= s2_ug; s3_us <= s2_us;
        s3_up_show <= s2_up_show; s3_up_rgb <= s2_up_rgb;
        s3_de <= s2_de; s3_hs <= s2_hs; s3_vs <= s2_vs;

        de <= s3_de;
        hs <= s3_hs;
        vs <= s3_vs;
        data <= s3_de ? apply_arrow(s3_data, s3_up_rgb, s3_u0, s3_ug, s3_us, s3_up_show) : 24'd0;
    end
end

endmodule
