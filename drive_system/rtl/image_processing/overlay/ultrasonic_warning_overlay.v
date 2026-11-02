// 四宫格超声波警告叠加：常亮分级边缘与128x128闪烁中央标志。
module ultrasonic_warning_overlay #(
    parameter integer ACTIVE_WIDTH = 1280,
    parameter integer ACTIVE_HEIGHT = 720,
    parameter integer BLINK_HALF_FRAMES = 15
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        de_in,
    input  wire        hs_in,
    input  wire        vs_in,
    input  wire [23:0] data_in,
    input  wire        quad_mode_active,
    input  wire [1:0]  alert_level,
    output reg         de,
    output reg         hs,
    output reg         vs,
    output reg  [23:0] data
);

localparam integer ROM_W = 32;
localparam integer ROM_H = 32;
localparam integer SCALE_SHIFT = 2;
localparam integer PATTERN_W = ROM_W << SCALE_SHIFT;
localparam integer PATTERN_H = ROM_H << SCALE_SHIFT;
localparam integer PATTERN_X = (ACTIVE_WIDTH - PATTERN_W) / 2;
localparam integer PATTERN_Y = (ACTIVE_HEIGHT - PATTERN_H) / 2;
localparam integer SHADOW_OFFSET = 4;

localparam [7:0] YELLOW_R = 8'd255;
localparam [7:0] YELLOW_G = 8'd208;
localparam [7:0] YELLOW_B = 8'd0;
localparam [7:0] ORANGE_R = 8'd255;
localparam [7:0] ORANGE_G = 8'd112;
localparam [7:0] ORANGE_B = 8'd0;
localparam [7:0] RED_R = 8'd255;
localparam [7:0] RED_G = 8'd32;
localparam [7:0] RED_B = 8'd0;
localparam [7:0] MARK_R = 8'd32;
localparam [7:0] MARK_G = 8'd24;
localparam [7:0] MARK_B = 8'd0;

reg [10:0] x_pos;
reg [10:0] y_pos;
reg        de_d;
reg        vs_d;
(* ASYNC_REG = "TRUE" *) reg [1:0] alert_meta;
(* ASYNC_REG = "TRUE" *) reg [1:0] alert_sync;
reg [1:0] alert_frame;
reg [4:0] blink_frame_cnt;
reg       icon_visible;

wire line_end = de_d && !de_in;
wire frame_rise = vs_in && !vs_d;
wire warning_active = quad_mode_active && (alert_frame != 2'd0);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        x_pos           <= 11'd0;
        y_pos           <= 11'd0;
        de_d            <= 1'b0;
        vs_d            <= 1'b0;
        alert_meta      <= 2'd0;
        alert_sync      <= 2'd0;
        alert_frame     <= 2'd0;
        blink_frame_cnt <= 5'd0;
        icon_visible    <= 1'b1;
    end else begin
        de_d       <= de_in;
        vs_d       <= vs_in;
        alert_meta <= alert_level;
        alert_sync <= alert_meta;

        if (frame_rise) begin
            x_pos       <= 11'd0;
            y_pos       <= 11'd0;
            alert_frame <= alert_sync;
            if (alert_sync == 2'd0) begin
                blink_frame_cnt <= 5'd0;
                icon_visible    <= 1'b1;
            end else if (blink_frame_cnt >= BLINK_HALF_FRAMES - 1) begin
                blink_frame_cnt <= 5'd0;
                icon_visible    <= ~icon_visible;
            end else begin
                blink_frame_cnt <= blink_frame_cnt + 1'b1;
            end
        end else if (de_in) begin
            x_pos <= x_pos + 1'b1;
        end else begin
            x_pos <= 11'd0;
            if (line_end)
                y_pos <= y_pos + 1'b1;
        end
    end
end

reg [31:0] pattern_rom [0:31];
initial begin
    pattern_rom[0]=32'h00000000;  pattern_rom[1]=32'h00000000;
    pattern_rom[2]=32'h00018000;  pattern_rom[3]=32'h0003c000;
    pattern_rom[4]=32'h0003c000;  pattern_rom[5]=32'h0007e000;
    pattern_rom[6]=32'h0007e000;  pattern_rom[7]=32'h000ff000;
    pattern_rom[8]=32'h000ff000;  pattern_rom[9]=32'h001ff800;
    pattern_rom[10]=32'h001ff800; pattern_rom[11]=32'h003ffc00;
    pattern_rom[12]=32'h003ffc00; pattern_rom[13]=32'h007ffe00;
    pattern_rom[14]=32'h007ffe00; pattern_rom[15]=32'h00ffff00;
    pattern_rom[16]=32'h00ffff00; pattern_rom[17]=32'h01ffff80;
    pattern_rom[18]=32'h01ffff80; pattern_rom[19]=32'h03ffffc0;
    pattern_rom[20]=32'h03ffffc0; pattern_rom[21]=32'h07ffffe0;
    pattern_rom[22]=32'h07ffffe0; pattern_rom[23]=32'h0ffffff0;
    pattern_rom[24]=32'h0ffffff0; pattern_rom[25]=32'h1ffffff8;
    pattern_rom[26]=32'h1ffffff8; pattern_rom[27]=32'h3ffffffc;
    pattern_rom[28]=32'h3ffffffc; pattern_rom[29]=32'h7ffffffe;
    pattern_rom[30]=32'h7ffffffe; pattern_rom[31]=32'h00000000;
end

function pattern_bit_at;
    input [10:0] px;
    input [10:0] py;
    integer ix;
    integer iy;
    begin
        pattern_bit_at = 1'b0;
        if ((px >= PATTERN_X) && (px < PATTERN_X + PATTERN_W) &&
            (py >= PATTERN_Y) && (py < PATTERN_Y + PATTERN_H)) begin
            ix = (px - PATTERN_X) >> SCALE_SHIFT;
            iy = (py - PATTERN_Y) >> SCALE_SHIFT;
            pattern_bit_at = pattern_rom[iy][ROM_W-1-ix];
        end
    end
endfunction

function [7:0] alpha_blend_16;
    input [7:0] base_color;
    input [7:0] overlay_color;
    input [3:0] alpha;
    reg [12:0] b;
    reg [12:0] o;
    reg [12:0] sum;
    begin
        b = {5'd0, base_color};
        o = {5'd0, overlay_color};
        case (alpha)
            4'd1:  sum = (b<<3)+(b<<2)+(b<<1)+b+o;
            4'd2:  sum = (b<<3)+(b<<2)+(b<<1)+(o<<1);
            4'd3:  sum = (b<<3)+(b<<2)+b+(o<<1)+o;
            4'd4:  sum = (b<<3)+(b<<2)+(o<<2);
            4'd5:  sum = (b<<3)+(b<<1)+b+(o<<2)+o;
            4'd6:  sum = (b<<3)+(b<<1)+(o<<2)+(o<<1);
            4'd7:  sum = (b<<3)+b+(o<<2)+(o<<1)+o;
            4'd8:  sum = (b<<3)+(o<<3);
            4'd9:  sum = (b<<2)+(b<<1)+b+(o<<3)+o;
            4'd10: sum = (b<<2)+(b<<1)+(o<<3)+(o<<1);
            4'd11: sum = (b<<2)+b+(o<<3)+(o<<1)+o;
            4'd12: sum = (b<<2)+(o<<3)+(o<<2);
            4'd13: sum = (b<<1)+b+(o<<3)+(o<<2)+o;
            4'd14: sum = (b<<1)+(o<<3)+(o<<2)+(o<<1);
            4'd15: sum = b+(o<<3)+(o<<2)+(o<<1)+o;
            default: sum = b << 4;
        endcase
        alpha_blend_16 = (sum + 13'd8) >> 4;
    end
endfunction

function [3:0] bayer12;
    input [1:0] px;
    input [1:0] py;
    begin
        case ({py,px})
            4'h0:bayer12=4'd0;  4'h1:bayer12=4'd6;
            4'h2:bayer12=4'd1;  4'h3:bayer12=4'd7;
            4'h4:bayer12=4'd9;  4'h5:bayer12=4'd3;
            4'h6:bayer12=4'd10; 4'h7:bayer12=4'd4;
            4'h8:bayer12=4'd2;  4'h9:bayer12=4'd8;
            4'ha:bayer12=4'd0;  4'hb:bayer12=4'd6;
            4'hc:bayer12=4'd11; 4'hd:bayer12=4'd5;
            4'he:bayer12=4'd9;  default:bayer12=4'd3;
        endcase
    end
endfunction

function [3:0] edge_alpha_at;
    input [10:0] distance;
    input [3:0] dither_threshold;
    reg [3:0] high_alpha;
    reg [3:0] remainder;
    begin
        if (distance < 12) begin high_alpha=8; remainder=distance; end
        else if (distance < 24) begin high_alpha=7; remainder=distance-12; end
        else if (distance < 36) begin high_alpha=6; remainder=distance-24; end
        else if (distance < 48) begin high_alpha=5; remainder=distance-36; end
        else if (distance < 60) begin high_alpha=4; remainder=distance-48; end
        else if (distance < 72) begin high_alpha=3; remainder=distance-60; end
        else if (distance < 84) begin high_alpha=2; remainder=distance-72; end
        else if (distance < 96) begin high_alpha=1; remainder=distance-84; end
        else begin high_alpha=0; remainder=0; end
        if (high_alpha == 0)
            edge_alpha_at = 0;
        else if (dither_threshold >= remainder)
            edge_alpha_at = high_alpha;
        else
            edge_alpha_at = high_alpha - 1'b1;
    end
endfunction

wire [10:0] x_edge_distance = (x_pos < ACTIVE_WIDTH/2) ? x_pos : ACTIVE_WIDTH-1-x_pos;
wire [10:0] y_edge_distance = (y_pos < ACTIVE_HEIGHT/2) ? y_pos : ACTIVE_HEIGHT-1-y_pos;
wire [3:0] x_edge_alpha = edge_alpha_at(x_edge_distance,bayer12(x_pos[1:0],y_pos[1:0]));
wire [3:0] y_edge_alpha = edge_alpha_at(y_edge_distance,bayer12(y_pos[1:0],x_pos[1:0]));

wire triangle_bit = pattern_bit_at(x_pos,y_pos);
wire aa_bit_00 = pattern_bit_at(x_pos-2,y_pos-2);
wire aa_bit_01 = pattern_bit_at(x_pos+2,y_pos-2);
wire aa_bit_10 = pattern_bit_at(x_pos-2,y_pos+2);
wire aa_bit_11 = pattern_bit_at(x_pos+2,y_pos+2);
wire [2:0] aa_count = {2'b00,aa_bit_00} + {2'b00,aa_bit_01} +
                      {2'b00,aa_bit_10} + {2'b00,aa_bit_11};
wire [3:0] icon_alpha = (aa_count==4) ? 4'd12 : (aa_count==3) ? 4'd9 :
                              (aa_count==2) ? 4'd6 : (aa_count==1) ? 4'd3 : 4'd0;
wire in_pattern_box = (x_pos >= PATTERN_X) && (x_pos < PATTERN_X+PATTERN_W) &&
                      (y_pos >= PATTERN_Y) && (y_pos < PATTERN_Y+PATTERN_H);
wire [5:0] rom_x = in_pattern_box ? ((x_pos-PATTERN_X)>>SCALE_SHIFT) : 0;
wire [5:0] rom_y = in_pattern_box ? ((y_pos-PATTERN_Y)>>SCALE_SHIFT) : 0;
wire outline_bit = triangle_bit &&
                   !(pattern_bit_at(x_pos-4,y_pos) && pattern_bit_at(x_pos+4,y_pos) &&
                     pattern_bit_at(x_pos,y_pos-4) && pattern_bit_at(x_pos,y_pos+4));
wire exclamation_bit = triangle_bit &&
                        (((rom_x>=14)&&(rom_x<=17)&&(rom_y>=10)&&(rom_y<=20)) ||
                         ((rom_x>=14)&&(rom_x<=17)&&(rom_y>=24)&&(rom_y<=27)));
wire fringe_bit = (aa_count != 0) && !triangle_bit;
wire [7:0] icon_r = exclamation_bit ? MARK_R : (outline_bit||fringe_bit) ? ORANGE_R : YELLOW_R;
wire [7:0] icon_g = exclamation_bit ? MARK_G : (outline_bit||fringe_bit) ? ORANGE_G : YELLOW_G;
wire [7:0] icon_b = exclamation_bit ? MARK_B : (outline_bit||fringe_bit) ? ORANGE_B : YELLOW_B;
wire shadow_bit = pattern_bit_at(x_pos-SHADOW_OFFSET,y_pos-SHADOW_OFFSET);
wire [7:0] edge_r = (alert_frame==1) ? YELLOW_R : (alert_frame==2) ? ORANGE_R : RED_R;
wire [7:0] edge_g = (alert_frame==1) ? YELLOW_G : (alert_frame==2) ? ORANGE_G : RED_G;
wire [7:0] edge_b = (alert_frame==1) ? YELLOW_B : (alert_frame==2) ? ORANGE_B : RED_B;

// 分三级完成横向边缘、纵向边缘和中央图标混色，缩短像素关键路径。
reg [23:0] s0_data;
reg [3:0] s0_x_alpha,s0_y_alpha,s0_icon_alpha;
reg [7:0] s0_edge_r,s0_edge_g,s0_edge_b,s0_icon_r,s0_icon_g,s0_icon_b;
reg s0_icon_show,s0_shadow_show,s0_de,s0_hs,s0_vs;
reg [23:0] s1_data;
reg [3:0] s1_y_alpha,s1_icon_alpha;
reg [7:0] s1_edge_r,s1_edge_g,s1_edge_b,s1_icon_r,s1_icon_g,s1_icon_b;
reg s1_icon_show,s1_shadow_show,s1_de,s1_hs,s1_vs;
reg [23:0] s2_data;
reg [3:0] s2_icon_alpha;
reg [7:0] s2_icon_r,s2_icon_g,s2_icon_b;
reg s2_icon_show,s2_shadow_show,s2_de,s2_hs,s2_vs;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        s0_data<=0; s0_x_alpha<=0; s0_y_alpha<=0; s0_icon_alpha<=0;
        s0_edge_r<=0; s0_edge_g<=0; s0_edge_b<=0; s0_icon_r<=0; s0_icon_g<=0; s0_icon_b<=0;
        s0_icon_show<=0; s0_shadow_show<=0; s0_de<=0; s0_hs<=0; s0_vs<=0;
        s1_data<=0; s1_y_alpha<=0; s1_icon_alpha<=0;
        s1_edge_r<=0; s1_edge_g<=0; s1_edge_b<=0; s1_icon_r<=0; s1_icon_g<=0; s1_icon_b<=0;
        s1_icon_show<=0; s1_shadow_show<=0; s1_de<=0; s1_hs<=0; s1_vs<=0;
        s2_data<=0; s2_icon_alpha<=0; s2_icon_r<=0; s2_icon_g<=0; s2_icon_b<=0;
        s2_icon_show<=0; s2_shadow_show<=0; s2_de<=0; s2_hs<=0; s2_vs<=0;
        data<=0; de<=0; hs<=0; vs<=0;
    end else begin
        s0_data <= data_in;
        s0_x_alpha <= warning_active ? x_edge_alpha : 0;
        s0_y_alpha <= warning_active ? y_edge_alpha : 0;
        s0_icon_alpha <= icon_alpha;
        s0_edge_r <= edge_r; s0_edge_g <= edge_g; s0_edge_b <= edge_b;
        s0_icon_r <= icon_r; s0_icon_g <= icon_g; s0_icon_b <= icon_b;
        s0_icon_show <= warning_active && icon_visible && (aa_count != 0);
        s0_shadow_show <= warning_active && icon_visible && shadow_bit;
        s0_de <= de_in; s0_hs <= hs_in; s0_vs <= vs_in;

        s1_data <= {alpha_blend_16(s0_data[23:16],s0_edge_r,s0_x_alpha),
                    alpha_blend_16(s0_data[15:8],s0_edge_g,s0_x_alpha),
                    alpha_blend_16(s0_data[7:0],s0_edge_b,s0_x_alpha)};
        s1_y_alpha <= s0_y_alpha; s1_icon_alpha <= s0_icon_alpha;
        s1_edge_r <= s0_edge_r; s1_edge_g <= s0_edge_g; s1_edge_b <= s0_edge_b;
        s1_icon_r <= s0_icon_r; s1_icon_g <= s0_icon_g; s1_icon_b <= s0_icon_b;
        s1_icon_show <= s0_icon_show; s1_shadow_show <= s0_shadow_show;
        s1_de <= s0_de; s1_hs <= s0_hs; s1_vs <= s0_vs;

        s2_data <= {alpha_blend_16(s1_data[23:16],s1_edge_r,s1_y_alpha),
                    alpha_blend_16(s1_data[15:8],s1_edge_g,s1_y_alpha),
                    alpha_blend_16(s1_data[7:0],s1_edge_b,s1_y_alpha)};
        s2_icon_alpha <= s1_icon_alpha;
        s2_icon_r <= s1_icon_r; s2_icon_g <= s1_icon_g; s2_icon_b <= s1_icon_b;
        s2_icon_show <= s1_icon_show; s2_shadow_show <= s1_shadow_show;
        s2_de <= s1_de; s2_hs <= s1_hs; s2_vs <= s1_vs;

        de <= s2_de; hs <= s2_hs; vs <= s2_vs;
        if (!s2_de)
            data <= 24'd0;
        else if (s2_icon_show)
            data <= {alpha_blend_16(s2_data[23:16],s2_icon_r,s2_icon_alpha),
                     alpha_blend_16(s2_data[15:8],s2_icon_g,s2_icon_alpha),
                     alpha_blend_16(s2_data[7:0],s2_icon_b,s2_icon_alpha)};
        else if (s2_shadow_show)
            data <= {alpha_blend_16(s2_data[23:16],8'd0,4'd4),
                     alpha_blend_16(s2_data[15:8],8'd0,4'd4),
                     alpha_blend_16(s2_data[7:0],8'd0,4'd4)};
        else
            data <= s2_data;
    end
end

endmodule
