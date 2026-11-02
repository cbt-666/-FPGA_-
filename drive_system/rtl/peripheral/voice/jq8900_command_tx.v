// JQ8900 FLASH按文件名播放：AA 08 0B 02 /0000N*??? checksum。
// 路径匹配根目录00001.mp3～00007.mp3，不依赖FLASH内部曲目排序。
// 使用普通路径播放而非0x16插播，抢占后不会恢复旧语音。
module jq8900_command_tx #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer UART_BPS = 9600
)(
    input wire clk, input wire rst_n,
    input wire start, input wire [7:0] track,
    output wire ready, output reg done, output wire txd
);
localparam IDLE=0, ISSUE=1, WAIT_HIGH=2, WAIT_LOW=3;
reg [1:0] state;
reg [3:0] index;
reg [7:0] track_latched;
reg [7:0] checksum;
reg [7:0] byte_data;
wire byte_busy;
assign ready = (state == IDLE);
always @* begin
    case (index)
        0: byte_data = 8'haa;
        1: byte_data = 8'h08;
        2: byte_data = 8'h0b; // 数据长度=FLASH盘符1字节+路径10字节
        3: byte_data = 8'h02;
        4: byte_data = 8'h2f; // /
        5, 6, 7, 8: byte_data = 8'h30; // 0000
        9: byte_data = 8'h30 + track_latched;
        10: byte_data = 8'h2a; // *：手册规定的文件名/扩展名分隔通配符
        11, 12, 13: byte_data = 8'h3f; // ???：按手册格式匹配文件扩展名
        default: byte_data = checksum;
    endcase
end
voice_uart_tx #(.CLK_FREQ(CLK_FREQ), .UART_BPS(UART_BPS)) u_uart(
    .clk(clk), .rst_n(rst_n), .start(state == ISSUE), .data(byte_data),
    .txd(txd), .busy(byte_busy)
);
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE; index <= 0; track_latched <= 0; checksum <= 0; done <= 0;
    end else begin
        done <= 0;
        case (state)
            IDLE: if (start && (track >= 1) && (track <= 7)) begin
                track_latched <= track; index <= 0; checksum <= 0; state <= ISSUE;
            end
            ISSUE: begin
                if (index < 14) checksum <= checksum + byte_data;
                state <= WAIT_HIGH;
            end
            WAIT_HIGH: if (byte_busy) state <= WAIT_LOW;
            WAIT_LOW: if (!byte_busy) begin
                if (index == 14) begin state <= IDLE; done <= 1; end
                else begin index <= index + 1'b1; state <= ISSUE; end
            end
        endcase
    end
end
endmodule
