// 9600 8N1串口发送器：使用系统时钟使能，不产生新的时钟域。
// busy保持到完整停止位结束；与参考工程相同，数据最低位先发。
module voice_uart_tx #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer UART_BPS = 9600
)(
    input wire clk, input wire rst_n,
    input wire start, input wire [7:0] data,
    output wire txd, output reg busy
);
localparam integer BIT_CYCLES = CLK_FREQ / UART_BPS;
reg [15:0] baud_count;
reg [3:0] bit_index;
reg [9:0] shift_data;
assign txd = busy ? shift_data[0] : 1'b1;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        baud_count <= 0; bit_index <= 0; shift_data <= 10'h3ff; busy <= 0;
    end else if (!busy) begin
        if (start) begin
            shift_data <= {1'b1, data, 1'b0};
            baud_count <= 0; bit_index <= 0; busy <= 1;
        end
    end else if (baud_count == BIT_CYCLES - 1) begin
        baud_count <= 0;
        shift_data <= {1'b1, shift_data[9:1]};
        if (bit_index == 9) busy <= 0;
        else bit_index <= bit_index + 1'b1;
    end else baud_count <= baud_count + 1'b1;
end
endmodule
