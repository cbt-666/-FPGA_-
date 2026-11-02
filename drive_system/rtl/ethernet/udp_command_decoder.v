// GE1 UDP二字节命令解析器。
// 载荷按小端顺序接收：先[7:0]、后[15:8]；非法包保持上一条有效命令。
module udp_command_decoder(
    input             clk,
    input             rst_n,
    input             rec_pkt_done,
    input             rec_en,
    input      [7:0]  rec_data,
    input      [15:0] rec_byte_num,
    output reg [15:0] command_word,
    output reg        command_toggle,
    output reg        single_arrow_active
);

reg [7:0] low_byte;
reg [1:0] byte_count;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        command_word   <= 16'd0;
        command_toggle <= 1'b0;
        single_arrow_active <= 1'b0;
        low_byte       <= 8'd0;
        byte_count     <= 2'd0;
    end else if (rec_pkt_done) begin
        // udp_rx在最后一个载荷字节同时拉高rec_en和rec_pkt_done。
        if (rec_en && (rec_byte_num == 16'd2) && (byte_count == 2'd1) &&
            (rec_data[7:4] == 4'd0) && (rec_data[3:2] != 2'b11)) begin
            command_word   <= {rec_data, low_byte};
            command_toggle <= ~command_toggle;
            // 第4段仅用于红/绿/黄单箭头；蓝色只保留原有第1段事件。
            // 边缘警告不计入数量，连续合格单箭头换方向/颜色不重新触发。
            case (low_byte[2:0])
                3'b001: single_arrow_active <= (low_byte[5:4] != 2'b10);
                3'b010: single_arrow_active <= (low_byte[7:6] != 2'b10);
                3'b100: single_arrow_active <= (rec_data[1:0] != 2'b10);
                default: single_arrow_active <= 1'b0;
            endcase
        end
        low_byte   <= 8'd0;
        byte_count <= 2'd0;
    end else if (rec_en) begin
        if (byte_count == 2'd0)
            low_byte <= rec_data;
        if (byte_count != 2'd3)
            byte_count <= byte_count + 1'b1;
    end
end

endmodule
