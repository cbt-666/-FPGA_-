`timescale 1ns/1ps

module tb_ge1_command;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg rec_en_drive = 1'b0;
    reg rec_done_drive = 1'b0;
    reg [7:0] rec_data_drive = 8'd0;
    reg [15:0] rec_len_drive = 16'd0;
    wire [15:0] command_word;
    wire command_toggle;
    reg expected_toggle;

    always #4 clk = ~clk;

    udp_command_decoder dut (
        .clk(clk), .rst_n(rst_n),
        .rec_pkt_done(rec_done_drive),
        .rec_en(rec_en_drive),
        .rec_data(rec_data_drive),
        .rec_byte_num(rec_len_drive),
        .command_word(command_word), .command_toggle(command_toggle),
        .single_arrow_active()
    );

    task drive_byte;
        input [7:0] value;
        input last;
        input [15:0] length;
        begin
            @(negedge clk);
            rec_data_drive = value;
            rec_en_drive = 1'b1;
            rec_done_drive = last;
            rec_len_drive = length;
            @(negedge clk);
            rec_en_drive = 1'b0;
            rec_done_drive = 1'b0;
        end
    endtask

    task send_two;
        input [7:0] low_byte;
        input [7:0] high_byte;
        input [15:0] declared_length;
        begin
            drive_byte(low_byte, 1'b0, declared_length);
            drive_byte(high_byte, 1'b1, declared_length);
            repeat (2) @(negedge clk);
        end
    endtask

    task expect_unchanged;
        input [15:0] previous_word;
        input previous_toggle;
        begin
            if (command_word !== previous_word || command_toggle !== previous_toggle)
                $fatal(1, "Invalid command changed state: word=%h toggle=%b", command_word, command_toggle);
        end
    endtask

    initial begin
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(negedge clk);

        expected_toggle = command_toggle;
        // 0x0839: 左箭头+警告，左箭头黄色，发送图像模式10；低字节39先发。
        send_two(8'h39, 8'h08, 16'd2);
        expected_toggle = ~expected_toggle;
        if (command_word !== 16'h0839 || command_toggle !== expected_toggle)
            $fatal(1, "Valid little-endian command was not accepted");

        send_two(8'haa, 8'h10, 16'd2); // bit15:12非零
        expect_unchanged(16'h0839, expected_toggle);

        send_two(8'haa, 8'h0c, 16'd2); // 图像模式11
        expect_unchanged(16'h0839, expected_toggle);

        send_two(8'haa, 8'h04, 16'd3); // 长度不是2
        expect_unchanged(16'h0839, expected_toggle);

        // 0x0507: 三方向显示，直行绿色，发送图像模式01。
        send_two(8'h07, 8'h05, 16'd2);
        expected_toggle = ~expected_toggle;
        if (command_word !== 16'h0507 || command_toggle !== expected_toggle)
            $fatal(1, "Second valid command was not atomically accepted");

        $display("PASS: GE1 two-byte command parser validation");
        $finish;
    end
endmodule
