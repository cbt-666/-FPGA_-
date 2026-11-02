`timescale 1ns / 1ps

module tb_voice_event_ctrl;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg [6:0] source_active = 7'd0;
    reg [6:0] source_valid = 7'd0;
    wire uart_txd;
    integer event_count = 0;
    integer track_log [0:15];
    reg [7:0] serial_log [0:479];
    integer serial_count = 0;
    integer i;
    integer bit_i;
    integer before_count;
    integer event_index;
    integer serial_index;

    always #5 clk = ~clk;

    voice_event_ctrl #(
        .CLK_FREQ(10_000_000),
        .UART_BPS(1_000_000),
        .BOOT_MS(0),
        .NIGHT_MS(4),
        .BLUE_MS(4),
        .SINGLE_MS(4),
        .WRONG_MS(4),
        .ZEBRA_MS(4),
        .YELLOW_MS(4),
        .RED_MS(4)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .source_active(source_active),
        .source_valid(source_valid),
        .uart_txd(uart_txd)
    );

    // Record the track selected at the same cycle its command starts.
    always @(posedge clk) begin
        if (rst_n && dut.dispatch) begin
            track_log[event_count] = dut.track;
            event_count = event_count + 1;
        end
    end

    // Decode each 8N1 byte at its bit centers and retain the emitted frame.
    always begin
        @(negedge uart_txd);
        if (rst_n) begin
            repeat (5) @(posedge clk);
            if (uart_txd !== 1'b0) begin
                $display("FAIL: UART start bit is invalid");
                $fatal(1);
            end
            for (bit_i = 0; bit_i < 8; bit_i = bit_i + 1) begin
                repeat (10) @(posedge clk);
                serial_log[serial_count][bit_i] = uart_txd;
            end
            repeat (10) @(posedge clk);
            if (uart_txd !== 1'b1) begin
                $display("FAIL: UART stop bit is invalid");
                $fatal(1);
            end
            serial_count = serial_count + 1;
        end
    end

    task automatic check_frame(input integer first_byte, input integer track);
        reg [79:0] expected_path;
        reg [7:0] expected_sum;
        integer p;
        begin
            expected_path = "/00000*???";
            expected_path[39:32] = 8'h30 + track;
            expected_sum = 8'hAA + 8'h08 + 8'h0B + 8'h02;
            for (p = 0; p < 10; p = p + 1) begin
                expected_sum = expected_sum + expected_path[79-p*8 -: 8];
                if (serial_log[first_byte+4+p] !== expected_path[79-p*8 -: 8])
                    $fatal(1, "Wrong filename byte %0d for track %0d", p, track);
            end
            if (serial_log[first_byte]     != 8'hAA ||
                serial_log[first_byte + 1] != 8'h08 ||
                serial_log[first_byte + 2] != 8'h0B ||
                serial_log[first_byte + 3] != 8'h02 ||
                serial_log[first_byte + 14] != expected_sum) begin
                $display("FAIL: invalid JQ8900 frame for track %0d", track);
                $fatal(1);
            end
        end
    endtask

    task automatic raise_event(input integer source_bit, input integer expected_track);
        begin
            event_index = event_count;
            serial_index = serial_count;
            source_active = 7'b0000001 << source_bit;
            source_valid = source_active;
            wait (event_count == event_index + 1);
            wait (serial_count == serial_index + 15);
            repeat (20) @(posedge clk);
            if (track_log[event_index] != expected_track) begin
                $display("FAIL: source bit %0d selected track %0d, expected %0d",
                         source_bit, track_log[event_index], expected_track);
                $fatal(1);
            end
            check_frame(serial_index, expected_track);
            source_active = 5'd0;
            source_valid = 5'd0;
            repeat (20) @(posedge clk);
        end
    endtask

    initial begin
        for (i = 0; i < 16; i = i + 1)
            track_log[i] = 0;
        for (i = 0; i < 480; i = i + 1)
            serial_log[i] = 0;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        // Night remains active beyond its playback window but must not replay.
        event_index = event_count;
        serial_index = serial_count;
        source_active = 5'b00001;
        source_valid = source_active;
        wait (event_count == event_index + 1);
        repeat (90_000) @(posedge clk);
        if (event_count != event_index + 1 || track_log[event_index] != 2 ||
            serial_count != serial_index + 15) begin
            $display("FAIL: night mapping or one-shot behavior is incorrect");
            $fatal(1);
        end
        check_frame(serial_index, 2);
        source_active = 5'd0;
        source_valid = 5'd0;
        repeat (10) @(posedge clk);

        // 任意方向蓝色箭头首次出现播放音频1；持续显示不重复。
        raise_event(5, 1);
        // 蓝色箭头消失后再次出现，应作为新的出现沿再次播放音频1。
        raise_event(5, 1);

        raise_event(6, 4); // Ethernet single-arrow transition

        raise_event(2, 3); // mechanical wrong direction
        raise_event(1, 5); // Ethernet yellow-edge warning
        raise_event(3, 6); // ultrasonic yellow
        raise_event(4, 7); // ultrasonic red

        // 调度器通用并发回归：蓝色事件优先于待播单箭头事件。
        repeat (90_000) @(negedge clk);
        event_index = event_count;
        serial_index = serial_count;
        source_active = 7'b1100000;
        source_valid = source_active;
        wait (event_count == event_index + 2);
        wait (serial_count == serial_index + 30);
        if (track_log[event_index] != 1 || track_log[event_index + 1] != 4)
            $fatal(1, "Blue/single-arrow simultaneous priority mismatch");
        check_frame(serial_index, 1);
        check_frame(serial_index + 15, 4);
        repeat (90_000) @(negedge clk);
        if (event_count != event_index + 2)
            $fatal(1, "Continuous blue/single arrow replayed");
        source_active = 0; source_valid = 0;
        repeat (20) @(negedge clk);

        // A red warning must start a new command while night audio is active.
        event_index = event_count;
        serial_index = serial_count;
        source_active = 5'b00001;
        source_valid = source_active;
        wait (event_count == event_index + 1);
        repeat (2_000) @(posedge clk);
        source_active = 5'b10000;
        source_valid = source_active;
        wait (event_count == event_index + 2);
        repeat (20) @(posedge clk);
        if (track_log[event_index] != 2 || track_log[event_index + 1] != 7) begin
            $display("FAIL: red warning did not preempt night audio");
            $fatal(1);
        end
        source_active = 5'd0;
        source_valid = 5'd0;
        wait (serial_count == serial_index + 30);
        check_frame(serial_index, 2);
        check_frame(serial_index + 15, 7);

        // Lower-priority pending events that expire must never play later.
        before_count = event_count;
        source_active = 5'b00110;
        source_valid = source_active;
        repeat (100) @(negedge clk);
        source_active = 0; source_valid = 0;
        repeat (90_000) @(negedge clk);
        if (event_count != before_count) $fatal(1, "Expired pending event played");

        // An uninterrupted low-priority command must complete before preemption.
        event_index = event_count; serial_index = serial_count;
        source_active = 5'b00001; source_valid = source_active;
        wait (event_count == event_index + 1);
        repeat (30) @(negedge clk);
        source_active = 5'b10001; source_valid = source_active;
        wait (event_count == event_index + 2);
        wait (serial_count == serial_index + 30);
        check_frame(serial_index, 2); check_frame(serial_index + 15, 7);
        repeat (90_000) @(negedge clk);
        if (event_count != event_index + 2) $fatal(1, "Preempted night replayed");

        $display("PASS: filename map, path/checksum UART frames, no-repeat, pending cancellation, and frame-safe priority preemption");
        $finish;
    end

    initial begin
        #20_000_000;
        $display("FAIL: simulation timeout");
        $fatal(1);
    end
endmodule
