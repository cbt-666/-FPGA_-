`timescale 1ns/1ps

module tb_ultrasonic;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg echo = 1'b0;
    wire [1:0] alert_level;

    always #10 clk = ~clk;

    ultrasonic_echo #(
        .LEVEL3_MAX_CYCLES(10),
        .LEVEL2_MAX_CYCLES(15),
        .LEVEL1_MAX_CYCLES(20),
        .ECHO_TIMEOUT_CYCLES(24),
        .STALE_CLEAR_CYCLES(30)
    ) u_dut (
        .sys_clk(clk),
        .sys_rst_n(rst_n),
        .echo(echo),
        .alert_level(alert_level)
    );

    task send_echo;
        input integer cycles;
        begin
            @(negedge clk); echo = 1'b1;
            repeat (cycles) @(negedge clk);
            echo = 1'b0;
            repeat (5) @(negedge clk);
        end
    endtask

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        repeat (3) @(negedge clk);

        send_echo(6);
        if (alert_level != 2'd3) $fatal(1,"Near echo must select level 3");
        send_echo(12);
        if (alert_level != 2'd2) $fatal(1,"Middle echo must select level 2");
        send_echo(17);
        if (alert_level != 2'd1) $fatal(1,"Far warning echo must select level 1");
        send_echo(21);
        if (alert_level != 2'd0) $fatal(1,"Echo beyond 20 cm must clear warning");

        send_echo(6);
        if (alert_level != 2'd3) $fatal(1,"Warning setup failed before stale test");
        repeat (32) @(negedge clk);
        if (alert_level != 2'd0) $fatal(1,"Missing echo must clear stale warning");

        send_echo(6);
        @(negedge clk); echo = 1'b1;
        repeat (30) @(negedge clk);
        if (alert_level != 2'd0) $fatal(1,"Echo stuck high must time out and clear warning");
        echo = 1'b0;

        $display("PASS: ultrasonic distance levels and timeout clearing");
        $finish;
    end
endmodule
