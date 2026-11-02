`timescale 1ns/1ps

module tb_ultrasonic_warning_overlay;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg de_in = 1'b1;
    reg hs_in = 1'b0;
    reg vs_in = 1'b0;
    reg [23:0] data_in = 24'h202020;
    reg quad_mode_active = 1'b1;
    reg [1:0] alert_level = 2'd1;
    wire de,hs,vs;
    wire [23:0] data;

    always #5 clk = ~clk;

    ultrasonic_warning_overlay u_dut (
        .clk(clk), .rst_n(rst_n),
        .de_in(de_in), .hs_in(hs_in), .vs_in(vs_in), .data_in(data_in),
        .quad_mode_active(quad_mode_active), .alert_level(alert_level),
        .de(de), .hs(hs), .vs(vs), .data(data)
    );

    task latch_frame;
        begin
            repeat (4) @(negedge clk);
            vs_in = 1'b1;
            repeat (2) @(negedge clk);
            vs_in = 1'b0;
            repeat (2) @(negedge clk);
        end
    endtask

    task sample_at;
        input [10:0] x;
        input [10:0] y;
        begin
            force u_dut.x_pos = x;
            force u_dut.y_pos = y;
            repeat (5) @(negedge clk);
        end
    endtask

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        latch_frame();

        sample_at(11'd0,11'd100);
        if (data == data_in || data[23:16] <= data[7:0])
            $fatal(1,"Yellow warning edge must remain visible in quad mode");

        sample_at(11'd640,11'd360);
        if (data == data_in)
            $fatal(1,"128x128 center warning icon must be visible during on phase");

        force u_dut.icon_visible = 1'b0;
        sample_at(11'd640,11'd360);
        if (data != data_in)
            $fatal(1,"Center icon must disappear during blink off phase");
        release u_dut.icon_visible;

        alert_level = 2'd0;
        latch_frame();
        sample_at(11'd0,11'd100);
        if (data != data_in)
            $fatal(1,"Overlay must become transparent after warning clears");

        release u_dut.x_pos;
        release u_dut.y_pos;
        $display("PASS: ultrasonic quad overlay edge, center icon and blink phase");
        $finish;
    end
endmodule
