`timescale 1ns/1ps

// 验证UDP字节流跨包空隙保持高/低字节相位，并在完整640x360帧后置位。
module tb_udp_data_rgb565;
reg clk = 1'b0;
reg rst_n = 1'b0;
reg rec_en = 1'b0;
reg [7:0] rec_data = 8'd0;
wire picture_data_vld;
wire frame_valid;
wire [15:0] picture_data;
integer pixel_seen = 0;
integer i;

always #4 clk = ~clk;

udp_data dut(
    .clk(clk), .rst_n(rst_n), .rec_en(rec_en), .rec_data(rec_data),
    .picture_data_vld(picture_data_vld), .frame_valid(frame_valid),
    .picture_data(picture_data)
);

always @(posedge clk) begin
    #1;
    if (picture_data_vld) begin
        pixel_seen = pixel_seen + 1;
        if (pixel_seen == 1 && picture_data !== 16'h1234)
            $fatal(1, "FAIL: first RGB565 pixel mismatch");
        if (pixel_seen == 2 && picture_data !== 16'hABCD)
            $fatal(1, "FAIL: byte phase was lost across UDP gap");
    end
end

task send_byte;
    input [7:0] value;
    input integer gap_cycles;
    integer gap;
    begin
        @(negedge clk); rec_en = 1'b1; rec_data = value;
        @(negedge clk); rec_en = 1'b0;
        for (gap=0; gap<gap_cycles; gap=gap+1)
            @(negedge clk);
    end
endtask

initial begin
    repeat(4) @(posedge clk);
    rst_n = 1'b1;

    send_byte(8'h12, 2);
    send_byte(8'h34, 3);
    send_byte(8'hAB, 1);
    send_byte(8'hCD, 4);

    // 补足一帧；每个字节之间保留一个空拍，模拟多个UDP包之间的间断。
    for (i=2; i<230400; i=i+1) begin
        send_byte(i[7:0], 0);
        send_byte(~i[7:0], 0);
    end

    wait(pixel_seen == 230400);
    @(posedge clk); #1;
    if (!frame_valid)
        $fatal(1, "FAIL: frame_valid did not assert after 230400 pixels");
    $display("PASS: UDP RGB565 assembly and full-frame qualification");
    $finish;
end

initial begin
    #10000000;
    $fatal(1, "FAIL: simulation timeout");
end
endmodule
