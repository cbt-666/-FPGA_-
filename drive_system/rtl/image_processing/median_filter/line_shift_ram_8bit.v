`timescale 1ns / 1ps

// Two line buffers for the 3x3 window.  Address and pipeline registers are
// reset so every FPGA configuration starts at a known pixel.
module line_shift_ram_8bit(
    input         clock,
    input         rst_n,
    input         clken,
    input         pre_frame_href,
    input  [7:0]  shiftin,
    output [7:0]  taps0x,
    output [7:0]  taps1x
);
reg [10:0] ram_rd_addr;
reg [10:0] ram_rd_addr_d0;
reg        clken_d0;
reg [7:0]  shiftin_d0;

always @(posedge clock or negedge rst_n) begin
    if (!rst_n)
        ram_rd_addr <= 11'd0;
    else if (!pre_frame_href)
        ram_rd_addr <= 11'd0;
    else if (clken)
        ram_rd_addr <= ram_rd_addr + 11'd1;
end

always @(posedge clock or negedge rst_n) begin
    if (!rst_n) begin
        ram_rd_addr_d0 <= 11'd0;
        clken_d0 <= 1'b0;
        shiftin_d0 <= 8'd0;
    end else begin
        ram_rd_addr_d0 <= ram_rd_addr;
        clken_d0 <= clken;
        shiftin_d0 <= shiftin;
    end
end

// Read x first, then write x one clock later. Never depend on undefined
// cross-port read-during-write behaviour at the same RAM address.
ram_2048x8 u_ram_2048x8_0 (
    .wr_data (shiftin_d0), .wr_addr (ram_rd_addr_d0),
    .wr_en   (clken_d0), .wr_clk (clock), .wr_rst (~rst_n),
    .rd_addr (ram_rd_addr), .rd_data (taps0x),
    .rd_clk  (clock), .rd_rst (~rst_n)
);

// taps0x is available one clock after RAM0's read. Its write address and
// enable therefore need exactly one clock of delay, not two/three.
ram_2048x8 u_ram_2048x8_1 (
    .wr_data (taps0x), .wr_addr (ram_rd_addr_d0),
    .wr_en   (clken_d0), .wr_clk (clock), .wr_rst (~rst_n),
    .rd_addr (ram_rd_addr), .rd_data (taps1x),
    .rd_clk  (clock), .rd_rst (~rst_n)
);
endmodule
