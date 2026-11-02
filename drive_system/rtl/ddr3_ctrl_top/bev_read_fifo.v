// 256-bit DDR BEV words to 16-bit HDMI pixels, with a frame-boundary flush.
module bev_read_fifo(
    input  wire         rst_n,
    input  wire         ui_clk,
    input  wire         pixel_clk,
    input  wire         frame_reset,
    input  wire         wr_en,
    input  wire [255:0] wr_data,
    output wire [10:0]  wr_count,
    input  wire         rd_en,
    output wire [15:0]  rd_data,
    output wire         rd_empty,
    output wire         rd_valid,
    output reg          rd_fault
);
reg frame_meta;
reg [15:0] reset_shift;
wire fifo_reset = ~rst_n | (reset_shift[0] && !reset_shift[14]);
wire fifo_full;

// rd_valid is the actual read acceptance condition.  The FIFO suppresses
// rd_en internally while empty; exporting the same condition prevents the
// HDMI path from treating a held/stale rd_data value as a new pixel.
assign rd_valid = rd_en && !rd_empty;

always @(posedge ui_clk or negedge rst_n) begin
    if (!rst_n) begin
        frame_meta<=0;
        reset_shift<=0;
    end else begin
        frame_meta<=frame_reset;
        reset_shift<={reset_shift[14:0],frame_meta};
    end
end

// Once the display FIFO underflows, the current raster position no longer
// corresponds to the next queued word. Hold a fault until the next frame
// reset instead of resuming mid-frame with a horizontally shifted image.
always @(posedge pixel_clk or negedge rst_n) begin
    if (!rst_n)
        rd_fault <= 1'b0;
    else if (fifo_reset)
        rd_fault <= 1'b0;
    else if (rd_en && rd_empty)
        rd_fault <= 1'b1;
end

rd_fifo u_bev_rd_fifo(
    .wr_clk(ui_clk), .wr_rst(fifo_reset),
    .wr_en(wr_en && !fifo_full), .wr_data(wr_data), .wr_full(fifo_full),
    .wr_water_level(wr_count), .almost_full(),
    .rd_clk(pixel_clk), .rd_rst(fifo_reset),
    .rd_en(rd_en && !rd_empty), .rd_data(rd_data),
    .rd_empty(rd_empty), .rd_water_level(), .almost_empty()
);
endmodule
