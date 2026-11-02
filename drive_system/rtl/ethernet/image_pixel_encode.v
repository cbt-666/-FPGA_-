`timescale 1ns / 1ps

// Network-only 8-bit encoding of the selected processed RGB stream.
// 0..253: grayscale, 254: pure red, 255: pure green.
// Binary RGB is black/white, so its network values become 0/253.
// Pure colors come from the YRG processor; no asynchronous mode flag is
// needed here. Combinational encoding adds no video/FIFO timing latency.
module image_pixel_encode(
    input  [23:0] rgb,
    output [7:0]  pixel
);
assign pixel = (rgb == 24'hff0000) ? 8'd254 :
               (rgb == 24'h00ff00) ? 8'd255 :
               (rgb[7:0] > 8'd253) ? 8'd253 : rgb[7:0];
endmodule
