`timescale 1ns / 1ps

// Common luminance adjustment followed by optional dark-chroma desaturation.
// A common offset preserves hue better than three independent RGB LUTs.
module rgb_adjust (
    input  [23:0] rgb_in,
    input  [7:0]  source_y,
    input  signed [9:0] luma_delta,
    input  [7:0]  highlight_reduction,
    input  [7:0]  dark_desat,
    output [23:0] rgb_out
);
    function [7:0] adjust_channel;
        input [7:0] channel;
        input [7:0] gray_value;
        input [7:0] desat_value;
        reg signed [11:0] value;
        reg signed [12:0] chroma_delta;
        reg signed [12:0] blend_correction;
        begin
            value = $signed({1'b0, channel}) + luma_delta -
                    $signed({4'd0, highlight_reduction});
            if (value < 0)
                value = 0;
            else if (value > 255)
                value = 255;

            // Smoothly blend toward the mapped gray level.  A four-bit
            // fractional coefficient gives sixteen evenly spaced levels,
            // avoiding the large 0/25/50/75/100% jumps that created asphalt
            // colour patches.  Each level is shift/add only; no variable
            // multiplier is placed in the RGB output timing path.
            chroma_delta = $signed({1'b0, gray_value}) - $signed({1'b0, value[7:0]});
            case (desat_value[7:4])
                4'd0:  blend_correction = 13'sd0;
                4'd1:  blend_correction = chroma_delta >>> 4;
                4'd2:  blend_correction = chroma_delta >>> 3;
                4'd3:  blend_correction = (chroma_delta >>> 2) - (chroma_delta >>> 4);
                4'd4:  blend_correction = chroma_delta >>> 2;
                4'd5:  blend_correction = (chroma_delta >>> 2) + (chroma_delta >>> 4);
                4'd6:  blend_correction = (chroma_delta >>> 1) - (chroma_delta >>> 3);
                4'd7:  blend_correction = (chroma_delta >>> 1) - (chroma_delta >>> 4);
                4'd8:  blend_correction = chroma_delta >>> 1;
                4'd9:  blend_correction = (chroma_delta >>> 1) + (chroma_delta >>> 4);
                4'd10: blend_correction = (chroma_delta >>> 1) + (chroma_delta >>> 3);
                4'd11: blend_correction = (chroma_delta >>> 1) +
                                           (chroma_delta >>> 2) - (chroma_delta >>> 4);
                4'd12: blend_correction = (chroma_delta >>> 1) + (chroma_delta >>> 2);
                4'd13: blend_correction = (chroma_delta >>> 1) +
                                           (chroma_delta >>> 2) + (chroma_delta >>> 4);
                4'd14: blend_correction = (chroma_delta >>> 1) +
                                           (chroma_delta >>> 2) + (chroma_delta >>> 3);
                default: blend_correction = chroma_delta;
            endcase
            value = value + blend_correction;
            if (value < 0)
                value = 0;
            else if (value > 255)
                value = 255;
            adjust_channel = value[7:0];
        end
    endfunction

    reg signed [11:0] gray_signed;
    reg [7:0] gray_value;
    always @* begin
        gray_signed = $signed({1'b0, source_y}) + luma_delta -
                      $signed({4'd0, highlight_reduction});
        if (gray_signed < 0)
            gray_value = 8'd0;
        else if (gray_signed > 255)
            gray_value = 8'hff;
        else
            gray_value = gray_signed[7:0];
    end

    assign rgb_out = {
        adjust_channel(rgb_in[23:16], gray_value, dark_desat),
        adjust_channel(rgb_in[15:8],  gray_value, dark_desat),
        adjust_channel(rgb_in[7:0],   gray_value, dark_desat)
    };
endmodule
