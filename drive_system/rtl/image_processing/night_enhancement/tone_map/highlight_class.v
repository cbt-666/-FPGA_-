`timescale 1ns / 1ps

// Classify bright pixels without confusing neutral lamp glare with coloured
// traffic signals. The reduction is a common-luminance offset, so hue is
// preserved by the RGB adjustment stage downstream.
module highlight_class (
    input      [23:0] rgb_in,
    input      [7:0]  y_in,
    input      [7:0]  local_mean_in,
    input      [7:0]  local_max_in,
    input      [7:0]  local_range_in,
    input             stats_valid_in,
    output reg [7:0]  reduction_out,
    output reg [1:0]  class_out
);
    reg [7:0] maximum;
    reg [7:0] minimum;
    reg [7:0] span;
    reg [7:0] reduction;
    reg [7:0] white_reduction;
    reg [7:0] color_reduction;

    always @* begin
        maximum = (rgb_in[23:16] > rgb_in[15:8]) ? rgb_in[23:16] : rgb_in[15:8];
        maximum = (maximum > rgb_in[7:0]) ? maximum : rgb_in[7:0];
        minimum = (rgb_in[23:16] < rgb_in[15:8]) ? rgb_in[23:16] : rgb_in[15:8];
        minimum = (minimum < rgb_in[7:0]) ? minimum : rgb_in[7:0];
        span = maximum - minimum;

        class_out       = 2'd0;
        reduction       = 8'd0;
        white_reduction = 8'd0;
        color_reduction = 8'd0;

        // Neutral bright sources are usually white road lamps or their halo.
        if ((maximum >= 8'd208) && (span <= 8'd56) &&
            ((y_in >= 8'd128) || (local_mean_in >= 8'd144))) begin
            class_out = 2'd1;
            white_reduction = 8'd8 + ((maximum - 8'd208) >> 1);
            if (y_in > 8'd160)
                white_reduction = white_reduction + ((y_in - 8'd160) >> 3);
            if (stats_valid_in && (local_max_in >= 8'd220) &&
                (local_mean_in >= 8'd128) && (local_range_in <= 8'd72) &&
                (y_in >= 8'd96))
                white_reduction = white_reduction + 8'd6;
            if (white_reduction > 8'd56)
                white_reduction = 8'd56;
            reduction = white_reduction;
        end

        // Saturated coloured lamps/signals are compressed through a common
        // offset. This lowers clipping while retaining the dominant colour.
        if ((maximum >= 8'd208) && (span >= 8'd64)) begin
            class_out = 2'd2;
            color_reduction = 8'd12 + ((maximum - 8'd208) >> 1);
            if (y_in > 8'd128)
                color_reduction = color_reduction + ((y_in - 8'd128) >> 4);
            if (color_reduction > 8'd48)
                color_reduction = 8'd48;
            if (color_reduction > reduction)
                reduction = color_reduction;
        end

        reduction_out = reduction;
    end
endmodule
