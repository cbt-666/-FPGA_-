`timescale 1ns / 1ps

// Piecewise night curve and frame-selected lift strength.
module night_tone_curve (
    input  [7:0] y_in,
    input  [1:0] scene_mode,
    output reg [7:0] y_strong,
    output reg [7:0] y_mapped
);
    reg [15:0] value;
    reg [15:0] delta;
    reg [19:0] scaled_delta;
    reg [8:0] positive_delta;
    reg [9:0] adaptive_value;

    always @* begin
        if (y_in < 8'd16) begin
            // 29*x = 16*x + 8*x + 4*x + x; shift/add avoids a DSP multiplier.
            scaled_delta = ({12'd0, y_in} << 4) +
                           ({12'd0, y_in} << 3) +
                           ({12'd0, y_in} << 2) +
                           {12'd0, y_in};
            value = scaled_delta >> 3;
        end
        else if (y_in < 8'd32) begin
            delta = {8'd0, y_in} - 16'd16;
            value = 16'd58 + (delta << 1);
        end else if (y_in < 8'd64) begin
            delta = {8'd0, y_in} - 16'd32;
            // 19*x = 16*x + 2*x + x.
            scaled_delta = (delta << 4) + (delta << 1) + delta;
            value = 16'd90 + (scaled_delta >> 4);
        end else if (y_in < 8'd96) begin
            delta = {8'd0, y_in} - 16'd64;
            // 31*x = 32*x - x.
            scaled_delta = (delta << 5) - delta;
            value = 16'd128 + (scaled_delta >> 5);
        end else if (y_in < 8'd128) begin
            delta = {8'd0, y_in} - 16'd96;
            // 11*x = 8*x + 2*x + x.
            scaled_delta = (delta << 3) + (delta << 1) + delta;
            value = 16'd159 + (scaled_delta >> 4);
        end else if (y_in < 8'd160) begin
            delta = {8'd0, y_in} - 16'd128;
            // 17*x = 16*x + x.
            scaled_delta = (delta << 4) + delta;
            value = 16'd181 + (scaled_delta >> 5);
        end else if (y_in < 8'd192) begin
            delta = {8'd0, y_in} - 16'd160;
            // 12*x = 8*x + 4*x.
            scaled_delta = (delta << 3) + (delta << 2);
            value = 16'd198 + (scaled_delta >> 5);
        end else if (y_in < 8'd224) begin
            delta = {8'd0, y_in} - 16'd192;
            // 18*x = 16*x + 2*x.
            scaled_delta = (delta << 4) + (delta << 1);
            value = 16'd210 + (scaled_delta >> 5);
        end else begin
            delta = {8'd0, y_in} - 16'd224;
            value = 16'd228 + (delta >> 1);
        end

        y_strong = (value > 16'd255) ? 8'hff : value[7:0];

        if (y_strong <= y_in) begin
            y_mapped = y_strong;
        end else begin
            positive_delta = {1'b0, y_strong} - {1'b0, y_in};
            case (scene_mode)
                2'd1: adaptive_value = {2'b00, y_in} + positive_delta -
                                       (positive_delta >> 2);
                2'd2: adaptive_value = {2'b00, y_in} + (positive_delta >> 1);
                default: adaptive_value = {2'b00, y_strong};
            endcase
            y_mapped = (adaptive_value > 10'd255) ? 8'hff : adaptive_value[7:0];
        end
    end
endmodule
