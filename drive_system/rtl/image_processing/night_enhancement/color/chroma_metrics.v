`timescale 1ns / 1ps

// Cheap RGB chroma metrics used to keep colored lamps and traffic signals
// from being washed out while suppressing low-light chroma noise.
module chroma_metrics (
    input  [23:0] rgb_in,
    input  [7:0]  y_in,
    output [7:0]  lift_weight_out,
    output reg [7:0] dark_desat_out
);
    reg [7:0] maximum;
    reg [7:0] minimum;
    reg [9:0] span_scaled;
    reg [7:0] lift_weight;
    reg [15:0] dark_product;

    always @* begin
        maximum = (rgb_in[23:16] > rgb_in[15:8]) ? rgb_in[23:16] : rgb_in[15:8];
        maximum = (maximum > rgb_in[7:0]) ? maximum : rgb_in[7:0];
        minimum = (rgb_in[23:16] < rgb_in[15:8]) ? rgb_in[23:16] : rgb_in[15:8];
        minimum = (minimum < rgb_in[7:0]) ? minimum : rgb_in[7:0];

        // Continuous weight: neutral pixels receive 100% of the positive
        // night lift, strongly colored pixels receive at least 25%.
        span_scaled = (maximum - minimum) * 9'd3;
        if (span_scaled >= 10'd764)
            lift_weight = 8'd64;
        else
            lift_weight = 8'd255 - span_scaled[9:2];

        // Dark, weakly colored pixels are most likely sensor/codec chroma
        // noise. The weight falls smoothly with luminance and saturation.
        if ((y_in < 8'd80) && ((maximum - minimum) < 8'd160)) begin
            dark_product = (8'd80 - y_in) * (8'd160 - (maximum - minimum));
            if (dark_product[15:7] > 8'd128)
                dark_desat_out = 8'd128;
            else
                dark_desat_out = dark_product[14:7];
        end else begin
            dark_desat_out = 8'd0;
        end
    end

    assign lift_weight_out = lift_weight;
endmodule
