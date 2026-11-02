`timescale 1ns / 1ps

// Smooth soft-knee highlight compression. It uses the original luminance so
// a dark pixel lifted by the night curve cannot accidentally trigger the knee.
module highlight_knee (
    input  [23:0] rgb_in,
    input  [7:0]  y_in,
    output [7:0] reduction_out
);
    reg [7:0] maximum;
    reg [7:0] minimum;
    reg [7:0] span;
    reg [7:0] distance;
    reg [15:0] square;
    reg [8:0] soft_knee_sum;
    reg [7:0] y_reduction;
    reg [7:0] color_reduction;
    reg [7:0] reduction;

    always @* begin
        maximum = (rgb_in[23:16] > rgb_in[15:8]) ? rgb_in[23:16] : rgb_in[15:8];
        maximum = (maximum > rgb_in[7:0]) ? maximum : rgb_in[7:0];
        minimum = (rgb_in[23:16] < rgb_in[15:8]) ? rgb_in[23:16] : rgb_in[15:8];
        minimum = (minimum < rgb_in[7:0]) ? minimum : rgb_in[7:0];
        span = maximum - minimum;

        // White highlights still use luminance.  This keeps lane markings
        // and neutral objects on the same soft knee as before.
        if (y_in <= 8'd148) begin
            y_reduction = 8'd0;
        end else begin
            distance  = y_in - 8'd148;
            square    = distance * distance;
            // Start the white-light knee slightly earlier. The quadratic
            // term stays gentle in road mid-tones and grows near clipping.
            soft_knee_sum = {1'b0, (distance >> 2)} +
                            {1'b0, square[15:7]};
            if (soft_knee_sum > 9'd72)
                y_reduction = 8'd72;
            else
                y_reduction = soft_knee_sum[7:0];
        end

        // Saturated coloured lamps can have modest Y (red and blue in
        // particular), so add a separate max-channel knee only when the
        // colour span indicates a real coloured source.  A common reduction
        // preserves the lamp hue while lowering its clipped peak.
        if ((maximum >= 8'd208) && (span >= 8'd64))
            color_reduction = 8'd18 + ((maximum - 8'd208) >> 1);
        else
            color_reduction = 8'd0;

        if (color_reduction > 8'd52)
            color_reduction = 8'd52;

        reduction = (y_reduction > color_reduction) ? y_reduction : color_reduction;
    end

    assign reduction_out = reduction;
endmodule
