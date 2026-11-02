`timescale 1ns / 1ps

// Convert registered 3x3 statistics into a smooth, hardware-friendly lift
// weight.  Sixteen small gain steps avoid the visible 1/2 and 1/4 boundaries
// produced by hard local-range thresholds on asphalt.
module local_gain_weight (
    input      [7:0] y_in,
    input      [7:0] local_mean_in,
    input      [7:0] local_max_in,
    input      [7:0] local_range_in,
    input            stats_valid_in,
    output reg [3:0] weight_out
);
    reg [4:0] range_attenuation;
    reg [2:0] mean_attenuation;
    reg [1:0] peak_attenuation;
    reg [2:0] bright_attenuation;
    reg [6:0] total_attenuation;

    always @* begin
        range_attenuation = 5'd0;
        mean_attenuation  = 3'd0;
        peak_attenuation   = 2'd0;
        bright_attenuation = 3'd0;
        total_attenuation  = 7'd0;
        weight_out        = 4'd15;

        if (stats_valid_in) begin
            // One 1/16 step for each eight codes of local range above 16.
            // Limit the range term so even strong edges retain a small,
            // continuous amount of night lift.
            if (local_range_in > 8'd16) begin
                range_attenuation = (local_range_in - 8'd16) >> 3;
                if (range_attenuation > 5'd10)
                    range_attenuation = 5'd10;
            end

            // Bright neighbourhoods and peaks add only small 1/16 steps;
            // this protects signs, lanes and lamp halos without producing a
            // new hard half-gain boundary.
            if ((y_in > 8'd128) && (local_mean_in > 8'd160)) begin
                mean_attenuation = (local_mean_in - 8'd160) >> 5;
                if (mean_attenuation > 3'd2)
                    mean_attenuation = 3'd2;
            end
            if ((y_in > 8'd144) && (local_max_in > 8'd208)) begin
                peak_attenuation = (local_max_in - 8'd208) >> 5;
                if (peak_attenuation > 2'd1)
                    peak_attenuation = 2'd1;
            end

            // A broad bright neighbourhood is usually lamp glare rather than
            // useful road texture. Reduce the local lift slightly more only
            // when the neighbourhood is reasonably flat; this preserves lane
            // edges and vehicle contours.
            if ((local_max_in >= 8'd232) && (local_mean_in >= 8'd144) &&
                (local_range_in <= 8'd72))
                bright_attenuation = 3'd2;
            else if ((local_max_in >= 8'd220) && (local_mean_in >= 8'd128) &&
                     (local_range_in <= 8'd48))
                bright_attenuation = 3'd1;

            total_attenuation = range_attenuation +
                                mean_attenuation + peak_attenuation +
                                bright_attenuation;
            if (total_attenuation >= 7'd13)
                weight_out = 4'd2;
            else
                weight_out = 4'd15 - total_attenuation[3:0];
        end
    end
endmodule
