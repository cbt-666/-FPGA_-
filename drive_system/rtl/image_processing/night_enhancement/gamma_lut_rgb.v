`timescale 1ns / 1ps

// Stable integration wrapper for the HDMI night-enhancement pipeline.
// Processing functions are split by role under image_proc/color, tone_map and
// chroma and local. The external interface remains unchanged; the local
// window adds one cycle before the tone-map and RGB stages.  Tone-map
// decisions are split across two registered stages to keep the 74.25 MHz
// path within timing while retaining one-pixel-per-clock throughput.  A
// frame-level day/night gate bypasses the enhancement during daytime without
// changing the pixel latency.
module gamma_lut_rgb (
    input              pclk,
    input              rst_n,
    input              de_in,
    input              hs_in,
    input              vs_in,
    input      [23:0]  rgb_in,
    output reg         de_out,
    output reg         hs_out,
    output reg         vs_out,
    output reg [23:0]  rgb_out,
    output reg         night_active
);
    reg        s1_de;
    reg        s1_hs;
    reg        s1_vs;
    reg [23:0] s1_rgb;
    reg [7:0]  s1_y;
    reg [7:0]  s1_local_mean;
    reg [7:0]  s1_local_max;
    reg [7:0]  s1_local_range;
    reg        s1_local_valid;
    reg [7:0]  s1_halo_reduction;
    reg        s1_halo_valid;

    reg        s2_de;
    reg        s2_hs;
    reg        s2_vs;
    reg [23:0] s2_rgb;
    reg [7:0]  s2_y;
    reg signed [9:0] s2_base_delta;
    reg [7:0]  s2_lift_weight;
    reg [3:0]  s2_local_gain_weight;
    reg [7:0]  s2_highlight_reduction;
    reg [7:0]  s2_dark_desat;

    reg        s3_de;
    reg        s3_hs;
    reg        s3_vs;
    reg [23:0] s3_rgb;
    reg [7:0]  s3_y;
    reg signed [9:0] s3_luma_delta;
    reg [7:0]  s3_highlight_reduction;
    reg [7:0]  s3_dark_desat;

    reg        vs_in_d;
    reg [3:0]  sample_divider;
    reg [31:0] luma_sum;
    reg [19:0] sample_count;
    reg [19:0] bright_count;
    reg [1:0]  scene_mode;
    reg        night_enable;
    reg [1:0]  night_confirm_count;
    reg [3:0]  gain_smooth_state;

    wire [7:0] input_y;
    wire        proc_de;
    wire        proc_hs;
    wire        proc_vs;
    wire [23:0] proc_rgb;
    wire [7:0]  proc_local_mean;
    wire [7:0]  proc_local_max;
    wire [7:0]  proc_local_range;
    wire        proc_local_valid;
    reg  [7:0]  proc_halo_reduction;
    reg         proc_halo_valid;
    wire [7:0]  proc_rgb_peak;
    wire [7:0] s1_y_strong;
    wire [7:0] s1_y_mapped;
    wire [7:0] s1_lift_weight;
    wire [7:0] s1_highlight_reduction;
    wire [7:0] s1_class_reduction;
    wire [1:0] s1_highlight_class;
    wire [7:0] s1_dark_desat_next;
    wire [3:0] s1_local_gain_weight;
    wire signed [9:0] s1_base_delta;
    reg signed [11:0] weighted_product;
    reg signed [11:0] local_product;
    reg signed [9:0] s2_limited_delta;
    wire [23:0] adjusted_rgb;

    // Limit gain movement in the pixel direction. Bright regions react
    // quickly, while recovery is slower so flat asphalt does not become a
    // sequence of visible local patches.
    function [3:0] smooth_gain;
        input [3:0] target;
        input [3:0] previous;
        begin
            if (target < previous) begin
                if ((previous - target) > 4'd2)
                    smooth_gain = previous - 4'd2;
                else
                    smooth_gain = target;
            end else if (target > previous) begin
                if ((target - previous) > 4'd1)
                    smooth_gain = previous + 4'd1;
                else
                    smooth_gain = target;
            end
            else
                smooth_gain = target;
        end
    endfunction
    wire [3:0] s1_gain_smooth = smooth_gain(s1_local_gain_weight,
                                            gain_smooth_state);

    function [7:0] max3_u8;
        input [7:0] a;
        input [7:0] b;
        input [7:0] c;
        reg [7:0] ab;
        begin
            ab = (a > b) ? a : b;
            max3_u8 = (ab > c) ? ab : c;
        end
    endfunction
    wire [7:0] s1_halo_reduction_used = s1_halo_valid ?
                                         s1_halo_reduction : 8'd0;
    wire [7:0] s1_combined_reduction = max3_u8(s1_highlight_reduction,
                                                s1_halo_reduction_used,
                                                s1_class_reduction);

    local_luma_3x3 u_local_luma_3x3 (
        .pclk            (pclk),
        .rst_n           (rst_n),
        .de_in           (de_in),
        .hs_in           (hs_in),
        .vs_in           (vs_in),
        .rgb_in          (rgb_in),
        .de_out          (proc_de),
        .hs_out          (proc_hs),
        .vs_out          (proc_vs),
        .rgb_out         (proc_rgb),
        .local_mean_out  (proc_local_mean),
        .local_max_out   (proc_local_max),
        .local_range_out (proc_local_range),
        .stats_valid_out (proc_local_valid)
    );

    // Timing-safe halo estimate from the registered 3x3 statistics.  The
    // local maximum and mean provide a bounded bright-neighbourhood term with
    // no extra RAM or long max/count reduction tree in the HDMI path.
    assign proc_rgb_peak = (proc_rgb[23:16] > proc_rgb[15:8]) ?
                           ((proc_rgb[23:16] > proc_rgb[7:0]) ? proc_rgb[23:16] : proc_rgb[7:0]) :
                           ((proc_rgb[15:8] > proc_rgb[7:0]) ? proc_rgb[15:8] : proc_rgb[7:0]);

    always @* begin
        proc_halo_reduction = 8'd0;
        proc_halo_valid     = proc_local_valid;
        if (proc_rgb_peak >= 8'd208)
            proc_halo_reduction = 8'd12 + ((proc_rgb_peak - 8'd208) >> 1);
        if (proc_local_valid && (proc_local_max >= 8'd220) &&
            (proc_local_mean >= 8'd128) && (input_y >= 8'd96) &&
            (proc_local_max > (input_y + 8'd16))) begin
            if ((((proc_local_max - input_y) >> 2) +
                 ((proc_local_mean - 8'd128) >> 3)) > proc_halo_reduction)
                proc_halo_reduction = ((proc_local_max - input_y) >> 2) +
                                      ((proc_local_mean - 8'd128) >> 3);
        end
        if (proc_halo_reduction > 8'd48)
            proc_halo_reduction = 8'd48;
    end

    rgb_luma u_rgb_luma_in (
        .rgb_in (proc_rgb),
        .y_out  (input_y)
    );

    night_tone_curve u_night_tone_curve (
        .y_in       (s1_y),
        .scene_mode (scene_mode),
        .y_strong   (s1_y_strong),
        .y_mapped   (s1_y_mapped)
    );

    chroma_metrics u_chroma_metrics (
        .rgb_in          (s1_rgb),
        .y_in            (s1_y),
        .lift_weight_out (s1_lift_weight),
        .dark_desat_out  (s1_dark_desat_next)
    );

    highlight_knee u_highlight_knee (
        .rgb_in        (s1_rgb),
        .y_in          (s1_y),
        .reduction_out (s1_highlight_reduction)
    );

    highlight_class u_highlight_class (
        .rgb_in          (s1_rgb),
        .y_in            (s1_y),
        .local_mean_in   (s1_local_mean),
        .local_max_in    (s1_local_max),
        .local_range_in  (s1_local_range),
        .stats_valid_in  (s1_local_valid),
        .reduction_out   (s1_class_reduction),
        .class_out       (s1_highlight_class)
    );

    local_gain_weight u_local_gain_weight (
        .y_in             (s1_y),
        .local_mean_in    (s1_local_mean),
        .local_max_in     (s1_local_max),
        .local_range_in   (s1_local_range),
        .stats_valid_in   (s1_local_valid),
        .weight_out       (s1_local_gain_weight)
    );

    assign s1_base_delta = $signed({2'b00, s1_y_mapped}) -
                           $signed({2'b00, s1_y});

    // Continuous chroma weighting removes hard spatial boundaries that could
    // appear as patches on asphalt while still protecting colored lamps.
    always @* begin
        weighted_product = s2_base_delta;
        local_product    = s2_base_delta;
        s2_limited_delta = s2_base_delta;

        // Quantized lift ratios replace a variable multiplier on the timing
        // path. The chroma metric is still continuous; only the final gain is
        // rounded to four hardware-friendly levels.
        if (s2_base_delta > 0) begin
            // More gain levels reduce spatial chroma banding on asphalt.
            if (s2_lift_weight >= 8'd224)
                weighted_product = s2_base_delta;
            else if (s2_lift_weight >= 8'd192)
                weighted_product = s2_base_delta - (s2_base_delta >>> 3);
            else if (s2_lift_weight >= 8'd160)
                weighted_product = (s2_base_delta + (s2_base_delta << 1)) >>> 2;
            else if (s2_lift_weight >= 8'd128)
                weighted_product = (s2_base_delta + (s2_base_delta >>> 2)) >>> 1;
            else if (s2_lift_weight >= 8'd96)
                weighted_product = s2_base_delta >>> 1;
            else if (s2_lift_weight >= 8'd64)
                weighted_product = (s2_base_delta + s2_base_delta) >>> 3;
            else
                weighted_product = s2_base_delta >>> 2;
        end

        // Apply the registered 3x3 local weight in sixteen small steps.  The
        // shift/add ratios keep the datapath multiplier-free and eliminate
        // the old abrupt 1/2 and 1/4 road-surface gain transitions.
        local_product = weighted_product;
        if (weighted_product > 0) begin
            case (s2_local_gain_weight)
                4'd15: local_product = weighted_product - (weighted_product >>> 4);
                4'd14: local_product = weighted_product - (weighted_product >>> 3);
                4'd13: local_product = weighted_product - (weighted_product >>> 2) +
                                       (weighted_product >>> 4);
                4'd12: local_product = weighted_product - (weighted_product >>> 2);
                4'd11: local_product = (weighted_product >>> 1) +
                                       (weighted_product >>> 2) -
                                       (weighted_product >>> 4);
                4'd10: local_product = (weighted_product >>> 1) +
                                       (weighted_product >>> 3);
                4'd9:  local_product = (weighted_product >>> 1) +
                                       (weighted_product >>> 4);
                4'd8:  local_product = weighted_product >>> 1;
                4'd7:  local_product = (weighted_product >>> 1) -
                                       (weighted_product >>> 4);
                4'd6:  local_product = (weighted_product >>> 1) -
                                       (weighted_product >>> 3);
                4'd5:  local_product = (weighted_product >>> 2) +
                                       (weighted_product >>> 4);
                4'd4:  local_product = weighted_product >>> 2;
                default: local_product = (weighted_product >>> 2) -
                                         (weighted_product >>> 4);
            endcase
        end
        s2_limited_delta = local_product[9:0];
    end

    rgb_adjust u_rgb_adjust (
        .rgb_in              (s3_rgb),
        .source_y            (s3_y),
        .luma_delta          (s3_luma_delta),
        .highlight_reduction (s3_highlight_reduction),
        .dark_desat          (s3_dark_desat),
        .rgb_out             (adjusted_rgb)
    );

    // One sparse sample per 16 active pixels gives a stable scene class with
    // very small state. A bright lamp cannot dominate the decision by itself.
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            s1_de                  <= 1'b0;
            s1_hs                  <= 1'b0;
            s1_vs                  <= 1'b0;
            s1_rgb                 <= 24'h000000;
            s1_y                   <= 8'd0;
            s1_local_mean         <= 8'd0;
            s1_local_max          <= 8'd0;
            s1_local_range        <= 8'd0;
            s1_local_valid        <= 1'b0;
            s1_halo_reduction     <= 8'd0;
            s1_halo_valid         <= 1'b0;

            s2_de                  <= 1'b0;
            s2_hs                  <= 1'b0;
            s2_vs                  <= 1'b0;
            s2_rgb                 <= 24'h000000;
            s2_y                   <= 8'd0;
            s2_base_delta          <= 10'sd0;
            s2_lift_weight         <= 8'd0;
            s2_local_gain_weight   <= 4'd15;
            s2_highlight_reduction <= 8'd0;
            s2_dark_desat          <= 8'd0;

            s3_de                  <= 1'b0;
            s3_hs                  <= 1'b0;
            s3_vs                  <= 1'b0;
            s3_rgb                 <= 24'h000000;
            s3_y                   <= 8'd0;
            s3_luma_delta          <= 10'sd0;
            s3_highlight_reduction <= 8'd0;
            s3_dark_desat          <= 8'd0;

            vs_in_d                <= 1'b0;
            sample_divider         <= 4'd0;
            luma_sum               <= 32'd0;
            sample_count           <= 20'd0;
            bright_count           <= 20'd0;
            scene_mode             <= 2'd0;
            night_enable           <= 1'b0;
            night_confirm_count    <= 2'd0;
            gain_smooth_state      <= 4'd15;

            de_out                <= 1'b0;
            hs_out                <= 1'b0;
            vs_out                <= 1'b0;
            rgb_out               <= 24'h000000;
            night_active          <= 1'b0;
        end else begin
            vs_in_d <= proc_vs;

            if (proc_vs && !vs_in_d) begin
                if (sample_count >= 20'd64) begin
                    if (luma_sum < ({12'd0, sample_count} << 6))
                        scene_mode <= 2'd0;
                    else if (luma_sum < (({12'd0, sample_count} << 6) +
                                        ({12'd0, sample_count} << 5)))
                        scene_mode <= 2'd1;
                    else
                        scene_mode <= 2'd2;

                    // Use the clipped frame-luma statistic plus the fraction
                    // of mid/high-luma pixels as a hysteretic day/night
                    // detector.  A dark road or a backlit daytime image can
                    // have a low mean, so mean alone is not sufficient.  The
                    // bright-pixel ratio rejects that false-night case while
                    // still ignoring a few lamps or headlamps.
                    if (!night_enable) begin
                        if ((luma_sum <= (({12'd0, sample_count} << 6) +
                                          ({12'd0, sample_count} << 2))) &&
                            (bright_count <= (sample_count >> 4))) begin
                            if (night_confirm_count >= 2'd1) begin
                                night_enable        <= 1'b1;
                                night_confirm_count <= 2'd0;
                            end else begin
                                night_confirm_count <= night_confirm_count + 1'b1;
                            end
                        end else begin
                            night_confirm_count <= 2'd0;
                        end
                    end else begin
                        if ((luma_sum >= (({12'd0, sample_count} << 6) +
                                          ({12'd0, sample_count} << 5) +
                                          ({12'd0, sample_count} << 4))) ||
                            (bright_count >= (sample_count >> 3))) begin
                            if (night_confirm_count >= 2'd1) begin
                                night_enable        <= 1'b0;
                                night_confirm_count <= 2'd0;
                            end else begin
                                night_confirm_count <= night_confirm_count + 1'b1;
                            end
                        end else begin
                            night_confirm_count <= 2'd0;
                        end
                    end
                end
                sample_divider <= 4'd0;
                luma_sum       <= 32'd0;
                sample_count   <= 20'd0;
                bright_count   <= 20'd0;
            end else if (proc_de) begin
                sample_divider <= sample_divider + 1'b1;
                if (sample_divider == 4'd0) begin
                    // Clip rare lamp samples before scene classification so
                    // one bright source cannot switch the whole frame mode.
                    if (input_y > 8'd192)
                        luma_sum <= luma_sum + 32'd192;
                    else
                        luma_sum <= luma_sum + {24'd0, input_y};
                    sample_count <= sample_count + 1'b1;
                    if (input_y >= 8'd128)
                        bright_count <= bright_count + 1'b1;
                end
            end

            // Stage 1: capture RGB and derive input luminance.
            s1_de <= proc_de;
            s1_hs <= proc_hs;
            s1_vs <= proc_vs;
            if (proc_de) begin
                s1_rgb        <= proc_rgb;
                s1_y          <= input_y;
                s1_local_mean <= proc_local_mean;
                s1_local_max  <= proc_local_max;
                s1_local_range <= proc_local_range;
                s1_local_valid <= proc_local_valid;
                s1_halo_reduction <= proc_halo_reduction;
                s1_halo_valid     <= proc_halo_valid;
            end else begin
                s1_rgb        <= 24'h000000;
                s1_y          <= 8'd0;
                s1_local_mean <= 8'd0;
                s1_local_max  <= 8'd0;
                s1_local_range <= 8'd0;
                s1_local_valid <= 1'b0;
                s1_halo_reduction <= 8'd0;
                s1_halo_valid     <= 1'b0;
            end

            // Stage 2: register the tone curve and source metrics.  This
            // breaks the long s1_y -> tone curve -> weighted/local clamp path.
            s2_de                  <= s1_de;
            s2_hs                  <= s1_hs;
            s2_vs                  <= s1_vs;
            s2_rgb                 <= s1_rgb;
            s2_y                   <= s1_y;
            s2_base_delta          <= s1_base_delta;
            s2_lift_weight         <= s1_lift_weight;
            s2_local_gain_weight   <= s1_gain_smooth;
            if (s1_de)
                gain_smooth_state <= s1_gain_smooth;
            else
                gain_smooth_state <= 4'd15;
            s2_highlight_reduction <= s1_combined_reduction;
            s2_dark_desat          <= s1_dark_desat_next;

            // Stage 3: apply the local gain clamp and align all pixel data
            // before the final RGB adjustment stage.
            s3_de                  <= s2_de;
            s3_hs                  <= s2_hs;
            s3_vs                  <= s2_vs;
            s3_rgb                 <= s2_rgb;
            s3_y                   <= s2_y;
            s3_luma_delta          <= s2_limited_delta;
            s3_highlight_reduction <= s2_highlight_reduction;
            s3_dark_desat          <= s2_dark_desat;

            // Stage 4: common luminance adjustment, soft-knee compression and
            // low-light chroma cleanup.
            de_out <= s3_de;
            hs_out <= s3_hs;
            vs_out <= s3_vs;
            if (s3_de) begin
                if (night_enable)
                    rgb_out <= adjusted_rgb;
                else
                    rgb_out <= s3_rgb;
            end
            else
                rgb_out <= 24'h000000;
            night_active <= night_enable;
        end
    end
endmodule
