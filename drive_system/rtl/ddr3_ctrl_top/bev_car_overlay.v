// Small procedural car sprite at the calibrated host-vehicle position.
//
// This is intentionally not a frame-sized bitmap. The sprite is generated
// from a few rectangles and rounded end caps, so it consumes only LUTs and
// comparators and follows the live BEV image underneath it.
module bev_car_overlay(
    input  wire [9:0]  bev_x,
    input  wire [9:0]  bev_y,
    output reg         overlay_valid,
    output reg  [15:0] overlay_rgb565
);

// The reference vehicle is defined in the calibrated BEV coordinate system,
// not by the HDMI screen midpoint.  Its reference bounds are screen
// The road calibration places the host vehicle on the right-hand lane at
// BEV x=410.  Keep this in BEV coordinates so changing the HDMI window does
// not move the vehicle relative to the transformed road.
localparam [9:0] CAR_CX = 10'd410;
localparam [9:0] CAR_CY = 10'd360;

// RGB565 values sampled from the reference car.
localparam [15:0] CAR_RED   = 16'hC925; // #CD262A
localparam [15:0] CAR_DARK  = 16'h21E8; // #263E46, glass
localparam [15:0] CAR_SHADE = 16'hB0E4; // #B11F23
localparam [15:0] CAR_BLACK = 16'h10A2; // #141416, wheels
localparam [15:0] CAR_LAMP  = 16'hFFB6; // #FFF5B4
localparam [15:0] CAR_TAIL  = 16'h9000; // #910000
localparam [15:0] CAR_HI    = 16'hF7BE; // #F2F2F2

wire [9:0] dx_abs = (bev_x >= CAR_CX) ? (bev_x - CAR_CX) : (CAR_CX - bev_x);
wire [9:0] dy_abs = (bev_y >= CAR_CY) ? (bev_y - CAR_CY) : (CAR_CY - bev_y);
wire       top_half = (bev_y < CAR_CY);

always @(*) begin
    overlay_valid = 1'b0;
    overlay_rgb565 = 16'h0000;

    // Keep the host vehicle clearly narrower than the calibrated right lane,
    // while giving it a physically plausible footprint in the 720x720 BEV.
    // Body width is 97 px, total width 115 px and height 211 px.
    if ((dx_abs <= 10'd48) && (dy_abs <= 10'd105)) begin
        overlay_valid = 1'b1;
        overlay_rgb565 = CAR_RED;

        // Rounded roof and bumper ends.
        if ((dy_abs >= 10'd96) && (dx_abs > 10'd41))
            overlay_valid = 1'b0;

        // Front/rear windows, narrower than the red pillars.
        if (dy_abs >= 10'd30 && dy_abs <= 10'd69)
            overlay_rgb565 = CAR_DARK;

        // Central roof/hood panel and a subtle body shade.
        if (dy_abs <= 10'd26)
            overlay_rgb565 = CAR_SHADE;
        if ((dx_abs <= 10'd6) && (dy_abs <= 10'd87))
            overlay_rgb565 = CAR_HI;

        // Front lamps (top) and tail lamps (bottom).
        if (top_half && dy_abs >= 10'd92 && dx_abs >= 10'd15 && dx_abs <= 10'd32)
            overlay_rgb565 = CAR_LAMP;
        if (!top_half && dy_abs >= 10'd92 && dx_abs >= 10'd15 && dx_abs <= 10'd32)
            overlay_rgb565 = CAR_TAIL;

        // Do not let the center highlight cross either window.
        if ((dy_abs >= 10'd30) && (dy_abs <= 10'd69))
            overlay_rgb565 = CAR_DARK;
    end

    // Four small wheel/sidewall blocks just outside the red body.
    if ((dx_abs >= 10'd49) && (dx_abs <= 10'd57) &&
        (dy_abs >= 10'd30) && (dy_abs <= 10'd68)) begin
        overlay_valid = 1'b1;
        overlay_rgb565 = CAR_BLACK;
    end

    // A thin pale bumper highlight makes the sprite readable on a light road.
    if (overlay_valid && (dy_abs == 10'd104) && (dx_abs <= 10'd26))
        overlay_rgb565 = CAR_HI;
end

endmodule
