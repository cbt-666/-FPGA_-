// Continuous calibrated BEV-to-source perspective mapper.
//
// The former implementation fitted four unrelated affine planes per camera.
// Those planes selected source coordinates that differed by as much as 56
// pixels at x/y=180/540, which made a visible step in otherwise continuous
// road texture. This implementation evaluates the calibrated ground-plane
// perspective relation directly:
//
//   source_x = cx + fx * camera_side / camera_depth
//   source_y = cy + fy * camera_height / camera_depth
//
// camera_depth and camera_side are linear in BEV x/y. Since the renderer
// always visits pixels in raster order, their values are formed by additions
// from one pixel/row to the next. Two small iterative unsigned dividers are
// used in parallel; no coordinate ROM or combinational divider is required.
module bev_coord_map(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    input  wire [9:0]  bev_x,
    input  wire [9:0]  bev_y,
    output reg         out_valid,
    output reg         source_valid,
    output reg         source_real,
    output reg  [1:0]  source_cam,
    output reg  [9:0]  source_x,
    output reg  [8:0]  source_y
);

// Q14.18 camera-space coefficients for the approved +/-4 m, 720x720
// calibration. BL/BR include the approved -0.25 m/+0.25 m lateral offsets.
// The coefficients are fixed calibration data; pixel values are never stored
// here.  For an out-of-frustum BEV location the mapper clips the projected
// donor coordinate to the current camera frame, allowing the renderer to
// obtain a dynamic sample from DDR.
localparam signed [31:0] FL_D0 =  32'sd1200627;
localparam signed [31:0] FL_DX = -32'sd2386;
localparam signed [31:0] FL_DY = -32'sd1671;
localparam signed [31:0] FL_S0 =  32'sd117590;
localparam signed [31:0] FL_SX =  32'sd1671;
localparam signed [31:0] FL_SY = -32'sd2386;

localparam signed [31:0] FR_D0 = -32'sd514875;
localparam signed [31:0] FR_DX =  32'sd2386;
localparam signed [31:0] FR_DY = -32'sd1671;
localparam signed [31:0] FR_S0 = -32'sd1318798;
localparam signed [31:0] FR_SX =  32'sd1671;
localparam signed [31:0] FR_SY =  32'sd2386;

localparam signed [31:0] BL_D0 =  32'sd96095;
localparam signed [31:0] BL_DX = -32'sd2386;
localparam signed [31:0] BL_DY =  32'sd1671;
localparam signed [31:0] BL_S0 =  32'sd1345583;
localparam signed [31:0] BL_SX = -32'sd1671;
localparam signed [31:0] BL_SY = -32'sd2386;

localparam signed [31:0] BR_D0 = -32'sd1619405;
localparam signed [31:0] BR_DX =  32'sd2386;
localparam signed [31:0] BR_DY =  32'sd1671;
localparam signed [31:0] BR_S0 = -32'sd144378;
localparam signed [31:0] BR_SX = -32'sd1671;
localparam signed [31:0] BR_SY =  32'sd2386;

localparam signed [31:0] FL_DX_360 = -32'sd858960;
localparam signed [31:0] FL_SX_360 =  32'sd601560;
localparam signed [31:0] FR_DX_360 =  32'sd858960;
localparam signed [31:0] FR_SX_360 =  32'sd601560;
localparam signed [31:0] BL_DX_360 = -32'sd858960;
localparam signed [31:0] BL_SX_360 = -32'sd601560;
localparam signed [31:0] BR_DX_360 =  32'sd858960;
localparam signed [31:0] BR_SX_360 = -32'sd601560;

localparam signed [31:0] MIN_DEPTH = 32'sd13107; // 0.05 m in Q14.18
localparam        [31:0] HEIGHT_Q18 = 32'd393216; // 1.5 m

reg signed [31:0] row_den_fl, row_den_fr, row_den_bl, row_den_br;
reg signed [31:0] row_side_fl, row_side_fr, row_side_bl, row_side_br;
reg signed [31:0] current_den, current_side;

reg signed [31:0] selected_row_den, selected_row_side;
reg signed [31:0] selected_d0, selected_s0;
reg signed [31:0] selected_dx, selected_dy;
reg signed [31:0] selected_sx, selected_sy;
reg signed [31:0] selected_dx_360, selected_sx_360;
reg        [8:0]  selected_fx, selected_fy;
reg        [1:0]  selected_cam;
reg signed [31:0] map_den_next, map_side_next;

// For a projective point outside the camera frustum, keep a dynamic donor on
// the nearest valid ground-plane boundary.  The boundary is advanced once per
// BEV row using a small remainder accumulator (|dy/dx| is about 0.70), so no
// divider or coordinate ROM is needed.  The donor is still read from DDR and
// therefore follows the live camera image.
reg [9:0]  edge_x_fl, edge_x_fr, edge_x_bl, edge_x_br;
reg [11:0] edge_rem_fl, edge_rem_fr, edge_rem_bl, edge_rem_br;
reg signed [31:0] edge_den_fl, edge_den_fr, edge_den_bl, edge_den_br;
reg signed [31:0] edge_side_fl, edge_side_fr, edge_side_bl, edge_side_br;
reg signed [31:0] selected_edge_den, selected_edge_side;

always @(*) begin
    selected_cam = (bev_y < 10'd360) ?
                   ((bev_x < 10'd360) ? 2'd0 : 2'd1) :
                   ((bev_x < 10'd360) ? 2'd2 : 2'd3);
    selected_row_den = row_den_fl;
    selected_row_side = row_side_fl;
    selected_d0 = FL_D0; selected_s0 = FL_S0;
    selected_dx = FL_DX; selected_dy = FL_DY;
    selected_sx = FL_SX; selected_sy = FL_SY;
    selected_dx_360 = FL_DX_360; selected_sx_360 = FL_SX_360;
    selected_fx = 9'd457; selected_fy = 9'd257;
    selected_edge_den = edge_den_fl;
    selected_edge_side = edge_side_fl;
    case (selected_cam)
        2'd1: begin
            selected_row_den = row_den_fr;
            selected_row_side = row_side_fr;
            selected_d0 = FR_D0; selected_s0 = FR_S0;
            selected_dx = FR_DX; selected_dy = FR_DY;
            selected_sx = FR_SX; selected_sy = FR_SY;
            selected_dx_360 = FR_DX_360; selected_sx_360 = FR_SX_360;
            selected_edge_den = edge_den_fr;
            selected_edge_side = edge_side_fr;
        end
        2'd2: begin
            selected_row_den = row_den_bl;
            selected_row_side = row_side_bl;
            selected_d0 = BL_D0; selected_s0 = BL_S0;
            selected_dx = BL_DX; selected_dy = BL_DY;
            selected_sx = BL_SX; selected_sy = BL_SY;
            selected_dx_360 = BL_DX_360; selected_sx_360 = BL_SX_360;
            selected_fx = 9'd224; selected_fy = 9'd126;
            selected_edge_den = edge_den_bl;
            selected_edge_side = edge_side_bl;
        end
        2'd3: begin
            selected_row_den = row_den_br;
            selected_row_side = row_side_br;
            selected_d0 = BR_D0; selected_s0 = BR_S0;
            selected_dx = BR_DX; selected_dy = BR_DY;
            selected_sx = BR_SX; selected_sy = BR_SY;
            selected_dx_360 = BR_DX_360; selected_sx_360 = BR_SX_360;
            selected_fx = 9'd224; selected_fy = 9'd126;
            selected_edge_den = edge_den_br;
            selected_edge_side = edge_side_br;
        end
        default: begin end
    endcase

    if (bev_x == 10'd0) begin
        if (bev_y == 10'd0) begin
            map_den_next = selected_d0;
            map_side_next = selected_s0;
        end else begin
            map_den_next = selected_row_den + selected_dy;
            map_side_next = selected_row_side + selected_sy;
        end
    end else if (bev_x == 10'd360) begin
        map_den_next = selected_row_den + selected_dx_360;
        map_side_next = selected_row_side + selected_sx_360;
    end else begin
        map_den_next = current_den + selected_dx;
        map_side_next = current_side + selected_sx;
    end

end

// Register the coordinate recurrence before the absolute-value, multiply and
// rounding logic.  Besides making the launch metadata explicit, this breaks
// the former out_x -> recurrence -> APM -> divider-register path into two
// short 100 MHz paths.
reg launch_valid;
reg signed [31:0] launch_den;
reg signed [31:0] launch_side;
reg        [8:0]  launch_fx, launch_fy;
reg               launch_depth_valid, launch_fallback;
reg        [1:0]  launch_cam;

wire [31:0] divider_den = launch_depth_valid ? launch_den[31:0] : 32'd1;
wire launch_side_negative = launch_side[31];
wire [31:0] launch_side_magnitude = launch_side_negative ?
                                    (~launch_side + 1'b1) : launch_side;
wire [40:0] u_product_wide = launch_side_magnitude * launch_fx;
wire [40:0] v_product_wide = HEIGHT_Q18 * launch_fy;
wire [32:0] u_rounded_wide = {1'b0,u_product_wide[31:0]} +
                             {1'b0,divider_den[31:1]};
wire [32:0] v_rounded_wide = {1'b0,v_product_wide[31:0]} +
                             {1'b0,divider_den[31:1]};

wire u_div_done, v_div_done;
wire [31:0] u_quotient, v_quotient;
reg pending_depth_valid, pending_side_negative;
reg pending_fallback;
reg [1:0] pending_cam;

// 将乘法/舍入结果先寄存，再送入迭代除法器。这样 APM 乘法器和
// 除法器输入之间不再共用同一个 UI 时钟周期，降低 BEV 坐标路径的
// 组合延迟；renderer 本身会等待 mapper 的完成脉冲，因此只增加一拍
// 内部延迟，不改变输出坐标顺序。
reg [31:0] divider_u_q, divider_v_q, divider_den_q;
reg        divider_start_q;

bev_unsigned_divider #(.WIDTH(32)) u_div_x(
    .clk(clk), .rst_n(rst_n), .start(divider_start_q),
    .dividend(divider_u_q), .divisor(divider_den_q),
    .done(u_div_done), .quotient(u_quotient)
);

bev_unsigned_divider #(.WIDTH(32)) u_div_y(
    .clk(clk), .rst_n(rst_n), .start(divider_start_q),
    .dividend(divider_v_q), .divisor(divider_den_q),
    .done(v_div_done), .quotient(v_quotient)
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        row_den_fl <= FL_D0; row_den_fr <= FR_D0;
        row_den_bl <= BL_D0; row_den_br <= BR_D0;
        row_side_fl <= FL_S0; row_side_fr <= FR_S0;
        row_side_bl <= BL_S0; row_side_br <= BR_S0;
        current_den <= 0; current_side <= 0;
        edge_x_fl <= 10'd359; edge_x_fr <= 10'd360;
        edge_x_bl <= 10'd286; edge_x_br <= 10'd433;
        edge_rem_fl <= 12'd0; edge_rem_fr <= 12'd0;
        edge_rem_bl <= 12'd0; edge_rem_br <= 12'd0;
        edge_den_fl <= 32'sd13910; edge_side_fl <= 32'sd240994;
        edge_den_fr <= 32'sd13942; edge_side_fr <= -32'sd240753;
        edge_den_bl <= 32'sd15259; edge_side_bl <= 32'sd8717;
        edge_den_br <= 32'sd15293; edge_side_br <= -32'sd8961;
        launch_valid <= 0; launch_den <= 1; launch_side <= 0;
        launch_fx <= 0; launch_fy <= 0;
        launch_depth_valid <= 0;
        launch_fallback <= 0;
        launch_cam <= 0;
        pending_depth_valid <= 0;
        pending_side_negative <= 0;
        pending_fallback <= 0; pending_cam <= 0;
        divider_u_q <= 0; divider_v_q <= 0; divider_den_q <= 1;
        divider_start_q <= 1'b0;
        out_valid <= 0; source_valid <= 0; source_real <= 0; source_cam <= 0;
        source_x <= 0; source_y <= 0;
    end else begin
        out_valid <= 1'b0;
        launch_valid <= 1'b0;
        divider_start_q <= 1'b0;
        if (in_valid) begin
            current_den <= map_den_next;
            current_side <= map_side_next;
            launch_valid <= 1'b1;
            // Keep the original calibrated projective coordinates for every
            // pixel.  Blind-area filling is handled only by the small output
            // clamp below, avoiding extra arithmetic on the DDR-sensitive
            // mapper path.
            launch_den <= map_den_next;
            launch_side <= map_side_next;
            launch_fx <= selected_fx;
            launch_fy <= selected_fy;
            launch_fallback <= (map_den_next <= MIN_DEPTH);
            if (map_den_next > MIN_DEPTH) begin
                launch_den <= map_den_next;
                launch_side <= map_side_next;
                launch_depth_valid <= 1'b1;
            end else begin
                // Replace an invalid projective point with the closest valid
                // boundary point for this camera and row.  This keeps the
                // source dynamic while avoiding division by a near-zero or
                // negative depth.
                launch_den <= selected_edge_den;
                launch_side <= selected_edge_side;
                launch_depth_valid <= (selected_edge_den > MIN_DEPTH);
            end
            launch_cam <= selected_cam;
            if (bev_x == 10'd0) begin
                if (bev_y == 10'd0) begin
                    row_den_fl <= FL_D0; row_den_fr <= FR_D0;
                    row_den_bl <= BL_D0; row_den_br <= BR_D0;
                    row_side_fl <= FL_S0; row_side_fr <= FR_S0;
                    row_side_bl <= BL_S0; row_side_br <= BR_S0;
                    edge_x_fl <= 10'd359; edge_x_fr <= 10'd360;
                    edge_x_bl <= 10'd286; edge_x_br <= 10'd433;
                    edge_rem_fl <= 12'd0; edge_rem_fr <= 12'd0;
                    edge_rem_bl <= 12'd0; edge_rem_br <= 12'd0;
                    edge_den_fl <= 32'sd13910; edge_side_fl <= 32'sd240994;
                    edge_den_fr <= 32'sd13942; edge_side_fr <= -32'sd240753;
                    edge_den_bl <= 32'sd15259; edge_side_bl <= 32'sd8717;
                    edge_den_br <= 32'sd15293; edge_side_br <= -32'sd8961;
                end else begin
                    row_den_fl <= row_den_fl + FL_DY;
                    row_den_fr <= row_den_fr + FR_DY;
                    row_den_bl <= row_den_bl + BL_DY;
                    row_den_br <= row_den_br + BR_DY;
                    row_side_fl <= row_side_fl + FL_SY;
                    row_side_fr <= row_side_fr + FR_SY;
                    row_side_bl <= row_side_bl + BL_SY;
                    row_side_br <= row_side_br + BR_SY;

                    // Exact boundary seeds are used where each rear/front
                    // camera first reaches the depth threshold.  Thereafter
                    // only an add/subtract and a remainder update are needed
                    // once per row.
                    if (bev_y == 10'd199) begin
                        edge_x_fl <= 10'd358; edge_rem_fl <= 12'd802;
                        edge_den_fl <= 32'sd13910; edge_side_fl <= 32'sd240994;
                        edge_x_fr <= 10'd361; edge_rem_fr <= 12'd1551;
                        edge_den_fr <= 32'sd13942; edge_side_fr <= -32'sd240753;
                    end else if (bev_y > 10'd199) begin
                        if (edge_rem_fl < 12'd1671) begin
                            edge_x_fl <= edge_x_fl - 1'b1;
                            edge_rem_fl <= edge_rem_fl + 12'd715;
                            // FL boundary moves one BEV column toward -x;
                            // apply the signed x step with the opposite sign.
                            // The previous +FL_DX update drove the donor depth
                            // negative after a few rows and created a black
                            // triangular region in the upper-left quadrant.
                            edge_den_fl <= edge_den_fl + FL_DY - FL_DX;
                            edge_side_fl <= edge_side_fl + FL_SY - FL_SX;
                        end else begin
                            edge_rem_fl <= edge_rem_fl - 12'd1671;
                            edge_den_fl <= edge_den_fl + FL_DY;
                            edge_side_fl <= edge_side_fl + FL_SY;
                        end
                        if ((edge_rem_fr + 12'd1671) >= 12'd2386) begin
                            edge_x_fr <= edge_x_fr + 1'b1;
                            edge_rem_fr <= edge_rem_fr - 12'd715;
                            edge_den_fr <= edge_den_fr + FR_DY + FR_DX;
                            edge_side_fr <= edge_side_fr + FR_SY + FR_SX;
                        end else begin
                            edge_rem_fr <= edge_rem_fr + 12'd1671;
                            edge_den_fr <= edge_den_fr + FR_DY;
                            edge_side_fr <= edge_side_fr + FR_SY;
                        end
                    end
                    if (bev_y == 10'd360) begin
                        edge_x_bl <= 10'd286; edge_rem_bl <= 12'd2151;
                        edge_den_bl <= 32'sd15259; edge_side_bl <= 32'sd8717;
                        edge_x_br <= 10'd433; edge_rem_br <= 12'd200;
                        edge_den_br <= 32'sd15293; edge_side_br <= -32'sd8961;
                    end else if (bev_y > 10'd360) begin
                        if ((edge_rem_bl + 12'd1671) >= 12'd2386) begin
                            edge_x_bl <= edge_x_bl + 1'b1;
                            edge_rem_bl <= edge_rem_bl - 12'd715;
                            edge_den_bl <= edge_den_bl + BL_DY + BL_DX;
                            edge_side_bl <= edge_side_bl + BL_SY + BL_SX;
                        end else begin
                            edge_rem_bl <= edge_rem_bl + 12'd1671;
                            edge_den_bl <= edge_den_bl + BL_DY;
                            edge_side_bl <= edge_side_bl + BL_SY;
                        end
                        if (edge_rem_br < 12'd1671) begin
                            edge_x_br <= edge_x_br - 1'b1;
                            edge_rem_br <= edge_rem_br + 12'd715;
                            edge_den_br <= edge_den_br + BR_DY - BR_DX;
                            edge_side_br <= edge_side_br + BR_SY - BR_SX;
                        end else begin
                            edge_rem_br <= edge_rem_br - 12'd1671;
                            edge_den_br <= edge_den_br + BR_DY;
                            edge_side_br <= edge_side_br + BR_SY;
                        end
                    end
                end
            end
        end

        if (launch_valid) begin
            pending_depth_valid <= launch_depth_valid;
            pending_side_negative <= launch_side_negative;
            pending_fallback <= launch_fallback;
            pending_cam <= launch_cam;
            divider_u_q <= u_rounded_wide[31:0];
            divider_v_q <= v_rounded_wide[31:0];
            divider_den_q <= divider_den;
            divider_start_q <= 1'b1;
        end

        if (u_div_done && v_div_done) begin
            out_valid <= 1'b1;
            source_cam <= pending_cam;
            source_x <= 10'd0;
            source_y <= 9'd0;
            source_valid <= 1'b0;
            source_real <= 1'b0;
            // Keep a clipped live donor for non-road outside regions, while
            // separately reporting whether the unmodified perspective point
            // is truly visible.  The renderer uses source_real to ensure
            // edge-clamped wall/lamp pixels never train or fill road areas.
            if (pending_depth_valid) begin
                source_y <= (v_quotient > 32'd179) ?
                            9'd359 : (9'd180 + v_quotient[8:0]);
                if (pending_side_negative)
                    source_x <= (u_quotient > 32'd320) ?
                                10'd0 : (10'd320 - u_quotient[9:0]);
                else
                    source_x <= (u_quotient > 32'd319) ?
                                10'd639 : (10'd320 + u_quotient[9:0]);
                source_valid <= 1'b1;
                source_real <= !pending_fallback &&
                               (v_quotient <= 32'd179) &&
                               (u_quotient <= (pending_side_negative ?
                                               32'd320 : 32'd319));
            end
        end
    end
end

endmodule

// Restoring unsigned divider. One result is produced after WIDTH clocks.
// The mapper launches two instances together and waits for their done pulses.
module bev_unsigned_divider #(
    parameter WIDTH = 32
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 start,
    input  wire [WIDTH-1:0]     dividend,
    input  wire [WIDTH-1:0]     divisor,
    output reg                  done,
    output reg  [WIDTH-1:0]     quotient
);

reg busy;
reg [5:0] count;
reg [WIDTH-1:0] dividend_shift;
reg [WIDTH-1:0] divisor_reg;
reg [WIDTH-1:0] quotient_work;
reg [WIDTH:0] remainder;

wire [WIDTH:0] remainder_shift =
    {remainder[WIDTH-1:0],dividend_shift[WIDTH-1]};
wire subtract_ok = remainder_shift >= {1'b0,divisor_reg};
wire [WIDTH:0] remainder_next = subtract_ok ?
    remainder_shift - {1'b0,divisor_reg} : remainder_shift;
wire [WIDTH-1:0] quotient_next =
    {quotient_work[WIDTH-2:0],subtract_ok};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        busy <= 1'b0; count <= 0; done <= 1'b0;
        dividend_shift <= 0; divisor_reg <= 1;
        quotient_work <= 0; quotient <= 0; remainder <= 0;
    end else begin
        done <= 1'b0;
        if (start && !busy) begin
            busy <= 1'b1;
            count <= WIDTH;
            dividend_shift <= dividend;
            divisor_reg <= (divisor == 0) ?
                           {{(WIDTH-1){1'b0}},1'b1} : divisor;
            quotient_work <= 0;
            remainder <= 0;
        end else if (busy) begin
            dividend_shift <= {dividend_shift[WIDTH-2:0],1'b0};
            quotient_work <= quotient_next;
            remainder <= remainder_next;
            if (count == 1) begin
                busy <= 1'b0;
                count <= 0;
                quotient <= quotient_next;
                done <= 1'b1;
            end else begin
                count <= count - 1'b1;
            end
        end
    end
end

endmodule
