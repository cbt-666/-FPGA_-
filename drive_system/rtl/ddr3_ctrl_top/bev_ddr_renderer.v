// Background DDR BEV renderer.
//
// One source cache line is 16 RGB565 pixels (one 256-bit DDR word).  Output
// pixels are packed into the same word size and written to a ping-pong BEV
// frame.  The renderer is deliberately latency tolerant: display reads have
// priority in the shared DDR scheduler, while this state machine simply waits.
module bev_ddr_renderer #(
    parameter [19:0] BEV_BASE_WORD = 20'h20000,
    parameter ROAD_PROFILE_ENABLE = 1
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         enable,
    input  wire [1:0]   source_page0,
    input  wire [1:0]   source_page1,
    input  wire [1:0]   source_page2,
    input  wire [1:0]   source_page3,
    input  wire [3:0]   source_ready,
    input  wire [1:0]   display_in_use_page,

    output reg          source_req,
    output reg  [23:0]  source_addr,
    input  wire         source_data_valid,
    input  wire [255:0] source_data,

    output reg          write_req,
    output reg  [23:0]  write_addr,
    output reg  [255:0] write_data,
    input  wire         write_done,

    output reg  [1:0]   display_page,
    output reg          frame_ready,
    // Held for the complete render transaction. The DDR scheduler uses this
    // lease to keep input write pages from rotating onto the four pages that
    // were captured at the start of this BEV frame.
    output reg          source_lock
);

localparam S_IDLE=3'd0, S_MAP_REQ=3'd1, S_MAP_WAIT=3'd2,
           S_SOURCE_WAIT=3'd3, S_WRITE_WAIT=3'd4;

reg [2:0] state;
reg [9:0] out_x, out_y;
reg [15:0] out_word_index;
reg [255:0] pack_word;
reg cache_valid;
reg [23:0] cache_addr;
reg [255:0] cache_data;
reg [3:0] pending_lane;
reg last_word_pending;
reg write_page;
reg road_warmed;
reg [1:0] page0, page1, page2, page3;

reg map_in_valid;
wire map_out_valid;
wire map_source_valid;
wire map_source_real;
wire [1:0] map_cam;
wire [9:0] map_sx;
wire [8:0] map_sy;

wire [19:0] source_word_offset = ({11'd0,map_sy} << 5) +
                                  ({11'd0,map_sy} << 3) +
                                  {14'd0,map_sx[9:4]};
wire [1:0] selected_page = (map_cam==2'd0) ? page0 :
                           (map_cam==2'd1) ? page1 :
                           (map_cam==2'd2) ? page2 : page3;
wire [23:0] mapped_addr = {map_cam,selected_page,source_word_offset};

wire        car_overlay_valid;
wire [15:0] car_overlay_rgb565;

bev_car_overlay u_bev_car_overlay(
    .bev_x          (out_x),
    .bev_y          (out_y),
    .overlay_valid  (car_overlay_valid),
    .overlay_rgb565 (car_overlay_rgb565)
);

function [15:0] word_pixel;
    input [255:0] word;
    input [3:0] lane;
    begin word_pixel = word >> {lane,4'b0000}; end
endfunction

// Component-wise RGB565 interpolation with a 0..64 weight.  A six-bit
// fractional weight gives a smooth field without RGB cross-component carry.
function [15:0] lerp64;
    input [15:0] a;
    input [15:0] b;
    input [6:0] weight;
    reg [12:0] r_sum, g_sum, b_sum;
    begin
        r_sum = a[15:11] * (7'd64-weight) + b[15:11] * weight + 13'd32;
        g_sum = a[10:5]  * (7'd64-weight) + b[10:5]  * weight + 13'd32;
        b_sum = a[4:0]   * (7'd64-weight) + b[4:0]   * weight + 13'd32;
        lerp64 = {r_sum[10:6],g_sum[11:6],b_sum[10:6]};
    end
endfunction

// Q4 exponential means are updated only from real perspective samples.  Fill
// colours are latched at frame boundaries, so one output frame never sees an
// anchor change part-way through its raster scan.
reg [9:0] road_front_r_q, road_front_g_q, road_front_b_q;
reg [9:0] road_bl_r_q,    road_bl_g_q,    road_bl_b_q;
reg [9:0] road_br_r_q,    road_br_g_q,    road_br_b_q;
reg [15:0] road_front_latched, road_bl_latched, road_br_latched;
reg [2:0] road_anchor_seen;
wire [15:0] road_front_mean={road_front_r_q[8:4],road_front_g_q[9:4],road_front_b_q[8:4]};
wire [15:0] road_bl_mean={road_bl_r_q[8:4],road_bl_g_q[9:4],road_bl_b_q[8:4]};
wire [15:0] road_br_mean={road_br_r_q[8:4],road_br_g_q[9:4],road_br_b_q[8:4]};
reg        road_sample_pending;
reg [1:0]  road_sample_cam;
reg [15:0] road_sample_pixel;

wire [15:0] road_front_color = road_front_latched;
wire [15:0] road_bl_color = road_bl_latched;
wire [15:0] road_br_color = road_br_latched;

wire road_zone = (out_x >= 10'd90) && (out_x < 10'd482);
wire left_curb_zone = (out_x >= 10'd70) && (out_x < 10'd90);
wire right_curb_zone = (out_x >= 10'd482) && (out_x < 10'd506);

wire [9:0] road_x_delta = (out_x <= 10'd250) ? 10'd0 :
                               (out_x >= 10'd470) ? 10'd220 : out_x-10'd250;
wire [9:0] road_y_delta = (out_y <= 10'd120) ? 10'd0 :
                               (out_y >= 10'd600) ? 10'd480 : out_y-10'd120;
wire [13:0] road_x_delta_wide = {4'd0,road_x_delta};
wire [13:0] road_y_delta_wide = {4'd0,road_y_delta};
wire [13:0] road_x_scaled = (road_x_delta_wide << 5) +
                                  (road_x_delta_wide << 2) +
                                  road_x_delta_wide + 14'd64;
wire [13:0] road_y_scaled = (road_y_delta_wide << 4) +
                                  road_y_delta_wide + 14'd64;
wire [6:0] road_x_weight = road_x_scaled[13:7];
wire [6:0] road_y_weight = road_y_scaled[13:7];
wire [15:0] road_back_color = lerp64(road_bl_color,road_br_color,
                                     road_x_weight);
reg  [15:0] road_back_color_pipe;
wire [15:0] road_fill_color = lerp64(road_front_color,road_back_color_pipe,
                                     road_y_weight);

// Publish local road profiles only between completed render frames. The
// existing pixel cache supplies all training samples; no extra DDR reads.
wire road_profile_busy,road_profile_valid;
wire [15:0] road_profile_pixel;
wire road_frame_begin=(state==S_IDLE) && enable && !road_profile_busy &&
                     ({1'b0,write_page}!=display_in_use_page);
wire road_frame_end=(state==S_WRITE_WAIT) && write_done && last_word_pending;
reg profile_sample_valid;
reg [9:0] profile_sample_x,profile_sample_y;
reg [15:0] profile_sample_pixel;
reg [1:0] profile_sample_cam;
wire profile_edge_sample;
generate if(ROAD_PROFILE_ENABLE) begin:g_road_profile
    bev_road_surface u_road_surface(
        .clk(clk),.rst_n(rst_n),.frame_begin(road_frame_begin),
        .frame_end(road_frame_end),.source_ready(source_ready),
        .sample_valid(profile_sample_valid),.sample_x(profile_sample_x),
        .sample_y(profile_sample_y),.sample_pixel(profile_sample_pixel),
        .edge_sample_valid(profile_edge_sample),
        .query_start(map_in_valid),.query_x(out_x),.query_y(out_y),
        .busy(road_profile_busy),.fill_valid(road_profile_valid),
        .fill_pixel(road_profile_pixel)
    );
end else begin:g_no_road_profile
    assign road_profile_busy=1'b0;
    assign road_profile_valid=1'b0;
    assign road_profile_pixel=16'h0000;
    assign profile_edge_sample=1'b0;
end endgenerate

// The two calibrated curb bands retain a small transverse profile.  The
// closest real sample above and below the blind region is kept independently,
// so a curb can never borrow road colour and vice versa.
reg [15:0] left_front_profile [0:19];
reg [15:0] left_rear_profile  [0:19];
reg [15:0] right_front_profile[0:23];
reg [15:0] right_rear_profile [0:23];
reg [19:0] left_front_valid, left_rear_valid, left_rear_seen;
reg [23:0] right_front_valid, right_rear_valid, right_rear_seen;

reg [15:0] curb_front_color, curb_rear_color;
reg [15:0] curb_fill_color_pipe;
reg curb_front_valid, curb_rear_valid;
reg [15:0] curb_front_color_pipe, curb_rear_color_pipe;
reg curb_front_valid_pipe, curb_rear_valid_pipe;
reg [6:0] curb_y_weight_pipe;
wire [9:0] curb_y_delta = (out_y <= 10'd250) ? 10'd0 :
                               (out_y >= 10'd470) ? 10'd220 : out_y-10'd250;
wire [13:0] curb_y_delta_wide = {4'd0,curb_y_delta};
wire [13:0] curb_y_scaled = (curb_y_delta_wide << 5) +
                                  (curb_y_delta_wide << 2) +
                                  curb_y_delta_wide + 14'd64;
wire [6:0] curb_y_weight = curb_y_scaled[13:7];
always @(*) begin
    curb_front_color = 16'h0000;
    curb_rear_color = 16'h0000;
    curb_front_valid = 1'b0;
    curb_rear_valid = 1'b0;
    if (left_curb_zone) begin
        curb_front_color = left_front_profile[out_x-10'd70];
        curb_rear_color = left_rear_profile[out_x-10'd70];
        curb_front_valid = left_front_valid[out_x-10'd70];
        curb_rear_valid = left_rear_valid[out_x-10'd70];
    end else if (right_curb_zone) begin
        curb_front_color = right_front_profile[out_x-10'd482];
        curb_rear_color = right_rear_profile[out_x-10'd482];
        curb_front_valid = right_front_valid[out_x-10'd482];
        curb_rear_valid = right_rear_valid[out_x-10'd482];
    end

end

wire [15:0] curb_fill_color_next =
    (curb_front_valid_pipe && curb_rear_valid_pipe) ?
        lerp64(curb_front_color_pipe,curb_rear_color_pipe,
               curb_y_weight_pipe) :
    curb_front_valid_pipe ? curb_front_color_pipe :
    curb_rear_valid_pipe  ? curb_rear_color_pipe  : 16'h0000;

bev_coord_map u_bev_coord_map(
    .clk(clk), .rst_n(rst_n), .in_valid(map_in_valid),
    .bev_x(out_x), .bev_y(out_y),
    .out_valid(map_out_valid), .source_valid(map_source_valid),
    .source_real(map_source_real),
    .source_cam(map_cam), .source_x(map_sx), .source_y(map_sy)
);

task advance_pixel;
    begin
        if (out_x == 10'd719) begin
            out_x <= 10'd0;
            if (out_y == 10'd719) out_y <= 10'd0;
            else out_y <= out_y + 1'b1;
        end else out_x <= out_x + 1'b1;
    end
endtask

task accept_pixel;
    input [15:0] pixel;
    reg   [15:0] pixel_final;
    begin
        if(map_source_real && road_zone && source_ready[map_cam]) begin
            profile_sample_valid<=1'b1;
            profile_sample_x<=out_x;profile_sample_y<=out_y;
            profile_sample_pixel<=pixel;
            profile_sample_cam<=map_cam;
        end
        // The disabled-profile compatibility mode also uses real input.
        if (!ROAD_PROFILE_ENABLE && map_source_real && road_zone && source_ready[map_cam] &&
            (out_y >= 10'd40) && (out_y < 10'd680)) begin
            road_sample_pending <= 1'b1;
            road_sample_cam <= map_cam;
            road_sample_pixel <= pixel;
        end

        if (map_source_real && left_curb_zone) begin
            if (out_y < 10'd250) begin
                left_front_profile[out_x-10'd70] <= pixel;
                left_front_valid[out_x-10'd70] <= 1'b1;
            end else if ((out_y >= 10'd470) &&
                         !left_rear_seen[out_x-10'd70]) begin
                left_rear_profile[out_x-10'd70] <= pixel;
                left_rear_valid[out_x-10'd70] <= 1'b1;
                left_rear_seen[out_x-10'd70] <= 1'b1;
            end
        end
        if (map_source_real && right_curb_zone) begin
            if (out_y < 10'd250) begin
                right_front_profile[out_x-10'd482] <= pixel;
                right_front_valid[out_x-10'd482] <= 1'b1;
            end else if ((out_y >= 10'd470) &&
                         !right_rear_seen[out_x-10'd482]) begin
                right_rear_profile[out_x-10'd482] <= pixel;
                right_rear_valid[out_x-10'd482] <= 1'b1;
                right_rear_seen[out_x-10'd482] <= 1'b1;
            end
        end

        pixel_final = car_overlay_valid ? car_overlay_rgb565 : pixel;
        if (out_x[3:0] == 4'd15) begin
            write_data <= {pixel_final,pack_word[239:0]};
            write_addr <= {2'b00,write_page,BEV_BASE_WORD + out_word_index};
            write_req <= 1'b1;
            last_word_pending <= (out_x==10'd719) && (out_y==10'd719);
            advance_pixel();
            state <= S_WRITE_WAIT;
        end else begin
            pack_word[out_x[3:0]*16 +: 16] <= pixel_final;
            advance_pixel();
            state <= S_MAP_REQ;
        end
    end
endtask

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state<=S_IDLE; out_x<=0; out_y<=0; out_word_index<=0;
        pack_word<=0; cache_valid<=0; cache_addr<=0; cache_data<=0;
        pending_lane<=0; last_word_pending<=0;
        source_req<=0; source_addr<=0; write_req<=0; write_addr<=0; write_data<=0;
        display_page<=0; write_page<=1'b1; frame_ready<=0;road_warmed<=0;
        source_lock<=1'b0;
        page0<=0; page1<=0; page2<=0; page3<=0; map_in_valid<=0;
        // Reset values are not a displayed road colour. The first render only
        // gathers real input statistics and is never published to the display.
        road_front_r_q<=0; road_front_g_q<=0; road_front_b_q<=0;
        road_bl_r_q<=0; road_bl_g_q<=0; road_bl_b_q<=0;
        road_br_r_q<=0; road_br_g_q<=0; road_br_b_q<=0;
        road_front_latched<=16'h0000; road_bl_latched<=16'h0000;
        road_br_latched<=16'h0000;
        road_back_color_pipe<=16'h0000;
        road_anchor_seen<=0;
        curb_fill_color_pipe<=16'h0000;
        curb_front_color_pipe<=16'h0000; curb_rear_color_pipe<=16'h0000;
        curb_front_valid_pipe<=1'b0; curb_rear_valid_pipe<=1'b0;
        curb_y_weight_pipe<=7'd0;
        road_sample_pending<=1'b0; road_sample_cam<=2'd0;
        road_sample_pixel<=16'h0000;
        profile_sample_valid<=1'b0;
        profile_sample_x<=0;profile_sample_y<=0;profile_sample_pixel<=0;
        profile_sample_cam<=0;
        left_front_valid<=0; left_rear_valid<=0; left_rear_seen<=0;
        right_front_valid<=0; right_rear_valid<=0; right_rear_seen<=0;
    end else begin
        map_in_valid <= 1'b0;
        road_sample_pending <= 1'b0;
        profile_sample_valid <= 1'b0;
        // Derive fallback anchors from the SAME real boundary samples.
        if (profile_edge_sample) begin
            road_sample_pending<=1;
            road_sample_pixel<=profile_sample_pixel;
            road_sample_cam<=profile_sample_cam;
        end
        // Apply the accepted road sample one cycle later.  This keeps the
        // colour classifier out of the EMA registers' high-fanout CE path.
        if (road_sample_pending) begin
            case (road_sample_cam)
                2'd0: begin
                    road_anchor_seen[0]<=1;
                    road_front_r_q <= road_anchor_seen[0] ? road_front_r_q - (road_front_r_q >> 4) + road_sample_pixel[15:11] : {1'b0,road_sample_pixel[15:11],4'd0};
                    road_front_g_q <= road_anchor_seen[0] ? road_front_g_q - (road_front_g_q >> 4) + road_sample_pixel[10:5] : {road_sample_pixel[10:5],4'd0};
                    road_front_b_q <= road_anchor_seen[0] ? road_front_b_q - (road_front_b_q >> 4) + road_sample_pixel[4:0] : {1'b0,road_sample_pixel[4:0],4'd0};
                end
                // FL and FR are both front views. FR must contribute to the
                // front road estimate instead of leaving it biased toward FL.
                2'd1: begin
                    road_anchor_seen[0]<=1;
                    road_front_r_q <= road_anchor_seen[0] ? road_front_r_q - (road_front_r_q >> 4) + road_sample_pixel[15:11] : {1'b0,road_sample_pixel[15:11],4'd0};
                    road_front_g_q <= road_anchor_seen[0] ? road_front_g_q - (road_front_g_q >> 4) + road_sample_pixel[10:5] : {road_sample_pixel[10:5],4'd0};
                    road_front_b_q <= road_anchor_seen[0] ? road_front_b_q - (road_front_b_q >> 4) + road_sample_pixel[4:0] : {1'b0,road_sample_pixel[4:0],4'd0};
                end
                2'd2: begin
                    road_anchor_seen[1]<=1;
                    road_bl_r_q <= road_anchor_seen[1] ? road_bl_r_q - (road_bl_r_q >> 4) + road_sample_pixel[15:11] : {1'b0,road_sample_pixel[15:11],4'd0};
                    road_bl_g_q <= road_anchor_seen[1] ? road_bl_g_q - (road_bl_g_q >> 4) + road_sample_pixel[10:5] : {road_sample_pixel[10:5],4'd0};
                    road_bl_b_q <= road_anchor_seen[1] ? road_bl_b_q - (road_bl_b_q >> 4) + road_sample_pixel[4:0] : {1'b0,road_sample_pixel[4:0],4'd0};
                end
                2'd3: begin
                    road_anchor_seen[2]<=1;
                    road_br_r_q <= road_anchor_seen[2] ? road_br_r_q - (road_br_r_q >> 4) + road_sample_pixel[15:11] : {1'b0,road_sample_pixel[15:11],4'd0};
                    road_br_g_q <= road_anchor_seen[2] ? road_br_g_q - (road_br_g_q >> 4) + road_sample_pixel[10:5] : {road_sample_pixel[10:5],4'd0};
                    road_br_b_q <= road_anchor_seen[2] ? road_br_b_q - (road_br_b_q >> 4) + road_sample_pixel[4:0] : {1'b0,road_sample_pixel[4:0],4'd0};
                end
                default: begin end
            endcase
        end
        case (state)
            S_IDLE: begin
                source_req<=0; write_req<=0;
                // Do not start overwriting a page until the display scheduler
                // has moved to the other completed page at a VS boundary.
                if (road_frame_begin) begin
                    road_anchor_seen<=0;
                    page0<=source_page0; page1<=source_page1;
                    page2<=source_page2; page3<=source_page3;
                    source_lock<=1'b1;
                    cache_valid<=0; out_x<=0; out_y<=0; out_word_index<=0;
                    pack_word<=0;
                    // Profiles are observations from the current source
                    // frame. Do not reuse last frame's curb pixels after an
                    // input disconnect or scene change.
                    left_front_valid<=0; left_rear_valid<=0;
                    left_rear_seen<=0;
                    right_front_valid<=0; right_rear_valid<=0;
                    right_rear_seen<=0;
                    state<=S_MAP_REQ;
                end
            end
            S_MAP_REQ: begin
                // The coordinate mapper provides ample latency before
                // S_MAP_WAIT completes.  Register the transverse road blend
                // here so the two RGB interpolation stages are never on one
                // 100 MHz combinational path.
                road_back_color_pipe<=road_back_color;
                curb_front_color_pipe<=curb_front_color;
                curb_rear_color_pipe<=curb_rear_color;
                curb_front_valid_pipe<=curb_front_valid;
                curb_rear_valid_pipe<=curb_rear_valid;
                curb_y_weight_pipe<=curb_y_weight;
                map_in_valid<=1'b1;
                state<=S_MAP_WAIT;
            end
            S_MAP_WAIT: begin
                curb_fill_color_pipe<=curb_fill_color_next;
                if (map_out_valid) begin
                // Keep an unplugged input's complete quadrant black.  True
                // perspective pixels are read normally; missing road/curb
                // pixels use live, surface-specific statistics.  Outside
                // those calibrated bands the mapper's live edge donor is
                // retained so scenery remains dynamic.
                if (!source_ready[map_cam]) begin
                    accept_pixel(16'h0000);
                end else if (!map_source_real && road_zone) begin
                    accept_pixel(road_profile_valid ? road_profile_pixel : road_fill_color);
                end else if (!map_source_real &&
                             (left_curb_zone || right_curb_zone)) begin
                    accept_pixel(curb_fill_color_pipe);
                end else if (!map_source_valid) begin
                    accept_pixel(16'h0000);
                end else if (cache_valid && cache_addr==mapped_addr) begin
                    accept_pixel(word_pixel(cache_data,map_sx[3:0]));
                end else begin
                    source_addr<=mapped_addr;
                    pending_lane<=map_sx[3:0];
                    source_req<=1'b1;
                    state<=S_SOURCE_WAIT;
                end
                end
            end
            S_SOURCE_WAIT: if (source_data_valid) begin
                source_req<=1'b0;
                cache_valid<=1'b1; cache_addr<=source_addr; cache_data<=source_data;
                accept_pixel(word_pixel(source_data,pending_lane));
            end
            S_WRITE_WAIT: if (write_done) begin
                write_req<=1'b0;
                if (last_word_pending) begin
                    // Borrow missing groups from this render's measured edge,
                    // never from an unrelated reset colour or old scene.
                    if (|road_anchor_seen) begin
                        road_front_latched <= road_anchor_seen[0] ? road_front_mean :
                                              road_anchor_seen[1] ? road_bl_mean : road_br_mean;
                        road_bl_latched <= road_anchor_seen[1] ? road_bl_mean :
                                           road_anchor_seen[0] ? road_front_mean : road_br_mean;
                        road_br_latched <= road_anchor_seen[2] ? road_br_mean :
                                           road_anchor_seen[0] ? road_front_mean : road_bl_mean;
                    end
                    // Warm-up writes are deliberately invisible. Reuse this
                    // unpublished page once the first input statistics exist.
                    if(road_warmed) begin
                        display_page<={1'b0,write_page};
                        frame_ready<=1'b1;
                        write_page<=~write_page;
                    end else if(|road_anchor_seen) road_warmed<=1'b1;
                    source_lock<=1'b0;
                    page0<=source_page0; page1<=source_page1;
                    page2<=source_page2; page3<=source_page3;
                    cache_valid<=0; out_word_index<=0; pack_word<=0;
                    last_word_pending<=0;
                    state<=S_IDLE;
                end else begin
                    out_word_index<=out_word_index+1'b1;
                    state<=S_MAP_REQ;
                end
            end
            default: state<=S_IDLE;
        endcase
    end
end

endmodule
