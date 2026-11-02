// 语音事件优先级（高->低）：红色超声波、黄色超声波、错误方向、斑马线、夜间、蓝色箭头、单箭头。
// source_active记录事件本身，source_valid限定当前是否仍有播放意义。
// 只在事件上升沿置待播；失效即取消；已经开始的低优先级语音被抢占后不重播。
// 无播放BUSY反馈：按MP3_7实测时长加约0.5秒保护间隔估计播放结束。
module voice_event_ctrl #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer UART_BPS = 9600,
    parameter integer BOOT_MS = 1000,
    parameter integer NIGHT_MS = 2600,
    parameter integer WRONG_MS = 3000,
    parameter integer ZEBRA_MS = 3400,
    parameter integer BLUE_MS = 2600,
    parameter integer SINGLE_MS = 3400,
    parameter integer YELLOW_MS = 3200,
    parameter integer RED_MS = 3400
)(
    input wire clk, input wire rst_n,
    input wire [6:0] source_active, input wire [6:0] source_valid,
    output wire uart_txd
);
localparam integer MS_CYCLES = CLK_FREQ / 1000;
reg [31:0] ms_count;
wire ms_tick = (ms_count == MS_CYCLES - 1);
reg [15:0] boot_left, play_left;
reg [6:0] active_d, pending;
reg [2:0] current_priority;
reg tx_inflight;
wire cmd_ready, cmd_done;
wire [6:0] candidates = (pending | (source_active & ~active_d)) & source_valid;
// selected表示优先级，独立映射到文件编号；单箭头为最低优先级1。
wire [2:0] selected = candidates[4] ? 3'd7 : candidates[3] ? 3'd6 :
                      candidates[2] ? 3'd5 : candidates[1] ? 3'd4 :
                      candidates[0] ? 3'd3 : candidates[5] ? 3'd2 :
                      candidates[6] ? 3'd1 : 3'd0;
wire dispatch = (boot_left == 0) && !tx_inflight && cmd_ready &&
                (selected != 0) && ((current_priority == 0) || (selected > current_priority));
wire [7:0] track = (selected == 7) ? 8'd7 : (selected == 6) ? 8'd6 :
                   (selected == 5) ? 8'd3 : (selected == 4) ? 8'd5 :
                   (selected == 3) ? 8'd2 : (selected == 2) ? 8'd1 : 8'd4;
reg [15:0] selected_ms;
always @* begin
    case (selected)
        1: selected_ms = SINGLE_MS;
        2: selected_ms = BLUE_MS;
        7: selected_ms = RED_MS;
        6: selected_ms = YELLOW_MS;
        5: selected_ms = WRONG_MS;
        4: selected_ms = ZEBRA_MS;
        3: selected_ms = NIGHT_MS;
        default: selected_ms = NIGHT_MS;
    endcase
end
jq8900_command_tx #(.CLK_FREQ(CLK_FREQ), .UART_BPS(UART_BPS)) u_command(
    .clk(clk), .rst_n(rst_n), .start(dispatch), .track(track),
    .ready(cmd_ready), .done(cmd_done), .txd(uart_txd)
);
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ms_count <= 0; boot_left <= BOOT_MS; play_left <= 0;
        active_d <= 0; pending <= 0; current_priority <= 0; tx_inflight <= 0;
    end else begin
        active_d <= source_active;
        // 启动期间保留已实际收到的单箭头命令，待模块就绪后播报；
        // 其余事件仍只跟踪基线，抑制初始化状态误播。失效事件照常取消。
        pending <= (boot_left != 0) ? (candidates & 7'b1000000) : candidates;
        if (ms_tick) begin
            ms_count <= 0;
            if (boot_left != 0) boot_left <= boot_left - 1'b1;
            if (!tx_inflight && play_left != 0) begin
                play_left <= play_left - 1'b1;
                if (play_left == 1) current_priority <= 0;
            end
        end else ms_count <= ms_count + 1'b1;
        if (cmd_done) tx_inflight <= 0;
        if (dispatch) begin
            // 源位保持原定义，新增bit6；抢占后不恢复已经开始的低优先级播报。
            case (selected)
                3'd1: pending <= candidates & ~7'b1000000; // 单箭头，音频4
                3'd2: pending <= candidates & ~7'b0100000; // 蓝色箭头
                3'd3: pending <= candidates & ~7'b0000001; // 夜间
                3'd4: pending <= candidates & ~7'b0000010; // 斑马线
                3'd5: pending <= candidates & ~7'b0000100; // 错误方向
                3'd6: pending <= candidates & ~7'b0001000; // 黄色超声波
                3'd7: pending <= candidates & ~7'b0010000; // 红色超声波
                default: pending <= candidates;
            endcase
            current_priority <= selected;
            play_left <= selected_ms;
            tx_inflight <= 1;
        end
    end
end
endmodule

// HDMI输入域夜间语音资格判断。与图像增强本身无关，仅抑制启动误播。
// 先观察8个带有效像素的完整帧，给两帧确认的日夜检测留出稳定时间。
// 初始夜景不播报；稳定后观察到日间，才允许下一次进入夜间播报。
// HDMI重新初始化时重新建立基线；RGB/YRG/二值化使用同一夜间状态。
module voice_night_event (
    input wire clk, input wire rst_n,
    input wire frame_vs, input wire frame_de,
    input wire night_active, input wire hdmi_visible,
    output reg event_active
);
reg vs_d, frame_has_pixels, armed;
reg [3:0] valid_frames;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        vs_d <= 0; frame_has_pixels <= 0; valid_frames <= 0;
        armed <= 0; event_active <= 0;
    end else begin
        vs_d <= frame_vs;
        if (frame_de) frame_has_pixels <= 1;
        if (frame_vs && !vs_d) begin
            frame_has_pixels <= 0;
            if (frame_has_pixels && valid_frames < 8)
                valid_frames <= valid_frames + 1'b1;
        end
        if (!night_active) begin
            event_active <= 0;
            if (valid_frames == 8) armed <= 1;
        end else if (armed && hdmi_visible) begin
            // 一次夜间状态只产生一次事件，显示布局变化不重新触发。
            event_active <= 1;
        end
    end
end
endmodule
