// ADV7611专用I2C写主机：每次发送从地址、8位寄存器地址和8位数据。
module hdmi_i2c_write #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer I2C_FREQ = 250_000
)(
    input             clk,
    input             rst_n,
    input             exec,
    input      [23:0] write_data,
    output reg        done,
    output reg        ack_error,
    output reg        scl,
    inout             sda
);
localparam integer TICK_DIV = CLK_FREQ / (I2C_FREQ * 4);
localparam [3:0] ST_IDLE         = 4'd0,
                 ST_START        = 4'd1,
                 ST_BIT_LOW      = 4'd2,
                 ST_BIT_HIGH     = 4'd3,
                 ST_ACK_LOW      = 4'd4,
                 ST_ACK_HIGH     = 4'd5,
                 ST_NEXT_BYTE    = 4'd6,
                 ST_STOP_LOW     = 4'd7,
                 ST_STOP_HIGH    = 4'd8,
                 ST_STOP_RELEASE = 4'd9;

reg [15:0] div_cnt;
reg [3:0]  state;
reg [23:0] shift_data;
reg [2:0]  bit_cnt;
reg [1:0]  byte_cnt;
reg        sda_drive_low;
wire       tick = (div_cnt == TICK_DIV-1);
wire       sda_in = sda;
assign sda = sda_drive_low ? 1'b0 : 1'bz;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        div_cnt       <= 16'd0;
        state         <= ST_IDLE;
        shift_data    <= 24'd0;
        bit_cnt       <= 3'd0;
        byte_cnt      <= 2'd0;
        sda_drive_low <= 1'b0;
        scl           <= 1'b1;
        done          <= 1'b0;
        ack_error     <= 1'b0;
    end else begin
        done <= 1'b0;
        if (state == ST_IDLE) begin
            div_cnt <= 16'd0;
            scl <= 1'b1;
            sda_drive_low <= 1'b0;
            if (exec) begin
                shift_data <= write_data;
                bit_cnt    <= 3'd0;
                byte_cnt   <= 2'd0;
                ack_error  <= 1'b0;
                state      <= ST_START;
            end
        end else if (tick) begin
            div_cnt <= 16'd0;
            case (state)
                ST_START: begin
                    scl <= 1'b1;
                    sda_drive_low <= 1'b1;
                    state <= ST_BIT_LOW;
                end
                ST_BIT_LOW: begin
                    scl <= 1'b0;
                    sda_drive_low <= ~shift_data[23];
                    state <= ST_BIT_HIGH;
                end
                ST_BIT_HIGH: begin
                    scl <= 1'b1;
                    shift_data <= {shift_data[22:0], 1'b0};
                    if (bit_cnt == 3'd7) begin
                        bit_cnt <= 3'd0;
                        state <= ST_ACK_LOW;
                    end else begin
                        bit_cnt <= bit_cnt + 1'b1;
                        state <= ST_BIT_LOW;
                    end
                end
                ST_ACK_LOW: begin
                    scl <= 1'b0;
                    sda_drive_low <= 1'b0;
                    state <= ST_ACK_HIGH;
                end
                ST_ACK_HIGH: begin
                    scl <= 1'b1;
                    if (sda_in)
                        ack_error <= 1'b1;
                    state <= ST_NEXT_BYTE;
                end
                ST_NEXT_BYTE: begin
                    scl <= 1'b0;
                    if (byte_cnt == 2'd2) begin
                        sda_drive_low <= 1'b1;
                        state <= ST_STOP_LOW;
                    end else begin
                        byte_cnt <= byte_cnt + 1'b1;
                        state <= ST_BIT_LOW;
                    end
                end
                ST_STOP_LOW: begin
                    scl <= 1'b0;
                    sda_drive_low <= 1'b1;
                    state <= ST_STOP_HIGH;
                end
                ST_STOP_HIGH: begin
                    scl <= 1'b1;
                    sda_drive_low <= 1'b1;
                    state <= ST_STOP_RELEASE;
                end
                ST_STOP_RELEASE: begin
                    scl <= 1'b1;
                    sda_drive_low <= 1'b0;
                    done <= 1'b1;
                    state <= ST_IDLE;
                end
                default: state <= ST_IDLE;
            endcase
        end else begin
            div_cnt <= div_cnt + 1'b1;
        end
    end
end
endmodule
