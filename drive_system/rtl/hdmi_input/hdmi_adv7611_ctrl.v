// ADV7611初始化控制。使用独立I2C写主机，避免与摄像头SCCB模块重名。
module hdmi_adv7611_ctrl #(
    parameter CLK_FREQ = 26'd50_000_000,
    parameter I2C_FREQ = 18'd250_000
)(
    input  clk,
    input  rst_n,
    output scl,
    inout  sda,
    output init_done
);
wire        i2c_exec;
wire        i2c_done;
wire [23:0] i2c_data;

hdmi_i2c_write #(
    .CLK_FREQ (CLK_FREQ),
    .I2C_FREQ (I2C_FREQ)
) u_hdmi_i2c_dri (
    .clk        (clk),
    .rst_n      (rst_n),
    .exec       (i2c_exec),
    .write_data(i2c_data),
    .done       (i2c_done),
    .ack_error  (),
    .scl        (scl),
    .sda        (sda)
);

i2c_adv7611_cfg u_hdmi_i2c_cfg (
    .clk       (clk),
    .rst_n     (rst_n),
    .i2c_done  (i2c_done),
    .i2c_exec  (i2c_exec),
    .i2c_data  (i2c_data),
    .init_done (init_done)
);
endmodule
