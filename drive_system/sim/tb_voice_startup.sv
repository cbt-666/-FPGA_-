`timescale 1ns/1ps
module tb_voice_startup;
    reg clk=0, rst_n=0;
    always #5 clk=~clk;
    reg vs=0, de=0, night=0, visible=1;
    wire night_event, txd;
    reg [6:0] events=0;
    integer dispatch_count=0, f;
    voice_night_event gate_dut(.clk(clk), .rst_n(rst_n), .frame_vs(vs),
        .frame_de(de), .night_active(night), .hdmi_visible(visible),
        .event_active(night_event));
    voice_event_ctrl #(.CLK_FREQ(100_000), .UART_BPS(10_000),
        .BOOT_MS(2), .NIGHT_MS(2), .RED_MS(2)) event_dut(
        .clk(clk), .rst_n(rst_n), .source_active(events),
        .source_valid(events), .uart_txd(txd));
    always @(posedge clk) if(rst_n && event_dut.dispatch) dispatch_count=dispatch_count+1;
    task frame;
        begin
            @(negedge clk); vs=1;
            repeat(2) @(negedge clk); vs=0; de=1;
            repeat(4) @(negedge clk); de=0;
            repeat(2) @(negedge clk);
        end
    endtask
    initial begin
        // Even a night event that is high throughout boot is not queued.
        events=1; night=1;
        repeat(4) @(negedge clk); rst_n=1;
        repeat(500) @(negedge clk);
        if(dispatch_count!=0 || txd!==1) $fatal(1, "Initial night was queued");
        // Initial dark HDMI may arrive after the voice boot timer has expired.
        for(f=0; f<12; f=f+1) frame();
        if(night_event!==0) $fatal(1, "Initial dark HDMI generated event");
        night=0; repeat(4) @(negedge clk);
        events=0;
        night=1; repeat(4) @(negedge clk);
        if(night_event!==1) $fatal(1, "Runtime day-to-night event missing");
        events=1;
        wait(dispatch_count==1);
        repeat(3000) @(negedge clk);
        if(dispatch_count!=1) $fatal(1, "Sustained night replayed");
        visible=0; repeat(4) @(negedge clk);
        if(night_event!==1) $fatal(1, "Night event lost on layout change");
        visible=1; repeat(4) @(negedge clk);
        if(night_event!==1) $fatal(1, "Night event changed on layout return");
        // Valid daytime initialization arms the next night transition.
        rst_n=0; events=0; night=0;
        repeat(4) @(negedge clk); rst_n=1;
        for(f=0; f<12; f=f+1) frame();
        night=1; repeat(4) @(negedge clk);
        if(night_event!==1) $fatal(1, "Daytime startup did not arm night");
        $display("PASS: boot queue suppression, late dark HDMI baseline, runtime night and layout qualification");
        $finish;
    end
    initial begin #1_000_000; $fatal(1, "Timeout"); end
endmodule
