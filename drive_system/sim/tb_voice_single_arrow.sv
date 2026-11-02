`timescale 1ns/1ps
// 完整UDP命令解析、异步同步、语音调度和串口帧联合回归。
module tb_voice_single_arrow;
    reg rx_clk=0, clk=0, rst_n=0;
    always #4 rx_clk=~rx_clk;
    always #5 clk=~clk;
    reg rec_en=0, done=0;
    reg [7:0] data=0;
    reg [15:0] byte_num=0;
    wire [15:0] command;
    wire toggle, single_arrow;
    reg single_meta=0, single_sync=0;
    reg red=0;
    wire [6:0] events={single_sync,1'b0,red,4'b0};
    wire txd;
    integer count=0, serial_count=0, b, i, before_count, mask, direction, color;
    integer tracks[0:95];
    reg [7:0] serial_bytes[0:1439];

    udp_command_decoder decoder(.clk(rx_clk), .rst_n(rst_n),
        .rec_pkt_done(done), .rec_en(rec_en), .rec_data(data),
        .rec_byte_num(byte_num), .command_word(command),
        .command_toggle(toggle), .single_arrow_active(single_arrow));
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin single_meta<=0; single_sync<=0; end
        else begin single_meta<=single_arrow; single_sync<=single_meta; end
    voice_event_ctrl #(.CLK_FREQ(100_000), .UART_BPS(10_000),
        .BOOT_MS(20), .SINGLE_MS(3), .RED_MS(3)) voice(
        .clk(clk), .rst_n(rst_n), .source_active(events),
        .source_valid(events), .uart_txd(txd));
    always @(posedge clk) if(rst_n && voice.dispatch) begin
        tracks[count]=voice.track;
        count=count+1;
    end
    always begin
        @(negedge txd);
        repeat(5) @(posedge clk);
        if(txd!==0) $fatal(1,"UART start bit");
        for(b=0;b<8;b=b+1) begin
            repeat(10) @(posedge clk);
            serial_bytes[serial_count][b]=txd;
        end
        repeat(10) @(posedge clk);
        if(txd!==1) $fatal(1,"UART stop bit");
        serial_count=serial_count+1;
    end
    task send(input [15:0] word_value, input integer packet_len);
        integer p;
        begin
            for(p=0;p<packet_len;p=p+1) begin
                @(negedge rx_clk); rec_en=1;
                data=(p==0)?word_value[7:0]:word_value[15:8];
                done=(p==packet_len-1); byte_num=packet_len;
            end
            @(negedge rx_clk); rec_en=0; done=0;
            repeat(10) @(negedge clk);
        end
    endtask
    task settle;
        begin repeat(3000) @(negedge clk); end
    endtask
    task check_count(input integer expected);
        if(count!=expected) $fatal(1,"Got %0d play commands, expected %0d",count,expected);
    endtask
    initial begin
        repeat(4) @(negedge clk); rst_n=1;
        // 启动等待期内实际收到的合法命令，应等模块就绪后播放。
        send(16'h0011,2); // 左箭头绿色
        repeat(100) @(negedge clk); check_count(0);
        settle(); check_count(1);
        if(tracks[0]!=4) $fatal(1,"Single arrow did not select file 4");
        send(16'h0011,2); send(16'h00c2,2); send(16'h030c,2);
        settle(); check_count(1); // 换右/直行、换颜色、加边框均不重播
        // 非法保留位、非法处理模式、错误长度不改变合法单箭头状态。
        send(16'h1000,2); send(16'h0c00,2); send(16'h0000,1);
        send(16'h0000,3); settle(); check_count(1);
        if(!single_sync) $fatal(1,"Invalid packet changed single-arrow state");
        send(16'h0003,2); send(16'h0002,2); settle(); check_count(2);
        send(16'h0000,2); send(16'h0004,2); settle(); check_count(3);
        // 蓝色不属于合格单箭头，切回红/绿/黄允许再次触发。
        send(16'h0204,2); settle(); check_count(3);
        if(single_sync) $fatal(1,"Blue straight arrow qualified for file 4");
        send(16'h0004,2); settle(); check_count(4);
        // 穷举每个方向的四种颜色，检查对应颜色位解码，不受隐藏箭头颜色干扰。
        for(direction=0;direction<3;direction=direction+1) begin
            for(color=0;color<4;color=color+1) begin
                send(16'h0000,2); before_count=count;
                send((16'h02a0 & ~(16'h0030 << (direction*2))) |
                     (color << (4+direction*2)) | (1 << direction),2);
                settle(); check_count(before_count+((color==2)?0:1));
                if(single_sync !== (color!=2))
                    $fatal(1,"Wrong color qualification direction=%0d color=%0d",direction,color);
            end
        end
        // 穷举3个方向显示位，只有001、010、100允许触发。
        for(mask=0;mask<8;mask=mask+1) begin
            send(16'h0000,2); before_count=count;
            send(mask,2); settle();
            check_count(before_count+((mask==1 || mask==2 || mask==4)?1:0));
        end
        // 高优先级播放期间，已经失效的待播单箭头必须取消。
        send(16'h0000,2); red=1;
        repeat(20) @(negedge clk); before_count=count;
        send(16'h0001,2); send(16'h0003,2); settle();
        check_count(before_count); red=0; settle(); check_count(before_count);
        // 仍有效的单箭头等红色告警播报结束后播放。
        red=1; repeat(20) @(negedge clk); before_count=count;
        send(16'h0001,2); settle(); settle();
        check_count(before_count+1);
        if(tracks[count-1]!=4) $fatal(1,"Valid pending single arrow not played");
        // 已开始的单箭头播报允许红色告警打断，之后不恢复/重播。
        red=0; send(16'h0000,2); before_count=count;
        send(16'h0001,2);
        repeat(20) @(negedge clk); red=1;
        settle(); settle(); check_count(before_count+2);
        if(tracks[before_count]!=4 || tracks[before_count+1]!=7)
            $fatal(1,"Red warning did not preempt single-arrow audio");
        red=0; settle(); check_count(before_count+2);
        // 检查每个实际UART文件名帧及校验和，第4段必须发送/00004*???。
        if(serial_count!=count*15) $fatal(1,"Incomplete UART commands");
        for(i=0;i<count;i=i+1) begin : check_frames
            reg [79:0] path;
            reg [7:0] sum;
            integer p;
            path="/00000*???"; path[39:32]=8'h30+tracks[i];
            sum=8'haa+8'h08+8'h0b+8'h02;
            if(serial_bytes[i*15]!=8'haa || serial_bytes[i*15+1]!=8'h08 ||
               serial_bytes[i*15+2]!=8'h0b || serial_bytes[i*15+3]!=8'h02)
                $fatal(1,"Bad UART header");
            for(p=0;p<10;p=p+1) begin
                if(serial_bytes[i*15+4+p]!==path[79-p*8 -: 8])
                    $fatal(1,"Wrong filename");
                sum=sum+path[79-p*8 -: 8];
            end
            if(serial_bytes[i*15+14]!==sum) $fatal(1,"Bad UART checksum");
        end
        $display("PASS: red/green/yellow single arrows, blue exclusion for all directions, transitions, invalid packets, boot, priority, cancellation and UART file 4");
        $finish;
    end
    initial begin #5_000_000; $fatal(1,"Timeout"); end
endmodule
