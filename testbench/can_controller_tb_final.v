`timescale 1ns/10ps

module CAN_CONTROLLER_TB;

    localparam BIT_CLKS = 40;
`ifdef GATE
    localparam B_MBOX = 2, B_FILT = 1, B_DEPTH = 1;
`else
    localparam B_MBOX = 3, B_FILT = 2, B_DEPTH = 3;
`endif
    localparam NA = 0, NB = 1, NC = 2;

    reg clk_a = 1'b0, clk_b = 1'b0, reset = 1'b1;
    always #5    clk_a = ~clk_a;
    always #5.05 clk_b = ~clk_b;

    reg  [7:0] a_addr = 0, a_wdata = 0, b_addr = 0, b_wdata = 0, c_addr = 0, c_wdata = 0;
    reg        a_we = 0, b_we = 0, c_we = 0;
    wire [7:0] a_rdata, b_rdata, c_rdata;
    wire       a_irq, b_irq, c_irq, a_tx, b_tx, c_tx;
    reg        jam_n = 1'b1;
    wire       bus = a_tx & b_tx & c_tx & jam_n;

    CAN_CONTROLLER nodeA (.clk(clk_a), .reset(reset), .addr(a_addr), .wdata(a_wdata),
                          .we(a_we), .rdata(a_rdata), .irq(a_irq), .rx(bus), .tx(a_tx));
`ifdef GATE
    CAN_CONTROLLER nodeB (.clk(clk_b), .reset(reset), .addr(b_addr), .wdata(b_wdata),
                          .we(b_we), .rdata(b_rdata), .irq(b_irq), .rx(bus), .tx(b_tx));
`else
    CAN_CONTROLLER #(.NUM_TX_MBOX(B_MBOX), .NUM_FILTERS(B_FILT), .RX_DEPTH(B_DEPTH))
                   nodeB (.clk(clk_b), .reset(reset), .addr(b_addr), .wdata(b_wdata),
                          .we(b_we), .rdata(b_rdata), .irq(b_irq), .rx(bus), .tx(b_tx));
`endif
    CAN_CONTROLLER nodeC (.clk(clk_a), .reset(reset), .addr(c_addr), .wdata(c_wdata),
                          .we(c_we), .rdata(c_rdata), .irq(c_irq), .rx(bus), .tx(c_tx));

    integer errors = 0;
    reg [7:0] d, d2;

    task wr(input integer n, input [7:0] ad, input [7:0] dat);
        begin
            if (n == NA) begin
                @(negedge clk_a); a_addr = ad; a_wdata = dat; a_we = 1;
                @(negedge clk_a); a_we = 0;
            end
            else if (n == NB) begin
                @(negedge clk_b); b_addr = ad; b_wdata = dat; b_we = 1;
                @(negedge clk_b); b_we = 0;
            end
            else begin
                @(negedge clk_a); c_addr = ad; c_wdata = dat; c_we = 1;
                @(negedge clk_a); c_we = 0;
            end
        end
    endtask

    task rd(input integer n, input [7:0] ad, output [7:0] dat);
        begin
            if (n == NA)      begin @(negedge clk_a); a_addr = ad; #1 dat = a_rdata; end
            else if (n == NB) begin @(negedge clk_b); b_addr = ad; #1 dat = b_rdata; end
            else              begin @(negedge clk_a); c_addr = ad; #1 dat = c_rdata; end
        end
    endtask

    task check(input cond, input [8*80-1:0] msg);
        begin
            if (cond) $display("  ok    %0s", msg);
            else begin $display("  FAIL  %0s", msg); errors = errors + 1; end
        end
    endtask

    function [28:0] sid(input [10:0] id);
        sid = {id, 18'd0};
    endfunction

    task load_mb(input integer n, input integer mb, input [28:0] id, input ide, input rtr,
                 input [3:0] dlc, input [63:0] data);
        reg [7:0] base; integer j;
        begin
            base = 8'h10 + 8'h10 * mb;
            wr(n, base + 1, {3'b000, id[28:24]});
            wr(n, base + 2, id[23:16]);
            wr(n, base + 3, id[15:8]);
            wr(n, base + 4, id[7:0]);
            wr(n, base + 5, {ide, rtr, 2'b00, dlc});
            for (j = 0; j < 8; j = j + 1) wr(n, base + 6 + j, data[63-8*j -: 8]);
        end
    endtask

    task request(input integer n, input integer mb);
        wr(n, 8'h10 + 8'h10 * mb, 8'h01);
    endtask

    task wait_tx(input integer n, input integer mb, input integer max_bits, output ok);
        integer t;
        begin
            ok = 0;
            for (t = 0; t < max_bits * 4 && !ok; t = t + 1) begin
                repeat (BIT_CLKS / 4) @(posedge clk_a);
                rd(n, 8'h10 + 8'h10 * mb, d);
                if (!d[0]) ok = 1;
            end
        end
    endtask

    task wait_rx(input integer n, input integer max_bits, output ok);
        integer t;
        begin
            ok = 0;
            for (t = 0; t < max_bits * 4 && !ok; t = t + 1) begin
                repeat (BIT_CLKS / 4) @(posedge clk_a);
                rd(n, 8'h01, d);
                if (d[5]) ok = 1;
            end
        end
    endtask

    task read_rx(input integer n, output [28:0] id, output ide, output rtr,
                 output [3:0] dlc, output [63:0] data, output [3:0] hit);
        integer j;
        begin
            rd(n, 8'h51, d); id[28:24] = d[4:0];
            rd(n, 8'h52, d); id[23:16] = d;
            rd(n, 8'h53, d); id[15:8]  = d;
            rd(n, 8'h54, d); id[7:0]   = d;
            rd(n, 8'h55, d); ide = d[7]; rtr = d[6]; dlc = d[3:0];
            for (j = 0; j < 8; j = j + 1) begin rd(n, 8'h56 + j, d); data[63-8*j -: 8] = d; end
            rd(n, 8'h5E, d); hit = d[3:0];
        end
    endtask

    task expect_rx(input integer n, input [28:0] id, input ide, input rtr, input [3:0] dlc,
                   input [63:0] data, input [8*80-1:0] msg);
        reg ok; reg [28:0] g_id; reg g_ide, g_rtr; reg [3:0] g_dlc, g_hit; reg [63:0] g_data;
        begin
            wait_rx(n, 400, ok);
            if (!ok) check(0, msg);
            else begin
                read_rx(n, g_id, g_ide, g_rtr, g_dlc, g_data, g_hit);
                wr(n, 8'h50, 8'h01);
                if (g_id == id && g_ide == ide && g_rtr == rtr && g_dlc == dlc && g_data == data)
                    check(1, msg);
                else begin
                    check(0, msg);
                    $display("        got id=%h ide=%b rtr=%b dlc=%0d data=%h", g_id, g_ide, g_rtr, g_dlc, g_data);
                    $display("        exp id=%h ide=%b rtr=%b dlc=%0d data=%h", id, ide, rtr, dlc, data);
                end
            end
        end
    endtask

    reg     exp_bits [0:255];
    integer exp_len;

    task build_frame(input [28:0] id, input ide, input rtr, input [3:0] dlc,
                     input [63:0] data, input ack);
        reg raw [0:255]; integer rl, j, run; reg [14:0] crc; reg nxt, last;
        begin
            rl = 0;
            raw[rl] = 1'b0; rl = rl + 1;
            for (j = 28; j >= 18; j = j - 1) begin raw[rl] = id[j]; rl = rl + 1; end
            if (ide) begin
                raw[rl] = 1'b1; rl = rl + 1;
                raw[rl] = 1'b1; rl = rl + 1;
                for (j = 17; j >= 0; j = j - 1) begin raw[rl] = id[j]; rl = rl + 1; end
                raw[rl] = rtr;  rl = rl + 1;
                raw[rl] = 1'b0; rl = rl + 1;
            end
            else begin
                raw[rl] = rtr;  rl = rl + 1;
                raw[rl] = 1'b0; rl = rl + 1;
            end
            raw[rl] = 1'b0; rl = rl + 1;
            for (j = 3; j >= 0; j = j - 1) begin raw[rl] = dlc[j]; rl = rl + 1; end
            if (!rtr)
                for (j = 0; j < ((dlc > 8) ? 64 : dlc * 8); j = j + 1) begin
                    raw[rl] = data[63 - j]; rl = rl + 1;
                end
            crc = 15'd0;
            for (j = 0; j < rl; j = j + 1) begin
                nxt = raw[j] ^ crc[14];
                crc = {crc[13:0], 1'b0} ^ (nxt ? 15'h4599 : 15'h0000);
            end
            for (j = 14; j >= 0; j = j - 1) begin raw[rl] = crc[j]; rl = rl + 1; end
            exp_len = 0; run = 0; last = 1'b0;
            for (j = 0; j < rl; j = j + 1) begin
                exp_bits[exp_len] = raw[j]; exp_len = exp_len + 1;
                if (j == 0 || raw[j] != last) begin last = raw[j]; run = 1; end
                else run = run + 1;
                if (run == 5) begin
                    exp_bits[exp_len] = ~last; exp_len = exp_len + 1;
                    last = ~last; run = 1;
                end
            end
            exp_bits[exp_len] = 1'b1;  exp_len = exp_len + 1;
            exp_bits[exp_len] = !ack;  exp_len = exp_len + 1;
            for (j = 0; j < 8; j = j + 1) begin
                exp_bits[exp_len] = 1'b1; exp_len = exp_len + 1;
            end
        end
    endtask

    reg     mon_arm = 1'b0, mon_done = 1'b0;
    reg     cap [0:255];
    integer mi;
    always begin
        wait (mon_arm);
        @(negedge bus);
        mon_arm = 1'b0;
        repeat (BIT_CLKS / 2) @(posedge clk_a);
        for (mi = 0; mi < exp_len; mi = mi + 1) begin
            cap[mi] = bus;
            if (mi < exp_len - 1) repeat (BIT_CLKS) @(posedge clk_a);
        end
        mon_done = 1'b1;
    end

    task check_stream(input [8*80-1:0] msg);
        integer j, bad;
        begin
            wait (mon_done);
            mon_done = 1'b0;
            bad = -1;
            for (j = exp_len - 1; j >= 0; j = j - 1) if (cap[j] !== exp_bits[j]) bad = j;
            check(bad == -1, msg);
            if (bad != -1) $display("        first mismatch at bit %0d of %0d", bad, exp_len);
        end
    endtask

    reg     jam_en = 1'b0;
    integer jam_idx, idle_cnt;
    always begin
        wait (jam_en);
        idle_cnt = 0;
        while (idle_cnt < 10 * BIT_CLKS) begin
            @(posedge clk_a);
            if (bus) idle_cnt = idle_cnt + 1; else idle_cnt = 0;
        end
        @(negedge bus);
        if (jam_en) begin
            repeat (jam_idx * BIT_CLKS) @(posedge clk_a);
            jam_n = 1'b0;
            repeat (BIT_CLKS) @(posedge clk_a);
            jam_n = 1'b1;
        end
    end

    task enable_node(input integer n);
        integer t;
        begin
            wr(n, 8'h00, 8'h01);
            d = 0;
            for (t = 0; t < 100 && !d[7]; t = t + 1) begin
                repeat (BIT_CLKS / 2) @(posedge clk_a);
                rd(n, 8'h01, d);
            end
        end
    endtask

    reg ok, ok2;
    reg [28:0] g_id; reg g_ide, g_rtr; reg [3:0] g_dlc, g_hit; reg [63:0] g_data;
    integer i, t, cnt;

    initial begin
      `ifdef VCD_FILE
        $dumpfile(`VCD_FILE);
      `else
        $dumpfile("dump.vcd");
      `endif
        $dumpvars(0, CAN_CONTROLLER_TB);

        repeat (5) @(posedge clk_b);
        reset = 1'b0;

        $display("T1  register defaults and read/write");
        rd(NA, 8'h04, d); check(d == 8'h43, "BTR0 reset = 0x43 (SJW 2, BRP 3)");
        rd(NA, 8'h05, d); check(d == 8'h11, "BTR1 reset = 0x11 (PROP 2, PH1 3)");
        rd(NA, 8'h06, d); check(d == 8'h03, "BTR2 reset = 0x03 (PH2 4)");
        wr(NA, 8'h02, 8'hA5); rd(NA, 8'h02, d); check(d == 8'hA5, "INT_EN write/read");
        wr(NA, 8'h02, 8'hFF);
        load_mb(NA, 0, 29'h12345678, 1'b1, 1'b0, 4'd5, 64'h0102030405060708);
        rd(NA, 8'h12, d); ok = (d == 8'h34);
        rd(NA, 8'h15, d); ok = ok && (d == 8'h85);
        rd(NA, 8'h1A, d); check(ok && d == 8'h05, "mailbox ID/control/data readback");

        $display("T2  loopback self-test (node A alone)");
        wr(NA, 8'h00, 8'h03);
        repeat (15 * BIT_CLKS) @(posedge clk_a);
        load_mb(NA, 0, sid(11'h3C5), 1'b0, 1'b0, 4'd3, 64'hDEADBE0000000000);
        request(NA, 0);
        wait_tx(NA, 0, 300, ok); check(ok, "loopback frame sent without ACK error");
        check(bus === 1'b1, "tx pin stays recessive in loopback");
        expect_rx(NA, sid(11'h3C5), 1'b0, 1'b0, 4'd3, 64'hDEADBE0000000000, "loopback frame received");
        rd(NA, 8'h07, d); check(d == 0, "TEC = 0 after loopback");
        wr(NA, 8'h00, 8'h00);
        wr(NA, 8'h03, 8'hFF);

        $display("T3  A -> B standard data frame, exact bitstream");
        enable_node(NA); enable_node(NB); enable_node(NC);
        rd(NB, 8'h01, d); check(d[7], "all nodes integrated (bus idle)");
        wr(NB, 8'h02, 8'hFF);
        load_mb(NA, 0, sid(11'h123), 1'b0, 1'b0, 4'd2, 64'h55EE000000000000);
        build_frame(sid(11'h123), 1'b0, 1'b0, 4'd2, 64'h55EE000000000000, 1'b1);
        mon_arm = 1'b1;
        request(NA, 0);
        wait_tx(NA, 0, 300, ok); check(ok, "TXREQ cleared after transmission");
        check_stream("bus bits match reference (CRC, stuffing, ACK)");
        rd(NA, 8'h03, d); check(d[0] && a_irq, "A: TX0 done flag and irq");
        rd(NB, 8'h03, d); check(d[4] && b_irq, "B: RX flag and irq");
        expect_rx(NB, sid(11'h123), 1'b0, 1'b0, 4'd2, 64'h55EE000000000000, "B received the frame");
        expect_rx(NC, sid(11'h123), 1'b0, 1'b0, 4'd2, 64'h55EE000000000000, "C received the frame");
        wr(NA, 8'h03, 8'hFF); wr(NB, 8'h03, 8'hFF); wr(NC, 8'h03, 8'hFF);
        check(!a_irq && !b_irq, "INT_FLAG write-1-to-clear drops irq");

        $display("T4  A -> B extended 8-byte frame, exact bitstream");
        load_mb(NA, 0, 29'h12345678, 1'b1, 1'b0, 4'd8, 64'h0000FFFF00FF55AA);
        build_frame(29'h12345678, 1'b1, 1'b0, 4'd8, 64'h0000FFFF00FF55AA, 1'b1);
        mon_arm = 1'b1;
        request(NA, 0);
        wait_tx(NA, 0, 300, ok); check(ok, "extended frame sent");
        check_stream("bus bits match reference");
        expect_rx(NB, 29'h12345678, 1'b1, 1'b0, 4'd8, 64'h0000FFFF00FF55AA, "B received the extended frame");
        expect_rx(NC, 29'h12345678, 1'b1, 1'b0, 4'd8, 64'h0000FFFF00FF55AA, "C received the extended frame");

        $display("T5  remote frames and DLC 0");
        load_mb(NA, 0, sid(11'h7EF), 1'b0, 1'b1, 4'd4, 64'hFFFFFFFFFFFFFFFF);
        build_frame(sid(11'h7EF), 1'b0, 1'b1, 4'd4, 64'h0, 1'b1);
        mon_arm = 1'b1;
        request(NA, 0);
        wait_tx(NA, 0, 300, ok);
        check_stream("standard remote frame: DLC 4, no data field");
        expect_rx(NB, sid(11'h7EF), 1'b0, 1'b1, 4'd4, 64'h0, "B received remote frame");
        expect_rx(NC, sid(11'h7EF), 1'b0, 1'b1, 4'd4, 64'h0, "C received remote frame");
        load_mb(NA, 0, 29'h1FFFFFFF, 1'b1, 1'b0, 4'd0, 64'h0);
        build_frame(29'h1FFFFFFF, 1'b1, 1'b0, 4'd0, 64'h0, 1'b1);
        mon_arm = 1'b1;
        request(NA, 0);
        wait_tx(NA, 0, 300, ok);
        check_stream("extended data frame with DLC 0");
        expect_rx(NB, 29'h1FFFFFFF, 1'b1, 1'b0, 4'd0, 64'h0, "B received DLC 0 frame");
        expect_rx(NC, 29'h1FFFFFFF, 1'b1, 1'b0, 4'd0, 64'h0, "C received DLC 0 frame");

        $display("T6  B -> A (B's clock is 1%% slow)");
        load_mb(NB, 0, sid(11'h0F0), 1'b0, 1'b0, 4'd8, 64'h00000000FFFFFFFF);
        request(NB, 0);
        wait_tx(NB, 0, 300, ok); check(ok, "B's frame sent");
        expect_rx(NA, sid(11'h0F0), 1'b0, 1'b0, 4'd8, 64'h00000000FFFFFFFF, "A received B's frame");
        expect_rx(NC, sid(11'h0F0), 1'b0, 1'b0, 4'd8, 64'h00000000FFFFFFFF, "C received B's frame");
        load_mb(NB, 0, 29'h0ABCDEF1, 1'b1, 1'b0, 4'd8, 64'h8000000000000001);
        request(NB, 0);
        wait_tx(NB, 0, 300, ok);
        expect_rx(NA, 29'h0ABCDEF1, 1'b1, 1'b0, 4'd8, 64'h8000000000000001, "A received B's extended frame");
        expect_rx(NC, 29'h0ABCDEF1, 1'b1, 1'b0, 4'd8, 64'h8000000000000001, "C received B's extended frame");

        $display("T7  arbitration: A and B start in the same bit");

        wr(NA, 8'h81, 8'h00); wr(NA, 8'h82, 8'h00); wr(NA, 8'h83, 8'h00); wr(NA, 8'h84, 8'h00);
        wr(NA, 8'h85, 8'h10); wr(NA, 8'h86, 8'h00); wr(NA, 8'h87, 8'h00); wr(NA, 8'h88, 8'h00);
        wr(NA, 8'h80, 8'h01);

        load_mb(NA, 0, sid(11'h155), 1'b0, 1'b0, 4'd1, 64'hAA00000000000000);
        load_mb(NB, 0, sid(11'h154), 1'b0, 1'b0, 4'd1, 64'hBB00000000000000);
        load_mb(NC, 0, sid(11'h7FF), 1'b0, 1'b0, 4'd8, 64'h1111111111111111);
        request(NC, 0);
        rd(NC, 8'h01, d);
        while (!d[0]) rd(NC, 8'h01, d);
        request(NA, 0); request(NB, 0);
        wait_tx(NB, 0, 400, ok);
        wr(NB, 8'h50, 8'h01);
        rd(NA, 8'h10, d); check(ok && d[0] && d[1], "B (0x154) won; A still pending, lost flag set");
        wait_tx(NA, 0, 400, ok); check(ok, "A retransmitted after losing");
        expect_rx(NA, sid(11'h154), 1'b0, 1'b0, 4'd1, 64'hBB00000000000000, "A: received winner B");
        expect_rx(NB, sid(11'h155), 1'b0, 1'b0, 4'd1, 64'hAA00000000000000, "B: received A after");
        for (i = 0; i < 3; i = i + 1) wr(NC, 8'h50, 8'h01);

        load_mb(NA, 0, 29'h0AA80001, 1'b1, 1'b0, 4'd1, 64'hEE00000000000000);
        load_mb(NB, 0, sid(11'h2AA),  1'b0, 1'b0, 4'd1, 64'h5500000000000000);
        request(NC, 0);
        rd(NC, 8'h01, d);
        while (!d[0]) rd(NC, 8'h01, d);
        request(NA, 0); request(NB, 0);
        wait_tx(NB, 0, 400, ok);
        wr(NB, 8'h50, 8'h01);
        rd(NA, 8'h10, d); check(ok && d[0] && d[1], "standard 0x2AA beat extended with base 0x2AA");
        wait_tx(NA, 0, 400, ok); check(ok, "extended frame followed");
        expect_rx(NA, sid(11'h2AA), 1'b0, 1'b0, 4'd1, 64'h5500000000000000, "A: received standard winner");
        expect_rx(NB, 29'h0AA80001, 1'b1, 1'b0, 4'd1, 64'hEE00000000000000, "B: received extended frame");
        for (i = 0; i < 3; i = i + 1) wr(NC, 8'h50, 8'h01);
        rd(NA, 8'h07, d); rd(NB, 8'h07, d2);
        check(d == 0 && d2 == 0, "no errors: TEC = 0 on A and B");

        if (B_MBOX >= 3) begin
            $display("T8  mailbox priority on B (lowest ID first)");
            load_mb(NB, 0, sid(11'h300), 1'b0, 1'b0, 4'd1, 64'h3000000000000000);
            load_mb(NB, 1, sid(11'h100), 1'b0, 1'b0, 4'd1, 64'h1000000000000000);
            load_mb(NB, 2, sid(11'h200), 1'b0, 1'b0, 4'd1, 64'h2000000000000000);
            request(NC, 0);
            rd(NC, 8'h01, d);
            while (!d[0]) rd(NC, 8'h01, d);
            request(NB, 0); request(NB, 1); request(NB, 2);
            expect_rx(NA, sid(11'h100), 1'b0, 1'b0, 4'd1, 64'h1000000000000000, "first 0x100 (mailbox 1)");
            expect_rx(NA, sid(11'h200), 1'b0, 1'b0, 4'd1, 64'h2000000000000000, "then 0x200 (mailbox 2)");
            expect_rx(NA, sid(11'h300), 1'b0, 1'b0, 4'd1, 64'h3000000000000000, "then 0x300 (mailbox 0)");
            rd(NB, 8'h03, d); check(d[2:0] == 3'b111, "B: TX done flags for all three mailboxes");
            wr(NB, 8'h03, 8'hFF);
            wr(NB, 8'h50, 8'h01);
            for (i = 0; i < 4; i = i + 1) wr(NC, 8'h50, 8'h01);
        end

        wr(NA, 8'h80, 8'h00);
        $display("T9  acceptance filters on B");

        wr(NB, 8'h81, 8'h04); wr(NB, 8'h82, 8'h80); wr(NB, 8'h83, 8'h00); wr(NB, 8'h84, 8'h00);
        wr(NB, 8'h85, 8'h1F); wr(NB, 8'h86, 8'hC0); wr(NB, 8'h87, 8'h00); wr(NB, 8'h88, 8'h00);
        wr(NB, 8'h80, 8'h01);
        if (B_FILT >= 2) begin
            wr(NB, 8'h91, 8'h1A); wr(NB, 8'h92, 8'hBC); wr(NB, 8'h93, 8'hDE); wr(NB, 8'h94, 8'h00);
            wr(NB, 8'h95, 8'h1F); wr(NB, 8'h96, 8'hFF); wr(NB, 8'h97, 8'hFF); wr(NB, 8'h98, 8'h00);
            wr(NB, 8'h90, 8'h03);
        end
        rd(NB, 8'h82, d); check(d == 8'h80, "filter register readback");
        load_mb(NA, 0, sid(11'h135), 1'b0, 1'b0, 4'd1, 64'h0100000000000000);
        request(NA, 0); wait_tx(NA, 0, 300, ok);
        check(ok, "rejected frame is still acknowledged by B");
        load_mb(NA, 0, 29'h04840000, 1'b1, 1'b0, 4'd1, 64'h0200000000000000);
        request(NA, 0); wait_tx(NA, 0, 300, ok);
        load_mb(NA, 0, sid(11'h125), 1'b0, 1'b0, 4'd1, 64'h0300000000000000);
        request(NA, 0); wait_tx(NA, 0, 300, ok);
        load_mb(NA, 0, 29'h1ABCDE55, 1'b1, 1'b0, 4'd1, 64'h0400000000000000);
        request(NA, 0); wait_tx(NA, 0, 300, ok);
        rd(NB, 8'h50, d);
        check(d[4:2] == ((B_FILT >= 2) ? 2 : 1), "B stored only the matching frames");
        read_rx(NB, g_id, g_ide, g_rtr, g_dlc, g_data, g_hit); wr(NB, 8'h50, 8'h01);
        check(g_id == sid(11'h125) && g_hit == 4'b1000, "0x125 stored, FILHIT = filter 0");
        if (B_FILT >= 2) begin
            read_rx(NB, g_id, g_ide, g_rtr, g_dlc, g_data, g_hit); wr(NB, 8'h50, 8'h01);
            check(g_id == 29'h1ABCDE55 && g_hit == 4'b1001, "0x1ABCDE55 stored, FILHIT = filter 1");
        end
        wr(NB, 8'h80, 8'h00); wr(NB, 8'h90, 8'h00);
        for (i = 0; i < 4; i = i + 1) wr(NC, 8'h50, 8'h01);
        wr(NA, 8'h03, 8'hFF); wr(NB, 8'h03, 8'hFF);

        $display("T10 receive FIFO overflow on A (depth 1)");
        load_mb(NB, 0, sid(11'h011), 1'b0, 1'b0, 4'd1, 64'h1100000000000000);
        request(NB, 0); wait_tx(NB, 0, 300, ok);
        load_mb(NB, 0, sid(11'h022), 1'b0, 1'b0, 4'd1, 64'h2200000000000000);
        request(NB, 0); wait_tx(NB, 0, 300, ok);
        rd(NA, 8'h50, d); check(d[1] && d[4:2] == 1, "overflow flagged, one frame kept");
        rd(NA, 8'h03, d); check(d[5], "RX overflow interrupt flag");
        expect_rx(NA, sid(11'h011), 1'b0, 1'b0, 4'd1, 64'h1100000000000000, "oldest frame kept");
        wr(NA, 8'h50, 8'h02); rd(NA, 8'h50, d); check(d == 0, "overflow cleared, FIFO empty");
        for (i = 0; i < 2; i = i + 1) wr(NC, 8'h50, 8'h01);
        wr(NA, 8'h03, 8'hFF);

        $display("T11 no receiver: ACK errors, error passive");
        wr(NB, 8'h00, 8'h00); wr(NC, 8'h00, 8'h00);
        load_mb(NA, 0, sid(11'h444), 1'b0, 1'b0, 4'd1, 64'h4400000000000000);
        request(NA, 0);
        ok = 0;
        for (t = 0; t < 4000 && !ok; t = t + 1) begin
            repeat (BIT_CLKS / 2) @(posedge clk_a);
            rd(NA, 8'h01, d); ok = d[3];
        end
        rd(NA, 8'h07, d); check(ok && d == 128, "error passive after 16 ACK errors (TEC = 128)");
        rd(NA, 8'h09, d); check(d == 5, "ERR_CODE = ACK error");
        rd(NA, 8'h10, d); check(d == 8'h05, "mailbox still pending with error flag");
        repeat (300 * BIT_CLKS) @(posedge clk_a);
        rd(NA, 8'h07, d); check(d == 128, "passive ACK errors do not raise TEC");
        wr(NA, 8'h10, 8'h00);
        repeat (150 * BIT_CLKS) @(posedge clk_a);
        rd(NA, 8'h01, d); check(d[7] && !d[0], "abort: bus idle, A no longer transmitting");

        $display("T12 bit errors -> bus-off -> recovery");
        wr(NA, 8'h00, 8'h00);
        enable_node(NA); enable_node(NB); enable_node(NC);
        wr(NA, 8'h03, 8'hFF);
        load_mb(NA, 0, sid(11'h0A0), 1'b0, 1'b0, 4'd2, 64'hFFFF000000000000);
        build_frame(sid(11'h0A0), 1'b0, 1'b0, 4'd2, 64'hFFFF000000000000, 1'b1);
        jam_idx = 21;
        while (exp_bits[jam_idx] !== 1'b1) jam_idx = jam_idx + 1;
        jam_en = 1'b1;
        repeat (12 * BIT_CLKS) @(posedge clk_a);
        request(NA, 0);
        ok = 0;
        for (t = 0; t < 8000 && !ok; t = t + 1) begin
            repeat (BIT_CLKS / 2) @(posedge clk_a);
            rd(NA, 8'h01, d); ok = d[4];
        end
        jam_en = 1'b0;
        rd(NA, 8'h07, d); check(ok && d == 8'hFF, "bus-off after 32 bit errors (TEC > 255)");
        rd(NA, 8'h09, d); check(d == 1, "ERR_CODE = bit error");
        rd(NA, 8'h03, d); check(d[7] && d[6], "bus-off and error interrupt flags");
        rd(NB, 8'h08, d); check(d > 0 && d < 128, "B's REC counted the errors, B still error active");
        ok2 = 0;
        for (t = 0; t < 8000 && !ok2; t = t + 1) begin
            repeat (BIT_CLKS / 2) @(posedge clk_a);
            rd(NA, 8'h01, d); ok2 = !d[4];
        end
        rd(NA, 8'h07, d);
        check(ok2 && d == 0, "recovered after 128 x 11 recessive bits, TEC = 0");
        wait_tx(NA, 0, 300, ok); check(ok, "pending frame sent after recovery");
        expect_rx(NB, sid(11'h0A0), 1'b0, 1'b0, 4'd2, 64'hFFFF000000000000, "B received it");

        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("%0d CHECK(S) FAILED", errors);
        $finish;
    end

    initial begin
        #20_000_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
