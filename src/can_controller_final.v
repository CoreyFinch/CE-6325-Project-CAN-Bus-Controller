module CAN_CONTROLLER #(
    parameter NUM_TX_MBOX = 2,
    parameter NUM_FILTERS = 1,
    parameter RX_DEPTH    = 1
)(
    input            clk,
    input            reset,
    input      [7:0] addr,
    input      [7:0] wdata,
    input            we,
    output reg [7:0] rdata,
    output           irq,
    input            rx,
    output           tx
);

    localparam ST_OFF     = 5'd0,  ST_INTEGRATE = 5'd1,  ST_IDLE    = 5'd2,
               ST_ID_A    = 5'd3,  ST_SRR       = 5'd4,  ST_IDE     = 5'd5,
               ST_ID_B    = 5'd6,  ST_RTR       = 5'd7,  ST_R1      = 5'd8,
               ST_R0      = 5'd9,  ST_DLC       = 5'd10, ST_DATA    = 5'd11,
               ST_CRC     = 5'd12, ST_CRC_DEL   = 5'd13, ST_ACK     = 5'd14,
               ST_ACK_DEL = 5'd15, ST_EOF       = 5'd16, ST_INTER   = 5'd17,
               ST_SUSPEND = 5'd18, ST_ERR_FLAG  = 5'd19, ST_ERR_DEL = 5'd20,
               ST_BUSOFF  = 5'd21;

    localparam E_NONE = 3'd0, E_BIT = 3'd1, E_STUFF = 3'd2,
               E_CRC  = 3'd3, E_FORM = 3'd4, E_ACK  = 3'd5;

    integer i_sel, i_flt, i_wm, i_wf, i_wk, i_q, i_rf, i_rm, i_rst;

    reg        en, loopback;
    reg  [7:0] int_en, int_flag;
    reg  [5:0] brp;
    reg  [1:0] sjw_m1;
    reg  [2:0] prop_m1, ph1_m1, ph2_m1;
    reg  [2:0] err_code;

    reg        mb_req  [0:NUM_TX_MBOX-1];
    reg        mb_lost [0:NUM_TX_MBOX-1];
    reg        mb_err  [0:NUM_TX_MBOX-1];
    reg [28:0] mb_id   [0:NUM_TX_MBOX-1];
    reg        mb_ide  [0:NUM_TX_MBOX-1];
    reg        mb_rtr  [0:NUM_TX_MBOX-1];
    reg  [3:0] mb_dlc  [0:NUM_TX_MBOX-1];
    reg [63:0] mb_data [0:NUM_TX_MBOX-1];

    reg        f_en   [0:NUM_FILTERS-1];
    reg        f_ide  [0:NUM_FILTERS-1];
    reg [28:0] f_id   [0:NUM_FILTERS-1];
    reg [28:0] f_mask [0:NUM_FILTERS-1];

    reg [28:0] q_id   [0:RX_DEPTH-1];
    reg        q_ide  [0:RX_DEPTH-1];
    reg        q_rtr  [0:RX_DEPTH-1];
    reg  [3:0] q_dlc  [0:RX_DEPTH-1];
    reg [63:0] q_data [0:RX_DEPTH-1];
    reg  [3:0] q_hit  [0:RX_DEPTH-1];
    reg  [2:0] q_wr, q_rd, q_count;
    reg        q_ovf;

    reg  [4:0] state;
    reg  [6:0] bcnt;
    reg        tx_int;
    reg        tx_sof;
    reg        transmitting;
    reg        was_tx;
    reg  [1:0] cur_mb;
    reg        s_on, s_last;
    reg  [2:0] s_cnt;
    reg [28:0] r_id;
    reg        r_srr, r_ide, r_rtr;
    reg  [3:0] r_dlc;
    reg [63:0] r_data;
    reg  [6:0] align_cnt;
    reg  [8:0] tec;
    reg  [7:0] rec;
    reg  [6:0] boff_cnt;
    reg        rx_meta, rx_s;
    reg        ev_tx_ok, ev_tx_err, ev_arb_lost, ev_rx_frame, ev_err, ev_busoff;
    reg  [2:0] ev_code;

    wire passive     = (tec > 9'd127) || (rec > 8'd127);
    wire warning     = (tec >= 9'd96) || (rec >= 8'd96);
    wire in_arb      = (state >= ST_ID_A) && (state <= ST_RTR);
    wire in_stuffed  = (state >= ST_ID_A) && (state <= ST_CRC);
    wire in_frame    = (state >= ST_ID_A) && (state <= ST_EOF);
    wire bus_idle_st = (state == ST_IDLE) || (state == ST_INTER) || (state == ST_SUSPEND);
    wire stuff_next  = s_on && (s_cnt == 3'd5);
    wire [6:0] r_bits = (r_dlc > 4'd8) ? 7'd64 : {r_dlc, 3'b000};

    wire tx_point, sample_point, sampled;
    wire hard_sync_en = (state == ST_INTEGRATE) || bus_idle_st || (state == ST_BUSOFF);

    CAN_BTL btl (
        .clk(clk), .reset(reset), .enable(en),
        .brp(brp), .sjw_m1(sjw_m1), .prop_m1(prop_m1), .ph1_m1(ph1_m1), .ph2_m1(ph2_m1),
        .rx_s(rx_s), .hard_sync_en(hard_sync_en),
        .resync_en(!hard_sync_en && state != ST_OFF),
        .tx_point(tx_point), .sample_point(sample_point), .sampled(sampled)
    );

    wire [14:0] crc;
    CAN_CRC15 crc_gen (
        .clk(clk), .reset(reset),
        .clear(sample_point && bus_idle_st && !sampled),
        .enable(sample_point && in_stuffed && !stuff_next),
        .bit_in(sampled), .crc(crc)
    );

    wire [28:0] t_id   = mb_id[cur_mb];
    wire        t_ide  = mb_ide[cur_mb];
    wire        t_rtr  = mb_rtr[cur_mb];
    wire  [3:0] t_dlc  = mb_dlc[cur_mb];
    wire [63:0] t_data = mb_data[cur_mb];
    wire        tx_busy = transmitting || tx_sof;

    reg frame_bit;
    always @(*) begin
        case (state)
            ST_ID_A: frame_bit = t_id[28 - bcnt];
            ST_SRR:  frame_bit = t_ide | t_rtr;
            ST_IDE:  frame_bit = t_ide;
            ST_ID_B: frame_bit = t_id[17 - bcnt];
            ST_RTR:  frame_bit = t_rtr;
            ST_DLC:  frame_bit = t_dlc[3 - bcnt];
            ST_DATA: frame_bit = t_data[63 - bcnt];
            ST_CRC:  frame_bit = crc[14];
            default: frame_bit = 1'b0;
        endcase
    end

    reg        sel_valid;
    reg  [1:0] sel_idx;
    reg [31:0] sel_key, key;
    always @(*) begin
        sel_valid = 1'b0;
        sel_idx   = 2'd0;
        sel_key   = 32'hFFFFFFFF;
        for (i_sel = 0; i_sel < NUM_TX_MBOX; i_sel = i_sel + 1) begin

            key = {mb_id[i_sel][28:18], mb_ide[i_sel] | mb_rtr[i_sel], mb_ide[i_sel],
                   mb_id[i_sel][17:0] & {18{mb_ide[i_sel]}}, mb_rtr[i_sel] & mb_ide[i_sel]};
            if (mb_req[i_sel] && (!sel_valid || key < sel_key)) begin
                sel_valid = 1'b1;
                sel_idx   = i_sel;
                sel_key   = key;
            end
        end
    end

    reg        any_en, hit;
    reg  [2:0] hit_idx;
    reg [28:0] fmask;
    always @(*) begin
        any_en  = 1'b0;
        hit     = 1'b0;
        hit_idx = 3'd0;
        fmask   = 29'd0;
        for (i_flt = NUM_FILTERS-1; i_flt >= 0; i_flt = i_flt - 1) begin
            fmask = {f_mask[i_flt][28:18], f_mask[i_flt][17:0] & {18{f_ide[i_flt]}}};
            if (f_en[i_flt]) begin
                any_en = 1'b1;
                if (f_ide[i_flt] == r_ide && ((r_id ^ f_id[i_flt]) & fmask) == 29'd0) begin
                    hit     = 1'b1;
                    hit_idx = i_flt;
                end
            end
        end
    end
    wire rx_accept = !any_en || hit;
    wire rx_push   = ev_rx_frame && rx_accept && (q_count != RX_DEPTH);
    wire rx_ovf    = ev_rx_frame && rx_accept && (q_count == RX_DEPTH);
    wire rx_pop    = we && (addr == 8'h50) && wdata[0] && (q_count != 3'd0);

    reg [2:0] err_type;
    always @(*) begin
        err_type = E_NONE;
        if (stuff_next) begin
            if (sampled == s_last) err_type = E_STUFF;
        end
        else if (!sampled && (state == ST_CRC_DEL || state == ST_ACK_DEL ||
                 (state == ST_EOF && (bcnt != 7'd6 || transmitting))))
            err_type = E_FORM;
        else if (transmitting && state == ST_ACK && sampled && !loopback)
            err_type = E_ACK;
        else if (transmitting && state != ST_ACK && tx_int != sampled && !(in_arb && tx_int))
            err_type = E_BIT;
        else if (!transmitting && state == ST_ACK_DEL && crc != 15'd0)
            err_type = E_CRC;
    end
    wire arb_lost = transmitting && in_arb && tx_int && !sampled && !stuff_next;

    assign tx = loopback ? 1'b1 : tx_int;

    always @(posedge clk) begin
        if (reset) begin
            rx_meta <= 1'b1;
            rx_s    <= 1'b1;
        end
        else begin
            rx_meta <= loopback ? tx_int : rx;
            rx_s    <= rx_meta;
        end
    end

    always @(posedge clk) begin
        ev_tx_ok <= 1'b0;  ev_tx_err <= 1'b0;  ev_arb_lost <= 1'b0;
        ev_rx_frame <= 1'b0;  ev_err <= 1'b0;  ev_busoff <= 1'b0;

        if (reset || !en) begin
            state <= ST_OFF;  bcnt <= 7'd0;  tx_int <= 1'b1;  tx_sof <= 1'b0;
            transmitting <= 1'b0;  was_tx <= 1'b0;  cur_mb <= 2'd0;
            s_on <= 1'b0;  s_last <= 1'b1;  s_cnt <= 3'd0;
            tec <= 9'd0;  rec <= 8'd0;  boff_cnt <= 7'd0;  ev_code <= E_NONE;
            align_cnt <= 7'd0;
        end
        else begin

            if (align_cnt != 7'd0) begin
                r_data    <= {r_data[62:0], 1'b0};
                align_cnt <= align_cnt - 7'd1;
            end

            if (state == ST_OFF) begin
                state <= ST_INTEGRATE;
                bcnt  <= 7'd0;
            end

            if (tx_point) begin
                tx_sof <= 1'b0;
                if (state == ST_IDLE && sel_valid) begin
                    tx_int <= 1'b0;
                    tx_sof <= 1'b1;
                    cur_mb <= sel_idx;
                end
                else if (transmitting && stuff_next)
                    tx_int <= ~s_last;
                else if (transmitting && in_stuffed)
                    tx_int <= frame_bit;
                else if (state == ST_ACK && !transmitting && crc == 15'd0)
                    tx_int <= 1'b0;
                else if (state == ST_ERR_FLAG && !passive)
                    tx_int <= 1'b0;
                else
                    tx_int <= 1'b1;
            end

            if (sample_point) begin
                case (state)
                    ST_OFF: ;
                    ST_INTEGRATE: begin
                        if (!sampled)            bcnt  <= 7'd0;
                        else if (bcnt == 7'd10) begin state <= ST_IDLE; bcnt <= 7'd0; end
                        else                     bcnt  <= bcnt + 7'd1;
                    end
                    ST_BUSOFF: begin
                        if (!sampled) bcnt <= 7'd0;
                        else if (bcnt == 7'd10) begin
                            bcnt <= 7'd0;
                            if (boff_cnt == 7'd127) begin
                                boff_cnt <= 7'd0;  tec <= 9'd0;  rec <= 8'd0;
                                state    <= ST_IDLE;
                            end
                            else boff_cnt <= boff_cnt + 7'd1;
                        end
                        else bcnt <= bcnt + 7'd1;
                    end
                    ST_IDLE, ST_INTER, ST_SUSPEND: begin
                        if (!sampled) begin
                            state <= ST_ID_A;  bcnt <= 7'd0;
                            transmitting <= tx_sof;
                            s_on  <= 1'b1;  s_last <= 1'b0;  s_cnt <= 3'd1;
                            r_id  <= 29'd0;  r_dlc <= 4'd0;
                            r_ide <= 1'b0;   r_rtr  <= 1'b0;   r_srr <= 1'b0;
                        end
                        else if (state == ST_INTER) begin
                            if (bcnt == 7'd2) begin
                                state <= (was_tx && passive) ? ST_SUSPEND : ST_IDLE;
                                bcnt  <= 7'd0;
                            end
                            else bcnt <= bcnt + 7'd1;
                        end
                        else if (state == ST_SUSPEND) begin
                            if (bcnt == 7'd7) begin state <= ST_IDLE; bcnt <= 7'd0; end
                            else bcnt <= bcnt + 7'd1;
                        end
                    end
                    ST_ERR_FLAG: begin
                        if (bcnt == 7'd5) begin state <= ST_ERR_DEL; bcnt <= 7'd0; end
                        else bcnt <= bcnt + 7'd1;
                    end
                    ST_ERR_DEL: begin
                        if (!sampled)           bcnt <= 7'd0;
                        else if (bcnt == 7'd7) begin state <= ST_INTER; bcnt <= 7'd0; end
                        else                    bcnt <= bcnt + 7'd1;
                    end
                    default: begin
                        if (err_type != E_NONE) begin
                            ev_err  <= 1'b1;
                            ev_code <= err_type;
                            s_on    <= 1'b0;
                            bcnt    <= 7'd0;
                            was_tx  <= transmitting;
                            transmitting <= 1'b0;
                            state   <= ST_ERR_FLAG;
                            if (transmitting) begin
                                ev_tx_err <= 1'b1;

                                if (!(err_type == E_ACK && passive)) begin
                                    tec <= tec + 9'd8;
                                    if (tec >= 9'd248) begin
                                        state     <= ST_BUSOFF;
                                        boff_cnt  <= 7'd0;
                                        ev_busoff <= 1'b1;
                                    end
                                end
                            end
                            else if (rec != 8'hFF) rec <= rec + 8'd1;
                        end
                        else if (stuff_next) begin
                            s_last <= sampled;
                            s_cnt  <= 3'd1;
                        end
                        else begin
                            if (arb_lost) begin
                                transmitting <= 1'b0;
                                ev_arb_lost  <= 1'b1;
                            end
                            if (in_stuffed) begin
                                if (sampled == s_last) s_cnt <= s_cnt + 3'd1;
                                else begin s_last <= sampled; s_cnt <= 3'd1; end
                            end
                            else s_on <= 1'b0;

                            case (state)
                                ST_ID_A: begin
                                    r_id <= {r_id[27:0], sampled};
                                    if (bcnt == 7'd10) begin state <= ST_SRR; bcnt <= 7'd0; end
                                    else bcnt <= bcnt + 7'd1;
                                end
                                ST_SRR: begin r_srr <= sampled; state <= ST_IDE; end
                                ST_IDE: begin
                                    r_ide <= sampled;
                                    if (sampled) begin state <= ST_ID_B; bcnt <= 7'd0; end
                                    else begin
                                        r_rtr <= r_srr;
                                        r_id  <= {r_id[10:0], 18'd0};
                                        state <= ST_R0;
                                    end
                                end
                                ST_ID_B: begin
                                    r_id <= {r_id[27:0], sampled};
                                    if (bcnt == 7'd17) begin state <= ST_RTR; bcnt <= 7'd0; end
                                    else bcnt <= bcnt + 7'd1;
                                end
                                ST_RTR:  begin r_rtr <= sampled; state <= ST_R1; end
                                ST_R1:   state <= ST_R0;
                                ST_R0:   begin state <= ST_DLC; bcnt <= 7'd0; end
                                ST_DLC: begin
                                    r_dlc <= {r_dlc[2:0], sampled};
                                    if (bcnt == 7'd3) begin
                                        bcnt <= 7'd0;
                                        if (r_rtr || {r_dlc[2:0], sampled} == 4'd0) begin
                                            state     <= ST_CRC;
                                            align_cnt <= 7'd64;
                                        end
                                        else state <= ST_DATA;
                                    end
                                    else bcnt <= bcnt + 7'd1;
                                end
                                ST_DATA: begin
                                    r_data <= {r_data[62:0], sampled};
                                    if (bcnt == r_bits - 7'd1) begin
                                        state     <= ST_CRC;
                                        bcnt      <= 7'd0;
                                        align_cnt <= 7'd64 - r_bits;
                                    end
                                    else bcnt <= bcnt + 7'd1;
                                end
                                ST_CRC: begin
                                    if (bcnt == 7'd14) begin state <= ST_CRC_DEL; bcnt <= 7'd0; end
                                    else bcnt <= bcnt + 7'd1;
                                end
                                ST_CRC_DEL: state <= ST_ACK;
                                ST_ACK:     state <= ST_ACK_DEL;
                                ST_ACK_DEL: begin state <= ST_EOF; bcnt <= 7'd0; end
                                ST_EOF: begin

                                    if (bcnt == 7'd5 && (!transmitting || loopback)) begin
                                        ev_rx_frame <= 1'b1;
                                        if (rec > 8'd127)     rec <= 8'd120;
                                        else if (rec != 8'd0) rec <= rec - 8'd1;
                                    end
                                    if (bcnt == 7'd6) begin
                                        if (transmitting) begin
                                            ev_tx_ok <= 1'b1;
                                            if (tec != 9'd0) tec <= tec - 9'd1;
                                        end
                                        was_tx <= transmitting;
                                        transmitting <= 1'b0;
                                        state <= ST_INTER;
                                        bcnt  <= 7'd0;
                                    end
                                    else bcnt <= bcnt + 7'd1;
                                end
                                default: state <= ST_IDLE;
                            endcase
                        end
                    end
                endcase
            end
        end
    end

    wire [7:0] int_set = {ev_busoff, ev_err, rx_ovf, rx_push,
                          ev_tx_ok ? (4'b0001 << cur_mb) : 4'b0000};

    always @(posedge clk) begin
        if (reset) begin
            en <= 1'b0;  loopback <= 1'b0;  int_en <= 8'd0;  int_flag <= 8'd0;
            brp <= 6'd3;  sjw_m1 <= 2'd1;
            prop_m1 <= 3'd1;  ph1_m1 <= 3'd2;  ph2_m1 <= 3'd3;
            err_code <= E_NONE;
            for (i_rst = 0; i_rst < NUM_TX_MBOX; i_rst = i_rst + 1) begin
                mb_req[i_rst] <= 1'b0;  mb_lost[i_rst] <= 1'b0;  mb_err[i_rst] <= 1'b0;
            end
            for (i_wf = 0; i_wf < NUM_FILTERS; i_wf = i_wf + 1) f_en[i_wf] <= 1'b0;
        end
        else begin
            if (we && addr == 8'h00) begin en <= wdata[0]; loopback <= wdata[1]; end
            if (we && addr == 8'h02) int_en <= wdata;
            if (we && addr == 8'h04) begin brp <= wdata[5:0]; sjw_m1 <= wdata[7:6]; end
            if (we && addr == 8'h05) begin prop_m1 <= wdata[2:0]; ph1_m1 <= wdata[5:3]; end
            if (we && addr == 8'h06) ph2_m1 <= wdata[2:0];
            int_flag <= ((we && addr == 8'h03) ? (int_flag & ~wdata) : int_flag) | int_set;
            if (ev_err) err_code <= ev_code;

            for (i_wm = 0; i_wm < NUM_TX_MBOX; i_wm = i_wm + 1) begin

                if (we && addr[7:4] == i_wm + 1 &&
                    (addr[3:0] == 4'd0 || !(mb_req[i_wm] || (tx_busy && cur_mb == i_wm)))) begin
                    case (addr[3:0])
                        4'd0: begin
                            mb_req[i_wm]  <= wdata[0];
                            mb_lost[i_wm] <= 1'b0;
                            mb_err[i_wm]  <= 1'b0;
                        end
                        4'd1: mb_id[i_wm][28:24] <= wdata[4:0];
                        4'd2: mb_id[i_wm][23:16] <= wdata;
                        4'd3: mb_id[i_wm][15:8]  <= wdata;
                        4'd4: mb_id[i_wm][7:0]   <= wdata;
                        4'd5: begin
                            mb_ide[i_wm] <= wdata[7];
                            mb_rtr[i_wm] <= wdata[6];
                            mb_dlc[i_wm] <= wdata[3:0];
                        end
                        default: ;
                    endcase
                    for (i_wk = 0; i_wk < 8; i_wk = i_wk + 1)
                        if (addr[3:0] == i_wk + 6) mb_data[i_wm][63-8*i_wk -: 8] <= wdata;
                end
                if (we && addr == 8'h00 && wdata[2]) mb_req[i_wm] <= 1'b0;
                if (cur_mb == i_wm) begin
                    if (ev_tx_ok)    mb_req[i_wm]  <= 1'b0;
                    if (ev_arb_lost) mb_lost[i_wm] <= 1'b1;
                    if (ev_tx_err)   mb_err[i_wm]  <= 1'b1;
                end
            end

            for (i_wf = 0; i_wf < NUM_FILTERS; i_wf = i_wf + 1) begin
                if (we && addr[7] && addr[6:4] == i_wf) begin
                    case (addr[3:0])
                        4'd0: begin f_en[i_wf] <= wdata[0]; f_ide[i_wf] <= wdata[1]; end
                        4'd1: f_id[i_wf][28:24]   <= wdata[4:0];
                        4'd2: f_id[i_wf][23:16]   <= wdata;
                        4'd3: f_id[i_wf][15:8]    <= wdata;
                        4'd4: f_id[i_wf][7:0]     <= wdata;
                        4'd5: f_mask[i_wf][28:24] <= wdata[4:0];
                        4'd6: f_mask[i_wf][23:16] <= wdata;
                        4'd7: f_mask[i_wf][15:8]  <= wdata;
                        4'd8: f_mask[i_wf][7:0]   <= wdata;
                        default: ;
                    endcase
                end
            end
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            q_wr <= 3'd0;  q_rd <= 3'd0;  q_count <= 3'd0;  q_ovf <= 1'b0;
        end
        else begin
            if (rx_push) begin
                for (i_q = 0; i_q < RX_DEPTH; i_q = i_q + 1)
                    if (q_wr == i_q) begin
                        q_id[i_q]  <= r_id;   q_ide[i_q]  <= r_ide;   q_rtr[i_q] <= r_rtr;
                        q_dlc[i_q] <= r_dlc;  q_data[i_q] <= r_data;  q_hit[i_q] <= {hit, hit_idx};
                    end
                q_wr <= (q_wr == RX_DEPTH-1) ? 3'd0 : q_wr + 3'd1;
            end
            if (rx_pop) q_rd <= (q_rd == RX_DEPTH-1) ? 3'd0 : q_rd + 3'd1;
            q_count <= q_count + {2'b00, rx_push} - {2'b00, rx_pop};
            if (we && addr == 8'h50 && wdata[1]) q_ovf <= 1'b0;
            if (rx_ovf) q_ovf <= 1'b1;
        end
    end

    function [7:0] frame_byte;
        input [3:0]  off;
        input [28:0] id;
        input        ide, rtr;
        input [3:0]  dlc;
        input [63:0] data;
        begin
            case (off)
                4'd1:  frame_byte = {3'b000, id[28:24]};
                4'd2:  frame_byte = id[23:16];
                4'd3:  frame_byte = id[15:8];
                4'd4:  frame_byte = id[7:0];
                4'd5:  frame_byte = {ide, rtr, 2'b00, dlc};
                4'd6:  frame_byte = data[63:56];
                4'd7:  frame_byte = data[55:48];
                4'd8:  frame_byte = data[47:40];
                4'd9:  frame_byte = data[39:32];
                4'd10: frame_byte = data[31:24];
                4'd11: frame_byte = data[23:16];
                4'd12: frame_byte = data[15:8];
                4'd13: frame_byte = data[7:0];
                default: frame_byte = 8'h00;
            endcase
        end
    endfunction

    always @(*) begin
        rdata = 8'h00;
        if (addr[7:4] == 4'h0) begin
            case (addr[3:0])
                4'd0: rdata = {6'd0, loopback, en};
                4'd1: rdata = {state == ST_IDLE, q_count == RX_DEPTH, q_count != 3'd0,
                               state == ST_BUSOFF, passive, warning,
                               in_frame && !transmitting, transmitting};
                4'd2: rdata = int_en;
                4'd3: rdata = int_flag;
                4'd4: rdata = {sjw_m1, brp};
                4'd5: rdata = {2'b00, ph1_m1, prop_m1};
                4'd6: rdata = {5'd0, ph2_m1};
                4'd7: rdata = tec[8] ? 8'hFF : tec[7:0];
                4'd8: rdata = rec;
                4'd9: rdata = {5'd0, err_code};
                default: rdata = 8'h00;
            endcase
        end
        else if (addr[7:4] == 4'h5) begin
            if (addr[3:0] == 4'd0)       rdata = {3'b000, q_count, q_ovf, q_count != 3'd0};
            else if (addr[3:0] == 4'd14) rdata = {4'd0, q_hit[q_rd]};
            else rdata = frame_byte(addr[3:0], q_id[q_rd], q_ide[q_rd], q_rtr[q_rd],
                                    q_dlc[q_rd], q_data[q_rd]);
        end
        else if (addr[7]) begin
            for (i_rf = 0; i_rf < NUM_FILTERS; i_rf = i_rf + 1)
                if (addr[6:4] == i_rf) begin
                    case (addr[3:0])
                        4'd0: rdata = {6'd0, f_ide[i_rf], f_en[i_rf]};
                        4'd1: rdata = {3'b000, f_id[i_rf][28:24]};
                        4'd2: rdata = f_id[i_rf][23:16];
                        4'd3: rdata = f_id[i_rf][15:8];
                        4'd4: rdata = f_id[i_rf][7:0];
                        4'd5: rdata = {3'b000, f_mask[i_rf][28:24]};
                        4'd6: rdata = f_mask[i_rf][23:16];
                        4'd7: rdata = f_mask[i_rf][15:8];
                        4'd8: rdata = f_mask[i_rf][7:0];
                        default: rdata = 8'h00;
                    endcase
                end
        end
        else begin
            for (i_rm = 0; i_rm < NUM_TX_MBOX; i_rm = i_rm + 1)
                if (addr[7:4] == i_rm + 1)
                    rdata = (addr[3:0] == 4'd0)
                          ? {5'd0, mb_err[i_rm], mb_lost[i_rm], mb_req[i_rm]}
                          : frame_byte(addr[3:0], mb_id[i_rm], mb_ide[i_rm], mb_rtr[i_rm],
                                       mb_dlc[i_rm], mb_data[i_rm]);
        end
    end

    assign irq = |(int_flag & int_en);

endmodule

module CAN_BTL (
    input            clk,
    input            reset,
    input            enable,
    input      [5:0] brp,
    input      [1:0] sjw_m1,
    input      [2:0] prop_m1,
    input      [2:0] ph1_m1,
    input      [2:0] ph2_m1,
    input            rx_s,
    input            hard_sync_en,
    input            resync_en,
    output reg       tx_point,
    output reg       sample_point,
    output reg       sampled
);
    localparam S_SYNC = 2'd0, S_TSEG1 = 2'd1, S_TSEG2 = 2'd2;

    reg [5:0] presc;
    reg [1:0] seg;
    reg [4:0] seg_cnt;
    reg [2:0] ext;
    reg [3:0] ph2_len;
    reg       rx_prev, synced;

    wire       tq_tick = (presc == brp);
    wire [2:0] sjw     = {1'b0, sjw_m1} + 3'd1;
    wire [3:0] ph2     = {1'b0, ph2_m1} + 4'd1;
    wire [4:0] tseg1   = {2'b00, prop_m1} + {2'b00, ph1_m1} + 5'd2 + {2'b00, ext};
    wire       fall    = rx_prev && !rx_s && sampled;
    wire       rsync   = fall && resync_en && !synced;
    wire [4:0] lag     = seg_cnt + 5'd1;
    wire [4:0] lead    = {1'b0, ph2_len} - seg_cnt;
    wire       restart = (fall && hard_sync_en) ||
                         (rsync && seg == S_TSEG2 && lead <= {2'b00, sjw});

    always @(posedge clk) begin
        tx_point     <= 1'b0;
        sample_point <= 1'b0;
        rx_prev      <= rx_s;
        if (reset || !enable) begin
            presc <= 6'd0;  seg <= S_SYNC;  seg_cnt <= 5'd0;  ext <= 3'd0;
            ph2_len <= ph2;  synced <= 1'b0;  sampled <= 1'b1;  rx_prev <= 1'b1;
        end
        else if (restart) begin
            presc <= 6'd0;  seg <= S_SYNC;  seg_cnt <= 5'd0;  ext <= 3'd0;
            ph2_len <= ph2;  synced <= 1'b1;  tx_point <= 1'b1;
        end
        else begin
            if (rsync) begin
                synced <= 1'b1;
                if (seg == S_TSEG1)      ext     <= (lag > {2'b00, sjw}) ? sjw : lag[2:0];
                else if (seg == S_TSEG2) ph2_len <= ph2_len - {1'b0, sjw};
            end
            if (tq_tick) begin
                presc <= 6'd0;
                case (seg)
                    S_SYNC: begin seg <= S_TSEG1; seg_cnt <= 5'd0; end
                    S_TSEG1:
                        if (seg_cnt >= tseg1 - 5'd1) begin
                            seg <= S_TSEG2;  seg_cnt <= 5'd0;
                            sample_point <= 1'b1;
                            sampled      <= rx_s;
                        end
                        else seg_cnt <= seg_cnt + 5'd1;
                    default:
                        if (seg_cnt >= {1'b0, ph2_len} - 5'd1) begin
                            seg <= S_SYNC;  seg_cnt <= 5'd0;  ext <= 3'd0;
                            ph2_len <= ph2;  synced <= 1'b0;  tx_point <= 1'b1;
                        end
                        else seg_cnt <= seg_cnt + 5'd1;
                endcase
            end
            else presc <= presc + 6'd1;
        end
    end
endmodule

module CAN_CRC15 (
    input             clk,
    input             reset,
    input             clear,
    input             enable,
    input             bit_in,
    output reg [14:0] crc
);
    wire crc_nxt = bit_in ^ crc[14];

    always @(posedge clk) begin
        if (reset || clear)
            crc <= 15'd0;
        else if (enable)
            crc <= {crc[13:0], 1'b0} ^ (crc_nxt ? 15'h4599 : 15'h0000);
    end
endmodule
