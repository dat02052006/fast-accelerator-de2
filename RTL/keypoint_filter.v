// ============================================================================
// keypoint_filter.v  (Single-Bank 10-Column, Per-Grid Immediate Drain, 3-State FSM)
// Bo loc Keypoint theo luoi Grid 10x8 (o 64x64 tren anh goc 640x480) + Top-4.
//
// Kien truc toi uu (khong dung Ping-Pong 2 bank, giam 50% thanh ghi & MUX):
//   - Chi su dung 1 Bank gom 10 cot o (c0..c9), moi cot 4 slot (28-bit/slot)
//     va 1 thanh ghi metadata 17-bit:
//     {dirty[16], band_y[15:13], cnt[12:10], min_idx[9:8], min_sc[7:0]}.
//   - Ngay khi xac nhan cot o k (0..9) da quet xong (qua ranh gioi cot o dong
//     cuoi bang, hoac sang bang moi, hoac nms_done), FSM (S_IDLE -> S_DRAIN ->
//     S_DONE) xa ngay 4 slot cua cot k xuong fast_result_packer.
// ============================================================================
module keypoint_filter #(
    parameter integer BORDER     = 3,       // Bu le bien FAST-9 (+3)
    parameter integer K_PER_CELL = 4,       // Co dinh Top-4 moi o 64x64 (max 320 KP)
    parameter integer OUT_CNT_W  = 12       // Do rong bo dem tong so keypoint xuat ra
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 start,          // Xung 1 chu ky bat dau frame moi

    // ---- Dau vao tu nms_stream_v3 ----
    input  wire                 in_valid,       // nms_kp_valid
    input  wire [9:0]           in_x,           // nms_kp_x (0..633)
    input  wire [8:0]           in_y,           // nms_kp_y (0..473)
    input  wire [7:0]           in_score,       // nms_kp_score (1..255)
    input  wire                 nms_done,       // nms_flush_done (xung 1 chu ky cuoi frame)

    // ---- Bat tay & ngo ra toi fast_result_packer ----
    input  wire                 packer_busy,    // = 1 khi packer dang dich word
    input  wire                 fifo_full,      // = 1 khi scfifo day
    output reg                  out_valid,      // Xung 1 chu ky gui 1 keypoint
    output reg  [9:0]           out_x,          // Toa do X tuyet doi (3..636)
    output reg  [8:0]           out_y,          // Toa do Y tuyet doi (3..476)
    output reg  [7:0]           out_score,

    // ---- Trang thai & thong ke ----
    output wire                 filter_done,    // Muc 1 khi FSM dat trang thai S_DONE
    output reg  [OUT_CNT_W-1:0] out_count       // Tong so keypoint da gui xuong packer
);

    // ========================================================================
    // Tinh toa do tuyet doi & chi so o 64x64
    // ========================================================================
    wire [9:0]  abs_x   = in_x + BORDER[9:0];
    wire [8:0]  abs_y   = in_y + BORDER[8:0];
    wire [3:0]  cell_x  = abs_x[9:6];   // 0..9
    wire [2:0]  cell_y  = abs_y[8:6];   // 0..7
    wire [27:0] new_pkt = {1'b1, in_score, abs_x, abs_y};

    // ========================================================================
    // 1 Bank duy nhat:
    //   - 40 slot du lieu 28-bit: {valid[27], score[26:19], x[18:9], y[8:0]}
    //   - 10 thanh ghi metadata 17-bit:
    //     {dirty[16], band_y[15:13], cnt[12:10], min_idx[9:8], min_sc[7:0]}
    // ========================================================================
    reg [27:0] c0_s0, c0_s1, c0_s2, c0_s3;
    reg [27:0] c1_s0, c1_s1, c1_s2, c1_s3;
    reg [27:0] c2_s0, c2_s1, c2_s2, c2_s3;
    reg [27:0] c3_s0, c3_s1, c3_s2, c3_s3;
    reg [27:0] c4_s0, c4_s1, c4_s2, c4_s3;
    reg [27:0] c5_s0, c5_s1, c5_s2, c5_s3;
    reg [27:0] c6_s0, c6_s1, c6_s2, c6_s3;
    reg [27:0] c7_s0, c7_s1, c7_s2, c7_s3;
    reg [27:0] c8_s0, c8_s1, c8_s2, c8_s3;
    reg [27:0] c9_s0, c9_s1, c9_s2, c9_s3;

    reg [16:0] m0, m1, m2, m3, m4, m5, m6, m7, m8, m9;

    // Theo doi vi tri quet moi nhat de xac nhan o nao da hoan thanh
    reg       scan_init;
    reg [3:0] latest_cell_x;
    reg [2:0] latest_cell_y;
    reg       latest_last_row;  // = 1 khi abs_y[5:0] == 6'd63
    reg       nms_done_latched;

    // Bo dem giu 1 keypoint neu roi vao cot dang cho xa du lieu bang cu
    reg        pend_vld;
    reg [3:0]  pend_col;
    reg [2:0]  pend_band;
    reg [27:0] pend_pkt;

    // Con tro xa du lieu
    reg [3:0] drain_col;        // 0..9
    reg [1:0] drain_slot;       // 0..3

    // ========================================================================
    // Doc metadata (17-bit) va 4 slot cua cot cell_x
    // ========================================================================
    reg [16:0] rd_meta;
    reg [27:0] rd_s0, rd_s1, rd_s2, rd_s3;

    always @(*) begin
        rd_meta = 17'd0;
        rd_s0   = 28'd0;
        rd_s1   = 28'd0;
        rd_s2   = 28'd0;
        rd_s3   = 28'd0;
        case (cell_x)
            4'd0: begin rd_meta = m0; rd_s0 = c0_s0; rd_s1 = c0_s1; rd_s2 = c0_s2; rd_s3 = c0_s3; end
            4'd1: begin rd_meta = m1; rd_s0 = c1_s0; rd_s1 = c1_s1; rd_s2 = c1_s2; rd_s3 = c1_s3; end
            4'd2: begin rd_meta = m2; rd_s0 = c2_s0; rd_s1 = c2_s1; rd_s2 = c2_s2; rd_s3 = c2_s3; end
            4'd3: begin rd_meta = m3; rd_s0 = c3_s0; rd_s1 = c3_s1; rd_s2 = c3_s2; rd_s3 = c3_s3; end
            4'd4: begin rd_meta = m4; rd_s0 = c4_s0; rd_s1 = c4_s1; rd_s2 = c4_s2; rd_s3 = c4_s3; end
            4'd5: begin rd_meta = m5; rd_s0 = c5_s0; rd_s1 = c5_s1; rd_s2 = c5_s2; rd_s3 = c5_s3; end
            4'd6: begin rd_meta = m6; rd_s0 = c6_s0; rd_s1 = c6_s1; rd_s2 = c6_s2; rd_s3 = c6_s3; end
            4'd7: begin rd_meta = m7; rd_s0 = c7_s0; rd_s1 = c7_s1; rd_s2 = c7_s2; rd_s3 = c7_s3; end
            4'd8: begin rd_meta = m8; rd_s0 = c8_s0; rd_s1 = c8_s1; rd_s2 = c8_s2; rd_s3 = c8_s3; end
            4'd9: begin rd_meta = m9; rd_s0 = c9_s0; rd_s1 = c9_s1; rd_s2 = c9_s2; rd_s3 = c9_s3; end
            default: ;
        endcase
    end

    wire       cur_dirty   = rd_meta[16];
    wire [2:0] cur_band_y  = rd_meta[15:13];
    wire [2:0] cur_cnt     = rd_meta[12:10];
    wire [1:0] cur_min_idx = rd_meta[9:8];
    wire [7:0] cur_min_sc  = rd_meta[7:0];

    // Neu cot cell_x dang chua du lieu cua bang cu chua kip xa -> day vao pend_pkt
    wire col_busy_old = cur_dirty && (cur_band_y != cell_y);
    wire defer_to_pend = in_valid && col_busy_old;

    // Logic chen Top-4 khong dich mang
    wire       cell_full = cur_dirty && (cur_cnt == 3'd4);
    wire       beat_min  = (in_score > cur_min_sc);
    wire       do_write  = in_valid && !col_busy_old && (!cell_full || beat_min);
    wire [1:0] wr_slot   = cell_full ? cur_min_idx : cur_cnt[1:0];

    // Tinh min moi khi thay the slot yeu nhat
    wire [7:0] s0_sc = (cur_min_idx == 2'd0) ? in_score : rd_s0[26:19];
    wire [7:0] s1_sc = (cur_min_idx == 2'd1) ? in_score : rd_s1[26:19];
    wire [7:0] s2_sc = (cur_min_idx == 2'd2) ? in_score : rd_s2[26:19];
    wire [7:0] s3_sc = (cur_min_idx == 2'd3) ? in_score : rd_s3[26:19];

    wire       m01_lt = (s1_sc < s0_sc);
    wire [7:0] m01_sc = m01_lt ? s1_sc : s0_sc;
    wire [1:0] m01_id = m01_lt ? 2'd1  : 2'd0;

    wire       m23_lt = (s3_sc < s2_sc);
    wire [7:0] m23_sc = m23_lt ? s3_sc : s2_sc;
    wire [1:0] m23_id = m23_lt ? 2'd3  : 2'd2;

    wire       m_lt           = (m23_sc < m01_sc);
    wire [7:0] recalc_min_sc  = m_lt ? m23_sc : m01_sc;
    wire [1:0] recalc_min_idx = m_lt ? m23_id : m01_id;

    wire       new_is_smaller = (!cur_dirty) || (in_score < cur_min_sc);
    wire [2:0] nxt_cnt        = cell_full ? 3'd4 : (cur_cnt + 3'd1);
    wire [1:0] nxt_min_idx    = cell_full ? recalc_min_idx : (new_is_smaller ? cur_cnt[1:0] : cur_min_idx);
    wire [7:0] nxt_min_sc     = cell_full ? recalc_min_sc  : (new_is_smaller ? in_score     : cur_min_sc);
    wire [16:0] nxt_meta      = {1'b1, cell_y, nxt_cnt, nxt_min_idx, nxt_min_sc};

    // ========================================================================
    // Kiem tra cot o nao da hoan thanh de xa ngay (col_ready[9:0])
    // ========================================================================
    wire [2:0] eff_lat_y    = in_valid ? cell_y : latest_cell_y;
    wire [3:0] eff_lat_x    = in_valid ? cell_x : latest_cell_x;
    wire       eff_last_row = in_valid ? (abs_y[5:0] == 6'd63) : latest_last_row;
    wire       eff_scan_vld = in_valid || scan_init;
    wire       eff_nms_done = nms_done || nms_done_latched;

    wire col_rdy0 = m0[16] && (eff_nms_done || (eff_scan_vld && ((eff_lat_y > m0[15:13]) || ((eff_lat_y == m0[15:13]) && eff_last_row && (eff_lat_x > 4'd0)))));
    wire col_rdy1 = m1[16] && (eff_nms_done || (eff_scan_vld && ((eff_lat_y > m1[15:13]) || ((eff_lat_y == m1[15:13]) && eff_last_row && (eff_lat_x > 4'd1)))));
    wire col_rdy2 = m2[16] && (eff_nms_done || (eff_scan_vld && ((eff_lat_y > m2[15:13]) || ((eff_lat_y == m2[15:13]) && eff_last_row && (eff_lat_x > 4'd2)))));
    wire col_rdy3 = m3[16] && (eff_nms_done || (eff_scan_vld && ((eff_lat_y > m3[15:13]) || ((eff_lat_y == m3[15:13]) && eff_last_row && (eff_lat_x > 4'd3)))));
    wire col_rdy4 = m4[16] && (eff_nms_done || (eff_scan_vld && ((eff_lat_y > m4[15:13]) || ((eff_lat_y == m4[15:13]) && eff_last_row && (eff_lat_x > 4'd4)))));
    wire col_rdy5 = m5[16] && (eff_nms_done || (eff_scan_vld && ((eff_lat_y > m5[15:13]) || ((eff_lat_y == m5[15:13]) && eff_last_row && (eff_lat_x > 4'd5)))));
    wire col_rdy6 = m6[16] && (eff_nms_done || (eff_scan_vld && ((eff_lat_y > m6[15:13]) || ((eff_lat_y == m6[15:13]) && eff_last_row && (eff_lat_x > 4'd6)))));
    wire col_rdy7 = m7[16] && (eff_nms_done || (eff_scan_vld && ((eff_lat_y > m7[15:13]) || ((eff_lat_y == m7[15:13]) && eff_last_row && (eff_lat_x > 4'd7)))));
    wire col_rdy8 = m8[16] && (eff_nms_done || (eff_scan_vld && ((eff_lat_y > m8[15:13]) || ((eff_lat_y == m8[15:13]) && eff_last_row && (eff_lat_x > 4'd8)))));
    wire col_rdy9 = m9[16] && (eff_nms_done || (eff_scan_vld && (eff_lat_y > m9[15:13])));

    wire any_col_rdy = col_rdy0 | col_rdy1 | col_rdy2 | col_rdy3 | col_rdy4 |
                       col_rdy5 | col_rdy6 | col_rdy7 | col_rdy8 | col_rdy9;

    // Bo ma hoa uu tien chon cot o da hoan thanh (uu tien cot co keypoint dang doi pend_col)
    reg [3:0] sel_col;
    always @(*) begin
        if (pend_vld)
            sel_col = pend_col;
        else if (col_rdy0) sel_col = 4'd0;
        else if (col_rdy1) sel_col = 4'd1;
        else if (col_rdy2) sel_col = 4'd2;
        else if (col_rdy3) sel_col = 4'd3;
        else if (col_rdy4) sel_col = 4'd4;
        else if (col_rdy5) sel_col = 4'd5;
        else if (col_rdy6) sel_col = 4'd6;
        else if (col_rdy7) sel_col = 4'd7;
        else if (col_rdy8) sel_col = 4'd8;
        else               sel_col = 4'd9;
    end

    // ========================================================================
    // Bo chon 4 slot cua cot dang xa (drain_col, drain_slot)
    // ========================================================================
    reg [27:0] dr_s0, dr_s1, dr_s2, dr_s3;

    always @(*) begin
        dr_s0 = 28'd0;
        dr_s1 = 28'd0;
        dr_s2 = 28'd0;
        dr_s3 = 28'd0;
        case (drain_col)
            4'd0: begin dr_s0 = c0_s0; dr_s1 = c0_s1; dr_s2 = c0_s2; dr_s3 = c0_s3; end
            4'd1: begin dr_s0 = c1_s0; dr_s1 = c1_s1; dr_s2 = c1_s2; dr_s3 = c1_s3; end
            4'd2: begin dr_s0 = c2_s0; dr_s1 = c2_s1; dr_s2 = c2_s2; dr_s3 = c2_s3; end
            4'd3: begin dr_s0 = c3_s0; dr_s1 = c3_s1; dr_s2 = c3_s2; dr_s3 = c3_s3; end
            4'd4: begin dr_s0 = c4_s0; dr_s1 = c4_s1; dr_s2 = c4_s2; dr_s3 = c4_s3; end
            4'd5: begin dr_s0 = c5_s0; dr_s1 = c5_s1; dr_s2 = c5_s2; dr_s3 = c5_s3; end
            4'd6: begin dr_s0 = c6_s0; dr_s1 = c6_s1; dr_s2 = c6_s2; dr_s3 = c6_s3; end
            4'd7: begin dr_s0 = c7_s0; dr_s1 = c7_s1; dr_s2 = c7_s2; dr_s3 = c7_s3; end
            4'd8: begin dr_s0 = c8_s0; dr_s1 = c8_s1; dr_s2 = c8_s2; dr_s3 = c8_s3; end
            4'd9: begin dr_s0 = c9_s0; dr_s1 = c9_s1; dr_s2 = c9_s2; dr_s3 = c9_s3; end
            default: ;
        endcase
    end

    reg [27:0] cur_slot_pkt;
    always @(*) begin
        case (drain_slot)
            2'd0: cur_slot_pkt = dr_s0;
            2'd1: cur_slot_pkt = dr_s1;
            2'd2: cur_slot_pkt = dr_s2;
            2'd3: cur_slot_pkt = dr_s3;
        endcase
    end

    // ========================================================================
    // FSM 3 trang thai cho Drain Engine (S_IDLE, S_DRAIN, S_DONE)
    // ========================================================================
    localparam [1:0] S_IDLE  = 2'd0,
                     S_DRAIN = 2'd1,
                     S_DONE  = 2'd2;

    reg [1:0] state, next_state;

    wire cur_slot_vld = cur_slot_pkt[27];
    wire can_send     = !packer_busy && !fifo_full && !out_valid;
    wire slot_advance = (state == S_DRAIN) && (!cur_slot_vld || can_send);
    wire col_finish   = slot_advance && (drain_slot == 2'd3);
    wire clr_en       = (state == S_DRAIN) && cur_slot_vld && can_send;
    wire flush_pend   = col_finish && pend_vld && (pend_col == drain_col);

    // 1) Logic to hop chuyen trang thai (Next-State Logic)
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (any_col_rdy)
                    next_state = S_DRAIN;
                else if (eff_nms_done && !pend_vld)
                    next_state = S_DONE;
            end

            S_DRAIN: begin
                if (col_finish)
                    next_state = S_IDLE;
            end

            S_DONE: begin
                next_state = S_DONE;
            end

            default: next_state = S_IDLE;
        endcase
    end

    // 2) Thanh ghi trang thai (State Register) - tach rieng rst_n va start
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= S_IDLE;
        else if (start)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    // 3) Ngo ra trang thai
    assign filter_done = (state == S_DONE);

    // ========================================================================
    // Cap nhat Datapath (Ghi don 1 slot + Metadata & Bo dem FSM Drain)
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scan_init        <= 1'b0;
            latest_cell_x    <= 4'd0;
            latest_cell_y    <= 3'd0;
            latest_last_row  <= 1'b0;
            nms_done_latched <= 1'b0;
            pend_vld         <= 1'b0;
            pend_col         <= 4'd0;
            pend_band        <= 3'd0;
            pend_pkt         <= 28'd0;
            drain_col        <= 4'd0;
            drain_slot       <= 2'd0;
            out_valid        <= 1'b0;
            out_x            <= 10'd0;
            out_y            <= 9'd0;
            out_score        <= 8'd0;
            out_count        <= {OUT_CNT_W{1'b0}};

            m0 <= 17'd0; m1 <= 17'd0; m2 <= 17'd0; m3 <= 17'd0; m4 <= 17'd0;
            m5 <= 17'd0; m6 <= 17'd0; m7 <= 17'd0; m8 <= 17'd0; m9 <= 17'd0;

            c0_s0 <= 28'd0; c0_s1 <= 28'd0; c0_s2 <= 28'd0; c0_s3 <= 28'd0;
            c1_s0 <= 28'd0; c1_s1 <= 28'd0; c1_s2 <= 28'd0; c1_s3 <= 28'd0;
            c2_s0 <= 28'd0; c2_s1 <= 28'd0; c2_s2 <= 28'd0; c2_s3 <= 28'd0;
            c3_s0 <= 28'd0; c3_s1 <= 28'd0; c3_s2 <= 28'd0; c3_s3 <= 28'd0;
            c4_s0 <= 28'd0; c4_s1 <= 28'd0; c4_s2 <= 28'd0; c4_s3 <= 28'd0;
            c5_s0 <= 28'd0; c5_s1 <= 28'd0; c5_s2 <= 28'd0; c5_s3 <= 28'd0;
            c6_s0 <= 28'd0; c6_s1 <= 28'd0; c6_s2 <= 28'd0; c6_s3 <= 28'd0;
            c7_s0 <= 28'd0; c7_s1 <= 28'd0; c7_s2 <= 28'd0; c7_s3 <= 28'd0;
            c8_s0 <= 28'd0; c8_s1 <= 28'd0; c8_s2 <= 28'd0; c8_s3 <= 28'd0;
            c9_s0 <= 28'd0; c9_s1 <= 28'd0; c9_s2 <= 28'd0; c9_s3 <= 28'd0;
        end else if (start) begin
            scan_init        <= 1'b0;
            latest_cell_x    <= 4'd0;
            latest_cell_y    <= 3'd0;
            latest_last_row  <= 1'b0;
            nms_done_latched <= 1'b0;
            pend_vld         <= 1'b0;
            pend_col         <= 4'd0;
            pend_band        <= 3'd0;
            pend_pkt         <= 28'd0;
            drain_col        <= 4'd0;
            drain_slot       <= 2'd0;
            out_valid        <= 1'b0;
            out_x            <= 10'd0;
            out_y            <= 9'd0;
            out_score        <= 8'd0;
            out_count        <= {OUT_CNT_W{1'b0}};

            m0 <= 17'd0; m1 <= 17'd0; m2 <= 17'd0; m3 <= 17'd0; m4 <= 17'd0;
            m5 <= 17'd0; m6 <= 17'd0; m7 <= 17'd0; m8 <= 17'd0; m9 <= 17'd0;

            c0_s0 <= 28'd0; c0_s1 <= 28'd0; c0_s2 <= 28'd0; c0_s3 <= 28'd0;
            c1_s0 <= 28'd0; c1_s1 <= 28'd0; c1_s2 <= 28'd0; c1_s3 <= 28'd0;
            c2_s0 <= 28'd0; c2_s1 <= 28'd0; c2_s2 <= 28'd0; c2_s3 <= 28'd0;
            c3_s0 <= 28'd0; c3_s1 <= 28'd0; c3_s2 <= 28'd0; c3_s3 <= 28'd0;
            c4_s0 <= 28'd0; c4_s1 <= 28'd0; c4_s2 <= 28'd0; c4_s3 <= 28'd0;
            c5_s0 <= 28'd0; c5_s1 <= 28'd0; c5_s2 <= 28'd0; c5_s3 <= 28'd0;
            c6_s0 <= 28'd0; c6_s1 <= 28'd0; c6_s2 <= 28'd0; c6_s3 <= 28'd0;
            c7_s0 <= 28'd0; c7_s1 <= 28'd0; c7_s2 <= 28'd0; c7_s3 <= 28'd0;
            c8_s0 <= 28'd0; c8_s1 <= 28'd0; c8_s2 <= 28'd0; c8_s3 <= 28'd0;
            c9_s0 <= 28'd0; c9_s1 <= 28'd0; c9_s2 <= 28'd0; c9_s3 <= 28'd0;
        end else begin
            out_valid <= 1'b0;

            // ----------------------------------------------------------------
            // A. Cap nhat vi tri quet & Bo dem cho 1 phan tu
            // ----------------------------------------------------------------
            if (in_valid) begin
                scan_init       <= 1'b1;
                latest_cell_x   <= cell_x;
                latest_cell_y   <= cell_y;
                latest_last_row <= (abs_y[5:0] == 6'd63);
            end

            if (nms_done)
                nms_done_latched <= 1'b1;

            if (defer_to_pend) begin
                pend_vld  <= 1'b1;
                pend_col  <= cell_x;
                pend_band <= cell_y;
                pend_pkt  <= new_pkt;
            end else if (flush_pend) begin
                pend_vld  <= 1'b0;
            end

            // ----------------------------------------------------------------
            // B. Luong ghi: Cap nhat Metadata & Ghi don 1 slot
            // ----------------------------------------------------------------
            if (do_write) begin
                case (cell_x)
                    4'd0: m0 <= nxt_meta;
                    4'd1: m1 <= nxt_meta;
                    4'd2: m2 <= nxt_meta;
                    4'd3: m3 <= nxt_meta;
                    4'd4: m4 <= nxt_meta;
                    4'd5: m5 <= nxt_meta;
                    4'd6: m6 <= nxt_meta;
                    4'd7: m7 <= nxt_meta;
                    4'd8: m8 <= nxt_meta;
                    4'd9: m9 <= nxt_meta;
                    default: ;
                endcase

                case ({cell_x, wr_slot})
                    6'd0:  c0_s0 <= new_pkt; 6'd1:  c0_s1 <= new_pkt; 6'd2:  c0_s2 <= new_pkt; 6'd3:  c0_s3 <= new_pkt;
                    6'd4:  c1_s0 <= new_pkt; 6'd5:  c1_s1 <= new_pkt; 6'd6:  c1_s2 <= new_pkt; 6'd7:  c1_s3 <= new_pkt;
                    6'd8:  c2_s0 <= new_pkt; 6'd9:  c2_s1 <= new_pkt; 6'd10: c2_s2 <= new_pkt; 6'd11: c2_s3 <= new_pkt;
                    6'd12: c3_s0 <= new_pkt; 6'd13: c3_s1 <= new_pkt; 6'd14: c3_s2 <= new_pkt; 6'd15: c3_s3 <= new_pkt;
                    6'd16: c4_s0 <= new_pkt; 6'd17: c4_s1 <= new_pkt; 6'd18: c4_s2 <= new_pkt; 6'd19: c4_s3 <= new_pkt;
                    6'd20: c5_s0 <= new_pkt; 6'd21: c5_s1 <= new_pkt; 6'd22: c5_s2 <= new_pkt; 6'd23: c5_s3 <= new_pkt;
                    6'd24: c6_s0 <= new_pkt; 6'd25: c6_s1 <= new_pkt; 6'd26: c6_s2 <= new_pkt; 6'd27: c6_s3 <= new_pkt;
                    6'd28: c7_s0 <= new_pkt; 6'd29: c7_s1 <= new_pkt; 6'd30: c7_s2 <= new_pkt; 6'd31: c7_s3 <= new_pkt;
                    6'd32: c8_s0 <= new_pkt; 6'd33: c8_s1 <= new_pkt; 6'd34: c8_s2 <= new_pkt; 6'd35: c8_s3 <= new_pkt;
                    6'd36: c9_s0 <= new_pkt; 6'd37: c9_s1 <= new_pkt; 6'd38: c9_s2 <= new_pkt; 6'd39: c9_s3 <= new_pkt;
                    default: ;
                endcase
            end

            // ----------------------------------------------------------------
            // C. Datapath cho FSM Drain Engine (Xa tung cot o da hoan thanh)
            // ----------------------------------------------------------------
            case (state)
                S_IDLE: begin
                    drain_slot <= 2'd0;
                    if (any_col_rdy)
                        drain_col <= sel_col;
                end

                S_DRAIN: begin
                    if (cur_slot_vld && can_send) begin
                        out_valid <= 1'b1;
                        out_score <= cur_slot_pkt[26:19];
                        out_x     <= cur_slot_pkt[18:9];
                        out_y     <= cur_slot_pkt[8:0];
                        out_count <= out_count + {{(OUT_CNT_W-1){1'b0}}, 1'b1};
                    end

                    if (slot_advance) begin
                        if (drain_slot == 2'd3) begin
                            drain_slot <= 2'd0;
                            // Xoa metadata cua cot vua xa xong (hoac nap ngay pend_pkt neu trung cot)
                            case (drain_col)
                                4'd0: m0 <= flush_pend ? {1'b1, pend_band, 3'd1, 2'd0, pend_pkt[26:19]} : 17'd0;
                                4'd1: m1 <= flush_pend ? {1'b1, pend_band, 3'd1, 2'd0, pend_pkt[26:19]} : 17'd0;
                                4'd2: m2 <= flush_pend ? {1'b1, pend_band, 3'd1, 2'd0, pend_pkt[26:19]} : 17'd0;
                                4'd3: m3 <= flush_pend ? {1'b1, pend_band, 3'd1, 2'd0, pend_pkt[26:19]} : 17'd0;
                                4'd4: m4 <= flush_pend ? {1'b1, pend_band, 3'd1, 2'd0, pend_pkt[26:19]} : 17'd0;
                                4'd5: m5 <= flush_pend ? {1'b1, pend_band, 3'd1, 2'd0, pend_pkt[26:19]} : 17'd0;
                                4'd6: m6 <= flush_pend ? {1'b1, pend_band, 3'd1, 2'd0, pend_pkt[26:19]} : 17'd0;
                                4'd7: m7 <= flush_pend ? {1'b1, pend_band, 3'd1, 2'd0, pend_pkt[26:19]} : 17'd0;
                                4'd8: m8 <= flush_pend ? {1'b1, pend_band, 3'd1, 2'd0, pend_pkt[26:19]} : 17'd0;
                                4'd9: m9 <= flush_pend ? {1'b1, pend_band, 3'd1, 2'd0, pend_pkt[26:19]} : 17'd0;
                                default: ;
                            endcase
                        end else begin
                            drain_slot <= drain_slot + 2'd1;
                        end
                    end
                end

                S_DONE: begin
                    drain_slot <= 2'd0;
                end

                default: ;
            endcase

            // Xoa trang slot sau khi da day ra ngoai (hoac nap pend_pkt vao slot 0)
            if (flush_pend) begin
                case (drain_col)
                    4'd0: c0_s0 <= pend_pkt;
                    4'd1: c1_s0 <= pend_pkt;
                    4'd2: c2_s0 <= pend_pkt;
                    4'd3: c3_s0 <= pend_pkt;
                    4'd4: c4_s0 <= pend_pkt;
                    4'd5: c5_s0 <= pend_pkt;
                    4'd6: c6_s0 <= pend_pkt;
                    4'd7: c7_s0 <= pend_pkt;
                    4'd8: c8_s0 <= pend_pkt;
                    4'd9: c9_s0 <= pend_pkt;
                    default: ;
                endcase
            end

            if (clr_en) begin
                case ({drain_col, drain_slot})
                    6'd0:  c0_s0 <= 28'd0; 6'd1:  c0_s1 <= 28'd0; 6'd2:  c0_s2 <= 28'd0; 6'd3:  c0_s3 <= 28'd0;
                    6'd4:  c1_s0 <= 28'd0; 6'd5:  c1_s1 <= 28'd0; 6'd6:  c1_s2 <= 28'd0; 6'd7:  c1_s3 <= 28'd0;
                    6'd8:  c2_s0 <= 28'd0; 6'd9:  c2_s1 <= 28'd0; 6'd10: c2_s2 <= 28'd0; 6'd11: c2_s3 <= 28'd0;
                    6'd12: c3_s0 <= 28'd0; 6'd13: c3_s1 <= 28'd0; 6'd14: c3_s2 <= 28'd0; 6'd15: c3_s3 <= 28'd0;
                    6'd16: c4_s0 <= 28'd0; 6'd17: c4_s1 <= 28'd0; 6'd18: c4_s2 <= 28'd0; 6'd19: c4_s3 <= 28'd0;
                    6'd20: c5_s0 <= 28'd0; 6'd21: c5_s1 <= 28'd0; 6'd22: c5_s2 <= 28'd0; 6'd23: c5_s3 <= 28'd0;
                    6'd24: c6_s0 <= 28'd0; 6'd25: c6_s1 <= 28'd0; 6'd26: c6_s2 <= 28'd0; 6'd27: c6_s3 <= 28'd0;
                    6'd28: c7_s0 <= 28'd0; 6'd29: c7_s1 <= 28'd0; 6'd30: c7_s2 <= 28'd0; 6'd31: c7_s3 <= 28'd0;
                    6'd32: c8_s0 <= 28'd0; 6'd33: c8_s1 <= 28'd0; 6'd34: c8_s2 <= 28'd0; 6'd35: c8_s3 <= 28'd0;
                    6'd36: c9_s0 <= 28'd0; 6'd37: c9_s1 <= 28'd0; 6'd38: c9_s2 <= 28'd0; 6'd39: c9_s3 <= 28'd0;
                    default: ;
                endcase
            end
        end
    end

endmodule