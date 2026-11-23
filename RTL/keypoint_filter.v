// ============================================================================
// keypoint_filter.v  (2-Stage Pipelined Input for >115 MHz Fmax on Cyclone II)
// Bo loc Keypoint theo luoi Grid 10x8 (o 64x64 tren anh goc 640x480) + Top-K.
//
// Pipeline 2 tang:
//   - Stage 1 (Input Register): Cong bu +BORDER (+3), tinh cell_x = abs_x[9:6],
//     cell_y = abs_y[8:6] va chot vao thanh ghi (cat doi duong gang tu NMS).
//   - Stage 2 (Ping-Pong Top-K & Drain): Chen sap xep Top-K vao o cell_x_r
//     va xa du lieu co bat tay (packer_busy | fifo_full) -> 0% mat diem.
// ============================================================================
module keypoint_filter #(
    parameter integer BORDER     = 3,       // Bu le bien FAST-9 (+3)
    parameter integer K_PER_CELL = 4,       // Top-K moi o 64x64 (4 -> max 320 KP)
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
    output wire                 filter_done,    // Muc 1 khi da nhan nms_done VA xa het 2 bank
    output reg  [OUT_CNT_W-1:0] out_count       // Tong so keypoint da gui xuong packer
);

    localparam integer NUM_COLS = 10; // 640 / 64 = 10 cot o (0..9)

    // ========================================================================
    // Stage 1: Cong bu bien (+3), giai ma chi so o & chot thanh ghi dau vao
    // ========================================================================
    wire [9:0] abs_x_w  = in_x + BORDER[9:0];
    wire [8:0] abs_y_w  = in_y + BORDER[8:0];

    reg        vld_s1;
    reg        nms_done_s1;
    reg  [9:0] abs_x_s1;
    reg  [8:0] abs_y_s1;
    reg  [7:0] score_s1;
    reg  [3:0] cell_x_s1;   // 0..9
    reg  [2:0] cell_y_s1;   // 0..7

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vld_s1      <= 1'b0;
            nms_done_s1 <= 1'b0;
            abs_x_s1    <= 10'd0;
            abs_y_s1    <= 9'd0;
            score_s1    <= 8'd0;
            cell_x_s1   <= 4'd0;
            cell_y_s1   <= 3'd0;
        end else if (start) begin
            vld_s1      <= 1'b0;
            nms_done_s1 <= 1'b0;
            abs_x_s1    <= 10'd0;
            abs_y_s1    <= 9'd0;
            score_s1    <= 8'd0;
            cell_x_s1   <= 4'd0;
            cell_y_s1   <= 3'd0;
        end else begin
            vld_s1      <= in_valid;
            nms_done_s1 <= nms_done;
            abs_x_s1    <= abs_x_w;
            abs_y_s1    <= abs_y_w;
            score_s1    <= in_score;
            cell_x_s1   <= abs_x_w[9:6];
            cell_y_s1   <= abs_y_w[8:6];
        end
    end

    // ========================================================================
    // Stage 2: Bo nho Ping-Pong 2 Bank & Logic chen sap xep Top-K
    // ========================================================================
    reg       slot_vld [0:1][0:NUM_COLS-1][0:K_PER_CELL-1];
    reg [7:0] slot_sc  [0:1][0:NUM_COLS-1][0:K_PER_CELL-1];
    reg [9:0] slot_x   [0:1][0:NUM_COLS-1][0:K_PER_CELL-1];
    reg [8:0] slot_y   [0:1][0:NUM_COLS-1][0:K_PER_CELL-1];

    reg       act_bank;         // Bank dang nhan keypoint (0 hoac 1)
    reg [2:0] cur_band_y;       // Hang o hien tai (0..7)
    reg       band_init;        // Da co keypoint dau tien trong frame chua
    reg       nms_done_latched; // Da nhan nms_done_s1

    reg       drain_req [0:1];
    reg       draining;
    reg       drain_bank;
    reg [3:0] drain_col;        // 0..9
    reg [2:0] drain_slot;       // 0..K_PER_CELL-1

    wire band_changed = vld_s1 && band_init && (cell_y_s1 != cur_band_y);

    wire [K_PER_CELL-1:0] beat;
    genvar g;
    generate
        for (g = 0; g < K_PER_CELL; g = g + 1) begin : gen_cmp
            assign beat[g] = (!slot_vld[act_bank][cell_x_s1][g]) ||
                             (score_s1 > slot_sc[act_bank][cell_x_s1][g]);
        end
    endgenerate

    wire can_send     = !packer_busy && !fifo_full && !out_valid;
    wire cur_slot_vld = slot_vld[drain_bank][drain_col][drain_slot];

    integer b_i, c_i, s_i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            act_bank         <= 1'b0;
            cur_band_y       <= 3'd0;
            band_init        <= 1'b0;
            nms_done_latched <= 1'b0;
            drain_req[0]     <= 1'b0;
            drain_req[1]     <= 1'b0;
            draining         <= 1'b0;
            drain_bank       <= 1'b0;
            drain_col        <= 4'd0;
            drain_slot       <= 3'd0;
            out_valid        <= 1'b0;
            out_x            <= 10'd0;
            out_y            <= 9'd0;
            out_score        <= 8'd0;
            out_count        <= {OUT_CNT_W{1'b0}};

            for (b_i = 0; b_i < 2; b_i = b_i + 1)
                for (c_i = 0; c_i < NUM_COLS; c_i = c_i + 1)
                    for (s_i = 0; s_i < K_PER_CELL; s_i = s_i + 1) begin
                        slot_vld[b_i][c_i][s_i] <= 1'b0;
                        slot_sc [b_i][c_i][s_i] <= 8'd0;
                        slot_x  [b_i][c_i][s_i] <= 10'd0;
                        slot_y  [b_i][c_i][s_i] <= 9'd0;
                    end
        end else if (start) begin
            act_bank         <= 1'b0;
            cur_band_y       <= 3'd0;
            band_init        <= 1'b0;
            nms_done_latched <= 1'b0;
            drain_req[0]     <= 1'b0;
            drain_req[1]     <= 1'b0;
            draining         <= 1'b0;
            drain_bank       <= 1'b0;
            drain_col        <= 4'd0;
            drain_slot       <= 3'd0;
            out_valid        <= 1'b0;
            out_count        <= {OUT_CNT_W{1'b0}};

            for (b_i = 0; b_i < 2; b_i = b_i + 1)
                for (c_i = 0; c_i < NUM_COLS; c_i = c_i + 1)
                    for (s_i = 0; s_i < K_PER_CELL; s_i = s_i + 1) begin
                        slot_vld[b_i][c_i][s_i] <= 1'b0;
                        slot_sc [b_i][c_i][s_i] <= 8'd0;
                        slot_x  [b_i][c_i][s_i] <= 10'd0;
                        slot_y  [b_i][c_i][s_i] <= 9'd0;
                    end
        end else begin
            out_valid <= 1'b0;

            // ----------------------------------------------------------------
            // A. Xu ly luong keypoint Stage 1 & chuyen doi Bank
            // ----------------------------------------------------------------
            if (vld_s1) begin
                if (!band_init) begin
                    band_init  <= 1'b1;
                    cur_band_y <= cell_y_s1;
                    slot_vld[act_bank][cell_x_s1][0] <= 1'b1;
                    slot_sc [act_bank][cell_x_s1][0] <= score_s1;
                    slot_x  [act_bank][cell_x_s1][0] <= abs_x_s1;
                    slot_y  [act_bank][cell_x_s1][0] <= abs_y_s1;
                end else if (band_changed) begin
                    drain_req[act_bank] <= 1'b1;
                    act_bank            <= ~act_bank;
                    cur_band_y          <= cell_y_s1;

                    slot_vld[~act_bank][cell_x_s1][0] <= 1'b1;
                    slot_sc [~act_bank][cell_x_s1][0] <= score_s1;
                    slot_x  [~act_bank][cell_x_s1][0] <= abs_x_s1;
                    slot_y  [~act_bank][cell_x_s1][0] <= abs_y_s1;
                end else begin
                    if (beat[0]) begin
                        slot_vld[act_bank][cell_x_s1][0] <= 1'b1;
                        slot_sc [act_bank][cell_x_s1][0] <= score_s1;
                        slot_x  [act_bank][cell_x_s1][0] <= abs_x_s1;
                        slot_y  [act_bank][cell_x_s1][0] <= abs_y_s1;
                    end
                    for (s_i = 1; s_i < K_PER_CELL; s_i = s_i + 1) begin
                        if (beat[s_i]) begin
                            if (!beat[s_i - 1]) begin
                                slot_vld[act_bank][cell_x_s1][s_i] <= 1'b1;
                                slot_sc [act_bank][cell_x_s1][s_i] <= score_s1;
                                slot_x  [act_bank][cell_x_s1][s_i] <= abs_x_s1;
                                slot_y  [act_bank][cell_x_s1][s_i] <= abs_y_s1;
                            end else begin
                                slot_vld[act_bank][cell_x_s1][s_i] <= slot_vld[act_bank][cell_x_s1][s_i - 1];
                                slot_sc [act_bank][cell_x_s1][s_i] <= slot_sc [act_bank][cell_x_s1][s_i - 1];
                                slot_x  [act_bank][cell_x_s1][s_i] <= slot_x  [act_bank][cell_x_s1][s_i - 1];
                                slot_y  [act_bank][cell_x_s1][s_i] <= slot_y  [act_bank][cell_x_s1][s_i - 1];
                            end
                        end
                    end
                end
            end

            if (nms_done_s1) begin
                nms_done_latched <= 1'b1;
                if (band_init) begin
                    if (band_changed)
                        drain_req[~act_bank] <= 1'b1;
                    else
                        drain_req[act_bank]  <= 1'b1;
                end
            end

            // ----------------------------------------------------------------
            // B. Co che xa du lieu co bat tay (Drain Engine)
            // ----------------------------------------------------------------
            if (!draining) begin
                if (drain_req[0]) begin
                    draining   <= 1'b1;
                    drain_bank <= 1'b0;
                    drain_col  <= 4'd0;
                    drain_slot <= 3'd0;
                end else if (drain_req[1]) begin
                    draining   <= 1'b1;
                    drain_bank <= 1'b1;
                    drain_col  <= 4'd0;
                    drain_slot <= 3'd0;
                end
            end else begin
                if (!cur_slot_vld) begin
                    if (drain_slot == K_PER_CELL[2:0] - 3'd1) begin
                        drain_slot <= 3'd0;
                        if (drain_col == NUM_COLS[3:0] - 4'd1) begin
                            draining              <= 1'b0;
                            drain_req[drain_bank] <= 1'b0;
                        end else begin
                            drain_col <= drain_col + 4'd1;
                        end
                    end else begin
                        drain_slot <= drain_slot + 3'd1;
                    end
                end else if (can_send) begin
                    out_valid <= 1'b1;
                    out_x     <= slot_x [drain_bank][drain_col][drain_slot];
                    out_y     <= slot_y [drain_bank][drain_col][drain_slot];
                    out_score <= slot_sc[drain_bank][drain_col][drain_slot];
                    out_count <= out_count + {{(OUT_CNT_W-1){1'b0}}, 1'b1};

                    slot_vld[drain_bank][drain_col][drain_slot] <= 1'b0;
                    slot_sc [drain_bank][drain_col][drain_slot] <= 8'd0;

                    if (drain_slot == K_PER_CELL[2:0] - 3'd1) begin
                        drain_slot <= 3'd0;
                        if (drain_col == NUM_COLS[3:0] - 4'd1) begin
                            draining              <= 1'b0;
                            drain_req[drain_bank] <= 1'b0;
                        end else begin
                            drain_col <= drain_col + 4'd1;
                        end
                    end else begin
                        drain_slot <= drain_slot + 3'd1;
                    end
                end
            end
        end
    end

    assign filter_done = nms_done_latched && !draining && !drain_req[0] && !drain_req[1];

endmodule
