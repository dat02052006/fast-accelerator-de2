// ============================================================================
// keypoint_filter.v  (Single cur_band, Band-Sweep Drain, 1x M4K, 3-State FSM)
// Bo loc Keypoint theo grid 32x32 bat dau tu (0,0) tren ngo ra NMS (634x474)
// -> 20 cot x 15 hang = 300 o, moi o giu Top-1 keypoint.
// ============================================================================
module keypoint_filter #(
    parameter integer OUT_CNT_W = 12        // Do rong bo dem tong so keypoint xuat ra
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 start,          // Xung 1 chu ky bat dau frame moi

    // ---- Bat tay & Dau vao tu FIFO trung gian (FWFT) ----
    input  wire                 in_valid,       // !fifo_empty && valid_bit
    input  wire [9:0]           in_x,           // 0..633
    input  wire [8:0]           in_y,           // 0..473
    input  wire [7:0]           in_score,       // 1..255
    input  wire                 nms_done,       // !fifo_empty && is_last_bit
    output wire                 in_ready,       // = 1 khi san sang doc tu FIFO

    // ---- Bat tay & ngo ra toi fast_result_packer ----
    input  wire                 packer_busy,    // = 1 khi packer dang dich word
    input  wire                 fifo_full,      // = 1 khi scfifo dau ra day
    output reg                  out_valid,      // Xung 1 chu ky gui 1 keypoint
    output reg  [9:0]           out_x,          // Toa do X (0..633)
    output reg  [8:0]           out_y,          // Toa do Y (0..473)
    output reg  [7:0]           out_score,

    // ---- Trang thai & thong ke ----
    output wire                 filter_done,    // Muc 1 khi FSM dat trang thai S_DONE
    output reg  [OUT_CNT_W-1:0] out_count       // Tong so keypoint da gui xuong packer
);

    // ========================================================================
    // Trang thai quan ly: Chi 1 thanh ghi cur_band (4-bit) cho ca 20 cot!
    // ========================================================================
    reg [19:0] col_dirty;
    reg [3:0]  cur_band;
    reg [7:0]  best_score [0:19];

    reg        vld_s1;
    reg [4:0]  col_s1;
    reg [27:0] pkt_s1;
    wire       ins_we;

    // ========================================================================
    // Stage 1: Cat bit toa do & Phat hien sang bang moi (1 phep so sanh duy nhat)
    // ========================================================================
    wire [4:0]  cell_x  = in_x[9:5];    // 0..19
    wire [3:0]  cell_y  = in_y[8:5];    // 0..14
    wire [27:0] new_pkt = {1'b1, in_score, in_x, in_y};

    wire has_data     = (col_dirty != 20'd0) || vld_s1;
    wire band_changed = (in_valid === 1'b1) && has_data && (cell_y != cur_band);

    // ========================================================================
    // FSM 3 trang thai: S_IDLE, S_DRAIN, S_DONE
    // ========================================================================
    localparam [1:0] S_IDLE  = 2'd0,
                     S_DRAIN = 2'd1,
                     S_DONE  = 2'd2;

    reg [1:0] state, next_state;
    reg       final_flush;      // = 1 khi xa bang cuoi cung luc nms_done
    reg [4:0] drain_col;        // Bo dem quet tuan tu 0..19

    assign in_ready = (state == S_IDLE) && !band_changed;

    wire accept_in   = in_valid && in_ready;
    wire accept_done = nms_done && in_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vld_s1 <= 1'b0;
            col_s1 <= 5'd0;
            pkt_s1 <= 28'd0;
        end else if (start) begin
            vld_s1 <= 1'b0;
            col_s1 <= 5'd0;
            pkt_s1 <= 28'd0;
        end else begin
            vld_s1 <= accept_in;
            col_s1 <= cell_x;
            pkt_s1 <= new_pkt;
        end
    end

    // ========================================================================
    // Stage 2: So sanh Top-1 tren Flip-Flop & Ghi 1 chieu vao M4K RAM
    // ========================================================================
    wire [7:0] in_sc_s1       = pkt_s1[26:19];
    wire [7:0] cur_best_score = best_score[col_s1];

    assign ins_we = vld_s1 && (!col_dirty[col_s1] || (in_sc_s1 > cur_best_score));

    // ========================================================================
    // M4K Block RAM: 32 x 28-bit
    // ========================================================================
    (* ramstyle = "M4K" *) reg [27:0] grid_ram [0:31];
    reg  [27:0] drain_ram_q;
    wire [4:0]  drain_raddr;

    always @(posedge clk) begin
        if (ins_we)
            grid_ram[col_s1] <= pkt_s1;
        drain_ram_q <= grid_ram[drain_raddr];
    end

    wire [4:0] next_col = drain_col + 5'd1;
    assign drain_raddr  = (state == S_IDLE) ? 5'd0 : next_col;

    wire can_send     = !packer_busy && !fifo_full && !out_valid;
    wire slot_valid   = col_dirty[drain_col] && drain_ram_q[27];
    wire step_advance = (state == S_DRAIN) && (!slot_valid || can_send);
    wire sweep_end    = step_advance && (drain_col == 5'd19);

    // 1) Next-State Logic
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if ((band_changed || (accept_done && has_data)) && !vld_s1)
                    next_state = S_DRAIN;
                else if (accept_done && !has_data)
                    next_state = S_DONE;
            end

            S_DRAIN: begin
                if (sweep_end)
                    next_state = final_flush ? S_DONE : S_IDLE;
            end

            S_DONE: begin
                next_state = S_DONE;
            end

            default: next_state = S_IDLE;
        endcase
    end

    // 2) State Register
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= S_IDLE;
        else if (start)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    // 3) Output Decode
    assign filter_done = (state == S_DONE);

    // ========================================================================
    // Datapath: Cap nhat Metadata & Quet xa tron bang (0..19)
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_dirty <= 20'd0;
            cur_band  <= 4'd0;

            best_score[0]  <= 8'd0; best_score[1]  <= 8'd0;
            best_score[2]  <= 8'd0; best_score[3]  <= 8'd0;
            best_score[4]  <= 8'd0; best_score[5]  <= 8'd0;
            best_score[6]  <= 8'd0; best_score[7]  <= 8'd0;
            best_score[8]  <= 8'd0; best_score[9]  <= 8'd0;
            best_score[10] <= 8'd0; best_score[11] <= 8'd0;
            best_score[12] <= 8'd0; best_score[13] <= 8'd0;
            best_score[14] <= 8'd0; best_score[15] <= 8'd0;
            best_score[16] <= 8'd0; best_score[17] <= 8'd0;
            best_score[18] <= 8'd0; best_score[19] <= 8'd0;

            final_flush <= 1'b0;
            drain_col   <= 5'd0;
            out_valid   <= 1'b0;
            out_x       <= 10'd0;
            out_y       <= 9'd0;
            out_score   <= 8'd0;
            out_count   <= {OUT_CNT_W{1'b0}};
        end else if (start) begin
            col_dirty <= 20'd0;
            cur_band  <= 4'd0;

            best_score[0]  <= 8'd0; best_score[1]  <= 8'd0;
            best_score[2]  <= 8'd0; best_score[3]  <= 8'd0;
            best_score[4]  <= 8'd0; best_score[5]  <= 8'd0;
            best_score[6]  <= 8'd0; best_score[7]  <= 8'd0;
            best_score[8]  <= 8'd0; best_score[9]  <= 8'd0;
            best_score[10] <= 8'd0; best_score[11] <= 8'd0;
            best_score[12] <= 8'd0; best_score[13] <= 8'd0;
            best_score[14] <= 8'd0; best_score[15] <= 8'd0;
            best_score[16] <= 8'd0; best_score[17] <= 8'd0;
            best_score[18] <= 8'd0; best_score[19] <= 8'd0;

            final_flush <= 1'b0;
            drain_col   <= 5'd0;
            out_valid   <= 1'b0;
            out_x       <= 10'd0;
            out_y       <= 9'd0;
            out_score   <= 8'd0;
            out_count   <= {OUT_CNT_W{1'b0}};
        end else begin
            out_valid <= 1'b0;

            // Cap nhat cur_band khi bat dau bang dau tien hoac bang moi
            if (accept_in)
                cur_band <= cell_y;

            // Cap nhat Top-1 metadata
            if (ins_we) begin
                col_dirty[col_s1]  <= 1'b1;
                best_score[col_s1] <= in_sc_s1;
            end

            // Xoa co dirty khi cot da xa xong
            if (step_advance)
                col_dirty[drain_col] <= 1'b0;

            case (state)
                S_IDLE: begin
                    drain_col <= 5'd0;
                    if (band_changed && !vld_s1)
                        final_flush <= 1'b0;
                    else if (accept_done && has_data && !vld_s1)
                        final_flush <= 1'b1;
                end

                S_DRAIN: begin
                    if (slot_valid && can_send) begin
                        out_valid <= 1'b1;
                        out_score <= drain_ram_q[26:19];
                        out_x     <= drain_ram_q[18:9];
                        out_y     <= drain_ram_q[8:0];
                        out_count <= out_count + {{(OUT_CNT_W-1){1'b0}}, 1'b1};
                    end

                    if (step_advance && !sweep_end)
                        drain_col <= next_col;
                end

                S_DONE: begin
                end

                default: begin
                end
            endcase
        end
    end

endmodule