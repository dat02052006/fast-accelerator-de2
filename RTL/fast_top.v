// ============================================================================
// fast_top.v
// Top-level FAST-9 Hardware Accelerator (Cyclone II / DE2):
//   SRAM Frame Buffer -> Line Buffer 7x7 -> FAST-9 Detector -> SAD Score
//   -> Sparse NMS 3x3 (634x474) -> FIFO 128x29b -> Grid 10x8 Top-4 Filter
//   -> Result Packer (3x16b) -> Result FIFO (scfifo) -> Avalon-MM DMA Write.
// ============================================================================
module fast_top #(
    parameter integer K_PER_CELL    = 4,    // Top-K moi o 64x64 (4 -> max 320 KP)
    parameter integer OUT_CNT_W     = 12,   // Do rong bo dem tong keypoint
    parameter integer DRAIN_CYC     = 16,   // So chu ky xa pipeline FAST/score sau frame_done
    parameter integer DMA_BURST_LEN = 1     // 1 cho Qsys SDR SDRAM controller
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // ---- Avalon-MM slave: cua so anh (tu JTAG-to-Avalon Master / DMA) ----
    input  wire [17:0]           avs_mem_address,        // dia chi word
    input  wire                  avs_mem_write,
    input  wire [15:0]           avs_mem_writedata,
    input  wire [1:0]            avs_mem_byteenable,
    output wire                  avs_mem_waitrequest,

    // ---- Avalon-MM slave: thanh ghi nap anh ----
    input  wire [1:0]            avs_ctrl_address,
    input  wire                  avs_ctrl_write,
    input  wire [15:0]           avs_ctrl_writedata,
    input  wire                  avs_ctrl_read,
    output wire [15:0]           avs_ctrl_readdata,

    // ---- Trang thai nap anh ----
    output wire                  image_loaded,   // = 1 khi da nap xong anh
    output wire                  load_mode,      // = 1 khi dang o che do nap anh

    // ---- Nios II: dieu khien xu ly & cau hinh DMA ----
    input  wire                  fast_start,     // xung 1 chu ky: xu ly 1 frame
    input  wire [7:0]            fast_threshold, // nguong FAST T
    input  wire [31:0]           dma_base_addr,  // dia chi byte bat dau luu ket qua tren SDRAM
    output wire                  fast_busy,      // muc 1 tu luc bat dau den khi DMA ghi xong
    output wire                  fast_done,      // xung 1 chu ky: xong toan bo frame + DMA
    output wire                  fast_error,     // xung 1 chu ky: loi line buffer / image_reader
    output wire                  packer_overflow,// muc 1 neu packer bi tran
    output wire [OUT_CNT_W-1:0]  out_count,      // tong so keypoint da loc va xuat ra
    output wire [31:0]           words_written,  // tong so word 16-bit DMA da ghi ra SDRAM

    // ---- Avalon-MM master: ghi ket qua keypoint ra SDRAM ----
    output wire [31:0]           avm_address,
    output wire                  avm_write,
    output wire [15:0]           avm_writedata,
    output wire [1:0]            avm_byteenable,
    output wire [7:0]            avm_burstcount,
    input  wire                  avm_waitrequest,

    // ---- Chan vat ly SRAM (DE2) ----
    output wire [17:0]           sram_addr,
    inout  wire [15:0]           sram_dq,
    output wire                  sram_ce_n,
    output wire                  sram_oe_n,
    output wire                  sram_we_n,
    output wire                  sram_ub_n,
    output wire                  sram_lb_n
);

    wire rst_h = ~rst_n;

    // ========================================================================
    // 1. FSM dieu khien toan khung hinh (Frame-level Master Controller)
    // ========================================================================
    localparam [2:0] T_IDLE      = 3'd0,
                     T_START     = 3'd1,
                     T_SCAN      = 3'd2,
                     T_DRAIN     = 3'd3,
                     T_NMS_FLUSH = 3'd4,
                     T_DMA_WAIT  = 3'd5,
                     T_DONE      = 3'd6,
                     T_ERR       = 3'd7;

    reg [2:0] state, next_state;
    reg [4:0] drain_cnt;
    reg       nms_ready;

    wire lb_frame_done;
    wire lb_error;
    wire nms_flush_done;
    wire dma_done;

    wire drain_end = (drain_cnt == (DRAIN_CYC - 1));

    always @(*) begin
        next_state = state;
        case (state)
            T_IDLE: begin
                if (fast_start && image_loaded && !load_mode && nms_ready)
                    next_state = T_START;
            end
            T_START:     next_state = T_SCAN;
            T_SCAN: begin
                if (lb_error)           next_state = T_ERR;
                else if (lb_frame_done) next_state = T_DRAIN;
            end
            T_DRAIN: begin
                if (drain_end) next_state = T_NMS_FLUSH;
            end
            T_NMS_FLUSH: next_state = T_DMA_WAIT;
            T_DMA_WAIT: begin
                if (dma_done) next_state = T_DONE;
            end
            T_DONE:      next_state = T_IDLE;
            T_ERR:       next_state = T_IDLE;
            default:     next_state = T_IDLE;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= T_IDLE;
        else        state <= next_state;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            drain_cnt <= 5'd0;
            nms_ready <= 1'b1;
        end else begin
            if (state == T_DRAIN)
                drain_cnt <= drain_end ? 5'd0 : (drain_cnt + 5'd1);
            else
                drain_cnt <= 5'd0;

            if (state == T_START)
                nms_ready <= 1'b0;
            else if (nms_flush_done)
                nms_ready <= 1'b1;
        end
    end

    wire frame_start    = (state == T_START);
    wire lb_start       = frame_start;
    wire nms_frame_done = (state == T_NMS_FLUSH);
    wire sel_fast       = (state == T_START) || (state == T_SCAN) || (state == T_DRAIN);

    assign fast_busy  = (state != T_IDLE);
    assign fast_done  = (state == T_DONE);
    assign fast_error = (state == T_ERR);

    // ========================================================================
    // 2. Nap anh tu PC / Nios II (avs_img_loader)
    // ========================================================================
    wire        wr_req;
    wire [17:0] wr_addr;
    wire [15:0] wr_data;
    wire        wr_ack;

    avs_img_loader #(
        .ADDR_W (18)
    ) u_avs_loader (
        .clk                (clk),
        .rst                (rst_h),
        .avs_mem_address    (avs_mem_address),
        .avs_mem_write      (avs_mem_write),
        .avs_mem_writedata  (avs_mem_writedata),
        .avs_mem_byteenable (avs_mem_byteenable),
        .avs_mem_waitrequest(avs_mem_waitrequest),
        .avs_ctrl_address   (avs_ctrl_address),
        .avs_ctrl_write     (avs_ctrl_write),
        .avs_ctrl_writedata (avs_ctrl_writedata),
        .avs_ctrl_read      (avs_ctrl_read),
        .avs_ctrl_readdata  (avs_ctrl_readdata),
        .wr_req             (wr_req),
        .wr_addr            (wr_addr),
        .wr_data            (wr_data),
        .wr_ack             (wr_ack),
        .load_mode          (load_mode),
        .load_done          (image_loaded)
    );

    // ========================================================================
    // 3. SRAM Controller + Mux 2 chu so huu (Loader / FAST Reader)
    // ========================================================================
    wire [17:0] ir_sram_addr;
    wire        ir_sram_re;
    wire        ir_sram_valid;

    wire [17:0] m_addr;
    wire [15:0] m_wdata;
    wire        m_we;
    wire        m_re;
    wire [15:0] m_rd_data;
    wire        m_ready;
    wire        m_valid;
    wire        m_done;

    wire        ld_grant = load_mode & ~fast_busy;

    assign m_addr        = sel_fast ? ir_sram_addr : wr_addr;
    assign m_re          = sel_fast ? ir_sram_re   : 1'b0;
    assign m_we          = ld_grant & wr_req;
    assign m_wdata       = wr_data;
    assign ir_sram_valid = m_valid & sel_fast;
    assign wr_ack        = m_done & ~m_valid & ld_grant;

    sram_controller u_sram (
        .clk       (clk),
        .rst_n     (rst_n),
        .addr      (m_addr),
        .wr_data   (m_wdata),
        .we        (m_we),
        .re        (m_re),
        .rd_data   (m_rd_data),
        .ready     (m_ready),
        .valid     (m_valid),
        .done      (m_done),
        .sram_addr (sram_addr),
        .sram_dq   (sram_dq),
        .sram_ce_n (sram_ce_n),
        .sram_oe_n (sram_oe_n),
        .sram_we_n (sram_we_n),
        .sram_ub_n (sram_ub_n),
        .sram_lb_n (sram_lb_n)
    );

    // ========================================================================
    // 4. Pipeline FAST-9 (BORDER = 3)
    // ========================================================================
    wire        ir_start;
    wire [8:0]  ir_first_row;
    wire [4:0]  ir_first_bank;
    wire [4:0]  ir_num_rows;
    wire        ir_done;
    wire        ir_error;

    wire        lb_we;
    wire [4:0]  lb_row;
    wire [8:0]  lb_col_word;
    wire [15:0] lb_wdata;

    wire        lb_rd_en;
    wire [9:0]  lb_rd_col;
    wire        lb_rd_valid;
    wire [7:0]  lb_p0, lb_p1, lb_p2, lb_p3, lb_p4, lb_p5, lb_p6;

    wire        win_clear;
    wire        win_in_valid;
    wire [7:0]  win_p0, win_p1, win_p2, win_p3, win_p4, win_p5, win_p6;
    wire [8:0]  lb_center_y;

    orb_line_buffer_ctrl u_lbctrl (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (lb_start),
        .busy          (),
        .frame_done    (lb_frame_done),
        .error         (lb_error),
        .row_done      (),
        .center_y      (lb_center_y),
        .ir_start      (ir_start),
        .ir_first_row  (ir_first_row),
        .ir_first_bank (ir_first_bank),
        .ir_num_rows   (ir_num_rows),
        .ir_done       (ir_done),
        .ir_error      (ir_error),
        .lb_rd_en      (lb_rd_en),
        .lb_rd_col     (lb_rd_col),
        .lb_rd_valid   (lb_rd_valid),
        .lb_p0 (lb_p0), .lb_p1 (lb_p1), .lb_p2 (lb_p2), .lb_p3 (lb_p3),
        .lb_p4 (lb_p4), .lb_p5 (lb_p5), .lb_p6 (lb_p6),
        .win_clear     (win_clear),
        .win_in_valid  (win_in_valid),
        .win_p0 (win_p0), .win_p1 (win_p1), .win_p2 (win_p2), .win_p3 (win_p3),
        .win_p4 (win_p4), .win_p5 (win_p5), .win_p6 (win_p6)
    );

    image_reader u_imgrd (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (ir_start),
        .first_row    (ir_first_row),
        .num_rows     (ir_num_rows),
        .first_bank   (ir_first_bank),
        .busy         (),
        .done         (ir_done),
        .error        (ir_error),
        .sram_addr    (ir_sram_addr),
        .sram_re      (ir_sram_re),
        .sram_rd_data (m_rd_data),
        .sram_valid   (ir_sram_valid),
        .lb_we        (lb_we),
        .lb_row       (lb_row),
        .lb_col_word  (lb_col_word),
        .lb_wdata     (lb_wdata)
    );

    orb_line_buffer u_linebuf (
        .clk         (clk),
        .lb_we       (lb_we),
        .lb_row      (lb_row),
        .lb_col_word (lb_col_word),
        .lb_wdata    (lb_wdata),
        .rd_en       (lb_rd_en),
        .rd_col      (lb_rd_col),
        .rd_valid    (lb_rd_valid),
        .rd_p0 (lb_p0), .rd_p1 (lb_p1), .rd_p2 (lb_p2), .rd_p3 (lb_p3),
        .rd_p4 (lb_p4), .rd_p5 (lb_p5), .rd_p6 (lb_p6)
    );

    wire        window_valid;
    wire [9:0]  win_center_x;
    wire [7:0]  w02, w03, w04;
    wire [7:0]  w11, w15;
    wire [7:0]  w20, w26;
    wire [7:0]  w30, w33, w36;
    wire [7:0]  w40, w46;
    wire [7:0]  w51, w55;
    wire [7:0]  w62, w63, w64;

    orb_window_generator #(
        .COL_OFFSET (10'd0)
    ) u_window (
        .clk          (clk),
        .rst_n        (rst_n),
        .clear        (win_clear),
        .in_valid     (win_in_valid),
        .in_p0 (win_p0), .in_p1 (win_p1), .in_p2 (win_p2), .in_p3 (win_p3),
        .in_p4 (win_p4), .in_p5 (win_p5), .in_p6 (win_p6),
        .window_valid (window_valid),
        .win_center_x (win_center_x),
        .window_flat  (),
        .w00(), .w01(), .w02(w02), .w03(w03), .w04(w04), .w05(), .w06(),
        .w10(), .w11(w11), .w12(), .w13(), .w14(), .w15(w15), .w16(),
        .w20(w20), .w21(), .w22(), .w23(), .w24(), .w25(), .w26(w26),
        .w30(w30), .w31(), .w32(), .w33(w33), .w34(), .w35(), .w36(w36),
        .w40(w40), .w41(), .w42(), .w43(), .w44(), .w45(), .w46(w46),
        .w50(), .w51(w51), .w52(), .w53(), .w54(), .w55(w55), .w56(),
        .w60(), .w61(), .w62(w62), .w63(w63), .w64(w64), .w65(), .w66()
    );

    wire         det_valid;
    wire         det_is_corner;
    wire [9:0]   det_x;
    wire [9:0]   det_y;
    wire [135:0] det_pack;

    orb_fast_detector #(
        .IMG_W   (640),
        .IMG_H   (480),
        .BORDER  (3),
        .COORD_W (10)
    ) u_fast (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (window_valid),
        .in_x      (win_center_x),
        .in_y      ({1'b0, lb_center_y}),
        .threshold (fast_threshold),
        .ip        (w33),
        .p0  (w03), .p1  (w04), .p2  (w15), .p3  (w26),
        .p4  (w36), .p5  (w46), .p6  (w55), .p7  (w64),
        .p8  (w63), .p9  (w62), .p10 (w51), .p11 (w40),
        .p12 (w30), .p13 (w20), .p14 (w11), .p15 (w02),
        .out_valid (det_valid),
        .is_corner (det_is_corner),
        .out_x     (det_x),
        .out_y     (det_y),
        .pack      (det_pack)
    );

    wire        score_valid;
    wire [7:0]  score_value;

    orb_fast_score #(
        .COORD_W   (10),
        .SCORE_DEC (1)
    ) u_score (
        .clk          (clk),
        .rst_n        (rst_n),
        .in_valid     (det_valid),
        .in_is_corner (det_is_corner),
        .in_x         (det_x),
        .in_pack      (det_pack),
        .score_valid  (score_valid),
        .score_x      (),
        .score_value  (score_value),
        .row_start    (),
        .row_done     ()
    );

    // ========================================================================
    // 5. Sparse NMS 3x3 (634 x 474)
    // ========================================================================
    reg [5:0] det_vld_sr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) det_vld_sr <= 6'd0;
        else        det_vld_sr <= {det_vld_sr[4:0], det_valid};
    end
    wire nms_pix_valid = det_vld_sr[5];

    wire       nms_kp_valid;
    wire [9:0] nms_kp_x;
    wire [8:0] nms_kp_y;
    wire [7:0] nms_kp_score;

    nms_stream_v3 #(
        .IMG_W      (634),              // 640 - 2*3
        .IMG_H      (474),              // 480 - 2*3
        .ADDR_WIDTH (10)
    ) u_nms (
        .clk        (clk),
        .rst_n      (rst_n),
        .pix_valid  (nms_pix_valid),
        .in_score   (score_value),
        .in_valid   (score_valid),
        .frame_done (nms_frame_done),
        .kp_valid   (nms_kp_valid),
        .kp_x       (nms_kp_x),
        .kp_y       (nms_kp_y),
        .kp_score   (nms_kp_score),
        .flush_done (nms_flush_done)
    );

    // ========================================================================
    // 6. FIFO trung gian 128 x 29-bit (NMS -> Keypoint Filter, 1 khoi M4K)
    // ========================================================================
    wire        nms_fifo_wrreq = nms_kp_valid || nms_flush_done;
    wire [28:0] nms_fifo_din = nms_flush_done ? 29'h1000_0000
                                          : {1'b0, nms_kp_valid, nms_kp_score, nms_kp_y, nms_kp_x};
    wire [28:0] nms_fifo_dout;
    wire        nms_fifo_empty;
    wire        nms_fifo_full;
    wire        kp_in_ready;
    wire        nms_fifo_rdreq = (!nms_fifo_empty) && kp_in_ready;

    scfifo #(
        .intended_device_family ("Cyclone II"),
        .lpm_numwords           (128),
        .lpm_showahead          ("ON"),
        .lpm_type               ("scfifo"),
        .lpm_width              (29),
        .lpm_widthu             (7),
        .overflow_checking      ("ON"),
        .underflow_checking     ("ON"),
        .use_eab                ("ON")
    ) u_nms_kp_fifo (
        .clock (clk),
        .sclr  (rst_h | frame_start),
        .data  (nms_fifo_din),
        .wrreq (nms_fifo_wrreq),
        .rdreq (nms_fifo_rdreq),
        .q     (nms_fifo_dout),
        .empty (nms_fifo_empty),
        .full  (nms_fifo_full),
        .usedw ()
    );

    // ========================================================================
    // 7. Keypoint Filter (Grid 10x8 tren 640x480, Top-4 moi o, M4K RAM)
    // ========================================================================
    wire       filt_out_valid;
    wire [9:0] filt_out_x;
    wire [8:0] filt_out_y;
    wire [7:0] filt_out_score;
    wire       filt_done;
    wire       packer_busy;
    wire       res_fifo_full;

    keypoint_filter #(
        .OUT_CNT_W  (OUT_CNT_W)
    ) u_kp_filter (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (frame_start),
        .in_valid    (!nms_fifo_empty && nms_fifo_dout[27]),
        .in_x        (nms_fifo_dout[9:0]),
        .in_y        (nms_fifo_dout[18:10]),
        .in_score    (nms_fifo_dout[26:19]),
        .nms_done    (!nms_fifo_empty && nms_fifo_dout[28]),
        .in_ready    (kp_in_ready),
        .packer_busy (packer_busy),
        .fifo_full   (res_fifo_full),
        .out_valid   (filt_out_valid),
        .out_x       (filt_out_x),
        .out_y       (filt_out_y),
        .out_score   (filt_out_score),
        .filter_done (filt_done),
        .out_count   (out_count)
    );

    // ========================================================================
    // 8. Result Packer (3x16b) + Result FIFO + Avalon-MM DMA Write
    // ========================================================================
    wire [15:0] res_fifo_wdata;
    wire        res_fifo_wr;
    wire [15:0] res_fifo_rdata;
    wire        res_fifo_empty;
    wire [9:0]  res_fifo_usedw;
    wire        res_fifo_rd;
    wire        dma_src_done;

    fast_result_packer u_res_packer (
        .clk             (clk),
        .rst             (rst_h),
        .start           (frame_start),
        .filter_done     (filt_done),
        .out_we          (filt_out_valid),
        .out_x           (filt_out_x),
        .out_y           (filt_out_y),
        .out_score       (filt_out_score),
        .busy            (packer_busy),
        .fifo_full       (res_fifo_full),
        .fifo_wdata      (res_fifo_wdata),
        .fifo_wr         (res_fifo_wr),
        .src_done        (dma_src_done),
        .packer_overflow (packer_overflow)
    );

    scfifo #(
        .intended_device_family ("Cyclone II"),
        .lpm_numwords           (1024),
        .lpm_showahead          ("ON"),
        .lpm_type               ("scfifo"),
        .lpm_width              (16),
        .lpm_widthu             (10),
        .overflow_checking      ("ON"),
        .underflow_checking     ("ON"),
        .use_eab                ("ON")
    ) u_res_fifo (
        .clock (clk),
        .sclr  (rst_h),
        .data  (res_fifo_wdata),
        .wrreq (res_fifo_wr),
        .rdreq (res_fifo_rd),
        .q     (res_fifo_rdata),
        .empty (res_fifo_empty),
        .full  (res_fifo_full),
        .usedw (res_fifo_usedw)
    );

    orb_dma_write #(
        .DATA_W    (16),
        .ADDR_W    (32),
        .BURST_LEN (DMA_BURST_LEN),
        .FIFO_CW   (10)
    ) u_dma_write (
        .clk            (clk),
        .rst            (rst_h),
        .start          (frame_start),
        .base_addr      (dma_base_addr),
        .src_done       (dma_src_done),
        .busy           (),
        .done           (dma_done),
        .words_written  (words_written),
        .fifo_rdata     (res_fifo_rdata),
        .fifo_empty     (res_fifo_empty),
        .fifo_full      (res_fifo_full),
        .fifo_count     (res_fifo_usedw),
        .fifo_rd        (res_fifo_rd),
        .avm_address    (avm_address),
        .avm_write      (avm_write),
        .avm_writedata  (avm_writedata),
        .avm_byteenable (avm_byteenable),
        .avm_burstcount (avm_burstcount),
        .avm_waitrequest(avm_waitrequest)
    );

endmodule