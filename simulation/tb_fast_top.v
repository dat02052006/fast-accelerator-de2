`timescale 1ns / 1ps

// ============================================================================
// tb_fast_top.v
// Full-system Testbench cho fast_top (Cyclone II / DE2):
//   - Mo hinh SRAM 256K x 16-bit (IS61LV25616AL)
//   - Mo hinh Avalon-MM Slave SDRAM (co waitrequest ngau nhien)
//   - Quy trinh nap anh 640x480 -> Chay FAST-9 + NMS + Grid Filter -> DMA SDRAM
// ============================================================================
module tb_fast_top;

    // ========================================================================
    // 1. Tham so & Tin hieu ket noi DUT
    // ========================================================================
    localparam integer IMG_W       = 640;
    localparam integer IMG_H       = 480;
    localparam integer TOTAL_WORDS = (IMG_W * IMG_H) / 2; // 153,600 words 16-bit
    localparam real    CLK_PERIOD  = 20.0;                // 50 MHz

    reg         clk;
    reg         rst_n;

    // Avalon-MM slave: cua so nap anh
    reg  [17:0] avs_mem_address;
    reg         avs_mem_write;
    reg  [15:0] avs_mem_writedata;
    reg  [1:0]  avs_mem_byteenable;
    wire        avs_mem_waitrequest;

    // Avalon-MM slave: thanh ghi dieu khien nap anh
    reg  [1:0]  avs_ctrl_address;
    reg         avs_ctrl_write;
    reg  [15:0] avs_ctrl_writedata;
    reg         avs_ctrl_read;
    wire [15:0] avs_ctrl_readdata;

    wire        image_loaded;
    wire        load_mode;

    // Nios II control & status
    reg         fast_start;
    reg  [7:0]  fast_threshold;
    reg  [31:0] dma_base_addr;
    wire        fast_busy;
    wire        fast_done;
    wire        fast_error;
    wire        packer_overflow;
    wire [11:0] out_count;
    wire [31:0] words_written;

    // Avalon-MM master: ghi ket qua ra SDRAM
    wire [31:0] avm_address;
    wire        avm_write;
    wire [15:0] avm_writedata;
    wire [1:0]  avm_byteenable;
    wire [7:0]  avm_burstcount;
    reg         avm_waitrequest;

    // Chan vat ly SRAM
    wire [17:0] sram_addr;
    wire [15:0] sram_dq;
    wire        sram_ce_n;
    wire        sram_oe_n;
    wire        sram_we_n;
    wire        sram_ub_n;
    wire        sram_lb_n;
    initial begin
        wait (fast_start);
        forever begin
            repeat (100000) @(posedge clk);
            $display("[%0t] DEBUG: top_state=%0d | nms_flush=%b, fifo_empty=%b, fifo_dout[28:27]=%b | kp_state=%0d, kp_in_rdy=%b, col_dirty=%b, out_cnt=%0d | packer_busy=%b, res_empty=%b",
                     $time, u_dut.state, u_dut.nms_flush_done, u_dut.nms_fifo_empty, u_dut.nms_fifo_dout[28:27],
                     u_dut.u_kp_filter.state, u_dut.kp_in_ready, u_dut.u_kp_filter.col_dirty, u_dut.out_count,
                     u_dut.packer_busy, u_dut.res_fifo_empty);
        end
    end
    always @(u_dut.state) begin
        $display("[%0t] fast_top FSM -> state = %0d (lb_done=%b, nms_flush=%b, filt_done=%b, dma_done=%b)",
                 $time, u_dut.state, u_dut.lb_frame_done, u_dut.nms_flush_done, u_dut.filt_done, u_dut.dma_done);
    end

    // ========================================================================
    // 2. Khoi tao DUT (fast_top)
    // ========================================================================
    fast_top #(
        .OUT_CNT_W     (12),
        .DRAIN_CYC     (16),
        .DMA_BURST_LEN (1)
    ) u_dut (
        .clk                (clk),
        .rst_n              (rst_n),

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

        .image_loaded       (image_loaded),
        .load_mode          (load_mode),

        .fast_start         (fast_start),
        .fast_threshold     (fast_threshold),
        .dma_base_addr      (dma_base_addr),
        .fast_busy          (fast_busy),
        .fast_done          (fast_done),
        .fast_error         (fast_error),
        .packer_overflow    (packer_overflow),
        .out_count          (out_count),
        .words_written      (words_written),

        .avm_address        (avm_address),
        .avm_write          (avm_write),
        .avm_writedata      (avm_writedata),
        .avm_byteenable     (avm_byteenable),
        .avm_burstcount     (avm_burstcount),
        .avm_waitrequest    (avm_waitrequest),

        .sram_addr          (sram_addr),
        .sram_dq            (sram_dq),
        .sram_ce_n          (sram_ce_n),
        .sram_oe_n          (sram_oe_n),
        .sram_we_n          (sram_we_n),
        .sram_ub_n          (sram_ub_n),
        .sram_lb_n          (sram_lb_n)
    );

    // ========================================================================
    // 3. Tao xung Clock 50 MHz
    // ========================================================================
    initial clk = 1'b0;
    always #(CLK_PERIOD / 2.0) clk = ~clk;

    // ========================================================================
    // 4. Mo hinh chip SRAM bat dong bo 256K x 16-bit (DE2)
    // ========================================================================
    reg [15:0] sram_mem [0:262143];
    reg [15:0] sram_dout_delayed;

    // Tre doc mo phong ~10ns
    always @(*) begin
        #10 sram_dout_delayed = sram_mem[sram_addr];
    end

    assign sram_dq = (!sram_ce_n && !sram_oe_n && sram_we_n) ? sram_dout_delayed : 16'hzzzz;

    // Ghi vao SRAM khi WE_N chuyen muc hoac trong luc WE_N = 0
    always @(posedge clk) begin
        if (!sram_ce_n && !sram_we_n) begin
            if (!sram_lb_n) sram_mem[sram_addr][7:0]  <= sram_dq[7:0];
            if (!sram_ub_n) sram_mem[sram_addr][15:8] <= sram_dq[15:8];
        end
    end

    // ========================================================================
    // 5. Mo hinh SDRAM Slave nhan ket qua DMA qua Avalon-MM Master
    // ========================================================================
    reg [15:0] sdram_rx_buf [0:4095];
    integer    dma_rx_cnt;
    integer    rng_seed;

    initial begin
        avm_waitrequest = 1'b0;
        dma_rx_cnt      = 0;
        rng_seed        = 32'h12345678;
    end

    // Tao backpressure ngau nhien tu SDRAM controller (~20% thoi gian)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            avm_waitrequest <= 1'b0;
        end else begin
            rng_seed = rng_seed * 1103515245 + 12345;
            avm_waitrequest <= (rng_seed[7:0] < 8'd50);
        end
    end

    // Bat cac giao dich ghi hop le (avm_write && !avm_waitrequest)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dma_rx_cnt <= 0;
        end else if (fast_start) begin
            dma_rx_cnt <= 0;
        end else if (avm_write && !avm_waitrequest) begin
            sdram_rx_buf[dma_rx_cnt] <= avm_writedata;
            dma_rx_cnt <= dma_rx_cnt + 1;
        end
    end

    // ========================================================================
    // 6. Bo dem anh mau 640x480 & Cac Task giao tiep Avalon-MM
    // ========================================================================
    reg [7:0] img_buf [0:(IMG_W*IMG_H)-1];

    // Tao anh test co nhieu goc vuong sang tren nen toi (tao keypoint FAST-9)
    task build_synthetic_image;
        integer x, y, idx;
        integer bx, by;
        begin
            // Nen toi dong nhat (pixel = 20)
            for (y = 0; y < IMG_H; y = y + 1) begin
                for (x = 0; x < IMG_W; x = x + 1) begin
                    img_buf[y * IMG_W + x] = 8'd20;
                end
            end

            // Ve luoi cac o vuong sang 12x12 cach nhau 48 pixel -> tao nhieu goc FAST-9
            for (by = 24; by < IMG_H - 32; by = by + 48) begin
                for (bx = 24; bx < IMG_W - 32; bx = bx + 48) begin
                    for (y = by; y < by + 12; y = y + 1) begin
                        for (x = bx; x < bx + 12; x = x + 1) begin
                            img_buf[y * IMG_W + x] = 8'd220;
                        end
                    end
                end
            end

            // Them 1 goc dac biet do tuong phan cuc cao o (100, 100)
            for (y = 100; y < 116; y = y + 1) begin
                for (x = 100; x < 116; x = x + 1) begin
                    img_buf[y * IMG_W + x] = 8'd255;
                end
            end
        end
    endtask

    // Task ghi 1 word vao avs_ctrl
    task avs_ctrl_wr(input [1:0] addr, input [15:0] data);
        begin
            @(posedge clk);
            avs_ctrl_address   <= addr;
            avs_ctrl_writedata <= data;
            avs_ctrl_write     <= 1'b1;
            @(posedge clk);
            avs_ctrl_write     <= 1'b0;
        end
    endtask

    // Task ghi 1 word vao avs_mem (co xu ly waitrequest)
    task avs_mem_wr(input [17:0] addr, input [15:0] data);
        begin
            @(posedge clk);
            avs_mem_address    <= addr;
            avs_mem_writedata  <= data;
            avs_mem_byteenable <= 2'b11;
            avs_mem_write      <= 1'b1;
            @(posedge clk);
            while (avs_mem_waitrequest) begin
                @(posedge clk);
            end
            avs_mem_write      <= 1'b0;
        end
    endtask

    // Task nap toan bo anh 640x480 qua avs_img_loader
    task load_image_to_sram;
        integer w;
        reg [15:0] packed_word;
        begin
            $display("[%0t] Bat dau nap anh 640x480 (%0d words) vao SRAM...", $time, TOTAL_WORDS);
            // Bat che do load_mode (bit 0 = 1)
            avs_ctrl_wr(2'd0, 16'h0001);

            for (w = 0; w < TOTAL_WORDS; w = w + 1) begin
                packed_word = {img_buf[2*w + 1], img_buf[2*w]};
                avs_mem_wr(w[17:0], packed_word);
            end

            // Ket thuc nap anh -> bat image_loaded (bit 1 = 1, bit 0 = 0)
            avs_ctrl_wr(2'd0, 16'h0002);
            repeat (5) @(posedge clk);
            $display("[%0t] Nap anh hoan tat! image_loaded = %b, load_mode = %b",
                     $time, image_loaded, load_mode);
        end
    endtask

    // ========================================================================
    // 7. Kich ban chay mo phong chinh
    // ========================================================================
    integer i, kp_idx;
    integer start_cycle, end_cycle, cycle_cnt;
    reg [15:0] w0, w1, w2;
    reg [9:0]  dec_x;
    reg [8:0]  dec_y;
    reg [7:0]  dec_score;

    // Bo dem chu ky he thong
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) cycle_cnt <= 0;
        else        cycle_cnt <= cycle_cnt + 1;
    end

    initial begin
        // Khoi tao tin hieu
        rst_n              = 1'b0;
        avs_mem_address    = 18'd0;
        avs_mem_write      = 1'b0;
        avs_mem_writedata  = 16'd0;
        avs_mem_byteenable = 2'b11;
        avs_ctrl_address   = 2'd0;
        avs_ctrl_write     = 1'b0;
        avs_ctrl_writedata = 16'd0;
        avs_ctrl_read      = 1'b0;
        fast_start         = 1'b0;
        fast_threshold     = 8'd40;
        dma_base_addr      = 32'h0100_0000;

        // Xoa sach SRAM mo phong
        for (i = 0; i < 262144; i = i + 1)
            sram_mem[i] = 16'd0;

        // Tao anh test
        build_synthetic_image();

        // Nha Reset
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        // Buoc 1: Nap anh vao SRAM thong qua Avalon-MM Slave
        load_image_to_sram();

        if (!image_loaded) begin
            $display("ERROR: image_loaded khong len muc 1!");
            $finish;
        end

        // Buoc 2: Phat xung fast_start 1 chu ky de bat dau xu ly
        @(posedge clk);
        start_cycle = cycle_cnt;
        fast_start  <= 1'b1;
        $display("[%0t] Phat xung fast_start (threshold = %0d)...", $time, fast_threshold);
        @(posedge clk);
        fast_start  <= 1'b0;

        // Buoc 3: Cho den khi fast_done = 1 hoac fast_error = 1 (co watchdog timeout)
        while (!fast_done && !fast_error && ((cycle_cnt - start_cycle) < 1500000)) begin
            @(posedge clk);
        end

        if (!fast_done && !fast_error) begin
            $display("ERROR: Timeout! Mach khong hoan thanh sau 600,000 chu ky.");
            $finish;
        end
        end_cycle = cycle_cnt;
        // Cho them 20 chu ky de cac giao dich DMA cuoi on dinh
        repeat (20) @(posedge clk);

        // ====================================================================
        // 8. In bao cao ket qua & Kiem tra tinh dung dan
        // ====================================================================
        $display("============================================================");
        $display("KET QUA MO PHONG TOAN HE THONG FAST-9 ACCELERATOR");
        $display("============================================================");
        $display("Thoi gian xu ly toan frame : %0d chu ky (%.2f ms @ 50MHz)",
                 (end_cycle - start_cycle), (end_cycle - start_cycle) * CLK_PERIOD / 1e6);
        $display("fast_error                 : %b", fast_error);
        $display("packer_overflow            : %b", packer_overflow);
        $display("out_count (Filter xuat)    : %0d keypoints", out_count);
        $display("words_written (DMA)        : %0d words (16-bit)", words_written);
        $display("So word SDRAM nhan duoc    : %0d words", dma_rx_cnt);
        $display("------------------------------------------------------------");

        if (fast_error || packer_overflow) begin
            $display("FAIL: Phat hien co loi (fast_error hoac packer_overflow)!");
        end else if (out_count == 0) begin
            $display("FAIL: Khong phat hien duoc keypoint nao!");
        end else if (dma_rx_cnt != out_count * 3) begin
            $display("WARNING: So word DMA (%0d) khac 3 * out_count (%0d)!",
                     dma_rx_cnt, out_count * 3);
        end else begin
            $display("PASS: Toàn bo %0d keypoint da duoc loc va ghi dung xuong SDRAM!", out_count);
        end

        // Giai ma 15 keypoint dau tien tu SDRAM (moi KP gom 3 word 16-bit)
        $display("Danh sach 15 keypoint dau tien trong SDRAM:");
        for (kp_idx = 0; kp_idx < out_count && kp_idx < 15; kp_idx = kp_idx + 1) begin
            w0 = sdram_rx_buf[kp_idx * 3 + 0];
            w1 = sdram_rx_buf[kp_idx * 3 + 1];
            w2 = sdram_rx_buf[kp_idx * 3 + 2];
            $display("  KP[%0d]: word0=0x%04h, word1=0x%04h, word2=0x%04h (x=%0d, y=%0d)",
                     kp_idx, w0, w1, w2, w0[9:0], w1[8:0]);
        end
        $display("============================================================");
        $finish;
    end

endmodule
