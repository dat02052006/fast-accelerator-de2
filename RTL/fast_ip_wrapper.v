// ============================================================================
// fast_ip_wrapper.v
// Qsys Top-Level Wrapper cho FAST-9 Hardware Accelerator IP Core (Cyclone II).
//
// Cau hinh Interfaces trong Qsys Component Editor:
//   1. clock            : Clock Input (clk)
//   2. reset            : Reset Input (rst_n, Active Low)
//   3. avs_csr          : Avalon-MM Slave (32-bit, Read Latency = 1, Write Latency = 0)
//   4. avs_mem          : Avalon-MM Slave (16-bit, use waitrequest = avs_mem_waitrequest)
//   5. avs_ctrl         : Avalon-MM Slave (16-bit, Read Latency = 1, Write Latency = 0)
//   6. avm              : Avalon-MM Master (16-bit, Address Units = Symbols/Bytes)
//   7. interrupt_sender : Interrupt Sender (irq, Associated Addressable Point = avs_csr)
//   8. sram_conduit     : Conduit End (sram_addr, sram_dq, sram_ce_n, oe_n, we_n, ub_n, lb_n)
// ============================================================================
module fast_ip_wrapper #(
    parameter integer OUT_CNT_W     = 12,   // Do rong bo dem so keypoint (max 300)
    parameter integer DRAIN_CYC     = 16,   // So chu ky xa pipeline FAST/score
    parameter integer DMA_BURST_LEN = 1     // 1 cho Qsys SDR SDRAM controller
)(
    input  wire         clk,
    input  wire         rst_n,

    // 1. Avalon-MM Slave: Thanh ghi CSR dieu khien & trang thai (32-bit)
    input  wire [2:0]   avs_csr_address,
    input  wire         avs_csr_write,
    input  wire [31:0]  avs_csr_writedata,
    input  wire [3:0]   avs_csr_byteenable,
    input  wire         avs_csr_read,
    output reg  [31:0]  avs_csr_readdata,
    output wire         irq,

    // 2. Avalon-MM Slave: Cua so nap anh 640x480 (16-bit)
    input  wire [17:0]  avs_mem_address,
    input  wire         avs_mem_write,
    input  wire [15:0]  avs_mem_writedata,
    input  wire [1:0]   avs_mem_byteenable,
    output wire         avs_mem_waitrequest,

    // 3. Avalon-MM Slave: Thanh ghi dieu khien nap anh (16-bit)
    input  wire [1:0]   avs_ctrl_address,
    input  wire         avs_ctrl_write,
    input  wire [15:0]  avs_ctrl_writedata,
    input  wire         avs_ctrl_read,
    output wire [15:0]  avs_ctrl_readdata,

    // 4. Avalon-MM Master: DMA ghi ket qua keypoint ra SDRAM (16-bit)
    output wire [31:0]  avm_address,
    output wire         avm_write,
    output wire [15:0]  avm_writedata,
    output wire [1:0]   avm_byteenable,
    output wire [7:0]   avm_burstcount,
    input  wire         avm_waitrequest,

    // 5. Conduit: Chan vat ly SRAM ngoai (Kit DE2)
    output wire [17:0]  sram_addr,
    inout  wire [15:0]  sram_dq,
    output wire         sram_ce_n,
    output wire         sram_oe_n,
    output wire         sram_we_n,
    output wire         sram_ub_n,
    output wire         sram_lb_n
);

    localparam [31:0] IP_SIGNATURE = 32'hFA57_2026;

    // ========================================================================
    // Thanh ghi cau hinh & trang thai CSR
    // ========================================================================
    reg         start_cmd_d;
    reg         fast_start_pulse;
    reg         irq_en;
    reg  [31:0] dma_base_addr_reg;
    reg  [7:0]  fast_thresh_reg;

    reg         done_sticky;
    reg         err_sticky;
    reg         pkt_ovf_sticky;

    wire                 image_loaded;
    wire                 load_mode;
    wire                 fast_busy;
    wire                 fast_done;
    wire                 fast_error;
    wire                 packer_overflow;
    wire [OUT_CNT_W-1:0] out_count;
    wire [31:0]          words_written;

    // Giai ma lenh ghi vao CSR[0] (CTRL)
    wire csr_wr_ctrl  = avs_csr_write && (avs_csr_address == 3'd0) && avs_csr_byteenable[0];
    wire start_cmd    = csr_wr_ctrl && avs_csr_writedata[0];
    wire clear_sticky = csr_wr_ctrl && avs_csr_writedata[2];

    // Bat suon len (Rising-Edge) tao xung khoi dong chuan 1 chu ky clock
    wire start_edge   = start_cmd && !start_cmd_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start_cmd_d       <= 1'b0;
            fast_start_pulse  <= 1'b0;
            irq_en            <= 1'b0;
            dma_base_addr_reg <= 32'h0000_0000;
            fast_thresh_reg   <= 8'd20;
        end else begin
            start_cmd_d      <= start_cmd;
            fast_start_pulse <= start_edge;

            if (avs_csr_write) begin
                case (avs_csr_address)
                    3'd0: begin // CTRL: bit[0]=START, bit[1]=IRQ_EN, bit[2]=CLR_STICKY
                        if (avs_csr_byteenable[0])
                            irq_en <= avs_csr_writedata[1];
                    end
                    3'd2: begin // DMA_BASE_ADDR (Byte address tren SDRAM)
                        if (avs_csr_byteenable[0]) dma_base_addr_reg[7:0]   <= avs_csr_writedata[7:0];
                        if (avs_csr_byteenable[1]) dma_base_addr_reg[15:8]  <= avs_csr_writedata[15:8];
                        if (avs_csr_byteenable[2]) dma_base_addr_reg[23:16] <= avs_csr_writedata[23:16];
                        if (avs_csr_byteenable[3]) dma_base_addr_reg[31:24] <= avs_csr_writedata[31:24];
                    end
                    3'd3: begin // THRESHOLD (Nguong FAST T: 0..255)
                        if (avs_csr_byteenable[0])
                            fast_thresh_reg <= avs_csr_writedata[7:0];
                    end
                    default: ;
                endcase
            end
        end
    end

    // ========================================================================
    // Chot co trang thai Sticky & Tin hieu ngat IRQ
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done_sticky    <= 1'b0;
            err_sticky     <= 1'b0;
            pkt_ovf_sticky <= 1'b0;
        end else if (start_edge || clear_sticky) begin
            done_sticky    <= 1'b0;
            err_sticky     <= 1'b0;
            pkt_ovf_sticky <= 1'b0;
        end else begin
            if (fast_done)       done_sticky    <= 1'b1;
            if (fast_error)      err_sticky     <= 1'b1;
            if (packer_overflow) pkt_ovf_sticky <= 1'b1;
        end
    end

    assign irq = irq_en && (done_sticky || err_sticky);

    // ========================================================================
    // Doc thanh ghi CSR (Read Latency = 1 chu ky clock)
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            avs_csr_readdata <= 32'd0;
        end else if (avs_csr_read) begin
            case (avs_csr_address)
                3'd0: avs_csr_readdata <= {30'd0, irq_en, 1'b0};
                3'd1: avs_csr_readdata <= {
                          25'd0,
                          load_mode,
                          image_loaded,
                          pkt_ovf_sticky,
                          1'b0,
                          err_sticky,
                          done_sticky,
                          fast_busy
                      };
                3'd2: avs_csr_readdata <= dma_base_addr_reg;
                3'd3: avs_csr_readdata <= {24'd0, fast_thresh_reg};
                3'd4: avs_csr_readdata <= {{(32-OUT_CNT_W){1'b0}}, out_count};
                3'd5: avs_csr_readdata <= words_written;
                3'd6: avs_csr_readdata <= IP_SIGNATURE;
                default: avs_csr_readdata <= 32'd0;
            endcase
        end
    end

    // ========================================================================
    // Khoi loi FAST-9 Hardware Accelerator
    // ========================================================================
    fast_top #(
        .OUT_CNT_W     (OUT_CNT_W),
        .DRAIN_CYC     (DRAIN_CYC),
        .DMA_BURST_LEN (DMA_BURST_LEN)
    ) u_fast_top (
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

        .fast_start         (fast_start_pulse),
        .fast_threshold     (fast_thresh_reg),
        .dma_base_addr      (dma_base_addr_reg),
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

endmodule