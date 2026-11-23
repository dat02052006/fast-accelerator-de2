// ============================================================================
// fast_result_packer.v
// Dong goi 1 keypoint (x, y, score) thanh 3 word 16-bit day vao Result FIFO:
//   - Word 0: {6'd0, out_x[9:0]}
//   - Word 1: {7'd0, out_y[8:0]}
//   - Word 2: {8'd0, out_score[7:0]}
// Xuat tin hieu busy ve keypoint_filter de bat tay chong tran (0% mat diem).
// ============================================================================
module fast_result_packer (
    input  wire         clk,
    input  wire         rst,            // sync, active-high
    input  wire         start,          // xung 1 chu ky xoa co

    // Giao tiep tu keypoint_filter
    input  wire         filter_done,    // muc 1 hoac xung 1 chu ky khi filter xa xong
    input  wire         out_we,
    input  wire [9:0]   out_x,
    input  wire [8:0]   out_y,
    input  wire [7:0]   out_score,
    output wire         busy,           // = 1 khi dang day 3 word vao FIFO

    // Giao tiep toi Result FIFO (scfifo 16-bit)
    input  wire         fifo_full,
    output wire [15:0]  fifo_wdata,
    output wire         fifo_wr,

    // Giao tiep toi orb_dma_write
    output wire         src_done,
    output wire         packer_overflow
);

    reg [47:0] shift_reg;
    reg [1:0]  word_cnt;        // dem tu 3 ve 0
    reg        done_latched;
    reg        overflow_reg;

    assign busy = (word_cnt != 2'd0);

    // Chot trang thai filter_done
    always @(posedge clk) begin
        if (rst || start) begin
            done_latched <= 1'b0;
        end else if (filter_done) begin
            done_latched <= 1'b1;
        end
    end

    // Thanh ghi dich 3 word 16-bit
    always @(posedge clk) begin
        if (rst || start) begin
            shift_reg    <= 48'd0;
            word_cnt     <= 2'd0;
            overflow_reg <= 1'b0;
        end else begin
            if (out_we) begin
                if (busy) begin
                    overflow_reg <= 1'b1;
                end else begin
                    shift_reg <= {
                        8'd0, out_score[7:0],   // Word 2
                        7'd0, out_y[8:0],       // Word 1
                        6'd0, out_x[9:0]        // Word 0
                    };
                    word_cnt <= 2'd3;
                end
            end else if (busy && !fifo_full) begin
                shift_reg <= {16'd0, shift_reg[47:16]};
                word_cnt  <= word_cnt - 2'd1;
            end
        end
    end

    assign fifo_wdata      = shift_reg[15:0];
    assign fifo_wr         = busy && !fifo_full;
    assign packer_overflow = overflow_reg;
    assign src_done        = done_latched && !busy && !out_we;

endmodule
