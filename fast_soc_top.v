module fast_soc_top (
    // Clock 50MHz & Reset (KEY[0])
    input  wire        CLOCK_50,
    input  wire [0:0]  KEY,

    // Giao tiếp SDRAM (IS42S16160B trên DE2)
    output wire        DRAM_CLK,
    output wire        DRAM_CKE,
    output wire        DRAM_CS_N,
    output wire        DRAM_RAS_N,
    output wire        DRAM_CAS_N,
    output wire        DRAM_WE_N,
    output wire        DRAM_BA_0,
    output wire        DRAM_BA_1,
    output wire        DRAM_LDQM,
    output wire        DRAM_UDQM,
    output wire [11:0] DRAM_ADDR,
    inout  wire [15:0] DRAM_DQ,

    // Giao tiếp SRAM (IS61LV25616AL trên DE2 - 512KB)
    output wire [17:0] SRAM_ADDR,
    inout  wire [15:0] SRAM_DQ,
    output wire        SRAM_WE_N,
    output wire        SRAM_OE_N,
    output wire        SRAM_UB_N,
    output wire        SRAM_LB_N,
    output wire        SRAM_CE_N
);

    // Dây trung gian xử lý độ rộng bus SDRAM (Qsys 32-bit -> DE2 16-bit)
    wire [1:0]  sdram_ba_w;
    wire [3:0]  sdram_dqm_w;
    wire [31:0] sdram_dq_w;

    assign {DRAM_BA_1, DRAM_BA_0} = sdram_ba_w;
    assign {DRAM_UDQM, DRAM_LDQM} = sdram_dqm_w[1:0];

    // Instance của Qsys system
    system u_system (
        // Clock & Reset
        .clk_clk       (CLOCK_50),
        .reset_reset_n (KEY[0]),

        // SDRAM
        .sdram_clk_clk (DRAM_CLK),
        .sdram_addr    (DRAM_ADDR),
        .sdram_ba      (sdram_ba_w),
        .sdram_cas_n   (DRAM_CAS_N),
        .sdram_cke     (DRAM_CKE),
        .sdram_cs_n    (DRAM_CS_N),
        .sdram_dq      ({sdram_dq_w[31:16], DRAM_DQ}),
        .sdram_dqm     (sdram_dqm_w),
        .sdram_ras_n   (DRAM_RAS_N),
        .sdram_we_n    (DRAM_WE_N),

        // SRAM (nối tới fast_accelerator_0)
        .sram_dq       (SRAM_DQ),
        .sram_ce_n     (SRAM_CE_N),
        .sram_oe_n     (SRAM_OE_N),
        .sram_we_n     (SRAM_WE_N),
        .sram_ub_n     (SRAM_UB_N),
        .sram_lb_n     (SRAM_LB_N),
        .sram_addr     (SRAM_ADDR)
    );

endmodule