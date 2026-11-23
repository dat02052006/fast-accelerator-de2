# ============================================================================
# fast_accelerator_de2.sdc
# Synopsys Design Constraints (SDC) cho FAST-9 Accelerator IP (50 MHz / 20 ns)
# ============================================================================

create_clock -name {clk} -period 20.000 -waveform { 0.000 10.000 } [get_ports {clk}]

derive_pll_clocks
derive_clock_uncertainty

set_false_path -from [get_ports {rst_n}] -to *
set_false_path -from * -to [get_ports {sram_*}]
set_false_path -from [get_ports {sram_*}] -to *
