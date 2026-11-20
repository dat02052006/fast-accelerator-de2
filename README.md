# FAST Corner Detection Hardware Accelerator (`fast-accelerator-de2`)

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Altera%20DE2%20(Cyclone%20II)-red.svg)]()
[![HDL](https://img.shields.io/badge/HDL-Verilog-green.svg)]()
[![Verification](https://img.shields.io/badge/Verification-Bit--Exact%20OpenCV-orange.svg)]()

A System-on-Chip (SoC) hardware accelerator IP core for **FAST (Features from Accelerated Segment Test)** corner detection, featuring **border handling**, **sparse Non-Maximum Suppression (NMS)**, and **hardware keypoint filtering**. Designed in Verilog HDL and integrated into a Nios II SoC architecture targeting the **Altera DE2 (Cyclone II EP2C35F672C6)** FPGA development board.

---

## Key Features

* **Frame-Buffered SoC Architecture:** Reads grayscale image frames stored in external **SRAM** in raster-scan order, processes pixels through a deterministic pipelined datapath, and writes compacted keypoint packets back to memory for Nios II software consumption.
* **Pipelined FAST-9/12 Corner Detector:**
  * Multi-line buffer and $7 \times 7$ sliding window generator supporting 1 pixel/cycle throughput.
  * Configurable **Border Handling** (border padding / pipeline flushing) to preserve full frame dimensions without coordinate offset errors.
  * Parallel **Corner Score (SAD)** calculation across the 16-pixel Bresenham circle.
* **Sparse Keypoint-Based NMS (Event-Driven Windowing):**
  * Eliminates the need for a dense $3 \times 3$ score line buffer across the entire frame.
  * Processes only non-zero corner candidates `(x, y, score)`, drastically reducing on-chip BRAM usage and dynamic switching power while maintaining **bit-exact** raster-scan tie-breaking behavior against OpenCV.
* **Hardware Keypoint Filtering & Stream Compaction:**
  * Filters suppressed corners and packs valid keypoints into compact 32-bit/64-bit `(x, y, score)` records, cutting memory write bandwidth by >99% compared to dense feature maps.
* **Bit-Exact Verification:** Verified cycle-by-cycle against a Python/OpenCV golden reference model on standard VGA (`640x480`) test frames.

---

## System Architecture

```text
+-----------------------------------------------------------------------------------+
|                            Altera DE2 SoC (Qsys / Platform Designer)              |
|                                                                                   |
|  +--------------+       Avalon-MM        +-------------------------------------+  |
|  |   Nios II    | <====================> |        FAST Accelerator IP          |  |
|  |  Processor   |   (Control / Status)   |                                     |  |
|  +--------------+                        |  +-------------------------------+  |  |
|         ^                                |  |  Line Buffer (7x7) & Border   |  |  |
|         |                                |  |           Handling            |  |  |
|         v                                |  +---------------+---------------+  |  |
|  +--------------+   Read Image Frame     |                  v                  |  |
|  |   External   | ---------------------> |  +-------------------------------+  |  |
|  |     SRAM     |                        |  |  FAST Detector & SAD Score    |  |  |
|  |              | <--------------------- |  +---------------+---------------+  |  |
|  +--------------+   Write Sparse         |                  v (Sparse Tokens)  |  |
|                     Keypoints (x,y,score)|  +-------------------------------+  |  |
|                                          |  |  Sparse NMS & Keypoint Filter |  |  |
|                                          |  +-------------------------------+  |  |
|                                          +-------------------------------------+  |
+-----------------------------------------------------------------------------------+
```

### Processing Pipeline Stages
1. **SRAM Memory Reader:** Fetches 8-bit grayscale pixels sequentially from the frame buffer in SRAM and generates frame synchronization flags (`sop`, `eop`, `valid`).
2. **Line Buffer & Border Handler:** Constructs a $7 \times 7$ sliding window and handles image boundaries (`R = 3`) so edge pixels are evaluated deterministically.
3. **FAST Corner & Score Unit:** Evaluates the 16-pixel Bresenham ring against the center pixel $I_p \pm t$ and computes the Sum of Absolute Differences (SAD) score in a pipelined adder tree.
4. **Sparse NMS Engine:** Buffers active corner candidates and suppresses non-maximal responses within a $3 \times 3$ spatial neighborhood ($|\Delta x| \le 1, |\Delta y| \le 1$).
5. **Keypoint Filter & Packetizer:** Filters out suppressed points and writes valid `(x, y, score)` packets to the output buffer.

---

## Repository Structure

```text
fast-accelerator-de2/
├── rtl/                        # Synthesizable Verilog HDL source files
│   ├── fast_top.v              # Top-level IP wrapper & Avalon-MM interface
│   ├── line_buffer_7x7.v       # 2D sliding window & border handling logic
│   ├── fast_core.v             # Bresenham circle comparator & SAD score tree
│   ├── sparse_nms.v            # Sparse candidate buffer & 3x3 NMS logic
│   └── keypoint_filter.v       # Stream compaction & packet formatting
├── tb/                         # SystemVerilog / Verilog testbenches
│   └── tb_fast_top.v           # Full-frame RTL verification testbench
├── sw/                         # Golden reference model & Nios II C drivers
│   ├── golden_model.py         # Bit-exact Python/OpenCV reference generator
│   └── nios_app/               # Embedded C firmware for Altera DE2
├── quartus/                    # Quartus II project, Qsys system & pin assignments
├── LICENSE                     # MIT License
└── README.md
```

---

## Getting Started

### Prerequisites
* **Hardware:** Altera DE2 Development and Education Board (Cyclone II `EP2C35F672C6`)
* **EDA Tools:**
  * Intel/Altera Quartus II (v13.0sp1 for Cyclone II support) & Qsys
  * ModelSim-Altera for RTL simulation
  * Nios II Software Build Tools (SBT) for Eclipse
* **Software Verification:** Python 3.x with `numpy` and `opencv-python`

### Simulation & Golden Model Verification
1. Generate test image hex vectors and expected golden keypoints:
   ```bash
   python3 sw/golden_model.py --image test_640x480.png --threshold 20 --out_dir sim/
   ```
2. Run RTL simulation in ModelSim:
   ```tcl
   vlib work
   vlog rtl/*.v tb/tb_fast_top.v
   vsim -c work.tb_fast_top -do "run -all; quit"
   ```
3. Compare the RTL output keypoint list against `sim/expected_keypoints.txt` to verify bit-exactness.

### Hardware Synthesis (Quartus II)
1. Open the project file in `quartus/` using Quartus II.
2. Regenerate the Qsys SoC system if any Avalon-MM parameters are modified.
3. Run **Start Compilation** to generate the `.sof` bitstream.
4. Program the DE2 board via USB-Blaster and run the Nios II firmware in `sw/nios_app/`.

---

## Register Map (Avalon-MM Slave)

| Offset | Name | Access | Description |
| :--- | :--- | :--- | :--- |
| `0x00` | `CTRL` | R/W | `[0]`: Start accelerator, `[1]`: Enable NMS, `[2]`: Soft reset |
| `0x04` | `STATUS` | RO | `[0]`: Busy, `[1]`: Done (Frame processing complete) |
| `0x08` | `IMG_DIM` | R/W | `[15:0]`: Image Width (`640`), `[31:16]`: Image Height (`480`) |
| `0x0C` | `THRESHOLD` | R/W | `[7:0]`: FAST intensity threshold $t$ |
| `0x10` | `SRC_ADDR` | R/W | Base address of input grayscale frame in SRAM |
| `0x14` | `DST_ADDR` | R/W | Base address for output keypoint list in SRAM |
| `0x18` | `KP_COUNT` | RO | Total number of valid keypoints extracted in the last frame |

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.