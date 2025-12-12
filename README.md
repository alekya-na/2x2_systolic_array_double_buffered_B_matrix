# 2x2_systolic_array_double_buffered_B_matrix
2×2 systolic array in Verilog with double-buffered weights, including top-level controller, reusable PE, and a testbench that demonstrates the throughput benefit of background B loading.

# 2×2 Systolic Array with Double-Buffered Weights (Verilog)

This repo contains a 2×2 systolic array implementation in Verilog, built around a reusable processing element (PE) and a small top-level controller. The design demonstrates how double-buffered weight storage can hide B-matrix load latency and improve steady-state throughput for back-to-back 2×2 matrix multiplies.

---

## Overview

### Architecture

- **Processing Element (`pe`)**
  - Performs a multiply–accumulate: `cout = cin (+ a_in * w_active)` when enabled.
  - Forwards data systolically:
    - `a_in` → forwarded to the right as `a_out`
    - `cin`  → forwarded downward as `cout`
  - Contains two internal weight registers:
    - `w0` and `w1` (double-buffered B storage)
  - Control signals:
    - `load_w`, `sel_w_load` – choose which weight bank to load from `b_in`
    - `sel_w_active` – choose which bank to use for MAC (w0 or w1)
    - `clear_psum`, `compute_en` – manage local accumulator reset and MAC enable

- **2×2 Systolic Array (`systolic_array_2x2`)**
  - Instantiates 4 PEs in a 2×2 grid.
  - Activations enter from the left as `a_row0_in` / `a_row1_in` and are forwarded across each row.
  - Partial sums flow from the top row to the bottom row via `cin`/`cout`.
  - B values are configured **per row**:
    - Top row: `b_row0_col0`, `b_row0_col1`
    - Bottom row: `b_row1_col0`, `b_row1_col1`
  - Row-wise enables `load_row0` / `load_row1` and `sel_w_load` load B into the selected weight bank (w0 or w1) in each PE.

- **Top-Level Controller (`systolic`)**
  - Matches the lab-style interface:
    - Inputs: 2×2 A and B (`a11..a22`, `b11..b22`), plus `in_valid`
    - Outputs: 2×2 C (`c11..c22`), plus `out_valid`
  - Maintains up to **two jobs in flight**:
    - `job0` → mapped to weight bank w0
    - `job1` → mapped to weight bank w1
  - Operation:
    - On the first `in_valid`:
      - Latches A0, B0 into job0 registers.
      - Spends 2 cycles loading B0 into w0 (top row then bottom row).
    - During compute for job0:
      - If a second job is queued, latches A1, B1 into job1 and preloads B1 into w1 in the background (again top row then bottom row), while `compute_en` remains high for job0.
    - When job0 completes:
      - Immediately switches `sel_w_active` from w0 to w1 and starts computing job1 without a separate B-load phase.
  - Uses a fixed internal 6-step schedule to:
    - Drive A into the array.
    - Capture final C values from the bottom row (`c10`, `c11`) at known steps and assert `out_valid` when all four outputs are ready.

### Key assumptions

- Matrix size is fixed at **2×2**; the A streaming schedule and C capture timing are hard-coded for this size.
- A and B are **4-bit unsigned**; C is **9-bit unsigned** (sufficient for worst-case 2×2 products without extra saturation logic).
- At most **two jobs** are stored at once (job0 → w0, job1 → w1). The design follows the lab’s simple `in_valid` / `out_valid` interface and assumes the source only asserts `in_valid` when at least one job slot is free.
- B is loaded **per row** directly into each PE’s local weight registers. `b_out` is still exposed from the PE so the same element can be reused if the array is extended to a deeper or more fully streaming configuration.

---

## Files

- `systolic.v`  
  Top-level controller: job buffering, B loading, A streaming schedule, result capture, and double-buffer control.

- `systolic_array_2x2.v`  
  2×2 systolic array built from four PEs. Handles A/partial-sum wiring and row-wise B loading.

- `pe.v`  
  Processing element with:
  - double-buffered weight registers (`w0`, `w1`),
  - systolic forwarding of `a_in` / `b_in`,
  - MAC and partial sum control (`clear_psum`, `compute_en`).

- `tb_systolic.v`  
  Testbench that:
  - applies three 2×2 matrix multiplies,
  - covers both overlapped (double-buffered) and non-overlapped cases,
  - checks C against golden results,
  - measures the cycle gap between `out_valid` pulses to show throughput improvement when B is preloaded in the background.

---

## How to run

You can simulate this with any Verilog simulator (e.g. Icarus Verilog, VCS, Questa, etc.).
