// -------------------------------------------------------------------
// systolic_array_2x2.v
// 2x2 systolic array built from 4 PEs.
// - a_row0_in / a_row1_in enter from the left and are forwarded right.
// - cin and cout form a vertical accumulation path (top->bottom).
// - B values are loaded per row into PE weight registers.
// - c10 and c11 provide bottom-row partial sums used as C elements.
// -------------------------------------------------------------------
`timescale 1ns/1ps

module systolic_array_2x2 #(
    parameter DW = 4,   // data width for A/B
    parameter CW = 9    // width for partial sums
)(
    input                  clk,
    input                  rst_n,

    // A inputs from the left (one per row)
    input      [DW-1:0]    a_row0_in,
    input      [DW-1:0]    a_row1_in,

    // B matrix values for weight loading (row-major)
    input      [DW-1:0]    b_row0_col0,  // B11
    input      [DW-1:0]    b_row0_col1,  // B12
    input      [DW-1:0]    b_row1_col0,  // B21
    input      [DW-1:0]    b_row1_col1,  // B22

    // Row-wise weight load enables
    input                  load_row0,    // load weights into top row PEs
    input                  load_row1,    // load weights into bottom row PEs

    // Shared weight buffer control
    input                  sel_w_load,   // which buffer to LOAD (0: w0, 1: w1)
    input                  sel_w_active, // which buffer to USE for MAC (0: w0, 1: w1)

    // MAC control
    input                  clear_psum,   // clear all couts
    input                  compute_en,   // enable MAC in all PEs

    // Expose all couts (top and bottom row)
    output     [CW-1:0]    c00,  // top-left  PE cout
    output     [CW-1:0]    c01,  // top-right PE cout
    output     [CW-1:0]    c10,  // bottom-left  PE cout
    output     [CW-1:0]    c11   // bottom-right PE cout
);

    // Internal A forwarding wires between PEs
    wire [DW-1:0] a00_out;  // from PE00 to PE01
    wire [DW-1:0] a10_out;  // from PE10 to PE11

    // B-forwarding wires are not used externally in this 2x2 version
    wire [DW-1:0] b00_out_unused;
    wire [DW-1:0] b01_out_unused;
    wire [DW-1:0] b10_out_unused;
    wire [DW-1:0] b11_out_unused;

    // ----------------------------------------------------------------
    // PE00: top-left
    // ----------------------------------------------------------------
    pe #(
        .DW(DW),
        .CW(CW)
    ) u_pe00 (
        .clk           (clk),
        .rst_n         (rst_n),

        .a_in          (a_row0_in),
        .b_in          (b_row0_col0),
        .cin           ({CW{1'b0}}),   // top row: initial partial sum = 0

        .a_out         (a00_out),
        .b_out         (b00_out_unused),
        .cout          (c00),

        .load_w        (load_row0),
        .sel_w_load    (sel_w_load),
        .sel_w_active  (sel_w_active),

        .clear_psum    (clear_psum),
        .compute_en    (compute_en)
    );

    // ----------------------------------------------------------------
    // PE01: top-right
    // ----------------------------------------------------------------
    pe #(
        .DW(DW),
        .CW(CW)
    ) u_pe01 (
        .clk           (clk),
        .rst_n         (rst_n),

        .a_in          (a00_out),        // from PE00
        .b_in          (b_row0_col1),
        .cin           ({CW{1'b0}}),     // top row: initial partial sum = 0

        .a_out         (/* unused */),
        .b_out         (b01_out_unused),
        .cout          (c01),

        .load_w        (load_row0),
        .sel_w_load    (sel_w_load),
        .sel_w_active  (sel_w_active),

        .clear_psum    (clear_psum),
        .compute_en    (compute_en)
    );

    // ----------------------------------------------------------------
    // PE10: bottom-left
    // ----------------------------------------------------------------
    pe #(
        .DW(DW),
        .CW(CW)
    ) u_pe10 (
        .clk           (clk),
        .rst_n         (rst_n),

        .a_in          (a_row1_in),
        .b_in          (b_row1_col0),
        .cin           (c00),            // accumulate with top-left PE output

        .a_out         (a10_out),
        .b_out         (b10_out_unused),
        .cout          (c10),

        .load_w        (load_row1),
        .sel_w_load    (sel_w_load),
        .sel_w_active  (sel_w_active),

        .clear_psum    (clear_psum),
        .compute_en    (compute_en)
    );

    // ----------------------------------------------------------------
    // PE11: bottom-right
    // ----------------------------------------------------------------
    pe #(
        .DW(DW),
        .CW(CW)
    ) u_pe11 (
        .clk           (clk),
        .rst_n         (rst_n),

        .a_in          (a10_out),        // from PE10
        .b_in          (b_row1_col1),
        .cin           (c01),            // accumulate with top-right PE output

        .a_out         (/* unused */),
        .b_out         (b11_out_unused),
        .cout          (c11),

        .load_w        (load_row1),
        .sel_w_load    (sel_w_load),
        .sel_w_active  (sel_w_active),

        .clear_psum    (clear_psum),
        .compute_en    (compute_en)
    );

endmodule