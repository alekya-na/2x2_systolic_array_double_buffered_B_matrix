//-----------------------------------------------------------------------//
// pe.v                                                                   //
// Systolic Processing Element with double-buffered weights.              //
// - a_in is forwarded to the right as a_out                              //
// - b_in is forwarded downward as b_out                                  //
// - cin is forwarded downward as cout, optionally adding a_in*w_active   //
// - w0/w1 implement two weight banks for preloading and switching        //
//-----------------------------------------------------------------------//
`timescale 1ns/1ps

module pe #(
    parameter DW = 4,   // data width for A and B
    parameter CW = 9    // width for partial sums (C)
)(
    input                  clk,
    input                  rst_n,        // active-low reset

    // Data inputs
    input      [DW-1:0]    a_in,
    input      [DW-1:0]    b_in,
    input      [CW-1:0]    cin,

    // Data outputs (forwarded / updated)
    output reg [DW-1:0]    a_out,
    output reg [DW-1:0]    b_out,
    output reg [CW-1:0]    cout,

    // Weight control
    input                  load_w,       // 1: capture b_in into selected weight buffer
    input                  sel_w_load,   // 0: load w0, 1: load w1
    input                  sel_w_active, // 0: use w0, 1: use w1 for MAC

    // MAC control
    input                  clear_psum,   // 1: force cout to zero this cycle
    input                  compute_en    // 1: add a_in*w_active to cin
);

    // Two weight registers (double-buffered storage for filter weights)
    reg [DW-1:0] w0, w1;

    // Active weight and multiplier result
    wire [DW-1:0]   w_active;
    wire [2*DW-1:0] mul_res;
    reg  [CW-1:0]   mul_ext;

    // Select active weight bank for MAC
    assign w_active = sel_w_active ? w1 : w0;

    // Multiply: DW x DW -> up to 2*DW bits
    assign mul_res = a_in * w_active;

    // Zero-extend mul_res into CW bits for accumulation width
    always @(*) begin
        mul_ext = {CW{1'b0}};
        mul_ext[2*DW-1:0] = mul_res;
    end

    // Sequential behavior
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w0    <= {DW{1'b0}};
            w1    <= {DW{1'b0}};
            a_out <= {DW{1'b0}};
            b_out <= {DW{1'b0}};
            cout  <= {CW{1'b0}};
        end else begin
            // Systolic forwarding of data to neighboring PEs
            a_out <= a_in;
            b_out <= b_in;

            // Weight loading into selected buffer (w0 or w1)
            if (load_w) begin
                if (sel_w_load)
                    w1 <= b_in;
                else
                    w0 <= b_in;
            end

            // Partial sum update
            if (clear_psum) begin
                // Explicit clear of local accumulator
                cout <= {CW{1'b0}};
            end else begin
                if (compute_en)
                    // Accumulate product into incoming partial sum
                    cout <= cin + mul_ext;
                else
                    // Pass-through of incoming partial sum
                    cout <= cin;
            end
        end
    end

endmodule
