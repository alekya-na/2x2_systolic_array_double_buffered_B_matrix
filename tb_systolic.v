`timescale 1ns/1ps

module tb_systolic;

    reg clk;
    reg rst_n;

    reg        in_valid;
    reg [3:0]  a11, a12, a21, a22;
    reg [3:0]  b11, b12, b21, b22;

    wire [8:0] c11, c12, c21, c22;
    wire       out_valid;

    // DUT: double-buffered systolic
    systolic dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .a11      (a11),
        .a12      (a12),
        .a21      (a21),
        .a22      (a22),
        .b11      (b11),
        .b12      (b12),
        .b21      (b21),
        .b22      (b22),
        .in_valid (in_valid),
        .c11      (c11),
        .c12      (c12),
        .c21      (c21),
        .c22      (c22),
        .out_valid(out_valid)
    );

    // Clock
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // VCD
    initial begin
        $dumpfile("tb_systolic.vcd");
        $dumpvars(0, tb_systolic);
    end

    // ----------------------------------------------------------------
    // Expected results for three jobs
    // ----------------------------------------------------------------
    localparam NUM_JOBS = 3;

    reg [8:0] exp_c11 [0:NUM_JOBS-1];
    reg [8:0] exp_c12 [0:NUM_JOBS-1];
    reg [8:0] exp_c21 [0:NUM_JOBS-1];
    reg [8:0] exp_c22 [0:NUM_JOBS-1];

    integer out_count;

    // Simple cycle counter so we can measure gaps between out_valid pulses
    integer cycle;
    integer out_cycle [0:NUM_JOBS-1];

    // Cycle counter
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cycle <= 0;
        else
            cycle <= cycle + 1;
    end

    initial begin
        // Job0: A0=[[1,2],[3,4]], B0=[[5,6],[7,8]]
        exp_c11[0] = 1*5 + 2*7;  // 19
        exp_c12[0] = 1*6 + 2*8;  // 22
        exp_c21[0] = 3*5 + 4*7;  // 43
        exp_c22[0] = 3*6 + 4*8;  // 50

        // Job1: A1=B1=all ones -> C=[[2,2],[2,2]]
        exp_c11[1] = 2;
        exp_c12[1] = 2;
        exp_c21[1] = 2;
        exp_c22[1] = 2;

        // Job2: A2 = identity, B2 = [[2,3],[4,5]] -> C2 = B2
        exp_c11[2] = 2;
        exp_c12[2] = 3;
        exp_c21[2] = 4;
        exp_c22[2] = 5;

        out_count = 0;
    end

    // ----------------------------------------------------------------
    // Stimulus
    // ----------------------------------------------------------------
    initial begin
        rst_n    = 1'b0;
        in_valid = 1'b0;
        a11 = 0; a12 = 0; a21 = 0; a22 = 0;
        b11 = 0; b12 = 0; b21 = 0; b22 = 0;

        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        // =========================================================
        // Test 0 & 1: back-to-back jobs to exercise double-buffering
        // =========================================================

        // -------------------------
        // Job 0: send first matmul
        // -------------------------
        @(posedge clk);
        a11      <= 4'd1;
        a12      <= 4'd2;
        a21      <= 4'd3;
        a22      <= 4'd4;
        b11      <= 4'd5;
        b12      <= 4'd6;
        b21      <= 4'd7;
        b22      <= 4'd8;
        in_valid <= 1'b1;

        @(posedge clk);
        in_valid <= 1'b0;
        a11 <= 0; a12 <= 0; a21 <= 0; a22 <= 0;
        b11 <= 0; b12 <= 0; b21 <= 0; b22 <= 0;

        // Wait a couple cycles, then send Job 1 while Job 0 is still running
        repeat (2) @(posedge clk);

        // -------------------------
        // Job 1: send second matmul (overlapped with Job 0 compute)
        // -------------------------
        @(posedge clk);
        a11      <= 4'd1;
        a12      <= 4'd1;
        a21      <= 4'd1;
        a22      <= 4'd1;
        b11      <= 4'd1;
        b12      <= 4'd1;
        b21      <= 4'd1;
        b22      <= 4'd1;
        in_valid <= 1'b1;

        @(posedge clk);
        in_valid <= 1'b0;
        a11 <= 0; a12 <= 0; a21 <= 0; a22 <= 0;
        b11 <= 0; b12 <= 0; b21 <= 0; b22 <= 0;

        // =========================================================
        // Test 2: sequential job (no overlap, baseline throughput)
        // Wait for both outputs from Job 0 and Job 1, then send Job 2.
        // =========================================================
        wait (out_count == 2);
        @(posedge clk);

        // -------------------------
        // Job 2: send after pipeline is idle
        // A2 = [[1,0],[0,1]], B2 = [[2,3],[4,5]]
        // -------------------------
        @(posedge clk);
        a11      <= 4'd1;
        a12      <= 4'd0;
        a21      <= 4'd0;
        a22      <= 4'd1;
        b11      <= 4'd2;
        b12      <= 4'd3;
        b21      <= 4'd4;
        b22      <= 4'd5;
        in_valid <= 1'b1;

        @(posedge clk);
        in_valid <= 1'b0;
        a11 <= 0; a12 <= 0; a21 <= 0; a22 <= 0;
        b11 <= 0; b12 <= 0; b21 <= 0; b22 <= 0;

        // Now wait for remaining outputs
        repeat (200) @(posedge clk);
        $display("ERROR: timeout waiting for outputs");
        $stop;
    end

    // ----------------------------------------------------------------
    // Monitor out_valid pulses and check results + capture throughput
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (rst_n && out_valid) begin
            out_cycle[out_count] = cycle;

            $display("-------------------------------------------------");
            $display("Output %0d at cycle %0d: C = [[%0d,%0d],[%0d,%0d]]",
                     out_count, cycle, c11, c12, c21, c22);
            $display("Expected                 [[%0d,%0d],[%0d,%0d]]",
                     exp_c11[out_count],
                     exp_c12[out_count],
                     exp_c21[out_count],
                     exp_c22[out_count]);

            if (c11 === exp_c11[out_count] &&
                c12 === exp_c12[out_count] &&
                c21 === exp_c21[out_count] &&
                c22 === exp_c22[out_count]) begin
                $display("  [PASS] output %0d", out_count);
            end else begin
                $display("  [ERROR] mismatch on output %0d", out_count);
                $stop;
            end

            out_count = out_count + 1;
            if (out_count == NUM_JOBS) begin
                integer gap01, gap12;
                gap01 = out_cycle[1] - out_cycle[0];  // Job0 -> Job1
                gap12 = out_cycle[2] - out_cycle[1];  // Job1 -> Job2

                $display("=================================================");
                $display("All functional checks PASSED.");
                $display("Job0->Job1 output gap (overlapped)   = %0d cycles", gap01);
                $display("Job1->Job2 output gap (non-overlap) = %0d cycles", gap12);

                if (gap01 < gap12)
                    $display("[PASS] Double-buffered B improves steady-state throughput.");
                else
                    $display("[WARN] Throughput not improved as expected (gap01 >= gap12).");

                $display("=================================================");
                $finish;
            end
        end
    end

endmodule
