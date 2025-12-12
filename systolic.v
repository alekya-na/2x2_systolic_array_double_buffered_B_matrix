`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// Top-level 2x2 systolic array controller.
// - Accepts 2x2 A,B matrices on a single in_valid pulse.
// - Supports up to two jobs (job0, job1) in flight using double-buffered weights
//   inside the PEs (w0 for job0, w1 for job1).
// - Preloads B for job0 into w0, then overlaps B loading for job1 into w1
//   while job0 is computing.
// - Drives a fixed streaming schedule for A to the 2x2 systolic array.
// - Asserts out_valid when a complete 2x2 C matrix is available on c11..c22.
// -----------------------------------------------------------------------------
module systolic (
    input        clk,
    input        rst_n,

    input  [3:0] a11, a12, a21, a22,
    input  [3:0] b11, b12, b21, b22,
    input        in_valid,

    output reg [8:0] c11, c12, c21, c22,
    output reg       out_valid
);

    // ------------------------------------------------------------
    // Job 0 and Job 1 storage
    // ------------------------------------------------------------
    // Job 0 (uses internal weight buffer w0)
    reg [3:0] A11_0, A12_0, A21_0, A22_0;  // latched A matrix for job0
    reg [3:0] B11_0, B12_0, B21_0, B22_0;  // latched B matrix for job0
    reg       job0_valid;                  // job0 slot in use
    reg       job0_B_loaded;               // job0 B fully loaded into w0 in all PEs

    // Job 1 (uses internal weight buffer w1)
    reg [3:0] A11_1, A12_1, A21_1, A22_1;  // latched A matrix for job1
    reg [3:0] B11_1, B12_1, B21_1, B22_1;  // latched B matrix for job1
    reg       job1_valid;                  // job1 slot in use
    reg       job1_B_loaded;               // job1 B fully loaded into w1 in all PEs
    reg [1:0] job1_load_cnt;               // job1 B load progress: 0=none, 1=row0, 2=row1

    // Current job / buffer selection
    reg       active_job;                  // 0 -> job0 is active, 1 -> job1 is active
    reg       active_buf;                  // 0 -> use w0, 1 -> use w1 (maps to sel_w_active)

    // Step counter for compute schedule (0..5)
    // 0..4 : drive A streaming pattern; 5 : final capture of C
    reg [2:0] step;

    // ------------------------------------------------------------
    // Wires to systolic_array_2x2
    // ------------------------------------------------------------
    // Internal C outputs from the 2x2 array (PE couts)
    wire [8:0] c00_int, c01_int, c10_int, c11_int;

    // A inputs to array (one per row, driven by controller)
    reg  [3:0] a_row0_in;
    reg  [3:0] a_row1_in;

    // B inputs used when loading weight registers in PEs
    reg  [3:0] b_row0_col0;
    reg  [3:0] b_row0_col1;
    reg  [3:0] b_row1_col0;
    reg  [3:0] b_row1_col1;

    // Control signals into array / PEs
    reg        load_row0;      // enable loading weights in top row PEs
    reg        load_row1;      // enable loading weights in bottom row PEs
    reg        sel_w_load;     // select weight buffer to load (0=w0, 1=w1)
    reg        sel_w_active;   // select weight buffer to use for MAC (0=w0, 1=w1)
    reg        clear_psum;     // clear partial sums in all PEs
    reg        compute_en;     // enable MAC operation in all PEs

    // 2x2 systolic array instance
    systolic_array_2x2 #(
        .DW(4),
        .CW(9)
    ) u_array (
        .clk          (clk),
        .rst_n        (rst_n),

        .a_row0_in    (a_row0_in),
        .a_row1_in    (a_row1_in),

        .b_row0_col0  (b_row0_col0),
        .b_row0_col1  (b_row0_col1),
        .b_row1_col0  (b_row1_col0),
        .b_row1_col1  (b_row1_col1),

        .load_row0    (load_row0),
        .load_row1    (load_row1),

        .sel_w_load   (sel_w_load),
        .sel_w_active (sel_w_active),

        .clear_psum   (clear_psum),
        .compute_en   (compute_en),

        .c00          (c00_int),
        .c01          (c01_int),
        .c10          (c10_int),
        .c11          (c11_int)
    );

    // ------------------------------------------------------------
    // FSM states for top-level controller
    // ------------------------------------------------------------
    localparam S_IDLE        = 2'd0;  // wait for first job
    localparam S_JOB0_B_ROW0 = 2'd1;  // load top row of B for job0 into w0
    localparam S_JOB0_B_ROW1 = 2'd2;  // load bottom row of B for job0 into w0
    localparam S_COMP        = 2'd3;  // compute for active job (job0 or job1)

    reg [1:0] state;

    // ------------------------------------------------------------
    // Combinational control
    // ------------------------------------------------------------
    // Drives array controls and A/B inputs based on current state, step,
    // active job, and job1 B load status. No state is updated here.
    always @(*) begin
        // Default values for all driven signals
        a_row0_in   = 4'd0;
        a_row1_in   = 4'd0;

        b_row0_col0 = 4'd0;
        b_row0_col1 = 4'd0;
        b_row1_col0 = 4'd0;
        b_row1_col1 = 4'd0;

        load_row0   = 1'b0;
        load_row1   = 1'b0;

        sel_w_load   = 1'b0;       // default: load w0
        sel_w_active = active_buf; // select buffer used for MAC based on active job

        clear_psum   = 1'b0;
        compute_en   = 1'b0;

        case (state)
            // ----------------------------------------------------
            S_IDLE: begin
                // No activity; all controls remain at defaults.
            end

            // ----------------------------------------------------
            // Load Job0 B into w0 (top row then bottom row)
            // ----------------------------------------------------
            S_JOB0_B_ROW0: begin
                sel_w_load   = 1'b0;   // target buffer: w0
                load_row0    = 1'b1;   // enable weight load in top row
                b_row0_col0  = B11_0;
                b_row0_col1  = B12_0;
                clear_psum   = 1'b1;   // clear pipeline before job0 compute
            end

            S_JOB0_B_ROW1: begin
                sel_w_load   = 1'b0;   // target buffer: w0
                load_row1    = 1'b1;   // enable weight load in bottom row
                b_row1_col0  = B21_0;
                b_row1_col1  = B22_0;
                clear_psum   = 1'b1;
            end

            // ----------------------------------------------------
            // Compute phase for active_job (job0 or job1)
            // ----------------------------------------------------
            S_COMP: begin
                compute_en   = 1'b1;
                clear_psum   = 1'b0;   // partial sums used across compute steps

                // ---- A streaming schedule for the active job ----
                // step = 0..4 corresponds to different A-row inputs.
                // Implementation chooses a specific 5-cycle pattern
                // that yields correct 2x2 matrix multiplication.

                if (!active_job) begin
                    // job0 A streaming
                    case (step)
                        3'd0: begin
                            a_row0_in = A11_0;
                            a_row1_in = 4'd0;
                        end
                        3'd1: begin
                            a_row0_in = A11_0;
                            a_row1_in = A22_0;
                        end
                        3'd2: begin
                            a_row0_in = A21_0;
                            a_row1_in = A12_0;
                        end
                        3'd3: begin
                            a_row0_in = A21_0;
                            a_row1_in = A22_0;
                        end
                        3'd4: begin
                            a_row0_in = 4'd0;
                            a_row1_in = 4'd0;
                        end
                        default: begin
                            a_row0_in = 4'd0;
                            a_row1_in = 4'd0;
                        end
                    endcase
                end else begin
                    // job1 A streaming
                    case (step)
                        3'd0: begin
                            a_row0_in = A11_1;
                            a_row1_in = 4'd0;
                        end
                        3'd1: begin
                            a_row0_in = A11_1;
                            a_row1_in = A22_1;
                        end
                        3'd2: begin
                            a_row0_in = A21_1;
                            a_row1_in = A12_1;
                        end
                        3'd3: begin
                            a_row0_in = A21_1;
                            a_row1_in = A22_1;
                        end
                        3'd4: begin
                            a_row0_in = 4'd0;
                            a_row1_in = 4'd0;
                        end
                        default: begin
                            a_row0_in = 4'd0;
                            a_row1_in = 4'd0;
                        end
                    endcase
                end

                // ---- Background B-load for Job1 into w1 ----
                // While computing job0, opportunistically preload B
                // for job1 into w1 (top row then bottom row).
                if (!active_job && job1_valid && !job1_B_loaded) begin
                    sel_w_load = 1'b1;  // target buffer: w1

                    if (job1_load_cnt == 2'd0) begin
                        // Load top row of job1 B into w1
                        load_row0    = 1'b1;
                        b_row0_col0  = B11_1;
                        b_row0_col1  = B12_1;
                    end
                    else if (job1_load_cnt == 2'd1) begin
                        // Load bottom row of job1 B into w1
                        load_row1    = 1'b1;
                        b_row1_col0  = B21_1;
                        b_row1_col1  = B22_1;
                    end
                end
            end

            default: begin
                // All signals remain at defaults.
            end
        endcase
    end

    // ------------------------------------------------------------
    // Sequential: state, job acceptance, step, B-load progress, C capture
    // ------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Global reset of controller and outputs
            state         <= S_IDLE;
            step          <= 3'd0;

            job0_valid    <= 1'b0;
            job0_B_loaded <= 1'b0;
            job1_valid    <= 1'b0;
            job1_B_loaded <= 1'b0;
            job1_load_cnt <= 2'd0;

            active_job    <= 1'b0;
            active_buf    <= 1'b0;

            A11_0 <= 4'd0; A12_0 <= 4'd0; A21_0 <= 4'd0; A22_0 <= 4'd0;
            B11_0 <= 4'd0; B12_0 <= 4'd0; B21_0 <= 4'd0; B22_0 <= 4'd0;

            A11_1 <= 4'd0; A12_1 <= 4'd0; A21_1 <= 4'd0; A22_1 <= 4'd0;
            B11_1 <= 4'd0; B12_1 <= 4'd0; B21_1 <= 4'd0; B22_1 <= 4'd0;

            c11         <= 9'd0;
            c12         <= 9'd0;
            c21         <= 9'd0;
            c22         <= 9'd0;
            out_valid   <= 1'b0;

        end else begin
            out_valid <= 1'b0; // default low; pulsed when a job completes

            // ---------------------------
            // Accept new jobs on in_valid
            // ---------------------------
            if (in_valid) begin
                if (!job0_valid && state == S_IDLE) begin
                    // First job: allocate job0 slot and latch A,B
                    A11_0 <= a11;
                    A12_0 <= a12;
                    A21_0 <= a21;
                    A22_0 <= a22;

                    B11_0 <= b11;
                    B12_0 <= b12;
                    B21_0 <= b21;
                    B22_0 <= b22;

                    job0_valid    <= 1'b1;
                    job0_B_loaded <= 1'b0;

                    // Start loading job0 B into w0
                    state      <= S_JOB0_B_ROW0;
                    active_job <= 1'b0;
                    active_buf <= 1'b0;
                    step       <= 3'd0;

                end else if (job0_valid && !job1_valid) begin
                    // Second job: allocate job1 slot and latch A,B
                    A11_1 <= a11;
                    A12_1 <= a12;
                    A21_1 <= a21;
                    A22_1 <= a22;

                    B11_1 <= b11;
                    B12_1 <= b12;
                    B21_1 <= b21;
                    B22_1 <= b22;

                    job1_valid    <= 1'b1;
                    job1_B_loaded <= 1'b0;
                    job1_load_cnt <= 2'd0;
                end
                // If both jobs are occupied, additional in_valid pulses are ignored.
            end

            // ---------------------------
            // FSM progression
            // ---------------------------
            case (state)
                S_IDLE: begin
                    // Remain idle until a new job0 is accepted.
                end

                S_JOB0_B_ROW0: begin
                    // After loading top row of job0 B, proceed to bottom row
                    state <= S_JOB0_B_ROW1;
                end

                S_JOB0_B_ROW1: begin
                    // Finished loading B for job0 into w0
                    job0_B_loaded <= 1'b1;
                    state         <= S_COMP;
                    active_job    <= 1'b0;
                    active_buf    <= 1'b0;  // use w0 for job0
                    step          <= 3'd0;
                end

                S_COMP: begin
                    // Compute phase for active job

                    // Step increments every cycle within a compute burst
                    if (step < 3'd5)
                        step <= step + 3'd1;
                    else
                        step <= 3'd0; // reset; will be reinitialized when starting next job

                    // --------- Background job1 B-load progress ----------
                    // Track which rows of job1 B have been loaded into w1.
                    if (!active_job && job1_valid && !job1_B_loaded) begin
                        if (job1_load_cnt == 2'd0) begin
                            job1_load_cnt <= 2'd1;   // just loaded row0
                        end else if (job1_load_cnt == 2'd1) begin
                            job1_load_cnt <= 2'd2;   // just loaded row1
                            job1_B_loaded <= 1'b1;
                        end
                    end

                    // --------- C capture based on step (for active job) ----------
                    // Depending on the step count, the bottom-row outputs c10_int
                    // and c11_int contain different C elements. Capture them when
                    // they are known to be final for the current job.

                    // At step==3: capture C11 from c10_int
                    if (step == 3'd3) begin
                        c11 <= c10_int;
                    end
                    // At step==4: capture C21 from c10_int and C12 from c11_int
                    if (step == 3'd4) begin
                        c21 <= c10_int;
                        c12 <= c11_int;
                    end
                    // At step==5: capture C22 from c11_int and complete the job
                    if (step == 3'd5) begin
                        c22      <= c11_int;
                        out_valid <= 1'b1;

                        // Mark current job as finished and select next job if any
                        if (!active_job) begin
                            // Finished Job0
                            job0_valid    <= 1'b0;
                            job0_B_loaded <= 1'b0;

                            if (job1_valid && job1_B_loaded) begin
                                // Immediately start computing Job1 using w1
                                active_job <= 1'b1;
                                active_buf <= 1'b1;  // w1
                                step       <= 3'd0;
                                // Remain in S_COMP for next job
                            end else if (job1_valid && !job1_B_loaded) begin
                                // Job1 exists but weights not fully loaded:
                                // return to IDLE and allow further B loading paths.
                                state         <= S_IDLE;
                            end else begin
                                // No more jobs
                                state <= S_IDLE;
                            end
                        end else begin
                            // Finished Job1
                            job1_valid    <= 1'b0;
                            job1_B_loaded <= 1'b0;
                            active_job    <= 1'b0;
                            active_buf    <= 1'b0;
                            state         <= S_IDLE;
                        end
                    end
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule