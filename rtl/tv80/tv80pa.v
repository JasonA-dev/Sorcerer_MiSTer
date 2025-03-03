`timescale 1ns / 1ps

//
// tv80s_pa - A pseudo-asynchronous Z80-like CPU top-level
//            using tv80_core internally,
//            with the same ports as tv80s.
//
// Inspired by Sorgelig's T80pa (VHDL). 
// Original T80pa used separate CEN_p/CEN_n signals. 
// Here, we only have one 'cen' input, so we emulate half-frequency 
// toggling with an internal 'cen_pol' register.
//
// This code is provided as an example or template. 
// Further tuning may be required for exact cycle timings.
//
// 2023+ [Your Name or Company]
//

module tv80pa
#(
    // If you wish, you can expose parameters for Mode, IOWait, etc.
    // For simplicity, we just fix them or keep them at defaults here.
    parameter Mode   = 0,  // 0 => Z80, 1 => Fast Z80, 2 => 8080, 3 => GB
    parameter IOWait = 1   // Standard Z80 I/O wait style
)
(
    input         reset_n,  // Active-low reset
    input         clk,      // Master clock
    input         cen,      // Clock enable (active high)
    input         wait_n,   // External WAIT signal (active high = no wait)
    input         int_n,    // Interrupt request (active low)
    input         nmi_n,    // Non-maskable interrupt (active low)
    input         busrq_n,  // Bus request (active low)
    output        busak_n,  // Bus acknowledge (active low)
    output        m1_n,     // M1 cycle indicator (active low)
    output        mreq_n,   // Memory request (active low)
    output        iorq_n,   // IO request (active low)
    output        rd_n,     // Read strobe (active low)
    output        wr_n,     // Write strobe (active low)
    output        rfsh_n,   // Refresh strobe (active low)
    output        halt_n,   // Halted status (active low)
    output [15:0] A,        // Address bus
    input  [7:0]  di,       // Data bus input
    output [7:0]  dout      // Data bus output
);

    //----------------------------------------------------------------
    // Internal signals reminiscent of T80pa logic
    //----------------------------------------------------------------
    reg           rd_reg, wr_reg, iorq_reg, mreq_reg;
    reg   [7:0]   di_reg;
    reg   [15:0]  A_last;
    reg           cen_pol;       // toggles to emulate two-phase enable
    reg   [1:0]   intcycle_dly;  // delayed intcycle
    wire          intcycle_n;
    wire          iorq;
    wire          no_read;
    wire          write;
    wire          busak;
    wire  [15:0]  A_int;
    wire  [2:0]   mcycle;
    wire  [2:0]   tstate;

    // Tie internal regs to output pins
    assign rd_n   = rd_reg;
    assign wr_n   = wr_reg;
    assign iorq_n = iorq_reg;
    assign mreq_n = mreq_reg;
    assign busak_n = busak;

    // Address bus:
    // T80pa often drives A=last or A=0 in non-bus cycles. We'll replicate 
    // the approach: if not reading and not writing, hold A_last; else drive A_int.
    assign A = (no_read == 1'b0 || write == 1'b1) ? A_int : A_last;

    //----------------------------------------------------------------
    // tv80_core instantiation
    // We pass .cen = (cen & ~cen_pol) so it only advances on half-cycles.
    //----------------------------------------------------------------
    tv80_core #(
        .Mode   (Mode),
        .IOWait (IOWait)
    ) i_tv80_core
    (
        .clk         (clk),              // System clock
        // Emulate half-frequency: the core sees enable only when cen=1 and cen_pol=0
        .cen         (cen & ~cen_pol),

        .reset_n     (reset_n),
        .wait_n      (wait_n),           // If tv80_core fully supports WAIT, pass it directly
        .int_n       (int_n),
        .nmi_n       (nmi_n),
        .busrq_n     (busrq_n),
        .busak_n     (busak),

        // Z80-like signals from the core
        .m1_n        (m1_n),
        .iorq        (iorq),
        .no_read     (no_read),
        .write       (write),
        .rfsh_n      (rfsh_n),
        .halt_n      (halt_n),
        
        // Data and address
        .A           (A_int),
        // For instruction fetch, pass di directly
        .dinst       (di),
        // For data read, we latch externally
        .di          (di_reg),
        .dout        (dout),

        // Internal cycle signals (useful if you want to observe or replicate T80pa logic)
        .mc          (mcycle),
        .ts          (tstate),
        .intcycle_n  (intcycle_n),

        // Some tv80_core outputs might not be used here
        .IntE        (),
        .stop        ()
    );

    //----------------------------------------------------------------
    // Pseudo-asynchronous process
    // Toggling approach: on each rising edge of CLK, if cen=1, flip cen_pol
    // and perform either "positive half" or "negative half" updates.
    //----------------------------------------------------------------
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            // Reset all signals
            rd_reg       <= 1'b1;
            wr_reg       <= 1'b1;
            iorq_reg     <= 1'b1;
            mreq_reg     <= 1'b1;
            di_reg       <= 8'h00;
            A_last       <= 16'h0000;
            cen_pol      <= 1'b0;
            intcycle_dly <= 2'b11;
        end else begin
            // Only do half-cycle toggling if cen=1. If cen=0 => "stall."
            if (cen) begin
                // Flip the pol
                cen_pol <= ~cen_pol;

                // If cen_pol was 0 => now rising to 1 => do "CEN_p"-like actions
                if (!cen_pol) begin
                    // Example partial logic from T80pa
                    // e.g. in M1 cycle (opcode fetch), do something in T2...
                    if (mcycle == 3'b001) begin
                        if (tstate == 3'b010) begin
                            iorq_reg <= 1'b1;
                            mreq_reg <= 1'b1;
                            rd_reg   <= 1'b1;
                        end
                    end else begin
                        // Non-M1 cycle example
                        if ((tstate == 3'b001) && (iorq == 1'b1)) begin
                            wr_reg   <= ~write;  // if write=1 => wr=0
                            rd_reg   <=  write;  // if write=0 => rd=0
                            iorq_reg <= 1'b0;
                        end
                    end
                end 
                // If cen_pol was 1 => now going to 0 => do "CEN_n"-like actions
                else begin
                    // For instance, latch DI in certain Tstates
                    if ((tstate == 3'b011) && (busak == 1'b1)) begin
                        di_reg <= di;
                    end

                    // M1 cycle logic for interrupt acknowledges
                    if (mcycle == 3'b001) begin
                        if (tstate == 3'b001) begin
                            intcycle_dly <= {intcycle_dly[0], intcycle_n};

                            rd_reg   <= ~intcycle_n; 
                            mreq_reg <= ~intcycle_n; 
                            iorq_reg <= intcycle_dly[1];
                            A_last   <= A_int;
                        end
                        if (tstate == 3'b011) begin
                            intcycle_dly <= 2'b11;
                            rd_reg   <= 1'b1;
                            mreq_reg <= 1'b0; 
                        end
                        if (tstate == 3'b100) begin
                            mreq_reg <= 1'b1;
                        end
                    end else begin
                        // Normal memory/IO cycle
                        if ((no_read == 1'b0) && (iorq == 1'b0)) begin
                            if (tstate == 3'b001) begin
                                rd_reg   <= (write) ? 1'b1 : 1'b0;
                                mreq_reg <= 1'b0;
                                A_last   <= A_int;
                            end
                        end
                        if (tstate == 3'b010) begin
                            wr_reg <= ~write;
                        end
                        if (tstate == 3'b011) begin
                            wr_reg   <= 1'b1;
                            rd_reg   <= 1'b1;
                            iorq_reg <= 1'b1;
                            mreq_reg <= 1'b1;
                        end
                    end // not M1
                end // else cen_pol
            end // if (cen)
        end // reset else
    end // always

endmodule
