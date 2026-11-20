// Copyright 2026 Franz Schoning
// Project: https://github.com/Schoning-Labs/pipe68k
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ---------------------------------------------------------------------------------------------------------------------
// p68_xseq: the exception sequencer, which writes the exception stack frame and jumps through the vector.
//
// Where it sits. In the decode stage (DS, p68_decode.v), beside the instruction sequencers. When an exception or an
// interrupt is taken, the system unit p68_sys.v in EX (the execute stage) flushes the pipeline, starts this sequencer and
// hands over the parameters, latched here. The uops (micro-ops, p68_uop.vh) it produces go through DR (register stage
// before address generation), AG (address generation), OC (operand cycle), EX and WB (write-back) like any others: an
// exception is handled by ordinary stores and one load, not by a separate state machine on the bus. The uop of each step
// is a function of step and parameters only, so a replay (the vector read misses the D-cache) or the M-clear step
// restarts it at a given step.
//
// The new SR (S set, trace off, interrupt mask) and the stack pointer swap are done by EX at the take, so A7 is
// already the supervisor stack pointer here. Frame stores are word stores (A7-6 is not long-aligned):
//   format 0: SR, PC high, PC low, format/vector word; A7 -= 8 with the last store
//   format 2: the same plus the instruction address (two words); A7 -= 12
//   interrupt with M = 1: format 0 on the MSP, then SY_MCLR (clears M, swaps to the ISP, restarts at the next step),
//   then a format 1 (throwaway) frame on the ISP with S set in its SR
//   last step: read the vector at VBR + 4 * vector and jump (SY_JMP); the instruction retires with it.
// Uops carry EXC[7:6] = XS_EXC / XS_IRQ (no exception raise, no interrupt in front of them, retire flag).
// Which format an exception uses (p68_uop.vh, EXC field): vectors 5, 6, 7 and 9 (divide by zero, CHK, TRAPcc/TRAPV,
// trace) use format 2, which also holds the address of the instruction that caused it; all others use format 0.
// The format/vector word is the format number in bits 15-12 and the vector offset (4 * vector) in bits 11-0.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`include "p68_sys.vh"
`default_nettype none
module p68_xseq (
  input  wire        clk,
  input  wire        rst,
  input  wire        flush,                  // pipeline flush (undelayed); a restart (rs) wins over it
  input  wire        go,             // EX takes an exception: latch xs_par, start at step 1
  input  wire [154:0] xs_par,        // {pcr, va, ia, pc, sr, vec, irq, tw, f2}
  input  wire        rs,             // restart at rs_step (replay of an exception-sequence uop, or after SY_MCLR)
  input  wire [4:0]  rs_step,
  input  wire        adv,                    // DS advances this clock
  output reg         act,                    // the sequence runs
  output reg  [4:0]  step,                   // the current step
  output wire        last,                   // the current step is the last (the vector read and jump)
  output reg  [`UW-1:0] u                    // the uop of the current step (zero while idle)
) /* synthesis syn_maxfan = 16 */;
// pcr: the PC the uops carry; va: the vector address; ia: the instruction address (format 2); pc: the PC to stack
reg [31:0] pcr, va, ia, pc;
reg [15:0] sr;                               // the SR to stack (the value before the exception)
reg [7:0]  vec;                              // the vector number
reg        irq, tw, f2;                      // an interrupt; throwaway frame needed (M = 1); format 2

wire [4:0] nfr   = f2 ? 5'd6 : 5'd4;                  // frame stores
wire [4:0] jstep = tw ? 5'd10 : nfr + 5'd1;           // vector read and jump
assign last = (step == jstep);

always @(posedge clk) begin
  if (rst) act <= 1'b0;
  else if (go) begin                                  // start: latch the parameters
    act <= 1'b1; step <= 5'd1;
    {pcr, va, ia, pc, sr, vec, irq, tw, f2} <= xs_par;
  end else if (rs) begin act <= 1'b1; step <= rs_step; end   // replay or M-clear restart
  else if (flush) act <= 1'b0;
  else if (act && adv) begin
    if (last) act <= 1'b0;
    step <= step + 5'd1;
  end
end

// frame word k of the main frame and of the throwaway frame
reg  [15:0] fw;                                       // the word to store
reg  [7:0]  fdisp, fsize;                             // its offset from A7, and the frame size in bytes
reg         fend;                                     // the last store of a frame (it moves A7)
wire [4:0]  k0 = step - 5'd1, k1 = step - 5'd6;       // word index in the main frame (steps 1..) and throwaway (6..)
always @* begin
  fsize = f2 ? 8'd12 : 8'd8;
  fdisp = 8'h0; fw = 16'h0; fend = 1'b0;
  if (step <= nfr) begin                              // main frame: word k at A7 - size + 2k
    fdisp = {2'b00, k0, 1'b0} - fsize; fend = (step == nfr);
    case (k0[2:0])
      3'd0: fw = sr;
      3'd1: fw = pc[31:16];
      3'd2: fw = pc[15:0];
      3'd3: fw = {f2 ? 4'h2 : 4'h0, 2'b00, vec, 2'b00};   // format 2 or 0, vector offset
      3'd4: fw = ia[31:16];                           // format 2: instruction address
      default: fw = ia[15:0];
    endcase
  end else begin                                      // throwaway frame (format 1) on the ISP, 8 bytes
    fsize = 8'd8;
    fdisp = {2'b00, k1, 1'b0} - 8'd8; fend = (k1 == 5'd3);
    case (k1[1:0])
      2'd0: fw = sr | 16'h2000;                       // the SR with S set
      2'd1: fw = pc[31:16];
      2'd2: fw = pc[15:0];
      default: fw = {4'h1, 2'b00, vec, 2'b00};        // format 1
    endcase
  end
end

always @* begin
  u = {`UW{1'b0}};
  if (act) begin
    u[`F_EXC] = {irq ? `XS_IRQ : `XS_EXC, 6'h0};          // marks the uop as part of an exception sequence
    u[`F_PC]  = pcr;
    if (last) begin                                         // vector read, jump
      // load the long at the absolute address va (base zero) and jump to it
      u[`F_LA_EN] = 1'b1; u[`F_LA_DISP] = va; u[`F_LSZ] = `SZ_L; u[`F_ASEL] = `AS_MEM;
      u[`F_UNIT] = `UN_SYS; u[`F_OP] = `SY_JMP;
    end else if (tw && step == 5'd5) begin                  // clear M: swap to the ISP, restart at step 6
      u[`F_UNIT] = `UN_SYS; u[`F_OP] = `SY_MCLR;
    end else begin                                          // frame word store at A7 + disp
      u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_MOV; u[`F_SZ] = `SZ_W; u[`F_ASEL] = `AS_IMM; u[`F_IMM] = {16'h0, fw};
      u[`F_SA_EN] = 1'b1; u[`F_SA_BSEL] = `BA_REG; u[`F_SA_BREG] = 5'd15; u[`F_SA_DISP] = {{24{fdisp[7]}}, fdisp};
      if (fend) begin u[`F_SA_UEN] = 1'b1; u[`F_SA_UDL] = 8'h0 - fsize; end   // the last store moves A7 down by the size
    end
  end
end

endmodule
`default_nettype wire
