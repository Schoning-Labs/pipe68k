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
// p68_sseq: the system sequencer, for the multi-step system instructions RTE, CHK2/CMP2, CHK, MOVES and
// MOVE (An)+/-(An),SR.
//
// Where it sits. In the decode stage (DS, p68_decode.v), beside the main sequencer p68_seq.v, with the same contract:
// dec_sys returns a class (SS_*) and parameters, step 0 is the decoder's (normally empty) micro-op (uop), and this module
// produces steps 1..n from registers latched at the start. Every uop is a function of class, step and parameters, so a
// replay restart only needs the step number. The uops go through DR (register stage before address generation), AG
// (address generation), OC (operand cycle), EX (execute, the system unit p68_sys.v for most of them) and WB
// (write-back). DS decides from the opcode alone which of the two sequencers starts, so their classes never overlap.
// These instructions have their own sequencer because they were built as one work package with the system unit; it
// also keeps the parameter block of the main sequencer narrower.
//
// SS_RTE   (A7-relative word loads, the stacked PC is two words because A7+2 is not long-aligned):
//          1 format word (A7+6): latch A7 + frame size, raise format error   2 SR (A7): latch
//          3 PC high (A7+2): latch   4 PC low (A7+4): SR <- latched SR, A7 <- latched value, jump (format 1:
//          restart the RTE on the new stack instead). A7 changes only in the last step (replay-safe).
// SS_CHK2  1 lower bound at EA: latch   2 upper bound at EA + size, compare with Rn, flags Z C   3 (CHK2 only)
//          raise CHK if out of bounds
// SS_CHK   1 raise CHK if step 0 found Dn out of bounds
// SS_MOVES 1 the access (load into Rn or store Rn); SFC/DFC are not on the bus yet
// SS_SRW   1 redirect to the next instruction after MOVE (An)+/-(An),SR (a redirecting uop must not update An)
// The raise steps (SY_XCHK) use the condition latched by the compare step, so no compare feeds the redirect. This is the
// rule from the exception work that never changed: a condition that raises an exception feeds the global flush, so it
// must come from a register (HISTORY.md section 7).
// The 68020 stack frame RTE reads: SR at (A7), the PC at 2(A7) to 5(A7), the format and vector offset word at 6(A7),
// then more words for the larger formats. RTE accepts format 0 (4 words), format 1 (throwaway, 4 words) and format 2
// (6 words); any other format raises vector 14 (format error).
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`include "p68_sys.vh"
`default_nettype none
module p68_sseq (
  input  wire        clk,
  input  wire        rst,
  input  wire        flush,                  // pipeline flush: the sequence is abandoned
  input  wire        start,          // step 0 of a system sequence leaves DS this clock
  input  wire [2:0]  cls_in,                 // class from dec_sys
  input  wire [44:0] par_in,                 // parameters from dec_sys: {x1, x0, rn, flag, size, mode, reg}
  input  wire        adv,                    // DS advances this clock
  output reg         act,                    // a sequence runs (steps 1..n)
  output reg  [4:0]  step,                   // the current step
  output wire        last,                   // the current step is the last
  output reg  [`UW-1:0] u,                   // the uop of the current step (zero while idle)
  input  wire        ts,             // register bases equal to {01, tr} become T1 (p68_fx main part)
  input  wire [2:0]  tr
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
reg [2:0]  cls;                              // the running class
reg [15:0] x0, x1;                           // the EA's extension words
reg [3:0]  rn;                               // Rn from the extension word (bit 3: An)
reg        fl;                               // CHK2 (not CMP2), or MOVES register to memory
reg [1:0]  sz;                               // operand size
reg [2:0]  em, er;                           // EA mode and register

// RTE has 4 steps, CHK2 3 (CMP2 2), the others 1
assign last = (cls == `SS_RTE) ? (step == 5'd4) : (cls == `SS_CHK2) ? (step == (fl ? 5'd3 : 5'd2)) : 1'b1;

always @(posedge clk) begin                    // parameters load while no sequence runs (read only while act)
  if (rst || flush) act <= 1'b0;
  else if (start) act <= 1'b1;
  else if (act && adv && last) act <= 1'b0;
  if (!act) begin
    step <= 5'd1; cls <= cls_in;
    {x1, x0, rn, fl, sz, em, er} <= par_in;
  end else if (adv) step <= step + 5'd1;
end

wire [`LA_W-1:0] la  = ea_lat(em, er, sz, x0, x1, 4'd4, ts, tr);     // EA extension words follow the extension word
wire [31:0]      sb  = (sz == `SZ_B) ? 32'd1 : (sz == `SZ_W) ? 32'd2 : 32'd4;   // operand size in bytes
// CHK2's upper bound displacement as a register (step 2 comes at least one clock after the parameters loaded)
reg  [31:0]      ubd;
always @(posedge clk) ubd <= la[`L_DISP] + sb;
always @* begin
  u = {`UW{1'b0}};
  if (act) case (cls)
    `SS_RTE: begin                                            // word loads relative to A7, all for the system unit
      u[`F_LA_EN] = 1'b1; u[`F_LA_BSEL] = `BA_REG; u[`F_LA_BREG] = tbs(5'd15, ts, tr); u[`F_LSZ] = `SZ_W;
      u[`F_ASEL] = `AS_MEM; u[`F_UNIT] = `UN_SYS;
      case (step)
        // format word; B = A7 so the unit can latch A7 + frame size; vector 14 if the format is not accepted
        5'd1: begin u[`F_LA_DISP] = 32'd6; u[`F_OP] = `SY_RTEF; u[`F_BREG] = 5'd15; u[`F_EXC] = 8'd14; end
        5'd2: u[`F_OP] = `SY_RTESR;                           // the stacked SR at (A7)
        5'd3: begin u[`F_LA_DISP] = 32'd2; u[`F_OP] = `SY_RTEPH; end   // PC high word
        default: begin                                        // PC low word: set SR, write A7, jump
          u[`F_LA_DISP] = 32'd4; u[`F_OP] = `SY_RTEJ; u[`F_SZ] = `SZ_L; u[`F_WEN] = 1'b1; u[`F_WREG] = 5'd15;
        end
      endcase
    end
    `SS_CHK2: begin                                           // CHK2 / CMP2 <ea>,Rn
      u[`F_LA] = la; u[`F_LSZ] = sz; u[`F_ASEL] = `AS_MEM; u[`F_UNIT] = `UN_SYS;
      if (step == 5'd1) u[`F_OP] = `SY_C2L;                   // lower bound at EA: latch it
      else if (step == 5'd2) begin                            // upper bound at EA + size: compare with Rn
        u[`F_LA_DISP] = ubd;
        u[`F_OP] = `SY_C2U; u[`F_SZ] = sz; u[`F_BREG] = {1'b0, rn}; u[`F_IMM] = {31'h0, rn[3]};   // IMM[0]: Rn is An
      end else begin                                          // CHK2: raise vector 6 from the latched result
        u[`F_LA] = {`LA_W{1'b0}}; u[`F_ASEL] = `AS_REG; u[`F_OP] = `SY_XCHK; u[`F_EXC] = 8'd6;
      end
    end
    `SS_CHK: begin u[`F_UNIT] = `UN_SYS; u[`F_OP] = `SY_XCHK; u[`F_EXC] = 8'd6; end   // raise vector 6 if latched
    `SS_SRW: begin u[`F_UNIT] = `UN_SYS; u[`F_OP] = `SY_SER; end                      // redirect to the next instruction
    `SS_MOVES: begin                                          // the access itself
      u[`F_LA] = la;
      if (fl) begin                                           // Rn -> <ea>
        u[`F_LA_EN] = 1'b0; u[`F_SA_EN] = 1'b1; u[`F_SA_SAME] = 1'b1;   // store at the EA's address, no load
        u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_MOV; u[`F_SZ] = sz; u[`F_ASEL] = `AS_REG; u[`F_AREG] = {1'b0, rn};
        if ((em == 3'd3 || em == 3'd4) && rn[3] && rn[2:0] == er) begin   // An,(An)+ / -(An) stores the updated An (68020)
          u[`F_AREG] = `R_T0;   // step 0 (dec_sys) left An +- size in T0
        end
      end else begin                                          // <ea> -> Rn
        u[`F_LSZ] = sz; u[`F_ASEL] = `AS_MEM; u[`F_WEN] = 1'b1; u[`F_WREG] = {1'b0, rn};
        if (rn[3]) begin u[`F_UNIT] = `UN_SYS; u[`F_OP] = `SY_MOVX; u[`F_SZ] = `SZ_L; end   // An: sign-extended to 32 bits
        else begin u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_MOV; u[`F_SZ] = sz; u[`F_BREG] = {1'b0, rn}; end   // Dn: merged
      end
    end
    default: ;
  endcase
end

endmodule
`default_nettype wire
