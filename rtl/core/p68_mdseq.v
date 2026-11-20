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
// p68_mdseq: the sequence uops of multiply and divide (classes SQ_MUL and SQ_DIV).
//
// Where it sits. Instantiated by the sequencer p68_seq.v inside the decode stage (DS). Combinational: a uop (micro-op,
// p68_uop.vh) is a function of class, step and the parameters latched at the start (layout in dec_md.v). p68_seq ORs it
// with the other sub-sequencers' uops, and the merged uop goes on through DR (register stage before address
// generation), AG (address generation), OC (operand cycle), EX (execute, where the multi-clock unit p68_md.v sits) and WB
// (write-back).
//
// The multi-clock unit takes its operands in step 0 (and 1) and has a fixed latency; the sequence issues empty uops until
// the result is ready, then the result uops (an empty uop is one clock in every stage, so a result uop is never in EX
// before the unit is done). The multiply uses four DSP products and needs 6 clocks; the divide is a radix-4 restoring
// divider of 16 clocks. Because the interface is only "start, wait a fixed number of uops, read", a faster divider can
// replace the unit without touching anything else but the step counts here.
// Parameters (dec_md.v): [2:0] Dl / Dq / Dn, [5:3] Dh / Dr, [6] signed, [7] 64-bit, [8] W form.
//   MUL: step 0 start, 1-5 empty, 6 (64-bit: high half to Dh), last: low half to Dl with the flags.
//   DIV: step 0 load divisor, 1 go (dividend low), 2 divide-by-zero check (raises vector 5 from the latched flag),
//        3-20 empty, 21 remainder to Dr (long forms), last: quotient (W form: remainder:quotient) to Dq with the flags.
//        Dr is written before Dq: with Dr = Dq the quotient wins (Musashi). Overflow: the unit returns the old values.
// The divide-by-zero check is its own uop because a raise condition must come from a register (the unit latches
// "divisor is zero" when it loads the divisor): the condition feeds the pipeline flush, and a compare in front of it
// would be too slow.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`include "p68_md.vh"
`default_nettype none
module p68_mdseq (
  input  wire [3:0]  cls,                    // the running sequence class
  input  wire [4:0]  step,                   // the current step (1..n)
  input  wire [63:0] sp,                     // the parameters latched at the start
  output reg  [`UW-1:0] u,                   // the uop of this step (zero for other classes)
  output reg         last                    // this step is the last
);
wire [2:0] dl = sp[2:0], dh = sp[5:3];                        // low / quotient register, high / remainder register
wire       sg = sp[6], s64 = sp[7], w16 = sp[8];              // signed, 64-bit, word form
wire [3:0] cc = {1'b0, s64, w16, sg};                         // the variant, in every UN_MUL / UN_DIV uop's CC field
always @* begin
  u = {`UW{1'b0}};
  last = 1'b1;
  if (cls == `SQ_MUL) begin
    last = s64 ? (step == 5'd7) : (step == 5'd6);             // one more step for the high half
    if (step >= 5'd6) begin                                   // steps 1-5 are empty (the multiply runs)
      u[`F_UNIT] = `UN_MUL; u[`F_SZ] = `SZ_L; u[`F_CC] = cc; u[`F_WEN] = 1'b1;
      // with Dh = Dl the 68020 leaves the high half in the register (the high write comes last, with the flags)
      if (s64 && step == 5'd6 && dh != dl) begin u[`F_OP] = `MD_GHI; u[`F_WREG] = {2'b00, dh}; end   // high to Dh
      else if (s64 && step == 5'd6) begin u[`F_OP] = `MD_GLO; u[`F_WREG] = {2'b00, dl}; end          // Dh = Dl: low first
      else if (s64 && dh == dl) begin u[`F_OP] = `MD_GHI; u[`F_WREG] = {2'b00, dh}; u[`F_FEN] = `FL_NZVC; end   // then high
      else begin u[`F_OP] = `MD_GLO; u[`F_WREG] = {2'b00, dl}; u[`F_FEN] = `FL_NZVC; end            // low to Dl, flags
    end
  end else if (cls == `SQ_DIV) begin
    last = w16 ? (step == 5'd21) : (step == 5'd22);           // the long forms write the remainder in a step of its own
    if (step == 5'd1) begin                                   // dividend low (Dq) to the unit, start the divide
      u[`F_UNIT] = `UN_DIV; u[`F_OP] = `MD_DGO; u[`F_SZ] = `SZ_L; u[`F_BSEL] = `BS_REG; u[`F_BREG] = {2'b00, dl}; u[`F_CC] = cc;
    end else if (step == 5'd2) begin                          // raise vector 5 if the latched divisor was zero
      u[`F_UNIT] = `UN_DIV; u[`F_OP] = `MD_CHK; u[`F_EXC] = 8'd5;
    end else if (step >= 5'd21) begin                         // results (steps 3-20 are empty)
      u[`F_UNIT] = `UN_DIV; u[`F_SZ] = `SZ_L; u[`F_CC] = cc; u[`F_WEN] = 1'b1;
      if (!w16 && step == 5'd21) begin u[`F_OP] = `MD_GHI; u[`F_WREG] = {2'b00, dh}; end            // remainder to Dr
      else begin u[`F_OP] = `MD_GLO; u[`F_WREG] = {2'b00, dl}; u[`F_FEN] = `FL_NZVC; end            // quotient, flags
    end
  end
end
endmodule
`default_nettype wire
