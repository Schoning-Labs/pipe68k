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
// dec_branch: the group decoder for Bcc and BRA (line 6, all conditions except 1, which is BSR).
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v): it sees the opcode word w0 and the
// words behind it, says whether the opcode is its own (hit), and gives one micro-op (uop, p68_uop.vh) that DS ORs with
// the other decoders' uops. The uop then runs through DR (register stage before address generation), AG (address
// generation), OC (operand cycle), EX (execute) and WB (write-back). Combinational logic only.
//
// The instruction.
//
//     0110 | cond (4 bits) | 8-bit displacement
//
// The target is the address of the opcode plus 2 plus the displacement (the 68k measures branches from the word behind
// the opcode). An 8-bit displacement of 0x00 means a 16-bit displacement follows in the next word, and on the 68020
// 0xFF means a 32-bit displacement follows in the next two words. Condition 0 is BRA (always), 1 is BSR (branch to
// subroutine, decoded by dec_flow), and 2-15 are the 14 conditions HI, LS, CC, CS, NE, EQ, VC, VS, PL, MI, GE, LT, GT, LE.
//
// How it becomes a uop. One uop. The target address is computed by the load address adder in AG as PC + 2 + disp
// (base PC, constant index 2, displacement), but no load happens. BRA is unconditional, so AG redirects the fetch
// itself (F_BRA) and the instruction costs very little. Bcc needs the flags, which are only known in EX, so it is unit
// UN_BRA and EX redirects when the condition holds. To avoid paying that late redirect in loops, a backward Bcc (negative
// displacement) is predicted taken: AG redirects as for BRA, and EX corrects to the next instruction if the branch was in
// fact not taken. The branch target cache (BTC, in fetch) can predict earlier still; F_BR marks the uop so that the BTC
// is updated when the branch commits.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_branch (
  input  wire [15:0] w0, w1, w2, w3,         // opcode word and the next words (16- or 32-bit displacement)
  output wire        hit,                    // line 6 and not BSR
  input  wire        pg,             // predecoded gate (this decoder's ug from p68_pdec, a register in the entry)
  output wire        ug,             // the gate of the uop, for p68_pdec's own copy of this decoder
  output wire [`UW-1:0] u                    // the uop (zero unless pg)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
wire       h = (w0[15:12] == 4'h6) && (w0[11:8] != 4'h1);   // every line-6 word except BSR is a legal branch
// The displacement as a data code (p68_dec.vh): 0x00 = sign-extended word w1, 0xFF = the long {w1, w2}, else the byte
// itself, sign-extended (a constant that fits the code).
wire [31:0] disp = (w0[7:0] == 8'h00) ? dcode(3'd1, 2'd1, 17'h0) : (w0[7:0] == 8'hFF) ? dcode(3'd4, 2'd1, 17'h0) : dcon({{24{w0[7]}}, w0[7:0]});
wire       dneg = (w0[7:0] == 8'h00 || w0[7:0] == 8'hFF) ? w1[15] : w0[7];   // the displacement's sign (backward branch)
reg  [`UW-1:0] v;
always @* begin
  v = {`UW{1'b0}};
  v[`F_CC] = w0[11:8];                                                  // the condition, tested in EX
  v[`F_LA_BSEL] = `BA_PC; v[`F_LA_XK] = 4'd2; v[`F_LA_DISP] = disp;   // target = PC + 2 + disp
  if (w0[11:8] == 4'h0) v[`F_BRA] = 1'b1;                              // BRA: AG redirects to the target
  else v[`F_UNIT] = `UN_BRA;                                            // Bcc: EX resolves it with the flags
  // A backward Bcc is predicted taken (AG redirects as for BRA, EX corrects to the next instruction when it is not
  // taken); loops branch backward and are taken almost every time. F_BR marks Bcc/BRA for the branch target cache.
  if (w0[11:8] != 4'h0 && dneg) begin v[`F_BRA] = 1'b1; v[`F_PRED] = 1'b1; end
  v[`F_BR] = 1'b1;
end
assign hit = h;
// The uop is gated by pg, this decoder's gate predecoded with the instruction buffer entry (p68_pdec).
assign ug   = h;
assign u   = pg ? v : {`UW{1'b0}};
endmodule
`default_nettype wire
