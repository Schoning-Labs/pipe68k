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
// dec_movem: the group decoder for MOVEM (move multiple registers, line 4).
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v): it sees the opcode word w0 and the
// words behind it, says whether the opcode is its own (hit), and gives one micro-op (uop, p68_uop.vh). MOVEM moves up to
// 16 registers, so it cannot be one uop; it is a sequence, and this decoder's job is mostly to start it: it gives the
// sequence class SQ_MOVEM and the parameters, and the sequencer (p68_seq.v, also in DS) produces the uops of steps 1 to
// n, one per clock. The uops run through DR (register stage before address generation), AG (address generation),
// OC (operand cycle), EX (execute) and WB (write-back). Combinational logic only.
//
// The instruction.
//
//     0100 1 | dr | 001 | sz | EA mode | EA reg         followed by a 16-bit register mask, then EA extension words
//              10          6
//
// dr (bit 10): 0 registers to memory, 1 memory to registers. sz (bit 6): 0 word, 1 long. A word load is sign-extended
// to 32 bits even into a data register. The mask has one bit per register: bit 0 = D0 ... bit 7 = D7, bit 8 = A0 ...
// bit 15 = A7. For -(An) (registers to memory with predecrement) the mask is reversed (bit 0 = A7 ... bit 15 = D0),
// because the registers are stored from the highest address downward. Allowed EAs: registers to memory takes (An),
// -(An), (d16,An), (d8,An,Xn), abs.W, abs.L; memory to registers takes (An), (An)+, (d16,An), (d8,An,Xn), abs.W, abs.L
// and the two PC-relative modes (the movem_ok function in p68_dec.vh, shared with the fetch predecode).
//
// How it becomes uops. MOVEM was the first sequence in the core, and it set the pattern for all the others (HISTORY.md
// section 6). Step 0, this decoder's uop, is normally empty: DS cannot afford a select between "decoder uop" and
// "sequencer uop" in the decode path (measured at about 20 MHz early on), so the decoder uop and the sequencer uop are
// simply ORed, and a sequence starts with an empty step. For control modes the sequencer's step 1 computes the EA into
// the temporary T0, then each further step moves one register. Sequence parameters (spar):
//   [15:0] mask (transfer order: bit 0 first), [16] memory to registers, [17] long, [18] -(An), [19] (An)+,
//   [24:20] base register for the transfers (An, or T0 for control modes), [27:25] EA mode, [30:28] EA register,
//   [63:32] the two words after the mask (EA extension words).
//
// One special case gives step 0 real work. MOVEM.L D0/A0,-(A0) stores A0, which is also the base. The 68020 stores the
// initial A0 minus the operand size in that case (the 68000 stores the initial A0). So when the base is in the mask,
// step 0 computes An - size into T0, and the sequencer stores T0 in place of that register.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_movem (
  input  wire [15:0] w0, w1, w2, w3,         // opcode, register mask, two EA extension words
  output wire        hit,                    // MOVEM with a legal EA
  input  wire        pg,             // predecoded gate (this decoder's ug from p68_pdec, a register in the entry)
  output wire        ug,             // the gate of the uop, for p68_pdec's own copy of this decoder
  output wire [`UW-1:0] u,                   // step 0 uop (zero unless pg)
  output wire [3:0]  seq,                    // SQ_MOVEM for a hit
  output wire [63:0] spar                    // sequence parameters (layout above)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
wire [2:0] m3 = w0[5:3], r0 = w0[2:0];                        // EA mode and register
wire       h   = movem_ok(w0);                                // line 4, bit 11, bits 9-7 = 001, direction-dependent EA check
wire       pre = (m3 == 3'd4), post = (m3 == 3'd3), ctl = !(pre || post);   // -(An), (An)+, or a control mode
wire [4:0] base = ctl ? `R_T0 : {2'b01, r0};                  // control modes: step 1 puts the EA into T0
assign hit  = h;
// -(An) with An in the list: step 0 computes An - size into T0 (the sequencer stores T0 for that register, as the 68020
// does). In the reversed -(An) mask, An's bit is 7 - n, which is ~r0 in three bits.
wire       basein = (m3 == 3'd4) && w1[{1'b0, ~r0}];
reg  [`UW-1:0] v0;
always @* begin
  v0 = {`UW{1'b0}};
  // T0 = An + (-4 or -2): an ALU add of a negative immediate, size by bit 6
  v0[`F_UNIT] = `UN_ALU; v0[`F_OP] = `OP_ADD; v0[`F_SZ] = `SZ_L; v0[`F_ASEL] = `AS_IMM; v0[`F_IMM] = dcon(32'h0 - (w0[6] ? 32'd4 : 32'd2));
  v0[`F_BREG] = {2'b01, r0}; v0[`F_WEN] = 1'b1; v0[`F_WREG] = `R_T0;   // operand B = An, result to T0, no flags
end
// The uop is gated by pg, this decoder's gate predecoded with the instruction buffer entry (p68_pdec). The gate is set
// only in the base-in-list case, so in every other MOVEM step 0 is empty.
assign ug   = h && basein;
assign u    = pg ? v0 : {`UW{1'b0}};
assign seq  = h ? `SQ_MOVEM : `SQ_NONE;
// The parameters are read only for a legal sequence of this decoder (its class is exact), so they are gated by the
// opcode pattern alone (no EA check); the pattern excludes every other decoder's sequence opcodes.
assign spar = (w0[15:12] == 4'h4 && w0[11] && w0[9:7] == 3'b001) ? {w2, w3, 1'b0, r0, m3, base, post, pre, w0[6], w0[10], w1} : 64'h0;
endmodule
`default_nettype wire
