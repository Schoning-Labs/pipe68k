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
// dec_md: the group decoder for multiply and divide: MULU.W, MULS.W, DIVU.W, DIVS.W <ea>,Dn and the 68020 long forms
// MULU.L, MULS.L, DIVU.L, DIVS.L, DIVUL.L, DIVSL.L.
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v): it sees the opcode word w0 and the
// words behind it, says whether the opcode is its own (hit), gives the first micro-op (uop, p68_uop.vh) and starts a
// sequence that the multiply/divide sequencer (p68_mdseq.v, inside p68_seq.v) continues. The uops run through DR
// (register stage before address generation), AG (address generation), OC (operand cycle), EX (execute, where the
// multi-clock unit p68_md.v sits) and WB (write-back). Combinational logic only.
//
// The instructions.
//
//     MULU.W  1100 Dn 011 EA      MULS.W  1100 Dn 111 EA       16 x 16 -> 32 bits into Dn
//     DIVU.W  1000 Dn 011 EA      DIVS.W  1000 Dn 111 EA       32 / 16 -> 16-bit remainder : 16-bit quotient in Dn
//     MULx.L  0100 1100 00 EA  + extension  0 Dl s size 0000000 Dh     32 x 32 -> 32 (Dl), or 64 (Dh:Dl) with size = 1
//     DIVx.L  0100 1100 01 EA  + extension  0 Dq s size 0000000 Dr     32 / 32 or 64 / 32 -> quotient Dq, remainder Dr
//
// In the word forms bit 8 is the signed bit (MULS, DIVS); in the long forms it is bit 11 of the extension word (s), and
// bit 10 (size) chooses the 64-bit product or the 64-bit dividend Dr:Dq. DIVUL/DIVSL are the 32-bit forms with a
// separate remainder register (Dr != Dq). The source EA is any data mode, including an immediate. The extension word of
// the long forms comes before the EA's own extension words, so these begin at w2 (xs = 1 tells DS for the index field).
//
// How they become uops. Every form is a sequence (class SQ_MUL or SQ_DIV) and step 0 is this decoder's uop, which in
// this case does real work: MULx starts the multiply (A = source EA, B = the other factor), DIVx loads the divisor
// (A = source EA) and the dividend high register. The unit sits behind a fixed-latency interface: the multiply uses four
// DSP products and takes 6 clocks, the divide is a radix-4 restoring divider of 16 clocks (three subtractors per clock).
// A divider built on DSP blocks (reciprocal estimate and Newton-Raphson) was planned first, but worked out in detail it
// needed about 20 clocks with the DSP pipelines and an error proof, no faster than the radix-4 divider, so the simpler
// exact one was built (HISTORY.md section 6). The CC field of the uop carries the variant: {0, 64-bit, W form, signed}.
// Sequence parameters: [2:0] Dn / Dl / Dq, [5:3] Dh / Dr, [6] signed, [7] 64-bit, [8] W form.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`include "p68_md.vh"
`default_nettype none
module dec_md (
  input  wire [15:0] w0, w1, w2, w3,         // opcode, extension word (long forms), EA extension words
  output wire        hit,                    // a legal multiply or divide
  input  wire        pg,             // predecoded gate (this decoder's ug from p68_pdec, a register in the entry)
  output wire        ug,             // the gate of the uop, for p68_pdec's own copy of this decoder
  output wire [`UW-1:0] u,                   // step 0 uop (zero unless pg)
  output wire [1:0]  xs,                     // 1 for the long forms: their EA extension words start at w2
  output wire [3:0]  seq,                    // SQ_MUL or SQ_DIV
  output wire [63:0] spar                    // sequence parameters (layout above)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
`include "p68_mdf.vh"
wire [2:0] m3 = w0[5:3], r0 = w0[2:0];                        // source EA mode and register
wire       hw = md_w(w0), hl = md_l(w0), h = hw | hl;         // word forms, long forms (legality in p68_mdf.vh)
// The parameters are read only for a legal sequence of this decoder (its class is exact), so they are gated by the
// opcode pattern alone; it excludes every other decoder's sequence opcodes.
wire       hwp = ((w0[15:12] == 4'hC) || (w0[15:12] == 4'h8)) && (w0[7:6] == 2'b11);   // the W forms without the EA check
wire       mul = hw ? (w0[15:12] == 4'hC) : !w0[6];           // line C = multiply; long forms: bit 6 clear = multiply
wire [1:0] lsz = hw ? `SZ_W : `SZ_L;                          // size of the source operand read
// operand A = the source EA; for the long forms its extension words start at word 2
wire [38:0] src = hw ? ea_srcc(m3, r0, `SZ_W, 2'd1) : ea_srcc(m3, r0, `SZ_L, 2'd2);
wire [2:0] dl  = hwp ? w0[11:9] : w1[14:12];                  // Dn (W form) or Dl / Dq (extension bits 14-12)
wire [2:0] dh  = hwp ? 3'd0 : w1[2:0];                        // Dh / Dr (extension bits 2-0)
wire       sg  = hwp ? w0[8] : w1[11];                        // signed
wire       s64 = hwp ? 1'b0 : w1[10];                         // 64-bit product or dividend
reg  [`UW-1:0] v;
always @* begin
  v = {`UW{1'b0}};
  v[`F_UNIT] = mul ? `UN_MUL : `UN_DIV; v[`F_OP] = mul ? `MD_MUL : `MD_DLD;   // start the multiply / load the divisor
  v[`F_SZ] = `SZ_L; v[`F_LSZ] = lsz;
  // source read; the long forms' EA extension words begin at word 2 (byte offset 4)
  v[`F_LA] = hw ? ea_lac(m3, r0, `SZ_W, 2'd1, 4'd2) : ea_lac(m3, r0, `SZ_L, 2'd2, 4'd4);
  v[`F_ASEL] = src[38:37]; v[`F_AREG] = src[36:32]; v[`F_IMM] = src[31:0];
  // B: the other factor (Dl) for a multiply; for a 64-bit divide the dividend high register Dr
  v[`F_BSEL] = `BS_REG; v[`F_BREG] = {2'b00, (!mul && s64) ? dh : dl};
  v[`F_CC] = {1'b0, s64, hw, sg};                             // the variant, for the unit
end
assign hit  = h;
// The uop is gated by pg, this decoder's gate predecoded with the instruction buffer entry (p68_pdec).
assign ug   = h;
assign u    = pg ? v : {`UW{1'b0}};
assign xs   = hl ? 2'd1 : 2'd0;
assign seq  = h ? (mul ? `SQ_MUL : `SQ_DIV) : `SQ_NONE;
assign spar = (hwp || (w0[15:7] == 9'b010011000)) ? {55'h0, hwp, s64, sg, dh, dl} : 64'h0;
endmodule
`default_nettype wire
