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
// dec_addx: the group decoder for the extended arithmetic ADDX and SUBX, the decimal (BCD) arithmetic ABCD, SBCD and
// NBCD, and the memory compare CMPM.
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v): it sees the opcode word w0, says
// whether the opcode is its own (hit), and gives one micro-op (uop, p68_uop.vh) that DS ORs with the other decoders'
// uops, or starts a sequence that the sequencer (p68_seq.v with seq_int.v) runs. The uops run through DR (register stage
// before address generation), AG (address generation), OC (operand cycle), EX (execute) and WB (write-back).
// Combinational logic only.
//
// The instructions. ADDX and SUBX add or subtract with the X flag as carry in, which is how the 68k does arithmetic on
// numbers wider than 32 bits. ABCD and SBCD do the same on two packed decimal digits per byte (BCD, binary coded
// decimal), and NBCD negates a BCD byte (0 - operand - X). All of these clear Z only when the result is nonzero and
// otherwise leave it as it was, so that after a chain of operations over a multi-precision number Z tells whether the
// whole number is zero (flag mode ZM_KEEP). CMPM compares two memory operands, both with postincrement. Encodings:
//
//     ADDX  1101 Rx 1 size 00 m Ry       SUBX  1001 Rx 1 size 00 m Ry       (size 00 .B, 01 .W, 10 .L)
//     ABCD  1100 Rx 1 0000    m Ry       SBCD  1000 Rx 1 0000    m Ry       (always byte)
//     CMPM  1011 Ax 1 size 001  Ay       (Ay)+,(Ax)+
//     NBCD  0100 1000 00 EA mode EA reg  (data alterable EA)
//
// Bit 3 (m) is 0 for the register form Dy,Dx and 1 for the memory form -(Ay),-(Ax). Rx in bits 11-9 is the destination,
// Ry in bits 2-0 the source.
//
// How they become uops.
//   Register forms and NBCD: one uop. EX computes Dx op Dy (or 0 - operand for NBCD) in the adder or the BCD part of the
//   extra unit, with ZM_KEEP. NBCD to memory reads, computes and stores back to the same address (SA_SAME).
//   Memory forms and CMPM: a uop can carry only one load, and these need two (both operands are in memory), so they are
//   a two-step sequence of class SQ_MM: step 1 loads the source into the temporary T0 with its address update, step 2
//   loads the destination, operates with T0 as operand A and stores the result back (CMPM stores nothing). Step 0, this
//   decoder's uop, is empty. SQ_MM parameters:
//     [4:0] op, [6:5] size, [11:7] flag enables, [14:12] C/V source, [16:15] Z mode, [17] store,
//     [18] -(An) (else (An)+), [21:19] source An (Ay), [24:22] destination An (Ax).
// BCD was at first three chained additions in one clock (about 10 logic levels in EX); it became three uops of one
// ten-bit add and a select each, since these instructions are rare (HISTORY.md section 6). That split is in EX's unit,
// not in this decoder: the decoder only names OP_ABCD, OP_SBCD and OP_NBCD.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_addx (
  input  wire [15:0] w0, w1, w2, w3,         // opcode word and the next words (NBCD's EA extension words)
  output wire        hit,                    // any instruction of this group
  input  wire        pg,             // predecoded gate (this decoder's ug from p68_pdec, a register in the entry)
  output wire        ug,             // the gate of the uop, for p68_pdec's own copy of this decoder
  output wire [`UW-1:0] u,                   // the uop (zero unless pg; empty for the memory sequences)
  output wire [3:0]  seq,                    // SQ_MM for the memory forms and CMPM
  output wire [63:0] spar                    // SQ_MM parameters (layout above)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
wire [3:0] g  = w0[15:12];                                    // opcode line: 8, 9, B, C, D (or 4 for NBCD)
wire [2:0] m3 = w0[5:3], r0 = w0[2:0], r9 = w0[11:9];         // EA mode (NBCD), source Ry, destination Rx
// legality predicates shared with the fetch predecode (p68_dec.vh): register forms, memory forms and CMPM, NBCD
wire       hr = p2_xrr(w0), hm = p2_xmm(w0), hn = p2_nbcd(w0);
wire       bcd = (g == 4'hC || g == 4'h8);                    // ABCD (line C) or SBCD (line 8)
wire       cmpm = (g == 4'hB);                                // CMPM (line B)
wire [1:0] sz = bcd ? `SZ_B : w0[7:6];                        // BCD is byte only
// the EX operation: NBCD, ABCD, SBCD, CMP (CMPM), ADDX (line D) or SUBX (line 9)
wire [4:0] op = hn ? `OP_NBCD : bcd ? ((g == 4'hC) ? `OP_ABCD : `OP_SBCD) : cmpm ? `OP_CMP : (g == 4'hD) ? `OP_ADDX : `OP_SUBX;
wire [2:0] cv = (hn || bcd) ? `CV_BCD : `CV_ADD;              // C and V from the BCD unit or from the adder
wire [4:0] fen_m = cmpm ? `FL_NZVC : `FL_ALL;                 // CMPM leaves X alone
wire [1:0] fzm_m = cmpm ? `ZM_SET : `ZM_KEEP;                 // CMPM sets Z normally; the others only clear it
// SQ_MM parameters: {Ax, Ay, predecrement, store, Z mode, C/V source, flag enables, size, op}; CMPM is the only
// postincrement form and the only one without a store
wire [24:0] pm = {r9, r0, !cmpm, !cmpm, fzm_m, cv, fen_m, sz, op};
reg  [`UW-1:0] v;
always @* begin
  v = {`UW{1'b0}};
  v[`F_UNIT] = `UN_ALU; v[`F_OP] = op; v[`F_SZ] = hn ? `SZ_B : sz; v[`F_LSZ] = hn ? `SZ_B : sz;   // NBCD is byte
  v[`F_FEN] = `FL_ALL; v[`F_FCV] = cv; v[`F_FZM] = `ZM_KEEP;  // all flags, Z only cleared (multi-precision rule)
  if (hn) begin                                               // NBCD <ea>: 0 - operand - X
    v[`F_ASEL] = `AS_IMM;                                     // operand A = 0 (the immediate code is zero)
    v[`F_LA] = ea_lac(m3, r0, `SZ_B, 2'd1, 4'd2);
    if (m3 == 3'd0) begin v[`F_BREG] = {2'b00, r0}; v[`F_WEN] = 1'b1; v[`F_WREG] = {2'b00, r0}; end   // NBCD Dn
    else begin v[`F_BSEL] = `BS_MEM; v[`F_SA_EN] = 1'b1; v[`F_SA_SAME] = 1'b1; end                   // read, store back
  end else begin                                           // Dy,Dx
    v[`F_ASEL] = `AS_REG; v[`F_AREG] = {2'b00, r0};           // A = Dy (source)
    v[`F_BREG] = {2'b00, r9}; v[`F_WEN] = 1'b1; v[`F_WREG] = {2'b00, r9};   // B = Dx, result to Dx
  end
end
assign hit  = hr || hm || hn;
// The uop is gated by pg, this decoder's gate predecoded with the instruction buffer entry (p68_pdec). The memory
// sequences have no gate, so their step 0 is empty.
assign ug   = hr || hn;
assign u    = pg ? v : {`UW{1'b0}};
assign seq  = hm ? `SQ_MM : `SQ_NONE;
assign spar = hm ? {39'h0, pm} : 64'h0;
endmodule
`default_nettype wire
