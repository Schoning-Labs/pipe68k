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
// dec_p7: the group decoder for the compare-and-swap instructions CAS and CAS2, and for BTST Dn,#imm.
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v): it sees the opcode word w0 and the
// words behind it, says whether the opcode is its own (hit), and gives one micro-op (uop, p68_uop.vh) or starts a
// sequence (seq_cas.v inside p68_seq.v). The uops run through DR (register stage before address generation), AG
// (address generation), OC (operand cycle), EX (execute) and WB (write-back). Combinational logic only. These
// instructions were added together with the 68020 addressing completion, which is why they share a decoder that is not
// grouped by opcode line.
//
// The instructions.
//
//     CAS  Dc,Du,<ea>      0000 1 size 011 EA     + extension  0000000 Du 000 Dc     size 01 .B, 10 .W, 11 .L
//     CAS2 Dc1:Dc2,Du1:Du2,(Rn1):(Rn2)
//                          0000 1 size 011 111100 + ext1  D/A Rn1 000 Du1 000 Dc1   + ext2  D/A Rn2 000 Du2 000 Dc2
//                                                                          (0CFC .W, 0EFC .L)
//     BTST Dn,#imm         0000 Dn 100 111100     + immediate word (the operand to test is the immediate)
//
// CAS compares Dc with the memory operand. If they are equal it stores Du to memory, otherwise it loads the memory
// operand into Dc; the flags come from the compare. CAS2 does the same with two operands in two places, and stores only
// if both compares are equal. Both are atomic on a real 68020 (a locked read-modify-write bus cycle), which matters for
// multiprocessor code and is rarely used on the Amiga, but AROS and some programs do use it. Rn in CAS2 can be a data or
// an address register (bit 15 of the extension word, part of the 4-bit Rn field here).
// BTST Dn,#imm is the only bit instruction that allows an immediate as the operand being tested.
//
// How they become uops. CAS and CAS2 are sequences of class SQ_CAS (step 0, this decoder's uop, is empty). Their steps
// are in seq_cas.v; the loads are uncached and the bus lock is held across the compare and the store. BTST Dn,#imm is a
// single uop with the immediate as operand B (BS_IMM), so the register's bit number can stay in operand A as for every
// other BTST.
// Parameters (spar):
//   CAS : [0] 0, [2:1] size, [5:3] Dc, [8:6] Du, [11:9] EA mode, [14:12] EA register, [46:15] EA extension words
//   CAS2: [0] 1, [2:1] size, [5:3] Dc1, [8:6] Du1, [12:9] Rn1, [15:13] Dc2, [18:16] Du2, [22:19] Rn2
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_p7 (
  input  wire [15:0] w0, w1, w2, w3,         // opcode, extension word(s), EA extension words
  output wire        hit,                    // CAS, CAS2 or BTST Dn,#imm
  output wire [`UW-1:0] u,                   // the uop: nonzero only for BTST Dn,#imm (hb), so it needs no pg gate
  output wire [3:0]  seq,                    // SQ_CAS for CAS and CAS2
  output wire [63:0] spar                    // sequence parameters (layouts above)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
`include "p68_sysf.vh"
`include "p68_p7f.vh"
wire hc = p7_cas(w0), hc2 = p7_cas2(w0), hb = p7_btimm(w0);   // legality, shared with the fetch predecode (p68_p7f.vh)
// The parameters are read only for a legal sequence of this decoder (its class is exact), so they are gated by the
// opcode pattern alone; it excludes every other decoder's sequence opcodes.
wire hcp = (w0[15:11] == 5'b00001) && (w0[10:9] != 2'b00) && (w0[8:6] == 3'b011);   // CAS without the EA check
wire [1:0] sz = w0[10:9] - 2'd1;                              // CAS size field 01/10/11 -> SZ_B/SZ_W/SZ_L
reg  [`UW-1:0] v;
reg  [63:0] p;
always @* begin
  v = {`UW{1'b0}}; p = 64'h0;
  if (hb) begin                                               // BTST Dn,#imm
    // bit number in Dn (operand A), byte operand = low byte of the immediate word (operand B)
    v[`F_UNIT] = `UN_ALU; v[`F_OP] = `OP_BTST; v[`F_SZ] = `SZ_B; v[`F_ASEL] = `AS_REG; v[`F_AREG] = {2'b00, w0[11:9]};
    v[`F_BSEL] = `BS_IMM; v[`F_IMM] = dcode(3'd3, 2'd1, 17'h0);
    v[`F_FEN] = 5'b00100; v[`F_FCV] = `CV_CLR; v[`F_FZM] = `ZM_BIT;   // only Z, from the tested bit
  end
  // CAS: {EA extension words w2 w3, EA register, EA mode, Du, Dc, size, 0}
  if (hcp) p = {17'h0, w2, w3, w0[2:0], w0[5:3], w1[8:6], w1[2:0], sz, 1'b0};
  // CAS2: {Rn2, Du2, Dc2 from w2, Rn1, Du1, Dc1 from w1, size, 1}
  if (hc2) p = {41'h0, w2[15:12], w2[8:6], w2[2:0], w1[15:12], w1[8:6], w1[2:0], sz, 1'b1};
end
assign hit  = hc || hc2 || hb;
assign u    = v;
assign seq  = (hc || hc2) ? `SQ_CAS : `SQ_NONE;
assign spar = p;
endmodule
`default_nettype wire
