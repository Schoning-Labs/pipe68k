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
// ==== p68_pdec.v: decoder predecode of one instruction buffer entry ==================================================
//
// What it is for. The decode stage (DS, p68_decode.v) has one group decoder per family of instructions (dec_move.v,
// dec_alu.v and so on, twenty-odd of them). Each decoder looks at the opcode word, decides whether the opcode is one of
// its instructions with a legal effective address (its "hit"), and gives a uop. DS ORs all the uops together, and only
// the decoder that hits may contribute, so each uop has to be gated by its hit. The hit is the deep part of a decoder
// (it checks every addressing mode the instruction allows), and on top of that DS needs the OR of all hits to know
// whether the instruction is legal at all. In the timing reports that chain, opcode word to hit to gate to OR to the DR
// register, was the deepest path into DR.
//
// So this module runs the same decoders one clock earlier, when fetch writes the instruction into DS's instruction
// buffer (p68_ib.vh). It keeps only their gates (ug, the "my uop counts" bit) and the OR of their hits, 20 bits, and DS
// stores them in the entry (field IB_PQ). When the instruction reaches the head of the buffer, DS gates the decoders'
// uops with these registered bits instead of with the decoders' own legality logic. p68_decode.v has two instances, one
// for each entry fetch can write per clock (xe0 and xe1).
//
// It also computes the data value selects of the entry (sx, field IB_SX): for each of the three 32-bit data fields of
// the uop (IMM, LA_DISP, SA_DISP) which extension word holds the value and how to extend it (function dsx in
// p68_dec.vh). DS then turns the data codes into values with selects that start from registers.
//
// Inputs and clocking. This module has no clock; it is pure logic between fetch's extractor outputs and the buffer's
// write port in p68_decode.v, all on the CPU clock. Its inputs are the entry's five words and the committed S and M bits
// of the status register (supervisor and master mode), which some decoders need (a privileged instruction in user mode
// decodes into a privilege violation). Using the values at write time is safe because every change of S or M redirects
// the pipeline and so empties the buffer: no entry is ever decoded with an old S or M.
//
// How it was checked. The predecoded gates were compared with the old decoders' own gating over all 65,536 opcodes with
// several sets of extension words before the old path was removed. The decoders' uop outputs are left unconnected here.
// Gowin's synthesizer prunes every gate that drives only an unconnected output, so the extra decoder copies cost only
// the logic behind the hits and gates.
//
// Output order of pq (the index each decoder's gate has in IB_PQ, which p68_decode.v reads as pqe[n]):
//   19 any hit (the instruction is legal)    18:17 dec_fpu {hh, h}    16 dec_sys    15 dec_md    14 dec_addx
//   13 dec_shiftm   12 dec_bit   11 dec_scc   10 dec_ext   9 dec_unary   8 dec_imm   7 dec_flow   6 dec_movem
//    5 dec_shift     4 dec_lea    3 dec_branch 2 dec_quick  1 dec_alu     0 dec_move
// Decoders that give no uop of their own (dec_rox, dec_pack, dec_movep, dec_bf, dec_p7: they only start sequences or are
// gated inside DS differently) appear only in the "any hit" bit.
`include "p68_uop.vh"
`default_nettype none
module p68_pdec (
  input  wire [15:0] w0, w1, w2, w3, w4,     // the entry's words: opcode w0 and up to four extension words
  input  wire        sup, msp,               // SR bits S (supervisor) and M (master), committed values
  output wire [19:0] pq,                     // decoder gates and the legality bit, order above
  output wire [14:0] sx               // data value selects {SA_DISP, LA_DISP, IMM} (function dsx, p68_dec.vh)
);
`include "p68_dec.vh"                        // shared decode functions (dsx and the EA helpers the decoders use)
assign sx = dsx(w0, w1, sup);                // the three value selects from the opcode, its first extension word and S
wire [`UW-1:0] nu;                                   // unused uops
wire [3:0] nc; wire [63:0] np; wire [1:0] nx; wire [44:0] nsp; wire [2:0] nsc; wire nr; wire [3:0] nl;   // further unused names (never connected)
// h_*: the hit of each group decoder (the opcode is one of its instructions with a legal EA)
wire h_move, h_alu, h_quick, h_branch, h_lea, h_shift, h_movem, h_flow, h_rox, h_imm, h_unary, h_ext, h_scc, h_bit;
wire h_shiftm, h_addx, h_pack, h_movep, h_bf, h_p7, h_md, h_fpu, h_sys;
// g_*: the gate of each decoder's uop (the decoder's uop counts in DS's OR); dec_fpu has two
wire g_move, g_alu, g_quick, g_branch, g_lea, g_shift, g_movem, g_flow, g_imm, g_unary, g_ext, g_scc, g_bit;
wire g_shiftm, g_addx, g_md, g_sys; wire [1:0] g_fpu;
// The decoder instances. ug is a decoder's gate (its hit, computed by its own logic). pg is the gate input that DS
// feeds from the entry to switch the decoder's uop on; it is tied to 0 here because the uop output is not used.
// Outputs u, seq, spar and xs are left open.
dec_move   p_move   (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .w4(w4), .hit(h_move), .pg(1'b0), .ug(g_move), .u(), .seq(), .spar());
dec_alu    p_alu    (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_alu),    .pg(1'b0), .ug(g_alu),    .u());
dec_quick  p_quick  (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_quick),  .pg(1'b0), .ug(g_quick),  .u());
dec_branch p_branch (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_branch), .pg(1'b0), .ug(g_branch), .u());
dec_lea    p_lea    (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_lea),    .pg(1'b0), .ug(g_lea),    .u());
dec_shift  p_shift  (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_shift),  .pg(1'b0), .ug(g_shift),  .u());
dec_movem  p_movem  (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_movem),  .pg(1'b0), .ug(g_movem),  .u(), .seq(), .spar());
dec_flow   p_flow   (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_flow),   .pg(1'b0), .ug(g_flow),   .u(), .seq(), .spar());
dec_rox    p_rox    (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_rox), .seq(), .spar());      // ROXL/ROXR register count: hit only
dec_imm    p_imm    (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .w4(w4), .hit(h_imm), .pg(1'b0), .ug(g_imm), .u(), .xs());
dec_unary  p_unary  (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_unary),  .pg(1'b0), .ug(g_unary),  .u());
dec_ext    p_ext    (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_ext),    .pg(1'b0), .ug(g_ext),    .u(), .seq(), .spar());
dec_scc    p_scc    (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_scc),    .pg(1'b0), .ug(g_scc),    .u());
dec_bit    p_bit    (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_bit),    .pg(1'b0), .ug(g_bit),    .u(), .xs());
dec_shiftm p_shiftm (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_shiftm), .pg(1'b0), .ug(g_shiftm), .u());
dec_addx   p_addx   (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_addx),   .pg(1'b0), .ug(g_addx),   .u(), .seq(), .spar());
dec_pack   p_pack   (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_pack),  .seq(), .spar());       // PACK/UNPK: sequence only
dec_movep  p_movep  (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_movep), .seq(), .spar());       // MOVEP: sequence only
dec_bf     p_bf     (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_bf), .seq(), .spar());          // bit fields: sequence only
dec_p7     p_p7     (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_p7), .u(), .seq(), .spar());    // CAS, CAS2 and other long forms
dec_md     p_md     (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .hit(h_md), .pg(1'b0), .ug(g_md), .u(), .xs(), .seq(), .spar());
// The FPU decoder exists only in a build with the FPU; without it, line-F coprocessor opcodes have no hit and DS
// raises the line-F exception for them.
generate if (`P68_FPU != 0) begin : gq_fpu
dec_fpu    p_fpu    (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .sup(sup), .nul(1'b0), .en(1'b0), .pv(6'd0), .hit(h_fpu), .pg(2'b00), .ug(g_fpu),
                     .u(), .xs(), .seq(), .spar(), .lov(), .lnew());   // FPU state inputs tied off: the gates do not depend on them
end else begin : gq_nofpu
assign h_fpu = 1'b0; assign g_fpu = 2'b00;   // no FPU: no hit, no gate
end endgenerate
// dec_sys gets the FPU hit because it turns every line-F opcode that dec_fpu does not claim into the line-F exception.
dec_sys    p_sys    (.w0(w0), .w1(w1), .w2(w2), .w3(w3), .sup(sup), .msp(msp), .fpu_hit(h_fpu), .hit(h_sys), .pg(1'b0), .ug(g_sys),
                     .u(), .seq(), .spar(), .is_rst());
// Legality: the opcode is legal when any decoder claims it. No hit at all means DS raises the illegal-instruction
// exception (vector 4) for this entry.
wire h_any = h_move | h_alu | h_quick | h_branch | h_lea | h_shift | h_movem | h_sys | h_imm | h_unary | h_ext | h_scc | h_bit |
             h_shiftm | h_addx | h_pack | h_movep | h_rox | h_md | h_flow | h_bf | h_p7 | h_fpu;
// Packed in the order p68_decode.v reads (pqe[0] = dec_move ... pqe[19] = legal)
assign pq = {h_any, g_fpu, g_sys, g_md, g_addx, g_shiftm, g_bit, g_scc, g_ext, g_unary, g_imm, g_flow, g_movem, g_shift, g_lea,
             g_branch, g_quick, g_alu, g_move};
endmodule
`default_nettype wire
