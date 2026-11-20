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
// dec_move: the group decoder for MOVE and MOVEA (opcode lines 1, 2 and 3).
//
// Where it sits. The decode stage (DS, p68_decode.v) has one small decoder per group of 68k instructions, and this is
// the one for MOVE, by far the most common instruction in real 68k code. All decoders look at the same instruction
// words at the same time: w0 is the opcode word and w1-w4 are the words behind it (extension words). Each decoder says
// whether the opcode is one of its own (hit) and gives a complete micro-op (uop, the fixed-format work order described
// in p68_uop.vh that travels down the pipeline). DS ORs the uops of all decoders together, which only works because a
// decoder that does not match gives an all-zero uop. If no decoder hits, the opcode is illegal and raises vector 4.
// The result goes through the DR queue (the decode register stage in front of address generation) to AG (address
// generation), OC (operand cycle: the data cache read), EX (execute) and WB (write-back). The decoder is pure
// combinational logic: no clock, no registers, only LUTs.
//
// The instruction. MOVE copies a source operand to a destination operand and sets the flags N and Z from the value
// (V and C cleared, X unchanged). Its opcode word is laid out like this:
//
//     15 14 | 13 12 | 11 10 9  | 8 7 6    | 5 4 3    | 2 1 0
//     0  0  | size  | dst reg  | dst mode | src mode | src reg
//
// The size field is unusual: 01 = byte (line 1), 11 = word (line 3), 10 = long (line 2), and 00 is not MOVE at all
// (line 0 holds the immediate and bit instructions). The destination is encoded with register and mode swapped compared
// to the source, which is why the code below reads the destination mode from bits 8-6 and its register from bits 11-9.
// MOVEA is MOVE with destination mode 1 (An): it writes a whole address register, sign-extends a word source to 32 bits
// and changes no flags. There is no MOVEA.B.
//
// The effective address (EA) modes in the 3-bit mode field, used by every decoder in this package:
//     0 Dn             data register direct
//     1 An             address register direct
//     2 (An)           memory at the address in An
//     3 (An)+          the same, then An is increased by the operand size
//     4 -(An)          An is decreased by the operand size first, then memory at An
//     5 (d16,An)       An plus a signed 16-bit displacement (one extension word)
//     6 (d8,An,Xn)     An plus an 8-bit displacement plus an index register (one "brief" extension word; the 68020
//                      also has a "full" format of up to 5 more words, handled by the pre-sequence p68_fx.v)
//     7 with the register field selecting a further mode: 0 abs.W, 1 abs.L, 2 (d16,PC), 3 (d8,PC,Xn), 4 #immediate
//
// How MOVE becomes uops. Almost every MOVE is a single uop: the load address descriptor (LA) describes the source
// operand in memory if there is one, the EX operation is a plain move through the ALU, and the destination is either a
// register write from WB or a store described by the store address descriptor (SA). A uop can carry one load and one
// store, so even a memory-to-memory MOVE such as MOVE.L (A0)+,(A1)+ is one uop and one clock in each stage.
// The one exception is a destination with an index register, (d8,An,Xn): the store address path in AG has no index
// adder (only the load address path has one), so that form becomes a two-uop sequence of class SQ_X. This uop moves the
// source into the temporary register T0 and sets the flags, and step 1 (in seq_cas.v) stores T0 through the load
// address path with "store at the load address" (SA_SAME). Sequence parameters: [1:0] 0 (the SQ_X kind), [3:2] size,
// [6:4] the destination An, [22:7] the destination's extension word.
//
// Register numbers. The base register numbers this decoder writes into LA_BREG and SA_BREG are not used as they are:
// DS leaves them out of its output and the DR stage puts in the numbers straight from the opcode word (bits 2-0 for the
// load base, bits 11-9 for the store base), which are the same registers but come from fewer logic levels.
//
// Data codes. The 32-bit fields of the uop (the immediate IMM and the displacements LA_DISP and SA_DISP) are not built
// here as values. The decoder gives a small code (the dcode/dcon functions in p68_dec.vh) that names which instruction
// word holds the value and how to extend it, and DS turns the OR of all codes into values once. Twenty decoders each
// building and ORing 32-bit values was measured as a large part of the decode depth, so this was one of the steps that
// made the decode shallower (HISTORY.md, section 5).
//
// Gowin note. The module carries a maximum fanout attribute of 16 in its header, which asks the synthesizer to
// duplicate drivers of nets with more loads; the opcode bits fan out to many LUTs in every decoder.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_move (
  input  wire [15:0] w0, w1, w2, w3, w4,     // w0 opcode; w1-w4 the following words (MOVE.L #imm,abs.L has five words)
  output wire        hit,                    // the opcode is a legal MOVE/MOVEA (no hit from any decoder = illegal instruction)
  input  wire        pg,             // predecoded gate: this decoder's ug, computed by p68_pdec when the instruction
                                     // entered the instruction buffer and stored with the entry, so the gate is a register
  output wire        ug,             // the gate of the uop, for p68_pdec's own copy of this decoder (unused in DS)
  output wire [`UW-1:0] u,                   // the uop (all zero unless pg)
  output wire [3:0]  seq,                    // sequence class: SQ_X for a (d8,An,Xn) destination, else SQ_NONE
  output wire [63:0] spar                    // sequence parameters for p68_seq (layout in the file introduction)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
// ---- opcode fields --------------------------------------------------------------------------------------------------
wire [3:0] g  = w0[15:12];                                    // opcode line: 1, 2 or 3 for MOVE
wire [2:0] r9 = w0[11:9], m6 = w0[8:6], m3 = w0[5:3], r0 = w0[2:0];   // destination reg and mode, source mode and reg
// size from the opcode line: line 1 byte, line 3 word, line 2 long (the 68k's odd size order for MOVE)
wire [1:0] sz = (g == 4'h1) ? `SZ_B : (g == 4'h3) ? `SZ_W : `SZ_L;
wire [1:0] xs = eax(m3, r0, sz);                              // extension words used by the source EA (0-2)
wire [15:0] dd = (xs == 2'd0) ? w1 : (xs == 2'd1) ? w2 : w3;   // destination extension word
wire [1:0] dk = xs + 2'd1;                                    // word index of dd (1-3), for the data code of SA_DISP
// (An)+ / -(An) step in bytes of the source and of the destination (1, 2 or 4; a byte step on A7 is 2 to keep the
// stack pointer even, a 68k rule)
wire [3:0] ss = stepb(sz, r0), sd = stepb(sz, r9);
// ---- source and destination on the same address register --------------------------------------------------------
// MOVE.W (A0)+,(A0) must store at the incremented A0. Both addresses are computed in AG in the same clock from the old
// A0, so the store address gets the source's step added as an adjustment (SA_ADJ) instead of waiting for the update.
// the destination EA sees the source's (An)+ / -(An) when both use the same An
wire       same = (m3 == 3'd3 || m3 == 3'd4) && r0 == r9 && m6 >= 3'd2 && m6 <= 3'd5;
wire [4:0] adj  = !same ? 5'd0 : (m3 == 3'd3) ? {1'b0, ss} : (5'd0 - {1'b0, ss});   // +step after (An)+, -step after -(An)
wire [38:0] src = ea_srcc(m3, r0, sz, 2'd1);                  // operand A: {ASEL, AREG, IMM code} of the source EA
// ---- legality (hit) ---------------------------------------------------------------------------------------------
// Lines 1-3, a destination that is data alterable or An (modes 0-6, abs.W, abs.L; never PC-relative or immediate),
// no MOVEA.B, no byte read of an address register, and only the five defined sub-modes of mode 7 as source.
wire       h = (g == 4'h1 || g == 4'h2 || g == 4'h3) && (m6 <= 3'd6 || (m6 == 3'd7 && r9 <= 3'd1)) && !(g == 4'h1 && m6 == 3'd1)
           && !(g == 4'h1 && m3 == 3'd1)    // MOVE.B An,<ea> is illegal (the 68k has no byte access to An)
           && !(m3 == 3'd7 && r0 > 3'd4);   // source mode 7 with register 5-7 is no addressing mode
// ---- the uop ----------------------------------------------------------------------------------------------------
// One always block fills the uop field by field. Everything not set stays zero, which in the uop format means
// "not used" (no load, no store, no write, no flags).
reg  [`UW-1:0] v;
always @* begin
  v = {`UW{1'b0}};
  // EX runs a move through the ALU: the result is operand A, sized by SZ; LSZ is the memory read size
  v[`F_UNIT] = `UN_ALU; v[`F_OP] = `OP_MOV; v[`F_SZ] = sz; v[`F_LSZ] = sz;
  // load address of the source (LA_EN only for memory modes; po = 2: the first extension word is 2 bytes after the
  // opcode, which PC-relative modes need because their base is the extension word's own address)
  v[`F_LA] = ea_lac(m3, r0, sz, 2'd1, 4'd2);
  v[`F_ASEL] = src[38:37]; v[`F_AREG] = src[36:32]; v[`F_IMM] = src[31:0];   // operand A = register, memory or immediate
  // operand B is the old destination register: for .B and .W the upper bits of a data register must survive, and EX
  // merges the new low byte or word into B
  v[`F_BREG] = {2'b00, r9};
  v[`F_FEN] = `FL_NZVC; v[`F_FCV] = `CV_CLR;                  // N and Z from the result, V and C cleared, X kept
  case (m6)                                                   // destination mode
    3'd0: begin v[`F_WEN] = 1'b1; v[`F_WREG] = {2'b00, r9}; end          // Dn: register write in WB
    3'd1: begin                                                          // MOVEA to An
      v[`F_WEN] = 1'b1; v[`F_WREG] = {2'b01, r9}; v[`F_FEN] = `FL_NONE;    // register 8-15 = A0-A7; MOVEA sets no flags
      v[`F_AEXT] = (sz == `SZ_W) ? `AX_SXW : `AX_NONE; v[`F_SZ] = `SZ_L;  // a word source is sign-extended; An is written whole
    end
    // (d8,An,Xn): two-uop sequence; this uop moves the source into T0 with the flags, step 1 stores T0
    3'd6: begin v[`F_WEN] = 1'b1; v[`F_WREG] = `R_T0; end
    3'd7: begin                                                               // abs.W, abs.L destination
      v[`F_SA_EN] = 1'b1; v[`F_SA_BSEL] = `BA_ZERO;                           // store address = displacement alone
      // abs.L (r9 = 1) takes the 32 bits from two words, abs.W (r9 = 0) one sign-extended word, at word index dk
      v[`F_SA_DISP] = dcode(r9[0] ? 3'd4 : 3'd1, dk, 17'h0);
    end
    default: begin                                                            // (An), (An)+, -(An), (d16,An)
      v[`F_SA_EN] = 1'b1; v[`F_SA_BSEL] = `BA_REG; v[`F_SA_BREG] = {2'b01, r9}; v[`F_SA_ADJ] = adj;
      if (m6 == 3'd5) v[`F_SA_DISP] = dcode(3'd1, dk, 17'h0);                 // (d16,An): sign-extended word dk
      if (m6 == 3'd4) v[`F_SA_DISP] = dcon(32'h0 - {28'h0, sd});              // -(An): store at An - step
      if (m6 == 3'd3) begin v[`F_SA_UEN] = 1'b1; v[`F_SA_UDL] = {4'h0, sd}; end   // (An)+: AG writes An + step
      if (m6 == 3'd4) begin v[`F_SA_UEN] = 1'b1; v[`F_SA_UDL] = 8'h0 - {4'h0, sd}; end   // -(An): AG writes An - step
    end
  endcase
end
assign hit = h;
// Gate. The uop is ANDed with pg, this decoder's gate predecoded with the instruction buffer entry (p68_pdec runs a copy
// of every decoder when an entry is written and stores the gates). Gating with a register instead of with h takes the
// whole legality check out of the path from the instruction words to the DS output, which saved levels in the full system.
assign ug   = h;
assign u   = pg ? v : {`UW{1'b0}};
assign seq  = (h && m6 == 3'd6) ? `SQ_X : `SQ_NONE;          // (d8,An,Xn) destination: sequence
// The parameters are read only for a legal sequence of this decoder (its class is exact), so they are gated by the
// opcode pattern alone, without the EA checks; the pattern excludes every other decoder's sequence opcodes, so the OR
// of all decoders' parameters in DS stays clean.
assign spar = ((g == 4'h1 || g == 4'h2 || g == 4'h3) && m6 == 3'd6) ? {41'h0, dd, r9, sz, 2'b00} : 64'h0;
endmodule
`default_nettype wire
