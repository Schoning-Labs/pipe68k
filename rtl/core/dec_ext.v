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
// dec_ext: the group decoder for sign extension (EXT.W, EXT.L, EXTB.L), SWAP and EXG.
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v): it sees the opcode word w0, says
// whether the opcode is its own (hit), and gives one micro-op (uop, p68_uop.vh) that DS ORs with the other decoders'
// uops, or starts a sequence run by the sequencer (p68_seq.v with seq_int.v). The uops run through DR (register stage
// before address generation), AG (address generation), OC (operand cycle), EX (execute) and WB (write-back).
// Combinational logic only.
//
// The instructions (all one word, all on registers):
//
//     EXT.W  Dn   0100 1000 1000 0 Dn   sign-extend the low byte to a word         (0x4880)
//     EXT.L  Dn   0100 1000 1100 0 Dn   sign-extend the low word to a long         (0x48C0)
//     EXTB.L Dn   0100 1001 1100 0 Dn   sign-extend the low byte to a long (68020) (0x49C0)
//     SWAP   Dn   0100 1000 0100 0 Dn   exchange the two 16-bit halves             (0x4840)
//     EXG         1100 Rx 1 opmode Ry   exchange two registers; opmode 01000 Dx,Dy  01001 Ax,Ay  10001 Dx,Ay
//
// All but EXG set N and Z from the result and clear V and C.
//
// How they become uops. EXT is a move through the ALU with the operand extension of the uop (AEXT: byte or word sign
// extension in front of EX) and the right result size. SWAP is a rotate by 16 in the rotator. EXG writes two registers,
// and a uop has only one register write, so it is a three-step sequence of class SQ_EXG through the temporary T0 (step
// 0, this decoder's uop, is empty; see seq_int.v). EXG parameters: [4:0] first register, [9:5] second register (D 0-7,
// A 8-15).
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_ext (
  input  wire [15:0] w0, w1, w2, w3,         // opcode word (the others are not used)
  output wire        hit,                    // EXT, EXTB, SWAP or EXG
  input  wire        pg,             // predecoded gate (this decoder's ug from p68_pdec, a register in the entry)
  output wire        ug,             // the gate of the uop, for p68_pdec's own copy of this decoder
  output wire [`UW-1:0] u,                   // the uop (zero unless pg; empty for EXG)
  output wire [3:0]  seq,                    // SQ_EXG for EXG
  output wire [63:0] spar                    // EXG parameters
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
wire [2:0] r0 = w0[2:0], r9 = w0[11:9];                       // Dn (or Ry), Rx
wire       he = p2_ext(w0), hx = p2_exg(w0);                  // EXT/EXTB/SWAP and EXG legality (p68_dec.vh)
wire       swp = (w0[15:3] == 13'b0100100001000);             // SWAP
reg  [`UW-1:0] v;
always @* begin
  v = {`UW{1'b0}};
  v[`F_BREG] = {2'b00, r0}; v[`F_WEN] = 1'b1; v[`F_WREG] = {2'b00, r0};   // Dn is the merge source and the destination
  v[`F_ASEL] = `AS_REG; v[`F_AREG] = {2'b00, r0};                        // operand A = Dn
  v[`F_FEN] = `FL_NZVC; v[`F_FCV] = `CV_CLR;                             // N, Z from the result, V and C cleared
  if (swp) begin                                           // rotate by 16, C and V cleared
    // OP_SHF | 6 = left rotate (the {left, rotate, arithmetic} bits 1, 1, 0) by the immediate 16
    v[`F_UNIT] = `UN_SHF; v[`F_OP] = `OP_SHF | 5'd6; v[`F_SZ] = `SZ_L; v[`F_ASEL] = `AS_IMM; v[`F_IMM] = dcon(32'd16);
  end else begin
    v[`F_UNIT] = `UN_ALU; v[`F_OP] = `OP_MOV;
    v[`F_SZ]   = w0[6] ? `SZ_L : `SZ_W;                    // EXT.W 0x4880, EXT.L 0x48C0, EXTB.L 0x49C0
    // bits 8-6: 010 EXT.W (byte to long, word result keeps the upper word), 011 EXT.L (word), 111 EXTB.L (byte)
    v[`F_AEXT] = (w0[8:6] == 3'b010) ? `AX_SXB : (w0[8:6] == 3'b011) ? `AX_SXW : `AX_SXB;
  end
end
// EXG register numbers: the Rx field is an address register only for Ax,Ay; the Ry field is a data register only for
// Dx,Dy (for Ax,Ay and Dx,Ay it is An)
wire [4:0] ra = (w0[7:3] == 5'b01001) ? {2'b01, r9} : {2'b00, r9};
wire [4:0] rb = (w0[7:3] == 5'b01000) ? {2'b00, r0} : {2'b01, r0};
assign hit  = he || hx;
// The uop is gated by pg, this decoder's gate predecoded with the instruction buffer entry (p68_pdec). EXG has no gate:
// its step 0 is empty.
assign ug   = he;
assign u    = pg ? v : {`UW{1'b0}};
assign seq  = hx ? `SQ_EXG : `SQ_NONE;
assign spar = hx ? {54'h0, rb, ra} : 64'h0;
endmodule
`default_nettype wire
