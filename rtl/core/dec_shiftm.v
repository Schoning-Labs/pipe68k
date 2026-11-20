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
// dec_shiftm: the group decoder for the memory shifts and rotates ASL, ASR, LSL, LSR, ROXL, ROXR, ROL, ROR <ea>
// (line E, bits 7-6 = 11, bit 11 = 0).
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v): it sees the opcode word w0 and the
// words behind it, says whether the opcode is its own (hit), and gives one micro-op (uop, p68_uop.vh) that DS ORs with
// the other decoders' uops. The uop runs through DR (register stage before address generation), AG (address
// generation), OC (operand cycle), EX (execute) and WB (write-back). Combinational logic only.
//
// The instruction.
//
//     1110 0 type dir 11 EA mode EA reg       type (bits 10-9): 00 AS, 01 LS, 10 ROX, 11 RO; dir: 0 right, 1 left
//
// The memory forms always shift a word by exactly one bit, and the EA must be alterable memory. Line E with bit 11 set
// and size 11 are the bit field instructions.
//
// How it becomes a uop. One uop, a read-modify-write: LA reads the word, the rotator in EX shifts it by one, and the
// result is stored to the same address (SA_SAME). The operation bits for the rotator are the same {left, rotate,
// arithmetic} as in dec_shift.v, and the ROX count is again given as the rotator wants it (left amount less one: 0 for
// ROXL by 1, and 16 - 1 = 15 for ROXR by 1 on the 17-bit value of a word plus X).
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_shiftm (
  input  wire [15:0] w0, w1, w2, w3,         // opcode and EA extension words
  output wire        hit,                    // memory shift or rotate with an alterable memory EA
  input  wire        pg,             // predecoded gate (this decoder's ug from p68_pdec, a register in the entry)
  output wire        ug,             // the gate of the uop, for p68_pdec's own copy of this decoder
  output wire [`UW-1:0] u                    // the uop (zero unless pg)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
wire [2:0] m3 = w0[5:3], r0 = w0[2:0];                        // EA mode and register
wire       h  = p2_mshift(w0);                                // legality (p68_dec.vh)
wire [1:0] t  = w0[10:9];                              // 00 AS, 01 LS, 10 ROX, 11 RO
reg  [`UW-1:0] v;
always @* begin
  v = {`UW{1'b0}};
  // rotator operation {left, rotate, arithmetic}; always a word, read and written at the same size
  v[`F_UNIT] = `UN_SHF; v[`F_OP] = `OP_SHF | {2'b00, w0[8], t[1], !t[0]}; v[`F_SZ] = `SZ_W; v[`F_LSZ] = `SZ_W;
  v[`F_ASEL] = `AS_IMM; v[`F_IMM] = dcon((t == 2'b10) ? (w0[8] ? 32'd0 : 32'd15) : 32'd1);   // ROXx: rotator count L-1
  v[`F_LA] = ea_lac(m3, r0, `SZ_W, 2'd1, 4'd2);                // read the word
  v[`F_BSEL] = `BS_MEM; v[`F_SA_EN] = 1'b1; v[`F_SA_SAME] = 1'b1;   // B = loaded word; store back to the same address
  v[`F_FEN] = {t != 2'b11, 4'b1111}; v[`F_FCV] = `CV_SHF;     // ROL/ROR leave X
end
assign hit = h;
// The uop is gated by pg, this decoder's gate predecoded with the instruction buffer entry (p68_pdec).
assign ug   = h;
assign u   = pg ? v : {`UW{1'b0}};
endmodule
`default_nettype wire
