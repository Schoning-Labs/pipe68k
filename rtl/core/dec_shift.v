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
// dec_shift: the group decoder for shifts and rotates of a data register (line E, size not 11): ASL, ASR, LSL, LSR,
// ROL, ROR, and ROXL/ROXR with an immediate count.
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v): it sees the opcode word w0, says
// whether the opcode is its own (hit), and gives one micro-op (uop, p68_uop.vh) that DS ORs with the other decoders'
// uops. The uop runs through DR (register stage before address generation), AG (address generation), OC (operand
// cycle), EX (execute) and WB (write-back). Combinational logic only.
//
// The instruction.
//
//     1110 | count/reg | dir | size | i/r | type | Dy
//     15-12  11-9        8     7-6    5     4-3    2-0
//
// dir: 0 right, 1 left. size: 00 byte, 01 word, 10 long (11 is the memory form, dec_shiftm, or a bit field
// instruction). i/r: 0 means bits 11-9 are an immediate count 1-8 (0 encodes 8), 1 means they name a data register Dx
// that holds the count (taken modulo 64). type: 00 AS (arithmetic), 01 LS (logical), 10 ROX (rotate through X),
// 11 RO (rotate). Rotates leave the X flag alone; shifts and ROX set X to the last bit shifted out.
//
// How it becomes a uop. One uop for the rotator unit (UN_SHF) in EX. The operation code tells the rotator three things
// in its low bits: left, rotate, arithmetic. ROX is encoded as "rotate and arithmetic both set". The rotator wants a
// ROX count in a different form, as the left rotate amount less one, because it rotates a 33-bit or 17-bit or 9-bit
// value (the operand plus X) with an ordinary 32-bit or 64-bit rotate. For an immediate count that conversion is a
// constant computed here. ROXL/ROXR with a register count needs the count reduced modulo (size + 1) at run time, which
// takes an extra uop, so that form is a sequence in dec_rox.v and this decoder leaves it out.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_shift (
  input  wire [15:0] w0, w1, w2, w3,         // opcode word (the other words are not used by this group)
  output wire        hit,                    // a register shift or rotate other than ROXx with a register count
  input  wire        pg,             // predecoded gate (this decoder's ug from p68_pdec, a register in the entry)
  output wire        ug,             // the gate of the uop, for p68_pdec's own copy of this decoder
  output wire [`UW-1:0] u                    // the uop (zero unless pg)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
wire [2:0] r9 = w0[11:9], r0 = w0[2:0];                       // count (or count register) and the data register Dy
wire       rox = (w0[4:3] == 2'b10);                          // type ROX
wire       h   = (w0[15:12] == 4'hE) && (w0[7:6] != 2'b11) && !(rox && w0[5]);   // ROXx with a register count: dec_rox (sequence)
wire [5:0] kq  = (r9 == 3'd0) ? 6'd8 : {3'b0, r9};            // immediate count 1-8
wire [5:0] nn  = (w0[7:6] == 2'd0) ? 6'd8 : (w0[7:6] == 2'd1) ? 6'd16 : 6'd32;   // operand width in bits
// ROX count as the rotator wants it (p68_ex: left rotate amount less one): ROXL by k is k - 1, ROXR by k is the same as
// a left rotate of the (n+1)-bit value by n + 1 - k, so n - k
wire [5:0] kl  = kq - 6'd1, kr = nn - kq;
wire       rot = (w0[4:3] == 2'b11);               // ROL/ROR (leave X); ROXx = rotate bit and arithmetic bit both set
reg  [`UW-1:0] v;
always @* begin
  v = {`UW{1'b0}};
  // OP_SHF plus {left = bit 8, rotate = bit 4, arithmetic = !bit 3}: AS 0,1  LS 0,0  ROX 1,1  RO 1,0
  v[`F_UNIT] = `UN_SHF; v[`F_OP] = `OP_SHF | {2'b00, w0[8], w0[4], !w0[3]}; v[`F_SZ] = w0[7:6];
  v[`F_ASEL] = w0[5] ? `AS_REG : `AS_IMM; v[`F_AREG] = {2'b00, r9};   // count from Dx, or the immediate below
  v[`F_IMM]  = dcon((r9 == 3'd0) ? 32'd8 : {29'h0, r9});              // immediate count 1-8 (0 encodes 8)
  if (rox) v[`F_IMM] = dcon({26'h0, w0[8] ? kl : kr});                // ROX: the converted count instead
  v[`F_BREG] = {2'b00, r0}; v[`F_WEN] = 1'b1; v[`F_WREG] = {2'b00, r0};   // Dy is the operand and the destination
  v[`F_FEN]  = {!rot, 4'b1111}; v[`F_FCV] = `CV_SHF;      // rotates leave X; C and V from the rotator
end
assign hit = h;
// The uop is gated by pg, this decoder's gate predecoded with the instruction buffer entry (p68_pdec).
assign ug   = h;
assign u   = pg ? v : {`UW{1'b0}};
endmodule
`default_nettype wire
