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
// dec_rox: the group decoder for ROXL and ROXR with the rotate count in a data register (line E).
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v). This form always runs as a
// sequence, so the decoder gives no uop, only a hit, the sequence class SQ_ROX and the parameters; the sequencer
// (p68_seq.v with seq_int.v) issues the uops, which run through DR (register stage before address generation), AG
// (address generation), OC (operand cycle), EX (execute) and WB (write-back). Combinational logic only.
//
// The instruction.
//
//     1110 Dx dir size 1 10 Dy        rotate Dy left (dir = 1) or right (dir = 0) through X by the count in Dx
//
// ROX rotates the operand together with the X flag, so a byte is a 9-bit rotate, a word 17 bits and a long 33 bits.
// The count in Dx is taken modulo 64, and since rotating an (n+1)-bit value by n+1 changes nothing, the real rotate
// amount is that count modulo n+1. A modulo by 9, 17 or 33 is too slow to put in front of the rotator in the same
// clock, so the register form takes two steps: step 1 reduces the count (mod 64, then mod n+1) into the temporary T0
// with the ALU operation OP_ROXC, and step 2 rotates by T0. The immediate form (count 1-8) is converted to a constant in
// the decoder dec_shift.v and needs only one uop. Step 0, this decoder's uop, is empty.
// Parameters: [2:0] data register Dy, [5:3] count register Dx, [7:6] size, [8] left.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_rox (
  input  wire [15:0] w0, w1, w2, w3,         // opcode word
  output wire        hit,                    // ROXL/ROXR Dx,Dy
  output wire [3:0]  seq,                    // SQ_ROX
  output wire [63:0] spar                    // sequence parameters (layout above)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
wire h = p2_roxr(w0);                        // line E, size not 11, register count (bit 5), type ROX (p68_dec.vh)
assign hit  = h;
assign seq  = h ? `SQ_ROX : `SQ_NONE;
assign spar = h ? {55'h0, w0[8], w0[7:6], w0[11:9], w0[2:0]} : 64'h0;   // {left, size, Dx, Dy}
endmodule
`default_nettype wire
