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
// dec_movep: the group decoder for MOVEP (move peripheral data, line 0).
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v). MOVEP always runs as a sequence, so
// this decoder gives no uop, only a hit, the sequence class SQ_MVP and the parameters; the sequencer (p68_seq.v with
// seq_int.v) issues the uops, which then run through DR (register stage before address generation), AG (address
// generation), OC (operand cycle), EX (execute) and WB (write-back). Combinational logic only.
//
// The instruction.
//
//     0000 Dn 1 opmode 001 An  + 16-bit displacement      opmode: 100 .W mem->reg  101 .L mem->reg
//                                                                 110 .W reg->mem  111 .L reg->mem
//
// MOVEP moves a word or long between a data register and every other byte in memory, starting at (d16,An): byte 0 at
// d16, byte 1 at d16 + 2, and so on. It was made for 8-bit peripheral chips wired to one half of the 68000's data bus.
// It is rare in Amiga code, but it is part of the instruction set, so it is in hardware as a short sequence of byte
// accesses (2 or 4 steps, one byte each, plus one step to merge into Dn for the word load).
// Parameters: [2:0] Dn, [5:3] An, [6] long, [7] register to memory, [23:8] displacement.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_movep (
  input  wire [15:0] w0, w1, w2, w3,         // opcode and displacement word
  output wire        hit,                    // MOVEP
  output wire [3:0]  seq,                    // SQ_MVP
  output wire [63:0] spar                    // sequence parameters (layout above)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
wire h = p2_movep(w0);                       // line 0, bit 8 set, EA mode field 001 (p68_dec.vh)
assign hit  = h;
assign seq  = h ? `SQ_MVP : `SQ_NONE;
assign spar = h ? {40'h0, w1, w0[7], w0[6], w0[2:0], w0[11:9]} : 64'h0;   // {d16, to memory, long, An, Dn}
endmodule
`default_nettype wire
