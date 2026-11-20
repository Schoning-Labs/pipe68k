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
// dec_pack: the group decoder for the 68020 BCD conversion instructions PACK and UNPK (line 8).
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v). PACK and UNPK always run as
// sequences, so this decoder gives no uop, only a hit, the sequence class SQ_PK and the parameters; the sequencer
// (p68_seq.v with seq_int.v) issues the uops, which run through DR (register stage before address generation), AG
// (address generation), OC (operand cycle), EX (execute) and WB (write-back). Combinational logic only.
//
// The instructions.
//
//     PACK  1000 Ry 10100 m Rx  + 16-bit adjustment      UNPK  1000 Ry 11000 m Rx  + 16-bit adjustment
//
// m (bit 3) = 0: data registers, Dx to Dy; m = 1: memory, -(Ax) to -(Ay). The manual calls the source Dx/Ax (bits 2-0)
// and the destination Dy/Ay (bits 11-9). PACK adds the adjustment to a word that holds two digits in its low nibbles
// (for example two ASCII digits 0x3132) and packs the two low nibbles into one byte (0x12). UNPK does the reverse:
// it spreads the two nibbles of a byte into the low nibbles of a word and adds the adjustment (0x12 + 0x3030 = 0x3132).
// No flags change.
//
// How they become uops. A two-step sequence: step 1 computes the converted value into the temporary T0 (from Dx, or
// from a value loaded with -(Ax)), step 2 writes it into Dy or stores it with -(Ay) (see seq_int.v). Step 0, this
// decoder's uop, is empty. Parameters: [0] UNPK, [1] memory form, [4:2] source register, [7:5] destination register,
// [23:8] adjustment.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_pack (
  input  wire [15:0] w0, w1, w2, w3,         // opcode and adjustment word
  output wire        hit,                    // PACK or UNPK
  output wire [3:0]  seq,                    // SQ_PK
  output wire [63:0] spar                    // sequence parameters (layout above)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
wire h = p2_pk(w0);                          // legality, shared with the fetch predecode (p68_dec.vh)
assign hit  = h;
assign seq  = h ? `SQ_PK : `SQ_NONE;
// {adjustment, destination Ry, source Rx, memory form (bit 3), UNPK (bit 7: opmode 11000 against PACK's 10100)}
assign spar = h ? {40'h0, w1, w0[11:9], w0[2:0], w0[3], w0[7]} : 64'h0;
endmodule
`default_nettype wire
