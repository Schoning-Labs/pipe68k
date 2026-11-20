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
// dec_bf: the group decoder for the 68020 bit field instructions BFTST, BFEXTU, BFCHG, BFEXTS, BFCLR, BFFFO, BFSET and
// BFINS (opcodes E8C0-EFFF).
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v). Bit field instructions always run as
// sequences, so this decoder gives no uop at all, only a hit, a sequence class and the parameters. The sequencer
// (p68_seq.v with seq_bf.v) then issues the uops, which use the bit field unit p68_bf.v in EX. The uops run through DR
// (register stage before address generation), AG (address generation), OC (operand cycle), EX (execute) and WB
// (write-back). Combinational logic only.
//
// The instructions. A bit field is a run of 1 to 32 bits starting at any bit offset in a data register or in memory.
//
//     opcode     1110 1 type 11 EA mode EA reg      type (bits 10-8): 0 BFTST 1 BFEXTU 2 BFCHG 3 BFEXTS
//                                                                      4 BFCLR 5 BFFFO  6 BFSET 7 BFINS
//     extension  0 reg | Do | offset | Dw | width   reg (bits 14-12): the data register for EXTU/EXTS/FFO/INS
//                                                   Do (bit 11): offset is in a register (bits 8-6) or immediate 0-31
//                                                   Dw (bit 5):  width is in a register (bits 2-0) or immediate (0 = 32)
//
// In memory the offset is a signed bit number counted from the most significant bit of the byte at the EA, so a field
// can start far before or after that byte and can span five bytes. BFTST, BFEXTU, BFEXTS and BFFFO only read, so they
// also allow the PC-relative modes; the others write the field and need an alterable EA. EA extension words come after
// the bit field extension word.
//
// How they become uops. Step 0 is empty and the sequence class is SQ_BFR for a data register (mode 0) and SQ_BFM for
// memory. Parameters (spar):
//   [15:0] extension word, [18:16] type (opcode bits 10-8), [21:19] EA mode, [24:22] EA register,
//   [63:48] first and [47:32] second EA extension word (the words after the bit field extension word).
// The sequences themselves are explained in seq_bf.v.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_bf (
  input  wire [15:0] w0, w1, w2, w3,         // opcode, bit field extension word, EA extension words
  output wire        hit,                    // a bit field opcode with a legal EA
  output wire [3:0]  seq,                    // SQ_BFR (register) or SQ_BFM (memory)
  output wire [63:0] spar                    // sequence parameters (layout above)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
`include "p68_bff.vh"
wire h = bf_ok(w0);                          // legality, shared with the fetch predecode (p68_bff.vh)
assign hit  = h;
assign seq  = h ? ((w0[5:3] == 3'd0) ? `SQ_BFR : `SQ_BFM) : `SQ_NONE;   // EA mode 0 = Dn
// The parameters are read only for a legal sequence of this decoder (its class is exact), so they are gated by the
// opcode pattern alone; it excludes every other decoder's sequence opcodes.
assign spar = (w0[15:11] == 5'b11101 && w0[7:6] == 2'b11) ? {w2, w3, 7'h0, w0[2:0], w0[5:3], w0[10:8], w1} : 64'h0;
endmodule
`default_nettype wire
