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
// dec_bit: the group decoder for the single-bit instructions BTST, BCHG, BCLR and BSET (line 0).
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v): it sees the opcode word w0 and the
// words behind it, says whether the opcode is its own (hit), and gives one micro-op (uop, p68_uop.vh) that DS ORs with
// the other decoders' uops. The uop runs through DR (register stage before address generation), AG (address
// generation), OC (operand cycle), EX (execute) and WB (write-back). Combinational logic only.
//
// The instructions. Each tests one bit and sets Z to the inverse of it (Z = 1 when the bit was 0); BCHG then inverts
// the bit, BCLR clears it and BSET sets it. No other flag changes. Two encodings:
//
//     dynamic  0000 Dn 1 type EA mode EA reg              bit number in data register Dn
//     static   0000 1000 type EA mode EA reg  + word      bit number in the low byte of the next word
//
// type (bits 7-6): 00 BTST, 01 BCHG, 10 BCLR, 11 BSET. With a data register as destination the operation is on all 32
// bits and the bit number is taken modulo 32; in memory it is a byte operation and the bit number is modulo 8.
// BTST may also read the PC-relative modes (a read-only test). The dynamic form with EA mode 1 is MOVEP, not a bit
// instruction, and BTST Dn,#imm is decoded by dec_p7.
//
// How they become uops. One uop in the ALU's extra unit (OP_BTST + type). The bit number is operand A (register or
// immediate), the operand is B (register, or the value loaded from memory), and for the changing forms the result goes
// back to the register or is stored to the same address (SA_SAME). For the static form the EA extension words come
// after the bit number word, which moves them one word later (xs tells DS where the index register field of a
// (d8,An,Xn) extension word is).
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_bit (
  input  wire [15:0] w0, w1, w2, w3,         // opcode, bit number word (static form), EA extension words
  output wire        hit,                    // dynamic or static bit instruction with a legal EA
  input  wire        pg,             // predecoded gate (this decoder's ug from p68_pdec, a register in the entry)
  output wire        ug,             // the gate of the uop, for p68_pdec's own copy of this decoder
  output wire [`UW-1:0] u,                   // the uop (zero unless pg)
  output wire [1:0]  xs                      // word (1 = w2) holding the first EA extension word
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
wire [2:0] m3 = w0[5:3], r0 = w0[2:0], r9 = w0[11:9];         // EA mode and register; bit number register (dynamic)
wire       hd = p2_bdyn(w0), hs = p2_bsta(w0);                // legality of the dynamic and static forms (p68_dec.vh)
wire [1:0] tt = w0[7:6];                                      // 00 BTST, 01 BCHG, 10 BCLR, 11 BSET
wire [1:0] sz = (m3 == 3'd0) ? `SZ_L : `SZ_B;                 // register: long, mod 32; memory: byte, mod 8
// The uop counts only with the hit, so the static form is told apart by its opcode byte alone (cheaper than hs)
wire        hsc = (w0[15:8] == 8'h08);
reg  [`UW-1:0] v;
always @* begin
  v = {`UW{1'b0}};
  v[`F_UNIT] = `UN_ALU; v[`F_OP] = `OP_BTST + {3'b000, tt}; v[`F_SZ] = sz; v[`F_LSZ] = sz;   // OP_BTST..OP_BSET
  v[`F_FEN] = 5'b00100; v[`F_FCV] = `CV_CLR; v[`F_FZM] = `ZM_BIT;   // only Z changes, set from the tested bit
  if (hsc) begin v[`F_ASEL] = `AS_IMM; v[`F_IMM] = dcode(3'd3, 2'd1, 17'h0); end   // bit number = low byte of w1
  else begin v[`F_ASEL] = `AS_REG; v[`F_AREG] = {2'b00, r9}; end                  // bit number in Dn
  if (m3 == 3'd0) begin                                                           // operand in a data register
    v[`F_BREG] = {2'b00, r0}; v[`F_WEN] = (tt != 2'b00); v[`F_WREG] = {2'b00, r0};   // BTST writes nothing
  end else begin                                                                  // operand in memory (a byte)
    // the EA's first extension word is w2 behind the static form's bit number word (byte offset 4), else w1 (offset 2)
    v[`F_LA] = ea_lac(m3, r0, `SZ_B, hsc ? 2'd2 : 2'd1, hsc ? 4'd4 : 4'd2);
    v[`F_BSEL] = `BS_MEM;
    if (tt != 2'b00) begin v[`F_SA_EN] = 1'b1; v[`F_SA_SAME] = 1'b1; end         // change forms store back
  end
end
assign hit = hd || hs;
// The uop is gated by pg, this decoder's gate predecoded with the instruction buffer entry (p68_pdec).
assign ug   = hd || hs;
assign u   = pg ? v : {`UW{1'b0}};
assign xs  = hsc ? 2'd1 : 2'd0;                                   // read only with a hit (index register field)
endmodule
`default_nettype wire
