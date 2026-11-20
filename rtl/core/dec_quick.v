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
// dec_quick: the group decoder for MOVEQ (line 7) and ADDQ/SUBQ (line 5).
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v). It sees the opcode word w0 (and the
// following words, which these instructions use only for the EA), says whether the opcode is its own (hit), and gives
// one micro-op (uop, format in p68_uop.vh) that DS ORs with the other decoders' uops. The uop then runs through DR
// (register stage before address generation), AG (address generation), OC (operand cycle), EX (execute) and WB
// (write-back). Combinational logic only.
//
// The instructions. These are the "quick" forms, with a small constant inside the opcode word itself, so they need no
// immediate extension word. They are very common in compiled code (loop counters, pointer steps, clearing registers).
//
//     MOVEQ #d8,Dn     0111 | Dn  | 0 | 8-bit signed data         the byte is sign-extended to 32 bits, always .L
//     ADDQ  #q,<ea>    0101 | q   | 0 | size | EA mode | EA reg   q = 1-8 (the field value 0 means 8)
//     SUBQ  #q,<ea>    0101 | q   | 1 | size | EA mode | EA reg
//
// Line 5 with size 11 is Scc, DBcc or TRAPcc (dec_scc, dec_sys). ADDQ/SUBQ to an address register always works on all
// 32 bits and leaves the flags alone, as ADDA does; a byte operation on An is illegal.
//
// How they become uops. All three are a single uop: MOVEQ is a move of an immediate into Dn with N and Z set; ADDQ and
// SUBQ add or subtract the constant from a register, or read memory, operate, and store to the same address (SA_SAME).
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_quick (
  input  wire [15:0] w0, w1, w2, w3,         // opcode word and the next three instruction words
  output wire        hit,                    // MOVEQ, ADDQ or SUBQ with a legal EA
  input  wire        pg,             // predecoded gate (this decoder's ug from p68_pdec, a register in the entry)
  output wire        ug,             // the gate of the uop, for p68_pdec's own copy of this decoder
  output wire [`UW-1:0] u                    // the uop (zero unless pg)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
wire [3:0] g  = w0[15:12];                                    // opcode line
wire [2:0] r9 = w0[11:9], m3 = w0[5:3], r0 = w0[2:0];         // Dn of MOVEQ or q of ADDQ/SUBQ; EA mode and register
wire       moveq = (g == 4'h7) && !w0[8];                     // bit 8 must be 0 for MOVEQ
// ADDQ/SUBQ: not size 11, no byte operation on An, and of mode 7 only abs.W and abs.L (alterable destinations)
wire       addq  = (g == 4'h5) && (w0[7:6] != 2'b11) && !(w0[7:6] == 2'b00 && m3 == 3'd1) && (m3 != 3'd7 || r0 <= 3'd1);   // ADDQ.B/SUBQ.B to An is illegal
wire [1:0] sz = w0[7:6];                                      // 00 byte, 01 word, 10 long
wire       h = moveq || addq;
reg  [`UW-1:0] v;
always @* begin
  v = {`UW{1'b0}};
  v[`F_UNIT] = `UN_ALU; v[`F_ASEL] = `AS_IMM;                 // operand A is always the constant
  if (moveq) begin
    // sign-extended 8-bit data; it fits the 9-bit constant of a data code, so no instruction word is needed
    v[`F_OP] = `OP_MOV; v[`F_SZ] = `SZ_L; v[`F_IMM] = dcon({{24{w0[7]}}, w0[7:0]});
    v[`F_WEN] = 1'b1; v[`F_WREG] = {2'b00, r9}; v[`F_FEN] = `FL_NZVC; v[`F_FCV] = `CV_CLR;   // Dn; N, Z set, V, C cleared
  end else begin
    v[`F_OP] = w0[8] ? `OP_SUB : `OP_ADD; v[`F_SZ] = sz; v[`F_LSZ] = sz;   // bit 8: 0 ADDQ, 1 SUBQ
    v[`F_IMM] = dcon((r9 == 3'd0) ? 32'd8 : {29'h0, r9});   // q = 1-7, and 0 encodes 8
    v[`F_FEN] = `FL_ALL; v[`F_FCV] = `CV_ADD;               // all five flags from the adder
    if (m3 <= 3'd1) begin                                   // Dn or An: register operation (m3[0] picks A0-A7)
      v[`F_BREG] = {1'b0, m3[0], r0}; v[`F_WEN] = 1'b1; v[`F_WREG] = {1'b0, m3[0], r0};
      if (m3[0]) begin v[`F_FEN] = `FL_NONE; v[`F_SZ] = `SZ_L; end   // ADDQ to An: long, no flags
    end else begin                                          // memory: read, add, store back to the same address
      v[`F_LA] = ea_lac(m3, r0, sz, 2'd1, 4'd2);
      v[`F_BSEL] = `BS_MEM; v[`F_SA_EN] = 1'b1; v[`F_SA_SAME] = 1'b1;
    end
  end
end
assign hit = h;
// The uop is gated by pg, this decoder's gate predecoded with the instruction buffer entry (p68_pdec).
assign ug   = h;
assign u   = pg ? v : {`UW{1'b0}};
endmodule
`default_nettype wire
