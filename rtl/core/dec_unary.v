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
// dec_unary: the group decoder for the one-operand instructions NEGX, CLR, NEG, NOT, TST and TAS, and for NOP
// (line 4).
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v): it sees the opcode word w0 and the
// words behind it, says whether the opcode is its own (hit), and gives one micro-op (uop, p68_uop.vh) that DS ORs with
// the other decoders' uops. The uop runs through DR (register stage before address generation), AG (address
// generation), OC (operand cycle), EX (execute) and WB (write-back). Combinational logic only.
//
// The instructions.
//
//     0100 kind size EA mode EA reg       kind (bits 11-8): 0000 NEGX  0010 CLR  0100 NEG  0110 NOT  1010 TST
//     0100 1010 11 EA mode EA reg         TAS (test and set; this is TST's encoding with size 11)
//     0100 1110 0111 0001                 NOP (0x4E71)
//
// NEGX computes 0 - operand - X, with Z only cleared (as ADDX); NEG computes 0 - operand; NOT inverts every bit; CLR
// writes zero; TST only sets N and Z from the operand; TAS sets N and Z from the byte and then sets its bit 7 (on the
// real chip in one locked bus cycle). The destination is Dn or alterable memory; TST may also read An (word and long),
// the PC-relative modes and an immediate on the 68020.
//
// How they become uops. Each is one uop, chosen so that the existing ALU operations do the work with operand A as a
// constant: NEG and NEGX are the adder with B inverted (OP_NEG, OP_NEGX), NOT is EOR with all ones, CLR is a move of
// the immediate 0, TAS is OR with 0x80 with flags from the old value (ZM_OLD), and TST is a move with no write. Memory
// forms read, compute and store back to the same address (SA_SAME); CLR also reads first, as the 68000 does.
// NOP is legal but has no work at all: its uop gate is off, so it becomes an empty uop that only retires the instruction.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_unary (
  input  wire [15:0] w0, w1, w2, w3,         // opcode and EA extension words
  output wire        hit,                    // one of the instructions above with a legal EA
  input  wire        pg,             // predecoded gate (this decoder's ug from p68_pdec, a register in the entry)
  output wire        ug,             // the gate of the uop, for p68_pdec's own copy of this decoder
  output wire [`UW-1:0] u                    // the uop (zero unless pg)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
wire [2:0] m3 = w0[5:3], r0 = w0[2:0];                        // EA mode and register
wire       hu = p2_unary(w0), ht = p2_tas(w0), hn = p2_nop(w0);   // legality (p68_dec.vh)
wire [3:0] k  = w0[11:8];                                     // kind field
// The uop counts only with the hit, so TAS and TST are told apart by their opcode bits alone (size 11 = TAS)
wire       htc = (w0[7:6] == 2'b11);
wire [1:0] sz = htc ? `SZ_B : w0[7:6];                        // TAS is a byte operation
wire       tst = !htc && (k == 4'hA);                         // TST: no write
wire [38:0] src = ea_srcc(m3, r0, sz, 2'd1);                  // TST's operand as a source (it may be An or #imm)
reg  [`UW-1:0] v;
always @* begin
  v = {`UW{1'b0}};
  v[`F_UNIT] = `UN_ALU; v[`F_SZ] = sz; v[`F_LSZ] = sz;
  v[`F_LA] = ea_lac(m3, r0, sz, 2'd1, 4'd2);                  // memory operand read (every kind reads first)
  v[`F_FEN] = `FL_NZVC; v[`F_FCV] = `CV_CLR;                  // default: N, Z from the result, V and C cleared
  v[`F_ASEL] = `AS_IMM;                                       // operand A: a constant (0 unless set below)
  if (htc) begin v[`F_OP] = `OP_OR; v[`F_IMM] = dcon(32'h80); v[`F_FZM] = `ZM_OLD; end   // TAS: B | 0x80, flags from old B
  else case (k)
    4'h0: begin v[`F_OP] = `OP_NEGX; v[`F_FEN] = `FL_ALL; v[`F_FCV] = `CV_ADD; v[`F_FZM] = `ZM_KEEP; end   // 0 - B - X
    4'h2: begin v[`F_OP] = `OP_MOV; end                                        // CLR: move the immediate 0
    4'h4: begin v[`F_OP] = `OP_NEG; v[`F_FEN] = `FL_ALL; v[`F_FCV] = `CV_ADD; end   // 0 - B
    4'h6: begin v[`F_OP] = `OP_EOR; v[`F_IMM] = dcon(32'hFFFFFFFF); end        // NOT: B ^ all ones
    default: begin                                               // TST: flags from the source, no write
      v[`F_OP] = `OP_MOV; v[`F_ASEL] = src[38:37]; v[`F_AREG] = src[36:32]; v[`F_IMM] = src[31:0];
    end
  endcase
  if (!tst) begin                                             // destination: Dn, or store back to memory
    if (m3 == 3'd0) begin v[`F_BREG] = {2'b00, r0}; v[`F_WEN] = 1'b1; v[`F_WREG] = {2'b00, r0}; end
    else begin v[`F_BSEL] = `BS_MEM; v[`F_SA_EN] = 1'b1; v[`F_SA_SAME] = 1'b1; end
  end
end
assign hit = hu || ht || hn;
// The uop is gated by pg, this decoder's gate predecoded with the instruction buffer entry (p68_pdec). NOP has no gate,
// so its uop is empty.
assign ug   = (hu || ht) && !hn;
assign u   = pg ? v : {`UW{1'b0}};
endmodule
`default_nettype wire
