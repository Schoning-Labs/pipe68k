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
// dec_imm: the group decoder for the immediate instructions ORI, ANDI, SUBI, ADDI, EORI and CMPI (line 0).
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v): it sees the opcode word w0 and the
// words behind it, says whether the opcode is its own (hit), and gives one micro-op (uop, p68_uop.vh) that DS ORs with
// the other decoders' uops. The uop runs through DR (register stage before address generation), AG (address
// generation), OC (operand cycle), EX (execute) and WB (write-back). Combinational logic only.
//
// The instructions.
//
//     0000 | op | 0 | size | EA mode | EA reg        then the immediate (one word for .B and .W, two for .L),
//     15-12  11-9  8   7-6                          then the EA extension words
//
// op: 000 ORI, 001 ANDI, 010 SUBI, 011 ADDI, 101 EORI, 110 CMPI (100 is the static bit group, 111 is MOVES). Size 11
// is CHK2/CMP2/CAS, and bit 8 set is the dynamic bit group or MOVEP. A byte immediate sits in the low byte of its word.
// The destination is Dn or alterable memory; on the 68020 CMPI may also read the PC-relative modes. ORI/ANDI/EORI with
// mode 7 register 4 (the "immediate" EA) mean "to CCR" or "to SR" and are decoded by dec_sys. The longest form,
// ADDI.L #imm,abs.L, is five words, which is why this decoder takes w4.
//
// How they become uops. One uop: operand A is the immediate, operand B is Dn or the loaded memory value, and the result
// goes to Dn or is stored to the same address (SA_SAME). CMPI only sets flags and never writes. ADDI and SUBI set all
// five flags, CMPI N Z V C, the logic forms N and Z with V and C cleared.
// Because the immediate sits between the opcode and the EA's extension words, the EA's first extension word is w2 (or
// w3 behind a long immediate). The xs output tells DS that word, for the index register field of a (d8,An,Xn) mode.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_imm (
  input  wire [15:0] w0, w1, w2, w3, w4,     // opcode, immediate word(s), EA extension words (up to five words)
  output wire        hit,                    // ORI..CMPI with a legal EA
  input  wire        pg,             // predecoded gate (this decoder's ug from p68_pdec, a register in the entry)
  output wire        ug,             // the gate of the uop, for p68_pdec's own copy of this decoder
  output wire [`UW-1:0] u,                   // the uop (zero unless pg)
  output wire [1:0]  xs                      // word (1 = w2, 2 = w3) that holds the first EA extension word
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
wire [2:0] m3 = w0[5:3], r0 = w0[2:0];                        // EA mode and register
wire [1:0] sz = w0[7:6];                                      // 00 byte, 01 word, 10 long
wire       lg = (sz == `SZ_L);                                // long immediate (two words)
wire       h  = p2_imm(w0);                                   // legality, shared with the fetch predecode (p68_dec.vh)
// immediate as a data code: {w1, w2} for .L, zero-extended w1 for .W, zero-extended low byte of w1 for .B
wire [31:0] imm = dcode(lg ? 3'd4 : (sz == `SZ_W) ? 3'd2 : 3'd3, 2'd1, 17'h0);
wire [2:0]  o  = w0[11:9];                                    // operation field
wire [4:0]  op = (o == 3'd0) ? `OP_OR : (o == 3'd1) ? `OP_AND : (o == 3'd2) ? `OP_SUB : (o == 3'd3) ? `OP_ADD :
                 (o == 3'd5) ? `OP_EOR : `OP_CMP;
wire        cmp = (o == 3'd6), arith = (o == 3'd2) || (o == 3'd3) || cmp;   // CMPI; adder operations
reg  [`UW-1:0] v;
always @* begin
  v = {`UW{1'b0}};
  v[`F_UNIT] = `UN_ALU; v[`F_OP] = op; v[`F_SZ] = sz; v[`F_LSZ] = sz;
  v[`F_ASEL] = `AS_IMM; v[`F_IMM] = imm;                      // operand A = the immediate
  v[`F_FEN] = (o == 3'd2 || o == 3'd3) ? `FL_ALL : `FL_NZVC;  // SUBI/ADDI also set X
  v[`F_FCV] = arith ? `CV_ADD : `CV_CLR;
  if (m3 == 3'd0) begin                                       // Dn: B = Dn, result to Dn (not for CMPI)
    v[`F_BREG] = {2'b00, r0}; v[`F_WEN] = !cmp; v[`F_WREG] = {2'b00, r0};
  end else begin                                              // memory: B = the loaded value
    // the EA's first extension word is w3 behind a long immediate (byte offset 6), else w2 (offset 4)
    v[`F_LA] = ea_lac(m3, r0, sz, lg ? 2'd3 : 2'd2, lg ? 4'd6 : 4'd4);      // PC-relative (CMPI): offset of the extension word
    v[`F_BSEL] = `BS_MEM;
    if (!cmp) begin v[`F_SA_EN] = 1'b1; v[`F_SA_SAME] = 1'b1; end         // store the result back
  end
end
assign hit = h;
// The uop is gated by pg, this decoder's gate predecoded with the instruction buffer entry (p68_pdec).
assign ug   = h;
assign u   = pg ? v : {`UW{1'b0}};
assign xs  = h ? (lg ? 2'd2 : 2'd1) : 2'd0;
endmodule
`default_nettype wire
