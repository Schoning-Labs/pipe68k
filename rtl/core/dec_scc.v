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
// dec_scc: the group decoder for Scc (set on condition) and DBcc (decrement and branch), both in line 5 with size 11.
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v): it sees the opcode word w0 and the
// words behind it, says whether the opcode is its own (hit), and gives one micro-op (uop, p68_uop.vh) that DS ORs with
// the other decoders' uops. The uop runs through DR (register stage before address generation), AG (address
// generation), OC (operand cycle), EX (execute) and WB (write-back). Combinational logic only.
//
// The instructions.
//
//     Scc   <ea>        0101 cond 11 EA mode EA reg        EA mode 1 is DBcc, mode 7 register 2-4 is TRAPcc
//     DBcc  Dn,label    0101 cond 11 001 Dn  + 16-bit displacement
//
// Scc writes 0xFF to a byte if the condition holds and 0x00 if not; it changes no flags. DBcc is the 68k's loop
// instruction: if the condition holds it does nothing more (the loop ends). Otherwise it decrements the low word of Dn
// and branches to PC + 2 + displacement unless Dn.W became -1. The condition is often F (false), so DBF / DBRA simply
// counts Dn down.
//
// How they become uops. Scc is one uop in the ALU's extra unit (OP_SCC): EX evaluates the condition and gives 0xFF or
// 0x00, written to Dn's low byte or stored to memory (the memory form reads first and stores to the same address, as
// on the 68000). DBcc is one uop of the unit UN_DBR: the ALU subtracts 1 from Dn.W, the register is written unless the
// condition holds, and EX redirects to the target when the condition is false and Dn.W was not 0. The target comes from
// the load address descriptor, computed in AG as for Bcc. A backward DBcc (the normal loop) is predicted taken: AG
// redirects to the target right away, and EX corrects to the next instruction when the loop ends. DBcc also updates the
// branch target cache (BTC) when it commits, so later passes can be predicted at fetch.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_scc (
  input  wire [15:0] w0, w1, w2, w3,         // opcode and the next words (displacement or EA extension words)
  output wire        hit,                    // Scc with a legal EA, or DBcc
  input  wire        pg,             // predecoded gate (this decoder's ug from p68_pdec, a register in the entry)
  output wire        ug,             // the gate of the uop, for p68_pdec's own copy of this decoder
  output wire [`UW-1:0] u                    // the uop (zero unless pg)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
wire [2:0] m3 = w0[5:3], r0 = w0[2:0];                        // EA mode and register (DBcc: mode 001, Dn)
wire       hs = p2_scc(w0), hd = p2_dbcc(w0);                 // legality (p68_dec.vh)
reg  [`UW-1:0] v;
always @* begin
  v = {`UW{1'b0}};
  v[`F_CC] = w0[11:8];                                        // the condition, tested in EX
  v[`F_ASEL] = `AS_IMM;                                       // operand A: the constant 1 (DBcc) or unused (Scc)
  if (hd) begin                                               // DBcc
    v[`F_UNIT] = `UN_DBR; v[`F_OP] = `OP_SUB; v[`F_SZ] = `SZ_W; v[`F_IMM] = dcon(32'd1);   // Dn.W - 1
    v[`F_BREG] = {2'b00, r0}; v[`F_WEN] = 1'b1; v[`F_WREG] = {2'b00, r0};               // Dn (not written if cond)
    v[`F_LA_BSEL] = `BA_PC; v[`F_LA_XK] = 4'd2; v[`F_LA_DISP] = dcode(3'd1, 2'd1, 17'h0);   // target PC + 2 + sext(w1)
    if (w1[15]) begin v[`F_BRA] = 1'b1; v[`F_PRED] = 1'b1; end   // backward DBcc predicted taken (loops)
    v[`F_BR] = 1'b1;                                               // DBcc updates the branch target cache
  end else begin                                              // Scc: byte result 0xFF / 0x00 from the condition
    v[`F_UNIT] = `UN_ALU; v[`F_OP] = `OP_SCC; v[`F_SZ] = `SZ_B; v[`F_LSZ] = `SZ_B;
    v[`F_LA] = ea_lac(m3, r0, `SZ_B, 2'd1, 4'd2);
    if (m3 == 3'd0) begin v[`F_BREG] = {2'b00, r0}; v[`F_WEN] = 1'b1; v[`F_WREG] = {2'b00, r0}; end   // Dn low byte
    else begin v[`F_BSEL] = `BS_MEM; v[`F_SA_EN] = 1'b1; v[`F_SA_SAME] = 1'b1; end                   // memory byte
  end
end
assign hit = hs || hd;
// The uop is gated by pg, this decoder's gate predecoded with the instruction buffer entry (p68_pdec).
assign ug   = hs || hd;
assign u   = pg ? v : {`UW{1'b0}};
endmodule
`default_nettype wire
