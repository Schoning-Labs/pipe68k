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
// dec_alu: the group decoder for the two-operand arithmetic and logic instructions ADD, SUB, AND, OR, EOR, CMP and the
// address forms ADDA, SUBA, CMPA (opcode lines 8, 9, B, C and D).
//
// Where it sits. This is one of about twenty group decoders in the decode stage (DS, p68_decode.v). Every decoder sees
// the opcode word w0 and the words behind it (w1-w3) in the same clock, says whether the opcode is one of its own (hit)
// and gives one micro-op (uop, the work order of p68_uop.vh). DS ORs all decoders' uops; a decoder that does not match
// gives zeros. The uop then goes through DR (the register stage in front of address generation), AG (address
// generation), OC (operand cycle: data cache read), EX (execute) and WB (write-back). The module is combinational logic
// only (LUTs, no clock).
//
// The instructions. The five lines share one layout:
//
//     15-12 | 11-9    | 8   | 7-6  | 5-3     | 2-0
//     line  | Dn      | dir | size | EA mode | EA reg
//
// line 8 = OR, 9 = SUB, B = CMP/EOR, C = AND, D = ADD. Bit 8 (dir) chooses the direction: 0 means <ea>,Dn (the data
// register is the destination), 1 means Dn,<ea> (memory is the destination, a read-modify-write). Line B is special:
// with dir = 0 it is CMP <ea>,Dn, with dir = 1 it is EOR Dn,<ea> (there is no CMP to memory and no EOR <ea>,Dn).
// Size 11 is not a size: in lines 9, B and D it means the address forms SUBA, CMPA, ADDA, with bit 8 giving word (0)
// or long (1); in lines 8 and C it is DIVU/DIVS and MULU/MULS, which dec_md decodes. Several encodings that look like
// Dn,<ea> with mode 0 or 1 belong to other instructions (ADDX, SUBX, ABCD, SBCD, CMPM, EXG, PACK, UNPK), so the hit
// leaves them out.
//
// How they become uops. Each form is a single uop:
//   <ea>,Dn     LA reads the source if it is in memory, EX computes Dn op src, WB writes Dn (CMP writes nothing).
//   Dn,<ea>     with a memory EA: LA reads the old value, EX computes it op Dn, and the store goes to the same address
//               (SA_SAME), all in one uop. With a register EA (EOR Dn,Dm) it is a plain register operation.
//   ADDA etc.   always a full 32-bit operation on An, a word source sign-extended first, no flags (CMPA sets flags).
// Flags: ADD and SUB set all five flags (X too); CMP sets N Z V C; the logic operations set N and Z and clear V and C.
//
// Register numbers in LA_BREG and SA_BREG are replaced by DR with the numbers from the opcode word, and the data fields
// are small codes that DS turns into values once (see dec_move.v and p68_dec.vh for both).
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_alu (
  input  wire [15:0] w0, w1, w2, w3,         // opcode word and the next three instruction words
  output wire        hit,                    // the opcode is a legal instruction of this group
  input  wire        pg,             // predecoded gate (this decoder's ug, computed by p68_pdec and kept with the entry)
  output wire        ug,             // the gate of the uop, for p68_pdec's own copy of this decoder
  output wire [`UW-1:0] u                    // the uop (zero unless pg)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
// ---- opcode fields ------------------------------------------------------------------------------------------------
wire [3:0] g  = w0[15:12];                                    // opcode line
wire [2:0] r9 = w0[11:9], m3 = w0[5:3], r0 = w0[2:0];         // Dn (or An), EA mode, EA register
wire       grp  = (g == 4'h8 || g == 4'h9 || g == 4'hB || g == 4'hC || g == 4'hD);   // one of the five lines
wire       isa  = (w0[7:6] == 2'b11);                     // ADDA/SUBA/CMPA
wire       dir  = w0[8];                                  // 1: Dn,<ea> (result to the EA)
wire [1:0] sz   = isa ? (w0[8] ? `SZ_L : `SZ_W) : w0[7:6];   // address forms: bit 8 is the size; else bits 7-6
// operation by line; line B is CMP (also CMPA) unless it is EOR (dir set, not an address form)
wire [4:0] op   = (g == 4'h8) ? `OP_OR : (g == 4'h9) ? `OP_SUB : (g == 4'hC) ? `OP_AND : (g == 4'hD) ? `OP_ADD :
                  ((w0[8] && !isa) ? `OP_EOR : `OP_CMP);
wire       arith = (op == `OP_ADD) || (op == `OP_SUB) || (op == `OP_CMP);   // C and V come from the adder
wire [38:0] src = ea_srcc(m3, r0, sz, 2'd1);              // operand A of the EA as source: {ASEL, AREG, IMM code}
wire       srcok = (m3 != 3'd7) || (r0 <= 3'd4);                // mode 7 register 5-7 is not an EA
wire       dstok = (m3 != 3'd7) || (r0 <= 3'd1);                // PC-relative and immediate are not alterable
// ---- legality ---------------------------------------------------------------------------------------------------
// Address forms: only lines 9, B, D (lines 8 and C with size 11 are divide and multiply) with a valid source EA.
// Other forms: Dn,<ea> may not have An as destination, nor Dn except for EOR (those encodings are ADDX/SUBX/ABCD/
// SBCD/CMPM/EXG/PACK/UNPK); the destination must be alterable or the source valid; and An may not be the source of a
// byte operation or of AND/OR.
wire       h = grp && (isa ? ((g == 4'h9 || g == 4'hB || g == 4'hD) && srcok)
                           : !(dir && (m3 == 3'd1 || (m3 == 3'd0 && g != 4'hB))) && (dir ? dstok : srcok)
                             && !(!dir && m3 == 3'd1 && (w0[7:6] == 2'b00 || g == 4'h8 || g == 4'hC)));   // An as source of a byte op, or of AND/OR, is illegal
// ---- the uop ----------------------------------------------------------------------------------------------------
reg  [`UW-1:0] v;
always @* begin
  v = {`UW{1'b0}};
  v[`F_UNIT] = `UN_ALU; v[`F_OP] = op; v[`F_SZ] = sz; v[`F_LSZ] = sz;   // one ALU operation in EX
  v[`F_FEN] = (op == `OP_ADD || op == `OP_SUB) ? `FL_ALL : `FL_NZVC;   // ADD/SUB also set X; CMP and logic do not
  v[`F_FCV] = arith ? `CV_ADD : `CV_CLR;                                // carry and overflow from the adder, or cleared
  if (isa) begin                                          // ADDA, SUBA, CMPA <ea>,An
    v[`F_LA] = ea_lac(m3, r0, sz, 2'd1, 4'd2);            // source read (first extension word is word 1, 2 bytes in)
    v[`F_ASEL] = src[38:37]; v[`F_AREG] = src[36:32]; v[`F_IMM] = src[31:0];
    v[`F_AEXT] = (sz == `SZ_W) ? `AX_SXW : `AX_NONE; v[`F_SZ] = `SZ_L;   // .W source sign-extended, the operation is 32-bit
    v[`F_BREG] = {2'b01, r9}; v[`F_WREG] = {2'b01, r9}; v[`F_WEN] = (op != `OP_CMP);   // An op src -> An (CMPA: no write)
    if (op != `OP_CMP) v[`F_FEN] = `FL_NONE;              // ADDA and SUBA change no flags
  end else if (!dir) begin                                // <ea>,Dn
    v[`F_LA] = ea_lac(m3, r0, sz, 2'd1, 4'd2);
    v[`F_ASEL] = src[38:37]; v[`F_AREG] = src[36:32]; v[`F_IMM] = src[31:0];
    // B = Dn is both the other operand and the merge source that keeps Dn's upper bits for .B and .W
    v[`F_BREG] = {2'b00, r9}; v[`F_WREG] = {2'b00, r9}; v[`F_WEN] = (op != `OP_CMP);
  end else begin                                          // Dn,<ea>
    v[`F_ASEL] = `AS_REG; v[`F_AREG] = {2'b00, r9};       // operand A = Dn
    // EOR Dn,Dm: the destination is a register
    if (m3 == 3'd0) begin v[`F_BREG] = {2'b00, r0}; v[`F_WEN] = 1'b1; v[`F_WREG] = {2'b00, r0}; end
    else begin                                            // memory destination: read, operate, store to the same address
      v[`F_LA] = ea_lac(m3, r0, sz, 2'd1, 4'd2);
      v[`F_BSEL] = `BS_MEM; v[`F_SA_EN] = 1'b1; v[`F_SA_SAME] = 1'b1;   // B = the loaded value; store at the load address
    end
  end
end
assign hit = h;
// The uop is gated by pg, this decoder's gate predecoded with the instruction buffer entry (p68_pdec), so the legality
// logic above is not in the path from the instruction words to DS's output.
assign ug   = h;
assign u   = pg ? v : {`UW{1'b0}};
endmodule
`default_nettype wire
