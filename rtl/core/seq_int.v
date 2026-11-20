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
// seq_int: the sequence uops of the smaller integer sequences: EXG, the memory-to-memory forms of ADDX/SUBX/ABCD/SBCD and
// CMPM, PACK/UNPK, MOVEP, and ROXL/ROXR with a register count (classes SQ_EXG, SQ_MM, SQ_PK, SQ_MVP, SQ_ROX).
//
// Where it sits. Instantiated by the sequencer p68_seq.v inside the decode stage (DS). It is combinational: a uop
// (micro-op, p68_uop.vh) is a function of class, step (1..n; step 0 is the decoder's empty uop) and the parameters
// latched at the start (layouts in dec_ext.v, dec_addx.v, dec_pack.v, dec_movep.v, dec_rox.v). p68_seq ORs the uop
// with the other sub-sequencers' uops, which are zero for classes that are not theirs, and the merged uop goes on
// through DR (register stage before address generation), AG (address generation), OC (operand cycle), EX (execute) and
// WB (write-back).
//
// The temporary register T0 (register 16 of the register file, invisible to programs) carries intermediate values
// between steps; WB writes it like any other register.
// Replay: every step can be restarted alone; steps read only T0 and registers earlier steps committed. That is why EXG
// copies the first register to T0 before it overwrites anything, and why every load of a memory form lands in T0
// first.
//
// The sequences (A = operand A, B = operand B of the uop):
//   SQ_EXG  1 T0 = r1 (a LEA in AG, with r1 as an index register, so a D or an A register works)  2 r1 = r2  3 r2 = T0
//   SQ_MM   1 T0 = load source (with its (An)+ / -(An) update)
//           2 load destination (with its update), A = T0, B = the loaded value, store the result back (CMPM: no store)
//   SQ_PK   register form: 1 T0 = pack(Dx + adj) or unpack(Dx) + adj   2 Dy = T0 (byte for PACK, word for UNPK)
//           memory form:   1 load the source with predecrement (PACK: a word; UNPK: a byte, unpacked + adj into T0)
//                          2 store with predecrement (PACK: pack(T0 + adj) as a byte; UNPK: T0 as a word)
//   SQ_MVP  register to memory: one byte of Dn per step, highest byte first, at d16(An), d16+2(An), ...
//           memory to register: byte 1 into T0, then each next byte shifted in (OP_CMB), the last one into Dn
//   SQ_ROX  1 T0 = the count reduced (mod 64, then mod size + 1) by OP_ROXC   2 rotate Dy through X by T0
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module seq_int (
  input  wire [3:0]  cls,                    // the running sequence class
  input  wire [4:0]  step,                   // the current step (1..n)
  input  wire [3:0]  sm1, sc4, sc2,      // step - 1, 4 - step, 2 - step as registers (p68_seq), no adders on the uop
  input  wire [63:0] sp,                     // the parameters latched at the start
  output reg  [`UW-1:0] u,                   // the uop of this step (zero for other classes)
  output reg         last,                   // this step is the last
  input  wire        last_fp,            // seq_fpu's last (a register), given out for class SQ_FPU
  input  wire        ts,                 // register bases equal to {01, tr} become T1 (p68_fx main part)
  input  wire [2:0]  tr
);
`include "p68_dec.vh"
reg  [1:0]  sz;                              // operand size
reg  [3:0]  stp;                             // (An)+ / -(An) step in bytes
reg  [2:0]  rg;                              // address register of this step
reg  [31:0] dsp;                             // displacement (-step for -(An), else 0)
reg  [7:0]  udl;                             // address register update delta
reg         pre;                             // predecrement form
reg  [4:0]  r1, r2;                          // EXG's two registers
reg  [31:0] d16;                             // sign-extended 16-bit adjustment or displacement

// uop field helpers (macros, so every tool sees plain assignments to u)
// LD_AN: a load from disp(An) with an optional update of An; ST_AN: a store to disp(An) with an optional update;
// MOV_R: a register-to-register move through the ALU (B is the merge source for .B and .W results)
`define LD_AN(an, disp, uen, ud, lsz) begin u[`F_LA_EN] = 1'b1; u[`F_LA_BSEL] = `BA_REG; u[`F_LA_BREG] = tbs({2'b01, an}, ts, tr); u[`F_LA_DISP] = disp; u[`F_LA_UEN] = uen; u[`F_LA_UDL] = ud; u[`F_LSZ] = lsz; end
`define ST_AN(an, disp, uen, ud) begin u[`F_SA_EN] = 1'b1; u[`F_SA_BSEL] = `BA_REG; u[`F_SA_BREG] = tbs({2'b01, an}, ts, tr); u[`F_SA_DISP] = disp; u[`F_SA_UEN] = uen; u[`F_SA_UDL] = ud; end
`define MOV_R(wr, ar, s, br) begin u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_MOV; u[`F_SZ] = s; u[`F_ASEL] = `AS_REG; u[`F_AREG] = ar; u[`F_BREG] = br; u[`F_WEN] = 1'b1; u[`F_WREG] = wr; end

always @* begin
  u = {`UW{1'b0}};
  last = 1'b1;                                         // for no class: last (p68_seq ignores it then)
  sz = `SZ_L; stp = 4'd0; rg = 3'd0; dsp = 32'h0; udl = 8'h0; pre = 1'b0; r1 = 5'd0; r2 = 5'd0; d16 = 32'h0;
  case (cls)
  `SQ_EXG: begin                                       // r1 <-> r2 through T0; step 1 reads r1 as an index register
    r1 = sp[4:0]; r2 = sp[9:5];
    last = (step == 5'd3);
    if (step == 5'd1) begin                            // T0 = 0 + r1.L (a LEA: AG writes the address into T0)
      u[`F_LA_XEN] = 1'b1; u[`F_LA_XREG] = {1'b0, r1[3:0]}; u[`F_LA_XL] = 1'b1;
      u[`F_LEA] = 1'b1; u[`F_SA_BREG] = `R_T0;
    end else if (step == 5'd2) `MOV_R(r1, r2, `SZ_L, 5'd0)     // r1 = r2
    else if (step == 5'd3) `MOV_R(r2, `R_T0, `SZ_L, 5'd0)      // r2 = T0 (the old r1)
  end
  `SQ_MM: begin                                        // memory to memory: load source to T0, operate on destination
    sz  = sp[6:5]; pre = sp[18];
    rg  = (step == 5'd1) ? sp[21:19] : sp[24:22];      // step 1: source Ay, step 2: destination Ax
    stp = stepb(sz, rg);
    dsp = pre ? (32'h0 - {28'h0, stp}) : 32'h0;        // -(An) reads at An - step, (An)+ at An
    udl = pre ? (8'h0 - {4'h0, stp}) : {4'h0, stp};    // and An moves by -step or +step
    last = (step == 5'd2);
    if (step == 5'd1) begin                            // T0 = the source operand
      u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_MOV; u[`F_SZ] = `SZ_L; u[`F_ASEL] = `AS_MEM;
      u[`F_WEN] = 1'b1; u[`F_WREG] = `R_T0;
      `LD_AN(rg, dsp, 1'b1, udl, sz)
    end else if (step == 5'd2) begin                   // destination op T0, flags as the decoder chose them
      u[`F_UNIT] = `UN_ALU; u[`F_OP] = sp[4:0]; u[`F_SZ] = sz;
      u[`F_ASEL] = `AS_REG; u[`F_AREG] = `R_T0; u[`F_BSEL] = `BS_MEM;
      u[`F_FEN] = sp[11:7]; u[`F_FCV] = sp[14:12]; u[`F_FZM] = sp[16:15];
      `LD_AN(rg, dsp, 1'b1, udl, sz)
      if (sp[17]) begin u[`F_SA_EN] = 1'b1; u[`F_SA_SAME] = 1'b1; end   // store back (not CMPM)
    end
  end
  `SQ_PK: begin                                        // PACK / UNPK (sp[0] UNPK, sp[1] memory)
    d16 = {{16{sp[23]}}, sp[23:8]};                    // the adjustment
    last = (step == 5'd2);   // the memory forms take two steps as well
    if (!sp[1]) begin                                  // register form
      if (step == 5'd1) begin                          // T0 = pack(Dx + adj) or unpack(Dx) + adj
        u[`F_UNIT] = `UN_ALU; u[`F_OP] = sp[0] ? `OP_UNPK : `OP_PACK; u[`F_SZ] = `SZ_L;
        u[`F_ASEL] = `AS_IMM; u[`F_IMM] = d16; u[`F_BREG] = {2'b00, sp[4:2]};
        u[`F_WEN] = 1'b1; u[`F_WREG] = `R_T0;
      end else if (step == 5'd2) `MOV_R({2'b00, sp[7:5]}, `R_T0, sp[0] ? `SZ_W : `SZ_B, {2'b00, sp[7:5]})   // Dy.B/.W = T0
    end else if (!sp[0]) begin                         // PACK in memory: one word from the source, one byte to the destination (as the 68020)
      stp = stepb(`SZ_B, sp[7:5]);                     // the destination's byte step (2 on A7)
      dsp = 32'h0 - {28'h0, stp}; udl = 8'h0 - {4'h0, stp};
      if (step == 5'd1) begin                          // T0 = the word at -2(source), source -= 2
        u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_MOV; u[`F_SZ] = `SZ_L; u[`F_ASEL] = `AS_MEM;
        u[`F_WEN] = 1'b1; u[`F_WREG] = `R_T0; `LD_AN(sp[4:2], 32'hFFFFFFFE, 1'b1, 8'hFE, `SZ_W)
      end else if (step == 5'd2) begin                 // store pack(T0 + adj) at -(destination)
        u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_PACK; u[`F_SZ] = `SZ_B;
        u[`F_ASEL] = `AS_IMM; u[`F_IMM] = d16; u[`F_BREG] = `R_T0;
        `ST_AN(sp[7:5], dsp, 1'b1, udl)
      end
    end else begin                                     // UNPK in memory: one byte from the source, one word to the destination
      stp = stepb(`SZ_B, sp[4:2]);                     // the source's byte step (2 on A7)
      dsp = 32'h0 - {28'h0, stp}; udl = 8'h0 - {4'h0, stp};
      if (step == 5'd1) begin                          // T0 = unpack(byte at -(source)) + adj
        u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_UNPK; u[`F_SZ] = `SZ_L;
        u[`F_ASEL] = `AS_IMM; u[`F_IMM] = d16; u[`F_BSEL] = `BS_MEM;
        u[`F_WEN] = 1'b1; u[`F_WREG] = `R_T0; `LD_AN(sp[4:2], dsp, 1'b1, udl, `SZ_B)
      end else if (step == 5'd2) begin                 // store T0 as a word at -2(destination), destination -= 2
        u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_MOV; u[`F_SZ] = `SZ_W;
        u[`F_ASEL] = `AS_REG; u[`F_AREG] = `R_T0; `ST_AN(sp[7:5], 32'hFFFFFFFE, 1'b1, 8'hFE)
      end
    end
  end
  `SQ_MVP: begin                                       // MOVEP: sp[2:0] Dn, [5:3] An, [6] long, [7] to memory, [23:8] d16
    // the byte address is d16(An) + 2 * (step - 1); the 2 * (step - 1) part goes into AG as the constant index (load)
    // or the store adjustment, from the register sm1, so the uop needs no adder
    d16 = {{16{sp[23]}}, sp[23:8]};
    if (sp[7]) begin                                   // register to memory: one byte per step
      last = sp[6] ? (step == 5'd4) : (step == 5'd2);
      // OP_BX: result = A >> (8 * CC[1:0]); CC counts down 3..0 (long) or 1..0 (word), highest byte first
      u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_BX; u[`F_SZ] = `SZ_B;
      u[`F_CC] = sp[6] ? sc4 : sc2;
      u[`F_ASEL] = `AS_REG; u[`F_AREG] = {2'b00, sp[2:0]};
      `ST_AN(sp[5:3], d16, 1'b0, 8'h0) u[`F_SA_ADJ] = {sm1, 1'b0};
    end else begin                                     // memory to register: bytes into T0, then into Dn
      last = sp[6] ? (step == 5'd4) : (step == 5'd3);
      if (!sp[6] && step == 5'd3) `MOV_R({2'b00, sp[2:0]}, `R_T0, `SZ_W, {2'b00, sp[2:0]})   // word: Dn.W = T0
      else begin
        u[`F_UNIT] = `UN_ALU; u[`F_SZ] = `SZ_L; u[`F_ASEL] = `AS_MEM; u[`F_WEN] = 1'b1;
        `LD_AN(sp[5:3], d16, 1'b0, 8'h0, `SZ_B) u[`F_LA_XK] = {sm1[2:0], 1'b0};
        if (step == 5'd1) begin u[`F_OP] = `OP_MOV; u[`F_WREG] = `R_T0; end      // T0 = first byte
        else begin
          // OP_CMB: (B << 8) | A[7:0], B = T0; the fourth byte of a long goes straight into Dn
          u[`F_OP] = `OP_CMB; u[`F_BREG] = `R_T0;
          u[`F_WREG] = (sp[6] && step == 5'd4) ? {2'b00, sp[2:0]} : `R_T0;
        end
      end
    end
  end
  `SQ_ROX: begin                                       // ROXL/ROXR Dx,Dy: sp[2:0] Dy, [5:3] Dx (count), [7:6] size, [8] left
    last = (step == 5'd2);
    if (step == 5'd1) begin                            // T0 = reduced count in the rotator's form (CC[0] = left)
      u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_ROXC; u[`F_SZ] = sp[7:6]; u[`F_CC] = {3'b0, sp[8]};
      u[`F_ASEL] = `AS_REG; u[`F_AREG] = {2'b00, sp[5:3]}; u[`F_WEN] = 1'b1; u[`F_WREG] = `R_T0;
    end else begin                                     // rotate Dy through X by T0: {left, rotate = 1, arithmetic = 1}
      u[`F_UNIT] = `UN_SHF; u[`F_OP] = `OP_SHF | {2'b00, sp[8], 2'b11}; u[`F_SZ] = sp[7:6];
      u[`F_ASEL] = `AS_REG; u[`F_AREG] = `R_T0; u[`F_BREG] = {2'b00, sp[2:0]};
      u[`F_WEN] = 1'b1; u[`F_WREG] = {2'b00, sp[2:0]}; u[`F_FEN] = `FL_ALL; u[`F_FCV] = `CV_SHF;
    end
  end
  4'd14: last = last_fp;                // SQ_FPU, so p68_seq's last mux keeps its shape
  default: ;
  endcase
end
endmodule
`undef LD_AN
`undef ST_AN
`undef MOV_R
`default_nettype wire
