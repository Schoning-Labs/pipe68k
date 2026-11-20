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
// seq_bf: the sequence uops of the bit field instructions (classes SQ_BFR for a data register, SQ_BFM for memory).
//
// Where it sits. Instantiated by the sequencer p68_seq.v inside the decode stage (DS). Combinational: a uop (micro-op,
// p68_uop.vh) is a function of class, step (1..n; step 0 is the decoder's empty uop) and the parameters latched at the
// start (layout in dec_bf.v). p68_seq ORs this module's uop with the other sub-sequencers' uops, and the merged uop goes
// on through DR (register stage before address generation), AG (address generation), OC (operand cycle), EX (execute,
// where the bit field unit p68_bf.v sits) and WB (write-back).
//
// The bit field unit takes operands when a uop commits and delivers 2 to 4 clocks later, so empty uops (E) separate a
// capture from its read: results at the unit are valid from the fourth clock after the capturing uop (the third clock
// for the second long, RD1; the third clock for ADDR's address). An empty uop takes one clock in every stage, so a
// fixed number of them is a fixed delay. Only loads replay; every step can restart alone, because the unit keeps what
// committed steps gave it. T0 = EA, T1 = aligned byte address, T2/T3 = the two aligned longs.
//   register form: SETUP [SETI] COMP E E E RD                     (SETI only for BFINS)
//   memory form:   1 LEA T0  2 SETUP  3 ADDR  4 SETI  5 E  6 RD0 -> T1  7 LD T2  8 LD T3  9 COMP  10 E  11 E
//                  extract: 12 E  13 RD0 -> Dn;  change/insert: 12 ST1 (second long)  13 ST0 (first long, flags)
// SETUP gives the unit the offset and width (from registers or the immediates of the extension word), SETI the value
// BFINS inserts, ADDR the EA from which the unit computes the aligned byte address, COMP the operand (the register, or
// the two longs that cover the field in memory: a field of up to 32 bits at any bit offset spans at most 5 bytes, so
// two aligned longs always hold it), and RD0/RD1 read the results back.
// The unit was first built with two barrel shifters on a 64-bit window (1,769 LUT, with a 19-level path). It was
// rewritten with one 64-bit rotator used in two passes (1,067 LUT), which cost one more empty uop in these sequences
// and left the measured performance unchanged (HISTORY.md section 6).
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module seq_bf (
  input  wire [3:0]  cls,                    // the running sequence class
  input  wire [4:0]  step,                   // the current step (1..n)
  input  wire [63:0] sp,                     // the parameters latched at the start
  output reg  [`UW-1:0] u,                   // the uop of this step (zero for other classes)
  output reg         last,                   // this step is the last
  input  wire        ts,                 // register bases equal to {01, tr} become T1 (p68_fx main part)
  input  wire [2:0]  tr
);
`include "p68_dec.vh"
wire [15:0] ext  = sp[15:0];                                            // the bit field extension word
wire [2:0]  t    = sp[18:16];                                           // type: 0 TST 1 EXTU 2 CHG 3 EXTS 4 CLR 5 FFO 6 SET 7 INS
wire [2:0]  em   = sp[21:19], er = sp[24:22];                           // EA mode and register
wire        ins  = (t == 3'd7);                                         // BFINS
wire        chg  = (t == 3'd2) || (t == 3'd4) || (t == 3'd6) || ins;     // changes the field
wire        tst  = (t == 3'd0);                                         // BFTST: flags only
wire [2:0]  dn   = ext[14:12];                                          // data register of EXTU/EXTS/FFO/INS
wire [4:0]  rd   = tst ? 5'd0 : {2'b00, chg ? er : dn};                 // destination register of the register form
wire [4:0]  v    = (!ins && step >= 5'd2 && cls == `SQ_BFR) ? step + 5'd1 : step;   // register form without SETI skips step 2

// Uop helpers. BF_SETUP_U(rf): A = offset register (ext bits 8-6), B = width register (ext bits 2-0), CC = {0, register
// form, width in a register (ext bit 5), offset in a register (ext bit 11)}, IMM = {width immediate, offset immediate}.
// BF_OP_U(o, ar, br): a bit field unit operation o with operand registers ar and br.
`define BF_SETUP_U(rf) begin u[`F_UNIT] = `UN_BFU; u[`F_OP] = `BF_SETUP; u[`F_SZ] = `SZ_L; \
  u[`F_ASEL] = `AS_REG; u[`F_AREG] = {2'b00, ext[8:6]}; u[`F_BSEL] = `BS_REG; u[`F_BREG] = {2'b00, ext[2:0]}; \
  u[`F_CC] = {1'b0, rf, ext[5], ext[11]}; u[`F_IMM] = {22'h0, ext[4:0], ext[10:6]}; end
`define BF_OP_U(o, ar, br) begin u[`F_UNIT] = `UN_BFU; u[`F_OP] = o; u[`F_SZ] = `SZ_L; u[`F_ASEL] = `AS_REG; \
  u[`F_AREG] = ar; u[`F_BREG] = br; end

always @* begin
  u = {`UW{1'b0}};
  last = 1'b1;
  if (cls == `SQ_BFR) begin                                          // field in a data register
    last = (v == 5'd7);
    case (v)
      5'd1: `BF_SETUP_U(1'b1)                                        // offset and width
      5'd2: `BF_OP_U(`BF_SETI, {2'b00, dn}, 5'd0)                    // BFINS: the value to insert
      5'd3: `BF_OP_U({2'b01, t}, {2'b00, er}, 5'd0)                  // compute (op 8 + type) on the register
      5'd7: begin                                                    // read the result after three empty steps
        `BF_OP_U(`BF_RD0, 5'd0, 5'd0)
        u[`F_WEN] = !tst; u[`F_WREG] = rd; u[`F_FEN] = `FL_NZVC;     // N, Z from the field; V, C cleared
      end
      default: ;                                                     // steps 4-6: empty
    endcase
  end else if (cls == `SQ_BFM) begin                                 // field in memory
    last = (step == 5'd13);
    case (step)
      5'd1: begin                                                    // T0 = EA
        u[`F_LA] = ea_lat(em, er, `SZ_L, sp[63:48], sp[47:32], 4'd4, ts, tr);   // po = 4: behind opcode and extension
        u[`F_LA_EN] = 1'b0; u[`F_LA_UEN] = 1'b0;
        u[`F_LEA] = 1'b1; u[`F_SA_BREG] = `R_T0;
      end
      5'd2: `BF_SETUP_U(1'b0)                                        // offset and width (memory form)
      5'd3: `BF_OP_U(`BF_ADDR, `R_T0, 5'd0)                          // the unit computes the aligned byte address
      5'd4: `BF_OP_U(`BF_SETI, {2'b00, dn}, 5'd0)                    // the value to insert (used by BFINS only)
      5'd6: begin `BF_OP_U(`BF_RD0, 5'd0, 5'd0) u[`F_WEN] = 1'b1; u[`F_WREG] = `R_T0 + 5'd1; end   // T1 = address
      5'd7, 5'd8: begin                                              // load a long: T2 = (T1), T3 = 4(T1)
        u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_MOV; u[`F_SZ] = `SZ_L; u[`F_LSZ] = `SZ_L; u[`F_ASEL] = `AS_MEM;
        u[`F_WEN] = 1'b1; u[`F_WREG] = `R_T0 + (step == 5'd7 ? 5'd2 : 5'd3);
        u[`F_LA_EN] = 1'b1; u[`F_LA_BSEL] = `BA_REG; u[`F_LA_BREG] = `R_T0 + 5'd1;
        u[`F_LA_DISP] = (step == 5'd7) ? 32'd0 : 32'd4;
      end
      5'd9: `BF_OP_U({2'b01, t}, `R_T0 + 5'd2, `R_T0 + 5'd3)         // compute on the two longs T2, T3
      5'd12: if (chg) begin                                          // store the new second long at 4(T1)
        `BF_OP_U(`BF_RD1, 5'd0, 5'd0)
        u[`F_SA_EN] = 1'b1; u[`F_SA_BSEL] = `BA_REG; u[`F_SA_BREG] = `R_T0 + 5'd1; u[`F_SA_DISP] = 32'd4;
      end
      5'd13: begin                                                   // result 0 and the flags
        `BF_OP_U(`BF_RD0, 5'd0, 5'd0)
        u[`F_FEN] = `FL_NZVC;
        if (chg) begin                                               // store the new first long at (T1)
          u[`F_SA_EN] = 1'b1; u[`F_SA_BSEL] = `BA_REG; u[`F_SA_BREG] = `R_T0 + 5'd1; u[`F_SA_DISP] = 32'd0;
        end else if (!tst) begin u[`F_WEN] = 1'b1; u[`F_WREG] = {2'b00, dn}; end   // extract or FFO into Dn
      end
      default: ;                                                     // steps 5, 10, 11 (and 12 for extracts): empty
    endcase
  end
end
`undef BF_SETUP_U
`undef BF_OP_U
endmodule
`default_nettype wire
