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
// seq_cas: the sequence uops of CAS and CAS2 (class SQ_CAS) and of the two-step instruction forms of class SQ_X.
//
// Where it sits. Instantiated by the sequencer p68_seq.v inside the decode stage (DS). Combinational: a uop (micro-op,
// p68_uop.vh) is a function of class, step and the parameters latched at the start (layouts in dec_p7.v and
// dec_move.v); step 0 is the decoder's uop. p68_seq ORs this module's uop with the other sub-sequencers' uops, and the
// merged uop goes on through DR (register stage before address generation), AG (address generation), OC (operand
// cycle), EX (execute) and WB (write-back).
//
// CAS and CAS2 compare memory with a register and either store a new value or load the memory value into the compare
// register (see dec_p7.v). On a real 68020 this is one locked read-modify-write bus cycle. Here it is a sequence that
// first puts the operand addresses into temporaries, then compares, then does the conditional steps, with the bus lock
// output held from the first compare to the end. The loads are uncached, so they really go to memory, and the lock keeps
// DMA and other bus masters out between the read and the write. A first version of the lock also held off the core's own
// instruction refills, and that deadlocked a replay (the replayed uop needed a refill the lock was blocking), so refills
// are not held (HISTORY.md section 6).
//   CAS  <ea>: 1 LEA ea -> T1 (with the (An)+ / -(An) update)  2 CMP Dc,(T1)  3 store Du at (T1) if Z  4 Dc <- (T1) if !Z
//   CAS2     : 1 LEA (Rn1) -> T1  2 LEA (Rn2) -> T2  (index path: Rn is Dn or An; taken before any Dc write)
//              3 CMP Dc1,(T1)  4 CMP Dc2,(T2) if Z  5 store Du1 at (T1) if Z  6 store Du2 at (T2) if Z
//              7 Dc1 <- (T1) if !Z  8 Dc2 <- (T2) if !Z
// The conditional steps are unit UN_CAS (p68_ex: store and flags only if CC holds, register write only if it does not);
// Z comes from the committed CCR of the compare steps. CC = 7 is the condition EQ (Z set). All steps from the first
// compare on carry U_LOCK (bus lock). Because every step reads the flags and T1/T2 from committed state, each step can be
// replayed alone.
// SQ_X kind 0: MOVE to (d8,An,Xn): step 0 (dec_move) moved the source into T0 with the flags; step 1 stores T0 at the
//              destination (load address path, which has the index, and SA_SAME; no load). The store address path in
//              AG has no index adder, which is why this form needs two steps.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module seq_cas (
  input  wire [3:0]  cls,                    // the running sequence class
  input  wire [4:0]  step,                   // the current step (1..n)
  input  wire [63:0] sp,                     // the parameters latched at the start
  output reg  [`UW-1:0] u,                   // the uop of this step (zero for other classes)
  output reg         last,                   // this step is the last
  input  wire        ts,                 // register bases equal to {01, tr} become T1 (p68_fx main part)
  input  wire [2:0]  tr
);
`include "p68_dec.vh"
wire [1:0]  sz  = sp[2:1];                                             // operand size
wire [2:0]  dc  = sp[5:3], du = sp[8:6], em = sp[11:9], er = sp[14:12];   // CAS: Dc, Du, EA mode and register
wire [3:0]  rn1 = sp[12:9], rn2 = sp[22:19];                           // CAS2: address registers (D/A + number)
wire [2:0]  dc2 = sp[15:13], du2 = sp[18:16];                          // CAS2: second Dc and Du
reg  [3:0]  rn;                                                        // CAS2: Rn of this step
// T1_AT: the load address is (T1); RN_AT(r): the load address is 0 + r.L through the index path (r may be Dn or An)
`define T1_AT begin u[`F_LA_BSEL] = `BA_REG; u[`F_LA_BREG] = `R_T1; end
`define RN_AT(r) begin u[`F_LA_BSEL] = `BA_ZERO; u[`F_LA_XEN] = 1'b1; u[`F_LA_XREG] = {1'b0, r}; u[`F_LA_XL] = 1'b1; end
always @* begin
  u = {`UW{1'b0}};
  last = 1'b1; rn = 4'd0;
  if (cls == `SQ_CAS && !sp[0]) begin                                  // CAS
    last = (step == 5'd4);
    u[`F_SZ] = sz; u[`F_LSZ] = sz; u[`U_LOCK] = (step != 5'd1);         // lock from the compare on
    case (step)
      5'd1: begin                                                      // T1 = EA (a LEA uop, with the An update)
        u[`F_LA] = ea_lat(em, er, sz, sp[46:31], sp[30:15], 4'd4, ts, tr);   // po = 4: behind opcode and extension
        u[`F_LA_EN] = 1'b0; u[`F_LEA] = 1'b1; u[`F_SA_BREG] = `R_T1;
      end
      5'd2: begin                                                      // CMP Dc,(T1): flags only
        u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_CMP; u[`F_LA_EN] = 1'b1; `T1_AT
        u[`F_ASEL] = `AS_REG; u[`F_AREG] = {2'b00, dc}; u[`F_BSEL] = `BS_MEM; u[`F_FEN] = `FL_NZVC; u[`F_FCV] = `CV_ADD;
      end
      5'd3: begin                                                      // if Z: store Du at (T1)
        u[`F_UNIT] = `UN_CAS; u[`F_OP] = `OP_MOV; u[`F_CC] = 4'h7; `T1_AT
        u[`F_SA_EN] = 1'b1; u[`F_SA_SAME] = 1'b1; u[`F_ASEL] = `AS_REG; u[`F_AREG] = {2'b00, du};
      end
      default: begin                                                   // if !Z: Dc = (T1)
        u[`F_UNIT] = `UN_CAS; u[`F_OP] = `OP_MOV; u[`F_CC] = 4'h7; u[`F_LA_EN] = 1'b1; `T1_AT
        u[`F_ASEL] = `AS_MEM; u[`F_BREG] = {2'b00, dc}; u[`F_WEN] = 1'b1; u[`F_WREG] = {2'b00, dc};
      end
    endcase
  end else if (cls == `SQ_CAS) begin                                   // CAS2
    last = (step == 5'd8);
    u[`F_SZ] = sz; u[`F_LSZ] = sz; u[`U_LOCK] = (step >= 5'd3);
    rn = step[0] ? rn1 : rn2;                                          // odd steps: first operand
    case (step)
      5'd1, 5'd2: begin `RN_AT(rn) u[`F_LEA] = 1'b1; u[`F_SA_BREG] = step[0] ? `R_T1 : `R_T2; end   // T1 / T2 = Rn
      5'd3, 5'd4: begin                                                // compare; the second only if the first matched
        u[`F_LA_BSEL] = `BA_REG; u[`F_LA_BREG] = step[0] ? `R_T1 : `R_T2;
        u[`F_UNIT] = step[0] ? `UN_ALU : `UN_CAS; u[`F_OP] = `OP_CMP; u[`F_CC] = 4'h7; u[`F_LA_EN] = 1'b1;
        u[`F_ASEL] = `AS_REG; u[`F_AREG] = {2'b00, step[0] ? dc : dc2}; u[`F_BSEL] = `BS_MEM;
        u[`F_FEN] = `FL_NZVC; u[`F_FCV] = `CV_ADD;
      end
      5'd5, 5'd6: begin                                                // if both matched: store Du1, Du2
        u[`F_LA_BSEL] = `BA_REG; u[`F_LA_BREG] = step[0] ? `R_T1 : `R_T2;
        u[`F_UNIT] = `UN_CAS; u[`F_OP] = `OP_MOV; u[`F_CC] = 4'h7;
        u[`F_SA_EN] = 1'b1; u[`F_SA_SAME] = 1'b1; u[`F_ASEL] = `AS_REG; u[`F_AREG] = {2'b00, step[0] ? du : du2};
      end
      default: begin                                                   // otherwise: load both memory values into Dc1, Dc2
        u[`F_LA_BSEL] = `BA_REG; u[`F_LA_BREG] = step[0] ? `R_T1 : `R_T2;
        u[`F_UNIT] = `UN_CAS; u[`F_OP] = `OP_MOV; u[`F_CC] = 4'h7; u[`F_LA_EN] = 1'b1; u[`F_ASEL] = `AS_MEM;
        u[`F_BREG] = {2'b00, step[0] ? dc : dc2}; u[`F_WEN] = 1'b1; u[`F_WREG] = {2'b00, step[0] ? dc : dc2};
      end
    endcase
  end else if (cls == `SQ_X && sp[1:0] == 2'd0) begin                  // MOVE to (d8,An,Xn): store T0
    u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_MOV; u[`F_SZ] = sp[3:2];
    // the destination's address on the load path (which has the index adder); po = 2 is not used by mode 6
    u[`F_LA] = ea_lat(3'd6, sp[6:4], sp[3:2], sp[22:7], 16'h0, 4'd2, ts, tr);
    u[`F_LA_EN] = 1'b0; u[`F_SA_EN] = 1'b1; u[`F_SA_SAME] = 1'b1; u[`F_ASEL] = `AS_REG; u[`F_AREG] = `R_T0;
  end
end
`undef T1_AT
`undef RN_AT
endmodule
`default_nettype wire
