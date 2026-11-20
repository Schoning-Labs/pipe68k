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
// seq_flow: the sequence uops of the program flow instructions RTD, RTR, PEA, LINK and UNLK (class SQ_FLOW).
//
// Where it sits. Instantiated by the sequencer p68_seq.v inside the decode stage (DS). A uop (micro-op, p68_uop.vh) is a
// function of the step number and the parameters latched at the start (kinds and layout in dec_flow.v); step 0 is the
// decoder's empty uop. p68_seq ORs this module's uop with the other sub-sequencers' uops, and the merged uop goes on
// through DR (register stage before address generation), AG (address generation), OC (operand cycle), EX (execute) and
// WB (write-back). The module is combinational except for one register (nd_r, below).
//
// Rules these sequences follow: a uop that redirects when it commits (SY_JMP) updates no An in AG; A7 is changed first
// by a LEA uop (an AG write, committed before the jump uop reaches EX) and the jump uop loads the target from below the
// new A7. The rule came from the early exception work. It stopped being necessary once the register file restored one
// clock after a redirect (that change is what let RTS become a single uop, see dec_flow.v), but these sequences were
// written under it and still follow it, which costs nothing because they are rare.
// Every step restarts alone after a replay: steps read A7 (committed by the earlier steps) and T0, nothing else.
//   RTD   1: A7 += 4 + d   2: load PC from -(4 + d)(A7), jump
//   RTR   1: A7 += 6   2: CCR <- word at -6(A7)   3: load PC from -4(A7), jump
//   UNLK  1: A7 = An   2: An = (A7)+
//   PEA   1: T0 = EA   2: store T0 at -(A7)
//   LINK  1: store An at -(A7)   2: An = A7   3: A7 += d
// LINK A7 pushes the old A7, as the 68020 does (the WinUAE cputester data for the 68020 passes). The 68000 and Musashi
// push A7 - 4, so the random program generator of the test bench avoids this case.
// UNLK A7 loads A7 from (A7), as Musashi does.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`include "p68_sys.vh"
`default_nettype none
module seq_flow (
  input  wire        clk,
  input  wire [3:0]  cls,                    // the running sequence class
  input  wire [4:0]  step,                   // the current step (1..n)
  input  wire [63:0] sp,                     // the parameters latched at the start (sp[2:0] = kind)
  output reg  [`UW-1:0] u,                   // the uop of this step (zero for other classes)
  output reg         last,                   // this step is the last
  input  wire        ts,                 // register bases equal to {01, tr} become T1 (p68_fx main part)
  input  wire [2:0]  tr
);
`include "p68_dec.vh"
reg  [31:0] d;                               // displacement parameter (RTD)
reg  [4:0]  an;                              // An of UNLK / LINK as a register number (8-15)
// RTD's second displacement (-4 - d) as a register (step 2 comes at least one clock after the parameters
// loaded); its first (4 + d) is d with the constant index 4 (AG adds base + index + displacement)
reg  [31:0] nd_r;
always @(posedge clk) nd_r <= 32'h0 - 32'd4 - sp[34:3];

// u fields: load long at disp(base), result as operand A; LEA base + disp into reg; push (store at -4(A7), A7 -= 4)
`define FL_LD(b, dsp, sz) begin u[`F_LA_EN] = 1'b1; u[`F_LA_BSEL] = `BA_REG; u[`F_LA_BREG] = tbs(b, ts, tr); u[`F_LA_DISP] = dsp; u[`F_LSZ] = sz; u[`F_ASEL] = `AS_MEM; end
`define FL_LEA(b, dsp, rg) begin u[`F_LA_BSEL] = `BA_REG; u[`F_LA_BREG] = tbs(b, ts, tr); u[`F_LA_DISP] = dsp; u[`F_LEA] = 1'b1; u[`F_SA_BREG] = rg; end
`define FL_PUSH begin u[`F_SA_EN] = 1'b1; u[`F_SA_BSEL] = `BA_REG; u[`F_SA_BREG] = tbs(5'd15, ts, tr); u[`F_SA_DISP] = 32'hFFFFFFFC; u[`F_SA_UEN] = 1'b1; u[`F_SA_UDL] = 8'hFC; end

always @* begin
  u = {`UW{1'b0}};
  last = 1'b1;
  d = sp[34:3]; an = {2'b01, sp[5:3]};
  if (cls == `SQ_FLOW) case (sp[2:0])                              // by kind
    3'd3: begin                                                    // RTD
      last = (step == 5'd2);
      // step 1: A7 = A7 + 4 + d (d with the constant index 4); step 2: load the PC from below the new A7, jump
      if (step == 5'd1) begin `FL_LEA(5'd15, d, 5'd15) u[`F_LA_XK] = 4'd4; end
      else begin
        `FL_LD(5'd15, nd_r, `SZ_L)
        u[`F_UNIT] = `UN_SYS; u[`F_OP] = `SY_JMP; u[`F_SZ] = `SZ_L;
      end
    end
    3'd4: begin                                                    // RTR
      last = (step == 5'd3);
      if (step == 5'd1) `FL_LEA(5'd15, 32'd6, 5'd15)               // A7 += 6 (CCR word and PC long)
      else if (step == 5'd2) begin                                 // CCR <- the word at -6(A7)
        `FL_LD(5'd15, 32'hFFFFFFFA, `SZ_W)
        u[`F_UNIT] = `UN_SYS; u[`F_OP] = `SY_MVSR; u[`F_SZ] = `SZ_B;   // SZ_B: only the CCR byte is written
      end else begin                                               // load the PC from -4(A7), jump
        `FL_LD(5'd15, 32'hFFFFFFFC, `SZ_L)
        u[`F_UNIT] = `UN_SYS; u[`F_OP] = `SY_JMP; u[`F_SZ] = `SZ_L;
      end
    end
    3'd5: begin                                                    // UNLK
      last = (step == 5'd2);
      if (step == 5'd1) `FL_LEA(an, 32'h0, 5'd15)                  // A7 = An
      else begin                                                   // An = (A7)+
        `FL_LD(5'd15, 32'h0, `SZ_L)
        u[`F_LA_UEN] = 1'b1; u[`F_LA_UDL] = 8'd4;
        u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_MOV; u[`F_SZ] = `SZ_L; u[`F_WEN] = 1'b1; u[`F_WREG] = an;
      end
    end
    3'd6: begin                                                    // PEA
      last = (step == 5'd2);
      if (step == 5'd1) begin                                      // T0 = the EA (a LEA uop)
        u[`F_LA] = ea_lat(sp[5:3], sp[8:6], `SZ_L, sp[24:9], sp[40:25], 4'd2, ts, tr);
        u[`F_LA_EN] = 1'b0; u[`F_LA_UEN] = 1'b0;
        u[`F_LEA] = 1'b1; u[`F_SA_BREG] = `R_T0;
      end else begin                                               // push T0
        u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_MOV; u[`F_SZ] = `SZ_L; u[`F_ASEL] = `AS_REG; u[`F_AREG] = `R_T0;
        `FL_PUSH
      end
    end
    default: begin                                                 // LINK: d = sp[37:6]
      last = (step == 5'd3);
      if (step == 5'd1) begin                                      // push An
        u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_MOV; u[`F_SZ] = `SZ_L; u[`F_ASEL] = `AS_REG; u[`F_AREG] = an;
        `FL_PUSH
      end else if (step == 5'd2) `FL_LEA(5'd15, 32'h0, an)         // An = A7 (the frame pointer)
      else `FL_LEA(5'd15, sp[37:6], 5'd15)                         // A7 += d (d is negative to reserve space)
    end
  endcase
end
endmodule
`default_nettype wire
