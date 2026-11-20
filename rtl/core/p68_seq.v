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
// p68_seq: the sequencer, which turns one long instruction into several micro-ops (uops).
//
// Where it sits. Inside the decode stage (DS, p68_decode.v), beside the group decoders. Most 68k instructions become a
// single uop (the work order of p68_uop.vh, one per clock through the pipeline stages DR, AG, OC, EX and WB: the
// register stage before address generation, address generation, operand cycle with the data cache, execute and
// write-back). Some cannot: MOVEM moves up to 16 registers, a memory-to-memory ADDX needs two loads, a divide needs
// to wait for a 16-clock unit, a bit field in memory needs two longs read, changed and written. For those, the decoder
// returns a sequence class and a block of parameters, and its own uop becomes step 0. This module then produces steps 1
// to n, one uop per clock in which DS may advance. The instruction stays at the head of the instruction queue until its
// last uop is issued, and the uop of its last step carries the LAST flag, with which the instruction retires.
// The module runs on the core clock; its state is a handful of registers, the uops are combinational from them.
//
// The contract every sequence follows. These rules come from how the pipeline works, and every sub-sequencer obeys
// them (seq_int.v, seq_flow.v, seq_cas.v, seq_bf.v, p68_mdseq.v, seq_fpu.v; the system sequencer p68_sseq.v follows the
// same contract):
//   1. The uop of a step is a function of class, step number and the parameters only. The parameters come from
//      registers loaded at the start, so no sequence uop depends on the queue words in the same clock, and the path
//      from the instruction queue to the sequencer's output is one register long.
//   2. Every step can be restarted alone. When a load misses the data cache, the pipeline replays from that uop: DS
//      restarts the instruction and suppresses the uops before the replayed step. A step may therefore read only
//      registers that earlier steps have already committed (temporaries T0-T3 and the architectural registers it has
//      not changed itself), never a value that only exists in the pipeline.
//   3. Step 0 is the decoder's uop and is usually empty. DS cannot afford a select between "decoder uop" and "sequencer
//      uop" in its output path (it was measured at about 20 MHz early on, HISTORY.md section 5), so the two are ORed,
//      and this module's uop is all zero while no sequence runs. The price is one empty uop per sequence.
//
// MOVEM (class SQ_MOVEM) is handled here directly; it was the first sequence and set the pattern. Step 0 (decoder) is
// empty, except when the base of -(An) is in the list (see dec_movem.v). For control modes step 1 computes the EA into
// T0 with a LEA uop, so loads into the base or index register cannot move later addresses. Then one register per uop,
// mask bit 0 first. Register = bit index, or 15 - bit index for -(An) (reversed mask). Address = base + offset (offset
// 0, step, 2*step ... ; for -(An) -step, -2*step ...). The last uop also updates An for (An)+ / -(An). A load into the
// base register of (An)+ is dropped (the update wins). An empty mask gives one empty uop that retires the instruction.
//
// The other classes come from sub-sequencers instantiated below; each gives its own uop and last flag, all zero for
// classes that are not its own, and this module merges them in one flat AND-OR.
//
// Timing techniques used here (all because the sequencer's outputs feed DS's issue logic, one of the longest paths):
//   - The class, step and parameters load in every clock without a running sequence, so their clock enable is the
//     register act and not the start decision (a wide enable from decode logic would be slow, and Gowin's synthesizer
//     never splits a clock-enable net).
//   - Counters that the sub-sequencers would otherwise compute with adders from the step (step - 1, 4 - step, 2 - step)
//     are kept as registers of their own.
//   - For MOVEM, the remaining mask after the next transfer (rstr) and the next register's index (bi) are registers, so
//     the "last" decision has no carry chain in front of it.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`include "p68_md.vh"
`include "p68_fpu.vh"
`default_nettype none
module p68_seq (
  input  wire        clk,
  input  wire        rst,
  input  wire        flush,                  // pipeline flush (redirect, exception): the sequence is abandoned
  input  wire        start,          // step 0 of a sequence leaves DS this clock
  input  wire [3:0]  cls_in,                 // sequence class from the decoders (OR of all decoders' seq)
  input  wire [63:0] par_in,                 // sequence parameters from the decoders (OR of all decoders' spar)
  input  wire [15:0] mw,             // the MOVEM mask word straight from the instruction (par_in[15:0] of MOVEM)
  input  wire        adv,            // DS advances this clock (AG accepts, no flush)
  output reg         act,            // sequence running: steps 1..n
  output reg  [4:0]  step,                   // the current step number (1..n while act)
  output wire        last,                   // the current step is the last one
  output reg  [`UW-1:0] u,           // uop of the current step (PC, LEN, FIRST, LAST, STEP added by DS)
  input  wire        ts,             // register bases equal to {01, tr} become T1 (p68_fx main part; the
  input  wire [2:0]  tr              // sequencers apply it where they choose each base)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"

// ---- MOVEM state --------------------------------------------------------------------------------------------------
reg [3:0]  cls;                              // the running class, latched at the start
reg [15:0] msk;                      // registers still to transfer
reg [7:0]  off;                      // address offset of the current transfer
reg        ld, lg, pre, post;                // memory to registers, long, -(An), (An)+
reg [4:0]  base;                             // base register of the transfers (An, or T0 for control modes)
reg        eap;                      // next step: EA into T0 (control modes)
reg [2:0]  em, er;                   // EA mode and register
reg [31:0] ex;                       // EA extension words

// ---- classes SQ_EXG .. SQ_ROX (seq_int.v) ----
reg  [63:0] sp;                      // parameters of those classes, latched at the start
reg  [3:0]  sm1, sc4, sc2;            // step - 1, 4 - step, 2 - step (seq_int's MOVEP fields without adders)
wire [`UW-1:0] u_i;
wire        last_i;
seq_int i_si (.cls(cls), .step(step), .sm1(sm1), .sc4(sc4), .sc2(sc2), .sp(sp), .u(u_i), .last(last_i), .last_fp(last_fp), .ts(ts), .tr(tr));
// ---- classes SQ_MUL, SQ_DIV (p68_mdseq.v) ----
wire [`UW-1:0] u_md;
wire        last_md;
p68_mdseq i_mdq (.cls(cls), .step(step), .sp(sp), .u(u_md), .last(last_md));
// ---- class SQ_FLOW (seq_flow.v) ----
wire [`UW-1:0] u_f;
wire        last_f;
seq_flow i_sf (.clk(clk), .cls(cls), .step(step), .sp(sp), .u(u_f), .last(last_f), .ts(ts), .tr(tr));
// ---- classes SQ_BFR, SQ_BFM (seq_bf.v) ----
wire [`UW-1:0] u_b;
wire        last_b;
seq_bf i_sb (.cls(cls), .step(step), .sp(sp), .u(u_b), .last(last_b), .ts(ts), .tr(tr));
wire        is_bf = (cls == `SQ_BFR) || (cls == `SQ_BFM);
// ---- classes SQ_CAS, SQ_X (seq_cas.v) ----
wire [`UW-1:0] u_c;
wire        last_c;
seq_cas i_sc (.cls(cls), .step(step), .sp(sp), .u(u_c), .last(last_c), .ts(ts), .tr(tr));
wire        is_p7 = (cls == `SQ_CAS) || (cls == `SQ_X);
// ---- class SQ_FPU (seq_fpu.v, only in a build with the FPU); hold: a wait keeps the step number ----
wire [`UW-1:0] u_fp;
wire        last_fp, hold_fp;
wire        is_fp = (cls == `SQ_FPU);
generate if (`P68_FPU != 0) begin : g_fpu
// seq_fpu starts with every p68_seq start (its state is read only while it runs, and its own start reloads it)
seq_fpu i_sfp (.clk(clk), .start(start), .adv(adv), .sp(sp), .act(act && is_fp),
               .u(u_fp), .last(last_fp), .hold(hold_fp), .ts(ts), .tr(tr));
end else begin : g_nofpu
// without the FPU: no uop, always last, never holding
assign u_fp = {`UW{1'b0}}; assign last_fp = 1'b1; assign hold_fp = 1'b0;
end endgenerate

function [3:0] low1;                 // index of the lowest set bit (0 when none)
  input [15:0] m;
  integer k;
  begin
    low1 = 4'd0;
    // scanning from the top down, the lowest set bit is written last and wins (a priority encoder in LUTs)
    for (k = 15; k >= 0; k = k - 1) if (m[k]) low1 = k[3:0];
  end
endfunction

// ---- MOVEM's mask arithmetic --------------------------------------------------------------------------------------
reg  [3:0]  bi;                                        // low1(msk) as a register, loaded with msk
wire [3:0]  rn    = pre ? ~bi : bi;                    // 15 - bit index for -(An)
// msk with its lowest set bit cleared: x & (x - 1) is the usual trick; the subtract is a 16-bit carry chain
wire [15:0] rest  = msk & (msk - 16'd1);
reg  [15:0] rstr;                                      // rest as a register (MOVEM last without the carry chain)
wire [15:0] rest2 = rest & (rest - 16'd1);             // the mask after the transfer after this one (next rstr)
wire [15:0] pin1  = mw & (mw - 16'd1);                 // the first rstr, from the instruction's mask word
wire        empty = (msk == 16'h0);                    // nothing (more) to transfer
wire [7:0]  st    = lg ? 8'd4 : 8'd2;                  // bytes per register
// The class lasts as a one-hot AND-OR (same priority result: the classes are disjoint). MOVEM ends when no register is
// left after the current one and the EA step is done; seq_int covers classes 8-15 except SQ_X; class 0 (no sequence)
// counts as last.
assign last = ((cls == `SQ_MOVEM) & !eap & (rstr == 16'h0)) | (is_bf & last_b) | (is_p7 & last_c) |
              (cls[3] & (cls != `SQ_X) & last_i) | ((cls == `SQ_MUL || cls == `SQ_DIV) & last_md) | ((cls == `SQ_FLOW) & last_f) |
              (cls == 4'd0);

// ---- state registers ----------------------------------------------------------------------------------------------
// The class, step and parameters load in every clock without a running sequence (they are read only while act), so
// their enable is !act (a register) and not the start decision; start sets act only.
always @(posedge clk) begin
  if (rst || flush) act <= 1'b0;                                  // a flush abandons the sequence
  else if (start) act <= 1'b1;                                    // step 0 left DS: steps 1.. follow
  else if (act && adv && last) act <= 1'b0;                       // the last step left DS
  if (!act) begin                                                 // idle: track the decoders' outputs
    step <= 5'd1; sm1 <= 4'd0; sc4 <= 4'd3; sc2 <= 4'd1; cls <= cls_in; sp <= par_in;
    msk <= mw; rstr <= pin1; bi <= low1(mw); ld <= par_in[16]; lg <= par_in[17]; pre <= par_in[18]; post <= par_in[19];
    base <= par_in[24:20]; em <= par_in[27:25]; er <= par_in[30:28]; ex <= par_in[63:32];
    eap <= !(par_in[18] || par_in[19]);                           // control modes start with the EA step
    off <= par_in[18] ? (par_in[17] ? 8'hFC : 8'hFE) : 8'h0;      // 0 - step (no subtractor)
  end else if (adv) begin                                         // a step left DS: move to the next one
    // a waiting FPU step keeps its number (and the counters with it)
    if (!(is_fp && hold_fp)) begin step <= step + 5'd1; sm1 <= sm1 + 4'd1; sc4 <= sc4 - 4'd1; sc2 <= sc2 - 4'd1; end
    if (eap) eap <= 1'b0;                                         // MOVEM: the EA step is done
    else begin                                                    // MOVEM: one register transferred
      msk <= rest; rstr <= rest2; bi <= low1(rest);
      off <= pre ? off - st : off + st;                           // next address offset, downward for -(An)
    end
  end
end

// ---- MOVEM uops ---------------------------------------------------------------------------------------------------
// The MOVEM uop and the class sequencers' uops merge in one flat AND-OR; the T1 replacement (ts, tr) is applied where
// each base is chosen.
reg  [`UW-1:0] u_mv;
always @* begin
  u_mv = {`UW{1'b0}};
  if (act && cls == `SQ_MOVEM && eap) begin                // EA into T0 (extension words follow the mask word)
    u_mv[`F_LA] = ea_lat(em, er, `SZ_L, ex[31:16], ex[15:0], 4'd4, ts, tr);   // po = 4: opcode and mask come first
    u_mv[`F_LA_EN] = 1'b0; u_mv[`F_LA_UEN] = 1'b0;          // address only, no load, no update
    u_mv[`F_LEA] = 1'b1; u_mv[`F_SA_BREG] = `R_T0;          // AG writes the address into T0
  end else if (act && cls == `SQ_MOVEM && !empty) begin     // one register transfer
    u_mv[`F_UNIT] = `UN_ALU; u_mv[`F_OP] = `OP_MOV;
    if (ld) begin                                     // memory to register: sign-extended for .W
      u_mv[`F_LA_EN] = 1'b1; u_mv[`F_LA_BSEL] = `BA_REG; u_mv[`F_LA_DISP] = {{24{off[7]}}, off};   // load at base + off
      u_mv[`F_LSZ] = lg ? `SZ_L : `SZ_W; u_mv[`F_SZ] = `SZ_L; u_mv[`F_ASEL] = `AS_MEM;
      u_mv[`F_AEXT] = lg ? `AX_NONE : `AX_SXW;              // a word load fills the whole register, sign-extended
      // no write when this is the base An of (An)+ (the address update wins, as on the 68020)
      u_mv[`F_WEN] = !(post && base == {1'b0, 1'b1, rn[2:0]} && rn[3]); u_mv[`F_WREG] = {1'b0, rn};
      if (last && post) begin u_mv[`F_LA_UEN] = 1'b1; u_mv[`F_LA_UDL] = off + st; end   // (An)+: An past the last
    end else begin                                    // register to memory
      u_mv[`F_SA_EN] = 1'b1; u_mv[`F_SA_BSEL] = `BA_REG; u_mv[`F_SA_DISP] = {{24{off[7]}}, off};   // store at base + off
      u_mv[`F_SZ] = lg ? `SZ_L : `SZ_W; u_mv[`F_ASEL] = `AS_REG; u_mv[`F_AREG] = {1'b0, rn};
      if (last && pre) begin u_mv[`F_SA_UEN] = 1'b1; u_mv[`F_SA_UDL] = off; end   // -(An): An = address of the last
      // the base register in the list is stored as initial - size (68020): step 0 left it in T0
      if (pre && rn[3] && base == {2'b01, rn[2:0]}) u_mv[`F_AREG] = `R_T0;
    end
  end
  // MOVEM's base register numbers without the empty-mask test (numbers are read only with the base select; the T1
  // replacement only with the base select, as before)
  if (act && cls == `SQ_MOVEM && !eap) begin
    if (ld) u_mv[`F_LA_BREG] = tbs(base, ts && !empty, tr); else u_mv[`F_SA_BREG] = tbs(base, ts && !empty, tr);
  end
end
always @* u = u_mv | ({`UW{act}} & (u_i | u_md | u_f | u_b | u_c | u_fp));   // the class uops are zero for the other classes

endmodule
`default_nettype wire
