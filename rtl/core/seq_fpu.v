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
// ==== seq_fpu.v: the uop sequences of the FPU instructions ===========================================================
//
// Abbreviations: DS = the decode stage, FPU = floating-point unit, EA = effective address, FPn = an FP register, FPcr = one
// of the control registers FPCR, FPSR, FPIAR, T0 = the sequencers' first temporary register, NULL/IDLE = the FPU states
// FSAVE writes as a short or a 60-byte frame.
//
// ---- Where it sits ----------------------------------------------------------------------------------------------------
//
// Every FPU instruction is a sequence of class SQ_FPU. dec_fpu.v gives step 0 and the parameters; this module,
// instantiated by p68_seq.v, counts the uops after step 0 (n) and gives each one. Combinational uop, registered state.
//
// The engine (p68_fpe.v) has a fixed worst-case latency per operation, so the sequences simply wait for it: the waits
// are empty uops that keep their step number (hold), so long operations fit the 5-bit STEP field. Only loads and stores
// replay, and each of them has its own step. While an FPU instruction waits, the integer pipe waits with it, because no
// other instruction issues while a sequence runs. That is the price of the simple fixed-latency interface (for example
// about 63 clocks for FMUL at P68_FPN = 2 before the engine was made smaller; docs/learn/HISTORY.md, section 13).
//
// ---- The sequences -----------------------------------------------------------------------------------------------------
//
// Every sequence starts with an empty pad uop (the start values load from registers while it runs), then:
//   GENR, CR : waits, FP_WR                    GEN1 : FP_GO, waits, FP_WR
//   GENM     : FP_IN 0, 1 [, 2], FP_GO, waits, FP_WR             (step 0: LEA into T0)
//   OUT      : waits (.D/.X: the first is the LEA into T0), FP_RD 0 [, 1 [, 2]] (stores, or Dn); the last commits the FPSR
//   CTLR     : FP_CR to Rn, or FP_CW from Rn / #imm (step 0 was FP_CHK)
//   CTLM     : one FP_CW / FP_CR per register (FPCR, FPSR, FPIAR order, ascending addresses)
//   MVM      : three FP_IN / FP_RX per register (ascending addresses FP0..FP7)
//   COND     : the conditional uop (Bcc, Scc, DBcc, TRAPcc with U_FPC)
//   SAVE     : FP_SV 0 (NULL) or FP_SV 0..14 (IDLE)
//   REST     : FP_RS 0, 1, 2 (T0 + 0, 0x18, 0x38), FP_RSC [, FP_RSA -> T0, An += T0 for (An)+], SY_SER (DS rereads the state)
// A sequence that may leave a pending trap or change what DS decodes (an FPCR load, FSAVE) ends with a redirecting
// SY_SER uop, so DS decodes the next FPU instruction with the committed state.
//
// ---- Timing ------------------------------------------------------------------------------------------------------------
//
// last, hold and the phase of the current uop are registers, computed one uop ahead (at the start from the parameters,
// then from registers), so no FPU term reaches DS's issue logic in the same clock. That was one of the timing steps of
// the first FPU (the sequencer terms registered, then the FPU terms taken out of the issue and fetch logic). Multi-piece
// transfers address T0 + offset (an 8-bit register): step 0 (or, for FMOVE out .D/.X, the first wait) is a LEA of the EA
// into T0 that also steps (An)+ / -(An) by the whole size, so no piece needs a wide adder.
`include "p68_uop.vh"                    // the uop format
`include "p68_fpu.vh"                    // FPU codes, sequence kinds, P68_FPN
`include "p68_sys.vh"                    // SY_SER, SY_TRAPCC
`default_nettype none
module seq_fpu (
  input  wire        clk,                // CPU clock
  input  wire        start,              // the sequence starts (step 0 leaves DS)
  input  wire        adv,                // a step is consumed (act && adv in p68_seq)
  input  wire [63:0] sp,                 // parameters (latched by p68_seq)
  input  wire        act,                // SQ_FPU runs
  output reg  [`UW-1:0] u,               // the uop of the current step
  output wire        last,               // it is the last uop of the instruction
  output wire        hold,               // keep the step number (inside a wait)
  input  wire        ts,                 // register bases equal to {01, tr} become T1 (p68_fx main part)
  input  wire [2:0]  tr
);
`include "p68_dec.vh"                    // shared decoder functions (EA descriptors)
wire [3:0]  knd = sp[3:0];               // the sequence kind (FK_*)
wire [15:0] cw  = sp[19:4];              // the command word (FBcc: the opcode)
wire [2:0]  r0  = sp[22:20], m3 = sp[25:23];   // the EA register and mode
wire [15:0] x0  = sp[57:42], x1 = sp[41:26];   // the EA's extension words
wire        nul = sp[58];                // FSAVE in the NULL state

function [7:0] rev8; input [7:0] v; integer i; begin for (i = 0; i < 8; i = i + 1) rev8[i] = v[7 - i]; end endfunction   // bit reverse
function [7:0] top3; input [7:0] v; begin top3 = v[2] ? 8'd4 : v[1] ? 8'd2 : 8'd1; end endfunction   // highest of three list bits
function [3:0] low8; input [7:0] v; integer i; begin low8 = 4'd0; for (i = 7; i >= 0; i = i - 1) if (v[i]) low8 = i[3:0]; end endfunction   // lowest set bit
function [3:0] cnt8; input [7:0] v; integer i; begin cnt8 = 4'd0; for (i = 0; i < 8; i = i + 1) cnt8 = cnt8 + {3'h0, v[i]}; end endfunction   // set bits
// Engine latency in steps (p68_fpe: worst case from the start until S_DONE, shifts and normalizations at their longest;
// an FP_RD's result is prepared one clock before EX). A step is P68_FPN clocks; the window adds the start (FP_GO commit,
// register file read, the next step), the copy into p68_fpu (one step for dn, one clock) and the FP_RD preparation:
// (lat + 2) * P68_FPN + 3 clocks.
function [8:0] lat; input [15:0] w;
  reg [6:0] o;
  begin
    o = w[6:0];
    if (w[15:13] == 3'd3) lat = 9'd24;                               // FMOVE out
    else if (w[14] && w[12:10] == 3'd7) lat = 9'd24;                 // FMOVECR
    else if (o == 7'h22 || o[6:3] == 4'b0101) lat = 9'd42;           // FADD, FSUB
    else if (o == 7'h23 || o == 7'h27) lat = 9'd50;                  // FMUL, FSGLMUL
    else if (o == 7'h20 || o == 7'h24 || o == 7'h04 || o == 7'h05) lat = 9'd96;   // FDIV, FSGLDIV, FSQRT
    else if (o == 7'h01 || o == 7'h03) lat = 9'd36;                  // FINT, FINTRZ
    else if (o == 7'h1e || o == 7'h1f) lat = 9'd29;                  // FGETEXP, FGETMAN
    else if (o == 7'h26) lat = 9'd35;                                // FSCALE
    else if (o[6:3] == 4'b0111) lat = 9'd20;                         // FCMP, FTST
    else lat = 9'd24;                                                // FMOVE, FABS, FNEG
  end
endfunction

// ---- Values derived from the parameters at the start --------------------------------------------------------------
// From p68_seq's parameter register: the first sequencer uop is an empty pad while these load (init), so nothing is
// computed from DS's decode in the start clock.
wire [3:0]  kp   = sp[3:0];
wire [15:0] cwp  = sp[19:4];
wire [2:0]  r0p  = sp[22:20], m3p = sp[25:23];
wire        nulp = sp[58];
wire        serp = sp[59];                                      // a trailing SY_SER (dec_fpu: pending traps, FPCR loads)
wire [8:0]  Lp   = (lat(cwp) + 9'd2) * `P68_FPN + 9'd3;              // engine steps of P68_FPN clocks, start and copy
wire [8:0]  ninp = (cwp[12:10] == 3'd2) ? 9'd3 : 9'd2;                                  // GENM: FP_IN pieces
wire [8:0]  noutp = (cwp[12:10] == 3'd2) ? 9'd3 : (cwp[12:10] == 3'd5) ? 9'd2 : 9'd1;  // OUT: FP_RD pieces
wire [3:0]  nregp = cnt8(cwp[7:0]);                                  // MVM: registers in the list
wire [1:0]  nctlp = {1'b0, cwp[12]} + {1'b0, cwp[11]} + {1'b0, cwp[10]};   // CTLM: control registers in the list
wire [7:0]  totp  = (kp == `FK_OUT) ? {noutp[5:0], 2'b00} : (kp == `FK_CTLM) ? {4'h0, nctlp, 2'b00} :
                    (kp == `FK_MVM) ? ({4'h0, nregp} << 3) + ({4'h0, nregp} << 2) : (kp == `FK_SAVE) ? (nulp ? 8'd4 : 8'd60) :
                    (kp == `FK_GENM) ? ((cwp[12:10] == 3'd2) ? 8'd12 : 8'd8) : 8'd0;   // bytes of the transfer
// index of the last uop, the FP_GO, and the wait window [w0, w1)
wire [8:0]  lastb = (kp == `FK_GENR || kp == `FK_CR) ? Lp : (kp == `FK_GEN1) ? Lp + 9'd1 : (kp == `FK_GENM) ? ninp + Lp + 9'd1 :
                    (kp == `FK_OUT) ? Lp + noutp - 9'd1 : (kp == `FK_CTLM) ? {5'h0, nctlp} - 9'd1 :
                    (kp == `FK_MVM) ? ((nregp == 4'd0) ? 9'd0 : {2'b00, nregp, 1'b0} + {3'b000, nregp} - 9'd1) :
                    (kp == `FK_SAVE) ? (nulp ? 9'd0 : 9'd14) : (kp == `FK_REST) ? ((m3p == 3'd3) ? 9'd6 : 9'd4) : 9'd0;
wire [8:0]  lastp = lastb + {6'h0, serp};                            // plus the trailing SY_SER
// The first last / hold / phase values without the latency adders: Lp >= 3 (the latency sum stays below 512 for
// P68_FPN up to 5), so lastb is 0 only for these kinds, and the first uop is in a wait only for GENR, CR, OUT.
wire        lb0   = !(kp == `FK_GENR || kp == `FK_CR || kp == `FK_GEN1 || kp == `FK_GENM || kp == `FK_OUT || kp == `FK_CTLM ||
                      kp == `FK_MVM || kp == `FK_SAVE || kp == `FK_REST) ||
                    (kp == `FK_CTLM && nctlp == 2'd1) || (kp == `FK_MVM && cwp[7:0] == 8'h0) || (kp == `FK_SAVE && nulp);
wire        last0 = serp ? (kp == `FK_CTLM && nctlp == 2'd0) : lb0;     // lastp == 0 (lastb == 511 only for an empty CTLM list)
wire        wt0   = (kp == `FK_GENR || kp == `FK_CR || kp == `FK_OUT);   // w0p == 0 and w1p = Lp > 1
wire [8:0]  gop   = (kp == `FK_GEN1) ? 9'd0 : (kp == `FK_GENM) ? ninp : 9'd511;   // the FP_GO's index (511: none)
wire [8:0]  w0p   = (kp == `FK_GEN1) ? 9'd1 : (kp == `FK_GENM) ? ninp + 9'd1 : (kp == `FK_GENR || kp == `FK_CR || kp == `FK_OUT) ? 9'd0 : 9'd511;
wire [8:0]  w1p   = (kp == `FK_GEN1) ? Lp + 9'd1 : (kp == `FK_GENM) ? ninp + Lp + 9'd1 : (kp == `FK_GENR || kp == `FK_CR || kp == `FK_OUT) ? Lp : 9'd511;
// ---- Sequencer state ----------------------------------------------------------------------------------------------
reg  [8:0]  n, rem, go_i, w0_i, w1_i, nin, nout;   // uop index; uops left; FP_GO index; wait window; piece counts
reg  [7:0]  msk, tot;                    // the register list still to do; bytes of the transfer
reg  [3:0]  nreg;                        // registers in an FMOVEM.X list
reg  [1:0]  wd, nctl;                    // the word (0-2) of the current FMOVEM.X register; control registers in the list
reg  [7:0]  off;                         // byte offset of the current piece from T0
reg         last_r, hold_r;              // the current uop is the last; it is inside a wait
reg  [1:0]  ph;                          // phase of a general sequence: 0 other, 1 FP_IN, 2 FP_GO, 3 wait
assign last = last_r;
assign hold = act && hold_r;
wire [8:0]  n1 = n + 9'd1;
wire        piece = (knd == `FK_OUT) ? (ph != 2'd3) : (knd == `FK_GENM) ? (ph == 2'd1) : 1'b1;   // the current uop is an EA piece
wire        oxd = (cw[12:10] == 3'd2) || (cw[12:10] == 3'd5);                 // FMOVE out .X / .D
reg         init;                        // the pad uop runs; the start values load when it leaves
reg         dsc;                         // FMOVEM.X: list type and EA disagree: registers in descending order (68882)
wire [7:0]  tp3 = top3(msk);
wire [3:0]  lo8 = low8(msk);
// highest register of the list (one-hot and number); Gowin synthesis failed ("error bus name set") on a shift by it
wire [7:0]  mtop = msk[7] ? 8'h80 : msk[6] ? 8'h40 : msk[5] ? 8'h20 : msk[4] ? 8'h10 : msk[3] ? 8'h08 : msk[2] ? 8'h04 : msk[1] ? 8'h02 : 8'h01;
wire [2:0]  mhi  = msk[7] ? 3'd7 : msk[6] ? 3'd6 : msk[5] ? 3'd5 : msk[4] ? 3'd4 : msk[3] ? 3'd3 : msk[2] ? 3'd2 : msk[1] ? 3'd1 : 3'd0;
wire [2:0]  mvr  = dsc ? mhi : lo8[2:0];                             // FMOVEM.X register of this piece
always @(posedge clk) begin
  if (start) begin
    init <= 1'b1; last_r <= 1'b0; hold_r <= 1'b0; ph <= 2'd0;
  end else if (act && adv && init) begin                     // the pad uop leaves: load the start values
    init <= 1'b0;
    n <= 9'd0; wd <= 2'd0; rem <= lastp;
    msk <= (kp == `FK_CTLM) ? {5'h0, cwp[12:10]} : (cwp[12] ? rev8(cwp[7:0]) : cwp[7:0]);   // MVM: bit i = FPi
    dsc <= (cwp[12] == (m3p == 3'd4));
    nreg <= nregp; nctl <= nctlp; tot <= totp; nin <= ninp; nout <= noutp;
    go_i <= gop; w0_i <= w0p; w1_i <= w1p;
    last_r <= last0; hold_r <= wt0;
    ph <= (kp == `FK_GEN1) ? 2'd2 : (kp == `FK_GENM) ? 2'd1 : wt0 ? 2'd3 : 2'd0;
    off <= 8'd0;
  end else if (act && adv) begin                             // the next uop: last, hold and phase one uop ahead
    n <= n1; rem <= rem - 9'd1;
    last_r <= (rem == 9'd1);
    hold_r <= (n1 >= w0_i) && (n1 + 9'd1 < w1_i);
    ph <= (n1 == go_i) ? 2'd2 : (n1 < go_i && knd == `FK_GENM) ? 2'd1 : (n1 >= w0_i && n1 < w1_i) ? 2'd3 : 2'd0;
    if (piece) begin                                         // the displacement moves on after each EA piece
      off <= (knd == `FK_REST) ? ((off == 8'd0) ? 8'h18 : 8'h38) : off + 8'd4;
    end
    if (knd == `FK_MVM) begin                                // three words per register, then the next register
      if (wd == 2'd2) begin wd <= 2'd0; msk <= dsc ? (msk & ~mtop) : (msk & (msk - 8'd1)); end else wd <= wd + 2'd1;
    end else if (knd == `FK_CTLM) msk <= msk & ~top3(msk);
  end
end
wire [2:0]  ofm = cw[12:10];             // FMOVE out format
wire [2:0]  lst = cw[12:10];             // the control register list
wire        ser = sp[59];                // a trailing SY_SER
wire        fin = ser ? (rem == 9'd1) : last_r;                      // the last uop before the trailing SY_SER
wire [1:0]  rdi = nout[1:0] - 2'd1 - rem[1:0] + {1'b0, ser};          // = pieces already done

// an EA piece: T0 + off
function [`LA_W-1:0] pla; input dummy;
  reg [`LA_W-1:0] v;
  begin v = {`LA_W{1'b0}}; v[`L_BSEL] = `BA_REG; v[`L_BREG] = `R_T0; v[`L_DISP] = {24'h0, off}; pla = v; end
endfunction

// ---- The uop of the current step -----------------------------------------------------------------------------------
always @* begin
  u = {`UW{1'b0}};
  if (!init) case (knd)
  `FK_GENR, `FK_CR, `FK_GEN1, `FK_GENM: begin                // general operations
    if (fin) begin u[`F_UNIT] = `UN_FPU; u[`F_OP] = `FP_WR; end
    else if (ph == 2'd2) begin u[`F_UNIT] = `UN_FPU; u[`F_OP] = `FP_GO; u[`F_IMM] = {16'h0, cw}; end
    else if (ph == 2'd1) begin                               // FP_IN 0, 1, 2 at T0 + 0, 4, 8
      u[`F_UNIT] = `UN_FPU; u[`F_OP] = `FP_IN; u[`F_SZ] = `SZ_L; u[`F_LSZ] = `SZ_L; u[`F_CC] = {2'b00, n[1:0]};
      u[`F_LA] = pla(1'b0); u[`F_LA_EN] = 1'b1; u[`F_ASEL] = `AS_MEM;
    end                                                      // otherwise: an empty wait uop
  end
  `FK_OUT: begin                                             // FMOVE FPn,<ea>
    if (n == 9'd0 && oxd && m3 != 3'd0) begin                 // first wait: LEA of the EA into T0 with the (An)+ / -(An) step
      u[`F_LA] = ea_lat(m3, r0, `SZ_L, x0, x1, 4'd4, ts, tr); u[`F_LA_EN] = 1'b0; u[`F_LEA] = 1'b1; u[`F_SA_BREG] = `R_T0;
      if (m3 == 3'd3) u[`F_LA_UDL] = tot;
      if (m3 == 3'd4) begin u[`F_LA_UDL] = 8'h0 - tot; u[`F_LA_DISP] = 32'h0 - {24'h0, tot}; end
    end
    if (ph != 2'd3) begin                                    // FP_RD pieces (rem counts down to the last)
      u[`F_UNIT] = `UN_FPU; u[`F_OP] = `FP_RD;
      u[`F_CC] = {1'b0, fin, rdi};
      u[`F_SZ] = (ofm == 3'd4) ? `SZ_W : (ofm == 3'd6) ? `SZ_B : `SZ_L;
      if (m3 == 3'd0) begin                                  // Dn: .B/.W merge into the register (operand B)
        u[`F_BSEL] = `BS_REG; u[`F_BREG] = {2'b00, r0}; u[`F_WEN] = 1'b1; u[`F_WREG] = {2'b00, r0};
      end else begin
        if (nout == 9'd1) begin                              // one piece: the EA with its own (An)+ / -(An) step
          u[`F_LA] = ea_lat(m3, r0, u[`F_SZ], x0, x1, 4'd4, ts, tr); u[`F_LA_EN] = 1'b0;
        end else u[`F_LA] = pla(1'b0);
        u[`F_SA_EN] = 1'b1; u[`F_SA_SAME] = 1'b1;            // store at the address computed on the load side
      end
    end
  end
  `FK_CTLR: begin                                          // step 0 was FP_CHK: no architectural change in a FIRST uop
    u[`F_UNIT] = `UN_FPU; u[`F_CC] = {1'b0, lst}; u[`F_SZ] = `SZ_L;
    if (cw[13]) begin                                        // FPcr to Dn / An
      u[`F_OP] = `FP_CR; u[`F_WEN] = 1'b1; u[`F_WREG] = {1'b0, m3[0], r0};
    end else begin                                           // Dn / An / #imm to FPcr
      u[`F_OP] = `FP_CW;
      if (m3 == 3'd7) begin u[`F_ASEL] = `AS_IMM; u[`F_IMM] = {x0, x1}; end
      else begin u[`F_ASEL] = `AS_REG; u[`F_AREG] = {1'b0, m3[0], r0}; end
    end
  end
  `FK_CTLM: begin                                          // one control register per uop, highest list bit first (FPCR)
    u[`F_UNIT] = `UN_FPU; u[`F_OP] = cw[13] ? `FP_CR : `FP_CW; u[`F_CC] = {1'b0, tp3[2:0]}; u[`F_SZ] = `SZ_L; u[`F_LSZ] = `SZ_L;
    u[`F_LA] = pla(1'b0);
    u[`F_LA_EN] = !cw[13];
    if (cw[13]) begin u[`F_SA_EN] = 1'b1; u[`F_SA_SAME] = 1'b1; end else u[`F_ASEL] = `AS_MEM;
  end
  `FK_MVM: begin                                           // FMOVEM.X: three longs per register
    if (nreg == 4'd0) begin u[`F_UNIT] = `UN_FPU; u[`F_OP] = `FP_CHK; end   // empty list: one FPU uop (NULL state)
    else begin
      u[`F_UNIT] = `UN_FPU; u[`F_SZ] = `SZ_L; u[`F_LSZ] = `SZ_L; u[`F_CC] = {1'b0, cw[13] ? 1'b0 : (wd == 2'd2), wd};
      u[`F_IMM] = {29'h0, mvr};
      u[`F_LA] = pla(1'b0);
      if (cw[13]) begin u[`F_OP] = `FP_RX; u[`F_SA_EN] = 1'b1; u[`F_SA_SAME] = 1'b1; end   // to memory
      else begin u[`F_OP] = `FP_IN; u[`F_LA_EN] = 1'b1; u[`F_ASEL] = `AS_MEM; end         // to registers (the third word writes FPn)
    end
  end
  `FK_COND: begin                                          // the conditional as an integer uop with the FP mark
    u[`U_FPC] = 1'b1;
    if (cw[15:12] == 4'hF) begin                             // FBcc: cw is the opcode, x0:x1 the displacement words
      u[`F_UNIT] = `UN_BRA; u[`F_CC] = cw[3:0]; u[`U_FPS] = cw[4];
      u[`F_LA_BSEL] = `BA_PC; u[`F_LA_XK] = 4'd2; u[`F_LA_DISP] = cw[6] ? {x0, x1} : {{16{x0[15]}}, x0};
    end else begin
      u[`F_CC] = cw[3:0]; u[`U_FPS] = cw[4];
      if (m3 == 3'd1) begin                                  // FDBcc Dn,disp (disp word at PC + 4)
        u[`F_UNIT] = `UN_DBR; u[`F_OP] = `OP_SUB; u[`F_SZ] = `SZ_W; u[`F_ASEL] = `AS_IMM; u[`F_IMM] = 32'd1;
        u[`F_BREG] = {2'b00, r0}; u[`F_WEN] = 1'b1; u[`F_WREG] = {2'b00, r0};
        u[`F_LA_BSEL] = `BA_PC; u[`F_LA_XK] = 4'd4; u[`F_LA_DISP] = {{16{x0[15]}}, x0};
      end else if (m3 == 3'd7 && r0 >= 3'd2 && r0 <= 3'd4) begin   // FTRAPcc (vector 7)
        u[`F_UNIT] = `UN_SYS; u[`F_OP] = `SY_TRAPCC; u[`F_EXC] = 8'd7;
      end else begin                                         // FScc
        u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_SCC; u[`F_SZ] = `SZ_B; u[`F_LSZ] = `SZ_B; u[`F_ASEL] = `AS_IMM;
        if (m3 == 3'd0) begin u[`F_BREG] = {2'b00, r0}; u[`F_WEN] = 1'b1; u[`F_WREG] = {2'b00, r0}; end
        else begin
          u[`F_LA] = ea_lat(m3, r0, `SZ_B, x0, x1, 4'd4, ts, tr);
          u[`F_BSEL] = `BS_MEM; u[`F_SA_EN] = 1'b1; u[`F_SA_SAME] = 1'b1;
        end
      end
    end
  end
  `FK_SAVE: begin                                          // FSAVE: one store per frame word (fin marks the last)
    u[`F_UNIT] = `UN_FPU; u[`F_OP] = `FP_SV; u[`F_CC] = n[3:0]; u[`F_SZ] = `SZ_L; u[`F_IMM] = {31'h0, fin};
    u[`F_LA] = pla(1'b0);
    u[`F_SA_EN] = 1'b1; u[`F_SA_SAME] = 1'b1;
  end
  `FK_REST: begin
    // pieces at EA + 0x18 and EA + 0x38 (never stepped; (An)+ is written at the end from the frame size)
    if (n <= 9'd2) begin
      u[`F_UNIT] = `UN_FPU; u[`F_OP] = `FP_RS; u[`F_CC] = {2'b00, n[1:0]}; u[`F_SZ] = `SZ_L; u[`F_LSZ] = `SZ_L;
      u[`F_LA] = pla(1'b0); u[`F_LA_EN] = 1'b1; u[`F_ASEL] = `AS_MEM;
    end else if (n == 9'd3) begin u[`F_UNIT] = `UN_FPU; u[`F_OP] = `FP_RSC; u[`F_EXC] = 8'd14; end   // commit (format error: vector 14)
    else if (n == 9'd4 && m3 == 3'd3) begin u[`F_UNIT] = `UN_FPU; u[`F_OP] = `FP_RSA; u[`F_SZ] = `SZ_L; u[`F_WEN] = 1'b1; u[`F_WREG] = `R_T0; end
    else if (n == 9'd5 && m3 == 3'd3) begin                  // An += T0 (the frame size)
      u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_ADD; u[`F_SZ] = `SZ_L; u[`F_ASEL] = `AS_REG; u[`F_AREG] = `R_T0;
      u[`F_BSEL] = `BS_REG; u[`F_BREG] = {2'b01, r0}; u[`F_WEN] = 1'b1; u[`F_WREG] = {2'b01, r0};
    end else begin u[`F_UNIT] = `UN_SYS; u[`F_OP] = `SY_SER; end
  end
  default: ;
  endcase
  if (!init && ser && last_r) begin u = {`UW{1'b0}}; u[`F_UNIT] = `UN_SYS; u[`F_OP] = `SY_SER; end   // DS rereads FPCR / the pending trap
  if (!act) u = {`UW{1'b0}};
end
endmodule
`default_nettype wire
