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
// ==== p68_ex.v: the execute stage (EX), lane A and lane B ============================================================
//
// Abbreviations used in this file: AG = address generation, OC1/OC2 = the operand stages, EX = execute (this file), WB =
// write-back, DS = decode, BTC = branch target cache, CCR = condition code register (the low byte of the status
// register SR, holding the flags X N Z V C), FPU = floating-point unit, FPSR = its status register, BSUN = the FPU's
// "branch or set on unordered" exception, smc = self-modifying code, ALU = arithmetic and logic unit.
//
// ---- What this unit is for ----------------------------------------------------------------------------------------
//
// EX is where the work of an instruction is actually done, and where the machine decides that it is done. It takes the
// uop from OC2 with its two operands (A and B), computes the result in the unit the uop names, computes the flags, and
// decides whether the uop commits. A uop commits when it leaves EX without a replay and without an interrupt taken in
// front of it; only a committed uop changes the CCR and the registers, so every flag and register value is "precise"
// (a replayed or flushed uop leaves no trace). When the uop is the last of its instruction (LAST), the instruction
// retires. EX also decides the redirects: a branch that went the other way than predicted, a replay, an exception, an
// SR write. The redirect target is ex_target, and the front end follows it one clock later.
//
// The units:
//   - The adder (ADD, SUB, CMP, NEG and the extended forms ADDX, SUBX, NEGX that take X as a carry in): one carry chain.
//   - The logic operations AND, OR, EOR, MOVE (result A), and the byte pick of MOVEP.
//   - One 32-bit rotator for every shift and rotate (ASL, ASR, LSL, LSR, ROL, ROR, ROXL, ROXR) with masks for the shifts.
//   - p68_exi.v (Scc, the BCD steps, the bit operations BTST/BCHG/BCLR/BSET, PACK/UNPK), p68_md.v (multiply and divide),
//     p68_bf.v (bit fields), p68_sys.v (SR, control registers, exceptions, interrupts) and p68_fpu.v (the FPU), all
//     instantiated here. The long units deliver their results from registers.
//   - Lane B's small ALU (p68_exb.v) for dual issue, with its own operands, flags and write port.
//
// ---- The flags from first principles ------------------------------------------------------------------------------
//
// After most instructions the 68k sets five flags in the CCR: N (negative: the top bit of the result, at the operation's
// size), Z (zero: every bit of the result is 0), V (overflow: a signed result that does not fit, e.g. two positive
// numbers whose sum looks negative), C (carry: the carry out of the top bit for an add, or the borrow for a subtract or
// compare), and X (extend: a copy of C for the arithmetic operations, kept separately so that a CMP or a MOVE between
// two halves of a 64-bit add does not lose the carry; ADDX, SUBX, NEGX and the BCD operations use X as their carry in).
// Which flags a uop writes and where C and V come from is in the uop's flag mode fields (FEN, FCV, FZM, p68_uop.vh). The
// condition codes of Bcc, DBcc, Scc and TRAPcc are 16 named tests on N, Z, V and C (HI, LS, CC, CS, NE, EQ, VC, VS, PL,
// MI, GE, LT, GT, LE, and true and false), and the table condf below lists them.
//
// The flags are completed in WB, one clock after the result. At commit EX registers the final C, N, Z and V it has
// computed together with the mask of the flags the uop writes; WB merges them into the committed CCR. The Z of a shift
// or a p68_exi result is the exception: it is formed from the result in WB (a zero test on 32 bits after the rotator
// was too deep for EX's clock). A uop in EX sees the CCR including the uop in WB (ccr_e) through two LUT levels, and AG
// keeps a flag reader one clock behind a uop whose Z comes from WB (see p68_ag.v). Completing flags in WB was also
// needed for dual issue, where two flag writers commit per clock.
//
// ---- Forwarding into EX ---------------------------------------------------------------------------------------------
//
// The operands come from OC2, which read them from the register file. But the uop just ahead (now in EX, in WB next
// clock) may write one of them, and its value is not in the register file yet. So when a uop enters EX, the check
// "does the uop now in EX (lane A or lane B) write my operand register?" is made and registered (n_afw, n_afb and so
// on), and in EX the operand is then taken from WB's value (wb_val, wb2_val) instead of the registered operand. That is
// "WB forwarding". The selects are one-hot registers with the sign extension folded in, so an operand is one AND-OR of
// two-input terms.
//
// ---- What was tried, and why it ended up like this ----------------------------------------------------------------
//
// The EX history is a long list of small cuts, nearly all kept: Z of the adder from the operands without the carry
// chain in front of it, N from the sum, the result as one flat AND-OR of twelve terms (a merged form tried once cost a
// level and was undone), operand selects as AND-ORs of one-hot registered controls (two levels instead of four), the
// final flags of both lanes registered at commit so the condition logic is two levels, the branch condition as a
// 16-entry truth table registered on entry to EX, and the shifter's count, masks and rotator input registered one stage
// earlier. The shifts, the bit fields, the p68_exi operations and CHK, CHK2 and TRAPcc take no WB forwarding at all;
// AG keeps them two behind the writer of their registers instead, which costs a clock only when they really follow
// each other. A round of EX register copies aimed at the full Amiga system grew the CPU by 6-8 % and cost about 4 MHz
// in the full build, so it was reverted. Carry-chain primitives for the small adders and zero tests, and hand-placed
// LUT4 primitives for the flag terms, were built in the level-reduction work and not merged.
//
// ---- Gowin techniques in this file --------------------------------------------------------------------------------
//
//  - syn_keep on r_reg, r_ab and n_xe keeps the early parts of the result and of N apart, so the late sources join in
//    the last LUT instead of at the head of a chain.
//  - syn_preserve on the copies of ex_v, exb_v, the AG copies and the lane B select tables keeps them from being merged
//    with their originals.
//  - syn_maxfan = 16 on the module.
//
// Runs on the CPU clock clk.
`include "p68_uop.vh"                // the uop format
`include "p68_fpu.vh"                // UN_FPU and the FP conditional bits
`include "p68_sys.vh"                // system unit operation codes
`default_nettype none
module p68_ex (
  input  wire        clk,            // CPU clock
  input  wire        rst,            // synchronous reset
  // ---- from OC2 ----
  input  wire        oc_v,           // a lane A uop is in OC2
  input  wire [`UW-1:0] oc_u,        // ... the uop
  input  wire [31:0] op_a, op_b,     // ... its operands
  input  wire [31:0] oc_addr,        // ... its load address (also the target of a branch)
  input  wire [31:0] oc_saddr,       // ... its store address
  input  wire        st_hit,         // ... the store's line is in the D-cache
  input  wire        oc_rep,         // ... it must replay (OC's causes)
  input  wire        q_rpb,          // the replay includes the older lane B (swapped pair); always 0, decided here
  input  wire        oc_miss,        // ... its load missed
  input  wire        oc_w1_en, oc_w2_en,   // ... its AG writes (committed here into the register file's committed copy)
  input  wire [4:0]  oc_w1_r, oc_w2_r,
  input  wire [31:0] oc_w1_v, oc_w2_v,
  input  wire [31:0] wb_val,         // lane A's value in WB (forwarded into EX's operands)
  // ---- misaligned accesses ----
  input  wire [31:0] oc_laddr,       // the load address for a fill
  input  wire        st_hit2,        // the second word of a misaligned store is in the D-cache
  output reg         ex_hit2,        // ... registered for WB
  // ---- stage registers ----
  output reg         ex_v,           // a lane A uop is in EX
  output reg  [`UW-1:0] ex_u,        // ... the uop
  output reg  [31:0] ex_saddr,       // ... its store address
  output reg         ex_hit,         // ... its store hits the D-cache
  output wire        ex_rep,         // ... it replays (OC's causes, or the store-into-code replay decided here)
  output reg         ex_miss,        // ... its load missed
  output reg  [31:0] ex_laddr,       // ... its load address
  output reg  [31:2] ex_laddr1,      // ex_laddr + 4 (the D-cache's next word, registered)
  output reg         ex_w1_en, ex_w2_en,   // its AG writes
  output reg  [4:0]  ex_w1_r, ex_w2_r,
  output reg  [31:0] ex_w1_v, ex_w2_v,
  output reg  [7:0]  ex_h1,          // ex_w1's register one-hot (A0-A7), registered with it
  output reg  [11:0] ex_h2,          // ex_w2's register one-hot (A0-A7, T0-T3)
  output wire        ex_wen,         // the EX uop writes its register (with the DBcc and CAS conditions)
  output wire [4:0]  ex_wreg,        // ... this register
  output wire        ex_st,          // the EX uop stores (with the CAS condition)
  output wire        ex_commit,      // the EX uop commits
  output wire [31:0] rm,             // the result, merged into the destination by size (a .B write keeps bits 31-8)
  output wire [4:0]  ccr_o,          // X N Z V C after the uop in WB (the flags are completed in WB)
  output wire        redirect_ex,    // EX redirects the front end
  output wire        redirect_rc,    // copy of redirect_ex (for rs_d and the register file's restore copies)
  output wire [31:0] ex_target,      // the redirect target
  // ---- system unit ports (see p68_sys.v) ----
  input  wire [2:0]  ipl,            // interrupt request level
  output wire        no_ret,         // the committing last uop does not retire its instruction
  output wire        rs_one,         // RTE of a throwaway frame: restart at step 1
  output wire        sup, msp, stopped, rst_done,   // S and M bits; STOP state; the RESET instruction's count is done
  output wire        xs_go,          // start the exception sequence
  output wire [154:0] xs_par,        // ... its parameters
  output wire        xs_rs,          // restart the exception sequence (a replay inside it)
  output wire [4:0]  xs_rs_step,     // ... at this step
  output wire        sw_en,          // swap A7 with a stack pointer bank
  output wire [1:0]  sw_old, sw_new,
  output wire        iack,           // interrupt acknowledge
  output wire [2:0]  iack_lvl,
  output wire        reset_o,        // RESET instruction output
  output wire [15:0] sr,             // the status register
  output wire [31:0] vbr, caar,      // vector base register, cache address register
  output wire [2:0]  sfc, dfc,       // function code registers (MOVES)
  output wire [3:0]  cacr,           // cache control register
  // ---- caches and prediction ----
  input  wire        irq_hold,       // an uncached read is pending: no interrupt take
  output wire        ex_irq,         // an interrupt is taken in front of the uop in EX (it does not commit)
  output wire        iwant,          // an unmasked interrupt is requested (register, p68_sys)
  input  wire        oc_pmiss,       // the predicted target was wrong (OC)
  output wire        ex_brc,         // a Bcc/BRA commits: branch target cache update
  output wire        ex_brt,         // ... taken
  output wire [31:0] ex_btgt,        // ... its target
  // ---- lane B (dual issue): from OC2, its WB register write and retire record ----
  input  wire [`UW-1:0] q_ub,        // lane B's uop in OC2
  input  wire        q_ubv,
  input  wire [31:0] opb_a, opb_b,   // its operands
  output reg         exb_v,          // lane B is in EX
  output reg  [`UW-1:0] exb_u,       // ... the uop
  output reg         wb2_wen,        // lane B writes its register in WB
  output reg  [3:0]  wb2_wreg,       // ... this register (D0-A7)
  output reg  [7:0]  wb2_h, wb2_hd,  // lane B's write hits A0-A7 / D0-D7 (one-hot, with wb2_wen)
  output reg  [31:0] wb2_val,        // ... this value
  output wire [4:0]  ccr_a,          // CCR after lane A's instruction (retire port)
  output wire [4:0]  ccr_b,          // CCR after lane B's instruction (retire port)
  output reg         g_o,            // the lane B in WB is the older one (retire port)
  output wire        ex_bb,          // the committing branch (ex_brc) is lane B's
  input  wire        rs_d,           // redirect_ex one clock ago
  input  wire        smc_r,          // the store that committed last clock hit fetched code (register): the uop here replays
  output wire [31:0] wb2_vg,         // copy of wb2_val (AG forwarding)
  output wire        ex_vg, ex_wug,  // copies of ex_v and the EX uop's WEN for AG's interlock
  output wire [4:0]  ex_wrg,         // copy of the EX uop's WREG
  output wire        ex_cwg,         // copy of "the EX uop writes only by its condition" (DBcc, CAS)
  output wire        exb_wug,        // copy of exb_v && lane B's WEN (AG interlock)
  output wire [3:0]  exb_wrg,        // copy of lane B's WREG
  output reg         ret2_v,         // lane B retires (retire port)
  output reg  [31:0] ret2_pc,        // ... its PC
  // ---- FPU ----
  output wire        fpu_null,           // the FPU is in its NULL state (DS: FSAVE frame size)
  output wire        fpu_en,             // an FPCR exception enable is set (DS)
  output wire [5:0]  fpu_pv,             // pending FP trap vector (DS)
  output wire [79:0] ret_fp0, ret_fp1, ret_fp2, ret_fp3, ret_fp4, ret_fp5, ret_fp6, ret_fp7,   // FP0-FP7 (retire port)
  output wire [31:0] ret_fpcr, ret_fpsr, ret_fpiar   // the FPU control registers (retire port)
) /* synthesis syn_maxfan = 16 */;
reg  [31:0] ex_a, ex_b, ex_tgt;      // lane A's operands as read in OC2; the branch target (the load address)
// ---- Operand preparation ------------------------------------------------------------------------------------------
// The adder inputs are prepared before EX: A extended and inverted for SUB, CMP and SUBX (ex_ax), B inverted for NEG
// and NEGX (ex_bq). A subtract b - a is computed as b + ~a + 1, and a negate 0 - b as ~b + 1, so the adder itself is
// always an add. A forwarded WB value gets the same inversion in front of the forwarding select (one LUT level per bit).
reg  [31:0] ex_ax, ex_bq;            // the prepared adder inputs (register sources)
reg         ex_afw, ex_bfw, ex_afb, ex_bfb;   // operand A / B forwarded from WB lane A (afw, bfw) or lane B (afb, bfb)
wire        ex_afwd = ex_afw | ex_afb, ex_bfwd = ex_bfw | ex_bfb;   // forwarded from either lane
wire [31:0] wfa = ex_afb ? wb2_val : wb_val, wfb = ex_bfb ? wb2_val : wb_val;   // the forwarded A / B value
// The forwarding selects are registered one-hot with the A extension folded in, so an operand is one AND-OR of
// two-input terms (WB lane A, WB lane B, the register) instead of a select, a forwarding select and the extension.
// fs: {R, B0, B1, Bb, Bw, Bx, A0, A1, Ab, Aw, Ax}: A* lane A's WB value, B* lane B's (wins), R the operand register;
// x / w / b the extension none / word / byte, A1 = Ax | Aw (bits 8-15 from the value), A0 = any A (bits 0-7).
function [10:0] fsv; input fw, fb; input [1:0] x;   // forwarded from lane A, from lane B; extension
  reg fa;
  begin
    fa = fw && !fb;                  // lane B is younger, so its value wins when both write
    fsv = {!(fw || fb), fb, fb && x != `AX_SXB, fb && x == `AX_SXB, fb && x == `AX_SXW, fb && x != `AX_SXB && x != `AX_SXW,
           fa, fa && x != `AX_SXB, fa && x == `AX_SXB, fa && x == `AX_SXW, fa && x != `AX_SXB && x != `AX_SXW};
  end
endfunction
// fwv: the forwarded part of an operand (0 when the register is used), with the extension: bits 0-7 always from the
// value, bits 8-15 from the value or the byte's sign, bits 16-31 from the value or the byte's or word's sign
function [31:0] fwv; input [31:0] w, w2; input [10:0] f;          // the forwarded part (0 when R)
  integer i;
  begin
    for (i = 0; i < 32; i = i + 1)
      fwv[i] = (i < 8)  ? ((f[4] & w[i]) | (f[9] & w2[i])) :
               (i < 16) ? ((f[3] & w[i]) | (f[8] & w2[i]) | (f[2] & w[7]) | (f[7] & w2[7])) :
                          ((f[0] & w[i]) | (f[5] & w2[i]) | (f[2] & w[7]) | (f[7] & w2[7]) | (f[1] & w[15]) | (f[6] & w2[15]));
  end
endfunction
reg  [10:0] ex_fa, exb_fa;               // lane A / lane B operand A selects
reg  [2:0]  ex_fb, exb_fb;               // operand B: {R, lane B, lane A}
reg         ex_subf, ex_negf, exb_subf, exb_ngf;   // the inversion of a forwarded A / B
reg  [31:0] ex_ae;                       // operand A extended (the register term of a)
reg  [31:0] ex_as /* synthesis syn_preserve = 1 */;   // its own copy for p68_sys (CHK2) and p68_exi
wire [31:0] fa_a = fwv(wb_val, wb2_val, ex_fa);                       // the forwarded part of A
wire [31:0] fb_a = ({32{ex_fb[0]}} & wb_val) | ({32{ex_fb[1]}} & wb2_val);   // the forwarded part of B
reg         ex_pmiss;                // the predicted target was wrong (from OC2)
wire        bmis, tk_b;              // lane B's branch: mispredicted; taken
wire [31:0] tgt_b;                   // lane B's branch target
reg  [31:0] exb_npc;                 // lane B's next instruction address

reg         cond;                    // the condition CC of the EX uop holds (on ccr_e)
wire        dbcc = (ex_u[`F_UNIT] == `UN_DBR);   // DBcc writes Dn only if the condition is false
wire        cas  = (ex_u[`F_UNIT] == `UN_CAS);   // CAS: store and flags if cond (the compare matched), register write if !cond
assign ex_wen    = ex_u[`F_WEN] && !(dbcc && cond) && !(cas && cond);
assign ex_wreg   = ex_u[`F_WREG];
assign ex_st     = ex_u[`F_SA_EN] && !(cas && !cond);
wire        irq_take, sy_red;        // an interrupt is taken in front of the EX uop; the system unit redirects
wire        irq_pnd, sy_cq;          // from p68_sys: an interrupt is pending; the latched CHK condition
// RTE frame format check. The format word's top nibble must be 0, 1 or 2 (the frames this core writes); it is checked
// on each registered source of A (after the A extension) and the forwarding selects last.
function fbf; input [31:0] v; input sxb; reg [3:0] f; begin f = sxb ? {4{v[7]}} : v[15:12]; fbf = !(f == 4'd0 || f == 4'd1 || f == 4'd2); end endfunction
wire        fbad = ex_afwd ? fbf(wfa, ex_u[`F_AEXT] == `AX_SXB) : fbf(ex_a, ex_u[`F_AEXT] == `AX_SXB);
wire [31:0] sy_r, sy_tgt;            // the system unit's result; its redirect target
wire [4:0]  sy_ccr;                  // its CCR
reg  [4:0]  ccr;                     // the CCR of the uops that left WB (the committed CCR)
reg  [4:0]  ccr_f, ccr_e;            // the CCR after the uop in WB (full, and the EX copy below)
reg  [31:0] ex_npc;                  // address of the next instruction (stacked PC, SR writes, not-taken branches)
// The uop behind a store into fetched code replays from EX (smc_r, or smc_pe when EX was empty in that clock, so the
// replay applies to the next uop that arrives); a swapped pair restarts at its older lane B.
reg         ex_rep_q, smc_pe;        // OC's replay, registered; an smc replay is pending for the next uop
wire        smc_h  = smc_r || smc_pe;
assign      ex_rep = ex_rep_q || smc_h;
always @(posedge clk) begin
  if (rst || rs_d || redirect_ex) smc_pe <= 1'b0;   // a redirect refetches anyway
  else if (smc_r && !ex_v) smc_pe <= 1'b1;
  else if (ex_v) smc_pe <= 1'b0;
end
assign ex_commit = ex_v && !ex_rep && !irq_take;
assign ex_irq    = irq_take;
wire [4:0]  op   = ex_u[`F_OP];      // the operation
wire [1:0]  sz   = ex_u[`F_SZ];      // the size
wire [4:0]  fen  = ex_u[`F_FEN];     // the flags it writes
wire [2:0]  fcv  = ex_u[`F_FCV];     // where C and V come from
wire [3:0]  cc   = ex_u[`F_CC];      // the condition (or a small parameter)

// ---- The adder ------------------------------------------------------------------------------------------------------
wire [31:0] b  = fb_a | ({32{ex_fb[2]}} & ex_b);                 // operand B (one-hot: forwarded, or the register)
wire [31:0] a  = fa_a | ({32{ex_fa[10]}} & ex_ae);                // operand A, extended
// ADDX, SUBX and NEGX take the carry in from X; NEG and NEGX invert B (0 - b = ~b + 1).
wire        sub  = (op == `OP_SUB) || (op == `OP_CMP) || (op == `OP_SUBX);      // A inverted
wire        negb = (op == `OP_NEG) || (op == `OP_NEGX);                          // B inverted
wire        bor  = sub || negb;                                                  // borrow type: C = !carry
// carry in: X for ADDX; !X for SUBX and NEGX (a borrow of 1 is a carry of 0); 1 for SUB, CMP and NEG (the +1 of ~a + 1)
wire        cin  = (op == `OP_ADDX) ? ccr[4] : (op == `OP_SUBX || op == `OP_NEGX) ? !ccr[4] : (sub || op == `OP_NEG);
function [31:0] aextf; input [31:0] v; input [1:0] x;   // sign-extend a word (ADDA.W, MOVEA.W) or a byte (EXT) to a long
  begin aextf = (x == `AX_SXW) ? {{16{v[15]}}, v[15:0]} : (x == `AX_SXB) ? {{24{v[7]}}, v[7:0]} : v; end
endfunction
wire [31:0] bq   = (fb_a ^ {32{ex_negf}}) | ({32{ex_fb[2]}} & ex_bq);   // B, inverted for NEG/NEGX
wire [31:0] ax   = (fa_a ^ {32{ex_subf}}) | ({32{ex_fa[10]}} & ex_ax);  // A, extended, inverted for SUB/CMP/SUBX
// One carry chain with a carry in: an extra low bit of 1 + cin produces exactly cin as the carry into bit 0.
wire [33:0] s2  = {1'b0, bq, 1'b1} + {1'b0, ax, cin};      // one carry chain with carry in
wire [32:0] sum = s2[33:1];                                // the 32-bit sum and the carry out of bit 31
// The carry out of bit 7 and of bit 15 (for byte and word operations): the sum bit is a ^ b ^ carry-in, so the carry
// into bit 8 is sum[8] ^ bq[8] ^ ax[8]. This avoids a second chain for the smaller sizes.
wire        c8  = sum[8] ^ bq[8] ^ ax[8], c16 = sum[16] ^ bq[16] ^ ax[16], c32 = sum[32];

// ---- The shifter: one 32-bit rotator, with masks for the shifts ----------------------------------------------------
// A left shift by n is a rotate left by n with the low n bits cleared. A right shift by n is a rotate left by 32 - n
// with the top n bits cleared (LSR) or filled with the sign (ASR). Rotates of a byte or word use a copy of the byte or
// word in every position of the 32 bits, so a 32-bit rotate gives the right answer in the low byte or word.
wire        sh_left = op[2], sh_rot = op[1], sh_ari = op[0];   // OP_SHF | {left, rotate, arithmetic}
// The shifter takes the EX operand registers only (no WB forwarding); AG keeps a shift two behind the uop that writes
// its count or operand register. The 68k takes a register shift count modulo 64.
wire [5:0]  sc   = ex_a[5:0];
// The rotate amount (with the direction applied), the masks and the count tests are registers, made from op_a in OC2.
reg  [4:0]  ex_rc;                   // the rotate-left amount
reg  [31:0] ex_tlo, ex_thi, ex_tkl, ex_ekx;   // masks: i < count; i >= 32 - count; i < kx; i == kx
reg         ex_shbig, ex_sc0;        // the count is beyond the size; the count is 0
// The rotator input (size extension or replication of B) and B's sign bit are registers, made from op_b in OC2. In
// EX they had made a real failing path of the full system: from the operand through the rotator and the result to WB.
function [32:0] shinf;               // {sign of the extended B, rotator input}
  input [31:0] v; input [1:0] z; input [4:0] o;
  reg [31:0] be, bp;                 // B extended by size (sign for arithmetic shifts); B replicated
  begin
    be = (z == `SZ_B) ? {{24{o[0] & v[7]}},  v[7:0]} : (z == `SZ_W) ? {{16{o[0] & v[15]}}, v[15:0]} : v;
    bp = (z == `SZ_B) ? {4{v[7:0]}} : (z == `SZ_W) ? {2{v[15:0]}} : v;
    shinf = {be[31], (o[2] || o[1]) ? bp : be};   // left shifts and rotates use the replicated form
  end
endfunction
reg  [31:0] ex_rin;                  // the rotator input
reg         ex_bs;                   // the sign of B at its size
wire [31:0] rin  = ex_rin;
// ROXL/ROXR (rotate and arithmetic bit both set) rotate the (n+1)-bit ring {operand, X}. The count is reduced modulo
// n + 1; the result is ROL by (L-1) of the operand with X inserted.
wire        sh_rox = sh_rot && sh_ari;
// The ROX count arrives encoded in A: [5] no rotation (C = X), [4:0] the rotate amount L-1 (made by OP_ROXC or by DS).
wire        rox_z  = ex_a[5];
wire [4:0]  kx     = ex_a[4:0];
wire [4:0]  rc   = ex_rc;            // the rotate amount (a register, no WB forwarding)
// The rotator: five stages, rotating by 1, 2, 4, 8 and 16 under the bits of rc (a barrel rotator, one LUT level each).
wire [31:0] r1 = rc[0] ? {rin[30:0], rin[31]}    : rin;
wire [31:0] r2 = rc[1] ? {r1[29:0],  r1[31:30]}  : r1;
wire [31:0] r3 = rc[2] ? {r2[27:0],  r2[31:28]}  : r2;
wire [31:0] r4 = rc[3] ? {r3[23:0],  r3[31:24]}  : r3;
wire [31:0] rot = rc[4] ? {r4[15:0], r4[31:16]}  : r4;
wire [31:0] tlo = ex_tlo, thi = ex_thi;   // tlo[i] = i < sc ; thi[i] = i >= 32 - sc
wire [31:0] tkl = ex_tkl, ekx = ex_ekx;   // i < kx, i == kx
function [31:0] mlo; input [5:0] n; integer i; begin for (i = 0; i < 32; i = i + 1) mlo[i] = (i < n); end endfunction
function [31:0] mhi; input [5:0] n; integer i; begin for (i = 0; i < 32; i = i + 1) mhi[i] = (i + n >= 32); end endfunction
function [31:0] meq; input [4:0] n; integer i; begin for (i = 0; i < 32; i = i + 1) meq[i] = (i == n); end endfunction
// ROX result: the rotated bits below kx, X at kx, and the bits above shifted up by one (X took one place in the ring)
wire [31:0] rox_r = rox_z ? rot : ((rot & tkl) | ({32{ccr[4]}} & ekx) | ({rot[30:0], 1'b0} & ~tkl & ~ekx));
wire        fill = sh_ari & ex_bs;   // ASR fills with the sign
wire [31:0] shres = sh_rot ? (sh_rox ? rox_r : rot) : sh_left ? (rot & ~tlo) : ((rot & ~thi) | ({32{fill}} & thi));
wire        msb_rot = (sz == `SZ_B) ? rot[7] : (sz == `SZ_W) ? rot[15] : rot[31];   // the top bit of the result at its size
wire        sh_big = ex_shbig;                                                     // count beyond the size
// C of a shift is the last bit shifted out: after a left rotate by n it sits in bit 0, after a right shift (rotate by
// 32 - n) in bit 31. A count of 0 clears C; a count beyond the size shifts everything out (C = 0, or the sign for ASR).
// A rotate's C is the bit that went round last (bit 0 after ROL, the top bit after ROR); ROX's C is the new X.
wire        sh_c = sh_rox ? (rox_z ? ccr[4] : msb_rot) : ex_sc0 ? 1'b0 : sh_rot ? (sh_left ? rot[0] : msb_rot) :
                   sh_big ? (!sh_left && sh_ari && ex_bs) : sh_left ? rot[0] : rot[31];

// ---- Operations 20-31: the extra integer unit (Scc, BCD, bit operations, PACK/UNPK) ----------------------------------
wire [31:0] ri; wire c_bcd, v_bcd, n_bcd, z_bit, z_keep;
// its operands are the EX registers without WB forwarding (AG keeps these uops two behind their writers)
p68_exi i_exi (.op(op), .sz(sz), .a(ex_as), .b(ex_b),   /* unforwarded operands (AG gap) */ .x(ccr[4]), .cond(cond), .left(cc[0]), .ri(ri), .c_bcd(c_bcd), .v_bcd(v_bcd), .n_bcd(n_bcd), .z_bit(z_bit), .z_keep(z_keep));

// ---- Multiply and divide unit (UN_MUL, UN_DIV): the result and flags come from registers of p68_md -----------------
wire        md_u = (ex_u[`F_UNIT] == `UN_MUL) || (ex_u[`F_UNIT] == `UN_DIV);
wire [31:0] md_r;                    // its result
wire [4:0]  md_ccr;                  // its flags
wire        md_dz;                   // divide by zero (raises vector 5)
p68_md i_md (.clk(clk), .ex_v(ex_v), .ex_rep(ex_rep), .ex_u(ex_u), .a(a), .b(b), .ccr(ccr_e),
             .md_r(md_r), .md_ccr(md_ccr), .md_dz(md_dz));
// UN_SYS and UN_BFU (7 and 6) both deliver registered results; the select needs only the unit field.
wire [31:0] bf_r; wire [4:0] bf_ccr;   // the bit field unit's result and flags
wire        is_u = (ex_u[`F_UNIT] >> 1) == 3'b011;   // unit 6 or 7
// The flag decode of the uop in EX, registered when it enters EX (block fdec below).
reg         ex_ad, ex_bor;           // an adder op; a borrow-type op
reg  [4:0]  ex_rl;                   // one-hot logic op: {byte pick, AND, OR, EOR, MOVE}
reg  [2:0]  ex_nr, ex_zm;            // N/Z source of non-adder ops: {p68_exi, shifter, logic}; Z mode {old B, bit, set}
reg  [3:0]  ex_fv;                   // C/V source: {BCD, shifter, adder for a non-adder op, adder}
reg  [2:0]  szh;                     // one-hot size {L, W, B}
function nszh; input [31:0] v; nszh = (szh[0] & v[7]) | (szh[1] & v[15]) | (szh[2] & v[31]); endfunction   // N at the size
function zszh; input [31:0] v; zszh = (szh[0] & (v[7:0] == 8'h0)) | (szh[1] & (v[15:0] == 16'h0)) | (szh[2] & (v == 32'h0)); endfunction   // Z at the size
wire        is_ad = ex_ad;

// ---- The result -----------------------------------------------------------------------------------------------------
// The result is an AND-OR of one-hot selects from the uop fields (registered), followed by the size merge with B, so
// the late sources (the sum, the shifter, p68_exi) pass about three LUT levels to WB instead of a priority chain. The
// one-hot selects are registers, decoded from the uop entering EX (selv). A UN_SYS uop whose result is A (or A
// sign-extended, MOVX) takes the A terms of r_ab (s_mov, s_xb, s_xw), so the system unit's result (sy_r) has register
// sources only, selected one-hot by ex_sys, with no op decode and no A behind it.
function sya; input [`UW-1:0] x;                       // UN_SYS with a register-source result (sy_r)
  sya = (x[`F_OP] == `SY_RDSR) || (x[`F_OP] == `SY_RDCCR) || (x[`F_OP] == `SY_MCR) || (x[`F_OP] == `SY_RTEJ) || (x[`F_OP] == `SY_NPC);
endfunction
// selv: the 14 one-hot result selects of uop x (see the s_* and u_* names below for the order)
function [13:0] selv; input [`UW-1:0] x;
  reg [4:0] o; reg iu, mu, ur, rl, ia, fu, sy, sr, mx;   // op; unit 6/7; mul/div; FPU; plain ALU; logic op; adder op; sys; sys reg result; MOVX
  begin
    o = x[`F_OP]; iu = (x[`F_UNIT] >> 1) == 3'b011; mu = (x[`F_UNIT] == `UN_MUL) || (x[`F_UNIT] == `UN_DIV);
    fu = (`P68_FPU != 0) && (x[`F_UNIT] == `UN_FPU);
    ur = !iu && !mu && !fu; rl = ur && !o[4] && !o[3];
    ia = o[4] ? (o[3:2] == 2'b00) : (!o[3] && (o[2:0] == 3'd0 || o[2:0] == 3'd1 || o[2:0] == 3'd4));
    sy = iu && x[`U_UNIT]; sr = sy && sya(x); mx = sy && !sr && (o == `SY_MOVX);
    selv = {mx && x[`F_LSZ] == `SZ_B, mx && x[`F_LSZ] == `SZ_W,
            fu, sr, iu && !x[`U_UNIT], !iu && mu, ur && ia, ur && o[4] && (o[3:2] != 2'b00), ur && !o[4] && o[3],
            rl && o[2:0] == 3'd6, rl && o[2:0] == 3'd2, rl && o[2:0] == 3'd3, rl && o[2:0] == 3'd7,
            (rl && o[2:0] == 3'd5) || (sy && !sr && !(mx && (x[`F_LSZ] == `SZ_B || x[`F_LSZ] == `SZ_W)))};
  end
endfunction
// the system unit's result source, one-hot: SR, CCR, latched A7 (RTEJ), next PC, MOVEC SFC, DFC, CACR, VBR, CAAR
function [8:0] sysv; input [`UW-1:0] x;
  reg m; reg [2:0] c;                // a system uop; the control register index
  begin
    m = (x[`F_UNIT] == `UN_SYS); c = x[`U_IMM +: 3];
    sysv = {m && x[`F_OP] == `SY_MCR && c != `CR_SFC && c != `CR_DFC && c != `CR_CACR && c != `CR_VBR,
            m && x[`F_OP] == `SY_MCR && c == `CR_VBR, m && x[`F_OP] == `SY_MCR && c == `CR_CACR,
            m && x[`F_OP] == `SY_MCR && c == `CR_DFC, m && x[`F_OP] == `SY_MCR && c == `CR_SFC,
            m && x[`F_OP] == `SY_NPC, m && x[`F_OP] == `SY_RTEJ, m && x[`F_OP] == `SY_RDCCR, m && x[`F_OP] == `SY_RDSR};
  end
endfunction
reg  [8:0]  ex_sys;                  // sysv of the EX uop
// The system unit's exception and redirect decode of the uop in EX, registered (p68_sys's sy_dec):
// {MCLR, JMP/RTEJ, SER class, raise always, FPU, DIV, RTEF, TRAPCC, XCHK, EXC raises (exc[5:0] != 0, exc[7:6] == 0)}
function [9:0] sdecv; input [3:0] un; input [4:0] o; input [1:0] z; input [7:0] e;
  reg us, fu;                        // a system uop; an FPU uop
  begin
    us = (un == `UN_SYS); fu = (`P68_FPU != 0) && (un == 4'd10);
    sdecv = {us && o == `SY_MCLR, us && (o == `SY_JMP || o == `SY_RTEJ),
             // the SER class: a word write of SR (it may change S, M or the mask), STOP, and the redirect step SER
             us && (((o == `SY_MVSR || o == `SY_ANSR || o == `SY_ORSR || o == `SY_EOSR) && z == `SZ_W) || o == `SY_STOP || o == `SY_SER),
             !us && un != `UN_DIV && !fu, fu, un == `UN_DIV, us && o == `SY_RTEF, us && o == `SY_TRAPCC, us && o == `SY_XCHK,
             e[5:0] != 6'd0 && e[7:6] == 2'b00};
  end
endfunction
reg  [9:0]  ex_sd;                   // sdecv of the EX uop
reg         rdK, rdD, rdR, rdF;      // redirect decodes of the uop in EX (see the redirect below)
reg         ex_vc /* synthesis syn_preserve = 1 */;   // copy of ex_v (for redirect_rc)
reg         exb_vc /* synthesis syn_preserve = 1 */;  // copy of exb_v (for redirect_rc's lane B term)
reg  [3:0]  brR, bmR;                // lane A's branch redirect / lane B's miss, per {condition, B.W == 0}
// brf: does a branch uop redirect, for condition c and B.W zero z? isb: Bcc; isd: DBcc; pr: predicted taken; psk: skip
// pair (never redirects); bpb: a BTC-predicted BRA; pm: the predicted target was wrong.
function brf; input isb, isd, pr, psk, bpb, pm, c, z;
  reg dt, tk, np;                    // DBcc taken (condition false and the counter not at -1); taken; predicted but not taken
  begin
    dt = isd && !c && !z; tk = isb ? c : isd ? dt : 1'b1; np = pr && ((isb && !c) || (isd && !dt));
    // predicted and not taken: redirect to the next instruction; predicted and taken with a wrong target, or not
    // predicted and taken: redirect to the target
    brf = !psk && (np || (pr ? (tk && pm) : ((isb && c) || dt)) || (bpb && pm));
  end
endfunction
function bmf; input isb, isd, isr, pf, c, z;   // lane B's branch mispredicted: isbr_b && (!tk_b || PFIX)
  bmf = (isb || isd || isr) && (!(isb ? c : isd ? (!c && !z) : 1'b1) || pf);
endfunction
reg  [13:0] ex_sel;                  // selv of the EX uop
wire        s_xb = ex_sel[13], s_xw = ex_sel[12];   // MOVX: A sign-extended from a byte / a word
wire        u_fp = ex_sel[11];                       // the FPU's result
wire        u_sy = ex_sel[10], u_bf = ex_sel[9], u_md = ex_sel[8];   // system unit (register source), bit field, mul/div
wire        s_sum = ex_sel[7], s_ri = ex_sel[6], s_sh = ex_sel[5];   // the adder, p68_exi, the shifter
wire        s_bx = ex_sel[4], s_and = ex_sel[3], s_or = ex_sel[2], s_xor = ex_sel[1], s_mov = ex_sel[0];   // the A/B ops
// The early sources are merged beside the carry chain and the shifter; the late ones join last (three terms).
reg         s_oth;                   // one of the early sources is selected
wire [13:0] sv_n = selv(oc_u);       // the selects of the uop entering EX
// The registered unit results and the operand ops as kept sums, so p68_exi's result (ri) joins in one LUT and not at
// the head of a chain of the twelve terms.
wire [31:0] r_reg /* synthesis syn_keep = 1 */;
wire [31:0] r_ab /* synthesis syn_keep = 1 */;
assign r_reg = sy_r | ({32{u_bf}} & bf_r) | ({32{u_md}} & md_r) | ({32{u_fp}} & fp_r);   // sy_r is 0 unless ex_sys selects
assign r_ab  = ({32{s_bx}} & (a >> {cc[1:0], 3'b000})) | ({32{s_and}} & (a & b)) | ({32{s_or}} & (a | b)) |
               ({32{s_xor}} & (a ^ b)) | ({32{s_mov}} & a) | ({32{s_xb}} & {{24{a[7]}}, a[7:0]}) | ({32{s_xw}} & {{16{a[15]}}, a[15:0]});
wire [31:0] r_early = ({32{s_ri}} & ri) | r_ab | r_reg;
wire [31:0] rx = ({32{s_sum}} & sum[31:0]) | ({32{s_sh}} & shres) | ({32{s_oth}} & r_early);   // the full result
// Size merge: a byte write keeps bits 31-8 of the destination, a word write bits 31-16 (B holds the old destination).
wire [31:0] keep_b = {{16{szh[0] | szh[1]}}, {8{szh[0]}}, 8'h0};   // from the one-hot size (registered)
wire        is_scc = s_ri && (op == `OP_SCC);                       // Scc: the condition joins the result last (p68_exi gives 0)
assign rm = (keep_b & b) | (~keep_b & (rx | {32{is_scc && cond}}));
// ---- Flags ----
// N and Z of the result of the non-adder ops, each source tested on its own and then selected.
function zsz; input [31:0] v; input [1:0] s; begin zsz = (s == `SZ_B) ? (v[7:0] == 8'h0) : (s == `SZ_W) ? (v[15:0] == 16'h0) : (v == 32'h0); end endfunction
function nsz; input [31:0] v; input [1:0] s; begin nsz = (s == `SZ_B) ? v[7] : (s == `SZ_W) ? v[15] : v[31]; end endfunction
// The flag sources' selects are registered one-hots (ex_rl, ex_nr, ex_fv, ex_zm) and the sizes the one-hot szh.
wire [31:0] r_l = ({32{ex_rl[4]}} & (a >> {cc[1:0], 3'b000})) | ({32{ex_rl[3]}} & (a & b)) | ({32{ex_rl[2]}} & (a | b)) |
                  ({32{ex_rl[1]}} & (a ^ b)) | ({32{ex_rl[0]}} & a);   // the logic result alone (for its N and Z)
wire n_r = (ex_nr[2] & nszh(ri)) | (ex_nr[1] & nszh(shres)) | (ex_nr[0] & nszh(r_l));   // N of a non-adder op
// Z of the adder ops without the carry chain. The low bits of bq + ax + cin are all 0 iff every bit's operand sum
// bq ^ ax equals its carry-in, and with all lower bits 0 the carry into bit i is bq | ax of bit i - 1 (cin into bit 0).
// So t = 0 over the size is Z, and it is formed in parallel with the carry chain instead of after it.
wire [31:0] zt   = (bq ^ ax) ^ {bq[30:0] | ax[30:0], cin};
wire        z_ad = (sz == `SZ_B) ? (zt[7:0] == 8'h0) : (sz == `SZ_W) ? (zt[15:0] == 16'h0) : (zt == 32'h0);
wire bm  = nszh(bq);                   // the sign of the adder's B input
wire zmo = zszh(b);
wire axm = nszh(ax);                   // the sign of the adder's A input
wire cy  = (szh[0] & c8) | (szh[1] & c16) | (szh[2] & c32);          // the carry out at the size (one-hot size, registered)
// V of the adder ops per size: both inputs have the same sign and the sum's sign differs (selected by szh in v_f)
wire v_a8 = (bq[7] == ax[7]) && (sum[7] != bq[7]), v_a16 = (bq[15] == ax[15]) && (sum[15] != bq[15]);
wire v_a32 = (bq[31] == ax[31]) && (sum[31] != bq[31]);
// ex_fv: {BCD, SHF, ADD of a non-adder op, ADD of an adder op}; ex_bor: borrow type.
// ASL: V is set when the top bit changes at any point during the shift, i.e. when any two neighbor bits differ in the
// top (count + 1) bits of the operand.
wire [31:0] vla = (sz == `SZ_B) ? {b[7:0], 24'h0} : (sz == `SZ_W) ? {b[15:0], 16'h0} : b;   // the operand at the top
wire [31:0] vtg = vla ^ {vla[30:0], 1'b0};                                                 // neighbor bits differ
wire asl_v = sh_left && sh_ari && !sh_rot && (sc != 6'd0) && |(vtg & thi);
wire c_f = ((ex_fv[0] | ex_fv[1]) & (cy ^ ex_bor)) | (ex_fv[2] & sh_c) | (ex_fv[3] & c_bcd);   // C: a borrow is the inverted carry
wire v_f = (ex_fv[0] & ((szh[0] & v_a8) | (szh[1] & v_a16) | (szh[2] & v_a32))) | (ex_fv[1] & (bm == axm) & (n_r != bm)) |
           (ex_fv[2] & asl_v) | (ex_fv[3] & v_bcd);
// N of the uop as one AND-OR of one-hot registered selects (nsel, from the uop entering EX) and sizes.
reg  [5:0]  nsel;                       // {BCD, old B (TAS), adder, p68_exi, shifter, logic op}
wire n_xe /* synthesis syn_keep = 1 */;                                // the early terms kept apart
assign n_xe = (nsel[4] & nszh(b)) | (nsel[0] & nszh(r_l));
wire n_x = n_xe | (nsel[5] & n_bcd) | (nsel[3] & nszh(sum[31:0])) | (nsel[2] & nszh(ri)) | (nsel[1] & nszh(shres));
function [5:0] nselv; input [`UW-1:0] x;
  reg [4:0] o; reg bc, ol, ad;       // op; BCD; old-B N and Z (TAS); adder op
  begin
    o = x[`F_OP]; bc = (x[`F_FCV] == `CV_BCD); ol = !bc && (x[`F_FZM] == `ZM_OLD);
    ad = o[4] ? (o[3:2] == 2'b00) : (!o[3] && (o[2:0] == 3'd0 || o[2:0] == 3'd1 || o[2:0] == 3'd4));
    nselv = {bc, ol, !bc && !ol && ad, !bc && !ol && !ad && o[4], !bc && !ol && !ad && !o[4] && o[3], !bc && !ol && !ad && !o[4] && !o[3]};
  end
endfunction
wire x_ok = (fcv != `CV_SHF) || !ex_sc0 || sh_rox;                  // a shift by 0 leaves X unchanged

// ---- The condition ------------------------------------------------------------------------------------------------
// The condition is a 16-entry truth table of the uop's CC, registered when the uop enters EX (tt_a; lane B tt_b); cond
// is then a 16:1 select by the flags N Z V C, which is two LUT levels.
function condf; input [3:0] c; input [3:0] f;         // f = {N, Z, V, C}
  begin
    case (c)
      4'h0: condf = 1'b1;                  4'h1: condf = 1'b0;               // T (true), F (false)
      4'h2: condf = !f[0] && !f[2];        4'h3: condf = f[0] || f[2];       // HI, LS (unsigned higher, lower or same)
      4'h4: condf = !f[0];                 4'h5: condf = f[0];               // CC, CS (carry clear, set)
      4'h6: condf = !f[2];                 4'h7: condf = f[2];               // NE, EQ
      4'h8: condf = !f[1];                 4'h9: condf = f[1];               // VC, VS (overflow clear, set)
      4'hA: condf = !f[3];                 4'hB: condf = f[3];               // PL, MI (plus, minus)
      4'hC: condf = f[3] == f[1];          4'hD: condf = f[3] != f[1];       // GE, LT (signed)
      4'hE: condf = !f[2] && (f[3] == f[1]);                                 // GT
      default: condf = f[2] || (f[3] != f[1]);                               // LE
    endcase
  end
endfunction
function [15:0] ctab; input [3:0] c;   // the truth table of condition c over all 16 flag combinations
  integer q;
  begin for (q = 0; q < 16; q = q + 1) ctab[q] = condf(c, q[3:0]); end
endfunction
reg  [15:0] tt_a, tt_b;              // lane A's and lane B's tables
always @* cond = tt_a[ccr_e[3:0]];
wire        cond_r = tt_a[ccr[3:0]];      // the condition on the committed CCR (TRAPcc: AG keeps it two behind any uop)
// ---- Prediction ----
// F_PRED: predicted taken (fetch or AG went to the target): redirect to the next instruction when not taken, to the
// target when taken but the BTC target was wrong (ex_pmiss). F_PRET: a return whose fetch went to the return stack's
// address: the jump redirects only when that address was wrong.
wire        pr     = ex_u[`F_PRED];
// The zero test of B.W (DBcc: the counter was 0 before the decrement, so it ends at -1 and the loop exits) on each
// registered source, the forwarding select last.
wire        bz_e = (ex_b[15:0] == 16'h0), bz_w = (wfb[15:0] == 16'h0);
wire        dbc_t  = dbcc && !cond && !(ex_bfwd ? bz_w : bz_e);         // DBcc branches back
wire        br_tk  = (ex_u[`F_UNIT] == `UN_BRA) ? cond : dbcc ? dbc_t : 1'b1;   // the branch is taken
wire        br_npc = pr && (((ex_u[`F_UNIT] == `UN_BRA) && !cond) || (dbcc && !dbc_t));   // predicted taken, but not taken
// A skip pair's Bcc never redirects. br_red for each value of the condition and of B.W == 0 is registered with the
// uop (brR, from the same terms); the late condition and zero test select one of four registers.
wire        bz_x   = ex_bfwd ? bz_w : bz_e;
wire        br_red = brR[{cond, bz_x}];
wire        pdrop  = ex_u[`U_PSK] && cond;                     // a taken skip-pair branch drops lane B
wire        sy_red_p = sy_red && (!(ex_u[`F_PRET] && !ex_pmiss) || irq_take);   // a correctly predicted return does not redirect, unless an interrupt is taken in front
assign ex_brc  = (ex_commit && ex_u[`F_BR]) || (exb_v && ex_commit && exb_u[`F_BR]);   // lane A's branch, or lane B's (below)
assign ex_brt  = ex_bb ? tk_b : br_tk;
assign ex_btgt = ex_bb ? tgt_b : ex_tgt;
// ---- The redirect ---------------------------------------------------------------------------------------------------
// The redirect comes from registered decodes (rdK: no correctly predicted return, rdD: raise with K, rdR: SER, JMP or
// MCLR with K, rdF: FIRST and not an exception sequence uop) and the late terms in one AND-OR. It equals
// sy_red_p || (ex_v && (ex_rep || br_red)) || bmis (an interrupt take is never a commit, and a replay redirects
// anyway). redirect_rc is the same from the copy ex_vc, for the register file's restore copies and rs_d.
// xcv: the raise condition of the EX uop (the same terms as xc in p68_sys.v)
wire        xcv  = (ex_sd[1] & sy_cq) | (ex_sd[2] & cond_r) | (ex_sd[3] & fbad) | (ex_sd[4] & md_dz) | (ex_sd[5] & xc_fpu) | ex_sd[6];
wire        rd_i = irq_pnd && !stopped;                         // an interrupt will be taken in front of a FIRST uop
assign redirect_ex = (ex_v && rdF && rd_i) || (stopped && irq_pnd && rdK) || bmis || (ex_v && (ex_rep || br_red || (rdD && xcv) || rdR));
// Lane B's misprediction from its own copy of exb_v (exb_vc) enters redirect_rc's last LUT. Before, the path from the
// WB value through lane B's condition, its taken bit and the 16-load bmis net into redirect_rc and rs_d was a real
// failing path of the full system.
wire        rrc_a /* synthesis syn_keep = 1 */;
assign      rrc_a = (ex_vc && rdF && rd_i) || (stopped && irq_pnd && rdK) || (ex_vc && (ex_rep || br_red || (rdD && xcv) || rdR));
wire        ex_repb = smc_h && exb_v && exb_u[`U_BOLD];               // the smc replay restarts at the older lane B
// The target: the system unit's, the replayed instruction (it restarts at its own step), lane B's branch, the next
// instruction (a predicted branch not taken), or the branch target.
assign ex_target   = sy_red_p ? sy_tgt : ex_rep ? (ex_repb ? exb_u[`F_PC] : ex_u[`F_PC]) : bmis ? (tk_b ? tgt_b : exb_npc) :
                     br_npc ? ex_npc : ex_tgt;   // replay restarts at the instruction (and its step)

// ---- Flags at commit ------------------------------------------------------------------------------------------------
// At commit EX registers the final C, N, Z, V of the uop (fA_val, after the carry chain) with the bits it writes
// (fA_m); the units' CCRs replace all five. Z of a shift or BCD comes from the result in WB (f_wz: zsz(wb_val));
// ZM_KEEP (ADDX, SUBX, NEGX, BCD) ANDs the old Z with it (fA_keep), so Z stays clear once any part of a multi-word
// result was nonzero. WB completes the CCR (ccr_f becomes ccr at the end of its clock); a uop in EX sees the CCR with
// the uop in WB (ccr_e) in two levels, Z of a shift or BCD in WB excepted: AG keeps a flag reader one clock behind such
// a uop (and an X reader one clock behind an X writer, which reads X from ccr).
reg         fA_v, fA_keep, fA_zq, fA_zk, f_wz;   // committed; Z mode KEEP; Z of the adder/logic op; p68_exi's keep term; Z from WB
reg  [4:0]  fA_m, fA_val;            // the flags written (mask) and their values
reg  [1:0]  f_sz;                    // the size (for Z from WB)
wire        z_e  = is_ad ? z_ad : zszh(r_l);                          // Z of the adder and logic ops (final in EX)
wire        z_e2 = (ex_zm[2] & zmo) | (ex_zm[1] & z_bit) | (ex_zm[0] & z_e);   // one-hot FZM
wire        f_wz_n = fen[2] && !ex_u[`U_FZM + 1] && !is_u && !md_u && !is_ad && (op[3] || op[4]);   // Z from the result in WB
always @(posedge clk) begin
  fA_v <= ex_commit; f_sz <= sz;
  // the mask: nothing without a commit; all five for the system and bit field units; for mul/div by FEN; a CAS step whose
  // condition is false writes no flags (a CAS step stores and sets flags only by its condition); else FEN, with X only when x_ok
  fA_m <= !ex_commit ? 5'd0 : is_u ? 5'b11111 : md_u ? (fen[3] ? 5'b11111 : 5'd0) : (cas && !cond) ? 5'd0 : {fen[4] && x_ok, fen[3:0]};
  fA_val <= ({5{is_u && ex_u[`U_UNIT]}} & sy_ccr) | ({5{is_u && !ex_u[`U_UNIT]}} & bf_ccr) | ({5{md_u}} & md_ccr) |
            ({5{!is_u && !md_u}} & {c_f, n_x, z_e2, v_f, c_f});   // X takes the new C
  fA_keep <= !is_u && !md_u && (ex_u[`F_FZM] == `ZM_KEEP); fA_zq <= z_e; fA_zk <= z_keep; f_wz <= f_wz_n && !is_u && !md_u;
end
// ---- Lane B -----------------------------------------------------------------------------------------------------------
// Operands prepared in front of EX as for lane A, forwarding from both WB lanes, the ALU in p68_exb.v, the flag sources
// registered at commit and completed in WB after lane A's (lane B is the younger instruction, unless swapped).
reg  [31:0] exb_a, exb_b, exb_ae, exb_ax, exb_bq;   // lane B's operands; A extended; prepared adder inputs
reg         exb_afw, exb_afb, exb_bfw, exb_bfb, exb_sub, exb_ngb;   // forwarded from lane A / lane B WB; A / B inverted
wire        exb_afwd = exb_afw | exb_afb, exb_bfwd = exb_bfw | exb_bfb;
wire [31:0] wbb = exb_bfb ? wb2_val : wb_val;      // lane B's forwarded B value (DBcc counter test)
wire [31:0] fa_b = fwv(wb_val, wb2_val, exb_fa);                  // one-hot, as lane A
wire [31:0] fb_b = ({32{exb_fb[0]}} & wb_val) | ({32{exb_fb[1]}} & wb2_val);
wire [31:0] b_a  = fa_b | ({32{exb_fa[10]}} & exb_ae);            // lane B's operand A
wire [31:0] b_b  = fb_b | ({32{exb_fb[2]}} & exb_b);              // lane B's operand B
wire [31:0] b_ax = (fa_b ^ {32{exb_subf}}) | ({32{exb_fa[10]}} & exb_ax);   // the adder's A input
wire [31:0] b_bq = (fb_b ^ {32{exb_ngf}}) | ({32{exb_fb[2]}} & exb_bq);     // the adder's B input
wire [31:0] rmb;                     // lane B's result, merged by size
wire        bc8, bc16, bc32, bbor, bbm, baxm, bzad, bzl;   // its carries, borrow type, input signs, Z of add / logic
p68_exb i_exb (.u(exb_u), .a(b_a), .b(b_b), .ax(b_ax), .bq(b_bq), .rm(rmb), .c8(bc8), .c16(bc16), .c32(bc32), .bor(bbor),
               .bm(bbm), .axm(baxm), .z_ad(bzad), .z_l(bzl));
// A BTC-predicted Bcc, BRA or DBcc in lane B (younger): its condition comes from ccr_e as lane A's does; a DBcc whose
// condition holds writes its counter back unchanged (forwarding stays valid). It redirects when not taken (to the next
// instruction) or when AG found the predicted target wrong (lane B's F_PFIX; the target is in its LA_DISP field).
wire        condb = tt_b[ccr_e[3:0]];                                 // lane B's condition (truth table)
wire        dbc_b  = (exb_u[`F_UNIT] == `UN_DBR);
// Lane B's write-back select ((dbc_b && condb) || pdrop) as one 16-entry table of the flags, registered with the pair
// (tt_s, a copy each for wb2_val and wb2_vag): one 16:1 select, with no condition and OR behind it.
reg  [15:0] tt_s0 /* synthesis syn_preserve = 1 */, tt_s1 /* synthesis syn_preserve = 1 */;
wire        ws_0 = tt_s0[ccr_e[3:0]], ws_1 = tt_s1[ccr_e[3:0]];      // write B back unchanged
wire        isbr_b = (exb_u[`F_UNIT] == `UN_BRA) || dbc_b || exb_u[`F_BRA];   // lane B is a branch
wire        bzb_e  = (exb_b[15:0] == 16'h0), bzb_w = (wbb[15:0] == 16'h0);   // lane B's DBcc counter is 0
assign      tk_b   = (exb_u[`F_UNIT] == `UN_BRA) ? condb : dbc_b ? (!condb && !(exb_bfwd ? bzb_w : bzb_e)) : 1'b1;
assign      bmis   = exb_v && bmR[{condb, exb_bfwd ? bzb_w : bzb_e}];   // = isbr_b && (!tk_b || PFIX), registered per case
wire        bmis_c = exb_vc && bmR[{condb, exb_bfwd ? bzb_w : bzb_e}];   // redirect_rc's copy
assign      redirect_rc = rrc_a || bmis_c;
assign      tgt_b  = exb_u[`F_LA_DISP];   // AG put lane B's computed target there
assign ex_bb   = exb_v && isbr_b;
// Lane B younger: it commits with lane A, and not when lane A (a fold-pair branch) redirects. Lane B older (swapped
// pair, U_BOLD): it commits unless the replay includes it; its flag writes are masked by lane A's (lane A's flags are
// completed first in WB).
wire        exb_o      = exb_u[`U_BOLD];
wire        exb_commit = exb_v && (exb_o ? !ex_repb : (ex_commit && !(ex_v && br_red)));
wire [4:0]  fa_m       = ex_commit ? {fen[4] && x_ok, fen[3:0]} : 5'd0;   // the flags lane A writes
reg  [4:0]  g_fenf;                  // lane B's FEN unmasked (for its own retire record)
reg         g_v, gf_c, gf_n, gf_z, gf_v; reg [4:0] g_fen;   // lane B committed; its C N Z V; its flag mask
wire        b_cy = ((exb_u[`F_SZ] == `SZ_B) ? bc8 : (exb_u[`F_SZ] == `SZ_W) ? bc16 : bc32) ^ bbor;   // C at the size
wire        b_nr = nsz(rmb, exb_u[`F_SZ]);                                                       // N at the size
always @(posedge clk) begin
  // lane B's final flags registered at commit
  g_v <= exb_commit && !pdrop; g_fen <= exb_o ? (exb_u[`F_FEN] & ~fa_m) : exb_u[`F_FEN]; g_fenf <= exb_u[`F_FEN]; g_o <= exb_o;
  gf_c <= (exb_u[`F_FCV] == `CV_ADD) && b_cy;
  gf_v <= (exb_u[`F_FCV] == `CV_ADD) && (bbm == baxm) && (b_nr != bbm);
  gf_n <= b_nr;
  gf_z <= ((exb_u[`F_OP] == `OP_ADD) || (exb_u[`F_OP] == `OP_SUB) || (exb_u[`F_OP] == `OP_CMP) || (exb_u[`F_OP] == `OP_NEG)) ? bzad : bzl;
  // lane B's register write for WB: enable, number, one-hot hits on A0-A7 / D0-D7, and the value (B unchanged when the
  // write-back select says so), in two copies (wb2_vag for AG)
  wb2_wen <= !rst && exb_commit && exb_u[`F_WEN]; wb2_wreg <= exb_u[`U_WREG +: 4]; wb2_h <= exb_u[`U_WREG + 3] ? (8'd1 << exb_u[`U_WREG +: 3]) : 8'd0; wb2_hd <= exb_u[`U_WREG + 3] ? 8'd0 : (8'd1 << exb_u[`U_WREG +: 3]); wb2_val <= ws_0 ? b_b : rmb; wb2_vag <= ws_1 ? b_b : rmb;
  ret2_v <= !rst && exb_commit && !pdrop; ret2_pc <= exb_u[`F_PC];
end
reg  [31:0] wb2_vag /* synthesis syn_preserve = 1 */;   // copy of wb2_val for AG's forwarding
// Copies for AG's interlock and forwarding. The attribute on a declaration list applies only to its last name, so each
// copy is declared on its own line with its own attribute (otherwise Gowin merges ex_vag back into ex_v).
reg         ex_vag /* synthesis syn_preserve = 1 */;               // copy of ex_v for AG
reg         ex_wuag /* synthesis syn_preserve = 1 */;              // copy of the EX uop's WEN for AG
reg         ex_cwag /* synthesis syn_preserve = 1 */;              // the EX uop is a conditional writer (DBcc, CAS), for AG
reg  [4:0]  ex_wrag /* synthesis syn_preserve = 1 */;              // copy of the EX uop's WREG for AG
reg         exb_vag /* synthesis syn_preserve = 1 */;              // lane B's valid for AG
reg         exb_wuag /* synthesis syn_preserve = 1 */;             // lane B's WEN for AG
reg  [3:0]  exb_wrag /* synthesis syn_preserve = 1 */;             // lane B's WREG for AG
wire [4:0]  gv = {gf_c, gf_n, gf_z, gf_v, gf_c};                   // lane B's flag values (X = C)
wire [4:0]  gm = g_v ? g_fen : 5'd0;                               // the bits lane B writes (masked by lane A when older)
wire [4:0]  gmf = g_v ? g_fenf : 5'd0;                             // ... unmasked
// Lane A's flags over the committed CCR: for EX (eA: Z of a shift or BCD in WB not final, never read, see above) and
// complete (cfa: with Z from the WB value).
wire        zr_w = f_wz ? zsz(wb_val, f_sz) : fA_zq;                // Z: from the WB result, or from EX
wire        zA_e = fA_keep ? (ccr[2] & (fA_zq | fA_zk)) : fA_val[2];
wire        zA_f = fA_keep ? (ccr[2] & (zr_w | fA_zk)) : f_wz ? zr_w : fA_val[2];
wire [4:0]  eA  = {fA_m[4] ? fA_val[4] : ccr[4], fA_m[3] ? fA_val[3] : ccr[3], fA_m[2] ? zA_e : ccr[2], fA_m[1] ? fA_val[1] : ccr[1], fA_m[0] ? fA_val[0] : ccr[0]};
wire [4:0]  cfa = {fA_m[4] ? fA_val[4] : ccr[4], fA_m[3] ? fA_val[3] : ccr[3], fA_m[2] ? zA_f : ccr[2], fA_m[1] ? fA_val[1] : ccr[1], fA_m[0] ? fA_val[0] : ccr[0]};
function [4:0] over; input [4:0] c; input [4:0] m; input [4:0] v;   // v's bits m over c
  integer q;
  begin for (q = 0; q < 5; q = q + 1) over[q] = m[q] ? v[q] : c[q]; end
endfunction
always @* begin ccr_f = over(cfa, gm, gv); ccr_e = over(eA, gm, gv); end   // lane B's flags after lane A's
assign ccr_o = ccr_f;
assign wb2_vg = wb2_vag; assign ex_vg = ex_vag; assign ex_wug = ex_wuag; assign ex_wrg = ex_wrag; assign ex_cwg = ex_cwag; assign exb_wug = exb_vag && exb_wuag; assign exb_wrg = exb_wrag;
// For the retire port, each instruction's own CCR: the older one's alone, the younger one's with both.
assign ccr_a = g_o ? ccr_f : cfa;
assign ccr_b = g_o ? over(ccr, gmf, gv) : ccr_f;
// ---- Bit field unit (its operands unforwarded: AG keeps bit field uops two behind their writers) ----
p68_bf i_bf (.clk(clk), .v(ex_v && !ex_rep), .sel(ex_u[`F_UNIT] == `UN_BFU), .op(op), .cc(cc),
  .imm(ex_u[`U_IMM +: 10]), .a(ex_a), .b(ex_b), .ccr(ccr_e),   /* unforwarded operands (AG gap) */ .fen(fen), .bf_r(bf_r), .bf_ccr(bf_ccr));
// ---- FPU (a lane A unit; its results and raise conditions come from its registers) ----
wire [31:0] fp_r; wire [3:0] fpcc; wire xc_fpu; wire [5:0] xv_fpu; wire bsun_en;   // result; FPSR condition codes; raise; vector; BSUN enabled
generate if (`P68_FPU != 0) begin : g_fpu
p68_fpu i_fpu (.clk(clk), .rst(rst), .ex_v(ex_v), .ex_rep(ex_rep), .ex_u(ex_u), .a(a), .oc_u(oc_u), .fp_r(fp_r), .fpcc(fpcc),
  .xc_fpu(xc_fpu), .xv_fpu(xv_fpu), .null_st(fpu_null), .en_any(fpu_en), .pv(fpu_pv), .bsun_en(bsun_en),
  .ret_fp0(ret_fp0), .ret_fp1(ret_fp1), .ret_fp2(ret_fp2), .ret_fp3(ret_fp3), .ret_fp4(ret_fp4), .ret_fp5(ret_fp5),
  .ret_fp6(ret_fp6), .ret_fp7(ret_fp7), .ret_fpcr(ret_fpcr), .ret_fpsr(ret_fpsr), .ret_fpiar(ret_fpiar));
end else begin : g_nofpu
// without the FPU: constants, and the NULL state so FSAVE writes the short frame
assign fp_r = 32'h0; assign fpcc = 4'h0; assign xc_fpu = 1'b0; assign xv_fpu = 6'd0; assign fpu_null = 1'b1; assign fpu_en = 1'b0;
assign fpu_pv = 6'd0; assign bsun_en = 1'b0;
assign ret_fp0 = 80'h0; assign ret_fp1 = 80'h0; assign ret_fp2 = 80'h0; assign ret_fp3 = 80'h0;
assign ret_fp4 = 80'h0; assign ret_fp5 = 80'h0; assign ret_fp6 = 80'h0; assign ret_fp7 = 80'h0;
assign ret_fpcr = 32'h0; assign ret_fpsr = 32'h0; assign ret_fpiar = 32'h0;
end endgenerate
// FP predicate on FPSR's condition codes (N Z I NAN), following WinUAE's 68882 table, in which OR and NE are true with Z
// and NAN both set. The predicate's bit 4 only says whether a NaN raises BSUN; bits 3-0 choose the test. An FP
// conditional's predicate is evaluated when it enters EX and turned into the integer condition T or F, so the ordinary
// Bcc, Scc, DBcc and TRAPcc logic does the rest.
function fpred; input [3:0] c; input [3:0] f;
  reg n, z, q;                       // negative; zero; unordered (a NaN was involved)
  begin
    n = f[3]; z = f[2]; q = f[0];
    case (c)
      4'h0: fpred = 1'b0;            4'h1: fpred = z;                  4'h2: fpred = !(q || z || n);   4'h3: fpred = z || !(q || n);
      4'h4: fpred = n && !(q || z);  4'h5: fpred = z || (n && !q);     4'h6: fpred = !(q || z);        4'h7: fpred = !q || z;
      4'h8: fpred = q;               4'h9: fpred = q || z;             4'hA: fpred = q || !(n || z);   4'hB: fpred = q || z || !n;
      4'hC: fpred = q || (n && !z);  4'hD: fpred = q || z || n;        4'hE: fpred = !z || q;          default: fpred = 1'b1;
    endcase
  end
endfunction
// ---- System unit ----
p68_sys i_sys (
  .clk(clk), .rst(rst), .ex_v(ex_v), .ex_rep(ex_rep), .ex_u(ex_u), .a(a), .b(b), .npc(ex_npc), .ccr(ccr_e),
  // ccr_e / ccr_f: the CCR with the uop in WB; a_r and b_r: the unforwarded operands for CHK and CHK2
  .ccr_n(ccr_f), .sy_sel(ex_sys), .sy_dec(ex_sd), .cond(cond), .cond_r(cond_r), .a_r(ex_as), .b_r(ex_b),   /* unforwarded */   /* flags with WB */ .ipl(ipl), .xc_div(md_dz), .irq_hold(irq_hold), .iwant(iwant), .fbad(fbad), .xc_fpu(xc_fpu), .xv_fpu(xv_fpu),
  .irq_pnd(irq_pnd), .sy_cq(sy_cq), .irq_take(irq_take), .sy_ccr(sy_ccr), .sy_r(sy_r), .sy_red(sy_red), .sy_tgt(sy_tgt), .no_ret(no_ret),
  .rs_one(rs_one), .sup(sup), .msp(msp), .stopped(stopped), .rst_done(rst_done),
  .xs_go(xs_go), .xs_par(xs_par), .xs_rs(xs_rs), .xs_rs_step(xs_rs_step),
  .sw_en(sw_en), .sw_old(sw_old), .sw_new(sw_new), .iack(iack), .iack_lvl(iack_lvl), .reset_o(reset_o),
  .sr(sr), .vbr(vbr), .caar(caar), .sfc(sfc), .dfc(dfc), .cacr(cacr));

// ---- Forwarding conditions of the uops entering EX (lane A and lane B, both from OC2) ------------------------------
// "The uop now in EX writes the register I read": next clock that uop is in WB, so the operand comes from WB's value.
// Lane B never writes registers 16 and up, so its matches also require the reader's register bit 4 to be clear.
wire        n_afw = (oc_u[`F_ASEL] == `AS_REG) && ex_v && ex_wen && (ex_wreg == oc_u[`F_AREG]);
wire        n_bfw = (oc_u[`F_BSEL] == `BS_REG) && ex_v && ex_wen && (ex_wreg == oc_u[`F_BREG]);
wire        n_afb = (oc_u[`F_ASEL] == `AS_REG) && exb_v && exb_u[`F_WEN] && (exb_u[`U_WREG +: 4] == oc_u[`F_AREG]) && !oc_u[`U_AREG + 4];
wire        n_bfb = (oc_u[`F_BSEL] == `BS_REG) && exb_v && exb_u[`F_WEN] && (exb_u[`U_WREG +: 4] == oc_u[`F_BREG]) && !oc_u[`U_BREG + 4];
wire        m_afw = (q_ub[`F_ASEL] == `AS_REG) && ex_v && ex_wen && (ex_wreg == q_ub[`F_AREG]);
wire        m_bfw = ex_v && ex_wen && (ex_wreg == q_ub[`F_BREG]);            // lane B's B is always a register
wire        m_afb = (q_ub[`F_ASEL] == `AS_REG) && exb_v && exb_u[`F_WEN] && (exb_u[`U_WREG +: 4] == q_ub[`F_AREG]) && !q_ub[`U_AREG + 4];
wire        m_bfb = exb_v && exb_u[`F_WEN] && (exb_u[`U_WREG +: 4] == q_ub[`F_BREG]) && !q_ub[`U_BREG + 4];
wire        ex_sbn = (oc_u[`F_OP] == `OP_SUB) || (oc_u[`F_OP] == `OP_CMP) || (oc_u[`F_OP] == `OP_SUBX);   // A inverted
wire        ex_nbn = (oc_u[`F_OP] == `OP_NEG) || (oc_u[`F_OP] == `OP_NEGX);                            // B inverted
wire        exb_sbn = (q_ub[`F_OP] == `OP_SUB) || (q_ub[`F_OP] == `OP_CMP);
wire        exb_nbn = (q_ub[`F_OP] == `OP_NEG);
// ---- Stage registers: OC2 -> EX, with every decode of the entering uop ----------------------------------------------
always @(posedge clk) begin
  exb_vag <= !rst && q_ubv && !redirect_ex && !rs_d; exb_wuag <= q_ub[`F_WEN]; exb_wrag <= q_ub[`U_WREG +: 4];
  ex_vag <= !rst && oc_v && !redirect_ex && !rs_d; ex_wuag <= oc_u[`F_WEN]; ex_wrag <= oc_u[`F_WREG];   // copies for AG
  ex_cwag <= oc_u[`F_WEN] && (oc_u[`F_UNIT] == `UN_DBR || oc_u[`F_UNIT] == `UN_CAS);
  if (rst) begin ex_v <= 1'b0; ex_vc <= 1'b0; ccr <= 5'h0; end
  else begin
    // a redirect now or one clock ago empties EX (OC empties one clock late, so rs_d is needed too)
    ex_v <= oc_v && !redirect_ex && !rs_d; ex_vc <= oc_v && !redirect_ex && !rs_d;
    ccr  <= ccr_f;                                                     // the CCR completed in WB
  end
  ex_npc <= oc_u[`F_PC] + {27'h0, oc_u[`F_LEN], 1'b0};                // PC + 2 * length in words
  ex_u <= oc_u; ex_sel <= sv_n; ex_sys <= sysv(oc_u); nsel <= nselv(oc_u);
  begin : fdec                       // the flag decode, registered
    reg [4:0] o; reg ad;
    o = oc_u[`F_OP]; ad = o[4] ? (o[3:2] == 2'b00) : (!o[3] && (o[2:0] == 3'd0 || o[2:0] == 3'd1 || o[2:0] == 3'd4));
    ex_ad <= ad;
    ex_bor <= (o == `OP_SUB) || (o == `OP_CMP) || (o == `OP_SUBX) || (o == `OP_NEG) || (o == `OP_NEGX);
    ex_rl <= {o[2:0] == 3'd6, o[2:0] == 3'd2, o[2:0] == 3'd3, o[2:0] == 3'd7, o[2:0] != 3'd6 && o[2:0] != 3'd2 && o[2:0] != 3'd3 && o[2:0] != 3'd7};
    ex_nr <= {o[4], !o[4] && o[3], !o[4] && !o[3]};
    ex_zm <= {oc_u[`F_FZM] == `ZM_OLD, oc_u[`F_FZM] != `ZM_OLD && oc_u[`F_FZM] == `ZM_BIT, oc_u[`F_FZM] != `ZM_OLD && oc_u[`F_FZM] != `ZM_BIT};
    ex_fv <= {oc_u[`F_FCV] == `CV_BCD, oc_u[`F_FCV] == `CV_SHF, oc_u[`F_FCV] == `CV_ADD && !ad, oc_u[`F_FCV] == `CV_ADD && ad};
  end
  begin : sdec                       // the system and redirect decode, registered
    reg bs;                          // BSUN: an FP conditional with a signaling predicate on a NaN, with BSUN enabled
    bs = (`P68_FPU != 0) && oc_u[`U_FPC] && oc_u[`U_FPS] && fpcc[0] && bsun_en;              // as the BSUN override below
    ex_sd <= sdecv(bs ? `UN_NONE : oc_u[`F_UNIT], oc_u[`F_OP], oc_u[`F_SZ], bs ? 8'd48 : oc_u[`F_EXC]);
    begin : rdd
      reg [9:0] d; reg k;
      d = sdecv(bs ? `UN_NONE : oc_u[`F_UNIT], oc_u[`F_OP], oc_u[`F_SZ], bs ? 8'd48 : oc_u[`F_EXC]);
      k = !(oc_u[`F_PRET] && !oc_pmiss);   // not a correctly predicted return
      rdK <= k; rdD <= k && d[0]; rdR <= k && (d[7] || d[8] || d[9]);
      rdF <= oc_u[`F_FIRST] && ((bs ? 8'd48 : oc_u[`F_EXC]) >> 6) == 8'd0;   // FIRST and not an exception sequence uop
    end
  end
  // one-hot size; s_oth: one of the early result sources is selected
  szh <= {oc_u[`F_SZ] != `SZ_B && oc_u[`F_SZ] != `SZ_W, oc_u[`F_SZ] == `SZ_W, oc_u[`F_SZ] == `SZ_B}; s_oth <= |{sv_n[13:8], sv_n[6], sv_n[4:0]};
  // the condition tables; an FP conditional's predicate becomes T (0) or F (1) here
  tt_a <= ctab(((`P68_FPU != 0) && oc_u[`U_FPC]) ? (fpred(oc_u[`F_CC], fpcc) ? 4'h0 : 4'h1) : oc_u[`F_CC]);
  tt_b <= ctab(q_ub[`F_CC]);
  begin : tts                        // lane B's write-back select table
    reg [15:0] t;
    // DBcc in lane B: its condition (true: the counter is written back unchanged); skip pair: lane A's condition
    // (true: the branch is taken and lane B is dropped)
    t = ({16{q_ub[`F_UNIT] == `UN_DBR}} & ctab(q_ub[`F_CC])) |
        ({16{oc_u[`U_PSK]}} & ctab(((`P68_FPU != 0) && oc_u[`U_FPC]) ? (fpred(oc_u[`F_CC], fpcc) ? 4'h0 : 4'h1) : oc_u[`F_CC]));
    tt_s0 <= t; tt_s1 <= t;
  end
  if ((`P68_FPU != 0) && oc_u[`U_FPC]) ex_u[`F_CC] <= fpred(oc_u[`F_CC], fpcc) ? 4'h0 : 4'h1;     // FP condition as T / F
  if ((`P68_FPU != 0) && oc_u[`U_FPC] && oc_u[`U_FPS] && fpcc[0] && bsun_en) begin   // BSUN trap: the conditional does nothing, raises 48
    ex_u[`F_UNIT] <= `UN_NONE; ex_u[`F_WEN] <= 1'b0; ex_u[`F_SA_EN] <= 1'b0; ex_u[`F_EXC] <= 8'd48;
  end
  ex_a <= op_a; ex_b <= op_b; ex_tgt <= oc_addr;
  begin : shprep                     // the shifter controls, one stage early
    reg [4:0] o; reg [5:0] n; reg [1:0] z;
    o = oc_u[`F_OP]; n = op_a[5:0]; z = oc_u[`F_SZ];
    ex_rc <= (o[2] || (o[1] && o[0])) ? n[4:0] : (~n[4:0] + 5'd1);    // left shifts and ROX take the count; right shifts rotate by 32 - count
    ex_tlo <= mlo(n); ex_thi <= mhi(n); ex_tkl <= mlo({1'b0, n[4:0]}); ex_ekx <= meq(n[4:0]);
    ex_shbig <= (z == `SZ_B) ? (n > 6'd8) : (z == `SZ_W) ? (n > 6'd16) : (n > 6'd32);
    ex_sc0 <= (n == 6'd0);
  end
  begin : prep                       // the adder inputs of lane A
    reg [4:0] o; reg sb, nb;
    o  = oc_u[`F_OP];
    sb = (o == `OP_SUB) || (o == `OP_CMP) || (o == `OP_SUBX);
    nb = (o == `OP_NEG) || (o == `OP_NEGX);
    ex_ax <= aextf(op_a, oc_u[`F_AEXT]) ^ {32{sb}};
    ex_bq <= op_b ^ {32{nb}};
  end
  ex_afw <= n_afw; ex_bfw <= n_bfw;                                                         // EX now = WB next clock
  ex_fa <= fsv(n_afw, n_afb, oc_u[`F_AEXT]); ex_fb <= {!(n_bfw || n_bfb), n_bfb, n_bfw && !n_bfb};
  ex_subf <= ex_sbn && (n_afw || n_afb); ex_negf <= ex_nbn && (n_bfw || n_bfb); ex_ae <= aextf(op_a, oc_u[`F_AEXT]);
  ex_as <= aextf(op_a, oc_u[`F_AEXT]); {ex_bs, ex_rin} <= shinf(op_b, oc_u[`F_SZ], oc_u[`F_OP]);
  ex_pmiss <= oc_pmiss;
  begin : brt                        // br_red and bmis for each case of {condition, B.W == 0}
    reg bs, ib, id;
    bs = (`P68_FPU != 0) && oc_u[`U_FPC] && oc_u[`U_FPS] && fpcc[0] && bsun_en;              // the BSUN override (unit NONE)
    ib = !bs && (oc_u[`F_UNIT] == `UN_BRA); id = !bs && (oc_u[`F_UNIT] == `UN_DBR);
    brR <= {brf(ib, id, oc_u[`F_PRED], oc_u[`U_PSK], oc_u[`F_BRA] && oc_u[`F_PBTC], oc_pmiss, 1'b1, 1'b1),
            brf(ib, id, oc_u[`F_PRED], oc_u[`U_PSK], oc_u[`F_BRA] && oc_u[`F_PBTC], oc_pmiss, 1'b1, 1'b0),
            brf(ib, id, oc_u[`F_PRED], oc_u[`U_PSK], oc_u[`F_BRA] && oc_u[`F_PBTC], oc_pmiss, 1'b0, 1'b1),
            brf(ib, id, oc_u[`F_PRED], oc_u[`U_PSK], oc_u[`F_BRA] && oc_u[`F_PBTC], oc_pmiss, 1'b0, 1'b0)};
    bmR <= {bmf(q_ub[`F_UNIT] == `UN_BRA, q_ub[`F_UNIT] == `UN_DBR, q_ub[`F_BRA], q_ub[`F_PFIX], 1'b1, 1'b1),
            bmf(q_ub[`F_UNIT] == `UN_BRA, q_ub[`F_UNIT] == `UN_DBR, q_ub[`F_BRA], q_ub[`F_PFIX], 1'b1, 1'b0),
            bmf(q_ub[`F_UNIT] == `UN_BRA, q_ub[`F_UNIT] == `UN_DBR, q_ub[`F_BRA], q_ub[`F_PFIX], 1'b0, 1'b1),
            bmf(q_ub[`F_UNIT] == `UN_BRA, q_ub[`F_UNIT] == `UN_DBR, q_ub[`F_BRA], q_ub[`F_PFIX], 1'b0, 1'b0)};
  end
  // lane B stage registers and the forwarding selects of both lanes
  ex_afb <= n_afb; ex_bfb <= n_bfb;
  exb_v <= !rst && q_ubv && !redirect_ex && !rs_d; exb_vc <= !rst && q_ubv && !redirect_ex && !rs_d; exb_u <= q_ub; exb_a <= opb_a; exb_b <= opb_b;
  exb_npc <= q_ub[`F_PC] + {27'h0, q_ub[`F_LEN], 1'b0};                                   // lane B's next instruction
  begin : prepb                      // the adder inputs of lane B
    reg [4:0] o; reg sb, nb;
    o = q_ub[`F_OP]; sb = (o == `OP_SUB) || (o == `OP_CMP); nb = (o == `OP_NEG);
    exb_sub <= sb; exb_ngb <= nb;
    exb_ae <= aextf(opb_a, q_ub[`F_AEXT]); exb_ax <= aextf(opb_a, q_ub[`F_AEXT]) ^ {32{sb}}; exb_bq <= opb_b ^ {32{nb}};
  end
  exb_afw <= m_afw; exb_bfw <= m_bfw; exb_afb <= m_afb; exb_bfb <= m_bfb;
  exb_fa <= fsv(m_afw, m_afb, q_ub[`F_AEXT]); exb_fb <= {!(m_bfw || m_bfb), m_bfb, m_bfw && !m_bfb};
  exb_subf <= exb_sbn && (m_afw || m_afb); exb_ngf <= exb_nbn && (m_bfw || m_bfb);
  // the memory side of the uop: store address and hits, replay, miss, fill address
  ex_saddr <= oc_saddr; ex_hit <= st_hit; ex_rep_q <= oc_rep; ex_miss <= oc_miss; ex_laddr <= oc_laddr; ex_laddr1 <= oc_laddr[31:2] + 30'd1; ex_hit2 <= st_hit2;
  ex_w1_en <= oc_w1_en; ex_w1_r <= oc_w1_r; ex_w1_v <= oc_w1_v;
  ex_h1 <= {8{oc_w1_en && oc_w1_r[4:3] == 2'b01}} & (8'd1 << oc_w1_r[2:0]);   // as OC's oc_h1 / oc_h2
  ex_h2 <= {12{oc_w2_en}} & (oc_w2_r[4] ? ((oc_w2_r[3:2] == 2'b00) ? (12'd256 << oc_w2_r[1:0]) : 12'd0) : oc_w2_r[3] ? (12'd1 << oc_w2_r[2:0]) : 12'd0);
  ex_w2_en <= oc_w2_en; ex_w2_r <= oc_w2_r; ex_w2_v <= oc_w2_v;
end

endmodule
`default_nettype wire
