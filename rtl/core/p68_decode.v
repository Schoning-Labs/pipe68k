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
// ==== p68_decode.v: DS, the decode stage =============================================================================
//
// What it does. DS turns 68k instructions into uops (the format is in p68_uop.vh). Fetch (p68_fetch.v) writes whole
// instructions into DS's instruction buffer, up to two per clock; DS decodes the two oldest entries every clock and
// hands the uops on to the DR queue at the front of address generation (p68_ag.v). One instruction can become one uop
// (most of them) or a sequence of uops (MOVEM, calls and returns, CAS, bit fields, the FPU, exceptions). DS also
// decides which instructions may be issued as a dual-issue pair, predicts returns with a small return stack, and in the
// CPU speed mode makes every instruction wait for the time a real 68EC020 would take. Everything is on the CPU clock.
//
// The parts, in the order the file has them:
//   - The instruction buffer: 16 entries in LUT RAM, with the two oldest (E0 and E1) as registers so they can be decoded
//     at once. DS consumes 0, 1 or 2 entries per clock.
//   - The group decoders (dec_*.v). Each one handles one family of instructions and gives a uop and a hit bit. Their
//     uops are gated by bits that p68_pdec.v predecoded when the entry was written, and ORed together. No hit at all is
//     an illegal instruction (vector 4). Why so many decoders and an OR: the instruction set was built in parallel work
//     packages, and one file per group kept them out of each other's way. The price was depth (twenty decoders plus the
//     OR were 16 levels with two module crossings), and most of the structure below exists to win that depth back.
//   - The data codes. The decoders do not build the 32-bit immediate and displacement values; they give 14-bit codes
//     ("take extension word k, sign-extended", plus a small constant), and DS turns the OR of the codes into values once
//     (dval in p68_dec.vh). An earlier measurement showed that a ROM for the control fields would save only about 700
//     LUTs, far less than its block RAMs cost, so the 32-bit data fields were the part worth shrinking.
//   - The sequencers. A sequence instruction issues its decoder's uop as step 0, then the sequencer's steps follow:
//     p68_seq.v for most long instructions, p68_sseq.v for system ones (RTE, CHK2/CMP2, MOVES), p68_xseq.v for exception
//     and interrupt stack frames (started by EX, it has priority and consumes no buffer entry), and p68_fx.v, the
//     pre-sequence for the 68020's full-format addressing modes (memory indirect and suppressed base/index), which
//     computes such an EA into T1/T2 first, after which the decoders see the opcode with that EA changed to (An).
//   - The pair decision: E1 may be issued together with E0 when the predecode says it is a lane-B instruction (or a
//     memory instruction behind a lane-B head, or a predicted branch). A second set of small decoders decodes E1 fully.
//   - The return stack (4 entries): JSR/BSR push the return address, RTS is predicted while it is still in the buffer.
//   - The override fields for DR (p68_dr.vh): PC, length, sequence step, prediction bits.
//   - The buffer pointers and the E0/E1 registers, at the end.
//
// The output is not a finished uop. DS ends at the decoder OR and the sequencer OR, and DR merges the parts in its own
// clock; that cut cost no CPI because DR was needed for dual issue anyway.
//
// Replay. When EX replays a uop of step s, its instruction restarts at step s; the steps below s run again without
// issuing (rs_skip). The restart step is latched at the redirect and applied with the delayed flush one clock later.
// DS also holds after the last uop of an exception sequence (until its jump redirects), while the CPU is stopped
// (STOP), and while the RESET instruction runs.
//
// History of the decode depth (each step checked exhaustively against the old decoders over all 65,536 opcodes with
// several sets of extension words): DS ends at the ORs and DR merges (one more stage, no CPI cost); every decoder gives
// its uop ungated and DS ANDs in the gates; the data values became codes; the value selects come straight from the
// opcode (and then predecoded with the entry); and the decoders' gates and legality are predecoded into the entry (20
// bits), so DS gates the uops with registered bits. A select between the decoder uop and the sequencer uop was measured
// to cost 20 MHz early on, which is why a sequence starts with an empty step 0 and the uops are ORed instead.
// Not kept: predecoding the decoders' code constants too (reason not recorded); a rework with about 2,000 more LUTs
// that improved the estimate (worst decode path 18.74 to 16.96 ns) but made the full system unroutable (306 unrouted
// nets); one more register stage between DS and AG, which cost CPI 1.035 and was dropped.
`include "p68_uop.vh"
`include "p68_dr.vh"
`include "p68_fpu.vh"
`include "p68_ib.vh"
`default_nettype none
module p68_decode (
  input  wire        clk,
  input  wire        rst,
  input  wire        slow,               // CPU speed mode (a register outside): issue throttle, no pairs
  // ---- instruction buffer (68060 style): fetch's extractor writes whole instructions as fixed-format entries; DS
  // decodes the two oldest, E0 and E1 (registers), and consumes 0, 1 or 2 per clock ----
  input  wire        ex1, ex2,           // fetch writes one / two entries
  input  wire [`IBW-1:0] xe0, xe1,       // the entries fetch writes
  output reg         ib_sp1, ib_sp2,     // room for one / two entries (registers)
  output wire        pop,                // the issue consumes the entry of fetch's prediction slot 0
  output wire        popa, popb,         // copies of pop for fetch's slot 0 and slot 1 fields
  output wire        hiss,               // the issue consumes a hold entry (fetch moves by its length)
  input  wire        hpf,                // fetch: the hold entry covers a BTC cut (register)
  input  wire [4:0]  fcnt,               // fetch queue words from its head (p68_fx)
  input  wire [3:0]  stall_ag,      // four register copies of AG's stall_dr (one per advance copy)
  input  wire        flush,              // restart: empty the buffer, PC = flush_tgt
  input  wire        flush_fx,           // the same flush for p68_fx (own copy)
  input  wire [31:0] flush_tgt,
  input  wire        redirect_ex,        // EX redirects now (replay or branch)
  input  wire        rs_d,               // redirect_ex one clock ago (flush now carries it)
  input  wire        xflush,             // the undelayed flush, for the exception sequencer
  input  wire        ex_rep,             // the redirect is a replay
  input  wire [4:0]  ex_step,            // ... of the uop at this step
  output wire        ds_go,              // a uop leaves DS
  output wire        ds_ill,             // no decoder hit (vector 4; AG puts it into EXC)
  output wire [`UW-1:0] o_dec,        // decoder uop (OR of the decoders), merged by DR (p68_ag)
  output wire [`UW-1:0] o_seq,        // sequencer uop (sequencers and p68_fx)
  output wire        o_act,              // a sequence or the full-format pre-sequence runs (o_seq counts)
  output wire [`DVW-1:0] o_ov,          // override fields (p68_dr.vh)
  output wire        issue,              // the head instruction is consumed
  // ---- supervisor mode and exceptions ----
  input  wire        sup, msp,           // SR S, M (committed)
  input  wire        stopped,            // STOP: waiting for an interrupt
  input  wire        rst_done,           // the RESET instruction's time is over
  input  wire        rs_one,             // RTE throwaway restart: skip step 0
  input  wire        ex_xs,              // the uop in EX belongs to an exception sequence
  input  wire        xs_go,              // start the exception sequence (registered in p68_core.v)
  input  wire [154:0] xs_par,            // its parameters
  input  wire        xs_rs,              // restart it after a replay
  input  wire [4:0]  xs_rs_step,         // ... at this step
  // ---- full-format EAs ----
  input  wire [15:0] wx,                 // queue word at head + xpos
  output wire [3:0]  xpos,               // p68_fx's read position in fetch's queue
  output wire        il_ov,              // issue length override (p68_fx)
  // ---- dual issue: the second instruction is entry E1 ----
  output wire [`UW-1:0] o_u2,          // the second instruction's uop (complete), valid with o_p2
  output wire        o_p2,               // a pair is issued
  output wire        o_sw,               // swap candidate (the second instruction would run in lane A)
  output wire [3:0]  il,                 // the override length for fetch
  input  wire        ex_fxm,             // the uop in EX is a main-part uop of a p68_fx instruction
  // ---- branch prediction ----
  input  wire [31:0] pt,                 // the BTC's predicted target
  input  wire        ag_pt,              // the predicted branch leaves AG: its target is ag_tgt
  input  wire [31:0] ag_tgt,
  output wire        bt_kill,            // invalidate the BTC entry of the wrong prediction
  output wire        dfl,                // return predicted: flush the queue, fetch from dtgt
  output wire [31:0] dtgt,
  input  wire        fpu_null,           // the FPU's committed NULL state
  input  wire        fpu_en,             // an FPCR exception enable is set (committed; FPCR writes redirect)
  input  wire [5:0]  fpu_pv              // pending FP trap vector (committed; every FPU instruction redirects at its end while fpu_en)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"                    // data codes (dval, dwk, dsx) and the other shared decode functions

// ---- instruction buffer. 16 entries in two LUT-RAM banks (even / odd index), all entries written there; E0 and E1 are
// register copies of entries rp and rp + 1. oc counts the entries from rp (E0 valid: at least 1, E1: at least 2).
// An issue consumes k = 0, 1 or 2 entries: E0 / E1 then take E1, the banks' entries rp + 2 / rp + 3 (read at
// registered addresses), or the entries fetch writes in this clock (xe0 at wp, xe1 at wp + 1). The selects by the count
// are registers (c0-c3); only the last 3:1 select uses the issue. Two banks, because fetch can write two entries per
// clock and a LUT RAM has one write port: consecutive entries always fall into different banks. ----
reg  [`IBW-1:0] E0, E1;                  // the two oldest entries
reg  [3:0]  rp, wp;                      // read pointer (E0's index) and write pointer
reg  [16:0] oc;                          // entries from rp, one-hot (0 .. 16)
wire        c0 = oc[0], c1 = oc[1], c2 = oc[2], c3 = oc[3];   // the buffer holds exactly 0 / 1 / 2 / 3 entries
reg         sqd;                         // a sequence's last uop left DS last clock: its entry is consumed now
reg  [6:0]  sqc /* synthesis syn_preserve = 1 */;   // copies of sqd (ik and pop copies)
reg         ib_v0, ib_v1;                // E0, E1 valid
reg         ib_p1;                       // E1 valid and a pair is allowed (not in the CPU speed mode): the pair terms use this one
reg  [2:0]  raa, rab;                    // bank read addresses of entries rp + 2 and rp + 3
reg         rpar;                        // rp odd
wire [`IBW-1:0] rda, rdb;                // even / odd entries (banks below)
wire [`IBW-1:0] R2 = rpar ? rdb : rda, R3 = rpar ? rda : rdb;   // entries rp + 2 and rp + 3
// each bank in three bit slices (own write enables, below)
reg  [31:0]      iba0 [0:7], ibb0 [0:7];
reg  [31:0]      iba1 [0:7], ibb1 [0:7];
reg  [`IBW-65:0] iba2 [0:7], ibb2 [0:7];
assign rda = {iba2[raa], iba1[raa], iba0[raa]};   // asynchronous LUT RAM reads at registered addresses
assign rdb = {ibb2[rab], ibb1[rab], ibb0[rab]};
wire [2:0]  wa_a = wp[0] ? wp[3:1] + 3'd1 : wp[3:1];   // write address in the even bank (the next even entry)
// The banks' write enables (fetch's late ex1 / ex2) in three copies, one per bit slice of the entries.
// Gowin merged the syn_keep copies into one net, so each copy takes its own preserved copy of wp[0].
reg  [5:0]  wpc /* synthesis syn_preserve = 1 */;
wire        we_a0 = wpc[0] ? ex2 : ex1, we_a1 = wpc[1] ? ex2 : ex1, we_a2 = wpc[2] ? ex2 : ex1;   // even bank: xe1 if wp is odd
wire        we_b0 = wpc[3] ? ex1 : ex2, we_b1 = wpc[4] ? ex1 : ex2, we_b2 = wpc[5] ? ex1 : ex2;   // odd bank: xe0 if wp is odd
// The decoder gates and legality of each entry fetch writes (p68_pdec), stored with the entry; the decoders below gate
// their uops with E0's copy (registered) instead of their legality logic.
wire [19:0] pq0, pq1;
wire [14:0] sx0, sx1;                    // data code selects of the entries (p68_pdec)
p68_pdec i_pq0 (.w0(xe0[15:0]), .w1(xe0[31:16]), .w2(xe0[47:32]), .w3(xe0[63:48]), .w4(xe0[79:64]), .sup(sup), .msp(msp), .pq(pq0), .sx(sx0));
p68_pdec i_pq1 (.w0(xe1[15:0]), .w1(xe1[31:16]), .w2(xe1[47:32]), .w3(xe1[63:48]), .w4(xe1[79:64]), .sup(sup), .msp(msp), .pq(pq1), .sx(sx1));
wire [`IBW-1:0]  xe0q = {sx0, pq0, xe0[94:0]}, xe1q = {sx1, pq1, xe1[94:0]};   // the complete entries
wire [`IBW-1:0]  wda = wp[0] ? xe1q : xe0q, wdb = wp[0] ? xe0q : xe1q;        // what each bank writes
always @(posedge clk) begin
  if (we_a0) iba0[wa_a] <= wda[31:0];  if (we_a1) iba1[wa_a] <= wda[63:32];  if (we_a2) iba2[wa_a] <= wda[`IBW-1:64];
  if (we_b0) ibb0[wp[3:1]] <= wdb[31:0]; if (we_b1) ibb1[wp[3:1]] <= wdb[63:32]; if (we_b2) ibb2[wp[3:1]] <= wdb[`IBW-1:64];
end
// E0's fields: the head instruction
wire [15:0] w0 = E0[15:0], w1 = E0[31:16], w2 = E0[47:32], w3 = E0[63:48], w4 = E0[79:64];   // opcode and extension words
wire [3:0]  d_len  = E0[`IB_LEN];        // its length
wire        d_sq   = E0[`IB_SQ];         // it is a sequence
wire        d_full = E0[`IB_FULL];       // it has a full-format EA
wire        e0h    = E0[`IB_HOLD];       // it is a hold entry
wire        e0m    = E0[`IB_PBTC] | E0[`IB_PFIX];   // it carries prediction slot 0's mark
wire        e1m    = E1[`IB_PBTC] | E1[`IB_PFIX];   // E1 carries a mark
wire        d_pbtc = E0[`IB_PBTC];       // E0 is the BTC-predicted branch
wire        d_pfix = e0h ? hpf : E0[`IB_PFIX];   // E0 was cut wrongly (for a hold entry fetch tells at the real length)
wire [15:0] x0 = E1[15:0], x1 = E1[31:16], x2 = E1[47:32];   // E1's words (a second instruction has at most three)
wire [3:0]  plen  = d_len + E1[`IB_LEN];  // length of the pair
wire        xsimp = E1[`IB_SIMP], xmem = E1[`IB_MEM], xbr = E1[`IB_BR], hsimp = E0[`IB_SIMP];   // pairing classes
wire        pcut  = e0m | e1m;           // slot 0's cut lies inside the pair
wire        pcx   = e0m & e1m;           // slot 1's cut too
wire        pbtc2 = E1[`IB_PBTC] & !e0m; // the second instruction is slot 0's predicted branch
wire        fx_mod, fx_emod;             // p68_fx changed the opcode's EA field(s)
reg         dfl_q;                       // a return prediction last clock: fetch's extractor writes of this clock are dropped
wire        exg1, exg2;                  // fetch's writes that are kept
wire [1:0]  fx_cv;                       // which EA p68_fx converted: {destination, source}

// ---- full-format EAs: the decoders see the opcode with a converted EA's mode field set to 2, (An) (p68_fx) ----
wire        fx_busy, fx_main, fx_go, fx_first, fx_enter, fx_cs, fx_cd, fx_sqclr;   // p68_fx state and outputs
wire [4:0]  fx_slot;                     // the pre-sequence's step
wire [3:0]  fx_len;                      // the real instruction length p68_fx found
wire        fx_ilv;                      // issue with fx_len (p68_fx MAIN, or an FPU #imm .B/.W)
wire [`UW-1:0] u_fx;                     // the pre-sequence's uop
// the entry holds the changed opcode in the main part (fx_mod), so the decoders read w0 directly
// register copies of the opcode word per decoder group (E0[15:0] has every decoder as load)
reg  [15:0] w0c1 /* synthesis syn_preserve = 1 */;
reg  [15:0] w0c2 /* synthesis syn_preserve = 1 */;
reg  [15:0] w0c3 /* synthesis syn_preserve = 1 */;
wire        h_p7; wire [`UW-1:0] u_p7; wire [3:0] sq_c_p7, sq_c_mvx; wire [63:0] sq_p_p7, sq_p_mvx;   // dec_p7 outputs; MOVE's sequence class and parameters
// ---- group decoders -------------------------------------------------------------------------------------------
// Each gets the opcode from one of the three copies (w0c1-w0c3) and the extension words from E0, and its uop gate pg
// from E0's predecoded bits (pqe, order in p68_pdec.v). seq/spar are a sequence class and its parameters for the
// sequencers (zero without a hit).
wire [19:0] pqe = E0[`IB_PQ];   // E0's predecoded gates
wire [`UW-1:0] u_move, u_alu, u_quick, u_branch, u_lea, u_shift, u_movem;
wire h_move, h_alu, h_quick, h_branch, h_lea, h_shift, h_movem;
wire [3:0]  sq_cls, sq_c_mv;             // the ORed sequence class; MOVEM's
wire [63:0] sq_par, sq_p_mv;             // the ORed sequence parameters; MOVEM's
dec_move   i_move   (.w0(w0c1), .w1(w1), .w2(w2), .w3(w3), .w4(w4), .hit(h_move), .pg(pqe[0]), .ug(),   .u(u_move), .seq(sq_c_mvx), .spar(sq_p_mvx));
dec_alu    i_alu    (.w0(w0c1), .w1(w1), .w2(w2), .w3(w3), .hit(h_alu), .pg(pqe[1]), .ug(),    .u(u_alu));
dec_quick  i_quick  (.w0(w0c1), .w1(w1), .w2(w2), .w3(w3), .hit(h_quick), .pg(pqe[2]), .ug(),  .u(u_quick));
dec_branch i_branch (.w0(w0c1), .w1(w1), .w2(w2), .w3(w3), .hit(h_branch), .pg(pqe[3]), .ug(), .u(u_branch));
dec_lea    i_lea    (.w0(w0c1), .w1(w1), .w2(w2), .w3(w3), .hit(h_lea), .pg(pqe[4]), .ug(),    .u(u_lea));
dec_shift  i_shift  (.w0(w0c1), .w1(w1), .w2(w2), .w3(w3), .hit(h_shift), .pg(pqe[5]), .ug(),  .u(u_shift));
dec_movem  i_movem  (.w0(w0c1), .w1(w1), .w2(w2), .w3(w3), .hit(h_movem), .pg(pqe[6]), .ug(),  .u(u_movem), .seq(sq_c_mv), .spar(sq_p_mv));
// ---- program flow (dec_flow.v): JMP, JSR single uops; BSR RTS RTD RTR PEA LINK UNLK sequences of class SQ_FLOW ----
wire [`UW-1:0] u_flow; wire h_flow; wire [3:0] sq_c_flow; wire [63:0] sq_p_flow;
dec_flow   i_flow   (.w0(w0c1), .w1(w1), .w2(w2), .w3(w3), .hit(h_flow), .pg(pqe[7]), .ug(), .u(u_flow), .seq(sq_c_flow), .spar(sq_p_flow));

// ---- the rest of the integer set (each file documents its sequence parameters) ----
wire [`UW-1:0] u_imm, u_unary, u_ext, u_scc, u_bit, u_shiftm, u_addx;
wire h_imm, h_unary, h_ext, h_scc, h_bit, h_shiftm, h_addx, h_pack, h_movep;
wire [1:0]  xs_imm, xs_bit;              // where the EA's extension word is (behind an immediate)
wire h_rox; wire [3:0] sq_c_rox; wire [63:0] sq_p_rox;
dec_rox    i_rox    (.w0(w0c2), .w1(w1), .w2(w2), .w3(w3), .hit(h_rox), .seq(sq_c_rox), .spar(sq_p_rox));
wire [3:0]  sq_c_ext, sq_c_addx, sq_c_pack, sq_c_movep;
wire [63:0] sq_p_ext, sq_p_addx, sq_p_pack, sq_p_movep;
dec_imm    i_imm    (.w0(w0c2), .w1(w1), .w2(w2), .w3(w3), .w4(w4), .hit(h_imm), .pg(pqe[8]), .ug(),    .u(u_imm), .xs(xs_imm));
dec_unary  i_unary  (.w0(w0c2), .w1(w1), .w2(w2), .w3(w3), .hit(h_unary), .pg(pqe[9]), .ug(),  .u(u_unary));
dec_ext    i_ext    (.w0(w0c2), .w1(w1), .w2(w2), .w3(w3), .hit(h_ext), .pg(pqe[10]), .ug(),    .u(u_ext), .seq(sq_c_ext), .spar(sq_p_ext));
dec_scc    i_scc    (.w0(w0c2), .w1(w1), .w2(w2), .w3(w3), .hit(h_scc), .pg(pqe[11]), .ug(),    .u(u_scc));
dec_bit    i_bit    (.w0(w0c2), .w1(w1), .w2(w2), .w3(w3), .hit(h_bit), .pg(pqe[12]), .ug(),    .u(u_bit), .xs(xs_bit));
dec_shiftm i_shiftm (.w0(w0c2), .w1(w1), .w2(w2), .w3(w3), .hit(h_shiftm), .pg(pqe[13]), .ug(), .u(u_shiftm));
dec_addx   i_addx   (.w0(w0c2), .w1(w1), .w2(w2), .w3(w3), .hit(h_addx), .pg(pqe[14]), .ug(),   .u(u_addx), .seq(sq_c_addx), .spar(sq_p_addx));
dec_pack   i_pack   (.w0(w0c3), .w1(w1), .w2(w2), .w3(w3), .hit(h_pack),   .seq(sq_c_pack), .spar(sq_p_pack));
dec_movep  i_movep  (.w0(w0c3), .w1(w1), .w2(w2), .w3(w3), .hit(h_movep),  .seq(sq_c_movep), .spar(sq_p_movep));
// ---- bit fields ----
wire h_bf; wire [3:0] sq_c_bf; wire [63:0] sq_p_bf;
dec_bf     i_bf     (.w0(w0c3), .w1(w1), .w2(w2), .w3(w3), .hit(h_bf), .seq(sq_c_bf), .spar(sq_p_bf));
dec_p7     i_p7     (.w0(w0c3), .w1(w1), .w2(w2), .w3(w3), .hit(h_p7), .u(u_p7), .seq(sq_c_p7), .spar(sq_p_p7));   // CAS, CAS2, long forms
// the sequence classes and parameters of all decoders, ORed (only the hitting decoder gives non-zero values)
assign sq_cls = sq_c_fpu | sq_c_mv | sq_c_ext | sq_c_addx | sq_c_pack | sq_c_movep | sq_c_rox | sq_c_md | sq_c_flow | sq_c_bf | sq_c_p7 | sq_c_mvx;
assign sq_par = sq_p_fpu | sq_p_mv | sq_p_ext | sq_p_addx | sq_p_pack | sq_p_movep | sq_p_rox | sq_p_md | sq_p_flow | sq_p_bf | sq_p_p7 | sq_p_mvx;
// index register field: the EA extension word is w1, or w2 / w3 behind an immediate operand (dec_imm, dec_bit)
wire [1:0]  xsl = xs_imm | xs_bit | xs_md | ((`P68_FPU != 0) && (w0[15:9] == 7'b1111001) ? xs_fpu : 2'd0);   // FPU: opcode test only
wire [15:0] xw  = (xsl == 2'd1) ? w2 : (xsl == 2'd2) ? w3 : w1;   // the brief extension word (index register in bits 15-12)
// ---- MULU/MULS/DIVU/DIVS (dec_md.v) ----
wire [`UW-1:0] u_md; wire h_md; wire [1:0] xs_md; wire [3:0] sq_c_md; wire [63:0] sq_p_md;
dec_md     i_md     (.w0(w0c3), .w1(w1), .w2(w2), .w3(w3), .hit(h_md), .pg(pqe[15]), .ug(), .u(u_md), .xs(xs_md), .seq(sq_c_md), .spar(sq_p_md));
wire [`UW-1:0] u_dec;          // decoders are off while a sequence runs (below)
// system decoder, illegal instruction on no hit
wire [`UW-1:0] u_sys, u_ill;
wire        h_sys, is_rst;               // dec_sys hit; the instruction is RESET
wire [2:0]  ss_cls;                      // system sequence class (p68_sseq)
wire [44:0] ss_par;                      // its parameters
// ---- FPU (dec_fpu.v); DS's view of the NULL state: cleared when an FPU instruction other than FSAVE and FRESTORE
// starts, reloaded from the committed state at every flush (FRESTORE ends with a redirect). The NULL state decides the
// size of the frame FSAVE writes. ----
wire [`UW-1:0] u_fpu; wire h_fpu, lov_fpu; wire [1:0] xs_fpu; wire [3:0] sq_c_fpu, ln_fpu; wire [63:0] sq_p_fpu;
reg         nul_ds;                      // DS's copy of the FPU NULL state
// dec_fpu's kind is FSAVE / FRESTORE exactly for these (supervisor, legal EA), from the opcode and S:
// FSAVE ($F3xx, type 100) with (An), -(An), (d16,An), (d8,An,Xn), abs.W, abs.L; FRESTORE (type 101) with (An), (An)+,
// (d16,An), (d8,An,Xn), abs.W, abs.L, (d16,PC), (d8,PC,Xn)
wire [2:0]  fm3 = w0[5:3], fr0 = w0[2:0];
wire        fp_sr = sup && (w0[15:9] == 7'b1111001) &&
                    ((w0[8:6] == 3'b100 && (fm3 == 3'd2 || fm3 == 3'd5 || fm3 == 3'd6 || (fm3 == 3'd7 && fr0 <= 3'd1) || fm3 == 3'd4)) ||
                     (w0[8:6] == 3'b101 && (fm3 == 3'd2 || fm3 == 3'd5 || fm3 == 3'd6 || (fm3 == 3'd7 && fr0 <= 3'd3) || fm3 == 3'd3)));
generate if (`P68_FPU != 0) begin : g_fpu
dec_fpu    i_fpu    (.w0(w0c3), .w1(w1), .w2(w2), .w3(w3), .sup(sup), .nul(nul_ds), .en(fpu_en), .pv(fpu_pv), .hit(h_fpu), .pg(pqe[18:17]), .ug(), .u(u_fpu), .xs(xs_fpu),
                     .seq(sq_c_fpu), .spar(sq_p_fpu), .lov(lov_fpu), .lnew(ln_fpu));
end else begin : g_nofpu
assign u_fpu = {`UW{1'b0}}; assign h_fpu = 1'b0; assign lov_fpu = 1'b0; assign xs_fpu = 2'd0; assign sq_c_fpu = 4'd0;
assign ln_fpu = 4'd0; assign sq_p_fpu = 64'h0;   // no FPU: the decoder's outputs are zero
end endgenerate
dec_sys    i_sys    (.w0(w0c3), .w1(w1), .w2(w2), .w3(w3), .sup(sup), .msp(msp), .fpu_hit(h_fpu), .hit(h_sys), .pg(pqe[16]), .ug(), .u(u_sys),
                     .seq(ss_cls), .spar(ss_par), .is_rst(is_rst));
assign u_ill = {`UW{1'b0}};                    // the illegal-instruction vector goes to AG as ds_ill (not through the OR)

// ---- instruction address: one add behind the issue ------------------------------------------------------------
// pcq is the address of the last issued instruction (or a jump target with lenq = 0), so the current PC is pcq plus
// the last length; this keeps the adder behind registers only.
reg  [31:0] pcq; reg [3:0] lenq;
wire [31:0] pc_cur = pcq + {27'h0, lenq, 1'b0};               // address of E0
wire        ds_ok  = ib_v0 && !sqd;                           // E0 is valid and not already consumed

// ---- sequencer ------------------------------------------------------------------------------------------------
wire        sq_act, sq_last;             // p68_seq runs; this is its last step
wire [4:0]  sq_step;                     // its step
wire [`UW-1:0] u_seq;                    // its uop
reg  [4:0]  rs_skip;                     // replay restart step
// system sequencer, exception sequencer, hold
wire        ss_act, ss_last, xs_act, xs_last;
wire [4:0]  ss_step, xs_step;
wire [`UW-1:0] u_sseq, u_xseq;
reg         hold_x, hold_r;              // after the exception sequence's last uop; during RESET
wire        any_act = sq_act | ss_act | xs_act | fx_busy;              // some sequencer runs (decoders off)
// ---- return stack (4 entries, pushed by JSR/BSR, popped by RTS/RTD/RTR). There is no wait for a BTC-predicted branch:
// the next instruction's PC is the BTC target pt (the address fetch went to; a wrong target redirects from EX anyway). ----
reg  [31:0] ras0, ras1, ras2, ras3, ras_l;   // the four return addresses; the address popped last
reg  [1:0]  rsp;                         // stack pointer (the newest entry), wraps around
reg         rk, rre;                     // a return sequence runs; it was restarted after a replay
wire [31:0] ras_t = (rsp == 2'd0) ? ras0 : (rsp == 2'd1) ? ras1 : (rsp == 2'd2) ? ras2 : ras3;   // top of the stack
// adv does not include flush (every DS register gives flush priority; the uop issued in a flush clock is dropped by
// DR), so the stall and flush nets stay apart.
// stopped as a local register (one clock late): STOP redirects when it commits, so DS sees the flush in the clock
// stopped rises; when it falls (an interrupt take, also a flush) DS waits one clock more.
reg         stp_r;
// CPU speed mode (slow): after an instruction issues, DS waits until its cost has passed since that issue. The cost is
// the 68EC020's cycle count from the cache case of the 68020 User's Manual (as in Musashi's 020 column), in core clocks
// (times 4.7 for 66.67 / 14.18 MHz), from the opcode word in E0 (cost_f, a few classes). The count starts at the
// issue, so a memory wait inside it overlaps the budget; no pairs (see cand2, c2p). The wait rides on stp_r, a register
// already in adv, so only its D input changed and the pipeline has nothing new. slow comes from a register.
// History: first a fixed throttle of N clocks per instruction with dual issue off; then each instruction was charged its
// cycles times 4.7 instead of a fixed 28 clocks; then the cost was registered in stages (opcode to base and EA cost to
// sum to counter) and the tables were scaled to the real core clock (the KB/KE macros in p68_uop.vh).
function [7:0] cea_f;                    // effective address cost: mode, register (core clocks)
  input [2:0] m, r;
  case (m)
    3'd2, 3'd3: cea_f = `P68_KE(19);           // (An), (An)+
    3'd4, 3'd5: cea_f = `P68_KE(24);           // -(An), (d16,An)
    3'd6:       cea_f = `P68_KE(33);           // (d8,An,Xn)
    3'd7:       cea_f = (r == 3'd0 || r == 3'd1) ? `P68_KE(19) : (r == 3'd2) ? `P68_KE(24) : (r == 3'd3) ? `P68_KE(33) : (r == 3'd4) ? `P68_KE(9) : 8'd0;   // abs, (d16,PC), (d8,PC,Xn), #imm
    default:    cea_f = 8'd0;            // Dn, An
  endcase
endfunction
function [7:0] cead_f;                    // the same less 3 (a MOVE destination is its base cost)
  input [2:0] m, r;
  case (m)
    3'd2, 3'd3: cead_f = `P68_KB(16);           // (An), (An)+
    3'd4, 3'd5: cead_f = `P68_KB(21);           // -(An), (d16,An)
    3'd6:       cead_f = `P68_KB(30);           // (d8,An,Xn)
    3'd7:       cead_f = (r == 3'd0 || r == 3'd1) ? `P68_KB(16) : (r == 3'd2) ? `P68_KB(21) : (r == 3'd3) ? `P68_KB(30) : (r == 3'd4) ? `P68_KB(6) : 8'd0;
    default:    cead_f = 8'd0;            // Dn, An
  endcase
endfunction
function [15:0] cost_f;                  // {base, effective address}; base is the cost less 3 (the pipeline below adds 3 clocks)
  input [15:0] w;
  reg [7:0] b, e, dd;                    // base cost, source EA cost, MOVE destination cost
  reg       mem, dm;                     // the source EA is in memory; the result goes to memory
  begin
    mem = (w[5:3] != 3'd0) && (w[5:3] != 3'd1);
    dm  = mem && w[8] && (w[7:6] != 2'b11);               // the result goes to memory (Dn,<ea> forms)
    e = cea_f(w[5:3], w[2:0]);
    dd = cead_f(w[8:6], w[11:9]);
    b = `P68_KB(6);                      // the cheapest instruction (a register operation)
    case (w[15:12])                      // by line (opcode bits 15-12)
      4'h0: begin b = (w[5:3] == 3'd0) ? `P68_KB(6) : `P68_KB(16); if (w[8] || w[11:9] == 3'd4) b = `P68_KB(16); end   // immediate and bit ops
      4'h1, 4'h2, 4'h3: begin                              // MOVE: destination cost + source EA
        b = (w[8:6] == 3'd0 || w[8:6] == 3'd1) ? `P68_KB(6) : (w[8:6] == 3'd7 && w[11:9] == 3'd1) ? `P68_KB(25) : dd;
        if (w[8:6] == 3'd7 && w[11:9] == 3'd0) b = `P68_KB(16);
      end
      4'h4: begin
        e = 8'd0;
        if (w == 16'h4E75) b = `P68_KB(44);                      // RTS
        else if (w == 16'h4E73) b = `P68_KB(57);                 // RTE
        else if (w == 16'h4E77) b = `P68_KB(63);                 // RTR
        else if (w[11:6] == 6'b111010 || w[11:6] == 6'b111011) b = `P68_KB(25);   // JSR, JMP
        else if (w[8:6] == 3'b111) b = `P68_KB(6);               // LEA
        else if (w[11:6] == 6'b100001) b = `P68_KB(21);          // PEA
        else if (w[11] && w[9:7] == 3'b001) b = `P68_KB(35);     // MOVEM
        else if (w[11:4] == 8'hE5) b = `P68_KB(21);              // LINK / UNLK
        else if (w[11:8] == 4'h8 && w[7:6] != 2'b11 && w[5:3] == 3'd0) b = `P68_KB(16);   // SWAP, EXT
        else if (w[7:6] != 2'b11 && mem) begin b = `P68_KB(16); e = cea_f(w[5:3], w[2:0]); end   // CLR, NEG, NOT, TST on memory
        else b = `P68_KB(6);
      end
      4'h5: begin
        if (w[7:6] != 2'b11) b = mem ? `P68_KB(16) : `P68_KB(6);       // ADDQ, SUBQ
        else if (w[5:3] == 3'd1) begin b = `P68_KB(25); e = 8'd0; end   // DBcc
        else begin b = mem ? `P68_KB(25) : `P68_KB(16); e = 8'd0; end  // Scc
      end
      4'h6: begin b = (w[11:8] == 4'h1) ? `P68_KB(30) : `P68_KB(25); e = 8'd0; end   // BSR, Bcc, BRA
      4'h7: begin b = `P68_KB(6); e = 8'd0; end                  // MOVEQ
      4'h8: begin b = (w[8:6] == 3'b011 || w[8:6] == 3'b111) ? `P68_KB(204) : dm ? `P68_KB(16) : `P68_KB(6); end   // DIVU, DIVS, OR
      4'h9, 4'hD: b = dm ? `P68_KB(16) : `P68_KB(6);                   // SUB, ADD
      4'hB: begin b = (w[8:6] == 3'b011 || w[8:6] == 3'b111) ? `P68_KB(16) : (w[8] && w[7:6] != 2'b11 && w[5:3] != 3'd1) ? (mem ? `P68_KB(16) : `P68_KB(6)) : `P68_KB(6); end   // CMPA, EOR, CMP
      4'hC: begin b = (w[8:6] == 3'b011 || w[8:6] == 3'b111) ? `P68_KB(124) : dm ? `P68_KB(16) : `P68_KB(6); end   // MULU, MULS, AND
      4'hE: begin
        if (w[7:6] == 2'b11) b = `P68_KB(16);                    // memory shift, bit field
        else if (w[4:3] == 2'b10 || w[4:3] == 2'b11) begin b = `P68_KB(35); e = 8'd0; end   // ROXL, ROL
        else begin b = w[5] ? `P68_KB(25) : `P68_KB(16); e = 8'd0; end // ASL, LSL
        if (w[7:6] != 2'b11) e = 8'd0;
      end
      4'hF: begin b = `P68_KB(35); e = 8'd0; end                 // FPU
      default: b = `P68_KB(6);
    endcase
    cost_f = {b, e};
  end
endfunction
// Cost pipeline, register to register: E0's opcode gives base and EA cost (thr_b, thr_e), their sum (thr_s), the sum
// loads the counter two clocks after the issue; the stall starts at once (issue, ths1, ths2) and the counter takes
// over, so the 3 clocks of the pipeline are part of the budget (the table values are less 3).
reg  [7:0]  thr_b, thr_e, thr_s, thr_c;  // base cost, EA cost, their sum, the down counter
reg         ths1, ths2;                  // the issue, delayed one and two clocks
reg         thr_m;                       // the count goes on: the wait continues in the next clock
wire        thr_nx = slow && (issue || ths1 || ths2 || thr_m);   // DS waits in the next clock
always @(posedge clk) begin
  {thr_b, thr_e} <= cost_f(w0);          // computed every clock from E0 (used only right after an issue)
  thr_s <= thr_b + thr_e;
  ths1 <= slow && issue;
  ths2 <= ths1;
  if (rst || !slow) begin thr_c <= 8'd0; thr_m <= 1'b0; end
  else if (ths2)    begin thr_c <= thr_s; thr_m <= 1'b1; end
  else if (thr_m)   begin thr_c <= thr_c - 8'd1; thr_m <= (|thr_c[7:2]) || (thr_c[1] && thr_c[0]); end   // stop when the count reaches 2
end
always @(posedge clk) stp_r <= !rst && (stopped || thr_nx);
// adv: DS may hand a uop to DR this clock (DR has room, no hold, not stopped)
wire        adv   = !stall_ag[0] && !hold_x && !hold_r && !stp_r;
// copies of adv for the units (each from its own register copy of stall_dr, because Gowin merges identical LUTs)
wire        adv_f = !stall_ag[1] && !hold_x && !hold_r && !stp_r, adv_q = !stall_ag[2] && !hold_x && !hold_r && !stp_r;
wire        adv_s = !stall_ag[3] && !hold_x && !hold_r && !stp_r;
wire        d_ok  = ds_ok && !any_act;   // E0 can be decoded (no sequencer owns DS)
// A full-format instruction goes through p68_fx first; in its main part the sequence flag is that of the changed opcode
// (a converted MOVE (d8,An,Xn) destination is no sequence).
wire        fx_msq;                      // fx_main && fx_sqclr (a p68_fx register)
wire        d_sq_e = d_sq && !fx_msq;    // E0 starts a sequence
reg         fxm1;                        // p68_fx in its main part for a clock (fetch's hpf has the real length)
// fmm = fx_main && fxm1 as one register (p68_fx leaves its main part only by an issue, a flush or reset)
reg         fmm;
wire        d_go   = d_ok && (!d_full || fmm);   // E0 may issue its decoder uop (a full-format one only in its main part)
wire        issue_d = adv && !xs_act && d_go && !d_sq_e;       // a decoder uop issues the instruction (registers only)
wire        start = d_go && adv && d_sq_e;                     // a sequence starts (its step 0 leaves now)
reg         rs_fxm;                      // the replay restarts in p68_fx's main part
reg  [4:0]  rsn; reg rfn;            // restart step and main-part flag latched at the redirect
p68_fx i_fx (.clk(clk), .rst(rst), .flush(flush_fx), .start(d_ok && adv && d_full && !fx_main), .w0(w0), .ill(!pqe[19]),
             .d_len(d_len), .cnt(fcnt), .wx(wx), .xpos(xpos), .adv(adv_f), .rs_fxm(rs_fxm), .rs_skip(rs_skip),
             .issue(issue), .busy(fx_busy), .main(fx_main), .go(fx_go), .u(u_fx), .first(fx_first), .slot(fx_slot),
             .enter_main(fx_enter), .emod(fx_emod), .cs(fx_cs), .cd(fx_cd), .len(fx_len), .sqclr(fx_sqclr), .msq(fx_msq),
             .fpo(start && h_fpu && lov_fpu), .ilv(fx_ilv));                           // fpo: an FPU #imm form needs the length override
assign il_ov = fx_ilv;                                                 // one register (p68_fx)
assign fx_mod = fx_emod;                                         // without the illegal-opcode path (no decoder hit in it)
assign pop    = issue && (e0m || (pair && e1m));                  // a marked entry leaves (a hold entry with hpf replays: no pop)
// the copies take their own preserved copies of sqd (issue = issue_d || sqd)
assign popa = (issue_d || sqc[5]) && (e0m || (pair && e1m)); assign popb = (issue_d || sqc[6]) && (e0m || (pair && e1m));
assign hiss   = issue && e0h;                                   // a hold entry leaves (not the illegal-opcode path)
assign fx_cv  = {fx_cd, fx_cs};
assign il    = fx_len;
// Which sequencer starts, from the opcode alone. A starting sequence instruction is legal (fetch's predecode marks only
// those), so the system sequencer's classes (dec_sys: RTE, CHK2/CMP2, MOVES, CHK, MOVE (An)+/-(An),SR; the privileged
// ones in user mode raise vector 8 with no class) are recognised by their opcode patterns, without the decoders' EA
// checks and class merge. sq_cls != SQ_NONE and ss_cls != SS_NONE exclude each other.
wire        ss_go  = ((w0[15:11] == 5'b00000) && (w0[10:9] != 2'b11) && (w0[8:6] == 3'b011)) ||       // CHK2/CMP2
                     ((w0[15:12] == 4'h4) && w0[8] && !w0[6]) ||                                       // CHK
                     (sup && ((w0 == 16'h4E73) || ((w0[15:8] == 8'h0E) && (w0[7:6] != 2'b11)) ||         // RTE, MOVES
                              ((w0[15:6] == 10'b0100011011) && (w0[5:3] == 3'd3 || w0[5:3] == 3'd4)))); // MOVE (An)+/-(An),SR
// the T1 replacement of p68_fx's main part is applied inside the sequencers (ts, tr)
wire        t1s    = fx_main && fx_cs;
p68_seq i_seq (.clk(clk), .rst(rst), .flush(flush), .start(start && !ss_go),
               .cls_in(sq_cls), .par_in(sq_par), .mw(w1), .adv(adv_q),
               .act(sq_act), .step(sq_step), .last(sq_last), .u(u_seq), .ts(t1s), .tr(w0[2:0]));
p68_sseq i_sseq (.clk(clk), .rst(rst), .flush(flush), .start(start && ss_go), .cls_in(ss_cls),
                 .par_in(ss_par), .adv(adv_s), .act(ss_act), .step(ss_step), .last(ss_last), .u(u_sseq), .ts(t1s), .tr(w0[2:0]));
p68_xseq i_xseq (.clk(clk), .rst(rst), .flush(xflush), .go(xs_go), .xs_par(xs_par), .rs(xs_rs), .rs_step(xs_rs_step),
                 .adv(adv_s), .act(xs_act), .step(xs_step), .last(xs_last), .u(u_xseq));
wire        q_act  = sq_act | ss_act;                // a queue sequence runs (one that belongs to E0)
wire        q_last = sq_act ? sq_last : ss_last;
wire [4:0]  q_step = sq_act ? sq_step : ss_step;

// a uop leaves DS: an exception sequence uop, a decoder uop (not one being skipped after a replay), a sequence step at
// or after the restart step, or a p68_fx uop
assign ds_go = adv && (xs_act || (d_go && !(d_sq_e && rs_skip != 5'd0)) || (q_act && q_step >= rs_skip)) || fx_go;
// A sequence's instruction is consumed one clock after its last uop (sqd, a register), so every user of issue (buffer,
// fetch, return stack, PC) sees registers only.
wire        q_end = adv && !xs_act && q_act && q_last;           // a sequence's last uop leaves DS
assign issue = issue_d || sqd;
// Pairs. A second instruction of lane B's class (predecoded) issues with a single-uop head instruction when no BTC cut
// lies inside them; DR decides whether they really run as a pair. Or a memory instruction behind a head of lane B's
// class (DR swaps them: the memory instruction runs in lane A, the head in lane B as the older one). The first
// instruction after a replay issues alone (solo): a pair whose lane-A store hits the second instruction's code replays
// and must then run apart.
reg         solo, rpn;                   // issue the next instruction alone; a replay was requested last clock
// The second instruction is entry E1 (registers), so heads of any length pair.
wire        cbase = ib_p1 && !pcut && !d_sq && !d_full && !any_act && !fx_main && !solo;
// a LEA head is of lane B's class only as the second instruction (as the head it runs in AG)
wire        cswp  = hsimp && xmem && !(w0[15:12] == 4'h4 && w0[8:6] == 3'b111);
// a BTC-predicted Bcc/BRA/DBcc as the second instruction (lane B; the BTC cut is its last word)
wire        cbr   = ib_p1 && xbr && pbtc2 && !d_sq && !d_full && !any_act && !fx_main && !solo;
// Branch fold. A BTC-predicted branch at the head (the cut is its last word) issues with the first instruction at its
// target when that one is of lane B's class: the branch runs in lane A, the target instruction in lane B (younger; EX
// drops it when the branch redirects). The branch then costs no clock, also behind its own flag writer (integer loop
// CPI 0.759 to 0.625 when it was added).
wire        cfb   = ib_p1 && d_pbtc && xsimp && !pcx && !d_sq && !d_full && !any_act && !fx_main && !solo;
wire        cand2 = (((xsimp || cswp) && cbase) || cbr || cfb) && !is_rts1;   // an RTS flushes the queue (dfl), so it never pairs
wire        pair  = issue_d && cand2;
assign o_p2 = pair;
assign o_sw = pair && cswp;
// The second instruction's decoders: only the groups a pair's second instruction can belong to. They gate themselves
// (pg tied to their own ug), because E1's gates are not predecoded.
wire [`UW-1:0] v_m2, v_a2, v_q2, v_un2, v_im2, v_ex2;
wire        g2_m2, g2_a2, g2_q2, g2_un2, g2_im2, g2_ex2, g2_br2, g2_sc2;   // the pair decoders gate themselves
wire        k_m2, k_a2, k_q2, k_un2, k_im2, k_ex2, k_br2, k_sc2; wire [`UW-1:0] v_br2, v_sc2;   // their hits; branch uops
wire [3:0]  z_c1, z_c2; wire [63:0] z_p1, z_p2; wire [1:0] z_x1, z_x2;   // unused outputs
dec_move   i_move2  (.w0(x0), .w1(x1), .w2(x2), .w3(16'h0), .w4(16'h0), .hit(k_m2), .pg(g2_m2), .ug(g2_m2), .u(v_m2), .seq(z_c1), .spar(z_p1));
dec_alu    i_alu2   (.w0(x0), .w1(x1), .w2(x2), .w3(16'h0), .hit(k_a2), .pg(g2_a2), .ug(g2_a2), .u(v_a2));
dec_quick  i_quick2 (.w0(x0), .w1(x1), .w2(x2), .w3(16'h0), .hit(k_q2), .pg(g2_q2), .ug(g2_q2), .u(v_q2));
dec_unary  i_unary2 (.w0(x0), .w1(x1), .w2(x2), .w3(16'h0), .hit(k_un2), .pg(g2_un2), .ug(g2_un2), .u(v_un2));
dec_imm    i_imm2   (.w0({x0[15:6], 3'b000, x0[2:0]}), .w1(x1), .w2(x2), .w3(16'h0), .w4(16'h0), .hit(k_im2), .pg(g2_im2), .ug(g2_im2), .u(v_im2), .xs(z_x1));   // mode forced to Dn
dec_ext    i_ext2   (.w0(x0), .w1(x1), .w2(x2), .w3(16'h0), .hit(k_ex2), .pg(g2_ex2), .ug(g2_ex2), .u(v_ex2), .seq(z_c2), .spar(z_p2));
dec_branch i_br2    (.w0(x0), .w1(x1), .w2(x2), .w3(16'h0), .hit(k_br2), .pg(g2_br2), .ug(g2_br2), .u(v_br2));      // Bcc/BRA as the second instruction
dec_scc    i_scc2   (.w0({x0[15:8], 5'b11001, x0[2:0]}), .w1(x1), .w2(x2), .w3(16'h0), .hit(k_sc2), .pg(g2_sc2), .ug(g2_sc2), .u(v_sc2));   // DBcc pattern forced
reg  [`UW-1:0] u2m, u2c;                 // the second instruction's uop; a copy of its OR before the data values
// E1's uop is used only for a pair, whose second instruction is of lane B's class, the memory class or the branch class
// (predecoded). None of them has an index or PC-relative EA, three or more words or an immediate EA mode in an
// ORI..CMPI; a DBcc is the only group-5 branch. So dec_imm2 sees mode 0, dec_scc2 the DBcc pattern, and E1's data codes
// need no brief-extension kind (5) and no index fields.
function [31:0] dv2; input [31:0] cd;     // dval for E1: data code to value
  begin
    case ({cd[2:0], cd[4:3]})                       // k = 1 or 2 (no third word); candidates first, the code selects last
      {3'd1, 2'd1}: dv2 = {{16{x1[15]}}, x1};       // kind 1: sign-extended word
      {3'd1, 2'd2}: dv2 = {{16{x2[15]}}, x2};
      {3'd2, 2'd1}: dv2 = {16'h0, x1};              // kind 2: zero-extended word
      {3'd2, 2'd2}: dv2 = {16'h0, x2};
      {3'd3, 2'd1}: dv2 = {24'h0, x1[7:0]};         // kind 3: zero-extended byte
      {3'd3, 2'd2}: dv2 = {24'h0, x2[7:0]};
      {3'd4, 2'd1}: dv2 = {x1, x2};                  // abs.L, #imm.L
      default:      dv2 = 32'h0;                     // kind 0: the constant only
    endcase
    dv2 = dv2 | {{23{cd[13]}}, cd[13:5]};            // add the decoder's constant (sign-extended 9 bits)
  end
endfunction
// LEA (An), (d16,An), abs.W, abs.L as a second instruction
wire        lea2 = (x0[15:12] == 4'h4) && (x0[8:6] == 3'b111) && (x0[5:3] == 3'd2 || x0[5:3] == 3'd5 || (x0[5:3] == 3'd7 && x0[2:1] == 2'b00));
always @* begin
  u2m = v_m2 | v_a2 | v_q2 | v_un2 | v_im2 | v_ex2 | (xbr ? (v_br2 | v_sc2) : {`UW{1'b0}});
  // data codes to values (p68_dec.vh)
  u2c = u2m;
  u2m[`F_IMM] = dv2({18'h0, u2c[(`U_IMM + 5) +: 9], dsim(x0, 1'b1)});      // selects from the opcode (no line F, no
  u2m[`F_LA_DISP] = dv2({18'h0, u2c[(`U_LA + `LO_DISP + 5) +: 9], dsla(x0, 1'b1)});   // privileged form pairs)
  u2m[`F_SA_DISP] = dv2({18'h0, u2c[(`U_SA + `SO_DISP + 5) +: 9], dssa(x0)});
  u2m[`F_PC] = d_pbtc ? pt : pc_cur + {27'h0, d_len, 1'b0}; u2m[`F_LEN] = E1[`IB_LEN];   // behind a predicted branch (fold pair): its target
  u2m[`F_FIRST] = 1'b1; u2m[`F_LAST] = 1'b1; u2m[`F_STEP] = 5'd0;   // a second instruction is always one uop
  if (xbr) begin u2m[`F_IMM] = pt; u2m[`F_PBTC] = pbtc2; u2m[`F_PRED] = pbtc2; end   // a branch carries its predicted target
  // LEA (An), (d16,An), abs.W, abs.L as the second instruction is an add (or a move) in lane B: An + displacement
  if (lea2) begin
    u2m[`F_UNIT] = `UN_ALU; u2m[`F_OP] = (x0[5:3] == 3'd7) ? `OP_MOV : `OP_ADD; u2m[`F_SZ] = `SZ_L; u2m[`F_LSZ] = `SZ_L;
    u2m[`F_AEXT] = `AX_NONE; u2m[`F_ASEL] = `AS_IMM; u2m[`F_AREG] = 5'd0;
    u2m[`F_IMM] = (x0[5:3] == 3'd2) ? 32'h0 : (x0[5:3] == 3'd7 && x0[0]) ? {x1, x2} : {{16{x1[15]}}, x1};   // 0, abs.L, or d16 / abs.W sign-extended
    u2m[`F_BSEL] = `BS_REG; u2m[`F_BREG] = {2'b01, x0[2:0]}; u2m[`F_WEN] = 1'b1; u2m[`F_WREG] = {2'b01, x0[11:9]};   // B = An, write Am
    u2m[`F_FEN] = 5'd0; u2m[`F_FCV] = (x0[5:3] == 3'd7) ? `CV_CLR : `CV_ADD; u2m[`F_FZM] = `ZM_SET;   // LEA changes no flags
    u2m[`F_LA] = {`LA_W{1'b0}}; u2m[`F_SA] = {`SA_W{1'b0}}; u2m[`F_LEA] = 1'b0; u2m[`U_EXC + 2] = 1'b0;
  end
  if (!(k_m2 || k_a2 || k_q2 || k_un2 || k_im2 || k_ex2 || lea2 || (xbr && (k_br2 || k_sc2)))) u2m[`U_EXC + 2] = 1'b1;   // no hit: vector 4 (DR does not pair it)
end
assign o_u2 = u2m;
// ---- return prediction ----
wire        is_ret = E0[`IB_RET];   // with start: RTD / RTR (RTS is one decoder uop), predecoded instead of the decoder's class
wire        r_push = issue && !any_act && E0[`IB_CALL];          // a call issues: push (BSR is one uop)
wire [31:0] r_val  = pc_cur + {27'h0, fx_main ? fx_len : d_len, 1'b0};  // the return address (a full-format JSR has p68_fx's length)
// RTS is one decoder uop; it pops at its issue (after a replay of it, solo: the address popped last time).
wire        is_rts1 = E0[`IB_RTS] && !any_act;                // predecoded
// An RTS is predicted from the buffer registers, not at its issue: when it is E0, or E1 behind an E0 that does not use
// the return stack (dfl0 / dfl1; unmarked entries only). DS keeps the entries up to the RTS and flushes the rest; the
// RTS then issues later with the popped address (ras_l, rdn). RTD and RTR pop at their jump uop. Making RTS a single
// uop predicted this way saved two clocks per return (CPI 1.091 to 1.007 when it was done).
reg         rdn;                         // the RTS in E0 / E1 is predicted (popped)
wire        dfl_ok = !rdn && !any_act && !xs_act && !hold_x;
wire        dfl0   = dfl_ok && ib_v0 && E0[`IB_RTS] && !e0m;
wire        dfl1   = dfl_ok && ib_v1 && E1[`IB_RTS] && !e1m && !e0m && !E0[`IB_CALL] && !E0[`IB_RET];
wire        dfl_s  = sqd && rk;                                 // RTD / RTR: one clock after the jump uop left DS
wire        r_pop  = dfl_s || dfl0 || dfl1;                     // pop the stack now
wire        rre_e  = rk ? rre : (solo && !dfl1);                // a replayed RTS (solo, E0) uses the address popped last time
wire [31:0] r_pred = rre_e ? ras_l : ras_t;                     // the predicted return address
wire        rts_p  = is_rts1 && (rdn || dfl0);                  // the RTS issuing now is predicted
wire [31:0] rts_t  = rdn ? ras_l : r_pred;                      // its target
assign      dfl     = r_pop;
assign      dtgt    = r_pred;
assign      bt_kill = adv && !any_act && d_go && !(d_sq_e && rs_skip != 5'd0) && d_pfix;   // = ds_go of a decoder uop (registers only)

assign ds_ill = !pqe[19] && !any_act;                // legality predecoded (p68_pdec)
assign u_dec = u_move | u_alu | u_quick | u_branch | u_lea | u_shift | u_movem |
               u_imm | u_unary | u_ext | u_scc | u_bit | u_shiftm | u_addx | u_sys | u_ill | u_md |
               u_flow | u_p7 | u_fpu;                                    // DR selects it when no sequence runs
// In the main part of a p68_fx instruction, sequence uops addressing through the converted EA's register use T1
// instead (the changed EA is (An) with that register); the sequencers apply it (t1s).
// DS output in parts, merged by DR (p68_ag, function drm). Register numbers of a decoder uop come straight from the
// instruction words (used only when the uop's base select, update, LEA or index enable says so).
// The fields DR always replaces for a decoder uop (base and index register numbers, PFIX: drm in p68_ag) are left out
// here, so their decode logic is removed.
reg  [`UW-1:0] o_dm;
// The value selects of the data codes come predecoded with the entry (IB_SX, p68_pdec: dsx in p68_dec.vh), so the value
// muxes start from registers; E0m recomputes them for p68_fx's changed opcode. The decoders' codes add only their
// constants (bits 13:5).
wire [14:0] sxe = E0[`IB_SX];
wire [4:0]  s_im = sxe[4:0], s_la = sxe[9:5], s_sa = sxe[14:10];   // {k, kind} for IMM, LA_DISP, SA_DISP
wire [15:0] lxw = dwk(s_la[4:3], w1, w2, w3);                       // the extension word of the load EA (brief index fields)
always @* begin
  o_dm = u_dec;
  // data codes to values (p68_dec.vh), once for all decoders; brief index fields from the word of the LA code
  o_dm[`F_IMM] = dval({18'h0, u_dec[(`U_IMM + 5) +: 9], s_im}, w1, w2, w3, w4);
  o_dm[`F_LA_DISP] = dval({18'h0, u_dec[(`U_LA + `LO_DISP + 5) +: 9], s_la}, w1, w2, w3, w4);
  o_dm[`F_SA_DISP] = dval({18'h0, u_dec[(`U_SA + `SO_DISP + 5) +: 9], s_sa}, w1, w2, w3, w4);
  o_dm[`F_LA_XL] = u_dec[`F_LA_XEN] & lxw[11]; o_dm[`F_LA_XSC] = {2{u_dec[`F_LA_XEN]}} & lxw[10:9];   // brief word: bit 11 W/L, bits 10-9 scale
  o_dm[`F_LA_BREG] = 5'd0; o_dm[`F_SA_BREG] = 5'd0; o_dm[`F_LA_XREG] = 5'd0; o_dm[`F_PFIX] = 1'b0;   // DR fills these
end
assign o_dec = o_dm;
assign o_seq = u_seq | u_sseq | u_xseq | u_fx;      // sequencer uops are zero while they do not run
assign o_act = any_act;
wire        ov_pret = (sq_act && sq_last && rk) || rts_p;   // a predicted return's jump uop
// the override fields (p68_dr.vh)
assign o_ov[`DV_LAB]   = (fx_main && fx_cs) ? `R_T1 : (w0 == 16'h4E75) ? 5'd15 : {2'b01, w0[2:0]};   // converted EA: T1; RTS loads from A7; else An of bits 2-0
assign o_ov[`DV_SAB]   = (fx_main && fx_cd) ? `R_T2 : (w0[15:8] == 8'h61) ? 5'd15 : ((`P68_FPU != 0) && w0[15:9] == 7'b1111001) ? `R_T0 : {2'b01, w0[11:9]};   // T2; BSR pushes on A7; FPU LEA into T0; else An of bits 11-9
assign o_ov[`DV_LXR]   = xw[15:12];
assign o_ov[`DV_ADJ0]  = fx_main && fx_cd;                                    // T2 includes the source's step
assign o_ov[`DV_PFIX]  = d_pfix;                                              // decoder uops only
assign o_ov[`DV_PBTC]  = d_pbtc;
assign o_ov[`DV_ISEL]  = ov_pret || (!any_act && d_pbtc);
assign o_ov[`DV_PRET]  = ov_pret;
assign o_ov[`DV_IMM]   = ov_pret ? rts_t : pt;
assign o_ov[`DV_PC]    = xs_act ? u_xseq[`F_PC] : pc_cur;
assign o_ov[`DV_LEN]   = (fx_busy || fx_ilv) ? fx_len : d_len;                  // fx_ilv also covers an FPU #imm .B/.W (3 words)
assign o_ov[`DV_FIRST] = fx_busy ? fx_first : (!any_act && !fx_main);
assign o_ov[`DV_LAST]  = xs_act ? xs_last : q_act ? q_last : fx_busy ? 1'b0 : !d_sq_e;
assign o_ov[`DV_STEP]  = xs_act ? xs_step : q_act ? q_step : fx_busy ? fx_slot : 5'd0;
assign o_ov[`DV_FXP]   = fx_busy;
assign o_ov[`DV_FXM]   = fx_main;

// ---- DS state: FPU NULL view, holds, PC, return stack, replay restart ----
always @(posedge clk) begin
  if (rst) begin
    pcq <= 32'h0; lenq <= 4'd0; rs_skip <= 5'd0; hold_x <= 1'b0; hold_r <= 1'b0; rs_fxm <= 1'b0;
    rdn <= 1'b0; rk <= 1'b0;             // cleared only by a flush before: a reset while running kept them
    solo <= 1'b0; rpn <= 1'b0; nul_ds <= 1'b1;
  end else begin
    // ---- FPU NULL state ----
    if (flush) nul_ds <= fpu_null;
    else if (start && h_fpu && !fp_sr) nul_ds <= 1'b0;                 // FSAVE / FRESTORE recognised from the opcode
    if (flush) hold_x <= 1'b0; else if (ds_go && xs_act && xs_last) hold_x <= 1'b1;   // wait for the exception sequence's jump
    if (flush || rst_done) hold_r <= 1'b0; else if (ds_go && !any_act && is_rst) hold_r <= 1'b1;   // wait while RESET runs
    // the PC register: a flush target, a return target, the BTC target, a pair's branch target, or the next instruction
    if (flush) begin pcq <= flush_tgt; lenq <= 4'd0; end
    else if (dfl_s || (issue && rts_p)) begin pcq <= dfl_s ? dtgt : rts_t; lenq <= 4'd0; end   // return stack: at the RTS's issue
    else if (issue && !any_act && d_pbtc) begin pcq <= pt; lenq <= pair ? plen - d_len : 4'd0; end   // BTC target; fold pair adds the target instruction
    else if (issue && !any_act && pair && cbr) begin pcq <= pt; lenq <= 4'd0; end                    // a pair's branch
    else if (issue) begin pcq <= pc_cur; lenq <= pair ? plen : il_ov ? il : d_len; end   // sequential (one or two instructions)
    // ---- return stack ----
    if (flush || (issue && is_rts1)) rdn <= 1'b0; else if (dfl0 || dfl1) rdn <= 1'b1;
    if (flush) rk <= 1'b0;
    else if (start && is_ret) begin rk <= 1'b1; rre <= (rs_skip != 5'd0); end   // an RTD/RTR sequence starts (or restarts)
    else if (r_pop) rk <= 1'b0;
    if (r_push) begin rsp <= rsp + 2'd1;
      case (rsp + 2'd1) 2'd0: ras0 <= r_val; 2'd1: ras1 <= r_val; 2'd2: ras2 <= r_val; default: ras3 <= r_val; endcase
    end else if (r_pop && !rre_e) begin rsp <= rsp - 2'd1; ras_l <= ras_t; end   // a replayed return pops nothing
    // the restart step is latched at the redirect and applied with the delayed flush (rs_d)
    rsn <= (ex_rep && !ex_xs) ? ex_step : rs_one ? 5'd1 : 5'd0; rfn <= ex_rep && !ex_xs && ex_fxm;   // no enable (used at rs_d only)
    rpn <= redirect_ex && ex_rep;
    if (rs_d) solo <= rpn; else if (issue) solo <= 1'b0;                 // after a replay, the next instruction issues alone
    if (rs_d) rs_skip <= rsn;
    else if (issue || (fx_enter && !rs_fxm)) rs_skip <= 5'd0;            // the pre-part's restart is done
    if (rs_d) rs_fxm <= rfn;
    else if (issue) rs_fxm <= 1'b0;
  end
end


// ---- instruction buffer pointers, count and the E0 / E1 registers. The buffer consumes k = 0, 1, 2 entries: a decoder
// issue (issue_d, pair: from registers only) or, one clock after a sequence's last uop (sqd, a register), one. The
// sequencer's last-step logic therefore stays off these registers. Copies of k per register group keep each select net
// near 50 loads. ----
// "No sequence runs" (four registers) enters each copy last: issue_d = idp && !any_act, cand2 = c2p && !any_act.
function [1:0] ikf; input sd, id, c2, na; ikf = sd ? 2'd1 : (id && na) ? (c2 ? 2'd2 : 2'd1) : 2'd0; endfunction   // entries consumed
wire        idp /* synthesis syn_keep = 1 */, c2p /* synthesis syn_keep = 1 */;
assign idp = adv && !xs_act && ds_ok && (!d_full || fmm) && !d_sq_e;   // issue_d without the "no sequence" term
assign c2p = (((xsimp || cswp) && ib_p1 && !pcut && !d_sq && !d_full && !fx_main && !solo) ||
              (ib_p1 && xbr && pbtc2 && !d_sq && !d_full && !fx_main && !solo) ||
              (ib_p1 && d_pbtc && xsimp && !pcx && !d_sq && !d_full && !fx_main && !solo)) && !E0[`IB_RTS];   // cand2 likewise
// Gowin merged the syn_keep copies into one net of 118 loads, so each copy takes its own preserved copy of sqd.
wire [1:0]  ik   = ikf(sqc[0], idp, c2p, !any_act);           // for the pointers and the count
wire [1:0]  ik0a = ikf(sqc[1], idp, c2p, !any_act), ik0b = ikf(sqc[2], idp, c2p, !any_act);   // for E0's two halves
wire [1:0]  ik1a = ikf(sqc[3], idp, c2p, !any_act), ik1b = ikf(sqc[4], idp, c2p, !any_act);   // for E1's two halves
// a return prediction keeps the entries up to the RTS (E0, or E0 and E1) and drops the rest and this clock's writes
wire [1:0]  ikeep = dfl1 ? 2'd2 : 2'd1;
assign exg1 = ex1 && !dfl_q; assign exg2 = ex2 && !dfl_q;
// The count after this clock: the count less k (the issue selects among three shifts of the register), the return
// prediction's fixed counts, and fetch's writes (ex1 / ex2, the latest inputs) selecting last.
wire        iss2 = issue_d && !sqd;
wire [16:0] okk  = iss2 ? (cand2 ? {2'b0, oc[16:2]} : {1'b0, oc[16:1]}) : (sqd ? {1'b0, oc[16:1]} : oc);
wire        k0   = !iss2 && !sqd;
wire [16:0] ocd  = (dfl1 && k0) ? 17'd4 : ((dfl1 || k0) ? 17'd2 : 17'd1);   // one-hot count kept by a return prediction (2, 1 or 0 entries)
function [3:0] vsf; input [16:0] o; vsf = {!o[0], !o[0] && !o[1], !o[16], !o[16] && !o[15]}; endfunction   // {E0 valid, E1 valid, room for 1, room for 2}
// Fetch's write flags (ex1 / ex2, the latest inputs) select last; the return prediction (dfl) and the dropped writes
// after it (dfl_q) are folded into the value they select against.
wire        nwr  = dfl || dfl_q;          // no write is taken this clock
wire [16:0] oct  = dfl ? ocd : okk;
wire [3:0]  vst  = dfl ? vsf(ocd) : vsf(okk);
wire [16:0] oc_n = nwr ? oct : ex2 ? {okk[14:0], 2'b0} : ex1 ? {okk[15:0], 1'b0} : okk;
wire [3:0]  vs_n = nwr ? vst : ex2 ? vsf({okk[14:0], 2'b0}) : ex1 ? vsf({okk[15:0], 1'b0}) : vsf(okk);
wire [3:0]  wp1  = wp + 4'd1, wp2 = wp + 4'd2;
wire [3:0]  wpt  = dfl ? rp + {2'd0, ikeep} : wp;   // a return prediction moves the write pointer back behind the RTS
wire [3:0]  wp_n = nwr ? wpt : ex2 ? wp2 : ex1 ? wp1 : wp;
wire [3:0]  rp1 = rp + 4'd1, rp2 = rp + 4'd2;
wire [3:0]  rp_n = (ik == 2'd0) ? rp : (ik == 2'd1) ? rp1 : rp2;
// the value selects of p68_fx's changed opcode for each conversion, from registers; fx_cv selects last
wire [14:0] sxm1 = dsx({w0[15:6], 3'b010, w0[2:0]}, w1, sup);              // source EA converted to (An)
wire [14:0] sxm2 = dsx({w0[15:9], 3'b010, w0[5:0]}, w1, sup);              // destination EA converted
wire [14:0] sxm3 = dsx({w0[15:9], 3'b010, 3'b010, w0[2:0]}, w1, sup);      // both
// E1's upper part (p68_pdec's gates of xe1) enters one LUT: its select e1x and the other candidates (e1r) are kept
// apart. Before, the path from the queue through WX and p68_pdec into E1 had two select LUTs behind it.
wire        e1x /* synthesis syn_keep = 1 */;
assign      e1x = (ik1b == 2'd0) ? c0 : (ik1b == 2'd1) ? c1 : c2;   // E1 takes fetch's second entry this clock
wire [`IBW-49:0] e1r /* synthesis syn_keep = 1 */;
assign      e1r = (ik1b == 2'd0) ? (c1 ? xe0q[`IBW-1:48] : E1[`IBW-1:48]) : (ik1b == 2'd1) ? (c2 ? xe0q[`IBW-1:48] : R2[`IBW-1:48]) :
                  (c3 ? xe0q[`IBW-1:48] : R3[`IBW-1:48]);
reg  [`IBW-1:0] E0m;                     // E0 with p68_fx's changed opcode and value selects
always @* begin
  E0m = E0;
  if (fx_mod) E0m[15:0] = {w0[15:9], fx_cv[1] ? 3'b010 : w0[8:6], fx_cv[0] ? 3'b010 : w0[5:3], w0[2:0]};   // mode 2 = (An)
  if (fx_mod && fx_cv != 2'b00) E0m[`IB_SX] = fx_cv[1] ? (fx_cv[0] ? sxm3 : sxm2) : sxm1;   // selects of the changed opcode
end
always @(posedge clk) begin
  fxm1 <= fx_main && !flush;
  fmm  <= fx_main && !flush && !rst && !issue;                         // = fx_main && fxm1 in the next clock
  sqd  <= !flush && q_end;
  sqc  <= {7{!flush && q_end}};                                       // copies of sqd
  dfl_q <= dfl && !flush;
  if (rst || flush) begin
    rp <= 4'd0; wp <= 4'd0; wpc <= 6'd0; oc <= 17'd1; ib_v0 <= 1'b0; ib_v1 <= 1'b0; ib_p1 <= 1'b0;
    ib_sp1 <= 1'b1; ib_sp2 <= 1'b1; raa <= 3'd1; rab <= 3'd1; rpar <= 1'b0;   // empty buffer
  end else begin
    rp <= rp_n; wp <= wp_n; oc <= oc_n; wpc <= {6{wp_n[0]}};
    {ib_v0, ib_v1, ib_sp1, ib_sp2} <= vs_n;
    ib_p1 <= vs_n[2] && !slow;                                        // no pairs in the CPU speed mode
    // bank read addresses of entries rp_n + 2 and rp_n + 3 (selected late by k)
    case (ik)
      2'd0:    begin rpar <= rp[0];  rab <= rp[3:1] + 3'd1;  raa <= rp[0]  ? rp[3:1] + 3'd2  : rp[3:1] + 3'd1; end
      2'd1:    begin rpar <= rp1[0]; rab <= rp1[3:1] + 3'd1; raa <= rp1[0] ? rp1[3:1] + 3'd2 : rp1[3:1] + 3'd1; end
      default: begin rpar <= rp2[0]; rab <= rp2[3:1] + 3'd1; raa <= rp2[0] ? rp2[3:1] + 3'd2 : rp2[3:1] + 3'd1; end
    endcase
  end
  // E0 and E1 after the consumption: the next entry comes from E1, the banks (R2, R3) or fetch's write of this clock
  // (when the buffer would otherwise be empty at that place)
  case (ik0a)                                                         // the opcode copies
    2'd0:    begin w0c1 <= c0 ? xe0[15:0] : E0m[15:0]; w0c2 <= c0 ? xe0[15:0] : E0m[15:0]; w0c3 <= c0 ? xe0[15:0] : E0m[15:0]; end
    2'd1:    begin w0c1 <= c1 ? xe0[15:0] : E1[15:0];  w0c2 <= c1 ? xe0[15:0] : E1[15:0];  w0c3 <= c1 ? xe0[15:0] : E1[15:0];  end
    default: begin w0c1 <= c2 ? xe0[15:0] : R2[15:0];  w0c2 <= c2 ? xe0[15:0] : R2[15:0];  w0c3 <= c2 ? xe0[15:0] : R2[15:0];  end
  endcase
  case (ik0a)                                                         // E0's lower 48 bits
    2'd0:    E0[47:0] <= c0 ? xe0q[47:0] : E0m[47:0];
    2'd1:    E0[47:0] <= c1 ? xe0q[47:0] : E1[47:0];
    default: E0[47:0] <= c2 ? xe0q[47:0] : R2[47:0];
  endcase
  case (ik0b)                                                         // E0's upper bits
    2'd0:    E0[`IBW-1:48] <= c0 ? xe0q[`IBW-1:48] : E0m[`IBW-1:48];
    2'd1:    E0[`IBW-1:48] <= c1 ? xe0q[`IBW-1:48] : E1[`IBW-1:48];
    default: E0[`IBW-1:48] <= c2 ? xe0q[`IBW-1:48] : R2[`IBW-1:48];
  endcase
  case (ik1a)                                                         // E1's lower 48 bits
    2'd0:    E1[47:0] <= c0 ? xe1q[47:0] : c1 ? xe0q[47:0] : E1[47:0];
    2'd1:    E1[47:0] <= c1 ? xe1q[47:0] : c2 ? xe0q[47:0] : R2[47:0];
    default: E1[47:0] <= c2 ? xe1q[47:0] : c3 ? xe0q[47:0] : R3[47:0];
  endcase
  E1[`IBW-1:48] <= e1x ? xe1q[`IBW-1:48] : e1r;     // xe1q (fetch's W[L0] select, then p68_pdec) selects last
end

endmodule
`default_nettype wire
