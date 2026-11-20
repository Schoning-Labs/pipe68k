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
// ==== p68_ag.v: the DR queue, the pairing decision and address generation (AG) =======================================
//
// Abbreviations used in this file: DS = decode stage (p68_decode.v), DR = the decode register queue at the front of
// this file, AG = address generation (the rest of this file), OC1/OC2 = the two operand stages (p68_oc.v), EX =
// execute (p68_ex.v), WB = write-back (p68_wb.v), BTC = branch target cache (in p68_fetch.v), EA = effective address
// (the 68k name for "where an operand lives"), An = an address register A0-A7, Dn = a data register D0-D7, T0-T3 = the
// temporaries the sequencers use, LA and SA = the load and store address descriptors of a uop (p68_uop.vh).
//
// ---- What this unit is for ----------------------------------------------------------------------------------------
//
// Every uop that touches memory needs an address, and on a 68k that address is the sum of up to three parts: a base
// (an address register, the PC, or zero for an absolute address), an index register that the 68020 can multiply by 1,
// 2, 4 or 8 (the "scaled index" of the (d8,An,Xn*4) mode), and a displacement that was in the instruction's extension
// words. DS has already turned every addressing mode into the same descriptor (base, index, scale, displacement), so
// AG has one adder for all of them and never needs to know which of the eighteen 68k modes it is looking at. AG also
// does the address register updates of (An)+ and -(An) and the write of LEA, early and speculatively (the register
// file keeps a committed copy that undoes them after a redirect, see p68_regfile.v), and it redirects fetch for
// branches whose target needs no register (BRA, a BSR, a Bcc predicted taken, a JMP to an absolute address).
//
// In front of AG sits DR, a two-entry queue. DS hands DR its output in parts (the ORed decoder uop, the ORed sequencer
// uop and some override fields, see p68_dr.vh) and DR merges them in its own clock (function drm). DR also makes the
// final decision whether the two instructions DS offered can go down the pipe together as a pair (dual issue).
//
// ---- Where it sits ------------------------------------------------------------------------------------------------
//
// Fed by: DS (o_dec, o_seq, o_ov, o_u2 and the pair hints), the register file (the base and index values, read one
// clock early), OC and EX and WB (for the interlock and for forwarding results into the address). Feeds: OC1 (n_u for
// lane A, n_ub for lane B, the load address ea_s and the store address saddr), the register file (the An updates w1_*
// and w2_*), and fetch (bra_ag with the target ag_tgt). Everything runs on the one CPU clock clk.
//
// ---- How it works in outline --------------------------------------------------------------------------------------
//
//  1. DR: DS writes an entry while the queue has room (stall_dr is a register, so AG's stall never reaches DS in the
//     same clock). The entry at the read pointer is merged into d_u (lane A) and d_ub (lane B) and enters AG when AG
//     does not stall.
//  2. The base and index registers are read from the register file one clock early, at the register numbers of the
//     uop that will be in AG next. A result that an older uop produces in the meantime is forwarded into those values
//     by small one-hot selects that are themselves registers, decided in the same early clock.
//  3. The load address is base + index * scale + displacement. Adding three numbers would take two carry chains in a
//     row; a 3:2 compressor (a row of full adders without carry propagation: one XOR and one majority per bit) turns
//     the three into two numbers first, and one carry chain adds those. On the Gowin chip that is one LUT level plus
//     one carry chain, which is about as fast as a single add.
//  4. The interlock: if the uop needs an address register that an older uop will only write in WB (a load into A0
//     followed by (A0)+, say), it waits here until the value can be forwarded. That decision is also a register
//     (hz_r), computed one clock early, because a stall fans out to every register of the stage.
//
// ---- What was tried, and why it ended up like this ----------------------------------------------------------------
//
// A DSP block can do a multiply-add, so base + index * scale + displacement looked like a natural job for it. Measured
// alone it ran at 114.7 MHz with two DSPs, against 200.5 MHz for the LUT compressor and carry chain, so AG kept the
// LUT adder. The interlock and the stall first came from logic in the same clock; they were made a register computed
// from next-clock values, and that version was checked against the old one over 410,000 instructions on the lockstep
// bench with no difference. The D-cache index bits (the low address bits that pick a cache line) come straight from
// the load address adder, so the path from a WB result through the forwarding, the index scale and the adder into the
// D-cache index was for a while the longest path family of the full Amiga system. It was cut in three steps: the
// index is scaled per forwarding part (so the late WB value passes only one small AND-OR before the compressor), the
// WB forward selects were folded into the scale selects, and OC's split decode takes AG's low store address bits
// directly. The base select got the same treatment. DR itself came from a timing round: the full merge of decoder and
// sequencer uops did not fit into DS's clock, and the pairing checks of dual issue needed a place to live anyway.
//
// ---- Gowin techniques in this file --------------------------------------------------------------------------------
//
//  - syn_keep on a wire stops synthesis from merging logic across it, so an AND-OR can be split into "early parts" and
//    one "late part" that meets them in the last LUT. That is how the late register file value or the late WB value
//    is kept to a single LUT in front of the adder.
//  - syn_preserve on a register keeps a copy that synthesis would otherwise merge with its twin. Gowin merges identical
//    LUTs even when each output has syn_keep, so a hand copy of a high-fanout enable only stays a copy when one of its
//    inputs is its own preserved register (agv_r, agv_f, agv_v, agv_u, agv_n below, and the copies of the DR pointer).
//  - syn_maxfan = 16 on the module asks synthesis to duplicate drivers with more than 16 loads inside this unit.
`include "p68_uop.vh"                // the uop format: field macros F_*, U_*, codes UN_*, OP_*, BA_* and so on
`include "p68_dr.vh"                 // the override fields DS hands to DR (DV_*)
`include "p68_sys.vh"                // system unit operation codes (SY_*), used by the pairing and gap checks
`default_nettype none
module p68_ag (
  input  wire        clk,            // CPU clock
  input  wire        rst,            // synchronous reset, active high
  input  wire        ds_go,          // DS writes a new entry into DR this clock
  input  wire [`UW-1:0] o_dec, o_seq,   // DS output in parts: the OR of all decoder uops and the OR of all sequencer uops
  input  wire        o_act,          // a sequencer is running, so o_seq counts and o_dec does not
  input  wire [`DVW-1:0] o_ov,       // the override fields (p68_dr.vh): register numbers, PC, length, step, prediction bits
  input  wire [`UW-1:0] o_u2,        // second instruction of a pair candidate, already a complete uop (decoder E1)
  input  wire        o_p2,           // DS offers o_u2 as a pair candidate
  input  wire        o_sw,           // swap candidate: the first instruction is of lane B's class, the second uses memory
  input  wire        ds_ill,         // illegal instruction (raises vector 4); kept out of the DS merge for timing
  input  wire        iwant,          // an unmasked interrupt is requested (a register): no pairs are formed, so every
                                     // instruction reaches EX with FIRST in lane A, where an interrupt can be taken
                                     // (without this a polling loop could run forever with the head always in lane B)
  input  wire        flush,          // global flush: everything in DR and AG is thrown away
  input  wire        rs_d,           // an EX redirect one clock ago (the front end flushes now)
  // ---- change/use interlock and forwarding: register writes of the older uops further down the pipe ----
  input  wire        oc_v, oc_wen,   // a uop is in OC1; it writes a register in WB
  input  wire [31:0] oc_w1_v, oc_w2_v,     // the values of the AG writes (An updates) of the uop in OC1, forwarded to the
                                           // early reads (AG decides those selects itself one clock earlier, pl*)
  input  wire [4:0]  oc_wreg,        // the WB destination of the uop in OC1
  input  wire        q_v, q_wen,     // the same for the uop in OC2
  input  wire        oc_s7, q_s7,    // a store through A7 in OC1 / OC2 (a predicted return's load waits for it)
  input  wire [4:0]  q_wreg,         // the WB destination of the uop in OC2
  // The EX inputs are EX's own register copies (ex_vg, ex_wug, ex_wrg, ex_cw, exb_wu, exb_wreg), so the interlock and
  // the forwarding selects do not load the high-fanout ex_v, ex_u and exb_v nets.
  input  wire        ex_vg,          // a uop is in EX (copy of ex_v)
  input  wire        ex_wug,         // the EX uop's WEN field (without the DBcc/CAS condition, so the interlock need not wait for it)
  input  wire [4:0]  ex_wrg,         // the WB destination of the uop in EX (copy of ex_wreg)
  input  wire        ex_rep,         // the EX uop replays (writes nothing)
  input  wire        ex_cw,          // the EX uop writes only by its condition (DBcc, CAS), a copy register in EX
  input  wire        wb_wa,          // WB lane A writes its register this clock (wb_wen)
  input  wire [31:0] wb_val,         // lane A's value being written in WB (forwarded into AG's held values)
  input  wire        stall_oc,       // OC1 holds a misaligned access for one more clock
  // ---- register file reads (one clock early, at the numbers of the uop that is in AG next) ----
  output wire [4:0]  rd_l, rd_s,     // LA base and SA base register numbers
  output wire [3:0]  rd_x,           // index register number (D0-D7, A0-A7: never a temporary)
  input  wire [31:0] rv_l, rv_s, rv_x,   // their values, valid after this clock (without AG's own writes)
  // ---- results ----
  output reg         ag_v,           // a uop is in AG
  output wire        stall_ag,       // the uop in AG stays (interlock, a split in OC, a flag gap)
  output wire        stall_dr,       // DR is full: DS must not advance (a register)
  output reg  [3:0]  sdr_q /* synthesis syn_preserve = 1 */,   // four register copies of stall_dr for DS's advance logic
  output wire        ag_go,          // the uop in AG moves to OC1 this clock
  output wire        ag_mv,          // ag_go without the redirect_ex term (register writes; the restore one clock later undoes them)
  output wire        bra_ag,         // AG redirects fetch to ag_tgt (an unconditional branch without a register)
  output wire        ag_pt,          // a branch the BTC predicted leaves AG (DS takes its target, ea_s)
  output wire [31:0] ea_s,           // the load address
  output wire [31:0] ag_tgt,         // branch target of a branch without a register (for bra_ag and ag_pt), a register
  input  wire        oc_bra,         // a branch through a register redirects from OC: the uop here is flushed
  output wire [31:0] saddr,          // the store address
  output wire [1:0]  sa_d,           // the low two bits of the store-only address (OC selects them itself when SA_SAME)
  output reg  [`UW-1:0] n_u,         // the uop to OC1 (operand A captured as an immediate when AG updates its register)
  // ---- lane B (dual issue). Lane B is register-only: AG passes it on and interlocks against it ----
  output reg  [`UW-1:0] n_ub,        // the lane B uop to OC1 (valid with n_ubv)
  output reg         n_ubv,          // lane B carries an instruction
  input  wire        ocb_wen, qb_wen, exb_wu,   // lane B register writes pending in OC1, OC2, EX (valid and WEN; EX's is a copy)
  input  wire [3:0]  ocb_wreg, qb_wreg, exb_wreg,   // ... their destinations (lane B never writes a temporary, so 4 bits)
  input  wire [31:0] wb_valb,        // lane B's value being written in WB
  output wire        w1_en,          // AG write 1 (the LA base update of (An)+ or -(An)), not yet gated by ag_go
  output wire [4:0]  w1_r,           // ... register number
  output wire [31:0] w1_v,           // ... new value
  output wire        w2_en,          // AG write 2 (the SA base update, or LEA)
  output wire [4:0]  w2_r,           // ... register number
  output wire [31:0] w2_v            // ... new value
) /* synthesis syn_maxfan = 16 */;
reg  [`UW-1:0] u;                    // the lane A uop in AG
reg            ill;                  // that uop is an illegal instruction (vector 4 is added on the way to OC)

// ---- DR queue ------------------------------------------------------------------------------------------------------
// Two entries, e0 and e1, each holding one DS output in parts. wp is the entry DS writes next, rp the entry AG takes
// next, dcnt the number of full entries (0, 1 or 2). Two entries are enough: DS writes one per clock, AG takes one per
// clock, and the second entry absorbs the one clock it takes DS to see that DR filled up (stall_dr is a register).
reg  [`UW-1:0] e0_dec, e0_seq, e1_dec, e1_seq;   // the decoder uop and the sequencer uop of each entry
reg  [`DVW-1:0] e0_ov, e1_ov;        // the override fields of each entry
reg            e0_act, e1_act, e0_ill, e1_ill;   // a sequence runs (o_seq counts); illegal instruction
reg  [`UW-1:0] e0_u2, e1_u2;         // the pair candidate's second instruction
reg            e0_p2, e1_p2, dh, e0_sw, e1_sw;   // pair candidate; swap candidate; dh: the entry's first instruction
                                     // left alone last clock and its second instruction goes now
reg  [1:0]     dcnt;                 // full entries
reg            rp, wp;               // read and write pointers (one bit: entry 0 or 1)
// drm: merge one DR entry into the uop AG works on. During a sequence the sequencer uop is complete except for the
// fields that depend on the instruction's place (PC, length, step and so on), which come from the override vector v.
// Outside a sequence the decoder uop is used, and its register numbers come from v as well, because DS extracts them
// from the opcode and extension words with its own small fast logic instead of ORing them through twenty decoders.
function [`UW-1:0] drm; input [`UW-1:0] dec, sq; input act; input [`DVW-1:0] v;
  begin
    drm = act ? sq : dec;                                     // which half of DS's output counts
    if (!act) begin                                           // decoder uop: take the register numbers from v
      drm[`F_LA_BREG] = v[`DV_LAB]; drm[`F_SA_BREG] = v[`DV_SAB]; drm[`F_LA_XREG] = {1'b0, v[`DV_LXR]};
      if (v[`DV_ADJ0]) drm[`F_SA_ADJ] = 5'd0;                 // full-format MOVE: T2 already holds the source's step
      drm[`F_PFIX] = v[`DV_PFIX];                             // fetch cut this instruction wrongly: OC will replay it
      if (v[`DV_PBTC]) begin drm[`F_PRED] = 1'b1; drm[`F_PBTC] = 1'b1; end   // predicted taken by the BTC
    end
    if (v[`DV_PRET]) drm[`F_PRET] = 1'b1;                     // predicted return (RTS and friends)
    if (v[`DV_ISEL]) drm[`F_IMM] = v[`DV_IMM];                // IMM = BTC target or predicted return address
    // the place of the uop in its instruction and in the program
    drm[`F_PC] = v[`DV_PC]; drm[`F_LEN] = v[`DV_LEN]; drm[`F_FIRST] = v[`DV_FIRST]; drm[`F_LAST] = v[`DV_LAST];
    drm[`F_STEP] = v[`DV_STEP]; drm[`U_FXP] = v[`DV_FXP]; drm[`U_FXM] = v[`DV_FXM];
  end
endfunction
wire [`UW-1:0] d_u1  = rp ? drm(e1_dec, e1_seq, e1_act, e1_ov) : drm(e0_dec, e0_seq, e0_act, e0_ov);   // first instruction of the entry at rp
wire [`UW-1:0] d_u2  = rp ? e1_u2 : e0_u2;   // its pair candidate (second instruction)
wire           d_p2  = rp ? e1_p2 : e0_p2;   // the entry has a pair candidate
wire           d_sw  = rp ? e1_sw : e0_sw;   // the entry is a swap candidate
wire           d_act = rp ? e1_act : e0_act; // the entry is a sequencer uop
wire           d_v   = (dcnt != 2'd0);       // DR holds an entry

// ---- Pairing (dual issue) -----------------------------------------------------------------------------------------
// The model is the 68060: lane A does everything, lane B only simple register instructions, and a pair has at most one
// memory operand. A plain pair needs a first instruction that is a single uop and cannot redirect, raise an exception
// or lock the bus, and a second instruction of lane B's class (a register-only ALU uop) that reads and writes no
// register the first one writes. The first instruction may store: OC replays the pair when the store lands near the
// second instruction's code, and after a replay the first instruction issues alone.
//
// i1ok: x may be the first (older, lane A) instruction of a plain pair.
function i1ok; input [`UW-1:0] x;
  // no unconditional branch, no Bcc, DBcc, system, multiply, divide, bit field or CAS uop (they redirect, raise, or
  // take several clocks), no exception, no bus lock, no prediction to check, no full-format pre-sequence, one uop only
  i1ok = !x[`F_BRA] && x[`F_UNIT] != `UN_BRA && x[`F_UNIT] != `UN_DBR && x[`F_UNIT] != `UN_SYS && x[`F_UNIT] != `UN_MUL &&
         x[`F_UNIT] != `UN_DIV && x[`F_UNIT] != `UN_BFU && x[`F_UNIT] != `UN_CAS && x[`F_EXC] == 8'h0 && !x[`U_LOCK] &&
         !x[`F_PFIX] && !x[`F_PRET] && !x[`U_FXP] && !x[`U_FXM] && x[`F_LAST] && x[`F_FIRST];
endfunction
// i2ok: x is of lane B's class. Lane B's ALU (p68_exb.v) has only an adder and the logic operations, no memory port,
// no AG writes, and only the "set" forms of the flags, so the check lists exactly what it can do.
function i2ok; input [`UW-1:0] x;
  reg [4:0] o;                                              // the ALU operation
  begin o = x[`F_OP];
    i2ok = x[`F_UNIT] == `UN_ALU && (o == `OP_ADD || o == `OP_SUB || o == `OP_AND || o == `OP_OR || o == `OP_EOR ||
           o == `OP_CMP || o == `OP_MOV || o == `OP_NEG) && !x[`F_LA_EN] && !x[`F_SA_EN] && !x[`F_LEA] && !x[`F_BRA] &&
           (x[`F_ASEL] == `AS_REG || x[`F_ASEL] == `AS_IMM) && x[`F_BSEL] == `BS_REG && x[`F_FZM] == `ZM_SET &&
           // register number bit 4 set means a temporary or a stack pointer bank (16-22): lane B touches only D0-A7
           (x[`F_FCV] == `CV_ADD || x[`F_FCV] == `CV_CLR) && !x[`U_WREG + 4] && !x[`U_BREG + 4] &&
           !(x[`F_ASEL] == `AS_REG && x[`U_AREG + 4]) && x[`F_EXC] == 8'h0;
  end
endfunction
// cf: register r is written by x, in WB or by its AG
function cf; input [4:0] r; input [`UW-1:0] x;
  cf = (x[`F_WEN] && x[`F_WREG] == r) || (x[`F_LA_UEN] && x[`F_LA_BREG] == r) ||
       ((x[`F_SA_UEN] || x[`F_LEA]) && x[`F_SA_BREG] == r);
endfunction
// Swapped pair. A memory instruction behind a simple head: the second instruction (memory, from the second decoder)
// runs in lane A, the first (lane B's class) in lane B, marked as the older one (U_BOLD). Lane A neither reads nor
// writes the register lane B writes, and lane A's AG updates no register lane B reads. Lane A reads no flags (lane B's
// flags are completed in WB after it, and EX masks lane B's flag writes with lane A's). Lane A takes no interrupt in
// front of it (its FIRST bit is cleared), because lane B would then have to commit alone with its flags.
//
// amok: x may be the memory instruction of a swapped pair: one of lane B's ALU operations with a memory operand,
// without an index or a PC base (so its address needs no index forwarding and no PC).
function amok; input [`UW-1:0] x;
  reg [4:0] o;                                              // the ALU operation
  begin o = x[`F_OP];
    amok = x[`F_UNIT] == `UN_ALU && (o == `OP_ADD || o == `OP_SUB || o == `OP_AND || o == `OP_OR || o == `OP_EOR ||
           o == `OP_CMP || o == `OP_MOV || o == `OP_NEG) && (x[`F_LA_EN] || x[`F_SA_EN]) && !x[`F_LEA] && !x[`F_BRA] &&
           !x[`F_LA_XEN] && x[`F_LA_BSEL] != `BA_PC && x[`F_FZM] == `ZM_SET && (x[`F_FCV] == `CV_ADD || x[`F_FCV] == `CV_CLR) &&
           !x[`U_WREG + 4] && !x[`U_BREG + 4] && !(x[`F_ASEL] == `AS_REG && x[`U_AREG + 4]) && x[`F_EXC] == 8'h0;
  end
endfunction
// rdx: register r is read by x (as an AG base or as an EX operand)
function rdx; input [4:0] r; input [`UW-1:0] x;
  rdx = (x[`F_LA_BSEL] == `BA_REG && x[`F_LA_BREG] == r) || (x[`F_SA_BSEL] == `BA_REG && x[`F_SA_BREG] == r) ||
        (x[`F_ASEL] == `AS_REG && x[`F_AREG] == r) || (x[`F_BSEL] == `BS_REG && x[`F_BREG] == r);
endfunction
// agw: register r is updated by x's AG ((An)+, -(An), LEA)
function agw; input [4:0] r; input [`UW-1:0] x;
  agw = (x[`F_LA_UEN] && x[`F_LA_BREG] == r) || ((x[`F_SA_UEN] || x[`F_LEA]) && x[`F_SA_BREG] == r);
endfunction
// Predicted branch in lane B. A BTC-predicted Bcc, BRA or DBcc as the second instruction runs in lane B. EX checks its
// condition with the flags of the uops in WB, so lane A may write no flag the branch reads, and lane A must not write a
// DBcc's counter. Its target is computed here (tb_b) and compared with the predicted one; a wrong target sets lane B's
// PFIX bit and OC replays.
function i2br; input [`UW-1:0] x;
  // CC = 0 is "true": a BRA is coded as an unconditional AG branch with condition T
  i2br = x[`F_PBTC] && x[`F_EXC] == 8'h0 &&
         (x[`F_UNIT] == `UN_BRA || x[`F_UNIT] == `UN_DBR || (x[`F_BRA] && x[`F_UNIT] == 4'd0 && x[`F_CC] == 4'h0));
endfunction
// Branch fold pair. The first instruction is a BTC-predicted Bcc, BRA, DBcc or BSR (lane A), the second the first
// instruction at its target (lane B, younger, register-only). Lane B reads and writes no register lane A writes (a
// DBcc counter, BSR's A7), and EX drops lane B when lane A redirects because the prediction was wrong. On the bench
// this took the integer loop from CPI 0.759 to 0.625, because a predicted branch no longer costs a clock of its own.
//
// hdbr: x may be the head (the branch) of a fold pair. SY_NPC with BRA is BSR (it writes the return address).
function hdbr; input [`UW-1:0] x;
  hdbr = x[`F_PBTC] && !x[`F_PFIX] && x[`F_EXC] == 8'h0 && x[`F_FIRST] && x[`F_LAST] && !x[`F_PRET] && !x[`U_FXP] &&
         !x[`U_FXM] && !x[`U_LOCK] && (x[`F_UNIT] == `UN_BRA || x[`F_UNIT] == `UN_DBR || (x[`F_BRA] && x[`F_UNIT] == 4'd0) ||
         (x[`F_BRA] && x[`F_UNIT] == `UN_SYS && x[`F_OP] == `SY_NPC));
endfunction
// Skip pair. A forward Bcc.B that jumps over exactly the next instruction (and was not predicted) runs in lane A with
// that instruction in lane B, and there is no redirect either way. When the branch is taken, EX drops lane B: lane B
// then writes its destination back unchanged, so the forwarding selects that already point at lane B's result stay
// valid; that is why lane B's destination must be its own B operand. Such a branch never enters the branch target
// cache (its F_BR bit is cleared). This one change took the bench from CPI 1.325 to 1.091, and the call-heavy test
// from 2.090 to 1.446, because short "skip one instruction" branches are everywhere in compiled 68k code.
//
// skb: x is such a branch, when the next instruction is l2 words long.
function skb; input [`UW-1:0] x; input [3:0] l2;
  // the target is PC + 2 (the XK offset of a Bcc) + displacement; it must equal the end of the next instruction,
  // i.e. displacement = 2 * (LEN - 1 + l2) bytes from the extension point
  skb = x[`F_UNIT] == `UN_BRA && !x[`F_PBTC] && !x[`F_PRED] && !x[`F_PFIX] && x[`F_EXC] == 8'h0 && x[`F_FIRST] &&
        x[`F_LAST] && !x[`U_FXP] && !x[`U_FXM] && x[`F_LA_BSEL] == `BA_PC && !x[`F_LA_XEN] && x[`F_LA_XK] == 4'd2 &&
        x[`F_LA_DISP] == {27'h0, x[`F_LEN] - 4'd1 + l2, 1'b0};     // target = the end of the next instruction
endfunction
// Rename. MOVE.L Rm,Rx as the first instruction: the second reads Rm where it reads Rx (a trick of the 68060), so the
// pair can go together even though the second instruction depends on the first. Not when the second writes Rx.
//
// pairq decides the pairing of one DR entry from its registers alone. A pair needs no sequencer uop, so the first
// instruction is simply the decoder uop with the override fields applied. Both entries are decided in parallel, and
// the read pointer selects between the two answers last, so the pointer is not in front of this deep logic.
function [6:0] pairq;                 // {rnb, rna, psk, pfb, pbr, pswp, pok} without the iwant / dh terms
  input [`UW-1:0] x1, x2; input p2, sw, act, il;    // first and second instruction, pair hint, swap hint, sequence, illegal
  reg mv1, rna, rnb, ok;
  begin
    ok  = p2 && !act && !il;                                  // a candidate at all: DS offered it, no sequence, legal
    // mv1: x1 is MOVE.L Rm,Rx between D0-A7 with no memory and no AG write (the rename case)
    mv1 = x1[`F_UNIT] == `UN_ALU && x1[`F_OP] == `OP_MOV && x1[`F_SZ] == `SZ_L && x1[`F_ASEL] == `AS_REG &&
          x1[`F_AEXT] == `AX_NONE && x1[`F_WEN] && !x1[`U_AREG + 4] && !x1[`U_WREG + 4] &&
          !x1[`F_LA_EN] && !x1[`F_SA_EN] && !x1[`F_LEA] && !x1[`F_LA_UEN] && !x1[`F_SA_UEN];
    rna = mv1 && x2[`F_ASEL] == `AS_REG && x2[`F_AREG] == x1[`F_WREG];   // x2's operand A is Rx: read Rm instead
    rnb = mv1 && x2[`F_BREG] == x1[`F_WREG];                           // x2's operand B is Rx: read Rm instead
    // [0] plain pair: x2 reads nothing x1 writes (unless renamed) and does not write what x1 writes
    pairq[0] = ok && i1ok(x1) && i2ok(x2) &&
               !(x2[`F_ASEL] == `AS_REG && cf(x2[`F_AREG], x1) && !rna) && !(cf(x2[`F_BREG], x1) && !rnb) &&
               !(x2[`F_WEN] && cf(x2[`F_WREG], x1));
    // [1] swapped pair: x1 (lane B's class) goes to lane B, x2 (memory) to lane A; no register crosses between them
    pairq[1] = ok && sw && i2ok(x1) && !x1[`F_PFIX] &&
               !x1[`F_PRET] && !x1[`U_FXP] && !x1[`U_FXM] && x1[`F_FIRST] && x1[`F_LAST] && amok(x2) &&
               !(x1[`F_WEN] && (rdx(x1[`F_WREG], x2) || cf(x1[`F_WREG], x2))) &&
               !(x1[`F_ASEL] == `AS_REG && agw(x1[`F_AREG], x2)) && !agw(x1[`F_BREG], x2);
    // [2] predicted branch in lane B: x1 writes no flag (or the branch is BRA/never, CC 0 or 1, which reads none)
    pairq[2] = ok && !sw && i1ok(x1) && i2br(x2) &&
               (x1[`F_FEN] == 5'd0 || x2[`F_CC] == 4'h0 || x2[`F_CC] == 4'h1) &&
               !(x2[`F_UNIT] == `UN_DBR && cf(x2[`F_BREG], x1));
    // [3] fold pair: x1 is the predicted branch, x2 its first target instruction, independent of x1's writes
    pairq[3] = ok && !sw && hdbr(x1) && i2ok(x2) &&
               !(x2[`F_ASEL] == `AS_REG && cf(x2[`F_AREG], x1)) && !cf(x2[`F_BREG], x1) &&
               !(x2[`F_WEN] && cf(x2[`F_WREG], x1));
    // [4] skip pair: x1 skips exactly x2, and x2 writes nothing or writes its own B operand
    pairq[4] = ok && !sw && skb(x1, x2[`F_LEN]) && i2ok(x2) && (!x2[`F_WEN] || x2[`F_WREG] == x2[`F_BREG]);
    pairq[5] = rna; pairq[6] = rnb;                           // the rename selects for lane B's operands
  end
endfunction
wire [6:0]     pq0   = pairq(drm(e0_dec, {`UW{1'b0}}, 1'b0, e0_ov), e0_u2, e0_p2, e0_sw, e0_act, e0_ill);   // entry 0's decision
wire [6:0]     pq1   = pairq(drm(e1_dec, {`UW{1'b0}}, 1'b0, e1_ov), e1_u2, e1_p2, e1_sw, e1_act, e1_ill);   // entry 1's decision
wire           pgo   = !iwant && !dh;      // pairs allowed: no interrupt waiting, and not the second half of a split entry
wire [6:0]     pq    = rp ? pq1 : pq0;     // the decision for the entry at the read pointer (the pointer selects last)
wire           pok   = pgo && pq[0];       // plain pair
wire           pswp  = pgo && pq[1];       // swapped pair
wire           pbr   = pgo && pq[2];       // predicted branch in lane B
wire           pfb   = pgo && pq[3];       // fold pair
wire           psk   = pgo && pq[4];       // skip pair
wire           rna   = pq[5], rnb = pq[6]; // rename lane B's operand A / B
// the target of a predicted branch in lane B: PC + XK + displacement, compared with the BTC target in IMM below
wire [31:0]    tb_b  = d_u2[`F_PC] + {28'h0, d_u2[`F_LA_XK]} + d_u2[`F_LA_DISP];
// Split entry. A candidate that did not pair goes in two clocks: the first instruction alone now, the second next
// clock from the same entry (dh). Formed per entry and selected by the read pointer last, like the pairing.
wire           dsp0  = e0_p2 && !(!iwant && |pq0[4:0]), dsp1 = e1_p2 && !(!iwant && |pq1[4:0]);
wire           dsplit = !dh && (rp ? dsp1 : dsp0);   // the entry stays in DR for its second instruction
reg  [`UW-1:0] d_u, d_ub;                            // the lane A and lane B uops that enter AG next
// The uops that leave DR: which instruction goes to which lane, and the small changes each kind of pair needs.
always @* begin
  d_u = (dh || pswp) ? d_u2 : d_u1;                    // lane A: the second instruction for a split's second half or a swap
  if (pswp) d_u[`F_FIRST] = 1'b0;                      // swap: no interrupt may be taken in front of lane A's uop
  if (psk) begin d_u[`U_PSK] = 1'b1; d_u[`F_BR] = 1'b0; end      // skip pair: mark it, and keep it out of the BTC
  d_ub = pswp ? d_u1 : d_u2;                           // lane B: the first instruction in a swap, else the second
  d_ub[`U_BOLD] = pswp;                                // lane B is the older instruction in a swap
  if (!pswp && rna) d_ub[`F_AREG] = d_u1[`F_AREG];      // rename: read Rm where the second instruction reads Rx
  if (!pswp && rnb) d_ub[`F_BREG] = d_u1[`F_AREG];
end
wire           d_ill = (dh || pswp) ? 1'b0 : (rp ? e1_ill : e0_ill);   // only a first instruction can be illegal
wire           d_ubv = pok || pswp || pbr || pfb || psk;               // lane B carries an instruction
// The AG register reads (one clock early) use the second instruction's numbers for a swap candidate whether or not it
// swaps. That is safe because a first instruction of lane B's class uses no AG register at all, and it keeps the late
// swap decision out of the register read address.
wire [`UW-1:0] d_ur  = (dh || d_sw) ? d_u2 : d_u1;

reg            fired;                // this uop's AG redirect was sent while it stalled (it must not redirect again)

// ---- Flag gaps ----------------------------------------------------------------------------------------------------
// The flags (the condition codes X N Z V C of the 68k status register) are completed in WB, one clock after the result
// (p68_ex.v explains why). Most flag readers get them forwarded, but two cases cannot be: a uop whose Z comes from a
// shift or a p68_exi result, followed directly by a flag reader, and a uop that writes X, followed directly by an X
// reader (ADDX, SUBX, NEGX, the BCD instructions and ROXL/ROXR use X as a carry input). AG holds the reader for one
// clock when such a uop is in OC1. OC1, OC2 and EX never close a gap between two uops, so a gap opened here stays.
// The classes are registers (of the uop that leaves DR, and of the uop moved to OC1), so the hold is one AND of
// registers.
//
// f_isad: ALU operation o uses the adder (ADD, SUB, CMP and the NEG/NEGX/ADDX/SUBX group), whose Z is formed early.
function f_isad; input [4:0] o; f_isad = o[4] ? (o[3:2] == 2'b00) : (!o[3] && (o[2:0] == 3'd0 || o[2:0] == 3'd1 || o[2:0] == 3'd4)); endfunction
// f_unit: x runs in one of the long units, whose OP field means something else
function f_unit; input [`UW-1:0] x; f_unit = (x[`F_UNIT] == `UN_SYS) || (x[`F_UNIT] == `UN_BFU) || (x[`F_UNIT] == `UN_MUL) || (x[`F_UNIT] == `UN_DIV); endfunction
function f_rdf; input [`UW-1:0] x;      // reads the CCR in EX (a condition, or a unit that keeps or reads CCR bits)
  f_rdf = f_unit(x) || (x[`F_UNIT] == `UN_BRA) || (x[`F_UNIT] == `UN_DBR) || (x[`F_UNIT] == `UN_CAS) ||
          (x[`F_OP] == `OP_SCC && x[`F_UNIT] == `UN_ALU);
endfunction
function f_rdx; input [`UW-1:0] x;      // reads X from the CCR in EX
  reg [4:0] o;
  begin o = x[`F_OP];
    // the extended-precision adder group, the BCD steps, and the ROX rotates (shifter ops 8-15 with bits 1 and 0 set)
    f_rdx = !f_unit(x) && (o == `OP_NEGX || o == `OP_ADDX || o == `OP_SUBX || o == `OP_ABCD || o == `OP_SBCD ||
                           o == `OP_NBCD || (!o[4] && o[3] && o[1] && o[0]));
  end
endfunction
function f_slz; input [`UW-1:0] x;      // writes Z from the result in WB (shifts, p68_exi): not formed early
  f_slz = x[`U_FEN + 2] && !x[`U_FZM + 1] && !f_unit(x) && !f_isad(x[`F_OP]) && (x[`U_OP + 3] || x[`U_OP + 4]);
endfunction
function f_wrx; input [`UW-1:0] x; f_wrx = x[`U_FEN + 4] || (x[`F_UNIT] == `UN_SYS); endfunction   // writes X (a system uop may write all of SR)
reg            a_rdf, a_rdx, o_slz, o_wrx;   // the uop in AG reads flags / X; the uop in OC1 writes a late Z / X
// Uops whose EX inputs take no WB forwarding: shifts (count and operand registers), bit fields and the p68_exi
// operations, and CHK, CHK2 and TRAPcc (operands and the condition). Leaving the WB forward out of those EX inputs
// shortened paths in EX; in exchange AG keeps such a uop two behind the uop that writes its registers (for the system
// ones, two behind any uop), which costs a clock only in the rare case where they really follow each other.
function f_gsh; input [`UW-1:0] x; f_gsh = (!f_unit(x) && (x[`F_OP] >= 5'd8) && (x[`F_OP] <= 5'd15)) || (x[`F_UNIT] == `UN_BFU) ||
                                               (!f_unit(x) && x[`F_OP] >= 5'd20); endfunction   // shifter ops, bit fields, p68_exi ops
function f_gsy; input [`UW-1:0] x; f_gsy = (x[`F_UNIT] == `UN_SYS) && (x[`F_OP] == `SY_CHK || x[`F_OP] == `SY_C2U || x[`F_OP] == `SY_TRAPCC); endfunction
reg            a_gsh, a_gsy, a_rts;  // the uop in AG is of the f_gsh / f_gsy class; it is a predicted return's load
// A predicted return's load (it reads the return address from the stack) waits in AG while a store through A7 is in
// OC1 or OC2, so it meets that store in WB, where it is forwarded, instead of missing it and replaying.
reg            fl, fs, fx;           // WB forwarding selects for the LA base, SA base and index, decided one clock early
reg            flb, fsb, fxb;        // the same from lane B's WB value

wire [4:0]  lbr  = u[`F_LA_BREG];    // the LA base register of the uop in AG
wire [4:0]  sbr  = u[`F_SA_BREG];    // the SA base register
wire [3:0]  xrn  = u[`F_LA_XREG];    // the index register (D0-A7)

// ---- Base and index values: read one clock early, then forwarded --------------------------------------------------
// The base and index registers are read one clock early at DR's register numbers into rl_r, rs_r and rx_r. The
// register file gives the value as it will be after this clock, but without the AG writes (those enter the register
// file one clock later from OC's registers). The AG writes of the uop that left AG last clock (now in OC1) are
// therefore forwarded from OC's registers; that uop is the only one between the read and its use, and the interlock
// keeps older WB writes of these registers out of the way. While AG stalls, the registers hold and take the forwarded
// values (an_l, an_s, xr), so a held uop sees writes that happen while it waits.
// The register numbers come from their own selects on hand copies of the DR pointer and swap flags: selecting them out
// of the full d_ur uop had loaded those pointer nets with all 271 bits.
reg            rpc /* synthesis syn_preserve = 1 */;   // copy of rp for the register number select
reg            dhc /* synthesis syn_preserve = 1 */;   // copy of dh
reg            e0_swc /* synthesis syn_preserve = 1 */;   // copy of e0_sw
reg            e1_swc /* synthesis syn_preserve = 1 */;   // copy of e1_sw
// drn: just the three register numbers drm would produce, {LA base, SA base, index}
function [13:0] drn; input [`UW-1:0] sq; input act; input [`DVW-1:0] v;     // {LA base, SA base, index} of drm
  drn = act ? {sq[`F_LA_BREG], sq[`F_SA_BREG], sq[(`U_LA + `LO_XREG) +: 4]} : {v[`DV_LAB], v[`DV_SAB], v[`DV_LXR]};
endfunction
reg  [13:0] e0_rn, e1_rn;            // drn of each entry, registered when DS writes the entry
wire [13:0] rn1 = rpc ? e1_rn : e0_rn;                                 // numbers of the first instruction
wire [`UW-1:0] u2c = rpc ? e1_u2 : e0_u2;                              // the second instruction (pointer copy)
wire [13:0] rn2 = {u2c[`F_LA_BREG], u2c[`F_SA_BREG], u2c[(`U_LA + `LO_XREG) +: 4]};   // its numbers
wire [13:0] rnn = (dhc || (rpc ? e1_swc : e0_swc)) ? rn2 : rn1;        // the same choice as d_ur
wire [4:0]  dl  = rnn[13:9];                                           // DR's LA base number
wire [4:0]  dsb = rnn[8:4];                                            // DR's SA base number
wire [3:0]  dx  = rnn[3:0];                                            // DR's index number
assign rd_l = dl; assign rd_s = dsb; assign rd_x = dx;                 // to the register file's AG read ports
reg  [31:0] rl_r, rs_r, rx_r;        // the held base and index values
// The selects of that forwarding are registers (pl*, ps*, px*), decided when the uop enters AG from the AG writes of
// the uop that moves to OC1 in the same clock. They are cleared while AG holds, because the held value has the write.
reg         pl1, pl2, ps1, ps2, px1, px2;   // forward OC's AG write 1 / 2 into the LA base, SA base, index
// The selects are made one-hot when they are registered (n*_r: none of the others, take the held value), so each value
// is a plain AND-OR of five terms: two LUT levels for 32 bits, with no priority chain.
reg         nl_r, ns_r, nx_r;
wire [31:0] an_l = ({32{fl}} & wb_val) | ({32{flb}} & wb_valb) | ({32{pl2}} & oc_w2_v) | ({32{pl1}} & oc_w1_v) | ({32{nl_r}} & rl_r);   // LA base value
wire [31:0] an_s = ({32{fs}} & wb_val) | ({32{fsb}} & wb_valb) | ({32{ps2}} & oc_w2_v) | ({32{ps1}} & oc_w1_v) | ({32{ns_r}} & rs_r);   // SA base value
wire [31:0] xr   = ({32{fx}} & wb_val) | ({32{fxb}} & wb_valb) | ({32{px2}} & oc_w2_v) | ({32{px1}} & oc_w1_v) | ({32{nx_r}} & rx_r);   // index value
// For the held registers, the late register file values (rv_*) enter only the last LUT; the held values are three
// AND-OR parts kept apart by syn_keep (p: from WB, q: from OC's AG writes, r: the held value itself).
wire [31:0] hl_p /* synthesis syn_keep = 1 */, hl_q /* synthesis syn_keep = 1 */, hl_r /* synthesis syn_keep = 1 */;
wire [31:0] hs_p /* synthesis syn_keep = 1 */, hs_q /* synthesis syn_keep = 1 */, hs_r /* synthesis syn_keep = 1 */;
wire [31:0] hx_p /* synthesis syn_keep = 1 */, hx_q /* synthesis syn_keep = 1 */, hx_r /* synthesis syn_keep = 1 */;
// The WB forward selects do not wait for the condition of a DBcc or CAS (whether such a uop writes at all is only known
// late in EX). When such a uop is in WB and did not write (wcw without wb_wa), the held register value is kept instead.
// AG holds the reader in that clock anyway (see pends), so the address computed from the wrong value is never used.
reg         wcw;                     // the uop now in WB is a conditional writer (ex_cw one clock ago)
wire        wkl = wcw && !wb_wa;     // ... and it did not write: keep the held value
assign hl_p = ({32{fl && !wkl}} & wb_val) | ({32{flb}} & wb_valb); assign hl_q = ({32{pl2}} & oc_w2_v) | ({32{pl1}} & oc_w1_v); assign hl_r = {32{nl_r || (fl && wkl)}} & rl_r;
assign hs_p = ({32{fs && !wkl}} & wb_val) | ({32{fsb}} & wb_valb); assign hs_q = ({32{ps2}} & oc_w2_v) | ({32{ps1}} & oc_w1_v); assign hs_r = {32{ns_r || (fs && wkl)}} & rs_r;
assign hx_p = ({32{fx && !wkl}} & wb_val) | ({32{fxb}} & wb_valb); assign hx_q = ({32{px2}} & oc_w2_v) | ({32{px1}} & oc_w1_v); assign hx_r = {32{nx_r || (fx && wkl)}} & rx_r;
// The held registers: while AG stalls (sa_r) they keep their own forwarded value, otherwise they load the register
// file's read of the next uop's registers.
always @(posedge clk) begin
  rl_r <= sa_r ? (hl_p | hl_q | hl_r) : rv_l;
  rs_r <= sa_r ? (hs_p | hs_q | hs_r) : rv_s;
  rx_r <= sa_r ? (hx_p | hx_q | hx_r) : rv_x;
end

// ---- The index: sign extension and scale, done per forwarding part -------------------------------------------------
// The index is scaled per forwarding part. The WB part (late, because wb_val is computed in the same clock in WB)
// passes an AND-OR of one-hot scale selects (kept apart, from the uop register) and meets the other parts in the
// compressor's LUT. Before this, wb_val passed the forward AND-OR, the sign extension, the scale select and the XEN
// select one after the other, and that chain into the load address and on into the D-cache index was the worst real
// path of the full system.
function [31:0] xsc;                 // sign extension and scale of an index value
  input [31:0] v; input xl; input [1:0] sc;    // value; long index (else word, sign-extended); scale code 0-3 = *1, *2, *4, *8
  reg [31:0] w;
  begin w = xl ? v : {{16{v[15]}}, v[15:0]};   // Xn.W uses the low word, sign-extended, as on every 68k
    case (sc) 2'd0: xsc = w; 2'd1: xsc = {w[30:0], 1'b0}; 2'd2: xsc = {w[29:0], 2'b0}; default: xsc = {w[28:0], 3'b0}; endcase   // shift left by 0-3
  end
endfunction
// One-hot scale selects from the uop register: xkl[k] = index used, scale 2^k, long; xkw[k] the same for a word index.
wire [3:0]  xkl /* synthesis syn_keep = 1 */, xkw /* synthesis syn_keep = 1 */;   // XEN, scale k, long / word index
genvar gk, gb;
generate for (gk = 0; gk < 4; gk = gk + 1) begin : xks
  assign xkl[gk] = u[`F_LA_XEN] && u[`F_LA_XSC] == gk && u[`F_LA_XL];
  assign xkw[gk] = u[`F_LA_XEN] && u[`F_LA_XSC] == gk && !u[`F_LA_XL];
end endgenerate
// The WB forward selects (fx, fxb) are folded into the scale selects: each of these is a register AND a kept uop bit,
// so wb_val meets its select in the first LUT of the scale AND-OR. A reader of a conditional writer in WB that did not
// write (wkl) is held in AG (pends), so its address is never used and wkl need not appear here.
wire [3:0]  xal /* synthesis syn_keep = 1 */, xaw /* synthesis syn_keep = 1 */;   // lane A's WB value, long / word, scale k
wire [3:0]  xbl /* synthesis syn_keep = 1 */, xbw /* synthesis syn_keep = 1 */;   // lane B's WB value, long / word, scale k
assign xal = {4{fx}} & xkl; assign xaw = {4{fx}} & xkw; assign xbl = {4{fxb}} & xkl; assign xbw = {4{fxb}} & xkw;
wire [31:0] xi_p;                    // the WB part of the scaled index
// Bit gb of the scaled index is bit gb - k of the value for scale 2^k. For a word index, bits 16 and up of the value
// are the sign (bit 15). Each bit is an OR of at most four terms of four inputs, which fits a LUT4 tree of two levels.
generate for (gb = 0; gb < 32; gb = gb + 1) begin : xip
  wire [3:0] t;                      // the term for each scale
  for (gk = 0; gk < 4; gk = gk + 1) begin : xt
    if (gb < gk) assign t[gk] = 1'b0;                     // the low bits shifted in are zero
    else if (gb - gk < 16) assign t[gk] = ((xal[gk] | xaw[gk]) & wb_val[gb - gk]) | ((xbl[gk] | xbw[gk]) & wb_valb[gb - gk]);   // the low word: same for long and word
    else assign t[gk] = (xal[gk] & wb_val[gb - gk]) | (xaw[gk] & wb_val[15]) | (xbl[gk] & wb_valb[gb - gk]) | (xbw[gk] & wb_valb[15]);   // high part: the bit, or the sign of the word
  end
  assign xi_p[gb] = |t;
end endgenerate
// The other parts (OC's AG writes and the held value) go through the plain xsc function. Without an index, the index
// input carries the small constant XK instead (the PC offset of the extension word in PC-relative modes).
wire [31:0] xi_q /* synthesis syn_keep = 1 */;
assign xi_q = u[`F_LA_XEN] ? xsc(hx_q | hx_r, u[`F_LA_XL], u[`F_LA_XSC]) : {28'h0, u[`F_LA_XK]};

// ---- Load address -------------------------------------------------------------------------------------------------
// The base select is folded into the forwarding selects (all registers or uop bits), so e_base is an OR of three kept
// parts and the WB value passes one LUT before the compressor. Before, it passed hl_p, an_l and the base select one
// after the other, and that was the worst real path into the D-cache index.
wire        bsr = (u[`F_LA_BSEL] == `BA_REG);                 // the base is a register
wire        fl_g = fl && bsr, flb_g = flb && bsr, pl1_g = pl1 && bsr, pl2_g = pl2 && bsr, nl_g = nl_r && bsr;   // selects gated by it
wire [31:0] pc_b = (u[`F_LA_BSEL] == `BA_PC) ? u[`F_PC] : 32'h0;   // the PC as base (PC-relative modes); zero for absolute
wire [31:0] hb_p /* synthesis syn_keep = 1 */, hb_q /* synthesis syn_keep = 1 */, hb_r /* synthesis syn_keep = 1 */;
assign hb_p = ({32{fl_g}} & wb_val) | ({32{flb_g}} & wb_valb);   // the late WB part
assign hb_q = ({32{pl2_g}} & oc_w2_v) | ({32{pl1_g}} & oc_w1_v); // OC's AG writes
assign hb_r = ({32{nl_g}} & rl_r) | pc_b;                        // the held register value, or the PC
wire [31:0] e_base = hb_p | hb_q | hb_r;                         // base
wire [31:0] e_idx  = xi_p | xi_q;                                // index * scale (or XK)
wire [31:0] e_disp = u[`F_LA_DISP];                              // displacement (already sign-extended by DS)
// 3:2 compressor: per bit, the sum bit of a full adder (XOR of three) and the carry bit (majority of three). The sum
// of the three inputs equals cs_s + 2 * cs_c, so a single carry chain finishes the job. Gowin maps each pair of bits to
// one LUT level in front of the ALU carry chain.
wire [31:0] cs_s = e_base ^ e_idx ^ e_disp;                                   // 3:2 compressor, one carry chain
wire [31:0] cs_c = (e_base & e_idx) | (e_base & e_disp) | (e_idx & e_disp);
assign      ea_s = cs_s + {cs_c[30:0], 1'b0};                                 // the load address (carries shifted up one bit)
// The load base update of (An)+ and -(An): base + UDL (+1/2/4 or -1/2/4; A7 steps by 2 for a byte, which DS has
// already put into UDL). The update uses an_l, the base without the index and displacement.
wire [31:0] upd_l = an_l + {{24{u[`U_LA + `LO_UDL + 7]}}, u[`F_LA_UDL]};

// ---- Branch target for the AG redirect ----------------------------------------------------------------------------
// Branches whose target needs no register (base PC or zero, constant index: Bcc, BRA, DBcc, BSR, JMP and JSR to an
// absolute address) redirect from AG with their own adder on uop fields only, with no forwarding select and no index.
// Branches through a register redirect from OC one clock later (oc_bra in p68_oc.v, with the target in oc_addr).
// The target is computed from DR's uop while it is still in DR and registered (tg_r), so the AG redirect sends a plain
// register to fetch. That matters because the redirect fans out into the whole front end.
wire        bfast = (u[`F_LA_BSEL] != `BA_REG) && !u[`F_LA_XEN];   // the uop in AG has a register-free target
wire [31:0] tf_b  = (d_ur[`F_LA_BSEL] == `BA_PC) ? d_ur[`F_PC] : 32'h0;     // base of DR's uop (d_ur is fine: no branch is ever in a swap)
wire [31:0] tf_k  = {28'h0, d_ur[`F_LA_XK]};                       // the constant index (the PC offset)
wire [31:0] tf_s  = tf_b ^ tf_k ^ d_ur[`F_LA_DISP];                // the same 3:2 compressor as above
wire [31:0] tf_c  = (tf_b & tf_k) | (tf_b & d_ur[`F_LA_DISP]) | (tf_k & d_ur[`F_LA_DISP]);
reg  [31:0] tg_r;                                                  // the target of the uop in AG
always @(posedge clk) if (!stall_ag) tg_r <= tf_s + {tf_c[30:0], 1'b0};   // loads together with u
assign      ag_tgt = tg_r;

// ---- Store address and store base update --------------------------------------------------------------------------
// The store side has no index (see the SA descriptor in p68_uop.vh). Its address is base + ADJ + displacement, where ADJ
// is the source's step when both EAs use the same address register (MOVE.L (A0)+,(A0) stores at the stepped A0). The
// base update of -(An) on the store side is base + ADJ + UDL. Both use the same compressor-and-carry-chain shape.
wire [31:0] s_base = (u[`F_SA_BSEL] == `BA_REG) ? an_s : 32'h0;   // the store base, or zero for an absolute address
wire [31:0] s_adj  = {{27{u[`U_SA + `SO_ADJ + 4]}}, u[`F_SA_ADJ]};   // ADJ, sign-extended from 5 bits
wire [31:0] s_udl  = {{24{u[`U_SA + `SO_UDL + 7]}}, u[`F_SA_UDL]};   // UDL, sign-extended from 8 bits
wire [31:0] s_disp = u[`F_SA_DISP];                                 // the store displacement
wire [31:0] ds_s   = s_base ^ s_adj ^ s_disp;                       // compressor for the store address
wire [31:0] ds_c   = (s_base & s_adj) | (s_base & s_disp) | (s_adj & s_disp);
wire [31:0] addr_d = ds_s + {ds_c[30:0], 1'b0};                     // the store address when it is not the load address
wire [31:0] us_s   = s_base ^ s_adj ^ s_udl;                        // compressor for the store base update
wire [31:0] us_c   = (s_base & s_adj) | (s_base & s_udl) | (s_adj & s_udl);
wire [31:0] upd_d  = us_s + {us_c[30:0], 1'b0};                     // the updated store base
assign      saddr  = u[`F_SA_SAME] ? ea_s : addr_d;                 // read-modify-write stores at the load address
// OC's split decode needs the low two address bits early; it selects between ea_s and addr_d by SA_SAME itself, so the
// SA_SAME select is not in front of that decode.
assign      sa_d   = addr_d[1:0];

// ---- The two AG register writes -----------------------------------------------------------------------------------
// They are passed to OC1 and enter the register file from OC's registers one clock later (gated there by ag_go).
assign w1_en = u[`F_LA_UEN];                  // write 1: the LA base update
assign w1_r  = lbr;
assign w1_v  = upd_l;
assign w2_en = u[`F_SA_UEN] | u[`F_LEA];      // write 2: the SA base update, or LEA
assign w2_r  = sbr;
assign w2_v  = u[`F_LEA] ? ea_s : upd_d;      // LEA writes the load address into the SA base register

// ---- The uop for OC1 ----------------------------------------------------------------------------------------------
// An A or T register read as operand A that this uop's AG also updates must be read with its old value (the 68k reads
// the operand before the EA's step takes effect, e.g. in MOVE.L A0,(A0)+ the stored value is the old A0). Since AG's
// update reaches the register file before OC2 reads the operand, AG captures the old value as an immediate here.
always @* begin
  n_u = u;
  if (ill) n_u[`U_EXC + 2] = 1'b1;                                 // illegal instruction: vector 4 (EXC bit 2)
  if (u[`F_ASEL] == `AS_REG && u[`U_AREG + 4 -: 2] != 2'b00) begin  // operand A is an An (8-15) or a T register (16-19)
    if (w2_en && u[`F_AREG] == sbr)      begin n_u[`F_ASEL] = `AS_IMM; n_u[`F_IMM] = an_s; end
    else if (w1_en && u[`F_AREG] == lbr) begin n_u[`F_ASEL] = `AS_IMM; n_u[`F_IMM] = an_l; end
  end
end

// ---- Change/use interlock -----------------------------------------------------------------------------------------
// AG reads or writes a register that an older uop in OC1, OC2 or EX will still write in WB (WB itself is forwarded).
// The interlock is decided one clock early (hz_r) for the uop that is in AG next (u when AG holds, else d_u) against
// the register writes pending then: OC1 then holds the uop now in AG if it moves (ag_go) or the split access that
// stays, OC2 the uop now in OC1, EX the uop now in OC2 (the uop now in EX is in WB then and forwarded). Lane B likewise.
// The uop in OC1 is pending next clock whether it stays (OC1) or moves (OC2); the uop now in AG only when AG moves
// (then it is in OC1; a move that does not reach OC1 is a flush or an empty AG, where a stall costs nothing). So both
// cases are formed from registers, and AG's own stall selects between them last.
wire        exb_ew = exb_wu && !ex_rep;                    // lane B in EX writes exb_wreg (it is in WB next clock)
function pends; input [4:0] r;                             // pending writes when AG holds (the uop in AG is u itself)
  // OC1, OC2, and a conditional writer in EX (it is in WB next clock, where its forward does not wait for the condition)
  pends = (oc_v && oc_wen && oc_wreg == r) || (q_v && q_wen && q_wreg == r) || (ex_vg && ex_cw && !ex_rep && ex_wrg == r) ||
          (ocb_wen && {1'b0, ocb_wreg} == r) || (qb_wen && {1'b0, qb_wreg} == r);   // lane B in OC1, OC2
endfunction
function pendd; input [4:0] r;                             // ... when AG takes the next uop (u moves to OC1)
  pendd = pends(r) || (ag_v && u[`F_WEN] && u[`F_WREG] == r) || (n_ubv && n_ub[`F_WEN] && {1'b0, n_ub[`U_WREG +: 4]} == r);
endfunction
reg         hz_r;                                          // the uop in AG must wait (decided last clock)
wire        hazard = hz_r;
function hzf; input [`UW-1:0] x;                           // the uop x, taken next, reads a pending register
  hzf = (x[`F_LA_BSEL] == `BA_REG && pendd(dl)) || (x[`F_LA_XEN] && pendd({1'b0, dx})) || ((x[`F_SA_BSEL] == `BA_REG || x[`F_LEA]) && pendd(dsb));
endfunction
// pend1: r is written by a uop in OC1 (either lane). Used by the "two behind" rule of f_gsh: with the uop one ahead in
// OC1, the writer would be only in EX when this uop reaches OC2, and the WB forward that is missing would be needed.
function pend1; input [4:0] r; pend1 = (oc_v && oc_wen && oc_wreg == r) || (ocb_wen && {1'b0, ocb_wreg} == r); endfunction
// gbub: hold for the no-WB-forward classes and for a predicted return behind a store through A7
wire   gbub     = (a_gsh && ((u[`F_ASEL] == `AS_REG && pend1(u[`F_AREG])) || (u[`F_BSEL] == `BS_REG && pend1(u[`F_BREG])))) ||
                  (a_gsy && oc_v) || (a_rts && (oc_s7 || q_s7));
wire   fbub     = oc_v && ((a_rdf && o_slz) || (a_rdx && o_wrx)) || gbub;       // the flag gaps, plus gbub
assign stall_ag = ag_v && (hazard || stall_oc || fbub);               // a split access in OC1 also holds AG
// Copies of the stall for its load groups, because synthesis never splits a clock-enable or LUT output net: the base and
// index registers' selects, the uop register, lane B's register and the rest. Gowin merges identical LUTs even with
// the syn_keep attribute (every earlier copy here had become stall_ag), so each copy takes its own preserved copy of
// ag_v; the uop and lane B registers get four copies each, one per quarter of their bits. (Before the copies were made
// this way, the merged stall was a single enable net of 7.5 ns.)
reg         agv_r /* synthesis syn_preserve = 1 */;   // ag_v copy for the held base and index registers
reg         agv_f /* synthesis syn_preserve = 1 */;   // ... for the forwarding selects and the interlock register
reg         agv_v /* synthesis syn_preserve = 1 */;   // ... for the DR queue counters
reg  [3:0]  agv_u /* synthesis syn_preserve = 1 */;   // ... for the four quarters of the lane A uop register
reg  [3:0]  agv_n /* synthesis syn_preserve = 1 */;   // ... for the four quarters of the lane B uop register
// sdr_q: dcnt == 2 after this clock, per case of the DS write (ds_go) and the AG take; preserved copies (the attribute
// on the output declaration keeps the four registers apart)
wire        sa_h = hazard || stall_oc || fbub;        // the stall reasons, without ag_v
wire        sa_r = agv_r && sa_h, sa_f = agv_f && sa_h, sa_v = agv_v && sa_h;   // stall copies
wire [3:0]  sa_u = agv_u & {4{sa_h}}, sa_n = agv_n & {4{sa_h}};
assign stall_dr = (dcnt == 2'd2);                                   // DR full; dcnt is a register, so this is one LUT
// The uop moves to OC1 unless it stalls, a branch in OC redirects (it is younger than that branch), or it was issued in
// the clock of an EX redirect (rs_d). OC drops it as well when redirect_ex is high in this clock.
assign ag_go    = ag_v && !stall_ag && !oc_bra && !rs_d;
// ag_mv: the same condition used for the register writes. It does not wait for redirect_ex: an EX redirect in the same
// clock is corrected by the register file restore one clock later.
assign ag_mv    = ag_v && !stall_ag && !oc_bra && !rs_d;
// The AG redirect does not wait for the stall (it uses registers only); a stalled branch keeps its place in AG and
// redirects only once (fired). A BTC-predicted branch does not redirect: fetch already went to its target.
assign bra_ag   = ag_v && !fired && !oc_bra && !rs_d && u[`F_BRA] && !u[`F_PBTC] && bfast;
assign ag_pt    = ag_go && u[`F_PBTC];

// Lane B's next value: the uop with the branch target and the target check of a predicted branch in lane B.
reg  [`UW-1:0] nub_d;
always @* begin nub_d = d_ub; nub_d[`F_LA_DISP] = tb_b; nub_d[`F_PFIX] = pbr && (tb_b != d_u2[`F_IMM]); end
integer ub_i;                        // bit loop index for the uop registers
// ---- The registers: DR queue, AG uop registers, classes, forwarding selects and interlock --------------------------
always @(posedge clk) begin
  // DS -> DR -> AG. DS writes the entry at wp while there is room (counted only with ds_go); AG takes the entry at rp
  // when AG does not stall and the entry is not being split (d_v && !sa_v && !dsplit).
  // sdr_q is (dcnt == 2) of the next dcnt, worked out per case so it is a register like stall_dr itself.
  if (rst || flush) sdr_q <= 4'd0;
  else sdr_q <= {4{(ds_go && !(d_v && !sa_v && !dsplit)) ? (dcnt == 2'd1) : (!ds_go && (d_v && !sa_v && !dsplit)) ? (dcnt == 2'd3) : (dcnt == 2'd2)}};
  if (rst || flush) begin dcnt <= 2'd0; rp <= 1'b0; wp <= 1'b0; end
  else begin
    dcnt <= dcnt + {1'b0, ds_go} - {1'b0, d_v && !sa_v && !dsplit};          // +1 written, -1 taken (a split keeps the entry)
    if (ds_go) wp <= !wp;                                                     // next entry to write
    if (d_v && !sa_v && !dsplit) rp <= !rp;                                   // next entry to read
  end
  if (rst || flush) dh <= 1'b0; else if (d_v && !sa_v) dh <= dsplit;          // the split's second half goes next
  if (rst || flush) begin dhc <= 1'b0; rpc <= 1'b0; end                      // the preserved copies of dh and rp
  else begin if (d_v && !sa_v) dhc <= dsplit; if (d_v && !sa_v && !dsplit) rpc <= !rpc; end
  // Writing an entry. DS writes only while DR is not full (stall_dr), into the entry at wp.
  if (!stall_dr && !wp) begin e0_swc <= o_sw; e0_rn <= drn(o_seq, o_act, o_ov); end
  if (!stall_dr &&  wp) begin e1_swc <= o_sw; e1_rn <= drn(o_seq, o_act, o_ov); end
  if (!stall_dr && !wp) begin e0_u2 <= o_u2; e0_p2 <= o_p2; e0_sw <= o_sw; end
  if (!stall_dr &&  wp) begin e1_u2 <= o_u2; e1_p2 <= o_p2; e1_sw <= o_sw; end
  if (!stall_dr && !wp) begin e0_dec <= o_dec; e0_seq <= o_seq; e0_act <= o_act; e0_ov <= o_ov; e0_ill <= ds_ill; end
  if (!stall_dr &&  wp) begin e1_dec <= o_dec; e1_seq <= o_seq; e1_act <= o_act; e1_ov <= o_ov; e1_ill <= ds_ill; end
  // AG valid. A flush empties AG with everything else (through d_v && !flush); AG's own redirect keeps a stalled uop.
  if (rst || rs_d || oc_bra) ag_v <= 1'b0;
  else if (!stall_ag) ag_v <= d_v && !flush;
  if (rst || rs_d || oc_bra) begin agv_r <= 1'b0; agv_f <= 1'b0; agv_v <= 1'b0; agv_u <= 4'd0; agv_n <= 4'd0; end   // the same for the copies
  else if (!stall_ag) begin agv_r <= d_v && !flush; agv_f <= d_v && !flush; agv_v <= d_v && !flush; agv_u <= {4{d_v && !flush}}; agv_n <= {4{d_v && !flush}}; end
  if (!stall_ag) fired <= 1'b0; else if (bra_ag) fired <= 1'b1;               // a new uop has not redirected yet
  // The uop registers, bit by bit, each quarter on its own stall copy.
  for (ub_i = 0; ub_i < `UW; ub_i = ub_i + 1) begin
    if (!sa_n[(ub_i * 4) / `UW]) n_ub[ub_i] <= nub_d[ub_i];                  // lane B
    if (!sa_u[(ub_i * 4) / `UW]) u[ub_i] <= d_u[ub_i];                       // lane A
  end
  if (!sa_n[0]) n_ubv <= d_ubv && d_v && !flush;                             // lane B valid (AG's output register)
  else if (rs_d || oc_bra) n_ubv <= 1'b0;
  // The classes for the flag gaps and the no-forward rule, from both candidates, with the pairing decision selecting
  // last. a_rdf also counts a predicted branch in lane B (pbr), which reads the flags beside lane A.
  if (!sa_u[0]) begin ill <= d_ill;
    a_rdf <= ((dh || pswp) ? f_rdf(d_u2) : f_rdf(d_u1)) || (pbr && f_rdf(d_u2)); a_rdx <= (dh || pswp) ? f_rdx(d_u2) : f_rdx(d_u1);
    a_gsh <= (dh || pswp) ? f_gsh(d_u2) : f_gsh(d_u1); a_gsy <= (dh || pswp) ? f_gsy(d_u2) : f_gsy(d_u1);
    // a_rts: a predicted return's load; o_slz / o_wrx: the uop moving to OC1 writes a late Z / writes X (either lane)
    a_rts <= (dh || pswp) ? (d_u2[`F_PRET] && d_u2[`F_LA_EN]) : (d_u1[`F_PRET] && d_u1[`F_LA_EN]); o_slz <= ag_go && f_slz(u); o_wrx <= ag_go && (f_wrx(u) || (n_ubv && n_ub[`U_FEN + 4])); end
  begin : agfwd                      // the forwarding selects, one-hot, and the interlock register
    // Both cases are formed from registers (AG holds: the numbers of u; AG moves: DR's numbers and the AG writes of u,
    // which is then in OC1), and the stall selects last.
    reg ew, agm, g2n, g1n;           // EX writes; AG moves (without the stall); AG write 2 / 1 reaches OC1
    reg fls, fss, fxs, bls, bss, bxs;   // AG holds: lane A / lane B in EX writes the LA base, SA base, index of u
    reg fln, fsn, fxn, bln, bsn, bxn, q2l, q2s, q2x, q1l, q1s, q1x;   // AG moves: the same for DR's numbers, and u's AG writes
    ew  = ex_vg && ex_wug && !ex_rep;    // EX's uop writes (WEN without the DBcc/CAS condition; a reader is held by pends)
    agm = ag_v && !oc_bra && !rs_d;                                  // ag_go without the stall
    g2n = agm && w2_en;
    g1n = agm && w1_en && !w1_r[4];      // write 1 to a register numbered 16 and up (a temporary) is not forwarded this way
    fls = ew && ex_wrg == lbr; fss = ew && ex_wrg == sbr; fxs = ew && ex_wrg == {1'b0, xrn};
    bls = exb_ew && {1'b0, exb_wreg} == lbr && !fls; bss = exb_ew && {1'b0, exb_wreg} == sbr && !fss; bxs = exb_ew && {1'b0, exb_wreg} == {1'b0, xrn} && !fxs;
    fln = ew && ex_wrg == dl; fsn = ew && ex_wrg == dsb; fxn = ew && ex_wrg == {1'b0, dx};
    bln = exb_ew && {1'b0, exb_wreg} == dl && !fln; bsn = exb_ew && {1'b0, exb_wreg} == dsb && !fsn; bxn = exb_ew && {1'b0, exb_wreg} == {1'b0, dx} && !fxn;
    // one select per value: lane A's EX write over lane B's, then AG write 2 over AG write 1 (when both AG writes
    // update the same register, write 2 already includes the load side's step through ADJ)
    q2l = g2n && w2_r == dl && !fln && !bln;          q2s = g2n && w2_r == dsb && !fsn && !bsn;          q2x = g2n && w2_r == {1'b0, dx} && !fxn && !bxn;
    q1l = g1n && w1_r == dl && !fln && !bln && !q2l;  q1s = g1n && w1_r == dsb && !fsn && !bsn && !q2s;  q1x = g1n && w1_r == {1'b0, dx} && !fxn && !bxn && !q2x;
    wcw <= ex_cw;                                                    // the uop in WB next clock is a conditional writer
    fl  <= sa_f ? fls : fln;  fs  <= sa_f ? fss : fsn;  fx  <= sa_f ? fxs : fxn;   // lane A's WB value
    flb <= sa_f ? bls : bln;  fsb <= sa_f ? bss : bsn;  fxb <= sa_f ? bxs : bxn;   // lane B's WB value
    // OC's AG writes; cleared while AG holds (the held registers already took them)
    pl2 <= !sa_f && q2l; ps2 <= !sa_f && q2s; px2 <= !sa_f && q2x; pl1 <= !sa_f && q1l; ps1 <= !sa_f && q1s; px1 <= !sa_f && q1x;
    nl_r <= sa_f ? !(fls || bls) : !(fln || bln || q2l || q1l);      // none of the others: use the held value
    ns_r <= sa_f ? !(fss || bss) : !(fsn || bsn || q2s || q1s);
    nx_r <= sa_f ? !(fxs || bxs) : !(fxn || bxn || q2x || q1x);
    // The interlock for the uop in AG next (fields of u when AG holds, else of d_u). The uop taken next is d_u2 or d_u1
    // by the late pairing decision, so both are formed, and that decision and the stall select last.
    hz_r <= !rst && (sa_f ? ((u[`F_LA_BSEL] == `BA_REG && pends(lbr)) || (u[`F_LA_XEN] && pends({1'b0, xrn})) || ((u[`F_SA_BSEL] == `BA_REG || u[`F_LEA]) && pends(sbr)))
                          : ((dh || pswp) ? hzf(d_u2) : hzf(d_u1)));
  end
end

endmodule
`default_nettype wire
