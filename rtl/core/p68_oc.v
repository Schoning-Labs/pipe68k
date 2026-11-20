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
// ==== p68_oc.v: operand fetch in two stages (OC1 and OC2) ============================================================
//
// Abbreviations used in this file: AG = address generation (p68_ag.v), OC1/OC2 = the first and second operand stage
// (this file), EX = execute (p68_ex.v), WB = write-back (p68_wb.v), DMA = direct memory access (the Amiga chipset
// writing memory on its own), BTC = branch target cache, smc = self-modifying code (a store into instructions that were
// already fetched), CAS = the 68020's compare-and-swap instruction, CACR/CAAR = the cache control and cache address
// registers of the 68020, An = an address register, T0-T3 = the sequencers' temporaries.
//
// ---- What this unit is for ----------------------------------------------------------------------------------------
//
// OC gets a uop from AG with its load and store addresses already computed, and hands EX the two operands EX will
// work on: each one is a register value, the loaded memory value, or the uop's immediate. To get the memory value it
// reads the 8 KB data cache (p68_dcache.v, instantiated here), and because recent stores may not be in the cache yet,
// it also forwards store data from WB and from the store before it. It splits misaligned accesses (a word or long that
// crosses a 4-byte boundary, which the 68020 allows) into two reads, and it decides when a uop must replay.
//
// It is two stages, each one clock:
//   - OC1 (the oc_* registers): the D-cache tag compare and data read, store forwarding, the misaligned split, and the
//     replay causes that are known that early (a miss, a load that meets a younger store, a load the D-cache kills).
//     It registers the forwarded word into OC2. Branches through a register (JMP (A0), RTS without a prediction)
//     redirect fetch from OC1 (oc_bra).
//   - OC2 (the q_* registers): the byte alignment of the loaded value, the operand selects together with the register
//     file reads, and the replay causes that are known late (the store buffer could be full, a snoop on the load's
//     line, a store near lane B's code).
//
// OC never stalls the pipeline for a cache result. A miss, a load that meets a store in OC2 or EX, a load the D-cache
// kills (a snoop or a tag write in the same clock), a store that could find the store buffer full, a wrong fetch
// prediction (F_PFIX) and the uop behind a store into fetched code all replay in EX: the uop does not commit, EX
// redirects to it, and the second time round it finds the line in the line buffer. The one case where OC holds a uop
// is a misaligned access, which stays in OC1 for one more clock per extra read (oc_stay), and AG waits behind it
// (stall_oc).
//
// Clock: everything runs on the CPU clock clk. Fed by AG (n_u, n_ub, ea_s, saddr, the AG register writes), the
// register file (the operand reads in OC2), the memory port (fill data), WB (the store being written now) and EX (the
// store in EX, the replayed load's miss). Feeds EX (op_a, op_b, the q_* registers, oc_rep), AG (the OC1 uop's AG writes
// for AG's forwarding, the stall), the register file (oc_w*, oc_h1, oc_h2: AG's An writes enter it from here), and
// fetch (oc_bra).
//
// ---- What was tried, and why it ended up like this ----------------------------------------------------------------
//
// Replay instead of stall goes back to the first one-file pipeline sample, where a stall on a cache result that fanned
// out to every stage alone cost 13 MHz. OC was one stage at first. The second timing round split it in two (cache,
// forwarding and split in OC1; alignment and operand selects in OC2), and that split also made room for lane B's
// operand reads in dual issue. Misaligned accesses are split here, not in the sequencer, because a split here costs one
// clock per extra read and no uops. In the first timing round, two cuts in this unit were tried and reverted because
// the build with them was slower (57.5 MHz, with the worst path running through exactly that logic): aligning the
// forwarded store word in OC itself, and comparing each store term separately. Later rounds made every select that
// steers data here a register: the split decode and the D-cache compare address are loaded with the uop, the byte
// selects of the load value and the operand selects are decoded from the uop moving to OC2, and the bytes forwarded
// from the WB store into the predicted-return check are compared one clock later in OC2.
//
// Two bugs the test benches found live here: a refill could write a stale word over a store to the same word (OC and
// the D-cache now drop such a refill), and a line-crossing load inside a sequence could replay faster than its second
// word's single read arrived, which became a livelock (found by WinUAE's cputester data with the caches off; the
// D-cache now notices that the second word is already there and just replays).
//
// ---- Gowin techniques in this file --------------------------------------------------------------------------------
//
//  - syn_preserve copies of oc_v (ocv_u, ocv_i, ocv_d, ocv_f): the stay enable has hundreds of loads, and Gowin merges
//    identical LUTs, so each copy of the enable is built from its own preserved copy of the valid register.
//  - syn_keep on x_lk keeps the "staying" part of the compare address apart, so the late AG address meets it in one LUT.
//  - syn_maxfan = 16 on the module asks synthesis to duplicate drivers with more than 16 loads.
`include "p68_uop.vh"                // the uop format
`default_nettype none
module p68_oc (
  input  wire        clk,            // CPU clock
  input  wire        rst,            // synchronous reset
  input  wire        redirect_ex,    // EX redirects this clock (unused here: OC empties one clock later, with rs_d)
  input  wire        rs_d,           // redirect_ex one clock ago (a register): OC1 and OC2 empty now
  // ---- from AG ----
  input  wire        ag_go,          // AG hands over a uop this clock
  input  wire [`UW-1:0] n_u,         // the lane A uop from AG
  input  wire [31:0] ea_s,           // its load address (also the target of a branch through a register)
  input  wire [31:0] saddr,          // its store address
  input  wire [1:0]  sa_d,           // AG's store-only address bits 1-0 (for the split decode, selected here by SA_SAME)
  input  wire        w1_en, w2_en,   // AG's two register writes (An updates, LEA), passed on to the register file
  input  wire [4:0]  w1_r, w2_r,
  input  wire [31:0] w1_v, w2_v,
  // ---- D-cache control, fill and snoop (passed to p68_dcache) ----
  input  wire [3:0]  cacr,           // CACR bits 3-0: C (clear all), CE (clear entry), F (freeze), E (enable)
  input  wire        clr_all, clr_ent,   // CACR written with C = 1 / CE = 1 (one clock each)
  input  wire [31:0] caar,           // CAAR: the address whose line CE clears
  input  wire [15:0] unc_map,        // uncached 1 MB granules of the 24-bit area (chip RAM, I/O)
  input  wire [7:0]  unc_hi,         // an uncached 32 MB window above 16 MB (the RTG board)
  input  wire [6:0]  cac_hi,         // the only cached window above 16 MB (the RAM board)
  input  wire        ex_commit,      // the uop in EX commits
  input  wire        d_miss,         // the replayed load in EX missed: start the fill
  input  wire [31:0] d_addr,         // its address
  input  wire [31:2] d_addr1,        // d_addr + 4 (its second word), registered in EX
  input  wire [1:0]  d_lsz,          // its load size
  input  wire        sn_v,           // external snoop: a DMA write to sn_addr
  input  wire [31:2] sn_addr,
  output wire        dr_req,         // D-cache read request to the memory unit
  output wire [31:2] dr_addr,        // ... line (or word, uncached)
  output wire        dr_unc,         // ... uncached: one word, exactly the bytes dr_be
  output wire [3:0]  dr_be,
  input  wire        dr_go,          // the memory unit took the request
  input  wire        r_d,            // a read beat (64 bits) on r_data
  input  wire [63:0] r_data,
  input  wire        r_done,         // the read ends
  output wire        unc_hold,       // an uncached read is in flight or its word is unused: interrupts wait
  input  wire [2:0]  sb_cnt,         // store buffer entries in use (in p68_mem.v)
  input  wire        smc,            // the store committing in EX hit fetched code: the uop in OC replays
  input  wire        d_fu,           // the replayed load in EX is a CAS/CAS2 step (U_LOCK): it reads uncached
  output wire        oc_pmiss,       // predicted target wrong: a BTC branch (computed target) or a return (loaded PC)
  // ---- the store in WB (being written into the D-cache now) and the store before it ----
  input  wire        wb_st,          // WB stores this clock
  input  wire        wb_hit,         // ... and the line is in the D-cache
  input  wire [31:0] wb_saddr,       // ... at this address
  input  wire [31:0] sd,             // ... this data (aligned to the word)
  input  wire [3:0]  sbe,            // ... these bytes (bit 3 = address +0)
  input  wire        ls_v,           // the store of one clock ago (its cache write is not readable yet)
  input  wire [31:2] ls_addr,
  input  wire [31:0] ls_data,
  input  wire [3:0]  ls_be,
  // ---- the store in EX (a load behind it replays) ----
  input  wire        ex_v, ex_st,
  input  wire [31:0] ex_saddr,
  // ---- register reads for lane A's operands (in OC2) ----
  output wire [4:0]  rd_a, rd_b,
  input  wire [31:0] rv_a, rv_b,
  // ---- stage registers and results to EX ----
  output reg         oc_v,           // a uop is in OC1
  output wire        oc_vrf,         // copy of oc_v for the register file's AG write enables
  output reg  [`UW-1:0] oc_u,        // the lane A uop in OC1
  output reg  [31:0] oc_addr,        // its load address
  output reg  [31:0] oc_saddr,       // its store address
  output reg         oc_w1_en, oc_w2_en,   // its AG writes (they enter the register file from here)
  output reg  [4:0]  oc_w1_r, oc_w2_r,
  output reg  [31:0] oc_w1_v, oc_w2_v,
  output wire [31:0] oc_w1_va, oc_w2_va,   // copies of oc_w1_v / oc_w2_v for AG's early register read only
  output reg  [7:0]  oc_h1,          // AG write 1 hits A0-A7 (one-hot, with oc_w1_en)
  output reg  [11:0] oc_h2,          // AG write 2 hits A0-A7, T0-T3 (one-hot, with oc_w2_en)
  output wire [31:0] op_a, op_b,     // lane A's operands for EX
  output wire        st_hit,         // the store's line is in the D-cache (WB writes the cache only then)
  output wire        oc_rep,         // the uop in OC2 must replay
  output wire        q_rpb,          // the replay includes the older lane B of a swapped pair (always 0, decided in EX)
  output wire        oc_miss,        // the load in OC2 missed (EX starts the fill when it replays)
  // ---- misaligned accesses ----
  output wire        oc_stay,        // the uop in OC1 stays another clock (second word of a misaligned access)
  output wire        stall_oc,       // AG must not hand over a uop
  output wire [31:0] oc_laddr,       // load address for the fill (always the first word)
  output wire        st_hit2,        // the second word of a misaligned store is in the D-cache
  output wire        oc_bra,         // a branch through a register (not AG's register-free kind) redirects fetch to oc_addr
  // ---- OC2 stage registers (to EX). op_a, op_b, st_hit, oc_rep, oc_miss, oc_laddr, st_hit2 and oc_pmiss above are
  //      OC2's too ----
  output reg         q_v,            // a uop is in OC2
  output reg  [`UW-1:0] q_u,         // the lane A uop in OC2
  output reg  [31:0] q_addr,         // its load address
  output reg  [31:0] q_saddr,        // its store address
  output reg         q_w1_en, q_w2_en,   // its AG writes, passed on to EX (which commits them)
  output reg  [4:0]  q_w1_r, q_w2_r,
  output reg  [31:0] q_w1_v, q_w2_v,
  // ---- lane B (register-only) through OC1 and OC2; OC2 reads its operands ----
  input  wire [`UW-1:0] n_ub,        // the lane B uop from AG
  input  wire        n_ubv,
  output reg  [`UW-1:0] oc_ub, q_ub, // lane B in OC1 and OC2
  output reg         oc_ubv, q_ubv,
  output wire [4:0]  rd_c, rd_d,     // lane B's register reads
  input  wire [31:0] rv_c, rv_d,
  output wire [31:0] opb_a, opb_b    // lane B's operands for EX
)  /* synthesis syn_maxfan = 16 */;

// ---- Misaligned access state (declared first, because the D-cache addresses use it) --------------------------------
// The 68020 allows a word at an odd address and a long at any address. The D-cache delivers one aligned 32-bit word
// per clock, so a word at offset 3 or a long at offset 1, 2 or 3 needs the next word as well. ld_x and st_x say that the
// load or the store crosses a word boundary; x_b says that the store's second word is a word of its own (not the
// load's second word), which needs a separate clock to read its tag ("phase B"). The uop stays in OC1 for x_n extra
// clocks, counted by xp.
reg  [1:0]  xp;                      // clocks the current uop has spent in OC1
wire        oc_ld    = oc_u[`F_LA_EN];   // the uop in OC1 loads
wire        oc_st    = oc_u[`F_SA_EN];   // the uop in OC1 stores
// The split decode (ld_x, st_x, x_b) and the compare address dc_ca are registers, loaded with the uop that enters OC1
// (they depend only on it), so the store address no longer passes a decode and a select in front of the D-cache hit.
wire        n_ldx = n_u[`F_LA_EN] && ((n_u[`F_LSZ] == `SZ_W) ? (ea_s[1:0] == 2'd3) :    // a word crosses only from offset 3
                                      (n_u[`F_LSZ] == `SZ_L) ? (ea_s[1:0] != 2'd0) : 1'b0);   // a long from any offset but 0
// The store's split decode is formed for both possible addresses (ea_s when SA_SAME, else AG's own store address) and
// selected last, so the SA_SAME select is not in front of the decode.
function sxf; input [1:0] a; sxf = n_u[`F_SA_EN] && ((n_u[`F_SZ] == `SZ_W) ? (a == 2'd3) : (n_u[`F_SZ] == `SZ_L) ? (a != 2'd0) : 1'b0); endfunction
wire        stxE = sxf(ea_s[1:0]), stxD = sxf(sa_d);   // the store crosses, at the load address / at its own address
wire        n_stx = n_u[`F_SA_SAME] ? stxE : stxD;
reg         ld_x, st_x, x_b;         // the load crosses; the store crosses; the store's second word needs phase B
reg  [31:0] dc_ca;                   // the address the D-cache compares this clock (the word whose data is wanted)
reg  [31:2] dc_cf /* synthesis syn_preserve = 1 */;   // dc_ca's own copy for the forwarding compares (f_a), local to dqm
wire [1:0]  x_n  = {1'b0, ld_x} + {1'b0, x_b};          // extra clocks this uop needs in OC1 (0, 1 or 2)
assign      oc_stay  = oc_v && (xp < x_n);
// Preserved copies of oc_v (Gowin merges identical LUTs, so copies of oc_stay need copies of a register): four for the
// stage registers' enables (quarters of oc_u, then the rest), five for the D-cache read index copies, one for the
// D-cache's own uses, one for the register file's AG write enables.
reg  [3:0]  ocv_u /* synthesis syn_preserve = 1 */;
reg  [4:0]  ocv_i /* synthesis syn_preserve = 1 */;
reg         ocv_d /* synthesis syn_preserve = 1 */;
reg         ocv_f /* synthesis syn_preserve = 1 */;
wire        xp_s = (xp < x_n);                         // oc_stay without oc_v
wire [3:0]  st_u = ocv_u & {4{xp_s}};                  // stay copies for the stage registers
wire [4:0]  st_i = ocv_i & {5{xp_s}};                  // stay copies for the D-cache read index
assign      oc_vrf = ocv_f;
wire        xx   = (xp != 2'd0);     // not the first clock of a split access (a register: keeps the decode off the data path)
wire [29:0] ld_w1 = oc_addr[31:2] + 30'd1, st_w1 = oc_saddr[31:2] + 30'd1;   // the second word of the load / the store
// While the uop stays, the D-cache reads the tags and data of the next word it needs (dc_la) and compares hit, data
// and kill at dc_ca: the first word in the first clock, then the store's second word (phase B), then the load's
// second word (phase A, always last, because its data is what goes on to OC2).
wire [29:0] x_la  = (x_b && xp == 2'd0) ? st_w1 : ld_w1;   // the next word to read while staying
wire [31:0] dc_la = oc_stay ? {x_la, 2'b00} : ea_s;        // the D-cache read address of this clock
// dc_ca and dc_cf are an AND-OR: ea_s (late, from AG's adder) meets the kept "staying" part in one LUT. Before, ea_s
// passed two LUTs in front of dc_ca, and that was one of the real failing paths of the full system.
wire [29:0] x_lk /* synthesis syn_keep = 1 */;
assign x_lk = {30{oc_stay}} & x_la;
always @(posedge clk) begin
  if (!oc_stay) begin                                      // a new uop enters OC1: decode its split
    ld_x <= n_ldx; st_x <= n_stx; x_b <= n_u[`F_SA_SAME] ? (stxE && !n_ldx) : stxD;   // same address: phase A covers the store
  end
  dc_ca <= ({32{!oc_stay}} & ea_s) | {x_lk, 2'b00};   // staying: the next clock's word (phase B after the first clock, then phase A)
  dc_cf <= ({30{!oc_stay}} & ea_s[31:2]) | x_lk;
end

// ---- D-cache --------------------------------------------------------------------------------------------------------
wire [31:0] dc_q, dq_c, dq_l, dq_u;  // the word at dc_ca; its sources: cache word, line buffer word, ub word
wire        dc_hit, dkill, st_hit1, ct_hit, sn_a0, lbh, ubh;   // hit; kill; store hit; tag hit; snoop on word 0; line buffer / ub hit
reg         ex_lh, q_lh;             // the load now in EX (OC2): first word hit, only the second word missed
// OC2 registers besides the outputs
reg  [31:0] q_dq, q_xh;              // the forwarded word of the last clock in OC1; the first word of a split load
reg         q_xx, q_rep1, q_miss, q_sth, q_sth2, q_pm, q_stx;   // split; OC1 replay; miss; store hits; prediction miss; crossing store
reg  [29:0] q_a1;                    // second word of the load (late snoop check)
wire        sn_q;                    // a snoop on the OC2 load's lines
wire        oc_lh;                   // split load: first word hit, second missed
p68_dcache i_dc (
  .clk(clk), .rst(rst), .cacr(cacr), .clr_all(clr_all), .clr_ent(clr_ent), .caar(caar), .unc_map(unc_map), .unc_hi(unc_hi), .cac_hi(cac_hi),
  // read addresses: AG's load address, the next word while staying (the select is inside, in copies), the store address
  .ea_s(ea_s), .la_x(x_la[10:2]), .saddr(saddr),   /* the read index select (oc_stay) is inside the D-cache, in copies */ .oc_v(ocv_d), .oc_ld(oc_ld), .oc_st(oc_st), .oc_addr(dc_ca), .oc_saddr(oc_saddr),
  .dq(dc_q), .dhit(dc_hit), .st_hit(st_hit1), .dkill(dkill), .dq_c(dq_c), .dq_l(dq_l), .lbh(lbh), .dq_u(dq_u), .ubh(ubh),
  .oc_a0(oc_addr), .oc_stay(oc_stay), .st_i(st_i), .oc_x0(!xx), .oc_x2(xx && ld_x), .oc_fu(oc_u[`U_LOCK]), .ct_hit(ct_hit),
  .sn_a0(sn_a0), .d_fu(d_fu), .d_lh(ex_lh),
  .q_ld(q_v && q_u[`F_LA_EN]), .q_x(q_xx), .q_a0(q_addr[31:2]), .q_a1(q_a1), .q_st(q_v && q_u[`F_SA_EN]), .q_sa(q_saddr), .sn_q(sn_q),
  .ex_v(ex_v), .ex_st(ex_st), .ex_saddr(ex_saddr), .ex_commit(ex_commit), .d_miss(d_miss), .d_addr(d_addr), .d_addr1(d_addr1), .d_lsz(d_lsz),
  .wb_st(wb_st), .wb_hit(wb_hit), .wb_saddr(wb_saddr), .sd(sd), .sbe(sbe),
  .sn_v(sn_v), .sn_addr(sn_addr),
  .dr_req(dr_req), .dr_addr(dr_addr), .dr_unc(dr_unc), .dr_be(dr_be), .dr_go(dr_go), .r_d(r_d), .r_data(r_data),
  .r_done(r_done), .unc_hold(unc_hold));
// A store replays when the store buffer could be full by the time it reaches WB: the entries in use plus the stores in
// EX and WB (one each) plus one more when the store crosses a word (it needs two entries). A crossing store in EX or
// in its first WB clock never meets a store in OC (AG stalls behind it), so the stores in EX and WB count one each.
// Four or more (bit 2 or bit 3 set) replays.
wire [3:0]  sb_fut   = {1'b0, sb_cnt} + {3'b0, ex_v && ex_st} + {3'b0, wb_st} + {3'b0, q_stx};   // checked in OC2

// ---- Store forwarding -----------------------------------------------------------------------------------------------
// The word whose data is in dc_q is f_a (dc_cf, the same as dc_ca; it differs only in phase B, whose data is not used).
// The D-cache's block RAM is written in WB, but a read in the same clock still returns the old word, and the word
// written one clock ago (ls_*) only shows up in the read after next. So the store in WB and the store before it are
// compared with the word being read, and their bytes replace the cache bytes.
wire [29:0] f_a  = dc_cf;
wire        f_wb = wb_st && (wb_saddr[31:2] == f_a);   // the WB store writes this word
wire        f_ls = ls_v  && (ls_addr == f_a);          // the store before it wrote this word
// The forwarded word as an AND-OR of one-hot byte selects (WB store, the store before it, the split buffer ub, the line
// buffer, the cache), so the hit compares select the data in two LUT levels. The WB store is the newest and wins.
reg  [31:0] dqm;
integer bk;
always @* for (bk = 0; bk < 4; bk = bk + 1) begin : dqb
  reg e_sd, e_ls, e_rs;              // byte bk comes from the WB store / the store before / the read
  e_sd = f_wb && sbe[bk];
  e_ls = !e_sd && f_ls && ls_be[bk];
  e_rs = !e_sd && !(f_ls && ls_be[bk]);
  dqm[bk*8 +: 8] = ({8{e_sd}} & sd[bk*8 +: 8]) | ({8{e_ls}} & ls_data[bk*8 +: 8]) |
                   ({8{e_rs && ubh}} & dq_u[bk*8 +: 8]) | ({8{e_rs && !ubh && lbh}} & dq_l[bk*8 +: 8]) |
                   ({8{e_rs && !ubh && !lbh}} & dq_c[bk*8 +: 8]);
end
// Stores in OC2 and EX reach WB after this load's word was formed, so they cannot be forwarded: the load replays.
wire   st_match = oc_ld && ((ex_v && ex_st && (ex_saddr[31:2] == f_a)) || (q_v && q_u[`F_SA_EN] && (q_saddr[31:2] == f_a)));

// ---- Misaligned accesses --------------------------------------------------------------------------------------------
// A word or long that crosses a longword boundary needs the next longword too. The uop stays in OC1 for one more clock
// per extra read: phase B reads the tag of the store's second word (when it is not the load's), phase A (last) reads the
// load's second word, data and tag. Captured from the first clock: the first word's data (forwarded), its hit and
// kill, the store-in-EX match and the store's first-word hit. A misaligned store is written by WB in two clocks; AG
// stalls so that nothing follows it in OC until its second write is in WB (forwarding covers it from then on).
reg  [31:0] x_hi;                    // the first word of a split load
reg         x_miss0, x_rep0, x_sth0, x_sth1, x_tail, x_tail2, x_kill0;   // first-clock miss, store match, store hits; tails; kill
assign      stall_oc = oc_stay || (oc_v && st_x) || x_tail || x_tail2;   // a crossing store holds AG until it is in WB
always @(posedge clk) begin
  if (rst || rs_d || !oc_stay) xp <= 2'd0; else xp <= xp + 2'd1;          // count the clocks of a staying uop
  if (xp == 2'd0) begin x_hi <= dqm; x_miss0 <= oc_ld && !dc_hit; x_rep0 <= st_match; x_sth0 <= st_hit1; x_kill0 <= dkill; end
  if (xp == 2'd1) x_sth1 <= ct_hit;                                       // phase B: the store's second word's tag hit
  x_tail <= !rst && !rs_d && oc_v && st_x && !oc_stay;                    // the crossing store went to OC2
  x_tail2 <= !rst && !rs_d && x_tail;                                     // ... and is in EX now
  q_lh   <= oc_lh; ex_lh <= q_lh;                                         // travels with the load to EX
end

// ---- OC2: the load value and the operands ---------------------------------------------------------------------------
// The load value and operands are AND-ORs of byte selects that depend on registers only (address bits, load size, split
// phase, operand selects). The 68k is big-endian: the byte at the lowest address is the most significant. Load byte j
// (0 = bits 7-0) is byte idx of {dhi, dqm} (bytes 7-4 dhi, 3-0 dqm), idx = j + 4 (long), j + 6 (word, j < 2), j + 7
// (byte, j = 0), minus the address offset; dhi is x_hi in a split access, else dqm. OC1 registers dqm and x_hi into
// q_dq and q_xh. For example a long at offset 2 takes bytes 5-2: the low half of the first word and the high half of
// the second. The byte selects, the operand selects and the A immediate are registers, decoded from the uop moving from
// OC1 to OC2 (gsel works on oc_u, oc_addr and xx).
function [31:0] gsel; input [1:0] lsz; input [1:0] ao; input x;      // {xh selects, dq selects}, [gj*4 + gm]
  integer kj, km; reg [3:0] ix; reg ok;   // load byte; source byte; source index; load byte exists for this size
  begin
    gsel = 32'h0;
    for (kj = 0; kj < 4; kj = kj + 1) begin
      case (lsz)
        `SZ_B:   begin ix = kj + 7 - ao; ok = (kj == 0); end
        `SZ_W:   begin ix = kj + 6 - ao; ok = (kj < 2); end
        default: begin ix = kj + 4 - ao; ok = 1'b1; end
      endcase
      for (km = 0; km < 4; km = km + 1) begin
        gsel[kj*4 + km]      = ok && (ix == km || (ix == km + 4 && !x));   // from dqm (and from dhi when not split: same word)
        gsel[16 + kj*4 + km] = ok && ix == km + 4 && x;                    // from x_hi in a split
      end
    end
  end
endfunction
reg  [31:0] q_gs;                    // the registered byte selects
reg         a_mem, a_reg, a_imm, b_mem, b_imm, b_reg;   // one-hot operand selects (registers)
reg  [31:0] imm_a;                   // operand A's immediate (1 for DBcc)
integer gj, gm;
reg  [31:0] ld_v;                    // the aligned load value (zero-extended; EX sign-extends where needed)
always @* for (gj = 0; gj < 4; gj = gj + 1) begin : ldb
  reg [7:0] v;
  v = 8'h0;
  for (gm = 0; gm < 4; gm = gm + 1)
    v = v | ({8{q_gs[gj*4 + gm]}} & q_dq[gm*8 +: 8]) | ({8{q_gs[16 + gj*4 + gm]}} & q_xh[gm*8 +: 8]);
  ld_v[gj*8 +: 8] = v;
end
// The operands: an AND-OR of the one-hot selects, so the register file value (late) meets its select in one LUT.
assign op_a = ({32{a_mem}} & ld_v) | ({32{a_reg}} & rv_a) | ({32{a_imm}} & imm_a);
assign op_b = ({32{b_mem}} & ld_v) | ({32{b_reg}} & rv_b) | ({32{b_imm}} & q_u[`F_IMM]);
// ---- OC1: the replay causes known in the first stage ----
wire   miss_hi  = ld_x && !dc_hit;                     // the second word of a split load missed
assign oc_lh    = xx && !x_miss0 && miss_hi;           // ... while its first word hit
// In the last clock of a split load, a snoop on the first word's line (sn_a0) still kills it: the load commits after
// that snoop, so it must see the DMA write.
wire   oc_kill  = oc_ld && (xx ? (x_kill0 || sn_a0 || (ld_x && dkill)) : dkill);
wire   o1_miss  = oc_ld && (xx ? (x_miss0 || miss_hi) : !dc_hit);
wire   o1_rep   = o1_miss || (xx && x_rep0) || st_match || oc_kill || oc_u[`F_PFIX];   // a wrongly cut instruction replays too
// ---- OC2: store buffer, self-modifying code, and a snoop in sn1/sn2 on the load's lines (the load commits after it) ----
assign oc_miss  = q_miss;
assign oc_laddr = q_addr;
// A lane A store with a younger lane B replays when it lands within eight bytes of lane B's instruction (it may change
// lane B's code); the store then runs alone, because DS issues the first instruction after a replay without a partner.
// bvd is the distance in 8-byte blocks between the store and lane B's PC: -1, 0 or +1 counts as near.
wire [28:0] bvd    = q_saddr[31:3] - q_ub[(`U_PC + 3) +: 29];
wire        bov    = q_ubv && !q_ub[`U_BOLD] && q_u[`F_SA_EN] && (bvd == 29'd0 || bvd == 29'd1 || bvd == {29{1'b1}});
// A swapped pair (lane B older) replayed by a store into fetched code restarts at lane B so lane B does not commit. That
// decision is made in EX from a register, so this output is always 0.
assign q_rpb    = 1'b0;
assign oc_rep   = q_rep1 || sn_q || (q_u[`F_SA_EN] && (sb_fut[3] || sb_fut[2])) || bov;   // all replay causes of OC
assign st_hit   = q_sth;
assign st_hit2  = q_sth2;

// ---- Predicted target check -----------------------------------------------------------------------------------------
// A BTC-predicted branch carries the predicted target in IMM: it is compared with the computed target (oc_addr). A
// predicted return (RTS) carries the return address popped from DS's return stack: it is compared with the PC loaded
// from the stack. That compare is done byte by byte with each source of the load value (cache word, line buffer word,
// the store before WB) and the compare bits pass the same selects as the data, so the compare is not behind the data
// path. Only an aligned long load in one clock is checked; any other (split, misaligned, uncached second word) counts as
// a miss, which only costs the redirect to the loaded PC.
wire [31:0] pim = oc_u[`F_IMM];      // the predicted target
reg  [3:0]  peq;                     // byte k of the loaded PC equals the prediction
integer pk;
always @* for (pk = 0; pk < 4; pk = pk + 1)
  peq[pk] = (f_wb && sbe[pk]) ? 1'b1 :            // the WB store's bytes are compared in OC2 (q_pw, q_sd)
            (f_ls && ls_be[pk]) ? (ls_data[pk*8 +: 8] == pim[pk*8 +: 8]) :
            lbh ? (dq_l[pk*8 +: 8] == pim[pk*8 +: 8]) : (dq_c[pk*8 +: 8] == pim[pk*8 +: 8]);
wire        pret_ok = (&peq) && !xx && !ld_x && (oc_addr[1:0] == 2'd0) && (oc_u[`F_LSZ] == `SZ_L);   // a checkable return that matches
wire        o1_pm   = (oc_u[`F_PBTC] && oc_addr != oc_u[`F_IMM]) || (oc_u[`F_PRET] && !pret_ok);
// The bytes forwarded from the store in WB (sd comes from WB's value in the same clock) are registered and compared in
// OC2. In the same clock they had made a real failing path from the WB value through sd and the compare into q_pm.
reg  [3:0]  q_pw;                    // the bytes of the loaded PC that came from the WB store
reg  [31:0] q_sd;                    // that store's data
wire [31:0] q_pim = q_u[`F_IMM];
wire [3:0]  q_pne = {q_sd[31:24] != q_pim[31:24], q_sd[23:16] != q_pim[23:16], q_sd[15:8] != q_pim[15:8], q_sd[7:0] != q_pim[7:0]};
assign oc_pmiss = q_pm || |(q_pw & q_pne);                             // registered into OC2

// ---- Branch through a register ----------------------------------------------------------------------------------
// Branches through a register (JMP (An), JSR (d16,An), an unpredicted RTS and so on) redirect here, in the first clock
// in OC1, to oc_addr (AG redirects the register-free kind one clock earlier). It is a register, loaded with the uop
// entering OC1 (a branch never stays), because the redirect fans out to the whole front end.
reg    oc_bra_r;
always @(posedge clk) oc_bra_r <= !rst && !rs_d && ag_go && !oc_stay && n_u[`F_BRA] && !n_u[`F_PBTC] && (n_u[`F_LA_BSEL] == `BA_REG || n_u[`F_LA_XEN]);
assign oc_bra = oc_bra_r;
// ---- Register reads in OC2 ----
assign rd_c = q_ub[`F_AREG]; assign rd_d = q_ub[`F_BREG];             // lane B's operand registers
// lane B's operand A: a register, 1 for a DBcc (the counter is decremented by an ALU subtract), or the immediate
assign opb_a = (q_ub[`F_ASEL] == `AS_REG) ? rv_c : (q_ub[`F_UNIT] == `UN_DBR) ? 32'd1 : q_ub[`F_IMM];
assign opb_b = rv_d;                                                  // lane B's operand B is always a register
assign rd_a = q_u[`F_AREG];                                           // lane A's operand registers
assign rd_b = q_u[`F_BREG];

reg  [31:0] w1_va /* synthesis syn_preserve = 1 */;   // copies of the AG write values for AG (one net into AG, few loads)
reg  [31:0] w2_va /* synthesis syn_preserve = 1 */;
assign oc_w1_va = w1_va; assign oc_w2_va = w2_va;
integer ou_i;                        // bit loop index
// ---- Stage registers: AG -> OC1 -> OC2 ----
always @(posedge clk) begin
  if (rst) oc_v <= 1'b0;
  else if (rs_d) oc_v <= 1'b0;                                         // empty one clock after an EX redirect
  else oc_v <= oc_stay || ag_go;                                       // a staying uop stays valid
  if (rst || rs_d) begin ocv_u <= 4'd0; ocv_i <= 5'd0; ocv_d <= 1'b0; ocv_f <= 1'b0; end   // the copies of oc_v
  else begin ocv_u <= {4{oc_stay || ag_go}}; ocv_i <= {5{oc_stay || ag_go}}; ocv_d <= oc_stay || ag_go; ocv_f <= oc_stay || ag_go; end
  for (ou_i = 0; ou_i < `UW; ou_i = ou_i + 1) if (!st_u[(ou_i * 4) / `UW]) oc_u[ou_i] <= n_u[ou_i];   // a quarter per stay copy
  if (!st_u[3]) begin                                                 // the rest of OC1 loads with the last quarter's copy
    oc_addr <= ea_s; oc_saddr <= saddr;
    oc_w1_en <= w1_en; oc_w1_r <= w1_r; oc_w1_v <= w1_v;
    oc_w2_en <= w2_en; oc_w2_r <= w2_r; oc_w2_v <= w2_v;
    w1_va <= w1_v; w2_va <= w2_v;
    // one-hot register hits for the register file's write enables: write 1 only to A0-A7 (numbers 8-15), write 2 to
    // A0-A7 (bits 0-7) or T0-T3 (bits 8-11, numbers 16-19).
    // oc_h2 tests only bit 4 for the temporaries, so the numbers 20-23 (the stack pointer banks) would land on T0-T3.
    // That never happens. Write 2 goes to the SA base register, and every uop source sets that field to A0-A7 (8-15),
    // T0, T1 or T2 (16-18): the decoder takes An from the opcode, A7, T0 or T2 (DR's override), and the sequencers and
    // the full-format EA unit use A7, An, T0, T1 or T2. The banks are only ever named in AREG and WREG (MOVEC and
    // MOVE USP). EX's ex_h2 also tests bits 3-2; both give the same hits for every number that occurs.
    oc_h1 <= {8{w1_en && w1_r[4:3] == 2'b01}} & (8'd1 << w1_r[2:0]);
    oc_h2 <= {12{w2_en}} & (w2_r[4] ? (12'd256 << w2_r[1:0]) : w2_r[3] ? (12'd1 << w2_r[2:0]) : 12'd0);
  end
  // lane B: AG -> OC1 -> OC2 (it stays in OC1 with lane A)
  if (rst || rs_d) oc_ubv <= 1'b0; else if (!st_u[0]) oc_ubv <= ag_go && n_ubv;
  for (ou_i = 0; ou_i < `UW; ou_i = ou_i + 1) if (!st_u[(ou_i * 4) / `UW]) oc_ub[ou_i] <= n_ub[ou_i];
  q_ubv <= !rst && !rs_d && oc_ubv && oc_v && !oc_stay; q_ub <= oc_ub;
  // OC1 -> OC2 (OC2 never stays, so its registers load every clock)
  q_v <= !rst && !rs_d && oc_v && !oc_stay;
  q_u <= oc_u; q_addr <= oc_addr; q_saddr <= oc_saddr;
  q_gs <= gsel(oc_u[`F_LSZ], oc_addr[1:0], xx);                          // the byte selects for OC2
  a_mem <= (oc_u[`F_ASEL] == `AS_MEM); a_reg <= (oc_u[`F_ASEL] == `AS_REG);   // the one-hot operand selects
  a_imm <= (oc_u[`F_ASEL] != `AS_MEM) && (oc_u[`F_ASEL] != `AS_REG);
  b_mem <= (oc_u[`F_BSEL] == `BS_MEM); b_imm <= (oc_u[`F_BSEL] == `BS_IMM);
  b_reg <= (oc_u[`F_BSEL] != `BS_MEM) && (oc_u[`F_BSEL] != `BS_IMM);
  imm_a <= (oc_u[`F_UNIT] == `UN_DBR) ? 32'd1 : oc_u[`F_IMM];   // DBcc subtracts 1 from the counter
  q_w1_en <= oc_w1_en; q_w1_r <= oc_w1_r; q_w1_v <= oc_w1_v;
  q_w2_en <= oc_w2_en; q_w2_r <= oc_w2_r; q_w2_v <= oc_w2_v;
  q_dq <= dqm; q_xh <= x_hi; q_xx <= xx; q_rep1 <= o1_rep; q_miss <= o1_miss; q_pm <= o1_pm; q_pw <= {4{oc_u[`F_PRET] && f_wb}} & sbe; q_sd <= sd; q_stx <= st_x;
  // the store hits: the first word's from the first clock of a split, the second word's from phase B (or phase A when
  // the load's second word is the store's)
  q_sth <= xx ? x_sth0 : st_hit1; q_sth2 <= (x_b && ld_x) ? x_sth1 : ct_hit; q_a1 <= ld_w1;
end

endmodule
`default_nettype wire
