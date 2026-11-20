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
// ==== p68_core.v: the pipe68k core, and a map of the whole pipeline ==================================================
//
// pipe68k is a pipelined 68020-compatible CPU that can run two instructions per clock, with a 68881/68882-compatible
// FPU, written in Verilog for the Gowin GW5AST-138C FPGA on the Sipeed Tang Mega 138K Pro board. It was written from
// Motorola's M68000 Family Programmer's Reference Manual only. Its first user is an Amiga 1200 system, where it runs at
// 66.7 MHz with an average of 1.031 clocks per instruction on the bench's performance test (CPI; below 1.0 would mean
// more than one instruction per clock on average, and the integer loop of that test does get below 1).
//
// This file is the top of the core. It has almost no logic of its own: it declares the nets between the pipeline
// stages and connects the stage modules, plus a handful of registers that sit between stages (the delayed redirect, the
// flush target, a few register copies). So it is the best place to get a map of the machine before reading any unit.
// The words used for the stages and units below are the ones every other file uses.
//
// ---- The stages, front to back ----------------------------------------------------------------------------------
//
// A pipeline works on several instructions at once, each in a different stage, the way a car factory has a car at every
// station of the assembly line. Each stage does its part in one clock and hands the result to the next stage through a
// register. pipe68k has these stages (one clock each unless noted):
//
//   IF   fetch, p68_fetch.v (two clocks: the I-cache read address, then the block). Reads 8 bytes per clock from the
//        8 KB instruction cache (p68_icache.v) into a 14-word queue. Each word arrives with a predecode (its length and
//        class as if it were an opcode, computed when the cache line was filled, p68_pdrom.v and p68_pdf.vh), so fetch
//        can cut whole instructions out of the queue without decoding them. The branch target cache (BTC, 256 entries)
//        is read together with the I-cache and redirects fetch to a predicted branch target. The extractor writes up to
//        two whole instructions per clock into DS's instruction buffer.
//   DS   decode, p68_decode.v. A 16-entry instruction buffer (p68_ib.vh) whose two oldest entries E0 and E1 are
//        decoded every clock by the group decoders (dec_*.v), one per instruction family, with their uops ORed
//        together. Instructions that need several uops get them from the sequencers (p68_seq.v for most, p68_sseq.v
//        for system instructions, p68_xseq.v for exception stack frames, p68_fx.v for the 68020's full-format
//        addressing modes). DS also keeps the return stack that predicts RTS, and decides whether E0 and E1 may be
//        issued as a pair.
//   DR   a two-entry queue at the front of p68_ag.v. DS hands it the decoder uop, the sequencer uop and a few override
//        fields separately (p68_dr.vh); DR merges them in its own clock and makes the final pairing decision. It exists
//        because the full merge did not fit into DS's clock.
//   AG   address generation, p68_ag.v. Computes the load and store addresses of a uop with LUT adders (a 3:2 compressor
//        and a carry chain for base + index * scale + displacement), updates An early for (An)+ and -(An), and resolves
//        branches that need no register (BRA, Bcc predicted taken, JMP to an absolute address) by redirecting fetch
//        (bra_ag). It also holds the change/use interlock: a uop that needs an address register that an older uop will
//        only write in WB waits here.
//   OC1  operand fetch, first half, p68_oc.v. Reads the 8 KB D-cache (p68_dcache.v), forwards store data that is not yet
//        in the cache, splits misaligned accesses into two reads (the uop stays one more clock), and redirects for
//        jumps through a register (oc_bra).
//   OC2  operand fetch, second half, also p68_oc.v. Aligns the loaded data, reads the register operands for EX from the
//        register file (p68_regfile.v), and decides replays (a D-cache miss, a full store buffer, a snoop on the line).
//   EX   execute, p68_ex.v. The ALU, the rotator, the flags, the branch decision and the commit. A uop commits here or
//        is thrown away: when it is the last uop of an instruction, the instruction is done. EX redirects the front end
//        when a branch went the other way than predicted, or when a uop must replay. The longer units hang off EX:
//        multiply and divide (p68_md.v), bit fields (p68_bf.v), Scc/BCD/bit operations (p68_exi.v), the system unit for
//        SR, MOVEC, traps and interrupts (p68_sys.v), and the FPU (p68_fpu.v, p68_fpe.v).
//   WB   write-back, p68_wb.v. Writes the result into the register file, completes the flags, sends a store into the
//        store buffer of the memory port unit (p68_mem.v) and into the D-cache, and drives the retire port that tells the
//        test benches what each instruction did.
//
// So an instruction spends at least eight clocks between its I-cache read and its write-back, and up to eight
// instructions (more with pairs) are in flight at the same time.
//
// ---- The three problems of every pipeline, and the answers here ---------------------------------------------------
//
//   - A uop needs a result that an older uop has not written yet. Answer: forwarding. Results are passed straight from
//     the later stages (EX, WB, and AG's own address register updates) into the reads of the earlier ones, so a uop does
//     not wait for the register file. Only address registers that a uop needs in AG before an older one writes them in
//     WB cause a wait (the interlock in AG).
//   - A branch changes the PC after later instructions were already fetched. Answer: prediction. The BTC predicts taken
//     branches in fetch, the return stack predicts returns in DS, and branches without a prediction are assumed not
//     taken (or taken when they jump backward). A wrong guess redirects: everything behind the branch is flushed and
//     fetch starts again at the right address, one clock after EX's decision.
//   - A load misses the D-cache. Answer: replay instead of stall. The uop that missed does not commit; EX redirects to
//     that uop again (restarting at its sequence step) and starts the cache fill, and the replayed uop then hits the
//     line buffer. The core never stops all stages on a cache result in the same clock, because such a stall fans out
//     to every stage; in the first pipeline sample that alone cost 13 MHz.
//
// A rule that follows from the last point and runs through the whole core: anything that steers the pipeline (a
// redirect, a flush, a stall, a clock enable with high fanout, an exception) comes from a register through very few
// LUT levels. Many registers in this file and in the units exist only for that reason, and the comments say so where
// it is the case.
//
// ---- Dual issue ---------------------------------------------------------------------------------------------------
//
// Two instructions per clock is the main goal of the project, so it runs through every stage. The model is the 68060:
// lane A (the normal pipe) does everything, lane B only simple single-uop register instructions (MOVEQ, register MOVEs,
// ADDQ, the ALU operations on data registers, and similar; p68_exb.v is lane B's ALU), and a pair has at most one memory
// operand. DS issues E1 together with E0 when the predecode says it fits; DR makes the final decision. Kinds of pairs:
//   - plain pair: E0 in lane A, E1 (lane-B class) in lane B.
//   - swapped pair: a memory instruction behind a simple head runs in lane A, the head in lane B as the older one.
//   - branch fold pair: a BTC-predicted branch issues with the first instruction at its target, which runs in lane B, so
//     the branch costs no clock.
//   - skip pair: a forward Bcc over exactly the next instruction runs together with it and never redirects; when the
//     branch is taken, lane B simply writes nothing.
// Lane B travels AG, OC1, OC2, EX and WB beside lane A with its own uop (n_ub, oc_ub, q_ub, exb_u), reads its operands
// in OC2 (rd_c, rd_d), and writes back through a second write port (wb2_*). It never stores, never updates an address
// register in AG and never changes system state. On the bench about 10 % of all instructions run in lane B. Going from
// single issue to dual issue took the core from 33.5 to 70.5 MIPS (clock divided by CPI) on core-alone builds.
//
// ---- How the core got here (short) ---------------------------------------------------------------------------------
//
// A one-file feasibility sample (rtl/sample/p68_core.v) came first; it is the best first read for the pipeline idea.
// The first real core split it into modules with the common uop format of p68_uop.vh and six stages (IF, DS, AG, OC, EX,
// WB). A first timing round put registers in front of everything that steers; a second round added DR and split OC in
// two, and completed the flags in WB. Then dual issue was built step by step (CPI 2.017 to 1.007), then the instruction
// buffer, then many rounds of timing work aimed at the full Amiga system, which is where the clock is judged. On this
// chip the wires cost more than the logic (about 85 % of a slow path is routing; a LUT is 0.26-0.52 ns, a net between
// two big units 1.3-3.1 ns), which explains most of the unusual-looking code: register copies, one-hot selects, and
// decisions computed a clock early. docs/learn/HISTORY.md tells the whole story with the measured numbers.
//
// ---- This file ------------------------------------------------------------------------------------------------------
//
// One clock (clk, the CPU clock) and one synchronous reset (rst). The ports are the memory port (one transaction at a
// time, 64-bit read beats, 32-bit writes; docs/MEMORY_PORT.md has the protocol), the snoop input for DMA writes, the uncached
// area maps, the interrupt and system pins, and the retire port. The retire port (ret_*) is for simulation only: the
// lockstep bench compares every retired instruction with a reference CPU through it, and a hardware build leaves it
// open, so it costs no logic there. obs_a, obs_b and obs_ccr are only there to give the timing top (p68_top.v)
// something to observe.
`include "p68_uop.vh"                // uop format
`include "p68_ib.vh"                 // instruction buffer entry (for IBW)
`include "p68_sys.vh"                // system unit codes (SY_MCW, CR_CACR)
`default_nettype none
module p68_core (
  input  wire        clk,          // CPU clock
  input  wire        rst,          // synchronous reset, active high
  // ---- memory port (docs/MEMORY_PORT.md) ----
  output wire        m_req,        // one clock: a transaction starts; the fields stay valid until m_done
  output wire        m_wr,         // write (else read)
  output wire [31:2] m_addr,       // reads: line aligned (2 or 4 beats), uncached: the word; writes: the word
  output wire [1:0]  m_len,        // read beats - 1: 0 uncached word, 1 D-cache line (16 bytes), 3 I-cache line (32 bytes)
  output wire        m_unc,        // uncached: exactly this access, bytes m_be
  output wire        m_ins,        // instruction fetch
  output wire [3:0]  m_be,         // byte enables (bit 3 = address +0)
  output wire [31:0] m_wdata,      // write data
  input  wire        m_rdy,        // a read beat on m_rdata (ascending addresses)
  input  wire [63:0] m_rdata,      // big-endian: byte +0 in [63:56]
  input  wire        m_done,       // the transaction ends (reads: with the last beat)
  input  wire        m_snoop,      // external write (chip-bus DMA) to m_snoop_addr: invalidate I and D lines
  input  wire [31:2] m_snoop_addr,
  input  wire [15:0] unc_map,      // bit k: data at A23-A20 = k is uncached (chip RAM, I/O), in the 24-bit area only
  input  wire [7:0]  unc_hi,       // data at A31-A25 = unc_hi[6:0] is uncached when unc_hi[7] (a 32 MB board, e.g. RTG)
  input  wire [6:0]  cac_hi,       // cac_hi[6]: data above 16 MB (A31-A24 not 00 or FF) is cached only in the
                                   // cac_hi[2:0] x 64 MB at A31-A29 = cac_hi[5:3] (a RAM board); elsewhere uncached (no memory
                                   // there: a probe's stores must not stay in the D-cache). 0: everything up there cached
                                   // (an AROS memory probe of empty space once read its own stores back from the D-cache,
                                   // added memory that did not exist and overwrote ExecBase; this input fixed that)
  input  wire        slow,         // CPU speed mode: each instruction takes at least its 68EC020 cycle count (cost_f in p68_decode), no dual issue (a register of the adapter)
  output wire [31:0] obs_a,        // observation outputs for the timing top only
  output wire [31:0] obs_b,
  output wire [4:0]  obs_ccr,
  // retire port (simulation): see p68_wb
  output wire        ret_u,        // a uop committed (its writes below are valid)
  output wire        ret_v,        // an instruction retired (its last uop)
  output wire [31:0] ret_pc,       // its address
  output wire        ret_wen,      // register write from WB
  output wire [3:0]  ret_wreg,     // ... register (D0-D7, A0-A7)
  output wire [31:0] ret_wval,     // ... value
  output wire        ret_ws_en,    // An update from AG, load EA ((An)+, -(An))
  output wire [2:0]  ret_ws_r,
  output wire [31:0] ret_ws_v,
  output wire        ret_wd_en,    // An update from AG, store EA or LEA
  output wire [2:0]  ret_wd_r,
  output wire [31:0] ret_wd_v,
  output wire        ret_st,       // store
  output wire [31:0] ret_saddr,
  output wire [31:0] ret_sdata,
  output wire [3:0]  ret_sbe,
  output wire [4:0]  ret_ccr,      // CCR after the instruction
  // ---- interrupts and system pins ----
  input  wire [2:0]  ipl,          // interrupt request level (0 none, 7 NMI), synchronous to clk
  output wire        iack,         // one clock: interrupt taken at level iack_lvl (autovector 24 + level)
  output wire [2:0]  iack_lvl,
  output wire        reset_o,      // RESET instruction: 512 clocks
  output wire        stopped,      // STOP: waiting for an interrupt
  // ---- retire port, system state (simulation) ----
  output wire [15:0] ret_sr,       // SR after the instruction
  output wire        ret_irq,      // the retired "instruction" is an interrupt entry (ret_pc = stacked PC)
  output wire        ret_bk_en,    // WB wrote stack pointer bank ret_bk_r (0 USP, 1 ISP, 2 MSP)
  output wire [1:0]  ret_bk_r,
  output wire        ret_sw,       // A7 swap: A7 saved into bank ret_sw_old, A7 = ret_sw_v (bank ret_sw_new)
  output wire [1:0]  ret_sw_old,
  output wire [1:0]  ret_sw_new,
  output wire [31:0] ret_sw_v,
  output wire [31:0] ret_vbr,      // control registers after the instruction
  output wire [31:0] ret_caar,
  output wire [2:0]  ret_sfc,
  output wire [2:0]  ret_dfc,
  output wire [3:0]  ret_cacr,     // CACR bits 3-0 (C, CE, F, E); also used inside the core to control the caches
  // ---- bus lock and misaligned stores ----
  output wire        lock,         // CAS/CAS2 in progress: the bus must not be given to another master (A1200 adapter)
  output wire        ret_st2,      // second word of a misaligned store (simulation), valid with ret_st
  output wire [31:0] ret_saddr2,
  output wire [31:0] ret_sdata2,
  output wire [3:0]  ret_sbe2,
  // ---- prediction events (simulation): {Bcc/BRA committed, predicted taken, taken, BTC-predicted,
  //      return jump committed, return mispredicted}
  output wire [5:0]  ret_bp,
  // ---- second retire record (lane B of a pair; simulation). ret2_first: it is the older of the two.
  //      Lane B never stores, updates no An in AG and changes no system state. ret_ccr / ret_sr hold lane A's CCR. ----
  output wire        ret2_v,
  output wire        ret2_first,
  output wire [31:0] ret2_pc,
  output wire        ret2_wen,
  output wire [3:0]  ret2_wreg,
  output wire [31:0] ret2_wval,
  output wire [4:0]  ret2_ccr,
  // ---- FPU state (simulation) ----
  output wire [79:0] ret_fp0, ret_fp1, ret_fp2, ret_fp3, ret_fp4, ret_fp5, ret_fp6, ret_fp7,
  output wire [31:0] ret_fpcr, ret_fpsr, ret_fpiar
) /* synthesis syn_maxfan = 16 */;
// (the attribute asks the synthesizer to keep every net of this module at a fanout of 16 or below)
wire        fpu_null, fpu_en;        // FPU state DS needs: the committed NULL state; an FPCR exception enable is set
wire [5:0]  fpu_pv;                  // pending FPU trap vector (committed)

// ---- nets between the stages ----------------------------------------------------------------------------------
wire        stall_dr;                // DR is full: DS must not hand over a uop (AG output)
wire [3:0]  sdr_q;                   // registered copies of stall_dr, one per group of DS's advance logic
// flush: the front end (fetch, DS) restarts at flush_tgt; issue: DS consumed its head instruction; ds_go: a uop left
// DS; stall_ag: AG holds; ag_go: a uop leaves AG; bra_ag: AG redirects (branch without a register); ag_v: AG holds a
// uop; ds_ill: DS's head is illegal; oc_bra: OC1 redirects (jump through a register)
wire        flush, issue, ds_go, stall_ag, ag_go, bra_ag, ag_v, ds_ill, oc_bra;
wire [31:0] ag_tgt;                  // AG's branch target (for bra_ag and for a predicted branch leaving AG)
wire        ag_mv;                   // AG moves without a redirect in EX
wire [31:0] flush_tgt;               // the address the front end restarts at
wire [4:0]  fcnt;                    // fetch queue words from its head (for p68_fx)
// instruction buffer between fetch's extractor and DS
wire        ib_ex1, ib_ex2, ib_sp1, ib_sp2, ib_pop, ib_popa, ib_popb, ib_hiss, ib_hpf; wire [`IBW-1:0] ib_xe0, ib_xe1;   // see p68_fetch.v ports
wire [`UW-1:0] n_u, oc_u, ex_u;      // lane A's uop leaving AG (to OC1), in OC1, in EX
wire [`UW-1:0] o_dec, o_seq; wire o_act; wire [95:0] o_ov;   // DS -> DR in parts (p68_dr.vh)
// pairs: o_p2 pair candidate, o_sw swapped pair, q_rpb replay includes the older lane B, g_o lane B in WB is the
// older, ex_bb the committing branch is lane B's, ccr_b lane B's CCR,
// o_u2 the second instruction's uop
wire o_p2, o_sw, q_rpb, g_o, ex_bb; wire [4:0] ccr_b; wire [`UW-1:0] o_u2;   // pairs
wire [31:0] if2_pc, fpc;             // fetch: address of the block read last clock; the read address now
wire [31:0] ea_s, saddr;             // AG: load address, store address
wire [1:0]  sa_d;                        // AG's store address bits 1-0, for OC's split decode
wire        w1_en, w2_en; wire [4:0] w1_r, w2_r; wire [31:0] w1_v, w2_v;   // AG's two address register writes
wire [4:0]  ag_rl, ag_rs; wire [3:0] ag_rx; wire [31:0] ag_vl, ag_vs, ag_vx;   // AG's early register reads (base, base, index)
wire        oc_v; wire [31:0] oc_addr, oc_saddr;   // OC1 holds a uop; its load address and store address
// lane B (dual issue)
wire [`UW-1:0] n_ub, oc_ub, q_ub, exb_u; wire n_ubv, oc_ubv, q_ubv, exb_v;   // lane B's uop and valid at AG out, OC1, OC2, EX
wire [4:0]  rd_c, rd_d; wire [31:0] rv_c, rv_d, opb_a, opb_b;   // lane B's register reads in OC2, their values, its operands
wire        ex_vg, ex_wug, ex_cwg, exb_wug; wire [4:0] ex_wrg; wire [3:0] exb_wrg;   // register copies for AG's interlock
// WB lane B write; lane A's CCR; one-hot register hits (oc_h1/oc_h2: AG writes in OC; wb_h/wb_hd, wb2_h/wb2_hd: WB lanes)
wire        wb2_wen; wire [3:0] wb2_wreg; wire [31:0] wb2_val, wb2_vg; wire [4:0] ccr_a; wire [7:0] oc_h1, wb_hd, wb2_h, wb2_hd; wire [11:0] oc_h2, wb_h;
// OC2 (second OC stage) to EX
wire        q_v; wire [`UW-1:0] q_u; wire [31:0] q_addr, q_saddr;   // OC2 holds a uop; the uop; its load and store address
wire        q_w1_en, q_w2_en; wire [4:0] q_w1_r, q_w2_r; wire [31:0] q_w1_v, q_w2_v;   // the AG writes of the uop in OC2
wire        oc_vrf;                    // OC's copy of oc_v for the register file
wire        oc_w1_en, oc_w2_en; wire [4:0] oc_w1_r, oc_w2_r; wire [31:0] oc_w1_v, oc_w2_v, oc_w1_va, oc_w2_va;   // AG writes of the uop in OC1 (and copies for AG)
wire [31:0] op_a, op_b; wire st_hit, oc_rep, oc_miss;   // EX's operands; store hits the D-cache; replay; D-cache miss
wire [4:0]  oc_ra, oc_rb; wire [31:0] oc_va, oc_vb;     // OC2's register reads and values
// EX: holds a uop, D-cache hit, replay, miss, register write, store, commit, redirect (and its copy for the restore)
wire        ex_v, ex_hit, ex_rep, ex_miss, ex_wen, ex_st, ex_commit, redirect_ex, redirect_rc;
wire [31:0] ex_saddr, ex_laddr, rm, ex_target; wire [31:2] ex_laddr1;   // EX: store and load address, result, redirect target, next word
wire [4:0]  ex_wreg, ccr;            // EX's destination register; the CCR after the uop in WB
wire        ex_w1_en, ex_w2_en; wire [4:0] ex_w1_r, ex_w2_r; wire [31:0] ex_w1_v, ex_w2_v; wire [7:0] ex_h1; wire [11:0] ex_h2;   // AG writes of the uop in EX
// WB: register write, store, store hit, line store; destination, value, store address (and copy), store data
wire        wb_wen, wb_st, wb_hit, ls_v; wire [4:0] wb_wreg; wire [31:0] wb_val, wb_saddr, wb_sac, sd, ls_data;
wire [3:0]  sbe, ls_be; wire [31:2] ls_addr;   // store byte enables; the line store into the D-cache
// supervisor mode and exceptions
wire        sup, msp, rst_done, no_ret, rs_one, xs_go, xs_rs, sw_en;   // S, M, reset done, no retire, RTE restart, exception sequence start/restart, A7 swap
wire [154:0] xs_par;                 // parameters of the exception sequence (vector, PC, SR, frame)
wire [4:0]  xs_rs_step;              // its restart step
wire [1:0]  sw_old, sw_new;          // A7 swap banks
wire [31:0] sw_v;                    // the new A7
// full-format EAs and misaligned accesses
wire [15:0] wx; wire [3:0] xpos, il; wire il_ov, stall_oc, st_hit2, ex_hit2; wire [31:0] oc_laddr;

// ---- memory unit, cache control ----
wire        iwant;                   // an unmasked interrupt is requested (p68_sys register): AG forms no pairs
// ex_irq: an interrupt is taken in front of the uop in EX; unc_hold: an uncached read is pending (no interrupt take);
// smc: a committing store hit fetched code; dr_*/ir_*: D and I read requests and grants; r_*: read beats
wire        ex_irq, unc_hold, smc, dr_req, dr_unc, dr_go, ir_req, ir_go, r_d, r_i, r_done;
wire [31:2] dr_addr, ir_addr;
wire [3:0]  dr_be;
wire [63:0] r_data;
wire [2:0]  sb_cnt;                  // store buffer entries
reg         cw_d;                    // MOVEC to CACR committed in the last clock (ret_cacr holds the new value)
always @(posedge clk) cw_d <= !rst && ex_commit && ex_u[`F_UNIT] == `UN_SYS && ex_u[`F_OP] == `SY_MCW &&
                             ex_u[`U_IMM +: 3] == `CR_CACR;
wire        clr_all = cw_d && ret_cacr[3];   // the new CACR has C set: clear both caches
wire        clr_ent = cw_d && ret_cacr[2];   // CE set: clear the entry at CAAR
wire        d_miss  = ex_v && ex_rep && ex_miss && !ex_irq;   // EX replays a D-cache miss: start the fill at ex_laddr
// branch prediction: DS <-> IF, BTC update from EX (registered)
// ag_pt: a predicted branch leaves AG; bt_kill: DS clears a wrong BTC entry; dfl: DS
// predicted a return; oc_pmiss: OC found the predicted target wrong; ex_brc/ex_brt: a Bcc/BRA commits / was taken
wire        ag_pt, bt_kill, dfl, oc_pmiss, ex_brc, ex_brt;
wire [31:0] pt, dtgt, ex_btgt;       // BTC's predicted target; return target; the committed branch's target
// The BTC update, registered once more after EX so the write into the BTC block RAM starts from registers. A
// committed branch of either lane updates it (ex_bb: the branch was lane B's).
reg         bu_v, bu_tk, bu_pb; reg [31:0] bu_pc, bu_tgt; reg [3:0] bu_len;
always @(posedge clk) begin
  bu_v <= !rst && ex_brc; bu_tk <= ex_brt; bu_pb <= ex_bb ? exb_u[`F_PBTC] : ex_u[`F_PBTC];          // lane B or lane A
  bu_pc <= ex_bb ? exb_u[`F_PC] : ex_u[`F_PC]; bu_len <= ex_bb ? exb_u[`F_LEN] : ex_u[`F_LEN]; bu_tgt <= ex_btgt;
end
// The memory port unit: store buffer, D reads, I reads, one transaction at a time.
p68_mem i_mem (
  .clk(clk), .rst(rst), .unc_map(unc_map), .unc_hi(unc_hi), .cac_hi(cac_hi),
  .sb_we(wb_st), .sb_addr(wb_saddr[31:2]), .sb_data(sd), .sb_be(sbe), .sb_cnt(sb_cnt),
  .dr_req(dr_req), .dr_addr(dr_addr), .dr_unc(dr_unc), .dr_be(dr_be), .ir_req(ir_req), .ir_addr(ir_addr),
  .dr_go(dr_go), .ir_go(ir_go), .r_d(r_d), .r_i(r_i), .r_data(r_data), .r_done(r_done),
  .m_req(m_req), .m_wr(m_wr), .m_addr(m_addr), .m_len(m_len), .m_unc(m_unc), .m_ins(m_ins), .m_be(m_be),
  .m_wdata(m_wdata), .m_rdy(m_rdy), .m_rdata(m_rdata), .m_done(m_done));

// ---- redirect, restore and flush ----
// The register file restores one clock after the redirect (AG, OC and EX are empty then; AG's early reads take the
// restore values, see p68_regfile.v). rs_d is EX's redirect one clock later, as a register.
reg         rs_d;
always @(posedge clk) rs_d <= !rst && redirect_rc;
// p68_fx's flush from its own preserved copy of rs_d, apart from DS's flush net (about 240 loads)
reg         rs_dfx /* synthesis syn_preserve = 1 */;
always @(posedge clk) rs_dfx <= !rst && redirect_rc;
// OC2's register reads compare their numbers with WB's write registers one clock early: oc_u is the uop that will be in
// OC2 next clock, and EX's write registers now are WB's next clock (both lanes); the register file ANDs in the write
// enables. Taking the compare out of the read clock shortens the OC2-to-EX paths.
reg  [3:0]  oc_eq;                                                     // {B = WB lane B, B = WB, A = WB lane B, A = WB}
always @(posedge clk) oc_eq <= {oc_u[`F_BREG] == {1'b0, exb_u[`U_WREG +: 4]}, oc_u[`F_BREG] == ex_wreg,
                                oc_u[`F_AREG] == {1'b0, exb_u[`U_WREG +: 4]}, oc_u[`F_AREG] == ex_wreg};
// OC2's register read numbers as one-hots, registered from OC1's uop, so the read is an AND-OR. Before, the register
// number passed a 23:1 select behind OC2's uop register, and paths from there through op_a into EX failed in the full
// system.
reg  [22:0] oc_oa, oc_ob;
always @(posedge clk) begin oc_oa <= 23'd1 << oc_u[`F_AREG]; oc_ob <= 23'd1 << oc_u[`F_BREG]; end
reg  [11:0] rs_dc /* synthesis syn_preserve = 1 */;                    // copies of rs_d for the register file's restore (one per A/T register)
always @(posedge clk) rs_dc <= {12{!rst && redirect_rc}};
reg         smc_r;                   // the store-into-code hit as a register; EX replays the uop behind the store
always @(posedge clk) smc_r <= !rst && smc;
// The front end (fetch, DS, AG valid) follows an EX redirect one clock later (rs_d, target in tgt_r); AG and the uop
// issued in the redirect clock are killed by rs_d (ag_go). That clock costs nothing, because every flush now reads its
// target from a register in the flush clock (when this was first done, the front end followed one clock later without
// the registered target, and CPI rose from 2.029 to 2.150 until the registered targets won the clock back).
reg  [31:0] tgt_r;
always @(posedge clk) tgt_r <= ex_target;
assign flush     = rs_d || bra_ag || oc_bra;                                   // EX (delayed), AG or OC redirects
wire   xflush    = rs_d || bra_ag || oc_bra;                                   // the same, for the exception sequencer (its start is registered)
wire   flush_fx  = rs_dfx || bra_ag || oc_bra;                                 // the same from p68_fx's own copy
assign flush_tgt = rs_d ? tgt_r : oc_bra ? oc_addr : ag_tgt;                   // the oldest redirect wins (EX, then OC, then AG)
// the flush target's next fetch block, one adder per source (all registers), selected last
wire [28:0] tgt_r8 = tgt_r[31:3] + 29'd1, oca8 = oc_addr[31:3] + 29'd1, agt8 = ag_tgt[31:3] + 29'd1;
wire [28:0] flush_t8 = rs_d ? tgt_r8 : oc_bra ? oca8 : agt8;
// The exception sequencer starts one clock after the take (its parameters registered without an enable), together with
// the delayed flush; it issues in the same clock as before (DS does not advance in the flush clock).
reg         xs_go_r, xs_rs_r; reg [154:0] xs_par_r; reg [4:0] xs_rs_step_r;
always @(posedge clk) begin
  xs_go_r <= !rst && xs_go; xs_rs_r <= !rst && xs_rs; xs_par_r <= xs_par; xs_rs_step_r <= xs_rs_step;
end

// ---- IF: fetch, with the I-cache and the branch target cache inside ----
p68_fetch i_fetch (
  .clk(clk), .rst(rst), .flush(flush), .flush_tgt(flush_tgt),
  .ffl(flush), .fft(flush_tgt), .fft8i(flush_t8),                                                                          // every flush target is a register: fetch reads it in the flush clock
  .cacr(ret_cacr), .clr_all(clr_all), .clr_ent(clr_ent), .caar(ret_caar),                                // cache control
  .oc_saddr(q_saddr), .ex_stc(ex_v && ex_u[`F_SA_EN]),   /* store check without the commit (an uncommitted store only costs a check) */   /* and without the CAS condition (a CAS that does not store only costs a check) */            /* the store tag copy is read at OC2's store */ .ex_saddr(ex_saddr), .wb_st(wb_st), .wb_saddr(wb_sac), .smc(smc),   /* wb_sac: WB's register copy of the store address */
  .wb_x2(ret_st2), .wb_saddr2(ret_saddr2),                                                                 // second word of a misaligned store
  .sn_v(m_snoop), .sn_addr(m_snoop_addr),                                                                  // DMA snoop
  .ir_req(ir_req), .ir_addr(ir_addr), .ir_go(ir_go), .r_i(r_i), .r_data(r_data), .r_done(r_done),          // memory unit
  .dfl(dfl), .dtgt(dtgt), .bt_kill(bt_kill), .bu_v(bu_v), .bu_tk(bu_tk), .bu_pb(bu_pb), .bu_pc(bu_pc), .bu_len(bu_len),   // predictions
  .bu_tgt(bu_tgt), .pt(pt),
  .if2_pc(if2_pc), .fpc(fpc),
  .ib_sp1(ib_sp1), .ib_sp2(ib_sp2), .ex1(ib_ex1), .ex2(ib_ex2), .xe0(ib_xe0), .xe1(ib_xe1), .pop(ib_pop), .popa(ib_popa), .popb(ib_popb), .hiss(ib_hiss),   // instruction buffer
  .hpf(ib_hpf), .fcnt(fcnt), .xpos(xpos), .wx(wx), .il_ov(il_ov), .il(il));

// ---- DS: decode, sequencers, return stack ----
p68_decode i_ds (
  .clk(clk), .rst(rst),
  .ex1(ib_ex1), .ex2(ib_ex2), .xe0(ib_xe0), .xe1(ib_xe1), .ib_sp1(ib_sp1), .ib_sp2(ib_sp2), .pop(ib_pop), .popa(ib_popa), .popb(ib_popb), .hiss(ib_hiss),   // instruction buffer
  .hpf(ib_hpf), .fcnt(fcnt), .slow(slow),
  .stall_ag(sdr_q), .flush(flush), .flush_fx(flush_fx), .flush_tgt(flush_tgt),                                              // DR full (register copies), flushes
  .redirect_ex(redirect_ex), .ex_rep(ex_rep), .ex_step(ex_u[`F_STEP]), .rs_d(rs_d), .xflush(xflush),    // replay restart
  .ds_go(ds_go), .o_dec(o_dec), .o_seq(o_seq), .o_act(o_act), .o_ov(o_ov), .issue(issue),
  .o_sw(o_sw), .o_u2(o_u2), .o_p2(o_p2), .ds_ill(ds_ill),   // pairs
  .sup(sup), .msp(msp), .stopped(stopped), .rst_done(rst_done), .rs_one(rs_one), .ex_xs(ex_u[`U_EXC + 6] | ex_u[`U_EXC + 7]),
  .xs_go(xs_go_r), .xs_par(xs_par_r), .xs_rs(xs_rs_r), .xs_rs_step(xs_rs_step_r),                         // exception sequence (registered)
  .wx(wx), .xpos(xpos), .il_ov(il_ov), .il(il), .ex_fxm(ex_u[`U_FXM]),          // full-format EAs
  .pt(pt), .ag_pt(ag_pt), .bt_kill(bt_kill), .dfl(dfl), .dtgt(dtgt),      // branch prediction
  .ag_tgt(ag_tgt), .fpu_null(fpu_null), .fpu_en(fpu_en), .fpu_pv(fpu_pv));                                                                   // AG target, FPU state

// ---- DR and AG: merge, pairing, address generation, interlock ----
p68_ag i_ag (
  .clk(clk), .rst(rst), .ds_go(ds_go), .o_dec(o_dec), .o_seq(o_seq), .o_act(o_act), .o_ov(o_ov), .o_u2(o_u2), .o_p2(o_p2), .o_sw(o_sw), .ds_ill(ds_ill), .iwant(iwant), .flush(flush), .rs_d(rs_d),
  .oc_v(oc_v), .oc_wen(oc_u[`F_WEN]), .oc_wreg(oc_u[`F_WREG]),
  .q_v(q_v), .q_wen(q_u[`F_WEN]), .q_wreg(q_u[`F_WREG]),                                                  // OC2's pending write
  .oc_s7(oc_v && oc_u[`F_SA_EN] && oc_u[`F_SA_BREG] == 5'd15), .q_s7(q_v && q_u[`F_SA_EN] && q_u[`F_SA_BREG] == 5'd15),   // stores through A7 in OC1 / OC2
  .n_ub(n_ub), .n_ubv(n_ubv), .ocb_wen(oc_ubv && oc_ub[`F_WEN]), .ocb_wreg(oc_ub[`U_WREG +: 4]),
  .qb_wen(q_ubv && q_ub[`F_WEN]), .qb_wreg(q_ub[`U_WREG +: 4]), .exb_wu(exb_wug), .exb_wreg(exb_wrg),   /* lane B's pending writes (EX's register copies) */
  .wb_valb(wb2_vg),                                                                                        /* lane B's WB value: EX's register copy */
  .oc_w1_v(oc_w1_va), .oc_w2_v(oc_w2_va),                                                                 // OC1's AG write values, OC's copies for AG
  .ex_rep(ex_rep), .wb_val(wb_val), .stall_oc(stall_oc), .wb_wa(wb_wen),
  .ex_vg(ex_vg), .ex_wug(ex_wug), .ex_wrg(ex_wrg), .ex_cw(ex_cwg),                                        // EX's register copies for the interlock
  .rd_l(ag_rl), .rd_s(ag_rs), .rd_x(ag_rx), .rv_l(ag_vl), .rv_s(ag_vs), .rv_x(ag_vx),
  .ag_v(ag_v), .stall_ag(stall_ag), .stall_dr(stall_dr), .sdr_q(sdr_q), .ag_go(ag_go), .bra_ag(bra_ag), .ag_pt(ag_pt), .ea_s(ea_s), .saddr(saddr), .sa_d(sa_d), .n_u(n_u),
  .ag_tgt(ag_tgt), .oc_bra(oc_bra), .ag_mv(ag_mv),                                                                     // redirects
  .w1_en(w1_en), .w1_r(w1_r), .w1_v(w1_v), .w2_en(w2_en), .w2_r(w2_r), .w2_v(w2_v));

// ---- register file ----
p68_regfile i_rf (
  .clk(clk), .wb_wen(wb_wen), .wb_wreg(wb_wreg), .wb_val(wb_val),
  .wb2_wen(wb2_wen), .wb2_wreg(wb2_wreg), .wb2_val(wb2_val), .oc_rc(rd_c), .oc_rd(rd_d), .oc_vc(rv_c), .oc_vd(rv_d),   // lane B write and reads
  // AG's register writes enter the file one clock later, from OC's registers (the AG adder no longer drives the
  // register file); a uop that stays in OC (split access) writes the same values again
  .p1_h(oc_h1), .p2_h(oc_h2), .wb_h(wb_h), .wb_hd(wb_hd), .wb2_h(wb2_h), .wb2_hd(wb2_hd),
  .p1_en(oc_vrf && oc_w1_en), .p1_r(oc_w1_r), .p1_v(oc_w1_v), .p2_en(oc_vrf && oc_w2_en), .p2_r(oc_w2_r), .p2_v(oc_w2_v),
  .c1_en(ex_commit && ex_w1_en), .c1_r(ex_w1_r), .c1_v(ex_w1_v), .c_cm(ex_commit), .c1_h(ex_h1), .c2_h(ex_h2),   // commits, with one-hot registers
  .c2_en(ex_commit && ex_w2_en), .c2_r(ex_w2_r), .c2_v(ex_w2_v),
  .restore(rs_dc | {12{sw_en}}),                                                                           // restore one clock after the redirect (one copy per A/T register); also in a swap
  .ag_rl(ag_rl), .ag_rs(ag_rs), .ag_rx(ag_rx), .ag_vl(ag_vl), .ag_vs(ag_vs), .ag_vx(ag_vx),
  .oc_ra(oc_ra), .oc_rb(oc_rb), .oc_va(oc_va), .oc_vb(oc_vb), .oc_eq(oc_eq), .oc_oa(oc_oa), .oc_ob(oc_ob),   /* registered compares and one-hots from above */
  .sw_en(sw_en), .sw_old(sw_old), .sw_new(sw_new), .sw_v(sw_v));

// ---- OC1 and OC2: operand fetch, D-cache ----
p68_oc i_oc (
  .clk(clk), .rst(rst), .redirect_ex(redirect_ex), .rs_d(rs_d),                                          // OC empties one clock after the redirect
  .ag_go(ag_go), .n_u(n_u), .ea_s(ea_s), .saddr(saddr), .sa_d(sa_d),
  .w1_en(w1_en), .w2_en(w2_en), .w1_r(w1_r), .w2_r(w2_r), .w1_v(w1_v), .w2_v(w2_v),
  .cacr(ret_cacr), .clr_all(clr_all), .clr_ent(clr_ent), .caar(ret_caar), .unc_map(unc_map), .unc_hi(unc_hi), .cac_hi(cac_hi), // cache control, uncached areas
  .ex_commit(ex_commit), .d_miss(d_miss), .d_addr(ex_laddr), .d_addr1(ex_laddr1), .d_lsz(ex_u[`F_LSZ]), .sn_v(m_snoop), .sn_addr(m_snoop_addr),
  .dr_req(dr_req), .dr_addr(dr_addr), .dr_unc(dr_unc), .dr_be(dr_be), .dr_go(dr_go), .r_d(r_d), .r_data(r_data),
  .r_done(r_done), .unc_hold(unc_hold), .sb_cnt(sb_cnt), .smc(smc), .oc_pmiss(oc_pmiss), .d_fu(ex_u[`U_LOCK]),
  .wb_st(wb_st), .wb_hit(wb_hit), .wb_saddr(wb_saddr), .sd(sd), .sbe(sbe),
  .ls_v(ls_v), .ls_addr(ls_addr), .ls_data(ls_data), .ls_be(ls_be),
  .ex_v(ex_v), .ex_st(ex_u[`F_SA_EN]), .ex_saddr(ex_saddr),   /* without the CAS condition (a CAS that does not store only costs a replay or a later install) */
  .rd_a(oc_ra), .rd_b(oc_rb), .rv_a(oc_va), .rv_b(oc_vb),
  .oc_v(oc_v), .oc_vrf(oc_vrf), .oc_u(oc_u), .oc_addr(oc_addr), .oc_saddr(oc_saddr),
  .oc_w1_en(oc_w1_en), .oc_w2_en(oc_w2_en), .oc_w1_r(oc_w1_r), .oc_w2_r(oc_w2_r), .oc_w1_v(oc_w1_v), .oc_w2_v(oc_w2_v), .oc_w1_va(oc_w1_va), .oc_w2_va(oc_w2_va), .oc_h1(oc_h1), .oc_h2(oc_h2),
  .op_a(op_a), .op_b(op_b), .st_hit(st_hit), .oc_rep(oc_rep), .q_rpb(q_rpb), .oc_miss(oc_miss),
  .oc_stay(), .stall_oc(stall_oc), .oc_laddr(oc_laddr), .st_hit2(st_hit2), .oc_bra(oc_bra),
  .q_v(q_v), .q_u(q_u), .q_addr(q_addr), .q_saddr(q_saddr),                                               // OC2 stage registers
  .q_w1_en(q_w1_en), .q_w2_en(q_w2_en), .q_w1_r(q_w1_r), .q_w2_r(q_w2_r), .q_w1_v(q_w1_v), .q_w2_v(q_w2_v),
  .n_ub(n_ub), .n_ubv(n_ubv), .oc_ub(oc_ub), .q_ub(q_ub), .oc_ubv(oc_ubv), .q_ubv(q_ubv),                  // lane B
  .rd_c(rd_c), .rd_d(rd_d), .rv_c(rv_c), .rv_d(rv_d), .opb_a(opb_a), .opb_b(opb_b));

// ---- EX: execute, commit, redirect, and the units behind it (system unit, multiply/divide, bit fields, FPU) ----
p68_ex i_ex (
  // EX takes the uop from OC2
  .clk(clk), .rst(rst), .oc_v(q_v), .oc_u(q_u), .op_a(op_a), .op_b(op_b), .oc_addr(q_addr),
  .oc_saddr(q_saddr), .st_hit(st_hit), .oc_rep(oc_rep), .q_rpb(q_rpb), .oc_miss(oc_miss),
  .oc_w1_en(q_w1_en), .oc_w2_en(q_w2_en), .oc_w1_r(q_w1_r), .oc_w2_r(q_w2_r), .oc_w1_v(q_w1_v), .oc_w2_v(q_w2_v),
  .wb_val(wb_val), .oc_laddr(oc_laddr), .st_hit2(st_hit2), .ex_hit2(ex_hit2),
  .ex_v(ex_v), .ex_u(ex_u), .ex_saddr(ex_saddr), .ex_hit(ex_hit), .ex_rep(ex_rep), .ex_miss(ex_miss),
  .ex_laddr(ex_laddr), .ex_laddr1(ex_laddr1),
  .ex_w1_en(ex_w1_en), .ex_w2_en(ex_w2_en), .ex_w1_r(ex_w1_r), .ex_w2_r(ex_w2_r), .ex_w1_v(ex_w1_v), .ex_w2_v(ex_w2_v), .ex_h1(ex_h1), .ex_h2(ex_h2),
  .ex_wen(ex_wen), .ex_wreg(ex_wreg), .ex_st(ex_st), .ex_commit(ex_commit), .rm(rm), .ccr_o(ccr),
  .redirect_ex(redirect_ex), .redirect_rc(redirect_rc), .ex_target(ex_target),
  .ipl(ipl), .no_ret(no_ret), .rs_one(rs_one), .sup(sup), .msp(msp), .stopped(stopped), .rst_done(rst_done),
  .xs_go(xs_go), .xs_par(xs_par), .xs_rs(xs_rs), .xs_rs_step(xs_rs_step),
  .sw_en(sw_en), .sw_old(sw_old), .sw_new(sw_new), .iack(iack), .iack_lvl(iack_lvl), .reset_o(reset_o),
  .sr(ret_sr), .vbr(ret_vbr), .caar(ret_caar), .sfc(ret_sfc), .dfc(ret_dfc), .cacr(ret_cacr),
  .irq_hold(unc_hold), .ex_irq(ex_irq), .iwant(iwant), .oc_pmiss(oc_pmiss), .ex_brc(ex_brc), .ex_brt(ex_brt), .ex_btgt(ex_btgt),
  .q_ub(q_ub), .q_ubv(q_ubv), .opb_a(opb_a), .opb_b(opb_b), .exb_v(exb_v), .exb_u(exb_u), .smc_r(smc_r),   // lane B; smc as a register
  .wb2_wen(wb2_wen), .wb2_wreg(wb2_wreg), .wb2_val(wb2_val), .wb2_h(wb2_h), .wb2_hd(wb2_hd), .ccr_a(ccr_a), .ccr_b(ccr_b), .g_o(g_o), .ex_bb(ex_bb), .wb2_vg(wb2_vg), .rs_d(rs_d), .ex_vg(ex_vg), .ex_wug(ex_wug), .ex_wrg(ex_wrg), .ex_cwg(ex_cwg), .exb_wug(exb_wug), .exb_wrg(exb_wrg), .ret2_v(ret2_v), .ret2_pc(ret2_pc),
  .fpu_null(fpu_null), .fpu_en(fpu_en), .fpu_pv(fpu_pv), .ret_fp0(ret_fp0), .ret_fp1(ret_fp1), .ret_fp2(ret_fp2), .ret_fp3(ret_fp3), .ret_fp4(ret_fp4),
  .ret_fp5(ret_fp5), .ret_fp6(ret_fp6), .ret_fp7(ret_fp7), .ret_fpcr(ret_fpcr), .ret_fpsr(ret_fpsr), .ret_fpiar(ret_fpiar));   // FPU state for the retire port

// ---- WB: write-back, stores, retire port ----
p68_wb i_wb (
  .clk(clk), .rst(rst), .ex_commit(ex_commit), .ex_u(ex_u), .ex_wen(ex_wen), .ex_wreg(ex_wreg), .ex_st(ex_st),
  .ex_saddr(ex_saddr), .ex_hit(ex_hit), .rm(rm),
  .ex_w1_en(ex_w1_en), .ex_w2_en(ex_w2_en), .ex_w1_r(ex_w1_r), .ex_w2_r(ex_w2_r), .ex_w1_v(ex_w1_v), .ex_w2_v(ex_w2_v),
  .ccr(ccr_a),                                                                                             // lane A's CCR
  .wb_wen(wb_wen), .wb_wreg(wb_wreg), .wb_val(wb_val), .wb_h(wb_h), .wb_hd(wb_hd), .wb_st(wb_st), .wb_saddr(wb_saddr), .wb_sac(wb_sac), .wb_hit(wb_hit),
  .sd(sd), .sbe(sbe), .ls_v(ls_v), .ls_addr(ls_addr), .ls_data(ls_data), .ls_be(ls_be),
  .bw_en(), .bw_addr(), .bw_data(), .bw_be(),                                                              // unused: the store buffer is in p68_mem
  .ret_u(ret_u), .ret_v(ret_v), .ret_pc(ret_pc), .ret_wen(ret_wen), .ret_wreg(ret_wreg), .ret_wval(ret_wval),
  .ret_ws_en(ret_ws_en), .ret_ws_r(ret_ws_r), .ret_ws_v(ret_ws_v), .ret_wd_en(ret_wd_en), .ret_wd_r(ret_wd_r),
  .ret_wd_v(ret_wd_v), .ret_st(ret_st), .ret_saddr(ret_saddr), .ret_sdata(ret_sdata), .ret_sbe(ret_sbe),
  .ret_ccr(ret_ccr),
  .no_ret(no_ret), .ret_irq(ret_irq), .ret_bk_en(ret_bk_en), .ret_bk_r(ret_bk_r),
  .ex_v(ex_v), .ex_hit2(ex_hit2), .redirect_ex(redirect_ex), .ex_rep(ex_rep), .sb_cnt(sb_cnt), .lock(lock),
  .ret_st2(ret_st2), .ret_saddr2(ret_saddr2), .ret_sdata2(ret_sdata2), .ret_sbe2(ret_sbe2));

// ---- observation and retire port outputs ----
assign obs_a   = wb_val ^ ex_u[`F_PC];   // for the timing top only: mixes of internal values so they are kept
assign obs_b   = fpc ^ oc_addr;
assign obs_ccr = ccr;
// prediction events for the bench's statistics (bit order as in the port list)
assign ret_bp = {ex_brc, ex_bb ? exb_u[`F_PRED] : ex_u[`F_PRED], ex_brt, ex_bb ? exb_u[`F_PBTC] : ex_u[`F_PBTC], ex_commit && ex_u[`F_PRET], ex_commit && ex_u[`F_PRET] && redirect_ex};
assign ret2_first = g_o; assign ret2_wen = wb2_wen; assign ret2_wreg = wb2_wreg; assign ret2_wval = wb2_val;   // lane B's retire record
assign ret2_ccr = ccr_b;
assign ret_sw = sw_en; assign ret_sw_old = sw_old; assign ret_sw_new = sw_new; assign ret_sw_v = sw_v;   // A7 swap

endmodule
`default_nettype wire
