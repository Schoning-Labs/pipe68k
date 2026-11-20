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
// ==== p68_fetch.v: IF, the fetch stage ===============================================================================
//
// What it does. Fetch (IF) keeps the decode stage (DS, p68_decode.v) supplied with whole instructions. It reads the
// instruction cache (p68_icache.v, 8 KB, 32-byte lines) 64 bits at a time, one "fetch block" of four 16-bit words per
// clock, into a 14-word queue, and every clock its extractor cuts up to two complete instructions off the front of the
// queue and writes them into DS's instruction buffer as fixed-format entries (p68_ib.vh). It also predicts taken
// branches with a branch target cache (BTC), so that it can run on at the branch target before the branch is even
// decoded. Everything runs on the CPU clock.
//
// Where it sits. Fetch takes redirects from three places: a flush (an EX redirect one clock late, an AG branch, or an
// OC jump through a register, all formed in p68_core.v), a predicted return from DS (dfl), and DS's request to drop a
// wrong BTC entry (bt_kill). It takes the BTC updates from EX (bu_*, registered in p68_core.v). It hands the memory
// port unit (p68_mem.v) its line fills through the I-cache. And it feeds DS: the entries xe0/xe1 with their write
// strobes ex1/ex2, the prediction slot 0's target pt, and for the long full-format addressing modes, which do not fit
// in a buffer entry, a read port into the queue itself (wx at position xpos, used by p68_fx.v).
//
// The two fetch clocks. In the first clock the read address fra (the next sequential block, a predicted target, or a
// flush target) goes to the I-cache's block RAMs and the BTC's block RAM. In the second clock (if2_pc holds that
// address, if2_v says it is a real read) the block, its predecode and the BTC entry come out of the RAMs; if the cache
// hit, the block's words are written into the queue behind the words already there, and the next read address is
// chosen. Every word carries its predecode from the cache (length, sequence flag, pairing classes; see p68_pdf.vh), so
// fetch never decodes an opcode itself.
//
// The queue. Fourteen words, Q[0] to Q[13], each with its predecode, 30 bits (Qv). Q[0] is always the first word of the
// next instruction to extract: the queue is also the window the extractor looks at. Each clock the queue moves forward
// by the number of words extracted (0, L0, or L0 + L1, where L0 and L1 are the two instructions' lengths) and takes the
// new block's words at the positions behind the valid count. th is the valid count as a thermometer (th[k]: word k is
// valid) and cnt the same as a number. head and tail are counters that only exist to place the BTC cut positions.
// Fourteen is the minimum: the longest instruction p68_fx walks is 11 words and must fit with the block behind it (a
// 12-word queue hung in the test runs), and fetch stops reading when more than NQ-4 words are waiting.
//
// The extractor. Instruction 0 starts at Q[0] (its length L0 is the predecoded length of Q[0], 1-5 words); instruction
// 1 starts at Q[L0] and is extracted in the same clock if it is at most 3 words, has no indexed EA, is not a hold entry
// and all its words are valid. A "hold entry" is an instruction whose length fetch cannot know (a full-format EA, which
// p68_fx walks word by word through wx, or an FPU general operation with an immediate, whose length depends on the
// command word): it is written as an entry, then the extractor stops until DS has issued it and told fetch its real
// length (hiss, il_ov, il), and the queue then moves by that length, up to five words a clock (the hmv steps).
//
// The branch target cache. 256 entries in block RAM, one per 8-byte fetch block (indexed by PC bits 10-3), read in the
// same clock as the I-cache. An entry holds a valid bit, the tag (PC bits 31-11), the word offset of the branch opcode
// in the block (bo), the branch's length in words (bl), and the target. An entry is written when a
// Bcc, BRA, BSR or DBcc commits taken, and cleared when a BTC-predicted one is not taken or when DS finds that the
// predicted cut was wrong (bt_kill). On a hit at or behind the fetch offset, fetch writes the block's words only up to
// the end of the branch (the "cut") and reads the target next. Two predictions can be outstanding (slot 0 and slot 1,
// cut_* and cut1_*); a branch whose words cross into the next block is cut in that next block (xp_*). Fetch does not
// check the opcode at the cut (that would put the I-cache data in front of the prediction); instead the extractor marks
// the entry that covers the cut: IB_PBTC when it is a branch ending exactly there, IB_PFIX otherwise, and an IB_PFIX
// entry replays in OC. A wrong target is caught when OC compares it with the computed one.
//
// History, in the order things happened (measured numbers from the project's records):
//   - From the first sample on, the queue held predecoded words, because the length decode behind the queue pointer
//     was the first slow path found (19 levels).
//   - The DS window (the first five words of the head instruction) became a register, and the predecode moved into the
//     I-cache (computed at fill time and stored with the line, 36 more bits per entry and 2 more block RAMs).
//   - For dual issue the window grew to eight words. Two 16:1 selects per window word did not route (576 unrouted nets);
//     one rotator by head plus length in four 2:1 stages routed at 57.9 MHz and became the base of the dual-issue work.
//   - A third fetch stage was built twice and not kept: the registers around the window loop spread the window logic
//     30-60 columns away from DS (57.9, 57.6 and 61.1 MHz against 71 MHz before), and it cost CPI (1.007 to 1.021).
//   - The instruction buffer in DS replaced issuing from the window: DS's issue no longer drives the window, and the
//     window loop dropped from 24.2 to 19.9 ns in the timing estimate (about 4,400 more logic, CPI 1.007 to 1.021).
//   - In the first full-system builds the worst CPU path was the 16:1 select from the queue into the window (17.7 ns).
//     The 16-word queue and eight window registers were replaced by this 14-word queue that is itself the window:
//     fetch went from 10,750 to 9,515 LUTs and the estimated endpoints above 15 ns from 1,065 to 812.
//   - The BTC first predicted only Bcc/BRA inside one block with one prediction outstanding and checked the opcode at
//     the cut. A trace showed the copy loop's DBF crossed a block and paid a redirect every iteration, so the BTC
//     learned DBcc, BSR and block-crossing branches (copy loop CPI 2.515 to 1.591). The second slot came with the branch
//     fold pair, and the opcode check was dropped for timing.
//   - A board test found a reset bug: a CPU reset while running left the queue's thermometer set, so new words went in
//     behind stale ones, nothing was ever extracted and the CPU hung at address 0. The reset now clears th and thb.
//
// Timing techniques used all over this file, because on this chip a net between units costs as much as several LUTs:
//   - Late selects: when a value depends on a late signal (the I-cache hit, the BTC tag match, the issue/pair decision),
//     every candidate is computed from registers first and the late signal only picks one at the end.
//   - One-hot register copies of lengths (xlo, tlo, wlo, xld): a select by a one-hot register is an AND-OR, with no
//     compare in front of it. Each copy feeds only one group of loads, so no single net has a huge fanout.
//   - Preserved copies: Gowin's synthesizer merges identical logic even when it is marked to be kept, so a copy only
//     stays a separate net when one of its inputs is its own preserved register (the preserve attribute on hmvc, xlo,
//     tlo, wlo, xld, thb, qob). Gowin had merged earlier hand copies into nets of 120-150 loads.
//   - Constant tables instead of arithmetic on RAM outputs (qtT, qtO): the BTC RAM output goes through a LUT table
//     instead of an adder, a compare and a decode.
`include "p68_uop.vh"
`include "p68_ib.vh"
`default_nettype none
module p68_fetch (
  input  wire        clk,
  input  wire        rst,
  input  wire        flush,          // restart fetch at flush_tgt (from p68_core.v)
  input  wire [31:0] flush_tgt,
  input  wire        ffl,            // the flush has a registered target (EX redirect, OC branch): read it now
  input  wire [31:0] fft,            // that target
  input  wire [28:0] fft8i,          // fft's block + 1, added per source from registers and selected late (core)
  // ---- I-cache control, own-store and external snoop, memory unit (passed to p68_icache) ----
  input  wire [3:0]  cacr,           // CACR bits 3-0
  input  wire        clr_all, clr_ent,   // CACR clear / clear entry committed
  input  wire [31:0] caar,           // cache address register
  input  wire [31:0] oc_saddr,       // store address of the uop in OC2 (store tag read)
  input  wire        ex_stc,         // the store in EX is checked
  input  wire [31:0] ex_saddr,
  input  wire        wb_st,          // a store in WB
  input  wire [31:0] wb_saddr,
  input  wire        wb_x2,          // second word of a crossing store (self-modifying code check)
  input  wire [31:0] wb_saddr2,
  output wire        smc,            // a committing store hit fetched code
  input  wire        sn_v,           // DMA snoop
  input  wire [31:2] sn_addr,
  output wire        ir_req,         // line fill request to the memory unit
  output wire [31:2] ir_addr,
  input  wire        ir_go,
  input  wire        r_i,
  input  wire [63:0] r_data,
  input  wire        r_done,
  // branch target cache and return stack redirect
  input  wire        dfl,            // DS: return predicted, flush the queue and fetch from dtgt
  input  wire [31:0] dtgt,
  input  wire        bt_kill,        // DS: the outstanding prediction cut an instruction wrongly
  input  wire        bu_v,           // EX (registered): a Bcc/BRA committed
  input  wire        bu_tk,          // ... taken
  input  wire        bu_pb,          // ... it was predicted by the BTC
  input  wire [31:0] bu_pc,          // ... its address
  input  wire [3:0]  bu_len,         // ... its length in words
  input  wire [31:0] bu_tgt,         // ... its target
  output reg  [31:0] pt,             // the target of prediction slot 0 (DS uses it as the next instruction's PC)
  output reg  [31:0] if2_pc,         // address of the block read in the last clock
  output reg  [31:0] fpc,            // the sequential read address
  // ---- instruction buffer. The extractor writes up to two whole instructions per clock (fixed-format entries,
  // p68_ib.vh) from the window into DS's buffer; DS reads entries only ----
  input  wire        ib_sp1, ib_sp2,  // DS's buffer has room for one / two entries (registers)
  output wire        ex1, ex2,        // write one / two entries (xe0, then xe1)
  output wire [`IBW-1:0] xe0, xe1,
  input  wire        pop,            // DS consumed the entry of prediction slot 0 (predicted branch or wrong cut)
  input  wire        popa, popb,     // copies of pop (slot 0 / slot 1 fields)
  input  wire        hiss,           // DS issued a hold entry (full-format EA, FPU #imm): the window moves by its length next clock
  output reg         hpf,            // the hold entry covers the oldest unmarked BTC cut (register; at the real length)
  output wire [4:0]  fcnt,           // queue words from the head (p68_fx walks the hold entry's words through wx)
  input  wire [3:0]  xpos,           // p68_fx's read position in the queue
  output wire [15:0] wx,             // queue word at head + xpos
  input  wire        il_ov,          // DS's length for the hold entry is il (p68_fx main part, FPU #imm .B/.W)
  input  wire [3:0]  il
) /* synthesis syn_maxfan = 16 */;
// decode helper functions; fetch uses only a few of them (the headers are included as a set, as in gen_pdrom.py)
`include "p68_dec.vh"
`include "p68_sysf.vh"
`include "p68_md.vh"
`include "p68_mdf.vh"
`include "p68_flowf.vh"           // flow instructions
`include "p68_bff.vh"
`include "p68_p7f.vh"             // full-format and long forms

localparam  NQ = 14;             // queue words (the queue is the window, word 0 at the head). Fetch stops
                                  // at more than NQ-4 words: NQ-3 must cover the longest instruction (11 words) for p68_fx
reg  [3:0]   head;                // counter of the queue head (for the BTC cut positions only)
reg  [4:0]   cnt;                 // valid words in the queue
assign fcnt = cnt;
reg          if2_v;               // the read of the last clock is real (its block may enter the queue now)
// ---- I-cache ----
wire [63:0]  ic_q;                    // the block read last clock
wire [47:0]  bpd;                     // its predecode (from the cache or the line buffer)
wire [47:0]  rpd;                     // predecode of a fill beat (p68_pdrom, below)
wire         ic_hit, im_busy;         // the block is valid; a fill is running
// A return prediction (dfl, DS) acts one clock later as a flush with a registered target (dfl_r, dtgt_r): the I-cache
// reads the target in that clock (as for ffl), so the redirect costs no clock, and DS's dfl logic stays off the fetch
// counters (DS drops the extractor's writes of that clock).
reg          dfl_r;
reg  [31:0]  dtgt_r;
wire         flushx  = flush || dfl_r;            // any restart of the queue
wire         fxl     = ffl || dfl_r;              // a restart whose target is a register (read it in this clock)
wire [31:0]  fxt     = ffl ? fft : dtgt_r;        // that target
wire         im_now  = if2_v && !ic_hit && !flushx;   // the block read last clock missed: start a fill
// The queue words, the prediction and the count do not need flush or dfl (both reset head and cnt and win over pred in
// every register pred loads): q_wd keeps the redirect off the queue write enables.
wire         q_wd    = if2_v && ic_hit;           // the block's words enter the queue this clock
p68_icache i_ic (
  .clk(clk), .rst(rst), .cacr(cacr), .clr_all(clr_all), .clr_ent(clr_ent), .caar(caar),
  .fpc(fra), .fpa(fra2), .ffl(ffl), .fft(fft), .if2_pc(if2_pc), .blk(ic_q), .bpd(bpd), .rpd(rpd), .hit(ic_hit), .miss(im_now), .busy(im_busy), .q_wr(q_wd),   /* q_wd: a flushed block in the self-modifying-code line list only costs a replay */
  .oc_saddr(oc_saddr), .ex_stc(ex_stc), .ex_saddr(ex_saddr), .wb_st(wb_st), .wb_saddr(wb_saddr), .smc(smc),
  .wb_x2(wb_x2), .wb_saddr2(wb_saddr2),
  .sn_v(sn_v), .sn_addr(sn_addr),
  .ir_req(ir_req), .ir_addr(ir_addr), .ir_go(ir_go), .r_i(r_i), .r_data(r_data), .r_done(r_done));
wire [1:0]  q_off   = if2_pc[2:1];                                   // the first word of the block that is wanted (0-3)
reg  [3:0]  tail;                                                    // head + cnt as a register (it moves only with the block writes)
// The window moves by the extracted instructions, or (one clock after DS issued it) by a hold entry's length.
reg         hst, hmv;                    // a hold entry was written: no extraction until DS issued it and the window moved past it
reg  [9:0]  rcq;                         // {rc == 4 .. rc == 0, rc < 5 .. rc < 1}, registered with rc
function [9:0] rqf; input [3:0] r; rqf = {r == 4'd4, r == 4'd3, r == 4'd2, r == 4'd1, r == 4'd0, r < 4'd5, r < 4'd4, r < 4'd3, r < 4'd2, r < 4'd1}; endfunction
reg  [NQ-1:0] hmvc /* synthesis syn_preserve = 1 */;   // copies of hmv, one per queue position (late selects)
reg  [3:0]  hl0, hr;                     // the hold entry's predecoded length; words still to move after this hmv step
wire [3:0]  ilen;                        // length of instruction 0
wire        issue, pair, hold0;          // the window moves by one / two instructions; the head instruction is a hold entry
wire [3:0]  plen;                        // length of both instructions of a pair
wire [3:0]  cons = issue ? (pair ? plen : ilen) : 4'd0;   // words leaving the window
wire [4:0]  cnt_n;                       // the count after this clock (below)
reg  [15:0] th_n;                        // the thermometer after this clock (below)
// ---- branch target cache, 256 entries (one per 8-byte fetch block), block RAM, read with the I-cache.
// Entry (57 bits): valid (56), tag PC 31-11 (55:35), word offset bo of the branch opcode in the block (34:33), length
// bl (32:31, 1-3 words), target bits 31-1 (30:0). Written when a branch commits taken; cleared when a BTC-predicted one
// is not taken, or by DS (bt_kill). A hit at if2 whose branch lies at or behind the fetch offset cuts the queue write
// behind the branch and sends fetch to the target.
reg  [56:0] bt [0:255];                  // the BTC entries
reg  [56:0] bt_q;                        // registered read output (the entry of if2_pc's block)
reg         bt_we; reg [7:0] bt_wa; reg [56:0] bt_wd;      // write port (combinational, below)
reg         bt_sw;                   // reset sweep: no predictions while the entries are cleared
reg  [7:0]  bt_sc;                   // the entry the sweep clears now
always @(posedge clk) if (bt_we) bt[bt_wa] <= bt_wd;
// A flush whose target is a register (ffl) reads the I-cache and the BTC at the target in the flush clock.
wire [31:0] fra2 = dfl_r ? dtgt_r : fpc;          // the read address without ffl (the I-cache selects the line buffer compare late)
wire [31:0] fra = fxl ? fxt : fpc;                // the read address of this clock
wire [28:0] dtgt8 = dtgt_r[31:3] + 29'd1;         // the return target's next block
wire [28:0] fft8 = ffl ? fft8i : dtgt8;           // the registered target's next block
always @(posedge clk) bt_q <= bt[fra[10:3]];      // BTC read (block RAM)
reg         cut_v;                       // prediction slot 0 is valid
reg  [3:0]  cut_pos, pos_o;              // its cut position (last word of the branch) and branch position, as head-relative counters
reg  [31:0] pt_pc;                       // address of slot 0's branch
// A second outstanding prediction: fetch runs past the first predicted branch; slot 1 moves to slot 0 when DS consumes
// the first. DS sees slot 0 only.
reg         cut1_v;
reg  [3:0]  cut1_pos, pos1_o;
reg  [31:0] pt1, pt1_pc;                 // slot 1's target and branch address
wire [1:0]  bo = bt_q[34:33], bl = bt_q[32:31];   // branch word offset in the block, branch length
// BSR and DBcc too; a branch whose words cross into the next block (xcr) writes this block whole and cuts in the next
// one (xp_*), which fetch reads next anyway.
reg         xp_v; reg [31:0] xp_t, xp_pc; reg [1:0] xp_n; reg [3:0] xp_pos;   // pending cut in the next block: target, branch address, words, position
// No check of the opcode in the block (the I-cache data stays off the prediction): a cut that does not end a branch is
// marked IB_PFIX by the extractor and replays.
wire        bhr   = q_wd && !bt_sw && bt_q[56] && (bt_q[55:35] == if2_pc[31:11]) && (bo >= q_off);   // BTC hit for this block
wire        predk = bhr && !(cut_v && cut1_v) && !xp_v;          // a slot is free: take the prediction
// A BTC hit while both slots are taken fetches the block again instead of running past the branch (the hit would be
// lost and the branch would redirect later); its words are not counted.
wire        bhold = bhr && cut_v && cut1_v && !xp_v && !flushx;
wire        xcr = ({1'b0, bo} + {1'b0, bl}) > 3'd4;              // the branch's words cross into the next block
wire        predx = predk && xcr;                                  // first block of a crossing branch
wire        predc = q_wd && xp_v;                                    // its second block: cut here
wire        pred  = (predk && !xcr) || predc;                        // a cut happens in this block
wire [31:0] p_tgt = predc ? xp_t : {bt_q[30:0], 1'b0};               // the predicted target
wire [2:0]  q_k     = xp_v ? {1'b0, xp_n} : {1'b0, bo} + {1'b0, bl} - {1'b0, q_off};   // words written up to the branch's end (xp_v = predc whenever pred, since predk needs !xp_v)
wire [2:0]  q_n     = bhold ? 3'd0 : pred ? q_k : 3'd4 - {1'b0, q_off};   // words of the block written into the queue
// Marks. An entry gets IB_PBTC / IB_PFIX from the oldest prediction slot not yet marked (slot S); DS pops slot 0 when it
// issues the marked entry (pop), so slot 0 always belongs to the oldest marked entry in DS's buffer.
reg         cut_m, cut1_m;               // slot 0 / 1 is marked in an entry
wire        setm;                        // the extractor marks slot S this clock
wire [3:0]  n_cpos  = tail + {1'b0, q_k} - 4'd1;      // cut position of a new prediction (last word written)
wire [3:0]  tlh1    = tail - head - 4'd1;   // registers only
wire [3:0]  n_poso  = predc ? xp_pos : tail + {2'b0, bo} - {2'b0, q_off};   // position of the new prediction's branch opcode
wire [31:0] n_ptpc  = predc ? xp_pc : {if2_pc[31:3], bo, 1'b0};             // address of the new prediction's branch
// BTC write port: DS's kill, a committed taken branch (new entry), a committed BTC-predicted branch that was not taken
// (clear), the reset sweep
always @* begin
  bt_we = 1'b0; bt_wa = bu_pc[10:3]; bt_wd = 57'h0;
  if (bt_kill) begin bt_we = 1'b1; bt_wa = pt_pc[10:3]; end
  else if (bu_v && bu_tk) begin
    bt_we = 1'b1; bt_wd = {1'b1, bu_pc[31:11], bu_pc[2:1], bu_len[1:0], bu_tgt[31:1]};
  end else if (bu_v && !bu_tk && bu_pb) bt_we = 1'b1;
  else if (bt_sw) begin bt_we = 1'b1; bt_wa = bt_sc; end
end
// read the next block when the I-cache is free and the queue has room for four more words after this read
wire        if_go   = !im_busy && ({1'b0, cnt} + (if2_v ? 6'd4 : 6'd0) <= NQ - 4);
wire [15:0] blk0 = ic_q[63:48], blk1 = ic_q[47:32], blk2 = ic_q[31:16], blk3 = ic_q[15:0];   // the block's words, word 0 first

// The I-cache hit (q_wd) and the BTC tag match (bh) select last; every candidate is formed from registers and the BTC
// fields (the same decisions as im_now, bhold, pred, predx, predc above).
wire        btm   = (bt_q[55:35] == if2_pc[31:11]);                     // BTC tag match
wire        bh    = !bt_sw && bt_q[56] && (bo >= q_off) && btm;         // BTC hit without the I-cache hit
wire        both  = cut_v && cut1_v;                                    // both slots are taken
wire [31:0] fpcG  = if_go ? {fpc[31:3] + 29'd1, 3'b000} : fpc;          // next sequential block, or stay
wire [31:0] fpcN  = xp_v ? xp_t : fpcG;                                   // block written, no BTC hit (or its second block)
wire [31:0] fpcT  = both ? if2_pc : !xcr ? {bt_q[30:0], 1'b0} : fpcG;    // block written, BTC hit, !xp_v
wire        ivT   = if_go && !both && xcr, ivN = if_go && !xp_v;        // if2_v candidates for the same cases
// The BTC-hit word count (!xcr ? bo + bl - q_off : 4 - q_off) and its one-hot as constant tables of {bo, bl, q_off}:
// LUTs off the BTC RAM output instead of an add, a compare and a decode.
wire [191:0] qtT; wire [319:0] qtO;
genvar qi;
generate for (qi = 0; qi < 64; qi = qi + 1) begin : qt
  localparam [2:0] QS = ((qi >> 4) & 3) + ((qi >> 2) & 3);                         // bo + bl for table index qi
  localparam [2:0] QV = (QS > 3'd4) ? (3'd4 - (qi & 3)) : (QS - (qi & 3));         // words written for this case
  assign qtT[qi*3 +: 3] = QV;
  assign qtO[qi*5 +: 5] = (QV == 3'd0) ? 5'd1 : (QV == 3'd1) ? 5'd2 : (QV == 3'd2) ? 5'd4 : (QV == 3'd3) ? 5'd8 : 5'd16;   // one-hot
end endgenerate
wire [5:0]  qtx   = {bo, bl, q_off};                                    // table index
wire [2:0]  qaT   = xp_v ? {1'b0, xp_n} : both ? 3'd0 : qtT[qtx*3 +: 3];   // block word count, BTC hit
wire [2:0]  qaN   = xp_v ? {1'b0, xp_n} : 3'd4 - {1'b0, q_off};                              // ... no BTC hit
wire [3:0]  tlT   = tail + {1'b0, qaT}, tlN = tail + {1'b0, qaN};        // tail after the write, per case
// ---- the fetch registers: read address, BTC slots, queue counters ----
always @(posedge clk) begin
  if (rst) begin
    fpc <= 32'h0; if2_v <= 1'b0; head <= 4'd0; cnt <= 5'd0; tail <= 4'd0;
    // The queue's fill thermometer too: a reset while running left th / thb of the old queue (only a flush cleared
    // them), the new words went in behind them and W0 kept a stale entry of length 0, so nothing was ever extracted
    // (on the A1200 board every CPU reset after the first hung the CPU at address 0).
    th <= 16'h0; thb <= 16'h0;
    cut_v <= 1'b0; cut1_v <= 1'b0; bt_sw <= 1'b1; bt_sc <= 8'd0;   // no predictions; start the BTC sweep
    // The return prediction and the slot marks as well. Without this a reset held them at their old values: a dfl_r
    // of 1 made the first clock after the reset a restart at the old dtgt_r instead of address 0, and a stale cut_m
    // marked the first new prediction.
    dfl_r <= 1'b0; dtgt_r <= 32'h0; cut_m <= 1'b0; cut1_m <= 1'b0;
  end else begin
    dfl_r <= dfl && !flush; dtgt_r <= dtgt;                            // the return prediction, one clock later
    // next read address: a restart reads its registered target now (or, while a fill runs, keeps it for later) and
    // continues with the following block; otherwise the late hit / BTC selects pick a candidate
    if (flushx)      fpc <= fxl ? (im_busy ? fxt : {fft8, 3'b000}) : flush_tgt;
    else             fpc <= q_wd ? ((bh && !xp_v) ? fpcT : fpcN) : (if2_v ? if2_pc : fpcG);   // a miss rereads if2_pc
    if2_v  <= fxl ? !im_busy : flushx ? 1'b0 : q_wd ? ((bh && !xp_v) ? ivT : ivN) : (!if2_v && if_go);
    if2_pc <= fra;
    // ---- branch target cache ----
    if (bt_sw && !bt_kill && !bu_v) begin bt_sc <= bt_sc + 8'd1; if (bt_sc == 8'd255) bt_sw <= 1'b0; end   // sweep step
    if (flushx || bt_kill) xp_v <= 1'b0;                              // the pending cross-block cut
    else if (predx) xp_v <= 1'b1;
    else if (predc) xp_v <= 1'b0;
    if (!xp_v) begin                                                   // fields load while free (read only while xp_v)
      xp_t <= {bt_q[30:0], 1'b0}; xp_pc <= {if2_pc[31:3], bo, 1'b0};
      xp_n <= bo + bl; xp_pos <= tail + {2'b0, bo} - {2'b0, q_off};   // xp_n: words in the next block (bo + bl - 4, mod 4)
    end
    // Two slots. pop: DS consumed slot 0's branch (or a pair ending with it); a new prediction goes into the first free
    // slot after the pop. The fields load without the valid bits (read only while valid).
    if (flushx || bt_kill) begin cut_v <= 1'b0; cut1_v <= 1'b0; cut_m <= 1'b0; cut1_m <= 1'b0; end
    else begin
      cut_v  <= pop ? (cut1_v || pred) : (cut_v || pred);
      cut1_v <= pop ? (cut1_v && pred) : (cut1_v || (cut_v && pred));
      // setm marks slot S (slot 1 when slot 0 is marked); a new prediction is unmarked
      cut_m  <= pop ? (cut1_v && (cut1_m || setm)) : (cut_m || (setm && cut_v));
      cut1_m <= pop ? 1'b0 : (cut1_v && (cut1_m || (setm && cut_m)));
    end
    if (popa ? !cut1_v : !cut_v) begin                      // slot 0 takes the new prediction
      cut_pos <= n_cpos; pos_o <= n_poso; pt <= p_tgt; pt_pc <= n_ptpc;
    end else if (popa) begin                                 // slot 0 takes slot 1
      cut_pos <= cut1_pos; pos_o <= pos1_o; pt <= pt1; pt_pc <= pt1_pc;
    end
    // slot 1 takes a new prediction whenever it is free after this clock (read only while valid; no BTC decision in the enable)
    if (popb || !cut1_v) begin cut1_pos <= n_cpos; pos1_o <= n_poso; pt1 <= p_tgt; pt1_pc <= n_ptpc; end   // slot 1 takes it
    // ---- queue counters ----
    if (flushx) begin head <= 4'd0; cnt <= 5'd0; tail <= 4'd0; th <= 16'h0; thb <= 16'h0; end
    else begin
      head <= head + cons;                                                 // extractor (issue, pair) or hold entry move
      cnt  <= cnt_n;
      tail <= q_wd ? (bh ? tlT : tlN) : tail;                              // the word count with the hit and tag match last
      th   <= th_n; thb <= th_n;                                           // thermometer and its copy for the block positions
    end
    // rc per case (none; one, which marks with cin0 unless it is a hold move; pair, which marks with cin1), with the
    // issue and pair decision selecting last
    rc <= issue ? (pair ? ((sv && !cin1) ? rcA2 : rcB2) : ((sv && !(cin0 && !hmv)) ? rcA1 : rcB1)) : (sv ? rcA0 : rcB0);
    // rc's compares with 0..4 (for cin0, gd0) registered, formed per candidate before the same late select
    rcq <= issue ? (pair ? ((sv && !cin1) ? rqf(rcA2) : rqf(rcB2)) : ((sv && !(cin0 && !hmv)) ? rqf(rcA1) : rqf(rcB1))) : (sv ? rqf(rcA0) : rqf(rcB0));
  end
end

// The predecode comes with the block from the I-cache (computed from the fill data, stored with each beat in the line
// buffer and with the line in the cache). Per word, 12 bits: {length and sequence (5), lane-B class (1), memory class
// (1), branch class (1), pd_fx (4)} (p68_pdf.vh).
wire [4:0] pl0 = bpd[47:43], pl1 = bpd[35:31], pl2 = bpd[23:19], pl3 = bpd[11:7];   // {sequence, length} of words 0-3
// the pairing classes are predecoded with the line as well (stored in the I-cache), so they are off the block RAM output
wire       sm0 = bpd[42], sm1 = bpd[30], sm2 = bpd[18], sm3 = bpd[6];               // lane-B class
wire       mm0 = bpd[41], mm1 = bpd[29], mm2 = bpd[17], mm3 = bpd[5];               // memory class
wire       mb0 = bpd[40], mb1 = bpd[28], mb2 = bpd[16], mb3 = bpd[4];               // branch class
wire [3:0] px0 = bpd[39:36], px1 = bpd[27:24], px2 = bpd[15:12], px3 = bpd[3:0];   // pd_fx (full-format positions)
// The fill predecode (p68_pdf.vh) comes from block-RAM tables indexed by the fill beat (p68_pdrom.v); it is valid two
// clocks after r_data (p68_icache writes it into lbp one clock later and holds the line-buffer hit that long).
p68_pdrom i_pdr (.clk(clk), .w(r_data), .pd(rpd));

// ---- The queue itself is the window. Q[p] is the queue word at head + p (p = 0 .. NQ-1) with its predecode
// {word, seq, len, lane-B class, memory class, branch class, pd_fx}. Each clock the queue moves by the consumption
// (0, L0 or L0 + L1) and takes the block words at their positions behind the count. There is no select by a head
// index: the move is a one-hot select by register copies of L0 (wlo), with the block words merged in the same AND-OR
// (bk), then a 4:1 by L1 and issue / pair (late). head and tail remain counters for the BTC cut positions only.
// p68_fx reads the hold entry's words at xpos from the head (wx).
// Bit layout of one queue word (30 bits): 29 may be a pair's second instruction (xek), 28 FPU #imm hold class (hfk),
// 27:12 the word, 11 sequence, 10:7 length, 6 lane-B class, 5 memory class, 4 branch class, 3:0 pd_fx.
reg  [NQ*30-1:0] Qv;                     // the queue registers
reg  [15:0]  thb /* synthesis syn_preserve = 1 */;   // copy of th for the block positions (bk)
reg  [1:0]   qob /* synthesis syn_preserve = 1 */;   // copy of q_off (if2_pc[2:1]) for bk
always @(posedge clk) qob <= fra[2:1];
wire [29:0]  Qa [0:NQ+7];                // Q[p]; zero for p >= NQ
// valid positions (thb, zero for p >= NQ): the queue parts of the move keep valid words only, so the block positions
// (if2_v, qob) leave the queue-word masks
wire [NQ+7:0] vq = {8'h0, thb[NQ-1:0]};
wire [4:0]   bk [0:NQ+7];                // the block writes position p this clock (positions before the move): {any, one-hot word}
assign wx = Qa[xpos][27:12];             // p68_fx's read port
wire [29:0] W0 = Qa[0], W1 = Qa[1], W2 = Qa[2], W3 = Qa[3], W4 = Qa[4], W5 = Qa[5], W6 = Qa[6], W7 = Qa[7];   // the window
wire [15:0] wq0 = W0[27:12], wq1 = W1[27:12], wq2 = W2[27:12], wq3 = W3[27:12], wq4 = W4[27:12];            // its words
wire        wqs = W0[11];                // the head instruction is a sequence
wire [3:0]  wql = W0[10:7], wqp = W0[3:0];   // its length L0; its pd_fx
// Bits 29 and 28 of a queue word: the word may be a pair's second instruction as far as the word itself tells (length
// 1-3, no indexed EA, not the FPU #imm hold class), and the word is that hold class; computed once from the block word.
// The FPU hold class is FPU general op ($F200 with bits 5-0 = 111100, an immediate source); its length depends on the
// command word, which is the next word.
function hfw; input [15:0] w; hfw = (`P68_FPU != 0) && (w[15:6] == 10'b1111001000) && (w[5:0] == 6'b111100); endfunction   // same as fpuh
wire [3:0]  hfk = {hfw(blk3), hfw(blk2), hfw(blk1), hfw(blk0)};
wire [3:0]  xek = {pl3[3:2] == 2'b00 && px3 == 4'd0 && !hfk[3], pl2[3:2] == 2'b00 && px2 == 4'd0 && !hfk[2],
                   pl1[3:2] == 2'b00 && px1 == 4'd0 && !hfk[1], pl0[3:2] == 2'b00 && px0 == 4'd0 && !hfk[0]};
// the block's four words in queue format
wire [29:0] BW0 = {xek[0], hfk[0], blk0, pl0, sm0, mm0, mb0, px0}, BW1 = {xek[1], hfk[1], blk1, pl1, sm1, mm1, mb1, px1},
            BW2 = {xek[2], hfk[2], blk2, pl2, sm2, mm2, mb2, px2}, BW3 = {xek[3], hfk[3], blk3, pl3, sm3, mm3, mb3, px3};
genvar j;
// per position: the queue word, and which block word (if any) is written there this clock
generate for (j = 0; j < NQ + 8; j = j + 1) begin : qp
  if (j < NQ) begin : v
    // Every invalid position p >= cnt takes block word (p - cnt + q_off) mod 4 (no upper bound cnt + 4 - q_off: the
    // words behind it land in positions the count leaves invalid, which no reader uses, because the queue parts below
    // are masked by thb).
    wire       b  = if2_v && !thb[j];
    wire [1:0] k  = j[1:0] - cnt[1:0] + qob;
    assign Qa[j] = Qv[j*30 +: 30];
    assign bk[j] = {b, b && (k == 2'd3), b && (k == 2'd2), b && (k == 2'd1), b && (k == 2'd0)};
  end else begin : z
    assign Qa[j] = 30'h0;                // positions past the queue read as zero (no select bounds needed)
    assign bk[j] = 5'h0;
  end
end endgenerate
wire [29:0] W0n;                         // W0 after this clock (its length loads the one-hot copies)
function [5:1] lo5; input [3:0] n; lo5 = {n == 4'd5, n == 4'd4, n == 4'd3, n == 4'd2, n == 4'd1}; endfunction   // length 1-5 as a one-hot
// the hold overrides and the queue part first (registers and the issue selects), the block part ORed in last
wire        w0ov = (hst && !hmv) || (hmv && hr != 4'd0);   // W0's length is overridden by the hold entry's length
// W0n's length one-hot formed per case (no issue, one, pair x1 / x2 / x3) with the hold override and the block part
// already in it; the late issue / pair select is the last step (it had three to four LUTs behind it before)
wire [5:1]  w0lv = (hst && !hmv) ? lo5(hT5) : lo5(hr5);      // the override, used while w0ov
wire [5:1]  w0no;                        // W0's one-hot length after this clock
// register copies of W0's one-hot length for the extractor (xlo) and the count (tlo): no length compare behind W0
reg  [5:1]  xlo /* synthesis syn_preserve = 1 */;
reg  [5:1]  tlo /* synthesis syn_preserve = 1 */;
always @(posedge clk) begin xlo <= w0no; tlo <= w0no; end
wire [5:1]  d5q [0:8];                   // lengths one-hot per source, decoded before the window select (wlo path)
wire [5:1]  d5b [0:3];                   // ... of the four block words
generate for (j = 0; j < 9; j = j + 1) begin : dq
  assign d5q[j] = lo5(Qa[j][10:7]);
end endgenerate
assign d5b[0] = lo5(BW0[10:7]); assign d5b[1] = lo5(BW1[10:7]); assign d5b[2] = lo5(BW2[10:7]); assign d5b[3] = lo5(BW3[10:7]);
function [5:1] cwk; input [5:1] m; input [3:0] k;  // one case: the queue part m (the hold override already folded in) and the block words' part (weights k)
  cwk = m | ({5{k[0]}} & d5b[0]) | ({5{k[1]}} & d5b[1]) | ({5{k[2]}} & d5b[2]) | ({5{k[3]}} & d5b[3]);
endfunction
// rb[j] = Q[j + L0] (zb[j]: the same at j, no move); the block word written at j + L0 joins in Wn. One AND-OR per
// position whose selects come from registers only (register copies of L0, wlo; cnt, q_off, if2_v); the late issue /
// pair select comes after it.
wire [29:0] rb [0:NQ+2];
wire [29:0] zb [0:NQ-1];
wire [3:0]  gb [0:NQ+2];                 // block word one-hot of rb[j] (g)
generate for (j = 0; j < NQ + 3; j = j + 1) begin : rs
  if (j < NQ) begin : v
    reg  [5:1]  wlo /* synthesis syn_preserve = 1 */;   // this position's own copy of W0's one-hot length
    always @(posedge clk) wlo <= w0no;
    wire [5:1]  a = wlo & {vq[j + 5], vq[j + 4], vq[j + 3], vq[j + 2], vq[j + 1]};   // source is a valid queue word (thb)
    wire [3:0]  g = ({4{wlo[1]}} & bk[j + 1][3:0]) | ({4{wlo[2]}} & bk[j + 2][3:0]) | ({4{wlo[3]}} & bk[j + 3][3:0]) |
                    ({4{wlo[4]}} & bk[j + 4][3:0]) | ({4{wlo[5]}} & bk[j + 5][3:0]);                   // ... or block word k
    // queue words only; the block words join after the late select (Wn) through one weight per block word
    assign rb[j] = ({30{a[1]}} & Qa[j + 1]) | ({30{a[2]}} & Qa[j + 2]) | ({30{a[3]}} & Qa[j + 3]) |
                   ({30{a[4]}} & Qa[j + 4]) | ({30{a[5]}} & Qa[j + 5]);
    assign gb[j] = g;
    assign zb[j] = {30{vq[j]}} & Qa[j];  // no move: the word stays where it is (if valid)
  end else begin : z
    assign rb[j] = 30'h0;
    assign gb[j] = 4'h0;
  end
end endgenerate
// The head's full-format flag from the window: its pd_fx gives the word offsets where a full-format extension word could
// be (source and destination), and bit 8 of an extension word is the 68020's full-format bit.
wire [1:0] wps = wqp[3:2], wpd = wqp[1:0];
wire   d_full = (wps == 2'd1 && wq1[8]) || (wps == 2'd2 && wq2[8]) || (wps == 2'd3 && wq3[8]) ||
                (wpd == 2'd1 && wq1[8]) || (wpd == 2'd2 && wq2[8]) || (wpd == 2'd3 && wq3[8]);
// ---- the extractor. Instruction 0 starts at W0 (length L0 = wql, 1-5 words); instruction 1 at W[L0] (length L1, at
// most 3 words, no indexed EA, no hold class). Both go into DS's instruction buffer as fixed-format entries; the window
// then moves by L0 or L0 + L1. A hold entry (full-format EA: p68_fx walks its words through wx; FPU general op with
// #imm: DS knows the length) stops the extractor until DS issued it; the window then moves by DS's length (hmv steps).
reg  [27:0] WX, WY, WZ;                  // W[L0], W[L0+1], W[L0+2]: the second instruction's three words with predecode
always @*
  case (wql)
    4'd1: begin WX = W1; WY = W2; WZ = W3; end
    4'd2: begin WX = W2; WY = W3; WZ = W4; end
    4'd3: begin WX = W3; WY = W4; WZ = W5; end
    4'd4: begin WX = W4; WY = W5; WZ = W6; end
    default: begin WX = W5; WY = W6; WZ = W7; end
  endcase
wire [15:0] wxo = WX[27:12];             // the second instruction's opcode
wire [3:0]  wll = WX[10:7];              // L1
// opcode classes the extractor records in the entries
function fpuh; input [15:0] w; fpuh = (`P68_FPU != 0) && (w[15:6] == 10'b1111001000) && (w[5:0] == 6'b111100); endfunction   // FPU general op with #imm
function iscall; input [15:0] w; iscall = (w[15:8] == 8'h61) || (w[15:6] == 10'b0100111010); endfunction   // BSR ($61xx) or JSR
function isret; input [15:0] w; isret = (w == 16'h4E75) || (w == 16'h4E74) || (w == 16'h4E77); endfunction   // RTS, RTD, RTR
function isbr; input [15:0] w; isbr = (w[15:12] == 4'h6) || ((w & 16'hF0F8) == 16'h50C8); endfunction       // Bcc/BRA/BSR or DBcc
// slot S: the oldest prediction not yet marked
wire        sv   = cut_m ? (cut1_v && !cut1_m) : cut_v;
// rc: slot S's cut position relative to the head, a register (moves with the head; reloaded when slot S changes), so
// the extractor's cut checks are compares of registers. th: the queue count as a thermometer (th[k]: word k valid).
reg  [3:0]  rc;
reg  [15:0] th;
wire [3:0]  ilq = wql;                   // a hold entry's real length is written into W0 when DS issues it
wire [3:0]  rcA0 = rc, rcA1 = rc - ilq;  // rc after no issue / one instruction, slot S unchanged
// rc minus the consumption when slot S changes, per case from registers; the new prediction's cut (q_k, BTC) is added last
wire        rcs1 = !cut_m && cut1_v;     // after the change, slot S is slot 1
wire [3:0]  c1h  = cut1_pos - head;      // slot 1's cut relative to the head
wire [3:0]  rcB0 = rcs1 ? c1h : (tlh1 + {1'b0, q_k});
wire [3:0]  rcB1 = rcs1 ? (c1h - wql) : ((tlh1 - wql) + {1'b0, q_k});
assign      hold0 = d_full || ((`P68_FPU != 0) && W0[28]);   // the head is a hold entry; W0[28] is fpuh(wq0) from the block (FPU=0: no register read)
assign      plen = wql + wll;
// Per start position j = 1..5 of a second instruction: x1 (it may be extracted: valid, at most 3 words, no indexed EA,
// no hold class, all its words valid), c1 / g1 (slot S's cut lies inside it / at its last word and it is a branch);
// the head length selects one (no adder behind the length select).
wire [29:0] Wj [1:5];
assign Wj[1] = W1; assign Wj[2] = W2; assign Wj[3] = W3; assign Wj[4] = W4; assign Wj[5] = W5;
// The pair's terms per start position j of the second instruction (j = L0, its length from W[j]), selected by the
// one-hot copy xlo of L0. Before, W0's length went through the WX select and the plen adder into rcq, a failing path
// of the full system.
reg  [3:0]  rcA2, c1hP, tlhP;
integer pj;
always @* begin
  rcA2 = 4'd0; c1hP = 4'd0; tlhP = 4'd0;
  for (pj = 1; pj <= 5; pj = pj + 1) begin
    rcA2 = rcA2 | ({4{xlo[pj]}} & (rc - pj[3:0] - Wj[pj][10:7]));
    c1hP = c1hP | ({4{xlo[pj]}} & (c1h - pj[3:0] - Wj[pj][10:7]));
    tlhP = tlhP | ({4{xlo[pj]}} & (tlh1 - pj[3:0] - Wj[pj][10:7]));
  end
end
wire [3:0]  rcB2 = rcs1 ? c1hP : (tlhP + {1'b0, q_k});   // rc after a pair when slot S changes
reg  [5:1]  x1j, c1j, g1j;
integer jj;
always @* begin
  for (jj = 1; jj <= 5; jj = jj + 1) begin
    x1j[jj] = th[jj] && Wj[jj][29] &&                                     // bit 29 from the block (may be a second instruction)
              ((Wj[jj][8:7] == 2'd1) ? th[jj] : (Wj[jj][8:7] == 2'd2) ? th[jj + 1] : th[jj + 2]);   // its last word is valid
    c1j[jj] = ({1'b0, rc} < jj[4:0] + {1'b0, Wj[jj][10:7]});                // slot S's cut lies before its end
    g1j[jj] = ({1'b0, rc} == jj[4:0] + {1'b0, Wj[jj][10:7]} - 5'd1) && isbr(Wj[jj][27:12]);   // ... at its last word, and it is a branch
  end
end
// Selects by the one-hot copy xlo of L0. The issue / pair decision has its own copy of L0 (xld), with the per-start
// terms formed from registers first (xlo's first net was 1.7-2.1 ns in the full system's reports).
reg  [5:1]  xld /* synthesis syn_preserve = 1 */;
always @(posedge clk) xld <= w0no;
wire [5:1]  pqk;
assign      pqk = x1j & ~({5{sv}} & rcq[4:0]);   // the second instruction may be extracted and slot S's cut is not in the first
wire        wv0d = |(xld & th[4:0]);             // all words of instruction 0 are valid
wire        x1d  = |(xld & pqk);                 // instruction 1 may go too
wire        cin0 = sv && |(xlo & rcq[4:0]);      // slot S's cut lies inside instruction 0
wire        gd0  = |(xlo & rcq[9:5]) && isbr(wq0);   // ... at its last word, and it is a branch
wire        cin1 = sv && |(xlo & c1j);           // slot S's cut lies inside instruction 1
wire        gd1  = |(xlo & g1j);                 // ... at its last word, and it is a branch
assign      ex1  = !hst && !hmv && ib_sp1 && wv0d;   // write instruction 0 (DS has room, no hold entry pending)
assign      ex2  = ex1 && !hold0 && ib_sp2 && x1d;      // write instruction 1 too (x1d = x1ok && !cin0)
assign      setm = (ex1 && !hold0 && cin0) || (ex2 && cin1);   // an entry takes slot S's mark
assign      issue = (ex1 && !hold0) || hmv;     // the window moves; hmv is a step of a hold entry's move (at most five words a clock)
assign      pair  = ex2;
assign      ilen  = wql;
// entry 0: the head instruction's five words and predecoded fields
assign xe0[`IB_WD] = {W4[27:12], W3[27:12], W2[27:12], W1[27:12], wq0};
assign xe0[`IB_LEN] = wql; assign xe0[`IB_SQ] = wqs; assign xe0[`IB_FULL] = d_full; assign xe0[`IB_HOLD] = hold0;
assign xe0[`IB_SIMP] = W0[6]; assign xe0[`IB_MEM] = W0[5]; assign xe0[`IB_BR] = W0[4];
assign xe0[`IB_PBTC] = cin0 && gd0 && !hold0; assign xe0[`IB_PFIX] = cin0 && !gd0 && !hold0;   // a hold entry's mark comes later (hpf)
assign xe0[`IB_RTS] = (wq0 == 16'h4E75);
assign xe0[`IB_CALL] = iscall(wq0); assign xe0[`IB_RET] = isret(wq0); assign xe0[`IB_PQ] = 20'h0; assign xe0[`IB_SX] = 15'h0;   // DS adds PQ and SX
// entry 1: the second instruction (at most three words, so the upper two words are zero)
assign xe1[`IB_WD] = {32'h0, WZ[27:12], WY[27:12], wxo};
assign xe1[`IB_LEN] = wll; assign xe1[`IB_SQ] = WX[11]; assign xe1[`IB_FULL] = 1'b0; assign xe1[`IB_HOLD] = 1'b0;
assign xe1[`IB_SIMP] = WX[6]; assign xe1[`IB_MEM] = WX[5]; assign xe1[`IB_BR] = WX[4];
assign xe1[`IB_PBTC] = cin1 && gd1; assign xe1[`IB_PFIX] = cin1 && !gd1;
assign xe1[`IB_RTS] = (wxo == 16'h4E75);
assign xe1[`IB_CALL] = iscall(wxo); assign xe1[`IB_RET] = isret(wxo); assign xe1[`IB_PQ] = 20'h0; assign xe1[`IB_SX] = 15'h0;
// The count and its thermometer after this clock: one candidate per consumption (none, one instruction, a pair) from
// registers (cnt, th, the window lengths), the block's word count added as a one-hot shift, the issue / pair decision
// last. The block word counts with and without a BTC tag match (qaT, qaN) are added to each candidate; the hit and the
// tag match select after the adds, issue and pair last.
wire [4:0]  qohT = xp_v ? ((xp_n == 2'd0) ? 5'd1 : (xp_n == 2'd1) ? 5'd2 : (xp_n == 2'd2) ? 5'd4 : 5'd8) : both ? 5'd1 : qtO[qtx*5 +: 5];   // one-hot count, BTC hit
wire [4:0]  qohN = (qaN == 3'd0) ? 5'd1 : (qaN == 3'd1) ? 5'd2 : (qaN == 3'd2) ? 5'd4 : (qaN == 3'd3) ? 5'd8 : 5'd16;   // ... no BTC hit
// a pair's count per start position j of the second instruction (j = L0; its length from W[j]), selected by L0
wire [4:0]  cI = cnt - {1'b0, wql};      // count after one instruction
reg  [4:0]  cP;                          // count after a pair
reg  [15:0] tI, tP;                      // thermometer after one instruction / a pair
reg  [79:0] tPj;                         // 16 bits per start position 1..5
integer ti, tj, tk;
always @* begin
  cP = 5'd0;
  for (tj = 1; tj <= 5; tj = tj + 1) begin
    for (ti = 0; ti < 16; ti = ti + 1)
      tPj[(tj - 1)*16 + ti] = ((ti + tj + Wj[tj][10:7]) < 16) ? th[(ti + tj + Wj[tj][10:7]) & 15] : 1'b0;   // th shifted down by j + L1
    cP = cP | ({5{tlo[tj]}} & (cnt - tj[4:0] - {1'b0, Wj[tj][10:7]}));
  end
  tP = ({16{tlo[1]}} & tPj[15:0]) | ({16{tlo[2]}} & tPj[31:16]) | ({16{tlo[3]}} & tPj[47:32]) | ({16{tlo[4]}} & tPj[63:48]) |
       ({16{tlo[5]}} & tPj[79:64]);
end
wire [4:0]  kZ = q_wd ? (bh ? cnt + {2'b0, qaT} : cnt + {2'b0, qaN}) : cnt;   // count candidates: no issue
wire [4:0]  kI = q_wd ? (bh ? cI + {2'b0, qaT} : cI + {2'b0, qaN}) : cI;      // one instruction
wire [4:0]  kP = q_wd ? (bh ? cP + {2'b0, qaT} : cP + {2'b0, qaN}) : cP;      // a pair
assign      cnt_n = issue ? (pair ? kP : kI) : kZ;
wire [4:0]  thsb /* synthesis syn_keep = 1 */;   // the block's one-hot word count, kept as its own net
assign thsb = q_wd ? (bh ? qohT : qohN) : 5'd1;
function [15:0] thq; input [15:0] tt; input [4:0] oh;    // thermometer tt plus the one-hot count oh (0..4)
  integer a, b;
  begin
    for (a = 0; a < 16; a = a + 1) begin
      thq[a] = 1'b0;
      for (b = 0; b < 5; b = b + 1) thq[a] = thq[a] | (oh[b] & ((a < b) ? 1'b1 : tt[(a - b) < 0 ? 0 : (a - b)]));   // shift up by b, fill with ones
    end
  end
endfunction
always @* begin
  for (ti = 0; ti < 16; ti = ti + 1) begin                 // th shifted down by L0 (one instruction consumed)
    tI[ti] = 1'b0;
    for (tk = 1; tk <= 5; tk = tk + 1) tI[ti] = tI[ti] | (tlo[tk] & (((ti + tk) < 16) ? th[(ti + tk) & 15] : 1'b0));
  end
  // The consumption selects the base thermometer first; the block's word count (BTC RAM output: qohT, bh) enters as
  // five one-hot shift selects in the last AND-OR. Before, the BTC RAM output passed qohT and the whole thq into th.
  th_n = thq(issue ? (pair ? tP : tI) : th, thsb);
end
// A hold entry's move. While the hold entry waits in DS, W0's length follows DS's length (at most 5, hl0 keeps the
// predecoded one); DS's issue (hiss) starts the move (hmv), five words a clock until hr is used up, then the extractor
// runs again. hmv replaced an earlier register of hiss, so hiss itself reaches no select.
wire [3:0]  hT  = il_ov ? il : hl0;                          // DS's length of the hold entry
wire [3:0]  hT5 = (hT > 4'd5) ? 4'd5 : hT;                   // the first step moves at most 5 words
wire [3:0]  hr5 = (hr > 4'd5) ? 4'd5 : hr;                   // each further step too
// hmv's next value as one kept LUT for the copies (the flush reached them through a shared reset net of 77 loads before)
wire        hmvn /* synthesis syn_keep = 1 */;
assign      hmvn = !rst && !flush && !dfl_r && (hmv ? (hr != 4'd0) : hiss);
always @(posedge clk) begin
  hpf <= sv && (rc < (il_ov ? il : (hst ? hl0 : wql)));      // the hold entry's mark at its real length (DS reads hpf)
  if (!hst) hl0 <= wql;                                       // remember the predecoded length while no hold entry waits
  if (rst || flushx) begin hst <= 1'b0; hmv <= 1'b0; hr <= 4'd0; end
  else begin
    if (hmv && hr == 4'd0) hst <= 1'b0; else if (ex1 && hold0) hst <= 1'b1;   // a hold entry written / moved past
    if (hmv) begin hmv <= (hr != 4'd0); hr <= hr - hr5; end                   // further move steps
    else if (hiss) begin hmv <= 1'b1; hr <= hT - hT5; end                     // DS issued it: start the move
  end
  hmvc <= {NQ{hmvn}};                                         // one copy of hmv per queue position
end
// The queue after this clock. issue and pair (copies per position) select last; L1 one-hot from the wlo copy of the
// position.
wire [29:0] Wn [0:NQ-1];
wire        isT = !hst && ib_sp1 && wv0d && !hold0;           // issue without a hold move (ex1 && !hold0 when !hmv)
wire        prP = isT && ib_sp2 && x1d;                      // pair (when !hmv)
generate for (j = 0; j < NQ; j = j + 1) begin : wn
  // One preserved copy of hmv per position makes each copy a LUT of its own (Gowin merged the syn_keep copies into two
  // nets of 120-150 loads).
  wire        is_k = isT || hmvc[j];     // this position moves by L0 (or by a hold step)
  wire        pr_k = prP && !hmvc[j];    // ... by L0 + L1
  wire [5:1]  wo = rs[j].v.wlo;          // this position's copy of L0
  // L1 as a one-hot (1, 2 or 3 words), from the lengths at W1-W5 selected by L0
  wire        x1 = (wo[1] & (W1[8:7] == 2'd1)) | (wo[2] & (W2[8:7] == 2'd1)) | (wo[3] & (W3[8:7] == 2'd1)) |
                   (wo[4] & (W4[8:7] == 2'd1)) | (wo[5] & (W5[8:7] == 2'd1));
  wire        x2 = (wo[1] & (W1[8:7] == 2'd2)) | (wo[2] & (W2[8:7] == 2'd2)) | (wo[3] & (W3[8:7] == 2'd2)) |
                   (wo[4] & (W4[8:7] == 2'd2)) | (wo[5] & (W5[8:7] == 2'd2));
  wire [3:0]  wk = is_k ? (pr_k ? (x1 ? gb[j+1] : x2 ? gb[j+2] : gb[j+3]) : gb[j]) : bk[j][3:0];   // block word weights
  assign Wn[j] = (is_k ? (pr_k ? (x1 ? rb[j+1] : x2 ? rb[j+2] : rb[j+3]) : rb[j]) : zb[j]) |
                 ({30{wk[0]}} & BW0) | ({30{wk[1]}} & BW1) | ({30{wk[2]}} & BW2) | ({30{wk[3]}} & BW3);
  if (j == 0) begin : oh                 // W0's one-hot length for the wlo copies: candidates decoded before the late select
    wire [5:1] o [0:3];
    genvar m;
    // the queue words' part (registers) and the block words' part apart: each block word gets one weight from the
    // selects (ek), so the block's predecode (I-cache / predecode RAM output) passes one AND-OR to the length
    wire [3:0] gq [0:3];
    for (m = 0; m < 4; m = m + 1) begin : om
      wire [5:1] aa = rs[m].v.a;
      assign gq[m] = rs[m].v.g;
      assign o[m] = ({5{aa[1]}} & d5q[m + 1]) | ({5{aa[2]}} & d5q[m + 2]) | ({5{aa[3]}} & d5q[m + 3]) | ({5{aa[4]}} & d5q[m + 4]) |
                    ({5{aa[5]}} & d5q[m + 5]);
    end
    wire [5:1] o0 = o[0], o1 = o[1], o2 = o[2], o3 = o[3];
    wire [5:1] oz = {5{vq[0]}} & d5q[0];
    // The hold override (w0ov, registers) is folded into the weights and the queue parts (kept, early), so the block's
    // predecode (I-cache RAM output) passes one AND-OR to each case. Before, the RAM output went through d5b and then a
    // select with the override into wlo and xlo, a failing path of the full system.
    wire [5:1] mz /* synthesis syn_keep = 1 */, m0 /* synthesis syn_keep = 1 */, mp /* synthesis syn_keep = 1 */;
    wire [3:0] kz /* synthesis syn_keep = 1 */, k0 /* synthesis syn_keep = 1 */, kp /* synthesis syn_keep = 1 */;
    assign mz = w0ov ? w0lv : oz;                       // no move: the queue part of W0's length
    assign m0 = w0ov ? w0lv : o0;                       // one instruction
    assign mp = w0ov ? w0lv : (x1 ? o1 : x2 ? o2 : o3); // a pair
    assign kz = {4{!w0ov}} & bk[0][3:0];                // the block word weights for the same three cases
    assign k0 = {4{!w0ov}} & gq[0];
    assign kp = {4{!w0ov}} & (x1 ? gq[1] : x2 ? gq[2] : gq[3]);
    wire [5:1] c0 = cwk(mz, kz), c1 = cwk(m0, k0), cp = cwk(mp, kp);
    assign w0no = is_k ? (pr_k ? cp : c1) : c0;         // the late select
  end
  if (j == 0) begin : h
    always @(posedge clk) Qv[29:0] <= W0n;                // W0 gets the hold length override
  end else begin : o
    always @(posedge clk) Qv[j*30 +: 30] <= Wn[j];
  end
end endgenerate
// W0 after this clock: while a hold entry waits or moves, its length field holds DS's length (at most 5 per step)
assign W0n = {Wn[0][29:11], (hst && !hmv) ? hT5 : (hmv && hr != 4'd0) ? hr5 : Wn[0][10:7], Wn[0][6:0]};

endmodule
`default_nettype wire
