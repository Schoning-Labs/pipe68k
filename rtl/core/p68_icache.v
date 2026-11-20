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
// ==== p68_icache.v: the instruction cache ============================================================================
//
// What it is. An 8 KB direct-mapped instruction cache with 32-byte lines, read 64 bits (four 16-bit words, one "fetch
// block") per clock, in block RAM. It sits inside the fetch stage (IF, p68_fetch.v), which gives it the read address
// (fpc) every clock and takes the block (blk) and its predecode (bpd) one clock later. On a miss it reads the line from
// memory through the memory port unit (p68_mem.v, 4 beats of 64 bits). It runs on the CPU clock.
// A real 68020 has a 256-byte instruction cache; the 8 KB here is a choice of this core, which has the block RAM for
// it. Both caches of the core are 8 KB direct mapped; the D-cache has 16-byte lines.
//
// Address split for a 32-bit address: bits 2-0 pick the byte in the 8-byte block, bits 4-3 the block in the line, bits
// 12-5 the line (256 lines), bits 31-13 are the tag. A tag entry is 20 bits: valid in bit 19, address bits 31-13 below.
//
// How it works, part by part:
//   - Data and predecode. Each data entry is 112 bits: the 64-bit block and the 48-bit predecode of its four words
//     (12 bits per word, see p68_pdf.vh). The predecode is computed from the fill data on the way in (p68_pdrom.v) and
//     stored with the line, so it is never computed on the path from the cache to the fetch queue. That costs 36 more
//     bits per entry than the length alone would, and 2 more block RAMs, which was a good trade on this chip.
//   - Line buffer. A miss fills the line buffer (lbi, 32 bytes) first; fetch can read from the line buffer as soon as
//     the line is there, before it is installed in the cache. The line buffer compare is formed in the read clock and
//     registered, so it only adds one 2:1 select in front of the queue. With allocation on (CACR E = 1, F = 0) the line
//     is then installed: tag invalidated, 4 data writes, tag written valid.
//   - Tags in three copies, each its own block RAM so each has its own read port: the fetch copy (tf, read at fpc), the
//     store copy (to, read at the store address of the uop in OC and compared one clock later), and the snoop copy
//     (tx, read at the external snoop address). All three are written together through one write port, in priority
//     order: own-store invalidation first, then external snoop, then CACR clear entry (CE), the clear sweep, install.
//   - Self-modifying code. A store committing in EX that hits a valid I-cache line, the line buffer, or one of the last
//     four lines that fed the fetch queue raises smc (OC makes the uop behind it replay, which flushes the queue and
//     refetches) and invalidates the line (written in the next clock, before the refetch reads the tags). A store or
//     snoop that meets a line being filled or installed makes it stale: it is not installed and the line buffer is
//     marked invalid. A real 68020 needs a CACR clear after code is written; this core does it in hardware, because old
//     Amiga games write code and run it without clearing any cache.
//   - External snoop (a DMA write by the Amiga chipset, m_snoop): invalidates a matching line and the line buffer; the
//     fetch queue is not flushed (a 68020 does not snoop at all, so software cannot expect more than this).
//
// Timing notes, measured in the full A1200 system:
//   - The fill predecode comes two clocks after its beat (block ROM tables). In the full 720p system the last slow fetch
//     path ran from the predecode ROM output straight into the queue's length logic (142 failing endpoints). That bypass
//     was removed: a read of the last beat's block in the clock its predecode arrives waits one clock (lbw3, busy) or,
//     when a flush target reads it, gets no line-buffer hit (lbn) and refetches without a new fill (lbx). CPI went from
//     1.021 to 1.031 for it, and this was the build that first met 66.70 MHz in the full system.
//   - Several decisions are formed for both possible read addresses (the flush target fft and the normal address fpa)
//     and selected last by ffl, because the flush arrives late in the clock.
//   - The tag RAM write data always carries fa's tag bits; only the valid bit says whether a write installs or clears.
//     That took the snoop tag compare off the tag RAM's data input.
//   - The snoop address registers are this cache's own (the preserve attribute on each one): the synthesizer had merged
//     them with the D-cache's copies, which made one shared 3.4 ns net. The attribute is on every declaration separately,
//     because on a declaration list it only applies to the last name.
`default_nettype none
module p68_icache (
  input  wire        clk,
  input  wire        rst,
  input  wire [3:0]  cacr,           // CACR bits 3-0 as committed: C (clear), CE (clear entry), F (freeze), E (enable)
  input  wire        clr_all,        // a MOVEC to CACR with C set committed: clear the whole cache (sweep)
  input  wire        clr_ent,        // ... with CE set: clear the line at caar
  input  wire [31:0] caar,           // cache address register (for CE)
  // fetch
  input  wire [31:0] fpc,            // read address this clock
  input  wire [31:0] fpa,            // fpc unless a flush with a registered target (ffl) comes: the read address
  input  wire        ffl,            //   is ffl ? fft : fpa; the line buffer compare is formed for both, ffl selects last
  input  wire [31:0] fft,            // the flush target
  input  wire [31:0] if2_pc,         // address of the block read in the last clock
  output wire [63:0] blk,            // the block read in the last clock (four words)
  output wire [47:0] bpd,            // predecode of the four words of blk (from the cache or the line buffer)
  input  wire [47:0] rpd,            // predecode of the four words of a fill beat (p68_pdrom, two clocks after the beat)
  output wire        hit,            // blk is valid (cache or line buffer hit)
  input  wire        miss,           // fetch missed at if2_pc: fill that line
  output wire        busy,           // a fill is running (or the last beat's predecode is not there yet): fetch waits
  input  wire        q_wr,           // words of the block at if2_pc entered the queue
  // own stores
  input  wire [31:0] oc_saddr,       // store address of the uop in OC (the store tag copy is read here)
  input  wire        ex_stc,         // the store in EX commits
  input  wire [31:0] ex_saddr,       // its address
  input  wire        wb_st,          // a store in WB
  input  wire [31:0] wb_saddr,       // its address (checked against a running fill)
  input  wire        wb_x2,          // the store in WB crosses a longword (its second word is written next)
  input  wire [31:0] wb_saddr2,      // ... at this address
  output wire        smc,            // the committing store hit fetched code
  // external snoop
  input  wire        sn_v,           // DMA write
  input  wire [31:2] sn_addr,        // ... at this address
  // memory unit
  output reg         ir_req,         // read request (held until ir_go)
  output wire [31:2] ir_addr,        // line address of the fill
  input  wire        ir_go,          // granted
  input  wire        r_i,            // a beat for the I-side
  input  wire [63:0] r_data,         // the beat
  input  wire        r_done          // the last beat
) /* synthesis syn_maxfan = 16 */;
wire        en = cacr[0];            // CACR E: the cache is enabled

// ---- tags ---------------------------------------------------------------------------------------------------------
reg         tw_en; reg [7:0] tw_idx; reg [19:0] tw_val;    // the tag write port: enable, line, value (combinational, below)
reg  [19:0] tf [0:255]; reg [19:0] to [0:255]; reg [19:0] tx [0:255];   // fetch, store and snoop copies of the tags
// The second word of a store that crosses a longword is checked in its second WB clock (EX and the store tag port are
// free then, because AG stalls behind such a store); the store tag copy is read at that word one clock before.
reg         s2;                      // the second word of a crossing store is checked this clock
wire [31:0] to_ra = wb_x2 ? wb_saddr2 : oc_saddr;   // read address of the store tag copy
reg  [31:0] ck_a;                    // the address checked this clock (s2 ? wb_saddr : ex_saddr), as a register loaded with their next values
always @(posedge clk) ck_a <= wb_x2 ? wb_saddr2 : oc_saddr;
wire        ck_v  = s2 || ex_stc;    // a store is checked this clock
reg  [19:0] tf_q, to_q, tx_q;        // registered read outputs of the three tag copies
reg  [31:2] sn1_a /* synthesis syn_preserve = 1 */;   // snoop address, first stage (own copy, apart from the D-cache's)
reg  [31:2] sn2_a /* synthesis syn_preserve = 1 */;
reg         sn1_v /* synthesis syn_preserve = 1 */;
reg         sn2_v /* synthesis syn_preserve = 1 */;
// one write port for all three copies (block RAM write), and one read per copy (block RAM synchronous read)
always @(posedge clk) if (tw_en) begin tf[tw_idx] <= tw_val; to[tw_idx] <= tw_val; tx[tw_idx] <= tw_val; end
always @(posedge clk) begin
  tf_q <= tf[fpc[12:5]];                 // fetch: the line of the block read now
  to_q <= to[to_ra[12:5]];               // store check: the line of the store in OC (compared in EX next clock)
  tx_q <= tx[sn1_a[12:5]];               // snoop: the line of the snoop in its first stage
end
// the tag write of the last clock, so a read of the same line in the same clock (which sees the old RAM value) can be
// corrected by a compare (to_col, tx_col below)
reg         twd; reg [7:0] twd_idx; reg [19:0] twd_val;
always @(posedge clk) begin twd <= tw_en; twd_idx <= tw_idx; twd_val <= tw_val; end

// ---- data ---------------------------------------------------------------------------------------------------------
reg  [111:0] dm [0:1023];            // data and predecode: 1024 blocks of 64 data bits and 48 predecode bits
reg  [111:0] dq;                     // registered read output
reg         dw_en; reg [9:0] dw_idx; reg [111:0] dw_d;      // the data write port (combinational, below)
always @(posedge clk) if (dw_en) dm[dw_idx] <= dw_d;
always @(posedge clk) dq <= dm[fpc[12:3]];   // read the block at fpc (line and block number)

// ---- line buffer and fetch hit ------------------------------------------------------------------------------------
reg  [255:0] lbi;                    // the line buffer: the 32 bytes of the line being filled, block 0 in bits 255:192
reg  [31:5]  lbi_tag;                // its line address
reg          lbi_v;                  // it holds a complete, valid line
reg          sw_act;                 // clear sweep running (CACR C, reset): hits off, all tags written invalid one by one
reg  [7:0]   sw_cnt;                 // the line the sweep clears now
reg          lbh_q, col_q;           // registered: the read hits the line buffer; a tag write may have hit the read line
reg  [63:0]  lbw_q;                  // the line buffer's block at the read address
// predecode of the line buffer (lbp, written with each beat from rpd), so the queue takes the predecode from the cache
// or the line buffer instead of computing it behind the block RAM
reg  [191:0] lbp;
wire [47:0]  lbp_q;                  // the line buffer's predecode at the read address
// The block select of the line buffer follows a register of the read address (the line buffer does not change between
// the read clock and the next one while lbh_q can be set, and the select arrives before the block RAM data anyway).
reg  [1:0]   fq34;                   // fpc[4:3] of the last clock: the block in the line
reg  [1:0]   rpb;                    // the predecode of a beat is written one clock after its data (from the pdrom's register)
reg          rpw;                    // ... the predecode write of the beat rpb happens now
// The fill predecode comes two clocks after the beat (p68_pdrom), so lbp gets the last beat one clock after the line
// buffer can first hit. There is no bypass from rpd, because the predecode RAM output feeding the queue's length logic
// was the worst real path: a read of the last beat's block in that clock waits one clock (busy, lbw3) or, when a flush
// target reads it, gets no line buffer hit (lbn) and replays without a new fill (lbx).
reg          lbx;                    // the last read missed only because of lbn: fetch rereads, the fill is not repeated
reg          rpw1; reg [1:0] rpb1;   // the beat whose predecode arrives one clock later (one stage before rpw / rpb)
wire         lbhA = lbi_v && (lbi_tag == fpa[31:5]);   // the line buffer holds the normal read line
wire         lbhT = lbi_v && (lbi_tag == fft[31:5]);   // ... the flush target's line
wire         lbn  = rpw1 && (rpb1 == fpc[4:3]);        // the block read now is the beat whose predecode lbp gets this clock
wire         lbw3 = rpw1 && (rpb1 == 2'd3) && (fpa[4:3] == 2'd3);   // registers only: the replay after the fill waits
// Any waiting tag writer at index ri. It is a superset of the one that really writes: no port priority and no snoop
// tag compare (x_hit) in it, so it is quick. A false collision only costs a refetch.
function colf; input [7:0] ri;              // a tag write of this clock may hit read index ri
  colf = (oi_p && oi_idx == ri) || (xi_p && xi_idx == ri) || (sn2_v && sn2_a[12:5] == ri) || (ce_p && ce_idx == ri) ||
         (sw_act && sw_cnt == ri) || (fs == 2'd3 && (ist == 3'd0 || ist == 3'd5) && fa[12:5] == ri);
endfunction
wire         colT = colf(fft[12:5]), colA = colf(fpa[12:5]);   // for the flush target and for the normal read
always @(posedge clk) begin
  lbh_q <= (ffl ? lbhT : lbhA) && !lbn;  // line buffer hit (not for the block whose predecode is still on its way)
  lbx   <= (ffl ? lbhT : lbhA) && lbn;   // ... that case: reread without a new fill
  fq34  <= fpc[4:3];
  // a superset of the tag-write collision that does not wait for the install's stale check (a false collision only
  // costs a refetch); formed for both read addresses (fft, fpa) with ffl selecting last, because the flush comes late
  col_q <= ffl ? colT : colA;
end
always @* begin
  lbw_q = (fq34 == 2'd0) ? lbi[255:192] : (fq34 == 2'd1) ? lbi[191:128] : (fq34 == 2'd2) ? lbi[127:64] : lbi[63:0];
end
assign lbp_q = (fq34 == 2'd0) ? lbp[191:144] : (fq34 == 2'd1) ? lbp[143:96] : (fq34 == 2'd2) ? lbp[95:48] : lbp[47:0];
// hit: the line buffer, or a valid matching tag while the cache is on, no sweep runs and no tag write collided
assign hit = lbh_q || (en && !sw_act && !col_q && tf_q[19] && (tf_q[18:0] == if2_pc[31:13]));
assign blk = lbh_q ? lbw_q : dq[111:48];   // the block: line buffer or cache data
assign bpd = lbh_q ? lbp_q : dq[47:0];     // its predecode from the same place

// ---- lines that fed the queue (self-modifying code check) ---------------------------------------------------------
// The last four distinct lines whose words entered the fetch queue. A store into one of them may change an instruction
// that is already fetched (in the queue or the instruction buffer) even when the line is not in the cache.
reg  [31:5] ff0, ff1, ff2, ff3;      // their line addresses, ff0 the newest
reg  [3:0]  ffv;                     // valid bits
always @(posedge clk) begin
  if (rst) ffv <= 4'h0;
  else if (q_wr && !(ffv[0] && ff0 == if2_pc[31:5])) begin   // a new line entered the queue: shift it in
    ff0 <= if2_pc[31:5]; ff1 <= ff0; ff2 <= ff1; ff3 <= ff2; ffv <= {ffv[2:0], 1'b1};
  end
end

// ---- own stores ---------------------------------------------------------------------------------------------------
always @(posedge clk) s2 <= !rst && wb_x2;   // the crossing store's second word is checked in the next clock
wire        to_col = twd && (twd_idx == ck_a[12:5]);      // the store tag read missed last clock's tag write
wire [19:0] to_e   = to_col ? twd_val : to_q;             // the corrected tag
wire        o_hit  = to_e[19] && (to_e[18:0] == ck_a[31:13]);   // the store hits a valid cache line
wire        o_lb   = lbi_v && (lbi_tag == ck_a[31:5]);    // ... the line buffer
wire        o_ff   = (ffv[0] && ff0 == ck_a[31:5]) || (ffv[1] && ff1 == ck_a[31:5]) ||
                     (ffv[2] && ff2 == ck_a[31:5]) || (ffv[3] && ff3 == ck_a[31:5]);   // ... a line that fed the queue
assign      smc    = ck_v && (o_hit || o_lb || o_ff);     // the store may have changed fetched code
reg         oi_p; reg [7:0] oi_idx;  // own-store invalidation, written in the next clock (first on the port)

// ---- fill and install ---------------------------------------------------------------------------------------------
reg  [1:0]  fs;                      // fill state: 0 idle, 1 request, 2 beats, 3 install
reg  [31:5] fa;                      // line address of the fill
reg  [1:0]  fb;                      // beat
reg  [2:0]  ist;                     // install step: 0 invalidate, 1-4 data, 5 tag
reg         fstale;                  // a store or snoop hit the line during the fill: do not install it
assign      ir_addr = {fa, 3'b000};  // the line address in longwords
assign      busy    = (fs == 2'd1) || (fs == 2'd2) || lbw3;   // fetch waits during the fill and for the last predecode
wire        st_f = wb_st && (wb_saddr[31:5] == fa) && fs != 2'd0;   // a store in WB hits the fill line
wire        sn_f = sn1_v && (sn1_a[31:5] == fa) && fs != 2'd0;      // a snoop hits it
// The own-store address compare against the fill line is a register (fa does not change while fs != 0, and the first
// clock of fs == 1, when it may still hold the old line, reads no stale check).
reg         exf_m;
always @(posedge clk) exf_m <= (to_ra[31:5] == fa);
wire        ex_f = ck_v && exf_m && fs != 2'd0;          // the store checked now hits the fill line
wire        stale_now = fstale || st_f || sn_f || ex_f;  // the line is stale (now or earlier in the fill)

// ---- external snoop -----------------------------------------------------------------------------------------------
// Two register stages: stage 1 reads the snoop tag copy and checks the line buffer, stage 2 compares the tag.
always @(posedge clk) begin
  if (rst) begin sn1_v <= 1'b0; sn2_v <= 1'b0; end
  else begin sn1_v <= sn_v; sn2_v <= sn1_v; end
  sn1_a <= sn_addr; sn2_a <= sn1_a;
end
wire        tx_col = twd && (twd_idx == sn2_a[12:5]);    // the snoop tag read missed last clock's tag write
wire [19:0] tx_e   = tx_col ? twd_val : tx_q;            // the corrected tag
wire        x_hit  = sn2_v && tx_e[19] && (tx_e[18:0] == sn2_a[31:13]);   // the snoop hits a valid line
reg         xi_p; reg [7:0] xi_idx;  // external invalidation waiting for the port

// ---- tag port -----------------------------------------------------------------------------------------------------
reg         ce_p; reg [7:0] ce_idx;  // a CACR clear entry waiting for the port
reg         g_inst;                  // install step uses the port this clock
always @* begin
  // The tag bits are always fa's; only the valid bit says whether a write installs (so the snoop compare x_hit is not
  // in front of the tag RAM's data input). Priority: own store, waiting snoop, snoop, clear entry, sweep, install.
  tw_en = 1'b0; tw_idx = oi_idx; tw_val = {1'b0, fa[31:13]}; g_inst = 1'b0;
  if (oi_p) begin tw_en = 1'b1; tw_idx = oi_idx; end
  else if (xi_p) begin tw_en = 1'b1; tw_idx = xi_idx; end
  else if (x_hit) begin tw_en = 1'b1; tw_idx = sn2_a[12:5]; end
  else if (ce_p) begin tw_en = 1'b1; tw_idx = ce_idx; end
  else if (sw_act) begin tw_en = 1'b1; tw_idx = sw_cnt; end
  // The install step waits while a snoop is in stage 2 (sn2_v, a register), not only on its tag hit x_hit, so the
  // install (g_inst, which steps ist and fs) does not wait for the snoop tag compare.
  else if (fs == 2'd3 && (ist == 3'd0 || ist == 3'd5) && !stale_now && !sn2_v) begin
    tw_en = 1'b1; tw_idx = fa[12:5]; tw_val[19] = (ist == 3'd5); g_inst = 1'b1;   // step 0 writes invalid, step 5 valid
  end
end
// data writes of the install: steps 1-4 write the four blocks of the line buffer, with their predecode
always @* begin
  dw_en = (fs == 2'd3) && ist >= 3'd1 && ist <= 3'd4 && !stale_now;
  dw_idx = {fa[12:5], ist[1:0] - 2'd1};
  case (ist[1:0] - 2'd1)
    2'd0: dw_d = {lbi[255:192], lbp[191:144]};
    2'd1: dw_d = {lbi[191:128], lbp[143:96]};
    2'd2: dw_d = {lbi[127:64], lbp[95:48]};
    default: dw_d = {lbi[63:0], lbp[47:0]};
  endcase
end

// rpd is the predecode of the beat from p68_pdrom (two clocks after r_data); it reaches lbp before the line buffer can
// hit (lbi_v follows r_done; lbw3 and lbn cover the last beat) and before the install reads it
always @(posedge clk) begin
  rpw1 <= !rst && (fs == 2'd2) && r_i; rpb1 <= fb;      // a beat arrives now: its predecode comes in two clocks
  rpw <= rpw1; rpb <= rpb1;
  if (rpw)
    case (rpb) 2'd0: lbp[191:144] <= rpd; 2'd1: lbp[143:96] <= rpd; 2'd2: lbp[95:48] <= rpd; default: lbp[47:0] <= rpd; endcase
end
// the control state: invalidations waiting for the port, clear entry, sweep, and the fill state machine
always @(posedge clk) begin
  if (rst) begin
    fs <= 2'd0; ir_req <= 1'b0; lbi_v <= 1'b0; oi_p <= 1'b0; xi_p <= 1'b0; ce_p <= 1'b0;
    sw_act <= 1'b1; sw_cnt <= 8'd0;      // after reset the sweep clears all 256 tags before the first hit
  end else begin
    // own-store invalidation (lines found in EX are written in the next clock)
    oi_p <= ck_v && (o_hit || o_lb); oi_idx <= ck_a[12:5];
    if (ck_v && o_lb) lbi_v <= 1'b0;
    // external snoop: line buffer at stage 1, tag at stage 2 (waits one clock behind an own-store invalidation)
    if (sn1_v && lbi_v && sn1_a[31:5] == lbi_tag) lbi_v <= 1'b0;
    if (oi_p && x_hit) begin xi_p <= 1'b1; xi_idx <= sn2_a[12:5]; end
    else if (!oi_p) xi_p <= 1'b0;
    // CACR clear entry: waits until no invalidation uses the port
    if (clr_ent) begin ce_p <= 1'b1; ce_idx <= caar[12:5]; end
    else if (ce_p && !oi_p && !xi_p && !x_hit) ce_p <= 1'b0;
    // CACR clear: sweep over all 256 lines, one per clock while the port is free
    if (clr_all) begin sw_act <= 1'b1; sw_cnt <= 8'd0; end
    else if (sw_act && !oi_p && !xi_p && !x_hit && !ce_p) begin sw_cnt <= sw_cnt + 8'd1; if (sw_cnt == 8'd255) sw_act <= 1'b0; end
    case (fs)
      2'd0: begin
              fa <= if2_pc[31:5];   // loads every clock while idle (only read when fs != 0), so the hit is not in its enable
              if (miss && !lbx) begin fs <= 2'd1; ir_req <= 1'b1; fb <= 2'd0; fstale <= 1'b0; lbi_v <= 1'b0; end   // start a fill
            end
      2'd1: if (ir_go) fs <= 2'd2;               // granted; ir_req drops in state 2 (no second grant comes: the memory unit is busy)
      2'd2: begin                                // the beats arrive
        ir_req <= 1'b0;
        if (st_f || sn_f || ex_f) fstale <= 1'b1;
        if (r_i) begin
          case (fb) 2'd0: lbi[255:192] <= r_data; 2'd1: lbi[191:128] <= r_data;
                    2'd2: lbi[127:64] <= r_data;  default: lbi[63:0] <= r_data; endcase
          fb <= fb + 2'd1;
        end
        if (r_done) begin                        // the line is complete: it is in the line buffer from now on
          lbi_tag <= fa; lbi_v <= !stale_now;
          ist <= 3'd0;
          fs <= (!stale_now && en && !cacr[1] && !sw_act) ? 2'd3 : 2'd0;   // install unless stale, off, frozen (F) or sweeping
        end
      end
      default: begin                                   // install
        if (st_f || sn_f || ex_f) fstale <= 1'b1;
        if (stale_now) fs <= 2'd0;                     // abandoned
        else if (ist == 3'd5) begin if (g_inst) fs <= 2'd0; end   // the valid tag is written: done
        else if (ist != 3'd0 || g_inst) ist <= ist + 3'd1;      // step 0 waits for the port, data steps run every clock
      end
    endcase
  end
end
endmodule
`default_nettype wire
