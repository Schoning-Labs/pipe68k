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
// ==== p68_dcache.v: the data cache ===================================================================================
//
// Abbreviations used in this file: AG = address generation, OC1/OC2 = the two operand stages (p68_oc.v, which
// instantiates this unit), EX = execute, WB = write-back, DMA = direct memory access (the Amiga chipset writing memory
// on its own), CACR/CAAR = the 68020's cache control register and cache address register, CAS = compare-and-swap,
// lb = the line buffer, ub = the buffer for the second word of a split load, BSRAM = Gowin's block RAM.
//
// ---- What a cache is, and what this one is like -------------------------------------------------------------------
//
// Main memory (DDR3 on the Tang board, or the Amiga's chip RAM) takes many clocks to answer, while the CPU wants a
// load every clock. A cache is a small fast memory on the chip that keeps copies of recently used memory, so most
// loads find their data there (a "hit") and only the rest (a "miss") wait for memory. This one holds 8 KB in 512 lines
// of 16 bytes. It is direct mapped: the address bits 12-4 (the "index") pick the one line an address can live in, and
// the line remembers the address bits 31-13 it holds (the "tag") plus a valid bit. A load hits when the tag stored at
// its index equals its own bits 31-13. Two addresses with the same index (8 KB apart) push each other out, which is
// the price of the simplest and fastest kind of cache.
//
// It is write-through without allocation on a store miss: every store goes to memory through the store buffer in any
// case, and also updates the cached line if it is there; a store to a line that is not cached does not bring the line
// in. That keeps the cache and memory always equal, so nothing ever has to be written back, and a DMA write by the
// chipset only needs to invalidate the cached copy (a "snoop").
//
// ---- How it is built ------------------------------------------------------------------------------------------------
//
//  - Tags in three copies, all written together: the load copy is read at AG's load address, the store copy at AG's
//    store address, and the snoop copy at the external snoop address. A Gowin block RAM has one write port and one
//    read port in this use, so three independent reads in one clock need three copies (20 bits x 512 is small).
//  - Data in 16 byte lanes of 512 entries each, so a store can write single bytes and an install can write the whole
//    line in one clock. Each lane becomes a block RAM.
//  - A miss fills the line buffer (lb, 16 bytes, two 64-bit beats from the memory port). A load hits the cache or the
//    line buffer, so the replayed load after a miss hits the line buffer as soon as the fill is done. The line is then
//    installed from the line buffer in one clock (data and tag) when allocation is on (CACR E = 1, F = 0, a cacheable
//    address), no store is in WB, and no store in OC or EX has the line's index (their hit bits were computed from the
//    old tag).
//  - Uncached loads (chip RAM and I/O registers, from the uncached-area inputs) read one word into the line buffer
//    (lb_unc). It is used once, cleared when that load commits, and interrupts wait while it is pending (unc_hold), so
//    an I/O register is read exactly once per load. That matters on the Amiga, where reading some chipset registers
//    changes them.
//  - Stores update a hit line (in WB, with byte enables) and the line buffer. A store that meets a line being filled
//    makes the fill stale: the line buffer stays invalid and the line is not installed (the load replays and refills).
//  - Any tag write (install, invalidation) makes a load whose tag read happened in that clock replay (dkill), and a
//    store in that position takes its hit from the written tag.
//  - External snoop (chip-bus DMA write): sn1 registers the address, compares it with the line buffer and the fill, and
//    reads the snoop tag copy; sn2 compares the tag and invalidates the line (the snoop is the tag port's first user).
//    Loads of that line in OC during sn1 and sn2 replay. So a DMA write is seen by every load that commits after sn1.
//  - CACR: E (bit 0) enables hits and allocation, F (bit 1) freezes (hits, no allocation), CE (bit 2) clears the line
//    of CAAR's index, C (bit 3) clears all (a sweep of 512 clocks, one tag per clock; also after reset, with hits off).
//  - Misaligned accesses and CAS. OC gives the read address of the next word while a split access stays (la_x) and the
//    compare address of the word it needs (oc_addr). A split load whose first word misses fills the first word's line
//    (or uncached word) into the line buffer and, when the second word is uncached or in another line, reads the
//    second word with one more single-word read into ub (used once, by that load). A split load whose first word hits
//    and second misses (d_lh) fills the second word's line or word. CAS and CAS2 loads (oc_fu, d_fu) are uncached:
//    they miss in the cache and in a cached line buffer and read the word from memory, because the bus is locked for
//    their read-modify-write and the read must really happen on the bus.
//
// ---- What was tried, and why it ended up like this ----------------------------------------------------------------
//
// The install decision first came through the CAS condition in the same clock. It was taken out of that condition, and
// finally the store checks became one register (stb) formed a clock early as a superset of the exact checks, which
// removed 38 failing endpoints in the full Amiga system. Synthesis had once merged this cache's snoop address registers
// with the I-cache's, which gave a 3.4 ns net across the chip; each cache now keeps its own, with one syn_preserve
// attribute per register (on a declaration list the attribute only applies to the last name, which is how the first
// attempt failed). The read index for the data RAMs is computed in copies next to the RAMs (one for the tags, one per
// four data lanes), because one index net into sixteen block RAMs spread over the chip was a long path. The uncached
// map once matched A23-A20 in every 16 MB, so 32-bit Zorro III RAM was uncached in 7 of every 16 MB; it was limited to
// the 24-bit area, and the two inputs for the high address space came later (see p68_core.v for the story of cac_hi).
//
// Runs on the CPU clock clk.
`default_nettype none
module p68_dcache (
  input  wire        clk,            // CPU clock
  input  wire        rst,            // synchronous reset (starts the clear sweep)
  input  wire [3:0]  cacr,           // CACR bits 3-0: C, CE, F, E
  input  wire        clr_all,        // CACR written with C = 1 (one clock)
  input  wire        clr_ent,        // CACR written with CE = 1 (one clock)
  input  wire [31:0] caar,           // CAAR: the line CE clears is CAAR's index
  input  wire [15:0] unc_map,        // bit k: addresses with A23-A20 = k are uncached, in the 24-bit area only
  input  wire [7:0]  unc_hi,         // A31-A25 = unc_hi[6:0] is uncached when unc_hi[7] (a 32 MB board, e.g. RTG)
  input  wire [6:0]  cac_hi,         // above 16 MB, cache only the RAM board window (details in p68_core.v)
  // ---- AG (read addresses, this clock) ----
  input  wire [31:0] ea_s,           // AG's load address; the read index is la_x[12:4] instead while the access stays
  input  wire [12:4] la_x,           // index of the next word of a staying (split) access
  input  wire [31:0] saddr,          // AG's store address (read index of the store tag copy)
  // ---- OC1 ----
  input  wire        oc_v,           // a uop is in OC1
  input  wire        oc_ld,          // the uop in OC1 loads
  input  wire        oc_st,          // the uop in OC1 stores
  input  wire [31:0] oc_addr,        // the word whose hit and data are wanted this clock
  input  wire [31:0] oc_saddr,       // the store address of the uop in OC1
  output wire [31:0] dq,             // the word at oc_addr (cache or line buffer)
  output wire        dhit,           // ... is valid (hit in ub, the line buffer or the cache)
  output wire        st_hit,         // the store's line is in the cache (WB writes the cache only then)
  output wire        dkill,          // a load at oc_addr must replay (snoop, tag written while it read, used uncached word)
  output wire [31:0] dq_c, dq_l,     // the cache word and the line buffer word at oc_addr (dq's sources, for OC's AND-OR)
  output wire        lbh,            // dq is the line buffer word (when not ub)
  output wire [31:0] dq_u,           // the ub word
  output wire        ubh,            // dq is ub
  // ---- misaligned accesses and CAS ----
  input  wire [31:0] oc_a0,          // first word of the load in OC1 (snoop check in the last clock of a split load)
  input  wire        oc_stay,        // the uop in OC1 stays (split access): it does not go on this clock
  input  wire [4:0]  st_i,           // copies of oc_stay (from copies of oc_v) for the read index copies
  input  wire        oc_x0,          // first clock of the uop in OC1 (oc_addr is its first word)
  input  wire        oc_x2,          // oc_addr is the second word of a split load
  input  wire        oc_fu,          // the load in OC1 is uncached by force (CAS/CAS2)
  output wire        ct_hit,         // tag hit at oc_addr (second word of a split store; no enable, like st_hit)
  output wire        sn_a0,          // a snoop in sn1/sn2 on oc_a0's line
  input  wire        d_fu,           // the replayed load in EX is uncached by force
  input  wire        d_lh,           // the replayed load in EX is split: first word hit, second word missed
  // ---- OC2 (between OC1 and EX) ----
  input  wire        q_ld,           // the uop in OC2 loads (valid)
  input  wire        q_x,            // ... it is split: its second word is at q_a1
  input  wire [31:2] q_a0, q_a1,     // ... its first and second word
  input  wire        q_st,           // the uop in OC2 stores (valid)
  input  wire [31:0] q_sa,           // ... at this address
  output wire        sn_q,           // a snoop in sn1/sn2 on the OC2 load's lines: it replays (it commits after the snoop)
  // ---- EX ----
  input  wire        ex_v,           // a uop is in EX
  input  wire        ex_st,          // the uop in EX stores
  input  wire [31:0] ex_saddr,       // ... at this address
  input  wire        ex_commit,      // the uop in EX commits
  input  wire        d_miss,         // the replayed load in EX missed (not when an interrupt is taken in front of it)
  input  wire [31:0] d_addr,         // its address
  input  wire [31:2] d_addr1,        // d_addr + 4 (registered in EX)
  input  wire [1:0]  d_lsz,          // its load size
  // ---- WB ----
  input  wire        wb_st,          // WB stores
  input  wire        wb_hit,         // ... and the line is cached
  input  wire [31:0] wb_saddr,       // ... at this address
  input  wire [31:0] sd,             // ... this data
  input  wire [3:0]  sbe,            // ... these bytes
  // ---- external snoop ----
  input  wire        sn_v,           // a DMA write to sn_addr
  input  wire [31:2] sn_addr,
  // ---- memory unit ----
  output reg         dr_req,         // read request (held until dr_go)
  output wire [31:2] dr_addr,        // the line (16-byte aligned) or, uncached, the word
  output reg         dr_unc,         // uncached: one word, the bytes dr_be
  output reg  [3:0]  dr_be,
  input  wire        dr_go,          // the memory unit took the request
  input  wire        r_d,            // a read beat on r_data (64 bits, big-endian)
  input  wire [63:0] r_data,
  input  wire        r_done,         // the read ends
  output wire        unc_hold        // an uncached read is in flight or its word is unused: hold interrupts
) /* synthesis syn_maxfan = 16 */;

// ---- Tag RAMs and the tag write port (one write per clock, into all three copies) ----------------------------------
// A tag entry is 20 bits: bit 19 valid, bits 18-0 the address bits 31-13.
reg         tw_en; reg [8:0] tw_idx; reg [19:0] tw_val;   // the tag write of this clock (combinational, below)
reg  [19:0] tl [0:511]; reg [19:0] ts [0:511]; reg [19:0] tn [0:511];   // load, store and snoop copies
reg  [19:0] tl_q, ts_q, tn_q;                             // their registered read data (block RAM output registers)
always @(posedge clk) if (tw_en) begin tl[tw_idx] <= tw_val; ts[tw_idx] <= tw_val; tn[tw_idx] <= tw_val; end
// The snoop address registers are this cache's own: synthesis had merged them with the I-cache's. There is one
// attribute per name, because on a declaration list it applied to the last name only, and the full system still read
// the I-cache's sn1_a here.
reg  [31:2] sn1_a /* synthesis syn_preserve = 1 */;   // snoop address, first clock
reg  [31:2] sn2_a /* synthesis syn_preserve = 1 */;   // ... second clock
reg         sn1_v /* synthesis syn_preserve = 1 */;   // a snoop in its first clock
reg         sn2_v /* synthesis syn_preserve = 1 */;   // ... in its second clock
// The read index (oc_stay ? la_x : ea_s) in copies near the RAMs: one for the tag copy, one per four data lanes. Each
// copy is selected by its own copy of oc_stay (st_i), so no single net drives all sixteen block RAM addresses.
wire [8:0]  ra_t /* synthesis syn_keep = 1 */;
wire [8:0]  ra_0 /* synthesis syn_keep = 1 */, ra_1 /* synthesis syn_keep = 1 */;
wire [8:0]  ra_2 /* synthesis syn_keep = 1 */, ra_3 /* synthesis syn_keep = 1 */;
assign ra_t = st_i[4] ? la_x : ea_s[12:4];
assign ra_0 = st_i[0] ? la_x : ea_s[12:4]; assign ra_1 = st_i[1] ? la_x : ea_s[12:4];
assign ra_2 = st_i[2] ? la_x : ea_s[12:4]; assign ra_3 = st_i[3] ? la_x : ea_s[12:4];
wire [35:0] ra_l = {ra_3, ra_2, ra_1, ra_0};               // the four data index copies side by side
// Synchronous reads: the address is presented in AG's clock and the data is there in OC1's clock.
always @(posedge clk) begin
  tl_q <= tl[ra_t];                  // load tag at the load address
  ts_q <= ts[saddr[12:4]];           // store tag at the store address
  tn_q <= tn[sn1_a[12:4]];           // snoop tag at the snoop address
end
reg         twd; reg [8:0] twd_idx; reg [19:0] twd_val;    // the tag write of the last clock (a read in that clock got the old tag)
always @(posedge clk) begin twd <= tw_en; twd_idx <= tw_idx; twd_val <= tw_val; end

// ---- Data RAMs: 16 byte lanes x 512 ----------------------------------------------------------------------------------
reg  [127:0] lb;                     // line buffer: the 16 bytes of the last fill (byte 0 of the line in [127:120])
reg  [31:4]  lb_tag;                 // the line address it holds
reg  [3:2]   lb_w;                   // the word of an uncached entry
reg          lb_v, lb_unc;           // the line buffer is valid; it holds one uncached word
reg          inst;                   // install the line buffer this clock (combinational)
reg          ex_ud;                  // the uop in EX read the uncached word in OC
wire [3:0]   wbe = sbe & {4{wb_st && wb_hit}};               // WB's byte enables, only for a cached line
wire [8:0]   dw_idx = inst ? lb_tag[12:4] : wb_saddr[12:4];  // write index: the install, else the WB store
wire [127:0] dw_d   = inst ? lb : {4{sd}};                   // write data: the whole line, or the store word in all four places
reg  [15:0]  dw_be;                  // byte enables for the 16 lanes
always @* begin
  dw_be = 16'h0;
  if (inst) dw_be = 16'hFFFF;        // an install writes all 16 bytes
  else case (wb_saddr[3:2])          // a store writes the four lanes of its word
    2'd0: dw_be[15:12] = wbe;
    2'd1: dw_be[11:8]  = wbe;
    2'd2: dw_be[7:4]   = wbe;
    default: dw_be[3:0] = wbe;
  endcase
end
wire [127:0] dl_q;                   // the line read at the index of this clock
genvar g;
// One block RAM per byte lane: a write port with its own enable, a synchronous read port.
generate for (g = 0; g < 16; g = g + 1) begin : ln     // lane g = byte 15 - g of the line (big-endian: byte 0 in [127:120])
  reg [7:0] m [0:511];
  reg [7:0] q;
  always @(posedge clk) if (dw_be[g]) m[dw_idx] <= dw_d[g*8 +: 8];
  always @(posedge clk) q <= m[ra_l[(g / 4)*9 +: 9]];   // lanes 0-3 use index copy 0, and so on
  assign dl_q[g*8 +: 8] = q;
end endgenerate

// ---- OC1: hit, data, store hit, kill ---------------------------------------------------------------------------------
wire        en   = cacr[0];          // CACR E: the cache is enabled
reg         sw_act;                  // clear sweep (C, reset) in progress: hits off
reg  [8:0]  sw_cnt;                  // the index the sweep clears next
// unc_o: oc_addr is uncached. The 24-bit area (A31-A24 all 0 or all 1, the A1200's 16 MB with its mirror) uses the
// 1 MB map; above it, the RTG window is uncached, and with cac_hi[6] everything outside the RAM board window is
// uncached as well. CAS reads are uncached by force.
wire        unc_o = (unc_map[oc_addr[23:20]] && (&oc_addr[31:24] || ~|oc_addr[31:24])) || (unc_hi[7] && oc_addr[31:25] == unc_hi[6:0]) || (cac_hi[6] && !(&oc_addr[31:24] || ~|oc_addr[31:24]) && !((oc_addr[31:29] == cac_hi[5:3]) && (oc_addr[28:26] < cac_hi[2:0]))) || oc_fu;
// the word of the line at oc_addr, from the cache read and from the line buffer
wire [31:0] cw = (oc_addr[3:2] == 2'd0) ? dl_q[127:96] : (oc_addr[3:2] == 2'd1) ? dl_q[95:64] :
                 (oc_addr[3:2] == 2'd2) ? dl_q[63:32] : dl_q[31:0];
wire [31:0] lw = (oc_addr[3:2] == 2'd0) ? lb[127:96] : (oc_addr[3:2] == 2'd1) ? lb[95:64] :
                 (oc_addr[3:2] == 2'd2) ? lb[63:32] : lb[31:0];
// line buffer hit: the right line, and for an uncached entry the right word; a forced-uncached load never uses a cached line
wire        lb_hit = lb_v && (lb_tag == oc_addr[31:4]) && (lb_unc ? (lb_w == oc_addr[3:2]) : !oc_fu);
// The second word of a split load, read by a single-word read into ub and used once.
reg  [31:0] ub; reg [31:2] ub_a; reg ub_v, ub_unc;   // the word, its address, valid, it was read uncached
reg         ex_ub, x_ulb0, q_ud, q_ub;   // the load in EX / OC2 used ub or the uncached word; the split used it in its first clock
wire        ub_hit = ub_v && oc_x2 && (ub_a == oc_addr[31:2]);
wire        tl_col = twd && (twd_idx == oc_addr[12:4]);   // the load tag read last clock collided with a tag write
wire        c_hit  = en && !sw_act && !unc_o && tl_q[19] && (tl_q[18:0] == oc_addr[31:13]);   // cache hit
assign      dhit   = ub_hit || lb_hit || c_hit;
assign      dq     = ub_hit ? ub : lb_hit ? lw : cw;
assign      dq_c = cw; assign dq_l = lw; assign lbh = lb_hit; assign dq_u = ub; assign ubh = ub_hit;   // the sources for OC
wire [19:0] tl_e   = tl_col ? twd_val : tl_q;            // the load tag including last clock's write
// ct_hit: the tag at oc_addr, for the second word of a split store. An install in this clock decides it from lb_tag.
assign      ct_hit = (inst && oc_addr[12:4] == lb_tag[12:4]) ? (oc_addr[31:13] == lb_tag[31:13]) :
                     (tl_e[19] && (tl_e[18:0] == oc_addr[31:13]));
assign      sn_a0  = (sn1_v && sn1_a[31:4] == oc_a0[31:4]) || (sn2_v && sn2_a[31:4] == oc_a0[31:4]);   // snoop on the first word's line
wire        ts_col = twd && (twd_idx == oc_saddr[12:4]);   // the store tag read collided with last clock's tag write
wire [19:0] ts_e   = ts_col ? twd_val : ts_q;
wire        st_hq  = ts_e[19] && (ts_e[18:0] == oc_saddr[31:13]);
// The install of this clock (inst: registers only) at the store's index decides the store's hit, the same way ts_col
// does it one clock later.
assign      st_hit = (inst && oc_saddr[12:4] == lb_tag[12:4]) ? (oc_saddr[31:13] == lb_tag[31:13]) : st_hq;
// dkill: the load must replay. A snoop on its line in sn1 or sn2; a tag written while it read (unless it hit the line
// buffer or ub, which the tag write does not touch); an uncached word or ub word already used by an older load still
// in OC2 or EX (one load per uncached read, so an I/O register is never read twice from one bus read).
assign      dkill  = (sn1_v && sn1_a[31:4] == oc_addr[31:4]) || (sn2_v && sn2_a[31:4] == oc_addr[31:4]) ||
                     (tl_col && !lb_hit && !ub_hit) || (lb_unc && lb_hit && ((ex_v && ex_ud) || q_ud)) ||
                     (ub_hit && ((ex_v && ex_ub) || q_ub));
// sn_q: a snoop on either line of the load in OC2 (that load commits after the snoop, so it must not use old data)
assign      sn_q   = q_ld && ((sn1_v && (sn1_a[31:4] == q_a0[31:4] || (q_x && sn1_a[31:4] == q_a1[31:4]))) ||
                              (sn2_v && (sn2_a[31:4] == q_a0[31:4] || (q_x && sn2_a[31:4] == q_a1[31:4]))));

// ---- Uncached word use: the load that hit lb_unc in OC consumes it when it commits ---------------------------------
// The use is marked only when the uop leaves OC1 (not while a split access stays). A split load used the uncached word
// in its first clock (x_ulb0) or uses it now; ub likewise. The marks travel with the load to EX.
always @(posedge clk) begin
  if (oc_x0) x_ulb0 <= lb_unc && lb_hit;
  q_ud  <= oc_v && !oc_stay && oc_ld && ((lb_unc && lb_hit && (oc_x0 || oc_x2)) || (!oc_x0 && x_ulb0));
  q_ub  <= oc_v && !oc_stay && oc_ld && ub_hit;
  ex_ud <= q_ud; ex_ub <= q_ub;
end
wire        ud_use = ex_v && ex_commit && ex_ud;   // the uncached word is used up now
wire        ub_use = ex_v && ex_commit && ex_ub;   // ub is used up now

// ---- Fill state machine ----------------------------------------------------------------------------------------------
// fs = 0: idle. A replayed load in EX that missed (d_miss) starts a fill unless the line buffer or ub already holds what
// it needs (then the replay alone will hit). fs = 1: the request is waiting for the memory unit (dr_go). fs = 2: the
// beats arrive (two for a line, one for an uncached word). When the fill ends, the line buffer becomes valid unless a
// store or snoop met the line meanwhile (fstale), and an install is requested for a cacheable line. A split load that
// needs a second word from another line then starts a second, single-word read into ub (f2).
reg  [1:0]  fs;                      // 0 idle, 1 request, 2 beats
reg  [31:2] fa;                      // the line (the word for uncached)
reg         fstale, fbeat;           // a store or snoop met the fill; the first beat has arrived
reg         inst_p;                  // install pending (line buffer complete, allocation on)
assign      dr_addr = fa;
// Byte enables for any offset: mk has the bytes of the first word in bits 7-4 and of the next word in bits 3-0. A
// split load fills the word that missed (d_lh: the second), and reads the second word into ub too when needed (two).
wire [7:0]  mk    = ((d_lsz == 2'd0) ? 8'h80 : (d_lsz == 2'd1) ? 8'hC0 : 8'hF0) >> d_addr[1:0];
wire [31:2] wb2   = d_addr1;                                   // the second word (registered in EX)
// uA / uB: the first / second word is uncached (the same map as unc_o)
wire        uA    = (unc_map[d_addr[23:20]] && (&d_addr[31:24] || ~|d_addr[31:24])) || (unc_hi[7] && d_addr[31:25] == unc_hi[6:0]) || (cac_hi[6] && !(&d_addr[31:24] || ~|d_addr[31:24]) && !((d_addr[31:29] == cac_hi[5:3]) && (d_addr[28:26] < cac_hi[2:0]))) || d_fu;
wire        uB    = (unc_map[wb2[23:20]] && (&wb2[31:24] || ~|wb2[31:24])) || (unc_hi[7] && wb2[31:25] == unc_hi[6:0]) || (cac_hi[6] && !(&wb2[31:24] || ~|wb2[31:24]) && !((wb2[31:29] == cac_hi[5:3]) && (wb2[28:26] < cac_hi[2:0]))) || d_fu;
wire [31:2] fw    = d_lh ? wb2 : d_addr[31:2];                // the word to fill
wire        f_unc = d_lh ? uB : uA;                           // ... uncached
wire [3:0]  be_l  = d_lh ? mk[3:0] : mk[7:4];                 // ... its bytes
// two: the load also needs its second word, which is uncached or in another line, so one fill cannot bring both
wire        two   = (mk[3:0] != 4'h0) && !d_lh && (uA || wb2[31:4] != d_addr[31:4]);
// lb_same: the line buffer (and ub, if needed) already holds what this load needs: no new fill
wire        lb_same = lb_v && (lb_tag == fw[31:4]) && (f_unc ? (lb_unc && lb_w == fw[3:2]) : (!lb_unc || lb_w == fw[3:2])) &&
                      (!two || (ub_v && ub_a == wb2));
reg         f2, f2p, f2u;            // the second read (into ub) is running / pending; it is uncached
// ub_same: a split load whose second word is already in ub. Without this check, a replayed split load that reached OC
// before the read into ub was done asked for the second word's line again, which dropped ub and the first line in turn:
// a livelock of MOVEM and CAS on a split access. Now the load just replays and finds ub.
wire        ub_same = d_lh && ub_v && (ub_a == wb2);
reg  [31:2] f2a;                     // the second read's word
reg  [3:0]  f2be;                    // ... and its bytes
wire        st_f  = wb_st && (wb_saddr[31:4] == fa[31:4]);       // a store meets the fill
wire        sn_f  = sn1_v && (sn1_a[31:4] == fa[31:4]);           // a snoop meets the fill
wire        st_lb = wb_st && lb_v && (wb_saddr[31:4] == lb_tag);  // a store to the line in the line buffer
wire        sn_lb = sn1_v && lb_v && (sn1_a[31:4] == lb_tag);     // a snoop on it
assign      unc_hold = (fs != 2'd0 && dr_unc) || (lb_v && lb_unc) || (ub_v && ub_unc);

always @(posedge clk) begin
  if (rst) begin
    fs <= 2'd0; dr_req <= 1'b0; lb_v <= 1'b0; inst_p <= 1'b0; ub_v <= 1'b0; f2 <= 1'b0; f2p <= 1'b0;
  end else begin
    if (inst) inst_p <= 1'b0;                                       // the install happened
    // ub is cleared by a store or snoop on its word and when its load commits
    if ((wb_st && wb_saddr[31:2] == ub_a) || (sn1_v && sn1_a[31:4] == ub_a[31:4]) || ub_use) ub_v <= 1'b0;
    case (fs)
      2'd0: if (d_miss && !lb_same && !ub_same) begin               // start a fill
              fs <= 2'd1; dr_req <= 1'b1; fa <= f_unc ? fw : {fw[31:4], 2'b00}; dr_unc <= f_unc; dr_be <= be_l;
              fstale <= 1'b0; fbeat <= 1'b0; lb_v <= 1'b0; inst_p <= 1'b0;
              ub_v <= 1'b0; f2 <= 1'b0; f2p <= two; f2a <= wb2; f2be <= mk[3:0]; f2u <= uB;
            end
      2'd1: if (dr_go) begin fs <= 2'd2; dr_req <= 1'b0; end         // the memory unit took it
      default: begin
        if (st_f || sn_f) fstale <= 1'b1;                           // the data arriving is now old
        if (r_d) begin
          if (f2) ub <= fa[2] ? r_data[31:0] : r_data[63:32];       // second read: one word into ub
          else if (dr_unc) lb <= fa[2] ? {r_data[31:0], r_data[31:0], r_data[31:0], r_data[31:0]}   // uncached: the word in all four places
                                       : {r_data[63:32], r_data[63:32], r_data[63:32], r_data[63:32]};
          else if (!fbeat) lb[127:64] <= r_data; else lb[63:0] <= r_data;   // line: first beat bytes 0-7, second 8-15
          fbeat <= 1'b1;
        end
        if (r_done && f2) begin                                     // the second read is done
          fs <= 2'd0; f2 <= 1'b0; ub_a <= fa; ub_unc <= f2u; ub_v <= !(fstale || st_f || sn_f);
        end else if (r_done) begin                                  // the fill is done
          lb_tag <= fa[31:4]; lb_w <= fa[3:2]; lb_unc <= dr_unc;
          lb_v <= !(fstale || st_f || sn_f);
          inst_p <= !(fstale || st_f || sn_f) && !dr_unc && en && !cacr[1];   // install a cacheable line unless frozen
          if (f2p) begin                       // second word: one single-word read into ub
            fs <= 2'd1; dr_req <= 1'b1; fa <= f2a; dr_unc <= 1'b1; dr_be <= f2be; f2 <= 1'b1; f2p <= 1'b0;
            fstale <= 1'b0; fbeat <= 1'b0;
          end else fs <= 2'd0;
        end
      end
    endcase
    // Stores update the line buffer byte by byte (so it stays equal to memory); snoops and a used uncached word clear it.
    if (st_lb && !lb_unc) begin
      if (wb_saddr[3:2] == 2'd0) begin if (sbe[3]) lb[127:120] <= sd[31:24]; if (sbe[2]) lb[119:112] <= sd[23:16]; if (sbe[1]) lb[111:104] <= sd[15:8]; if (sbe[0]) lb[103:96] <= sd[7:0]; end
      if (wb_saddr[3:2] == 2'd1) begin if (sbe[3]) lb[95:88]   <= sd[31:24]; if (sbe[2]) lb[87:80]   <= sd[23:16]; if (sbe[1]) lb[79:72]   <= sd[15:8]; if (sbe[0]) lb[71:64]  <= sd[7:0]; end
      if (wb_saddr[3:2] == 2'd2) begin if (sbe[3]) lb[63:56]   <= sd[31:24]; if (sbe[2]) lb[55:48]   <= sd[23:16]; if (sbe[1]) lb[47:40]   <= sd[15:8]; if (sbe[0]) lb[39:32]  <= sd[7:0]; end
      if (wb_saddr[3:2] == 2'd3) begin if (sbe[3]) lb[31:24]   <= sd[31:24]; if (sbe[2]) lb[23:16]   <= sd[23:16]; if (sbe[1]) lb[15:8]    <= sd[15:8];  if (sbe[0]) lb[7:0]    <= sd[7:0];  end
    end
    // a store to an uncached word, a snoop, or the commit of the load that used the uncached word empties the line buffer
    if ((fs == 2'd0 || f2) && ((st_lb && lb_unc) || sn_lb || ud_use)) begin lb_v <= 1'b0; inst_p <= 1'b0; end
  end
end

// ---- External snoop ----------------------------------------------------------------------------------------------
// sn1 (first clock): address registered; line buffer and fill compared; the snoop tag copy is read at sn1_a (tn_q in
// the second clock). sn2 (second clock): tag compared (a tag write at that index in the first clock is taken from twd)
// and the line invalidated.
always @(posedge clk) begin
  if (rst) begin sn1_v <= 1'b0; sn2_v <= 1'b0; end
  else begin sn1_v <= sn_v; sn2_v <= sn1_v; end
  sn1_a <= sn_addr; sn2_a <= sn1_a;
end
wire        tn_col = twd && (twd_idx == sn2_a[12:4]);   // the snoop tag read collided with a tag write
// Both tag compares are formed, and the collision selects last.
wire        sn_hw  = twd_val[19] && (twd_val[18:0] == sn2_a[31:13]);
wire        sn_hq  = tn_q[19] && (tn_q[18:0] == sn2_a[31:13]);
wire        sn_hit = sn2_v && (tn_col ? sn_hw : sn_hq);  // the snooped line is cached: invalidate it

// ---- Tag port: install, snoop invalidation, clear entry (CE), clear sweep ------------------------------------------
reg         ce_p; reg [8:0] ce_idx;  // a CE clear is pending, at this index
// An install would race the snoop's tag read. The check is made a clock early (snb) against both tags lb_tag can hold
// next clock (lb_tag itself, or fa, which it loads at r_done), so the install enable has no compare in front of it.
reg         snb;
always @(posedge clk) snb <= sn_v && ((sn_addr[12:4] == lb_tag[12:4]) || (sn_addr[12:4] == fa[12:4]));
wire        sn_block = snb;
// The store checks are a register, formed one clock early as a superset: a store in OC2 or EX now, or one that stays in
// OC1, was in OC1, OC2 or EX last clock. Its index is compared with both tags lb_tag can hold now (lb_tag or fa, as
// for snb). A store that enters OC1 in the install clock takes its hits (st_hit, ct_hit) from the installed tag
// instead. The install enable then has only registers in front of the data RAMs' address and byte enables (the
// oc_saddr compare and the OR of the checks had been the worst D-cache paths). A snoop in sn2 holds the install a
// clock, so the tag port's priority select no longer waits for the snoop tag compare.
wire [8:0]  lbt1 = lb_tag[12:4] - 9'd1, fat1 = fa[12:4] - 9'd1;   // the line before (a split store's first word is there)
function stm; input v; input [12:2] a;   // a store at a (valid v) touches the line lb_tag or fa
  stm = v && (a[12:4] == lb_tag[12:4] || a[12:4] == fa[12:4] ||
              (a[3:2] == 2'd3 && (a[12:4] == lbt1 || a[12:4] == fat1)));   // a split store's second word reaches into the next line
endfunction
reg         stb;                     // some store in OC1, OC2 or EX might touch the line to install
always @(posedge clk) stb <= stm(oc_v && oc_st, oc_saddr[12:2]) || stm(q_st, q_sa[12:2]) || stm(ex_v && ex_st, ex_saddr[12:2]);
wire        can_inst = inst_p && en && !cacr[1] && !sw_act && !wb_st && !sn_block && !stb && !sn2_v && !ce_p;
// The tag port's priority select. can_inst already excludes the others (sn2_v, ce_p, sw_act), so it comes first and
// inst = can_inst is formed from registers only.
always @* begin
  tw_en = 1'b0; tw_idx = sn2_a[12:4]; tw_val = 20'h0; inst = 1'b0;
  if (can_inst) begin tw_en = 1'b1; tw_idx = lb_tag[12:4]; tw_val = {1'b1, lb_tag[31:13]}; inst = 1'b1; end   // install: valid tag
  else if (sn_hit) begin tw_en = 1'b1; tw_idx = sn2_a[12:4]; end   // snoop: write an invalid tag
  else if (ce_p) begin tw_en = 1'b1; tw_idx = ce_idx; end          // CE: invalidate CAAR's line
  else if (sw_act) begin tw_en = 1'b1; tw_idx = sw_cnt; end        // C or reset: invalidate line after line
end
// The clear sweep and the CE request. A snoop takes the port first, so the sweep and CE wait for it.
always @(posedge clk) begin
  if (rst) begin sw_act <= 1'b1; sw_cnt <= 9'd0; ce_p <= 1'b0; end
  else begin
    if (clr_all) begin sw_act <= 1'b1; sw_cnt <= 9'd0; end
    else if (sw_act && !sn_hit && !ce_p) begin sw_cnt <= sw_cnt + 9'd1; if (sw_cnt == 9'd511) sw_act <= 1'b0; end
    if (clr_ent) begin ce_p <= 1'b1; ce_idx <= caar[12:4]; end
    else if (ce_p && !sn_hit) ce_p <= 1'b0;
  end
end
endmodule
`default_nettype wire
