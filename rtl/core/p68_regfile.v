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
// ==== p68_regfile.v: the register file ===============================================================================
//
// What it holds. The 68k's sixteen 32-bit registers D0-D7 (numbers 0-7) and A0-A7 (8-15), four temporaries T0-T3
// (16-19) that multi-uop sequences use to pass values between their uops, and the three stack pointer banks USP, ISP and
// MSP (20-22) that hold whichever stack pointers are not active at the moment. Everything is plain flip-flops in packed
// vectors (rd_ for D, at for A and T, bk for the banks). Block RAM or LUT RAM would be smaller, but they have one or two
// read ports and a clock of read latency, and this file needs five read ports with values that are valid in the same
// clock, so registers are the only choice that fits.
//
// Who writes it. The pipeline stages are named here as in p68_core.v: AG computes addresses, OC1 and OC2 read the
// D-cache and the operands, EX executes and commits, WB writes back.
//   - WB lane A (wb_*): any register, the result of the uop that committed in EX in the last clock.
//   - WB lane B (wb2_*): D0-D7 or A0-A7, the result of the second instruction of a dual-issue pair. It never writes the
//     same register as lane A in the same clock (DR does not pair such instructions).
//   - AG write 1 (p1_*): the base register update of a load EA, (An)+ or -(An). A registers only.
//   - AG write 2 (p2_*): the base update of a store EA, or the address of a LEA. A or T registers. It wins over write 1.
//   AG computes these updates early, so the next instruction's address can use the new An at once (a common case in
//   68k code, e.g. MOVE.L (A0)+,(A1)+ in a copy loop). The AG writes enter the file one clock after AG, from OC1's
//   registers (OC1 holds the uop that left AG last clock); AG forwards its own last writes itself. Entering from OC's
//   registers keeps the AG adder off the register file's write path, which was worth doing for timing.
//
// Speculation and the committed copy. An AG write happens before the uop is known to commit. If a later branch turns
// out to be mispredicted, or the uop replays, the A and T registers must go back to what they were. So the file keeps
// a second copy of A0-A7 and T0-T3, ash, that only takes committed values: an AG write enters ash when its uop leaves EX
// and commits (the c1/c2 inputs from EX), and a WB write enters it always (WB only ever writes committed results). A
// redirect from EX sets restore, and in that clock every A and T register is loaded from ash (plus any WB write of the
// same clock). D registers need no copy, because only WB writes them.
// The restore happens one clock after EX's redirect. In that clock AG, OC and EX are empty anyway, because the front
// end follows the redirect one clock later, so the extra clock costs nothing; and it means the restore input comes from
// a register (rs_d in p68_core.v), so the redirect logic is not in front of 384 register enables. Moving the restore one
// clock later was also what allowed RTS to be a single uop (before, a uop that redirects at commit was not allowed to
// update An in AG, and RTS has to update A7).
//
// The stack pointer swap. A7 is always the active stack pointer, so no uop has to know which one is active. When a
// committed instruction changes the S or M bit of the status register, the system unit asserts sw_en one clock later
// (no AG write is in flight then, and the core asserts restore in the same clock). The swap saves the committed A7 into
// bank sw_old and loads A7, and its committed copy, from bank sw_new.
//
// Who reads it, and why the reads look the way they do.
//   - AG reads base and index registers (ag_rl, ag_rs, ag_rx) one clock early, at the register numbers DS gives, into
//     AG's own operand registers. The value read must be the register's value after this clock: the value as written
//     by WB, by the AG writes of the uop now in OC, or by a restore. A plain "read at_n" would put the write data and
//     the restore selects in front of the 12:1 read select. In the full A1200 build the paths from wb_val and from the
//     restore copies through at_n into AG's read registers were among the failing ones, so the read was turned around:
//     the held values (at, ash, rd_) pass the read select first, and the write data (WB lanes, AG writes, swap value)
//     is ORed in at the end under one-hot selects (e_*), which are read with the same register number.
//   - OC2 reads operands A and B (oc_ra, oc_rb) for EX, bypassed from WB. The read select uses one-hot register
//     numbers registered in p68_core.v from OC1's uop (oc_oa, oc_ob, a 23-input AND-OR), and the WB compares are made a
//     clock early (oc_eq). Before, the register number passed a 23:1 select behind OC1's uop, and those paths from OC2
//     into EX were among the failing ones of the full system; together with the AG change above this took the CPU from
//     68.20 to 68.70 MHz at the 70.6 MHz target.
//   - OC2 reads lane B's operands (oc_rc, oc_rd), D and A registers only, with a plain 16:1 select and the WB bypass.
// Reading the operands in OC1 into registers (so OC2 only had to bypass WB) was built twice and not kept: the first time
// those paths were not among the slow ones, the second time the WB value nets grew to 9-11 ns in the full system's
// place-and-route.
//
// Clock: everything is on the CPU clock. The maximum-fanout attribute after the port list asks Gowin's synthesizer to
// keep the fanout of each net in this module at 16 or below by duplicating drivers.
`default_nettype none
module p68_regfile (
  input  wire        clk,
  input  wire        wb_wen,         // WB lane A writes
  input  wire [4:0]  wb_wreg,        // ... register number (0-22)
  input  wire [31:0] wb_val,         // ... value
  input  wire        wb2_wen,        // lane B's WB write (D0-D7, A0-A7; never the same register as lane A's)
  input  wire [3:0]  wb2_wreg,       // ... register number (0-15)
  input  wire [31:0] wb2_val,        // ... value
  // the write hits as one-hot registers (made where the register number became known), so no compare is needed here
  input  wire [7:0]  p1_h,           // AG write 1 hits A0-A7
  input  wire [11:0] p2_h,           // AG write 2 hits A0-A7, T0-T3
  input  wire [11:0] wb_h,           // WB lane A hits A0-A7, T0-T3
  input  wire [7:0]  wb_hd,          // ... D0-D7
  input  wire [7:0]  wb2_h, wb2_hd,  // WB lane B hits A0-A7 / D0-D7
  input  wire [4:0]  oc_rc, oc_rd,   // lane B's operand reads (OC2), bypassed from WB
  output wire [31:0] oc_vc, oc_vd,   // ... their values
  input  wire        p1_en,          // AG write 1: load base update
  input  wire [4:0]  p1_r,           // ... register (only for reference; the one-hot p1_h is what selects)
  input  wire [31:0] p1_v,           // ... value
  input  wire        p2_en,          // AG write 2: store base update or LEA (priority over write 1)
  input  wire [4:0]  p2_r,
  input  wire [31:0] p2_v,
  input  wire        c1_en,          // commit of AG write 1 (from EX: the uop with that AG write commits now)
  input  wire [4:0]  c1_r,
  input  wire [31:0] c1_v,
  input  wire        c2_en,          // commit of AG write 2
  input  wire [4:0]  c2_r,
  input  wire [31:0] c2_v,
  input  wire        c_cm,           // EX commits (c1_en = c_cm && write 1 enable, c2_en likewise)
  input  wire [7:0]  c1_h,           // the commit writes' register one-hots (registered in EX)
  input  wire [11:0] c2_h,
  input  wire [11:0] restore,        // load A0-A7 and T0-T3 from the committed copy (one register copy per entry, for fanout)
  input  wire [4:0]  ag_rl, ag_rs,   // AG base reads (A or T) for the load and the store address
  input  wire [3:0]  ag_rx,          // AG index read (D or A)
  output wire [31:0] ag_vl, ag_vs, ag_vx,   // ... their values after this clock
  input  wire [4:0]  oc_ra, oc_rb,   // OC operand reads, bypassed from WB
  output wire [31:0] oc_va, oc_vb,   // ... their values
  input  wire [22:0] oc_oa, oc_ob,   // oc_ra, oc_rb as one-hots (registered in p68_core.v)
  input  wire [3:0]  oc_eq,          // {B = WB lane B, B = WB, A = WB lane B, A = WB} write register (compared a clock early)
  // stack pointer swap
  input  wire        sw_en,          // swap A7 with a bank now
  input  wire [1:0]  sw_old, sw_new, // bank that receives the old A7 / bank the new A7 comes from (0 USP, 1 ISP, 2 MSP)
  output wire [31:0] sw_v            // the new A7 of a swap (retire port)
) /* synthesis syn_maxfan = 16 */;
reg  [255:0] rd_;                    // D0-D7, register n at bits n*32
reg  [383:0] at;                     // A0-A7, T0-T3 (entry i = register 8 + i)
reg  [383:0] ash;                    // committed copy of A0-A7, T0-T3
reg  [383:0] at_n;                   // next value of at (combinational)
reg  [95:0]  bk;                     // banks: USP, ISP, MSP (registers 20-22)
wire [735:0] rf = {bk, at, rd_};     // all 23 registers in one vector: register n at bits n*32
// the committed A7 including a WB write of this clock: the value a swap saves into the old bank
wire [31:0]  a7c = (wb_wen && wb_wreg == 5'd15) ? wb_val : (wb2_wen && wb2_wreg == 4'd15) ? wb2_val : ash[7*32 +: 32];   // committed A7 incl. WB
assign sw_v = bk[sw_new*32 +: 32];   // the bank the new A7 comes from

integer i;
// The next value of each A/T register. The priority restore > AG write 2 > AG write 1 > WB lane A > WB lane B > hold is
// folded into one-hot selects (exactly one of restore, s2, s1, sw, sb, sh is true per entry), and the value is an AND-OR
// of the sources under those selects. An AND-OR is two LUT levels after the compares, where a priority chain of ?:
// would be one LUT level per source.
always @* begin
  for (i = 0; i < 12; i = i + 1) begin : an       // entry i is register 8 + i (A0-A7 = 8..15, T0-T3 = 16..19)
    reg h2, h1, hw, hb, s2, s1, sw, sb, sh;
    reg [31:0] rsv;
    h2 = p2_en && p2_h[i];                        // AG write 2 hits this entry
    h1 = (i < 8) && p1_en && p1_h[i < 8 ? i : 0];   // AG write 1 hits it (A registers only)
    hw = wb_wen && wb_h[i];                       // WB lane A hits it
    hb = (i < 8) && wb2_wen && wb2_h[i < 8 ? i : 0];   // WB lane B hits it (A registers only)
    // restore (also forced in the swap clock): the committed value, a WB write of this clock, or for A7 in a swap the
    // new bank value; the swap loads A7 inside the restore branch, so it stays off the AG write path
    rsv = (sw_en && i == 7) ? sw_v : hw ? wb_val : hb ? wb2_val : ash[i*32 +: 32];
    // one-hot selects in priority order
    s2 = !restore[i] && h2; s1 = !restore[i] && !h2 && h1; sw = !restore[i] && !h2 && !h1 && hw; sb = !restore[i] && !h2 && !h1 && !hw && hb;
    sh = !restore[i] && !h2 && !h1 && !hw && !hb;   // nothing writes: hold
    at_n[i*32 +: 32] = ({32{restore[i]}} & rsv) | ({32{s2}} & p2_v) | ({32{s1}} & p1_v) | ({32{sw}} & wb_val) | ({32{sb}} & wb2_val) |
                       ({32{sh}} & at[i*32 +: 32]);
  end
end
// All register writes of the clock
always @(posedge clk) begin
  at <= at_n;                                     // A and T registers: next value from above
  for (i = 0; i < 8; i = i + 1)                   // D registers: only WB writes them (lane A first; the two never collide)
    if (wb_wen && wb_wreg == i) rd_[i*32 +: 32] <= wb_val;
    else if (wb2_wen && wb2_wreg == i) rd_[i*32 +: 32] <= wb2_val;
  // committed copy of A0-A7: the AG writes of the committing uop, else the WB writes. The register compares come as
  // one-hots (c1_h, c2_h from EX; wb_h, wb2_h from WB).
  for (i = 0; i < 8; i = i + 1)
    if (c_cm && c2_h[i])      ash[i*32 +: 32] <= c2_v;
    else if (c_cm && c1_h[i]) ash[i*32 +: 32] <= c1_v;
    else if (wb_wen && wb_h[i]) ash[i*32 +: 32] <= wb_val;
    else if (wb2_wen && wb2_h[i]) ash[i*32 +: 32] <= wb2_val;
  for (i = 8; i < 12; i = i + 1)                  // committed copy of T0-T3: AG write 2 and WB lane A only reach them
    if (c_cm && c2_h[i])      ash[i*32 +: 32] <= c2_v;
    else if (wb_wen && wb_h[i]) ash[i*32 +: 32] <= wb_val;
  // stack pointer banks: the swap saves the committed A7 into the old bank (it wins over a WB write to the same bank);
  // otherwise WB may write a bank directly (MOVE USP, MOVEC, and the exception sequences use registers 20-22)
  for (i = 0; i < 3; i = i + 1)
    if (sw_en && sw_old == i)              bk[i*32 +: 32] <= a7c;
    else if (wb_wen && wb_wreg == 5'd20 + i) bk[i*32 +: 32] <= wb_val;
  if (sw_en) ash[7*32 +: 32] <= sw_v;             // the swap also loads the committed A7 (wins over the A7 writes above)
end

// ---- AG's early reads ----------------------------------------------------------------------------------------------
// AG reads one clock early (at DS's register numbers, into AG's operand registers), and each port gives the register's
// value after this clock, as at_n would hold it: the WB writes, the AG writes of the uop in OC (from OC's registers; AG
// forwards the writes of the uop that left AG last clock itself), and the restore values.
// The reads are AND-ORs of per-port scalar selects: the sum over registers of at_n is swapped with the sum over sources.
// The held values (at, ash, rd_) pass the one-hot read select, and the write data (WB lanes, AG writes, swap value)
// enters only the last levels. Before this change wb_val and the restore selects passed at_n's three levels and then
// the 12:1 read, and those were real failing paths of the full system.
// e_*: per A/T entry, which source its value after this clock comes from (exactly one is set):
reg  [11:0] e_s, e_w, e_b, e_a, e_2, e_1, e_h;   // per A/T entry after this clock: swap value, WB A, WB B, ash, AG 2, AG 1, at
reg  [7:0]  d_w, d_b, d_h;                       // per D register: WB A, WB B, held
always @* begin
  for (i = 0; i < 12; i = i + 1) begin : es
    reg h2, h1, hw, hb, rs, sv;
    h2 = p2_en && p2_h[i];                       // same hit terms as in at_n above
    h1 = (i < 8) && p1_en && p1_h[i < 8 ? i : 0];
    hw = wb_wen && wb_h[i];
    hb = (i < 8) && wb2_wen && wb2_h[i < 8 ? i : 0];
    rs = restore[i];
    sv = sw_en && (i == 7);                      // A7 in a swap clock
    e_s[i] = rs && sv;                           // swap value
    e_w[i] = rs ? (!sv && hw) : (!h2 && !h1 && hw);           // WB lane A (inside or outside a restore)
    e_b[i] = rs ? (!sv && !hw && hb) : (!h2 && !h1 && !hw && hb);   // WB lane B
    e_a[i] = rs && !sv && !hw && !hb;            // committed copy (restore without a WB write)
    e_2[i] = !rs && h2;                          // AG write 2
    e_1[i] = !rs && !h2 && h1;                   // AG write 1
    e_h[i] = !rs && !h2 && !h1 && !hw && !hb;    // hold
  end
  for (i = 0; i < 8; i = i + 1) begin            // D registers: WB lane A, lane B, or hold
    d_w[i] = wb_wen && wb_hd[i];
    d_b[i] = !(wb_wen && wb_hd[i]) && wb2_wen && wb2_hd[i];
    d_h[i] = !(wb_wen && wb_hd[i]) && !(wb2_wen && wb2_hd[i]);
  end
end
// per entry the held value after this clock (registers and selects only, no write data), read by the register number
// with the usual binary select; the selects of the write data are bits of e_* read the same way
reg  [383:0] at_h;
reg  [255:0] rd_h;
always @* begin
  for (i = 0; i < 12; i = i + 1) at_h[i*32 +: 32] = ({32{e_h[i]}} & at[i*32 +: 32]) | ({32{e_a[i]}} & ash[i*32 +: 32]);   // held or restored
  for (i = 0; i < 8; i = i + 1)  rd_h[i*32 +: 32] = {32{d_h[i]}} & rd_[i*32 +: 32];   // held D register (zero when WB writes it)
end
function [3:0] ben; input [4:0] r; ben = r[4] ? {2'b10, r[1:0]} : {1'b0, r[2:0]}; endfunction   // base register: entry
                                                 // (A0-A7 = 8-15 give entries 0-7, T0-T3 = 16-19 give entries 8-11)
wire [3:0]  bl = ben(ag_rl), bs = ben(ag_rs);    // entry of the load base and the store base
wire [2:0]  xa = ag_rx[2:0];                     // index register within its bank
wire [31:0] hl = at_h[bl*32 +: 32], hs = at_h[bs*32 +: 32];   // held part of the two base reads (12:1 selects)
wire [511:0] rh = {at_h[255:0], rd_h};           // D0-D7 and A0-A7 held values, for the index read (16:1)
wire [31:0] hx = rh[ag_rx*32 +: 32];             // held part of the index read
wire [4:0]  ql = {e_s[bl], e_2[bl], e_1[bl], e_w[bl], e_b[bl]};   // write-data selects of the load base read
wire [4:0]  qs = {e_s[bs], e_2[bs], e_1[bs], e_w[bs], e_b[bs]};   // ... of the store base read
wire [4:0]  qx = ag_rx[3] ? {e_s[xa], e_2[xa], e_1[xa], e_w[xa], e_b[xa]} : {3'b000, d_w[xa], d_b[xa]};   // ... of the index read
function [31:0] rdo;                 // held part | swap value | AG 2 | AG 1 | WB A | WB B
  input [31:0] h; input [4:0] q;
  rdo = h | ({32{q[4]}} & sw_v) | ({32{q[3]}} & p2_v) | ({32{q[2]}} & p1_v) | ({32{q[1]}} & wb_val) | ({32{q[0]}} & wb2_val);
endfunction
assign ag_vl = rdo(hl, ql);                      // load base value after this clock
assign ag_vs = rdo(hs, qs);                      // store base value after this clock
assign ag_vx = rdo(hx, qx);                      // index value after this clock
// ---- OC2's operand reads -------------------------------------------------------------------------------------------
// AND-OR reads at the one-hot register numbers (23 registers)
reg  [31:0] ro_a, ro_b;
always @* begin
  ro_a = 32'h0; ro_b = 32'h0;
  for (i = 0; i < 23; i = i + 1) begin
    ro_a = ro_a | ({32{oc_oa[i]}} & rf[i*32 +: 32]);
    ro_b = ro_b | ({32{oc_ob[i]}} & rf[i*32 +: 32]);
  end
end
// the WB bypass: the register compare was made a clock early (oc_eq), only the write enables are ANDed in here
wire oa_w = wb_wen && oc_eq[0], oa_b = wb2_wen && oc_eq[1], ob_w = wb_wen && oc_eq[2], ob_b = wb2_wen && oc_eq[3];
assign oc_va = ({32{oa_w}} & wb_val) | ({32{!oa_w && oa_b}} & wb2_val) | ({32{!oa_w && !oa_b}} & ro_a);
assign oc_vb = ({32{ob_w}} & wb_val) | ({32{!ob_w && ob_b}} & wb2_val) | ({32{!ob_w && !ob_b}} & ro_b);
// lane B reads only D0-D7 and A0-A7 (DR pairs no uop with a T register): 16:1 selects with the WB bypass
function [31:0] rdb16;
  input [3:0] r;
  begin rdb16 = (wb_wen && wb_wreg == {1'b0, r}) ? wb_val : (wb2_wen && wb2_wreg == r) ? wb2_val : rf[r*32 +: 32]; end
endfunction
assign oc_vc = rdb16(oc_rc[3:0]);                // lane B operand A
assign oc_vd = rdb16(oc_rd[3:0]);                // lane B operand B

endmodule
`default_nettype wire
