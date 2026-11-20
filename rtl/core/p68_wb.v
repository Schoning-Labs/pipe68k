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
// ==== p68_wb.v: write-back (WB) and the retire port ===================================================================
//
// Abbreviations: EX = the execute stage, WB = write-back (this file), OC = the operand stages, AG = address generation,
// CAS = compare-and-swap, CCR = the condition code register, smc = self-modifying code, SP = stack pointer.
//
// ---- What this unit is for ----------------------------------------------------------------------------------------
//
// WB is the last stage. A uop that committed in EX arrives here with its result, and WB:
//   - writes the result into the register file (wb_wen, wb_wreg, wb_val, with one-hot copies of the register number
//     for the register file's write enables),
//   - aligns a store's data to its byte offset and hands it to the D-cache (wb_st, wb_saddr, sd, sbe) and to the memory
//     port's store buffer (bw_*), and keeps the store of one clock earlier (ls_*) for OC's forwarding,
//   - holds the bus lock of CAS and CAS2,
//   - drives the retire port, which tells the simulation test benches what every instruction did.
// Lane B's register write does not pass through here: EX registers it directly (wb2_* in p68_ex.v).
//
// ---- Stores of any alignment --------------------------------------------------------------------------------------
//
// A store can start at any byte (the 68020 allows misaligned words and longs). The value is put at the top of a
// 64-bit vector and shifted right by the byte offset, which gives the bytes for the first word in the top half and any
// bytes that spill into the next word in the bottom half; the byte enables are shifted the same way. A store that
// crosses a word boundary is written in two clocks, the second from the st2_* registers. AG and OC keep the clock after
// it free of other stores, so the two writes never collide.
//
// ---- The bus lock -------------------------------------------------------------------------------------------------
//
// CAS and CAS2 read memory, compare, and write back, and no other bus master (the Amiga chipset's DMA) may touch memory
// in between. lock goes high when the first CAS step reaches EX and stays high until the last step committed and its
// stores have left the store buffer, and it drops one clock after a redirect that throws the CAS away (from the
// registered rdn, so no redirect logic sits in front of the lock output).
//
// ---- The retire port ------------------------------------------------------------------------------------------------
//
// The retire port exists only for simulation: the lockstep bench runs the core and a reference CPU side by side and
// compares them after every instruction through it. ret_u marks every committed uop; its register write (ret_wen), An
// updates (ret_ws_*, ret_wd_*) and store (ret_st) are valid in that clock. ret_v marks the last uop of an instruction:
// ret_pc and ret_ccr then describe the retired instruction. Temporaries are not reported. A hardware build leaves these
// outputs open, so synthesis removes the logic behind them and they cost nothing there.
//
// Runs on the CPU clock clk.
`include "p68_uop.vh"                // the uop format
`default_nettype none
module p68_wb (
  input  wire        clk,            // CPU clock
  input  wire        rst,            // synchronous reset
  input  wire        ex_commit,      // the uop in EX commits
  input  wire [`UW-1:0] ex_u,        // ... the uop
  input  wire        ex_wen,         // ... it writes a register (with the DBcc and CAS conditions)
  input  wire [4:0]  ex_wreg,        // ... this register (0-22)
  input  wire        ex_st,          // ... it stores (with the CAS condition)
  input  wire [31:0] ex_saddr,       // ... at this address
  input  wire        ex_hit,         // ... and the store's line is in the D-cache
  input  wire [31:0] rm,             // ... the result, merged by size
  input  wire        ex_w1_en, ex_w2_en,   // ... its AG writes (An updates), for the retire port
  input  wire [4:0]  ex_w1_r, ex_w2_r,
  input  wire [31:0] ex_w1_v, ex_w2_v,
  input  wire [4:0]  ccr,            // the CCR after the uop in WB (for the retire port)
  // ---- WB stage ----
  output reg         wb_wen,         // WB writes a register this clock
  output reg  [4:0]  wb_wreg,        // ... this one
  output reg  [11:0] wb_h,           // the WB write hits A0-A7, T0-T3 (one-hot, with wb_wen)
  output reg  [7:0]  wb_hd,          // ... D0-D7
  output reg  [31:0] wb_val,         // ... the value (also forwarded to AG, OC and EX)
  output wire        wb_st,          // WB stores this clock (also high in the second clock of a misaligned store)
  output wire [31:0] wb_saddr,       // ... at this address
  output reg  [31:0] wb_sac,         // copy of wb_saddr as a register (I-cache store check)
  output wire        wb_hit,         // ... the line is in the D-cache
  output wire [31:0] sd,             // ... the data, aligned to the word
  output wire [3:0]  sbe,            // ... the byte enables (bit 3 = address +0)
  output reg         ls_v,           // the store of one clock ago (OC forwards it: its D-cache write is not readable yet)
  output reg  [31:2] ls_addr,
  output reg  [31:0] ls_data,
  output reg  [3:0]  ls_be,
  // ---- write-through bus (to the store buffer of p68_mem.v) ----
  output reg         bw_en,
  output reg  [31:2] bw_addr,
  output reg  [31:0] bw_data,
  output reg  [3:0]  bw_be,
  // ---- retire port (simulation) ----
  output reg         ret_u,          // a uop committed
  output reg         ret_v,          // an instruction retired
  output reg  [31:0] ret_pc,         // its address
  output wire        ret_wen,        // a register write of D0-A7
  output wire [3:0]  ret_wreg,
  output wire [31:0] ret_wval,
  output reg         ret_ws_en,      // An update from AG, load side ((An)+, -(An))
  output reg  [2:0]  ret_ws_r,
  output reg  [31:0] ret_ws_v,
  output reg         ret_wd_en,      // An update from AG, store side or LEA
  output reg  [2:0]  ret_wd_r,
  output reg  [31:0] ret_wd_v,
  output wire        ret_st,         // a store
  output wire [31:0] ret_saddr,
  output wire [31:0] ret_sdata,
  output wire [3:0]  ret_sbe,
  output wire [4:0]  ret_ccr,        // the CCR after the instruction
  // ---- exceptions ----
  input  wire        no_ret,         // the committing last uop does not retire (exception raised, RTE restart)
  output reg         ret_irq,        // the retired instruction is an interrupt entry
  output wire        ret_bk_en,      // WB writes a stack pointer bank (R_USP, R_ISP, R_MSP)
  output wire [1:0]  ret_bk_r,       // 0 USP, 1 ISP, 2 MSP
  // ---- misaligned stores and the bus lock ----
  input  wire        ex_v,           // a uop is in EX
  input  wire        ex_hit2,        // second word of a misaligned store is in the D-cache
  input  wire        redirect_ex,    // EX redirects
  input  wire        ex_rep,         // ... because of a replay
  input  wire [2:0]  sb_cnt,         // store buffer entries (the lock holds until the CAS stores are written)
  output reg         lock,           // CAS/CAS2 running: no other bus master may access memory (A1200 adapter)
  output wire        ret_st2,        // the retired store's second word (misaligned store), same clock as ret_st
  output wire [31:0] ret_saddr2,
  output wire [31:0] ret_sdata2,
  output wire [3:0]  ret_sbe2
) /* synthesis syn_maxfan = 16 */;
reg [1:0] wb_sz;                     // the store's size
// ---- Store data and byte enables for any byte offset; a store that crosses a longword is written in two clocks (the
// second from st2_*; OC and AG keep the clock after it free of other stores) ----
reg         wb_st1, wb_hit1, wb_hit2;   // the first store write; its hit; the second word's hit
reg  [31:0] wb_sa1;                  // the store address
reg         st2_v, st2_hit;          // the second write of a crossing store happens now; its hit
reg  [29:0] st2_a;                   // ... its word
reg  [31:0] st2_d;                   // ... its data
reg  [3:0]  st2_be;                  // ... its bytes
wire [63:0] sv  = (wb_sz == `SZ_B) ? {wb_val[7:0], 56'h0} : (wb_sz == `SZ_W) ? {wb_val[15:0], 48'h0} : {wb_val, 32'h0};   // the value at the top
wire [63:0] sdx = sv >> {wb_sa1[1:0], 3'b000};      // shifted to its byte offset: first word in 63-32, spill in 31-0
wire [7:0]  sbx = ((wb_sz == `SZ_B) ? 8'h80 : (wb_sz == `SZ_W) ? 8'hC0 : 8'hF0) >> wb_sa1[1:0];   // the byte enables likewise
reg         x2c;                     // the store crosses a longword (registered: x2 selects the I-cache check address)
wire        x2  = wb_st1 && x2c;     // a crossing store is in its first WB clock
reg  [29:0] a2;                      // wb_sa1's next longword, registered with it (the I-cache smc check reads it)
assign wb_st    = wb_st1 || st2_v;
assign wb_saddr = st2_v ? {st2_a, 2'b00} : wb_sa1;
assign wb_hit   = st2_v ? st2_hit : wb_hit1;
assign sd       = st2_v ? st2_d : sdx[63:32];
assign sbe      = st2_v ? st2_be : sbx[7:4];
reg  [1:0]  lk_d;                    // the last locked step committed one / two clocks ago
reg         rdn;                     // a redirect that is not a replay, one clock ago
always @(posedge clk) rdn <= !rst && redirect_ex && !ex_rep;
reg         lk_p;                    // the last locked step committed, waiting for the store buffer to drain
wire        ex_v_lk = ex_v && ex_u[`U_LOCK];   // a CAS step is in EX
assign ret_st2 = x2; assign ret_saddr2 = {a2, 2'b00}; assign ret_sdata2 = sdx[31:0]; assign ret_sbe2 = sbx[3:0];
always @(posedge clk) begin
  st2_v <= !rst && x2;                                // the second write follows the first
  wb_sac <= x2 ? {a2, 2'b00} : ex_saddr;              // the address the I-cache checks next clock
  st2_a <= a2; st2_d <= sdx[31:0]; st2_be <= sbx[3:0]; st2_hit <= wb_hit2;
  // lock: from the first CAS/CAS2 step that reaches EX until its last step committed and the store buffer is empty
  // with no store in WB (the CAS writes have reached memory; younger stores behind them can extend it); dropped on an
  // exception or interrupt redirect in front of a CAS step
  if (rst) begin lock <= 1'b0; lk_d <= 2'b00; lk_p <= 1'b0; end
  else begin
    lk_d <= {lk_d[0], ex_commit && ex_u[`F_LAST] && ex_u[`U_LOCK]};
    if (lk_d[1]) lk_p <= 1'b1;
    if (rdn && !lk_d[0] && !lk_d[1] && !lk_p) lock <= 1'b0;                     // one clock after the redirect (a register)
    else if (ex_v_lk) begin lock <= 1'b1; lk_p <= 1'b0; end
    else if ((lk_d[1] || lk_p) && sb_cnt == 3'd0 && !wb_st) begin lock <= 1'b0; lk_p <= 1'b0; end
  end
end

// ---- The WB registers -------------------------------------------------------------------------------------------
always @(posedge clk) begin
  if (rst) begin wb_wen <= 1'b0; wb_st1 <= 1'b0; ls_v <= 1'b0; bw_en <= 1'b0; end
  else begin
    wb_wen <= ex_commit && ex_wen;   // only a committed uop writes
    wb_st1 <= ex_commit && ex_st;
    ls_v   <= wb_st;                 // this clock's store is "the store before" next clock
    bw_en  <= wb_st;                 // and goes to the store buffer
  end
  wb_wreg <= ex_wreg; wb_val <= rm;
  // one-hot register hits: A0-A7 in bits 0-7 and T0-T3 in bits 8-11 (registers 20-23, the SP banks, hit nothing); D0-D7
  wb_h  <= ex_wreg[4] ? (ex_wreg[2] ? 12'd0 : (12'd256 << ex_wreg[1:0])) : ex_wreg[3] ? (12'd1 << ex_wreg[2:0]) : 12'd0;
  wb_hd <= (ex_wreg[4:3] == 2'b00) ? (8'd1 << ex_wreg[2:0]) : 8'd0; wb_sa1 <= ex_saddr; a2 <= ex_saddr[31:2] + 30'd1; wb_sz <= ex_u[`F_SZ];
  // a word at offset 3 or a long at offset 1-3 crosses a longword
  x2c <= ((ex_u[`F_SZ] == `SZ_W) && (ex_saddr[1:0] == 2'd3)) || ((ex_u[`F_SZ] == `SZ_L) && (ex_saddr[1:0] != 2'd0)); wb_hit1 <= ex_hit; wb_hit2 <= ex_hit2;
  ls_addr <= wb_saddr[31:2]; ls_data <= sd; ls_be <= sbe;
  bw_addr <= wb_saddr[31:2]; bw_data <= sd; bw_be <= sbe;
end

// ---- The retire port --------------------------------------------------------------------------------------------
always @(posedge clk) begin
  ret_u <= !rst && ex_commit;
  ret_v <= !rst && ex_commit && ex_u[`F_LAST] && !no_ret;
  ret_irq <= ex_u[`U_EXC + 7];       // EXC bits 7-6 = 10: a uop of an interrupt sequence
  ret_pc <= ex_u[`F_PC];
  // AG's An updates (only A0-A7, numbers 8-15, are reported)
  ret_ws_en <= ex_commit && ex_w1_en && ex_w1_r[4:3] == 2'b01; ret_ws_r <= ex_w1_r[2:0]; ret_ws_v <= ex_w1_v;
  ret_wd_en <= ex_commit && ex_w2_en && ex_w2_r[4:3] == 2'b01; ret_wd_r <= ex_w2_r[2:0]; ret_wd_v <= ex_w2_v;
end
assign ret_wen = wb_wen && !wb_wreg[4]; assign ret_wreg = wb_wreg[3:0]; assign ret_wval = wb_val;   // D0-A7 only
assign ret_bk_en = wb_wen && wb_wreg[4] && wb_wreg[2]; assign ret_bk_r = wb_wreg[1:0];          // registers 20-22
assign ret_st = wb_st1; assign ret_saddr = wb_saddr; assign ret_sdata = sd; assign ret_sbe = sbe; assign ret_ccr = ccr;

endmodule
`default_nettype wire
