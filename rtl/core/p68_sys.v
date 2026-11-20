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
// ==== p68_sys.v: the system unit (supervisor state, exceptions, interrupts) ==========================================
//
// Abbreviations: EX = the execute stage (p68_ex.v, which instantiates this unit), DS = decode, SR = the status register,
// CCR = its low byte (the flags X N Z V C), VBR = vector base register, SFC/DFC = source and destination function code
// registers (used by MOVES), CACR/CAAR = cache control and cache address registers, USP/ISP/MSP = the user, interrupt and
// master stack pointers, NMI = non-maskable interrupt (level 7), RTE = return from exception, xseq = the exception
// sequencer in DS (p68_xseq.v), which writes the stack frame.
//
// ---- Supervisor state and exceptions from first principles ---------------------------------------------------------
//
// The 68k has two privilege levels. User mode runs application code; supervisor mode (the S bit of SR set) runs the
// operating system and may change the whole SR, the control registers and the stack pointers. The upper byte of SR is
// the "system byte": T1 and T0 (trace), S (supervisor), M (master: on the 68020 the supervisor has two stacks, MSP and
// ISP), and I2-I0 (the interrupt mask, 0-7). The CCR stays in p68_ex.v, where the flags are made; this unit holds only
// the system byte.
//
// An exception is the CPU's way of leaving the running program for a handler: a TRAP instruction, an illegal opcode, a
// divide by zero, a CHK out of bounds, a privilege violation, an interrupt from a device. In each case the CPU switches
// to supervisor mode, pushes a small "stack frame" (the old SR, a PC to return to, a word with the frame format and the
// vector number, and for some kinds the address of the instruction) on the supervisor stack, and jumps to the address
// stored in the vector table at VBR + 4 * vector. RTE pops the frame and continues where the program was. For some
// vectors the stacked PC is the instruction itself (it did not happen and may be retried, e.g. an illegal opcode or a
// privilege violation), for the others it is the next instruction (it completed, e.g. TRAP or a divide by zero).
//
// Interrupts: a device requests a level on the ipl pins (1-6, or 7 = NMI). It is taken when its level is above the
// mask in SR; level 7 cannot be masked and is taken on its rising edge. Taking one sets the mask to its level, so only
// a higher level can interrupt the handler, and the vector is the autovector 24 + level (the Amiga uses autovectors).
//
// ---- What this unit does ------------------------------------------------------------------------------------------
//
// It holds the system byte of SR, VBR, SFC, DFC, CACR and CAAR (plain registers), the latches of the RTE and CHK2
// sequences, the STOP state, the RESET counter and the interrupt logic. It executes UN_SYS uops and decides when an
// exception is taken:
//   - a uop with an exception vector (EXC[5:0]) raises it after it commits (UN_SYS and UN_DIV only when their condition
//     holds; the multiply/divide unit drives xc_div);
//   - a pending interrupt is taken in front of the first uop of an instruction when that uop reaches EX (the uop does
//     not commit), or at once while stopped.
// On a take: SR gets S set and T1 T0 cleared (an interrupt also sets the mask to its level), the stack pointers swap if
// S changed, the pipeline flushes, and DS (p68_xseq.v) runs the exception sequence with the parameters in xs_par.
// Every write of the system byte (MOVE, ANDI, ORI, EORI to SR, STOP, RTE, a take) flushes the pipeline, so DS always
// decodes with the committed S and M, and the A7 swap (done by the register file one clock later) never meets an
// address computation in flight.
//
// ---- Design rules and what was tried ------------------------------------------------------------------------------
//
// The rule that came out of the exception work and never changed: a condition that raises an exception must come from
// a register, because it feeds the global flush. So CHK, CHK2 and divide by zero latch their compare (sy_c, md_dz) and
// raise it in a following uop (SY_XCHK, MD_CHK). The interrupt pending decision is also a register, one clock old.
// A 2000-program lockstep run once found a polling loop (MOVEQ, a load from uncached memory, DBcs) that could never
// take an interrupt: the uncached load holds interrupts until it commits, and the head of the loop always ran in lane
// B, where no interrupt is taken. The fix is iwant: while an unmasked interrupt is requested, AG forms no pairs. It
// costs nothing without interrupts (CPI unchanged at 1.007), and the bench gained a test group for exactly this case
// (179,224 interrupts in the directed run). The signed compares of CHK and CHK2 are written as 33-bit subtractions,
// because synthesis built $signed compares as chains of ten and more LUTs, while a subtraction maps to a carry chain.
// Several decodes of the EX uop (the result source, the raise and redirect classes) are made in p68_ex.v when the uop
// enters EX and passed here as registers (sy_sel, sy_dec), so this unit's outputs have registers in front of them.
//
// Runs on the CPU clock clk.
`include "p68_uop.vh"                // the uop format
`include "p68_sys.vh"                // system unit operation codes SY_* and control register indices CR_*
`default_nettype none
module p68_sys (
  input  wire        clk,            // CPU clock
  input  wire        rst,            // synchronous reset: supervisor mode, mask 7, VBR 0
  input  wire        ex_v,           // a uop is in EX
  input  wire        ex_rep,         // ... it replays
  input  wire [`UW-1:0] ex_u,        // ... the uop
  input  wire [31:0] a, b,           // EX operands
  input  wire [31:0] npc,            // address of the next instruction
  input  wire [4:0]  ccr,            // CCR now
  input  wire [4:0]  ccr_n,          // CCR after this clock (stacked SR includes the faulting uop's flags)
  input  wire [8:0]  sy_sel,         // result source one-hot (p68_ex sysv): SR, CCR, sy_t, npc, SFC, DFC, CACR, VBR, CAAR
  input  wire [9:0]  sy_dec,         // exception and redirect decode of the uop in EX (p68_ex sdecv), registered
  input  wire        cond,           // condition CC of the uop in EX
  input  wire        cond_r,         // the same on the committed CCR (TRAPcc; AG keeps it two behind any uop)
  input  wire [31:0] a_r, b_r,       // the EX operands without WB forwarding (CHK, CHK2; AG keeps them two behind)
  input  wire [2:0]  ipl,            // the requested interrupt level, synchronous to clk
  input  wire        xc_div,         // divide by zero (UN_DIV raises its EXC when set)
  input  wire        xc_fpu,         // UN_FPU raise condition (FRESTORE format error, pending FP trap)
  input  wire [5:0]  xv_fpu,         // its vector
  input  wire        irq_hold,       // an uncached read is pending (interrupts wait, registered)
  input  wire        fbad,           // RTE format word not 0, 1, 2 (a[15:12])
  output wire        irq_take,       // interrupt in front of the uop in EX: it does not commit
  output reg         iwant,          // an unmasked interrupt is requested (the pending term without the uncached hold)
  output reg  [4:0]  sy_ccr,         // CCR after a committing UN_SYS uop
  output reg  [31:0] sy_r,           // result of a UN_SYS uop
  output wire        sy_red,         // redirect: take, SR write, jump
  output wire [31:0] sy_tgt,         // ... to here
  output wire        no_ret,         // the committing last uop does not retire its instruction
  output wire        rs_one,         // RTE throwaway frame: restart the instruction at step 1
  output wire        sup, msp,       // the S and M bits
  output reg         stopped,        // STOP: waiting for an interrupt
  output wire        irq_pnd,        // an interrupt is pending (EX's redirect)
  output wire        sy_cq,          // the latched CHK condition sy_c (EX's raise condition)
  output wire        rst_done,       // the RESET instruction's 512 clocks are over
  output wire        xs_go,          // start the exception sequence
  output wire [154:0] xs_par,        // its parameters (see the exception take below)
  output wire        xs_rs,          // restart the exception sequence (a replay or MCLR inside it)
  output wire [4:0]  xs_rs_step,     // ... at this step
  output reg         sw_en,          // swap A7 with bank sw_new, saving it into bank sw_old (next clock)
  output reg  [1:0]  sw_old, sw_new, // 0 USP, 1 ISP, 2 MSP
  output reg         iack,           // interrupt acknowledge (one clock)
  output reg  [2:0]  iack_lvl,       // ... the level taken
  output wire        reset_o,        // the RESET instruction's output (high for 512 clocks)
  output wire [15:0] sr,             // the full SR (system byte and CCR)
  output reg  [31:0] vbr, caar,      // vector base register; cache address register
  output reg  [2:0]  sfc, dfc,       // function code registers
  output reg  [3:0]  cacr            // cache control register (C, CE, F, E)
) /* synthesis syn_maxfan = 16 */;
reg        t1, t0, s, m;             // trace bits, supervisor, master
reg  [2:0] im;                       // interrupt mask
reg  [31:0] sy_t, stop_pc;           // RTE: new A7; CHK2: lower bound; STOP: the PC to stack when an interrupt ends it
reg  [15:0] sy_sr, sy_pch;           // RTE: the stacked SR; the stacked PC's high word
reg  [1:0]  sy_f;                    // RTE frame format (0, 1, 2)
reg         sy_c;                    // CHK / CHK2 out of bounds (raised by the following SY_XCHK uop)
assign      sy_cq = sy_c;
reg  [9:0]  rcnt;                    // RESET counter
reg  [2:0]  ipl_r, ipl_q;            // ipl registered, and one clock older (for the level 7 edge)
reg         nmi_p;                   // a level 7 edge is waiting to be taken
assign sr  = {t1, t0, s, m, 1'b0, im, 3'b000, ccr_n};          // ccr_n = CCR with the uop in WB (full)
wire [15:0] srf = {t1, t0, s, m, 1'b0, im, 3'b000, ccr};        // for the uop in EX (a flag reader)
assign sup = s;
assign msp = m;

wire        us   = (ex_u[`F_UNIT] == `UN_SYS);   // the EX uop is a system uop
wire [4:0]  op   = ex_u[`F_OP];
wire [1:0]  sz   = ex_u[`F_SZ];
wire [1:0]  lsz  = ex_u[`F_LSZ];
wire [7:0]  exc  = ex_u[`F_EXC];
wire        xs   = (exc[7:6] != 2'b00);          // a uop of the exception sequence itself

function [31:0] sx;                   // sign extension by size
  input [31:0] v; input [1:0] z;
  begin sx = (z == `SZ_B) ? {{24{v[7]}}, v[7:0]} : (z == `SZ_W) ? {{16{v[15]}}, v[15:0]} : v; end
endfunction
function [1:0] spi;                   // stack pointer of S, M: 0 USP, 1 ISP, 2 MSP
  input sv; input mv;
  begin spi = sv ? (mv ? 2'd2 : 2'd1) : 2'd0; end
endfunction

// ---- Interrupts -----------------------------------------------------------------------------------------------------
reg  [2:0]  lvl;                     // the level to take, registered with pend_r
// The pending decision is a register (the compare had been in front of commit, redirect and fetch). It is one clock
// old: blocked in the clock after any system-unit commit or take (the mask, S or the NMI latch may have changed), and
// irq_hold (registered sources) only delays a take by a clock.
reg         pend_r, pend_blk;
wire        irq_pend = pend_r && !pend_blk;
assign      irq_take = ex_v && ex_u[`F_FIRST] && !xs && irq_pend && !stopped;   // only between instructions
wire        stop_irq = stopped && irq_pend;  // an interrupt ends STOP
assign      irq_pnd  = irq_pend;
wire        commit   = ex_v && !ex_rep && !irq_take;

// ---- UN_SYS operations ------------------------------------------------------------------------------------------
// srv: the new SR of MOVE / AND / OR / EOR to SR (OP bits 1-0 choose the operation)
reg  [15:0] srv;
always @* case (op[1:0])
  2'd0: srv = a[15:0];
  2'd1: srv = srf & a[15:0];
  2'd2: srv = srf | a[15:0];
  default: srv = srf ^ a[15:0];
endcase
// CHK: trap when Dn < 0 or Dn > bound (signed, word or long)
wire [31:0] cs  = (sz == `SZ_W) ? {{16{b_r[15]}}, b_r[15:0]} : b_r;      // CHK: Dn and bound (unforwarded)
wire [31:0] cb  = (sz == `SZ_W) ? {{16{a_r[15]}}, a_r[15:0]} : a_r;
// the signed compares as subtractions (33 bits, sign of the difference), so they map to carry chains (synthesis built
// $signed compares as chains of ten and more LUTs)
wire [32:0] d_ck = {cb[31], cb} - {cs[31], cs};                                 // cs > cb
wire        chk_t = cs[31] || d_ck[32];
// CHK2/CMP2: Rn against the lower bound (sy_t, latched by SY_C2L) and the upper bound (A here). Z when Rn equals
// either bound; C (out of bounds) when Rn < lower or Rn > upper, or, when the upper bound is below the lower one (a
// range that wraps), when both hold.
wire [31:0] up  = sx(a_r, lsz);                                            // CHK2: bounds and Rn (unforwarded)
wire [31:0] rg  = ex_u[`U_IMM] ? b_r : sx(b_r, sz);                       // Rn: an An is used whole, a Dn at the size
wire        c2z = (rg == sy_t) || (rg == up);
wire [32:0] d_tu = {up[31], up} - {sy_t[31], sy_t};                           // up < sy_t
wire [32:0] d_rt = {rg[31], rg} - {sy_t[31], sy_t};                           // rg < sy_t
wire [32:0] d_ur = {up[31], up} - {rg[31], rg};                               // rg > up
wire        c2c = !c2z && (!d_tu[32] ? (d_rt[32] || d_ur[32]) : (d_ur[32] && d_rt[32]));
wire [3:0]  fmt = a[15:12];          // RTE: the frame format nibble of the format/vector word
wire        fmt_bad = fbad;                      // from p68_ex (format check on the registered sources)
// the CCR after a system uop
always @* begin
  sy_ccr = ccr;
  case (op)
    `SY_MVSR, `SY_ANSR, `SY_ORSR, `SY_EOSR, `SY_MVSRN: sy_ccr = srv[4:0];   // writes to SR or CCR
    // CHK: Z from Dn, V and C cleared, N set by the trap condition (Dn < 0 sets it, Dn > bound clears it)
    `SY_CHK:  begin sy_ccr[2] = (cs == 32'h0); sy_ccr[1] = 1'b0; sy_ccr[0] = 1'b0; if (chk_t) sy_ccr[3] = cs[31]; end
    `SY_C2U:  begin sy_ccr[2] = c2z; sy_ccr[0] = c2c; end
    `SY_RTEJ: sy_ccr = sy_sr[4:0];   // RTE: the stacked CCR
    `SY_STOP: sy_ccr = a[4:0];       // STOP #imm loads all of SR
    default: ;
  endcase
end
// Register sources only, one-hot by sy_sel (registered in EX); results A and MOVX's sign-extended A are p68_ex's A
// terms (s_mov, s_xb, s_xw); sy_r is 0 for every other uop.
always @* sy_r = ({32{sy_sel[0]}} & {16'h0, srf}) | ({32{sy_sel[1]}} & {27'h0, ccr}) | ({32{sy_sel[2]}} & sy_t) |
                 ({32{sy_sel[3]}} & npc) | ({32{sy_sel[4]}} & {29'h0, sfc}) | ({32{sy_sel[5]}} & {29'h0, dfc}) |
                 ({32{sy_sel[6]}} & {30'h0, cacr[1:0]}) | ({32{sy_sel[7]}} & vbr) | ({32{sy_sel[8]}} & caar);

// ---- Exception take ---------------------------------------------------------------------------------------------
wire        fu      = (`P68_FPU != 0) && (ex_u[`F_UNIT] == 4'd10);      // UN_FPU
// xc and the raise condition from the registered decode (sy_dec), one AND-OR
wire        xc      = (sy_dec[1] & sy_c) | (sy_dec[2] & cond_r) | (sy_dec[3] & fmt_bad) | (sy_dec[4] & xc_div) |
                      (sy_dec[5] & xc_fpu) | sy_dec[6];
wire        x_raise = commit && sy_dec[0] && xc;   // the committing uop raises its vector
wire        irq     = irq_take || stop_irq;
wire        take    = x_raise || irq;
wire [5:0]  ev      = fu ? xv_fpu : exc[5:0];      // the vector of a raise (the FPU gives its own)
wire [7:0]  vec     = irq ? {5'b00011, lvl} : {2'b00, ev};   // an interrupt uses the autovector 24 + level
// frame format 2 (six words, with the instruction address) for CHK, CHK2, TRAPcc, TRAPV, divide by zero and trace
wire        v_f2    = !irq && (ev == 6'd5 || ev == 6'd6 || ev == 6'd7 || ev == 6'd9);
// the stacked PC is the instruction itself for the FPU exceptions (48-54) and vectors 4, 8, 10, 11 and 14
wire        v_pre   = fu || (ev >= 6'd48 && ev <= 6'd54) || (ev == 6'd4 || ev == 6'd8 || ev == 6'd10 || ev == 6'd11 || ev == 6'd14);
wire [31:0] pc_st   = stop_irq ? stop_pc : (irq_take || v_pre) ? ex_u[`F_PC] : npc;   // the PC to stack
wire [31:0] pcr     = irq ? pc_st : ex_u[`F_PC];   // the PC the exception sequence's uops carry
wire [15:0] sr_st   = {t1, t0, s, m, 1'b0, im, 3'b000, ccr_n};   // the SR to stack (before the take changes it)
wire [31:0] va      = vbr + {22'h0, vec, 2'b00};   // the vector's address: VBR + 4 * vector
assign xs_go  = take;
// the exception sequence's parameters: {uop PC, vector address, instruction address, stacked PC, stacked SR, vector,
// interrupt, interrupt with M set (throwaway frame), frame format 2}
assign xs_par = {pcr, va, ex_u[`F_PC], pc_st, sr_st, vec, irq, irq && m, v_f2};

// ---- Redirects ------------------------------------------------------------------------------------------------------
wire        c_sys = commit && us;
wire        ser   = commit && sy_dec[7];             // SR written (or STOP, SER): continue at the next instruction
wire        jmp   = commit && sy_dec[8];             // JMP (the vector read, RTS, RTD, RTR) or RTE's jump
wire        mclr  = commit && sy_dec[9];             // M cleared in the exception sequence
wire        rte1  = c_sys && (op == `SY_RTEJ) && (sy_f == 2'd1);   // RTE of a throwaway frame: run RTE again on the new stack
assign sy_red     = take || ser || jmp || mclr;
assign sy_tgt     = rte1 ? ex_u[`F_PC] : (jmp && op == `SY_RTEJ) ? {sy_pch, a[15:0]} : jmp ? a : ser ? npc :
                    stop_irq ? stop_pc : ex_u[`F_PC];
assign no_ret     = x_raise || rte1;
assign rs_one     = rte1;
assign xs_rs      = ex_v && xs && (ex_rep || mclr);
assign xs_rs_step = ex_rep ? ex_u[`F_STEP] : ex_u[`F_STEP] + 5'd1;   // a replay restarts at its step, MCLR at the next
assign rst_done   = (rcnt == 10'd1);
assign reset_o    = (rcnt != 10'd0);

// ---- State ----------------------------------------------------------------------------------------------------------
task wr_sr;                           // write the system byte; swap stack pointers when S/M select another one
  input [15:0] v;
  begin
    t1 <= v[15]; t0 <= v[14]; s <= v[13]; m <= v[12]; im <= v[10:8];
    if (spi(v[13], v[12]) != spi(s, m)) begin sw_en <= 1'b1; sw_old <= spi(s, m); sw_new <= spi(v[13], v[12]); end
  end
endtask

always @(posedge clk) begin
  if (rst) begin
    t1 <= 1'b0; t0 <= 1'b0; s <= 1'b1; m <= 1'b0; im <= 3'd7;   // the 68k comes out of reset in supervisor mode, mask 7
    vbr <= 32'h0; caar <= 32'h0; sfc <= 3'd0; dfc <= 3'd0; cacr <= 4'd0;
    stopped <= 1'b0; rcnt <= 10'd0; ipl_r <= 3'd0; ipl_q <= 3'd0; nmi_p <= 1'b0;
    pend_r <= 1'b0; pend_blk <= 1'b1; lvl <= 3'd0; iwant <= 1'b0;
    sw_en <= 1'b0; iack <= 1'b0; iack_lvl <= 3'd0; sy_f <= 2'd0;
  end else begin
    sw_en <= 1'b0; iack <= 1'b0;
    // sw_old / sw_new count only with sw_en: the exception take's values by default (no take term in their enable);
    // the system unit's writes below replace them
    sw_old <= 2'd0; sw_new <= spi(1'b1, m);
    ipl_r <= ipl; ipl_q <= ipl_r;
    // pending: an NMI edge, or a level above the mask, and no uncached read waiting
    pend_r <= (nmi_p || (ipl_r > im)) && !irq_hold; iwant <= nmi_p || (ipl_r > im); lvl <= nmi_p ? 3'd7 : ipl_r; pend_blk <= take || c_sys;
    if (ipl_r == 3'd7 && ipl_q != 3'd7) nmi_p <= 1'b1;   // the rising edge to level 7
    if (rcnt != 10'd0) rcnt <= rcnt - 10'd1;
    if (take) begin
      t1 <= 1'b0; t0 <= 1'b0; s <= 1'b1; stopped <= 1'b0;
      if (irq) begin im <= lvl; iack <= 1'b1; iack_lvl <= lvl; if (lvl == 3'd7) nmi_p <= 1'b0; end
      if (!s) sw_en <= 1'b1;                              // user to supervisor: A7 becomes the ISP or MSP
    end
    // Not behind take: only XCHK, TRAPcc and RTEF can raise while committing; RTEF keeps the check (it alone waits
    // for take, because of its format check), the other two write nothing here. An interrupt take is never a commit.
    if (c_sys) begin
      case (op)
        `SY_MVSR, `SY_ANSR, `SY_ORSR, `SY_EOSR, `SY_MVSRN: if (sz == `SZ_W) wr_sr(srv);   // a byte write is to CCR only
        `SY_RTEJ:  wr_sr(sy_sr);
        `SY_STOP:  begin wr_sr(a[15:0]); stopped <= 1'b1; stop_pc <= npc; end
        `SY_MCLR:  begin m <= 1'b0; sw_en <= 1'b1; sw_old <= 2'd2; sw_new <= 2'd1; end   // MSP out, ISP in
        `SY_MCW:   case (ex_u[`U_IMM +: 3])                // MOVEC Rn,cr
                     `CR_SFC:  sfc  <= a[2:0];
                     `CR_DFC:  dfc  <= a[2:0];
                     `CR_CACR: cacr <= a[3:0];
                     `CR_VBR:  vbr  <= a;
                     default:  caar <= a;
                   endcase
        `SY_C2L:   sy_t <= sx(a, lsz);                     // CHK2 lower bound
        `SY_CHK:   sy_c <= chk_t;
        `SY_C2U:   sy_c <= c2c;
        `SY_RTEF:  if (!take) begin sy_t <= b + ((fmt == 4'd2) ? 32'd12 : 32'd8); sy_f <= fmt[1:0]; end   // A7 after the frame
        `SY_RTESR: sy_sr <= a[15:0];
        `SY_RTEPH: sy_pch <= a[15:0];
        `SY_RESET: rcnt <= 10'd512;
        default: ;
      endcase
    end
  end
end

endmodule
`default_nettype wire
