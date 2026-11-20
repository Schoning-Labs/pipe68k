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
// ==== p68_fpu.v: the FPU unit (registers, uop interface, the boundary to the engine) ==================================
//
// Abbreviations: FPU = floating-point unit, EX = execute (p68_ex.v instantiates this unit in lane A), OC2 = the second
// operand stage, DS = decode, FPn = FP0-FP7, FPCR = FP control register, FPSR = FP status register, FPIAR = FP
// instruction address register, NaN = not a number, EXC/AEXC = the exception and accrued exception bytes of FPSR,
// BSUN/SNAN/OPERR/OVFL/UNFL/DZ/INEX = the FPU's exceptions (branch on unordered, signaling NaN, operand error, overflow,
// underflow, divide by zero, inexact), RAM16SDP4 = Gowin's 16 x 4 distributed (LUT) RAM.
//
// ---- The 68881/68882 programming model from first principles ------------------------------------------------------
//
// The 68020 has no floating-point instructions of its own; it hands them to a coprocessor, the 68881 or the faster
// 68882, through the line-F opcodes ($Fxxx with coprocessor id 1). pipe68k builds that coprocessor into the core.
// Software sees eight data registers FP0-FP7 of 80 bits each, in the "extended precision" format:
//
//     bit 79      sign (1 = negative)
//     bits 78-64  exponent, 15 bits, biased by $3FFF (so $3FFF means 2^0, $4000 means 2^1, $3FFE means 2^-1)
//     bits 63-0   mantissa, 64 bits, with the integer bit (bit 63) stored explicitly
//
// The value is (-1)^sign * mantissa * 2^(exponent - $3FFF - 63). A "normalized" number has bit 63 set, so the mantissa
// reads 1.xxxx in binary. Exponent 0 with bit 63 clear is a denormal (a number too small to normalize), exponent $7FFF
// is infinity (mantissa fraction 0) or a NaN (anything else); a NaN whose bit 62 is clear is "signaling" and raises
// SNAN when an operation uses it. Memory operands can also be single precision (32 bits: 1 sign, 8 exponent bits biased
// by 127, 23 fraction bits with a hidden integer bit), double precision (64 bits: 1, 11 bits biased by 1023, 52), or
// integers (byte, word, long); the FPU converts them on the way in and out.
//
// FPCR controls the arithmetic: bits 15-8 enable a trap for each of the eight exceptions, bits 7-6 the rounding
// precision (extended, single or double), bits 5-4 the rounding mode (to nearest, toward zero, toward minus infinity,
// toward plus infinity). FPSR reports: bits 27-24 the condition codes N (negative), Z (zero), I (infinity) and NAN of
// the last result, bits 23-16 a quotient byte (FMOD/FREM, trapped here), bits 15-8 the exceptions of the last operation
// (EXC) and bits 7-3 the exceptions accrued since software last cleared them (AEXC). FPIAR holds the address of the last
// FPU instruction that could trap.
//
// ---- This unit -------------------------------------------------------------------------------------------------------
//
// p68_fpu.v holds the architectural state (FP0-FP7, FPCR, FPSR, FPIAR, the NULL state, the pending trap, the FRESTORE
// words), the uop interface, and everything DS and OC2 read. The arithmetic is in the engine p68_fpe.v, instantiated
// here. What is done in hardware and what traps to software: everything of the 68881/68882 except the transcendentals
// (FSIN, FETOX and friends), FMOD/FREM, the packed decimal format and a few immediate and dynamic forms, which trap to
// line F (vector 11) as on the 68040 and 68060. Packed decimal needs a 64-digit scaling with exact rounding (several
// thousand LUTs for a format Amiga software hardly uses), FMOD/FREM loop for up to 16,000 clocks, and the immediate
// forms need instruction words the decoders do not see.
//
// The interface: the unit takes operand A when a UN_FPU uop is in EX and does not replay (FP_IN, FP_CW, FP_RS), starts
// the engine with FP_GO (IMM = the command word), and delivers results from registers: fp_r is prepared from the uop in
// OC2 and is the result when that uop is in EX (FP_RD, FP_CR, FP_RX, FP_SV, FP_RSA). Sequences give a read of state
// another instruction may still change (FP_CR, FP_RX, FP_SV) an empty step 0, so the state is final when it is read.
//
// Timing rules: no input of the unit depends on the interrupt take. A uop "commits" here when it is in EX and does not
// replay (fc); an interrupt is taken only in front of a FIRST uop, and FIRST FPU uops change only scratch state (the
// staging registers, the engine start, the FRESTORE header). Architectural state changes with later uops only (FP_WR,
// FP_RD last, FP_CW, FP_IN with CC[2], FP_RSC, FP_SV last, the last uop of an instruction for the NULL state).
//
// ---- The half-rate engine on a clock enable ------------------------------------------------------------------------
//
// The engine steps once every P68_FPN clocks (2 in the builds): ce is a register here that is high in one clock out of
// P68_FPN, and every register inside p68_fpe.v changes only in a clock with ce set. The build script then gives every
// path from an engine register to an engine register P68_FPN clocks with set_multicycle_path (Gowin's SUG940 and
// SUG113 describe the method), so the engine's long arithmetic paths do not count against the CPU clock. ce itself has
// a large fanout, so it is put on one of the chip's global clock lines (CLOCK_LOC ... BUFG in the constraints), and a
// preserve attribute makes synthesis keep it as one register. The boundary is registered on both sides: FP_GO latches the
// command word, the next clock reads the register file into the command registers (cb, ca) and raises rq, and the
// engine copies them at its next step; its result is copied here (res, o0-o2, fl, ncc) while its dn is set. FP_WR and
// FP_RD read the copies, and seq_fpu's fixed waits cover the latency (scaled by P68_FPN). So the only single-clock paths
// that touch the engine are flop-to-flop copies, rq, ce and reset.
//
// ---- What was tried, and why it ended up like this ----------------------------------------------------------------
//
// The first version was one unit in EX on the core clock, with every FPU instruction a DS sequence with fixed waits.
// It built at 53.3 MHz. Three timing steps followed (no interrupt term into the unit, comparisons and adder carries
// registered, one shared normalizer and shifter, FP0-FP7 in distributed RAM, which took the unit alone from 9,235 to
// 6,668 LUTs and from 2,690 to 1,507 flip-flops), and all of WinUAE's cputester FPU sets passed (864,016 tests run, 0
// fail). But the integer core ran 71.6 MHz alone and 42-56 MHz with the FPU beside it in six builds of the same RTL,
// with the same logic depth: the extra third of design size spread the placement. Two ways to take the FPU out of the
// core's timing were built. An own asynchronous clock with a two-phase handshake was benched (lockstep passed at clock
// ratios 0.63 and 1.37) but never placed and routed. The half-rate clock enable won: 52.1 MHz at place option 3 and
// 60.0 MHz at place option 2, with no FPU path among the 200 worst. Region boxes for the FPU failed at the same time (a
// box partly outside the chip was refused, and a 27 x 51 box for the engine failed routing with about 3,500 unrouted
// nets). FP0-FP7 live in 40 RAM16SDP4 LUT RAMs, and because those are disabled on one revision of the chip, a board test
// ran 1,512 of them for about 100 seconds at 50 MHz (about 7.6e12 reads, 0 errors) before they were kept.
//
// Runs on the CPU clock clk; the engine steps on ce.
`include "p68_uop.vh"                // the uop format
`include "p68_fpu.vh"                // UN_FPU, FP_* operations, U_FPC/U_FPS, P68_FPN
`default_nettype none
module p68_fpu (
  input  wire        clk,                // CPU clock
  input  wire        rst,                // synchronous reset (the FPU starts in the NULL state)
  input  wire        ex_v,               // uop in EX (register)
  input  wire        ex_rep,             // it replays (register)
  input  wire [`UW-1:0] ex_u,            // the uop in EX
  input  wire [31:0] a,                  // operand A of the uop in EX
  input  wire [`UW-1:0] oc_u,            // uop in OC2 (result preparation)
  output reg  [31:0] fp_r,               // result of the UN_FPU uop in EX
  output wire [3:0]  fpcc,               // FPSR condition codes N Z I NAN (conditionals, evaluated in OC2)
  output wire        xc_fpu,             // raise condition of the UN_FPU uop in EX (a register)
  output wire [5:0]  xv_fpu,             // its vector
  output wire        null_st,            // NULL state (committed)
  output wire        en_any,             // an exception enable bit is set (FPCR[15:8])
  output wire [5:0]  pv,                 // pending arithmetic trap vector (0: none), committed (DS)
  output wire        bsun_en,            // FPCR BSUN enable (EX: trap at the conditional)
  output wire [79:0] ret_fp0, ret_fp1, ret_fp2, ret_fp3, ret_fp4, ret_fp5, ret_fp6, ret_fp7,   // FP0-FP7 for the retire port
  output wire [31:0] ret_fpcr, ret_fpsr, ret_fpiar   // the control registers for the retire port
);
// the default NaN: positive, exponent $7FFF, all mantissa bits set (what an invalid operation produces)
localparam [79:0] DNAN = {1'b0, 15'h7fff, 64'hFFFFFFFFFFFFFFFF};
// ---- State ---------------------------------------------------------------------------------------------------------
reg  [79:0] frm [0:7];                   // FP0-FP7 (distributed RAM)
reg  [7:0]  fdn;                         // FPn holds the default NaN
reg  [15:0] fpcr;                        // bits 3:0 always 0
reg  [27:0] fpsr;                        // bits 2:0 always 0
reg  [31:0] fpiar;                       // the address of the last FPU instruction that could trap
reg         nul;                         // NULL state
reg  [5:0]  pend;                        // pending arithmetic trap vector (0: none); DS raises it at the next FPU instruction
reg         rs_bad;                      // FRESTORE: bad frame (format error)
reg  [31:0] rs_h, rs_b1, rs_b2;          // FRESTORE words
assign ret_fpcr = {16'h0, fpcr}; assign ret_fpsr = {4'h0, fpsr}; assign ret_fpiar = fpiar;
// The retire port's view of FP0-FP7 exists only in simulation: reading all eight registers at once would need eight
// read ports of the LUT RAM, so a hardware build ties these outputs to 0.
`ifdef VERILATOR
assign ret_fp0 = fdn[0] ? DNAN : frm[0]; assign ret_fp1 = fdn[1] ? DNAN : frm[1];
assign ret_fp2 = fdn[2] ? DNAN : frm[2]; assign ret_fp3 = fdn[3] ? DNAN : frm[3];
assign ret_fp4 = fdn[4] ? DNAN : frm[4]; assign ret_fp5 = fdn[5] ? DNAN : frm[5];
assign ret_fp6 = fdn[6] ? DNAN : frm[6]; assign ret_fp7 = fdn[7] ? DNAN : frm[7];
`else
assign ret_fp0 = 80'h0; assign ret_fp1 = 80'h0; assign ret_fp2 = 80'h0; assign ret_fp3 = 80'h0;
assign ret_fp4 = 80'h0; assign ret_fp5 = 80'h0; assign ret_fp6 = 80'h0; assign ret_fp7 = 80'h0;
`endif
assign fpcc = fpsr[27:24];               // N Z I NAN
assign null_st = nul;
assign en_any = |fpcr[15:8];
assign pv = pend;
assign bsun_en = fpcr[15];

wire        u_fp  = (ex_u[`F_UNIT] == `UN_FPU);   // the EX uop is an FPU uop
wire [4:0]  op    = ex_u[`F_OP];
wire [3:0]  cc    = ex_u[`F_CC];
wire        fc    = ex_v && !ex_rep;                                  // see the header: no interrupt term
wire        c_fp  = fc && u_fp;                                       // an FPU uop commits here
wire        c_go  = c_fp && (op == `FP_GO);                           // ... and it is FP_GO
// raise condition: FRESTORE's bad frame (FP_RSC is the only UN_FPU uop with EXC set); a register
assign xc_fpu = rs_bad;
assign xv_fpu = 6'd14;                   // format error

// ---- Command (latched at FP_GO; the register file is read in the next clock) -----------------------------------------
// The command word is the FPU instruction's second word: bits 15-13 the opclass (000 register to register, 010 memory
// or Dn to register, 011 register to memory), bit 14 "source is not an FP register", bits 12-10 the source register or
// the memory format, bits 9-7 the destination register, bits 6-0 the operation (opmode).
reg  [15:0] cw;                          // the command word
reg  [1:0]  md;                          // rounding mode
reg  [1:0]  pp;                          // FPCR precision field
reg  [4:0]  k;                           // operation kind (K_*)
localparam K_MOV = 5'd0, K_ABS = 5'd1, K_NEG = 5'd2, K_INT = 5'd3, K_INTRZ = 5'd4, K_SQRT = 5'd5, K_GETE = 5'd6,
           K_GETM = 5'd7, K_DIV = 5'd8, K_ADD = 5'd9, K_MUL = 5'd10, K_SDIV = 5'd11, K_SCALE = 5'd12, K_SMUL = 5'd13,
           K_SUB = 5'd14, K_CMP = 5'd15, K_TST = 5'd16, K_CR = 5'd17, K_OUT = 5'd18;
// kind: the operation from the command word (opmodes of the 68881 manual; the variants that only differ in rounding
// precision on the 68040 are not decoded here, dec_fpu.v lets only the 68882 opmodes through)
function [4:0] kind; input [15:0] w;
  reg [6:0] o;
  begin
    o = w[6:0];
    if (w[15:13] == 3'd3) kind = K_OUT;                              // FMOVE FPn,<ea>
    else if (w[14] && w[12:10] == 3'd7) kind = K_CR;                 // FMOVECR (source format 7)
    else casez (o)
      7'h00: kind = K_MOV;  7'h01: kind = K_INT;  7'h03: kind = K_INTRZ; 7'h04, 7'h05: kind = K_SQRT;
      7'h18: kind = K_ABS;  7'h1a, 7'h1b: kind = K_NEG; 7'h1e: kind = K_GETE; 7'h1f: kind = K_GETM;
      7'h20: kind = K_DIV;  7'h22: kind = K_ADD; 7'h23: kind = K_MUL; 7'h24: kind = K_SDIV; 7'h26: kind = K_SCALE;
      7'h27: kind = K_SMUL; 7'b0101???: kind = K_SUB; 7'h38, 7'h39, 7'h3c, 7'h3d: kind = K_CMP;
      default: kind = K_TST;                                         // 0x3a, 0x3b, 0x3e, 0x3f
    endcase
  end
endfunction

// ---- Register file reads: port 1 (engine source, FMOVEM.X store), port 2 (engine destination) ------------------------
wire        rmf  = cw[14] && (cw[15:13] != 3'd3);   // the source comes from memory, Dn or #imm (the staging words)
// port 1 address: the register an FMOVEM.X store in OC2 reads, else FMOVE out's source (bits 9-7), else the source FPm
wire [2:0]  ra1  = (oc_u[`F_UNIT] == `UN_FPU && oc_u[`F_OP] == `FP_RX) ? oc_u[`U_IMM +: 3] : (cw[15:13] == 3'd3) ? cw[9:7] : cw[12:10];
wire [2:0]  ra2  = cw[9:7];              // port 2 address: the destination FPn
wire [79:0] rd1  = fdn[ra1] ? DNAN : frm[ra1];
wire [79:0] rd2  = fdn[ra2] ? DNAN : frm[ra2];
reg  [31:0] s0, s1, s2;                  // input staging

// ---- Step enable, command registers, engine, result copies -----------------------------------------------------------
reg  [1:0]  fph;                         // step phase 0 .. P68_FPN - 1
reg         ce /* synthesis syn_preserve = 1 */;   // the engine steps at the end of this clock (CLOCK_LOC: global line)
reg         ld, rq, wt;                  // read the register file; start request; FP_GO committed, engine not started
reg  [95:0] cb;                          // source: staging words or {FPm, 16'h0}
reg  [79:0] ca;                          // destination FPn
wire        e_dn;                        // the engine's result is complete
wire [79:0] e_res;                       // ... the value
wire [31:0] e_o0, e_o1, e_o2;            // ... the FMOVE out words
wire [7:0]  e_fl;                        // ... the EXC byte
wire [3:0]  e_cc;                        // ... the condition codes
p68_fpe i_fpe (.clk(clk), .rst(rst), .ce(ce), .rq(rq), .c_cw(cw), .c_k(k), .c_md(md), .c_pp(pp), .c_b(cb), .c_a(ca),
               .dn(e_dn), .res(e_res), .o0(e_o0), .o1(e_o1), .o2(e_o2), .fl(e_fl), .rcc(e_cc));
reg  [79:0] res;                         // copies of the engine's result
reg  [31:0] o0, o1, o2;
reg  [7:0]  fl;
reg  [3:0]  ncc;
reg         cv, cvd;                     // the copies hold the current operation's result (cvd: one clock later)
always @(posedge clk) begin
  if (rst) begin fph <= 2'd0; ce <= 1'b0; ld <= 1'b0; rq <= 1'b0; wt <= 1'b0; cv <= 1'b0; end
  else begin
    fph <= (fph == `P68_FPN - 1) ? 2'd0 : fph + 2'd1;     // counts 0 .. P68_FPN - 1, round and round
    ce  <= (fph == `P68_FPN - 1);                          // one clock in P68_FPN
    ld  <= c_go;                                           // the clock after FP_GO reads the register file
    if (ld) begin cb <= rmf ? {s0, s1, s2} : {rd1, 16'h0}; ca <= rd2; rq <= 1'b1; end
    else if (ce) rq <= 1'b0;                               // the engine took the start at this step
    if (c_go) wt <= 1'b1; else if (rq && ce) wt <= 1'b0;   // the old result is still in the engine until it starts
    if (c_go) cv <= 1'b0; else if (e_dn && !wt) cv <= 1'b1;
  end
  cvd <= cv;
  if (e_dn && !wt) begin res <= e_res; o0 <= e_o0; o1 <= e_o1; o2 <= e_o2; fl <= e_fl; ncc <= e_cc; end   // the copies
  if (c_go) begin
    cw <= ex_u[`U_IMM +: 16]; k <= kind(ex_u[`U_IMM +: 16]); pp <= fpcr[7:6];
    md <= (kind(ex_u[`U_IMM +: 16]) == K_INTRZ) ? 2'd1 : fpcr[5:4];          // FINTRZ rounds toward zero
  end
end

// ---- Result read check (simulation): a result uop must not reach EX before the copies hold the result -----------------
// If seq_fpu's waits were ever too short, this message shows it in the lockstep bench at once.
// synthesis translate_off
always @(posedge clk) if (!rst && c_fp && (op == `FP_WR || op == `FP_RD) && !(cv && cvd))
  $display("FPU: result read before the engine finished (command %04x)", cw);
// synthesis translate_on

// ---- Result preparation from the uop in OC2 -------------------------------------------------------------------------
// fp_r is a register loaded from the uop in OC2, so the FPU's result enters EX's result select as a register.
wire [4:0]  qop = oc_u[`F_OP];
wire [3:0]  qcc = oc_u[`F_CC];
always @(posedge clk) begin
  case (qop)
    `FP_RD:  fp_r <= (qcc[1:0] == 2'd0) ? o0 : (qcc[1:0] == 2'd1) ? o1 : o2;          // FMOVE out word
    `FP_CR:  fp_r <= qcc[2] ? {16'h0, fpcr} : qcc[1] ? {4'h0, fpsr} : fpiar;          // control register
    `FP_RX:  fp_r <= (qcc[1:0] == 2'd0) ? {rd1[79:64], 16'h0} : (qcc[1:0] == 2'd1) ? rd1[63:32] : rd1[31:0];   // FMOVEM.X word
    // FSAVE: a NULL frame is one long $00380000; an IDLE frame (60 bytes) starts with $1F380000 and ends with the BIU
    // flags word $5C0EFFFF, with zeros between (the 68882's frame as WinUAE writes it)
    `FP_SV:  fp_r <= nul ? 32'h00380000 : (qcc == 4'd0) ? 32'h1F380000 : (qcc == 4'd14) ? 32'h5C0EFFFF : 32'h0;
    `FP_RSA: fp_r <= (rs_h[31:24] == 8'h0) ? 32'd4 : {24'h0, rs_h[23:16]} + 32'd4;   // FRESTORE (An)+: frame size + 4
    default: fp_r <= 32'h0;
  endcase
end

// ---- Commit -----------------------------------------------------------------------------------------------------------
// An operation that raises SNAN, OPERR or DZ with that trap enabled does not write its destination (68882 rule).
wire        wsup = |(fl & fpcr[15:8] & 8'b01100100);                 // SNAN, OPERR, DZ with their trap enabled: no write
function [4:0] aex; input [7:0] f;       // accrued bits from EXC
  // AEXC: IOP = BSUN | SNAN | OPERR, OVFL, UNFL = UNFL & INEX2, DZ, INEX = INEX2 | INEX1 | OVFL
  begin aex = {|f[7:5], f[4], f[3] & f[1], f[2], |f[1:0] | f[4]}; end
endfunction
function [5:0] vecof; input [7:0] x;     // the trap vector of the highest-priority enabled exception
  begin
    vecof = x[7] ? 6'd48 : x[6] ? 6'd54 : x[5] ? 6'd52 : x[4] ? 6'd53 : x[3] ? 6'd51 : x[2] ? 6'd50 : (x[1] | x[0]) ? 6'd49 : 6'd0;
  end
endfunction
wire        bsun = fc && ex_u[`U_FPC] && ex_u[`U_FPS] && fpsr[24];   // a signaling predicate saw NAN: set BSUN
// register file write port: FP_WR (the result) or FMOVEM.X (the staging words)
wire        fwe  = c_fp && ((op == `FP_WR && k != K_CMP && k != K_TST && !wsup) || (op == `FP_IN && cc[2]));
wire [2:0]  fwa  = (op == `FP_WR) ? cw[9:7] : ex_u[`U_IMM +: 3];
wire [79:0] fwd  = (op == `FP_WR) ? res : {s0[31:16], s1, a};        // FMOVEM.X: exponent word, mantissa high, mantissa low (A)
always @(posedge clk) if (fwe) frm[fwa] <= fwd;
// the last uop of an FPU instruction (never a FIRST uop) leaves the NULL state, except FSAVE's and FRESTORE's
wire        lastu = fc && ex_u[`F_LAST] && (ex_u[`U_FPC] || (u_fp && op != `FP_SV && op != `FP_RS && op != `FP_RSC && op != `FP_RSA));
always @(posedge clk) begin
  if (rst) begin
    fpcr <= 16'h0; fpsr <= 28'h0; fpiar <= 32'h0; nul <= 1'b1; pend <= 6'd0; rs_bad <= 1'b0; fdn <= 8'hFF;
  end else begin
    if (lastu || (c_fp && (op == `FP_WR || op == `FP_CW || (op == `FP_RD && cc[2])))) nul <= 1'b0;   // never FIRST uops
    if (fwe) fdn[fwa] <= 1'b0;           // the register now holds a real value
    if (bsun) begin fpsr[15] <= 1'b1; fpsr[7] <= 1'b1; end   // EXC BSUN and AEXC IOP
    if (c_fp) case (op)
      `FP_IN: case (cc[1:0]) 2'd0: s0 <= a; 2'd1: s1 <= a; default: s2 <= a; endcase
      `FP_WR: begin                      // the operation commits: condition codes, EXC, AEXC; FPIAR and a trap if enabled
        fpsr <= {ncc, fpsr[23:16], fl, fpsr[7:3] | aex(fl), 3'b000};
        if (|fpcr[14:8]) fpiar <= ex_u[`F_PC];   // FPIAR is loaded only when an enable bit 14-8 is set (68882 as WinUAE)
        if (|(fl & fpcr[15:8])) pend <= vecof(fl & fpcr[15:8]);
      end
      `FP_RD: if (cc[2]) begin           // FMOVE out's last word: the same, without the condition codes
        fpsr <= {fpsr[27:16], fl, fpsr[7:3] | aex(fl), 3'b000};
        if (|fpcr[14:8]) fpiar <= ex_u[`F_PC];
        if (|(fl & fpcr[15:8])) pend <= vecof(fl & fpcr[15:8]);
      end
      `FP_CW: begin                      // control register write (one-hot CC[2:0])
        if (cc[2]) fpcr <= {a[15:4], 4'h0};
        else if (cc[1]) fpsr <= {a[27:3], 3'b000};
        else fpiar <= a;
      end
      `FP_SV: if (ex_u[`U_IMM]) pend <= 6'd0;   // FSAVE's last word clears the pending trap
      `FP_RS: begin                      // FRESTORE: the header and two longs of the frame
        case (cc[1:0]) 2'd0: rs_h <= a; 2'd1: rs_b1 <= a; default: rs_b2 <= a; endcase
        // a valid header is NULL ($00......) or version $1F with an IDLE ($18, $38) or BUSY ($B4, $D4) size
        if (cc[1:0] == 2'd0)
          rs_bad <= (a[31:24] != 8'h0) && !(a[31:24] == 8'h1F && (a[23:16] == 8'h18 || a[23:16] == 8'h38 || a[23:16] == 8'hB4 || a[23:16] == 8'hD4));
      end
      `FP_RSC: if (!rs_bad) begin        // FRESTORE commits
        if (rs_h[31:24] == 8'h0) begin   // NULL frame: reset the FPU
          fpcr <= 16'h0; fpsr <= 28'h0; fpiar <= 32'h0; nul <= 1'b1; fdn <= 8'hFF;
        end else begin                   // IDLE frame: restore the pending trap from the frame's flag bit
          nul <= 1'b0;
          if (rs_h[23:16] == 8'h18) pend <= rs_b1[27] ? 6'd0 : vecof(fpsr[15:8] & fpcr[15:8]);
          if (rs_h[23:16] == 8'h38) pend <= rs_b2[27] ? 6'd0 : vecof(fpsr[15:8] & fpcr[15:8]);
        end
      end
      default: ;
    endcase
  end
end

endmodule
`default_nettype wire
