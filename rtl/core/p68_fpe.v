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
// ==== p68_fpe.v: the FPU engine (the arithmetic, on a half-rate clock enable) =========================================
//
// Abbreviations: FPU = floating-point unit, FPn/FPm = FP registers, FPCR/FPSR = FP control and status registers, NaN =
// not a number, EXC = the exception byte (BSUN SNAN OPERR OVFL UNFL DZ INEX2 INEX1: branch on unordered, signaling NaN,
// operand error, overflow, underflow, divide by zero, inexact), DSP = the FPGA's hard multiplier block, ROM = read-only
// memory, LSB = least significant bit.
//
// ---- What this module does ------------------------------------------------------------------------------------------
//
// p68_fpu.v holds the FPU's registers and talks to the pipeline; this module does the arithmetic of one FPU operation at
// a time: FMOVE in (with the conversion from byte, word, long, single, double or extended), FABS, FNEG, FINT, FINTRZ,
// FSQRT, FGETEXP, FGETMAN, FADD, FSUB, FMUL, FSGLMUL, FDIV, FSGLDIV, FSCALE, FCMP, FTST, FMOVECR (the constant ROM) and
// FMOVE out (the conversion to any memory format). p68_fpu.v starts it (rq with the command registers), and it raises
// dn when the result, the FMOVE out words, the EXC byte and the condition codes are complete.
//
// ---- The clock enable and the multicycle constraint ---------------------------------------------------------------
//
// Every register of this module changes only in a clock with ce set (one clock in P68_FPN, a register in p68_fpu) or
// with rst, so the paths between them have P68_FPN core clocks (tools/build.sh: set_multicycle_path on *i_fpe/*). Paths
// from outside are flop to flop: the command registers of p68_fpu (copied here at the start, rq) and ce; paths out go
// to p68_fpu's copy registers (taken while dn is set). That is why there are no register copies for fanout in here
// (syn_maxfan = 10000 on the busiest registers): with two clocks per step a long net does not hurt, and copies would
// only cost area.
//
// ---- Floating point from first principles -------------------------------------------------------------------------
//
// A floating-point number is sign * mantissa * 2^exponent. The engine keeps every operand "unpacked": a sign, a 20-bit
// signed exponent biased by $3FFF (so it can go far beyond the 15-bit range in the middle of an operation without
// wrapping), a 64-bit mantissa with the integer bit at the top, and a class (finite, zero, infinity, NaN). Finite
// operands are normalized first, i.e. shifted left until bit 63 is 1 (the exponent counts down by the same amount),
// because then every operation can assume its inputs look like 1.xxxx.
//
// The work happens in the working value W: a sign (ws), an exponent (we) and 67 bits (wv) that are the 64 mantissa
// bits plus three more at the bottom, the guard, round and sticky bits. When a result has more bits than fit, the bits
// that fall off the end decide the rounding: the guard bit is the first bit below the last kept bit, and the sticky bit
// is the OR of every bit below that (once a 1 has passed through it, it "sticks"). That is all the information rounding
// needs: whether the part thrown away is less than, exactly or more than half of the last kept bit.
//
// Rounding (the rounder, states S_RT to S_RO). FPCR's precision says how many mantissa bits to keep (64 for extended,
// 24 for single, 53 for double) and its mode says which way: to nearest (round up when the rest is more than half, or
// exactly half and the last kept bit is 1, so ties go to the even number), toward zero (never round up), toward minus
// infinity (round up the magnitude of negative numbers when anything was lost) and toward plus infinity (the same for
// positive numbers). When anything was lost the result is inexact (INEX2). The exponent range depends on the precision
// too: a result whose exponent is below the smallest normal exponent is "tiny" (checked before rounding, as on the
// 68882), is shifted right to the smallest exponent with its bits going into the sticky bit (it becomes a denormal), and
// raises UNFL (the 68882 sets it even when the denormal is exact). A result above the largest exponent raises OVFL and
// becomes infinity, or the largest finite number when the rounding mode points toward zero.
//
// Special values follow fixed rules (state S_SPC): an operation with a NaN gives a NaN (a signaling NaN raises SNAN and
// comes out quiet); invalid operations (infinity minus infinity, zero times infinity, zero divided by zero, the square
// root of a negative number) give the default NaN and raise OPERR; a finite number divided by zero gives infinity and
// raises DZ.
//
// ---- How the engine works -----------------------------------------------------------------------------------------
//
// Operands are unpacked and fully normalized (sign, 20-bit signed exponent biased $3FFF, 64-bit mantissa, class), both
// through W (the normalizer), then kept in A (the destination) and B (the source). Every wide register takes its next
// value from a short list chosen by an operation code per step (wo, eo, ro, dop, aop, mo), and shifts are steps: the
// normalizer shifts W left by 32, 8 or 1-7 (the leading zeros of the top byte) per step, the right shifter (sticky into
// bit 0) by 32, 8 or 1-7. Operations: add and subtract (one adder), multiply (one 16 x 16 DSP product per step, column
// sums), divide and square root (radix 2, one 70-bit subtractor), round to integer, FGETEXP/FGETMAN/FSCALE, FCMP/FTST,
// FMOVECR; the rounder with the 68882's rules. Special results are set up in W and written by S_WR. The number of steps
// varies with the shifts; seq_fpu's fixed waits are set from the worst case of each operation (lat there).
//
// ---- What was tried, and why it ended up like this ----------------------------------------------------------------
//
// The first engine had the same algorithms with wide hardware: a 66-bit leading-zero counter, 67-bit barrel shifters,
// and a multiplier of four 16 x 16 DSP products per step (64 x 16 bits per clock). With the FPU on a half-rate clock
// enable that was no longer needed for speed, so the engine was rewritten small with bit-exact results: per-step
// next-value codes for each wide register, step shifters (by 32, 8 or 1-7 per step) instead of barrel shifters and the
// leading-zero counter, one DSP product per step summed by columns, and the FMOVECR constants in block ROM. It went
// from 8,188 LUTs and 4 DSPs to 4,100 LUTs, 1 DSP and 3 block RAMs. The price is longer waits: at P68_FPN = 2 an
// instruction takes about 12 clocks more for FMOVE, 36 more for FADD, 48 more for FMUL and 22 more for FDIV. Region
// boxes for the engine were tried as well and failed routing (see p68_fpu.v).
//
// The results are checked against the bench's own 68882 reference model (sim/lockstep/fpu_ref.c, written from the
// 68881/68882 manual and WinUAE's behavior) at every retired instruction, and against WinUAE's cputester FPU data.
`include "p68_uop.vh"                    // the uop format
`include "p68_fpu.vh"                    // FPU codes
`default_nettype none
module p68_fpe (
  input  wire        clk,                // CPU clock
  input  wire        rst,                // synchronous reset
  input  wire        ce,                 // step enable (one clock in P68_FPN)
  input  wire        rq,                 // start: the command registers below are loaded (taken at the next ce)
  input  wire [15:0] c_cw,               // command word
  input  wire [4:0]  c_k,                // operation kind (K_*)
  input  wire [1:0]  c_md, c_pp,         // rounding mode, FPCR precision
  input  wire [95:0] c_b,                // source: staging words s0, s1, s2 (memory, Dn, #imm) or {FPm, 16'h0}
  input  wire [79:0] c_a,                // destination FPn
  output reg         dn,                 // the result below is complete (cleared at the start)
  output reg  [79:0] res,                // result value (FPn)
  output reg  [31:0] o0, o1, o2,         // FMOVE out words
  output reg  [7:0]  fl,                 // EXC: BSUN SNAN OPERR OVFL UNFL DZ INEX2 INEX1
  output reg  [3:0]  rcc                 // condition codes N Z I NAN of the result
);
reg  [95:0] fb;                          // source as taken at the start
wire [31:0] s0 = fb[95:64], s1 = fb[63:32], s2 = fb[31:0];   // the three staging words
// ---- Command (taken at the start) ------------------------------------------------------------------------------------
reg  [15:0] cw /* synthesis syn_maxfan = 10000 */;   // multicycle paths: no copies for fanout
reg  [1:0]  md;                          // rounding mode (0 nearest, 1 toward zero, 2 toward -infinity, 3 toward +infinity)
reg  [1:0]  pp;                          // FPCR precision field (0 extended, 1 single, 2 double)
reg  [4:0]  k /* synthesis syn_maxfan = 10000 */;    // operation kind (K_*)
localparam K_MOV = 5'd0, K_ABS = 5'd1, K_NEG = 5'd2, K_INT = 5'd3, K_INTRZ = 5'd4, K_SQRT = 5'd5, K_GETE = 5'd6,
           K_GETM = 5'd7, K_DIV = 5'd8, K_ADD = 5'd9, K_MUL = 5'd10, K_SDIV = 5'd11, K_SCALE = 5'd12, K_SMUL = 5'd13,
           K_SUB = 5'd14, K_CMP = 5'd15, K_TST = 5'd16, K_CR = 5'd17, K_OUT = 5'd18;
wire        dya = (k >= K_DIV && k <= K_CMP);       // a dyadic operation (two operands: the destination is A)
wire [2:0]  of  = cw[12:10];                                         // FMOVE out format
wire        oint = (of == 3'd0) || (of == 3'd4) || (of == 3'd6);    // ... long, word or byte integer

// ---- Engine registers ------------------------------------------------------------------------------------------------
// The states. S_UNP0..S_NA load and normalize the operands, S_PRE registers every comparison the next step needs, S_SPC
// handles special values and chooses the operation's own states, S_RT..S_RO round, S_WR writes a special result,
// S_FIN packs FMOVE out, S_DONE raises dn.
localparam S_IDLE = 5'd0, S_UNP0 = 5'd1, S_UNP = 5'd2, S_NB = 5'd3, S_NA = 5'd4, S_PRE = 5'd5, S_SPC = 5'd6,
           S_ADD0 = 5'd7, S_ADD1 = 5'd8, S_NZ1 = 5'd9, S_NRM = 5'd10, S_SHR = 5'd11, S_MUL = 5'd12, S_MUL2 = 5'd13,
           S_DIV = 5'd14, S_SQRT = 5'd15, S_LDQ = 5'd16, S_RT = 5'd17, S_RT2 = 5'd18, S_RD = 5'd19, S_RI = 5'd20,
           S_RO = 5'd21, S_INT = 5'd22, S_ISAT = 5'd23, S_ISAT2 = 5'd24, S_SCL = 5'd25, S_WR = 5'd26, S_FIN = 5'd27,
           S_DONE = 5'd28;
reg  [4:0]  st /* synthesis syn_maxfan = 10000 */;   // the state
reg  [4:0]  sh_ret, nz_ret, rd_ret;      // the state to return to after the right shift / the normalizer / the round decision
reg  [79:0] avr;                         // the raw destination
reg         sq;                          // square root runs (operand selects of the shared subtractor)
reg         sqd;                         // divide or square root: the remainder is sticky (S_LDQ)
reg         rupr;                        // S_RD: the round-up decision, registered before the add
reg         qn, ones;                    // S_WR: quiet NaN bit, all-ones mantissa (default NaN)
reg  signed [19:0] e_op;                 // S_PRE: result exponent of multiply, divide, square root
reg  [19:0] f_uabs;                      // |unbiased exponent of B| (FGETEXP)
// operands: A = destination (dyadic), B = source; class 0 finite, 1 zero, 2 infinity, 3 NaN. NaN and infinity keep
// their raw mantissa and exponent (no normalization).
reg         as_, bs_;                    // signs
reg  signed [19:0] ae, be;               // exponents (biased $3FFF, 20-bit signed)
reg  [63:0] am, bm;                      // mantissas
reg  [1:0]  ac, bc;                      // classes
// working value W: sign, exponent, {m[63:0], g, r, s}
reg         ws;
reg  signed [19:0] we, we1;              // the exponent; we + 1 (for a rounding carry)
reg  [66:0] wv;                          // 64 mantissa bits, guard, round, sticky
reg  [6:0]  shn, cnt;                    // right shift still to do; step counter of multiply, divide, square root
reg         sub_e, zsg, cy;              // effective subtract; sign of an exact zero result; adder carry out
// comparisons, registered in S_PRE
reg         f_mgt, f_mlt, f_mlo, f_ge3e, f_lt3fff, f_gt400f, f_eq3fff, f_blt0;
reg  [6:0]  f_shab, f_shi;               // |exponent difference| (alignment shift); 63 - unbiased exponent (to integer)
reg  signed [19:0] f_ue;                 // B's unbiased exponent
// result
reg  [3:0]  ccr_o;                       // FCMP's own condition codes
reg         cc_own;                      // ... are used instead of those of the result
// rounder
reg  [1:0]  rp;                          // 0: 64 bits, 1: 24, 2: 53
reg         rx;                          // extended exponent range
reg         tiny, rinx, ovf;             // tiny before rounding; inexact; integer overflow
reg  signed [19:0] dlt;                  // emin - we (the denormalizing shift)
reg         rcy, og0, og1;               // the rounding add carried out; we > emax; we + 1 > emax

function [2:0] lz8; input [7:0] v;       // leading zeros of a byte (0-7; 7 also for an all-zero byte)
  begin lz8 = v[7] ? 3'd0 : v[6] ? 3'd1 : v[5] ? 3'd2 : v[4] ? 3'd3 : v[3] ? 3'd4 : v[2] ? 3'd5 : v[1] ? 3'd6 : 3'd7; end
endfunction
function [6:0] sat7; input signed [19:0] v;                          // 0 .. 127
  begin sat7 = v[19] ? 7'd0 : (|v[18:7]) ? 7'd127 : v[6:0]; end
endfunction
function [1:0] cls80; input [79:0] v;    // the class of an extended value: NaN, infinity, zero or finite
  begin
    if (v[78:64] == 15'h7fff) cls80 = (v[62:0] != 63'h0) ? 2'd3 : 2'd2;
    else if (v[63:0] == 64'h0) cls80 = 2'd1;
    else cls80 = 2'd0;
  end
endfunction
function [3:0] ccof; input [79:0] v;     // N Z I NAN of a value
  reg nn, ii;
  begin
    nn = (v[78:64] == 15'h7fff) && (v[62:0] != 63'h0); ii = (v[78:64] == 15'h7fff) && (v[62:0] == 63'h0);
    ccof = {v[79], !nn && !ii && (v[63:0] == 64'h0), ii, nn};
  end
endfunction

// ---- Source from memory, Dn or #imm (unpacked from the staging words) or FPm ------------------------------------------
wire        rmf  = cw[14] && (cw[15:13] != 3'd3);   // the source is in the staging words (not an FP register)

// ---- Unpack of the source from the staging words -------------------------------------------------------------------
// Every format becomes the raw 80-bit extended form; the normalizer then does the rest.
//   integer (L, W, B): the magnitude at mantissa bits 63-32 with exponent $401E (= $3FFF + 31), so the value is right
//     before normalization;
//   single: exponent + $3F80 (rebias from 127 to $3FFF), the hidden integer bit made explicit; exponent $FF is
//     infinity or NaN, exponent 0 a denormal (exponent $3F81, integer bit 0);
//   double: the same with $3C00 (rebias from 1023), $7FF and $3C01;
//   extended: the 96-bit memory form, whose 16 bits between the exponent and the mantissa are unused.
wire [2:0]  sf  = cw[12:10];             // the source format (0 L, 1 S, 2 X, 4 W, 5 D, 6 B)
reg  [79:0] unp;                         // raw extended form (integers: magnitude at m[63:32], exponent 0x3FFF + 31)
reg  [31:0] iv, imag;                    // the integer sign-extended to 32 bits; its magnitude
always @* begin
  iv = (sf == 3'd4) ? {{16{s0[15]}}, s0[15:0]} : (sf == 3'd6) ? {{24{s0[7]}}, s0[7:0]} : s0;
  imag = iv[31] ? (~iv + 32'd1) : iv;
  unp = 80'h0;
  case (sf)
    3'd0, 3'd4, 3'd6: unp = {iv[31], 15'h401e, imag, 32'h0};      // zero: class zero (results never use its exponent)
    3'd1: unp = (s0[30:23] == 8'hff) ? {s0[31], 15'h7fff, 1'b0, s0[22:0], 40'h0} :
                (s0[30:23] == 8'h00) ? {s0[31], 15'h3f81, 1'b0, s0[22:0], 40'h0} :
                {s0[31], 15'h3f80 + {7'h0, s0[30:23]}, 1'b1, s0[22:0], 40'h0};
    3'd5: unp = (s0[30:20] == 11'h7ff) ? {s0[31], 15'h7fff, 1'b0, s0[19:0], s1, 11'h0} :
                (s0[30:20] == 11'h000) ? {s0[31], 15'h3c01, 1'b0, s0[19:0], s1, 11'h0} :
                {s0[31], 15'h3c00 + {4'h0, s0[30:20]}, 1'b1, s0[19:0], s1, 11'h0};
    default: unp = {s0[31:16], s1, s2};   // .X
  endcase
end

// ---- FMOVECR constant ROM (offset -> inexact class, exponent, mantissa; class 1 = {0,-1,-1,0}, 2 = {0,0,0,+1}) -------------
// FMOVECR loads one of the 68881's built-in constants (pi at 0, log10(2) at $0B, e at $0C, ..., 1 at $32, 10 at $33, up to
// 10^4096 at $3F). Each is stored with 64 mantissa bits rounded to nearest, plus a class that says how to correct the
// last bit for the other rounding modes: class 1 means the true value is a little below the stored one (subtract 1 in
// toward-zero and toward-minus modes), class 2 a little above (add 1 toward plus), class 0 exact.
function [80:0] crom; input [6:0] o;
  begin
    case (o)
      7'h00: crom = {2'd1, 15'h4000, 64'hc90fdaa22168c235}; 7'h0b: crom = {2'd2, 15'h3ffd, 64'h9a209a84fbcff798};
      7'h0c: crom = {2'd2, 15'h4000, 64'hadf85458a2bb4a9a}; 7'h0d: crom = {2'd1, 15'h3fff, 64'hb8aa3b295c17f0bc};
      7'h0e: crom = {2'd0, 15'h3ffd, 64'hde5bd8a937287195}; 7'h0f: crom = {2'd0, 15'h0000, 64'h0};
      7'h30: crom = {2'd1, 15'h3ffe, 64'hb17217f7d1cf79ac}; 7'h31: crom = {2'd1, 15'h4000, 64'h935d8dddaaa8ac17};
      7'h32: crom = {2'd0, 15'h3fff, 64'h8000000000000000}; 7'h33: crom = {2'd0, 15'h4002, 64'ha000000000000000};
      7'h34: crom = {2'd0, 15'h4005, 64'hc800000000000000}; 7'h35: crom = {2'd0, 15'h400c, 64'h9c40000000000000};
      7'h36: crom = {2'd0, 15'h4019, 64'hbebc200000000000}; 7'h37: crom = {2'd0, 15'h4034, 64'h8e1bc9bf04000000};
      7'h38: crom = {2'd1, 15'h4069, 64'h9dc5ada82b70b59e}; 7'h39: crom = {2'd2, 15'h40d3, 64'hc2781f49ffcfa6d5};
      7'h3a: crom = {2'd1, 15'h41a8, 64'h93ba47c980e98ce0}; 7'h3b: crom = {2'd1, 15'h4351, 64'haa7eebfb9df9de8e};
      7'h3c: crom = {2'd1, 15'h46a3, 64'he319a0aea60e91c7}; 7'h3d: crom = {2'd2, 15'h4d48, 64'hc976758681750c17};
      7'h3e: crom = {2'd1, 15'h5a92, 64'h9e8b3b5dc53d5de5}; 7'h3f: crom = {2'd1, 15'h7525, 64'hc46052028a20979b};
      // undefined offsets (class 3), the 68881/68882 ROM contents as WinUAE has them; 0x10-0x2F all read entry 0
      7'h01: crom = {2'd3, 15'h4001, 64'hfe00068200000000}; 7'h02: crom = {2'd3, 15'h4001, 64'hffc0050380000000};
      7'h03: crom = {2'd3, 15'h2000, 64'h7fffffff00000000}; 7'h04: crom = {2'd3, 15'h0000, 64'hffffffffffffffff};
      7'h05: crom = {2'd3, 15'h3c00, 64'hfffffffffffff800}; 7'h06: crom = {2'd3, 15'h3f80, 64'hffffff0000000000};
      7'h07: crom = {2'd3, 15'h0001, 64'hf65d8d9c00000000}; 7'h08: crom = {2'd3, 15'h7fff, 64'h001e000000000000};
      7'h09: crom = {2'd3, 15'h43ff, 64'h000e000000000000}; 7'h0a: crom = {2'd3, 15'h407f, 64'h0006000000000000};
      default: crom = {2'd3, 15'h4000, 64'h0};
    endcase
  end
endfunction
// The ROM is read into a register at every step, so synthesis can map it to block RAM (the attribute asks for that).
reg  [80:0] crv /* synthesis syn_romstyle = "block_rom" */;   // read every step from the command input (valid from S_UNP0)
always @(posedge clk) if (ce) crv <= crom(c_cw[6:0]);
wire        crnr = (cw[6:0] == 7'h0f) || (cw[6:0] == 7'h08) || (cw[6:4] == 3'b001) || (cw[6:4] == 3'b010);   // zero mantissa or 0x7FFF
reg  [1:0]  cr_adj, cr_cc;               // FMOVECR undefined offsets: mantissa word 1 +-0x80; extra I, NAN
// the last-bit correction of the class for the rounding mode
wire [7:0]  crl = crv[7:0] - {7'h0, crv[80:79] == 2'd1 && (md == 2'd1 || md == 2'd2)} + {7'h0, crv[80:79] == 2'd2 && md == 2'd3};
wire [63:0] crm = {crv[63:8], crl};                                  // the mode adjustment never carries out of the low byte

// ---- Load bus into W: the source (S_UNP0: unpacked, FPm, or the FMOVECR constant) or the destination (S_NB) ------------
wire [79:0] lb  = (st == S_NB) ? avr : (k == K_CR) ? {1'b0, crv[78:64], crm} : rmf ? unp : fb[95:16];
wire [1:0]  wcls = cls80({ws, we[14:0], wv[66:3]});                  // the class of what was loaded into W

// ---- Shifters: steps of W -------------------------------------------------------------------------------------------
wire        wz   = (wv[66:1] == 66'h0);                              // no data bits (bit 0 is sticky)
wire        t32z = (wv[66:35] == 32'h0), t8z = (wv[66:59] == 8'h0);  // the top 32 / top 8 bits are zero
wire [2:0]  lzb  = lz8(wv[66:59]);                                   // leading zeros of the top byte
wire [2:0]  fk   = (st == S_SQRT) ? 3'd2 : lzb;                      // left 1-7 (square root: 2)
wire [66:0] fls  = wv << fk;                                         // the fine left shift
wire [2:0]  rk   = (st == S_NZ1) ? 3'd1 : shn[2:0];                  // right 1-7 with sticky (carry out: 1, top bit set)
wire [66:0] rsh  = wv >> rk;
wire        rstk = |(wv & ~(67'h7FFFFFFFFFFFFFFFF << rk));           // the bits shifted out
wire [66:0] frs  = {rsh[66] | (st == S_NZ1), rsh[65:1], rsh[0] | rstk};   // the fine right shift (an add's carry comes in at the top)
// the adder: A's mantissa plus or minus W (the smaller operand, aligned)
wire [67:0] t    = sub_e ? {1'b0, am, 3'b0} - {1'b0, wv} : {1'b0, am, 3'b0} + {1'b0, wv};   // the adder
wire [20:0] dex  = {ae[19], ae} - {be[19], be};                      // S_PRE: exponent difference
wire        meq  = (am == bm);                                       // equal mantissas
wire signed [19:0] ueb = be - 20'sh03FFF;                            // B's unbiased exponent

// ---- Multiplier: one 16 x 16 product per step (DSP), summed by columns; digits shift into acc, the low 3 into sticky ---
// The two 64-bit mantissas are four 16-bit digits each, so the full product is the sum of 16 digit products. They are
// formed in column order (all products of weight 2^0, then 2^16, and so on), one per step in the single DSP, and added
// into the column sum cs; when a column is complete its low 16 bits are pushed into acc and the rest carries on into the
// next column. The low 48 bits of the 128-bit product only matter as sticky.
reg  [31:0] p;                           // the digit product of this step (the DSP's output register)
reg  [35:0] cs;                          // column sum
reg  [79:0] acc;                         // product bits 127..48; divide / square root: quotient bits from acc[14] up
reg         msti, smk;                   // sticky of the product's low 48 bits; FSGLMUL: low 40 bits of A and B are 0
// the schedule of the 16 products in column order
function [4:0] msch; input [3:0] j;      // product j: {A digit, B digit, first of its column}
  begin
    case (j)
      4'd0: msch = 5'b00001; 4'd1: msch = 5'b00011; 4'd2: msch = 5'b01000; 4'd3: msch = 5'b00101;
      4'd4: msch = 5'b01010; 4'd5: msch = 5'b10000; 4'd6: msch = 5'b00111; 4'd7: msch = 5'b01100;
      4'd8: msch = 5'b10010; 4'd9: msch = 5'b11000; 4'd10: msch = 5'b01111; 4'd11: msch = 5'b10100;
      4'd12: msch = 5'b11010; 4'd13: msch = 5'b10111; 4'd14: msch = 5'b11100; default: msch = 5'b11111;
    endcase
  end
endfunction
// digit i of a mantissa; FSGLMUL (m) uses single precision operands, so their low 40 bits count as zero
function [15:0] dgt; input [63:0] v; input [1:0] i; input m;
  begin
    case (i)
      2'd0: dgt = m ? 16'h0 : v[15:0];
      2'd1: dgt = m ? 16'h0 : v[31:16];
      2'd2: dgt = m ? {v[47:40], 8'h0} : v[47:32];
      default: dgt = v[63:48];
    endcase
  end
endfunction
wire [4:0]  ms_p = msch(cnt[3:0]);                                   // product started at this step (cnt 0 .. 15)
wire [3:0]  mj   = cnt[3:0] - 4'd1;
wire [4:0]  ms_a = msch(mj);                                         // product summed at this step (cnt 1 .. 16)
wire        mpush = (cnt >= 7'd1 && cnt <= 7'd16 && ms_a[0] && mj != 4'd0) || cnt == 7'd17 || cnt == 7'd18;   // a column is complete
always @(posedge clk) if (ce) p <= dgt(am, ms_p[4:3], smk) * dgt(bm, ms_p[2:1], smk);

// ---- Divider / square root: one subtractor; quotient bits in acc, the square root's radicand bits in wv[66:1] -------
// Divide (radix 2, restoring): each step compares the partial remainder dr with the divisor (B's mantissa); when it is
// not smaller, the divisor is subtracted and the quotient bit is 1. The remainder then doubles for the next bit. Square
// root works digit by digit the same way: the trial value is the root so far with "01" appended (that is 4 * root + 1),
// compared with the remainder extended by the next two radicand bits. Both share the one subtractor dd.
reg  [69:0] dr;                          // the partial remainder
wire [69:0] dt  = sq ? {2'b00, acc[79:14], 2'b01} : {6'h0, bm};     // what is subtracted: 4 * root + 1, or the divisor
wire [69:0] drn = sq ? {dr[67:0], wv[66:65]} : dr;                   // square root: the remainder with two more radicand bits
wire [70:0] dd  = {1'b0, drn} - {1'b0, dt};
wire        dge = !dd[70];                                           // it fits: the next quotient / root bit is 1
wire        mlo = dd[70];                                            // S_PRE: dr = A's mantissa, so dd = A - B (borrow: A < B)
wire [66:0] accq = {acc[79:14], |acc[13:0] | msti | (sqd && dr != 70'h0)};   // the result from acc, with the sticky bit

// ---- Rounder increment stage inputs (from wv after the denormalizing shift) -------------------------------------------
wire [63:0] rlow = (rp == 2'd1) ? 64'h000000FFFFFFFFFF : (rp == 2'd2) ? 64'h00000000000007FF : 64'h0;   // the bits below the precision
wire [63:0] sm   = wv[66:3];                                         // the mantissa
wire        rb   = (rp == 2'd1) ? sm[39] : (rp == 2'd2) ? sm[10] : wv[2];   // the guard bit
wire        rst_ = (rp == 2'd1) ? (|sm[38:0] | |wv[2:0]) : (rp == 2'd2) ? (|sm[9:0] | |wv[2:0]) : (|wv[1:0]);   // the sticky bit
wire        lsb  = (rp == 2'd1) ? sm[40] : (rp == 2'd2) ? sm[11] : sm[0];   // the last kept bit
// round up? nearest: more than half, or exactly half and odd; toward zero: never; toward -inf / +inf: when anything was
// lost and the sign points that way
wire        rup  = (md == 2'd0) ? (rb && (rst_ || lsb)) : (md == 2'd1) ? 1'b0 : (md == 2'd2) ? (ws && (rb || rst_)) : (!ws && (rb || rst_));
wire [64:0] rsum = {1'b0, sm & ~rlow} + (rupr ? {1'b0, rlow} + 65'd1 : 65'd0);   // clear the lost bits, add one last bit
// the exponent range of the precision (single and double use their own limits unless the extended range rx applies)
wire signed [19:0] emin = (rx || rp == 2'd0) ? 20'sd0 : (rp == 2'd1) ? 20'sh03F81 : 20'sh03C01;
wire signed [19:0] emax = (rx || rp == 2'd0) ? 20'sh07FFE : (rp == 2'd1) ? 20'sh0407E : 20'sh043FE;
wire        ovr  = !tiny && (rcy ? og1 : og0);                       // overflow after rounding
wire        omax = (md == 2'd1) || (ws && md == 2'd3) || (!ws && md == 2'd2);   // an overflow gives the largest finite number

// ---- Round to integer (FINT, FINTRZ, FMOVE to integer) -----------------------------------------------------------------
wire [31:0] imx  = (of == 3'd0) ? 32'h7fffffff : (of == 3'd4) ? 32'h00007fff : 32'h0000007f;   // the largest integer of the format

// ---- Special values (S_SPC) ----------------------------------------------------------------------------------------------
wire        anan = (ac == 2'd3), bnan = (bc == 2'd3), ainf = (ac == 2'd2), binf = (bc == 2'd2), az = (ac == 2'd1), bz = (bc == 2'd1);
wire        afin = (ac == 2'd0), bfin = (bc == 2'd0);
wire        asn  = anan && !am[62], bsn = bnan && !bm[62];            // signaling NaNs
wire [79:0] qb   = {bs_, 15'h7fff, bm | 64'h4000000000000000};       // B made quiet
wire        bse  = bs_ ^ (k == K_SUB);                               // B's effective sign (FSUB adds -B)
wire        zs   = as_ ^ bs_;                                        // the sign of a product or quotient
wire        bs2  = (k == K_ABS) ? 1'b0 : (k == K_NEG) ? !bs_ : bs_;   // sign of FMOVE / FABS / FNEG
// FCMP (WinUAE's floatx80_cmp): magnitude order zero < finite < infinity
wire        agt  = (ainf && !binf) || (afin && bz) || (afin && bfin && f_mgt);   // |A| > |B|
wire        alt  = (binf && !ainf) || (bfin && az) || (afin && bfin && f_mlt);   // |A| < |B|

wire [14:0] rxe = res[78:64];            // the result's exponent (for FMOVE out)

// ---- Engine -----------------------------------------------------------------------------------------------------------
// next-value codes of the wide registers (blocking, set per step below)
localparam W_HOLD = 4'd0, W_LB = 4'd1, W_AM = 4'd2, W_BM = 4'd3, W_ADD = 4'd4, W_L32 = 4'd5, W_L8 = 4'd6, W_FL = 4'd7,
           W_R32 = 4'd8, W_R8 = 4'd9, W_FR = 4'd10, W_ACC = 4'd11, W_RS = 4'd12, W_GE = 4'd13, W_Z = 4'd14;
localparam E_HOLD = 4'd0, E_LB = 4'd1, E_ADD = 4'd2, E_BE = 4'd3, E_AE = 4'd4, E_Z = 4'd5, E_7FFF = 4'd6, E_3FFF = 4'd7,
           E_OP = 4'd8, E_403E = 4'd9, E_SCL = 4'd10, E_EMIN = 4'd11;
localparam R_HOLD = 2'd0, R_WR = 2'd1, R_RO = 2'd2, R_ADJ = 2'd3;
localparam D_HOLD = 3'd0, D_LD = 3'd1, D_CLR = 3'd2, D_DIV = 3'd3, D_SQ = 3'd4;
localparam A_HOLD = 2'd0, A_CLR = 2'd1, A_PUSH = 2'd2, A_SHQ = 2'd3;
localparam M_HOLD = 2'd0, M_W = 2'd1, M_B = 2'd2;
// The state machine. Each state sets the next-value codes (blocking variables), and the wide registers are written once
// at the end of the block from those codes. That way each wide register has one short multiplexer instead of one input
// per state, which is where most of the size saving of the small engine came from.
always @(posedge clk) begin : eng
  reg [3:0]  wo, eo;                     // next value of wv, of we
  reg [1:0]  ro, aop, mo;                // next value of res, of acc, of am
  reg [2:0]  dop;                        // next value of dr
  reg        eng, dgo;                   // E_ADD subtracts edv (negative step); the divider keeps its subtraction
  reg [16:0] edv;                        // the exponent step of E_ADD
  wo = W_HOLD; eo = E_HOLD; ro = R_HOLD; aop = A_HOLD; mo = M_HOLD; dop = D_HOLD; eng = 1'b0; dgo = 1'b0; edv = 17'h0;
  if (rst) begin st <= S_IDLE; dn <= 1'b0; end
  else if (ce) begin
    case (st)
    S_IDLE: ;
    S_UNP0: begin wo = W_LB; eo = E_LB; ws <= lb[79]; st <= S_UNP; end     // B raw into W
    S_UNP: begin                         // classify both operands, reset the flags, normalize B if finite
      bs_ <= ws; bc <= wcls; as_ <= avr[79]; ac <= cls80(avr);
      sq <= 1'b0; sqd <= 1'b0; qn <= 1'b0; ones <= 1'b0;
      fl <= 8'h0; cc_own <= 1'b0; rx <= 1'b0; tiny <= 1'b0; cy <= 1'b0; cr_adj <= 2'd0; cr_cc <= 2'd0;
      rp <= (pp == 2'd0) ? 2'd0 : (pp == 2'd1) ? 2'd1 : 2'd2;
      if (wcls == 2'd0 && k != K_CR) begin nz_ret <= S_NB; st <= S_NRM; end else st <= S_NB;
    end
    S_NB: begin                          // B normalized (finite only); A raw into W
      bm <= wv[66:3]; be <= we; wo = W_LB; eo = E_LB;
      if (ac == 2'd0 && dya) begin nz_ret <= S_NA; st <= S_NRM; end else st <= S_NA;
    end
    S_NA: begin mo = M_W; ae <= we; dop = D_LD; st <= S_PRE; end      // the compare in S_PRE uses the divider's subtractor
    S_PRE: begin                         // every comparison the operation step selects on (one subtraction each)
      f_mgt <= !dex[20] && (dex[19:0] != 20'h0 || (!mlo && !meq));   // |A| > |B|
      f_mlt <= dex[20] || (dex[19:0] == 20'h0 && mlo);               // |A| < |B|
      f_mlo <= mlo;                                                  // mantissa A < B
      f_shab <= dex[20] ? sat7(20'h0 - dex[19:0]) : sat7(dex[19:0]);  // the alignment shift of an add
      f_ge3e <= !ueb[19] && (ueb[18:6] != 13'h0 || ueb[5:0] == 6'h3f);                // exponent >= 0x403E (unbiased >= 63)
      f_lt3fff <= ueb[19];                                           // |B| < 1
      f_gt400f <= !ueb[19] && (ueb[18:5] != 14'h0 || ueb[4:0] > 5'd16);   // |B| >= 2^17 (FSCALE: the result surely over/underflows)
      f_eq3fff <= (ueb == 20'h0); f_blt0 <= be[19];
      f_shi <= sat7(20'sd63 - ueb);                                  // right shift that leaves the integer part
      f_ue <= ueb; f_uabs <= ueb[19] ? 20'h0 - ueb : ueb;
      // result exponent: multiply ae + be - $3FFE; divide ae - be + $3FFF (+1 when A's mantissa is not below B's);
      // square root (B's unbiased exponent) / 2 + $3FFF
      e_op <= ((k == K_SQRT) ? 20'h0 : ae) + ((k == K_SQRT) ? {ueb[19], ueb[19:1]} : (k == K_MUL || k == K_SMUL) ? be : ~be) +
              ((k == K_SQRT) ? 20'h03FFF : (k == K_MUL || k == K_SMUL) ? 20'hFC002 : mlo ? 20'h03FFF : 20'h04000);   // -be = ~be + 1
      st <= S_SPC;
    end
    S_SPC: begin                         // special values, and the start of each operation
      ws <= bs_; eo = E_BE; wo = W_BM;   // W = B; special results are written from W by S_WR
      st <= S_WR;
      case (k)
      K_MOV, K_ABS, K_NEG: begin
        if (bnan) begin qn <= 1'b1; fl[6] <= bsn; end
        else begin
          ws <= bs2;
          if (bz) eo = E_Z;
          else if (!binf) st <= S_RT;    // a finite value is rounded to the precision
        end
      end
      K_INT, K_INTRZ: begin              // shift right so the integer part ends at the guard position, then round
        zsg <= bs_;
        if (bnan) begin qn <= 1'b1; fl[6] <= bsn; end
        else if (bz) eo = E_Z;
        else if (!binf && !f_ge3e) begin shn <= f_shi; rp <= 2'd0; sh_ret <= S_RD; rd_ret <= S_INT; st <= S_SHR; end
      end
      K_SQRT: begin
        if (bnan) begin qn <= 1'b1; fl[6] <= bsn; end
        else if (bz) eo = E_Z;
        else if (bs_) begin ws <= 1'b0; eo = E_7FFF; ones <= 1'b1; fl[5] <= 1'b1; end   // negative: default NaN, OPERR
        else if (!binf) begin
          // X = m (exponent even) or 2m (odd), X << 1 in wv[66:1]: the first pair is X bits 64, 63
          eo = E_OP; sq <= 1'b1; sqd <= 1'b1; dop = D_CLR; aop = A_CLR; cnt <= 7'd0; msti <= 1'b0;
          if (!f_ue[0]) begin shn <= 7'd1; sh_ret <= S_SQRT; st <= S_SHR; end else st <= S_SQRT;
        end
      end
      K_GETE: begin                      // FGETEXP: the unbiased exponent as a floating-point number
        if (bnan) begin qn <= 1'b1; fl[6] <= bsn; end
        else if (binf) begin ws <= 1'b0; eo = E_7FFF; ones <= 1'b1; fl[5] <= 1'b1; end
        else if (bz) eo = E_Z;
        else if (f_eq3fff) begin ws <= 1'b0; eo = E_Z; wo = W_Z; end
        else begin ws <= f_ue[19]; eo = E_403E; wo = W_GE; rp <= 2'd0; nz_ret <= S_RT; st <= S_NRM; end
      end
      K_GETM: begin                      // FGETMAN: the mantissa with the exponent set to 0 (a value in [1, 2))
        if (bnan) begin qn <= 1'b1; fl[6] <= bsn; end
        else if (binf) begin ws <= 1'b0; eo = E_7FFF; ones <= 1'b1; fl[5] <= 1'b1; end
        else if (bz) eo = E_Z;
        else eo = E_3FFF;
      end
      K_TST: begin                       // condition codes only
        if (bnan) begin qn <= 1'b1; fl[6] <= bsn; end
        else if (bfin && f_blt0) eo = E_Z;
      end
      K_CR: begin                        // W = the constant (taken as the source, not normalized)
        fl[1] <= (crv[80:79] == 2'd1 || crv[80:79] == 2'd2);
        // undefined offsets (WinUAE): single precision moves mantissa word 1 by 0x80 after rounding (offsets 1, 2, 7);
        // offsets 3 and 7 add I or NAN to the condition codes
        cr_adj <= (pp != 2'd1) ? 2'd0 : (cw[6:0] == 7'h01 || cw[6:0] == 7'h07) ? ((md == 2'd0) ? 2'd3 : (md == 2'd3) ? 2'd0 : 2'd1) :
                  (cw[6:0] == 7'h02 && md == 2'd3) ? 2'd3 : 2'd0;
        cr_cc <= (cw[6:0] == 7'h03) ? ((pp == 2'd1 && (md == 2'd0 || md == 2'd3)) ? 2'b10 : 2'b01) : (cw[6:0] == 7'h07) ? 2'b01 : 2'b00;
        ws <= 1'b0;
        if (pp != 2'd0 && !crnr) begin rx <= 1'b1; st <= S_RT; end   // no rounding of a zero mantissa or exponent 0x7FFF
      end
      K_OUT: begin                       // FMOVE out
        if (oint) begin                                              // integer
          if (bnan) begin
            fl[6] <= bsn; fl[5] <= !bsn || of == 3'd4;
            o0 <= (of == 3'd0) ? qb[63:32] : (of == 3'd4) ? {16'h0, qb[63:48]} : {24'h0, qb[63:56]};
          end else if (binf || (bfin && f_ge3e)) begin               // out of range: the largest integer, OPERR
            fl[5] <= 1'b1; o0 <= bs_ ? ~imx : imx;
          end else if (bz) o0 <= 32'h0;
          else begin shn <= f_shi; rp <= 2'd0; sh_ret <= S_RD; rd_ret <= S_INT; st <= S_SHR; end
        end else if (bnan) begin qn <= 1'b1; fl[6] <= bsn; end
        else if (bz) eo = E_Z;
        else if (binf) ;
        else if (of == 3'd2) begin                                   // .X: denormals rounded to the FPCR precision
          if (f_blt0) st <= S_RT;
        end else begin rp <= (of == 3'd1) ? 2'd1 : 2'd2; st <= S_RT; end   // .S / .D: round to that precision
      end
      default: begin                                                 // dyadic
        if (anan || bnan) begin                                      // a NaN operand: the NaN (A's first), made quiet
          qn <= 1'b1; fl[6] <= asn || bsn;
          if (anan) begin ws <= as_; eo = E_AE; wo = W_AM; end
          if (k == K_CMP) ws <= 1'b0;
        end else case (k)
        K_ADD, K_SUB: begin
          if (ainf && binf) begin                                    // inf - inf: default NaN, OPERR
            if (as_ != bse) begin ws <= 1'b0; eo = E_7FFF; ones <= 1'b1; fl[5] <= 1'b1; end else ws <= as_;
          end
          else if (ainf) begin ws <= as_; eo = E_AE; wo = W_AM; end
          else if (binf) ws <= bse;
          else if (az && bz) begin ws <= (as_ == bse) ? as_ : (md == 2'd2); eo = E_Z; end   // -0 only toward -inf
          else if (az) begin ws <= bse; st <= S_RT; end
          else if (bz) begin ws <= as_; eo = E_AE; wo = W_AM; st <= S_RT; end
          else st <= S_ADD0;
        end
        K_MUL, K_SMUL: begin
          if (ainf) begin                                            // inf * 0: default NaN, OPERR
            if (bz) begin ws <= 1'b0; eo = E_7FFF; ones <= 1'b1; fl[5] <= 1'b1; end else begin ws <= zs; eo = E_AE; wo = W_AM; end
          end
          else if (binf) begin
            if (az) begin ws <= 1'b0; eo = E_7FFF; ones <= 1'b1; fl[5] <= 1'b1; end else ws <= zs;
          end
          else if (az || bz) begin ws <= zs; eo = E_Z; wo = W_Z; end
          else begin
            if (k == K_SMUL) begin rp <= 2'd1; rx <= 1'b1; end      // FSGLMUL: single precision, extended range
            smk <= (k == K_SMUL);
            ws <= zs; eo = E_OP; cnt <= 7'd0; aop = A_CLR; msti <= 1'b0; cs <= 36'h0;
            st <= S_MUL;
          end
        end
        K_DIV, K_SDIV: begin
          if (ainf) begin                                            // inf / inf: default NaN, OPERR
            if (binf) begin ws <= 1'b0; eo = E_7FFF; ones <= 1'b1; fl[5] <= 1'b1; end else begin ws <= zs; eo = E_AE; wo = W_AM; end
          end
          else if (binf) begin ws <= zs; eo = E_Z; wo = W_Z; end
          else if (bz) begin
            if (az) begin ws <= 1'b0; eo = E_7FFF; ones <= 1'b1; fl[5] <= 1'b1; end   // 0 / 0: OPERR
            else begin ws <= zs; eo = E_7FFF; wo = W_Z; fl[2] <= 1'b1; end           // x / 0: infinity, DZ
          end
          else if (az) begin ws <= zs; eo = E_Z; wo = W_Z; end
          else begin
            ws <= zs; eo = E_OP; aop = A_CLR; cnt <= 7'd0; msti <= 1'b0; sqd <= 1'b1;
            if (f_mlo) dop = D_DIV;                                  // dr = 2 x A (no quotient bit: dgo is 0)
            if (k == K_SDIV) begin rp <= 2'd1; rx <= 1'b1; end
            st <= S_DIV;
          end
        end
        K_SCALE: begin                   // FSCALE: A * 2^(integer part of B), an exponent add
          if (binf) begin ws <= 1'b0; eo = E_7FFF; ones <= 1'b1; fl[5] <= 1'b1; end
          else if (ainf) begin ws <= as_; eo = E_AE; wo = W_AM; end
          else if (az) begin ws <= as_; eo = E_Z; wo = W_Z; end
          else begin
            ws <= as_; eo = E_AE;
            if (bz || f_lt3fff) begin wo = W_AM; st <= S_RT; end
            else if (f_gt400f) begin wo = W_AM; eo = E_SCL; st <= S_RT; end
            else begin shn <= f_shi; sh_ret <= S_SCL; st <= S_SHR; end   // wv = {bm, 000}: integer part of the source
          end
        end
        default: begin                                               // K_CMP
          cc_own <= 1'b1;
          if (agt) ccr_o <= {as_, 3'b000};
          else if (alt) ccr_o <= {!bs_, 3'b000};
          else if (ainf) ccr_o <= (as_ == bs_) ? {as_, 3'b100} : {as_, 3'b000};
          else if (az) ccr_o <= {as_, 3'b100};
          else if (as_ == bs_) ccr_o <= 4'b0100;
          else ccr_o <= {as_, 3'b000};
        end
        endcase
      end
      endcase
    end
    // ---- add / subtract: the larger magnitude into am, the smaller through the shifter (wv) -------------------------
    S_ADD0: begin
      sub_e <= (as_ != bse); zsg <= (md == 2'd2);                    // different signs subtract; an exact 0 is -0 only toward -inf
      if (f_mlt) begin mo = M_B; ws <= bse; wo = W_AM; end           // |B| > |A| (we = be from S_SPC)
      else begin ws <= as_; eo = E_AE; end                           // wv = {bm, 000} from S_SPC
      shn <= f_shab; sh_ret <= S_ADD1; st <= S_SHR;                  // align the smaller one by the exponent difference
    end
    S_ADD1: begin wo = W_ADD; cy <= t[67] && !sub_e; st <= S_NZ1; end
    S_NZ1: begin                                                     // a carry out: right by one; else normalize
      if (cy) begin wo = W_FR; eo = E_ADD; edv = 17'd1; cy <= 1'b0; st <= S_RT; end
      else begin nz_ret <= S_RT; st <= S_NRM; end
    end
    // ---- shared steps -------------------------------------------------------------------------------------------------
    S_NRM: begin                                                     // normalize W; no data bits: the result is zero
      if (wv[66]) st <= nz_ret;
      else if (wz) begin ws <= zsg; eo = E_Z; st <= S_WR; end
      else if (t32z) begin wo = W_L32; eo = E_ADD; eng = 1'b1; edv = 17'd32; end
      else if (t8z) begin wo = W_L8; eo = E_ADD; eng = 1'b1; edv = 17'd8; end
      else begin wo = W_FL; eo = E_ADD; eng = 1'b1; edv = {14'h0, lzb}; st <= nz_ret; end
    end
    S_SHR: begin                                                     // right by shn with sticky
      if (shn == 7'd0 || wz) st <= sh_ret;
      else if (shn[6:5] != 2'd0) begin wo = W_R32; shn <= shn - 7'd32; end
      else if (shn[4:3] != 2'd0) begin wo = W_R8; shn <= shn - 7'd8; end
      else begin wo = W_FR; shn <= 7'd0; st <= sh_ret; end
    end
    // ---- multiply: product j starts at cnt j and is summed at cnt j + 1; digits 6 and 7 at cnt 17, 18 --------------------
    S_MUL: begin
      cnt <= cnt + 7'd1;
      if (cnt != 7'd0) cs <= (mpush ? {16'h0, cs[35:16]} : cs) + ((cnt <= 7'd16) ? {4'h0, p} : 36'h0);
      if (mpush) begin aop = A_PUSH; msti <= msti | (acc[15:0] != 16'h0); end   // the digit leaving acc's bottom goes to sticky
      if (cnt == 7'd18) st <= S_MUL2;
    end
    S_MUL2: begin                                                    // product bits 127..48 in acc, 47..0 in sticky
      // the product of two values in [1, 2) is in [1, 4): if its top bit is 0, shift left once (exponent - 1)
      if (!acc[79]) begin aop = A_SHQ; eo = E_ADD; eng = 1'b1; edv = 17'd1; end
      else begin wo = W_ACC; st <= S_RT; end
    end
    // ---- divide: 66 quotient bits (the first is the integer bit), remainder into sticky -----------------------------
    S_DIV: begin
      cnt <= cnt + 7'd1; aop = A_SHQ; dop = D_DIV; dgo = dge;
      if (cnt == 7'd65) st <= S_LDQ;
    end
    // ---- square root: 66 root bits ---------------------------------------------------------------------------------
    S_SQRT: begin
      cnt <= cnt + 7'd1; aop = A_SHQ; dop = D_SQ; wo = W_FL;         // W shifts the next radicand pair up (fk = 2)
      if (cnt == 7'd65) st <= S_LDQ;
    end
    S_LDQ: begin wo = W_ACC; sq <= 1'b0; sqd <= 1'b0; st <= S_RT; end  // the remainder as sticky
    // ---- rounder -----------------------------------------------------------------------------------------------------
    S_RT: begin dlt <= emin - we; st <= S_RT2; end
    S_RT2: begin                                                     // tiny: we < emin
      tiny <= !dlt[19] && (dlt != 20'h0);
      shn <= sat7(dlt);                                              // denormalize by emin - we
      if (!dlt[19] && (dlt != 20'h0)) eo = E_EMIN;
      sh_ret <= S_RD; rd_ret <= S_RI; st <= S_SHR;
    end
    S_RD: begin rupr <= rup; rinx <= rb || rst_; we1 <= we + 20'sd1; st <= rd_ret; end   // round-up decision, inexact
    S_RI: begin                                                      // the carry out and the overflow tests as registers
      wo = W_RS; rcy <= rsum[64];
      og0 <= (we > emax); og1 <= (we1 > emax);
      st <= S_RO;
    end
    S_RO: begin
      fl[1] <= fl[1] | rinx;                                         // INEX2
      if (ovr) fl[4] <= 1'b1; else fl[3] <= fl[3] | tiny;            // OVFL, or UNFL when tiny
      ro = R_RO; st <= S_FIN;
    end
    // ---- round to integer ---------------------------------------------------------------------------------------------
    S_INT: begin                                                     // rinx, rupr from S_RD
      wo = W_RS;
      if (k == K_OUT) st <= S_ISAT;                                  // FMOVE to integer: range next
      else begin                                                     // FINT, FINTRZ: normalize (zero: no data bits)
        fl[1] <= rinx;
        ws <= bs_; eo = E_403E; rp <= 2'd0; nz_ret <= S_RT; st <= S_NRM;
      end
    end
    S_ISAT: begin                        // the rounded integer does not fit the format (a negative one may be one larger)
      ovf <= (wv[66:35] != 32'h0) || (wv[34:3] > imx + {31'h0, bs_});
      st <= S_ISAT2;
    end
    S_ISAT2: begin
      if (ovf) begin fl[5] <= 1'b1; o0 <= bs_ ? ~imx : imx; end      // OPERR and the saturated integer
      else begin fl[1] <= rinx; o0 <= bs_ ? (32'h0 - wv[34:3]) : wv[34:3]; end
      st <= S_DONE;
    end
    S_SCL: begin                                                     // FSCALE: count = integer part of the source (we = ae)
      eo = E_ADD; eng = bs_; edv = wv[19:3]; wo = W_AM;
      st <= S_RT;
    end
    S_WR: begin ro = R_WR; st <= S_FIN; end                          // result from W
    S_FIN: begin                                                     // FMOVE out of .S/.D/.X: pack res
      if (k == K_CR && cr_adj != 2'd0) ro = R_ADJ;
      if (k != K_OUT || oint) ;
      else if (of == 3'd1) begin                                     // single: rebias by -$3F80, drop the integer bit
        if (rxe == 15'h7fff) o0 <= {res[79], 8'hff, (res[62:0] != 63'h0) ? {1'b1, res[61:40]} : 23'h0};
        else if (res[63:0] == 64'h0) o0 <= {res[79], 31'h0};
        else if (!res[63]) o0 <= {res[79], 8'h00, res[62:40]};       // a denormal
        else o0 <= {res[79], rxe[7:0] - 8'h80, res[62:40]};
      end else if (of == 3'd5) begin                                 // double: rebias by -$3C00
        if (rxe == 15'h7fff) {o0, o1} <= {res[79], 11'h7ff, (res[62:0] != 63'h0) ? {1'b1, res[61:11]} : 52'h0};
        else if (res[63:0] == 64'h0) {o0, o1} <= {res[79], 63'h0};
        else if (!res[63]) {o0, o1} <= {res[79], 11'h000, res[62:11]};
        else {o0, o1} <= {res[79], rxe[10:0] - 11'h400, res[62:11]};
      end else begin                                                 // extended: the 96-bit memory form
        o0 <= {res[79:64], 16'h0}; o1 <= res[63:32]; o2 <= res[31:0];
      end
      st <= S_DONE;
    end
    S_DONE: if (!dn) begin rcc <= cc_own ? ccr_o : (ccof(res) | {2'b00, cr_cc}); dn <= 1'b1; end
    default: st <= S_IDLE;
    endcase
    // ---- the wide registers ------------------------------------------------------------------------------------------
    case (wo)
      W_LB:  wv <= {lb[63:0], 3'b000};                              // load
      W_AM:  wv <= {am, 3'b000};
      W_BM:  wv <= {bm, 3'b000};
      W_ADD: wv <= t[66:0];                                         // the sum
      W_L32: wv <= {wv[34:0], 32'h0};                               // left by 32
      W_L8:  wv <= {wv[58:0], 8'h0};                                // left by 8
      W_FL:  wv <= fls;                                             // left by 1-7
      W_R32: wv <= {32'h0, wv[66:33], |wv[32:0]};                   // right by 32, with sticky
      W_R8:  wv <= {8'h0, wv[66:9], |wv[8:0]};                      // right by 8, with sticky
      W_FR:  wv <= frs;                                             // right by 1-7, with sticky
      W_ACC: wv <= accq;                                            // the product, quotient or root
      W_RS:  wv <= {rsum[63:0], 3'b000};                            // the rounded mantissa
      W_GE:  wv <= {44'h0, f_uabs, 3'b000};                         // FGETEXP's integer
      W_Z:   wv <= 67'h0;
      default: ;
    endcase
    case (eo)
      E_LB:   we <= {5'h0, lb[78:64]};
      E_ADD:  we <= we + ({3'h0, edv} ^ {20{eng}}) + {19'h0, eng};  // we + edv, or we - edv when eng
      E_BE:   we <= be;
      E_AE:   we <= ae;
      E_Z:    we <= 20'sh0;
      E_7FFF: we <= 20'sh07FFF;
      E_3FFF: we <= 20'sh03FFF;
      E_OP:   we <= e_op;
      E_403E: we <= 20'sh0403E;                                     // an integer at bits 63-0
      E_SCL:  we <= bs_ ? -20'sh06001 : 20'sh0E000;                 // FSCALE by a huge count: far out of range
      E_EMIN: we <= emin;
      default: ;
    endcase
    case (ro)
      // a special result from W (NaN quieted by qn, default NaN by ones)
      R_WR:  res <= {ws, we[14:0], wv[66] | ones, wv[65] | ones | qn, wv[64:3] | {62{ones}}};
      // the rounded result; on overflow infinity or the largest finite number of the precision
      R_RO:  if (ovr) res <= omax ? {ws, emax[14:0], (rx && rp != 2'd0) ? 64'hFFFFFFFFFFFFFFFF : ~rlow} : {ws, 15'h7fff, 64'h0};
             else res <= {ws, rcy ? we1[14:0] : (!tiny && wv[66:3] == 64'h0) ? 15'h0 : we[14:0], wv[66] | rcy, wv[65:3]};
      R_ADJ: res <= {res[79:64], res[63:32] + (cr_adj[1] ? 32'hFFFFFF80 : 32'h80), res[31:0]};
      default: ;
    endcase
    case (mo)
      M_W: am <= wv[66:3];
      M_B: am <= bm;                     // add: the larger magnitude goes to am
      default: ;
    endcase
    case (dop)
      D_LD:  dr <= {6'h0, wv[66:3]};                                // A's mantissa (also for the S_PRE compare)
      D_CLR: dr <= 70'h0;
      D_DIV: dr <= {dgo ? dd[68:0] : dr[68:0], 1'b0};               // subtract when it fits, then double
      D_SQ:  dr <= dge ? dd[69:0] : drn;
      default: ;
    endcase
    case (aop)
      A_CLR:  acc <= 80'h0;
      A_PUSH: acc <= {cs[15:0], acc[79:16]};                        // a finished column digit in at the top
      A_SHQ:  acc <= {acc[78:14], (st == S_MUL2) ? acc[13] : dge, acc[12:0], 1'b0};   // shift in a quotient / root bit
      default: ;
    endcase
    if (rq) begin                        // start: the command, flop to flop from p68_fpu
      cw <= c_cw; k <= c_k; md <= c_md; pp <= c_pp; fb <= c_b; avr <= c_a; dn <= 1'b0; st <= S_UNP0;
    end
  end
end
endmodule
`default_nettype wire
