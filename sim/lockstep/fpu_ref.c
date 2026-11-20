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
// Reference model of the Motorola 68881/68882 floating point unit (FPU), written for the lockstep bench of pipe68k.
//
// What it is for. The core has an FPU, and the lockstep bench has to know what the right answer of every FPU instruction is. Musashi,
// the reference CPU, has an FPU emulation of its own (m68kfpu.c), but it has no precision control, no exception bits in the status
// register, wrong results for FMOD, FINT and FGETEXP, and it supports few addressing modes. It could not be the reference. This file
// replaces it. tools/lockstep.sh writes a copy of Musashi's m68kcpu.c that includes this file where it would include m68kfpu.c,
// and tools/musashi_patch.py makes the F-line opcode handlers call m68040_fpu_op0() and m68040_fpu_op1() below. So the code here runs inside
// Musashi, uses Musashi's register macros (REG_FP, REG_FPCR, REG_FPSR, REG_FPIAR, REG_D, REG_A, REG_IR) and its memory and exception
// functions, and the bench reads the result through p68_fpu_get() and p68_fpu_ctl() at the end of the file.
//
// How it works. Everything operates on values. An operand is read from a register or from memory in one of the 68881 formats
// (byte, word, long, single, double, extended), converted to a normalized value (sign, exponent, 64-bit mantissa with the leading
// one at bit 63), and combined with exact integer arithmetic on 64 and 128-bit numbers. One single rounder (rnd) then produces the
// register result with the rules of the 68882: single and double precision use their own exponent limits, tininess is detected before
// rounding, and underflow is flagged even when the result is exact. The status register gets the exception byte and the accrued
// exception byte, enabled exceptions become pending traps that are taken before the next FPU instruction, and FSAVE and FRESTORE
// use the 68882 frames (a 4-byte NULL frame or a 60-byte IDLE frame).
//
// Scope. The core does in hardware what the Amiga software uses and sends the rest to the F-line exception (vector 11). This model
// does the same, so both sides trap the same instructions: the transcendental functions (FSIN, FLOGN and so on), FMOD, FREM, packed
// decimal operands, immediate .D, .X and .P operands, FMOVEM.X with a dynamic register list, FMOVEM.L #imm with two or three registers,
// and the opmodes 0x40 to 0x7F (the 68040 single and double rounding forms, which a 68882 does not have). The documentation of the
// whole FPU design is told in docs/learn/HISTORY.md, section 13.
//
// Origin and license. The code is project code (Apache-2.0). The arithmetic, the rounder, the conversions and the instruction
// handlers were written for this project from the 68881/68882 manual, without SoftFloat and without code from WinUAE. WinUAE's 68882
// emulation (GPL-2.0), which has been compared with real chips, served as the specification of what the real chips do where the
// manual is silent, and the comments say "as WinUAE" at those places. Matching behavior is all that was taken; no code or table was
// copied. The FMOVECR constants come from the Motorola MC68881/MC68882 User's Manual. The results for undefined ROM offsets and
// the rounding steps of the inexact constants are measured 68882 behavior, written down here as a small flag table of our own.
// Musashi's own FPU file and SoftFloat are not used for any result.
#include <stdio.h>
// stdio and stdlib are not used by the model itself (only by Musashi's headers); m68kcpu.h is Musashi's private header with all register macros.
#include <stdlib.h>
// stdio.h and stdlib.h: standard C headers that Musashi's private header expects to be present.
#include "m68kcpu.h"
// m68kcpu.h: Musashi's private header (register macros, EA macros such as EA_AY_DI_8, memory access functions, exception functions).

// Condition code byte of the FPSR (bits 27 to 24): N negative, Z zero, I infinity, NAN not a number.
#define FPCC_N    0x08000000
#define FPCC_Z    0x04000000
#define FPCC_I    0x02000000
#define FPCC_NAN  0x01000000
// Exception byte of the FPSR (bits 15 to 8) and the same positions in the FPCR enable byte: BSUN (branch or set on unordered), SNAN
// (signaling NaN operand), OPERR (operand error), OVFL (overflow), UNFL (underflow), DZ (divide by zero), INEX2 (inexact result of
// an operation), INEX1 (inexact conversion of a packed decimal input).
#define X_BSUN  0x8000
#define X_SNAN  0x4000
#define X_OPERR 0x2000
#define X_OVFL  0x1000
#define X_UNFL  0x0800
#define X_DZ    0x0400
#define X_INEX2 0x0200
#define X_INEX1 0x0100

// u64 is an unsigned 64-bit integer, used for mantissas.
typedef unsigned long long u64;
typedef struct { int s, e; u64 m; } fx;  // fx: a finite non-zero value in the form used for arithmetic. s = sign, e = exponent biased by 0x3FFF but allowed to leave the range of the format during a calculation, m = mantissa with bit 63 always set (the explicit leading one of the extended format)
// The four classes of a value: finite non-zero, zero, infinity, not a number.
enum { C_FIN, C_ZERO, C_INF, C_NAN };
// Model state that survives from one instruction to the next:

static int fpu_null = 1;  // NULL state: set by reset and by FRESTORE of a NULL frame; the first FPU instruction clears it, and FSAVE writes a NULL frame while it is set
static int fpu_pend = 0;  // vector number of an arithmetic exception that is pending (48 to 54), or 0 for none; taken before the next FPU instruction
static unsigned fl;  // exception byte (EXC) bits collected while the current instruction runs

// ---- Helpers on the 80-bit register format ------------------------------------------------------------------------------------
// floatx80 is the 80-bit type of the SoftFloat library: .high = sign (bit 15) and 15 exponent bits, .low = the 64-bit mantissa
// including the explicit integer bit. The helpers below only build and take apart that format, SoftFloat's arithmetic is not used.
// mk(): build a register value from sign, biased exponent and mantissa.
static floatx80 mk(int s, int e, u64 m) { floatx80 r; r.high = (bits16)(((s & 1) << 15) | (e & 0x7fff)); r.low = m; return r; }
// xs(): the sign bit. xe(): the 15-bit biased exponent.
static int xs(floatx80 a) { return (a.high >> 15) & 1; }
static int xe(floatx80 a) { return a.high & 0x7fff; }
// A NaN has exponent 0x7FFF and a non-zero fraction (the shift by one drops the explicit integer bit before the test).
static int is_nan(floatx80 a) { return xe(a) == 0x7fff && (a.low << 1) != 0; }
// A signaling NaN is a NaN whose first fraction bit (bit 62 of the mantissa) is zero; a quiet NaN has it set.
static int is_snan(floatx80 a) { return is_nan(a) && !(a.low & 0x4000000000000000ull); }
// An infinity has exponent 0x7FFF and a zero fraction.
static int is_inf(floatx80 a) { return xe(a) == 0x7fff && (a.low << 1) == 0; }
// The default NaN that an invalid operation produces: sign 0, exponent 0x7FFF, mantissa all ones.
static floatx80 dnan(void) { return mk(0, 0x7fff, ~0ull); }
// quiet(): turn a NaN into its quiet form by setting the first fraction bit; a signaling NaN also raises the SNAN exception bit.
static floatx80 quiet(floatx80 a) { if (is_snan(a)) fl |= X_SNAN; a.low |= 0x4000000000000000ull; return a; }
// lz64(): number of leading zero bits of m (64 for zero), counted one at a time.
static int lz64(u64 m) { int n = 0; if (!m) return 64; while (!(m >> 63)) { m <<= 1; n++; } return n; }
// ucls(): classify a register value and give its normalized value in v. Unnormals and denormals have the leading one somewhere
// below bit 63, so they are normalized by value: the mantissa is shifted left by the number of leading zeros and the exponent is
// reduced by the same amount.
static int ucls(floatx80 a, fx* v) {
  // e = biased exponent field.
  int e = xe(a);
  // v->s = sign.
  v->s = xs(a);
  // Exponent 0x7FFF: infinity (zero fraction) or NaN.
  if (e == 0x7fff) return (a.low << 1) ? C_NAN : C_INF;
  // A zero mantissa with any other exponent is a zero.
  if (a.low == 0) return C_ZERO;
  // Shift out the leading zeros and lower the exponent by the same count.
  { int n = lz64(a.low); v->m = a.low << n; v->e = e - n; }
  return C_FIN;
}

// ---- The rounder ---------------------------------------------------------------------------------------------------------------
// One function rounds every result of the model. The value to round is m0.m1 (two 64-bit words: m0 holds the mantissa with its integer
// bit at bit 63, m1 holds extra bits below it, with bit 0 of m1 used as a sticky bit that says "something non-zero was shifted out
// below"), multiplied by 2 to the power (e - 0x3FFF). prec is 64 (extended), 53 (double) or 24 (single). ext = 1 keeps the extended exponent
// range even at precision 24 or 53 (needed for FSGLMUL, FSGLDIV and FMOVECR).
// Rounding mode of the FPCR (bits 5 and 4): 0 nearest even, 1 toward zero, 2 toward minus infinity, 3 toward plus infinity.
static int mode(void) { return (REG_FPCR >> 4) & 3; }
// Precision of the FPCR (bits 7 and 6): 0 extended (64 mantissa bits), 1 single (24), 2 double (53); 3 is reserved and acts like double.
static int precf(void) { int p = (REG_FPCR >> 6) & 3; return p == 0 ? 64 : p == 1 ? 24 : 53; }
// jam(): shift the 128-bit value m0:m1 right by n bits. Bits that fall out are not lost but ORed into bit 0 of m1, so the sticky information
// survives for the rounding decision. Three cases: shift by 128 or more leaves only the sticky bit, shift by 64 to 127 moves m0 into m1, and
// a shift below 64 moves the low bits of m0 into the top of m1.
static void jam(u64* m0, u64* m1, int n) {      /* shift right by n with sticky into bit 0 of m1 */
  if (n <= 0) return;
  if (n >= 128) { *m1 = (*m0 | *m1) != 0; *m0 = 0; return; }
  if (n >= 64) { u64 st = *m1 != 0 || (n > 64 && (*m0 << (128 - n)) != 0); *m1 = (n == 64 ? *m0 : *m0 >> (n - 64)) | st; *m0 = 0; return; }
  { u64 st = (*m1 << (64 - n)) != 0; *m1 = (*m1 >> n) | (*m0 << (64 - n)) | st; *m0 >>= n; }
}
// incr(): decide whether to add one unit in the last place. s = sign, lsb = lowest kept bit, rb = first dropped bit (round bit),
// st = OR of all bits below it (sticky). The four cases are the four rounding modes.
static int incr(int s, int lsb, int rb, int st) {   /* round up? (rb: round bit, st: sticky below it) */
  switch (mode()) {
  // Nearest even: round up when the dropped part is above one half, or exactly one half and the kept part is odd.
  case 0: return rb && (st || lsb);
  // Toward zero: never round up (the magnitude is truncated).
  case 1: return 0;
  // Toward minus infinity: a negative value grows in magnitude when anything was dropped.
  case 2: return s && (rb || st);
  // Toward plus infinity: a positive value grows in magnitude when anything was dropped.
  default: return !s && (rb || st);
  }
}
// rnd(): round to prec bits and return the register value, setting the exception bits in fl.
static floatx80 rnd(int prec, int ext, int s, int e, u64 m0, u64 m1) {
  // k = number of low mantissa bits that are dropped (0 for extended, 11 for double, 40 for single).
  int k = 64 - prec;                             /* kept bits: m0[63:k] */
  // off = difference between the extended exponent bias (0x3FFF) and the bias of the target format (single 127, double 1023).
  int off = prec == 24 ? 0x3F80 : prec == 53 ? 0x3C00 : 0;
  // emax = largest biased exponent of a finite number in the target format, expressed with the extended bias.
  int emax = (ext || prec == 64) ? 0x7FFE : 0x7FFE - off;
  int emin = (ext || prec == 64) ? 0 : off + 1;  // emin = smallest exponent of a normal number in the target format. Anything below is tiny and is denormalized to emin
  // low = mask of the dropped bits; m = the rounded mantissa.
  u64 low = k ? ((1ull << k) - 1) : 0, m;
  // rb, st, lsb are the three rounding inputs; up = round up or not; tiny = the result is below the normal range.
  int rb, st, lsb, up, tiny = 0;
  if (e < emin) {  // tiny result: underflow is flagged even when the result turns out to be exact (as the 68882 does), and the value is denormalized by shifting it right until its exponent is emin
    tiny = 1; fl |= X_UNFL;
    jam(&m0, &m1, emin - e); e = emin;
  }
  // Extended precision (k = 0): the round bit is the top bit of m1, the sticky bit is the rest of m1, the last kept bit is bit 0 of m0.
  if (k == 0) { rb = (int)(m1 >> 63); st = (m1 << 1) != 0; lsb = (int)(m0 & 1); }
  // Reduced precision: the round bit is the highest dropped bit of m0, the sticky bit is the other dropped bits plus all of m1, and the
  // last kept bit sits just above the dropped part.
  else { rb = (int)((m0 >> (k - 1)) & 1); st = (m0 & (low >> 1)) != 0 || m1 != 0; lsb = (int)((m0 >> k) & 1); }
  // Decide the rounding direction from the mode.
  up = incr(s, lsb, rb, st);
  // Clear the dropped bits.
  m = m0 & ~low;
  // If rounding up, add one unit in the last place by adding low + 1 (the dropped bits are all zero now, so this adds 1 at the right position).
  if (up) {
    u64 n = m + (low + 1);
    if (n < m) { e++; n = 0x8000000000000000ull; }  // an overflow of the mantissa addition (all ones became zero): the value is 1.0 times two to the next exponent
    m = n;
  }
  // Overflow is checked after the increment, because rounding up can push the exponent past the maximum. OVFL is flagged, and INEX2
  // as well when the result is not exact.
  if (!tiny && e > emax) {                       /* overflow (after the increment) */
    fl |= X_OVFL; if (rb || st) fl |= X_INEX2;
    // In the modes that round toward zero for this sign (and in toward-zero mode) the overflow result is the largest finite number, otherwise infinity.
    if (mode() == 1 || (s && mode() == 3) || (!s && mode() == 2))
      // The largest finite mantissa is all ones in the kept bits (all 64 ones when the extended range is kept at a reduced precision).
      return mk(s, emax, (ext && prec != 64) ? ~0ull : ~low);
    // Infinity: exponent 0x7FFF and mantissa zero.
    return mk(s, 0x7fff, 0);
  }
  // An inexact result raises INEX2.
  if (rb || st) fl |= X_INEX2;
  if (tiny) return mk(s, e, m);  // a tiny result keeps exponent emin, also when the mantissa is zero
  // A zero mantissa after rounding is a true zero: biased exponent 0.
  if (m == 0) e = 0;
  return mk(s, e, m);
}
// rndf(): round a normalized value to the precision that the FPCR selects, with the normal exponent range.
static floatx80 rndf(fx v, u64 m1) { return rnd(precf(), 0, v.s, v.e, v.m, m1); }

// ---- FPSR -----------------------------------------------------------------------------------------------------------------------
// The status register is updated in two steps. set_cc() writes the condition code byte from the result. status() writes the exception byte and
// the accrued exception byte.
// set_cc(): set N, Z, I and NAN from the result value r.
static void set_cc(floatx80 r) {
  // Clear the old condition code byte and the unused top nibble; the quotient byte (bits 23-16) is kept.
  REG_FPSR &= 0x00ffffff;
  // N is the sign bit; it is set for negative numbers, negative zeros, negative infinities and NaNs with the sign bit set.
  if (xs(r)) REG_FPSR |= FPCC_N;
  // NaN sets NAN.
  if (is_nan(r)) REG_FPSR |= FPCC_NAN;
  // A zero (zero mantissa, exponent not 0x7FFF) sets Z.
  else if (r.low == 0 && xe(r) != 0x7fff) REG_FPSR |= FPCC_Z;
  // An infinity sets I.
  else if (is_inf(r)) REG_FPSR |= FPCC_I;
}
static void status(void) {  // status(): EXC (bits 15-8) = the exceptions of this instruction; the accrued byte (bits 7-3) only collects, it is never cleared by an instruction
  // a = accrued bits to add.
  unsigned a = 0;
  // Keep the cc byte, the quotient and the accrued byte, replace the exception byte by fl.
  REG_FPSR = (REG_FPSR & 0x0fff00f8) | fl;
  // Accrued IOP (bit 7) = BSUN or SNAN or OPERR.
  if (fl & (X_BSUN | X_SNAN | X_OPERR)) a |= 0x80;
  // Accrued OVFL (bit 6) = OVFL.
  if (fl & X_OVFL) a |= 0x40;
  // Accrued UNFL (bit 5) = UNFL together with INEX2 (underflow only counts when the result is also inexact).
  if ((fl & X_UNFL) && (fl & X_INEX2)) a |= 0x20;
  // Accrued DZ (bit 4) = DZ.
  if (fl & X_DZ) a |= 0x10;
  // Accrued INEX (bit 3) = INEX1 or INEX2 or OVFL.
  if (fl & (X_INEX1 | X_INEX2 | X_OVFL)) a |= 0x08;
  // Add the accrued bits to the status register.
  REG_FPSR |= a;
}
static void trap_check(uint32 pc) {  // trap_check(): look at the enable bits. An enabled exception that occurred makes a trap that is pending, and it is taken before the next FPU instruction
  // x = exceptions that occurred and are enabled in the FPCR.
  unsigned x = REG_FPSR & REG_FPCR & 0xff00;
  // When any exception enable bit is set, FPIAR gets the address of this instruction (it is where the trap is reported from).
  if (REG_FPCR & 0x7f00) REG_FPIAR = pc;
  // No enabled exception: nothing is pending.
  if (!x) return;
  // Pending vector, in priority order: BSUN 48, SNAN 54, OPERR 52, OVFL 53, UNFL 51, DZ 50, inexact 49 (the vectors 48 to 54 of the 68020 coprocessor exceptions).
  fpu_pend = (x & X_BSUN) ? 48 : (x & X_SNAN) ? 54 : (x & X_OPERR) ? 52 : (x & X_OVFL) ? 53 : (x & X_UNFL) ? 51 : (x & X_DZ) ? 50 : 49;
}

// ---- Arithmetic ----------------------------------------------------------------------------------------------------------------
// nan2(): result of a two-operand instruction when an operand is a NaN. The destination register's NaN wins when both are NaNs
// (68882 rule), and a signaling NaN raises SNAN. The result is always the quiet form.
static floatx80 nan2(floatx80 d, floatx80 s) {   /* dyadic NaN result: the destination NaN wins */
  // Any signaling NaN among the operands raises SNAN.
  if (is_snan(d) || is_snan(s)) fl |= X_SNAN;
  // Destination NaN: return it, made quiet.
  if (is_nan(d)) { d.low |= 0x4000000000000000ull; return d; }
  // Otherwise the source operand is the NaN: return it, made quiet.
  s.low |= 0x4000000000000000ull; return s;
}
// invalid(): an invalid operation (for example infinity minus infinity) raises OPERR and returns the default NaN.
static floatx80 invalid(void) { fl |= X_OPERR; return dnan(); }
// zero_add(): the sum of two zeros is a zero with the common sign, and positive zero when the signs differ (negative in the
// round toward minus infinity mode, mode 2).
static floatx80 zero_add(int s1, int s2) { return mk((s1 == s2) ? s1 : (mode() == 2), 0, 0); }

// f_add(): d + s, or d - s when sub is 1. Steps: classify both operands, handle NaN, infinity and zero cases, then add the
// magnitudes (equal signs) or subtract them (different signs) in 128 bits and normalize.
static floatx80 f_add(floatx80 d, floatx80 s, int sub) {   /* d + s or d - s */
  // a and b = normalized operands, ca and cb = their classes.
  fx a, b; int ca = ucls(d, &a), cb = ucls(s, &b);
  // A NaN operand decides the result.
  if (ca == C_NAN || cb == C_NAN) return nan2(d, s);
  // For a subtraction flip the sign of the source operand, and then both cases are additions of signed values.
  b.s ^= sub;
  // Infinity with infinity: the same sign gives that infinity, opposite signs are invalid (OPERR). The infinity keeps its mantissa bits.
  if (ca == C_INF && cb == C_INF) { if (a.s != b.s) return invalid(); return mk(a.s, 0x7fff, s.low); }
  // Only the destination is infinite: it is the result.
  if (ca == C_INF) return d;
  // Only the source is infinite: the result is an infinity with the source's (possibly flipped) sign.
  if (cb == C_INF) return mk(b.s, 0x7fff, s.low);
  // Both zero: see zero_add().
  if (ca == C_ZERO && cb == C_ZERO) return zero_add(a.s, b.s);
  // Destination zero: the result is the source, rounded to the current precision.
  if (ca == C_ZERO) return rndf(b, 0);
  // Source zero: the result is the destination, rounded.
  if (cb == C_ZERO) return rndf(a, 0);
  {
    // b1 = extra low bits of the aligned smaller operand, r0:r1 = the 128-bit result, n = shift count for normalizing.
    u64 b1 = 0, r0, r1; int n;
    if (a.e < b.e || (a.e == b.e && a.m < b.m)) { fx t = a; a = b; b = t; }  // order the operands so that a has the larger magnitude (exponent first, then mantissa)
    // Align: shift the smaller mantissa right by the exponent difference, with the dropped bits kept as sticky in b1.
    jam(&b.m, &b1, a.e - b.e);
    // Equal signs: add the magnitudes.
    if (a.s == b.s) {
      // Add the mantissas and the extra bits (r1 = b1).
      r0 = a.m + b.m; r1 = b1;
      // A carry out of the 64-bit mantissa: shift right by one (with sticky), put the carry in as the new leading one, and raise the exponent.
      if (r0 < a.m) { jam(&r0, &r1, 1); r0 |= 0x8000000000000000ull; a.e++; }
    // Different signs: subtract the smaller magnitude from the larger one.
    } else {
      // Subtract the extra bits too: r1 = 0 - b1 and borrow one from the top part when b1 was not zero.
      r1 = 0 - b1; r0 = a.m - b.m - (b1 != 0);
      // An exact zero result is positive, except in the toward minus infinity mode where it is negative.
      if (r0 == 0 && r1 == 0) return mk(mode() == 2, 0, 0);
      // n = number of leading zeros of the 128-bit difference (cancellation can remove many leading bits).
      n = r0 ? lz64(r0) : 64 + lz64(r1);
      // A shift of 64 or more: the new top word comes from the lower word.
      if (n >= 64) { r0 = r1 << (n - 64); r1 = 0; }
      // Shift left by n across both words.
      else if (n) { r0 = (r0 << n) | (r1 >> (64 - n)); r1 <<= n; }
      // Lower the exponent by the shift.
      a.e -= n;
    }
    // Put the normalized top word back into a and round the result with the extra bits.
    a.m = r0;
    return rndf(a, r1);
  }
}
// mul64(): 64 by 64 bit multiply giving a 128-bit product in hi:lo, built from four 32 by 32 bit products because plain C has no 128-bit integer.
static void mul64(u64 a, u64 b, u64* hi, u64* lo) {
  // Split both factors into 32-bit halves.
  u64 al = a & 0xffffffff, ah = a >> 32, bl = b & 0xffffffff, bh = b >> 32;
  // The four partial products.
  u64 ll = al * bl, lh = al * bh, hl = ah * bl, hh = ah * bh;
  // The middle column: the carries of the low part plus the low halves of the two cross products.
  u64 mid = (ll >> 32) + (lh & 0xffffffff) + (hl & 0xffffffff);
  // The low 64 bits of the product.
  *lo = (ll & 0xffffffff) | (mid << 32);
  // The high 64 bits: the high product, the high halves of the cross products and the carry from the middle column.
  *hi = hh + (lh >> 32) + (hl >> 32) + (mid >> 32);
}
// f_mul(): d times s. sgl = 1 is FSGLMUL, which cuts both inputs to 24 mantissa bits and rounds the result to single precision with the extended exponent range.
static floatx80 f_mul(floatx80 d, floatx80 s, int sgl) {
  // Classify the operands; zs = sign of the product (the XOR of both signs).
  fx a, b; int ca = ucls(d, &a), cb = ucls(s, &b); int zs = a.s ^ b.s;
  // NaN operand: see nan2().
  if (ca == C_NAN || cb == C_NAN) return nan2(d, s);
  // Infinity times zero is invalid; infinity times anything else is an infinity of the product's sign.
  if (ca == C_INF) { if (cb == C_ZERO) return invalid(); return mk(zs, 0x7fff, d.low); }
  // The same with the source as the infinity.
  if (cb == C_INF) { if (ca == C_ZERO) return invalid(); return mk(zs, 0x7fff, s.low); }
  // A zero operand gives a zero with the product's sign.
  if (ca == C_ZERO || cb == C_ZERO) return mk(zs, 0, 0);
  {
    // Exponent of the product: the sum of the exponents minus the bias, plus one for the case that the product of the two mantissas is 2.0 or more (the other case is corrected below).
    u64 hi, lo; int e = a.e + b.e - 0x3FFE;
    // FSGLMUL: cut both mantissas to their top 24 bits first.
    if (sgl) { a.m &= 0xFFFFFF0000000000ull; b.m &= 0xFFFFFF0000000000ull; }
    // 128-bit product of the two mantissas.
    mul64(a.m, b.m, &hi, &lo);
    // Two numbers in [1,2) multiply to [1,4). If the top bit is not set the product is below 2, so shift left once and lower the exponent.
    if (!(hi >> 63)) { hi = (hi << 1) | (lo >> 63); lo <<= 1; e--; }
    // Round: single with extended exponent range for FSGLMUL, else the precision of the FPCR.
    return sgl ? rnd(24, 1, zs, e, hi, lo) : rnd(precf(), 0, zs, e, hi, lo);
  }
}
// div128(): divide the mantissa a by the mantissa b (both normalized) with a restoring division that produces 128 quotient bits in q0:q1, bit by bit,
// and a sticky bit in q1 when a remainder is left. Returns the exponent correction (-1 when a is smaller than b, so that the quotient is below 1).
static int div128(u64 a, u64 b, u64* q0, u64* q1) {
  // r = running remainder (with its overflow bit rh), h:l = the quotient being built.
  u64 r = a, rh = 0, h = 0, l = 0; int i, e = 0;
  // If a is smaller than b, double the remainder first and remember that the quotient exponent drops by one.
  if (a < b) { rh = r >> 63; r <<= 1; e = -1; }
  // One loop pass per quotient bit.
  for (i = 0; i < 128; i++) {
    // The next quotient bit is 1 when the remainder (with its overflow bit) is at least b.
    int bit = rh || r >= b;
    // Subtract b when the bit is 1.
    if (bit) { r -= b; rh = 0; }
    // Shift the bit into the 128-bit quotient.
    h = (h << 1) | (l >> 63); l = (l << 1) | (u64)bit;
    // Double the remainder for the next bit, keeping its top bit in rh.
    rh = r >> 63; r <<= 1;
  }
  // Return the quotient; bit 0 of q1 also records whether anything was left over (sticky).
  *q0 = h; *q1 = l | ((r | rh) ? 1 : 0);
  return e;
}
// f_div(): d divided by s. sgl = 1 is FSGLDIV.
static floatx80 f_div(floatx80 d, floatx80 s, int sgl) {
  // Classify, and zs = sign of the quotient.
  fx a, b; int ca = ucls(d, &a), cb = ucls(s, &b); int zs = a.s ^ b.s;
  // NaN operand: see nan2().
  if (ca == C_NAN || cb == C_NAN) return nan2(d, s);
  // Infinity divided by infinity is invalid, infinity divided by a finite number is an infinity.
  if (ca == C_INF) { if (cb == C_INF) return invalid(); return mk(zs, 0x7fff, d.low); }
  // A finite number divided by infinity is zero.
  if (cb == C_INF) return mk(zs, 0, 0);
  // Division by zero: zero by zero is invalid, a finite value by zero raises DZ and gives an infinity.
  if (cb == C_ZERO) { if (ca == C_ZERO) return invalid(); fl |= X_DZ; return mk(zs, 0x7fff, 0); }
  // Zero divided by a finite number is zero.
  if (ca == C_ZERO) return mk(zs, 0, 0);
  {
    // Exponent: difference of the exponents plus the bias plus the correction from div128.
    u64 q0, q1; int e = a.e - b.e + 0x3FFF + div128(a.m, b.m, &q0, &q1);
    // Round (single for FSGLDIV, else the FPCR precision).
    return sgl ? rnd(24, 1, zs, e, q0, q1) : rnd(precf(), 0, zs, e, q0, q1);
  }
}
// u128: a 128-bit unsigned integer as two 64-bit halves, used by the square root.
typedef struct { u64 h, l; } u128;
// sh2_u128(): shift left by 2 and bring in two new bits (add).
static u128 sh2_u128(u128 a, unsigned add) { u128 r; r.h = (a.h << 2) | (a.l >> 62); r.l = (a.l << 2) | add; return r; }
// ge_u128(): compare, a >= b.
static int ge_u128(u128 a, u128 b) { return a.h > b.h || (a.h == b.h && a.l >= b.l); }
// sub_u128(): a - b with borrow.
static u128 sub_u128(u128 a, u128 b) { u128 r; r.l = a.l - b.l; r.h = a.h - b.h - (a.l < b.l); return r; }
// f_sqrt(): square root by digit recurrence, which produces one root bit per step from two bits of the radicand, so the result is exact
// with a correct sticky bit and no division or iteration with a guess is needed.
static floatx80 f_sqrt(floatx80 x) {
  // Classify the operand.
  fx a; int c = ucls(x, &a);
  // NaN operand: result is the quiet NaN.
  if (c == C_NAN) return quiet(x);
  // The square root of zero is zero with the same sign.
  if (c == C_ZERO) return mk(a.s, 0, 0);
  // Any negative non-zero number (and negative infinity) is invalid.
  if (a.s) return invalid();
  // The square root of +infinity is +infinity.
  if (c == C_INF) return x;
  {
    // The radicand is the mantissa m (when the unbiased exponent ue is even) or 2m (when it is odd), so that it lies in [1,4) in units of 2^63;
    // the root then lies in [1,2). 66 root bits are computed (64 for the mantissa plus two more that serve for rounding) with two radicand bits per step.
    /* X = m (ue even) or 2m (ue odd), X / 2^63 in [1, 4): 66 root bits by digit recurrence, two radicand bits per step */
    // ue = unbiased exponent, odd = its lowest bit.
    int ue = a.e - 0x3FFF, odd = ue & 1, i;
    // X as a 65-bit number xh:xl (the shift by one for odd exponents moves bit 63 into xh).
    u64 xh = odd ? (a.m >> 63) : 0, xl = odd ? (a.m << 1) : a.m;   /* X = xh:xl, bits 64..0 */
    // r = remainder, q = root so far, t = trial value for the next root bit.
    u128 r = { 0, 0 }, q = { 0, 0 }, t;
    for (i = 0; i < 66; i++) {
      // Pick the next two radicand bits, pos+1 and pos (bits beyond the number are zero).
      unsigned pr;
      // pos = position of the lower bit of the pair; pr = the pair as a 2-bit number.
      int pos = 63 - 2 * i;                       /* pair = X bits pos+1, pos */
      pr = (unsigned)((pos + 1 >= 64 ? xh & 1 : pos + 1 >= 0 ? (xl >> (pos + 1)) & 1 : 0) << 1 | (pos >= 0 ? (xl >> pos) & 1 : 0));
      // Bring the pair into the remainder (remainder times 4 plus pair).
      r = sh2_u128(r, pr);
      // Trial value: 4 times the root so far plus 1.
      t = sh2_u128(q, 1);
      // Make room for the next root bit.
      q.h = (q.h << 1) | (q.l >> 63); q.l <<= 1;
      // If the remainder covers the trial value the next root bit is 1: subtract.
      if (ge_u128(r, t)) { r = sub_u128(r, t); q.l |= 1; }
    }
    {
      // The mantissa of the root is the top 64 bits of the 66 root bits; the lower two bits and the remainder become the sticky bits.
      u64 m0 = (q.h << 62) | (q.l >> 2), m1 = (q.l << 62) | ((r.h | r.l) ? 1 : 0);
      // The result exponent is half the even part of the unbiased exponent, plus the bias.
      fx v; v.s = 0; v.e = (ue - odd) / 2 + 0x3FFF; v.m = m0;
      // Round with the extra bits.
      return rndf(v, m1);
    }
  }
}
static floatx80 f_mono(floatx80 x, int op) {  // f_mono(): FMOVE, FABS and FNEG between registers; the value is rounded to the FPCR precision (op 0 move, 1 absolute value, 2 negate)
  fx a; int c = ucls(x, &a);
  // NaN operand: the quiet NaN.
  if (c == C_NAN) return quiet(x);
  // op 1 clears the sign, op 2 flips it.
  if (op == 1) a.s = 0; if (op == 2) a.s ^= 1;
  // Infinity and zero keep their class with the new sign.
  if (c == C_INF) return mk(a.s, 0x7fff, x.low);
  // Anything else goes through the rounder, because a move to a lower precision can change the value.
  if (c == C_ZERO) return mk(a.s, 0, 0);
  return rndf(a, 0);
}
// f_int(): FINT (round to an integer with the current rounding mode) and FINTRZ (toward zero, rz = 1).
static floatx80 f_int(floatx80 x, int rz) {
  // md = rounding mode used; ue = unbiased exponent.
  fx a; int c = ucls(x, &a), ue, md = rz ? 1 : mode();
  // A NaN stays a NaN, an infinity and a zero stay as they are.
  if (c == C_NAN) return quiet(x);
  if (c == C_INF) return x;
  if (c == C_ZERO) return mk(a.s, 0, 0);
  // Unbiased exponent.
  ue = a.e - 0x3FFF;
  // From 2^63 on every extended value is already an integer.
  if (ue >= 63) return mk(a.s, a.e, a.m);
  {
    // i0 = integer part, i1 = fraction bits, with rb and st as round and sticky bits.
    u64 i0 = a.m, i1 = 0; int rb, st, up = 0, n;
    // Shift right so that the binary point is at the end of i0.
    jam(&i0, &i1, 63 - ue);
    // Round bit and sticky bit of the fraction.
    rb = (int)(i1 >> 63); st = (i1 << 1) != 0;
    // A non-zero fraction raises INEX2 and decides the rounding.
    if (rb || st) {
      fl |= X_INEX2;
      // Round up (add one) by mode: nearest even, never, for negative numbers in the toward minus infinity mode, for positive numbers in the toward plus infinity mode.
      switch (md) { case 0: up = rb && (st || (i0 & 1)); break; case 1: up = 0; break; case 2: up = a.s; break; default: up = !a.s; }
    }
    // Apply the increment.
    i0 += up;
    // A result that is zero keeps the sign of the operand.
    if (!i0) return mk(a.s, 0, 0);
    // Normalize the integer back into extended form.
    n = lz64(i0);
    return mk(a.s, 0x3FFF + 63 - n, i0 << n);
  }
}
// f_getexp(): FGETEXP, the unbiased exponent of the operand as a number (an infinity is invalid, a zero gives a zero).
static floatx80 f_getexp(floatx80 x) {
  fx a; int c = ucls(x, &a), n; long v; u64 m;
  if (c == C_NAN) return quiet(x);
  if (c == C_INF) return invalid();
  if (c == C_ZERO) return mk(a.s, 0, 0);
  // v = unbiased exponent as a signed integer.
  v = a.e - 0x3FFF;
  // Exponent zero gives +0.
  if (!v) return mk(0, 0, 0);
  // m = the magnitude of the exponent.
  m = v < 0 ? (u64)(-v) : (u64)v;
  // Convert the integer to extended form (the leading bit at bit 63).
  n = lz64(m);
  return mk(v < 0, 0x3FFF + 63 - n, m << n);
}
// f_getman(): FGETMAN, the mantissa of the operand with the exponent set to zero, so the value lies in [1,2) with the original sign.
static floatx80 f_getman(floatx80 x) {
  fx a; int c = ucls(x, &a);
  if (c == C_NAN) return quiet(x);
  if (c == C_INF) return invalid();
  if (c == C_ZERO) return mk(a.s, 0, 0);
  return mk(a.s, 0x3FFF, a.m);
}
// f_scale(): FSCALE, d times 2 to the power of the integer part of s.
static floatx80 f_scale(floatx80 d, floatx80 s) {
  fx a, b; int ca = ucls(d, &a), cb = ucls(s, &b);
  // NaN source, or NaN destination with an infinite source: NaN rules.
  if (cb == C_NAN || (cb == C_INF && ca == C_NAN)) return nan2(d, s);
  // An infinite scale factor is invalid (unless a NaN decided already).
  if (cb == C_INF) return invalid();
  if (ca == C_NAN) return nan2(d, s);
  // An infinite destination stays.
  if (ca == C_INF) return d;
  // A zero destination stays a zero.
  if (ca == C_ZERO) return mk(a.s, 0, 0);
  // A zero or fractional scale factor (below 1.0) leaves the value unchanged apart from rounding.
  if (cb == C_ZERO || b.e < 0x3FFF) return rndf(a, 0);
  // A scale factor of 2^17 or more (exponent above 16) overflows or underflows at once; the exponent is moved far out of range so that the rounder raises OVFL or UNFL.
  if (b.e > 0x400F) return rnd(precf(), 0, a.s, b.s ? -0x6001 : 0xE000, a.m, 0);
  // Otherwise take the integer part of the scale factor and add or subtract it from the exponent.
  { long n = (long)(b.m >> (0x403E - b.e)); return rnd(precf(), 0, a.s, b.s ? a.e - n : a.e + n, a.m, 0); }
}
// f_cmp(): FCMP, condition codes only. WinUAE compares magnitudes and signs, no subtraction takes place (so no inexact or overflow from the compare); the 68882 results are the same
// Classify the operands.
static floatx80 f_cmp(floatx80 d, floatx80 s) {
  fx a, b; int ca = ucls(d, &a), cb = ucls(s, &b);
  // one = the value 1.0, used as a result that is neither zero nor NaN; its sign bit then carries the answer for the N flag.
  floatx80 one = mk(0, 0x3FFF, 0x8000000000000000ull);
  // NaN operand: the result is that NaN with the sign cleared (only the NAN flag matters).
  if (ca == C_NAN || cb == C_NAN) { floatx80 r = nan2(d, s); r.high &= 0x7fff; return r; }
  // Zero is the smallest magnitude (exponent far below any real value) ...
  if (ca == C_ZERO) { a.e = -0x100000; a.m = 0; }
  if (cb == C_ZERO) { b.e = -0x100000; b.m = 0; }
  // ... and infinity the largest.
  if (ca == C_INF) { a.e = 0x100000; a.m = 0; }
  if (cb == C_INF) { b.e = 0x100000; b.m = 0; }
  // d larger in magnitude: the result is negative when d is negative.
  if (a.e > b.e || (a.e == b.e && a.m > b.m)) { one.high |= a.s << 15; return one; }
  // d smaller in magnitude: the result is negative when s is positive (d - s is negative then).
  if (a.e < b.e || (a.e == b.e && a.m < b.m)) { one.high |= (b.s ^ 1) << 15; return one; }
  // Equal infinities: a zero with that sign when the signs are equal, else the sign of d decides.
  if (ca == C_INF) { if (a.s == b.s) return mk(a.s, 0, 0); one.high |= a.s << 15; return one; }
  // Equal zeros: a zero with d's sign.
  if (ca == C_ZERO) return mk(a.s, 0, 0);
  // Equal finite values with equal signs: zero.
  if (a.s == b.s) return mk(0, 0, 0);
  // Equal magnitudes with different signs: the sign decides.
  one.high |= a.s << 15; return one;
}

// ---- Number formats: conversions between register values and memory or Dn operands --------------------------------------------
/* ---- formats --------------------------------------------------------------------------------------------------------- */
// from_int(): a signed integer (byte, word or long, already sign extended to long) as an extended value.
static floatx80 from_int(long v) {
  u64 m; int n;
  // Zero is the all-zero value.
  if (!v) return mk(0, 0, 0);
  // Take the magnitude (as a 64-bit number, so the most negative long works) and shift the leading one up to bit 63.
  m = v < 0 ? (u64)(-(long long)v) : (u64)v; n = lz64(m);
  // The exponent is 0x3FFF + 63 - n (n = shift), the sign is the sign of the integer.
  return mk(v < 0, 0x3FFF + 63 - n, m << n);
}
// from_s(): an IEEE single (1 sign bit, 8 exponent bits, 23 fraction bits) as an extended value.
static floatx80 from_s(uint32 v) {
  // s = sign, e = exponent field, f = fraction.
  int s = v >> 31, e = (v >> 23) & 0xff; u64 f = v & 0x7fffff;
  // Exponent 255 is infinity or NaN: keep the fraction at the top of the mantissa (shifted by 40 so that the NaN type bit lands on bit 62).
  if (e == 0xff) return mk(s, 0x7fff, f << 40);
  // Exponent 0 is zero or a denormal; a denormal is normalized by value (shift out the leading zeros, exponent 0x3F81 minus the shift).
  if (e == 0) { int n; if (!f) return mk(s, 0, 0); n = lz64(f << 40); return mk(s, 0x3F81 - n, (f << 40) << n); }
  // A normal number: rebias the exponent (add 0x3F80) and add the explicit leading one in front of the fraction.
  return mk(s, e + 0x3F80, 0x8000000000000000ull | (f << 40));
}
// from_d(): the same for an IEEE double (11 exponent bits, 52 fraction bits, rebias 0x3C00, fraction shifted by 11).
static floatx80 from_d(u64 v) {
  // s = sign, e = exponent field, f = fraction.
  int s = (int)(v >> 63), e = (int)(v >> 52) & 0x7ff; u64 f = v & 0xfffffffffffffull;
  // Exponent 0x7FF: infinity or NaN.
  if (e == 0x7ff) return mk(s, 0x7fff, f << 11);
  // Exponent 0: zero or denormal, normalized by value.
  if (e == 0) { int n; if (!f) return mk(s, 0, 0); n = lz64(f << 11); return mk(s, 0x3C01 - n, (f << 11) << n); }
  // Normal number: rebias and add the leading one.
  return mk(s, e + 0x3C00, 0x8000000000000000ull | (f << 11));
}
// to_sd(): an extended value as single or double (dbl = 1) in memory format. The rounder runs at 24 or 53 bits with the exponent limits of
// the format, and the result is packed.
/* to single/double: the rounder at 24/53 bits with the format's exponent limits, then packed */
static u64 to_sd(floatx80 x, int dbl) {
  fx a; int c = ucls(x, &a);
  // fb = fraction bits, eb = exponent bits, bias = exponent offset between extended and the target format.
  int fb = dbl ? 52 : 23, eb = dbl ? 11 : 8, bias = dbl ? 0x3C00 : 0x3F80;
  // sg = the sign in its position in the packed word.
  u64 sg = (u64)xs(x) << (fb + eb);
  // NaN: set the exponent field to all ones and take the top fraction bits of the quieted NaN.
  if (c == C_NAN) { floatx80 q = quiet(x); return sg | (((1ull << eb) - 1) << fb) | ((q.low << 1) >> (64 - fb)); }
  // Infinity: exponent all ones, fraction zero.
  if (c == C_INF) return sg | (((1ull << eb) - 1) << fb);
  // Zero: only the sign.
  if (c == C_ZERO) return sg;
  {
    // Round to the target precision; the result is still an extended value.
    floatx80 r = rnd(dbl ? 53 : 24, 0, a.s, a.e, a.m, 0);
    int e = xe(r);
    // Overflow to infinity.
    if (e == 0x7fff) return sg | (((1ull << eb) - 1) << fb);
    // A zero after rounding (underflow to zero).
    if (r.low == 0) return sg;
    // A denormal in the target format has no leading one in the stored mantissa; the fraction is the mantissa bits shifted into place and the exponent field is zero.
    if (!(r.low >> 63)) return sg | ((r.low << 1) >> (64 - fb));                 /* denormal */
    // A normal number: exponent field = exponent minus the bias, fraction = the 23 or 52 bits behind the leading one.
    return sg | ((u64)(e - bias) << fb) | ((r.low << 1) >> (64 - fb));
  }
}
// to_int(): an extended value to a signed integer of nb bits (8, 16 or 32), rounded with the FPCR mode. An out of range value raises OPERR and gives the largest or smallest integer.
/* to integer of n bits (8, 16, 32) */
static long to_int(floatx80 x, int nb) {
  // mx = the largest positive value of the target size.
  fx a; int c = ucls(x, &a); long mx = (1L << (nb - 1)) - 1;
  // NaN operand: the integer is made of the top bits of the NaN. A quiet NaN raises OPERR; a signaling NaN raises SNAN and, for the word size only, also OPERR (68882 behavior).
  if (c == C_NAN) {
    int wasq = !is_snan(x); floatx80 q = quiet(x);
    if (wasq || nb == 16) fl |= X_OPERR;
    return (long)(q.low >> (64 - nb));
  }
  // Infinity: OPERR and the largest or smallest integer by sign.
  if (c == C_INF) { fl |= X_OPERR; return a.s ? -mx - 1 : mx; }
  // Zero gives zero.
  if (c == C_ZERO) return 0;
  {
    // ue = unbiased exponent, i0 = integer part, i1 = fraction.
    int ue = a.e - 0x3FFF; u64 i0 = a.m, i1 = 0; int rb, st, up = 0;
    // A magnitude of 2^63 or more cannot be in range.
    if (ue > 62) { fl |= X_OPERR; return a.s ? -mx - 1 : mx; }
    // Shift so that the binary point is at the end of i0.
    jam(&i0, &i1, 63 - ue);
    // Round bit and sticky bit.
    rb = (int)(i1 >> 63); st = (i1 << 1) != 0;
    // Round up by mode: nearest even, never (toward zero), negative numbers for toward minus infinity, positive ones for toward plus infinity (this time the dropped bits themselves decide).
    switch (mode()) { case 0: up = rb && (st || (i0 & 1)); break; case 1: break; case 2: up = a.s && (rb || st); break; default: up = !a.s && (rb || st); }
    // Apply the increment.
    i0 += up;
    // Out of range after rounding (the negative range has one more value than the positive one): OPERR and the limit.
    if (a.s ? i0 > (u64)mx + 1 : i0 > (u64)mx) { fl |= X_OPERR; return a.s ? -mx - 1 : mx; }
    // A non-zero fraction raises INEX2.
    if (rb || st) fl |= X_INEX2;
    // Return the signed integer.
    return a.s ? -(long)i0 : (long)i0;
  }
}
// to_x(): FMOVE.X to memory. NaNs are quieted, a denormal is rounded to the FPCR precision (which raises UNFL), and everything else is written unchanged.
/* FMOVE.X out: denormals are rounded to the FPCR precision (UNFL), NaNs quieted, everything else raw */
static floatx80 to_x(floatx80 x) {
  fx a; int c = ucls(x, &a);
  if (c == C_NAN) return quiet(x);
  // A denormal (exponent below zero after normalizing) goes through the rounder.
  if (c == C_FIN && a.e < 0) return rndf(a, 0);
  // A normal finite value is written as is.
  if (c == C_FIN) return mk(a.s, a.e, a.m);
  if (c == C_ZERO) return mk(a.s, 0, 0);  // a zero is a zero (the value was normalized first, so an unnormal with a zero mantissa is a plain zero)
  // Infinity is written unchanged.
  return x;
}

// ---- Effective addresses -----------------------------------------------------------------------------------------------------
// An FPU instruction has an opcode word whose low 6 bits give the effective address (mode in bits 5-3, register in bits 2-0), exactly as
// an integer instruction. ea_m and ea_r hold them while an instruction runs.
/* ---- effective addresses ---------------------------------------------------------------------------------------------- */
// ea_m = addressing mode, ea_r = register number.
static int ea_m, ea_r;
static uint32 ea_addr(int size) {  // ea_addr(): the address of a memory operand of the given size in bytes (modes 2 to 7 without immediate); (An)+ and -(An) update An here
  uint32 a;
  // Mode 2: (An).
  switch (ea_m) {
  // Mode 3: (An)+: use An, then add the size (byte accesses through A7 step by 2 to keep the stack even).
  case 2: return REG_A[ea_r];
  // Mode 4: -(An): subtract the size first, then use An.
  case 3: a = REG_A[ea_r]; REG_A[ea_r] += (size == 1 && ea_r == 7) ? 2 : size; return a;
  case 4: REG_A[ea_r] -= (size == 1 && ea_r == 7) ? 2 : size; return REG_A[ea_r];
  // Mode 5: (d16,An); the macro reads the extension word and adds it.
  case 5: return EA_AY_DI_8();
  // Mode 6: (d8,An,Xn) and the 68020 full forms, handled by Musashi's macro.
  case 6: return EA_AY_IX_8();
  default:
    // Mode 7 has four forms picked by the register field:
    switch (ea_r) {
    // absolute word, ...
    case 0: return EA_AW_8();
    // ... absolute long, ...
    case 1: return EA_AL_8();
    // ... (d16,PC) ...
    case 2: return EA_PCDI_8();
    // ... and (d8,PC,Xn).
    case 3: return EA_PCIX_8();
    }
  }
  return 0;
}
// fsz[]: bytes per source format (0 long 4, 1 single 4, 2 extended 12, 3 packed 12, 4 word 2, 5 double 8, 6 byte 1, 7 not used).
static const int fsz[8] = { 4, 4, 12, 12, 2, 8, 1, 0 };
// read_src(): read the source operand of format f (0 long, 1 single, 2 extended, 4 word, 5 double, 6 byte) into *v; returns 0 for forms the model traps (packed decimal, some modes)
static int read_src(int f, floatx80* v) {
  uint32 a, h, l;
  // A data register as source: only the integer formats and single are allowed.
  if (ea_m == 0) {
    uint32 d = REG_D[ea_r];
    // Long and single come from the whole register, ...
    switch (f) { case 0: *v = from_int((sint32)d); return 1; case 1: *v = from_s(d); return 1;
                 // ... word and byte from its low part.
                 case 4: *v = from_int((sint16)d); return 1; case 6: *v = from_int((sint8)d); return 1; }
    // Other formats from a data register are not allowed (the caller takes line F).
    return 0;
  }
  // An address register is not a valid source.
  if (ea_m == 1) return 0;
  // Immediate data (mode 7, register 4): integer formats and single are read from the instruction stream; double, extended and packed are trapped.
  if (ea_m == 7 && ea_r == 4) {
    switch (f) { case 0: *v = from_int((sint32)OPER_I_32()); return 1; case 1: *v = from_s(OPER_I_32()); return 1;
                 case 4: *v = from_int((sint16)OPER_I_16()); return 1; case 6: *v = from_int((sint8)(OPER_I_16() & 0xff)); return 1; }
    return 0;
  }
  // Modes 7 with registers 5-7 do not exist.
  if (ea_m == 7 && ea_r > 3) return 0;
  // Memory source: compute the address (which also steps An for (An)+ and -(An)).
  a = ea_addr(fsz[f]);
  switch (f) {
  // Read by format: long and single are one 32-bit word ...
  case 0: *v = from_int((sint32)m68ki_read_32(a)); break;
  case 1: *v = from_s(m68ki_read_32(a)); break;
  // ... word and byte one 16 or 8-bit access (sign extended to long first) ...
  case 4: *v = from_int((sint16)m68ki_read_16(a)); break;
  case 6: *v = from_int((sint8)m68ki_read_8(a)); break;
  // ... a double is two 32-bit words, high word first ...
  case 5: h = m68ki_read_32(a); l = m68ki_read_32(a + 4); *v = from_d(((u64)h << 32) | l); break;
  // ... an extended is three: sign and exponent in the top half of the first, then the 64-bit mantissa.
  case 2: h = m68ki_read_32(a); *v = mk(h >> 31, (h >> 16) & 0x7fff, ((u64)m68ki_read_32(a + 4) << 32) | m68ki_read_32(a + 8)); break;
  }
  return 1;
}

static void vec0(int v) {  // vec0(): raise exception v with a format 0 stack frame; the stacked PC is the address of the FPU instruction itself (REG_PPC)
  // Start the exception: save SR, enter supervisor mode.
  uint sr = m68ki_init_exception();
  // Push the 4-word format 0 frame.
  m68ki_stack_frame_0000(REG_PPC, sr, v);
  // Jump through the vector table.
  m68ki_jump_vector(v);
}
// ---- Conditions ----------------------------------------------------------------------------------------------------------------
/* ---- conditions ---------------------------------------------------------------------------------------------------- */
// fcond(): evaluate one of the 32 FPU conditions c (bit 4 = signal on unordered, bits 3-0 the test) from the status register's N, Z and NAN bits. Returns 1 or 0, or -1 when a BSUN trap was taken.
static int fcond(int c) {
  // n, z, nan = the condition code bits (the I bit does not enter any condition).
  int n = (REG_FPSR & FPCC_N) != 0, z = (REG_FPSR & FPCC_Z) != 0, nan = (REG_FPSR & FPCC_NAN) != 0, r = 0;
  // The conditions 0x10-0x1F signal BSUN when the operand is unordered (NaN).
  if ((c & 0x10) && nan) {
    // Set BSUN and the accrued IOP bit.
    REG_FPSR |= X_BSUN | 0x80;
    if (REG_FPCR & X_BSUN) { vec0(48); return -1; }  // when the BSUN trap is enabled the exception is taken before the instruction acts, and the caller stops
  }
  // The sixteen tests, written as in the Motorola manual's table: 0 false, 1 equal, 2 ordered greater than, 3 ordered greater or equal, 4 ordered less than, 5 ordered less or equal, 6 ordered not equal ...
  switch (c & 15) {
  // ... 7 ordered, 8 unordered, 9 unordered or equal, 10 unordered or greater, 11 unordered or greater or equal, 12 unordered or less, 13 unordered or less or equal, 14 not equal, 15 true.
  case 0: r = 0; break; case 1: r = z; break; case 2: r = !(nan || z || n); break; case 3: r = z || !(nan || n); break;
  case 4: r = n && !(nan || z); break; case 5: r = z || (n && !nan); break; case 6: r = !(nan || z); break;
  case 7: r = !nan || z; break; case 8: r = nan; break; case 9: r = nan || z; break; case 10: r = nan || !(n || z); break;
  case 11: r = nan || z || !n; break; case 12: r = nan || (n && !z); break; case 13: r = nan || z || n; break;
  case 14: r = !z || nan; break; default: r = 1;
  }
  // With Z and NAN both set (the combination cannot come from a normal result) the OR forms and NE are true in WinUAE's table, and the formulas above give the same.
  return r;
}
// take_pending(): before an FPU instruction runs, a pending arithmetic exception from an earlier instruction is taken as a trap. Returns 1 if the trap was taken.
static int take_pending(void) {                  /* pre-instruction trap of a pending arithmetic exception */
  // Nothing pending.
  if (!fpu_pend) return 0;
  // Take the trap now.
  vec0(fpu_pend);
  return 1;
}

// ---- FMOVECR: the constant ROM ------------------------------------------------------------------------------------------------
// The ROM of the 68881/68882 holds 22 constants. Each is stored as a sign and exponent word and a 64-bit mantissa, copied from the
// FMOVECR table of the MC68881/MC68882 User's Manual. Offset $00 is pi, $0B to $0F are log10(2), e, log2(e), log10(e) and 0.0, $30 and
// $31 are ln(2) and ln(10), and $32 to $3F are 10^0, 10^1, 10^2, 10^4, 10^8 and so on up to 10^4096.
// The inexact ones are stored rounded in one direction. With the rounding mode toward zero, toward minus infinity or toward plus
// infinity the chip returns the neighbor on the other side when the mode points away from the stored rounding, and it flags INEX2.
enum { ROM_EXACT, ROM_STORED_HIGH, ROM_STORED_LOW };  // stored above the true value, stored below it
struct rom_const { unsigned char offset; unsigned char dir; unsigned short se; uint32 hi, lo; };
static const struct rom_const rom_tab[22] = {
  {0x00, ROM_STORED_HIGH, 0x4000, 0xc90fdaa2, 0x2168c235},  // pi
  {0x0b, ROM_STORED_LOW,  0x3ffd, 0x9a209a84, 0xfbcff798},  // log10(2)
  {0x0c, ROM_STORED_LOW,  0x4000, 0xadf85458, 0xa2bb4a9a},  // e
  {0x0d, ROM_STORED_HIGH, 0x3fff, 0xb8aa3b29, 0x5c17f0bc},  // log2(e)
  {0x0e, ROM_EXACT,       0x3ffd, 0xde5bd8a9, 0x37287195},  // log10(e)
  {0x0f, ROM_EXACT,       0x0000, 0x00000000, 0x00000000},  // 0.0
  {0x30, ROM_STORED_HIGH, 0x3ffe, 0xb17217f7, 0xd1cf79ac},  // ln(2)
  {0x31, ROM_STORED_HIGH, 0x4000, 0x935d8ddd, 0xaaa8ac17},  // ln(10)
  {0x32, ROM_EXACT,       0x3fff, 0x80000000, 0x00000000},  // 10^0
  {0x33, ROM_EXACT,       0x4002, 0xa0000000, 0x00000000},  // 10^1
  {0x34, ROM_EXACT,       0x4005, 0xc8000000, 0x00000000},  // 10^2
  {0x35, ROM_EXACT,       0x400c, 0x9c400000, 0x00000000},  // 10^4
  {0x36, ROM_EXACT,       0x4019, 0xbebc2000, 0x00000000},  // 10^8
  {0x37, ROM_EXACT,       0x4034, 0x8e1bc9bf, 0x04000000},  // 10^16
  {0x38, ROM_STORED_HIGH, 0x4069, 0x9dc5ada8, 0x2b70b59e},  // 10^32
  {0x39, ROM_STORED_LOW,  0x40d3, 0xc2781f49, 0xffcfa6d5},  // 10^64
  {0x3a, ROM_STORED_HIGH, 0x41a8, 0x93ba47c9, 0x80e98ce0},  // 10^128
  {0x3b, ROM_STORED_HIGH, 0x4351, 0xaa7eebfb, 0x9df9de8e},  // 10^256
  {0x3c, ROM_STORED_HIGH, 0x46a3, 0xe319a0ae, 0xa60e91c7},  // 10^512
  {0x3d, ROM_STORED_LOW,  0x4d48, 0xc9767586, 0x81750c17},  // 10^1024
  {0x3e, ROM_STORED_HIGH, 0x5a92, 0x9e8b3b5d, 0xc53d5de5},  // 10^2048
  {0x3f, ROM_STORED_HIGH, 0x7525, 0xc4605202, 0x8a20979b}   // 10^4096
};
// rom_find(): the entry for a ROM offset, or 0 when the offset is not defined.
static const struct rom_const *rom_find(int offset) {
  int i;
  for (i = 0; i < 22; i++) if (rom_tab[i].offset == offset) return &rom_tab[i];
  return 0;
}
// The undefined offsets $01 to $0A return odd values (measured on a 68882). Every other undefined offset returns entry 0 of this
// table. The flags say what changes with precision, rounding mode and condition codes.
enum {
  ODD_STEP_NEAR_DOWN_ELSE_UP = 1,  // single precision: one step down when rounding to nearest, one step up toward zero or minus infinity
  ODD_STEP_DOWN_RP = 2,            // single precision: one step down when rounding toward plus infinity
  ODD_CC_INF_OR_NAN = 4,           // sets I (single precision, nearest or plus infinity) or else NAN
  ODD_CC_NAN = 8                   // sets NAN
};
struct rom_odd { unsigned short se; uint32 hi, lo; unsigned char flags; };
static const struct rom_odd rom_odd_tab[11] = {
  {0x4000, 0x00000000, 0x00000000, 0},
  {0x4001, 0xfe000682, 0x00000000, ODD_STEP_NEAR_DOWN_ELSE_UP},
  {0x4001, 0xffc00503, 0x80000000, ODD_STEP_DOWN_RP},
  {0x2000, 0x7fffffff, 0x00000000, ODD_CC_INF_OR_NAN},
  {0x0000, 0xffffffff, 0xffffffff, 0},
  {0x3c00, 0xffffffff, 0xfffff800, 0},
  {0x3f80, 0xffffff00, 0x00000000, 0},
  {0x0001, 0xf65d8d9c, 0x00000000, ODD_STEP_NEAR_DOWN_ELSE_UP | ODD_CC_NAN},
  {0x7fff, 0x001e0000, 0x00000000, 0},
  {0x43ff, 0x000e0000, 0x00000000, 0},
  {0x407f, 0x00060000, 0x00000000, 0}
};

// ---- Instructions -------------------------------------------------------------------------------------------------------------
/* ---- instructions ---------------------------------------------------------------------------------------------------- */
// fline(): the F-line exception (vector 11) for everything the core traps.
static void fline(void) { m68ki_exception_1111(); }
// idle(): the first executed FPU instruction ends the NULL state, so FSAVE writes an IDLE frame from then on.
static void idle(void) { fpu_null = 0; }

// fp_general(): the arithmetic instructions, class 000 (register to register) and 010 (memory or Dn to register). The second opcode word w2 is the
// command word: bit 14 = rm (1: the source comes from the effective address), bits 12-10 = source register or source format, bits 9-7 =
// destination register, bits 6-0 = opmode (which operation). pc is the address of the instruction (for FPIAR and the trap frame).
static void fp_general(uint16 w2, uint32 pc) {
  // rm = source from EA; src = source register or format; dst = destination register; op = opmode.
  int rm = (w2 >> 14) & 1, src = (w2 >> 10) & 7, dst = (w2 >> 7) & 7, op = w2 & 0x7f;
  // v = source value, r = result, d = destination register's value (read before anything changes).
  floatx80 v, r, d = REG_FP[dst];
  // Opmodes 0x40 to 0x7F are the 68040's single and double rounding forms; a 68882 does not know them, so they take line F (the exception is FMOVECR, which uses rm = 1 and src = 7 with the ROM offset in the opmode field).
  if (op >= 0x40 && !(rm && src == 7)) { fline(); return; }
  if (rm && src == 7) {  // FMOVECR: load a constant from the on-chip ROM into the destination register; the opmode field is the ROM offset
    // k = ROM offset; md = rounding mode; p = precision.
    int k = op, md = mode(), p = precf();
    // Offsets 0x40 and above in this form are not FMOVECR: line F.
    if (k >= 0x40) { fline(); return; }
    // A pending trap from an earlier instruction is taken before this one.
    if (take_pending()) return;
    // The first executed FPU instruction ends the NULL state; clear the exception bits of this instruction.
    idle(); fl = 0;
    {
      const struct rom_const *rc = rom_find(k);
      u64 m; int ex;
      if (!rc) {  // an offset the ROM does not define: the chip returns whatever sits at that spot
        const struct rom_odd *od = &rom_odd_tab[k <= 0x0a ? k : 0];
        int pr = (REG_FPCR >> 6) & 3, adj = 0; unsigned sr = 0;
        m = ((u64)od->hi << 32) | od->lo; ex = od->se;
        // Measured on the chip: at single precision some of these values come out one step off, depending on the rounding mode.
        if (pr == 1) {
          if (od->flags & ODD_STEP_NEAR_DOWN_ELSE_UP) adj = md == 0 ? -1 : (md == 1 || md == 2) ? 1 : 0;
          if (od->flags & ODD_STEP_DOWN_RP) adj = md == 3 ? -1 : 0;
        }
        // Status bits the chip sets besides the usual condition codes.
        if (od->flags & ODD_CC_INF_OR_NAN) sr = (pr == 1 && (md == 0 || md == 3)) ? FPCC_I : FPCC_NAN;
        if (od->flags & ODD_CC_NAN) sr = FPCC_NAN;
        r = mk(0, ex, m);
        // At single or double precision the value is rounded first (extended exponent range).
        if (p != 64 && m && ex != 0x7fff) r = rnd(p, 1, 0, ex, m, 0);
        // The step is 0x80 in the high mantissa word.
        if (adj) r.low += (u64)(uint32)(adj * 0x80) << 32;
        REG_FP[dst] = r; set_cc(r); REG_FPSR |= sr; status(); trap_check(pc);
        return;
      }
      m = ((u64)rc->hi << 32) | rc->lo; ex = rc->se;
      // The ROM holds the inexact constants rounded one way; the directed modes step the last bit the other way.
      if (rc->dir != ROM_EXACT) {
        fl |= X_INEX2;
        if (rc->dir == ROM_STORED_HIGH && (md == 1 || md == 2)) m--;
        if (rc->dir == ROM_STORED_LOW && md == 3) m++;
      }
      r = mk(0, ex, m);
      if (p != 64 && m) r = rnd(p, 1, 0, xe(r), m, 0);
    }
    REG_FP[dst] = r; set_cc(r); status(); trap_check(pc);
    return;
  }
  // The opmodes that the model computes (the same set the core has in hardware). Everything else, the transcendental functions (FSIN, FCOS, FLOGN, FETOX and so on),
  // FMOD and FREM included, takes line F, so that software which needs them has to emulate them in its F-line handler.
  switch (op) {
  // FMOVE, FINT, FINTRZ, FSQRT, FABS, FNEG, FGETEXP, FGETMAN, FDIV, FADD, FMUL, FSGLDIV, FSCALE, FSGLMUL, FCMP, FTST and the alias opmodes of the same operations.
  case 0x00: case 0x01: case 0x03: case 0x04: case 0x05: case 0x18: case 0x1a: case 0x1b: case 0x1e: case 0x1f:
  case 0x20: case 0x22: case 0x23: case 0x24: case 0x26: case 0x27: case 0x38: case 0x39: case 0x3a: case 0x3b:
  case 0x3c: case 0x3d: case 0x3e: case 0x3f: break;
  // All eight opmodes 0x28 to 0x2F are treated as FSUB.
  default: if (op >= 0x28 && op <= 0x2f) break; fline(); return;
  }
  // Source operand: from the effective address (rm = 1) ...
  if (rm) {
    if (src == 3 || src == 7) { fline(); return; }  // packed decimal source (format 3, or 7 with a dynamic k-factor): line F by design
    if (ea_m == 7 && ea_r == 4 && (src == 2 || src == 5)) { fline(); return; }  // immediate .X or .D operands: line F by design
    // A data register can only be the source of the integer formats and single.
    if (ea_m == 0 && !(src == 0 || src == 1 || src == 4 || src == 6)) { fline(); return; }
    // An address register source and the PC-relative forms 5-7 do not exist.
    if (ea_m == 1 || (ea_m == 7 && ea_r > 4)) { fline(); return; }
    // A pending trap is taken before reading the operand.
    if (take_pending()) return;
    // Read the operand; a form that read_src does not support goes to line F.
    if (!read_src(src, &v)) { fline(); return; }
  // ... or from another FP register.
  } else {
    // (A pending trap comes first here too.)
    if (take_pending()) return;
    // The source register is the one in bits 12-10.
    v = REG_FP[src];
  }
  // From here on the instruction really runs: end the NULL state, clear the exception bits.
  idle(); fl = 0;
  // Pick the operation by opmode.
  switch (op) {
  // FMOVE.
  case 0x00: r = f_mono(v, 0); break;
  // FINT.
  case 0x01: r = f_int(v, 0); break;
  // FINTRZ.
  case 0x03: r = f_int(v, 1); break;
  // FSQRT.
  case 0x04: case 0x05: r = f_sqrt(v); break;
  // FABS.
  case 0x18: r = f_mono(v, 1); break;
  // FNEG (0x1a, and 0x1b).
  case 0x1a: case 0x1b: r = f_mono(v, 2); break;
  // FGETEXP.
  case 0x1e: r = f_getexp(v); break;
  // FGETMAN.
  case 0x1f: r = f_getman(v); break;
  // FDIV.
  case 0x20: r = f_div(d, v, 0); break;
  // FADD.
  case 0x22: r = f_add(d, v, 0); break;
  // FMUL.
  case 0x23: r = f_mul(d, v, 0); break;
  // FSGLDIV.
  case 0x24: r = f_div(d, v, 1); break;
  // FSCALE.
  case 0x26: r = f_scale(d, v); break;
  // FSGLMUL.
  case 0x27: r = f_mul(d, v, 1); break;
  // FCMP and aliases: only the condition codes change, the destination register is not written.
  case 0x38: case 0x39: case 0x3c: case 0x3d: r = f_cmp(d, v); set_cc(r); status(); trap_check(pc); return;
  // FTST and aliases: the condition codes come from the source value itself (a NaN is quieted); no register is written.
  case 0x3a: case 0x3b: case 0x3e: case 0x3f: r = is_nan(v) ? quiet(v) : v; set_cc(r); status(); trap_check(pc); return;
  default: r = f_add(d, v, 1); break;  // FSUB (opmodes 0x28 to 0x2F)
  }
  // When SNAN, OPERR or DZ occur and their trap is enabled, the instruction does not write its result (the destination register stays as it was).
  // Write the destination only when no such trap applies.
  if (!(fl & REG_FPCR & (X_SNAN | X_OPERR | X_DZ))) REG_FP[dst] = r;
  // Condition codes, status, and the check for a trap that is now pending.
  set_cc(r); status(); trap_check(pc);
}

static void fp_store(uint16 w2, uint32 pc) {  // fp_store(): FMOVE FPn,<ea>, class 011. w2: bits 12-10 = destination format, bits 9-7 = source register
  // f = format, src = source register, x = its value.
  int f = (w2 >> 10) & 7, src = (w2 >> 7) & 7; floatx80 x = REG_FP[src]; uint32 a;
  // Packed decimal formats are trapped.
  if (f == 3 || f == 7) { fline(); return; }
  // Not allowed: an address register, a PC-relative or immediate destination (not alterable), and a data register with a format other than the integer formats and single.
  if (ea_m == 1 || (ea_m == 7 && ea_r > 1) || (ea_m == 0 && !(f == 0 || f == 1 || f == 4 || f == 6))) { fline(); return; }
  // A pending trap is taken first.
  if (take_pending()) return;
  // The instruction runs.
  idle(); fl = 0;
  // Data register destination: ...
  if (ea_m == 0) {
    // d = the destination data register.
    uint32* d = &REG_D[ea_r];
    // ... long and single replace the whole register ...
    switch (f) { case 0: *d = (uint32)to_int(x, 32); break; case 1: *d = (uint32)to_sd(x, 0); break;
                 // ... word and byte only the low part.
                 case 4: *d = (*d & 0xffff0000) | ((uint32)to_int(x, 16) & 0xffff); break;
                 case 6: *d = (*d & 0xffffff00) | ((uint32)to_int(x, 8) & 0xff); break; }
  // Memory destination: ...
  } else {
    // ... the address (this also steps An for (An)+ and -(An)) ...
    a = ea_addr(fsz[f]);
    // ... and the converted value is written by format: long, single, word, byte, double (two long words) ...
    switch (f) {
    case 0: m68ki_write_32(a, (uint32)to_int(x, 32)); break;
    case 1: m68ki_write_32(a, (uint32)to_sd(x, 0)); break;
    case 4: m68ki_write_16(a, (uint32)to_int(x, 16) & 0xffff); break;
    case 6: m68ki_write_8(a, (uint32)to_int(x, 8) & 0xff); break;
    case 5: { u64 v = to_sd(x, 1); m68ki_write_32(a, (uint32)(v >> 32)); m68ki_write_32(a + 4, (uint32)v); break; }
    // ... or extended (three long words: sign and exponent in the top half of the first).
    case 2: { floatx80 v = to_x(x); m68ki_write_32(a, (uint32)v.high << 16); m68ki_write_32(a + 4, (uint32)(v.low >> 32)); m68ki_write_32(a + 8, (uint32)v.low); break; }
    }
  }
  // Update the status register and check for traps.
  status(); trap_check(pc);
}

static void fp_ctl(uint16 w2) {  // fp_ctl(): FMOVE and FMOVEM between the control registers FPCR, FPSR, FPIAR and a data or address register, memory or an immediate
  // dir = bit 13 (1: register to EA); lst = register select bits 12-10 (4 FPCR, 2 FPSR, 1 FPIAR), an empty list counts as FPIAR; n = how many registers are listed.
  int dir = (w2 >> 13) & 1, lst = ((w2 >> 10) & 7) ? (w2 >> 10) & 7 : 1, n = (lst & 4 ? 1 : 0) + (lst & 2 ? 1 : 0) + (lst & 1 ? 1 : 0);
  // a = address, i = loop counter.
  uint32 a; int i;
  // An empty register list means FPIAR (68882 behavior).
  // A data register can hold exactly one control register.
  if (ea_m == 0 && n != 1) { fline(); return; }
  // An address register only FPIAR.
  if (ea_m == 1 && lst != 1) { fline(); return; }
  // Immediate data is only a source, and for one register.
  if (ea_m == 7 && ea_r == 4 && (dir || n != 1)) { fline(); return; }
  // Control registers cannot be stored to PC-relative addresses.
  if (dir && ea_m == 7 && ea_r > 1) { fline(); return; }
  // Modes 7 with registers 5 to 7 do not exist.
  if (ea_m == 7 && ea_r > 4) { fline(); return; }
  // A pending trap is taken first.
  if (take_pending()) return;
  // Mark the FPU as used (no NULL frame any more).
  idle();
  // Register operand or immediate data: ...
  if (ea_m <= 1 || (ea_m == 7 && ea_r == 4)) {
    // rp = the integer register (Dn or An); for the immediate form the data is read below.
    uint32* rp = ea_m == 0 ? &REG_D[ea_r] : &REG_A[ea_r];
    // Store to the register: pick the control register by the list bit.
    if (dir) *rp = (lst & 4) ? REG_FPCR : (lst & 2) ? REG_FPSR : REG_FPIAR;
    else {
      // Load from the register or from the instruction stream (immediate).
      uint32 v = ea_m == 7 ? OPER_I_32() : *rp;
      // FPCR keeps bits 15 to 4 (exception enables, precision, rounding); FPSR keeps bits 27 to 3 (condition codes, quotient, exceptions, accrued bits); FPIAR is all 32 bits.
      if (lst & 4) REG_FPCR = v & 0xfff0; else if (lst & 2) REG_FPSR = v & 0x0ffffff8; else REG_FPIAR = v;
    }
    return;
  }
  // Memory operand: pre-decrement moves An down by 4 bytes per register first; post-increment moves up afterwards; other modes use the effective address.
  if (ea_m == 4) { REG_A[ea_r] -= 4 * n; a = REG_A[ea_r]; }
  else if (ea_m == 3) { a = REG_A[ea_r]; REG_A[ea_r] += 4 * n; }
  else a = ea_addr(4);
  // Transfer in the order FPCR, FPSR, FPIAR at increasing addresses; i = 2 is FPCR, 1 is FPSR, 0 is FPIAR.
  for (i = 2; i >= 0; i--) if (lst & (1 << i)) {
    // Store to memory.
    if (dir) m68ki_write_32(a, i == 2 ? REG_FPCR : i == 1 ? REG_FPSR : REG_FPIAR);
    // Load from memory, with the same masking.
    else { uint32 v = m68ki_read_32(a); if (i == 2) REG_FPCR = v & 0xfff0; else if (i == 1) REG_FPSR = v & 0x0ffffff8; else REG_FPIAR = v; }
    a += 4;
  }
}

static void fp_movem(uint16 w2) {  // fp_movem(): FMOVEM of FP data registers, class 110 and 111. w2: bit 13 = direction (1: registers to memory), bits 12-11 = mode (static or dynamic list, pre-decrement or other), bits 7-0 = register list
  // dir = direction, md = mode, lst = list byte, n = number of registers.
  int dir = (w2 >> 13) & 1, md = (w2 >> 11) & 3, lst = w2 & 0xff, i, n = 0;
  uint32 a;
  if (md & 1) { fline(); return; }  // a dynamic list (the list is in a data register) is trapped
  // Data registers, address registers and immediate data are not valid.
  if (ea_m <= 1 || (ea_m == 7 && ea_r == 4)) { fline(); return; }
  // Loading from post-increment is fine, but storing to it or to PC-relative addresses is not.
  if (dir && (ea_m == 3 || (ea_m == 7 && ea_r > 1))) { fline(); return; }
  // Loading from memory through a pre-decrement address is not valid.
  if (!dir && ea_m == 4) { fline(); return; }
  // Modes 7 with registers 4 to 7 are not valid.
  if (ea_m == 7 && ea_r > 3) { fline(); return; }
  // The register list has two layouts. With the pre-decrement list type, bit i stands for FPi; otherwise bit 7 stands for FP0. The list type and the effective address mode can disagree, in which case the registers go in descending order.
  // A pending trap is taken first.
  if (take_pending()) return;
  // Mark the FPU as used.
  idle();
  // Count the registers in the list.
  for (i = 0; i < 8; i++) n += (lst >> i) & 1;
  // Pre-decrement: An moves down by 12 bytes per register first.
  if (ea_m == 4) { REG_A[ea_r] -= 12 * n; a = REG_A[ea_r]; }
  // Post-increment: An is used and then moves up.
  else if (ea_m == 3) { a = REG_A[ea_r]; REG_A[ea_r] += 12 * n; }
  // Other modes: the effective address.
  else a = ea_addr(4);
  // Walk through the eight registers.
  for (int j = 0; j < 8; j++) {
    // dsc = the transfer runs from FP7 down to FP0 when exactly one of "pre-decrement list type" and "pre-decrement EA" is true.
    int dsc = (md == 0) != (ea_m == 4), on;
    // i = the register for this step.
    i = dsc ? 7 - j : j;
    // on = is this register in the list (the two list layouts mirror each other).
    on = md == 0 ? (lst >> i) & 1 : (lst >> (7 - i)) & 1;
    // Skip registers that are not listed.
    if (!on) continue;
    // To memory: the 12-byte image (sign and exponent word, 16 unused bits, 64-bit mantissa).
    if (dir) { floatx80 v = REG_FP[i]; m68ki_write_32(a, (uint32)v.high << 16); m68ki_write_32(a + 4, (uint32)(v.low >> 32)); m68ki_write_32(a + 8, (uint32)v.low); }
    // From memory: assemble the register from the same three words.
    else { uint32 h = m68ki_read_32(a); REG_FP[i] = mk(h >> 31, (h >> 16) & 0x7fff, ((u64)m68ki_read_32(a + 4) << 32) | m68ki_read_32(a + 8)); }
    // Next register image.
    a += 12;
  }
}

// m68040_fpu_op0(): the handler Musashi calls for the opcodes F200-F27F, F280-F3BF (FPU general, FScc/FDBcc/FTRAPcc and FBcc). Opcode bits 11-9 give the coprocessor
// id (001 = the FPU, tools/musashi_patch.py routes the other ids to line F), bits 8-6 the type: 000 general, 001 FScc/FDBcc/FTRAPcc, 010 FBcc with a word displacement, 011 FBcc with a long displacement.
void m68040_fpu_op0(void) {
  USE_CYCLES(4);  // Musashi's cycle table gives 0 cycles for these opcodes on a 68020, so m68k_execute(1) would not stop after this instruction; four cycles make it stop
#ifdef P68_NOFPU
  m68ki_exception_1111(); return;  // the core built without the FPU takes the F-line exception for every FPU opcode
#endif
  // pc = address of the instruction (REG_PPC); the effective address fields of the opcode are split into mode and register.
  uint32 pc = REG_PPC; uint16 w2;
  ea_m = (REG_IR >> 3) & 7; ea_r = REG_IR & 7;
  // Bits 7-6 of the opcode pick the group.
  switch ((REG_IR >> 6) & 3) {
  case 0:
    // Group 0: a general instruction, the command word follows the opcode.
    w2 = OPER_I_16();
    // The top three bits of the command word pick the class.
    switch (w2 >> 13) {
    // 000 and 010: arithmetic, register to register or EA to register.
    case 0: case 2: fp_general(w2, pc); return;
    // 011: FMOVE FPn,<ea>.
    case 3: fp_store(w2, pc); return;
    // 100 and 101: FMOVE and FMOVEM of the control registers.
    case 4: case 5: fp_ctl(w2); return;
    // 110 and 111: FMOVEM of the data registers.
    case 6: case 7: fp_movem(w2); return;
    // 001 and anything else are not valid: line F.
    default: fline(); return;
    }
  case 1: {  // group 1: FScc, FDBcc and FTRAPcc, told apart by the effective address mode
    int c;
    if (ea_m == 1) {  // mode 1 is FDBcc (decrement and branch on FPU condition)
      // w2 = condition word, disp = 16-bit displacement.
      sint16 disp; w2 = OPER_I_16(); disp = (sint16)OPER_I_16();
      // The condition word has only 6 condition bits; anything above is not valid.
      if (w2 & 0xffc0) { fline(); return; }
      // Pending trap first.
      if (take_pending()) return;
      // Mark the FPU as used and evaluate the condition (a BSUN trap ends the instruction).
      idle(); c = fcond(w2 & 0x3f); if (c < 0) return;
      // Condition false: decrement the low word of Dn and branch when it did not run out (-1). The displacement is relative to the displacement word, pc + 4.
      if (!c) { uint32 v = (REG_D[ea_r] - 1) & 0xffff; REG_D[ea_r] = (REG_D[ea_r] & 0xffff0000) | v; if (v != 0xffff) REG_PC = pc + 4 + disp; }
      return;
    }
    if (ea_m == 7 && ea_r >= 2) {  // mode 7 with register 2, 3 or 4 is FTRAPcc: no operand, a word or a long operand
      // Registers above 4 are not valid.
      if (ea_r > 4) { fline(); return; }
      // Read the condition word.
      w2 = OPER_I_16();
      // Condition bits only.
      if (w2 & 0xffc0) { fline(); return; }
      // Skip the operand word or long word that follows (its value is not used).
      if (ea_r == 2) OPER_I_16(); else if (ea_r == 3) OPER_I_32();
      // Pending trap first.
      if (take_pending()) return;
      // Mark the FPU as used.
      idle();
      // Condition true: take the TRAPV exception (vector 7).
      { int t = fcond(w2 & 0x3f); if (t > 0) m68ki_exception_trap(EXCEPTION_TRAPV); }
      return;
    }
    // Everything else is FScc: set a byte to 0xFF (true) or 0x00 (false).
    w2 = OPER_I_16();
    // The condition word must have only the 6 condition bits.
    if (w2 & 0xffc0) { fline(); return; }
    // Non-alterable destinations (PC-relative, immediate) are not valid.
    if (ea_m == 7 && ea_r > 1) { fline(); return; }
    // Pending trap first.
    if (take_pending()) return;
    // Mark the FPU as used, evaluate the condition (BSUN trap ends the instruction), turn it into 0xFF or 0x00.
    idle(); c = fcond(w2 & 0x3f); if (c < 0) return; c = c ? 0xff : 0;
    // Data register: replace the low byte.
    if (ea_m == 0) REG_D[ea_r] = (REG_D[ea_r] & 0xffffff00) | c;
    // Memory: write the byte at the effective address.
    else m68ki_write_8(ea_addr(1), c);
    return;
  }
  case 2: case 3: {  // groups 2 and 3: FBcc with a 16-bit or 32-bit displacement (bit 6 of the opcode tells which)
    // The displacement is read from the instruction stream.
    sint32 disp = ((REG_IR >> 6) & 1) ? (sint32)OPER_I_32() : (sint16)OPER_I_16();
    // The condition is the low 6 opcode bits; conditions 0x20 to 0x3F act as 0x00 to 0x1F (fcond ignores bit 5).
    // Pending trap first.
    if (take_pending()) return;
    // Mark the FPU as used.
    idle();
    // If the condition is true, branch: the displacement is relative to the address behind the opcode word (pc + 2).
    { int t = fcond(REG_IR & 0x3f); if (t > 0) REG_PC = pc + 2 + disp; }
    return;
  }
  }
}

// m68040_fpu_op1(): FSAVE and FRESTORE (opcode bits 8-6 = 100 and 101). They save and restore the internal state of the FPU through a state frame in memory.
// The frames are those of the 68882: the NULL frame is the long word 0x00380000 (the FPU is in its reset state, nothing to save), and the IDLE frame is
// 0x3C bytes (60) long, with the header 0x1F380000 first (version 0x1F, 0x38 bytes of state) and a BIU flags word last. The IDLE frame here holds zeros apart
// from those two words, because no program can see the internal state; WinUAE's frame keeps the last command word and operand.
void m68040_fpu_op1(void) {
  // Same cycle correction as in m68040_fpu_op0().
  USE_CYCLES(4);
#ifdef P68_NOFPU
  // The core built without the FPU: line F.
  m68ki_exception_1111(); return;                /* core built without the FPU: line F */
#endif
  // i = loop counter, a = address, h = header word read by FRESTORE.
  int i; uint32 a, h;
  // ea_m and ea_r = effective address fields of the opcode.
  ea_m = (REG_IR >> 3) & 7; ea_r = REG_IR & 7;
  if (((REG_IR >> 6) & 3) == 0) {  // FSAVE: the destination must be control alterable ((An), d16(An), d8(An,Xn), absolute) or pre-decrement
    // sz = size of the frame: 4 bytes in the NULL state, 60 bytes otherwise.
    int sz = fpu_null ? 4 : 0x3C;
    // Other effective address modes are not valid.
    if (!(ea_m == 2 || ea_m == 4 || ea_m == 5 || ea_m == 6 || (ea_m == 7 && ea_r <= 1))) { fline(); return; }
    // Pre-decrement: An moves down by the frame size first. Other modes use the effective address.
    if (ea_m == 4) { REG_A[ea_r] -= sz; a = REG_A[ea_r]; } else a = ea_addr(4);
    // NULL state: one long word, the NULL frame.
    if (fpu_null) { m68ki_write_32(a, 0x00380000); return; }
    // IDLE frame: the header ...
    m68ki_write_32(a, 0x1F380000);
    // ... zeros for the internal state ...
    for (i = 4; i < 0x38; i += 4) m68ki_write_32(a + i, 0);
    // ... and the BIU word at offset 0x38.
    m68ki_write_32(a + 0x38, 0x5C0EFFFF);
    // FSAVE clears a pending arithmetic exception (it is part of the saved state).
    fpu_pend = 0;
    return;
  }
  if (((REG_IR >> 6) & 3) == 1) {  // FRESTORE: the source must be control addressable or post-increment
    // sz = the state size announced by the frame header.
    int sz;
    // Other effective address modes are not valid.
    if (!(ea_m == 2 || ea_m == 3 || ea_m == 5 || ea_m == 6 || (ea_m == 7 && ea_r <= 3))) { fline(); return; }
    // Post-increment reads from An (and steps it below); other modes use the effective address.
    a = ea_m == 3 ? REG_A[ea_r] : ea_addr(4);
    // The header word: version in the top byte, state size in the next.
    h = m68ki_read_32(a);
    // A version byte of 0 is a NULL frame: ...
    if ((h >> 24) == 0) {
      // ... back to the reset state: NULL, all control registers cleared, ...
      fpu_null = 1; REG_FPCR = 0; REG_FPSR = 0; REG_FPIAR = 0;
      // ... all eight data registers set to the default NaN (as after a reset).
      for (i = 0; i < 8; i++) REG_FP[i] = dnan();
      // Post-increment: An moves up by the 4 bytes of the NULL frame.
      if (ea_m == 3) REG_A[ea_r] += 4;
      return;
    }
    // Size of the state area in bytes.
    sz = (h >> 16) & 0xff;
    // Only version 0x1F with the sizes 0x18 and 0x38 (68881 and 68882 idle) and 0xB4 and 0xD4 (busy frames) are accepted; anything else is a format error exception (vector 14).
    if ((h >> 24) != 0x1F || !(sz == 0x18 || sz == 0x38 || sz == 0xB4 || sz == 0xD4)) { vec0(14); return; }
    // Not NULL any more.
    fpu_null = 0;
    // Idle frames carry the BIU flags word at the end of the state area; it tells whether an exception was pending when the frame was saved.
    if (sz == 0x18 || sz == 0x38) {
      // biu = the BIU word, x = the enabled exceptions in the restored status and control registers.
      uint32 biu = m68ki_read_32(a + sz), x = REG_FPSR & REG_FPCR & 0xff00;
      // When bit 27 of the BIU word is clear an exception was pending: it becomes pending again, by the same priority as in trap_check(), ...
      fpu_pend = (biu & 0x08000000) ? 0 : (x & X_BSUN) ? 48 : (x & X_SNAN) ? 54 : (x & X_OPERR) ? 52 : (x & X_OVFL) ? 53 :
                 // ... otherwise nothing is pending (the BIU bit 27 clear means pending).
                 (x & X_UNFL) ? 51 : (x & X_DZ) ? 50 : x ? 49 : 0;   /* BIU bit 27 clear: exception pending */
    }
    // Post-increment: An moves up by the whole frame, the state area plus the 4 byte header.
    if (ea_m == 3) REG_A[ea_r] += sz + 4;
    return;
  }
  // Any other opcode form: line F.
  fline();
}

// ---- Bench interface -----------------------------------------------------------------------------------------------------------
// The lockstep bench reaches the FPU state through three functions, which it declares itself in lockstep.cpp.
// p68_fpu_reset(): the state after a hardware reset, called before every program.
void p68_fpu_reset(void) {
  int i;
  // Control registers zero, NULL state, nothing pending.
  REG_FPCR = 0; REG_FPSR = 0; REG_FPIAR = 0; fpu_null = 1; fpu_pend = 0;
  // All eight data registers hold the default NaN.
  for (i = 0; i < 8; i++) REG_FP[i] = dnan();
}
// p68_fpu_get(): read data register i as the high 16 bits (sign and exponent) and the 64-bit mantissa.
#ifdef P68_NOFPU  // core built without the FPU: every register reads zero
void p68_fpu_get(int i, unsigned* hi, unsigned long long* lo) { (void)i; *hi = 0; *lo = 0; }
unsigned p68_fpu_ctl(int i) { (void)i; return 0; }
#else
// With the FPU: the register values.
void p68_fpu_get(int i, unsigned* hi, unsigned long long* lo) { *hi = REG_FP[i].high; *lo = REG_FP[i].low; }
// p68_fpu_ctl(): FPCR (0), FPSR (1) or FPIAR (2).
unsigned p68_fpu_ctl(int i) { return i == 0 ? REG_FPCR : i == 1 ? REG_FPSR : REG_FPIAR; }
#endif

// The PMMU code of Musashi (m68kmmu.h) refers to these functions even when the PMMU is switched off (M68K_EMULATE_PMMU=0 in tools/lockstep.sh), so empty
// stubs make the file compile and link. They are never called.
static uint32 READ_EA_32(int ea) { (void)ea; return 0; }
static uint64 READ_EA_64(int ea) { (void)ea; return 0; }
static void WRITE_EA_32(int ea, uint32 v) { (void)ea; (void)v; }
static void WRITE_EA_64(int ea, uint64 v) { (void)ea; (void)v; }
// The same for the error function of m68kmmu.h.
static void fatalerror(const char* f, ...) { (void)f; }   /* m68kmmu.h (PMMU off) */
