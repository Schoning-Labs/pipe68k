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
// The lockstep bench: the RTL core, simulated cycle by cycle in Verilator, runs the same random programs as Musashi (a software 68020, the reference CPU), and the two are
// compared after every instruction. The first difference stops the run and prints the seed, the instruction and both values, which is usually enough to find the fault in the core.
//
// This is the main test of the project. It found the six faults of the first pipeline sample (ROL/ROR changing X, ASL not setting V, shift counts above the operand size, ADDA.W from
// memory loading a long word, and two cases of the order in which (An)+ and the destination address see a register), a stale-refill fault of the data cache, and many later ones;
// one run found an interrupt that could never be taken inside a polling loop (program 1876 of 2000). The check before every merge is 2000 programs of 400 instructions with all
// groups (about 2.1 million instructions, 5.24 million in the runs that include the FPU groups). The quick check after a change is 300 programs of 200 instructions with seeds 1 and
// 7000 with the FPU and seed 1 without. A directed run of the polling-loop group alone (200 programs of 300 instructions) executed 15.1 million instructions with 179,224 interrupts.
//
// usage: lockstep [programs] [seed] [length] [trace] [groups]
//   programs  how many random programs to run (default 100); the program n uses the seed (seed + n), so any failure can be repeated alone
//   seed      first seed (default 1)
//   length    instructions per program (default 200)
//   trace     1 = print every retired instruction with A0-A7 and SR, 2 = also D0-D7, 3 = also the stored bytes, 4 = also the FPU registers
//   groups    bit mask of generator groups (decimal or 0x hex, default all; see the table of groups below)
//   lockstep perf [trace]   runs the fixed performance test instead (clocks per instruction), see perf_build()
//   P68_AROS=rom,ext        environment variable: runs an AROS Kickstart image from reset against Musashi instead of random programs
//   Further environment variables: P68_CYC (clock limit of the AROS run), P68_CACHI (value for the cac_hi input), P68_SLOW=1 (CPU speed mode, same results but slower)
// Exit code 0 means every program matched. tools/lockstep.sh builds the bench (Verilator, Musashi with the patches of tools/musashi_patch.py and the FPU reference of
// sim/lockstep/fpu_ref.c) and runs it.
//
// How the comparison works. The core has a retire port, a set of simulation-only outputs that report every committed micro-operation (uop) and every retired instruction.
// A uop's report carries the register write it made, the two address register updates of the address generation stage, and its stores (address, data, byte enables). The bench keeps
// its own copy of D0-D7 and A0-A7 and of the three stack pointer banks and applies those writes as they come. When an instruction retires (ret_v for lane A, ret2_v for lane B of a
// dual-issue pair; ret2_first says that lane B holds the older instruction), the bench runs Musashi for one instruction and compares: PC, D0-D7, A0-A7, SR, the three stack pointers,
// VBR, SFC, DFC, CACR, CAAR, the FPU registers and control registers, and every byte stored by either side. An interrupt entry also counts as an instruction (ret_irq): the bench raises the
// interrupt level on Musashi at the same instruction boundary and checks that Musashi takes it.
//
// What each side sees of memory. Both have a 16 MB flat memory (24-bit addresses; mem_rtl for the core and mem_ref for Musashi) that starts with the same random contents and the program.
// Musashi reads and writes its copy directly through the callbacks below. The core talks to a memory port model with a random latency per request and per beat
// (writes too), so that timing-dependent faults show up. Chip-bus DMA writes (group 31) and self-modifying code (group 32) make the two memories change behind the CPU's
// back, and the timing rule for that is described at the memory model in run_program().
//
// The random programs. A generator (struct Gen) writes a program of the requested length from a table of groups, each a family of instructions (MOVE, ALU, shifts, bit fields,
// exceptions, FPU and so on) with a weight. The generator follows a few rules so that every program is valid for both sides and ends: base registers A0-A5 and A7 always hold
// 4-byte aligned data addresses (0x200000-0xDFFFFF), and move only in aligned, bounded steps; A6 is a scratch register that is never used as a base; the code area below 0x10000
// is read only (except for the self-modifying code group); branches go forward only (loops are built with a bounded counter); instructions inside a multi-instruction unit are never
// branch targets. A program that breaks a rule stops with "generator error", so it is never mistaken for a fault of the core.
// Musashi is not perfect, and a few generator choices keep away from things Musashi does differently from a real 68020 (for example DIVS.W of 0x80000000 by -1, CHK2.B with
// (d16,PC), CAS2.W with an address register). Where the real chip is known (from WinUAE's cputester data) the patched Musashi follows it instead, see tools/musashi_patch.py.
#include "Vp68_core.h"
// Verilator model of the core (generated from rtl/core) and Verilator's library header.
#include "verilated.h"
// Standard C and C++ headers.
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <map>
// The bench is C++, Musashi is C: these are the C functions it uses. m68k.h is Musashi's public header.
extern "C" {
#include "m68k.h"
// Defined in musashi_glue.c: take a pending interrupt now, without running an instruction.
void m68k_glue_check_irq(void);   // musashi_glue.c: take a pending interrupt now
void p68_fpu_reset(void);  // defined in fpu_ref.c: reset the reference FPU; the next two read FPn (sign and exponent, mantissa) and a control register (0 FPCR, 1 FPSR, 2 FPIAR)
void p68_fpu_get(int i, unsigned* hi, unsigned long long* lo);
unsigned p68_fpu_ctl(int i);
}

// Verilator asks for the simulated time; the bench has no use for it.
double sc_time_stamp() { return 0; }

// MEMSZ = 16 MB, MASK = the address mask, CODE_END = the code area below this address is read only for the program.
static const uint32_t MEMSZ = 1u << 24, MASK = MEMSZ - 1, CODE_END = 0x10000;
// mem_rtl = the core's memory, mem_ref = Musashi's memory.
static std::vector<uint8_t> mem_rtl(MEMSZ), mem_ref(MEMSZ);
// ref_wr = the bytes Musashi wrote during the current instruction (address and value), compared with the core's stores.
static std::map<uint32_t, uint8_t> ref_wr;   // bytes Musashi wrote during one instruction
// gen_error = text of a generator problem, found while Musashi runs (for example a write into the code area).
static std::string gen_error;
// trace = the trace level from the command line.
static int trace = 0;
static long n_irq, n_user, n_swap, n_msp, n_stop, n_pair, n_pold;  // Counters for the coverage line printed with PASS: interrupts taken, user-mode instructions, stack pointer swaps, instructions with M set, STOPs, pairs, and pairs with lane B older.

// The generator groups. Bit n of the groups argument switches group n on. A new instruction group adds an entry here and in GROUPS below. The groups are numbered in the order they were added, and
// the numbers are the bit positions: 0-20 the integer instruction families, 21-23 system instructions, traps and interrupts, 24-26 program flow, 27-28 multiply and divide, 29-30 bit fields,
// 31-32 DMA and self-modifying code, 33-35 full-format addressing, long instructions and CAS, 36-38 the FPU, 39 the polling loop.
// The first 21 groups are the integer instruction families, MOVEQ to MOVEP.
enum { G_MOVEQ, G_MOVE, G_ALU_EA_DN, G_ALU_DN_EA, G_ADDA, G_ADDQ, G_BCC, G_LEA, G_SHIFT, G_MOVEM,
       G_IMMOP, G_UNARY, G_EXT, G_SCC, G_DBCC, G_ADDX, G_BCD, G_BIT, G_MSHIFT, G_PACK, G_MOVEP,
       G_SYS, G_TRAP, G_IRQ, G_CALL, G_STACK, G_LOOP,  // system and program flow: traps, interrupts, calls, stack frames, loops
       G_MUL, G_DIV,  // multiply and divide
       G_BF_R = 29, G_BF_M,  // bit fields in registers (29) and in memory (30)
       G_DMA = 31, G_SMC,  // DMA writes done by the bench (31) and self-modifying code (32)
       G_EA = 33, G_LONG, G_CAS,  // full-format effective addresses (33), long instructions (34), CAS and CAS2 (35)
       G_FPU = 36, G_FPC, G_FPM,  // FPU: arithmetic and FMOVE (36), conditionals (37), FMOVEM, control registers and FSAVE (38)
       G_ULOOP = 39, G_COUNT };  // a tight loop with a load from an uncached address (group 39): the test for interrupt starvation (an interrupt must still be taken within a few instructions)

// GROUPS[]: name and weight of every group. The weight is the chance of the group relative to the others when the generator picks the next instruction (a weight of 0, as for the
// interrupt group, means the group generates no instructions; it only makes the bench raise interrupts).
static const struct { const char* name; int weight; } GROUPS[G_COUNT] = {
  {"MOVEQ", 1}, {"MOVE", 2}, {"ALU <ea>,Dn", 2}, {"ALU Dn,<ea>", 1}, {"ADDA/SUBA/CMPA", 1}, {"ADDQ/SUBQ", 1},
  {"Bcc", 1}, {"LEA", 1}, {"shift", 2}, {"MOVEM", 1},
  {"ADDI..CMPI", 1}, {"NEG NOT CLR TST TAS NOP", 1}, {"EXT SWAP EXG", 1}, {"Scc", 1}, {"DBcc loop", 1}, {"ADDX SUBX CMPM", 1},
  {"ABCD SBCD NBCD", 1}, {"bit ops", 1}, {"memory shifts", 1}, {"PACK UNPK", 1}, {"MOVEP", 1},
  {"SR/CCR, MOVEC, MOVE USP, MOVES", 2}, {"traps, CHK/CHK2, illegal, STOP, RESET, RTE", 2},
  {"interrupts (bench, no instructions)", 0},
  {"JMP JSR BSR RTS RTD RTR", 2}, {"PEA LINK UNLK", 1}, {"backward branches, loops", 1},  // program flow: calls and returns, stack frames, backward branches and loops
  {"MULU MULS", 2}, {"DIVU DIVS", 3},
  {"bit fields, register", 1}, {"bit fields, memory", 2},
  {"DMA (bench writes, DMA window loads)", 1}, {"self-modifying code", 1},  // DMA window loads and self-modifying code
  {"full-format EAs", 3}, {"long instructions, MOVE destinations, BTST Dn,#imm", 2}, {"CAS CAS2", 2},  // full-format effective addresses, long instructions (and the MOVE destinations they need), CAS and CAS2
  {"FPU arithmetic, FMOVE", 4}, {"FPU conditionals", 2}, {"FMOVEM, FPU control registers, FSAVE", 2},  // the three FPU groups
  {"loops polling an uncached word", 1}};
// ---- The group mask is 64 bits wide, because the group numbers go above 31 ----
// G_BITS = number of groups; G_ALL = a mask with every group on.
static const int G_BITS = G_COUNT;
static const uint64_t G_ALL = (1ull << G_COUNT) - 1;
// gweight(): the weight of group i.
static int gweight(int i) { return GROUPS[i].weight; }
// Counters for the PASS line: DMA writes injected, DMA window loads retired, stores into code retired.
static long n_dma, n_probe, n_smc;                 // DMA writes injected, DMA window loads retired, stores into code retired
// Counters for branch prediction events of the core (taken, predicted, not predicted and so on), and the number of clocks.
static long n_br, n_brp, n_brpm, n_brnm, n_btc, n_ret, n_retm, n_cyc;   // prediction events (ret_bp), clocks
// DMA windows: one cached and one uncached (bit 15 of unc_map covers 0xF00000-0xFFFFFF, and absolute word accesses land there too). No other generated access reaches them,
// and loads from them are single uops (see the DMA timing rule in run_program()).
// DMA windows: one cached, one uncached (unc_map bit 15: 0xF00000-0xFFFFFF; abs.W accesses land there too). No other
// generated access reaches them, and their loads are single uops (see the DMA timing rule in run_program).
// DMA_C = cached window, DMA_U = uncached window, DMA_SZ = size of each (512 bytes).
static const uint32_t DMA_C = 0xE04000, DMA_U = 0xF04000, DMA_SZ = 0x200;
// unc_map value given to the core: bit 15 = the 1 MB regions at 0xF00000 are uncached.
static const uint16_t UNC_MAP = 0x8000;
// dma_rd(): count loads that read a DMA window (called by the Musashi read callbacks).
static void dma_rd(unsigned int a) { a &= MASK; if (a - DMA_C < DMA_SZ || a - DMA_U < DMA_SZ) n_probe++; }
// ---- end of the DMA definitions ----
// groups = the groups that are switched on (command line argument 5).
static uint64_t groups = G_ALL;

// rd(): read n bytes (big endian) from a memory vector.
static uint32_t rd(const std::vector<uint8_t>& m, uint32_t a, int n) {
  uint32_t v = 0;
  for (int i = 0; i < n; i++) v = (v << 8) | m[(a + i) & MASK];
  return v;
}
// AROS mode (environment variable P68_AROS=rom.bin,ext.bin): the core and Musashi run the Kickstart image of the AROS operating system from reset, on the same flat memory. The ROM is at 0xF80000, the
// extended ROM at 0xE00000, the reset longwords at address 0, and the uncached map is the one of the A1200. No interrupts and no DMA are generated.
// The beam position registers ($DFF004-$DFF007, VPOSR and VHPOSR of the Amiga chipset) count per access, in the same way on both sides, so that the waiting loops in the ROM end.
// aros = the AROS mode is on.
static bool aros = false;
// io_cnt = the access count of the beam position, one for each side (0 = Musashi, 1 = the core).
static uint32_t io_cnt[2];
// io_byte(): in AROS mode, the byte at an address in the beam position window comes from a counter instead of from memory. Returns false for every other byte.
static bool io_byte(int side, uint32_t a, uint8_t& v, bool first) {
  a &= MASK;
  // Only in AROS mode and only for the four bytes $DFF004-$DFF007.
  if (!aros || a < 0xDFF004 || a > 0xDFF007) return false;
  // Count one access per read (not per byte) of this register group; first = this is the first byte of the access.
  if (first) io_cnt[side]++;
  // c = access count, vp = vertical position (312 lines, one new line every 8 accesses), hp = horizontal position.
  uint32_t c = io_cnt[side], vp = (c >> 3) % 312, hp = (c * 13) & 0xff;
  // w = the register value as the chipset gives it: bit 16 = V8, bits 15-8 = V7-V0, bits 7-0 = H8-H1.
  uint32_t w = (vp >> 8) << 16 | (vp & 0xff) << 8 | hp;          // VPOSR low bit = V8, VHPOSR = V7-V0, H8-H1
  // Pick the byte that was asked for.
  v = (w >> (8 * (7 - (a - 0xDFF000)))) & 0xff;
  return true;
}
// AROS mode: above 16 MB there is nothing (the A1200 adapter before autoconfig): reads give 0, writes are dropped. Musashi runs as a full 68020 with 32-bit addresses there, as the core does.
// hole(): true for an address above 16 MB in AROS mode.
static bool hole(uint32_t a) { return aros && (a >> 24) != 0; }
// rd_io(): read n bytes from a memory, with the beam position window and the hole handled.
static uint32_t rd_io(int side, const std::vector<uint8_t>& m, uint32_t a, int n) {
  if (hole(a)) return 0;
  uint32_t v = 0; bool first = true;
  for (int i = 0; i < n; i++) { uint8_t b; if (io_byte(side, a + i, b, first)) first = false; else b = m[(a + i) & MASK]; v = (v << 8) | b; }
  return v;
}
// wr_ref(): Musashi's write: apply it to its memory and remember the bytes for the comparison.
static void wr_ref(uint32_t a, int n, uint32_t v) {
  // Writes above 16 MB in AROS mode are dropped.
  if (hole(a)) return;
  if (!aros && (a & MASK) < CODE_END && !(((groups >> G_SMC) & 1) && (a & MASK) < 0x3000)) gen_error = "write into the code area";  // a write into the code area (below 0x10000) is a generator error, except in the self-modifying code group, which may write below 0x3000 on purpose
  // Count stores into the code part (used for the PASS line).
  if ((a & MASK) < 0x3000) n_smc++;
  // Store the n bytes, last byte first (big endian: the most significant byte is at the lowest address).
  for (int i = n - 1; i >= 0; i--) { uint32_t x = (a + i) & MASK; mem_ref[x] = v & 0xff; ref_wr[x] = v & 0xff; v >>= 8; }
}
extern "C" {  // The Musashi memory callbacks (C linkage). Musashi is built with separate reads for data, immediate data and PC-relative data (M68K_SEPARATE_READS=1), so that each kind has its own function.
// Data reads count loads from the DMA windows.
unsigned int m68k_read_memory_8(unsigned int a) { dma_rd(a); return rd_io(0, mem_ref, a, 1); }
unsigned int m68k_read_memory_16(unsigned int a) { dma_rd(a); return rd_io(0, mem_ref, a, 2); }
unsigned int m68k_read_memory_32(unsigned int a) { dma_rd(a); return rd_io(0, mem_ref, a, 4); }
// Instruction stream reads (immediate data) and PC-relative reads come from the reference memory without side effects.
unsigned int m68k_read_immediate_16(unsigned int a) { return hole(a) ? 0 : rd(mem_ref, a, 2); }
unsigned int m68k_read_immediate_32(unsigned int a) { return hole(a) ? 0 : rd(mem_ref, a, 4); }
unsigned int m68k_read_pcrelative_8(unsigned int a) { return hole(a) ? 0 : rd(mem_ref, a, 1); }
unsigned int m68k_read_pcrelative_16(unsigned int a) { return hole(a) ? 0 : rd(mem_ref, a, 2); }
unsigned int m68k_read_pcrelative_32(unsigned int a) { return hole(a) ? 0 : rd(mem_ref, a, 4); }
// The disassembler reads too (for the failure message).
unsigned int m68k_read_disassembler_8(unsigned int a) { return rd(mem_ref, a, 1); }
unsigned int m68k_read_disassembler_16(unsigned int a) { return rd(mem_ref, a, 2); }
unsigned int m68k_read_disassembler_32(unsigned int a) { return rd(mem_ref, a, 4); }
// Writes go through wr_ref().
void m68k_write_memory_8(unsigned int a, unsigned int v) { wr_ref(a, 1, v); }
void m68k_write_memory_16(unsigned int a, unsigned int v) { wr_ref(a, 2, v); }
void m68k_write_memory_32(unsigned int a, unsigned int v) { wr_ref(a, 4, v); }
}

// ------------------------------------------------------------------------------------------------------------------
// Random program generator. Base registers A0-A5 and A7 always hold 4-byte aligned data addresses (0x200000 to 0xDFFFFF) and move only in aligned, bounded steps; A6 is a scratch
// register that is never a base. Index registers are used as sign extended words. Displacements and index scales are free, so accesses are misaligned as often as not
// (the flag "al" restores the aligned forms where the effective address becomes a base register, for LEA).
// ------------------------------------------------------------------------------------------------------------------
// EA = an effective address: mode (bits 5-3 of the opcode), register (bits 2-0) and the extension words that follow the opcode.
struct EA { int mode, reg; std::vector<uint16_t> ext; };

// Gen = the program generator: it writes 16-bit words into code and keeps the position of every instruction.
struct Gen {
  // rng = the random number generator (Mersenne twister), seeded with the program's seed; code = the program words; start = word index of each instruction; notgt = marks instructions inside a unit.
  std::mt19937 rng;
  std::vector<uint16_t> code;
  std::vector<uint32_t> start;                  // word index of each instruction
  std::vector<uint8_t> notgt;                   // instruction inside a multi-instruction unit: no branch target
  // Br = a forward branch that still needs its displacement: which instruction it is, how many instructions to skip, and the displacement form.
  struct Br { size_t ins; int skip, form; };    // forward branch placeholder
  std::vector<Br> br;
  // The constructor seeds the random generator.
  explicit Gen(uint32_t seed) : rng(seed) {}
  // r(n): a random number from 0 to n-1.
  uint32_t r(uint32_t n) { return rng() % n; }
  // pc(): the byte address where the next word will go.
  uint32_t pc() const { return 2 * (uint32_t)code.size(); }
  // base(): a random base register: A0 to A5 or A7.
  int base() { static const int b[7] = {0, 1, 2, 3, 4, 5, 7}; return b[r(7)]; }
  // align(): operand alignment in bytes for the size code (0 byte, 1 word, 2 long).
  static int align(int sz) { return sz == 0 ? 1 : sz == 1 ? 2 : 4; }

  bool al = false;  // al = aligned EAs only (used when the address ends up in a base register, LEA).
  // am(): the alignment mask for the current mode and size.
  int am(int sz) { return al ? align(sz) - 1 : 0; }
  // brief(): the brief extension word of an indexed EA, d8(An,Xn.W*scale): index register type (bit 15), number (bits 14-12), index size word (bit 11 is 0), scale (bits 10-9), 8-bit displacement. With al the scale is chosen so that the access stays aligned.
  uint16_t brief(int sz) {   // d8(An,Xn.W*scale): with al, scale >= access size
    // xa = address or data register, xn = register number, sc = scale.
    int xa = r(2), xn = r(8), sc = !al ? r(4) : sz == 0 ? r(4) : sz == 1 ? 1 + r(3) : 2 + r(2);
    // d8 = a random signed displacement, with the low bits cleared when alignment is needed.
    int d8 = ((int)r(256) - 128) & ~am(sz);
    // Assemble the word (bit 8 is 0 for the brief format).
    return (uint16_t)((xa << 15) | (xn << 12) | (sc << 9) | (d8 & 0xff));
  }
  // memp(): a memory EA with every mode; (An)+ and -(An) are also allowed for bytes and words (the caller undoes the step with ea_fix()).
  EA memp(int sz) { return mem(sz, true, true); }   // every mode, also (An)+ and -(An) for byte and word (caller undoes the step: ea_fix)
  // mem(): a random alterable memory EA. postpre = (An)+ and -(An) allowed (for long operands; for bytes and words only if all is set).
  EA mem(int sz, bool postpre, bool all = false) {   // memory EA, alterable
    int a = am(sz) + 1;
    // One of up to 7 forms: (An), d16(An), d8(An,Xn), absolute word, absolute long, (An)+, -(An).
    switch (r(postpre && (sz == 2 || all) ? 7 : 5)) {
      case 0: return {2, base(), {}};
      // d16(An) with a displacement of +-4096, aligned as needed.
      case 1: return {5, base(), {(uint16_t)((((int)r(0x2000) - 0x1000) & ~(a - 1)) & 0xffff)}};
      case 2: return {6, base(), {brief(sz)}};
      // Absolute word in the top 32 KB (0xFF8000 and up, the area the 16-bit address sign-extends to).
      case 3: return {7, 0, {(uint16_t)(0x8000 + (r(0x7ff0) & ~(a - 1)))}};                  // abs.W -> 0xFF8000+
      // Absolute long in the data area 0x200000 to 0xDFFFFF, long aligned.
      case 4: { uint32_t v = 0x200000 + (r(0xc00000) & ~3u); return {7, 1, {(uint16_t)(v >> 16), (uint16_t)v}}; }
      case 5: return {3, base(), {}};
      default: return {4, base(), {}};
    }
  }
  // imm(): an immediate operand (mode 7 register 4), a byte, word or long of random data.
  EA imm(int sz) {
    if (sz == 2) { uint32_t v = rng(); return {7, 4, {(uint16_t)(v >> 16), (uint16_t)v}}; }
    return {7, 4, {(uint16_t)(sz == 0 ? r(256) : r(65536))}};
  }
  // pcrel(): a PC-relative source operand, d16(PC) or d8(PC,Xn); "at" is the offset of its extension word from the instruction start, so that the displacement can be computed from the address of the extension word.
  EA pcrel(int sz, int at = 2) {   // source only; the extension word is at pc()+at
    int a = am(sz) + 1;
    uint32_t ext_at = pc() + at;
    // d16(PC): a target in the first 16 KB, aligned.
    if (r(2)) { int32_t tgt = (int32_t)(r(0x4000) & ~(uint32_t)(a - 1)); return {7, 2, {(uint16_t)((tgt - (int32_t)ext_at) & 0xffff)}}; }
    int xa = r(2), xn = r(8), sc = !al ? r(4) : sz == 0 ? r(4) : sz == 1 ? 1 + r(3) : 2 + r(2);
    // d8(PC,Xn): brief extension word with a displacement adjusted for the alignment.
    int d8 = (((int)r(200) - 100) & ~(a - 1)) - (int)(ext_at % a);
    return {7, 3, {(uint16_t)((xa << 15) | (xn << 12) | (sc << 9) | (d8 & 0xff))}};
  }
  // src(): a random source EA: Dn, An (if allowed), immediate (if allowed), PC-relative or memory.
  EA src(int sz, bool an_ok, bool imm_ok) {
    switch (r(8)) {
      case 0: case 1: return {0, (int)r(8), {}};
      case 2: if (an_ok && sz != 0) return {1, (int)r(8), {}}; return {0, (int)r(8), {}};
      case 3: if (imm_ok) return imm(sz); return {0, (int)r(8), {}};
      case 4: return pcrel(sz);
      default: return mem(sz, true);
    }
  }
  // put(): write an opcode word and the extension words of one or two EAs.
  void put(uint16_t op, const EA& a, const EA* b = nullptr) {
    code.push_back(op);
    for (auto x : a.ext) code.push_back(x);
    if (b) for (auto x : b->ext) code.push_back(x);
  }
  // stp(): the step of (An)+ and -(An) for the size (A7 steps by 2 for bytes).
  static int stp(int sz, int an) { return sz == 0 ? (an == 7 ? 2 : 1) : sz == 1 ? 2 : 4; }   // (An)+ / -(An) step
  // an_adj(): ADDQ.L or SUBQ.L #|delta|,An, to move a base register by a small amount.
  void an_adj(int an, int delta) {   // ADDQ.L / SUBQ.L #|delta|,An
    int q = delta < 0 ? -delta : delta;
    code.push_back((uint16_t)(0x5000 | ((q & 7) << 9) | ((delta < 0 ? 1 : 0) << 8) | (2 << 6) | (1 << 3) | an));
  }
  // ea_fix(): a (An)+ or -(An) step that is not a multiple of 4 would break the rule that base registers stay aligned, so an extra ADDQ/SUBQ undoes it after the instruction.
  // An steps that are not a multiple of 4 would break the base register rule: undo them after the instruction
  void ea_fix(const EA& e, int sz) {
    if (e.mode != 3 && e.mode != 4) return;
    int s = stp(sz, e.reg);
    if (s != 4) an_adj(e.reg, e.mode == 3 ? -s : s);
  }
  // fix_regs(): the same for several registers at once (list of register and step): the sum per register is corrected.
  void fix_regs(std::vector<std::pair<int, int>> v, bool post) {   // (An, step) list: sum per register
    std::map<int, int> t;
    for (auto& p : v) t[p.first] += p.second;
    for (auto& p : t) if (p.second % 4) an_adj(p.first, post ? -p.second : p.second);
  }
  // ea6(): the 6-bit effective address field (mode in bits 5-3, register in bits 2-0).
  static uint16_t ea6(const EA& e) { return (uint16_t)((e.mode << 3) | e.reg); }

  // ---- System instructions, traps and user mode ----
  // Memory in the code area (read only for the programs; code below BND): an instruction length table (LENTAB plus the PC), two copies of the vector table, the handlers and the CHK2/CMP2 bound pairs.
  // Handlers: H_RTE returns; H_SKIP returns behind the faulting instruction (illegal instruction, line A and F, privilege violation stack the address of the instruction, so the handler
  // adds its length from the table); H_SUPER (TRAP #15) returns in supervisor mode, which ends user-mode stretches. A format error (aliased stacks) ends the program.
  // Addresses: length table, the two vector tables, the handlers (RTE, skip, supervisor, FPU trap), BND (the bound pairs, within reach of d16(PC)) and the SR value table. The handlers lie below 0x8000 so that MOVEA.W reaches them.
  static const uint32_t LENTAB = 0x4000, VEC0 = 0x8000, VEC1 = 0x8400, H_RTE = 0x7800, H_SKIP = 0x7810,
                        H_SUPER = 0x7840, H_FPX = 0x7870, BND = 0x3000, SRTAB = 0x3E00;   // H_FPX: FPU traps 48-54   // BND: in reach of d16(PC); handlers below 0x8000 (MOVEA.W)
  // sys_on(): are any of the system groups, or divide (which needs the divide-by-zero vector), switched on? Then the program starts with a prologue that sets up VBR, USP and MSP.
  static bool sys_on() { return (groups & ((1ull << G_SYS) | (1ull << G_TRAP) | (1ull << G_IRQ) | (1ull << G_DIV))) != 0; }   // DIV: zero divide needs the vector table
  // sr_imm(): a random SR value: the trace bits stay clear (Musashi does not trace), everything else (supervisor, master, interrupt mask, CCR) is random.
  uint16_t sr_imm() { return (uint16_t)(rng() & 0x371f); }        // T1 T0 clear (Musashi does not trace), any S M I CCR
  // w2() and w3(): append an opcode word with one or two more words (w3 takes a 32-bit value).
  void w2(uint16_t a, uint16_t b) { code.push_back(a); code.push_back(b); }
  void w3(uint16_t a, uint32_t v) { code.push_back(a); code.push_back((uint16_t)(v >> 16)); code.push_back((uint16_t)v); }
  // dptr(): a random aligned data pointer for stack pointers.
  uint32_t dptr() { return 0x400000 + (r(0x800000) & ~3u); }

  // sys_prologue(): set VBR to the first vector table copy, USP and MSP to data addresses; mask 0 for the interrupt group.
  void sys_prologue() {
    // MOVE.L #VEC0,D0 and MOVEC D0,VBR (control register 0x801).
    ins(); w3(0x203C, VEC0); ins(); w2(0x4E7B, 0x0801);                         // MOVE.L #VEC0,D0; MOVEC D0,VBR
    // MOVEA.L #ptr,A6 and MOVE A6,USP.
    ins(); w3(0x2C7C, dptr()); ins(); code.push_back(0x4E66);                   // MOVEA.L #,A6; MOVE A6,USP
    // MOVEA.L #ptr,A6 and MOVEC A6,MSP (control register 0x803).
    ins(); w3(0x2C7C, dptr()); ins(); w2(0x4E7B, 0xE803);                       // MOVEA.L #,A6; MOVEC A6,MSP
    // With interrupts on: ANDI #$F8FF,SR clears the interrupt mask so that all levels can come in.
    if ((groups >> G_IRQ) & 1) { ins(); w2(0x027C, 0xF8FF); }                 // ANDI #$F8FF,SR: mask 0
  }

  // g_sys(): system instructions: moves to CCR and SR, logical operations on SR, MOVEC, MOVE USP and MOVES.
  void g_sys() {
    int dn = r(8);
    // The control registers: SFC, DFC, USP, VBR, CACR, CAAR, MSP, ISP (0x000-0x002 and 0x800-0x804).
    static const uint16_t crs[8] = {0x000, 0x001, 0x002, 0x800, 0x801, 0x802, 0x803, 0x804};
    switch (r(12)) {
      // MOVE <ea>,CCR.
      case 0: { EA s = src(1, false, true); put((uint16_t)(0x44C0 | ea6(s)), s); break; }              // MOVE <ea>,CCR
      case 1:
        // MOVE #imm,SR, or MOVE (A6)+,SR or -(A6),SR from a table of SR values.
        if (r(2)) { EA s = {7, 4, {sr_imm()}}; put((uint16_t)(0x46C0 | ea6(s)), s); }                 // MOVE #,SR
        else { w3(0x2C7C, SRTAB + 2 + 2 * r(254)); cont(); code.push_back(r(2) ? 0x46DE : 0x46E6); } // (A6)+ / -(A6)
        break;
      // MOVE SR,<ea> and MOVE CCR,<ea>.
      case 2: case 3: {                                                                               // MOVE SR/CCR,<ea>
        EA d = r(2) ? EA{0, dn, {}} : mem(1, true);
        put((uint16_t)((r(2) ? 0x40C0 : 0x42C0) | ea6(d)), d); break;
      }
      // ORI, ANDI and EORI to CCR (0x003C, 0x023C, 0x0A3C).
      case 4: { static const uint16_t o[3] = {0x003C, 0x023C, 0x0A3C}; w2(o[r(3)], (uint16_t)r(256)); break; }
      // ORI, ANDI and EORI to SR; ANDI gets any value, the others the safe SR values.
      case 5: {                                                                                       // ORI/ANDI/EORI #,SR
        int k = r(3);
        w2(k == 0 ? 0x007C : k == 1 ? 0x027C : 0x0A7C, k == 1 ? (uint16_t)rng() : sr_imm()); break;
      }
      // MOVEC from a control register to a register, never into a base register so that the stacks do not overlap.
      case 6: {                                                                                       // MOVEC cr,Rn
        uint16_t cr = crs[r(8)];
        bool sp = (cr == 0x800 || cr == 0x803 || cr == 0x804);
        int rn = r(2) ? dn : 14;                                        // never into a base register: stacks must not overlap
        w2(0x4E7A, (uint16_t)((rn << 12) | cr)); break;
      }
      // MOVEC to a control register: VBR gets one of the two table copies, a stack pointer register gets a fresh pointer so that the stacks never overlap, others any value.
      case 7: {                                                                                       // MOVEC Rn,cr
        uint16_t cr = crs[r(8)];
        if (cr == 0x801) {                                                                            // VBR: a table copy
          w3((uint16_t)(0x203C | (dn << 9)), r(2) ? VEC0 : VEC1); cont(); w2(0x4E7B, (uint16_t)((dn << 12) | cr));
        } else if (cr == 0x800 || cr == 0x803 || cr == 0x804) {                                     // stack pointers:
          w3(0x2C7C, dptr()); cont(); w2(0x4E7B, (uint16_t)((14 << 12) | cr));                    // a fresh pointer, so
        }                                                                                           // stacks never overlap
        else w2(0x4E7B, (uint16_t)((dn << 12) | cr));
        break;
      }
      // MOVE USP,A6 or MOVE A6,USP with a fresh pointer.
      case 8: {                                                                                       // MOVE USP
        if (r(2)) { w3(0x2C7C, dptr()); cont(); code.push_back(0x4E66); }                         // MOVEA.L #,A6; MOVE A6,USP
        else code.push_back(0x4E6E);                                                              // MOVE USP,A6 (not a base register)
        break;
      }
      // MOVES (move to or from the address space of SFC and DFC).
      default: {                                                                                      // MOVES
        int sz = r(3), st = r(2);
        EA e = mem(sz, true);
        int rn = st ? (int)r(16) : (r(4) ? dn : 14);
        code.push_back((uint16_t)(0x0E00 | (sz << 6) | ea6(e)));
        code.push_back((uint16_t)((rn << 12) | (st << 11)));
        for (auto x : e.ext) code.push_back(x);
        break;
      }
    }
  }

  // g_trap(): exceptions: TRAP, TRAPV, TRAPcc, CHK, CHK2, CMP2, ILLEGAL, BKPT, line A and F, STOP, RESET, unknown MOVEC registers, RTE of a frame the program builds.
  void g_trap() {
    int dn = r(8);
    switch (r(17)) {
      // TRAP #n.
      case 0: code.push_back((uint16_t)(0x4E40 | r(16))); break;                                   // TRAP #n
      // TRAP #15 is the handler that returns in supervisor mode.
      case 1: case 2: code.push_back(0x4E4F); break;                                               // TRAP #15: supervisor
      case 3: code.push_back(0x4E76); break;                                                       // TRAPV
      // TRAPcc with a word operand, a long operand or none (opcode 0x50F8 + condition, with 2, 3 or 4 in the low bits for the operand form).
      case 4: {                                                                                    // TRAPcc
        int f = r(3), cc = r(16);
        code.push_back((uint16_t)(0x50F8 | (cc << 8) | (2 + f)));
        if (f == 0) code.push_back((uint16_t)rng());
        if (f == 1) { code.push_back((uint16_t)rng()); code.push_back((uint16_t)rng()); }
        break;
      }
      // CHK.W and CHK.L (0x4180 and 0x4100 with a data register and a source EA).
      case 5: case 6: {                                                                            // CHK.W / CHK.L
        int l = r(2); EA s = src(l ? 2 : 1, false, true);
        put((uint16_t)(0x4000 | (dn << 9) | (l ? 0x100 : 0x180) | ea6(s)), s); break;
      }
      // CHK2 and CMP2 with a bound pair from the table, in all EA forms.
      case 7: case 8: {                                                                            // CHK2/CMP2 Dn
        int sz = r(3), k = r(256);
        uint32_t a = sz == 0 ? BND + 2 * k : sz == 1 ? BND + 0x200 + 4 * k : BND + 0x600 + 8 * k;
        uint16_t w1 = (uint16_t)((dn << 12) | (r(2) << 11)), op = (uint16_t)(0x00C0 | (sz << 9));
        // CHK2.B with (d16,PC) is not generated: Musashi does not sign extend the bounds there.
        int em = r(4); if (em == 1 && sz == 0) em = 0;   // Musashi: CHK2.B (d16,PC) does not sign-extend the bounds
        switch (em) {
          case 0: w2((uint16_t)(op | 0x39), w1); code.push_back((uint16_t)(a >> 16)); code.push_back((uint16_t)a); break;
          case 1: { uint32_t at = pc() + 4; w2((uint16_t)(op | 0x3A), w1); code.push_back((uint16_t)(a - at)); break; }
          case 2: w3(0x2C7C, a); cont(); w2((uint16_t)(op | 0x16), w1); break;                  // MOVEA.L #a,A6; (A6)
          default: { int d = (int)r(0x2000) - 0x1000;                                              // d16(A6)
                     w3(0x2C7C, a - d); cont(); w2((uint16_t)(op | 0x2E), w1); code.push_back((uint16_t)d); break; }
        }
        break;
      }
      // ILLEGAL (0x4AFC) or BKPT.
      case 9: code.push_back(r(2) ? 0x4AFC : (uint16_t)(0x4848 | r(8))); break;                   // ILLEGAL, BKPT
      // Line A (0xAxxx).
      case 10: code.push_back((uint16_t)(0xA000 | r(0x1000))); break;                             // line A
      // Line F for coprocessors that are not the FPU; Musashi runs these as no-ops with a coprocessor id other than 1.
      case 11:                                                                                     // line F; Musashi runs
        { int id = r(7); id += (id >= 1);  // id is 0 to 6 without 1, which is the FPU.
          code.push_back((uint16_t)(0xF000 | (id << 9) | ((4 + r(4)) << 6) | r(64))); } break;   // cp ops 000-011 as no-ops
      // STOP #imm.
      case 12: w2(0x4E72, (uint16_t)(0x2000 | (r(2) << 12) | (r(8) << 8) | r(32))); break;       // STOP #
      // RESET.
      case 13: code.push_back(0x4E70); break;                                                      // RESET
      // MOVEC with a control register number the 68020 does not have.
      case 14: {                                                                                   // MOVEC, unknown cr
        uint16_t cr; do { cr = (uint16_t)r(0x1000); } while ((cr & 0x7ff) <= 4);
        w2((uint16_t)(0x4E7A | r(2)), (uint16_t)((dn << 12) | cr)); break;
      }
      // RTE of a frame built on the stack: MOVE.W #x,-(A7) for each word (format 0 with four words or format 2 with six), then RTE (0x4E73).
      default: {                                                                                   // RTE of a pushed frame
        int f2 = r(3) == 0, n = f2 ? 6 : 4;
        uint32_t ret = pc() + 4 * n + 2;
        std::vector<uint16_t> wv;
        if (f2) { uint32_t ia = rng(); wv.push_back((uint16_t)ia); wv.push_back((uint16_t)(ia >> 16)); }
        wv.push_back((uint16_t)((f2 ? 0x2000 : 0) | (r(256) << 2)));
        wv.push_back((uint16_t)ret); wv.push_back((uint16_t)(ret >> 16)); wv.push_back(sr_imm());
        for (size_t i = 0; i < wv.size(); i++) { if (i) cont(); w2(0x3F3C, wv[i]); }                 // MOVE.W #,-(A7)
        cont(); code.push_back(0x4E73);
        break;
      }
    }
  }

  // ---- MULU, MULS, DIVU and DIVS ----
  // The operands are set by MOVE.L #imm,Dn in front (one unit, never a branch target), so that the generator knows the values and can keep away from the two cases Musashi does differently from
  // the 68020: DIVS.W of 0x80000000 by -1 gives 0 and no overflow on Musashi, and the signed 64 by 32 divide of 0x8000000000000000 by -1 is undefined in C. Divisors include 0, 1, -1, large and small
  // values; dividends are mostly built as divisor times quotient plus remainder, so that they do not overflow.
  // sp32(): a "special" 32-bit value: boundary values, small values, small negative values or a random value.
  uint32_t sp32() {
    static const uint32_t s[] = {0, 1, 2, 3, 7, 0x7f, 0x80, 0xff, 0x100, 0x7fff, 0x8000, 0xffff, 0x10000, 0x10001, 0x7fffffff,
                                 0x80000000u, 0x80000001u, 0xffff8000u, 0xffff7fffu, 0xffffff80u, 0xfffffffeu, 0xffffffffu};
    switch (r(4)) {
      case 0: return s[r(sizeof s / sizeof s[0])];
      case 1: return r(1000);
      case 2: return (uint32_t)(0 - (int32_t)r(1000));
      default: return rng();
    }
  }
  // Unit: a helper for multi-instruction units. nx() registers each following instruction of the unit as not branchable, and movel() emits MOVE.L #value,Dn.
  struct Unit { Gen& g; bool first = true; explicit Unit(Gen& gg) : g(gg) {}
    void nx() { if (!first) g.cont(); first = false; }
    void movel(int dn, uint32_t v) { nx(); g.w3((uint16_t)(0x203C | (dn << 9)), v); } };   // MOVE.L #v,Dn
  // msrc(): the source EA of a multiply or divide: a data register set in front (k 0 to 2), an immediate with the value (3 to 5), PC-relative (6) or memory with random data (7).
  // source EA of kind k (0-2: Dy, set in front; 3-5: immediate dv; 6: PC-relative; 7: memory with random data)
  EA msrc(int k, int sz, bool lg, int dy, uint32_t dv) {
    switch (k) {
      case 0: case 1: case 2: return {0, dy, {}};
      case 3: case 4: case 5: if (sz == 2) return {7, 4, {(uint16_t)(dv >> 16), (uint16_t)dv}}; return {7, 4, {(uint16_t)dv}};
      case 6: return pcrel(sz, lg ? 4 : 2);
      default: return mem(sz, true);
    }
  }
  // md_emit(): write the multiply or divide instruction: the word form (0x80C0 DIVU, 0xC0C0 MULU, bit 8 for signed) or the long form (0x4C00 or 0x4C40 and an extension word with the registers, signed bit and 64-bit bit).
  void md_emit(bool div, bool lg, int sg, int s64, int dq, int dr, const EA& s) {
    if (!lg) code.push_back((uint16_t)((div ? 0x80C0 : 0xC0C0) | (sg << 8) | (dq << 9) | ea6(s)));
    else {
      code.push_back((uint16_t)(0x4C00 | (div ? 0x40 : 0) | ea6(s)));
      code.push_back((uint16_t)((dq << 12) | (sg << 11) | (s64 << 10) | dr));
    }
    for (auto x : s.ext) code.push_back(x);
  }
  // g_mul(): MULU/MULS, word or long, with 32 or 64 bit results.
  void g_mul() {
    Unit u(*this);
    // lg = long form; sg = signed; s64 = 64-bit result; dl, dh = result registers; dy = the source register; k = source kind.
    int lg = r(2), sg = r(2), s64 = lg ? r(2) : 0, dl = r(8), dh = r(8), dy = r(8), k = r(8);
    // The source register must differ from the result registers.
    while (dy == dl || dy == dh) dy = r(8);
    uint32_t vs = sp32();
    // Sometimes preload Dl with a special value.
    if (r(4)) u.movel(dl, sp32());
    // If the source is a register, load it with the value.
    if (k < 3) u.movel(dy, vs);
    u.nx();
    EA s = msrc(k, lg ? 2 : 1, lg, dy, vs);
    md_emit(false, lg, sg, s64, dl, dh, s);
  }
  // g_div(): DIVU/DIVS, word or long, 32 by 32 (quotient only or quotient and remainder) or 64 by 32.
  void g_div() {
    Unit u(*this);
    int lg = r(2), sg = r(2), dq = r(8), dy = r(8), k = r(8);
    // form: 0 = 32/32 quotient only (Dr = Dq), 1 = 32/32 with remainder, 2 = 64/32.
    int form = lg ? (int)r(3) : 0;                    // long: 0 = 32/32 quotient only (Dr = Dq), 1 = 32/32 with remainder, 2 = 64/32
    int dr = dq;
    if (form) { dr = r(8); if (dr == dq) dr = (dr + 1) & 7; }
    while (dy == dq || dy == dr) dy = r(8);
    uint32_t d = sp32(), lo, hi = 0;
    // A word divisor has 16 bits.
    if (!lg) d &= 0xffff;
    // A 32-bit dividend in Dq: mostly built as divisor times quotient plus remainder so that it does not overflow.
    if (!lg || form < 2) {                             // 32-bit dividend in Dq
      lo = sp32();
      if (r(3) && d) {
        if (sg) { int64_t sd = lg ? (int32_t)d : (int16_t)d, q = lg ? (int32_t)sp32() : (int16_t)sp32();
                  int64_t rem = (int64_t)r((uint32_t)(sd < 0 ? -sd : sd)); if (r(2)) rem = -rem; lo = (uint32_t)(sd * q + rem); }
        else { uint64_t q = lg ? sp32() : (sp32() & 0xffff); lo = (uint32_t)((uint64_t)d * q + r(d)); }
      }
      // DIVS.W of 0x80000000 is changed by one bit (see the note above).
      if (sg && !lg && lo == 0x80000000u) lo ^= 1;
    // A 64-bit dividend in Dr:Dq, built the same way; 0x8000000000000000 is avoided.
    } else {                                           // 64-bit dividend Dr:Dq
      uint64_t N;
      if (r(4) == 0 || !d) N = ((uint64_t)sp32() << 32) | sp32();
      else if (sg) { int64_t sd = (int32_t)d, q = (int32_t)sp32(), ad = sd < 0 ? -sd : sd;
                     int64_t rem = (int64_t)(((uint64_t)rng() << 32 | rng()) % (uint64_t)ad); if (r(2)) rem = -rem;
                     N = (uint64_t)(sd * q + rem); }
      else { uint64_t q = sp32(); N = (uint64_t)d * q + (((uint64_t)rng() << 32 | rng()) % d); }
      if (sg && N == 0x8000000000000000ull) N ^= 1;
      hi = (uint32_t)(N >> 32); lo = (uint32_t)N;
    }
    u.movel(dq, lo);
    if (lg && form == 2) u.movel(dr, hi);
    // The divisor register value: full for a long divide, with random upper half for a word divide (the word form uses the low half only).
    uint32_t dfull = lg ? d : ((d & 0xffff) | (rng() << 16));
    if (k < 3) u.movel(dy, dfull);
    u.nx();
    EA s = msrc(k, lg ? 2 : 1, lg, dy, d);
    md_emit(true, lg, sg, lg && form == 2, dq, dr, s);
  }

  // sysmem(): fill the system memory: instruction length table, vector tables, handlers, SR table and CHK2 bound pairs. Called before the program runs.
  void sysmem(std::vector<uint8_t>& m) {
    // w16 and w32: write a big endian word or long.
    auto w16 = [&](uint32_t a, uint16_t v) { m[a] = v >> 8; m[a + 1] = v & 0xff; };
    auto w32 = [&](uint32_t a, uint32_t v) { w16(a, (uint16_t)(v >> 16)); w16(a + 2, (uint16_t)v); };
    // The length table: for every instruction start, its length in bytes (the handlers add it to the stacked PC).
    for (size_t i = 0; i + 1 < start.size(); i++) m[LENTAB + 2 * start[i]] = (uint8_t)(2 * (start[i + 1] - start[i]));
    // Both vector tables: illegal instruction, privilege violation, line A and line F (vectors 4, 8, 10, 11) go to the skip handler, TRAP #15 (vector 47) to the supervisor handler, the FPU
    // vectors 48 to 54 to the FPU trap handler, the format error vector 14 to the end of the program, everything else to a plain RTE.
    for (uint32_t t : {VEC0, VEC1})
      for (uint32_t v = 0; v < 256; v++)
        w32(t + 4 * v, (v == 4 || v == 8 || v == 10 || v == 11) ? H_SKIP : v == 47 ? H_SUPER : (v >= 48 && v <= 54) ? H_FPX :
                       v == 14 ? 2 * start.back() : H_RTE);                  // format error: corrupt stack, end
    // Handler RTE: just RTE (0x4E73).
    w16(H_RTE, 0x4E73);
    w16(H_FPX, 0xF327); w16(H_FPX + 2, 0xF35F); w16(H_FPX + 4, 0x4E73);  // FPU trap handler: FSAVE -(A7) (which clears the pending trap), FRESTORE (A7)+, RTE.
    // Skip handler: pushes D0 and A6 (MOVEM), loads the low word of the stacked PC into A6 (MOVEA.W, which is why the handlers lie below 0x8000), reads the length of the faulting instruction from the length table into D0, adds it to the stacked PC, restores D0 and A6 and returns with RTE.
    static const uint16_t skip[] = {0x48E7, 0x8002, 0x3C6F, 0x000C, 0x7000, 0x102E, 0x4000, 0xD16F, 0x000C,
                                    0x4CDF, 0x4001, 0x4E73};
    for (size_t i = 0; i < sizeof skip / 2; i++) w16(H_SKIP + 2 * (uint32_t)i, skip[i]);
    // Supervisor handler for TRAP #15: sets the supervisor bit in the stacked SR (so the program continues in supervisor mode), then returns.
    static const uint16_t sup[] = {0x48E7, 0x8000, 0x302F, 0x0004, 0x807C, 0x2000, 0x3F40, 0x0004,
                                   0x4CDF, 0x0001, 0x4E73};
    for (size_t i = 0; i < sizeof sup / 2; i++) w16(H_SUPER + 2 * (uint32_t)i, sup[i]);
    // The SR values for MOVE (A6)+,SR.
    for (int k = 0; k < 256; k++) w16(SRTAB + 2 * k, sr_imm());                  // SR values for MOVE (A6)+,SR
    // Bound pairs for CHK2 and CMP2: byte, word and long pairs with lower <= upper (signed), 256 of each.
    for (int k = 0; k < 256; k++) {                                              // bound pairs, lower <= upper (signed)
      int8_t b0 = (int8_t)rng(), b1 = (int8_t)rng(); if (b0 > b1) std::swap(b0, b1);
      m[BND + 2 * k] = (uint8_t)b0; m[BND + 2 * k + 1] = (uint8_t)b1;
      int16_t h0 = (int16_t)rng(), h1 = (int16_t)rng(); if (h0 > h1) std::swap(h0, h1);
      w16(BND + 0x200 + 4 * k, (uint16_t)h0); w16(BND + 0x202 + 4 * k, (uint16_t)h1);
      int32_t l0 = (int32_t)rng(), l1 = (int32_t)rng(); if (l0 > l1) std::swap(l0, l1);
      w32(BND + 0x600 + 8 * k, (uint32_t)l0); w32(BND + 0x604 + 8 * k, (uint32_t)l1);
    }
  }

  // ---- Program flow: calls, returns, stack frames, loops ----
  // G_CALL: a subroutine (defined in line behind a BRA over it, or an earlier one called again; it ends with RTS, RTD #n or RTR) and its call: BSR.S, BSR.W or BSR.L, JSR with absolute word,
  // absolute long, d16(PC), d8(PC,Xn), (A6) or d16(A6), or a forward JMP. All instructions of such a unit are no branch targets, so every call returns. The bodies are register-only, so the stack is
  // balanced and untouched. G_STACK: PEA, and LINK ... UNLK. G_LOOP: backward Bcc and BRA loops with a counter from 1 to 4.
  // Sub = a subroutine: address, kind (0 ends with RTS, 1 with RTD #4*argc, 2 with RTR) and the number of arguments the caller pushes for RTD.
  struct Sub { uint32_t addr; int kind, argc; };   // kind 0 RTS, 1 RTD #4*argc, 2 RTR
  // subs = the subroutines defined so far (the six latest are kept for repeated calls).
  std::vector<Sub> subs;
  // Fx = a forward JMP whose target is patched when the program is complete: the instruction it belongs to, how many instructions to skip, the form, where the target word is, the base for PC-relative forms and an extra offset.
  struct Fx { size_t own; int skip, kind; uint32_t at, base; int extra; };   // forward JMP target patch
  std::vector<Fx> fx;
  // fresh = the next instruction is the first one of a new unit, which one() has registered as an instruction start already.
  bool fresh = false;                                 // the first instruction of a unit is registered by one()
  // nxt(): start the next instruction of a unit: the first one is already registered; the others are registered as "inside a unit" (no branch target).
  void nxt() { if (fresh) fresh = false; else cont(); }
  // rinstr(): one register-only instruction that never writes D<avoid> (used for bodies and loops).
  void rinstr(int avoid) {                            // one register-only instruction that never writes D<avoid>
    nxt();
    // m = the destination register, never the one to avoid.
    int m = r(8); while (m == avoid) m = r(8);
    // y = a second register, sz = size code.
    int y = r(8), sz = r(3);
    // The ALU opcode groups: OR (8), SUB (9), CMP (B), AND (C), ADD (D).
    static const int ops[5] = {0x8, 0x9, 0xB, 0xC, 0xD};
    switch (r(8)) {
      // MOVEQ #imm,Dm.
      case 0: code.push_back((uint16_t)(0x7000 | (m << 9) | r(256))); break;
      // ADDQ or SUBQ #q,Dm (opcode 0x5000, bit 8 = subtract).
      case 1: code.push_back((uint16_t)(0x5000 | (r(8) << 9) | (r(2) << 8) | (sz << 6) | m)); break;
      // OR, SUB, CMP, AND or ADD Dy,Dm.
      case 2: case 3: code.push_back((uint16_t)((ops[r(5)] << 12) | (m << 9) | (sz << 6) | y)); break;
      // NEG or NOT (0x4400, 0x4600).
      case 4: code.push_back((uint16_t)((r(2) ? 0x4400 : 0x4600) | (sz << 6) | m)); break;
      // SWAP (0x4840), EXT.W (0x4880) or EXT.L (0x48C0).
      case 5: code.push_back((uint16_t)((r(3) == 0 ? 0x4840 : r(2) ? 0x4880 : 0x48C0) | m)); break;
      // A shift or rotate by an immediate or by a register (0xE000 plus type, direction, size, count mode).
      case 6: code.push_back((uint16_t)(0xE000 | (r(8) << 9) | (r(2) << 8) | (sz << 6) | (r(2) << 5) | ((int)r(4) << 3) | m)); break;
      // EOR Dy,Dm (0xB100).
      default: code.push_back((uint16_t)(0xB100 | (y << 9) | (sz << 6) | m)); break;
    }
  }
  // call_sub(): emit a call of subroutine s in one of seven forms.
  void call_sub(const Sub& s) {
    // RTD returns and pops its arguments, so the caller pushes them first (MOVE Dn,-(A7) or MOVE.L #imm,-(A7)).
    for (int i = 0; i < s.argc; i++) {                // RTD: the caller pushes the arguments
      nxt();
      if (r(2)) code.push_back((uint16_t)(0x2F00 | r(8))); else w3(0x2F3C, (uint32_t)rng());   // MOVE.L Dn|#,-(A7)
    }
    // f = the form of the call.
    int f = r(7);
    // x = a random offset used in the d8(PC,Dn) form; d8 = its displacement.
    int32_t x = (int32_t)r(41) - 20, d8 = 0;
    // Form 6: MOVEQ #x,Dn; JSR d8(PC,Dn.W): the displacement must fit in a signed byte, otherwise use form 3.
    if (f == 6) {                                     // MOVEQ #x,Dn; JSR d8(PC,Dn.W): the extension word is at pc() + 4
      d8 = (int32_t)s.addr - (int32_t)(pc() + 4) - x;
      if (d8 < -128 || d8 > 127) f = 3;
    }
    // at = address of this instruction, d = displacement from the end of the opcode word to the subroutine.
    uint32_t at = pc();
    int32_t d = (int32_t)s.addr - (int32_t)(at + 2);
    // Form 0: BSR with a byte displacement (only for a short backward distance), a word displacement (0x6100) or a long displacement (0x61FF).
    if (f == 0) {
      int form = r(3);
      if (form == 0 && (d < -128 || d > -2)) form = 1;
      nxt();
      if (form == 0) code.push_back((uint16_t)(0x6100 | (d & 0xff)));
      else if (form == 1) { code.push_back(0x6100); code.push_back((uint16_t)d); }
      else { code.push_back(0x61FF); code.push_back((uint16_t)(d >> 16)); code.push_back((uint16_t)d); }
    // Form 1: JSR absolute long (0x4EB9). Form 2: JSR absolute word (0x4EB8). Form 3: JSR d16(PC) (0x4EBA).
    } else if (f == 1) { nxt(); w3(0x4EB9, s.addr); }
    else if (f == 2) { nxt(); w2(0x4EB8, (uint16_t)s.addr); }
    else if (f == 3) { nxt(); w2(0x4EBA, (uint16_t)d); }
    // Form 4: LEA d16(PC),A6 and JSR (A6) (0x4E96).
    else if (f == 4) { nxt(); w2(0x4DFA, (uint16_t)d); nxt(); code.push_back(0x4E96); }          // LEA d16(PC),A6; JSR (A6)
    // Form 5: LEA d16(PC),A6 and JSR d16(A6).
    else if (f == 5) {                                                                           // LEA d16(PC),A6; JSR d16(A6)
      int32_t dd = 2 * ((int32_t)r(0x40) - 0x20);
      nxt(); w2(0x4DFA, (uint16_t)(d - dd)); nxt(); w2(0x4EAE, (uint16_t)dd);
    // Form 6: MOVEQ #x,Dn, then JSR d8(PC,Dn.W) (0x4EBB).
    } else {
      int dn = r(8);
      nxt(); code.push_back((uint16_t)(0x7000 | (dn << 9) | (x & 0xff)));
      nxt(); w2(0x4EBB, (uint16_t)((dn << 12) | (d8 & 0xff)));
    }
  }
  // g_jmp(): a forward JMP to a later instruction; the target is patched when the program is complete (fx list).
  void g_jmp() {                                      // forward JMP to a later instruction (patched in build)
    // k = the form (0 absolute word, 1 absolute long, 2 d16(PC), 3 and 4 through A6), skip = how many instructions to jump over.
    int k = r(5), skip = r(4);
    nxt();
    switch (k) {
      // The word that follows the opcode is a placeholder for the target; build() fills it in.
      case 0: code.push_back(0x4EF8); fx.push_back({start.size() - 1, skip, 0, (uint32_t)code.size(), 0, 0}); code.push_back(0); break;
      case 1: code.push_back(0x4EF9); fx.push_back({start.size() - 1, skip, 1, (uint32_t)code.size(), 0, 0}); code.push_back(0); code.push_back(0); break;
      case 2: code.push_back(0x4EFA); fx.push_back({start.size() - 1, skip, 2, (uint32_t)code.size(), pc(), 0}); code.push_back(0); break;
      // Forms 3 and 4: LEA d16(PC),A6 and JMP (A6) or JMP d16(A6).
      default: {                                      // LEA d16(PC),A6; JMP (A6) or JMP d16(A6)
        int dd = k == 3 ? 0 : 2 * ((int)r(0x40) - 0x20);
        code.push_back(0x4DFA); uint32_t at = (uint32_t)code.size(), base = pc(); code.push_back(0);
        nxt(); code.push_back(k == 3 ? 0x4ED6 : 0x4EEE);
        if (k != 3) code.push_back((uint16_t)dd);
        fx.push_back({start.size() - 1, skip, 2, at, base, dd});
      }
    }
  }
  // g_call(): pick between defining a new subroutine and calling one that exists (or a forward JMP).
  void g_call() {
    // fresh = this is the first instruction of a unit.
    fresh = true;
    int k = r(10);
    // A subroutine is defined behind a BRA.W (0x6000 and a word displacement) so the program flows around it.
    if (k < 3 || subs.empty()) {                      // define a subroutine behind a BRA.W over it, then call it
      Sub s; s.kind = r(3); s.argc = s.kind == 1 ? 1 + (int)r(2) : 0;
      // The BRA.W displacement is a placeholder, patched below once the subroutine's length is known.
      nxt(); size_t bra = code.size(); code.push_back(0x6000); code.push_back(0);
      s.addr = pc();
      // 0 to 3 register instructions as the body.
      int nb = r(4); for (int i = 0; i < nb; i++) rinstr(-1);
      // RTR pops a CCR word from the stack: push one first.
      if (s.kind == 2) { nxt(); w2(0x3F3C, (uint16_t)rng()); }           // MOVE.W #,-(A7): the CCR word RTR pops
      nxt();
      // The return: RTS (0x4E75), RTD #n (0x4E74) or RTR (0x4E77).
      if (s.kind == 0) code.push_back(0x4E75); else if (s.kind == 1) w2(0x4E74, (uint16_t)(4 * s.argc)); else code.push_back(0x4E77);
      // Patch the displacement of the BRA over the subroutine.
      code[bra + 1] = (uint16_t)(pc() - (2 * (uint32_t)bra + 2));
      // Remember the subroutine (the six latest), then call it.
      subs.push_back(s); if (subs.size() > 6) subs.erase(subs.begin());
      call_sub(s);
    // Call a known subroutine.
    } else if (k < 8) call_sub(subs[r((uint32_t)subs.size())]);
    else g_jmp();
  }
  // g_stack(): PEA, or LINK ... UNLK.
  void g_stack() {
    fresh = true;
    // PEA <ea> (0x4840), then sometimes pop the value into a register or drop it with ADDQ.L #4,A7.
    if (r(2)) {                                       // PEA, pushed value read back or dropped sometimes
      EA s = r(3) == 0 ? pcrel(2) : mem(2, false);
      nxt(); put((uint16_t)(0x4840 | ea6(s)), s);
      int k = r(4);
      if (k == 0) { nxt(); code.push_back((uint16_t)(0x201F | (r(8) << 9))); }        // MOVE.L (A7)+,Dn
      else if (k == 1) { nxt(); an_adj(7, 4); }                                       // ADDQ.L #4,A7
    // LINK An,#d with a word displacement (0x4E50) or a long displacement (0x4808), a register-only body, then UNLK An (0x4E58). With the system groups on the frame register is always A6, otherwise A0 to A6.
    } else {                                          // LINK An,#d; register-only body; UNLK An
      int an = sys_on() ? 6 : (int)r(7), d = 4 * ((int)r(33) - 16);
      nxt();
      if (r(2)) w3((uint16_t)(0x4808 | an), (uint32_t)d); else w2((uint16_t)(0x4E50 | an), (uint16_t)d);
      int nb = r(4); for (int i = 0; i < nb; i++) rinstr(-1);
      nxt(); code.push_back((uint16_t)(0x4E58 | an));
    }
  }
  // g_uloop(): a tight loop as Amiga code polls a chip register: a pairable register instruction, a load from the uncached area (absolute word 0xFF8000 and up), then DBF or SUBQ and BNE back.
  // With the interrupt group on, an interrupt must still be taken within a few instructions. This is the test for the fault where the head instruction of such a loop ran in lane B (where no interrupt is taken) and an
  // interrupt could never come in.
  // a tight loop as Amiga code polls a chip register: a pairable register instruction, a load from the uncached area
  // (abs.W 0xFF8000+), DBF or SUBQ/BNE back. With group 23 an interrupt must still be taken within a few instructions.
  void g_uloop() {
    fresh = true;
    // k = the counter register, n = the loop count (20 to 119), m and dd = other registers for the pairable instruction and the load.
    int k = r(8), n = 20 + (int)r(100), m = (k + 1 + r(7)) % 8, dd = (k + 1 + r(7)) % 8;
    // MOVEQ #n,Dk.
    nxt(); code.push_back((uint16_t)(0x7000 | (k << 9) | n));                        // MOVEQ #n,Dk
    uint32_t loop = pc();
    nxt();
    // The pairable head: MOVEQ, ADDQ.L #q,Dm or MOVE.L Dx,Dm.
    switch (r(3)) {                                                                  // pairable head
      case 0: code.push_back((uint16_t)(0x7000 | (m << 9) | r(256))); break;         // MOVEQ
      case 1: code.push_back((uint16_t)(0x5080 | m | ((1 + r(7)) << 9))); break;      // ADDQ.L #q,Dm
      default: code.push_back((uint16_t)(0x2000 | (m << 9) | ((m + 1) % 8 == k ? (m + 2) % 8 : (m + 1) % 8))); break;   // MOVE.L Dx,Dm
    }
    nxt();
    // The uncached address, a word at 0xFF8000 and up.
    uint16_t aw = (uint16_t)(0x8000 + (r(0x7ff0) & ~1u));
    // The four loads: ADD.W, CMP.W, MOVE.W from absolute word into Dd (0xD078, 0xB078, 0x3038) and TST.W absolute word (0x4A78).
    static const uint16_t ld[4] = {0xD078, 0xB078, 0x3038, 0x4A78};                 // ADD.W CMP.W MOVE.W abs.W,Dd; TST.W abs.W
    // Choose one; the destination register is added; if the destination would be the counter, use TST.W instead.
    int li = r(4); uint16_t lop = ld[li]; if (li < 3) lop |= (uint16_t)(dd << 9); if (dd == k && li != 1 && li != 3) lop = 0x4A78;
    code.push_back(lop); code.push_back(aw);
    nxt();
    // The loop end, DBF Dk,loop (0x51C8) or SUBQ.L #1,Dk and BNE.W loop (0x5380, 0x6600).
    if (r(2)) { uint32_t at = pc() + 2; code.push_back((uint16_t)(0x51C8 | k)); code.push_back((uint16_t)((int32_t)loop - (int32_t)at)); }   // DBF Dk,loop
    else {
      code.push_back((uint16_t)(0x5380 | k)); nxt();                                  // SUBQ.L #1,Dk
      uint32_t at = pc() + 2; code.push_back(0x6600); code.push_back((uint16_t)((int32_t)loop - (int32_t)at));   // BNE.W loop
    }
  }
  // g_loop(): MOVEQ #n,Dk, a body of register instructions, SUBQ.L #1,Dk and a backward Bcc, or BEQ out with a BRA back.
  void g_loop() {
    fresh = true;
    int k = r(8), n = 1 + (int)r(4);
    nxt(); code.push_back((uint16_t)(0x7000 | (k << 9) | n));                         // MOVEQ #n,Dk
    uint32_t loop = pc();
    int nb = 1 + (int)r(3);
    // First form: a conditional branch back; the conditions T, F, VC and LE are not used because they never exit.
    if (r(3) != 0) {                                  // body; SUBQ.L #1,Dk; Bcc loop (backward). Not T, F, VC, LE: they never exit.
      for (int i = 0; i < nb; i++) rinstr(k);
      nxt(); code.push_back((uint16_t)(0x5380 | k));
      static const int ccs[12] = {2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14};
      int cc = ccs[r(12)], form = r(3);
      uint32_t at = pc(); int32_t d = (int32_t)loop - (int32_t)(at + 2);
      if (form == 0 && (d < -128 || d > -2)) form = 1;
      nxt();
      if (form == 0) code.push_back((uint16_t)(0x6000 | (cc << 8) | (d & 0xff)));
      else if (form == 1) { code.push_back((uint16_t)(0x6000 | (cc << 8))); code.push_back((uint16_t)d); }
      else { code.push_back((uint16_t)(0x60FF | (cc << 8))); code.push_back((uint16_t)(d >> 16)); code.push_back((uint16_t)d); }
    // Second form: BEQ.S out behind a BRA back (the BEQ skips the BRA when the counter reaches zero).
    } else {                                          // body; SUBQ.L #1,Dk; BEQ out; BRA loop; out:
      for (int i = 0; i < nb; i++) rinstr(k);
      nxt(); code.push_back((uint16_t)(0x5380 | k));
      int form = r(3);
      // The BRA's displacement is computed from the position behind the BEQ.
      uint32_t at = pc() + 2; int32_t d = (int32_t)loop - (int32_t)(at + 2);          // BRA behind the BEQ
      if (form == 0 && (d < -128 || d > -2)) form = 1;
      int blen = form == 0 ? 2 : form == 1 ? 4 : 6;
      nxt(); code.push_back((uint16_t)(0x6700 | blen));                              // BEQ.S behind the BRA
      nxt();
      if (form == 0) code.push_back((uint16_t)(0x6000 | (d & 0xff)));
      else if (form == 1) { code.push_back(0x6000); code.push_back((uint16_t)d); }
      else { code.push_back(0x60FF); code.push_back((uint16_t)(d >> 16)); code.push_back((uint16_t)d); }
    }
  }

  // ---- DMA window loads and self-modifying code ----
  // g_dma: one single-uop load from a DMA window (MOVE abs.L,Dn, or ADD, CMP or AND abs.L,Dn, or TST abs.L), cached or uncached, at any byte offset (a word or long may cross a
  // longword or a cache line). The bench writes into those windows behind the CPU's back (see the DMA rule in run_program()), and these loads read them.
  // g_smc: (a) LEA slot(PC),A6; MOVE.W #new,d(A6); 0-2 or 8-24 instructions; slot. That is a store ahead of the PC, near (still in the fetch queue) or far (in the cache).
  // (b) MOVEQ #2,Dk; loop: slot; body; LEA slot(PC),A6; MOVE.W #new,d(A6); SUBQ.L #1,Dk; BNE loop. That patches code that already ran and runs it again.
  // A slot is MOVEQ #x,Dm (the whole word is replaced) or ADDI.W #x,Dm (only the immediate is replaced); the length never changes. Musashi does not prefetch (M68K_EMULATE_PREFETCH
  // is off), so it runs the patched code at once, and the core has to catch the store and drop its stale copy of the instruction (docs/MEMORY_PORT.md, Coherency).
  // Sx = a slot patch to be fixed after the program is complete (not used further).
  struct Sx { size_t lea; uint32_t at; };            // forward slot patch: LEA displacement word index, slot word
  // g_dma(): sz = size, dn = register, a = alignment of the size.
  void g_dma() {
    int sz = r(3), dn = r(8), a = align(sz);
    // The address: a random window, and half of the loads misaligned at any byte.
    uint32_t ad = (r(2) ? DMA_C : DMA_U) + (r(2) ? (r(DMA_SZ) & ~(uint32_t)(a - 1)) : r(DMA_SZ - 3));   // merge 7+8: half misaligned
    // The EA is absolute long (mode 7, register 1).
    EA s = {7, 1, {(uint16_t)(ad >> 16), (uint16_t)ad}};
    // szc[]: the size field of MOVE for byte, word and long (1, 3, 2).
    static const int szc[3] = {1, 3, 2};
    int k = r(5);
    // MOVE abs.L,Dn.
    if (k == 0) put((uint16_t)((szc[sz] << 12) | (dn << 9) | ea6(s)), s);                        // MOVE abs.L,Dn
    // TST.size abs.L.
    else if (k == 4) put((uint16_t)(0x4A00 | (sz << 6) | ea6(s)), s);                             // TST abs.L
    // ADD, CMP or AND abs.L,Dn (0xD, 0xB, 0xC).
    else { static const int ops[3] = {0xD, 0xB, 0xC}; put((uint16_t)((ops[k - 1] << 12) | (dn << 9) | (sz << 6) | ea6(s)), s); }
  }
  // slot_word(): the word that a store puts into a slot: MOVEQ #x,Dm (kind 0) or a random immediate (kind 1).
  uint16_t slot_word(int kind, int avoid) {          // a slot's patched word: MOVEQ #x,Dm or an immediate
    if (kind == 0) { int m = r(8); while (m == avoid) m = r(8); return (uint16_t)(0x7000 | (m << 9) | r(256)); }
    return (uint16_t)rng();
  }
  // slot(): emit the slot instruction itself: MOVEQ #x,Dm or ADDI.W #x,Dm (0x0640 and an immediate word).
  void slot(int kind, int avoid) {                   // the slot instruction itself
    nxt();
    if (kind == 0) code.push_back(slot_word(0, avoid));
    else { int m = r(8); while (m == avoid) m = r(8); w2((uint16_t)(0x0640 | m), (uint16_t)rng()); }   // ADDI.W #,Dm
  }
  // smc_store(): LEA slot(PC),A6 (0x4DFA) and a MOVE.W #new,(A6) or MOVE.W #new,2(A6) that writes the new word into the slot.
  void smc_store(uint32_t slot_at, int kind, int avoid, size_t* lea_fix) {   // LEA slot(PC),A6; MOVE.W #new,d(A6)
    nxt(); uint32_t lp = pc(); code.push_back(0x4DFA);
    // Remember where the LEA displacement word is, for the later patch.
    if (lea_fix) *lea_fix = code.size();
    code.push_back((uint16_t)((int32_t)slot_at - (int32_t)(lp + 2)));
    nxt();
    uint16_t nw = slot_word(kind, avoid);
    // Kind 0: MOVE.W #new,(A6) (0x3CBC); kind 1: MOVE.W #new,2(A6) (0x3D7C), which replaces the immediate of an ADDI.W.
    if (kind == 0) w2(0x3CBC, nw);                                                       // MOVE.W #new,(A6)
    else { code.push_back(0x3D7C); code.push_back(nw); code.push_back(2); }             // MOVE.W #new,2(A6)
  }
  // g_smc(): either (a) store ahead, or (b) patch executed code and run it again (a third of the time).
  void g_smc() {
    fresh = true;
    int kind = r(2);
    // (a) the store goes ahead of the PC; 0 to 2 instructions in between (near, in the fetch queue) or 8 to 24 (far, in the cache).
    if (r(3) != 0) {                                  // (a) store ahead
      size_t lf = 0;
      smc_store(0, kind, -1, &lf);
      int n = r(2) ? (int)r(3) : 8 + (int)r(17);
      for (int i = 0; i < n; i++) rinstr(-1);
      uint32_t at = pc();
      slot(kind, -1);
      // The LEA displacement is relative to the word itself.
      code[lf] = (uint16_t)((int32_t)at - (int32_t)(2 * (uint32_t)lf));                 // LEA displacement: from the word itself
    // (b) MOVEQ #2,Dk; the loop runs twice; the slot is patched after its first run.
    } else {                                          // (b) patch executed code and run it again
      int k = r(8);
      nxt(); code.push_back((uint16_t)(0x7002 | (k << 9)));                              // MOVEQ #2,Dk
      uint32_t loop = pc();
      slot(kind, k);
      int nb = r(3); for (int i = 0; i < nb; i++) rinstr(k);
      smc_store(loop, kind, k, nullptr);
      // SUBQ.L #1,Dk and BNE.
      nxt(); code.push_back((uint16_t)(0x5380 | k));                                     // SUBQ.L #1,Dk
      uint32_t at = pc(); int32_t d = (int32_t)loop - (int32_t)(at + 2);
      nxt();
      if (d >= -128) code.push_back((uint16_t)(0x6600 | (d & 0xff))); else w2(0x6600, (uint16_t)d);   // BNE loop
    }
  }
  // ---- Bit fields ----
  // Memory fields: byte offsets and EA alignment are free (the core reads and writes the two aligned longs around the field); register offsets are kept small so the field stays in the data area.
  // g_bf(): memf = the field is in memory (otherwise in a data register).
  void g_bf(bool memf) {
    // t = the instruction: 0 BFTST, 1 BFEXTU, 2 BFCHG, 3 BFEXTS, 4 BFCLR, 5 BFFFO, 6 BFSET, 7 BFINS (bits 10-8 of the opcode).
    int t = r(8);                                                        // 0 TST 1 EXTU 2 CHG 3 EXTS 4 CLR 5 FFO 6 SET 7 INS
    // ro = read-only forms (they can use PC-relative EAs).
    bool ro = (t == 0 || t == 1 || t == 3 || t == 5);
    // hasdn = forms that have a data register (the destination or source).
    bool hasdn = (t == 1 || t == 3 || t == 5 || t == 7);
    int dn = hasdn ? r(8) : 0;
    // dynO, dynW = offset and width come from data registers instead of the extension word.
    bool dynO = r(2), dynW = r(2);
    int oreg = r(8), wreg = r(8), ofld = 0, wfld = 0;
    // PC-relative addressing needs a non-negative offset.
    bool pc_ok = memf && ro;                                            // PC-relative needs a non-negative offset
    bool pc = pc_ok && r(4) == 0;
    // Dynamic offset: load the offset register first with MOVE.L #v,Dn, with values near 0, near 31/32, on byte boundaries, medium, negative, large, or any (register form: only the low 5 bits count).
    if (dynO) {
      int32_t v;
      switch (r(7)) {
        case 0: v = (int32_t)r(9); break;                               // near 0
        case 1: v = 28 + (int32_t)r(9); break;                          // around 31/32
        case 2: v = 8 * (int32_t)r(40) + 7 - (int32_t)r(3); break;      // byte boundaries
        case 3: v = (int32_t)r(600); break;
        case 4: v = pc ? (int32_t)r(300) : -(int32_t)r(300) - 1; break; // negative
        case 5: v = pc ? (int32_t)r(4000) : (int32_t)r(8000) - 4000; break;
        default: v = memf ? (int32_t)r(64) - (pc ? 0 : 32) : (int32_t)rng(); break;
      }
      if (!memf && r(3) == 0) v = (int32_t)rng();                       // register form: only the low 5 bits count
      w3((uint16_t)(0x203C | (oreg << 9)), (uint32_t)v); cont();
      ofld = oreg;
    } else ofld = (r(3) == 0) ? 31 - (int)r(2) : (r(4) == 0 ? (int)r(2) : (int)r(32));
    // Dynamic width: load the width register with 0 (which means 32), 1, 32, 31 to 33, or any value.
    if (dynW) {
      uint32_t v;
      switch (r(5)) { case 0: v = 0; break; case 1: v = 1; break; case 2: v = 32; break; case 3: v = 31 + r(3); break; default: v = rng(); }
      if (dynO && wreg == oreg) wreg = (oreg + 1) & 7;
      w3((uint16_t)(0x203C | (wreg << 9)), v); cont();
      wfld = wreg;
    } else wfld = (r(4) == 0) ? 0 : (r(4) == 0 ? 1 : (r(8) == 0 ? 31 : (int)r(32)));
    // The effective address: a data register for the register form; for memory the usual forms: (An), d16(An), d8(An,Xn), absolute word, absolute long and PC-relative (only for read forms).
    EA e;
    if (!memf) e = {0, (int)r(8), {}};
    else {
      int k = r(pc_ok ? 6 : 5);
      if (k == 5 && !pc) k = r(5);
      switch (k) {
        case 0: e = {2, base(), {}}; break;
        case 1: e = {5, base(), {(uint16_t)(((int)r(0x2000) - 0x1000) & 0xffff)}}; break;
        case 2: e = {6, base(), {brief(0)}}; break;
        case 3: e = {7, 0, {(uint16_t)(0x8000 + r(0x6000))}}; break;
        case 4: { uint32_t a = 0x200000 + r(0xc00000); e = {7, 1, {(uint16_t)(a >> 16), (uint16_t)a}}; break; }
        default: e = pcrel(0, 4); break;
      }
    }
    // The opcode word (0xE8C0 plus type in bits 10-8 and the EA) and the extension word: data register in bits 14-12, offset type (bit 11), offset (bits 10-6), width type (bit 5), width (bits 4-0).
    code.push_back((uint16_t)(0xE8C0 | (t << 8) | ea6(e)));
    code.push_back((uint16_t)((dn << 12) | (dynO << 11) | (ofld << 6) | (dynW << 5) | wfld));
    for (auto x : e.ext) code.push_back(x);
  }

  // ---- Full-format effective addresses (group 33), long instructions (34), CAS and CAS2 (35) ----
  // A unit sets A6 to a known data address first (a6v). Full-format EAs use A6, PC or a suppressed base, an index register that the unit sets (Dk or A6.W), and for memory indirect a pointer that the unit
  // stores right in front (MOVE.L #V,(P).L, a five-word instruction). Targets and pointers are random data addresses of any alignment.
  // a6v = the value of A6 in the current unit. ix_used = a mask of the data registers used as index in this unit (they must not be changed).
  uint32_t a6v = 0;
  int ix_used = 0;
  // daddr(): a random data address (0x300000 to 0xBFFFFF).
  uint32_t daddr() { return 0x300000 + r(0x900000); }
  // mvop(): the opcode word of MOVE with the destination EA (register in bits 11-9, mode in bits 8-6) and the source EA, with the size code.
  static uint16_t mvop(int sz, const EA& d, const EA& s) { static const int szc[3] = {1, 3, 2}; return (uint16_t)((szc[sz] << 12) | (d.reg << 9) | (d.mode << 6) | ea6(s)); }
  // at: byte offset of the EA's first extension word from the instruction start; adj: A6 change by the instruction's
  // source operand before this EA is computed (MOVE (A6)+ / -(A6) source); maxl: bd and od long, memory indirect.
  // fullea(): generate a full-format effective address (the 68020 extension word with base and index suppression, base and outer displacements and memory indirection) together with the instructions that set up the
  // registers and memory it needs, so that the effective address ends up at a known target T. pc_ok = a PC-relative base is allowed; at = byte offset of the extension word from the instruction start.
  // The extension word: bit 8 = 1 for the full format, bit 7 = base suppress, bit 6 = index suppress, bits 5-4 = base displacement size (1 none, 2 word, 3 long), bits 2-0 = memory indirect mode (with bit 2 = post-indexed).
  EA fullea(Unit& u, bool pc_ok, int at, int32_t adj = 0, uint32_t* tgt = nullptr, bool maxl = false) {
    // kb = the base choice: A6, ZA6 (suppressed), PC, ZPC.
    int kb = r(8);
    // pcb = PC as base; bs = base suppressed.
    bool pcb = pc_ok && (kb == 0 || kb == 2), bs = (kb == 1 || kb == 2);         // kb 2: ZPC
    // mi = memory indirect; is = index suppressed; post = post-indexed (the index is added after the indirection).
    bool mi = maxl || r(2), is = !maxl && r(4) == 0, post = mi && !is && r(2);
    // bdsz and odsz = sizes of the base and the outer displacement (0 none, 1 null, 2 word, 3 long).
    int bdsz = maxl ? 3 : 1 + (int)r(3), odsz = mi ? (maxl ? 3 : 1 + (int)r(3)) : 0;
    // With a suppressed base or PC as base a long base displacement is used, because a word would not reach the data area.
    if (bs || pcb) bdsz = 3;                                                      // base 0 or PC: a long bd reaches the data area
    // xk = index register number, xl = long index, sc = scale.
    int xk = 0, xl = r(2), sc = r(4); bool xa6 = false; int32_t xv = 0;
    if (!is) {
      // Sometimes A6 itself is the index (as a sign extended word); otherwise a free data register is loaded with a small value.
      if (adj == 0 && r(4) == 0) { xa6 = true; xl = 0; xv = (int16_t)a6v; }
      else { do xk = r(8); while ((ix_used >> xk) & 1); ix_used |= 1 << xk; xv = (int32_t)r(0x800) - 0x400; u.movel(xk, (uint32_t)xv); }
    }
    // iv = the index value after scaling.
    int32_t iv = is ? 0 : (int32_t)((uint32_t)xv << sc);
    size_t plant = 0;
    // For memory indirect: plant a MOVE.L #V,(P).L (opcode 0x23FC and four words) that writes the pointer V to the address P just before the instruction; the words are filled in below.
    if (mi) { u.nx(); plant = code.size(); code.push_back(0x23FC); for (int i = 0; i < 4; i++) code.push_back(0); }
    // B = the base value, T = the target address of the operand.
    uint32_t B = bs ? 0 : pcb ? pc() + at : a6v + adj;
    uint32_t T = daddr();
    int32_t bd = 0, od = 0;
    // Without memory indirection: the displacement makes the effective address equal to T.
    if (!mi) {
      if (bdsz == 1) T = B + iv; else if (bdsz == 2) { bd = (int16_t)r(0x10000); T = B + bd + iv; } else bd = (int32_t)(T - B - iv);
    // With memory indirection: P = the address of the pointer; the pointer value V is chosen so that the final target is T.
    } else {
      uint32_t P = daddr();
      int32_t pre = post ? 0 : iv;
      if (bdsz == 1) P = B + pre; else if (bdsz == 2) { bd = (int16_t)r(0x10000); P = B + bd + pre; } else bd = (int32_t)(P - B - pre);
      if (odsz == 2) od = (int16_t)r(0x10000); else if (odsz == 3) od = (int32_t)r(0x200000) - 0x100000;
      uint32_t V = T - od - (post ? iv : 0);
      code[plant + 1] = (uint16_t)(V >> 16); code[plant + 2] = (uint16_t)V; code[plant + 3] = (uint16_t)(P >> 16); code[plant + 4] = (uint16_t)P;
    }
    // The EA field of the extension word.
    int iis = !mi ? 0 : ((post ? 4 : 0) | odsz);
    // Assemble the extension word.
    uint16_t ext = (uint16_t)((is ? 0 : xa6 ? 0xE000 : (xk << 12)) | (xl << 11) | (sc << 9) | 0x100 | (bs << 7) | (is << 6) | (bdsz << 4) | iis);
    // The EA is mode 6 (An) or mode 7 register 3 (PC) with the full format.
    EA e = {pcb ? 7 : 6, pcb ? 3 : 6, {ext}};
    // Append the displacement words.
    if (bdsz == 2) e.ext.push_back((uint16_t)bd);
    if (bdsz == 3) { e.ext.push_back((uint16_t)(bd >> 16)); e.ext.push_back((uint16_t)bd); }
    if (odsz == 2) e.ext.push_back((uint16_t)od);
    if (odsz == 3) { e.ext.push_back((uint16_t)(od >> 16)); e.ext.push_back((uint16_t)od); }
    // Return the target address if wanted.
    if (tgt) *tgt = T;
    return e;
  }
  // a6set(): MOVEA.L #a6v,A6 as the first instruction of a unit.
  void a6set(Unit& u) { ix_used = 0; a6v = daddr(); u.nx(); w3(0x2C7C, a6v); }       // MOVEA.L #a6v,A6
  // emit(): write an opcode, middle words (immediate or mask) and the EA words.
  void emit(uint16_t op, const std::vector<uint16_t>& mid, const EA& e) {               // opcode, middle words, EA words
    code.push_back(op); for (auto x : mid) code.push_back(x); for (auto x : e.ext) code.push_back(x);
  }

  // g_ea(): one of 14 instruction forms with a full-format EA (or the other forms that a full-format EA is used in).
  void g_ea() {
    Unit u(*this);
    a6set(u);
    int sz = r(3), dn = r(8);
    switch (r(14)) {
      // MOVE <full>,Dn or <mem>.
      case 0: {                                                                  // MOVE <full>,Dn / <mem>
        EA s = fullea(u, true, 2), d = r(3) == 0 ? EA{0, (int)r(8), {}} : mem(sz, true);
        u.nx(); put(mvop(sz, d, s), s, &d); ea_fix(d, sz); break;
      }
      // MOVE <any>,<full>.
      case 1: {                                                                  // MOVE <any>,<full>
        EA s = src(sz, true, true), d = fullea(u, false, 2 + 2 * (int)s.ext.size());
        u.nx(); put(mvop(sz, d, s), s, &d); ea_fix(s, sz); break;
      }
      // MOVE <full>,<full>.
      case 2: {                                                                  // MOVE <full>,<full>
        EA s = fullea(u, false, 2), d = fullea(u, false, 2 + 2 * (int)s.ext.size());
        u.nx(); put(mvop(sz, d, s), s, &d); break;
      }
      // OR, SUB, CMP, AND, ADD <full>,Dn.
      case 3: {                                                                  // OR SUB CMP AND ADD <full>,Dn
        static const int ops[5] = {0x8, 0x9, 0xB, 0xC, 0xD};
        EA s = fullea(u, true, 2); u.nx(); put((uint16_t)((ops[r(5)] << 12) | (dn << 9) | (sz << 6) | ea6(s)), s); break;
      }
      // OR, SUB, AND, ADD, EOR Dn,<full>.
      case 4: {                                                                  // OR SUB AND ADD EOR Dn,<full>
        static const int ops[5] = {0x8, 0x9, 0xC, 0xD, 0xB};
        EA d = fullea(u, false, 2); u.nx(); put((uint16_t)((ops[r(5)] << 12) | (dn << 9) | ((4 + sz) << 6) | ea6(d)), d); break;
      }
      // ORI, ANDI, SUBI, ADDI, EORI, CMPI #imm,<full>.
      case 5: {                                                                  // ORI..CMPI #,<full>
        static const int ops[6] = {0x0000, 0x0200, 0x0400, 0x0600, 0x0A00, 0x0C00};
        int op = ops[r(6)]; std::vector<uint16_t> im;
        if (sz == 2) { uint32_t v = rng(); im = {(uint16_t)(v >> 16), (uint16_t)v}; } else im = {(uint16_t)(sz == 0 ? r(256) : r(65536))};
        EA d = fullea(u, op == 0x0C00, 2 + 2 * (int)im.size());
        u.nx(); emit((uint16_t)(op | (sz << 6) | ea6(d)), im, d); break;
      }
      // NEGX, CLR, NEG, NOT, TST, Scc <full>.
      case 6: {                                                                  // NEGX CLR NEG NOT TST Scc
        int k = r(6);
        EA d = fullea(u, k == 4, 2); u.nx();
        if (k < 5) { static const int ops[5] = {0x4000, 0x4200, 0x4400, 0x4600, 0x4A00}; put((uint16_t)(ops[k] | (sz << 6) | ea6(d)), d); }
        else put((uint16_t)(0x50C0 | (r(16) << 8) | ea6(d)), d);
        break;
      }
      // BTST, BCHG, BCLR, BSET with a bit number in a register or an immediate.
      case 7: {                                                                  // BTST BCHG BCLR BSET #n / Dn
        int tt = r(4), dyn = r(2);
        EA d = fullea(u, tt == 0, dyn ? 2 : 4); u.nx();
        if (dyn) put((uint16_t)(0x0100 | (dn << 9) | (tt << 6) | ea6(d)), d);
        else emit((uint16_t)(0x0800 | (tt << 6) | ea6(d)), {(uint16_t)r(256)}, d);
        break;
      }
      // LEA <full>,A6 or PEA <full> and MOVE.L (A7)+,Dn.
      case 8: {                                                                  // LEA <full>,A6 / PEA <full>; MOVE.L (A7)+,Dn
        EA s = fullea(u, true, 2); u.nx();
        if (r(2)) put((uint16_t)(0x4DC0 | ea6(s)), s);
        else { put((uint16_t)(0x4840 | ea6(s)), s); u.nx(); code.push_back((uint16_t)(0x201F | (dn << 9))); }
        break;
      }
      // MOVEM <full>,registers or registers,<full>.
      case 9: {                                                                  // MOVEM <full>,regs / regs,<full>
        int ld = r(2), l = r(2);
        uint16_t mask = (uint16_t)rng(); if (ld) mask &= 0x40FF;
        EA e = fullea(u, ld != 0, 4); u.nx();
        emit((uint16_t)(0x4880 | (ld << 10) | (l << 6) | ea6(e)), {mask}, e);
        break;
      }
      // MULU.W, MULS.W, MULU.L, MULS.L <full>,Dn.
      case 10: {                                                                 // MULU.W MULS.W / MULU.L MULS.L <full>,Dn
        int lg = r(2), sg = r(2);
        EA s = fullea(u, true, lg ? 4 : 2); u.nx();
        if (lg) emit((uint16_t)(0x4C00 | ea6(s)), {(uint16_t)((dn << 12) | (sg << 11))}, s);
        else put((uint16_t)(0xC0C0 | (dn << 9) | (sg << 8) | ea6(s)), s);
        break;
      }
      // BFEXTU or BFTST <full>{offset:width}.
      case 11: {                                                                 // BFEXTU / BFTST <full>{#o:#w}
        int t = r(2) ? 1 : 0;
        EA s = fullea(u, true, 4); u.nx();
        emit((uint16_t)(0xE8C0 | (t << 8) | ea6(s)), {(uint16_t)((dn << 12) | (r(32) << 6) | r(32))}, s);
        break;
      }
      // MOVE (A6)+ or -(A6),<full>: the destination EA sees the step of the source.
      case 12: {                                                                 // MOVE (A6)+ / -(A6),<full>: the destination sees the step
        EA s = {r(2) ? 3 : 4, 6, {}};
        int st = stp(sz, 6);
        EA d = fullea(u, false, 2, s.mode == 3 ? st : -st);
        u.nx(); put(mvop(sz, d, s), s, &d); break;
      }
      // JMP (bd.L) or JMP ([bd.L]) to a later instruction (patched in build()).
      default: {                                                                 // JMP (bd.L) / JMP ([bd.L]) to a later instruction
        int skip = r(4);
        if (r(2)) {
          u.nx(); code.push_back(0x4EF6); code.push_back(0x01F0);
          fx.push_back({start.size() - 1, skip, 1, (uint32_t)code.size(), 0, 0}); code.push_back(0); code.push_back(0);
        } else {
          uint32_t P = daddr();
          u.nx(); code.push_back(0x23FC); size_t at = code.size(); code.push_back(0); code.push_back(0);
          code.push_back((uint16_t)(P >> 16)); code.push_back((uint16_t)P);
          u.nx(); code.push_back(0x4EF6); code.push_back(0x01F1); code.push_back((uint16_t)(P >> 16)); code.push_back((uint16_t)P);
          fx.push_back({start.size() - 1, skip, 1, (uint32_t)at, 0, 0});
        }
      }
    }
  }

  // g_long(): instructions with long encodings (several extension words), the MOVE destinations and the other forms that the earlier groups do not produce.
  void g_long() {
    Unit u(*this);
    int sz = r(3), dn = r(8);
    // absl(): an absolute long EA.
    auto absl = [&](uint32_t a) { return EA{7, 1, {(uint16_t)(a >> 16), (uint16_t)a}}; };
    switch (r(6)) {
      // ORI to CMPI #imm,abs.L.
      case 0: {                                                                  // ORI..CMPI.x #imm,abs.L
        static const int ops[6] = {0x0000, 0x0200, 0x0400, 0x0600, 0x0A00, 0x0C00};
        std::vector<uint16_t> im;
        if (sz == 2) { uint32_t v = rng(); im = {(uint16_t)(v >> 16), (uint16_t)v}; } else im = {(uint16_t)(sz == 0 ? r(256) : r(65536))};
        EA d = absl(daddr()); u.nx(); emit((uint16_t)(ops[r(6)] | (sz << 6) | ea6(d)), im, d); break;
      }
      // MOVE with an immediate, absolute long or any source, to an absolute long or absolute word destination.
      case 1: {                                                                  // MOVE #imm / abs.L / any,abs.L / abs.W
        EA s = r(3) == 0 ? absl(daddr()) : r(2) ? imm(sz) : src(sz, true, true);
        EA d = r(2) ? absl(daddr()) : EA{7, 0, {(uint16_t)(0x8000 + r(0x7ff0))}};
        u.nx(); put(mvop(sz, d, s), s, &d); ea_fix(s, sz); break;
      }
      // MOVE <any>,(d8,An,Xn).
      case 2: {                                                                  // MOVE <any>,(d8,An,Xn)
        EA s = src(sz, true, true), d = {6, base(), {brief(sz)}};
        u.nx(); put(mvop(sz, d, s), s, &d); ea_fix(s, sz); break;
      }
      // BTST Dn,#imm (legal on the 68020 and illegal on the 68000).
      case 3: u.nx(); w2((uint16_t)(0x013C | (dn << 9)), (uint16_t)rng()); break;   // BTST Dn,#imm
      // CMPI #imm,(d16,PC) and (d8,PC,Xn): legal on the 68020 only.
      case 4: {                                                                  // CMPI #,(d16,PC) / (d8,PC,Xn)
        std::vector<uint16_t> im;
        if (sz == 2) { uint32_t v = rng(); im = {(uint16_t)(v >> 16), (uint16_t)v}; } else im = {(uint16_t)(sz == 0 ? r(256) : r(65536))};
        u.nx(); EA d = pcrel(sz, 2 + 2 * (int)im.size()); emit((uint16_t)(0x0C00 | (sz << 6) | ea6(d)), im, d); break;
      }
      // MOVE ([bd.L,..],od.L),([bd.L,..],od.L): two memory indirect EAs, 11 words in all.
      default: {                                                                 // MOVE ([bd.L,..],od.L),([bd.L,..],od.L): 11 words
        a6set(u);
        EA s = fullea(u, false, 2, 0, nullptr, true), d = fullea(u, false, 12, 0, nullptr, true);
        u.nx(); put(mvop(sz, d, s), s, &d); break;
      }
    }
  }

  // g_cas(): CAS (compare and swap) and CAS2 (two operands).
  void g_cas() {
    Unit u(*this);
    a6set(u);
    // szc[]: the size field of MOVE for the helper that plants a value in memory.
    static const int szc[3] = {1, 3, 2};
    int sz = r(3), dc = r(8), du = r(8);
    // v = the value to compare with, masked to the operand size.
    uint32_t v = rng() & (sz == 0 ? 0xff : sz == 1 ? 0xffff : 0xffffffffu);
    // plant(): MOVE.x #val,(a).L: writes the memory value that the CAS will compare against.
    auto plant = [&](uint32_t a, uint32_t val, int s) {                         // MOVE.x #val,(a).L
      u.nx(); code.push_back((uint16_t)((szc[s] << 12) | 0x03FC));
      if (s == 2) code.push_back((uint16_t)(val >> 16)); code.push_back((uint16_t)val);
      code.push_back((uint16_t)(a >> 16)); code.push_back((uint16_t)a);
    };
    // setc(): load the compare register Dc with the value (or with something different, a third of the time).
    auto setc = [&](int d, uint32_t val, int s) {                               // Dc: equal to val in the operand size, or not
      uint32_t m = s == 0 ? 0xff : s == 1 ? 0xffff : 0xffffffffu;
      uint32_t c = (rng() & ~m) | (r(3) ? val : (val ^ (1u + r(m == 0xff ? 255 : 1000)))) & m;
      if (s == 2) c = r(3) ? val : rng();
      u.movel(d, c);
    };
    // CAS: the EA is (A6), d16(A6), (A6)+, -(A6), absolute long, a full-format EA or random memory (where the compare usually fails).
    if (r(3)) {                                                                  // CAS
      if (sz == 0 && r(2)) sz = 1 + r(2);
      int k = r(7);
      uint32_t x = 0; EA e;
      switch (k) {
        case 0: e = {2, 6, {}}; x = a6v; break;
        case 1: { int d = (int)r(0x2000) - 0x1000; e = {5, 6, {(uint16_t)d}}; x = a6v + d; break; }
        case 2: e = {3, 6, {}}; x = a6v; break;
        case 3: e = {4, 6, {}}; x = a6v - stp(sz, 6); break;
        case 4: { x = daddr(); e = {7, 1, {(uint16_t)(x >> 16), (uint16_t)x}}; break; }
        case 5: e = fullea(u, false, 4, 0, &x); break;
        default: e = mem(sz, true); break;                                       // random memory: usually not equal
      }
      // The index register of a full EA stays as it was set.
      while ((ix_used >> dc) & 1) dc = r(8);                                    // the index register stays as set
      if (k != 6) plant(x, v, sz);
      setc(dc, v, sz);
      u.nx(); emit((uint16_t)(0x08C0 | ((sz + 1) << 9) | ea6(e)), {(uint16_t)((du << 6) | dc)}, e);
      if (k == 6) ea_fix(e, sz);
    // CAS2: two compare registers, two update registers, two address registers; sometimes both addresses are the same.
    } else {                                                                     // CAS2
      int lg = r(2), s2 = lg ? 2 : 1;
      int rn1 = r(8), rn2 = r(8);
      // Rn1 = A6 only for the long form, because Musashi sign extends Dc for the word form with an address register.
      bool a1 = lg && r(3) == 0;                                                  // Rn1 = A6 (.L only: Musashi sign-extends Dc for .W with An)
      uint32_t p1 = a1 ? a6v : daddr(), p2 = r(4) == 0 ? p1 : daddr();
      if (!a1) u.movel(rn1, p1);
      if (rn2 == rn1 && !a1 && p2 != p1) rn2 = (rn1 + 1) & 7;
      if (rn2 != rn1 || a1) u.movel(rn2, p2); else p2 = p1;
      uint32_t m = lg ? 0xffffffffu : 0xffff;
      uint32_t v1 = rng() & m, v2 = (p2 == p1) ? v1 : rng() & m;
      plant(p1, v1, s2); if (p2 != p1) plant(p2, v2, s2);
      int dc1 = r(8), dc2 = r(8);
      while (dc1 == rn1 || dc1 == rn2) dc1 = r(8);
      while (dc2 == rn1 || dc2 == rn2 || dc2 == dc1) dc2 = r(8);
      setc(dc1, v1, s2); setc(dc2, v2, s2);
      int du1 = r(8), du2 = r(8);
      u.nx(); code.push_back(lg ? 0x0EFC : 0x0CFC);
      code.push_back((uint16_t)(((a1 ? 0x8000 : 0) | ((a1 ? 6 : rn1) << 12)) | (du1 << 6) | dc1));
      code.push_back((uint16_t)((rn2 << 12) | (du2 << 6) | dc2));
    }
  }


  // ---- FPU (groups 36 to 38) ----
  // FPCR keeps the exception enables 0 in most programs (traps are described in docs/learn/HISTORY.md, section 13). Values come from FMOVE.S/.L #imm (special bit patterns), from random memory (.D and .X: any exponent, NaNs, denormals) and from earlier results.
  // fsz(): integer access size code of the formats byte, word, long and single.
  static int fsz(int f) { return f == 6 ? 0 : f == 4 ? 1 : 2; }          // integer access size code of formats B W L S
  // fsingle(): FMOVE.S #v,FPn or FMOVE.L #v,FPn.
  void fsingle(int fp) {                                                // FMOVE.S #v,FPn or FMOVE.L #v,FPn
    // Special single precision bit patterns: zeros, infinities, NaNs, denormals, 1.0, -1.0, largest and smallest values and others.
    static const uint32_t sp[] = {0x00000000, 0x80000000, 0x7F800000, 0xFF800000, 0x7FC00000, 0xFFC00001, 0x7F800001,
                                  0x00000001, 0x807FFFFF, 0x00800000, 0x3F800000, 0xBF800000, 0x7F7FFFFF, 0x3F000000,
                                  0x4B000000, 0x3EAAAAAB, 0x40490FDB, 0x5F000000, 0xCF000000, 0x3FC00000};
    // Mostly a special pattern, sometimes random bits.
    uint32_t v = r(3) ? sp[r(sizeof sp / 4)] : (uint32_t)rng();
    // FMOVE.S #imm,FPn (0xF23C and command word 0x4400 + register).
    code.push_back(0xF23C);
    // A quarter of the time FMOVE.L #imm,FPn (command word 0x4000 + register) with a random or small integer.
    if (r(4) == 0) { code.push_back((uint16_t)(0x4000 | (fp << 7))); v = r(2) ? (uint32_t)rng() : (uint32_t)((int)r(41) - 20); }
    else code.push_back((uint16_t)(0x4400 | (fp << 7)));
    code.push_back((uint16_t)(v >> 16)); code.push_back((uint16_t)v);
  }
  // fpcr_set(): FMOVE.L #v,FPCR, with a random rounding mode (bits 5-4) and precision (bits 7-6).
  void fpcr_set() {                                                     // FMOVE.L #v,FPCR: rounding mode and precision
    uint32_t v = (r(4) << 4) | (r(4) << 6);
    // With the system groups on, sometimes also exception enables (traps go through the FPU trap handler); BSUN is not enabled because it would trap the same conditional again.
    if (sys_on() && r(3) == 0) v |= ((uint32_t)r(256) & 0x7F) << 8;   // exception enables (traps through H_FPX); not BSUN: it traps the same conditional again
    code.push_back(0xF23C); code.push_back(0x9000); code.push_back((uint16_t)(v >> 16)); code.push_back((uint16_t)v);
  }
  // fea(): a random EA for format f: a data register (B, W, L, S only), memory, immediate (source only) or PC-relative (source only).
  EA fea(int f, bool dst) {                                             // EA of format f: Dn for B W L S, memory, #imm, PC-relative
    int isz = (f == 5 || f == 2) ? 2 : fsz(f);
    bool one = (f == 0 || f == 1 || f == 4 || f == 6);
    int k = r(8);
    if (one && k < 2) return {0, (int)r(8), {}};
    if (!dst && one && k == 2) return imm(isz);
    if (!dst && k == 3) return pcrel(isz, 4);
    return mem(isz, true, f == 6 || f == 4);
  }
  void fea_fix(const EA& e, int f) { if (f == 6 || f == 4) ea_fix(e, fsz(f)); }
  // fput(): write the opcode (0xF200 plus the EA) and the command word and the extension words.
  void fput(uint16_t w2, const EA& e) {
    code.push_back((uint16_t)(0xF200 | ea6(e))); code.push_back(w2);
    for (auto x : e.ext) code.push_back(x);
  }
  // fop(): a random opmode from the list that the core does in hardware (FMOVE, FINT, FINTRZ, FSQRT, FABS, FNEG, FGETEXP, FGETMAN, FDIV, FADD, FMUL, FSGLDIV, FSCALE, FSGLMUL, FSUB, FCMP, FTST and aliases).
  int fop() {                                                           // opmodes done in hardware
    static const int ops[] = {0x00, 0x01, 0x03, 0x04, 0x18, 0x1a, 0x1e, 0x1f, 0x20, 0x22, 0x22, 0x23, 0x23, 0x24, 0x26, 0x27,
                              0x28, 0x28, 0x38, 0x3a, 0x05, 0x1b, 0x2b, 0x39, 0x3e};
    return ops[r(sizeof ops / sizeof ops[0])];
  }
  // g_fpu(): arithmetic and FMOVE.
  void g_fpu() {
    // k = which form, fp = a destination register.
    int k = r(16), fp = r(8);
    // FMOVE.L #v,FPCR.
    if (k == 0) { fpcr_set(); return; }
    // FMOVE.S/.L #v,FPn.
    if (k <= 2) { fsingle(fp); return; }
    // A trapped instruction (line F, skipped by the handler) when the system groups are on.
    if (k == 3 && sys_on()) {                                           // a trapped instruction: line F, skipped by the handler
      static const int tr[] = {0x0e, 0x1d, 0x10, 0x14, 0x21, 0x25, 0x41, 0x60};
      code.push_back(0xF200); code.push_back((uint16_t)((r(8) << 10) | (fp << 7) | tr[r(8)])); return;
    }
    // FMOVE FPn,<ea>.
    if (k <= 5) {                                                       // FMOVE FPn,<ea>
      static const int fm[] = {0, 1, 2, 4, 5, 6};
      int f = fm[r(6)]; EA e = fea(f, true);
      fput((uint16_t)(0x6000 | (f << 10) | (fp << 7)), e); fea_fix(e, f); return;
    }
    // FMOVECR, also with undefined ROM offsets.
    if (k == 6) {                                                       // FMOVECR
      static const int co[] = {0x00, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
                               0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f};
      code.push_back(0xF200); code.push_back((uint16_t)(0x5C00 | (fp << 7) | (r(4) ? co[r(22)] : (int)r(64)))); return;   // undefined offsets too
    }
    int op = fop();
    // Register to register: FPm,FPn with a random opmode.
    if (k <= 9) { code.push_back(0xF200); code.push_back((uint16_t)((r(8) << 10) | (fp << 7) | op)); return; }   // FPm,FPn
    // Memory or register source: format and EA, then the operation.
    static const int fm[] = {0, 1, 2, 4, 5, 6};
    int f = fm[r(6)]; EA e = fea(f, false);
    fput((uint16_t)(0x4000 | (f << 10) | (fp << 7) | op), e); fea_fix(e, f);
  }
  // g_fpc(): a compare or test and then a conditional: FBcc, FScc, FDBcc, FNOP or FTRAPcc.
  void g_fpc() {                                                        // a compare or test, then a conditional
    Unit u(*this);
    // c = the condition (0x20-0x3F act as 0x00-0x1F).
    int c = r(64);                                                      // 0x20-0x3F act as 0x00-0x1F
    // Half of the time an FCMP or FTST first, so that the condition codes have defined values.
    if (r(2)) { u.nx(); code.push_back(0xF200); code.push_back((uint16_t)((r(8) << 10) | (r(8) << 7) | (r(2) ? 0x38 : 0x3a))); }
    int k = r(sys_on() ? 5 : 4);
    u.nx();
    // FBcc.W or FBcc.L forward.
    if (k == 0) {                                                       // FBcc .W / .L forward
      int form = r(2) ? 3 : 4;
      br.push_back({start.size() - 1, (int)r(4) + 1, form});
      code.push_back((uint16_t)((form == 3 ? 0xF280 : 0xF2C0) | c)); code.push_back(0); if (form == 4) code.push_back(0);
    // FScc.
    } else if (k == 1) {                                                // FScc
      EA e = r(2) ? EA{0, (int)r(8), {}} : mem(0, true, true);
      code.push_back((uint16_t)(0xF240 | ea6(e))); code.push_back((uint16_t)c); for (auto x : e.ext) code.push_back(x);
      ea_fix(e, 0);
    // FDBcc forward.
    } else if (k == 2) {                                                // FDBcc forward
      int dn = r(8);
      br.push_back({start.size() - 1, (int)r(4) + 1, 5});
      code.push_back((uint16_t)(0xF248 | dn)); code.push_back((uint16_t)c); code.push_back(0);
    // FNOP (FBF.W with displacement 0).
    } else if (k == 3) {                                                // FNOP (FBF.W 0)
      code.push_back(0xF280); code.push_back(0);
    // FTRAPcc (vector 7, handler RTE).
    } else {                                                            // FTRAPcc (vector 7, handler RTE)
      int fm = 2 + r(3);
      code.push_back((uint16_t)(0xF278 | fm)); code.push_back((uint16_t)c);
      if (fm == 2) code.push_back((uint16_t)rng()); else if (fm == 3) { code.push_back((uint16_t)rng()); code.push_back((uint16_t)rng()); }
    }
  }
  // g_fpm(): FMOVEM, control register moves and FSAVE/FRESTORE.
  void g_fpm() {
    int k = r(8);
    // FMOVEM.X with a static register list.
    if (k <= 2) {                                                       // FMOVEM.X static list
      int lst = r(256), st_ = r(2), an = base();
      EA e;
      if (st_) e = r(2) ? EA{4, an, {}} : mem(2, false);
      else e = r(2) ? EA{3, an, {}} : (r(3) ? mem(2, false) : pcrel(2, 4));
      // The list type (pre-decrement or not) and the EA may disagree, which is a case the model has to get right.
      bool pre = r(4) ? (e.mode == 4) : r(2);                           // list type and EA may disagree
      // Absolute word EAs: 96 bytes must not wrap into the code.
      if (e.mode == 7 && e.reg == 0) e.ext[0] = (uint16_t)(0x8000 + (r(0x7000) & ~3u));   // abs.W: 96 bytes must not wrap into the code
      code.push_back((uint16_t)(0xF200 | ea6(e))); code.push_back((uint16_t)((st_ ? 0xE000 : 0xC000) | (pre ? 0 : 0x1000) | lst));
      for (auto x : e.ext) code.push_back(x);
      return;
    }
    // Control register moves. FPCR is only loaded from an immediate with just the rounding and precision bits, so the exception enables stay 0.
    if (k <= 5) {                                                       // control registers (FPCR from memory/Dn: never, enables stay 0)
      static const int l1[] = {4, 2, 1};
      int dir = r(2), m = r(4), lst = l1[r(3)];
      EA e;
      if (m == 0) e = {0, (int)r(8), {}};
      else if (m == 1) { e = {1, 6, {}}; lst = 1; }
      else if (m == 2 && !dir) { uint32_t v = rng(); if (lst == 4) v &= 0xF0; e = {7, 4, {(uint16_t)(v >> 16), (uint16_t)v}}; }
      else { int l = r(8); lst = l ? l : 1; e = mem(2, true); }
      if (!dir && (lst & 4) && !(e.mode == 7 && e.reg == 4)) { lst &= 3; if (!lst) lst = 2; }
      if (e.mode == 0 && lst != 4 && lst != 2 && lst != 1) lst = 2;
      // No stack pointer copies when the system groups are on.
      if (e.mode == 1 && dir == 0 && sys_on()) { e = {0, (int)r(8), {}}; }  // no stack pointer copies: A6 only read
      // An empty list means FPIAR.
      if (r(16) == 0 && lst == 1) lst = 0;                              // empty list: FPIAR
      code.push_back((uint16_t)(0xF200 | ea6(e))); code.push_back((uint16_t)((dir ? 0xA000 : 0x8000) | (lst << 10)));
      for (auto x : e.ext) code.push_back(x);
      return;
    }
    // FSAVE -(An) and FRESTORE (An)+ (or a NULL frame written by hand with MOVE.L #0,-(An)).
    if (k == 6 && fsave_ok()) {                                         // FSAVE -(An); FRESTORE (An)+ (or a NULL frame)
      Unit u(*this); int an = base();
      if (an == 7) an = 0;
      u.nx();
      if (r(4)) code.push_back((uint16_t)(0xF320 | an));               // FSAVE -(An)
      else { code.push_back((uint16_t)(0x2000 | (an << 9) | (4 << 6) | 0x3C)); code.push_back(0); code.push_back(0); }   // MOVE.L #0,-(An)
      u.nx(); code.push_back((uint16_t)(0xF358 | an));                  // FRESTORE (An)+
      return;
    }
    fsingle(r(8));
  }
  // fsave_ok(): FSAVE and FRESTORE are generated only when the system groups are off, because they are privileged and the program would otherwise run them in user mode.
  bool fsave_ok() { return !((groups >> G_SYS) & 1) && !((groups >> G_TRAP) & 1); }   // supervisor only (no user-mode stretches)
  // ins(): register the next instruction (a possible branch target) at the current position.
  void ins() { start.push_back((uint32_t)code.size()); notgt.push_back(0); }   // next instruction starts here
  // cont(): register the next instruction as part of a multi-instruction unit (never a branch target).
  void cont() { start.push_back((uint32_t)code.size()); notgt.push_back(1); }  // ... inside a unit

  // one(): generate the next instruction or unit: pick a group by weight and call its generator.
  void one() {
    ins();
    // dn = a random data register, sz = a random size code (0 byte, 1 word, 2 long), used by the simple groups below.
    int dn = r(8), sz = r(3);
    // tot = the sum of the weights of the groups that are switched on, grp = the chosen group.
    int tot = 0, grp = 0;
    for (int i = 0; i < G_BITS; i++) if ((groups >> i) & 1) tot += gweight(i);  // add up the weights of the groups that are on
    // Walk through the groups again with a random number below the total, to find the chosen one.
    for (int k = tot ? (int)r(tot) : 0, i = 0; tot && i < G_BITS; i++)
      if ((groups >> i) & 1) { if (k < gweight(i)) { grp = i; break; } k -= gweight(i); }
    // The groups with their own generator functions:
    if (grp == G_SYS) { g_sys(); return; }
    if (grp == G_TRAP) { g_trap(); return; }
    if (grp == G_CALL) { g_call(); return; }  // program flow: calls, stack frames and loops
    if (grp == G_STACK) { g_stack(); return; }
    if (grp == G_LOOP) { g_loop(); return; }
    if (grp == G_MUL) { g_mul(); return; }
    if (grp == G_DIV) { g_div(); return; }
    if (grp == G_BF_R || grp == G_BF_M) { g_bf(grp == G_BF_M); return; }  // bit fields in registers and in memory
    if (grp == G_EA) { g_ea(); return; }  // full-format effective addresses, long instructions and CAS
    if (grp == G_LONG) { g_long(); return; }
    if (grp == G_CAS) { g_cas(); return; }
    if (grp == G_DMA) { g_dma(); return; }  // DMA window loads and self-modifying code
    if (grp == G_SMC) { g_smc(); return; }
    if (grp == G_FPU) { g_fpu(); return; }  // FPU arithmetic, polling loop, FPU conditionals and FMOVEM
    if (grp == G_ULOOP) { g_uloop(); return; }
    if (grp == G_FPC) { g_fpc(); return; }
    if (grp == G_FPM) { g_fpm(); return; }
    // The remaining groups are simple enough to be written in place.
    switch (grp) {
      // MOVEQ #imm,Dn.
      case G_MOVEQ: code.push_back((uint16_t)(0x7000 | (dn << 9) | r(256))); break;                      // MOVEQ
      // MOVE: size code, source EA, destination EA. szc[] converts the size (0 byte, 1 word, 2 long) to the MOVE size field (1 byte, 3 word, 2 long).
      case G_MOVE: {                                                                            // MOVE
        static const int szc[3] = {1, 3, 2};
        EA s = src(sz, true, true), d;
        int k = r(4);
        // Destination: a data register, an address register (MOVEA, not for byte size), or any memory EA.
        if (k == 0) d = {0, (int)r(8), {}};
        else if (k == 1 && sz != 0) {                                                              // MOVEA
          // A MOVEA from an address register may go into a base register (aligned), else into A6.
          if (s.mode == 1 && s.reg != 6 && sz == 2 && r(2)) d = {1, base(), {}}; else d = {1, 6, {}};
          if (d.reg == 7 && sys_on()) d.reg = 6;                                                 // no stack pointer copies
        } else {
          d = mem(sz, true);  // any alterable memory destination
        }
        put((uint16_t)((szc[sz] << 12) | (d.reg << 9) | (d.mode << 6) | ea6(s)), s, &d);
        break;
      }
      // ALU <ea>,Dn: OR, SUB, CMP, AND, ADD with a source EA (an address register only for SUB, CMP and ADD).
      case G_ALU_EA_DN: {                                                                            // ALU <ea>,Dn
        static const int ops[5] = {0x8, 0x9, 0xB, 0xC, 0xD};
        int op = ops[r(5)];
        EA s = src(sz, op == 0x9 || op == 0xB || op == 0xD, true);
        put((uint16_t)((op << 12) | (dn << 9) | (sz << 6) | ea6(s)), s);
        break;
      }
      // ALU Dn,<ea>: OR, SUB, AND, ADD and EOR to a memory EA (CMP has no such form; the 0xB group with the memory EA is EOR).
      case G_ALU_DN_EA: {                                                                                    // ALU Dn,<ea>
        static const int ops[5] = {0x8, 0x9, 0xC, 0xD, 0xB};
        int op = ops[r(5)];
        EA d = (op == 0xB && r(3) == 0) ? EA{0, (int)r(8), {}} : mem(sz, true);
        put((uint16_t)((op << 12) | (dn << 9) | ((4 + sz) << 6) | ea6(d)), d);
        break;
      }
      // ADDA, SUBA, CMPA: the source is a small constant for a base register (so it stays aligned), anything for A6 and for CMPA.
      case G_ADDA: {                                                                                    // ADDA/SUBA/CMPA
        static const int ops[3] = {0x9, 0xB, 0xD};
        int op = ops[r(3)], l = r(2), an = r(8);
        EA s;
        if (op == 0xB || an == 6) s = src(l ? 2 : 1, true, true);
        else { int32_t v = ((int32_t)r(0x2000) - 0x1000) & ~3; s = l ? EA{7, 4, {(uint16_t)(v >> 16), (uint16_t)v}} : EA{7, 4, {(uint16_t)v}}; }
        put((uint16_t)((op << 12) | (an << 9) | ((l ? 7 : 3) << 6) | ea6(s)), s);
        break;
      }
      // ADDQ and SUBQ: the quick value is 4 or 8 for base registers (so they stay aligned).
      case G_ADDQ: {                                                                                    // ADDQ/SUBQ
        int q = r(8), sub = r(2);
        EA d;
        int k = r(3);
        if (k == 0) d = {0, (int)r(8), {}};
        else if (k == 1 && sz != 0) { d = {1, (int)r(8), {}}; if (d.reg != 6) q = r(2) ? 4 : 0; }   // q 4 or 8
        else d = mem(sz, true);
        put((uint16_t)(0x5000 | (q << 9) | (sub << 8) | (sz << 6) | ea6(d)), d);
        break;
      }
      // Bcc forward: a condition (not the BSR code 1), a form (byte, word or long displacement) and how many instructions to skip. The displacement is patched at the end of the program (build()).
      case G_BCC: {                                                                                    // Bcc forward
        int cc = r(16); if (cc == 1) cc = 0;
        int form = r(3), skip = r(4);
        if (form == 0 && skip == 0) skip = 1;
        br.push_back({start.size() - 1, skip, form});
        code.push_back((uint16_t)(0x6000 | (cc << 8)));
        if (form == 1) code.push_back(0);
        if (form == 2) { code.push_back(0); code.push_back(0); }
        break;
      }
      // LEA <ea>,An: with an aligned EA (al) when the result goes into a base register; never an (An)+ or -(An) source, and no absolute word base (an index could wrap into the code area).
      case G_LEA: {                                                                                    // LEA
        int an = r(8);
        if (an == 7 && sys_on()) an = 6;                                                        // no stack pointer copies
        EA s;
        al = (an != 6);  // base registers stay aligned
        if (an == 6 && r(2)) s = pcrel(2);
        else { do { s = mem(2, false); } while (s.mode == 3 || s.mode == 4 || (an != 6 && s.mode == 7 && s.reg == 0)); }  // no abs.W base: index could wrap into the code area
        al = false;
        put((uint16_t)(0x41C0 | (an << 9) | ea6(s)), s);
        break;
      }
      // Shifts and rotates of a register: type, direction, size, count by immediate or by register.
      case G_SHIFT: {                                                                              // ASx/LSx/ROx
        int ir = r(2), dir = r(2);
        code.push_back((uint16_t)(0xE000 | (r(8) << 9) | (dir << 8) | (sz << 6) | (ir << 5) | ((int)r(4) << 3) | dn));   // AS LS ROX RO
        break;
      }
      // ORI, ANDI, SUBI, ADDI, EORI, CMPI with a register or memory destination.
      case G_IMMOP: {                                                                              // ORI ANDI SUBI ADDI EORI CMPI
        static const int ops[6] = {0x0000, 0x0200, 0x0400, 0x0600, 0x0A00, 0x0C00};
        int op = ops[r(6)];
        EA d;
        if (r(3) == 0) d = {0, (int)r(8), {}};
        else d = memp(sz);  // all memory destinations, including long immediates with absolute long
        code.push_back((uint16_t)(op | (sz << 6) | ea6(d)));
        if (sz == 2) { uint32_t v = rng(); code.push_back((uint16_t)(v >> 16)); code.push_back((uint16_t)v); }
        else code.push_back((uint16_t)(sz == 0 ? r(256) : r(65536)));
        for (auto x : d.ext) code.push_back(x);
        ea_fix(d, sz);
        break;
      }
      // NEGX, CLR, NEG, NOT, TST, TAS, NOP.
      case G_UNARY: {                                                                              // NEGX CLR NEG NOT TST TAS NOP
        int k = r(7);
        if (k == 6) { code.push_back(0x4E71); break; }
        if (k == 5) { EA d = r(3) == 0 ? EA{0, (int)r(8), {}} : memp(0); put((uint16_t)(0x4AC0 | ea6(d)), d); ea_fix(d, 0); break; }
        if (k == 4) { EA s = src(sz, sz != 0, true); put((uint16_t)(0x4A00 | (sz << 6) | ea6(s)), s); break; }
        static const int ops[4] = {0x4000, 0x4200, 0x4400, 0x4600};
        EA d = r(3) == 0 ? EA{0, (int)r(8), {}} : memp(sz);
        put((uint16_t)(ops[k] | (sz << 6) | ea6(d)), d);
        ea_fix(d, sz);
        break;
      }
      // EXT, EXTB, SWAP, EXG (data registers, base registers with each other, or a data register with A6).
      case G_EXT: {                                                                                // EXT EXTB SWAP EXG
        int k = r(8);
        if (k < 4) { static const int ops[4] = {0x4880, 0x48C0, 0x49C0, 0x4840}; code.push_back((uint16_t)(ops[k] | dn)); break; }
        int f = r(3), x, y, opm;
        if (f == 0) { opm = 0x40; x = r(8); y = r(8); }                                            // Dx,Dy
        else if (f == 1) {                                                                         // bases swap, A6 with A6
          opm = 0x48; if (r(8) == 0) { x = 6; y = 6; } else { x = base(); y = base(); }
          if (sys_on() && (x == 7) != (y == 7)) x = y = 6;                                       // no stack pointer copies
        }
        else { opm = 0x88; x = r(8); y = 6; }                                                      // Dx,A6
        code.push_back((uint16_t)(0xC100 | (x << 9) | opm | y));
        break;
      }
      // Scc to a register or memory.
      case G_SCC: {                                                                                // Scc
        EA d = r(3) == 0 ? EA{0, (int)r(8), {}} : memp(0);
        put((uint16_t)(0x50C0 | ((int)r(16) << 8) | ea6(d)), d);
        ea_fix(d, 0);
        break;
      }
      // DBcc: a bounded loop MOVEQ #0..3,Dk; a body that never writes Dk; DBcc Dk,body.
      case G_DBCC: {                                                                               // bounded loop: MOVEQ #0..3,Dk; body; DBcc Dk,body
        int k = r(8);
        code.push_back((uint16_t)(0x7000 | (k << 9) | r(4)));
        uint32_t loop = pc();
        int nb = 1 + r(3);
        for (int i = 0; i < nb; i++) {
          int m = r(8); while (m == k) m = r(8);                                                   // the body never writes Dk
          switch (r(3)) {
            case 0: code.push_back((uint16_t)(0x5000 | (r(8) << 9) | (r(2) << 8) | (r(3) << 6) | m)); break;
            case 1: code.push_back((uint16_t)(0x7000 | (m << 9) | r(256))); break;
            default: {
              static const int ops[5] = {0x8, 0x9, 0xB, 0xC, 0xD};
              int op = ops[r(5)], s2 = r(3);
              EA s = src(s2, op == 0x9 || op == 0xB || op == 0xD, true);
              put((uint16_t)((op << 12) | (m << 9) | (s2 << 6) | ea6(s)), s);
            }
          }
        }
        uint32_t at = pc();
        code.push_back((uint16_t)(0x50C8 | ((int)r(16) << 8) | k));
        code.push_back((uint16_t)((int32_t)loop - (int32_t)(at + 2)));
        break;
      }
      // ADDX, SUBX (data registers) and CMPM, ADDX, SUBX in the memory form (-(Ay),-(Ax) and (Ay)+,(Ax)+).
      case G_ADDX: {                                                                               // ADDX SUBX CMPM
        int k = r(6), x = r(8), y = r(8);
        if (k < 2) code.push_back((uint16_t)((k ? 0x9100 : 0xD100) | (x << 9) | (sz << 6) | y));
        else {
          int ax = base(), ay = base();
          int opc = k < 4 ? (k == 2 ? 0xD108 : 0x9108) : 0xB108;
          code.push_back((uint16_t)(opc | (ax << 9) | (sz << 6) | ay));
          fix_regs({{ax, stp(sz, ax)}, {ay, stp(sz, ay)}}, k >= 4);
        }
        break;
      }
      // BCD: ABCD, SBCD and NBCD.
      case G_BCD: {                                                                                // ABCD SBCD NBCD
        int k = r(5), x = r(8), y = r(8);
        if (k < 2) code.push_back((uint16_t)((k ? 0x8100 : 0xC100) | (x << 9) | y));
        else if (k < 4) {
          int ax = base(), ay = base();
          code.push_back((uint16_t)((k == 2 ? 0xC108 : 0x8108) | (ax << 9) | ay));
          fix_regs({{ax, stp(0, ax)}, {ay, stp(0, ay)}}, false);
        } else { EA d = r(3) == 0 ? EA{0, (int)r(8), {}} : memp(0); put((uint16_t)(0x4800 | ea6(d)), d); ea_fix(d, 0); }
        break;
      }
      // Bit operations BTST, BCHG, BCLR, BSET with a register or immediate bit number, on a register, memory or PC-relative (BTST only) operand.
      case G_BIT: {                                                                                // BTST BCHG BCLR BSET
        int tt = r(4), dyn = r(2), dr = r(8);
        EA d;
        if (r(3) == 0) d = {0, (int)r(8), {}};
        else if (tt == 0 && r(3) == 0) d = pcrel(0, dyn ? 2 : 4);
        else d = memp(0);
        if (dyn) put((uint16_t)(0x0100 | (dr << 9) | (tt << 6) | ea6(d)), d);
        else {
          code.push_back((uint16_t)(0x0800 | (tt << 6) | ea6(d))); code.push_back((uint16_t)r(256));
          for (auto x : d.ext) code.push_back(x);
        }
        ea_fix(d, 0);
        break;
      }
      // Memory shifts and rotates (one bit, word size).
      case G_MSHIFT: {                                                                             // memory shifts and rotates
        int t = r(4), dir = r(2);
        EA d = memp(1);
        put((uint16_t)(0xE0C0 | (t << 9) | (dir << 8) | ea6(d)), d);
        ea_fix(d, 1);
        break;
      }
      // PACK and UNPK, register and memory forms.
      case G_PACK: {                                                                               // PACK UNPK (x = source, y = destination)
        int unpk = r(2), mm = r(2), x = r(8), y = r(8);
        if (mm) { x = base(); y = base(); }
        code.push_back((uint16_t)((unpk ? 0x8180 : 0x8140) | (mm ? 8 : 0) | (y << 9) | x));
        code.push_back((uint16_t)rng());
        if (mm) fix_regs({{x, unpk ? stp(0, x) : 2}, {y, unpk ? 2 : stp(0, y)}}, false);   // PACK reads a word, UNPK writes one
        break;
      }
      // MOVEP.
      case G_MOVEP: {                                                                              // MOVEP
        int oo = r(4), d = r(8), an = base();
        code.push_back((uint16_t)(0x0108 | (d << 9) | (oo << 6) | an));
        code.push_back((uint16_t)((int)r(0x2000) - 0x1000));
        break;
      }
      // MOVEM. Loads go only into D0-D7 and A6 (base registers stay aligned); stores with -(An) may include the base register.
      default: {                                                                                   // MOVEM
        int ld = r(2), l = r(2), msz = l ? 2 : 1, a = align(msz), an = base();
        uint16_t mask = (uint16_t)rng();
        if (r(3) == 0) mask &= (uint16_t)rng();
        if (r(16) == 0) mask = 0;
        EA e;
        int k = ld ? (int)r(8) : (int)r(6);
        switch (k) {
          case 0: e = {2, an, {}}; break;
          case 1: e = ld ? EA{3, an, {}} : EA{4, an, {}}; break;
          case 2: e = {5, an, {(uint16_t)((((int)r(0x2000) - 0x1000) & ~(a - 1)) & 0xffff)}}; break;
          case 3: e = {6, an, {brief(msz)}}; break;
          case 4: e = {7, 0, {(uint16_t)(0x8000 + (r(0x7f00) & ~(a - 1)))}}; break;            // abs.W, room for 64 bytes
          case 5: { uint32_t v = 0x200000 + (r(0xc00000) & ~3u); e = {7, 1, {(uint16_t)(v >> 16), (uint16_t)v}}; break; }
          default: e = pcrel(msz, 4); break;                                                     // loads only
        }
        if (ld) mask &= 0x40FF;
        if ((e.mode == 3 || e.mode == 4) && !l) {                                                // .W: An moves by 2n,
          int c = 0; for (int i = 0; i < 16; i++) c += (mask >> i) & 1;                         // keep n even (aligned)
          if (c & 1) mask &= (uint16_t)(mask - 1);
        }
        code.push_back((uint16_t)(0x4880 | (ld << 10) | (l << 6) | ea6(e)));
        code.push_back(mask);
        for (auto x : e.ext) code.push_back(x);
        break;
      }
    }
  }

  // build(): assemble a program of n instructions.
  void build(int n) {
    ins(); code.push_back(0x7001); ins(); w2(0x4E7B, 0x0002);  // MOVEQ #1,D0 and MOVEC D0,CACR switch the caches on
    // With the system groups on, a prologue sets VBR, USP and MSP.
    if (sys_on()) sys_prologue();
    // Eight data registers get MOVE.L #imm,Dn with random values.
    for (int i = 0; i < 8; i++) {                                                // MOVE.L #imm,Dn / MOVEA.L #imm,An
      ins(); EA s = imm(2); put((uint16_t)(0x2000 | (i << 9) | ea6(s)), s);
    }
    // Eight address registers: A6 gets any value, the others get aligned data addresses (0x400000 and above) with MOVEA.L #imm,An.
    for (int i = 0; i < 8; i++) {
      uint32_t v = (i == 6) ? rng() : (0x400000 + (r(0x800000) & ~3u));
      EA s = {7, 4, {(uint16_t)(v >> 16), (uint16_t)v}};
      ins(); put((uint16_t)(0x2040 | (i << 9) | ea6(s)), s);
    }
    // first = the index of the first generated instruction (unused).
    size_t first = start.size();
    for (int i = 0; i < n; i++) one();
    // The end of the program: MOVEQ #0,D0 as a pad (so that no short branch can have displacement 0) and BRA.S * (a branch to itself) at the end.
    ins();
    code.push_back(0x7000);                                                      // MOVEQ #0,D0: pad, so no short branch has displacement 0
    ins();
    code.push_back(0x60FE);                                                      // BRA.S *  (end)
    // Patch the forward branches: for each, find the target (the instruction after the given number of skips, moving on past instructions inside units) and put the displacement into the opcode or the extension word.
    for (auto& b : br) {
      size_t t = b.ins + 1 + b.skip; if (t > start.size() - 1) t = start.size() - 1;
      while (t < start.size() - 1 && notgt[t]) t++;
      int32_t disp = 2 * (int32_t)start[t] - (2 * (int32_t)start[b.ins] + 2);
      // The displacement is relative to the address behind the opcode word.
      uint32_t at = start[b.ins];
      // Byte displacement into the opcode word, word displacement, FBcc.W, FBcc.L, FDBcc (the displacement word is at +4 and counts from there), or the long form (0xFF in the opcode byte).
      if (b.form == 0) code[at] |= (uint16_t)(disp & 0xff);
      else if (b.form == 1) code[at + 1] = (uint16_t)disp;
      else if (b.form == 3) code[at + 1] = (uint16_t)disp;  // FBcc.W
      else if (b.form == 4) { code[at + 1] = (uint16_t)(disp >> 16); code[at + 2] = (uint16_t)disp; }   // FBcc.L
      else if (b.form == 5) code[at + 2] = (uint16_t)(disp - 2);                           // FDBcc (disp word at +4)
      else { code[at] |= 0xff; code[at + 1] = (uint16_t)(disp >> 16); code[at + 2] = (uint16_t)disp; }
    }
    for (auto& x : fx) {  // forward JMP targets: absolute word, absolute long, or PC-relative with a base and extra offset
      size_t t = x.own + 1 + x.skip; if (t > start.size() - 1) t = start.size() - 1;
      while (t < start.size() - 1 && notgt[t]) t++;
      uint32_t tg = 2 * start[t];
      if (x.kind == 0) code[x.at] = (uint16_t)tg;
      else if (x.kind == 1) { code[x.at] = (uint16_t)(tg >> 16); code[x.at + 1] = (uint16_t)tg; }
      else code[x.at] = (uint16_t)((int32_t)tg - (int32_t)x.base - x.extra);
    }
    (void)first;
  }
};

// ------------------------------------------------------------------------------------------------------------------
// rn(): a register name for the messages.
static const char* rn(int i) { static const char* n[16] = {"D0","D1","D2","D3","D4","D5","D6","D7","A0","A1","A2","A3","A4","A5","A6","A7"}; return n[i]; }

// ---- The fixed performance test (lockstep perf) ----
// Three kernels, each run twice; the second (warm) pass is measured from the retirement of its outer DBF to the next one, so the caches are warm. The memory latency is fixed at 1 clock, there are no
// interrupts and no DMA. The result is the number of clocks per instruction (CPI), which measures how well the dual issue works: 2.029 for the first single-issue core, 1.007 for the dual-issue core and
// 1.031 for the release state (the small rise comes from the wait on an instruction cache fill).
// perf_mode = the perf test is running; perf_code = its program.
static bool perf_mode = false;
static std::vector<uint16_t> perf_code;
// perf_end = the address of the final branch.
static uint32_t perf_end = 0;
// perf_mark = address of the outer DBF of each kernel; perf_hit = for each mark, the clock and instruction count at each retirement.
static std::vector<uint32_t> perf_mark;                        // PC of each kernel's outer DBF D6
static std::vector<std::vector<std::pair<long, long>>> perf_hit; // (clock, instructions) at each retirement of a mark
// perf_build(): assemble the three kernels with small helpers: b8 (short branch), dbf, fwd (forward branch to patch) and land (patch it).
static void perf_build() {
  auto& c = perf_code; c.clear(); perf_mark.clear();
  auto pc  = [&]() { return (uint32_t)(2 * c.size()); };
  auto b8  = [&](uint16_t op, uint32_t t) { int d = (int)t - (int)(pc() + 2); c.push_back((uint16_t)(op | (d & 0xff))); };
  auto dbf = [&](int r, uint32_t t) { int d = (int)t - (int)(pc() + 2); c.push_back((uint16_t)(0x51C8 | r)); c.push_back((uint16_t)d); };
  auto fwd = [&](uint16_t op) { c.push_back(op); return c.size() - 1; };
  auto land = [&](size_t at) { int d = (int)pc() - (int)(2 * at + 2); c[at] |= (uint16_t)(d & 0xff); };
  // Caches on (MOVEQ #1,D0; MOVEC D0,CACR) and A7 = 0x30000.
  for (uint16_t x : {0x7001, 0x4E7B, 0x0002, 0x2E7C, 0x0003, 0x0000}) c.push_back(x);   // caches on; A7 = $30000
  // D0 = 0, D1 = 100, D2 to D4 = 0.
  for (uint16_t x : {0x7000, 0x7264, 0x7400, 0x7600, 0x7800}) c.push_back(x);            // D0 = 0, D1 = 100, D2-D4 = 0
  // Kernel A: an integer loop of 7 instructions run 50 times: ADD.L D1,D0; EOR.L D0,D2; LSL.L #1,D3; ADDQ.L #1,D4; MOVE.L D0,D5; SUBQ.L #1,D7; BNE.
  // A: integer loop, 7 instructions x 50: ADD.L D1,D0; EOR.L D0,D2; LSL.L #1,D3; ADDQ.L #1,D4; MOVE.L D0,D5; SUBQ.L #1,D7; BNE
  c.push_back(0x7C01); uint32_t ka = pc(); c.push_back(0x7E32); uint32_t la = pc();
  for (uint16_t x : {0xD081, 0xB182, 0xE38B, 0x5284, 0x2A00, 0x5387}) c.push_back(x);
  b8(0x6600, la); perf_mark.push_back(pc()); dbf(6, ka);
  // Kernel B: a memory copy of 64 long words with MOVE.L (A0)+,(A1)+ and DBF.
  // B: memory copy of 64 longs: MOVE.L (A0)+,(A1)+; DBF
  c.push_back(0x7C01); uint32_t kb = pc();
  for (uint16_t x : {0x41F9, 0x0002, 0x0000, 0x43F9, 0x0002, 0x1000, 0x7E3F}) c.push_back(x);
  uint32_t lb = pc(); c.push_back(0x22D8); dbf(7, lb); perf_mark.push_back(pc()); dbf(6, kb);
  // Kernel C: branchy code with calls, 32 iterations: BSR to a subroutine (ADD.L D7,D0; RTS); BTST #0,D7; BEQ; ADDQ; CMP.L D1,D0; BGE; ADDQ; DBF.
  // C: branchy code with calls, 32 iterations: BSR sub (ADD.L D7,D0; RTS); BTST #0,D7; BEQ; ADDQ; CMP.L D1,D0; BGE; ADDQ; DBF
  size_t ov = fwd(0x6000); uint32_t sub = pc(); c.push_back(0xD087); c.push_back(0x4E75); land(ov);
  c.push_back(0x7C01); uint32_t kc = pc(); c.push_back(0x7000); c.push_back(0x7E1F); uint32_t lc = pc();
  b8(0x6100, sub); c.push_back(0x0807); c.push_back(0x0000);
  size_t f1 = fwd(0x6700); c.push_back(0x5482); land(f1);
  c.push_back(0xB081);
  size_t f2 = fwd(0x6C00); c.push_back(0x5283); land(f2);
  dbf(7, lc); perf_mark.push_back(pc()); dbf(6, kc);
  // The end: MOVEQ #0,D0 and a branch to itself.
  c.push_back(0x7000); perf_end = pc(); c.push_back(0x60FE);
  perf_hit.assign(perf_mark.size(), {});
}

// run_program(): run one program (one seed) on both sides. Returns 0 for a pass (or the normal end of the AROS and perf runs), 1 for a difference or hang, 2 for a generator error.
static int run_program(uint32_t seed, int len, long& total) {
  // Generate the program.
  Gen g(seed);
  g.build(len);
  // The program must fit below BND (the bound pairs of CHK2).
  if (2 * g.code.size() > Gen::BND) { printf("seed %u: generator error: program longer than %x bytes\n", seed, Gen::BND); return 2; }
  // Fill all of memory with random data (a different pattern for each seed) so that stray reads show.
  std::mt19937 fill(seed ^ 0x9e3779b9u);
  for (uint32_t a = 0; a < MEMSZ; a += 4) { uint32_t v = fill(); for (int i = 0; i < 4; i++) mem_ref[a + i] = (uint8_t)(v >> (24 - 8 * i)); }
  // Clear the code area, then copy the program into it.
  memset(mem_ref.data(), 0, CODE_END);
  for (size_t i = 0; i < g.code.size(); i++) { mem_ref[2 * i] = g.code[i] >> 8; mem_ref[2 * i + 1] = g.code[i] & 0xff; }
  // The system memory: length table, vectors, handlers.
  g.sysmem(mem_ref);
  // The core's memory starts as a copy.
  mem_rtl = mem_ref;
  // end_pc = the PC of the final branch.
  uint32_t end_pc = 2 * g.start.back();
  // cyc_max = the clock limit of the run.
  long cyc_max = 2000000;
  // AROS mode: clear memory and load the ROM images.
  if (aros) {
    std::string spec = getenv("P68_AROS"); size_t k = spec.find(',');
    std::fill(mem_ref.begin(), mem_ref.end(), 0);
    auto load = [&](const std::string& f, uint32_t at) {
      FILE* fp = fopen(f.c_str(), "rb"); if (!fp) { printf("cannot open %s\n", f.c_str()); exit(2); }
      size_t n = fread(&mem_ref[at], 1, 0x80000, fp); fclose(fp); printf("AROS: %s at %06x, %zu bytes\n", f.c_str(), at, n);
    };
    load(spec.substr(0, k), 0xF80000); if (k != std::string::npos) load(spec.substr(k + 1), 0xE00000);
    // The reset longwords: copy the first 8 bytes of the ROM to address 0 (initial stack pointer and PC).
    memcpy(&mem_ref[0], &mem_ref[0xF80000], 8);
    mem_rtl = mem_ref; end_pc = 0xFFFFFFFF;
    cyc_max = getenv("P68_CYC") ? atol(getenv("P68_CYC")) : 400000000L;
  }
  // The perf test replaces the program.
  if (perf_mode) {                               // the perf test: the fixed program replaces the generated one
    memset(mem_ref.data(), 0, CODE_END);
    for (size_t i = 0; i < perf_code.size(); i++) { mem_ref[2 * i] = perf_code[i] >> 8; mem_ref[2 * i + 1] = perf_code[i] & 0xff; }
    mem_rtl = mem_ref; end_pc = perf_end;
  }

  // Reset Musashi and the FPU model, let Musashi eat the reset cycles, then set its state to what the core has after reset: supervisor mode with SR 0x2700, all registers and control registers 0, PC 0.
  m68k_pulse_reset();
  p68_fpu_reset();                               // reset the reference FPU (sim/lockstep/fpu_ref.c)
  m68k_execute(1);                               // eats the reset cycles, runs no instruction
  m68k_set_reg(M68K_REG_SR, 0x2700);             // supervisor, CCR 0 (as the RTL after reset)
  for (int i = 0; i < 16; i++) m68k_set_reg((m68k_register_t)(M68K_REG_D0 + i), 0);
  static const m68k_register_t zr[] = {M68K_REG_USP, M68K_REG_MSP, M68K_REG_VBR, M68K_REG_SFC, M68K_REG_DFC,
                                       M68K_REG_CACR, M68K_REG_CAAR};
  for (auto z : zr) m68k_set_reg(z, 0);
  m68k_set_irq(0);
  m68k_set_reg(M68K_REG_PC, 0);
  gen_error.clear();

  // The Verilator model of the core.
  auto ctx = std::make_unique<VerilatedContext>();
  auto top = std::make_unique<Vp68_core>(ctx.get());
  // sh = shadow registers D0-D7 and A0-A7 built from the retire records, bk = the three stack pointer banks.
  uint32_t sh[16] = {0}, bk[3] = {0};            // shadow registers and stack pointer banks built from retire records
  // rtl_wr = the bytes the core stored during the current instruction.
  std::map<uint32_t, uint8_t> rtl_wr;            // bytes the RTL stored during the current instruction
  // irng = random numbers for the interrupt source.
  std::mt19937 irng(seed ^ 0x51ed270bu);
  // irq_on = interrupts are generated (group 23 on, not in AROS mode).
  const bool irq_on = !aros && ((groups >> G_IRQ) & 1);
  // ipl = the interrupt level presented to the core, ipl_hold = how long it has been waiting, iack_lvl = the level the core acknowledged, pend_n = instructions retired while an unmasked request is pending.
  int ipl = 0, ipl_hold = 0, iack_lvl = 0, pend_n = 0;
  // ---- Memory model behind the core's memory port, and the DMA source ----
  // The core makes one transaction at a time (it never starts one before m_done). The model answers with a random latency: 0 to 6 clocks to the first beat, or to the write's done, and 0 to 2 clocks between beats.
  // A write changes memory when its done is given. A read beat is 64 bits wide.
  // DMA timing rule (docs/MEMORY_PORT.md, Coherency): a DMA write decided in loop iteration c changes mem_rtl at the end of iteration c (after the beat for the next clock was read) and pulses m_snoop at the next clock
  // edge. Musashi gets the write before the first instruction that retires in iteration c + 3 or later, since the core's loads that commit in the clock after the snoop is registered, or later, see the new
  // value, and the ones before see the old value.
  // mrng = random numbers for the memory model (latency and DMA).
  std::mt19937 mrng(seed ^ 0x2545f491u);
  // dma_on = DMA writes are generated (group 31 on, not in AROS mode).
  const bool dma_on = !aros && ((groups >> G_DMA) & 1);
  // Dma = a DMA write that Musashi still has to see: when, address, size, value. dmaq = the queue of them.
  struct Dma { long when; uint32_t a; int n; uint32_t v; };
  std::vector<Dma> dmaq;
  // State of the memory port model: busy, write, address, write data, number of beats, current beat, wait counter and byte enables.
  bool mbusy = false, mwr = false; uint32_t maddr = 0, mwdata = 0; int mbeats = 0, mnb = 0, mwait = 0, mbe = 0;
  // rdy = a read beat is valid, done = the transaction is finished, snoop = a snoop pulse for the core, rdata = the beat, snaddr = the snooped word address.
  bool rdy = false, done = false, snoop = false; uint64_t rdata = 0; uint32_t snaddr = 0;
  // rd64(): a 64-bit read beat (two long words, big endian) from the core's memory.
  auto rd64 = [&](uint32_t a) { return ((uint64_t)rd(mem_rtl, a, 4) << 32) | rd(mem_rtl, a + 4, 4); };
  // rd64u(): the same for an uncached word read in AROS mode, where the beam position counter must count one access.
  auto rd64u = [&](uint32_t a, int be) {          // AROS mode: an uncached word read of the beam position counts once
    uint64_t v = rd64(a & ~7u);
    if (!aros) return v;
    bool first = true;
    for (int i = 0; i < 4; i++) if (be & (8 >> i)) {
      uint32_t ba = ((a & ~3u) + i) & MASK; uint8_t b;
      if (io_byte(1, ba, b, first)) { first = false; int sh = 8 * (7 - (ba & 7)); v = (v & ~(0xffull << sh)) | ((uint64_t)b << sh); }
    }
    return v;
  };
  // munc = the access is uncached, mfull = its full 32-bit address.
  bool munc = false; uint32_t mfull = 0;
  // Reset values of the inputs: no memory answers, no snoop.
  top->clk = 0; top->rst = 1; top->m_rdy = 0; top->m_done = 0; top->m_rdata = 0; top->m_snoop = 0; top->m_snoop_addr = 0;
  // unc_map (the uncached 1 MB regions: for AROS the A1200 map), cac_hi, no interrupt, and CPU speed mode from the environment.
  top->unc_map = aros ? 0x7C03 : UNC_MAP; top->cac_hi = (aros && getenv("P68_CACHI")) ? strtoul(getenv("P68_CACHI"), 0, 0) : 0; top->ipl = 0; top->slow = getenv("P68_SLOW") && atoi(getenv("P68_SLOW")) ? 1 : 0; top->eval();   // P68_SLOW=1: CPU speed mode (same results, slower)
  // idle = clocks without any retire.
  int idle = 0;
  // The clock loop: one pass per clock until a program ends, fails or hangs.
  for (long cyc = 0; ; cyc++) {
    // Falling edge: present the answers of the memory model, the snoop and the interrupt level.
    top->clk = 0; top->m_rdy = rdy; top->m_done = done; top->m_rdata = rdata; top->m_snoop = snoop; top->m_snoop_addr = snaddr;
    top->ipl = ipl; top->eval();
    // Rising edge: the core's registers change.
    top->clk = 1; top->eval();
    // Release reset after 4 clocks.
    if (cyc == 4) top->rst = 0;
    // The one-clock answers last for one clock only.
    rdy = done = snoop = false;
    // A new request from the core: accept it. It must not come while a transaction is running.
    if (top->m_req) {
      if (mbusy) { printf("seed %u: memory port: request during a transaction\n", seed); return 1; }
      // Take over the direction, address, write data, byte enables and the uncached flag.
      mbusy = true; mwr = top->m_wr; maddr = (top->m_addr << 2) & MASK; mwdata = top->m_wdata; mbe = top->m_be; munc = top->m_unc; mfull = top->m_addr << 2;
      // beats = the number of beats; the first wait is random (0 to 6 clocks; 1 in the perf test, which has a fixed latency).
      mbeats = top->m_len + 1; mnb = 0; mwait = perf_mode ? 1 : (int)(mrng() % 7);
      // A cached line read must be aligned to its size.
      if (!mwr && (maddr & (8 * mbeats - 1)) && !top->m_unc) { printf("seed %u: memory port: unaligned line read %06x\n", seed, maddr); return 1; }
    // A transaction is running.
    } else if (mbusy) {
      // Count down the wait.
      if (mwait > 0) mwait--;
      // Write: when the wait is over, the enabled bytes go into memory and the transaction is done.
      else if (mwr) {
        if (!hole(mfull)) for (int i = 0; i < 4; i++) if (mbe & (8 >> i)) mem_rtl[(maddr + i) & MASK] = (mwdata >> (24 - 8 * i)) & 0xff;
        done = true; mbusy = false;
      // Read: the next beat, from the aligned double word for a single access or from the line for a burst; uncached single reads in AROS mode go through rd64u.
      } else {
        rdata = hole(mfull) ? 0 : (mbeats == 1 && munc) ? rd64u(maddr, mbe) : rd64(mbeats == 1 ? (maddr & ~7u) : maddr + 8 * mnb); rdy = true;
        // After the last beat the transaction is done; otherwise a short random wait before the next beat.
        if (++mnb == mbeats) { done = true; mbusy = false; } else mwait = perf_mode ? 0 : (int)(mrng() % 3);
      }
    }
    // DMA: now and then (1 in 40 clocks, after the first 20) a write of 1, 2 or 4 bytes into one of the windows. Memory changes at once, the snoop reaches the core at the next edge, and the write is queued for Musashi.
    if (dma_on && cyc > 20 && mrng() % 40 == 0) {   // DMA write into a window: memory now, snoop at the next edge
      int n = 1 << (mrng() % 3);
      uint32_t a = ((mrng() & 1) ? DMA_C : DMA_U) + ((uint32_t)mrng() % DMA_SZ & ~(uint32_t)(n - 1)), v = mrng();
      for (int i = n - 1, x = v; i >= 0; i--, x >>= 8) mem_rtl[a + i] = x & 0xff;
      snoop = true; snaddr = a >> 2; dmaq.push_back({cyc + 3, a, n, v}); n_dma++;
    }
    // Interrupt source: a request is held until the core acknowledges it (iack), or withdrawn after a while if it is masked. A STOP instruction always gets one, at a level above the mask.
    // interrupt source: a request is held until acknowledged (or withdrawn after a while if masked); STOP gets one
    // mask = Musashi's interrupt mask.
    int mask = (m68k_get_reg(NULL, M68K_REG_SR) >> 8) & 7;
    // The core acknowledged: remember the level and remove the request.
    if (top->iack) { iack_lvl = top->iack_lvl; ipl = 0; }
    // No request: maybe raise one (1 in 400 clocks after the first 200), with a random level 1 to 7.
    else if (ipl == 0) {
      if (irq_on && cyc > 200 && irng() % 400 == 0) { ipl = 1 + irng() % 7; ipl_hold = 0; }
      // The core is stopped (STOP): raise a level that is above the mask, or 7.
      if (top->stopped) { ipl = mask == 7 ? 7 : mask + 1 + (int)(irng() % (7 - mask)); ipl_hold = 0; }
    // A masked request that waits for more than 3000 clocks is withdrawn (level 7 is not maskable and stays).
    } else if (++ipl_hold > 3000 && ipl <= mask && ipl != 7) ipl = 0;
    // Every committed uop: register writes and stores, and stack pointer swaps. The instruction itself is compared at ret_v.
    // wrA(): apply lane A's committed uop writes: the address register updates of the address generation stage (source and destination), the write-back register write, a stack pointer bank write, and the stores with their byte enables.
    auto wrA = [&]() {
      if (top->ret_u) {
        if (top->ret_ws_en) sh[8 + top->ret_ws_r] = top->ret_ws_v;
        if (top->ret_wd_en) sh[8 + top->ret_wd_r] = top->ret_wd_v;
        if (top->ret_wen) sh[top->ret_wreg] = top->ret_wval;
        if (top->ret_bk_en) bk[top->ret_bk_r] = top->ret_wval;
        // Stores: each of the four byte lanes of the word with its byte enable goes into rtl_wr (not for addresses above 16 MB in AROS mode).
        if (top->ret_st && !hole(top->ret_saddr)) for (int i = 0; i < 4; i++) if (top->ret_sbe & (8 >> i)) rtl_wr[((top->ret_saddr & ~3u) + i) & MASK] = (top->ret_sdata >> (24 - 8 * i)) & 0xff;
        if (top->ret_st2 && !hole(top->ret_saddr2)) for (int i = 0; i < 4; i++) if (top->ret_sbe2 & (8 >> i)) rtl_wr[((top->ret_saddr2 & ~3u) + i) & MASK] = (top->ret_sdata2 >> (24 - 8 * i)) & 0xff;  // a second word of a misaligned store that crosses a longword
      }
      // A7 swap: the old stack pointer goes into its bank and A7 takes the new value (S and M bit changes).
      if (top->ret_sw) { bk[top->ret_sw_old] = sh[15]; sh[15] = top->ret_sw_v; n_swap++; }
    };
    // wrB(): lane B's register write (only a write-back write; lane B holds simple instructions).
    auto wrB = [&]() { if (top->ret2_wen) sh[top->ret2_wreg] = top->ret2_wval; };
    {  // prediction counters from the core's ret_bp output: bit 5 branch retired, 4 predicted taken, 3 actually taken, 2 predicted by the branch target cache, 1 return retired, 0 return mispredicted
      int bp = top->ret_bp; n_cyc++;
      if (bp & 32) { n_br++; if (bp & 16) { n_brp++; if (!(bp & 8)) n_brpm++; } else if (bp & 8) n_brnm++; if (bp & 4) n_btc++; }
      if (bp & 2) { n_ret++; if (bp & 1) n_retm++; }
    }
    // A stack pointer swap without any retire still has to be applied.
    if (!top->ret_u && !top->ret2_v && top->ret_sw) { bk[top->ret_sw_old] = sh[15]; sh[15] = top->ret_sw_v; n_swap++; }
    // Nothing retired this clock: continue, but after 5000 clocks without any retire report a hang (the PC and whether the core's lock is set).
    if (!top->ret_u && !top->ret2_v) { if (++idle > 5000) { printf("seed %u: no instruction retired for 5000 clocks (hang), last PC %06x (lock %d)\n", seed, m68k_get_reg(NULL, M68K_REG_PC), (int)top->lock); return 1; } continue; }
    // Something retired.
    idle = 0;
    // retire(): handle one retired instruction (lane A, lane B or an interrupt entry). Up to two instructions retire per clock; the older one is compared first. Returns -1 to go on, or the result of the program.
    auto retire = [&](uint32_t r_pc, bool r_irq, uint32_t r_sr) -> int {
      for (size_t i = 0; i < dmaq.size();) {  // DMA writes that Musashi must see from this instruction on
        if (dmaq[i].when <= cyc) { for (int k = dmaq[i].n - 1, x = dmaq[i].v; k >= 0; k--, x >>= 8) mem_ref[dmaq[i].a + k] = x & 0xff; dmaq.erase(dmaq.begin() + i); }
        else i++;
      }
      // rpc = the PC the core reports, mpc = Musashi's PC; dis = the disassembly for the messages.
      uint32_t rpc = r_pc, mpc = m68k_get_reg(NULL, M68K_REG_PC);
      char dis[128];
      if (r_irq) snprintf(dis, sizeof dis, "interrupt level %d", iack_lvl);
      else m68k_disassemble(dis, mpc, M68K_CPU_TYPE_68EC020);
      if (rpc != mpc) {  // PC differs: print both PCs and the code around Musashi's PC
        printf("seed %u: PC differs: RTL retired %06x, Musashi is at %06x (%s)\n", seed, rpc, mpc, dis);
        printf("  code at %06x:", (mpc - 8) & MASK); for (int k = 0; k < 10; k++) printf(" %04x", rd(mem_ref, (mpc - 8 + 2 * k) & MASK, 2));
        printf("\n"); return 1;
      }
      // ref_wr is cleared before Musashi runs, so that it holds only the bytes this instruction writes.
      ref_wr.clear();
      // An interrupt entry in the core: Musashi takes the interrupt at the same boundary, through the glue function (an instruction is not run).
      if (r_irq) {                          // Musashi takes the interrupt at the same instruction boundary
        uint32_t sr0 = m68k_get_reg(NULL, M68K_REG_SR), sp0 = m68k_get_reg(NULL, M68K_REG_A7);
        m68k_set_irq(iack_lvl); m68k_glue_check_irq(); m68k_set_irq(0);
        // If Musashi did not take it (PC, SR and A7 unchanged), the test fails.
        if (m68k_get_reg(NULL, M68K_REG_PC) == mpc && m68k_get_reg(NULL, M68K_REG_SR) == sr0 &&
            m68k_get_reg(NULL, M68K_REG_A7) == sp0) {
          printf("seed %u: RTL took an interrupt of level %d at %06x, Musashi does not (SR %04x)\n", seed, iack_lvl, mpc, sr0); return 1;
        }
        // The request has been taken: reset the counter of instructions with a request pending and count it.
        pend_n = 0; n_irq++;
        // Count interrupts taken with the M bit set (the master stack pointer in use).
        if ((sr0 & 0x3000) == 0x3000) n_msp++;
      // No interrupt: run Musashi for one instruction.
      } else {
        // Count instructions in user mode.
        if ((m68k_get_reg(NULL, M68K_REG_SR) & 0x2000) == 0) n_user++;
        m68k_execute(1);
        // Count STOP instructions.
        if (top->stopped) n_stop++;
        // Format error in RTE (aliased stacks): Musashi stacks the address of the RTE plus 2, while the 68020 manual and the core stack the address of the RTE itself; the bench rewrites Musashi's stacked PC to the correct value.
        // format error in RTE (aliased stacks): Musashi stacks RTE+2, the 68020 manual and the RTL the RTE's address
        uint32_t a7 = m68k_get_reg(NULL, M68K_REG_A7);
        if (rd(mem_ref, mpc, 2) == 0x4E73 && rd(mem_ref, a7 + 6, 2) == 0x0038 && rd(mem_ref, a7 + 2, 4) == mpc + 2 &&
            m68k_get_reg(NULL, M68K_REG_PC) == rd(mem_ref, (m68k_get_reg(NULL, M68K_REG_VBR) + 0x38) & MASK, 4))
          wr_ref(a7 + 2, 4, mpc);
      }
      if (trace) {
        if (perf_mode) printf("%6ld ", cyc);  // print the clock of the retire in perf mode
        if ((rd(mem_ref, mpc, 2) >> 12) == 0xF) {  // FPU words are printed as hex words, no disassembly
          char fb[40]; snprintf(fb, sizeof fb, "F %04x %04x %04x %04x", rd(mem_ref, mpc, 2), rd(mem_ref, mpc + 2, 2), rd(mem_ref, mpc + 4, 2), rd(mem_ref, mpc + 6, 2));
          printf("%06x %-36s", mpc, fb);
        } else printf("%06x %-36s", mpc, dis);
        // Trace level 4: FP0-FP7 and FPSR, FPCR.
        if (trace > 3) {
          for (int i = 0; i < 8; i++) { unsigned hi; unsigned long long lo; p68_fpu_get(i, &hi, &lo); printf(" FP%d %04x_%016llx", i, hi, lo); }
          printf(" FPSR %08x FPCR %04x\n%45s", p68_fpu_ctl(1), p68_fpu_ctl(0), "");
        }
        // D0-D7 (trace 2 and up) and A0-A7, then SR.
        for (int i = trace > 1 ? 0 : 8; i < 16; i++) printf(" %08x", m68k_get_reg(NULL, (m68k_register_t)(M68K_REG_D0 + i)));
        printf(" SR %04x\n", m68k_get_reg(NULL, M68K_REG_SR));
      }
      // Count the instruction.
      total++;
      // In the perf test: remember the clock and instruction count at each retire of a kernel's outer DBF.
      if (perf_mode) for (size_t k = 0; k < perf_mark.size(); k++) if (rpc == perf_mark[k]) perf_hit[k].push_back({cyc, total});
      // A generator error found while Musashi ran: stop with return code 2.
      if (!gen_error.empty()) { printf("seed %u: generator error at %06x (%s): %s\n", seed, mpc, dis, gen_error.c_str()); return 2; }
      // diff = the text of the differences, cmp() adds one value if it differs.
      std::string diff;
      auto cmp = [&](const char* name, uint32_t rtl, uint32_t ref) {
        if (rtl != ref) { char b[80]; snprintf(b, sizeof b, " %s RTL %08x ref %08x;", name, rtl, ref); diff += b; }
      };
      // D0-D7 and A0-A7 of the core against Musashi.
      for (int i = 0; i < 16; i++) cmp(rn(i), sh[i], m68k_get_reg(NULL, (m68k_register_t)(M68K_REG_D0 + i)));
      // SR (made of the system byte and the CCR that the retire port reports).
      uint32_t rsr = r_sr, msr = m68k_get_reg(NULL, M68K_REG_SR);
      cmp("SR", rsr, msr);
      // The stack pointer banks: the active one is the shadow A7 (SR bit 13 = supervisor, bit 12 = master selects the bank).
      int act = (rsr & 0x2000) ? ((rsr & 0x1000) ? 2 : 1) : 0;
      uint32_t rsp[3] = {bk[0], bk[1], bk[2]}; rsp[act] = sh[15];
      // USP, ISP, MSP, VBR, SFC, DFC, CACR, CAAR.
      cmp("USP", rsp[0], m68k_get_reg(NULL, M68K_REG_USP));
      cmp("ISP", rsp[1], m68k_get_reg(NULL, M68K_REG_ISP));
      cmp("MSP", rsp[2], m68k_get_reg(NULL, M68K_REG_MSP));
      cmp("VBR", top->ret_vbr, m68k_get_reg(NULL, M68K_REG_VBR));
      cmp("SFC", top->ret_sfc, m68k_get_reg(NULL, M68K_REG_SFC));
      cmp("DFC", top->ret_dfc, m68k_get_reg(NULL, M68K_REG_DFC));
      cmp("CACR", top->ret_cacr, m68k_get_reg(NULL, M68K_REG_CACR));
      cmp("CAAR", top->ret_caar, m68k_get_reg(NULL, M68K_REG_CAAR));
      {  // FPU state: the eight registers (as 80-bit values in three words) and FPCR, FPSR and FPIAR
        const uint32_t* fw[8] = {&top->ret_fp0[0], &top->ret_fp1[0], &top->ret_fp2[0], &top->ret_fp3[0], &top->ret_fp4[0],
                                 &top->ret_fp5[0], &top->ret_fp6[0], &top->ret_fp7[0]};
        static const char* fn[8] = {"FP0", "FP1", "FP2", "FP3", "FP4", "FP5", "FP6", "FP7"};
        for (int i = 0; i < 8; i++) {
          unsigned hi; unsigned long long lo; p68_fpu_get(i, &hi, &lo);
          if ((fw[i][2] & 0xffff) != hi || fw[i][1] != (uint32_t)(lo >> 32) || fw[i][0] != (uint32_t)lo) {
            char b[120]; snprintf(b, sizeof b, " %s RTL %04x_%08x%08x ref %04x_%016llx;", fn[i], fw[i][2] & 0xffff, fw[i][1], fw[i][0], hi, lo); diff += b;
          }
        }
        cmp("FPCR", top->ret_fpcr, p68_fpu_ctl(0));
        cmp("FPSR", top->ret_fpsr, p68_fpu_ctl(1));
        cmp("FPIAR", top->ret_fpiar, p68_fpu_ctl(2));
      }
      // A store of a value that is already in memory, which Musashi skipped, is not a difference (the core writes whole aligned long words for bit fields).
      // a store of the value already in memory that the reference skipped (the core writes whole aligned longs for bit fields) is not a difference
      for (auto it = rtl_wr.begin(); it != rtl_wr.end();)
        if (!ref_wr.count(it->first) && mem_ref[it->first] == it->second) it = rtl_wr.erase(it); else ++it;
      // The stores must match exactly: address and value of every byte.
      if (rtl_wr != ref_wr) {
        diff += " stores RTL{";
        for (auto& w : rtl_wr) { char b[24]; snprintf(b, sizeof b, "%06x=%02x ", w.first, w.second); diff += b; }
        diff += "} ref{";
        for (auto& w : ref_wr) { char b[24]; snprintf(b, sizeof b, "%06x=%02x ", w.first, w.second); diff += b; }
        diff += "};";
      }
      // The first difference: print the seed, the PC, the disassembly and the number of the instruction, then the text, and stop.
      if (!diff.empty()) {
        printf("seed %u: difference after %06x  %s  (instruction %ld of this program)\n ", seed, mpc, dis, total);
        printf("%s\n", diff.c_str());
        return 1;
      }
      // Trace level 3: the stored bytes.
      if (trace > 2) { printf("   stores:"); for (auto& w : rtl_wr) printf(" %06x=%02x", w.first, w.second); printf("\n"); }
      rtl_wr.clear();
      // An unmasked request, or a level 7 edge, must be taken within a few instructions: after 8 instructions with the request pending and unmasked the test fails.
      // an unmasked request (or a level-7 edge) must be taken within a few instructions
      if (ipl && (ipl == 7 || ipl > (int)((msr >> 8) & 7))) {
        if (++pend_n > 8) { printf("seed %u: interrupt level %d pending and unmasked for 8 instructions, not taken (at %06x)\n", seed, ipl, mpc); return 1; }
      } else pend_n = 0;
      // The program ended (the PC reached the final branch).
      if (mpc == end_pc) return 0;
      // AROS mode: a progress line every million instructions.
      if (aros && total % 1000000 == 0) { printf("AROS: %ld instructions, clock %ld, PC %06x\n", total, cyc, mpc); fflush(stdout); }
      // A program that runs too long (more clocks than cyc_max) fails.
      if (cyc > cyc_max) { printf("seed %u: too long\n", seed); return 1; }
      return -1;
    };
    // Up to two instructions retire per clock: rB = lane B retires, bF = lane B holds the older instruction.
    const bool rB = top->ret2_v, bF = rB && top->ret2_first;
    if (rB) { n_pair++; if (bF) n_pold++; }  // count pairs, and pairs in which lane B is older
    int rc;
    // Lane B first when it is older ...
    if (bF) { wrB(); if ((rc = retire(top->ret2_pc, false, (top->ret_sr & 0xffe0) | top->ret2_ccr)) >= 0) return rc; }
    // ... then lane A's writes, lane A's instruction ...
    wrA();
    if (top->ret_v && (rc = retire(top->ret_pc, top->ret_irq, (top->ret_sr & 0xffe0) | top->ret_ccr)) >= 0) return rc;
    // ... and lane B when it is younger.
    if (rB && !bF) { wrB(); if ((rc = retire(top->ret2_pc, false, (top->ret_sr & 0xffe0) | top->ret2_ccr)) >= 0) return rc; }
  }
}

// main(): the three modes. Perf test: run the fixed program and print the clocks per instruction of each kernel and in total.
int main(int argc, char** argv) {
  if (argc > 1 && !strcmp(argv[1], "perf")) {  // perf test: the fixed program replaces the random ones
    perf_mode = true; groups = 1; perf_build(); trace = argc > 2 ? atoi(argv[2]) : 0;  // only group 1 is on; the second argument is the trace level
    // Musashi as a 68EC020 (24-bit addresses).
    m68k_init(); m68k_set_cpu_type(M68K_CPU_TYPE_68EC020);
    long total = 0;
    int rc = run_program(1, 10, total);
    if (rc) { printf("perf: FAIL\n"); return rc; }
    static const char* nm[3] = {"integer loop", "memory copy", "branches and calls"};
    // nm = the kernel names.
    long ci = 0, ii = 0;
    for (size_t k = 0; k < perf_hit.size(); k++) {
      if (perf_hit[k].size() < 2) { printf("perf: kernel %zu not measured\n", k); return 1; }
      // Clocks and instructions between the two retirements of the kernel's outer DBF: the warm pass.
      long c = perf_hit[k][1].first - perf_hit[k][0].first, i = perf_hit[k][1].second - perf_hit[k][0].second;
      ci += c; ii += i;
      printf("perf %-20s %5ld instructions %6ld clocks  %.3f clocks per instruction\n", nm[k], i, c, (double)c / i);
    }
    printf("perf %-20s %5ld instructions %6ld clocks  %.3f clocks per instruction\n", "all", ii, ci, (double)ci / ii);
    return 0;
  }
  // AROS mode: one run of the Kickstart image, with Musashi as a full 68020 (32-bit addresses); the first argument is the trace level.
  if (getenv("P68_AROS")) {  // AROS mode: one run of the Kickstart image against Musashi
    aros = true; trace = argc > 1 ? atoi(argv[1]) : 0;
    m68k_init(); m68k_set_cpu_type(M68K_CPU_TYPE_68020);
    long total = 0;
    int rc = run_program(1, 10, total);
    printf("AROS: %s after %ld instructions\n", rc ? "FAIL" : "end", total);
    return rc;
  }
  // Random mode: n programs of len instructions from the seed, with the groups from the command line.
  int n = argc > 1 ? atoi(argv[1]) : 100;
  uint32_t seed = argc > 2 ? (uint32_t)strtoul(argv[2], 0, 0) : 1;
  int len = argc > 3 ? atoi(argv[3]) : 200;
  trace = argc > 4 ? atoi(argv[4]) : 0;   // 1 = print every retired instruction
  if (argc > 5) groups = (uint64_t)strtoull(argv[5], 0, 0) & G_ALL;
  if (!groups) { printf("no generator group selected\n"); return 2; }
  // Musashi as a 68EC020, as the core has 24-bit addresses.
  m68k_init();
  m68k_set_cpu_type(M68K_CPU_TYPE_68EC020);
  long total = 0;
  // Run the programs one after the other; the first failure ends the bench.
  for (int p = 0; p < n; p++) {
    long before = total;
    int rc = run_program(seed + p, len, total);
    // A failure prints how many programs and instructions had passed.
    if (rc) { printf("FAIL after %d programs, %ld instructions compared\n", p, total); return rc; }
    (void)before;
  }
  // Success: the PASS line and the coverage lines.
  printf("PASS: %d programs, %ld instructions compared (groups 0x%llx)\n", n, total, (unsigned long long)groups);
  printf("  memory: %ld DMA writes, %ld DMA window loads, %ld stores into code; %ld clocks (%.2f per instruction)\n",
         n_dma, n_probe, n_smc, n_cyc, total ? (double)n_cyc / total : 0.0);  // memory: DMA writes, DMA window loads, stores into code, clocks per instruction
  // The branch prediction counters of the core.
  printf("  branches: %ld Bcc/BRA, %ld predicted taken (%ld by the BTC, %ld not taken), %ld taken not predicted; %ld returns, %ld mispredicted\n",
         n_br, n_brp, n_btc, n_brpm, n_brnm, n_ret, n_retm);
  // The system line is printed only when a system group was on; the dual issue line below it is printed always.
  if ((groups >> G_SYS) & 7)
    printf("  system: %ld interrupts (%ld with M set), %ld user-mode instructions, %ld stack pointer swaps, %ld STOPs\n",
           n_irq, n_msp, n_user, n_swap, n_stop);
  printf("  dual issue: %ld instructions in lane B, %ld of them older than lane A\n", n_pair, n_pold);
  return 0;
}
