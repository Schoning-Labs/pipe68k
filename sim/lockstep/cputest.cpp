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
// Runner for the WinUAE cputester data (68020 integer instructions and, with the FPU sets, the 68882): it loads the data files that the cputester generator wrote
// and runs every test on the RTL core in Verilator.
//
// What the data is. WinUAE's cputester is a CPU test that normally runs on a real Amiga: a generator makes millions of one-instruction tests on WinUAE's CPU model (which
// has been checked against real chips), and an Amiga program replays them on the real CPU and compares. This runner plays the Amiga program's role on the simulated core.
// tools/cputest_build.sh builds and runs the generator (WinUAE's source is not part of pipe68k and none of its code is copied here; this file was written from the data
// format that the generator writes and that the Amiga program reads). For each instruction (a folder such as ADD.L) there is a file 0000.dat with chunks that
// start with the four bytes AF 'M' 'R' 'G'. The first chunk is a header (version 24, memory layout, stacks); the others hold groups of tests.
// A group is one instruction encoding (opcode and extension words) with generated register and memory values. It is run in several rounds: with different status
// register settings (user mode, supervisor mode, master bit) and, in each round, with different condition code settings (CCR all clear, all set, or the 32 values for the
// conditional instructions). The data is stored as differences: each item changes one register or memory value of the "expected state" that the runner carries from
// test to test, exactly as the Amiga program does, so the runner has to decode every item in the right order to stay in step with the file, also for tests it skips.
//
// How a test runs (the same idea as sst.cpp, without any change to the core). The core starts from reset and fetches from address 0, where the runner puts a branch to a
// generated prologue. The prologue sets VBR (a vector table with one stub per vector), USP, MSP, the ISP, D0-D7, A0-A6 and SR, and branches to the instruction. The
// words behind the instruction are ILLEGAL and MOVEA.L A0,A0 (swapped every test, as the Amiga program does), so every test ends in a vector stub: vector 4 (illegal
// instruction) right behind the instruction when it ran normally, or the instruction's own exception. The state at the first retire inside a stub is compared with the data:
// D0-D7, A0-A6, USP, the stacked SR, the stacked PC, the frame format and vector word, the exception number, and every memory write the data lists. Stores that the
// data does not list (and that are not part of the exception frame) are reported too. For the FPU sets the FP registers, FPCR, FPSR and FPIAR are loaded by the prologue
// and compared as well.
//
// usage: cputest SKIPFILE DATADIR NAME [max_groups] [show] [only_group] [stride]
//   SKIPFILE = tools/cputest_skip.txt, DATADIR = build/cputest-gen/data/2_BASIC (or the FPU data folder), NAME = instruction folder (ADD.L),
//   max_groups 0 = all, show = number of failures to print in full (default 3), only_group = run just that group with a retire trace, stride N = every Nth group
//   (CPUTEST_OFFSET chooses which, so that one big folder can be run as N parallel shards).
// Results on the release state: 5,134,638 tests run, 5,128,108 pass, 0 fail, 6,530 undefined (CHK and CHK2 flags, which WinUAE does not know from silicon) and 1,984
// skipped (trace); the FPU sets: 864,016 run, all pass, 53,344 skipped by design (the forms the core sends to line F).
#include "Vp68_core.h"
// Verilator model of the core and Verilator's library header.
#include "verilated.h"
// Standard C and C++ headers.
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <algorithm>

// Verilator asks for the simulated time; the runner has none.
double sc_time_stamp() { return 0; }

// ---------------------------------------------------------------- Data format constants (the generator's side)
// An item in the data starts with one byte. Its low 5 bits say which value it is (CT_DREG... CT_MEMWRITES below), and bits 6-5 say how it is stored (a byte, a word, a long,
// or an FPU register; or for addresses: relative word, absolute word, absolute long). A byte with the top bit set (CT_END) ends a list.
// ---------------------------------------------------------------- data format constants (generator side)
// Which value: 0-7 D0-D7, 8-15 A0-A7, 16 supervisor stack pointer, 17 master stack pointer, 18 SR, 19 PC, 20-22 FPIAR, FPSR, FPCR, 23 extra data, 25 cycle count,
// 26 PC after the instruction, 27 branch target, 28 and 29 source and destination address of the instruction, 30 and 31 one memory write and the opcode bytes.
enum { CT_DREG = 0, CT_AREG = 8, CT_SSP = 16, CT_MSP = 17, CT_SR = 18, CT_PC = 19, CT_FPIAR = 20, CT_FPSR = 21, CT_FPCR = 22, CT_EDATA = 23,
       CT_CYCLES = 25, CT_ENDPC = 26, CT_BRANCHTARGET = 27, CT_SRCADDR = 28, CT_DSTADDR = 29, CT_MEMWRITE = 30, CT_MEMWRITES = 31,
       // The size codes, for values: byte, word, long, 10-byte FPU register; for addresses: relative word, absolute word, absolute long, relative byte (the same code 3 for both).
       CT_DATA_MASK = 31, CT_SIZE_MASK = 3 << 5, CT_SIZE_BYTE = 0, CT_SIZE_WORD = 1 << 5, CT_SIZE_LONG = 2 << 5, CT_SIZE_FPU = 3 << 5,
       CT_REL_WORD = 0, CT_ABS_WORD = 1 << 5, CT_ABS_LONG = 2 << 5, CT_PC_BYTES = 3 << 5, CT_REL_BYTE = 3 << 5,
       // List endings and markers: end of data with finish (0xff) or init (0xC0), skip this test (0xC1), skip registers, empty (0xC0 in a value list), override register (0xD0),
       // branched (0x40) and the exception number mask (63) in the end byte of a test.
       CT_END = 0x80, CT_END_FINISH = 0xff, CT_END_INIT = 0xC0, CT_END_SKIP = 0xC1, CT_SKIP_REGS = 0xC2, CT_EMPTY = 0xC0, CT_OVERRIDE_REG = 0xD0,
       CT_BRANCHED = 0x40, CT_EXC_MASK = 63 };
// The two words behind every tested instruction: MOVEA.L A0,A0 (a harmless instruction that the generator calls NOP) and ILLEGAL.
static const uint16_t NOP_OPCODE = 0x2048, ILLG_OPCODE = 0x4AFC;   // MOVEA.L A0,A0 and ILLEGAL

// ---------------------------------------------------------------- Bench memory
// ---------------------------------------------------------------- bench memory
// 16 MB of memory, 24 address bits.
static const uint32_t MEMSZ = 1u << 24, MASK = MEMSZ - 1;
// mem = the memory the core sees; base = the state the Amiga program would have between tests (changes by a test are undone from base).
static std::vector<uint8_t> mem(MEMSZ, 0), base(MEMSZ, 0);   // mem = live, base = state the Amiga program would have between tests
// touched = addresses written during a test, restored from base afterwards.
static std::vector<uint32_t> touched;
// Addresses of the runner's own code and data: vector table, stubs (8 bytes each), prologue.
static const uint32_t VBR_ADDR = 0xF00000, STUB_BASE = 0xF00200, PRO_ADDR = 0xF10000;   // vector table, stubs (8 bytes each), prologue
// put8, put16, put32: write into mem and record the address in touched (test side effects).
static void put8(uint32_t a, uint8_t v) { a &= MASK; mem[a] = v; touched.push_back(a); }
static void put16(uint32_t a, uint16_t v) { put8(a, v >> 8); put8(a + 1, v & 0xff); }
static void put32(uint32_t a, uint32_t v) { put16(a, v >> 16); put16(a + 2, v & 0xffff); }
// bput8: write into both mem and base (permanent for the test group).
static void bput8(uint32_t a, uint8_t v) { a &= MASK; mem[a] = base[a] = v; }
// bw16 and bw32: read a big endian word or long from base.
static uint16_t bw16(uint32_t a) { return (uint16_t)((base[a & MASK] << 8) | base[(a + 1) & MASK]); }
static uint32_t bw32(uint32_t a) { return ((uint32_t)bw16(a) << 16) | bw16(a + 2); }

// ---------------------------------------------------------------- Test data header
// ---------------------------------------------------------------- test data header
// Hdr = the header chunk: FPU model, test memory start and size, address of the opcode, low memory range, user and supervisor stack, flags (the CPU type is in bits 19-16) and the name.
struct Hdr { uint32_t fpu_model = 0, tmem_start = 0, tmem_size = 0, opc_start = 0, lmem_start = 0, lmem_end = 0, user_stack = 0, super_stack = 0, flags = 0; char name[17] = {0}; };
// be32 and be16: read big endian numbers from the data.
static uint32_t be32(const uint8_t* p) { return ((uint32_t)p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]; }
static uint16_t be16(const uint8_t* p) { return (uint16_t)((p[0] << 8) | p[1]); }

// ---------------------------------------------------------------- Skip rules (same format as the skip list of sst.cpp, see tools/cputest_skip.txt)
// ---------------------------------------------------------------- skip rules
// OpSkip = an opcode rule: opcode AND mask = value, with the reason.
struct OpSkip { uint32_t mask, val; std::string why; };
// file_skip = skip a whole instruction folder, with the reason.
static std::map<std::string, std::string> file_skip;
// rule_off = built-in rules switched off by the skip file.
static std::set<std::string> rule_off;
// op_skip = the opcode rules.
static std::vector<OpSkip> op_skip;
// rule(): true unless the skip file switched the built-in rule off.
static bool rule(const char* n) { return !rule_off.count(n); }
// load_skips(): read the skip file (lines "file NAME reason", "op MASK VALUE reason", "rule NAME off"; lines that start with # are comments).
static void load_skips(const char* path) {
  FILE* f = fopen(path, "r"); if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
  char line[512];
  // rest(): the text behind the first n characters of the line, without leading spaces and the line end: the reason.
  auto rest = [](const char* s, int n) { std::string w = s + n; while (!w.empty() && w[0] == ' ') w.erase(0, 1); while (!w.empty() && (w.back() == '\n' || w.back() == '\r')) w.pop_back(); return w; };
  while (fgets(line, sizeof line, f)) {
    // k, a, b = the first three words of the line, n = characters used by the first two.
    char k[16], a[64], b[64]; int n = 0;
    // Comments and short lines are ignored.
    if (line[0] == '#' || sscanf(line, "%15s %63s%n", k, a, &n) < 2) continue;
    if (!strcmp(k, "rule")) { char o[16] = ""; sscanf(line, "%15s %63s %15s", k, a, o); if (!strcmp(o, "off")) rule_off.insert(a); }
    else if (!strcmp(k, "file")) file_skip[a] = rest(line, n);
    else if (!strcmp(k, "op")) { int m = 0; if (sscanf(line, "%15s %63s %63s%n", k, a, b, &m) < 3) continue; op_skip.push_back({(uint32_t)strtoul(a, 0, 0), (uint32_t)strtoul(b, 0, 0), rest(line, m)}); }
  }
  fclose(f);
}

// ---------------------------------------------------------------- Expected state (the Amiga program's last_regs)
// ---------------------------------------------------------------- expected state (the Amiga program's last_regs)
// Regs = a complete processor state: 16 integer registers, 8 FPU registers of 3 words, FPCR, FPSR, FPIAR, SR, PC, PC after the instruction, source and destination address, branch target and its mode.
struct Regs { uint32_t r[16] = {0}; uint32_t fp[8][3] = {{0}}, fpcr = 0, fpsr = 0, fpiar = 0; uint32_t sr = 0, pc = 0, endpc = 0, srcaddr = 0xffffffff, dstaddr = 0xffffffff, btarget = 0xffffffff; int bmode = 0; };
// Hist = one memory write of the initial data, with the old value, so that it can be undone at the end of the group.
struct Hist { uint32_t addr; uint32_t oldv; int size; };
// Frame = the expected exception frame bytes and length.
struct Frame { uint8_t b[32]; int len = 0; };

// H = the header of the current data file.
static Hdr H;
static const uint8_t *dp_end;

// restore_value(): decode one value item. The item says byte, word or long; a byte or word only replaces the low part of the old value (that is how differences are stored).
// Returns the position behind the item and the size code in *sz.
static const uint8_t* restore_value(const uint8_t* p, uint32_t* vp, int* sz = nullptr) {
  uint32_t val = *vp; uint8_t v = *p++; int s = 0;
  // Look at the size bits: a byte replaces the low 8 bits, a word the low 16, a long everything.
  switch (v & CT_SIZE_MASK) {
    case CT_SIZE_BYTE: val = (val & 0xffffff00u) | *p++; s = 0; break;
    case CT_SIZE_WORD: val = (val & 0xffff0000u) | (p[0] << 8) | p[1]; p += 2; s = 1; break;
    case CT_SIZE_LONG: val = be32(p); p += 4; s = 2; break;
    // Any other size code is not valid for an integer value.
    default: fprintf(stderr, "unexpected FPU size item %02x\n", v); exit(2);
  }
  *vp = val; if (sz) *sz = s; return p;
}
// restore_fpvalue(): decode an FPU register item. A size byte of 0xff means a full 10 byte register (exponent word with the sign, then the 64-bit mantissa). Otherwise the item
// is a difference against the old value: the high nibble n1 says that the leading bytes 0 to n1-1 follow in order, the low nibble n2 that the bytes 9, 8 and down to 10-n2 follow. v[0] is the
// exponent word, v[1] the high mantissa word and v[2] the low mantissa word.
// FPU register item (generator store_fpureg): size marker 0xff = full 10 bytes (exponent word with sign, 64-bit mantissa), otherwise a delta
// against the old value: high nibble n1 = leading bytes 0..n1-1 follow in order, low nibble n2 = bytes 9, 8, ... 10-n2 follow. v[0] = exponent
// word, v[1] = mantissa high, v[2] = mantissa low.
static const uint8_t* restore_fpvalue(const uint8_t* p, uint32_t* v) {
  if ((*p++ & CT_SIZE_MASK) != CT_SIZE_FPU) { fprintf(stderr, "expected FPU size item\n"); exit(2); }
  uint8_t sz = *p++;
  if (sz == 0xff) { v[0] = be16(p); v[1] = be32(p + 2); v[2] = be32(p + 6); return p + 10; }
  if (sz == 0) { v[0] = v[1] = v[2] = 0; return p; }
  // Spread the old value into 10 bytes, replace the leading and trailing bytes the item carries, and put the bytes back together.
  uint8_t f[10] = {(uint8_t)(v[0] >> 8), (uint8_t)v[0], (uint8_t)(v[1] >> 24), (uint8_t)(v[1] >> 16), (uint8_t)(v[1] >> 8), (uint8_t)v[1],
                   (uint8_t)(v[2] >> 24), (uint8_t)(v[2] >> 16), (uint8_t)(v[2] >> 8), (uint8_t)v[2]};
  for (int i = 0; i < (sz >> 4); i++) f[i] = *p++;
  for (int i = 0; i < (sz & 15); i++) f[9 - i] = *p++;
  v[0] = (f[0] << 8) | f[1]; v[1] = ((uint32_t)f[2] << 24) | (f[3] << 16) | (f[4] << 8) | f[5]; v[2] = ((uint32_t)f[6] << 24) | (f[7] << 16) | (f[8] << 8) | f[9];
  return p;
}
// restore_rel(): decode an address or branch item stored relative to the old value: a signed byte or word offset, or an absolute word (sign extended) or long.
static const uint8_t* restore_rel(const uint8_t* p, uint32_t* vp) {
  uint32_t v = *vp;
  switch (*p++ & CT_SIZE_MASK) {
    case CT_REL_BYTE: v += (int8_t)*p++; break;
    case CT_REL_WORD: v += (int16_t)be16(p); p += 2; break;
    case CT_ABS_WORD: v = (uint32_t)(int32_t)(int16_t)be16(p); p += 2; break;
    case CT_ABS_LONG: v = be32(p); p += 4; break;
  }
  *vp = v; return p;
}
// restore_rel_ordered(): the same, but the marker CT_EMPTY means no value follows.
static const uint8_t* restore_rel_ordered(const uint8_t* p, uint32_t* vp) { if (*p == CT_EMPTY) return p + 1; return restore_rel(p, vp); }

// mem_addr(): the address of a memory write item: absolute word, absolute long, or relative to the opcode address.
// address of a CT_MEMWRITE item: returns the 32-bit address (low memory, test memory or relative to the opcode area)
static const uint8_t* mem_addr(const uint8_t* p, uint32_t* ap) {
  uint8_t v = *p++;
  switch (v & CT_SIZE_MASK) {
    // Absolute word: a negative value is the sign extended 32-bit address at the top of memory, which the 24-bit bench memory maps to 0xFF8000 and up.
    case CT_ABS_WORD: { int16_t o = (int16_t)be16(p); p += 2; *ap = o < 0 ? 0xFFFF8000u + (uint32_t)(o + 32768) : (uint32_t)o; return p; }
    case CT_ABS_LONG: *ap = be32(p); return p + 4;
    case CT_REL_WORD: *ap = H.opc_start + (int16_t)be16(p); return p + 2;
  }
  fprintf(stderr, "bad memory address item %02x\n", v); exit(2);
}
// rd_mem(): read a value of 2 to the power size bytes from base, big endian.
static uint32_t rd_mem(uint32_t a, int size) {   // from base image (test start state)
  a &= MASK; uint32_t v = 0; for (int i = 0; i < (1 << size); i++) v = (v << 8) | base[(a + i) & MASK]; return v;
}
// tomem(): write a value into base (and mem) and optionally remember the old value in a history list for undoing at the end of the group.
static void tomem(uint32_t a, uint32_t v, int size, std::vector<Hist>* h, uint32_t oldv) {
  if (h) h->push_back({a, oldv, size});
  for (int i = 0; i < (1 << size); i++) bput8(a + i, (uint8_t)(v >> (8 * ((1 << size) - 1 - i))));
}

// ---------------------------------------------------------------- Running the core
// ---------------------------------------------------------------- RTL run
// Snap = what the core did: finished or not, the vector of the stub it reached, the register values, stack pointer banks, SR, the bytes it stored, the FPU state and a reason text for failures.
struct Snap {
  bool done = false; int vec = -1; uint32_t sh[16] = {0}, bk[3] = {0}, sr = 0; std::map<uint32_t, uint8_t> wr;
  uint32_t fp[8][3] = {{0}}, fpcr = 0, fpsr = 0, fpiar = 0;   // FPU state at the compare point (ret_fp0..7, ret_fpcr, ret_fpsr, ret_fpiar)
  std::string why;
};
// cycles_per_test = the longest run of one test, in clocks.
static int cycles_per_test = 12000;
// trace_on: with the environment variable CPUTEST_TRACE set the runner prints every retire and every memory read (use it with a single group).
static bool trace_on = getenv("CPUTEST_TRACE") != nullptr;   // retire trace (use with only_group)

// Address where the prologue finds the FPU register image to load with FMOVEM.X.
static const uint32_t FPD_ADDR = 0xF20000;   // FPU register image the prologue loads with FMOVEM.X
// FpuIn = the starting FPU state for the prologue.
struct FpuIn { const uint32_t (*fp)[3]; uint32_t fpcr, fpsr, fpiar; };
// put_prologue(): write the prologue into memory. It loads the state of the test and branches to the instruction.
static void put_prologue(uint32_t pro, uint32_t usp, uint32_t msp, uint32_t ssp, const uint32_t* d, const uint32_t* a, uint32_t sr, uint32_t pc, const FpuIn* fpu = nullptr) {
  // c = code words, l2 = emit a 6-byte instruction (opcode and 32-bit immediate).
  std::vector<uint16_t> c;
  auto l2 = [&](uint16_t op, uint32_t v) { c.push_back(op); c.push_back((uint16_t)(v >> 16)); c.push_back((uint16_t)v); };
  l2(0x2C7C, VBR_ADDR); c.push_back(0x4E7B); c.push_back(0xE801);             // MOVEA.L #vbr,A6; MOVEC A6,VBR
  l2(0x2C7C, usp); c.push_back(0x4E66);                                       // MOVEA.L #usp,A6; MOVE A6,USP
  l2(0x2C7C, msp); c.push_back(0x4E7B); c.push_back(0xE803);                  // MOVEA.L #msp,A6; MOVEC A6,MSP
  l2(0x2E7C, ssp);                                                            // MOVEA.L #ssp,A7 (S = 1, M = 0: ISP)
  // FPU tests: as the Amiga program does, load FP0-FP7 from an image in memory with FMOVEM.X, then FPIAR, FPCR and FPSR with one FMOVE.L #imm each (the core does these in hardware).
  if (fpu) {   // as the Amiga program: FMOVEM.X (image),FP0-FP7, then FPIAR, FPCR, FPSR (one FMOVE.L #imm each: the core does these in hardware)
    // Write the register images (12 bytes each: exponent word, 16 unused bits, 64-bit mantissa).
    for (int i = 0; i < 8; i++) { uint32_t b = FPD_ADDR + 12 * i; put16(b, (uint16_t)fpu->fp[i][0]); put16(b + 2, 0); put32(b + 4, fpu->fp[i][1]); put32(b + 8, fpu->fp[i][2]); }
    l2(0x2C7C, FPD_ADDR); c.push_back(0xF216); c.push_back(0xD0FF);           // MOVEA.L #img,A6; FMOVEM.X (A6),FP0-FP7
    c.push_back(0xF23C); c.push_back(0x8400); c.push_back(fpu->fpiar >> 16); c.push_back((uint16_t)fpu->fpiar);   // FMOVE.L #fpiar,FPIAR
    c.push_back(0xF23C); c.push_back(0x9000); c.push_back(fpu->fpcr >> 16); c.push_back((uint16_t)fpu->fpcr);     // FMOVE.L #fpcr,FPCR
    c.push_back(0xF23C); c.push_back(0x8800); c.push_back(fpu->fpsr >> 16); c.push_back((uint16_t)fpu->fpsr);     // FMOVE.L #fpsr,FPSR
  }
  // D0-D7 with MOVE.L #imm,Dn and A0-A6 with MOVEA.L #imm,An (0x203C and 0x207C are the opcodes for register 0; the register number goes into bits 11-9).
  for (int i = 0; i < 8; i++) l2((uint16_t)(0x203C | (i << 9)), d[i]);
  for (int i = 0; i < 7; i++) l2((uint16_t)(0x207C | (i << 9)), a[i]);
  c.push_back(0x46FC); c.push_back((uint16_t)sr);                             // MOVE #sr,SR (swaps A7 when S/M change)
  // Address of the final branch, needed for its displacement.
  uint32_t bra_at = pro + 2 * (uint32_t)c.size();
  // BRA.L to the instruction under test: 0x60FF and a 32-bit displacement relative to the address behind the opcode word.
  c.push_back(0x60FF); { uint32_t dd = pc - (bra_at + 2); c.push_back((uint16_t)(dd >> 16)); c.push_back((uint16_t)dd); }
  for (size_t i = 0; i < c.size(); i++) put16(pro + 2 * (uint32_t)i, c[i]);
}

// ---------------------------------------------------------------- The memory port model
// The only code that touches the core's memory pins. The port handles one transaction at a time: it takes 2 clocks for each read beat and for the completion of a write, a read
// returns 64 bits per beat (a line read has several beats), and a write changes memory when it completes. unc_map is 0, so everything is cacheable, but the caches
// are off (CACR stays 0), so every access goes to this port. Same model as in sst.cpp.
// ---------------------------------------------------------------- memory port model (the only code that touches the core's memory pins)
struct MemPort {
  // State: busy, write, ready, done, address, write data, beats, current beat, wait counter, byte enables, read data.
  bool busy = false, wr = false, rdy = false, done = false; uint32_t addr = 0, wdata = 0; int beats = 0, nb = 0, wait = 0, be = 0; uint64_t rdata = 0;
  // rd32(): a big endian 32-bit word of mem.
  uint32_t rd32(uint32_t a) { return ((uint32_t)mem[a & MASK] << 24) | ((uint32_t)mem[(a + 1) & MASK] << 16) | ((uint32_t)mem[(a + 2) & MASK] << 8) | mem[(a + 3) & MASK]; }
  void init(Vp68_core* t) { t->m_rdy = 0; t->m_done = 0; t->m_rdata = 0; t->m_snoop = 0; t->m_snoop_addr = 0; t->unc_map = 0; }
  // pre_clock(): present the answers before the rising clock edge.
  void pre_clock(Vp68_core* t) { t->m_rdy = rdy; t->m_done = done; t->m_rdata = rdata; }          // before the rising edge
  // post_clock(): after the edge, take a request, count the latency, serve beats, apply writes.
  void post_clock(Vp68_core* t) {                                                  // after the rising edge: accept requests, serve beats, apply writes to mem
    // The answer flags last for one clock.
    rdy = done = false;
    // New request: take over its address, data, byte enables and number of beats; the latency starts at 2.
    if (t->m_req) {
      busy = true; wr = t->m_wr; addr = (t->m_addr << 2) & MASK; wdata = t->m_wdata; be = t->m_be; beats = t->m_len + 1; nb = 0; wait = 2;
      if (!wr && trace_on && addr < 0x700000) printf("    read %08x\n", addr);
    // A transaction is running: count down the latency; when it is over a write puts the enabled bytes into memory, a read returns one 64-bit beat (a single access gets the aligned double word) and restarts the wait of 2 for the next beat.
    } else if (busy) {
      if (wait > 1) wait--;
      else if (wr) {
        for (int i = 0; i < 4; i++) if (be & (8 >> i)) { uint32_t a = (addr + i) & MASK; mem[a] = (wdata >> (24 - 8 * i)) & 0xff; touched.push_back(a); }
        done = true; busy = false;
      } else {
        uint32_t a = beats == 1 ? (addr & ~7u) : addr + 8 * nb;
        rdata = ((uint64_t)rd32(a) << 32) | rd32(a + 4); rdy = true; wait = 2;
        if (++nb == beats) { done = true; busy = false; }
      }
    }
  }
};

// grab_fpu(): copy the FP registers and the three control registers from the retire port into a Snap.
static void grab_fpu(Vp68_core* t, Snap& s) {
// One macro line per register: the exponent word is the low 16 bits of word 2 of the port's 80-bit value.
#define GFP(n) { s.fp[n][0] = t->ret_fp##n[2] & 0xffff; s.fp[n][1] = t->ret_fp##n[1]; s.fp[n][2] = t->ret_fp##n[0]; }
  GFP(0) GFP(1) GFP(2) GFP(3) GFP(4) GFP(5) GFP(6) GFP(7)
#undef GFP
  s.fpcr = t->ret_fpcr; s.fpsr = t->ret_fpsr; s.fpiar = t->ret_fpiar;
}

// run_rtl(): run the core until it reaches a vector stub (or gives up) and return the state at that retire.
static Snap run_rtl(uint32_t pc_test) {
  Snap s;
  // The reset stub: BRA.L from address 0 to the prologue.
  put16(0, 0x60FF); { uint32_t d = PRO_ADDR - 2; put16(2, d >> 16); put16(4, d & 0xffff); }   // reset stub: BRA.L to the prologue
  auto ctx = std::make_unique<VerilatedContext>();
  auto top = std::make_unique<Vp68_core>(ctx.get());
  // Shadow registers D0-D7 and A0-A7 and the stack pointer banks, built from the retire records; wr = the bytes the core stored.
  uint32_t sh[16] = {0}, bk[3] = {0};
  std::map<uint32_t, uint8_t> wr;
  MemPort mp;
  top->clk = 0; top->rst = 1; mp.init(top.get()); top->ipl = 0; top->eval();
  // idle = clocks without a retire; stub_gone = the reset stub has been left.
  int idle = 0; bool stub_gone = false;
  (void)pc_test;
  // The clock loop, with a limit of cycles_per_test clocks: falling edge with the memory answers, rising edge, reset released after 4 clocks, then the memory port reacts.
  for (long cyc = 0; cyc < cycles_per_test; cyc++) {
    top->clk = 0; mp.pre_clock(top.get()); top->eval();
    top->clk = 1; top->eval();
    if (cyc == 4) top->rst = 0;
    mp.post_clock(top.get());
    // wrA(): apply lane A's committed writes to the shadow registers and the store list; the core can retire two instructions per clock, in two lanes (A and B).
    auto wrA = [&]() {
      if (top->ret_u) {
        // The two address register updates of the address generation stage ((An)+ and -(An) of the source and of the destination or LEA), the write-back register write, a stack pointer bank write, and the stores with their byte enables.
        if (top->ret_ws_en) sh[8 + top->ret_ws_r] = top->ret_ws_v;
        if (top->ret_wd_en) sh[8 + top->ret_wd_r] = top->ret_wd_v;
        if (top->ret_wen) sh[top->ret_wreg] = top->ret_wval;
        if (top->ret_bk_en) bk[top->ret_bk_r] = top->ret_wval;
        if (top->ret_st) for (int i = 0; i < 4; i++) if (top->ret_sbe & (8 >> i)) wr[((top->ret_saddr & ~3u) + i) & MASK] = (top->ret_sdata >> (24 - 8 * i)) & 0xff;
        // A second word of a misaligned store that crosses a longword.
        if (top->ret_st2) for (int i = 0; i < 4; i++) if (top->ret_sbe2 & (8 >> i)) wr[((top->ret_saddr2 & ~3u) + i) & MASK] = (top->ret_sdata2 >> (24 - 8 * i)) & 0xff;
      }
      // A7 swap (the S and M bits changed).
      if (top->ret_sw) { bk[top->ret_sw_old] = sh[15]; sh[15] = top->ret_sw_v; }
    };
    // wrB(): lane B's register write.
    auto wrB = [&]() { if (top->ret2_wen) sh[top->ret2_wreg] = top->ret2_wval; };
    // Nothing retired: only a stack pointer swap can still happen; give up after 3000 idle clocks.
    if (!top->ret_u && !top->ret2_v) { if (top->ret_sw) { bk[top->ret_sw_old] = sh[15]; sh[15] = top->ret_sw_v; } if (++idle > 3000) break; continue; }
    idle = 0;
    // ev(): look at one retired instruction. True = stop the run.
    auto ev = [&](uint32_t rpc, uint32_t rsr, bool rirq) -> bool {   // true: stop
      (void)rirq;
      if (trace_on) printf("    retire %08x sr %04x\n", rpc, rsr);
      // After the first retire outside the reset stub, put the original low memory (vectors) back: the reset stub had overwritten its first 6 bytes.
      if (!stub_gone && rpc != 0) {   // the reset stub is done: low memory gets its test content back
        stub_gone = true; for (int i = 0; i < 6; i++) mem[i] = base[i];
      }
      // A retire at the start of one of the 64 stubs (one per vector, 8 bytes apart) ends the test. Take the snapshot of registers, banks, SR, FPU state and stores, and the vector number.
      if (rpc >= STUB_BASE && rpc < STUB_BASE + 8 * 64 && ((rpc - STUB_BASE) & 7) == 0) {
        memcpy(s.sh, sh, sizeof sh); memcpy(s.bk, bk, sizeof bk); s.sr = rsr; grab_fpu(top.get(), s); s.wr = wr; s.vec = 2 + (int)((rpc - STUB_BASE) >> 3); s.done = true; return true;
      }
      return false;
    };
    // Process the retires of this clock in program order: lane B first if it is the older, then lane A's writes and instruction, then lane B if it is the younger.
    const bool rB = top->ret2_v, bF = rB && top->ret2_first;
    if (bF) { wrB(); if (ev(top->ret2_pc, (top->ret_sr & 0xffe0) | top->ret2_ccr, false)) break; }
    wrA();
    if (top->ret_v && ev(top->ret_pc, (top->ret_sr & 0xffe0) | top->ret_ccr, top->ret_irq)) break;
    if (rB && !bF) { wrB(); if (ev(top->ret2_pc, (top->ret_sr & 0xffe0) | top->ret2_ccr, false)) break; }
  }
  // The reason text if the run did not finish.
  if (!s.done) s.why = stub_gone ? "no exception stub reached (hang or wrong path)" : "reset stub never left";
  return s;
}

// ---------------------------------------------------------------- Statistics
// ---------------------------------------------------------------- statistics
// Stats = counters (groups, tests, run, pass, fail, undefined, skipped) and counts per skip reason and failure cause.
struct Stats { long groups = 0, tests = 0, run = 0, pass = 0, fail = 0, undef = 0, skip = 0; std::map<std::string, long> skipwhy, cause; };

// fmt_hex(): an 8-digit hex string.
static std::string fmt_hex(uint32_t v) { char b[16]; snprintf(b, sizeof b, "%08x", v); return b; }

// main(): parse the arguments, load the data, walk through the groups, run the tests and print the summary.
int main(int argc, char** argv) {
  if (argc < 4) { fprintf(stderr, "usage: cputest SKIPFILE DATADIR NAME [max_groups] [show] [only_group] [stride]\n"); return 2; }
  load_skips(argv[1]);
  std::string ddir = argv[2], name = argv[3];
  // maxg = group limit, show, only = one group with trace, stride = every Nth group.
  long maxg = argc > 4 ? atol(argv[4]) : 0; int show = argc > 5 ? atoi(argv[5]) : 3; long only = argc > 6 ? atol(argv[6]) : 0; long stride = argc > 7 ? atol(argv[7]) : 1;
  if (stride < 1) stride = 1;
  // offset = which group of every stride group: lets N processes each take a different slice of one folder.
  long offset = getenv("CPUTEST_OFFSET") ? atol(getenv("CPUTEST_OFFSET")) : 0;   // with stride N: groups offset, offset + N, ... (run one folder as N parallel shards)
  // slurp(): read a whole file into a byte vector.
  auto slurp = [](const std::string& path, std::vector<uint8_t>& out) { FILE* f = fopen(path.c_str(), "rb"); if (!f) return false; fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET); out.resize(n); size_t r = fread(out.data(), 1, n, f); fclose(f); return r == (size_t)n; };
  // lm = low memory image, tm = test memory image, df = the instruction's test data.
  std::vector<uint8_t> lm, tm, df;
  if (!slurp(ddir + "/lmem.dat", lm) || !slurp(ddir + "/tmem.dat", tm) || !slurp(ddir + "/" + name + "/0000.dat", df)) { fprintf(stderr, "cannot read data for %s\n", name.c_str()); return 2; }
  // Split the data into chunks: marker, size, data. A chunk starts with AF 'M' 'R' 'G'; the size is big endian and a zero size ends the data.
  // chunks: AF 'M' 'R' 'G' size(4) data
  std::vector<std::pair<const uint8_t*, size_t>> chunks;
  for (size_t o = 0; o + 8 <= df.size();) {
    if (!(df[o] == 0xaf && df[o + 1] == 'M' && df[o + 2] == 'R' && df[o + 3] == 'G')) break;
    size_t n = be32(&df[o + 4]); if (n == 0) break; chunks.push_back({&df[o + 8], n}); o += 8 + n;
  }
  // At least the header chunk and one test chunk are needed.
  if (chunks.size() < 2) { fprintf(stderr, "no test chunks in %s\n", name.c_str()); return 2; }
  // The header chunk. Version 24 of the data format is the only one the runner knows; then FPU model, test memory start and size, opcode address, flags, low memory range and the two stacks.
  { const uint8_t* h = chunks[0].first;
    if (be32(h) != 24) { fprintf(stderr, "data version %u, expected 24\n", be32(h)); return 2; }
    H.fpu_model = be32(h + 40);
    H.tmem_start = be32(h + 12); H.tmem_size = be32(h + 16); H.opc_start = be32(h + 20); H.flags = be32(h + 24);
    H.lmem_start = be32(h + 44); H.lmem_end = be32(h + 48); H.user_stack = be32(h + 68); H.super_stack = be32(h + 72);
    memcpy(H.name, h + 92, 16);
    // Only 68020 data is accepted (flags bits 19-16 = 2).
    if (((H.flags >> 16) & 15) != 2) { fprintf(stderr, "not 68020 data\n"); return 2; }
  }
  // Load the low memory image and the test memory image into mem and base.
  for (size_t i = 0; i < lm.size() && i < 0x8000; i++) { mem[i] = base[i] = lm[i]; }
  for (size_t i = 0; i < tm.size(); i++) { mem[(H.tmem_start + i) & MASK] = base[(H.tmem_start + i) & MASK] = tm[i]; }
  // The vector table and the stubs: every vector points to a stub NOP, BRA.S * (the stub is entered, the runner sees the retire there, and the stub keeps looping).
  // vector table and stubs: NOP; BRA.S *
  // Vectors 0 and 1 (reset) get the last two stubs; the others come in order from vector 2.
  for (int v = 0; v < 64; v++) { uint32_t stub = STUB_BASE + 8 * (v - 2); if (v < 2) stub = STUB_BASE + 8 * 62 + 8 * v;
    for (int i = 0; i < 4; i++) bput8(VBR_ADDR + 4 * v + i, (uint8_t)(stub >> (24 - 8 * i)));
    bput8(stub, 0x4E); bput8(stub + 1, 0x71); bput8(stub + 2, 0x60); bput8(stub + 3, 0xFE); }
  // The supervisor stacks: the ISP is 0x80 below the master stack pointer.
  const uint32_t ssp = H.super_stack - 0x80, msp = H.super_stack;

  // st = statistics, shown = failures printed so far, gidx = group number.
  Stats st; int shown = 0; long gidx = 0;
  // cur = the current state: registers and addresses of the group.
  Regs cur; cur.sr = 0; cur.srcaddr = cur.dstaddr = cur.btarget = 0xffffffff;
  // last_frame = the last exception frame data (a frame can be reused with the marker 0xff); start_pc and end_pc = addresses of the instruction and behind it; ignore = the mask of undefined flags that the data gives with SR, kept from test to test.
  Frame last_frame; uint32_t start_pc = H.opc_start, end_pc = H.opc_start, ignore = 0;   // ignore: high half of the last SR item (undefined-flag mask), kept between tests
  // sizes = the operand size taken from the folder name (".L" gives "L").
  std::string sizes = name.size() > 2 && name[name.size() - 2] == '.' ? name.substr(name.size() - 1) : "";
  // The two CCR move folders use a word operand.
  if (name == "MV2SR.B" || name == "MVSR2.B") sizes = "W";   // MOVE to CCR and from CCR use a word operand
  // abort_all = stop everything (group limit or selected group reached).
  bool abort_all = false;
  // For each test chunk, p = read position in the chunk (after its 16-byte header); for each group in the chunk:
  for (size_t ci = 1; ci < chunks.size() && !abort_all; ci++) {
    const uint8_t* p = chunks[ci].first + 16; const uint8_t* cend = chunks[ci].first + chunks[ci].second; (void)cend;
    for (;;) {
      // ---- group initial data: items that set the starting state of this group, ended by CT_END_INIT (or CT_END_FINISH at the end of the chunk).
      // ---- group initial data
      cur.endpc = end_pc; cur.pc = start_pc;
      // hist = memory writes of this group, undone at its end.
      std::vector<Hist> hist;
      for (;;) {
        uint8_t v = *p; if (v == CT_END_INIT || v == CT_END_FINISH) break;
        // mode = which value this item sets. Source and destination address, PC after the instruction, PC of the instruction, branch target with its mode (how the end words at the target are toggled), FPU register, D0-D7 and A0-A7 (a byte, word or long replaces the old value), FPIAR, FPSR, FPCR and SR follow.
        int mode = v & CT_DATA_MASK;
        if (mode == CT_SRCADDR) p = restore_value(p, &cur.srcaddr);
        else if (mode == CT_DSTADDR) p = restore_value(p, &cur.dstaddr);
        else if (mode == CT_ENDPC) p = restore_value(p, &cur.endpc);
        else if (mode == CT_PC) p = restore_value(p, &cur.pc);
        else if (mode == CT_BRANCHTARGET) { p = restore_value(p, &cur.btarget); cur.bmode = *p++; }
        else if (mode < CT_AREG + 8 && (v & CT_SIZE_MASK) == CT_SIZE_FPU) p = restore_fpvalue(p, cur.fp[mode]);
        else if (mode < CT_AREG + 8) { p = restore_value(p, &cur.r[mode]); }
        else if (mode == CT_FPIAR) p = restore_value(p, &cur.fpiar);
        else if (mode == CT_FPSR) p = restore_value(p, &cur.fpsr);
        else if (mode == CT_FPCR) p = restore_value(p, &cur.fpcr);
        else if (mode == CT_SR) p = restore_value(p, &cur.sr);
        // Opcode bytes: an offset and a length (a short form in one byte with the offset in the top 3 bits and the length in the low 5, or a long form after 0xff; a length of 0 means 32 or 256), then the bytes; they go into the memory images where the instruction lives.
        else if (mode == CT_MEMWRITES) {   // opcode bytes: offset/len then the bytes
          p++; uint32_t off, len;
          if (*p == 0xff) { p++; off = *p++; len = *p++; if (!len) len = 256; } else { uint8_t b = *p++; off = b >> 5; len = b & 31; if (!len) len = 32; }
          for (uint32_t i = 0; i < len; i++) bput8(H.opc_start + off + i, p[i]);
          p += len;
        // One memory write: address, old value (for undoing) and new value; the write goes into the images and the history. CPUTEST_CHECKOLD makes the runner verify that the old value is what the data says.
        } else if (mode == CT_MEMWRITE) {
          uint32_t a; int sz; uint32_t ov = 0, nv = 0;
          p = mem_addr(p, &a); p = restore_value(p, &ov, &sz); p = restore_value(p, &nv, &sz);
          if (getenv("CPUTEST_CHECKOLD") && rd_mem(a, sz) != ov) printf("  OLDMISMATCH %s chunk %zu group %ld addr %08x size %d: memory %x, data says old %x\n", name.c_str(), ci, gidx + 1, a, 1 << sz, rd_mem(a, sz), ov);
          tomem(a, nv, sz, &hist, ov);
          if (trace_on) printf("    init write %08x size %d old %x new %x\n", a, 1 << sz, ov, nv);
        // Extra data (3 bytes): not used. Anything else is a data error.
        } else if (mode == CT_EDATA) { p += 3; }
        else { fprintf(stderr, "%s: unexpected initial item %02x at test chunk %zu\n", name.c_str(), v, ci); return 2; }
      }
      // The end of the chunk.
      if (*p == CT_END_FINISH) break;
      p++;   // CT_END_INIT
      // The group starts at cur.pc and ends at cur.endpc.
      start_pc = cur.pc; end_pc = cur.endpc;
      gidx++;
      if (trace_on) printf("  == group %ld starts, pc %08x endpc %08x\n", gidx, cur.pc, cur.endpc);
      // Does this group run? It must be the selected group (if any), fall in the stride and offset slice, and be within the group limit. Groups that do not run are still parsed completely to stay in step with the data.
      bool run_this = (!only || gidx == only) && ((gidx - 1) % stride == offset) && (!maxg || st.groups < maxg);
      if (run_this) st.groups++;
      if (maxg && st.groups >= maxg && !only && !run_this) { /* keep parsing the group to stay in step, then stop below */ }

      // lr = the expected state after the test (last_regs): starts as the group's starting state and is changed by the result items of each test.
      Regs lr = cur;   // last_regs: copyregs(&last_regs, &cur_regs)
      // opcodeend = the two end words, ILLEGAL and MOVEA.L A0,A0, in swapped order from test to test.
      uint32_t opcodeend = ((uint32_t)NOP_OPCODE << 16) | ILLG_OPCODE;
      // extraccr = which extra SR bits the round sets (see srmask below).
      int extraccr = 0;
      // opw = the first opcode word of the group.
      uint16_t opw = bw16(start_pc);
      // fpumode: the FPU state is loaded and compared for groups whose opcode is in the line F range, when the data has an FPU model.
      const bool fpumode = H.fpu_model && (base[H.opc_start & MASK] & 0xf0) == 0xf0;   // as the Amiga program: FPU state is set and compared for F-line groups
      // Rounds: each round is one SR setting.
      for (;;) {   // rounds (SR modes)
        // srmask = the SR bits to set in this round, from extraccr: bit 0 supervisor (0x2000), bit 1 trace bit T0 (0x4000), bit 2 trace bit T1 (0x8000), bit 3 master (0x1000).
        uint16_t srmask = 0;
        if (extraccr & 1) srmask |= 0x2000; if (extraccr & 2) srmask |= 0x4000; if (extraccr & 4) srmask |= 0x8000; if (extraccr & 8) srmask |= 0x1000;
        // ccrmode = how many condition code settings follow (maxccr tests); the flag bits 0x80 and 0x40 mark FPU groups and FPU condition code rounds.
        uint8_t ccrmode = *p++; int maxccr = ccrmode & 0x3f;   // 0x80: FPU group, 0x40: FPU condition code rounds
        // One test per condition code setting.
        for (int ccr = 0; ccr < maxccr; ccr++) {
          // The two end words swap every test.
          opcodeend = (opcodeend >> 16) | (opcodeend << 16);
          // endextra = bytes to add to the expected PC: 2 when the word behind the instruction is the NOP (it runs first and the ILLEGAL behind it raises the exception), 0 when the ILLEGAL comes right behind the instruction.
          int endextra = (opcodeend >> 16) == NOP_OPCODE ? 2 : 0;
          // The CCR for this test: the round number for 32-value rounds, otherwise all clear or all set (31).
          uint32_t tsr = (maxccr >= 32) ? (uint32_t)(ccr & 0xff) : ((ccr & 1) ? 31u : 0u);
          // Add the SR bits of the round.
          tsr |= srmask;
          // For FPU tests FPIAR is reset to all ones.
          if (fpumode) cur.fpiar = 0xffffffff;
          // tr = the registers of this test (starting state with overrides).
          Regs tr = cur;   // test_regs
          // FPU tests: FPSR and FPCR of the test come from the round number (not stored in the data): few rounds use all clear or all set condition codes and rounding bits, condition code rounds use the round number as the condition code, the others use it as rounding mode and precision.
          if (fpumode) {   // FPSR and FPCR of the test come from the round number (not stored in the data)
            tr.fpcr = tr.fpsr = 0;
            if (maxccr < 16) { tr.fpsr = ((ccr & 1) ? 15u : 0u) << 24; tr.fpcr = ((ccr & 1) ? 15u : 0u) << 4; }
            else if (ccrmode & 0x40) tr.fpsr = (uint32_t)(ccr & 15) << 24;
            else tr.fpcr = (uint32_t)(ccr & 15) << 4;
          }
          // Branch target orientation: for branch instructions the data toggles the words at the target between two forms every test (skipped tests too), so that the target is hit with the right code. Mode 1 swaps the two halves of the long word at the target, mode 2 swaps the word between NOP and ILLEGAL.
          // branch target orientation (toggles every test, skipped ones included)
          uint32_t bt_word = 0; bool bt_set = false; uint32_t bt_addr = cur.btarget;
          if (cur.btarget != 0xffffffff && !(cur.btarget & 1)) {
            if (cur.bmode == 1) { uint32_t bv = bw32(cur.btarget); bv = (bv >> 16) | (bv << 16); bput8(cur.btarget, bv >> 24); bput8(cur.btarget + 1, bv >> 16); bput8(cur.btarget + 2, bv >> 8); bput8(cur.btarget + 3, bv); bt_set = true; }
            else if (cur.bmode == 2) { uint16_t bv = bw16(cur.btarget); bv = bv == NOP_OPCODE ? ILLG_OPCODE : NOP_OPCODE; bput8(cur.btarget, bv >> 8); bput8(cur.btarget + 1, bv); bt_set = true; }
          }
          // The bt_ variables are not needed beyond this point.
          (void)bt_word; (void)bt_set; (void)bt_addr;
          // End words at the end of the instruction, written into the base image; the group restores the originals at its end.
          // end opcodes at the end of the instruction (base image: the group restores them at its end)
          { uint32_t e = cur.endpc; bput8(e, opcodeend >> 24); bput8(e + 1, opcodeend >> 16); bput8(e + 2, opcodeend >> 8); bput8(e + 3, opcodeend); }
          tsr |= 0;
          // Overrides: registers for this and following tests, SR for this one. They come as items with the marker CT_OVERRIDE_REG.
          // overrides (registers for the next round, SR for this one)
          while (*p == CT_OVERRIDE_REG) {
            // r = which register, size = how it is stored. FPU registers change in this test only, FPSR, FPCR and FPIAR for this and the following tests, an SR byte or word replaces the SR of this test, and an integer register becomes a full long for the following tests.
            p++; uint8_t v = *p++; int r = v & CT_DATA_MASK, size = v & CT_SIZE_MASK;
            if (size == CT_SIZE_FPU && r < 8) { tr.fp[r][0] = be32(p) >> 16; tr.fp[r][1] = be32(p + 4); tr.fp[r][2] = be32(p + 8); p += 12; }   // test only, not cur
            else if (r == CT_FPSR) { cur.fpsr = tr.fpsr = be32(p); p += 4; }
            else if (r == CT_FPCR) { cur.fpcr = tr.fpcr = be32(p); p += 4; }
            else if (r == CT_FPIAR) { cur.fpiar = tr.fpiar = be32(p); p += 4; }
            else if (r == CT_SR) {
              if (size == CT_SIZE_BYTE) { tsr = (tsr & 0xff00) | *p++; } else { tsr = (p[0] << 8) | p[1]; p += 2; }
              cur.sr = tsr;
            } else if (r < 16) { cur.r[r] = be32(p); p += 4; }
            else { fprintf(stderr, "%s: unknown override register %02x\n", name.c_str(), v); return 2; }
          }
          // The SR of this test; for FPU tests also the FPSR and FPCR.
          cur.sr = tsr;
          if (fpumode) { cur.fpsr = tr.fpsr; cur.fpcr = tr.fpcr; }
          // The first test of a group defines the expected SR and FPU control state.
          if (!srmask && !ccr) { lr.sr = tsr; if (fpumode) { lr.fpsr = tr.fpsr; lr.fpcr = tr.fpcr; } }
          // skipped = the generator marked this test skipped (CT_END_SKIP): it must still be parsed, but not run.
          bool skipped = false;
          if (*p == CT_END_SKIP) { p++; skipped = true; }
          // ---- expected results of this test, always parsed to stay in step.
          // ---- expected results of this test (always parsed to stay in step)
          // expw = memory writes the data lists: address, value, size.
          std::vector<std::pair<uint32_t, std::pair<uint32_t, int>>> expw;   // addr -> (value, size) listed memory writes
          // hist2 = old values of those writes; exc = exception number; trace = the exception is a trace exception; branched = a branch was taken.
          std::vector<Hist> hist2; int exc = 0; bool trace = false, branched = false;
          // fr = the expected exception frame; have_frame = the data has one. opc_restore = opcode bytes to put back after the test.
          Frame fr; bool have_frame = false;
          std::vector<std::pair<uint32_t, uint8_t>> opc_restore;
          // Skipped tests have no result items.
          if (!skipped) {
            // Read result items until the end byte.
            for (;;) {
              uint8_t v = *p;
              // The end byte of the test: the exception number in the low 6 bits and the branched flag.
              if (v & CT_END) {
                exc = v & CT_EXC_MASK; branched = (v & CT_BRANCHED) != 0; p++;
                // With an exception the frame data follows: a length byte (0xff = the same frame as the last one), a flags byte (extra 9 is the trace exception), the stacked PC relative to the opcode address, the format word (format in the top 4 bits) and for format 2 the instruction address.
                if (exc) {   // exception data
                  const uint8_t* op = p; uint8_t len = *p++;
                  if (len == 0xff) { fr = last_frame; have_frame = fr.len > 0; }
                  else if (len) {
                    uint8_t extra = *p++; if (extra & 0x40) { p++; extra &= ~0x40; }
                    if ((extra & 0x3f) == 9) trace = true; else if (extra) { fprintf(stderr, "%s: unsupported exception extra %d\n", name.c_str(), extra); return 2; }
                    if (exc != 1) {
                      uint32_t pcv = H.opc_start; p = restore_rel_ordered(p, &pcv);
                      uint16_t fmt = (uint16_t)((p[0] << 8) | p[1]); p += 2;
                      // Build the expected frame bytes: bytes 2 to 5 = the PC, 6 and 7 = the format word; format 0 is 8 bytes long, format 2 is 12 bytes long, other formats are not compared (length -1).
                      memset(fr.b, 0, sizeof fr.b);
                      fr.b[2] = pcv >> 24; fr.b[3] = pcv >> 16; fr.b[4] = pcv >> 8; fr.b[5] = pcv; fr.b[6] = fmt >> 8; fr.b[7] = fmt;
                      int f = fmt >> 12;
                      if (f == 0) fr.len = 8;
                      else if (f == 2) { uint32_t av = H.opc_start; p = restore_rel_ordered(p, &av); fr.b[8] = av >> 24; fr.b[9] = av >> 16; fr.b[10] = av >> 8; fr.b[11] = av; fr.len = 12; }
                      else fr.len = -1;   // other formats: not compared
                      have_frame = true; last_frame = fr;
                    }
                  }
                  // Continue behind the frame data.
                  p = op + (len == 0xff || len == 0 ? 0 : len) + 1;
                }
                break;
              }
              // Result items before the end byte: FPU register, FPIAR, FPSR, FPCR, D0-D7 and A0-A7, SR (the upper 16 bits of the item are the mask of undefined flags, kept in ignore), PC relative to the opcode address, the cycle count and memory writes.
              int mode = v & CT_DATA_MASK;
              if (mode < CT_AREG + 8 && (v & CT_SIZE_MASK) == CT_SIZE_FPU) { p = restore_fpvalue(p, lr.fp[mode]); }
              else if (mode == CT_FPIAR) { p = restore_value(p, &lr.fpiar); }
              else if (mode == CT_FPSR) { p = restore_value(p, &lr.fpsr); }
              else if (mode == CT_FPCR) { p = restore_value(p, &lr.fpcr); }
              else if (mode < CT_AREG + 8) { p = restore_value(p, &lr.r[mode]); }
              else if (mode == CT_SR) { p = restore_value(p, &lr.sr); ignore = lr.sr >> 16; }
              else if (mode == CT_PC) { p = restore_rel(p, &lr.pc); }
              // Cycle count: the data has it but the core's clocks are not compared; the value is decoded into a scratch variable and dropped.
              else if (mode == CT_CYCLES) { uint32_t cyc = 0; p = restore_value(p, &cyc); }
              // Opcode bytes to restore after the test.
              else if (mode == CT_MEMWRITES) {
                p++; uint32_t off, len;
                if (*p == 0xff) { p++; off = *p++; len = *p++; if (!len) len = 256; } else { uint8_t b = *p++; off = b >> 5; len = b & 31; if (!len) len = 32; }
                for (uint32_t i = 0; i < len; i++) opc_restore.push_back({H.opc_start + off + i, p[i]});
                p += len;
              // A memory write: address, old and new value.
              } else if (mode == CT_MEMWRITE) {
                uint32_t a; int sz; uint32_t ov = 0, nv = 0;
                p = mem_addr(p, &a); p = restore_value(p, &ov, &sz); p = restore_value(p, &nv, &sz);
                expw.push_back({a, {nv, sz}}); hist2.push_back({a, ov, sz});
              } else { fprintf(stderr, "%s: unknown test item %02x\n", name.c_str(), v); return 2; }
            }
          }
          // registers for the next group modified by the test: none here (done in the group-initial data)
          // ---- skip decision for a test that would run:
          // ---- skip decision
          // why = the reason to skip, empty to run. live = this test is selected and not skipped by the generator.
          std::string why;
          bool live = run_this && !skipped;
          // Skip by folder name and by opcode from the skip file.
          if (live) {
            auto f = file_skip.find(name); if (f != file_skip.end()) why = "file: " + f->second;
            // Built-in rule: tests that expect or use a trace exception, or run with the T bits of SR set (0xC000), because the core has no trace; exception 1 is the trace exception.
            for (auto& o : op_skip) if (why.empty() && (opw & o.mask) == o.val) why = "op: " + o.why;
            if (why.empty() && trace && rule("trace")) why = "trace exception (trace not in the core)";
            if (why.empty() && exc == 1) why = "trace exception (trace not in the core)";
            if (why.empty() && (tsr & 0xC000) && rule("trace")) why = "T bit set in SR (trace not in the core)";
            // Built-in rule misaligned: operands at odd addresses (switched off in the skip file now). m = alignment mask for word (1) and long (3).
            if (why.empty() && rule("misaligned") && (sizes == "W" || sizes == "L")) {
              uint32_t m = sizes == "W" ? 1 : 3;
              // The data's srcaddr and dstaddr come from the first (user) round; in supervisor rounds an A7-based effective address uses the ISP or MSP, so the addresses are adjusted by the stack pointer difference.
              // the data's srcaddr/dstaddr come from the first (user) round; in supervisor rounds an A7-based EA uses the ISP or MSP
              uint32_t sa = cur.srcaddr, da = cur.dstaddr;
              if (tsr & 0x2000) {
                uint32_t sp2 = (tsr & 0x1000) ? msp : ssp, adj = sp2 - tr.r[15];
                // a7ea(): is this EA an A7 mode that touches memory ((A7), (A7)+, -(A7), d16(A7), d8(A7,Xn)).
                auto a7ea = [](int mode, int reg) { return reg == 7 && mode >= 2 && mode <= 6; };
                // g = instruction group (MOVE has source and destination, so both addresses are adjusted).
                int g = opw >> 12;
                if (g >= 1 && g <= 3) { if (a7ea((opw >> 3) & 7, opw & 7)) sa += adj; if (a7ea((opw >> 6) & 7, (opw >> 9) & 7)) da += adj; }
                else if (a7ea((opw >> 3) & 7, opw & 7)) { sa += adj; da += adj; }
              }
              if ((sa != 0xffffffff && (sa & m)) || (da != 0xffffffff && (da & m))) why = "misaligned operand address (core wants aligned word and long)";
            }
            // Built-in rule misaligned, second part: implicit stack and pointer accesses of flow instructions. sp = the stack pointer in use for this round.
            if (why.empty() && rule("misaligned")) {   // implicit stack and pointer accesses of flow instructions
              uint32_t sp = (tsr & 0x2000) ? ((tsr & 0x1000) ? msp : ssp) : tr.r[15];
              bool bad = false;
              // RTS and RTD pop a long from SP; RTR pops a word and a long; JSR, PEA.L, LINK and BSR push a long at SP - 4; UNLK pops through An. Each must be long aligned.
              if (name == "RTS" || name == "RTD") bad = (sp & 3) != 0;
              else if (name == "RTR") bad = ((sp + 2) & 3) != 0 || (sp & 1);
              else if (name == "JSR" || name == "PEA.L" || name == "LINK.W" || name == "LINK.L" || name.rfind("BSR", 0) == 0) bad = ((sp - 4) & 3) != 0;
              else if (name == "UNLK.L") { int an = opw & 7; bad = ((an == 7 ? sp : tr.r[8 + an]) & 3) != 0; }
              if (bad) why = "misaligned stack or pointer access (core wants aligned long)";
            }
            // Built-in rule fpu_trapped: FPU forms that the core sends to line F by design (the model in sim/lockstep/fpu_ref.c does the same): packed decimal operands, immediate .D, .X and .P operands, FMOVEM.L #imm with two or three registers, FMOVEM.X with a dynamic register list.
            if (why.empty() && fpumode && rule("fpu_trapped")) {   // forms the core traps to line F by design (docs/learn/HISTORY.md, section 13)
              // o = opcode, e = command word; cls = class, sz = format; imm = immediate EA.
              uint16_t o = bw16(cur.pc), e = bw16(cur.pc + 2); int cls = (e >> 13) & 7, sz = (e >> 10) & 7; bool imm = (o & 0x3f) == 0x3c;
              if ((o & 0xFFC0) == 0xF200) {   // cpGEN, coprocessor 1
                if ((cls == 2 || cls == 3) && (sz == 3 || sz == 7) && (o & 0x38)) why = "fpu: packed decimal operand (trapped to line F by design)";
                else if (cls == 2 && imm && (sz == 2 || sz == 3 || sz == 5 || sz == 7)) why = "fpu: immediate .D/.X/.P operand (trapped to line F by design)";
                else if ((cls == 4 || cls == 5) && imm && ((sz & 1) + ((sz >> 1) & 1) + ((sz >> 2) & 1)) >= 2) why = "fpu: FMOVEM.L #imm with two or three registers (trapped to line F by design)";
                else if ((cls == 6 || cls == 7) && (e & 0x0800)) why = "fpu: FMOVEM.X with a dynamic register list (trapped to line F by design)";
              }
            }
            // Trace exception number 9 as a reported exception; frame formats other than 0 and 2 are not compared.
            if (why.empty() && exc == 9 && rule("trace")) why = "trace exception (trace not in the core)";
            if (why.empty() && have_frame && fr.len < 0) why = "exception frame format other than 0 and 2";
          }
          // A test that is not live: nothing is run; the memory changes it lists are restored below.
          if (!live) {
            // not run: restore memory writes listed (harness undoes them), nothing else
          // A skipped test: count it under its reason.
          } else if (!why.empty()) { st.skip++; st.skipwhy[why]++; st.tests++; }
          else {
            // A test that runs:
            st.tests++; st.run++;
            // Memory the Amiga program prepares before each test: the user stack is copied to the ISP area (always, for RTE and stack instructions).
            // memory the Amiga program prepares: user stack copied to the ISP area (always, for RTE and stack instructions)
            std::vector<std::pair<uint32_t, uint8_t>> saved;
            uint32_t usp = tr.r[15];
            // Copy 32 bytes of the user stack to the ISP area when the stack is inside test memory.
            if (usp >= H.tmem_start && usp + 0x20 <= H.tmem_start + H.tmem_size) for (int i = 0; i < 0x20; i++) { put8(ssp + i, base[(usp + i) & MASK]); }
            // fin = the FPU state for the prologue. Write the prologue for this test.
            FpuIn fin = {tr.fp, tr.fpcr, tr.fpsr, tr.fpiar};
            put_prologue(PRO_ADDR, usp, msp, ssp, &tr.r[0], &tr.r[8], tsr, cur.pc, fpumode ? &fin : nullptr);
            // With tracing, print the starting state.
            if (trace_on) {
              printf("    usp %08x ssp %08x memory at usp:", usp, ssp); for (int i = 0; i < 16; i++) printf(" %02x", mem[(usp + i) & MASK]);
              printf("\n    D/A:"); for (int i = 0; i < 16; i++) printf(" %08x", tr.r[i]);
              printf("\n    insn:"); for (int i = 0; i < 5; i++) printf(" %04x", bw16(cur.pc + 2 * i));
              printf("\n    srcaddr %08x dstaddr %08x btarget %08x mode %d\n", cur.srcaddr, cur.dstaddr, cur.btarget, cur.bmode);
              if (cur.srcaddr != 0xffffffff) { printf("    mem@src:"); for (int i = 0; i < 8; i++) printf(" %02x", mem[(cur.srcaddr + i) & MASK]); printf("\n"); }
            }
            // Run the core until the stub.
            Snap s = run_rtl(cur.pc);
            // ---- compare the core's result with the expected state lr.
            // ---- compare
            // diff = difference text, keys = the groups that differed.
            std::string diff, keys;
            // fail(): add a difference to the text and its key to the key list once.
            auto fail = [&](const char* key, const std::string& d) { if (keys.find(key) == std::string::npos) keys += std::string(keys.empty() ? "" : "+") + key; diff += " " + d + ";"; };
            // The core did not finish: HANG.
            if (!s.done) fail("HANG", s.why);
            else {
              // g8(): a byte of the result memory: the core's stored byte or else memory.
              auto g8 = [&](uint32_t a) -> uint8_t { auto w = s.wr.find(a & MASK); return w != s.wr.end() ? w->second : mem[a & MASK]; };
              // fa = the stack pointer after the exception (the frame address); the stacked SR, PC and format word are read back from the frame.
              uint32_t fa = s.sh[15];
              uint32_t got_sr = (g8(fa) << 8) | g8(fa + 1), got_pc = (g8(fa + 2) << 24) | (g8(fa + 3) << 16) | (g8(fa + 4) << 8) | g8(fa + 5);
              uint16_t got_fmt = (uint16_t)((g8(fa + 6) << 8) | g8(fa + 7));
              char b[96];
              // D0-D7 and A0-A6, then USP (bank 0) against the expected A7 (the user stack pointer).
              for (int i = 0; i < 15; i++) if (s.sh[i] != lr.r[i]) { snprintf(b, sizeof b, "%c%d got %08x exp %08x", i < 8 ? 'D' : 'A', i & 7, s.sh[i], lr.r[i]); fail(i < 8 ? "D" : "A", b); }
              if (s.bk[0] != lr.r[15]) { snprintf(b, sizeof b, "A7(USP) got %08x exp %08x", s.bk[0], lr.r[15]); fail("A7", b); }
              // SR compared without the undefined flag bits that the data names.
              uint32_t msk = ~ignore & 0xffff;
              if (((got_sr ^ lr.sr) & msk) != 0) { snprintf(b, sizeof b, "SR got %04x exp %04x", got_sr, lr.sr & 0xffff); fail("SR", b); }
              // No exception expected: the core must have reached vector 4 (the ILLEGAL word behind the instruction), and the stacked PC must be the end of the instruction (plus 2 if the end word is the NOP).
              if (exc == 0) {
                if (s.vec != 4) { snprintf(b, sizeof b, "exception %d, expected none", s.vec); fail("EXC", b); }
                else if (got_pc != lr.pc + endextra) { snprintf(b, sizeof b, "PC got %08x exp %08x", got_pc, lr.pc + endextra); fail("PC", b); }
              // An exception expected: the vector must match, and the frame bytes must match (format 0 or 2, PC and instruction address; the first two bytes, the SR, were checked above).
              } else {
                if (s.vec != exc) { snprintf(b, sizeof b, "exception %d, expected %d", s.vec, exc); fail("EXC", b); }
                else if (have_frame && fr.len > 0) {
                  for (int i = 2; i < fr.len; i++) if (g8(fa + i) != fr.b[i]) { std::string hx; char t[8]; for (int j = 0; j < fr.len; j++) { snprintf(t, sizeof t, "%02x", g8(fa + j)); hx += t; } hx += " exp "; for (int j = 0; j < fr.len; j++) { snprintf(t, sizeof t, "%02x", j < 2 ? g8(fa + j) : fr.b[j]); hx += t; } fail("FRAME", "frame got " + hx); break; }
                }
              }
              (void)got_fmt;
              // FPU: compare the 8 data registers (3 words each), FPSR, FPCR and FPIAR.
              if (fpumode) {
                char fb[200];
                for (int i = 0; i < 8; i++) if (memcmp(s.fp[i], lr.fp[i], sizeof s.fp[i])) {
                  snprintf(fb, sizeof fb, "FP%d got %04x-%08x%08x exp %04x-%08x%08x (in %04x-%08x%08x)", i, s.fp[i][0], s.fp[i][1], s.fp[i][2], lr.fp[i][0], lr.fp[i][1], lr.fp[i][2], tr.fp[i][0], tr.fp[i][1], tr.fp[i][2]); fail("FPREG", fb); }
                if (s.fpsr != lr.fpsr) { snprintf(fb, sizeof fb, "FPSR got %08x exp %08x (in %08x)", s.fpsr, lr.fpsr, tr.fpsr); fail("FPSR", fb); }
                if (s.fpcr != lr.fpcr) { snprintf(fb, sizeof fb, "FPCR got %08x exp %08x (in %08x)", s.fpcr, lr.fpcr, tr.fpcr); fail("FPCR", fb); }
                if (s.fpiar != lr.fpiar) { snprintf(fb, sizeof fb, "FPIAR got %08x exp %08x", s.fpiar, lr.fpiar); fail("FPIAR", fb); }
              }
              // Listed memory writes: the core must have stored the same value. listed = the addresses the data lists.
              // listed memory writes
              std::set<uint32_t> listed;
              for (auto& e : expw) {
                uint32_t a = e.first, nv = e.second.first; int sz = e.second.second; uint32_t gv = 0;
                for (int i = 0; i < (1 << sz); i++) { gv = (gv << 8) | g8(a + i); listed.insert((a + i) & MASK); }
                if (gv != nv) { snprintf(b, sizeof b, "mem %08x got %0*x exp %0*x", a, 2 << sz, gv, 2 << sz, nv); fail("MEM", b); }
              }
              // FPU data: the generator's memory differs from the Amiga program's (initial writes persist in the generator and are undone per group), and the Amiga program compares only the listed writes, so unlisted stores cannot be judged here.
              // FPU data: the generator's memory differs from the Amiga program's (init writes persist in the generator, restoreahist undoes them per group), and the
              // Amiga program compares only the listed writes: unlisted stores cannot be judged, so they are not checked.
              // Integer data: every store of the core that the data does not list must not change memory, except the exception frame, the stacks and the prologue.
              if (!fpumode) for (auto& w : s.wr) {   // stores the data does not list (value changes only), except the exception frame and the prologue area
                uint32_t a = w.first; if (listed.count(a)) continue;
                if (a >= fa && a < fa + 32) continue;
                if (a >= ssp - 0x40 && a < msp) continue;
                if (a >= PRO_ADDR && a < PRO_ADDR + 0x1000) continue;
                if (w.second != base[a]) { snprintf(b, sizeof b, "extra store %06x=%02x (was %02x)", a, w.second, base[a]); fail("XMEM", b); break; }
              }
            }
            // Classification: a mismatch only in the flags of DIV, CHK and CHK2 is undefined behavior (WinUAE has no 68020 silicon values for them); count it as undef.
            // classification of undefined-flag behaviour (WinUAE does not know the 68020 DIV and CHK results either)
            bool undef = false;
            if (!keys.empty() && keys == "SR" && (name.rfind("DIV", 0) == 0 || name.rfind("CHK", 0) == 0) && rule("undef_flags")) undef = true;
            // No differences: pass. Undefined flags: count apart. Anything else is a failure, counted by cause (the key list; for ILLEGAL the opcode group is added).
            if (keys.empty()) st.pass++;
            else if (undef) { st.undef++; st.cause["undef flags (DIV/CHK)"]++; }
            else {
              st.fail++; { std::string ck = keys; if (name == "ILLEGAL") { char ob[24]; snprintf(ob, sizeof ob, " op %04x..", opw & 0xFFC0); ck += ob; } st.cause[ck]++; }
              // FPU failure details for the log and the printout: instruction words, source operand in memory, FPU control values and the source register for store forms.
              if (fpumode) {   // opcode words and the source operand in memory, for the failure log (CPUTEST_FAILLOG=file) and the shown failures
                char ib[300]; int n = snprintf(ib, sizeof ib, " insn"); for (int i = 0; i < 6; i++) n += snprintf(ib + n, sizeof ib - n, " %04x", bw16(cur.pc + 2 * i));
                if (cur.srcaddr != 0xffffffff) { n += snprintf(ib + n, sizeof ib - n, " src@%08x:", cur.srcaddr); for (int i = 0; i < 12; i++) n += snprintf(ib + n, sizeof ib - n, "%02x", base[(cur.srcaddr + i) & MASK]); }
                { uint16_t e2 = bw16(cur.pc + 2); n += snprintf(ib + n, sizeof ib - n, " in: fpcr %08x fpsr %08x", tr.fpcr, tr.fpsr);
                  if (((e2 >> 13) & 7) == 3) { int r = (e2 >> 7) & 7; n += snprintf(ib + n, sizeof ib - n, " FP%d %04x-%08x%08x", r, tr.fp[r][0], tr.fp[r][1], tr.fp[r][2]); } }   // store form: the source register
                diff += ib;
                // Memory failures: all stores of the core and the listed expected writes.
                if (keys.find("MEM") != std::string::npos) {   // all stores of the core outside the exception frame, and the listed expected writes
                  std::string w = " core-stores:"; uint32_t fa2 = s.sh[15]; char t[32];
                  for (auto& q : s.wr) { if (q.first >= fa2 && q.first < fa2 + 32) continue; if (q.first >= ssp - 0x40 && q.first < msp) continue; if (q.first >= PRO_ADDR && q.first < PRO_ADDR + 0x1000) continue; snprintf(t, sizeof t, " %06x=%02x", q.first, q.second); w += t; }
                  w += " expected-writes:"; for (auto& e3 : expw) { snprintf(t, sizeof t, " %06x/%d=%x", e3.first, 1 << e3.second.second, e3.second.first); w += t; }
                  diff += w;
                }
                // CPUTEST_FAILLOG=prefix writes one file per folder and shard with a line per failing test.
                static FILE* flog = getenv("CPUTEST_FAILLOG") ? fopen((std::string(getenv("CPUTEST_FAILLOG")) + "." + name + "." + std::to_string(offset)).c_str(), "w") : nullptr;   // one file per folder and shard
                if (flog) fprintf(flog, "%s g%ld ccr%d %s:%s\n", name.c_str(), gidx, ccr, keys.c_str(), diff.c_str());
              }
              // The first failures are printed in full.
              if (shown < show) { shown++; char hdr[160]; snprintf(hdr, sizeof hdr, "  FAIL %s group %ld op %04x pc %08x sr %04x ccr %d end %d exc %d", name.c_str(), gidx, opw, cur.pc, tsr, ccr, endextra, exc); printf("%s:%s\n", hdr, diff.c_str()); }
            }
            // Restore the memory this test changed.
            for (uint32_t a : touched) mem[a] = base[a];
            touched.clear();
          }
          // After the test: restore the opcode bytes the data lists, and undo the stores.
          // harness side effects after the test: opcode memory restore items
          for (auto& o : opc_restore) bput8(o.first, o.second);
          for (uint32_t a : touched) mem[a] = base[a];
          touched.clear();
          if (only && gidx == only && live) { /* single group trace requested */ }
        }
        // End of the rounds: the end marker CT_END, otherwise the next SR round code.
        if (*p == CT_END) { p++; break; }
        extraccr = *p++;
      }
      // ---- group end: undo the initial memory writes, restore the end opcodes and the branch target, as the Amiga program does.
      // ---- group end: undo the initial memory history, restore the end opcodes and branch target (as the Amiga program does)
      // Put the original end words back (the same is written a second time on the next line with the original constants).
      { uint32_t e = cur.endpc; bput8(e, NOP_OPCODE >> 8); bput8(e + 1, NOP_OPCODE & 0xff); bput8(e + 2, ILLG_OPCODE >> 8); bput8(e + 3, ILLG_OPCODE & 0xff); }
      { uint32_t e = cur.endpc; uint32_t orig = ((uint32_t)NOP_OPCODE << 16) | ILLG_OPCODE; bput8(e, orig >> 24); bput8(e + 1, orig >> 16); bput8(e + 2, orig >> 8); bput8(e + 3, orig); }
      // Undo the group's initial memory writes in reverse order.
      for (int i = (int)hist.size() - 1; i >= 0; i--) tomem(hist[i].addr, hist[i].oldv, hist[i].size, nullptr, 0);   // restoreahist
      // Stop after the group limit, or after the selected group.
      if (maxg && st.groups >= maxg && !only) { abort_all = true; break; }
      if (only && gidx >= only) { abort_all = true; break; }
    }
  }
  // Summary line for the folder, then one line per skip reason and one per failure cause.
  printf("%-10s groups %6ld tests %7ld run %7ld pass %7ld fail %6ld undef %5ld skip %6ld\n", name.c_str(), st.groups, st.tests, st.run, st.pass, st.fail, st.undef, st.skip);
  for (auto& s : st.skipwhy) printf("  skip %6ld  %s\n", s.second, s.first.c_str());
  for (auto& s : st.cause) printf("  cause %6ld  %s\n", s.second, s.first.c_str());
  return 0;
}
