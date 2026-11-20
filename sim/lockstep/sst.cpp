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
// SingleStepTests runner: Tom Harte's 680x0 JSON tests (the 68000 set) run against the RTL core in Verilator.
//
// What the tests are. The SingleStepTests project (github.com/SingleStepTests/680x0) published about one million tests of single 68000 instructions,
// made by running each instruction on a cycle exact 68000 model. A test lists the complete starting state (D0-D7, A0-A6, user and supervisor stack
// pointer, status register, program counter, two prefetch words and a list of memory bytes), then one instruction, then the complete final state. The data is a
// JSON file per instruction and size (ADD.b, MOVE.l and so on, 124 or 125 files, fetched by tools/fetch_tests.sh), with the test cases as an array in it.
// There is no reference CPU in this runner: the final state in the data is the answer.
//
// How a test runs, with no change to the core. The core starts from reset and fetches its first instruction from address 0, where the runner puts a branch to
// a small generated prologue. The prologue loads the starting state with ordinary instructions (MOVE.L #imm,Dn for each data register, MOVEA.L #imm,An for each address
// register, MOVE A6,USP, MOVE #sr,SR) and then branches to the test's PC. The runner watches the core's retire port (the signals that report every retired
// instruction, the same port the lockstep bench uses). The first retire at the test PC is the instruction under test, the registers are copied at that moment, and the PC
// of the next retire is the PC after the instruction. Compared are D0-D7, A0-A6, USP, SSP, SR, PC and every stored byte. Exception tests (the vector for the exception is in
// the starting memory) compare D0-D7, A0-A6, USP, SR and PC only, because the 68000 exception frame is a different size from the 68020 frame.
//
// What is skipped and why. The tests are 68000 tests and the core is a 68020, so a few things differ on purpose: address errors, the trace bit, the 68000 frame, indexed
// addressing with the extension bits the 68000 ignores, and some undefined flag cases. Every skip is counted with its reason, the reasons are the skip file
// tools/sst_skip.txt (entries and a description of each built-in rule) and the rules in skip_reason() below. The runs of the release state pass 745,646 tests with 3 known
// failures (two ASL.b entries with wrong data, one DIVU divide by zero flag case where the 68000 and 68020 differ), 7,162 undefined results (BCD flags) and 6,500
// cases where the data disagrees with Musashi, WinUAE and the core alike.
//
// usage: sst SKIPFILE NAME [max_tests] [show] [only] < tests.json     (tools/sst.sh feeds each gzip file through gzip into standard input)
//   SKIPFILE = tools/sst_skip.txt, NAME = the data file name without .json.gz (used in the skip rules), max_tests = stop after that many tests (0 = all),
//   show = how many failures to print in full (default 3), only = run only the test with this number and print a retire trace of it.
#include "Vp68_core.h"
// Verilator model of the core: its header, generated from rtl/core.
#include "verilated.h"
// Verilator's own library header.
#include <cstdio>
// Standard C and C++ headers the runner uses.
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <algorithm>

// Verilator calls this function to get the simulated time; the runner does not use time, so it returns 0.
double sc_time_stamp() { return 0; }

// ---------------------------------------------------------------- A minimal JSON reader
// The test files are plain JSON, and a small parser is enough: objects, arrays, strings without escapes and integers. It is written here to avoid a library; it also
// treats commas as white space, so the separators of objects and arrays need no special handling.
// J = one JSON value. t = kind ('o' object, 'a' array, 's' string, 'n' number), n = number, s = string, a = array elements, o = object members in file order.
struct J {
  char t = 0;                                   // 'o' 'a' 's' 'n'
  int64_t n = 0; std::string s;
  std::vector<J> a; std::vector<std::pair<std::string, J>> o;
  // get(): find the member named k of an object (linear search, objects are small); null if missing.
  const J* get(const char* k) const { for (auto& p : o) if (p.first == k) return &p.second; return nullptr; }
};
// jp = read position and je = end of the text; the whole file is read into memory first.
static const char *jp, *je;
// ws(): skip spaces, newlines, tabs and commas.
static void ws() { while (jp < je && (*jp == ' ' || *jp == '\n' || *jp == '\r' || *jp == '\t' || *jp == ',')) jp++; }
// jparse(): read one value at jp into v; false at the end or on a syntax error.
static bool jparse(J& v) {
  ws(); if (jp >= je) return false;
  char c = *jp;
  // An object: read key (a string), a colon, a value, until the closing brace.
  if (c == '{') {
    v.t = 'o'; jp++;
    for (;;) {
      ws(); if (jp < je && *jp == '}') { jp++; return true; }
      J k; if (!jparse(k) || k.t != 's') return false;
      ws(); if (jp >= je || *jp != ':') return false; jp++;
      J x; if (!jparse(x)) return false;
      v.o.emplace_back(std::move(k.s), std::move(x));
    }
  }
  // An array: read values until the closing bracket.
  if (c == '[') {
    v.t = 'a'; jp++;
    for (;;) {
      ws(); if (jp < je && *jp == ']') { jp++; return true; }
      J x; if (!jparse(x)) return false;
      v.a.push_back(std::move(x));
    }
  }
  // A string: everything up to the next quote (the test data has no escape sequences).
  if (c == '"') { v.t = 's'; jp++; while (jp < je && *jp != '"') v.s += *jp++; jp++; return true; }
  // Anything else is a number (base 10, 64 bit so that all 32 bit addresses fit).
  v.t = 'n'; char* e; v.n = strtoll(jp, &e, 10); if (e == jp) return false; jp = e; return true;
}

// ---------------------------------------------------------------- Test data
// St = a processor state: data registers, address registers A0-A6, the two stack pointers, SR, PC, the two prefetch words and the memory bytes (address, value) the test lists.
struct St { uint32_t d[8], a[7], usp, ssp, sr, pc; uint16_t pf[2]; std::vector<std::pair<uint32_t, uint8_t>> ram; };
// Tx = one bus transaction of the original 68000 model (read or write, function code, address, size). It is parsed but not used: this runner checks results, not bus timing.
struct Tx { char type; int fc; uint32_t addr; char size; };
// Test = name, starting state, final state and the bus transactions.
struct Test { std::string name; St in, fin; std::vector<Tx> tx; };
// load_state(): fill an St from the JSON object "initial" or "final".
static bool load_state(const J* j, St& s) {
  if (!j) return false;
  // The member names of the registers in the data.
  static const char* dn[8] = {"d0","d1","d2","d3","d4","d5","d6","d7"}; static const char* an[7] = {"a0","a1","a2","a3","a4","a5","a6"};
  // Data registers d0 to d7.
  for (int i = 0; i < 8; i++) { auto x = j->get(dn[i]); if (!x) return false; s.d[i] = (uint32_t)x->n; }
  // Address registers a0 to a6 (A7 is the stack pointer, given as usp and ssp).
  for (int i = 0; i < 7; i++) { auto x = j->get(an[i]); if (!x) return false; s.a[i] = (uint32_t)x->n; }
  // The other members.
  auto u = j->get("usp"), sp = j->get("ssp"), sr = j->get("sr"), pc = j->get("pc"), pf = j->get("prefetch"), r = j->get("ram");
  // A state without these members is not a valid test.
  if (!u || !sp || !sr || !pc || !r) return false;
  // Copy the values.
  s.usp = (uint32_t)u->n; s.ssp = (uint32_t)sp->n; s.sr = (uint32_t)sr->n; s.pc = (uint32_t)pc->n;
  // The prefetch words are only in the starting state.
  s.pf[0] = s.pf[1] = 0;
  // Copy up to two prefetch words.
  if (pf) for (size_t i = 0; i < pf->a.size() && i < 2; i++) s.pf[i] = (uint16_t)pf->a[i].n;
  // The memory list: each entry is a pair [address, byte].
  for (auto& e : r->a) if (e.a.size() == 2) s.ram.emplace_back((uint32_t)e.a[0].n, (uint8_t)e.a[1].n);
  return true;
}

// ---------------------------------------------------------------- Skip rules
// A test can be skipped by a line of the skip file, or by a built-in rule in skip_reason(). A skipped test is counted under its reason and never run.
// The core's memory model is 16 MB (24 address bits); addresses are masked to it.
static const uint32_t MEMSZ = 1u << 24, MASK = MEMSZ - 1;
// OpSkip = a skip rule of the form: opcode AND mask = value, with the reason text.
struct OpSkip { uint32_t mask, val; std::string why; };
// file_skip = reasons by data file name (skip whole files).
static std::map<std::string, std::string> file_skip;
static std::set<std::string> rule_off;   // built-in rules switched off by `rule NAME off` lines of the skip file
// rule(): true unless the skip file switched the built-in rule off.
static bool rule(const char* n) { return !rule_off.count(n); }
// op_skip = the opcode rules from the skip file.
static std::vector<OpSkip> op_skip;
// load_skips(): read the skip file. A line is "file NAME reason", "op MASK VALUE reason" or "rule NAME off"; lines starting with # are comments.
static void load_skips(const char* path) {
  FILE* f = fopen(path, "r"); if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
  char line[512];
  // Read one line at a time.
  while (fgets(line, sizeof line, f)) {
    // k = keyword, a = first argument, b = second argument, n = characters used by the first two words (the reason text starts after them).
    char k[16], a[64], b[64]; int n = 0;
    // Comment lines and lines with fewer than two words are ignored.
    if (line[0] == '#' || sscanf(line, "%15s %63s%n", k, a, &n) < 2) continue;
    // "rule NAME off": remember that this built-in rule is switched off.
    if (!strcmp(k, "rule")) { char o[16] = ""; sscanf(line, "%15s %63s %15s", k, a, o); if (!strcmp(o, "off")) rule_off.insert(a); }
    // "file NAME reason": skip a whole data file; the rest of the line (without leading spaces and the line end) is the reason.
    else if (!strcmp(k, "file")) { std::string w = line + n; while (!w.empty() && (w[0] == ' ')) w.erase(0, 1); while (!w.empty() && (w.back() == '\n' || w.back() == '\r')) w.pop_back(); file_skip[a] = w; }
    // "op MASK VALUE reason": skip by opcode; mask and value are read with the C rules (so 0x prefix works).
    else if (!strcmp(k, "op")) {
      int m = 0; if (sscanf(line, "%15s %63s %63s%n", k, a, b, &m) < 3) continue;
      std::string w = line + m; while (!w.empty() && (w[0] == ' ')) w.erase(0, 1); while (!w.empty() && (w.back() == '\n' || w.back() == '\r')) w.pop_back();
      op_skip.push_back({(uint32_t)strtoul(a, 0, 0), (uint32_t)strtoul(b, 0, 0), w});
    }
  }
  fclose(f);
}

// Img = the bytes of the test as the core will see them: the two prefetch words at pc and pc + 2, then the memory list. Used to look at the instruction (opcode, extension words) for the skip rules.
// bytes of the test's instruction as the core will see them (prefetch words at pc, pc+2, then ram)
struct Img {
  std::map<uint32_t, uint8_t> m;
  // w(): the 16-bit word at address a (big endian); bytes that are not listed read as 0.
  uint16_t w(uint32_t a) const { auto x = m.find(a & MASK), y = m.find((a + 1) & MASK); return (uint16_t)(((x == m.end() ? 0 : x->second) << 8) | (y == m.end() ? 0 : y->second)); }
};
// ea_ext_words(): how many extension words an effective address (mode, register) has, and whether it is an indexed form (index = true). A size of 2 means long.
static int ea_ext_words(int mode, int reg, int size /*0 b 1 w 2 l*/, bool* index) {
  // Mode 5, (d16,An): one displacement word.
  if (mode == 5) return 1;
  // Mode 6, (d8,An,Xn): one word, indexed.
  if (mode == 6) { *index = true; return 1; }
  // Mode 7 has five forms by register: 0 absolute word and 2 (d16,PC) have one word; 1 absolute long has two; 3 (d8,PC,Xn) has one word and is indexed; 4 immediate has two words for a long and one otherwise.
  if (mode == 7) { if (reg == 0 || reg == 2) return 1; if (reg == 1) return 2; if (reg == 3) { *index = true; return 1; } if (reg == 4) return size == 2 ? 2 : 1; }
  // Modes 0 to 4 have no extension words.
  return 0;
}
// does an indexed EA of this instruction have bits 10-8 of its extension word set (68000 ignores them, 68020 scales / full format)?
// index_bits(): for the opcode at pc, find the effective addresses that are indexed and test bits 10 to 8 of their extension word. On the 68000 those bits are ignored; on the 68020 bits 10-9 are the index scale and bit 8 selects the full extension word format, so such a test gives different results on the two CPUs.
static bool index_bits(const Img& im, uint32_t pc) {
  uint16_t op = im.w(pc); int g = op >> 12; int at = 1;   // word index of the next extension word
  // chk(): examine one effective address (mode, register, operand size): count its extension words, test the bits when it is indexed, and move the word position on.
  auto chk = [&](int mode, int reg, int sz) {
    // n = number of extension words, idx = indexed or not.
    bool idx = false; int n = ea_ext_words(mode, reg, sz, &idx);
    // bad = indexed and bits 10-8 of the first extension word are set (mask 0x0700).
    bool bad = idx && (im.w(pc + 2 * at) & 0x0700) != 0;
    // Advance to the next extension word position.
    at += n; return bad;
  };
  // Groups 1, 2 and 3 are MOVE (byte, long, word): source EA in bits 5-0, destination EA with mode in bits 8-6 and register in bits 11-9.
  if (g == 1 || g == 2 || g == 3) {
    // The size field of MOVE is coded differently from other instructions: group 1 is byte, 3 is word, 2 is long.
    int sz = g == 1 ? 0 : g == 3 ? 1 : 2;
    // Check the source EA first (it comes first in the instruction stream), ...
    if (chk((op >> 3) & 7, op & 7, sz)) return true;
    // ... then the destination EA.
    return chk((op >> 6) & 7, (op >> 9) & 7, sz);
  }
  // For all other groups the size is in bits 7-6 (3 means a word-sized ADDA/SUBA/CMPA or memory shift).
  int sz = (op >> 6) & 3; if (sz == 3) sz = 1;
  switch (g) {
    // Group 0: immediate operations, bit operations, MOVEP.
    case 0:
      if ((op & 0x0138) == 0x0108) return false;                       // MOVEP
      if ((op & 0x0F00) == 0x0800) { at += 1; return chk((op >> 3) & 7, op & 7, 0); }                 // static bit op
      if (op & 0x0100) return chk((op >> 3) & 7, op & 7, 0);           // dynamic bit op
      if ((op & 0x003F) == 0x003C) return false;                        // to CCR / SR
      // For an immediate operation the immediate data (1 or 2 words) comes before the EA's extension words.
      at += sz == 2 ? 2 : 1; return chk((op >> 3) & 7, op & 7, sz);    // immediate op
    // Group 4: miscellaneous. MOVEM carries a register mask word before the EA's extension words.
    case 4:
      if ((op & 0xFF80) == 0x4880) { at += 1; return chk((op >> 3) & 7, op & 7, 2); }               // MOVEM to memory
      if ((op & 0xFF80) == 0x4C80) { at += 1; return chk((op >> 3) & 7, op & 7, 2); }               // MOVEM from memory
      // Instructions such as TRAP, LINK, UNLK, MOVE USP and RTS have no effective address with extension words.
      if ((op & 0xFFF0) == 0x4E70 || (op & 0xFFF8) == 0x4E50 || (op & 0xFFF8) == 0x4E58 || (op & 0xFFF0) == 0x4E40) return false;
      // Other group 4 instructions (CLR, NEG, NOT, TST, JMP, JSR, LEA, PEA and so on) use their EA field.
      return chk((op >> 3) & 7, op & 7, sz);
    // Groups 5, 8, 9, B, C, D and E: ADDQ/SUBQ/Scc/DBcc, OR/DIV/SBCD, SUB/SUBA/SUBX, CMP/EOR/CMPM, AND/MUL/ABCD/EXG, ADD/ADDA/ADDX and the shifts.
    case 5: case 8: case 9: case 0xB: case 0xC: case 0xD: case 0xE:
      if (g == 0xE && ((op >> 6) & 3) != 3) return false;              // register shifts
      // Register direct EAs (modes 0 and 1) have no extension words; the shortcut does not apply to group 5, which goes through chk() like the others.
      if (((op >> 3) & 7) < 2 && g != 5) return false;
      // The operand size for the check; a long or word ADDA/SUBA/CMPA (opmode 3 or 7) takes the size from bit 8.
      return chk((op >> 3) & 7, op & 7, (g == 0xD || g == 9 || g == 0xB) && ((op >> 6) & 3) == 3 ? ((op >> 8) & 1 ? 2 : 1) : sz);
    // Other groups (6 branches, 7 MOVEQ, A and F lines) have no EA to check.
    default: return false;
  }
}

// ---------------------------------------------------------------- Bench
// mem = the 16 MB memory the core sees (all zero between tests).
static std::vector<uint8_t> mem(MEMSZ, 0);
// The core starts executing at address 0.
static const uint32_t RESET_PC = 0;
// Four places for the prologue near the top of memory; the first one that does not overlap the test's memory is used.
static const uint32_t PRO_CAND[] = {0xFF0000, 0xFE0000, 0xFD0000, 0xFC0000};

// Stats = counters of the run: executed, passed, failed, no final PC seen, undefined results, disagreements with the data, and counts per skip reason and failure cause.
struct Stats { long run = 0, pass = 0, fail = 0, nopc = 0, undef = 0, disagree = 0; std::map<std::string, long> skip; std::map<std::string, long> cause; };

// skip_reason(): decide whether a test is skipped. Returns the reason, or an empty string if the test should run, and tells the caller whether it is an exception test (*exc) and which prologue area to use (*pro).
static std::string skip_reason(const Test& t, const Img& im, bool* exc, uint32_t* pro, const std::string& file) {
  // Not an exception test unless the vector check below finds one.
  *exc = false;
  // Skip rule by file name.
  auto f = file_skip.find(file); if (f != file_skip.end()) return "file: " + f->second;
  // op = the first opcode word.
  uint16_t op = im.w(t.in.pc);
  // Skip rules by opcode.
  for (auto& o : op_skip) if ((op & o.mask) == o.val) return "op: " + o.why;
  // A test with the trace bit set in SR: the core has no trace mode.
  if ((t.in.sr & 0x8000) && rule("trace")) return "trace bit set (trace not in the core)";
  // an exception test has the vector in its memory and ends at the vector's address (a low operand address alone is no exception)
  // Collect the starting memory below address 0x100 (the vector table).
  std::map<uint32_t, uint8_t> lo;
  // lo = bytes of the vector table that the test lists.
  for (auto& r : t.in.ram) if (r.first < 0x100) lo[r.first] = r.second;
  // Vectors 2 to 63: ...
  for (uint32_t n = 2; n < 64; n++) {
    // v = the vector's address as listed; all = all four bytes are listed.
    uint32_t v = 0; bool all = true;
    // Read the four bytes.
    for (int i = 0; i < 4; i++) { auto x = lo.find(4 * n + i); if (x == lo.end()) all = false; else v = (v << 8) | x->second; }
    // A vector whose address equals the test's final PC marks an exception test (the instruction ends at the handler).
    if (!all || v != t.fin.pc) continue;
    // Vectors 2 and 3 are bus error and address error: the 68000 frame for those is 14 bytes and the 68020 does not fault on odd addresses, so skip.
    if (n == 2 || n == 3) return "address/bus error test (68000 frame)";
    // Any other vector: it is an exception test (privilege violation, divide by zero, TRAP and so on).
    *exc = true;
  }
  // Bcc, BRA and BSR with the byte displacement 0xFF: the 68020 reads a long displacement after the opcode, the 68000 treats FF as a displacement of -1.
  if ((op & 0xF0FF) == 0x60FF && rule("bcc_ff")) return "Bcc/BRA/BSR with byte displacement FF (68020: 32-bit displacement)";
  // MOVE from SR in user mode: privileged on the 68020, allowed on the 68000.
  if ((op & 0xFFC0) == 0x40C0 && !(t.in.sr & 0x2000) && rule("move_from_sr")) return "MOVE from SR in user mode (privileged on the 68020)";
  // MOVEM registers to memory with pre-decrement: the 68000 stores the old value of the base register when it is in the list, the 68020 the decremented one.
  if ((op & 0xFB80) == 0x4880 && ((op >> 3) & 7) == 4) {            // MOVEM to -(An): base register in the mask
    // The register mask word follows the opcode.
    uint16_t mask = im.w(t.in.pc + 2);
    // In the register mask of the pre-decrement form bit 0 is A7 and bit 7 is A0, so the base register An has the bit 7 - n.
    if (mask & (1u << (7 - (op & 7)))) return "MOVEM -(An) with the base register in the mask (68000 stores the old value)";
  }
  // Indexed effective addresses with extension bits set (see index_bits()).
  if (rule("index_bits") && index_bits(im, t.in.pc)) return "indexed EA with extension word bits 10-8 set (68000 ignores them)";
  // The bench needs addresses 0 to 5 for its reset stub; a test that lists memory there cannot run (exception tests are different, they list vector bytes below 0x100 and that is fine).
  bool lowm = false;
  // lowm = the test lists memory in the first six bytes.
  for (auto& r : t.in.ram) if (r.first < 6 && !*exc) lowm = true;
  // Skip, with the reason.
  if (lowm) return "bench: memory at 0-5 (reset stub)";
  // Find a prologue area that the test's memory does not touch (a 1 KB window at each candidate).
  for (uint32_t c : PRO_CAND) {
    // hit = one of the test's memory bytes lies in the window.
    bool hit = false;
    // Check all test memory bytes.
    for (auto& r : t.in.ram) if (r.first >= c && r.first < c + 0x400) hit = true;
    // Free area found: use it. The empty string means "run this test".
    if (!hit) { *pro = c; return ""; }
  }
  // None of the four is free (practically never).
  return "bench: no free prologue area";
}

// bcd_bad(): ABCD, SBCD and NBCD with an operand byte that is not valid BCD (a nibble above 9) give an undefined result: the data has the 68000's result, the core follows WinUAE's 68020. The function reads the operand byte or bytes the way the instruction addresses them, so it can tell.
// ABCD/SBCD/NBCD: an operand byte that is not valid BCD gives an undefined result on the 68000 (the data) and the 68020 (RTL follows WinUAE's 68020)
static bool bcd_bad(const Test& t, const Img& im, const std::string& file) {
  // op = the opcode word (the prefetch word).
  uint16_t op = t.in.pf[0];
  // a(): value of address register r in the starting state (A7 is the SSP or USP depending on the S bit).
  auto a = [&](int r) -> uint32_t { return r == 7 ? ((t.in.sr & 0x2000) ? t.in.ssp : t.in.usp) : t.in.a[r]; };
  // byt(): memory byte at an address.
  auto byt = [&](uint32_t ad) -> uint8_t { auto x = im.m.find(ad & MASK); return x == im.m.end() ? 0 : x->second; };
  // bad(): a byte whose low or high nibble is above 9.
  auto bad = [](uint8_t b) { return (b & 15) > 9 || (b >> 4) > 9; };
  // ABCD and SBCD: two operands.
  if (file.rfind("ABCD", 0) == 0 || file.rfind("SBCD", 0) == 0) {
    // x and y = the register fields (bits 11-9 and 2-0).
    int x = (op >> 9) & 7, y = op & 7;
    // Register form (bit 3 clear): both operands are the low bytes of the data registers.
    if (!(op & 8)) return bad(t.in.d[y] & 255) || bad(t.in.d[x] & 255);
    // Memory form: both addresses are pre-decremented by one byte (two for A7), and the source address register first.
    uint32_t ay = a(y) - (y == 7 ? 2 : 1), ax = (x == y ? ay : a(x)) - (x == 7 ? 2 : 1);
    // Test both bytes.
    return bad(byt(ay)) || bad(byt(ax));
  }
  // NBCD: one operand through the EA fields (mode m, register r).
  int m = (op >> 3) & 7, r = op & 7;
  // Data register: its low byte.
  if (m == 0) return bad(t.in.d[r] & 255);
  // (An) and (An)+: the byte at An.
  if (m == 2 || m == 3) return bad(byt(a(r)));
  // -(An): the byte at An minus 1 (2 for A7).
  if (m == 4) return bad(byt(a(r) - (r == 7 ? 2 : 1)));
  // (d16,An): displacement from the extension word.
  if (m == 5) return bad(byt(a(r) + (int16_t)im.w(t.in.pc + 2)));
  // (d8,An,Xn): base register, 8-bit displacement and index register (word or long by bit 11, taken from the data or address register by bit 15).
  if (m == 6) {
    uint16_t x = im.w(t.in.pc + 2); int xr = (x >> 12) & 7;
    // Index register value (sign extended from word when bit 11 is clear).
    uint32_t xv = (x & 0x8000) ? (xr == 7 ? ((t.in.sr & 0x2000) ? t.in.ssp : t.in.usp) : t.in.a[xr]) : t.in.d[xr];
    if (!(x & 0x0800)) xv = (uint32_t)(int32_t)(int16_t)xv;
    // Address = base + displacement + index.
    return bad(byt(a(r) + (int8_t)(x & 0xff) + xv));
  }
  // Absolute word.
  if (m == 7 && r == 0) return bad(byt((uint32_t)(int32_t)(int16_t)im.w(t.in.pc + 2)));
  // Absolute long.
  if (m == 7 && r == 1) return bad(byt(((uint32_t)im.w(t.in.pc + 2) << 16) | im.w(t.in.pc + 4)));
  // Other forms cannot occur for NBCD.
  return false;
}

// touched = the memory addresses a test changed, so that they can be cleared after the test (the 16 MB array is not cleared as a whole).
static std::vector<uint32_t> touched;
// trace = 1 when only one test was selected: print the retire records.
static int trace = 0;
// put8, put16, put32: write bytes, words and long words into the core's memory, big endian, and record the addresses in touched.
static void put8(uint32_t a, uint8_t v) { a &= MASK; mem[a] = v; touched.push_back(a); }
static void put16(uint32_t a, uint16_t v) { put8(a, v >> 8); put8(a + 1, v & 0xff); }
static void put32(uint32_t a, uint32_t v) { put16(a, v >> 16); put16(a + 2, v & 0xffff); }

// run_test(): run one test on the core and compare. Returns "" for a pass, "keys|text" for a failure (keys = the groups of registers that differed, text = the details), or a string that starts with SKIP:, UNDEF: or DISAGREE: for the classes of difference that the 68000 data cannot settle.
static std::string run_test(const Test& t, bool exc, uint32_t pro, bool* nopc, const std::string& file, const Img& im) {
  // No final PC seen unless the second retire happens.
  *nopc = false;
  // Memory contents before the run: the test's bytes, the two prefetch words at the test PC, the prologue and the reset stub.
  // memory: test bytes, prefetch words, reset stub, prologue
  // The test's memory.
  for (auto& r : t.in.ram) put8(r.first, r.second);
  // The prefetch words are the first two words of the instruction.
  put16(t.in.pc, t.in.pf[0]); put16(t.in.pc + 2, t.in.pf[1]);
  // c = prologue code words; l2 = emit a 6 byte instruction (opcode, 32 bit immediate).
  std::vector<uint16_t> c;
  auto l2 = [&](uint16_t a, uint32_t v) { c.push_back(a); c.push_back((uint16_t)(v >> 16)); c.push_back((uint16_t)v); };
  // MOVEA.L #usp,A6 and MOVE A6,USP: set the user stack pointer.
  l2(0x2C7C, t.in.usp); c.push_back(0x4E66);                                  // MOVEA.L #usp,A6; MOVE A6,USP
  // MOVEA.L #ssp,A7: set the supervisor stack pointer (the core runs in supervisor mode at reset, with M = 0, so this is the ISP).
  l2(0x2E7C, t.in.ssp);                                                       // MOVEA.L #ssp,A7 (supervisor, M = 0: ISP)
  // MOVE.L #imm,Dn for D0-D7 (0x203C is MOVE.L #imm,D0; each register adds 0x200 for the destination register field).
  for (int i = 0; i < 8; i++) l2((uint16_t)(0x203C | (i << 9)), t.in.d[i]);   // MOVE.L #imm,Dn
  // MOVEA.L #imm,An for A0-A6 (0x207C and the destination register field).
  for (int i = 0; i < 7; i++) l2((uint16_t)(0x207C | (i << 9)), t.in.a[i]);   // MOVEA.L #imm,An
  // MOVE #sr,SR as the last instruction: it also swaps A7 to the user stack pointer when the test starts in user mode.
  c.push_back(0x46FC); c.push_back((uint16_t)t.in.sr);                        // MOVE #sr,SR (swaps A7 if user)
  // Address of the final BRA.L, needed to compute its displacement.
  uint32_t bra_at = pro + 2 * (uint32_t)c.size();
  // BRA.L to the test PC: 0x60FF and a 32-bit displacement relative to the address after the opcode word.
  c.push_back(0x60FF); { uint32_t d = t.in.pc - (bra_at + 2); c.push_back((uint16_t)(d >> 16)); c.push_back((uint16_t)d); }
  // Write the prologue into memory.
  for (size_t i = 0; i < c.size(); i++) put16(pro + 2 * (uint32_t)i, c[i]);
  put16(RESET_PC, 0x60FF);                                                     // reset stub: BRA.L to the prologue
  // The reset stub: BRA.L from address 0 to the prologue.
  { uint32_t d = pro - (RESET_PC + 2); put16(RESET_PC + 2, d >> 16); put16(RESET_PC + 4, d & 0xffff); }

  // Create the core model and prepare the state the run needs.
  auto ctx = std::make_unique<VerilatedContext>();
  auto top = std::make_unique<Vp68_core>(ctx.get());
  // sh = shadow copy of the registers D0-D7 and A0-A7 (built from the retire records), bk = the three stack pointer banks USP, ISP, MSP.
  uint32_t sh[16] = {0}, bk[3] = {0};
  // wr = the bytes the core stored while the instruction ran.
  std::map<uint32_t, uint8_t> wr;
  // The memory port: one transaction at a time and a fixed latency, 2 clocks to each read beat and to the completion of a write; a write changes memory when it completes.
  // unc_map 0 makes all addresses cacheable, but CACR stays 0 so the caches are off and every access is a line read.
  // Clock low, reset active, no memory answers, no DMA snoop, no interrupts.
  top->clk = 0; top->rst = 1; top->m_rdy = 0; top->m_done = 0; top->m_rdata = 0; top->m_snoop = 0; top->m_snoop_addr = 0;
  // Evaluate once so that the outputs settle.
  top->unc_map = 0; top->ipl = 0; top->eval();
  // idle = clocks without any retire; phase = 0 before the test instruction, 1 after.
  int idle = 0, phase = 0;
  // State of the memory port model: busy, write, ready (read beat valid) and done flags, address, data, beat count, beat number, wait counter and byte enables.
  bool mbusy = false, mwr = false, rdy = false, done = false; uint32_t maddr = 0, mwdata = 0; int mbeats = 0, mnb = 0, mwait = 0, mbe = 0;
  // Read data of the current beat (64 bits).
  uint64_t rdata = 0;
  // rd32(): read a big endian 32-bit word from the core's memory.
  auto rd32 = [&](uint32_t a) { return ((uint32_t)mem[a & MASK] << 24) | ((uint32_t)mem[(a + 1) & MASK] << 16) | ((uint32_t)mem[(a + 2) & MASK] << 8) | mem[(a + 3) & MASK]; };
  // Snapshot taken at the retire of the test instruction.
  uint32_t snap_sh[16], snap_bk[3], snap_sr = 0; std::map<uint32_t, uint8_t> snap_wr;
  // next_pc = PC of the retire after the test instruction.
  uint32_t next_pc = 0; bool have_next = false;
  // The clock loop; 6000 clocks is far more than any single instruction and its prologue need.
  for (long cyc = 0; cyc < 6000; cyc++) {
    // Falling edge: present the memory port's answers, ...
    top->clk = 0; top->m_rdy = rdy; top->m_done = done; top->m_rdata = rdata; top->eval();
    // ... rising edge: the core registers its state.
    top->clk = 1; top->eval();
    // After 4 clocks release reset.
    if (cyc == 4) top->rst = 0;
    // Clear the one-clock answer flags.
    rdy = done = false;
    // A new request from the core: accept it.
    if (top->m_req) {
      // write or read, word address times 4, write data, byte enables.
      mbusy = true; mwr = top->m_wr; maddr = (top->m_addr << 2) & MASK; mwdata = top->m_wdata; mbe = top->m_be;
      // Number of beats of the transfer (a line read has several, a single access one) and the initial wait.
      mbeats = top->m_len + 1; mnb = 0; mwait = 2;
    // A transaction in progress: count down the latency.
    } else if (mbusy) {
      // Latency is over (two clocks).
      if (mwait > 1) mwait--;
      // Write: apply the enabled bytes to memory when the write completes.
      else if (mwr) {
        for (int i = 0; i < 4; i++) if (mbe & (8 >> i)) { uint32_t a = (maddr + i) & MASK; mem[a] = (mwdata >> (24 - 8 * i)) & 0xff; touched.push_back(a); }
        // Signal done and free the port.
        done = true; mbusy = false;
      // Read: return the next 64-bit beat (a single access is the aligned double word that contains it).
      } else {
        // Address of this beat.
        uint32_t a = mbeats == 1 ? (maddr & ~7u) : maddr + 8 * mnb;
        // Both 32-bit halves of the beat, ready, and restart the wait.
        rdata = ((uint64_t)rd32(a) << 32) | rd32(a + 4); rdy = true; mwait = 2;
        // After the last beat the transaction is done.
        if (++mnb == mbeats) { done = true; mbusy = false; }
      }
    }
    // wrA(): apply what lane A's uop retired in this clock to the shadow registers and the store list (the core can commit two instructions per clock, in two lanes).
    auto wrA = [&]() {
      // A uop retired: its writes are valid.
      if (top->ret_u) {
        // Address register update by the source EA ((An)+ or -(An)).
        if (top->ret_ws_en) sh[8 + top->ret_ws_r] = top->ret_ws_v;
        // Address register update by the destination EA or LEA.
        if (top->ret_wd_en) sh[8 + top->ret_wd_r] = top->ret_wd_v;
        // Register write from the write-back stage.
        if (top->ret_wen) sh[top->ret_wreg] = top->ret_wval;
        // Write to a stack pointer bank.
        if (top->ret_bk_en) bk[top->ret_bk_r] = top->ret_wval;
        // Store: the four byte lanes with their byte enables go into wr.
        if (top->ret_st) for (int i = 0; i < 4; i++) if (top->ret_sbe & (8 >> i)) wr[((top->ret_saddr & ~3u) + i) & MASK] = (top->ret_sdata >> (24 - 8 * i)) & 0xff;
        if (top->ret_st2) for (int i = 0; i < 4; i++) if (top->ret_sbe2 & (8 >> i)) wr[((top->ret_saddr2 & ~3u) + i) & MASK] = (top->ret_sdata2 >> (24 - 8 * i)) & 0xff;  // a second store word of a misaligned access that crosses a longword
      }
      // A7 swap (the stack pointer changed between USP, ISP and MSP).
      if (top->ret_sw) { bk[top->ret_sw_old] = sh[15]; sh[15] = top->ret_sw_v; }
    };
    // wrB(): lane B's register write.
    auto wrB = [&]() { if (top->ret2_wen) sh[top->ret2_wreg] = top->ret2_wval; };
    // If nothing retired in this clock, only a stack pointer swap can still happen; give up after 2000 clocks without any retire.
    if (!top->ret_u && !top->ret2_v) { if (top->ret_sw) { bk[top->ret_sw_old] = sh[15]; sh[15] = top->ret_sw_v; } if (++idle > 2000) break; continue; }
    // Some retire happened.
    idle = 0;
    // ev(): handle one retired instruction. Phase 0: wait for the retire at the test PC and take the snapshot. Phase 1: the next retire is the end of the test.
    auto ev = [&](uint32_t rpc, uint32_t rsr, bool rirq) -> bool {   // true: stop
      // With only one test selected, print each retire.
      if (trace) printf("    retire %06x sr %04x%s\n", rpc, rsr, rirq ? " irq" : "");
      // Phase 0: the instruction under test.
      if (phase == 0) {
        // Retires of the prologue (other PCs) are ignored.
        if (rpc != t.in.pc) return false;
        // Snapshot of the registers, the SR after the instruction (rsr), and the stores so far; go to phase 1.
        memcpy(snap_sh, sh, sizeof sh); memcpy(snap_bk, bk, sizeof bk); snap_sr = rsr; snap_wr = wr; phase = 1;
      // Phase 1: remember the PC of the next retire and stop.
      } else { next_pc = rpc; have_next = true; return true; }
      return false;
    };
    // Two lanes can retire in one clock; rB = lane B retires, bF = lane B holds the older instruction (so it is processed first).
    const bool rB = top->ret2_v, bF = rB && top->ret2_first;
    // If lane B is older, process it first.
    if (bF) { wrB(); if (ev(top->ret2_pc, (top->ret_sr & 0xffe0) | top->ret2_ccr, false)) break; }
    // Then lane A's writes.
    wrA();
    // Lane A's instruction, with the SR made of the system byte and the CCR of this lane.
    if (top->ret_v && ev(top->ret_pc, (top->ret_sr & 0xffe0) | top->ret_ccr, top->ret_irq)) break;
    // Lane B's instruction when it is the younger one.
    if (rB && !bF) { wrB(); if (ev(top->ret2_pc, (top->ret_sr & 0xffe0) | top->ret2_ccr, false)) break; }
  }
  // The test instruction never retired.
  if (phase == 0) return "never retired at the test PC (hang or wrong path)";
  // diff = text of the differences; keys = which kinds of register differed.
  std::string diff, keys;
  // cmp(): compare one value, append to the text and the key list.
  auto cmp = [&](const char* name, const char* key, uint32_t rtl, uint32_t ref) {
    if (rtl != ref) { char b[96]; snprintf(b, sizeof b, " %s RTL %08x ref %08x;", name, rtl, ref); diff += b; if (keys.find(key) == std::string::npos) keys += std::string(keys.empty() ? "" : "+") + key; }
  };
  // Register names for the messages.
  static const char* rnm[16] = {"D0","D1","D2","D3","D4","D5","D6","D7","A0","A1","A2","A3","A4","A5","A6","A7"};
  // D0-D7 against the final state, key "D".
  for (int i = 0; i < 8; i++) cmp(rnm[i], "D", snap_sh[i], t.fin.d[i]);
  // A0-A6, key "A".
  for (int i = 0; i < 7; i++) cmp(rnm[8 + i], "A", snap_sh[8 + i], t.fin.a[i]);
  // rsp = the three stack pointers: the banks, with the active one taken from the shadow A7 (SR bit 13 chooses user or supervisor).
  uint32_t rsp[3] = {snap_bk[0], snap_bk[1], snap_bk[2]}; rsp[(snap_sr & 0x2000) ? 1 : 0] = snap_sh[15];
  // USP (A7 in user mode), compared always.
  cmp("USP", "USP", rsp[0], t.fin.usp);
  // SSP, only when no exception happened (the 68000 frame size differs).
  if (!exc) cmp("SSP", "SSP", rsp[1], t.fin.ssp);
  // SR.
  cmp("SR", "SR", snap_sr, t.fin.sr);
  // PC after the instruction, compared with the PC of the next retire.
  if (have_next) cmp("PC", "PC", next_pc, t.fin.pc); else *nopc = true;
  // Memory comparison, for tests that are not exception tests.
  if (!exc) {
    // fin = final memory, ini = initial memory, as maps.
    std::map<uint32_t, uint8_t> fin; for (auto& r : t.fin.ram) fin[r.first & MASK] = r.second;
    std::map<uint32_t, uint8_t> ini; for (auto& r : t.in.ram) ini[r.first & MASK] = r.second;
    // md = text of the memory differences.
    std::string md;
    // Every byte listed in the final state: the core's value is the stored one if it stored the byte, else the initial value.
    for (auto& r : fin) { auto w = snap_wr.find(r.first); uint8_t got = w != snap_wr.end() ? w->second : (ini.count(r.first) ? ini[r.first] : 0); if (got != r.second) { char b[40]; snprintf(b, sizeof b, " %06x=%02x(%02x)", r.first, got, r.second); md += b; } }
    // Every byte the core stored that the final state does not list: it must equal the initial value (the core may rewrite a byte with the same value).
    for (auto& w : snap_wr) if (!fin.count(w.first) && w.second != (ini.count(w.first) ? ini[w.first] : 0)) { char b[40]; snprintf(b, sizeof b, " %06x=%02x(none)", w.first, w.second); md += b; }
    // A difference: add to the text and the key list.
    if (!md.empty()) { diff += " mem RTL(ref):" + md + ";"; if (keys.find("MEM") == std::string::npos) keys += std::string(keys.empty() ? "" : "+") + "MEM"; }
  }
  // No difference: pass.
  if (diff.empty()) return "";
  // A mismatch: now sort out the classes the 68000 data cannot judge.
  uint16_t op = t.in.pf[0]; uint32_t dsr = snap_sr ^ t.fin.sr;
  // SR writes: the 68000 data clears M (bit 12) and T0 (bit 14), the 68020 has them and A7 follows M, so the test is skipped.
  // 68000-only SR bits: the data clears M (bit 12) and T0 (bit 14), the 68020 has them and A7 follows M
  if (((op & 0xFFC0) == 0x46C0 || op == 0x007C || op == 0x027C || op == 0x0A7C) && (snap_sr & 0x5000) && rule("sr_m_t0")) return "SKIP:write of SR bit 12 or 14 (M and T0 exist on the 68020 only)";
  // BCD with only N or V different (undefined on both CPUs): count as undefined.
  // BCD: N and V are undefined
  if ((file.rfind("ABCD", 0) == 0 || file.rfind("SBCD", 0) == 0 || file.rfind("NBCD", 0) == 0) && keys == "SR" && (dsr & ~0x0Au) == 0 && rule("bcd_nv")) return "UNDEF:BCD N/V";
  // BCD with an operand that is not valid BCD: undefined result.
  if ((file.rfind("ABCD", 0) == 0 || file.rfind("SBCD", 0) == 0 || file.rfind("NBCD", 0) == 0) && rule("bcd_nv") && bcd_bad(t, im, file)) return "UNDEF:BCD operand is not valid BCD (result undefined)";
  // DIVU and DIVS overflow or divide by zero: the 68000 clears C, the 68020 leaves it; WinUAE and the core agree with the 68020, so count it as a disagreement with the data.
  // DIVU/DIVS overflow and divide by zero: the 68000 clears C, the 68020 (WinUAE, and the RTL) leaves C unchanged
  if ((file == "DIVU" || file == "DIVS") && keys == "SR" && dsr == 1 && (snap_sr & 1) && rule("div_c")) return "DISAGREE:DIV overflow or zero divide, C kept (data clears C; WinUAE 68020 keeps it)";
  // ASR with a register count above the operand size on a negative operand: the data has C = X = 0 for counts 33 to 63, WinUAE, Musashi and the core have C = X = the sign bit.
  // ASR by a register count above the operand size, negative operand: the data has C = X = 0 (count 33-63), WinUAE and Musashi C = X = sign
  if (file.rfind("ASR.", 0) == 0 && keys == "SR" && (op & 0xF038) == 0xE020 && !(op & 0x0100) && rule("asr_big")) {
    int sz = (op >> 6) & 3, bits = sz == 0 ? 8 : sz == 1 ? 16 : 32; uint32_t cnt = t.in.d[(op >> 9) & 7] & 63, v = t.in.d[op & 7];
    bool neg = (v >> (bits - 1)) & 1;
    if (neg && (int)cnt > bits && (snap_sr & 0x11) == 0x11 && (t.fin.sr & 0x11) == 0 && (dsr & ~0x11u) == 0) return "DISAGREE:ASR count above size, negative operand (data C=X=0, WinUAE/Musashi/RTL C=X=1)";
  }
  // A real failure: the keys, a bar and the details.
  return keys + "|" + diff;
}

// main(): read the arguments, read all of the JSON from standard input, loop over the tests and print the summary.
int main(int argc, char** argv) {
  if (argc < 3) { fprintf(stderr, "usage: sst SKIPFILE NAME [max_tests] [show] [only] < tests.json\n"); return 2; }
  load_skips(argv[1]);
  std::string file = argv[2];
  // max_tests and show, with their defaults.
  long maxn = argc > 3 ? atol(argv[3]) : 0; int show = argc > 4 ? atoi(argv[4]) : 3;
  long only = argc > 5 ? atol(argv[5]) : 0;   // run only test number `only` (1-based) with a retire trace
  // Read the whole input.
  std::string data; { char buf[1 << 16]; size_t n; while ((n = fread(buf, 1, sizeof buf, stdin)) > 0) data.append(buf, n); }
  jp = data.data(); je = jp + data.size();
  // Skip the opening bracket of the array.
  ws(); if (jp < je && *jp == '[') jp++;
  // st = counters, shown = failures printed so far, idx = test number.
  Stats st; int shown = 0; long idx = 0;
  for (;;) {
    // Stop at the closing bracket or the end.
    ws(); if (jp >= je || *jp == ']') break;
    // Parse the next test object.
    J j; if (!jparse(j)) { fprintf(stderr, "%s: JSON error at test %ld\n", file.c_str(), idx); return 2; }
    // Count the test; stop when the limit is reached.
    idx++; if (maxn && idx > maxn) break;
    // With a selected test, ignore the others.
    if (only && idx != only) continue;
    // Retire trace only for a selected test.
    trace = only ? 1 : 0;
    // Test name for the messages.
    Test t; auto nm = j.get("name"); if (nm) t.name = nm->s;
    // Both states must be present.
    if (!load_state(j.get("initial"), t.in) || !load_state(j.get("final"), t.fin)) { fprintf(stderr, "bad test %s\n", t.name.c_str()); return 2; }
    // The bus transactions are parsed but not used.
    if (auto tx = j.get("transactions")) for (auto& e : tx->a) {
      if (e.a.size() < 6 || e.a[0].t != 's') continue;
      t.tx.push_back({e.a[0].s[0], (int)e.a[2].n, (uint32_t)e.a[3].n, e.a[4].s.size() > 1 ? e.a[4].s[1] : 'w'});
    }
    // Build the instruction image.
    Img im; for (auto& r : t.in.ram) im.m[r.first & MASK] = r.second;
    // The prefetch words overwrite the first four bytes at the test PC.
    im.m[t.in.pc & MASK] = t.in.pf[0] >> 8; im.m[(t.in.pc + 1) & MASK] = t.in.pf[0] & 0xff; im.m[(t.in.pc + 2) & MASK] = t.in.pf[1] >> 8; im.m[(t.in.pc + 3) & MASK] = t.in.pf[1] & 0xff;
    // exc = exception test, pro = free prologue area.
    bool exc; uint32_t pro = 0;
    // Decide if the test is skipped.
    std::string why = skip_reason(t, im, &exc, &pro, file);
    if (!why.empty()) { st.skip[why]++; continue; }
    // Clear the list of touched addresses.
    touched.clear();
    // Run it.
    bool nopc; std::string r = run_test(t, exc, pro, &nopc, file, im);
    // Clear the memory the test used.
    for (uint32_t a : touched) mem[a] = 0;
    // A skip decided during the comparison.
    if (r.rfind("SKIP:", 0) == 0) { st.skip[r.substr(5)]++; continue; }
    // Count the run; nopc = no final PC was seen (the instruction did not retire a successor).
    st.run++; if (nopc) st.nopc++;
    // Pass.
    if (r.empty()) st.pass++;
    // Undefined result (BCD flags and similar).
    else if (r.rfind("UNDEF:", 0) == 0) { st.undef++; st.cause[r.substr(6)]++; }
    // The data disagrees with the core and with the other references: counted apart.
    else if (r.rfind("DISAGREE:", 0) == 0) { st.disagree++; st.cause[r.substr(9)]++; }
    // A failure: count by cause, print the first show of them with the details.
    else {
      st.fail++;
      size_t bar = r.find('|'); std::string key = bar == std::string::npos ? r : r.substr(0, bar);
      st.cause[(exc ? "exc " : "") + key]++;
      if (shown < show) { shown++; printf("  FAIL %s: %s\n", t.name.c_str(), bar == std::string::npos ? r.c_str() : r.c_str() + bar + 1); }
    }
  }
  // Total of the skips.
  long sk = 0; for (auto& s : st.skip) sk += s.second;
  // Summary line: counts of run, pass, fail, undefined, disagree, skip and no-final-PC.
  printf("%-14s run %6ld pass %6ld fail %6ld undef %5ld disagree %5ld skip %6ld nopc %ld\n", file.c_str(), st.run, st.pass, st.fail, st.undef, st.disagree, sk, st.nopc);
  // One line per skip reason.
  for (auto& s : st.skip) printf("  skip %6ld  %s\n", s.second, s.first.c_str());
  // One line per failure cause (groups of registers).
  for (auto& s : st.cause) printf("  cause %6ld  %s\n", s.second, s.first.c_str());
  return 0;
}
