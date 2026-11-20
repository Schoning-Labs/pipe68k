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
// ==== p68_pdf.vh: the fill predecode of one word =====================================================================
//
// What it is. Fetch has to know where each instruction ends before anything else can happen, because the 68k has
// instructions of 1 to 11 words and the length is hidden in the opcode and its addressing modes. Working the length out
// in the clock where fetch needs it would put a full opcode decode in front of the queue pointer, and in the first
// pipeline sample exactly that (queue pointer, 16:1 word select, length decode, 19 levels) was the slowest path. So
// every word is "predecoded" ahead of time, as if it were an opcode, and the result travels with the word. Only the
// predecode of the word that really is an opcode is ever used; the others are ignored.
//
// The predecode of one word is 12 bits (function pdall at the end):
//   bit  11     sequence: the instruction needs more than one uop (a sequencer runs after the decoder's step 0)
//   bits 10-7   length in words, opcode included (1-5 here; longer forms are hold entries, see p68_ib.vh)
//   bit   6     lane-B class: a simple register instruction that may run in the second pipe (psimp)
//   bit   5     memory class: a single-uop instruction with one simple memory operand (pmem)
//   bit   4     branch class: Bcc, BRA or DBcc (pbrc)
//   bits  3-0   pd_fx: where a full-format extension word could sit (p68_p7f.vh), so fetch can flag such instructions
//
// Where it runs. These functions are not synthesized at all. tools/gen_pdrom.py evaluates them over all 65,536 words in
// Icarus Verilog and writes the block ROM tables of p68_pdrom.v (p68_pdrom.vh). The ROMs compute the predecode of each
// I-cache fill beat, and the I-cache stores it next to the words, so it is never on the path from the cache into the
// fetch queue. Earlier the same functions were synthesized as logic in fetch (about 1,100 LUTs); the history of that
// change is in p68_pdrom.v. The decoders in DS check every instruction again, so a wrong predecode can make the core
// slow or make it replay, but the length rules here must match the decoders exactly, and the bench checks that.
//
// This file is included inside a module body after p68_dec.vh, p68_sysf.vh, p68_md.vh, p68_mdf.vh, p68_flowf.vh,
// p68_bff.vh and p68_p7f.vh, whose functions (eax, pdec_sys, pd_int, pd_md, pd_flow, pd_bf, pd_p7, pd_fx, movem_ok) it
// calls. Each of those gives {hit, sequence, length} for its own group of instructions.
//
// 68k encoding facts used throughout (M68000 Family Programmer's Reference Manual):
//   - bits 15-12 of the opcode are the "line", the main instruction group (0 immediate and bit ops, 1-3 MOVE, 4 misc,
//     5 ADDQ/SUBQ/Scc/DBcc, 6 Bcc/BSR, 7 MOVEQ, 8 OR/DIV, 9 SUB, B CMP/EOR, C AND/MUL, D ADD, E shifts, F coprocessor)
//   - an effective address (EA) is a 3-bit mode and a 3-bit register; the source EA is bits 5-3 (mode) and 2-0
//     (register). Mode 7 uses the register field as a sub-mode: 0 abs.W, 1 abs.L, 2 (d16,PC), 3 (d8,PC,Xn), 4 #imm
//   - MOVE's destination EA is bits 11-6 with the fields swapped: register in 11-9, mode in 8-6
//   - for most instructions bits 7-6 are the size (00 byte, 01 word, 10 long); 11 marks another form (e.g. ADDA)
//   - eax(mode, register, size) in p68_dec.vh counts the extension words an EA needs (0-2 for the 68000 modes and
//     the brief index format; a full-format index is handled by p68_fx and is not counted here)

// predecode: {sequence, length in words} from the opcode word alone (same rules as the decoders)
// FPU instructions (coprocessor id 1): {hit, sequence, length}. Every FPU instruction is a sequence. A general
// operation counts the longest immediate of the forms done in hardware (.L/.S, two words); the three-word #imm forms
// (.B and .W take one word, .D and .X are trapped) correct the length at issue, through DS's length override.
function [5:0] pd_fpu;
  input [15:0] w;
  reg [2:0] m, r;                        // the EA mode and register field of the opcode
  begin
    m = w[5:3]; r = w[2:0];
    pd_fpu = 6'h0;                       // not an FPU opcode: no hit
    if (w[15:9] == 7'b1111001)           // line F with coprocessor id 1 in bits 11-9: the FPU's opcodes
      case (w[8:6])                      // coprocessor instruction type
        3'b000: pd_fpu = {2'b11, 4'd2 + {2'b00, eax(m, r, `SZ_L)}};    // general op: opcode, command word, then the EA's words
        3'b001: pd_fpu = {2'b11, (m == 3'd1) ? 4'd3 : (m == 3'd7 && r == 3'd2) ? 4'd3 : (m == 3'd7 && r == 3'd3) ? 4'd4 :
                                 (m == 3'd7 && r == 3'd4) ? 4'd2 : 4'd2 + {2'b00, eax(m, r, `SZ_B)}};
                                         // FDBcc (mode 1) 3 words; FTRAPcc.W 3, .L 4, without operand 2; FScc 2 + EA
        3'b010: pd_fpu = {2'b11, 4'd2};  // FBcc.W: opcode and a 16-bit displacement
        3'b011: pd_fpu = {2'b11, 4'd3};  // FBcc.L: opcode and a 32-bit displacement
        3'b100, 3'b101: pd_fpu = {2'b11, 4'd1 + {2'b00, eax(m, r, `SZ_L)}};   // FSAVE, FRESTORE: opcode and the EA's words
        default: ;
      endcase
  end
endfunction
// {sequence, length} of any opcode. The groups with special rules go first (each helper gives a hit bit in bit 5),
// then the general rules for the common instruction shapes.
function [4:0] pdec;
  input [15:0] w;
  reg [3:0] g; reg [1:0] sz, xs_, xd_; reg mv, ea, mm; reg [5:0] ps, pi, pm, pf, pb, p7, pq;   // ps: system group (pdec_sys), pi: integer group (pd_int)
  begin
    ps = pdec_sys(w);                                                      // system instructions (p68_sysf.vh)
    pq = (`P68_FPU != 0) ? pd_fpu(w) : 6'h0;                               // FPU instructions, only in a build with the FPU
    g  = w[15:12];                                                         // the line
    mv = (g == 4'h1 || g == 4'h2 || g == 4'h3);                            // MOVE.B, MOVE.L, MOVE.W (and MOVEA)
    mm = movem_ok(w);                                                      // MOVEM with a legal EA
    // operand size for the EA's immediate length: MOVE's size is in its line (1 byte, 3 word, 2 long); otherwise bits
    // 7-6, and the "11" form is a word or long address operation chosen by bit 8 (ADDA.W / ADDA.L)
    sz = mv ? ((g == 4'h1) ? `SZ_B : (g == 4'h3) ? `SZ_W : `SZ_L) : (w[7:6] == 2'b11) ? (w[8] ? `SZ_L : `SZ_W) : w[7:6];
    xs_ = eax(w[5:3], w[2:0], mm ? `SZ_W : sz);                           // extension words of the source EA
    xd_ = eax(w[8:6], w[11:9], `SZ_W);                                    // ... of MOVE's destination EA (never immediate)
    // instructions that are one opcode word plus their source EA's extension words: ADDQ/SUBQ (line 5, size not 11),
    // LEA (line 4, bits 8-6 = 111), and the two-operand lines 8, 9, B, C, D. In those lines, size 11 is ADDA/SUBA/CMPA
    // (9, B, D; DIVU/MULU on 8 and C are handled by pd_md), and with bit 8 set the register-only forms with mode 0 or 1
    // (ABCD/SBCD/ADDX/SUBX/EXG and similar) are excluded, except that mode 0 on line B is EOR Dn,Dm, which is an EA form.
    ea = (g == 4'h5 && w[7:6] != 2'b11) || (g == 4'h4 && w[8:6] == 3'b111) ||
         ((g == 4'h8 || g == 4'h9 || g == 4'hB || g == 4'hC || g == 4'hD) &&
          ((w[7:6] == 2'b11) ? (g == 4'h9 || g == 4'hB || g == 4'hD)
                             : !(w[8] && (w[5:3] == 3'd1 || (w[5:3] == 3'd0 && g != 4'hB)))));
    pi = pd_int(w);                                                         // the rest of the integer set (p68_dec.vh)
    pb = pd_bf(w);                                                          // bit fields
    p7 = pd_p7(w);                                                          // CAS, CAS2, BTST Dn,#imm and other long forms
    pm = pd_md(w);                                                         // multiply and divide
    pf = pd_flow(w);                                                      // program flow (calls, returns, LINK, ...)
    // the first group that claims the word decides; the order matters only where two groups overlap
    if (pq[5])        pdec = pq[4:0];                                     // FPU (coprocessor id 1)
    else if (ps[5])   pdec = ps[4:0];                                     // system group ahead of the integer group
    else if (pi[5])   pdec = pi[4:0];
    else if (pm[5])   pdec = pm[4:0];                                     // MUL/DIV sequences
    else if (pf[5])   pdec = pf[4:0];                                     // flow instructions
    else if (pb[5])   pdec = pb[4:0];                                     // bit fields
    else if (p7[5])   pdec = p7[4:0];                                     // long forms
    else if (mm) pdec = {1'b1, 4'd2 + {2'b0, xs_}};                       // MOVEM: opcode, register mask, EA words; a sequence
    else if (mv) pdec = {w[8:6] == 3'd6, 4'd1 + {2'b0, xs_} + {2'b0, xd_}};   // MOVE: both EAs; a (d8,An,Xn) destination is a sequence
    else if (ea) pdec = {1'b0, 4'd1 + {2'b0, xs_}};                       // the one-EA shapes above
    else if (g == 4'h6 && w[11:8] != 4'h1) pdec = {1'b0, (w[7:0] == 8'h00) ? 4'd2 : (w[7:0] == 8'hFF) ? 4'd3 : 4'd1};
                                         // Bcc/BRA (not BSR, line 6 condition 1): an 8-bit displacement of $00 means a
                                         // 16-bit displacement word follows, $FF a 32-bit one (68020), else it is in the opcode
    else         pdec = {1'b0, 4'd1};                                     // everything else: one word
  end
endfunction
// Lane-B class of an opcode word: the register-only single-uop instructions that the second pipe's ALU (p68_exb.v) can
// do. DR checks the decoded uop again before it forms a pair, so this class is a first filter that keeps the pairing
// terms in DS small. The list: MOVEQ; MOVE/MOVEA from Dn/An/#imm to Dn/An; ADDQ/SUBQ to Dn/An;
// OR/SUB/CMP/AND/ADD <Dn/An/#imm>,Dn; ADDA/SUBA/CMPA; EOR Dn,Dm; CLR/NEG/NOT/TST Dn; EXT/EXTB;
// ORI/ANDI/SUBI/ADDI/EORI/CMPI #imm,Dn; and LEA as a pair's second instruction (DS turns it into an add or a move).
function psimp;
  input [15:0] w;
  reg [3:0] g; reg [2:0] om, m, r, dm; reg [1:0] s; reg src, bw;
  begin
    g = w[15:12]; om = w[8:6]; m = w[5:3]; r = w[2:0]; dm = w[8:6]; s = w[7:6];   // line, opmode, source EA, MOVE dest mode, size
    psimp = 1'b0;
    if (g == 4'h7) psimp = !w[8];                                        // MOVEQ (bit 8 must be 0)
    else if (g == 4'h1 || g == 4'h2 || g == 4'h3)                        // MOVE: to Dn, or to An (MOVEA, not byte); from Dn, An (not byte) or #imm
      psimp = (dm == 3'd0 || (dm == 3'd1 && g != 4'h1)) && (m == 3'd0 || (m == 3'd1 && g != 4'h1) || (m == 3'd7 && r == 3'd4));
    else if (g == 4'h5) psimp = (s != 2'b11) && (m == 3'd0 || (m == 3'd1 && s != 2'b00));   // ADDQ/SUBQ to Dn, or An (not byte)
    else if (g == 4'h8 || g == 4'h9 || g == 4'hB || g == 4'hC || g == 4'hD) begin
      bw  = (om[1:0] != 2'b00);                                          // word or long
      src = (m == 3'd0) || (m == 3'd1 && bw && g != 4'h8 && g != 4'hC) || (m == 3'd7 && r == 3'd4);   // source Dn, An (not for OR/AND), #imm
      if (om == 3'd0 || om == 3'd1 || om == 3'd2) psimp = src;                         // <ea>,Dn forms
      else if (om == 3'd3 || om == 3'd7) psimp = (g == 4'h9 || g == 4'hB || g == 4'hD) && (m == 3'd0 || m == 3'd1 || (m == 3'd7 && r == 3'd4));   // ADDA/SUBA/CMPA
      else psimp = (g == 4'hB) && (m == 3'd0);                                         // Dn,<ea> forms: only EOR Dn,Dm
    end
    else if (g == 4'h4)
      // CLR (bits 11-8 = 2), NEG (4), NOT (6), TST (A) on Dn; EXT.W, EXT.L, EXTB.L; LEA with (An), (d16,An), abs.W, abs.L
      psimp = ((w[11:8] == 4'h2 || w[11:8] == 4'h4 || w[11:8] == 4'h6 || w[11:8] == 4'hA) && s != 2'b11 && m == 3'd0) ||
              (w[11:6] == 6'b100010 && m == 3'd0) || (w[11:6] == 6'b100011 && m == 3'd0) || (w[11:6] == 6'b100111 && m == 3'd0) ||
              (w[8:6] == 3'b111 && (m == 3'd2 || m == 3'd5 || (m == 3'd7 && r <= 3'd1)));     // LEA: DS makes it an add
    else if (g == 4'h0)
      // immediate to Dn: ORI (bits 11-9 = 0), ANDI (1), SUBI (2), ADDI (3), EORI (5), CMPI (6); not the bit operations
      psimp = !w[8] && s != 2'b11 && m == 3'd0 && (w[11:9] == 3'd0 || w[11:9] == 3'd1 || w[11:9] == 3'd2 || w[11:9] == 3'd3 ||
                                                   w[11:9] == 3'd5 || w[11:9] == 3'd6);
  end
endfunction
// Memory class of a second instruction. DR may swap such an instruction into lane A when the head instruction is of
// lane B's class (the memory instruction then runs in lane A, the head in lane B as the older of the two). The class:
// single-uop instructions with one memory operand at (An), (An)+, -(An), (d16,An), abs.W or abs.L (no index, no
// PC-relative) and at most two extension words in total: MOVE/MOVEA, OR/SUB/CMP/AND/ADD <ea>,Dn and Dn,<ea>,
// EOR Dn,<ea>, ADDA/SUBA/CMPA, ADDQ/SUBQ, CLR/NEG/NOT/TST. DR checks the decoded uop again.
function pmem;
  input [15:0] w;
  reg [3:0] g; reg [2:0] m, r, dm, dr; reg [1:0] s, xs, xd, xi; reg ms, md, so, dk;
  begin
    g = w[15:12]; m = w[5:3]; r = w[2:0]; dm = w[8:6]; dr = w[11:9]; s = w[7:6];
    ms = (m >= 3'd2 && m <= 3'd5) || (m == 3'd7 && r <= 3'd1);           // the source EA is one of the simple memory modes
    md = (dm >= 3'd2 && dm <= 3'd5) || (dm == 3'd7 && dr <= 3'd1);       // MOVE's destination EA is one
    xs = (m == 3'd5 || (m == 3'd7 && r == 3'd0)) ? 2'd1 : (m == 3'd7 && r == 3'd1) ? 2'd2 : 2'd0;     // source extension words
    xd = (dm == 3'd5 || (dm == 3'd7 && dr == 3'd0)) ? 2'd1 : (dm == 3'd7 && dr == 3'd1) ? 2'd2 : 2'd0; // destination extension words
    xi = (m == 3'd7 && r == 3'd4) ? ((g == 4'h2) ? 2'd2 : 2'd1) : 2'd0;  // immediate source: two words for MOVE.L, else one
    pmem = 1'b0;
    if (g == 4'h1 || g == 4'h2 || g == 4'h3) begin                       // MOVE: register/immediate/memory to register/memory
      so = (m == 3'd0) || (m == 3'd1 && g != 4'h1) || (m == 3'd7 && r == 3'd4) || ms;
      dk = (dm == 3'd0) || (dm == 3'd1 && g != 4'h1) || md;
      pmem = so && dk && (ms || md) && ({1'b0, xs} + {1'b0, xi} + {1'b0, xd} <= 3'd2);   // one memory side, two words at most
    end
    else if (g == 4'h8 || g == 4'h9 || g == 4'hB || g == 4'hC || g == 4'hD) begin
      if (dm == 3'd3 || dm == 3'd7) pmem = (g == 4'h9 || g == 4'hB || g == 4'hD) && ms;     // ADDA/SUBA/CMPA
      else pmem = ms;                      // <ea>,Dn and Dn,<ea> (memory modes exclude ADDX, SBCD, CMPM, EXG, PACK)
    end
    else if (g == 4'h5) pmem = (s != 2'b11) && ms;                       // ADDQ/SUBQ to memory
    else if (g == 4'h4) pmem = (w[11:8] == 4'h2 || w[11:8] == 4'h4 || w[11:8] == 4'h6 || w[11:8] == 4'hA) && (s != 2'b11) && ms;   // CLR/NEG/NOT/TST
  end
endfunction
// Branch class: Bcc and BRA (line 6 except BSR, condition 1), or DBcc (0101 cccc 1100 1rrr)
function pbrc; input [15:0] w; pbrc = ((w[15:12] == 4'h6) && (w[11:8] != 4'h1)) || ((w & 16'hF0F8) == 16'h50C8); endfunction
// The whole 12-bit predecode, in the bit order listed at the top
function [11:0] pdall; input [15:0] w; pdall = {pdec(w), psimp(w), pmem(w), pbrc(w), pd_fx(w)}; endfunction
