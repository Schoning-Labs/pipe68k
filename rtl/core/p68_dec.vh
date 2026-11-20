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
// ---------------------------------------------------------------------------------------------------------------------
// p68_dec.vh: the decode functions shared by all decoders, the sequencers, the instruction buffer predecode and the
// fetch predecode.
//
// What this file is. A collection of Verilog functions, included inside a module body (after `include "p68_uop.vh" at
// file level), so that every module that needs them gets its own copy of the logic. All inputs are arguments, so
// simulators re-evaluate them whenever an input changes. Most of the 68k instruction set knowledge that more than one
// unit needs lives here: how many extension words an effective address (EA) has, how an EA becomes a load address
// descriptor (LA) of the micro-op (uop, p68_uop.vh), which EAs an instruction allows, and how long an instruction is.
// Keeping one copy of each rule matters because fetch (which cuts the instruction stream into instructions) and the
// decoders in DS (the decode stage, which turn an instruction into uops) must agree on every length and every hit.
//
// The 68k effective address. Most instructions name an operand with a 6-bit EA field: a 3-bit mode and a 3-bit
// register, normally in bits 5-3 and 2-0 of the opcode word.
//     mode 0 Dn            1 An            2 (An)          3 (An)+         4 -(An)
//     mode 5 (d16,An)      6 (d8,An,Xn)    7 + register:   0 abs.W   1 abs.L   2 (d16,PC)   3 (d8,PC,Xn)   4 #imm
// Modes 5, 6 and mode 7 registers 0, 2, 3 take one extension word; abs.L takes two; an immediate takes one word for
// .B and .W and two for .L. The brief extension word of the indexed modes is:
//     15 D/A | 14-12 index register | 11 W/L | 10-9 scale | 8 = 0 | 7-0 signed displacement
// (bit 8 = 1 selects the 68020 full format, which the pre-sequence p68_fx.v handles; for the decoders the brief layout
// is all there is). A PC-relative address uses the address of the extension word itself as the base, which is why
// the functions take the byte offset of that word from the instruction address (po).
//
// Data codes. The section on dcode and dval below describes how the 32-bit data fields of a uop are passed as small
// codes. This was one step of making the decode shallower (HISTORY.md section 5): twenty decoders each building 32-bit
// immediates and displacements, plus the OR of all of them, was a large part of a path that was 16 logic levels long
// from the window word to the DR entry. Measured first: an ideal opcode ROM for the control fields of the uop would have
// saved only about 700 LUT, far less than the block RAMs it would cost, so the data fields were the target. The codes
// were checked exhaustively against the old value-building decoders over all 65,536 opcodes with several sets of
// extension words.
//
// Abbreviations: AG address generation, OC operand cycle (data cache), EX execute, WB write-back, DR the register stage
// between DS and AG, T0-T3 the temporary registers 16-19 of the register file (not visible to programs).
// ---------------------------------------------------------------------------------------------------------------------

function [1:0] eax;   // extension words of an effective address (68000 modes, brief index format)
  input [2:0] m; input [2:0] r; input [1:0] s;                    // mode, register, operand size (for #imm)
  begin
    if (m == 3'd5 || m == 3'd6) eax = 2'd1;                       // (d16,An), (d8,An,Xn): one word
    // mode 7: abs.L two words; #imm two for .L and one otherwise; abs.W, (d16,PC), (d8,PC,Xn) one; registers 5-7 none
    else if (m == 3'd7) eax = (r == 3'd1) ? 2'd2 : (r == 3'd4) ? ((s == `SZ_L) ? 2'd2 : 2'd1) : (r <= 3'd3) ? 2'd1 : 2'd0;
    else eax = 2'd0;                                              // register and (An) modes: none
  end
endfunction

function [3:0] stepb;  // (An)+ / -(An) step in bytes (byte access on A7 moves it by 2)
  input [1:0] s; input [2:0] r;
  // A7 is the stack pointer and must stay even, so a byte push or pop on A7 moves it by 2 (a 68k rule)
  begin stepb = (s == `SZ_B) ? ((r == 3'd7) ? 4'd2 : 4'd1) : (s == `SZ_W) ? 4'd2 : 4'd4; end
endfunction

function movem_ok;     // MOVEM with a legal EA (same rule in the fetch predecode and in dec_movem)
  input [15:0] w;
  reg [2:0] m, r;
  begin
    m = w[5:3]; r = w[2:0];
    // 0100 1 d 001 s EA: memory to registers (bit 10) takes (An), (An)+, (d16,An), (d8,An,Xn), abs, PC-relative;
    // registers to memory takes (An), -(An), (d16,An), (d8,An,Xn), abs
    movem_ok = (w[15:12] == 4'h4) && w[11] && (w[9:7] == 3'b001) &&
               (w[10] ? (m == 3'd2 || m == 3'd3 || m == 3'd5 || m == 3'd6 || (m == 3'd7 && r <= 3'd3))
                      : (m == 3'd2 || m == 3'd4 || m == 3'd5 || m == 3'd6 || (m == 3'd7 && r <= 3'd1)));
  end
endfunction

// Load address descriptor of an EA. x0, x1: its first two extension words; po: byte offset of x0 from the
// instruction address (PC-relative modes use it). LA_EN is set for memory operands; the caller may clear it.
// AG computes base + index + displacement; this function only says which base, which index and which displacement.
// The sequencers use this value version, because their extension words come from registers latched at the start.
function [`LA_W-1:0] ea_la;
  input [2:0] m; input [2:0] r; input [1:0] s; input [15:0] x0; input [15:0] x1; input [3:0] po;
  reg [`LA_W-1:0] v; reg [7:0] st; reg [8:0] d8p;
  begin
    v = {`LA_W{1'b0}};
    st = {4'h0, stepb(s, r)};                                     // the (An)+ / -(An) step
    d8p = {x0[7], x0[7:0]} + {5'h0, po};                          // (d8,PC,Xn): displacement plus the word's offset
    v[`L_EN] = (m >= 3'd2) && !(m == 3'd7 && r == 3'd4);          // a memory operand (not Dn, An or #imm)
    if (m >= 3'd2 && m <= 3'd6) begin v[`L_BSEL] = `BA_REG; v[`L_BREG] = {2'b01, r}; end   // base An (register 8 + r)
    case (m)
      3'd3: begin v[`L_UEN] = 1'b1; v[`L_UDL] = st; end           // (An)+: read at An, then An += step
      3'd4: begin v[`L_DISP] = 32'h0 - {24'h0, st}; v[`L_UEN] = 1'b1; v[`L_UDL] = 8'h0 - st; end   // -(An): at An - step
      3'd5: v[`L_DISP] = {{16{x0[15]}}, x0};                      // (d16,An)
      3'd6: begin                                                 // (d8,An,Xn): index fields from the brief word
        v[`L_DISP] = {{24{x0[7]}}, x0[7:0]};
        v[`L_XEN] = 1'b1; v[`L_XREG] = {1'b0, x0[15:12]}; v[`L_XL] = x0[11]; v[`L_XSC] = x0[10:9];
      end
      3'd7: case (r)
        3'd0: v[`L_DISP] = {{16{x0[15]}}, x0};                    // abs.W (base zero, sign-extended address)
        3'd1: v[`L_DISP] = {x0, x1};                              // abs.L
        // (d16,PC): base PC, the constant index XK = po makes it the extension word's address
        3'd2: begin v[`L_BSEL] = `BA_PC; v[`L_XK] = po; v[`L_DISP] = {{16{x0[15]}}, x0}; end
        3'd3: begin                                               // (d8,PC,Xn): po folded into the displacement
          v[`L_BSEL] = `BA_PC; v[`L_DISP] = {{23{d8p[8]}}, d8p};
          v[`L_XEN] = 1'b1; v[`L_XREG] = {1'b0, x0[15:12]}; v[`L_XL] = x0[11]; v[`L_XSC] = x0[10:9];
        end
        default: ;                                                // #imm: no address
      endcase
      default: ;                                                  // Dn, An, (An): nothing more
    endcase
    ea_la = v;
  end
endfunction

// ---- data codes ---------------------------------------------------------------------------------------------------
// The decoders give the 32-bit data fields IMM, LA_DISP and SA_DISP as codes; p68_decode turns the
// OR of the codes into values once (dval), instead of every decoder building and OR-ing 32-bit values. Code bits:
// [2:0] kind, [4:3] k (word index 1-3 of the first extension word), [13:5] c. Kinds: 0 c sign-extended, 1 sext16(w[k]),
// 2 zext16(w[k]), 3 zext8(w[k]), 4 {w[k], w[k+1]}, 5 sext8(w[k]) (brief extension on An), 6 sext8(w[k]) + 2k (brief
// extension on the PC: the word's offset from the instruction address is 2k); with LA_XEN the index fields come from
// w[k] too. Code 0 is value 0, so a decoder without a hit still adds nothing. Every candidate value is formed from the
// words alone and the code selects last ({kind, k}: one mux, no adder behind the code).
function [31:0] dcode; input [2:0] kd; input [1:0] k; input [16:0] c; dcode = {18'h0, c[8:0], k, kd}; endfunction
function [31:0] dcon; input [31:0] v; dcon = {18'h0, v[8:0], 5'h0}; endfunction    // constant (fits 9 bits signed)
// word k (1-3) of the instruction
function [15:0] dwk; input [1:0] k; input [15:0] a1, a2, a3; dwk = (k == 2'd1) ? a1 : (k == 2'd2) ? a2 : a3; endfunction
// code to value: a1-a4 are instruction words 1-4. In hardware this is one wide multiplexer per bit selected by the five
// code bits {kind, k}, plus three 9-bit adders (p1-p3) that work on the words in parallel, before the select.
function [31:0] dval;
  input [31:0] cd; input [15:0] a1, a2, a3, a4;
  reg [8:0] p1, p2, p3;
  begin
    // brief extension on the PC: displacement plus the word's offset 2k
    p1 = {a1[7], a1[7:0]} + 9'd2; p2 = {a2[7], a2[7:0]} + 9'd4; p3 = {a3[7], a3[7:0]} + 9'd6;
    case ({cd[2:0], cd[4:3]})
      {3'd1, 2'd1}: dval = {{16{a1[15]}}, a1};                    // kind 1: sign-extended word
      {3'd1, 2'd2}: dval = {{16{a2[15]}}, a2};
      {3'd1, 2'd3}: dval = {{16{a3[15]}}, a3};
      {3'd2, 2'd1}: dval = {16'h0, a1};                           // kind 2: zero-extended word
      {3'd2, 2'd2}: dval = {16'h0, a2};
      {3'd2, 2'd3}: dval = {16'h0, a3};
      {3'd3, 2'd1}: dval = {24'h0, a1[7:0]};                      // kind 3: zero-extended low byte
      {3'd3, 2'd2}: dval = {24'h0, a2[7:0]};
      {3'd3, 2'd3}: dval = {24'h0, a3[7:0]};
      {3'd4, 2'd1}: dval = {a1, a2};                              // kind 4: a long from two words
      {3'd4, 2'd2}: dval = {a2, a3};
      {3'd4, 2'd3}: dval = {a3, a4};
      {3'd5, 2'd1}: dval = {{24{a1[7]}}, a1[7:0]};                // kind 5: sign-extended low byte (brief on An)
      {3'd5, 2'd2}: dval = {{24{a2[7]}}, a2[7:0]};
      {3'd5, 2'd3}: dval = {{24{a3[7]}}, a3[7:0]};
      {3'd6, 2'd1}: dval = {{23{p1[8]}}, p1};                     // kind 6: brief on the PC
      {3'd6, 2'd2}: dval = {{23{p2[8]}}, p2};
      {3'd6, 2'd3}: dval = {{23{p3[8]}}, p3};
      default:      dval = 32'h0;                         // kind 0 (k = 0): the constant only
    endcase
    dval = dval | {{23{cd[13]}}, cd[13:5]};            // the constant enters last (0 with every other kind)
  end
endfunction
// The kind and word index of the codes ({k, kind}, code bits 4:0) straight from the opcode (S for the privileged forms),
// so the value selects do not wait for the decoders' hits and their OR (the decoders' codes then only add constants).
// Equal to the decoders' codes for every legal opcode outside line F (checked exhaustively; line F: dec_fpu's codes,
// whose kinds depend on the command word). For illegal opcodes the value does not count (vector 4).
function dprv; input [15:0] w; input su;               // a privileged form in user mode (its uop only raises vector 8)
  // ORI/ANDI/EORI to SR, STOP, MOVE from SR, MOVE to SR, and the FPU's save/restore (0xF3xx)
  dprv = !su && ((w == 16'h007C) || (w == 16'h027C) || (w == 16'h0A7C) || (w == 16'h4E72) || (w[15:6] == 10'b0100000011) ||
               (w[15:6] == 10'b0100011011) || (w[15:8] == 8'hF3));
endfunction
function dnea; input [15:0] w;                        // the low six bits are no EA with extension words of this uop
  // MOVEQ (line 7), line A, register shifts and bit fields (line E), line F other than the FPU's general group,
  // CHK2/CMP2/CAS (line 0, bits 8-6 = 011, not the static bit group), PEA/SWAP/BKPT, TRAPcc, MOVEM, MOVES, 0x4E60-0x4E7F
  dnea = (w[15:12] == 4'h7) || (w[15:12] == 4'hA) || (w[15:12] == 4'hE && (w[7:6] != 2'b11 || w[11])) ||
         (w[15:12] == 4'hF && w[11:9] != 3'd1) || (w[15:12] == 4'h0 && w[8:6] == 3'd3 && w[15:8] != 8'h08) ||
         (w[15:6] == 10'b0100100001) || ((w & 16'hF0F8) == 16'h50F8 && w[2:0] >= 3'd2) ||
         (w[15:12] == 4'h4 && w[11] && w[9:7] == 3'd1) || (w[15:8] == 8'h0E) || (w[15:5] == 11'b01001110011);
endfunction
function [1:0] dkw; input [15:0] w;                   // word index of the first EA extension word
  // static bit group: behind the bit number word (2); ORI..CMPI: behind the immediate (3 for .L, else 2); line F:
  // FSAVE/FRESTORE 1, other FPU forms behind the command word (2); MULx.L/DIVx.L behind the extension word (2); else 1
  dkw = (w[15:8] == 8'h08) ? 2'd2 :
        (w[15:12] == 4'h0 && !w[8] && w[11:9] != 3'd4 && w[11:9] != 3'd7 && w[7:6] != 2'b11) ? ((w[7:6] == 2'b10) ? 2'd3 : 2'd2) :
        (w[15:12] == 4'hF) ? ((w[8:7] == 2'b10) ? 2'd1 : 2'd2) :
        (w[15:7] == 9'b010011000) ? 2'd2 : 2'd1;
endfunction
function [4:0] dsla; input [15:0] w; input su;          // LA_DISP
  reg [2:0] m, r, kd;
  begin
    m = w[5:3]; r = w[2:0];
    // kind by EA mode: (d16,An) 1, (d8,An,Xn) 5, abs.W and (d16,PC) 1, abs.L 4, (d8,PC,Xn) 6, others 0
    kd = (m == 3'd5) ? 3'd1 : (m == 3'd6) ? 3'd5 : (m != 3'd7) ? 3'd0 :
         (r == 3'd0 || r == 3'd2) ? 3'd1 : (r == 3'd1) ? 3'd4 : (r == 3'd3) ? 3'd6 : 3'd0;
    if (dprv(w, su)) dsla = 5'd0;
    // line 6 branches: a 16-bit (disp 00) or 32-bit (disp FF) displacement at word 1; an 8-bit one is a constant
    else if (w[15:12] == 4'h6) dsla = (w[7:0] == 8'h00) ? {2'd1, 3'd1} : (w[7:0] == 8'hFF) ? {2'd1, 3'd4} : 5'd0;
    else if ((w & 16'hF0F8) == 16'h50C8) dsla = {2'd1, 3'd1};     // DBcc: the displacement word
    else if (dnea(w) || kd == 3'd0) dsla = 5'd0;
    else dsla = {dkw(w), kd};
  end
endfunction
function [4:0] dsim; input [15:0] w; input su;          // IMM
  reg [2:0] m, r; reg [1:0] sz; reg [3:0] g; reg im;
  begin
    m = w[5:3]; r = w[2:0]; sz = w[7:6]; g = w[15:12]; im = (m == 3'd7 && r == 3'd4);   // im: the EA is #imm
    dsim = 5'd0;
    if (dprv(w, su)) dsim = 5'd0;
    else if (g == 4'h0) begin                                    // line 0
      if (w[15:8] == 8'h08) dsim = {2'd1, 3'd3};                  // static bit number: low byte of word 1
      else if (!w[8] && w[11:9] != 3'd4 && w[11:9] != 3'd7 && sz != 2'b11 && !im)
        dsim = {2'd1, (sz == 2'b00) ? 3'd3 : (sz == 2'b01) ? 3'd2 : 3'd4};   // ORI..CMPI: byte, word or long immediate
      else if ((w & 16'hFF3F) == 16'h003C || (w & 16'hFF3F) == 16'h023C || (w & 16'hFF3F) == 16'h0A3C) dsim = {2'd1, 3'd2};   // to CCR/SR
      else if (w[8] && sz == 2'b00 && im) dsim = {2'd1, 3'd3};    // BTST Dn,#imm
    end
    else if (w == 16'h4E72) dsim = {2'd1, 3'd2};                  // STOP #imm
    else if (!im) dsim = 5'd0;                                    // every other group needs an immediate EA
    else if (g == 4'h1 || g == 4'h2 || g == 4'h3) dsim = {2'd1, (g == 4'h2) ? 3'd4 : 3'd2};   // MOVE #imm (.L in line 2)
    // lines 8, 9, B, C, D: long for size 10 and for ADDA.L/SUBA.L/CMPA.L, else a word (also MUL.W/DIV.W)
    else if (g == 4'h8 || g == 4'h9 || g == 4'hB || g == 4'hC || g == 4'hD)
      dsim = {2'd1, ((sz == 2'b11) ? (w[8] && (g == 4'h9 || g == 4'hB || g == 4'hD)) : (sz == 2'b10)) ? 3'd4 : 3'd2};
    else if (g == 4'h4) begin                                    // line 4
      if (w[15:7] == 9'b010011000) dsim = {2'd2, 3'd4};           // MULx.L/DIVx.L #imm: a long at word 2
      else if (w[8] && !w[6]) dsim = {2'd1, w[7] ? 3'd2 : 3'd4};  // CHK.W / CHK.L
      else if (w[15:6] == 10'b0100010011 || w[15:6] == 10'b0100011011) dsim = {2'd1, 3'd2};   // MOVE #imm,CCR/SR
      else if (w[15:8] == 8'h4A && sz != 2'b11) dsim = {2'd1, (sz == 2'b10) ? 3'd4 : 3'd2};   // TST #imm
    end
  end
endfunction
function [4:0] dssa; input [15:0] w;                   // SA_DISP (MOVE destinations)
  reg [1:0] sz, k;
  begin
    sz = (w[13:12] == 2'b01) ? `SZ_B : (w[13:12] == 2'b10) ? `SZ_L : `SZ_W;   // MOVE size field
    k  = eax(w[5:3], w[2:0], sz) + 2'd1;                          // the destination's word comes after the source's
    dssa = 5'd0;
    if (w[15:14] == 2'b00 && w[13:12] != 2'b00) begin             // a MOVE
      if (w[8:6] == 3'd5 || (w[8:6] == 3'd7 && w[11:9] == 3'd0)) dssa = {k, 3'd1};   // (d16,An), abs.W
      else if (w[8:6] == 3'd7 && w[11:9] == 3'd1) dssa = {k, 3'd4};                   // abs.L
    end
  end
endfunction
// The three value selects {SA_DISP, LA_DISP, IMM} of one opcode (w) with its first extension word (w1, line F's
// command word) and S. p68_pdec stores them with each instruction buffer entry, p68_decode recomputes them only for
// p68_fx's changed opcode, so the value muxes of DS's uop start from registers.
// For the FPU's general instructions (line F, coprocessor 1, bits 15-9 = 1111001, bits 8-6 = 000) the immediate select
// depends on the command word: opc (bits 15-13) is the command class and sf (bits 12-10) the source format.
function [14:0] dsx; input [15:0] w, w1; input su;
  reg [2:0] opc, sf; reg gn, go, e1i; reg [4:0] fim;
  begin
    opc = w1[15:13]; sf = w1[12:10]; gn = (w[8:6] == 3'b000);
    // register-to-register, FMOVECR (opc 2, sf 7) and FMOVE out (opc 3): IMM is the command word itself
    go  = gn && (opc == 3'd0 || (opc == 3'd2 && sf == 3'd7) || opc == 3'd3);
    // memory source given as #imm in long, single, word or byte format: the immediate behind the command word
    e1i = gn && (opc == 3'd2) && (sf != 3'd7) && (sf != 3'd5) && (sf != 3'd2) && (w[5:0] == 6'b111100);
    fim = go ? {2'd1, 3'd2} : e1i ? {2'd2, (sf == 3'd4 || sf == 3'd6) ? 3'd2 : 3'd4} : 5'd0;
    dsx = {dssa(w), dsla(w, su), (w[15:9] == 7'b1111001) ? fim : dsim(w, su)};
  end
endfunction
// ea_la with the displacement as a code (k: word index of x0) and no index fields (p68_decode adds them from w[k])
// The decoders use this code version; their uops then carry no 32-bit values.
function [`LA_W-1:0] ea_lac;
  input [2:0] m; input [2:0] r; input [1:0] s; input [1:0] k; input [3:0] po;
  reg [`LA_W-1:0] v; reg [7:0] st;
  begin
    v = {`LA_W{1'b0}};
    st = {4'h0, stepb(s, r)};
    v[`L_EN] = (m >= 3'd2) && !(m == 3'd7 && r == 3'd4);          // memory operand
    if (m >= 3'd2 && m <= 3'd6) begin v[`L_BSEL] = `BA_REG; v[`L_BREG] = {2'b01, r}; end   // base An
    case (m)
      3'd3: begin v[`L_UEN] = 1'b1; v[`L_UDL] = st; end           // (An)+
      3'd4: begin v[`L_DISP] = dcon(32'h0 - {24'h0, st}); v[`L_UEN] = 1'b1; v[`L_UDL] = 8'h0 - st; end   // -(An)
      3'd5: v[`L_DISP] = dcode(3'd1, k, 17'h0);                   // (d16,An)
      3'd6: begin v[`L_DISP] = dcode(3'd5, k, 17'h0); v[`L_XEN] = 1'b1; end   // (d8,An,Xn)
      3'd7: case (r)
        3'd0: v[`L_DISP] = dcode(3'd1, k, 17'h0);                 // abs.W
        3'd1: v[`L_DISP] = dcode(3'd4, k, 17'h0);                 // abs.L
        3'd2: begin v[`L_BSEL] = `BA_PC; v[`L_XK] = po; v[`L_DISP] = dcode(3'd1, k, 17'h0); end   // (d16,PC)
        3'd3: begin v[`L_BSEL] = `BA_PC; v[`L_DISP] = dcode(3'd6, k, 17'h0); v[`L_XEN] = 1'b1; end   // po = 2k (every caller)
        default: ;
      endcase
      default: ;
    endcase
    ea_lac = v;
  end
endfunction
// ea_src with the immediate as a code
function [38:0] ea_srcc;
  input [2:0] m; input [2:0] r; input [1:0] s; input [1:0] k;
  begin
    if (m == 3'd0)                   ea_srcc = {`AS_REG, 2'b00, r, 32'h0};   // Dn
    else if (m == 3'd1)              ea_srcc = {`AS_REG, 2'b01, r, 32'h0};   // An
    else if (m == 3'd7 && r == 3'd4) ea_srcc = {`AS_IMM, 5'h0, dcode((s == `SZ_L) ? 3'd4 : 3'd2, k, 17'h0)};   // #imm
    else                             ea_srcc = {`AS_MEM, 5'h0, 32'h0};       // memory: the loaded value
  end
endfunction

// ---- the T1 replacement of the full-format pre-sequence ------------------------------------------------------------
// p68_fx's main part replaces every register base equal to the converted EA's An by T1 (sequence uops). The
// sequencers apply it where each base is chosen (ts: the replacement is on, tr: that An), from their parameter registers,
// instead of DS comparing the merged uop. tbs: a base register number; ea_lat: ea_la with the replacement.
function [4:0] tbs;
  input [4:0] b; input ts; input [2:0] tr;
  begin tbs = (ts && b == {2'b01, tr}) ? 5'd17 : b; end           // register 17 = T1
endfunction
function [`LA_W-1:0] ea_lat;
  input [2:0] m; input [2:0] r; input [1:0] s; input [15:0] x0; input [15:0] x1; input [3:0] po; input ts; input [2:0] tr;
  reg [`LA_W-1:0] v;
  begin
    v = ea_la(m, r, s, x0, x1, po);
    if (ts && m >= 3'd2 && m <= 3'd6 && r == tr) v[`L_BREG] = 5'd17;   // an An-based mode on that An: base T1
    ea_lat = v;
  end
endfunction

// Operand A of a source EA: {ASEL, AREG, IMM} (value version, for the sequencers)
function [38:0] ea_src;
  input [2:0] m; input [2:0] r; input [1:0] s; input [15:0] x0; input [15:0] x1;
  begin
    if (m == 3'd0)                   ea_src = {`AS_REG, 2'b00, r, 32'h0};
    else if (m == 3'd1)              ea_src = {`AS_REG, 2'b01, r, 32'h0};
    else if (m == 3'd7 && r == 3'd4) ea_src = {`AS_IMM, 5'h0, (s == `SZ_L) ? {x0, x1} : {16'h0, x0}};
    else                             ea_src = {`AS_MEM, 5'h0, 32'h0};
  end
endfunction

// ---- legality predicates and instruction length of the integer groups ----------------------------------------------
// The decoders (dec_imm ... dec_movep) and the fetch predecode (pd_int, used by pdec in p68_fetch.v) share these, so
// length and sequence flag always agree with the decoder's hit.
function ea_alt;       // data alterable: Dn or alterable memory
  input [2:0] m; input [2:0] r;
  ea_alt = (m == 3'd0) || (m >= 3'd2 && m <= 3'd6) || (m == 3'd7 && r <= 3'd1);
endfunction
function ea_bt;        // BTST source: Dn, memory, PC-relative (no immediate)
  input [2:0] m; input [2:0] r;
  ea_bt = (m == 3'd0) || (m >= 3'd2 && m <= 3'd6) || (m == 3'd7 && r <= 3'd3);
endfunction
function ea_tst;       // TST source: any data mode, An for word and long
  input [2:0] m; input [2:0] r; input [1:0] s;
  ea_tst = (m == 3'd0) || (m == 3'd1 && s != `SZ_B) || (m >= 3'd2 && m <= 3'd6) || (m == 3'd7 && r <= 3'd4);
endfunction

function p2_imm;       // ORI ANDI SUBI ADDI EORI CMPI (not to CCR/SR); .L with abs.L (five words), CMPI PC-relative
  input [15:0] w;
  begin
    // line 0, bit 8 clear, op not 100 (static bit) or 111 (MOVES), size not 11; Dn or alterable memory, or CMPI (op 110)
    // with (d16,PC) / (d8,PC,Xn)
    p2_imm = (w[15:12] == 4'h0) && !w[8] && (w[11:9] != 3'd4) && (w[11:9] != 3'd7) && (w[7:6] != 2'b11) &&
             (ea_alt(w[5:3], w[2:0]) || (w[11:9] == 3'd6 && w[5:3] == 3'd7 && (w[2:0] == 3'd2 || w[2:0] == 3'd3)));
  end
endfunction
function p2_bdyn;      // BTST BCHG BCLR BSET Dn,<ea>
  input [15:0] w;
  begin
    // line 0, bit 8 set, not EA mode 1 (that is MOVEP); BTST (type 00) may read PC-relative
    p2_bdyn = (w[15:12] == 4'h0) && w[8] && (w[5:3] != 3'd1) &&
              ((w[7:6] == 2'b00) ? ea_bt(w[5:3], w[2:0]) : ea_alt(w[5:3], w[2:0]));
  end
endfunction
function p2_bsta;      // BTST BCHG BCLR BSET #n,<ea>
  input [15:0] w;
  begin
    p2_bsta = (w[15:8] == 8'h08) && ((w[7:6] == 2'b00) ? ea_bt(w[5:3], w[2:0]) : ea_alt(w[5:3], w[2:0]));
  end
endfunction
function p2_movep;     // MOVEP: line 0, bit 8 set, EA mode field 001
  input [15:0] w;
  p2_movep = (w[15:12] == 4'h0) && w[8] && (w[5:3] == 3'd1);
endfunction
function p2_mshift;    // memory shifts and rotates (word, count 1)
  input [15:0] w;
  p2_mshift = (w[15:12] == 4'hE) && !w[11] && (w[7:6] == 2'b11) && (w[5:3] >= 3'd2) && ea_alt(w[5:3], w[2:0]);
endfunction
function p2_unary;     // NEGX CLR NEG NOT TST
  input [15:0] w;
  begin
    // line 4, size not 11; kind 0/2/4/6 (NEGX, CLR, NEG, NOT) need a data alterable EA, kind A (TST) a TST source
    p2_unary = (w[15:12] == 4'h4) && (w[7:6] != 2'b11) &&
               ((w[11:8] == 4'h0 || w[11:8] == 4'h2 || w[11:8] == 4'h4 || w[11:8] == 4'h6) ? ea_alt(w[5:3], w[2:0]) :
                (w[11:8] == 4'hA) ? ea_tst(w[5:3], w[2:0], w[7:6]) : 1'b0);
  end
endfunction
function p2_tas;       // TAS <ea> (0x4AC0)
  input [15:0] w;
  p2_tas = (w[15:6] == 10'b0100101011) && ea_alt(w[5:3], w[2:0]);
endfunction
function p2_nop;       // NOP
  input [15:0] w;
  p2_nop = (w == 16'h4E71);
endfunction
function p2_ext;       // EXT.W EXT.L EXTB.L SWAP
  input [15:0] w;
  // 0x4880, 0x48C0, 0x49C0, 0x4840 (any Dn in bits 2-0)
  p2_ext = (w[15:3] == 13'b0100100010000) || (w[15:3] == 13'b0100100011000) || (w[15:3] == 13'b0100100111000) ||
           (w[15:3] == 13'b0100100001000);
endfunction
function p2_exg;       // sequence
  input [15:0] w;
  // line C, bit 8, opmode 01000 (Dx,Dy), 01001 (Ax,Ay), 10001 (Dx,Ay)
  p2_exg = (w[15:12] == 4'hC) && w[8] && (w[7:3] == 5'b01000 || w[7:3] == 5'b01001 || w[7:3] == 5'b10001);
endfunction
function p2_scc;       // Scc <ea>: line 5, size 11, not mode 1 (DBcc), data alterable
  input [15:0] w;
  p2_scc = (w[15:12] == 4'h5) && (w[7:6] == 2'b11) && (w[5:3] != 3'd1) && ea_alt(w[5:3], w[2:0]);
endfunction
function p2_dbcc;      // DBcc Dn,label: line 5, size 11, mode 1
  input [15:0] w;
  p2_dbcc = (w[15:12] == 4'h5) && (w[7:3] == 5'b11001);
endfunction
function p2_xrr;       // ADDX SUBX ABCD SBCD, data registers
  input [15:0] w;
  p2_xrr = (((w[15:12] == 4'hD || w[15:12] == 4'h9) && w[8] && w[7:6] != 2'b11 && w[5:3] == 3'd0) ||
            ((w[15:12] == 4'hC || w[15:12] == 4'h8) && w[8:3] == 6'b100000));
endfunction
function p2_xmm;       // ADDX SUBX ABCD SBCD -(Ay),-(Ax) and CMPM (Ay)+,(Ax)+: sequence
  input [15:0] w;
  p2_xmm = (((w[15:12] == 4'hD || w[15:12] == 4'h9 || w[15:12] == 4'hB) && w[8] && w[7:6] != 2'b11 && w[5:3] == 3'd1) ||
            ((w[15:12] == 4'hC || w[15:12] == 4'h8) && w[8:3] == 6'b100001));
endfunction
function p2_nbcd;      // NBCD <ea> (0x4800)
  input [15:0] w;
  p2_nbcd = (w[15:6] == 10'b0100100000) && ea_alt(w[5:3], w[2:0]);
endfunction
function p2_roxr;       // ROXL/ROXR Dx,Dy: sequence
  input [15:0] w;
  p2_roxr = (w[15:12] == 4'hE) && (w[7:6] != 2'b11) && w[5] && (w[4:3] == 2'b10);   // register count, type ROX
endfunction
function p2_pk;        // PACK UNPK: sequence
  input [15:0] w;
  p2_pk = (w[15:12] == 4'h8) && w[8] && (w[7:4] == 4'b0100 || w[7:4] == 4'b1000);
endfunction

// {hit, sequence, length in words} of an opcode word (the fetch predecode of the groups above)
function [5:0] pd_int;
  input [15:0] w;
  reg [2:0] m, r; reg [1:0] s;
  begin
    m = w[5:3]; r = w[2:0]; s = w[7:6];
    pd_int = 6'd0;
    // ORI..CMPI: opcode, one or two immediate words, the EA's words (eax with SZ_W: the EA is never #imm here)
    if (p2_imm(w))        pd_int = {2'b10, 4'd1 + ((s == `SZ_L) ? 4'd2 : 4'd1) + {2'b0, eax(m, r, `SZ_W)}};
    else if (p2_bdyn(w))  pd_int = {2'b10, 4'd1 + {2'b0, eax(m, r, `SZ_B)}};
    else if (p2_bsta(w))  pd_int = {2'b10, 4'd2 + {2'b0, eax(m, r, `SZ_B)}};     // plus the bit number word
    else if (p2_movep(w)) pd_int = {2'b11, 4'd2};                               // sequence; opcode and d16
    else if (p2_mshift(w)) pd_int = {2'b10, 4'd1 + {2'b0, eax(m, r, `SZ_W)}};
    else if (p2_unary(w)) pd_int = {2'b10, 4'd1 + {2'b0, eax(m, r, s)}};       // TST #imm: size matters
    else if (p2_tas(w))   pd_int = {2'b10, 4'd1 + {2'b0, eax(m, r, `SZ_B)}};
    else if (p2_nop(w) || p2_ext(w) || p2_xrr(w)) pd_int = {2'b10, 4'd1};       // one word, one uop
    else if (p2_exg(w) || p2_xmm(w)) pd_int = {2'b11, 4'd1};                    // one word, sequence
    else if (p2_scc(w))   pd_int = {2'b10, 4'd1 + {2'b0, eax(m, r, `SZ_B)}};
    else if (p2_dbcc(w))  pd_int = {2'b10, 4'd2};                               // opcode and displacement
    else if (p2_nbcd(w))  pd_int = {2'b10, 4'd1 + {2'b0, eax(m, r, `SZ_B)}};
    else if (p2_pk(w))    pd_int = {2'b11, 4'd2};                               // opcode and adjustment
    else if (p2_roxr(w))  pd_int = {2'b11, 4'd1};
  end
endfunction
