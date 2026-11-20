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
// p68_p7f.vh: predecode and legality functions for CAS, CAS2 and BTST Dn,#imm, and the helpers that find an EA in
// the 68020's full extension format.
//
// Shared by the fetch predecode, the decoders (dec_p7.v) and the full-format pre-sequence p68_fx.v, so that the
// length fetch computes, the decoders' hit and p68_fx's idea of where the extension words are can never disagree.
// Included inside a module body after p68_dec.vh and p68_sysf.vh (uses eax, the EA's extension word count, and ea_mal,
// the memory alterable modes).
//
// The full extension format. On the 68000 the indexed modes (d8,An,Xn) and (d8,PC,Xn) always have one extension word
// (the "brief" format). The 68020 adds a "full" format, chosen by bit 8 of that word: it can suppress the base or the
// index, scale the index by 1, 2, 4 or 8, take a 16- or 32-bit base displacement, and read a pointer from memory
// (memory indirect) with an outer displacement behind it. Such an EA can be up to 5 words long, so an instruction can be
// up to 11 words. The decode window does not hold that many words, and widening it would lengthen the path from the
// queue to DS for every instruction, while the full format is rare. So an instruction with a full-format EA runs first
// through a pre-sequence (p68_fx.v) that computes the EA into a temporary register (HISTORY.md section 6). Fetch keeps
// with each queue word where an indexed extension word would be (pd_fx below), so that "the instruction at the head has
// a full-format EA" is one bit, read like the sequence flag.
// ---------------------------------------------------------------------------------------------------------------------

function p7_cas;       // CAS.B/.W/.L Dc,Du,<ea> (0AC0, 0CC0, 0EC0), memory alterable EA
  input [15:0] w;
  p7_cas = (w[15:11] == 5'b00001) && (w[10:9] != 2'b00) && (w[8:6] == 3'b011) && ea_mal(w[5:3], w[2:0]);
endfunction
function p7_cas2;      // CAS2.W/.L (0CFC, 0EFC)
  input [15:0] w;
  p7_cas2 = (w == 16'h0CFC) || (w == 16'h0EFC);
endfunction
function p7_btimm;     // BTST Dn,#imm (B operand immediate; only BTST allows it)
  input [15:0] w;
  p7_btimm = ((w & 16'hF1FF) == 16'h013C);          // 0000 Dn 100 111 100, any Dn
endfunction
// {hit, sequence, length in words}
function [5:0] pd_p7;
  input [15:0] w;
  begin
    if (p7_cas(w))        pd_p7 = {2'b11, 4'd2 + {2'b0, eax(w[5:3], w[2:0], `SZ_W)}};   // opcode, extension, EA words
    else if (p7_cas2(w))  pd_p7 = {2'b11, 4'd3};                        // opcode and two extension words
    else if (p7_btimm(w)) pd_p7 = {2'b10, 4'd2};                        // one uop: opcode and the immediate word
    else                  pd_p7 = 6'd0;
  end
endfunction

// ---- full extension format (p68_fx) ----
// Words between the opcode and the first extension word of the EA in bits 5-3 (0 for most instructions).
function [1:0] fx_pre;
  input [15:0] w;
  begin
    fx_pre = 2'd0;
    if (w[15:12] == 4'h0 && !w[8])                                              // line 0 without bit 8
      fx_pre = (w[11:8] == 4'h8 || w[7:6] == 2'b11 || w[11:8] == 4'hE) ? 2'd1 :   // bit static, CHK2/CMP2/CAS, MOVES
               (w[7:6] == 2'b10) ? 2'd2 : 2'd1;                                   // ORI..CMPI: immediate word(s)
    else if (w[15:12] == 4'h4 && (w[11:7] == 5'b11000 || (w[11] && w[9:7] == 3'b001)))   // MULx.L/DIVx.L, MOVEM
      fx_pre = 2'd1;                                                              // extension word or register mask
    else if (w[15:12] == 4'hE && w[11])                                           // bit fields
      fx_pre = 2'd1;                                                              // the bit field extension word
    else if ((`P68_FPU != 0) && w[15:9] == 7'b1111001 && (w[8:6] == 3'b000 || w[8:6] == 3'b001))       // FPU command / condition word
      fx_pre = 2'd1;
  end
endfunction
// The opcode has an EA in bits 5-3 (opcode classes whose bits 5-3 are something else are left out).
function fx_hasea;
  input [15:0] w;
  reg [3:0] g;
  begin
    g = w[15:12];                                                             // opcode line
    fx_hasea = (g == 4'h0) || (g == 4'h1) || (g == 4'h2) || (g == 4'h3) || (g == 4'h8) || (g == 4'h9) ||
               (g == 4'hB) || (g == 4'hC) || (g == 4'hD) ||
               (g == 4'h4 && w[11:6] != 6'b111001) ||                         // not 4E40-4E7F
               (g == 4'h5 && w[7:3] != 5'b11111) ||                           // not TRAPcc
               (g == 4'hE && w[7:6] == 2'b11) ||                              // memory shifts, bit fields
               ((`P68_FPU != 0) && w[15:9] == 7'b1111001 && (w[8:6] == 3'b000 || w[8:6] == 3'b100 || w[8:6] == 3'b101 ||
                                          (w[8:6] == 3'b001 && w[5:3] != 3'd1 && w[5:3] != 3'd7)));   // FPU EAs
  end
endfunction
function fx_cand;      // indexed mode: (d8,An,Xn) / full format, or PC-relative indexed
  input [2:0] m; input [2:0] r;
  fx_cand = (m == 3'd6) || (m == 3'd7 && r == 3'd3);
endfunction
function [1:0] fx_msz; // MOVE size from the opcode group
  input [3:0] g;
  fx_msz = (g == 4'h1) ? `SZ_B : (g == 4'h3) ? `SZ_W : `SZ_L;                // line 1 byte, 3 word, 2 long
endfunction
function fx_move;      // MOVE / MOVEA (lines 1-3): the only instructions with a second EA (the destination)
  input [15:0] w;
  fx_move = (w[15:12] == 4'h1) || (w[15:12] == 4'h2) || (w[15:12] == 4'h3);
endfunction
// {ps, pd}: word offset of the source EA's indexed extension word (0 = none) and, for MOVE, of the destination's
// (d8,An,Xn) extension word assuming the source is not in full format. The fetch keeps these per queue word, so
// "the instruction at the head has a full-format EA" is one bit read like the sequence flag (iqf).
function [3:0] pd_fx;
  input [15:0] w;
  reg [1:0] ps, pd;
  begin
    ps = (fx_hasea(w) && fx_cand(w[5:3], w[2:0])) ? 2'd1 + fx_pre(w) : 2'd0;           // behind opcode and pre words
    pd = (fx_move(w) && w[8:6] == 3'd6) ? 2'd1 + eax(w[5:3], w[2:0], fx_msz(w[15:12])) : 2'd0;   // behind the source's
    pd_fx = {ps, pd};
  end
endfunction
