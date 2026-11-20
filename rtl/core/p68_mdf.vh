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
// p68_mdf.vh: legality and predecode of MULU, MULS, DIVU and DIVS (word and long forms).
//
// Shared by the fetch predecode and the decoder dec_md.v, so the instruction length fetch computes and the decoder's
// hit can never disagree. Included inside a module body (after p68_dec.vh, for eax). Encodings are in dec_md.v.
// ---------------------------------------------------------------------------------------------------------------------
function md_ea;        // data addressing modes: Dn, (An), (An)+, -(An), d16(An), d8(An,Xn), abs.W, abs.L, PC-relative, #imm
  input [2:0] m; input [2:0] r;
  md_ea = (m == 3'd0) || (m >= 3'd2 && m <= 3'd6) || (m == 3'd7 && r <= 3'd4);   // everything but An
endfunction
function md_w;         // MULU.W MULS.W DIVU.W DIVS.W <ea>,Dn
  input [15:0] w;
  md_w = ((w[15:12] == 4'hC) || (w[15:12] == 4'h8)) && (w[7:6] == 2'b11) && md_ea(w[5:3], w[2:0]);   // lines C, 8; size 11
endfunction
function md_l;         // MULx.L (4C00-4C3F), DIVx.L / DIVxL.L (4C40-4C7F); the extension word follows the opcode
  input [15:0] w;
  md_l = (w[15:7] == 9'b010011000) && md_ea(w[5:3], w[2:0]);
endfunction
function [5:0] pd_md;  // {hit, sequence, length in words}
  input [15:0] w;
  begin
    pd_md = 6'd0;
    if (md_w(w))      pd_md = {2'b11, 4'd1 + {2'b0, eax(w[5:3], w[2:0], `SZ_W)}};   // opcode and EA words (#imm: 1 word)
    else if (md_l(w)) pd_md = {2'b11, 4'd2 + {2'b0, eax(w[5:3], w[2:0], `SZ_L)}};   // opcode, extension, EA words
  end
endfunction
