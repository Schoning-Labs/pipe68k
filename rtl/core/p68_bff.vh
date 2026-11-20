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
// p68_bff.vh: legality and predecode of the bit field instructions (BFTST BFEXTU BFCHG BFEXTS BFCLR BFFFO BFSET BFINS,
// opcodes E8C0-EFFF).
//
// Shared by the fetch predecode and the decoder dec_bf.v, so the instruction length fetch computes and the decoder's
// hit can never disagree. Included inside a module body after p68_dec.vh (uses eax, the EA's extension word count).
// Opcode layout: 1110 1 type 11 EA mode EA reg, followed by the bit field extension word and the EA's extension words
// (see dec_bf.v).
// ---------------------------------------------------------------------------------------------------------------------
function bf_ok;        // bit field opcode with a legal EA (extension word bit 15 is not checked, as in Musashi)
  input [15:0] w;
  reg [2:0] m, r, t;                                                // EA mode, EA register, type (bits 10-8)
  begin
    m = w[5:3]; r = w[2:0]; t = w[10:8];
    // Dn or a control mode; the read-only types also allow the PC-relative modes (mode 7 registers 2-3)
    bf_ok = (w[15:11] == 5'b11101) && (w[7:6] == 2'b11) &&
            ((t == 3'd0 || t == 3'd1 || t == 3'd3 || t == 3'd5)      // BFTST BFEXTU BFEXTS BFFFO: also PC-relative
               ? (m == 3'd0 || m == 3'd2 || m == 3'd5 || m == 3'd6 || (m == 3'd7 && r <= 3'd3))
               : (m == 3'd0 || m == 3'd2 || m == 3'd5 || m == 3'd6 || (m == 3'd7 && r <= 3'd1)));
  end
endfunction
// {hit, sequence, length in words}: opcode, extension word, EA extension words
function [5:0] pd_bf;
  input [15:0] w;
  pd_bf = bf_ok(w) ? {2'b11, 4'd2 + {2'b0, eax(w[5:3], w[2:0], `SZ_W)}} : 6'd0;
endfunction
