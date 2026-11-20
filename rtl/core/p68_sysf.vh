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
// p68_sysf.vh: functions shared by the fetch predecode and the system instruction decoder dec_sys.v.
//
// Included inside a module body after p68_dec.vh. It holds three classes of addressing modes that the 68k manuals use
// to say which EAs an instruction allows, and the predecode (length and sequence flag) of the system instructions.
// As with the other shared predecode files, fetch and the decoders use the same function, so the instruction length
// fetch computes and the decoders' hit can never disagree.
//
// EA mode numbers (bits 5-3 of the opcode, register in bits 2-0): 0 Dn, 1 An, 2 (An), 3 (An)+, 4 -(An), 5 (d16,An),
// 6 (d8,An,Xn), 7 with register 0 abs.W, 1 abs.L, 2 (d16,PC), 3 (d8,PC,Xn), 4 #immediate.
// ---------------------------------------------------------------------------------------------------------------------

function ea_dat;        // data addressing modes (all but An)
  input [2:0] m; input [2:0] r;
  begin ea_dat = (m != 3'd1) && (m != 3'd7 || r <= 3'd4); end     // every mode except An and mode 7 registers 5-7
endfunction
function ea_mal;        // memory alterable
  input [2:0] m; input [2:0] r;
  begin ea_mal = (m >= 3'd2 && m <= 3'd6) || (m == 3'd7 && r <= 3'd1); end   // memory modes that may be written
endfunction
function ea_ctl;        // control modes
  input [2:0] m; input [2:0] r;
  // memory modes with no side effect and an operand size that does not matter: (An), (d16,An), (d8,An,Xn), abs, PC-rel.
  begin ea_ctl = (m == 3'd2) || (m == 3'd5) || (m == 3'd6) || (m == 3'd7 && r <= 3'd3); end
endfunction

// Predecode of the system instructions whose length is not 1 word or that are sequences: {hit, seq, length}.
// The fetch predecode checks this first (TRAPcc and the SR/CCR immediates would otherwise get Scc/ORI lengths, since
// their encodings sit inside those groups).
function [5:0] pdec_sys;
  input [15:0] w;                                                 // the opcode word
  reg [2:0] m, r;
  begin
    m = w[5:3]; r = w[2:0];
    pdec_sys = 6'h0;
    if (w[15:11] == 5'b01000 && w[8:6] == 3'b011)                        // MOVE from/to SR/CCR
      // a sequence only for MOVE (An)+ / -(An),SR (bits 10-9 = 11); length 1 + the EA's extension words (word operand)
      pdec_sys = {1'b1, w[10:9] == 2'b11 && (m == 3'd3 || m == 3'd4), 4'd1 + {2'b00, eax(m, r, `SZ_W)}};
    else if (w[15:12] == 4'h0 && (w[11:9] == 3'b000 || w[11:9] == 3'b001 || w[11:9] == 3'b101) &&
             w[8:7] == 2'b00 && w[5:0] == 6'b111100)                         // ORI/ANDI/EORI to CCR/SR
      pdec_sys = {2'b10, 4'd2};                                             // opcode and the immediate word
    else if (w == 16'h4E72 || w == 16'h4E7A || w == 16'h4E7B)               // STOP, MOVEC
      pdec_sys = {2'b10, 4'd2};                                             // opcode and one word
    else if (w == 16'h4E73)                                                 // RTE
      pdec_sys = {2'b11, 4'd1};                                             // a sequence of the system sequencer
    else if (w[15:12] == 4'h5 && w[7:3] == 5'b11111 && w[2:0] >= 3'd2 && w[2:0] <= 3'd4)   // TRAPcc
      pdec_sys = {2'b10, (w[2:0] == 3'd2) ? 4'd2 : (w[2:0] == 3'd3) ? 4'd3 : 4'd1};   // .W, .L operand, or none
    else if (w[15:12] == 4'h4 && w[8] && !w[6])                             // CHK.W / CHK.L
      pdec_sys = {2'b11, 4'd1 + {2'b00, eax(m, r, w[7] ? `SZ_W : `SZ_L)}};   // sequence; bit 7: 1 word, 0 long
    else if (w[15:11] == 5'b00000 && w[10:9] != 2'b11 && w[8:6] == 3'b011 && ea_ctl(m, r))   // CHK2/CMP2
      pdec_sys = {2'b11, 4'd2 + {2'b00, eax(m, r, w[10:9])}};              // opcode, extension word, EA words
    else if (w[15:8] == 8'h0E && w[7:6] != 2'b11 && ea_mal(m, r))          // MOVES
      pdec_sys = {2'b11, 4'd2 + {2'b00, eax(m, r, w[7:6])}};               // opcode, extension word, EA words
  end
endfunction
