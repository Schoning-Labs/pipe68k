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
// p68_flowf.vh: predecode of the program flow instructions JMP, JSR, PEA, BSR, RTS, RTD, RTR, LINK and UNLK.
//
// The fetch unit predecodes every instruction word as it enters the instruction queue: it needs to know the length of
// each instruction (to find where the next one starts) and whether it is a sequence (more than one uop). The decoders
// in DS (the decode stage) decide the same things again from the same opcode. If the two ever disagreed, fetch would
// cut the instruction stream in the wrong place, so both use the same function: this file is included by the fetch
// predecode and by dec_flow.v. It is included inside a module body after p68_dec.vh and p68_sysf.vh, because it uses
// eax (extension words of an EA) and ea_ctl (control addressing modes) from them.
//
// The result is {hit, sequence, length in words}. JMP, JSR, BSR and RTS are single uops; PEA, RTR, RTD, UNLK and LINK
// are sequences of class SQ_FLOW (see dec_flow.v for the encodings and seq_flow.v for the steps).
// ---------------------------------------------------------------------------------------------------------------------
function [5:0] pd_flow;
  input [15:0] w;                                                 // the opcode word
  reg [2:0] m, r;                                                 // EA mode and register
  begin
    m = w[5:3]; r = w[2:0];
    pd_flow = 6'h0;                                               // default: not a flow instruction
    if (w[15:7] == 9'b010011101 && ea_ctl(m, r))                 // JSR (4E80), JMP (4EC0)
      pd_flow = {2'b10, 4'd1 + {2'b00, eax(m, r, `SZ_L)}};       // one uop; opcode plus the EA's extension words
    else if (w[15:6] == 10'b0100100001 && ea_ctl(m, r))           // PEA
      pd_flow = {2'b11, 4'd1 + {2'b00, eax(m, r, `SZ_L)}};       // sequence
    else if (w[15:8] == 8'h61)                                    // BSR
      pd_flow = {2'b10, (w[7:0] == 8'h00) ? 4'd2 : (w[7:0] == 8'hFF) ? 4'd3 : 4'd1};   // one uop; 1, 2 or 3 words
    else if (w == 16'h4E75)                                       // RTS (one uop)
      pd_flow = {2'b10, 4'd1};
    else if (w == 16'h4E77)                                       // RTR
      pd_flow = {2'b11, 4'd1};
    else if (w == 16'h4E74)                                       // RTD (opcode and d16)
      pd_flow = {2'b11, 4'd2};
    else if (w[15:3] == 13'b0100111001011)                        // UNLK
      pd_flow = {2'b11, 4'd1};
    else if (w[15:3] == 13'b0100111001010)                        // LINK.W (opcode and d16)
      pd_flow = {2'b11, 4'd2};
    else if (w[15:3] == 13'b0100100000001)                        // LINK.L (opcode and d32)
      pd_flow = {2'b11, 4'd3};
  end
endfunction
