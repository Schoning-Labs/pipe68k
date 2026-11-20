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
// dec_lea: the group decoder for LEA (load effective address, line 4).
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v): it sees the opcode word w0 and the
// words behind it, says whether the opcode is its own (hit), and gives one micro-op (uop, p68_uop.vh) that DS ORs with
// the other decoders' uops. The uop then runs through DR (register stage before address generation), AG (address
// generation), OC (operand cycle), EX (execute) and WB (write-back). Combinational logic only.
//
// The instruction.
//
//     0100 | An | 111 | EA mode | EA reg        LEA <ea>,An
//
// LEA computes an address and puts it into An without reading memory. Only the control addressing modes are allowed,
// the ones that name a place in memory without a side effect: (An), (d16,An), (d8,An,Xn), abs.W, abs.L, (d16,PC) and
// (d8,PC,Xn). No flags change.
//
// How it becomes a uop. AG already has the adder that computes addresses, so LEA never goes to EX at all: the uop has
// the address described as a load (LA) but with the load itself switched off, and the LEA flag tells AG to write that
// address into the register named in SA_BREG. The value is then ready for the next instruction's address straight from
// AG, without the trip through EX and WB. The same LEA uop is used by the sequencers to put a computed address into a
// temporary register (T0-T2). In dual issue, a LEA at the head runs in AG; as the second instruction of a pair it may
// run in lane B.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module dec_lea (
  input  wire [15:0] w0, w1, w2, w3,         // opcode word and the next words (EA extension words)
  output wire        hit,                    // LEA with a control EA
  input  wire        pg,             // predecoded gate (this decoder's ug from p68_pdec, a register in the entry)
  output wire        ug,             // the gate of the uop, for p68_pdec's own copy of this decoder
  output wire [`UW-1:0] u                    // the uop (zero unless pg)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
wire [2:0] r9 = w0[11:9], m3 = w0[5:3], r0 = w0[2:0];         // destination An, EA mode, EA register
// line 4, bits 8-6 = 111, and a control mode: (An), (d16,An), (d8,An,Xn), or mode 7 registers 0-3 (abs and PC-relative)
wire       h = (w0[15:12] == 4'h4) && (w0[8:6] == 3'b111) &&
               (m3 == 3'd2 || m3 == 3'd5 || m3 == 3'd6 || (m3 == 3'd7 && r0 <= 3'd3));
reg  [`UW-1:0] v;
always @* begin
  v = {`UW{1'b0}};
  v[`F_LA] = ea_lac(m3, r0, `SZ_L, 2'd1, 4'd2);                // the address computation of the EA
  v[`F_LA_EN] = 1'b0;                                          // but no memory read
  v[`F_LEA] = 1'b1; v[`F_SA_BREG] = {2'b01, r9};               // AG writes the address into An (8 + r9)
end
assign hit = h;
// The uop is gated by pg, this decoder's gate predecoded with the instruction buffer entry (p68_pdec).
assign ug   = h;
assign u   = pg ? v : {`UW{1'b0}};
endmodule
`default_nettype wire
