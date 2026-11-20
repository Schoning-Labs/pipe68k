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
// ==== p68_exb.v: lane B's ALU, the second execute unit for dual issue ================================================
//
// Abbreviations: EX = the execute stage (p68_ex.v, which instantiates this unit), OC2 = the second operand stage, WB =
// write-back, ALU = arithmetic and logic unit, CCR = the condition code register (flags X N Z V C).
//
// pipe68k can run two instructions per clock. Following the 68060, the second pipe (lane B) is deliberately small: it
// runs only register-only, single-uop instructions with the operations ADD, SUB, AND, OR, EOR, CMP, MOVE and NEG, on
// operand A (a register or an immediate, with the sign extension of A) and operand B (always a register, which is also
// the old destination that a .B or .W result is merged into). AG's pairing check (i2ok in p68_ag.v) lets only such
// instructions into lane B, so this unit needs no shifter, no memory port and no flag modes beyond the "set" forms.
// That is what made dual issue affordable: about 10 % of all instructions on the bench run here, and the core went from
// 33.5 to 70.5 MIPS (clock divided by CPI) while dual issue was built.
//
// It is pure logic with no clock. Its operands come prepared from OC2 (ax and bq already inverted where a subtract or
// negate needs it, as in p68_ex.v), with the WB forwarding of both lanes in front of it in p68_ex.v. It gives the
// result, merged by size, and the flag sources; p68_ex.v registers the final flags at commit and WB completes them
// after lane A's. The adder and the zero test use the same tricks as lane A's, which p68_ex.v explains in more detail.
`include "p68_uop.vh"                // the uop format
`default_nettype none
module p68_exb (
  input  wire [`UW-1:0] u,           // lane B's uop in EX
  input  wire [31:0] a,              // A after forwarding and extension
  input  wire [31:0] b,              // B after forwarding
  input  wire [31:0] ax,             // adder input A (inverted for SUB/CMP)
  input  wire [31:0] bq,             // adder input B (inverted for NEG)
  output wire [31:0] rm,             // result merged by size
  output wire        c8, c16, c32, bor, bm, axm, z_ad, z_l   // carries out of bits 7, 15, 31; borrow type; signs of the
                                     // adder inputs; Z of an adder op; Z of a logic op
);
wire [4:0]  op  = u[`F_OP];          // the operation
wire [1:0]  sz  = u[`F_SZ];          // the size
wire        sub = (op == `OP_SUB) || (op == `OP_CMP);   // b - a = b + ~a + 1
wire        ngb = (op == `OP_NEG);                       // 0 - b = ~b + 1
wire        cin = sub || ngb;                            // the + 1 of the two's complement
// one carry chain with a carry in: the extra low bit 1 + cin carries exactly cin into bit 0
wire [33:0] s2  = {1'b0, bq, 1'b1} + {1'b0, ax, cin};
wire [32:0] sum = s2[33:1];          // the sum and the carry out of bit 31
assign c8  = sum[8] ^ bq[8] ^ ax[8];    // carry into bit 8 = carry out of a byte operation
assign c16 = sum[16] ^ bq[16] ^ ax[16]; // carry out of a word operation
assign c32 = sum[32];                   // carry out of a long operation
assign bor = sub || ngb;             // C is the inverted carry (a borrow) for these
wire        is_ad = (op == `OP_ADD) || sub || ngb;   // the result is the sum
// the logic result: AND, OR, EOR, else MOVE (result = A)
wire [31:0] r_l = (op == `OP_AND) ? (a & b) : (op == `OP_OR) ? (a | b) : (op == `OP_EOR) ? (a ^ b) : a;
wire [31:0] rx  = is_ad ? sum[31:0] : r_l;
// a byte write keeps bits 31-8 of the destination register, a word write bits 31-16 (as on every 68k)
wire [31:0] keep_b = (sz == `SZ_B) ? 32'hFFFFFF00 : (sz == `SZ_W) ? 32'hFFFF0000 : 32'h0;
assign rm  = (keep_b & b) | (~keep_b & rx);
// the signs of the adder inputs at the size (for V: same sign in, different sign out)
assign bm  = (sz == `SZ_B) ? bq[7] : (sz == `SZ_W) ? bq[15] : bq[31];
assign axm = (sz == `SZ_B) ? ax[7] : (sz == `SZ_W) ? ax[15] : ax[31];
// Z of the adder without waiting for the carry chain: the sum is 0 at every bit exactly when each bit's bq ^ ax equals
// the carry into it, and with all lower bits 0 that carry is bq | ax of the bit below (cin for bit 0)
wire [31:0] zt = (bq ^ ax) ^ {bq[30:0] | ax[30:0], cin};
assign z_ad = (sz == `SZ_B) ? (zt[7:0] == 8'h0) : (sz == `SZ_W) ? (zt[15:0] == 16'h0) : (zt == 32'h0);
assign z_l  = (sz == `SZ_B) ? (r_l[7:0] == 8'h0) : (sz == `SZ_W) ? (r_l[15:0] == 16'h0) : (r_l == 32'h0);   // Z of a logic op
endmodule
`default_nettype wire
