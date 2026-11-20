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
// ==== p68_exi.v: the extra integer unit (operations 20-31) ===========================================================
//
// Abbreviations: EX = the execute stage (p68_ex.v, which instantiates this unit), AG = address generation, BCD = binary
// coded decimal (two decimal digits in one byte, one per 4-bit nibble, so $42 means forty-two), X = the extend flag.
//
// This unit holds the integer operations that do not fit the adder, the logic operations or the rotator:
//   - Scc (set a byte to all ones when a condition holds): this unit gives 0, and EX ORs the condition in last, so the
//     late condition is not in front of this unit's logic.
//   - The BCD arithmetic ABCD, SBCD and NBCD, one byte step per uop (DS makes these instructions three uops).
//   - The count step of ROXL/ROXR with a register count (OP_ROXC): the count reduced modulo the ring size, encoded for
//     the rotator in EX.
//   - The bit operations BTST, BCHG, BCLR and BSET (Z = the tested bit was 0).
//   - PACK and UNPK (68020: convert between unpacked BCD, one digit per byte, and packed BCD, two digits per byte, with
//     an adjustment constant added), and the byte combine of MOVEP and PACK from memory.
//
// It is pure logic with no clock. Its operands are EX's operand registers without WB forwarding (AG keeps these uops
// two behind the uop that writes their registers), which kept the paths through it short.
//
// BCD from first principles: adding two BCD digits in binary can give a nibble above 9 (or a carry out of the nibble);
// adding 6 then skips the six codes $A-$F and gives the right decimal digit with a carry into the next digit.
// Subtraction does the same with a borrow and a subtract of 6. Motorola's manual leaves N and V of the BCD instructions
// undefined, but real chips do something definite, and software and test suites depend on it. The results and flags
// here follow the 68020 as WinUAE's cputester data has it (V cleared, N = bit 7 of the result), which differs from what
// Musashi does; the lockstep bench patches Musashi the same way, so the two references agree.
`include "p68_uop.vh"                // the operation codes OP_*
`default_nettype none
module p68_exi (
  input  wire [4:0]  op,             // the operation (20-31)
  input  wire [1:0]  sz,             // the size
  input  wire [31:0] a, b,           // operands (EX's registers, without WB forwarding)
  input  wire        x,              // the X flag
  input  wire        cond,           // condition of the uop's CC field (not used here; EX ORs it into the Scc result)
  input  wire        left,                // OP_ROXC direction
  output reg  [31:0] ri,                  // the result (Scc gives 0 here; the core ORs the condition in last)
  output reg         c_bcd, v_bcd, n_bcd, // C, V and N of a BCD step
  output wire        z_bit,               // Z of a bit operation: the tested bit was 0
  output wire        z_keep               // NBCD without change (Musashi leaves Z alone); tied to 0
);
// ---- BCD (the 68020 as WinUAE's cputester data has it; V is cleared, N is bit 7 of the result) --------------------
// ABCD: b + a + X. The low digit is added with X and corrected by +6 when above 9; then the high digit is corrected by
// +$60 when the decimal carry goes out of the byte.
wire [4:0]  lo_a  = {1'b0, a[3:0]} + {1'b0, b[3:0]} + {4'b0, x};   // the low digits and X
wire [8:0]  hi_a  = {1'b0, a[7:4]} + {1'b0, b[7:4]};               // the high digits
wire [9:0]  t_a   = {hi_a, 4'b0} + {5'b0, lo_a};                   // the binary sum
wire [9:0]  u_a   = (lo_a > 5'd9) ? t_a + 10'd6 : t_a;             // low digit decimal adjust
wire        car_a = (u_a[9:4] > 6'd9);                             // a decimal carry out of the byte
wire [9:0]  f_a   = car_a ? u_a + 10'h060 : u_a;                   // high digit decimal adjust
// SBCD: b - a - X (10-bit wrapping arithmetic)
wire [9:0]  lo_s  = {6'b0, b[3:0]} - {6'b0, a[3:0]} - {9'b0, x};   // the low digits less X
wire [9:0]  hi_s  = {2'b0, b[7:4], 4'b0} - {2'b0, a[7:4], 4'b0};   // the high digits
wire        neg_s = lo_s[9];                                        // a borrow out of the low digit
wire [9:0]  t_s   = hi_s + lo_s - (neg_s ? 10'd6 : 10'd0);         // low digit decimal adjust
wire [9:0]  d_s   = {2'b0, b[7:0]} - {2'b0, a[7:0]} - {9'b0, x};   // the plain binary difference
wire        brw_s = d_s[9];                                         // ... borrows out of the byte
wire [9:0]  f_s   = brw_s ? t_s - 10'h060 : t_s;                   // high digit decimal adjust
wire [9:0]  c_s   = {2'b0, b[7:0]} - {2'b0, a[7:0]} - (neg_s ? 10'd6 : 10'd0) - {9'b0, x};
wire        car_s = c_s[9];                                         // C of SBCD (as the 68020 forms it)
// NBCD: 0 - b - X
wire [9:0]  hi_n  = 10'd0 - {2'b0, b[7:4], 4'b0};
wire [9:0]  lo_n  = 10'd0 - {6'b0, b[3:0]} - {9'b0, x};
wire [9:0]  t_n   = hi_n + lo_n - (lo_n[9] ? 10'd6 : 10'd0);         // low digit adjust on a borrow
wire        car_n = (t_n[8:4] > 5'd9);                              // the high digit went out of range: a borrow
wire [9:0]  f_n   = car_n ? t_n - 10'h060 : t_n;                   // high digit adjust
assign z_keep = 1'b0;

// C, V and N of the BCD step (Z is formed by EX from the result, with ZM_KEEP over the whole multi-byte chain)
always @* begin
  case (op)
    `OP_ABCD: begin c_bcd = car_a; v_bcd = 1'b0; n_bcd = f_a[7]; end
    `OP_SBCD: begin c_bcd = car_s; v_bcd = 1'b0; n_bcd = f_s[7]; end
    default:  begin c_bcd = car_n; v_bcd = 1'b0; n_bcd = f_n[7]; end
  endcase
end

// ---- ROXL/ROXR count: a mod (n+1), then the rotate amount L-1 (left) or n-k (right); bit 5 set when no rotation ----
// ROX rotates through X, so the ring is n + 1 bits (9, 17 or 33). The register count (0-63) is reduced modulo n + 1 by at
// most three conditional subtracts of 4, 2 and 1 times the ring size, which is far cheaper than a divider.
reg  [5:0] kk;                       // the count modulo n + 1
always @* begin
  kk = a[5:0];
  if (sz == `SZ_B)      begin if (kk >= 6'd36) kk = kk - 6'd36; if (kk >= 6'd18) kk = kk - 6'd18; if (kk >= 6'd9) kk = kk - 6'd9; end
  else if (sz == `SZ_W) begin if (kk >= 6'd34) kk = kk - 6'd34; if (kk >= 6'd17) kk = kk - 6'd17; end
  else if (kk >= 6'd33) kk = kk - 6'd33;
end
wire [5:0] rox_nn = (sz == `SZ_B) ? 6'd8 : (sz == `SZ_W) ? 6'd16 : 6'd32;   // the operand size n
wire [5:0] rox_kl = kk - 6'd1, rox_kr = rox_nn - kk;                        // left: k - 1; right: n - k
wire [5:0] rox_enc = (kk == 6'd0) ? 6'h20 : left ? {1'b0, rox_kl[4:0]} : {1'b0, rox_kr[4:0]};   // what EX's rotator expects

// ---- Bit operations --------------------------------------------------------------------------------------------
// The bit number is taken modulo 32 for a register operand (.L) and modulo 8 for a memory byte (.B), as the 68k does.
wire [4:0]  bn   = (sz == `SZ_L) ? a[4:0] : {2'b00, a[2:0]};
wire [31:0] mask = 32'd1 << bn;
assign z_bit = ((b & mask) == 32'h0);   // Z is set when the tested bit was 0 (before BCHG/BCLR/BSET change it)

// ---- PACK, UNPK, byte combine ------------------------------------------------------------------------------------
wire [31:0] pk_s = b + a;            // PACK: the two unpacked digits plus the adjustment; the digits land in bits 11-8 and 3-0
wire [31:0] up_s = {16'h0, 4'h0, b[7:4], 4'h0, b[3:0]} + a;   // UNPK: spread the two digits into two bytes, plus the adjustment

// the result by operation
always @* begin
  case (op)
    `OP_SCC:  ri = 32'h0;
    `OP_ABCD: ri = {24'h0, f_a[7:0]};
    `OP_SBCD: ri = {24'h0, f_s[7:0]};
    `OP_NBCD: ri = {24'h0, f_n[7:0]};
    `OP_BCHG: ri = b ^ mask;         // change (invert) the bit
    `OP_BCLR: ri = b & ~mask;        // clear it
    `OP_BSET: ri = b | mask;         // set it
    `OP_PACK: ri = {24'h0, pk_s[11:8], pk_s[3:0]};
    `OP_UNPK: ri = up_s;
    `OP_CMB:  ri = {b[23:0], a[7:0]};   // shift the bytes collected so far up and append the new one
    `OP_ROXC: ri = {26'h0, rox_enc};
    default:  ri = b;                                              // BTST
  endcase
end
endmodule
`default_nettype wire
