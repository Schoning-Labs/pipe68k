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
// ==== p68_md.v: the multiply and divide unit ==========================================================================
//
// Abbreviations: EX = the execute stage (p68_ex.v, which instantiates this unit), DS = decode, DSP = the FPGA's hard
// multiplier blocks, CCR = the condition code register (flags X N Z V C).
//
// ---- What it does ---------------------------------------------------------------------------------------------------
//
// The 68020 multiplies and divides in several forms: MULU.W and MULS.W (16 x 16 = 32 bits), MULU.L and MULS.L (32 x 32 =
// 32 or 64 bits), DIVU.W and DIVS.W (32 / 16 = 16-bit quotient and 16-bit remainder), and DIVU.L, DIVS.L, DIVUL.L and
// DIVSL.L (32 / 32 or 64 / 32 = 32-bit quotient and remainder). All of them run here. These take far longer than one
// clock, so the unit sits beside EX with a fixed-latency interface: a uop hands over the operands when it passes EX, the
// unit then runs on its own, and DS's sequencer (p68_mdseq.v) issues one empty uop per clock until the result is ready,
// so the uops that read the result (MD_GLO, MD_GHI) never meet an unfinished unit. The latency is counted from the
// capture uop in EX (clock T):
//   multiply: result registers valid in clock T+6   (4 DSP products of 17 x 17 bits, three adds, flags)
//   divide:   result registers valid in clock T+20  (T = the MD_DGO uop; radix-4 restoring divider, 16 clocks)
//
// ---- Multiply from first principles -------------------------------------------------------------------------------
//
// A Gowin DSP block multiplies two 18-bit signed numbers. A 32 x 32 product is built from four smaller products by
// splitting each operand into a high and a low half: (ah * 2^16 + al) * (bh * 2^16 + bl) = ah*bh * 2^32 + (ah*bl +
// al*bh) * 2^16 + al*bl. To handle signed and unsigned multiplies with the same hardware, each operand is first widened
// to 33 bits (with its sign for MULS, with 0 for MULU), so it is always a signed 33-bit number; the low half is then the
// low 16 bits with a 0 on top (17 bits, always positive) and the high half the top 17 bits (signed). Every partial
// product is then a 17 x 17 signed multiply, which fits one DSP. The three adds are spread over three clocks.
//
// ---- Divide from first principles ---------------------------------------------------------------------------------
//
// Long division in base 4: the remainder is shifted left by two bits and the next two dividend bits come in at the
// bottom; then the largest of 0, 1, 2 or 3 times the divisor that still fits is subtracted, and that multiple is the
// next quotient digit (two bits). The three candidate subtractions run in parallel on three carry chains, so each
// clock makes two quotient bits, and 32 bits take 16 clocks. This is "restoring" division (the remainder never goes
// negative), which keeps the result exact. Signed division works on the magnitudes and applies the signs at the end
// (the quotient is negative when the signs differ, the remainder takes the dividend's sign, as on the 68k). The 64-bit
// form overflows when the high half of the dividend is already at least the divisor (the quotient would not fit 32
// bits); the word forms overflow when the quotient does not fit 16 bits. On an overflow the destination registers keep
// their old values (the result uops write them back) and only V is set.
//
// ---- What was tried ------------------------------------------------------------------------------------------------
//
// The plan was a DSP-based divider (a reciprocal estimate refined by Newton-Raphson steps, about 8-10 clocks). Worked
// out in detail, it needed about 20 clocks with the DSP pipelines plus a proof that the error never shows in the last
// bit, which is no faster than the radix-4 restoring divider of 16 clocks. So the radix-4 divider was built: exact, with
// three subtractors per clock. Because the unit sits behind a fixed-latency interface, a faster divider could replace
// it later without touching anything else. The divide-by-zero check (vector 5) is a register (md_dz), raised by a
// separate MD_CHK uop, because a raise condition feeds the global flush and must come from a register.
//
// Runs on the CPU clock clk. The multiplier maps to four DSP blocks.
`include "p68_uop.vh"                // the uop format
`include "p68_md.vh"                 // the multiply/divide operation codes MD_*
`default_nettype none
module p68_md (
  input  wire        clk,            // CPU clock
  input  wire        ex_v,           // a uop is in EX
  input  wire        ex_rep,         // ... it replays (it must not start anything)
  input  wire [`UW-1:0] ex_u,        // ... the uop
  input  wire [31:0] a, b,           // EX's operands (with WB forwarding)
  input  wire [4:0]  ccr,            // the CCR (X and, on an overflow, N Z C are kept)
  output wire [31:0] md_r,           // result of MD_GLO / MD_GHI
  output wire [4:0]  md_ccr,         // CCR after the flag-writing result uop
  output wire        md_dz           // latched divisor is zero (raise condition of MD_CHK, a register)
);
wire        un_mul = (ex_u[`F_UNIT] == `UN_MUL);
wire        un_div = (ex_u[`F_UNIT] == `UN_DIV);
wire [4:0]  op = ex_u[`F_OP];        // MD_* operation
wire [3:0]  cc = ex_u[`F_CC];        // the form: cc[0] signed, cc[1] word operands, cc[2] 64-bit (product or dividend)
wire        go = ex_v && !ex_rep;    // the uop really passes EX
wire        m_start = go && un_mul && (op == `MD_MUL);   // capture the multiply operands
wire        d_ld    = go && un_div && (op == `MD_DLD);   // capture the divisor and the dividend's high half
wire        d_go    = go && un_div && (op == `MD_DGO);   // capture the dividend's low half and start

// ---- Multiply ----------------------------------------------------------------------------------------------------
reg  [32:0] ma, mb;                  // 33-bit operands: sign bit of a signed multiply
reg         mm64, msg;               // a 64-bit product (MULx.L Dh:Dl); signed
// widen each operand to 33 bits: a word operand sign- or zero-extended from bit 15, a long one from bit 31
wire [32:0] ax = cc[1] ? {{17{cc[0] & a[15]}}, a[15:0]} : {cc[0] & a[31], a};
wire [32:0] bx = cc[1] ? {{17{cc[0] & b[15]}}, b[15:0]} : {cc[0] & b[31], b};
always @(posedge clk) if (m_start) begin ma <= ax; mb <= bx; mm64 <= cc[2]; msg <= cc[0]; end   // clock T

// the halves: low 16 bits with a 0 on top (positive), high 17 bits signed
wire signed [16:0] a_l = {1'b0, ma[15:0]}, b_l = {1'b0, mb[15:0]};
wire signed [16:0] a_h = ma[32:16],        b_h = mb[32:16];
reg  signed [33:0] p_ll, p_lh, p_hl, p_hh;       // DSP blocks (17 x 17 signed)
always @(posedge clk) begin                      // T+1: the four partial products
  p_ll <= a_l * b_l; p_lh <= a_l * b_h; p_hl <= a_h * b_l; p_hh <= a_h * b_h;
end
reg  signed [34:0] mid, mid2;        // the middle terms' sum (weight 2^16), and its delayed copy
reg  signed [33:0] q_ll, q_hh;       // the low and high products, delayed to line up
reg  [63:0] u1, res;                 // low + high product; the full product
always @(posedge clk) begin
  mid <= p_lh + p_hl; q_ll <= p_ll; q_hh <= p_hh;                  // T+2
  u1 <= {{30{q_ll[33]}}, q_ll} + {q_hh[31:0], 32'h0}; mid2 <= mid; // T+3: low product + high product * 2^32
  res <= u1 + ({{29{mid2[34]}}, mid2} << 16);                      // T+4: + middle * 2^16
end
// the multiply's flags: V when a 32-bit result does not hold the full product (the high half is not the sign or zero
// extension of the low half), N and Z of the 32- or 64-bit result; a 64-bit product never overflows
wire        mv32 = msg ? (res[63:32] != {32{res[31]}}) : (res[63:32] != 32'h0);
wire        mn   = mm64 ? res[63] : res[31];
wire        mz   = mm64 ? (res == 64'h0) : (res[31:0] == 32'h0);
wire        mv   = mm64 ? 1'b0 : mv32;

// ---- Divide ------------------------------------------------------------------------------------------------------
reg  [31:0] dv, xh_ld;               // MD_DLD: divisor (extended), dividend high
always @(posedge clk) if (d_ld) begin
  dv    <= cc[1] ? {{16{cc[0] & a[15]}}, a[15:0]} : a;   // a word divisor is extended to 32 bits
  xh_ld <= b;
end
reg  [31:0] xl, xh, o_lo, o_hi;      // MD_DGO: dividend (high from Dr, or the extension of the 32-bit dividend), old values
reg         dsg, dw16, d64;          // signed; word form; 64-bit dividend
reg  [4:0]  cn;                      // clocks since MD_DGO (0 = idle)
always @(posedge clk) begin
  if (d_go) begin
    xl <= b; xh <= cc[2] ? xh_ld : {32{cc[0] & b[31]}};   // a 32-bit dividend is sign- or zero-extended to 64 bits
    dsg <= cc[0]; dw16 <= cc[1]; d64 <= cc[2];
    o_lo <= b; o_hi <= xh_ld;                              // the old register values, written back on an overflow
    cn <= 5'd1;
  end else if (cn != 5'd0) cn <= (cn == 5'd19) ? 5'd0 : cn + 5'd1;
end
reg         dz;                      // the divisor is zero
always @(posedge clk) dz <= (dv == 32'h0);               // valid from T+1
assign md_dz = dz;

// cn = 1: magnitudes and signs
wire        nneg = dsg & xh[31], dneg = dsg & dv[31];   // the dividend / the divisor is negative
wire [63:0] xn   = {xh, xl};
reg  [31:0] nh, nl, dd;              // the dividend's magnitude (high, low), the divisor's magnitude
reg         qng, nng;                // the quotient will be negative; the remainder will be negative
always @(posedge clk) begin
  {nh, nl} <= nneg ? (~xn + 64'd1) : xn;
  dd  <= dneg ? (~dv + 32'd1) : dv;
  qng <= nneg ^ dneg; nng <= nneg;
end
// cn = 2: 3 * divisor, overflow of the 64-bit form (high >= divisor), start values
reg  [33:0] d3;                      // 3 * divisor, for the third candidate subtraction
reg         ovf0;                    // the 64-bit form's quotient will not fit 32 bits
reg  [31:0] R, Q;                    // the partial remainder; the dividend bits still to come, then the quotient
// cn = 3 .. 18: radix-4 restoring steps; R < divisor
wire        it = (cn >= 5'd3) && (cn <= 5'd18);
wire [33:0] t  = {R, Q[31:30]};      // the remainder shifted left by two, with the next two dividend bits
wire [34:0] s1 = {1'b0, t} - {3'b000, dd};       // t - d
wire [34:0] s2 = {1'b0, t} - {2'b00, dd, 1'b0};  // t - 2d
wire [34:0] s3 = {1'b0, t} - {1'b0, d3};         // t - 3d
wire        g1 = !s1[34], g2 = !s2[34], g3 = !s3[34];   // ... does not go negative
wire [1:0]  q2 = g3 ? 2'd3 : g2 ? 2'd2 : g1 ? 2'd1 : 2'd0;   // the quotient digit
wire [31:0] rn = g3 ? s3[31:0] : g2 ? s2[31:0] : g1 ? s1[31:0] : t[31:0];   // the new remainder
always @(posedge clk) begin
  d3   <= {2'b00, dd} + {1'b0, dd, 1'b0};
  ovf0 <= (nh >= dd);
  if (cn == 5'd2) begin R <= nh; Q <= nl; end
  else if (it) begin R <= rn; Q <= {Q[29:0], q2}; end   // the dividend bits shift out at the top as quotient bits come in
end
// cn = 19: signs applied, overflow, flags
// A signed quotient may reach $8000 (or $80000000) only when it is negative.
wire        ovf_w = dw16 & (dsg ? (qng ? (Q > 32'h8000) : (Q > 32'h7fff)) : (Q > 32'hffff));
wire        ovf_l = d64 & (ovf0 | (dsg & (qng ? (Q > 32'h80000000) : (Q > 32'h7fffffff))));
wire        ovf   = ovf_w | ovf_l;
wire [31:0] qs = qng ? (~Q + 32'd1) : Q;     // the signed quotient
wire [31:0] rs = nng ? (~R + 32'd1) : R;     // the signed remainder (sign of the dividend)
wire [31:0] dq_v = ovf ? o_lo : dw16 ? {rs[15:0], qs[15:0]} : qs;   // the word form packs remainder:quotient in one register
wire [31:0] dr_v = ovf ? o_hi : rs;
wire        dn_v = qng ? (Q != 32'h0) : (dw16 ? Q[15] : Q[31]);     // N of the quotient
wire        dz_v = (Q == 32'h0);                                    // Z of the quotient

// ---- Result registers (both units; the last one started is selected) --------------------------------------------
reg         ldiv;                    // the last unit started was the divider
always @(posedge clk) if (m_start) ldiv <= 1'b0; else if (d_go) ldiv <= 1'b1;
reg  [31:0] rq, rr;                  // the low result (product low or quotient) and the high result (product high or remainder)
reg         f_n, f_z, f_v, f_k;      // f_k: overflow, N Z C stay
always @(posedge clk) begin
  if (ldiv) begin rq <= dq_v; rr <= dr_v; f_n <= dn_v; f_z <= dz_v; f_v <= ovf; f_k <= ovf; end
  else      begin rq <= res[31:0]; rr <= res[63:32]; f_n <= mn; f_z <= mz; f_v <= mv; f_k <= 1'b0; end
end
assign md_r   = op[1] ? rr : rq;     // MD_GHI (op 6) reads the high result, MD_GLO (op 5) the low one
assign md_ccr = {ccr[4], f_k ? ccr[3] : f_n, f_k ? ccr[2] : f_z, f_v, f_k ? ccr[0] : 1'b0};   // X kept; C cleared unless overflow

endmodule
`default_nettype wire
