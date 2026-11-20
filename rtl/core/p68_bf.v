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
// ==== p68_bf.v: the bit field unit ===================================================================================
//
// Abbreviations: EX = the execute stage (p68_ex.v, which instantiates this unit), AG = address generation, EA =
// effective address, CCR = the condition code register (flags X N Z V C).
//
// ---- What a bit field is ------------------------------------------------------------------------------------------
//
// The 68020 added eight instructions that work on a "bit field": a run of 1 to 32 bits that can start at any bit, not
// only at a byte boundary. BFTST tests it, BFEXTU and BFEXTS extract it (zero- or sign-extended), BFCHG, BFCLR and BFSET
// invert, clear or set it, BFINS inserts a value into it, and BFFFO finds its first set bit. The field is given by an
// offset and a width, each either an immediate in the extension word or a data register. The 68k counts the offset from
// the most significant bit: offset 0 is bit 7 of the byte at the EA (or bit 31 of a data register). In memory the offset
// is a signed 32-bit number, so the field can start far before or after the EA; in a register only the low 5 bits count
// and the field wraps around from bit 0 back to bit 31. A field in memory touches at most five bytes, so the unit works
// on a 64-bit window of two longs read from the aligned address.
//
// ---- How the unit works -------------------------------------------------------------------------------------------
//
// UN_BFU uops capture their operands when they commit; the results come from registers 2 to 4 clocks later, so nothing
// here sits in the single-clock EX paths. The sequencer (seq_bf.v) spaces the uops so the
// reads come late enough:
//   uop at EX clock t:   SETUP / SETI write the offset, width and insert value registers
//                        ADDR / COMP capture A and B (cap_a, cap_b) and the operation
//   clock t+1:  pass 1   the 64-bit window {A, B} (register form: {A, A}, the field wraps) rotated left by s + w: the
//                        field right aligned in r1[w-1:0]                    (ADDR: the aligned address into pa)
//   clock t+2:  pass 2   the field replaced in r1 (CHG CLR SET INS) and rotated back into r2: the new window (register
//                        form: r2[31:0]); f = the field (BFINS: the inserted value); fd = BFFFO's count
//                                                                          r2 valid for a uop in EX at t+3 (RD1)
//   clock t+3:  stage 3  q0 (new first long or register, field, sign extension, BFFFO), flags N and Z
//                                                                          valid for a uop in EX at t+4 (ADDR: t+3)
// The window starts at the aligned address; s = bit position of the field inside it (0-31), w = width (1-32).
// The rotator runs pass 1 only in the clock after a capture and pass 2 in every other clock, so every stage register is
// recomputed every clock from held values and a read may come any time later.
//
// ---- What was tried -------------------------------------------------------------------------------------------------
//
// The first unit used two barrel shifters on the 64-bit window, one to bring the field down and one to put it back. It
// took 1,769 LUTs and had a 19-level path. It was rewritten with one 64-bit rotator used twice (rotate right by -(s + w)
// to bring the field down, then by s + w to put the modified window back), which took 1,067 LUTs, cost one more empty
// uop in the sequences and left the bench's performance unchanged. Later the unit stopped taking WB forwarding (AG keeps
// a bit field uop two behind the writer of its registers), which removed it from EX's forwarding paths.
//
// The rotator is its own module (p68_bfrot) with syn_hier = "hard", so synthesis keeps it as one block that both passes
// share instead of copying it into each use. Runs on the CPU clock clk.
`include "p68_uop.vh"                // the uop format and the BF_* operation codes
`default_nettype none
module p68_bf (
  input  wire        clk,            // CPU clock
  input  wire        v,             // a uop is in EX and not replayed
  input  wire        sel,           // it is a UN_BFU uop
  input  wire [4:0]  op,            // BF_* operation, or 8 + type for the compute ops (type = opcode bits 10-8)
  input  wire [3:0]  cc,            // SETUP: {regform, dynamic width, dynamic offset}
  input  wire [9:0]  imm,           // SETUP: the immediate offset (4-0) and width (9-5)
  input  wire [31:0] a, b,          // EX's operand registers (without WB forwarding)
  input  wire [4:0]  ccr,           // the CCR (X kept)
  input  wire [4:0]  fen,           // the uop's flag enables (bit 3: this read writes N and Z)
  output wire [31:0] bf_r,          // the result of RD0 / RD1
  output wire [4:0]  bf_ccr         // the CCR after it
);
wire take = v && sel;               // a bit field uop passes EX

// ---- Parameters -------------------------------------------------------------------------------------------------------
reg  [31:0] o;                      // offset (register form: low 5 bits only)
reg  [31:0] ofull;                  // the unreduced offset (BFFFO returns offset + position, as the 68020 does)
reg  [5:0]  w;                      // width 1-32
reg         regf;                   // register form
reg  [31:0] ins;                    // BFINS value
reg  [4:0]  s;                      // field position in the window
wire [31:0] off_v = cc[0] ? a : {27'h0, imm[4:0]};   // the offset: from a register, or the immediate
wire [4:0]  wd_v  = cc[1] ? b[4:0] : imm[9:5];       // the width: from a register (mod 32), or the immediate
always @(posedge clk) begin
  if (take && op == `BF_SETUP) begin
    o    <= cc[2] ? {27'h0, off_v[4:0]} : off_v;
    ofull <= off_v;
    w    <= (wd_v == 5'd0) ? 6'd32 : {1'b0, wd_v};    // a width of 0 means 32, as on the 68020
    regf <= cc[2];
  end
  if (take && op == `BF_SETI) ins <= a;
end
// values derived from the parameters, registered (valid 2 clocks after the parameter write)
reg  [31:0] lm, insm, ofw;          // low-w-bits mask, inserted value masked, offset + width
reg  [4:0]  wm1;                    // w - 1
always @(posedge clk) begin
  lm   <= 32'hFFFFFFFF >> (6'd32 - w);
  insm <= ins & (32'hFFFFFFFF >> (6'd32 - w));
  ofw  <= ofull + {26'h0, w};
  wm1  <= w[4:0] - 5'd1;
end

// ---- Capture ------------------------------------------------------------------------------------------------------------
reg  [31:0] cap_a, cap_b;           // the window {cap_a, cap_b} (register form: cap_b = A)
reg  [1:0]  cap_k;                  // 1 ADDR, 2 COMP
reg  [2:0]  cap_t;                  // the compute type (0 TST, 1 EXTU, 2 CHG, 3 EXTS, 4 CLR, 5 FFO, 6 SET, 7 INS)
reg         addr_v;                 // the clock after an ADDR capture: s takes the address bits
reg         p1st;                   // the clock after a capture: the rotator runs pass 1
reg  [5:0]  amt;                    // rotate-right amount of this clock: -(s + w) (pass 1), s + w (pass 2)
wire cap_en = take && (op == `BF_ADDR || op[4:3] == 2'b01);   // ADDR or a compute op (8-15)
always @(posedge clk) begin
  if (cap_en) begin cap_a <= a; cap_b <= regf ? a : b; cap_k <= (op == `BF_ADDR) ? 2'd1 : 2'd2; cap_t <= op[2:0]; end
  addr_v <= take && op == `BF_ADDR;
  p1st   <= cap_en;
  amt    <= cap_en ? 6'd0 - {1'b0, s} - w : {1'b0, s} + w;
end

// ---- Passes 1 and 2: one 64-bit rotator -------------------------------------------------------------------------------
reg  [63:0] r1, r2;                 // the window after pass 1 (field at the bottom), after pass 2 (the new window)
wire        modf  = (cap_t == 3'd2) || (cap_t == 3'd4) || (cap_t == 3'd6) || (cap_t == 3'd7);   // CHG, CLR, SET, INS modify
// the field replaced in r1's low bits: CHG inverts (xv), CLR clears (kv), SET sets (sv), INS clears and inserts
// (TST, EXTU, EXTS, FFO: unused); the masks are registers, valid in pass 2
reg  [31:0] kv, sv, xv;
always @(posedge clk) begin
  kv <= (cap_t == 3'd4 || cap_t == 3'd7) ? lm : 32'h0;
  sv <= (cap_t == 3'd6) ? lm : (cap_t == 3'd7) ? insm : 32'h0;
  xv <= (cap_t == 3'd2) ? lm : 32'h0;
end
wire [31:0] mlo   = ((r1[31:0] & ~kv) | sv) ^ xv;            // the low long with the field modified
// the rotator input: the captured window for pass 1, the modified window for pass 2 (register form: both halves the
// same long, so a field that wraps from bit 0 to bit 31 comes out right)
wire [63:0] rin   = p1st ? {cap_a, cap_b} : {regf ? mlo : r1[63:32], mlo};
wire [63:0] rot;                                                         // rotate right by amt (one rotator for both passes)
p68_bfrot i_rot (.x(rin), .a(amt), .y(rot));
wire [31:0] fr1   = r1[31:0] & lm;                                       // field right aligned
wire [31:0] fld   = (cap_t == 3'd7) ? insm : fr1;                        // BFINS: the value (flags)
// leading zeros of the field (fr1 != 0): per byte, then the first nonzero byte (shallow tree)
function [2:0] lz8; input [7:0] v;
  begin lz8 = v[7] ? 3'd0 : v[6] ? 3'd1 : v[5] ? 3'd2 : v[4] ? 3'd3 : v[3] ? 3'd4 : v[2] ? 3'd5 : v[1] ? 3'd6 : 3'd7; end
endfunction
wire [3:0]  bnz = {|fr1[31:24], |fr1[23:16], |fr1[15:8], |fr1[7:0]};   // which bytes are nonzero
wire [4:0]  clz = bnz[3] ? {2'd0, lz8(fr1[31:24])} : bnz[2] ? {2'd1, lz8(fr1[23:16])} :
                  bnz[1] ? {2'd2, lz8(fr1[15:8])} : {2'd3, lz8(fr1[7:0])};
reg  [31:0] f, pa;                  // the field (or the inserted value); the aligned address of a memory field
reg  [5:0]  fd;                     // BFFFO: zeros in front of the first one of the left-aligned field, minus w
// the aligned address: EA + (offset >> 3, arithmetic, so a negative offset reaches below the EA), rounded down to a long
wire [31:0] bsum = cap_a + {{3{o[31]}}, o[31:3]};                        // EA + (offset >> 3, arithmetic)
always @(posedge clk) begin
  if (p1st) r1 <= rot; else r2 <= rot;
  f  <= fld;
  // The field sits right-aligned, so its leading zeros counted in 32 bits are clz = (32 - w) + (zeros in the field).
  // BFFFO's result is offset + zeros in the field = offset + w + (clz - 32); stored here as clz - 32 (6-bit signed,
  // since clz < 32), and stage 3 adds it to offset + w. An all-zero field gives 0 here, so the result is offset + w.
  fd <= (bnz == 4'h0) ? 6'd0 : {1'b1, clz};                              // clz - 32 (clz < 32); field 0: 0
  pa <= {bsum[31:2], 2'b00};
end
wire [4:0]  s_n = {bsum[1:0], o[2:0]};   // the field's bit position inside the long at pa
always @(posedge clk) begin                                              // s: register form from SETUP, memory form from ADDR
  if (take && op == `BF_SETUP && cc[2]) s <= off_v[4:0];
  else if (addr_v) s <= s_n;
end

// ---- Stage 3 ------------------------------------------------------------------------------------------------------------
wire        fn = f[wm1];                                                 // the field's top bit
reg  [31:0] q0;                     // result 0
reg         n_r, z_r;               // N (the field's top bit) and Z (the field is zero) of the field before the change
always @(posedge clk) begin
  n_r <= fn;
  z_r <= (f == 32'h0);
  q0  <= (cap_k == 2'd1) ? pa :                                          // BF_ADDR: the aligned address
         modf ? (regf ? r2[31:0] : r2[63:32]) :                          // the modified register or first long
         (cap_t == 3'd3) ? (fn ? (f | ~lm) : f) :                        // BFEXTS
         (cap_t == 3'd5) ? ofw + {{26{fd[5]}}, fd} : f;                  // BFFFO; BFEXTU, BFTST
end

assign bf_r   = (op == `BF_RD1) ? r2[31:0] : q0;                         // RD1: the new second long
assign bf_ccr = fen[3] ? {ccr[4], n_r, z_r, 2'b00} : ccr;                // N, Z from the field; V and C cleared; X kept
endmodule
// 64-bit rotate right (log rotator), its own module so both passes share it. Six stages rotate by 1, 2, 4, 8, 16 and 32
// bits under the bits of a.
module p68_bfrot (input wire [63:0] x, input wire [5:0] a, output reg [63:0] y) /* synthesis syn_hier = "hard" */;
integer j;
always @* begin
  y = x;
  for (j = 0; j < 6; j = j + 1) if (a[j]) y = (y >> (1 << j)) | (y << (64 - (1 << j)));
end
endmodule
`default_nettype wire
