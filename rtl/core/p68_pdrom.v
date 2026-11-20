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
// ==== p68_pdrom.v: the I-cache fill predecode as block-ROM tables ====================================================
//
// What it does. When the I-cache fills a line from memory, every 16-bit word of the line gets a 12-bit predecode, worked
// out as if the word were an opcode: the instruction length in words, whether it is a sequence, whether it may run in
// lane B, whether it is a simple memory instruction or a branch, and where a full-format extension word would sit (the
// functions are in p68_pdf.vh). The predecode is stored in the cache next to the word, so later, when fetch reads the
// word into its queue, the length is already there and fetch never has to decode an opcode on its critical path.
// This module computes the predecode for the four words of one 64-bit fill beat, on the CPU clock. Its input w is the
// beat as it comes from the memory port (r_data in p68_fetch.v); its output pd is ready two clocks later and goes to
// p68_icache.v, which writes it into the line buffer's predecode (lbp) and from there into the cache.
//
// How it works. As logic, the fill predecode of four words was about 1,100 LUTs. A function of a 16-bit word can be a
// table of 65,536 entries instead, but that table at 12 bits is 768 Kbit, far too big. The trick is that most of the
// 16 bits only matter in a few combinations. So there are two table levels:
//   - T1 (1024 x 7 bits) takes the top 10 bits of the word (15-6) and gives a class number. Two rows of 64 words (one
//     row is one value of bits 15-6, with all 64 values of bits 5-0) get the same class when all 64 predecodes are equal.
//     Over all 1024 rows there are only 75 different rows (counting both FPU settings), so 7 bits of class are
//     enough.
//   - T2 (8192 x 12 bits) takes the class and the low 6 bits of the word (5-0, the effective address field of most
//     opcodes) and gives the predecode.
// Each table is a synchronous read (the output is registered), which is how Gowin's synthesizer recognizes it as block
// RAM used as ROM (the contents come from the initial block in p68_pdrom.vh). Two table reads in a row make the two
// clocks of latency. One block RAM port reads one word, and a Gowin block RAM has two ports, so one instance of p68_pdr2
// handles two words and the module has two instances for the four words of a beat.
//
// History. This saved about 1,150 LUTs in fetch for 28 small block RAMs. The first time it was built it was taken out
// again, because those 28 blocks pushed the full A1200 system to 367 block RAMs of the 340 the chip has. It came back
// (saving 1,055 LUTs) once the system's own block RAM use had been reduced. A version with 14 true dual-port block RAM
// primitives instead of 28 inferred ROMs was also written, but Gowin's simulation model of that primitive was far too
// slow to check it (25 minutes for 1,100 words without finishing, and the first output it did give did not match), so
// the inferred ROMs stayed. The predecode got one more clock of delay with the tables (two clocks instead of one), which
// p68_icache.v covers by holding a line-buffer hit until the predecode of the last beat has arrived.
//
// Regenerating. The tables are written by tools/gen_pdrom.py from the functions in p68_pdf.vh. Rerun it after any change
// to those functions (or to the decode helpers they call); the lockstep bench then checks the result in simulation.
`include "p68_uop.vh"
`default_nettype none
module p68_pdrom (
  input  wire        clk,                // CPU clock
  input  wire [63:0] w,                  // one fill beat: four 16-bit words, word +0 in bits 63:48 (big-endian)
  output wire [47:0] pd                  // their predecodes, 12 bits each in the same order, two clocks after w
);
p68_pdr2 i_a (.clk(clk), .a(w[63:48]), .b(w[47:32]), .qa(pd[47:36]), .qb(pd[35:24]));   // words 0 and 1 of the beat
p68_pdr2 i_b (.clk(clk), .a(w[31:16]), .b(w[15:0]),  .qa(pd[23:12]), .qb(pd[11:0]));    // words 2 and 3
endmodule

// two words: one dual-port read of each table (port a and port b of the same block RAMs)
module p68_pdr2 (
  input  wire        clk,
  input  wire [15:0] a, b,               // two words
  output reg  [11:0] qa, qb              // their predecodes, two clocks later
);
reg  [6:0]  t1 [0:1023];                 // level 1: bits 15-6 of a word -> row class (75 classes used)
reg  [11:0] t2 [0:8191];                 // level 2: {class, bits 5-0} -> 12-bit predecode
`include "p68_pdrom.vh"                  // the generated initial block that fills t1 and t2
reg  [6:0]  ca, cb;                      // class of each word (T1's registered read output)
reg  [5:0]  la, lb;                      // the low 6 bits, delayed one clock to line up with the class
always @(posedge clk) begin
  ca <= t1[a[15:6]]; cb <= t1[b[15:6]]; la <= a[5:0]; lb <= b[5:0];   // clock 1: class lookup (block RAM read)
  qa <= t2[{ca, la}]; qb <= t2[{cb, lb}];                             // clock 2: predecode lookup (block RAM read)
end
endmodule
`default_nettype wire
