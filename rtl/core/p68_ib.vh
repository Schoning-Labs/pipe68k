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
// ==== p68_ib.vh: one entry of the instruction buffer ==================================================================
//
// The instruction buffer (IB) sits between fetch (IF, p68_fetch.v) and decode (DS, p68_decode.v). Fetch's extractor
// cuts whole instructions out of its queue of 16-bit words and writes each one as an entry of fixed format, up to two
// entries per clock, and DS reads the two oldest entries (E0 and E1, kept in registers) and decodes them. This is the
// way the 68060 does it, and it is the structure that stayed after several other front ends were tried: before the
// buffer, DS read straight from a window over the fetch queue, so every DS issue had to move the window in the same
// clock, and that loop (issue decision, window rotate, next length) was the slowest path of the core. With the buffer
// in between, the window loop dropped from 24.2 to 19.9 ns in the timing estimate, at a cost of about 4,400 more logic
// and a small loss of CPI (clocks per instruction 1.007 to 1.021 on the bench's performance test).
//
// One entry is one instruction: its first five 16-bit words (an opcode plus up to four extension words, which covers
// every instruction except the long full-format addressing modes, which p68_fx reads from the fetch queue itself) and
// a set of bits that fetch and the buffer write logic predecoded, so DS does not have to work them out again. The
// entry is 130 bits wide (`IBW). Bits 94-0 come from fetch; bits 129-95 are added by DS's p68_pdec.v instances when the
// entry is written (decoder gates, legality and data value selects), which takes those decodes off the path from the
// buffer into DS's output register.
`ifndef P68_IB_VH                // include guard
`define P68_IB_VH
`define IB_W     129:0           // the whole entry as a part select
`define IBW      130             // entry width in bits
`define IB_WD    79:0            // the five words: w0 (the opcode) in 15:0, w1 in 31:16, ... w4 in 79:64
`define IB_LEN   83:80           // predecoded length in words (for a full-format EA and an FPU #imm form DS gives the real one)
`define IB_SQ    84              // the instruction is a sequence (several uops, made by a sequencer after the decoder's step 0)
`define IB_FULL  85              // it has a full-format EA (68020 memory indirect or suppressed base/index): p68_fx runs first
`define IB_HOLD  86              // hold entry: fetch must wait for DS's real length (full-format EA, FPU general op with #imm)
`define IB_SIMP  87              // lane-B class: a simple register-only single-uop instruction that may run in the second pipe
`define IB_MEM   88              // memory class: a single-uop instruction with one simple memory operand (may swap into lane A)
`define IB_BR    89              // branch class: Bcc, BRA or DBcc
`define IB_PBTC  90              // this is the branch the branch target cache (BTC) predicted (its target is in fetch's slot 0 or 1)
`define IB_PFIX  91              // a BTC cut lies inside this entry and it is no such branch: OC replays it
`define IB_RTS   92              // RTS ($4E75)
`define IB_CALL  93              // BSR or JSR: pushes the return stack
`define IB_RET   94              // RTS, RTD or RTR: pops the return stack
`define IB_PQ    114:95          // decoder gates and legality (p68_pdec), added by DS when the entry is written
`define IB_SX    129:115         // data value selects {SA_DISP, LA_DISP, IMM}, 5 bits each as {k, kind} (p68_pdec), added by DS
`endif
