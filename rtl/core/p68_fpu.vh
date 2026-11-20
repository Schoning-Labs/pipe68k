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
// ==== p68_fpu.vh: FPU operation codes, sequence kinds and uop bits ===================================================
//
// Abbreviations: FPU = floating-point unit (p68_fpu.v with its engine p68_fpe.v), EX = execute, OC2 = the second operand
// stage, DS = decode, FPn = one of the eight floating-point registers FP0-FP7, FPCR/FPSR/FPIAR = the FPU's control,
// status and instruction address registers, BSUN = the "branch or set on unordered" exception, NaN = not a number.
//
// The FPU is unit 10 of EX lane A. Every FPU instruction becomes a DS sequence (class SQ_FPU): dec_fpu.v decodes it and
// gives step 0 and the parameters, seq_fpu.v gives the later uops. The UN_FPU uops below are the only way the sequences
// talk to the unit: the unit takes operand A when a uop passes EX, and gives its results from its own registers. This
// file is included at file level after p68_uop.vh (docs/learn/HISTORY.md, section 13, has the design and its history).
//
// History in short: the first FPU had every operation in one unit on the core clock, and the core with it ran at 53 MHz;
// the arithmetic was then moved into an engine that steps once every P68_FPN clocks on a clock enable (multicycle
// constraints give its paths that many clocks), and later rewritten to about half its size. The codes here stayed the
// same through all of it.
`ifndef P68_FPU_VH                       // include guard
`define P68_FPU_VH
`define UN_FPU   4'd10                   // the FPU's unit number in the uop's UNIT field
`define SQ_FPU   4'd14                   // the sequence class of every FPU instruction (seq_fpu.v)
// UN_FPU operations (the unit takes operand A when the uop commits; results come from its registers)
`define FP_IN    5'd0    // operand word CC[1:0] into the input staging register; CC[2]: then FPn (IMM[2:0]) <- staging raw (FMOVEM.X)
`define FP_GO    5'd1    // start: IMM[15:0] = the command word (opclass 000/010: operation, 011: FMOVE out conversion)
`define FP_WR    5'd2    // commit the operation: FPn, FPSR, FPIAR (last uop)
`define FP_RD    5'd3    // result = output word CC[1:0] of the FMOVE out conversion; CC[2]: commit FPSR (last store)
`define FP_CW    5'd4    // control register CC[2:0] (FPCR, FPSR, FPIAR one-hot) <- A
`define FP_CR    5'd5    // result = control register CC[2:0]
`define FP_RX    5'd6    // result = word CC[1:0] of FPn (IMM[2:0]) raw (FMOVEM.X store)
`define FP_SV    5'd7    // result = FSAVE frame word CC[3:0]; IMM[0]: last (clears the pending trap)
`define FP_RS    5'd8    // FRESTORE: A = frame word CC[1:0] (0 header, 1 long at +0x18, 2 long at +0x38)
`define FP_RSC   5'd9    // FRESTORE commit (EXC 14 raised on a bad frame, from the unit's register)
`define FP_RSA   5'd10   // result = A + frame size (FRESTORE (An)+)
`define FP_CHK   5'd11   // raises the pending arithmetic trap (EXC set; vector from the unit)
// The engine (p68_fpe.v) steps once every P68_FPN core clocks (1-4; tools/build.sh FPN=, which also writes the matching
// set_multicycle_path constraints). 2 is the value the builds use.
`ifndef P68_FPN
`define P68_FPN 2
`endif
// FP conditionals: Bcc/Scc/DBcc/TRAPcc uops with an FP predicate (CC = predicate bits 3:0); OC2 rewrites CC to T or F
`define U_FPC    (`U_SPARE + 9)          // the uop is an FP conditional (FBcc, FScc, FDBcc, FTRAPcc)
`define U_FPS    (`U_SPARE + 10)  // predicate bit 4 (0x10-0x1F: BSUN when NAN)
// sequence kinds (dec_fpu -> seq_fpu, parameter bits [3:0])
`define FK_GENR  4'd0    // general operation, source FPm
`define FK_GEN1  4'd1    // general operation, source of one long/word/byte (Dn, #imm, memory)
`define FK_GENM  4'd2    // general operation, source .D or .X in memory (2 or 3 longs)
`define FK_OUT   4'd3    // FMOVE FPn,<ea>
`define FK_CTLR  4'd4    // FMOVE FPcr,Rn / Rn,FPcr / #imm,FPcr (one register)
`define FK_CTLM  4'd5    // FMOVE(M) FPcr list <-> memory
`define FK_MVM   4'd6    // FMOVEM.X static list
`define FK_COND  4'd7    // FBcc FScc FDBcc FTRAPcc
`define FK_SAVE  4'd8    // FSAVE
`define FK_REST  4'd9    // FRESTORE
`define FK_CR    4'd10   // FMOVECR
`endif
