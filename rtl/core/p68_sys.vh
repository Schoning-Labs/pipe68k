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
// p68_sys.vh: codes of the system unit, for exceptions and supervisor mode.
//
// The system unit (p68_sys.v, in EX, the execute stage) does everything that touches the status register (SR), the
// control registers and the program flow on exceptions. A uop for it has unit UN_SYS, and its operation field (OP)
// holds one of the SY_* codes below. The file also has the index numbers of the control registers that MOVEC reaches
// through the system unit, and the classes of the system sequencer p68_sseq.v (the SS_* codes that dec_sys.v gives).
// It is included at file level after p68_uop.vh.
//
// Two rules shape several of these operations. First, a condition that raises an exception must come from a register,
// because the raise feeds the global flush of the whole pipeline; so CHK and CHK2 compare in one uop, latch the result,
// and raise in a later uop (SY_XCHK). Second, a uop that changes SR and also updates An in AG was made into two uops
// (SY_MVSRN and SY_SER), because the redirect that follows an SR change was, when this was built, not allowed together
// with an address register update in the same uop.
// ---------------------------------------------------------------------------------------------------------------------
`ifndef P68_SYS_VH
`define P68_SYS_VH

`define SY_MVSR   5'd0     // SR (SZ_W) or CCR (SZ_B) <- A
`define SY_ANSR   5'd1     // SR/CCR <- SR & A
`define SY_ORSR   5'd2     // SR/CCR <- SR | A
`define SY_EOSR   5'd3     // SR/CCR <- SR ^ A
`define SY_RDSR   5'd4     // result = SR
`define SY_RDCCR  5'd5     // result = CCR (zero-extended)
`define SY_MCW    5'd6     // MOVEC Rn,cr: control register IMM[2:0] <- A
`define SY_MCR    5'd7     // MOVEC cr,Rn: result = control register IMM[2:0]
`define SY_CHK    5'd8     // CHK: B = Dn, A = bound; flags; latches the trap condition Dn < 0 or Dn > bound
`define SY_C2L    5'd9     // CHK2/CMP2 step 1: latch the lower bound A
`define SY_C2U    5'd10    // CHK2/CMP2 step 2: A = upper bound, B = Rn (IMM[0]: Rn is An); flags Z, C; latches C
`define SY_TRAPCC 5'd11    // raises EXC when condition CC holds (TRAPcc, TRAPV)
`define SY_RTEF   5'd12    // RTE step 1: A = format word, B = A7; latch A7 + frame size; raises EXC on a bad format
`define SY_RTESR  5'd13    // RTE step 2: latch the stacked SR
`define SY_RTEPH  5'd14    // RTE step 3: latch the stacked PC high word
`define SY_RTEJ   5'd15    // RTE step 4: A = PC low word; SR <- latched SR, result (to A7) = latched A7, jump
`define SY_JMP    5'd16    // exception sequence: jump to A (the vector read); also RTS, RTD, RTR
`define SY_STOP   5'd17    // SR <- A, stop until an interrupt
`define SY_RESET  5'd18    // RESET: reset_o for 512 clocks
`define SY_MCLR   5'd19    // exception sequence: clear M (interrupt with M = 1: throwaway frame on the ISP)
`define SY_MOVX   5'd20    // result = A sign-extended by LSZ (MOVES to An)
`define SY_XCHK   5'd21    // raises EXC when the condition latched by SY_CHK / SY_C2U is set (keeps the compare
                           // out of the redirect path)
`define SY_MVSRN  5'd24    // MOVE (An)+/-(An),SR step 0: SR <- A without redirect (OP[1:0] = 0 as SY_MVSR)
`define SY_SER    5'd25    // its step 1: redirect to the next instruction (the step-0 An update is committed by then)
// ---- program flow ----
`define SY_NPC    5'd26    // result = address of the next instruction (JSR, BSR return address)

// MOVEC control register numbers (from the extension word) and the index the system unit uses for them:
// 0x000 SFC, 0x001 DFC (function codes for MOVES), 0x002 CACR (cache control), 0x801 VBR (vector base),
// 0x802 CAAR (cache address). The stack pointers 0x800 USP, 0x803 MSP and 0x804 ISP are plain registers instead (see
// dec_sys.v).
`define CR_SFC    3'd0     // control register index for SY_MCR / SY_MCW
`define CR_DFC    3'd1
`define CR_CACR   3'd2
`define CR_VBR    3'd3
`define CR_CAAR   3'd4

`define SS_NONE   3'd0     // system sequencer classes (dec_sys -> p68_sseq)
`define SS_RTE    3'd1     // RTE: return from exception (four word loads from the stack frame)
`define SS_CHK2   3'd2     // CHK2 / CMP2: two bound loads, compare, raise
`define SS_MOVES  3'd3     // MOVES: the access in a step of its own
`define SS_CHK    3'd4     // CHK: raise from the condition step 0 latched
`define SS_SRW    3'd5     // MOVE (An)+/-(An),SR: the redirect step

`endif
