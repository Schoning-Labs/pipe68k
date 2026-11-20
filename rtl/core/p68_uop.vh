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
// ==== p68_uop.vh: the micro-op (uop) format ==========================================================================
//
// Every 68k instruction that pipe68k runs is first turned into one or more micro-ops (uops). A uop is the unit of work
// that travels down the pipeline, one stage per clock: the decode stage (DS, p68_decode.v) builds it, the address
// generation stage (AG, p68_ag.v) computes its memory addresses, the two operand stages (OC1 and OC2, p68_oc.v) read the
// D-cache and the registers, the execute stage (EX, p68_ex.v) computes the result and decides whether the uop commits,
// and the write-back stage (WB, p68_wb.v) writes the register and sends the store to memory. p68_core.v has the full map
// of the pipeline. This file only defines what a uop looks like, and nearly every other file in rtl/core includes it.
//
// The 68k instruction set is very irregular (an ADD can read memory, write memory, or both, with eighteen addressing
// modes on either side), so a fixed uop that can say "read memory here, compute this, write that register and store
// there" keeps the back end simple. A simple instruction such as ADD.L (A0)+,D1 is a single uop: one load at (A0), an
// update of A0 by 4 done in AG, one ALU add in EX and one register write of D1 in WB. A long instruction such as MOVEM
// becomes a sequence of uops, one per register, made by the sequencers (p68_seq.v and friends). The instruction commits
// (retires) together with its last uop, marked by LAST.
//
// The uop is one flat bit vector instead of a SystemVerilog struct, because Verilog-2001 (which Gowin's tools and
// the Verilator simulator both read without trouble) has no structs. Each field has two macros: U_NAME is its lowest bit and F_NAME is
// the part select (`U_NAME +: width`), so a field is read as u[`F_NAME]. Fields are chained (each U_ is the previous one
// plus its width), which means a field can be widened without renumbering by hand. The whole uop is one packed
// vector (unused bits are removed by synthesis); the U_* macros are bit offsets.
// In practice that means the 271 bits cost nothing where a stage does not read them: a flip-flop whose output has no
// load is dropped by Gowin's synthesizer, so for example the PC field only exists in the stages that use it.
//
// What one uop can do: at most one load (the load address descriptor LA) and one store (the store address descriptor
// SA), one operation in one execution unit (UNIT, OP), one register write from WB (WEN, WREG) and up to two
// address-register updates done early in AG (LA_UEN updates the load's base register, as in (An)+ or -(An); SA_UEN or
// LEA writes the store's base register). The AG updates are speculative and are undone by the register file's
// committed copy when a later uop redirects (see p68_regfile.v).
//
// Register numbers are 5 bits everywhere in the core:
//    0-7   D0-D7
//    8-15  A0-A7 (A7 is always the active stack pointer)
//   16-19  T0-T3, temporaries that sequences use for values between their uops (not visible to software)
//   20-22  the inactive stack pointer banks USP, ISP and MSP (read as operands, written by WB; see the EXC part below)
//
// History in short: the format was fixed with the first real core, when the one-file feasibility sample was split into
// modules, and every later work package added its fields at the end (the spare field, the exception field, the pair
// bits, the prediction flags), which is why related fields are not always next to each other.
`ifndef P68_UOP_VH                       // include guard: the file is included by many modules and headers
`define P68_UOP_VH
// ---- build switch: with or without the FPU ----------------------------------------------------------------------
// P68_FPU = 0 builds the integer core without the 68881/68882 floating-point unit: no FPU unit, no FPU decoder and no
// FPU sequences, and a coprocessor-1 opcode (line F, $F2xx) takes the line-F exception (vector 11) as on a 68020 with
// no coprocessor. tools/build.sh passes FPU=1 for a build with the FPU, and the lockstep bench builds with the FPU by
// default. The default here is 0 because a stand-alone core test is the only place where this file decides; every real
// build of the chip is made with the FPU (the timing of the core is never judged without it, because the extra third
// of design size changes the placement of everything else).
`ifndef P68_FPU
`define P68_FPU 0
`endif

// ---- CPU speed mode (slow) cost scaling --------------------------------------------------------------------------
// For old Amiga games the system can lock the CPU to roughly the speed of a 14 MHz 68EC020. p68_decode.v (cost_f)
// then makes every instruction wait for its cost in core clocks. The cost tables are written in clocks of a 66.67 MHz
// core: the 68EC020 cycle count of the cache case times 4.7, which is 66.67 / 14.18 MHz. When the core runs at another
// clock, the entries have to be scaled, otherwise the speed mode would be slower or faster than a real A1200.
// P68_CLK_KHZ is the real core clock in kHz. The system's build script sets it from its clock divider (1,200,000 kHz /
// CPU_DIV); 66670 is the 66.67 MHz the A1200 system runs the CPU at.
`ifndef P68_CLK_KHZ
`define P68_CLK_KHZ 66670
`endif
// KB(n): a base cost entry. The tables hold the base cost less 3, because the cost pipeline in DS adds 3 clocks of its
// own. So the 3 clocks are added back before the scaling and taken off after it (they stay 3 clocks at any core
// clock). Adding 33335 (half of 66670) before the integer divide rounds to the nearest clock instead of truncating.
// At 66670 kHz both macros return the table value unchanged, e.g. KB(16) = ((19 * 66670 + 33335) / 66670) - 3 = 16.
`define P68_KB(n) ((((n) + 3) * `P68_CLK_KHZ + 33335) / 66670 - 3)
// KE(n): an effective-address cost entry, scaled and rounded the same way with no offset.
`define P68_KE(n) (((n) * `P68_CLK_KHZ + 33335) / 66670)

// ---- operation ------------------------------------------------------------------------------------------------
`define U_UNIT   0                       // execution unit that does the work in EX (UN_* codes below)
`define F_UNIT   `U_UNIT +: 4            // 4 bits: up to 16 units
`define U_OP     (`U_UNIT + 4)           // operation inside the unit (OP_* codes for the ALU and shifter; units define their own)
`define F_OP     `U_OP +: 5
`define U_SZ     (`U_OP + 5)             // operation size (SZ_*): byte, word or long, as in the .B/.W/.L suffix
`define F_SZ     `U_SZ +: 2
`define U_LSZ    (`U_SZ + 2)             // load size (memory operand size); may differ from SZ, e.g. MOVEA.W loads a word and writes a long
`define F_LSZ    `U_LSZ +: 2
// ---- operands -------------------------------------------------------------------------------------------------
// EX has two operand inputs, A and B. Each comes from a register, the loaded memory value or the immediate.
`define U_AEXT   (`U_LSZ + 2)            // extension of operand A before EX (AX_*), e.g. sign-extend a word for ADDA.W
`define F_AEXT   `U_AEXT +: 2
`define U_ASEL   (`U_AEXT + 2)           // operand A source (AS_*)
`define F_ASEL   `U_ASEL +: 2
`define U_AREG   (`U_ASEL + 2)           // operand A register number (0-22) when ASEL = AS_REG
`define F_AREG   `U_AREG +: 5
`define U_BSEL   (`U_AREG + 5)           // operand B source (BS_*); B is also the old value a .B/.W result is merged into
`define F_BSEL   `U_BSEL +: 2            // (a 68k byte or word write keeps the upper bits of the data register)
`define U_BREG   (`U_BSEL + 2)           // operand B register number when BSEL = BS_REG
`define F_BREG   `U_BREG +: 5
`define U_IMM    (`U_BREG + 5)           // 32-bit immediate: operand A when ASEL = AS_IMM, or a target or constant for some units
`define F_IMM    `U_IMM +: 32
// ---- result ---------------------------------------------------------------------------------------------------
`define U_WEN    (`U_IMM + 32)           // WB writes the result into register WREG
`define F_WEN    `U_WEN +: 1
`define U_WREG   (`U_WEN + 1)            // destination register number (0-22)
`define F_WREG   `U_WREG +: 5
`define U_CC     (`U_WREG + 5)           // 68k condition code (opcode bits 11-8 of Bcc, Scc, DBcc, TRAPcc); some units use it as a small parameter
`define F_CC     `U_CC +: 4
// ---- flag mode: which flags change and how ---------------------------------------------------------------------
// The 68k condition codes are X (extend), N (negative), Z (zero), V (overflow) and C (carry). Flags are precise: only a
// uop that commits in EX changes the CCR, so a replayed or flushed uop leaves no trace in the flags.
`define U_FEN    (`U_CC + 4)             // write enables for X N Z V C (bit 4 = X, bit 0 = C)
`define F_FEN    `U_FEN +: 5
`define U_FCV    (`U_FEN + 5)            // where C and V come from (CV_*); X, when enabled, takes the new C
`define F_FCV    `U_FCV +: 3
`define U_FZM    (`U_FCV + 3)            // how Z is formed (ZM_*)
`define F_FZM    `U_FZM +: 2
// ---- load address descriptor LA: address = base + index + displacement; update An = base + UDL ------------------
// The descriptor is a sub-vector of 62 bits that describes one 68k effective address in a uniform way, so AG has a
// single adder for every addressing mode: base register (or zero, or the PC) plus index register times scale plus
// displacement. For example (d16,A2) is base A2 with displacement d16, (d8,PC,D3.W*4) is base PC with index D3 scaled
// by 4 and displacement d8, and an absolute address is base zero with the address as displacement. The LO_ macros are
// bit offsets inside the descriptor, so the same functions in p68_dec.vh can build either descriptor.
`define LO_EN    0                       // a load happens
`define LO_BSEL  1                       // base select: BA_ZERO (absolute), BA_REG (An or a T register), BA_PC (PC-relative)
`define LO_BREG  3                       // base register (A0-A7 or T0-T3)
`define LO_XEN   8                       // an index register is added
`define LO_XREG  9                       // index register (D0-D7, A0-A7)
`define LO_XL    14                      // index is the whole long (else its low word, sign-extended), the .W/.L of Xn.W / Xn.L
`define LO_XSC   15                      // index scale 1, 2, 4 or 8 (68020 scaled index, coded 0-3 as in the extension word)
`define LO_XK    17                      // constant added as index when XEN = 0 (the PC offset of the extension word for PC-relative modes)
`define LO_DISP  21                      // displacement, 32 bits (sign-extended by DS where the 68k field is shorter)
`define LO_UEN   53                      // AG writes base + UDL back into BREG: the (An)+ and -(An) update
`define LO_UDL   54                      // update delta, signed 8 bits (e.g. +4 for (A0)+ on a long, -2 for -(A0) on a word)
`define LA_W     62                      // width of the LA descriptor
`define U_LA     (`U_FZM + 2)            // where the LA descriptor sits in the uop
`define F_LA     `U_LA +: `LA_W
`define L_EN     `LO_EN   +: 1           // part selects inside an LA block on its own (used by the functions in p68_dec.vh)
`define L_BSEL   `LO_BSEL +: 2
`define L_BREG   `LO_BREG +: 5
`define L_XEN    `LO_XEN  +: 1
`define L_XREG   `LO_XREG +: 5
`define L_XL     `LO_XL   +: 1
`define L_XSC    `LO_XSC  +: 2
`define L_XK     `LO_XK   +: 4
`define L_DISP   `LO_DISP +: 32
`define L_UEN    `LO_UEN  +: 1
`define L_UDL    `LO_UDL  +: 8
`define F_LA_EN    (`U_LA + `LO_EN)   +: 1    // the same fields as part selects of the whole uop
`define F_LA_BSEL  (`U_LA + `LO_BSEL) +: 2
`define F_LA_BREG  (`U_LA + `LO_BREG) +: 5
`define F_LA_XEN   (`U_LA + `LO_XEN)  +: 1
`define F_LA_XREG  (`U_LA + `LO_XREG) +: 5
`define F_LA_XL    (`U_LA + `LO_XL)   +: 1
`define F_LA_XSC   (`U_LA + `LO_XSC)  +: 2
`define F_LA_XK    (`U_LA + `LO_XK)   +: 4
`define F_LA_DISP  (`U_LA + `LO_DISP) +: 32
`define F_LA_UEN   (`U_LA + `LO_UEN)  +: 1
`define F_LA_UDL   (`U_LA + `LO_UDL)  +: 8
// ---- store address descriptor SA: address = SAME ? load address : base + index + displacement + ADJ --------------
// The store's base update is base + ADJ + UDL. SAME is the read-modify-write case (ADD.L D0,(A1) loads and stores at
// the same address), so AG does not compute the address twice. ADJ covers instructions whose two EAs use the same
// address register, e.g. MOVE.L (A0)+,(A0): the 68k does the source's step first, so the destination address is the
// base plus the source's step, signed 5 bits. AG adds no index on the store side: the SA descriptor has room for an
// index (SO_XEN to SO_XSC) that nothing sets or reads, and a MOVE to (d8,An,Xn) runs as a two-uop sequence (SQ_X) that computes
// the destination address into T0 first (the store side of the first AG had no index, and it stayed that way).
`define SO_EN    0                       // a store happens
`define SO_SAME  1                       // store at the load address (read-modify-write)
`define SO_BSEL  2                       // base select (BA_*)
`define SO_BREG  4                       // base register
`define SO_XEN   9                       // index used
`define SO_XREG  10                      // index register
`define SO_XL    15                      // index long
`define SO_XSC   16                      // index scale
`define SO_DISP  18                      // displacement, 32 bits
`define SO_ADJ   50                      // signed 5 bits: the source EA's step when both EAs use the same An
`define SO_UEN   55                      // AG writes the updated base back into BREG
`define SO_UDL   56                      // update delta, signed 8 bits
`define SA_W     64                      // width of the SA descriptor
`define U_SA     (`U_LA + `LA_W)         // where the SA descriptor sits in the uop
`define F_SA     `U_SA +: `SA_W
`define F_SA_EN    (`U_SA + `SO_EN)   +: 1    // part selects of the SA fields in the whole uop
`define F_SA_SAME  (`U_SA + `SO_SAME) +: 1
`define F_SA_BSEL  (`U_SA + `SO_BSEL) +: 2
`define F_SA_BREG  (`U_SA + `SO_BREG) +: 5
`define F_SA_DISP  (`U_SA + `SO_DISP) +: 32
`define F_SA_ADJ   (`U_SA + `SO_ADJ)  +: 5
`define F_SA_UEN   (`U_SA + `SO_UEN)  +: 1
`define F_SA_UDL   (`U_SA + `SO_UDL)  +: 8
// ---- control --------------------------------------------------------------------------------------------------
`define U_LEA    (`U_SA + `SA_W)         // AG writes the load address into SA_BREG (LEA; sequences also use it to put an EA into T0)
`define F_LEA    `U_LEA +: 1
`define U_BRA    (`U_LEA + 1)            // unconditional jump to the load address, resolved in AG (BRA, JMP without a register redirect early)
`define F_BRA    `U_BRA +: 1
`define U_FIRST  (`U_BRA + 1)            // first uop of its instruction
`define F_FIRST  `U_FIRST +: 1
`define U_LAST   (`U_FIRST + 1)          // last uop: the instruction commits (retires) with it
`define F_LAST   `U_LAST +: 1
`define U_STEP   (`U_LAST + 1)           // sequence step (0 = the decoder's own uop); a replay restarts the instruction at this step
`define F_STEP   `U_STEP +: 5
`define U_PC     (`U_STEP + 5)           // address of the instruction's first word
`define F_PC     `U_PC +: 32
`define U_LEN    (`U_PC + 32)            // instruction length in 16-bit words (PC + 2 * LEN is the next instruction)
`define F_LEN    `U_LEN +: 4
`define U_EXC    (`U_LEN + 4)            // exception control, see the EXC description further down
`define F_EXC    `U_EXC +: 8
`define U_SPARE  (`U_EXC + 8)            // eight single-bit flags, each named further down (F_PRED, U_FXP, U_LOCK, ...)
`define F_SPARE  `U_SPARE +: 8
`define U_PSK    (`U_SPARE + 8)          // skip pair: this forward Bcc jumps over exactly lane B's instruction, so it never
                                         // redirects; when it is taken, EX drops lane B's result instead
                                         // U_SPARE + 9 and + 10: the FPU's conditional bits (defined in p68_fpu.vh)
`define UW       (`U_SPARE + 11)         // total uop width: 271 bits

// ---- codes ----------------------------------------------------------------------------------------------------
// Execution units (UNIT field). Codes 8-10 are further down (DBcc, CAS, and the FPU in p68_fpu.vh).
`define UN_NONE  4'd0                    // no EX operation (e.g. a pure LEA, or a uop that only raises an exception)
`define UN_ALU   4'd1                    // adder, logic and move (p68_ex.v)
`define UN_SHF   4'd2                    // rotator: shifts and rotates
`define UN_BRA   4'd3                    // conditional branch (Bcc), resolved in EX against the flags
`define UN_MUL   4'd4                    // multiply unit (p68_md.v; operations in p68_md.vh)
`define UN_DIV   4'd5                    // divide unit (p68_md.v)
`define UN_BFU   4'd6                    // bit-field unit (p68_bf.v)
`define UN_SYS   4'd7                    // system unit p68_sys.v: SR, MOVEC, traps, RTE, STOP

// ALU operations (OP field with UNIT = UN_ALU). The codes 0-7 follow no 68k encoding; they are local to p68_ex.v.
`define OP_ADD   5'd0                    // r = b + a
`define OP_SUB   5'd1                    // r = b - a
`define OP_AND   5'd2
`define OP_OR    5'd3
`define OP_CMP   5'd4                    // b - a for the flags only (CMP, CMPA, CMPI, CMPM, TST via a = 0)
`define OP_MOV   5'd5                    // r = a (MOVE and friends, flags from a)
`define OP_EOR   5'd7
`define OP_SHF   5'd8                    // shifter operations: OP_SHF | {left, rotate, arithmetic} in the low 3 bits

`define SZ_B     2'd0                    // byte
`define SZ_W     2'd1                    // word (16 bits)
`define SZ_L     2'd2                    // long (32 bits)

`define AS_REG   2'd0                    // operand A from register AREG
`define AS_MEM   2'd1                    // operand A is the loaded value
`define AS_IMM   2'd2                    // operand A is IMM
`define BS_REG   2'd0                    // operand B from register BREG
`define BS_MEM   2'd1                    // operand B is the loaded value
`define BS_IMM   2'd2                    // operand B is IMM (BTST Dn,#imm, where A holds the bit number, so A cannot be the immediate)

`define BA_ZERO  2'd0                    // address base zero (absolute addresses)
`define BA_REG   2'd1                    // address base from a register
`define BA_PC    2'd2                    // address base is the PC (PC-relative modes)

`define AX_NONE  2'd0                    // operand A used as it is
`define AX_SXW   2'd1                    // sign-extend the low word to a long (MOVEA.W, ADDA.W, MOVEM.W loads)

`define CV_CLR   3'd0                    // C = V = 0 (logic operations and moves)
`define CV_ADD   3'd1                    // from the adder (carry or borrow, signed overflow)
`define CV_SHF   3'd2                    // from the shifter (X keeps its value when the shift count is 0)

`define ZM_SET   2'd0                    // Z = result is zero
`define ZM_KEEP  2'd1                    // Z cleared if the result is nonzero, else unchanged (ADDX, SUBX, NEGX, BCD: Z accumulates over a multi-precision chain)

`define FL_NONE  5'b00000                // FEN shorthands: no flag
`define FL_NZVC  5'b01111                // all but X
`define FL_ALL   5'b11111                // X N Z V C

`define R_T0     5'd16                   // register number of temporary T0

// ---- integer instructions beyond the first subset ---------------------------------------------------------------
`define UN_DBR   4'd8                    // DBcc: ALU subtract 1 from Dn.W; EX redirects when the condition is false and Dn.W was not 0; no write if the condition is true
`define OP_BX    5'd6                    // ALU: r = a >> (8 * CC[1:0]), the byte pick of MOVEP and UNPK
`define OP_NEG   5'd16                   // adder group (16-19): r = sum with B inverted for NEG and NEGX, carry-in from X
`define OP_NEGX  5'd17
`define OP_ADDX  5'd18
`define OP_SUBX  5'd19
`define OP_SCC   5'd20                   // codes 20-31 run in the extra integer unit p68_exi.v
`define OP_ABCD  5'd21                   // BCD add (one ten-bit add step; BCD instructions are three uops)
`define OP_SBCD  5'd22
`define OP_NBCD  5'd23
`define OP_BTST  5'd24                   // bit number in A (mod 32 for .L, mod 8 for .B), operand in B; Z = !bit (ZM_BIT)
`define OP_BCHG  5'd25
`define OP_BCLR  5'd26
`define OP_BSET  5'd27
`define OP_PACK  5'd28                   // r = pack(b + a): two BCD digits from two bytes
`define OP_UNPK  5'd29                   // r = unpack(b) + a
`define OP_CMB   5'd30                   // r = (b << 8) | a[7:0], the byte combine of MOVEP loads and PACK from memory
`define OP_ROXC  5'd31                   // ROXL/ROXR count for the rotator: {count mod (n+1) == 0, L-1}; CC[0] = left (register-count ROX, step 1)
`define AX_SXB   2'd2                    // sign-extend the low byte to a long (EXT, EXTB)
`define CV_BCD   3'd3                    // C, V (and N) from the BCD unit
`define ZM_BIT   2'd2                    // Z = the tested bit was 0 (BTST family)
`define ZM_OLD   2'd3                    // N and Z from the old B value (TAS tests before it sets bit 7)
// Sequence classes (the SEQ output of a decoder chooses which sub-sequencer runs the instruction's later steps).
`define SQ_EXG   4'd8                    // EXG
`define SQ_MM    4'd9                    // memory to memory: ADDX, SUBX, ABCD, SBCD with -(An), CMPM
`define SQ_PK    4'd10                   // PACK, UNPK
`define SQ_MVP   4'd11                   // MOVEP
`define SQ_ROX   4'd12                   // ROXL/ROXR with a register count
// ---- program flow ----------------------------------------------------------------------------------------------
`define SQ_FLOW  4'd2                    // calls, returns, LINK, UNLK, PEA (the kind is a parameter, see dec_flow.v)
// ---- exceptions and supervisor mode ----------------------------------------------------------------------------
// EXC field: bits 5-0 are the vector number raised after the uop commits (0 = none), bits 7-6 = 00 for a normal uop.
// The raise is unconditional for UN_NONE, UN_ALU and the other plain units; UN_SYS raises only when its condition holds
// (CHK, CHK2, TRAPcc, a bad RTE frame format), and UN_DIV when the divide unit reports a divide by zero. Vectors 4, 8,
// 10, 11 and 14 stack the instruction's own PC (the instruction is not done), all others the next instruction's PC
// (the instruction completed, e.g. TRAP or a divide by zero). Vectors 5, 6, 7 and 9 use stack frame format 2 (six
// words, with the instruction address), all others format 0 (four words). Raising an exception always flushes the
// pipeline, so the condition must come from a register (this is a fixed rule of the core: a raise straight from logic
// would put the compare in front of the global flush).
// Bits 7-6 = 01 mark a uop of an exception sequence (p68_xseq.v, the stack frame writes), 10 the same for an interrupt
// (the retire port then reports ret_irq).
`define XS_EXC   2'b01
`define XS_IRQ   2'b10
// Stack pointer banks (register numbers, read in OC, written by WB): the inactive ones of USP (user), ISP (interrupt)
// and MSP (master). A7 (15) is always the active stack pointer; a change of the S or M bit swaps A7 with its bank
// (p68_regfile.v), so no uop ever has to ask which stack pointer is active.
`define R_USP    5'd20
`define R_ISP    5'd21
`define R_MSP    5'd22

// ---- branch prediction bits (spare field bits 3-7) -------------------------------------------------------------
`define F_PRED   (`U_SPARE + 3) +: 1     // predicted taken (static backward-taken rule in DS, or the BTC)
`define F_PBTC   (`U_SPARE + 4) +: 1     // predicted by the branch target cache in fetch: the target is in IMM and OC checks it
`define F_PFIX   (`U_SPARE + 5) +: 1     // a fetch prediction cut this instruction in the wrong place: OC replays it
`define F_BR     (`U_SPARE + 6) +: 1     // Bcc/BRA: updates the branch target cache when it commits
`define F_PRET   (`U_SPARE + 7) +: 1     // predicted return (RTS/RTD/RTR jump uop): the target is in IMM and OC checks it
`define U_BOLD   (`U_LA + `LO_EN)        // lane B only: lane B is the older instruction of the pair (a lane-B uop never
                                         // loads, so its LA enable bit is free and is reused for this)

// sequence classes (decoder output SEQ, sequencer p68_seq.v)
`define SQ_NONE  4'd0                    // a single-uop instruction
`define SQ_MOVEM 4'd1                    // MOVEM: one uop per register in the mask

// ---- bit fields (unit p68_bf.v, sequences seq_bf.v) --------------------------------------------------------------
// UN_BFU ops. The unit takes its operands when a uop commits and delivers results 2 to 4 clocks later:
`define SQ_BFR    4'd6                   // bit field in a data register
`define SQ_BFM    4'd7                   // bit field in memory
`define BF_SETUP  5'd0                   // offset and width: A = offset register, B = width register, IMM[4:0] / IMM[9:5] immediates, CC = {regform, dynW, dynO}
`define BF_SETI   5'd1                   // insert value: A
`define BF_ADDR   5'd2                   // memory field address: A = EA; result (RD0, 3 clocks later) = aligned byte address
`define BF_RD0    5'd3                   // result 0 (field result, new first long, address) and flags (FEN bit 3: N, Z; V, C cleared)
`define BF_RD1    5'd4                   // result 1 (new second long)
// compute ops are 8 + type, type = opcode bits 10-8: 0 BFTST, 1 BFEXTU, 2 BFCHG, 3 BFEXTS, 4 BFCLR, 5 BFFFO, 6 BFSET,
// 7 BFINS; A = first long (or the register), B = second long

// ---- addressing completion, misaligned accesses, CAS/CAS2 -------------------------------------------------------
`define UN_CAS    4'd9                   // CAS/CAS2 step: the ALU op as usual, but store and flags only if cond (CC), register write only if !cond
`define SQ_CAS    4'd3                   // CAS, CAS2 (seq_cas.v)
`define SQ_X      4'd13                  // two-step forms: MOVE to (d8,An,Xn) (store T0), BTST Dn,#imm (seq_cas.v)
`define R_T1      5'd17                  // register number of temporary T1
`define R_T2      5'd18                  // register number of temporary T2
`define U_FXP     (`U_SPARE + 0)         // uop of the full-format EA pre-sequence (p68_fx); STEP = its slot
`define U_FXM     (`U_SPARE + 1)         // uop of the main part of an instruction whose full-format EAs p68_fx computed
`define U_LOCK    (`U_SPARE + 2)         // CAS/CAS2 step: the bus lock output is held while these run

`endif
