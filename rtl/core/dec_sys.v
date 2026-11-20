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
// dec_sys: the group decoder for system instructions and every instruction whose job is to raise an exception
// (lines 0, 4, 5, A and F).
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v): it sees the opcode word w0 and the
// words behind it, says whether the opcode is its own (hit), and gives one micro-op (uop, p68_uop.vh) that DS ORs with
// the other decoders' uops, or starts a sequence of the system sequencer p68_sseq.v. The uops run through DR (register
// stage before address generation), AG (address generation), OC (operand cycle), EX (execute; most of these uops use
// the system unit p68_sys.v there) and WB (write-back). Combinational logic only. Unlike the other decoders it also reads
// the current supervisor bit S and master bit M of the status register (SR), because what an instruction does depends
// on them: a privileged instruction in user mode becomes an exception, and MOVEC names a stack pointer by M.
//
// The instructions.
//
//     MOVE SR,<ea>          0100 0000 11 EA   (privileged on the 68020)   MOVE CCR,<ea>   0100 0010 11 EA
//     MOVE <ea>,CCR         0100 0100 11 EA                               MOVE <ea>,SR    0100 0110 11 EA (privileged)
//     ORI/ANDI/EORI #,CCR   0000 0000/0010/1010 0011 1100 + word          the same to SR with bit 6 set (privileged)
//     MOVE USP,An / An,USP  0100 1110 0110 d An                           (privileged)
//     MOVEC Rc,Rn / Rn,Rc   0100 1110 0111 101 d + word: D/A Rn(4) Rc(12) (privileged)
//     MOVES <ea>,Rn / Rn,<ea> 0000 1110 size EA + word: D/A Rn d ...      (privileged)
//     RTE 4E73, STOP #imm 4E72 + word, RESET 4E70                         (privileged)
//     TRAP #n 0100 1110 0100 n, TRAPV 4E76, TRAPcc 0101 cond 11111 opmode (no operand, .W, .L)
//     CHK <ea>,Dn  0100 Dn 1 s 0 EA (s: 1 word, 0 long), CHK2/CMP2 <ea>,Rn  0000 0 size 011 EA + word
//     BKPT #n  0100 1000 0100 1 n, line A (1010 ...) and line F (1111 ...) opcodes
//
// What happens with them.
//   - A privileged instruction in user mode gives an empty uop that raises vector 8 (privilege violation).
//   - MOVE to and from SR/CCR, ORI/ANDI/EORI to SR/CCR, MOVEC, STOP and RESET are single uops for the system unit.
//     MOVE USP and MOVEC of a stack pointer are plain register moves, because the inactive stack pointers live in the
//     register file as extra registers (R_USP, R_ISP, R_MSP) and A7 is always the active one. MOVEC therefore names
//     the active stack pointer as A7 and the inactive ones as their banks, using the current S and M (every change of S
//     or M flushes the pipeline, so the S and M seen here are never stale). An unknown MOVEC control register raises
//     vector 4.
//   - TRAP, BKPT, line A and line F raise their vector from an empty uop (TRAP: 32 + n; BKPT: 4, since there is no
//     debugger on the bus to answer it; line A: 10; line F: 11). TRAPV and TRAPcc are a system unit uop that raises
//     vector 7 when the condition holds. Line F words of coprocessor 1 belong to the FPU decoder dec_fpu.v, except
//     that coprocessor save and restore (bits 8-6 = 100 / 101) are privileged for every coprocessor.
//   - RTE, CHK2/CMP2 and MOVES are sequences of the system sequencer p68_sseq: step 0 (this uop) is empty, the steps
//     come from the parameters (spar) latched at the start.
//   - CHK is a sequence too: step 0 (this uop) compares and sets the flags, step 1 raises the exception from the latched
//     result. A raise condition must come from a register because it feeds the global flush (HISTORY.md section 7).
//   - MOVE (An)+,SR and MOVE -(An),SR are a two-step sequence: step 0 writes SR without a redirect, step 1 redirects.
//
// The uop is built as an AND-OR of one-hot cases (one prepared uop per instruction kind, each ANDed with its own select
// and all ORed), which is flatter than a priority chain of ifs and keeps the uop a few LUT levels from the opcode.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`include "p68_sys.vh"
`default_nettype none
module dec_sys (
  input  wire [15:0] w0, w1, w2, w3,         // opcode, extension word, EA extension words
  input  wire        sup,            // SR S
  input  wire        msp,            // SR M
  input  wire        fpu_hit,        // dec_fpu decodes this F-line word
  output wire        hit,                    // any instruction of this group
  input  wire        pg,             // predecoded gate (this decoder's ug from p68_pdec, a register in the entry)
  output wire        ug,             // the gate of the uop, for p68_pdec's own copy of this decoder
  output wire [`UW-1:0] u,                   // the uop (zero unless pg)
  output wire [2:0]  seq,            // system sequencer class (SS_*)
  output wire [44:0] spar,           // {x1, x0, rn, flag, size, mode, reg}
  output wire        is_rst          // RESET in supervisor mode (DS waits until reset_o ends)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
`include "p68_sysf.vh"
wire [2:0] m3 = w0[5:3], r0 = w0[2:0], r9 = w0[11:9];         // EA mode and register; Dn of CHK
// ---- recognizing the instructions (with their legal EAs) --------------------------------------------------------
wire [1:0] mk  = w0[10:9];                                      // 0 from SR, 1 from CCR, 2 to CCR, 3 to SR
// MOVE to SR/CCR reads any data mode; MOVE from SR/CCR writes Dn or alterable memory
wire mvsr  = (w0[15:11] == 5'b01000) && (w0[8:6] == 3'b011) && (mk[1] ? ea_dat(m3, r0) : (m3 == 3'd0 || ea_mal(m3, r0)));
wire immsr = (w0[15:12] == 4'h0) && (w0[11:9] == 3'b000 || w0[11:9] == 3'b001 || w0[11:9] == 3'b101) &&
             (w0[8:7] == 2'b00) && (w0[5:0] == 6'b111100);      // ORI/ANDI/EORI #,CCR (bit 6 = 0) or SR (bit 6 = 1)
wire musp  = (w0[15:4] == 12'h4E6);                             // MOVE USP (bit 3: 1 USP->An, 0 An->USP)
wire movec = (w0[15:1] == 15'h273D);                            // MOVEC (4E7A, 4E7B; bit 0 = direction)
wire rte   = (w0 == 16'h4E73), stop = (w0 == 16'h4E72), rset = (w0 == 16'h4E70), trapv = (w0 == 16'h4E76);
wire trap  = (w0[15:4] == 12'h4E4);                             // TRAP #0-15
wire trapc = (w0[15:12] == 4'h5) && (w0[7:3] == 5'b11111) && (w0[2:0] >= 3'd2) && (w0[2:0] <= 3'd4);   // TRAPcc
wire chk   = (w0[15:12] == 4'h4) && w0[8] && !w0[6] && ea_dat(m3, r0);              // CHK.W / CHK.L
wire chk2  = (w0[15:11] == 5'b00000) && (w0[10:9] != 2'b11) && (w0[8:6] == 3'b011) && ea_ctl(m3, r0);   // CHK2/CMP2
wire moves = (w0[15:8] == 8'h0E) && (w0[7:6] != 2'b11) && ea_mal(m3, r0);           // MOVES
wire bkpt  = (w0[15:3] == 13'h0909);                            // BKPT (0x4848-0x484F)
wire linea = (w0[15:12] == 4'hA), linef = (w0[15:12] == 4'hF) && !fpu_hit;   // dec_fpu takes coprocessor id 1
// coprocessor save and restore (F-line, bits 8-6 = 100 / 101, any coprocessor id) are privileged: user mode gives
// vector 8, supervisor vector 11; save takes the -(An) style modes, restore the (An)+ style modes
wire cpsr  = linef && (w0[8:7] == 2'b10) && (w0[6] ? (m3 == 3'd2 || m3 == 3'd3 || m3 == 3'd5 || m3 == 3'd6 || (m3 == 3'd7 && r0 <= 3'd3))
                                                   : (m3 == 3'd2 || m3 == 3'd4 || m3 == 3'd5 || m3 == 3'd6 || (m3 == 3'd7 && r0 <= 3'd1)));
// privileged: MOVE from/to SR, the immediates to SR, MOVE USP, MOVEC, RTE, STOP, RESET, MOVES, coprocessor save/restore
wire priv  = (mvsr && (mk == 2'd0 || mk == 2'd3)) || (immsr && w0[6]) || musp || movec || rte || stop || rset || moves || cpsr;
wire h     = mvsr || immsr || musp || movec || rte || stop || rset || trapv || trap || trapc || chk || chk2 ||
             moves || bkpt || linea || linef;
wire pv    = priv && !sup;                                      // privilege violation
wire srw   = mvsr && mk == 2'd3 && (m3 == 3'd3 || m3 == 3'd4);  // MOVE (An)+/-(An),SR: sequence, step 1 redirects

// ---- MOVEC control register: system unit register (index) or stack pointer (register number) ----------------------
wire [11:0] cr = w1[11:0];                                      // control register number from the extension word
wire        cr_sys = (cr == 12'h000) || (cr == 12'h001) || (cr == 12'h002) || (cr == 12'h801) || (cr == 12'h802);
wire        cr_sp  = (cr == 12'h800) || (cr == 12'h803) || (cr == 12'h804);   // USP, MSP, ISP
wire [2:0]  cr_idx = (cr == 12'h000) ? `CR_SFC : (cr == 12'h001) ? `CR_DFC : (cr == 12'h002) ? `CR_CACR :
                     (cr == 12'h801) ? `CR_VBR : `CR_CAAR;
// USP is always the bank (MOVEC needs supervisor mode). MSP is A7 when M = 1, else its bank; ISP is A7 when M = 0.
wire [4:0]  cr_reg = (cr == 12'h800) ? `R_USP : (cr == 12'h803) ? (msp ? 5'd15 : `R_MSP) : (msp ? `R_ISP : 5'd15);
wire [4:0]  rn     = {1'b0, w1[15:12]};                         // general register: bit 15 D/A plus 3 bits = 0-15

// ---- the uop: an AND-OR of one-hot cases --------------------------------------------------------------------------
// The uop counts only with the hit, so the cases are told apart by their opcode patterns (the EA checks are in the
// hit, and the hit is in the predecoded gate); the privilege violation overrides them all.
wire mvsr_c = (w0[15:11] == 5'b01000) && (w0[8:6] == 3'b011);   // MOVE to/from SR/CCR, by pattern
wire chk_c  = (w0[15:12] == 4'h4) && w0[8] && !w0[6];           // CHK, by pattern
wire mvs_c  = (w0[15:8] == 8'h0E) && (w0[7:6] != 2'b11);        // MOVES, by pattern
wire pv_c   = !sup && ((mvsr_c && (mk == 2'd0 || mk == 2'd3)) || (immsr && w0[6]) || musp || movec || rte || stop || rset || mvs_c || cpsr);
wire srw_c  = mvsr_c && mk == 2'd3 && (m3 == 3'd3 || m3 == 3'd4);
wire [38:0] src_w = ea_srcc(m3, r0, `SZ_W, 2'd1);               // a word source EA (MOVE to SR/CCR)
wire [1:0]  csz   = w0[7] ? `SZ_W : `SZ_L;                      // CHK size: bit 7 set = word
wire [38:0] src_c = ea_srcc(m3, r0, csz, 2'd1);                 // CHK's bound operand
reg  [`UW-1:0] v_mf, v_mt, v_im, v_mu, v_mc, v_tc, v_ck, v_ex;  // one prepared uop per case
always @* begin                                                     // MOVE SR/CCR,<ea>
  v_mf = {`UW{1'b0}};
  v_mf[`F_UNIT] = `UN_SYS; v_mf[`F_OP] = mk[0] ? `SY_RDCCR : `SY_RDSR; v_mf[`F_SZ] = `SZ_W;   // result = SR or CCR
  if (m3 == 3'd0) begin v_mf[`F_BREG] = {2'b00, r0}; v_mf[`F_WEN] = 1'b1; v_mf[`F_WREG] = {2'b00, r0}; end   // to Dn.W
  else begin                                                         // to memory: no read (68010 and later), store only
    v_mf[`F_LA] = ea_lac(m3, r0, `SZ_W, 2'd1, 4'd2); v_mf[`F_LA_EN] = 1'b0;
    v_mf[`F_SA_EN] = 1'b1; v_mf[`F_SA_SAME] = 1'b1;
  end
end
always @* begin                                                     // MOVE <ea>,CCR/SR
  v_mt = {`UW{1'b0}};
  v_mt[`F_LA] = ea_lac(m3, r0, `SZ_W, 2'd1, 4'd2);                  // read the word source
  v_mt[`F_ASEL] = src_w[38:37]; v_mt[`F_AREG] = src_w[36:32]; v_mt[`F_IMM] = src_w[31:0];
  v_mt[`F_UNIT] = `UN_SYS; v_mt[`F_LSZ] = `SZ_W; v_mt[`F_SZ] = mk[0] ? `SZ_W : `SZ_B;   // SZ_W writes SR, SZ_B the CCR
  v_mt[`F_OP] = srw_c ? `SY_MVSRN : `SY_MVSR;                       // with (An)+/-(An): no redirect here (step 1 does it)
end
always @* begin                                                     // ORI/ANDI/EORI #,CCR/SR
  v_im = {`UW{1'b0}};
  v_im[`F_UNIT] = `UN_SYS; v_im[`F_SZ] = w0[6] ? `SZ_W : `SZ_B;     // bit 6: SR (word) or CCR (byte)
  v_im[`F_OP] = (w0[11:9] == 3'b000) ? `SY_ORSR : (w0[11:9] == 3'b001) ? `SY_ANSR : `SY_EOSR;
  v_im[`F_ASEL] = `AS_IMM; v_im[`F_IMM] = dcode(3'd2, 2'd1, 17'h0); // the immediate word w1
end
always @* begin                                                     // MOVE USP,An / MOVE An,USP
  v_mu = {`UW{1'b0}};
  v_mu[`F_UNIT] = `UN_ALU; v_mu[`F_OP] = `OP_MOV; v_mu[`F_SZ] = `SZ_L; v_mu[`F_ASEL] = `AS_REG; v_mu[`F_WEN] = 1'b1;
  if (w0[3]) begin v_mu[`F_AREG] = `R_USP; v_mu[`F_WREG] = {2'b01, r0}; end   // USP -> An
  else begin v_mu[`F_AREG] = {2'b01, r0}; v_mu[`F_WREG] = `R_USP; end         // An -> USP
end
always @* begin                                                     // MOVEC
  v_mc = {`UW{1'b0}};
  if (cr_sys) begin                                                 // a system unit register, its index in IMM
    v_mc[`F_UNIT] = `UN_SYS; v_mc[`F_IMM] = dcon({29'h0, cr_idx}); v_mc[`F_SZ] = `SZ_L;
    if (w0[0]) begin v_mc[`F_OP] = `SY_MCW; v_mc[`F_ASEL] = `AS_REG; v_mc[`F_AREG] = rn; end   // Rn -> control register
    else begin v_mc[`F_OP] = `SY_MCR; v_mc[`F_WEN] = 1'b1; v_mc[`F_WREG] = rn; end            // control register -> Rn
  end else if (cr_sp) begin                                         // a stack pointer: a register move
    v_mc[`F_UNIT] = `UN_ALU; v_mc[`F_OP] = `OP_MOV; v_mc[`F_SZ] = `SZ_L; v_mc[`F_ASEL] = `AS_REG; v_mc[`F_WEN] = 1'b1;
    if (w0[0]) begin v_mc[`F_AREG] = rn; v_mc[`F_WREG] = cr_reg; end
    else begin v_mc[`F_AREG] = cr_reg; v_mc[`F_WREG] = rn; end
  end else v_mc[`F_EXC] = 8'd4;                                     // unknown control register
end
// STOP: a constant (the immediate is a data code): no always block without inputs
wire [`UW-1:0] v_st = ({{(`UW - 4){1'b0}}, `UN_SYS} << `U_UNIT) | ({{(`UW - 5){1'b0}}, `SY_STOP} << `U_OP) |
                      ({{(`UW - 2){1'b0}}, `SZ_W} << `U_SZ) | ({{(`UW - 2){1'b0}}, `AS_IMM} << `U_ASEL) |
                      ({{(`UW - 32){1'b0}}, dcode(3'd2, 2'd1, 17'h0)} << `U_IMM);
wire [`UW-1:0] v_rs = {{(`UW - 9){1'b0}}, `SY_RESET, `UN_SYS};   // RESET (a constant: no always block without inputs)
always @* begin                                                     // TRAPV, TRAPcc
  v_tc = {`UW{1'b0}};
  // TRAPV is TRAPcc with condition VS (9); vector 7 is raised when the condition holds
  v_tc[`F_UNIT] = `UN_SYS; v_tc[`F_OP] = `SY_TRAPCC; v_tc[`F_CC] = trapv ? 4'h9 : w0[11:8]; v_tc[`F_EXC] = 8'd7;
end
always @* begin                                                     // CHK
  v_ck = {`UW{1'b0}};
  v_ck[`F_LA] = ea_lac(m3, r0, csz, 2'd1, 4'd2);                    // read the bound
  v_ck[`F_ASEL] = src_c[38:37]; v_ck[`F_AREG] = src_c[36:32]; v_ck[`F_IMM] = src_c[31:0];   // A = bound
  v_ck[`F_UNIT] = `UN_SYS; v_ck[`F_OP] = `SY_CHK; v_ck[`F_SZ] = csz; v_ck[`F_LSZ] = csz;   // compare, latch the result
  v_ck[`F_BREG] = {2'b00, r9};                                      // B = Dn
end
always @* begin                                                     // exception only: privilege, TRAP, BKPT, line A, line F
  v_ex = {`UW{1'b0}};
  // the vector to raise after the empty uop commits: 8, 32 + n, 4, 10 or 11 (a privilege violation wins)
  v_ex[`F_EXC] = ({8{pv_c}} & 8'd8) | ({8{!pv_c && trap}} & {4'b0010, w0[3:0]}) | ({8{!pv_c && bkpt}} & 8'd4) |
                 ({8{!pv_c && linea}} & 8'd10) | ({8{!pv_c && linef}} & 8'd11);
end
wire        n_pv = !pv_c;
// the one-hot AND-OR merge: each case uop ANDed with its select, all ORed; v_ex is zero unless it applies
wire [`UW-1:0] v = ({`UW{n_pv && mvsr_c && !mk[1]}} & v_mf) | ({`UW{n_pv && mvsr_c && mk[1]}} & v_mt) | ({`UW{n_pv && immsr}} & v_im) |
                   ({`UW{n_pv && musp}} & v_mu) | ({`UW{n_pv && movec}} & v_mc) | ({`UW{n_pv && stop}} & v_st) |
                   ({`UW{n_pv && rset}} & v_rs) | ({`UW{n_pv && (trapv || trapc)}} & v_tc) | ({`UW{n_pv && chk_c}} & v_ck) | v_ex;

assign hit    = h;
// MOVES An,(An)+ / -(An) stores the updated An (as the 68020 does): step 0 computes An +- size into T0, and the
// sequencer stores T0 in place of An. w1 bit 11 set = register to memory, bit 15 set = An, bits 14-12 = the register.
wire       mvbase = mvs_c && !pv_c && (m3 == 3'd3 || m3 == 3'd4) && w1[11] && w1[15] && w1[14:12] == r0;
wire [31:0] mvstp = {28'h0, stepb(w0[7:6], r0)};                 // the (An)+ / -(An) step of the operand size
reg  [`UW-1:0] v0;
always @* begin
  v0 = {`UW{1'b0}};
  v0[`F_UNIT] = `UN_ALU; v0[`F_OP] = `OP_ADD; v0[`F_SZ] = `SZ_L; v0[`F_ASEL] = `AS_IMM; v0[`F_IMM] = dcon((m3 == 3'd3) ? mvstp : (32'h0 - mvstp));
  v0[`F_BREG] = {2'b01, r0}; v0[`F_WEN] = 1'b1; v0[`F_WREG] = `R_T0;   // T0 = An +- step
end
// The uop is gated by pg, this decoder's gate predecoded with the instruction buffer entry (p68_pdec).
assign ug   = h;
assign u      = pg ? (v | (mvbase ? v0 : {`UW{1'b0}})) : {`UW{1'b0}};
// system sequencer class; none in user mode for the privileged ones (they only raise vector 8)
assign seq    = (!h || pv) ? `SS_NONE : rte ? `SS_RTE : chk2 ? `SS_CHK2 : moves ? `SS_MOVES : chk ? `SS_CHK :
                srw ? `SS_SRW : `SS_NONE;
// parameters {EA words w3 w2, w1 bits 15-11 (D/A, Rn, direction), size (MOVES bits 7-6, else bits 10-9), EA mode, reg};
// MOVES is recognized by its opcode alone, because the parameters are read only for a legal start
assign spar   = {w3, w2, w1[15:11], (w0[15:8] == 8'h0E) ? w0[7:6] : w0[10:9], m3, r0};
assign is_rst = rset && sup;                                         // RESET runs only in supervisor mode
endmodule
`default_nettype wire
