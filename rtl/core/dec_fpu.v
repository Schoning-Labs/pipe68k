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
// ==== dec_fpu.v: the group decoder for FPU instructions (coprocessor 1) ===============================================
//
// Abbreviations: DS = the decode stage (p68_decode.v), FPU = floating-point unit, EA = effective address, FPn/FPm = FP
// registers, FPcr = one of the FPU control registers FPCR, FPSR, FPIAR, NULL = the FPU state after reset (FSAVE then writes
// a one-long frame), T0 = the sequencers' first temporary register.
//
// ---- Where it sits ----------------------------------------------------------------------------------------------------
//
// One of the group decoders of DS: it sees the opcode word w0 and the three words behind it, says whether the opcode is
// its own (hit), and gives step 0 of a sequence plus the parameters that seq_fpu.v uses for the steps after it. Every
// FPU instruction is a sequence of class SQ_FPU. Combinational logic only. It is only built with P68_FPU set; without
// the FPU, line F opcodes go to dec_sys.v and raise vector 11.
//
// ---- The instructions -------------------------------------------------------------------------------------------------
//
// The 68020 hands line F opcodes with coprocessor id 1 (bits 11-9 = 001) to the FPU:
//
//     general         1111 001 000 EA   + command word: opclass(15-13) R/M(14) source(12-10) dest(9-7) opmode(6-0)
//                       opclass 000 FPm to FPn, 010 <ea> to FPn (source format in 12-10, 7 = FMOVECR), 011 FPn to <ea>,
//                       100 / 101 <ea> to FPcr list / FPcr list to <ea>, 110 / 111 FMOVEM.X to / from registers
//     FDBcc, FScc,    1111 001 001 EA   + condition word (predicate in bits 5-0)
//     FTRAPcc
//     FBcc            1111 001 01s predicate   + 16-bit (s = 0) or 32-bit (s = 1) displacement
//     FSAVE, FRESTORE 1111 001 100 EA / 1111 001 101 EA   (privileged)
//
// The forms the FPU traps get no hit here (line F, vector 11 through dec_sys): the transcendentals, FMOD, FREM, opmodes
// 0x40-0x7F, packed decimal, #imm .D/.X/.P, FMOVEM.L #imm with several registers, FMOVEM.X with a dynamic list, and
// illegal EAs. FSAVE and FRESTORE in user mode: no hit (dec_sys raises the privilege violation for any coprocessor id).
// The reasons for what traps are in p68_fpu.v.
//
// ---- What it gives ----------------------------------------------------------------------------------------------------
//
// Step 0: FP_GO (source FPm, FMOVE out, FMOVECR), FP_IN 0 with the EA (sources in memory, Dn, #imm), a LEA of the EA into
// T0 (.D/.X sources, FMOVEM, control register lists, FSAVE, FRESTORE), FP_CHK (one control register) or empty
// (conditionals). A FIRST uop changes no architectural FPU state (p68_fpu takes no interrupt term), which is why the
// first step is always one of these harmless uops.
// Parameters (spar): [3:0] kind, [19:4] command / condition word (FBcc: the opcode), [25:20] EA mode and register,
// [57:26] the EA's two extension words (FBcc: displacement words), [58] FSAVE in the NULL state, [59] a trailing redirect.
// Length: general operations with #imm .B/.W are three words; the fetch predecode gives four (it sees one word), so
// lov/lnew override the issue length (DS latches them when the sequence starts).
//
// A pending arithmetic trap (pv) is raised by the next FPU instruction before it does anything: its decoder uop is then
// an empty uop with that vector (format 0 frame, the stacked PC is this instruction), as the 68882 does with a 68020.
`include "p68_uop.vh"                    // the uop format
`include "p68_fpu.vh"                    // FPU codes and sequence kinds
`default_nettype none
module dec_fpu (
  input  wire [15:0] w0, w1, w2, w3,     // the opcode word and the three words behind it
  input  wire        sup,                // supervisor mode
  input  wire        nul,                // NULL state as DS sees it (FSAVE frame size)
  input  wire        en,                 // an FPCR exception enable is set (committed): sequences that can set a pending
                                         // trap end with a redirect, so pv below is committed when the next one decodes
  input  wire [5:0]  pv,                 // pending trap vector: the next FPU instruction raises it before it executes
  output wire        hit,                // the opcode is an FPU instruction (coprocessor 1, not a privilege violation)
  input  wire [1:0]  pg,             // predecoded {hh, h} of this instruction buffer entry (registered)
  output wire [1:0]  ug,             // {hh, h} for that predecode
  output wire [`UW-1:0] u,           // step 0's uop
  output wire [1:0]  xs,             // EA extension words start this many words after the opcode
  output wire [3:0]  seq,            // sequence class (SQ_FPU, or none)
  output wire [63:0] spar,           // the sequence parameters
  output wire        lov,            // override the issue length ...
  output wire [3:0]  lnew            // ... with this one
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"                    // shared decoder functions (EA descriptors, data codes)
wire [2:0] m3 = w0[5:3], r0 = w0[2:0];   // the EA mode and register fields
wire       cp1  = (w0[15:9] == 7'b1111001);           // line F, coprocessor 1
wire       gen  = cp1 && (w0[8:6] == 3'b000);         // a general instruction (command word follows)
wire [2:0] opc  = w1[15:13];                          // opclass
wire [2:0] sf   = w1[12:10];                          // source format or register
wire [6:0] om   = w1[6:0];                            // opmode
// EA classes of the 68k: data register, address register, immediate, PC-relative, memory, memory alterable, control,
// control alterable
wire       e_dn = (m3 == 3'd0), e_an = (m3 == 3'd1), e_imm = (m3 == 3'd7 && r0 == 3'd4);
wire       e_pc = (m3 == 3'd7 && (r0 == 3'd2 || r0 == 3'd3));
wire       e_mem = (m3 >= 3'd2 && m3 <= 3'd6) || (m3 == 3'd7 && r0 <= 3'd3);     // memory, PC-relative included
wire       e_mal = (m3 >= 3'd2 && m3 <= 3'd6) || (m3 == 3'd7 && r0 <= 3'd1);     // memory alterable
wire       e_ctl = (m3 == 3'd2) || (m3 == 3'd5) || (m3 == 3'd6) || (m3 == 3'd7 && r0 <= 3'd3);
wire       e_cal = (m3 == 3'd2) || (m3 == 3'd5) || (m3 == 3'd6) || (m3 == 3'd7 && r0 <= 3'd1);
// opmodes done in hardware: FMOVE, FINT, FINTRZ, FSQRT, FABS, FNEG, FGETEXP, FGETMAN, FDIV, FADD, FMUL, FSGLDIV, FSCALE,
// FSGLMUL, FSUB, FCMP, FTST
wire       om_ok = (om == 7'h00) || (om == 7'h01) || (om == 7'h03) || (om == 7'h04) || (om == 7'h05) || (om == 7'h18) ||
                   (om == 7'h1a) || (om == 7'h1b) || (om == 7'h1e) || (om == 7'h1f) || (om == 7'h20) || (om == 7'h22) ||
                   (om == 7'h23) || (om == 7'h24) || (om == 7'h26) || (om == 7'h27) || (om[6:3] == 4'b0101) ||
                   (om[6:3] == 4'b0111);                                                 // 0x28-0x2F FSUB, 0x38-0x3F FCMP/FTST
wire       cr_ok = !om[6];                                            // every ROM offset 0x00-0x3F (undefined ones as WinUAE)
wire       fsz1  = (sf == 3'd0) || (sf == 3'd1) || (sf == 3'd4) || (sf == 3'd6);     // L S W B
wire       fszm  = (sf == 3'd5) || (sf == 3'd2);                                     // D X
// general forms
wire       g_rr  = gen && (opc == 3'd0) && om_ok;                                    // FPm to FPn
wire       g_cr  = gen && (opc == 3'd2) && (sf == 3'd7) && cr_ok;                    // FMOVECR
wire       g_e1  = gen && (opc == 3'd2) && fsz1 && om_ok && (e_dn || e_mem || e_imm);   // one-long source
wire       g_em  = gen && (opc == 3'd2) && fszm && om_ok && e_mem;                  // .D/.X source in memory
wire       g_out = gen && (opc == 3'd3) && ((fsz1 && (e_dn || e_mal)) || (fszm && e_mal));   // FMOVE out (no packed)
wire [2:0] lst   = (w1[12:10] == 3'd0) ? 3'd1 : w1[12:10];               // an empty control register list is FPIAR (68882)
wire       one   = (lst == 3'd4) || (lst == 3'd2) || (lst == 3'd1);      // the list names one register
wire       g_ctr = gen && (opc == 3'd4 || opc == 3'd5) && ((e_dn && one) || (e_an && lst == 3'd1) || (e_imm && opc == 3'd4 && one));
wire       g_ctm = gen && (opc == 3'd4 || opc == 3'd5) && (opc == 3'd4 ? e_mem : e_mal);
wire       g_mvm = gen && (opc == 3'd6 || opc == 3'd7) && !w1[11] &&      // static list only
                   (opc == 3'd7 ? (e_cal || m3 == 3'd4) : (e_ctl || m3 == 3'd3));   // list type and EA may disagree (68882)
// conditionals, FSAVE, FRESTORE
wire       cnd_ok = (w1[15:6] == 10'h0);                               // predicates 0x20-0x3F act as 0x00-0x1F
wire       c_dbcc = cp1 && (w0[8:6] == 3'b001) && e_an && cnd_ok;     // mode 001 means FDBcc Dn here
wire       c_trap = cp1 && (w0[8:6] == 3'b001) && (m3 == 3'd7) && (r0 >= 3'd2 && r0 <= 3'd4) && cnd_ok;
wire       c_scc  = cp1 && (w0[8:6] == 3'b001) && (e_dn || e_mal) && cnd_ok;
wire       c_bcc  = cp1 && (w0[8:7] == 2'b01);
wire       c_sav  = cp1 && (w0[8:6] == 3'b100) && sup && (e_cal || m3 == 3'd4);
wire       c_rst  = cp1 && (w0[8:6] == 3'b101) && sup && (e_ctl || m3 == 3'd3);
// hit: every coprocessor-1 word except FSAVE/FRESTORE with a legal EA in user mode (dec_sys: privilege violation), so the
// hit is a shallow opcode test; the forms not done here get an empty uop with vector 11 (line F) and no sequence
wire       cpsr = (w0[8:6] == 3'b100 && (e_cal || m3 == 3'd4)) || (w0[8:6] == 3'b101 && (e_ctl || m3 == 3'd3));
wire       hh   = cp1 && !(cpsr && !sup);
wire       h = g_rr || g_cr || g_e1 || g_em || g_out || g_ctr || g_ctm || g_mvm || c_dbcc || c_trap || c_scc || c_bcc || c_sav || c_rst;

// the exact sequence kind and transfer size (the parameters below use the cheaper forms knd_c and tot_c instead)
wire [3:0] knd = g_rr ? `FK_GENR : g_cr ? `FK_CR : g_e1 ? `FK_GEN1 : g_em ? `FK_GENM : g_out ? `FK_OUT : g_ctr ? `FK_CTLR :
                 g_ctm ? `FK_CTLM : g_mvm ? `FK_MVM : (c_dbcc || c_trap || c_scc || c_bcc) ? `FK_COND : c_sav ? `FK_SAVE : `FK_REST;
wire [1:0] s1z  = (sf == 3'd4) ? `SZ_W : (sf == 3'd6) ? `SZ_B : `SZ_L;  // the size of a one-long source
wire [7:0] msz  = (sf == 3'd5) ? 8'd8 : 8'd12;                         // .D / .X bytes
wire [3:0] nrg  = {3'h0, w1[0]} + {3'h0, w1[1]} + {3'h0, w1[2]} + {3'h0, w1[3]} + {3'h0, w1[4]} + {3'h0, w1[5]} + {3'h0, w1[6]} + {3'h0, w1[7]};   // registers in an FMOVEM.X list
wire [7:0] tot  = g_em ? msz : g_ctm ? {3'h0, {1'b0, lst[2]} + {1'b0, lst[1]} + {1'b0, lst[0]}, 2'b00} :
                  g_mvm ? ({4'h0, nrg} << 3) + ({4'h0, nrg} << 2) : nul ? 8'd4 : 8'd60;   // bytes of the transfer
wire [15:0] cwd = c_bcc ? w0 : (g_ctr || g_ctm) ? {w1[15:13], lst, w1[9:0]} : w1;
wire [31:0] exw = c_bcc ? {w1, w2} : (c_sav || c_rst) ? {w1, w2} : {w2, w3};

// The uop is an AND-OR of one-hot cases; it counts only with the hit h, so the cases are told apart by the opcode and
// command word fields without the legality terms (fewer LUT levels in DS's decode path).
wire       gn_c  = (w0[8:6] == 3'b000);                                  // a general instruction
wire       ctl_c = gn_c && (opc == 3'd4 || opc == 3'd5);                 // a control register move
wire       ctr_c = e_dn || e_an || e_imm;                                  // one control register to / from Rn or #imm
wire       go_c  = gn_c && (opc == 3'd0 || (opc == 3'd2 && sf == 3'd7) || opc == 3'd3);   // step 0 is FP_GO
wire       e1_c  = gn_c && opc == 3'd2 && sf != 3'd7 && !fszm;           // step 0 is FP_IN with the EA
wire       sr_c  = (w0[8:6] == 3'b100) || (w0[8:6] == 3'b101);          // FSAVE or FRESTORE
wire       mp_c  = (gn_c && opc == 3'd2 && fszm) || (ctl_c && !ctr_c) || (gn_c && (opc == 3'd6 || opc == 3'd7)) || sr_c;   // step 0 is the LEA into T0
wire       ct_c  = ctl_c && ctr_c;                                       // step 0 is FP_CHK
wire [7:0] tot_c = (gn_c && opc == 3'd2) ? msz : ctl_c ? {3'h0, {1'b0, lst[2]} + {1'b0, lst[1]} + {1'b0, lst[0]}, 2'b00} :
                   gn_c ? ({4'h0, nrg} << 3) + ({4'h0, nrg} << 2) : nul ? 8'd4 : 8'd60;   // bytes of the transfer
wire [38:0] src  = ea_srcc(m3, r0, ctl_c ? `SZ_L : s1z, 2'd2);          // the source operand of a register or #imm EA
reg  [`UW-1:0] v_e1, v_mp;
// a constant (the immediate is a data code, see p68_dec.vh): no always block without inputs
wire [`UW-1:0] v_go = ({{(`UW - 4){1'b0}}, `UN_FPU} << `U_UNIT) | ({{(`UW - 5){1'b0}}, `FP_GO} << `U_OP) |
                      ({{(`UW - 32){1'b0}}, dcode(3'd2, 2'd1, 17'h0)} << `U_IMM);
// FP_IN 0 with the EA: the first (or only) long of the source
always @* begin
  v_e1 = {`UW{1'b0}};
  v_e1[`F_UNIT] = `UN_FPU; v_e1[`F_OP] = `FP_IN; v_e1[`F_SZ] = `SZ_L; v_e1[`F_LSZ] = s1z;
  v_e1[`F_LA] = ea_lac(m3, r0, s1z, 2'd2, 4'd4);
  v_e1[`F_ASEL] = src[38:37]; v_e1[`F_AREG] = src[36:32]; v_e1[`F_IMM] = src[31:0];
end
always @* begin                                                   // multi-piece transfers: LEA of the EA into T0 (DR puts T0
  v_mp = {`UW{1'b0}};                                             // in SA_BREG for FPU opcodes); (An)+ / -(An) step the whole
  v_mp[`F_LA] = sr_c ? ea_lac(m3, r0, `SZ_L, 2'd1, 4'd2) : ea_lac(m3, r0, `SZ_L, 2'd2, 4'd4);   // size here, the pieces
  v_mp[`F_LA_EN] = 1'b0; v_mp[`F_LEA] = 1'b1; v_mp[`F_SA_BREG] = `R_T0;                          // address T0 + offset
  if (m3 == 3'd3) begin v_mp[`F_LA_UDL] = tot_c; v_mp[`F_LA_UEN] = (w0[8:6] != 3'b101); end    // (An)+: FRESTORE steps An later from the frame size
  if (m3 == 3'd4) begin v_mp[`F_LA_UDL] = 8'h0 - tot_c; v_mp[`F_LA_DISP] = dcon(32'h0 - {24'h0, tot_c}); end   // -(An): T0 = An - size
end
wire [`UW-1:0] v_ct = {{(`UW - 9){1'b0}}, `FP_CHK, `UN_FPU};      // empty step 0 (an FPU uop; a constant)
// empty step 0 for the rest (conditionals: their uop leaves the NULL state; FSAVE does not)
wire [`UW-1:0] v = ({`UW{go_c}} & v_go) | ({`UW{e1_c}} & v_e1) | ({`UW{mp_c}} & v_mp) | ({`UW{ct_c}} & v_ct);
wire [`UW-1:0] vf = {{(`UW - 8){1'b0}}, 8'd11} << `U_EXC;           // empty uop raising vector 11 (line F)
wire [`UW-1:0] vp = {{(`UW - 8){1'b0}}, 2'b00, pv} << `U_EXC;       // empty uop raising the pending trap (format 0, this PC)
wire       ptk = (pv != 6'd0) && !(w0[8:7] == 2'b10);               // not FSAVE / FRESTORE
// trailing redirect (seq_fpu, parameter bit 59): operations that can set a pending trap while an enable is set, FPCR
// loads (DS then sees the enables), FSAVE while a trap is pending or enabled (it clears the pending trap)
wire       ser = ((g_rr || g_cr || g_e1 || g_em || g_out) && en) || ((g_ctr || g_ctm) && opc == 3'd4 && lst[2]) ||
                 (c_sav && (en || pv != 6'd0));
assign hit  = hh;
// hh and h are predecoded with the instruction buffer entry (pg = {hh, h} of this entry's words, registered), so the
// final select below has registers in front of it
assign ug   = {hh, h};
assign u    = !pg[1] ? {`UW{1'b0}} : (ptk && pg[0]) ? vp : pg[0] ? v : vf;
assign xs   = (w0[8:7] == 2'b10) ? 2'd0 : 2'd1;                 // EA extension words behind the command word (not FSAVE/FRESTORE)
assign seq  = (h && !ptk) ? `SQ_FPU : `SQ_NONE;
// The parameters are read only for an FPU sequence (h, its class exact), so they come from the opcode and command word
// fields alone: kind, command word, extension words and trailing redirect without the legality terms.
wire [3:0] knd_c = gn_c ? ((opc == 3'd0) ? `FK_GENR : (opc == 3'd2) ? ((sf == 3'd7) ? `FK_CR : fszm ? `FK_GENM : `FK_GEN1) :
                           (opc == 3'd3) ? `FK_OUT : (opc == 3'd4 || opc == 3'd5) ? (ctr_c ? `FK_CTLR : `FK_CTLM) : `FK_MVM) :
                   (w0[8:6] == 3'b100) ? `FK_SAVE : (w0[8:6] == 3'b101) ? `FK_REST : `FK_COND;
wire [15:0] cwd_c = (w0[8:7] == 2'b01) ? w0 : (gn_c && (opc == 3'd4 || opc == 3'd5)) ? {w1[15:13], lst, w1[9:0]} : w1;
wire [31:0] exw_c = (w0[8:7] == 2'b01 || w0[8:6] == 3'b100 || w0[8:6] == 3'b101) ? {w1, w2} : {w2, w3};
wire       ser_c = (gn_c && (opc == 3'd0 || opc == 3'd2 || opc == 3'd3) && en) || (gn_c && opc == 3'd4 && lst[2]) ||
                   (w0[8:6] == 3'b100 && (en || pv != 6'd0));
assign spar = cp1 ? {4'h0, ser_c, nul, exw_c, m3, r0, cwd_c, knd_c} : 64'h0;
assign lov  = g_e1 && e_imm && (sf == 3'd4 || sf == 3'd6) && !ptk;   // #imm .B/.W: opcode, command word, one immediate word
assign lnew = 4'd3;
endmodule
`default_nettype wire
