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
// dec_flow: the group decoder for program flow: JMP, JSR, BSR, RTS, RTD, RTR, PEA, LINK and UNLK (lines 4 and 6).
//
// Where it sits. One of the group decoders of the decode stage (DS, p68_decode.v): it sees the opcode word w0 and the
// words behind it, says whether the opcode is its own (hit), and gives one micro-op (uop, p68_uop.vh) or starts a
// sequence of class SQ_FLOW (seq_flow.v, inside the sequencer p68_seq.v). The uops run through DR (register stage
// before address generation), AG (address generation), OC (operand cycle), EX (execute) and WB (write-back).
// Combinational logic only.
//
// The instructions.
//
//     JSR  <ea>     0100 1110 10 EA         push the return address, jump to the EA (control modes only)
//     JMP  <ea>     0100 1110 11 EA         jump to the EA
//     BSR  label    0110 0001 disp8         push the return address, branch to PC + 2 + disp (disp 00 / FF as in Bcc)
//     RTS           0100 1110 0111 0101     pop the PC
//     RTD  #d16     0100 1110 0111 0100     pop the PC, then add d16 to A7 (68010 and later)
//     RTR           0100 1110 0111 0111     pop CCR (a word), then the PC
//     PEA  <ea>     0100 1000 01 EA         push the EA (control modes only)
//     LINK An,#d16  0100 1110 0101 0 An     push An, An = A7, A7 += d16 (makes a stack frame)
//     LINK.L An,#d32 0100 1000 0000 1 An    the same with a 32-bit displacement (68020)
//     UNLK An       0100 1110 0101 1 An     A7 = An, then pop An
//
// The return address of JSR and BSR is the address of the next instruction. The legality and length of all of these is
// the pd_flow function in p68_flowf.vh, shared with the fetch predecode so that the two can never disagree.
//
// How they become uops.
//   JMP and JSR: one uop. The EA is computed by the load address adder in AG, with the load switched off, and BRA makes
//   AG redirect the fetch to it. JSR also pushes the return address: a store at -4(A7) with A7 updated by -4, where the
//   value comes from the system unit operation SY_NPC (next PC). A decoder uop normally cannot name A7 as a base,
//   because DR takes the base register numbers straight from the opcode word, but JMP and JSR can: their bits 11-9 are
//   111, so the store base read from those bits is A7, and the EA base An is bits 2-0 as for every EA.
//   BSR: one uop too, the same push with a PC-relative branch target; DS names A7 as its store base specially.
//   RTS: one uop. It loads the PC from (A7)+ and jumps (SY_JMP). DS predicts the return address from its return stack
//   and gives it to the uop, so the fetch has already continued there; the uop redirects only when that prediction was
//   wrong. RTS was first a two-uop sequence (move A7, then jump); making it one uop saved two clocks per return (CPI 1.091
//   to 1.007, calls 1.446 to 1.216, HISTORY.md section 4). This needed the register file to restore one clock after a
//   redirect, from the committed copy that already holds this uop's A7 update; before that, a redirecting uop was not
//   allowed to update An in AG.
//   RTD, RTR, PEA, LINK, UNLK: sequences of class SQ_FLOW, step 0 (this uop) empty; seq_flow.v has the steps. BSR and
//   RTS were sequences too before they became single uops, and seq_flow still has their kinds.
// Sequence parameters (par[2:0] = kind):
//   1 BSR  [34:3] displacement           2 RTS            3 RTD  [34:3] displacement (sign-extended word)    4 RTR
//   5 UNLK [5:3] An                      6 PEA  [5:3] mode, [8:6] register, [24:9] ext word 0, [40:25] ext word 1
//   7 LINK [5:3] An, [37:6] displacement (sign-extended word for LINK.W)
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`include "p68_sys.vh"
`default_nettype none
module dec_flow (
  input  wire [15:0] w0, w1, w2, w3,         // opcode and the next words (displacements, EA extension words)
  output wire        hit,                    // any instruction of this group with a legal EA
  input  wire        pg,             // predecoded gate (this decoder's ug from p68_pdec, a register in the entry)
  output wire        ug,             // the gate of the uop, for p68_pdec's own copy of this decoder
  output wire [`UW-1:0] u,                   // the uop (JMP, JSR, BSR, RTS; empty for the sequences)
  output wire [3:0]  seq,                    // SQ_FLOW for RTD, RTR, PEA, LINK, UNLK
  output wire [63:0] spar                    // sequence parameters (layout above)
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
`include "p68_sysf.vh"
`include "p68_flowf.vh"
wire [2:0] m3 = w0[5:3], r0 = w0[2:0];                        // EA mode and register (JMP, JSR, PEA), An (LINK, UNLK)
wire [5:0] pf  = pd_flow(w0);                                 // {hit, sequence, length} (p68_flowf.vh)
wire       h   = pf[5];
wire       bsr = h && (w0[15:8] == 8'h61);                        // BSR is one uop (DS gives it A7 as SA base)
wire       rts = h && (w0 == 16'h4E75);                           // RTS is one uop
wire       isj = h && !pf[4] && !bsr && !rts;                     // JMP, JSR (the remaining non-sequences)
wire       jsr = isj && !w0[6];                                   // bit 6: 0 JSR, 1 JMP
// sequence kind from the opcode (BSR and RTS kinds remain from when they were sequences)
wire [2:0] kind = (w0[15:8] == 8'h61) ? 3'd1 : (w0 == 16'h4E75) ? 3'd2 : (w0 == 16'h4E74) ? 3'd3 : (w0 == 16'h4E77) ? 3'd4 :
                  (w0[15:3] == 13'b0100111001011) ? 3'd5 : (w0[15:6] == 10'b0100100001) ? 3'd6 : 3'd7;
// BSR displacement as a value (for the parameters) and as a data code (for the uop): 00 = word, FF = long, else byte
wire [31:0] bd  = (w0[7:0] == 8'h00) ? {{16{w1[15]}}, w1} : (w0[7:0] == 8'hFF) ? {w1, w2} : {{24{w0[7]}}, w0[7:0]};
wire [31:0] bdc = (w0[7:0] == 8'h00) ? dcode(3'd1, 2'd1, 17'h0) : (w0[7:0] == 8'hFF) ? dcode(3'd4, 2'd1, 17'h0) : dcon({{24{w0[7]}}, w0[7:0]});
// The uop counts only with the hit, so its forms are told apart by their opcode patterns alone (cheaper than via pf)
wire       isj_c = (w0[15:7] == 9'b010011101), jsr_c = isj_c && !w0[6], rts_c = (w0 == 16'h4E75), bsr_c = (w0[15:8] == 8'h61);
reg  [`UW-1:0] v;
always @* begin
  v = {`UW{1'b0}};
  if (isj_c) begin                                                  // JMP / JSR <ea>
    v[`F_LA] = ea_lac(m3, r0, `SZ_L, 2'd1, 4'd2);                   // the target is the EA's address
    v[`F_LA_EN] = 1'b0; v[`F_LA_UEN] = 1'b0;                        // no load, no An update
    v[`F_BRA] = 1'b1;                                               // AG redirects to it
    if (jsr_c) begin                                                // JSR: push the next instruction's address
      v[`F_UNIT] = `UN_SYS; v[`F_OP] = `SY_NPC; v[`F_SZ] = `SZ_L;   // EX gives the next PC as the result
      v[`F_SA_EN] = 1'b1; v[`F_SA_BSEL] = `BA_REG; v[`F_SA_BREG] = 5'd15;   // store base A7
      v[`F_SA_DISP] = dcon(32'hFFFFFFFC); v[`F_SA_UEN] = 1'b1; v[`F_SA_UDL] = 8'hFC;   // at A7 - 4, A7 -= 4
    end
  end
  // RTS loads the PC from (A7)+ and jumps (SY_JMP; DS gives it A7 as LA base and the return stack's predicted
  // address; it redirects only when that address was wrong). The register file restores one clock after a redirect,
  // from the committed copy that already holds this uop's A7 update, so the update in AG is safe.
  if (rts_c) begin
    v[`F_UNIT] = `UN_SYS; v[`F_OP] = `SY_JMP; v[`F_SZ] = `SZ_L;
    v[`F_LA_EN] = 1'b1; v[`F_LA_BSEL] = `BA_REG; v[`F_LA_BREG] = 5'd15; v[`F_LA_DISP] = 32'h0; v[`F_LSZ] = `SZ_L;   // long at (A7)
    v[`F_ASEL] = `AS_MEM; v[`F_LA_UEN] = 1'b1; v[`F_LA_UDL] = 8'd4;                                               // A7 += 4
  end
  if (bsr_c) begin                                                  // BSR: push the next PC, branch to PC + 2 + disp
    v[`F_UNIT] = `UN_SYS; v[`F_OP] = `SY_NPC; v[`F_SZ] = `SZ_L;
    v[`F_LA_BSEL] = `BA_PC; v[`F_LA_XK] = 4'd2; v[`F_LA_DISP] = bdc; v[`F_BRA] = 1'b1; v[`F_BR] = 1'b1;   // F_BR: BTC update
    v[`F_SA_EN] = 1'b1; v[`F_SA_BSEL] = `BA_REG; v[`F_SA_BREG] = 5'd15;
    v[`F_SA_DISP] = dcon(32'hFFFFFFFC); v[`F_SA_UEN] = 1'b1; v[`F_SA_UDL] = 8'hFC;
  end
end
// ---- sequence parameters ----------------------------------------------------------------------------------------
wire [31:0] ld32 = w0[3] ? {w1, w2} : {{16{w1[15]}}, w1};      // LINK.L / LINK.W displacement (LINK.L has bit 3 set)
reg [63:0] sp;
always @* begin
  sp = 64'h0;
  sp[2:0] = kind;
  case (kind)
    3'd1: sp[34:3] = bd;                                            // BSR (no longer started as a sequence)
    3'd3: sp[34:3] = {{16{w1[15]}}, w1};                            // RTD: sign-extended d16
    3'd5: sp[5:3] = r0;                                             // UNLK An
    3'd6: begin sp[5:3] = m3; sp[8:6] = r0; sp[24:9] = w1; sp[40:25] = w2; end   // PEA: the EA and its words
    3'd7: begin sp[5:3] = r0; sp[37:6] = ld32; end                  // LINK An and displacement
    default: ;
  endcase
end
assign hit  = h;
// The uop is gated by pg, this decoder's gate predecoded with the instruction buffer entry (p68_pdec); only the four
// single-uop forms have a gate, so the sequences' step 0 is empty.
assign ug   = h && (isj_c || bsr_c || rts_c);
assign u    = pg ? v : {`UW{1'b0}};
assign seq  = (h && !isj && !bsr && !rts) ? `SQ_FLOW : `SQ_NONE;
// The parameters are read only for a legal sequence of this decoder (its class is exact), so they are gated by the
// opcode pattern alone (0x4Exx, or 0x48xx with bit 7 clear for PEA and LINK.L); it excludes every other decoder's
// sequence opcodes.
assign spar = (w0[15:8] == 8'h4E || (w0[15:8] == 8'h48 && !w0[7])) ? sp : 64'h0;
endmodule
`default_nettype wire
