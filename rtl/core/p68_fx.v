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
// p68_fx: the pre-sequence for effective addresses in the 68020's full extension format.
//
// Where it sits. In the decode stage (DS, p68_decode.v), beside the decoders and the sequencers. It reads extension
// words from the instruction queue through its own port (wx, the word at queue head + xpos), keeps a small state machine
// on the core clock, and gives micro-ops (uops, p68_uop.vh) that DS sends on like sequencer uops, through DR (register
// stage before address generation), AG (address generation), OC (operand cycle), EX (execute) and WB (write-back).
//
// The problem it solves. The 68000's indexed modes (d8,An,Xn) and (d8,PC,Xn) have one extension word. The 68020 adds a
// full format, selected by bit 8 of that word:
//
//     15 D/A | 14-12 index reg | 11 W/L | 10-9 scale | 8 = 1 | 7 BS | 6 IS | 5-4 BD size | 3 = 0 | 2-0 I/IS
//     then the base displacement (bd: none, one or two words), then the outer displacement (od: none, one or two words)
//
// BS suppresses the base register (or the PC), IS suppresses the index, BD size 01 = no bd, 10 = word, 11 = long, and
// I/IS selects memory indirection: 000 none, 001/010/011 pre-indexed with null/word/long od (the pointer is read from
// base + index + bd, then od is added), 101/110/111 post-indexed (the pointer is read from base + bd, then index and od
// are added). One EA can so take up to 5 words, and an instruction with one up to 11. The decode window cannot afford
// that: the choice was between a window of 11 words with a length decode over several words, or a pre-sequence. The
// pre-sequence won because the full format is rare and a wide window would lengthen the queue-to-DS path for every
// instruction (HISTORY.md section 6). The longest instruction this walker reads, 11 words, is also the reason the
// instruction queue has 14 words (a 12-word queue hung).
//
// How it works. An instruction whose EA uses the full format (68020: base displacement null/word/long, base and index
// suppress, memory indirect pre- or post-indexed with outer displacement, PC-relative and ZPC forms) is run in two parts:
//   1. WALK: the extension words are read one per two clocks through the queue port wx (queue word at head + xpos) and
//      latched; this also gives the instruction length (up to 11 words). ISSUE: pre-sequence uops compute each such EA
//      into a temporary (source T1, MOVE destination T2), from registers only:
//        slot 0/2  U1: LEA  base + bd [+ index] -> Tk, or (memory indirect) load the pointer at that address into Tk
//        slot 1/3  U2: LEA  Tk [+ index, post-indexed] + od -> Tk                           (memory indirect only)
//   2. MAIN: the decoders see the opcode with that EA's mode field changed to 2, (An); DS replaces the base register of
//      that EA by Tk (decoder uop: base fields of the uop; sequence uops: every A(reg)-based address of the EA register).
// So every decoder and sequencer can handle the full format without knowing it exists: in MAIN they see a plain (An).
// MOVE: once the source is converted, a destination with extension words is converted as well (its words moved), and
// a destination computed in the pre-sequence includes the source's (An)+ / -(An) step (adj). Opcodes without a decoder
// hit go to MAIN unchanged (illegal instruction).
// Replays: pre uops carry U_FXP and STEP = slot; a pre-phase restart re-walks and issues from that slot. Main uops carry
// U_FXM; a main-phase restart re-walks and goes straight to MAIN (the temporaries are committed).
//
// Timing. The walk at first read the queue port straight into its decisions. That port is a wide select behind the
// queue head, so it was put into a register (wxr): each extension word now takes two clocks (one to read, one to use),
// which costs a few clocks on a rare instruction and takes the queue select out of these paths. For the same reason
// the four displacements of the pre uops are computed into registers during the walk, so the uop's adders are not in
// the issue path, and the busy and main outputs are registers of their own instead of a decode of the state.
// ---------------------------------------------------------------------------------------------------------------------
`include "p68_uop.vh"
`default_nettype none
module p68_fx (
  input  wire        clk,
  input  wire        rst,
  input  wire        flush,                  // pipeline flush: back to idle
  input  wire        start,          // the head instruction has a full-format EA and DS may take it
  input  wire [15:0] w0,             // its opcode (real, not the changed one)
  input  wire        ill,            // no decoder hit
  input  wire [3:0]  d_len,                  // DS's length of the instruction (as fetch cut it)
  input  wire [4:0]  cnt,                    // words in the fetch queue from its head
  input  wire [15:0] wx,                     // the queue word at head + xpos
  output wire [3:0]  xpos,                   // the queue position to read
  input  wire        adv,                    // DS advances this clock
  input  wire        rs_fxm,         // restart in the main part: no pre uops
  input  wire [4:0]  rs_skip,                // replay restart: slots below this one are not issued again
  input  wire        issue,                  // the instruction leaves the queue (end of MAIN)
  output wire        busy,           // walking or issuing pre uops (decoders off)
  output wire        main,                   // in the main part
  output wire        go,             // a pre uop leaves DS
  output reg  [`UW-1:0] u,                   // the pre uop of the current slot (zero outside ISSUE)
  output wire        first,                  // the current slot is the instruction's first uop
  output wire [4:0]  slot,                   // the current slot (STEP field of the pre uop)
  output wire        enter_main,             // the main part starts this clock
  output wire        emod,           // enter_main of a walk (not the illegal-opcode path): the opcode changes
  output wire        cs, cd,         // source / MOVE destination converted (valid in MAIN)
  output reg  [3:0]  len,                    // the instruction length found by the walk
  input  wire        fpo,            // an FPU #imm .B/.W instruction starts: issue with len = 3
  output reg         ilv,            // issue with len (MAIN, or the FPU override): one register for fetch's ilen
  output wire        sqclr,          // MOVE (d8,An,Xn) destination converted: the changed opcode is no sequence
  output reg         msq             // main && sqclr as a register
) /* synthesis syn_maxfan = 16 */;
`include "p68_dec.vh"
`include "p68_sysf.vh"
`include "p68_p7f.vh"

localparam S_IDLE = 2'd0, S_WALK = 2'd1, S_ISSUE = 2'd2, S_MAIN = 2'd3;   // main states
localparam W_EXT = 3'd0, W_VAL = 3'd1, W_END = 3'd2, W_DST = 3'd3;       // walk states: extension word, value words,
                                                                          // one EA done, MOVE destination
reg [1:0]  st;                               // main state
reg        wk_q, ok_q;                       // last clock: in WALK; the word at pos was in the queue
reg        il_q;                     // ill one clock late (DS's opcode holds while walking)
reg [2:0]  ws;                               // walk state
reg        cur;                      // EA being read: 0 source, 1 destination
reg [3:0]  pos;                              // word position in the instruction (0 = opcode)
reg [15:0] op;                               // the opcode, latched
reg [1:0]  tn;                       // words of the value being read
reg        tl, tod;                  // long value; value is od (else bd)
reg [15:0] thi;                              // high word of a long value
reg [1:0]  ip;                       // issue slot
reg [1:0]  odw_c;                    // od words of the EA being read
// per EA (s_ source, d_ destination): converted, PC base, base suppressed, pre-index, post-index, memory indirect,
// index long; base register, index register (D/A + number), word position of the extension word, scale, bd, od
reg        s_cv, s_pc, s_bs, s_xi, s_xp, s_ind, s_xl, d_cv, d_bs, d_xi, d_xp, d_ind, d_xl;
reg [2:0]  s_br, d_br;
reg [3:0]  s_xr, d_xr, s_ep;
reg [1:0]  s_xs, d_xs;
reg [31:0] s_bd, s_od, d_bd, d_od;

// busy and main as registers of their own (loaded with st), so DS's issue terms do not decode st (fanout 14); the
// preserve attribute keeps the synthesizer from merging them with the state decode
reg        busyc /* synthesis syn_preserve = 1 */, mainc /* synthesis syn_preserve = 1 */;
assign busy  = busyc;
assign main  = mainc;
assign xpos  = pos;
assign cs    = s_cv;
assign cd    = d_cv;
wire   mv    = fx_move(op);                                   // MOVE: has a destination EA too
wire [1:0] msz = fx_msz(op[15:12]);                           // MOVE size
// a converted MOVE (d8,An,Xn) destination becomes (An): the changed opcode needs no SQ_X sequence any more
assign sqclr = mv && d_cv && (op[8:6] == 3'd6);
wire   ok    = ({1'b0, pos} < cnt);                           // the word at pos is in the queue

// issue slots: 0 source U1, 1 source U2, 2 destination U1, 3 destination U2
wire [3:0] en = {d_cv && d_ind, d_cv, s_cv && s_ind, s_cv};
function [2:0] nxt;                  // {found, slot}: first enabled slot above s (s = 3'b111: from the start)
  input [3:0] e; input [2:0] s;
  begin
    nxt = 3'b000;
    // checked from the highest slot down, so the lowest enabled slot above s is written last and wins
    if (e[3] && (s == 3'd2 || s == 3'd1 || s == 3'd0 || s == 3'd7)) nxt = 3'b111;
    if (e[2] && (s == 3'd1 || s == 3'd0 || s == 3'd7)) nxt = 3'b110;
    if (e[1] && (s == 3'd0 || s == 3'd7)) nxt = 3'b101;
    if (e[0] && s == 3'd7) nxt = 3'b100;
  end
endfunction
wire [2:0] n_first = nxt(en, 3'd7);                           // the first slot to issue
wire [2:0] n_next  = nxt(en, {1'b0, ip});                     // the slot after the current one
assign first = (ip == n_first[1:0]);
assign slot  = {3'b000, ip};
// a pre uop leaves DS; after a replay the slots below rs_skip are skipped (their results are committed)
assign go    = (st == S_ISSUE) && adv && ({3'b000, ip} >= rs_skip);
// the main part starts: after the last pre uop, or at the end of the walk with nothing to issue (or a main-part restart)
assign emod = ((st == S_ISSUE) && adv && !n_next[2]) || (st == S_WALK && ws == W_END && !(cur == 1'b0 && mv) && (rs_fxm || en == 4'd0));
// the same, or the illegal-opcode path (no decoder hit: straight to MAIN, where the illegal instruction is raised)
assign enter_main = ((st == S_ISSUE) && adv && !n_next[2]) || (st == S_WALK && ws == W_END && !(cur == 1'b0 && mv) && (rs_fxm || en == 4'd0)) ||
                    (st == S_WALK && !wk_q && il_q);

// ---- decisions while walking ----------------------------------------------------------------------------------------
wire [2:0] dm  = op[8:6];                                     // MOVE destination mode
// destination value words: (d16,An) and abs.W one, abs.L two, others none
wire [1:0] dwn = (dm == 3'd5 || (dm == 3'd7 && op[11:9] == 3'd0)) ? 2'd1 : (dm == 3'd7 && op[11:9] == 3'd1) ? 2'd2 : 2'd0;
// The queue word at head + pos is registered (wxr); a WALK step uses it only when it was read at this
// pos in the last clock of the walk and was valid (wv), so each extension word costs two clocks and the queue port is
// not in front of these registers
reg  [15:0] wxr;
reg  [3:0]  pos_q;
always @(posedge clk) begin wxr <= wx; pos_q <= pos; wk_q <= (st == S_WALK); ok_q <= ok; il_q <= ill; end
wire        wv = wk_q && ok_q && (pos_q == pos);              // read at this pos while the word was in the queue
// fields of an extension word (valid when wxr is one)
wire       e_ful = wxr[8];                                    // full format
wire [2:0] e_ii  = wxr[2:0];                                  // I/IS
wire       e_ind = (e_ii[1:0] != 2'd0);                       // memory indirect
wire [1:0] e_bdw = (wxr[5:4] == 2'd3) ? 2'd2 : (wxr[5:4] == 2'd2) ? 2'd1 : 2'd0;   // bd words
wire [1:0] e_odw = !e_ind ? 2'd0 : (e_ii[1:0] == 2'd3) ? 2'd2 : (e_ii[1:0] == 2'd2) ? 2'd1 : 2'd0;   // od words
wire [31:0] tval = tl ? {thi, wxr} : {{16{wxr[15]}}, wxr};    // the value just completed (a long, or a sign-extended word)

// ---- the state machine ----------------------------------------------------------------------------------------------
always @(posedge clk) begin
  if (rst || flush) begin st <= S_IDLE; busyc <= 1'b0; mainc <= 1'b0; msq <= 1'b0; end
  else case (st)
    S_IDLE: begin                       // the walk registers load in every idle clock (read only after a start)
      op <= w0; s_cv <= 1'b0; d_cv <= 1'b0; ip <= 2'd0;
      // a MOVE whose source is not indexed: start at the destination, behind the source's words
      if (fx_move(w0) && !fx_cand(w0[5:3], w0[2:0])) begin
        pos <= 4'd1 + {2'b00, eax(w0[5:3], w0[2:0], fx_msz(w0[15:12]))}; cur <= 1'b1; ws <= W_DST;
      end else begin                    // otherwise the source's extension word, behind any words before it
        pos <= 4'd1 + {2'b00, fx_pre(w0)}; cur <= 1'b0; ws <= W_EXT;
      end
      if (start) begin len <= d_len; st <= S_WALK; busyc <= 1'b1; end               // no decoder hit goes to MAIN after one walk clock
    end
    // il_q (registered) is the start clock's ill: an illegal opcode goes to MAIN unchanged
    S_WALK: if (!wk_q && il_q) begin st <= S_MAIN; busyc <= 1'b0; mainc <= 1'b1; msq <= mv && d_cv && (op[8:6] == 3'd6); end
    else case (ws)
      W_EXT: if (ok && wv) begin                                             // first extension word of an indexed EA
        pos <= pos + 4'd1;
        odw_c <= e_ful ? e_odw : 2'd0;
        if (e_ful) begin                                                     // full format: this EA is converted
          // read bd first if there is one, else od if there is one, else this EA is done
          tod <= 1'b0; tl <= (e_bdw == 2'd2); tn <= e_bdw;
          ws <= (e_bdw != 2'd0) ? W_VAL : (e_odw != 2'd0) ? W_VAL : W_END;
          if (e_bdw == 2'd0 && e_odw != 2'd0) begin tod <= 1'b1; tl <= (e_odw == 2'd2); tn <= e_odw; end
          if (!cur) begin                                                    // the source EA
            s_cv <= 1'b1; s_pc <= (op[5:3] == 3'd7); s_bs <= wxr[7]; s_br <= op[2:0]; s_ep <= pos;
            s_xr <= wxr[15:12]; s_xl <= wxr[11]; s_xs <= wxr[10:9]; s_ind <= e_ind;
            // index: used before the indirection (pre) unless suppressed or post-indexed; after it (post) if I/IS bit 2
            s_xi <= !wxr[6] && !e_ii[2]; s_xp <= !wxr[6] && e_ii[2] && e_ind; s_bd <= 32'h0; s_od <= 32'h0;
          end else begin                                                     // the MOVE destination EA
            d_cv <= 1'b1; d_bs <= wxr[7]; d_br <= op[11:9];
            d_xr <= wxr[15:12]; d_xl <= wxr[11]; d_xs <= wxr[10:9]; d_ind <= e_ind;
            d_xi <= !wxr[6] && !e_ii[2]; d_xp <= !wxr[6] && e_ii[2] && e_ind; d_bd <= 32'h0; d_od <= 32'h0;
          end
        end else begin                                                       // brief format: one word, done
          ws <= W_END;
          if (cur && s_cv) begin                                         // brief destination behind a converted source
            // its word moves when the source's words disappear, so it is computed into T2 as well
            d_cv <= 1'b1; d_bs <= 1'b0; d_br <= op[11:9]; d_ind <= 1'b0; d_xi <= 1'b1; d_xp <= 1'b0;
            d_xr <= wxr[15:12]; d_xl <= wxr[11]; d_xs <= wxr[10:9]; d_bd <= {{24{wxr[7]}}, wxr[7:0]};
          end
        end
      end
      W_VAL: if (ok && wv) begin                                             // bd or od words
        pos <= pos + 4'd1;
        if (tn == 2'd2) begin thi <= wxr; tn <= 2'd1; end                    // high word of a long first
        else begin                                                           // value complete: store it
          if (!cur) begin if (tod) s_od <= tval; else s_bd <= tval; end
          else      begin if (tod) d_od <= tval; else d_bd <= tval; end
          if (!tod && odw_c != 2'd0) begin                                   // bd done, od follows
            tod <= 1'b1; tl <= (odw_c == 2'd2); tn <= odw_c;
          end else ws <= W_END;
        end
      end
      W_DST: begin                                                     // MOVE destination
        cur <= 1'b1;
        if (dm == 3'd6) ws <= W_EXT;                                     // indexed: read its extension word
        else if (dwn != 2'd0 && s_cv) begin                              // d16(An), abs.W, abs.L behind a converted source
          // converted too, since its words move: base An (or none for abs), the words read as bd
          d_cv <= 1'b1; d_bs <= (dm == 3'd7); d_br <= op[11:9]; d_ind <= 1'b0; d_xi <= 1'b0; d_xp <= 1'b0;
          tod <= 1'b0; tl <= (dwn == 2'd2); tn <= dwn; ws <= W_VAL; odw_c <= 2'd0;
        end else begin pos <= pos + {2'b00, dwn}; ws <= W_END; end       // nothing to convert: skip its words
      end
      default: begin                                                   // W_END: one EA done
        if (!cur && mv) ws <= W_DST;                                     // MOVE: the destination next
        else begin
          len <= pos;                                                    // the walk ends behind the last word
          if (rs_fxm || en == 4'd0) begin st <= S_MAIN; busyc <= 1'b0; mainc <= 1'b1; msq <= mv && d_cv && (op[8:6] == 3'd6); end
          else begin st <= S_ISSUE; ip <= n_first[1:0]; end
        end
      end
    endcase
    // one pre uop per DS advance; after the last one, MAIN
    S_ISSUE: if (adv) begin
      if (n_next[2]) ip <= n_next[1:0]; else begin st <= S_MAIN; busyc <= 1'b0; mainc <= 1'b1; msq <= mv && d_cv && (op[8:6] == 3'd6); end
    end
    default: if (issue) begin st <= S_IDLE; mainc <= 1'b0; msq <= 1'b0; end                                  // S_MAIN
  endcase
  if (fpo) len <= 4'd3;                                                // FPU #imm .B/.W (p68_fx is idle then)
end
// The issue length select as one register (main = MAIN state, or the FPU override), so fetch's ilen is
// a 3-input function per bit
always @(posedge clk) begin
  if (rst || flush) ilv <= 1'b0;
  else if (enter_main || fpo) ilv <= 1'b1;
  else if (issue) ilv <= 1'b0;
end

// ---- the pre uop of slot ip -----------------------------------------------------------------------------------------
reg        e_d, e_bsr, e_pcr, e_xir, e_xpr, e_indr, e_xlr;    // the selected EA's fields (e_d: destination)
reg [2:0]  e_brr;
reg [3:0]  e_xrr;
reg [1:0]  e_xsr;
reg [31:0] dsp;
wire [7:0] sstep = {4'h0, stepb(msz, op[2:0])};               // the source's (An)+ / -(An) step
wire       adj_on = mv && (op[5:3] == 3'd3 || op[5:3] == 3'd4);   // MOVE with an (An)+ / -(An) source
// The four displacements (source / destination, U1 / U2) as registers. Their inputs last change while walking,
// at least one clock before the first ISSUE clock (the W_END clock lies between), so the uop's adders are off the issue
wire [31:0] adjw = (op[5:3] == 3'd3) ? {24'h0, sstep} : (32'h0 - {24'h0, sstep});
reg  [31:0] s_u1d, d_u1d, s_u2d, d_u2d;
always @(posedge clk) begin
  // source U1: bd, plus the extension word's offset 2 * s_ep for a PC base (the PC base is the extension word's address)
  s_u1d <= s_bd + ((s_pc && !s_bs) ? {27'h0, s_ep, 1'b0} : 32'h0);
  // destination U1: bd, plus the source's step when the destination's base or pre-index is the source's An
  d_u1d <= d_bd + ((adj_on && !d_bs && d_br == op[2:0]) ? adjw : 32'h0) + ((adj_on && d_xi && d_xr == {1'b1, op[2:0]}) ? (adjw << d_xs) : 32'h0);
  s_u2d <= s_od;
  // destination U2: od, plus the scaled step when the post-index is the source's An
  d_u2d <= d_od + ((adj_on && d_xp && d_xr == {1'b1, op[2:0]}) ? (adjw << d_xs) : 32'h0);
end
always @* begin
  e_d = ip[1];                                                         // slots 2 and 3 are the destination's
  e_bsr = e_d ? d_bs : s_bs;  e_pcr = e_d ? 1'b0 : s_pc;  e_xir = e_d ? d_xi : s_xi;  e_xpr = e_d ? d_xp : s_xp;
  e_indr = e_d ? d_ind : s_ind;  e_xlr = e_d ? d_xl : s_xl;  e_brr = e_d ? d_br : s_br;  e_xrr = e_d ? d_xr : s_xr;
  e_xsr = e_d ? d_xs : s_xs;  u = {`UW{1'b0}};
  u[`U_FXP] = 1'b1;                                                    // marks a pre uop (for replays)
  if (!ip[0]) begin                                                    // U1
    dsp = e_d ? d_u1d : s_u1d;
    // base: none (suppressed), the PC, or An; index if pre-indexed; displacement bd
    u[`F_LA_BSEL] = e_bsr ? `BA_ZERO : e_pcr ? `BA_PC : `BA_REG; u[`F_LA_BREG] = {2'b01, e_brr};
    u[`F_LA_XEN] = e_xir; u[`F_LA_XREG] = {1'b0, e_xrr}; u[`F_LA_XL] = e_xlr; u[`F_LA_XSC] = e_xsr; u[`F_LA_DISP] = dsp;
    if (e_indr) begin                                                  // memory indirect: load the pointer into Tk
      u[`F_UNIT] = `UN_ALU; u[`F_OP] = `OP_MOV; u[`F_SZ] = `SZ_L; u[`F_LSZ] = `SZ_L; u[`F_ASEL] = `AS_MEM;
      u[`F_LA_EN] = 1'b1; u[`F_WEN] = 1'b1; u[`F_WREG] = e_d ? `R_T2 : `R_T1;
    end else begin u[`F_LEA] = 1'b1; u[`F_SA_BREG] = e_d ? `R_T2 : `R_T1; end   // no indirection: Tk = the address
  end else begin                                                       // U2
    dsp = e_d ? d_u2d : s_u2d;
    // Tk = Tk + post-index + od (a LEA)
    u[`F_LA_BSEL] = `BA_REG; u[`F_LA_BREG] = e_d ? `R_T2 : `R_T1;
    u[`F_LA_XEN] = e_xpr; u[`F_LA_XREG] = {1'b0, e_xrr}; u[`F_LA_XL] = e_xlr; u[`F_LA_XSC] = e_xsr; u[`F_LA_DISP] = dsp;
    u[`F_LEA] = 1'b1; u[`F_SA_BREG] = e_d ? `R_T2 : `R_T1;
  end
  if (st != S_ISSUE) u = {`UW{1'b0}};                                  // zero unless issuing (DS ORs it)
end

endmodule
`default_nettype wire
