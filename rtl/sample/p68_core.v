// pipe68k pipeline sample (feasibility study). Written from the M68000 Family Programmer's Reference
// Manual only. Purpose: a real Fmax and LUT count on the GW5AST-138C for the planned stage-1 pipeline. It runs a
// subset of the 68020 integer instructions and is not verified against a reference CPU. Not for use.
//
// Stages (68060-like in-order integer pipeline):
//   IF  I-cache read (4 KB, 8-byte lines, 64-bit wide)      -> 16-word instruction queue
//   DS  instruction length decode from the queue, issue
//   AG  effective address (base + index*scale + disp), An update for (An)+ / -(An), LEA, BRA redirect
//   OC  D-cache read (4 KB, 4-byte lines), store forwarding, register operand read
//   EX  adder, logic, barrel shifter, flags, Bcc resolve
//   WB  register write, store to D-cache and write-through bus
// Sample instruction set: MOVEQ, MOVE/MOVEA (dst Dn, An, (An), (An)+, -(An), d16(An)), ADD/SUB/AND/OR/EOR/CMP in
// both directions (memory read-modify-write), ADDA/SUBA/CMPA, ADDQ/SUBQ, Bcc/BRA (8/16/32-bit), LEA, ASx/LSx/ROx.
// Source EA: all 68000 modes plus the brief index format with scale. Not covered: the full extension format
// (memory indirect), misaligned long accesses, MUL/DIV, bit fields, BSR/JSR/RTS, MOVEM, exceptions, supervisor.
`default_nettype none
module p68_core (
  input  wire        clk,
  input  wire        rst,
  output reg         bus_req,
  output reg  [31:2] bus_addr,
  input  wire [31:0] bus_rdata,
  input  wire        bus_ack,
  output reg         bw_en,
  output reg  [31:2] bw_addr,
  output reg  [31:0] bw_data,
  output reg  [3:0]  bw_be,
  output wire [31:0] obs_a,
  output wire [31:0] obs_b,
  output wire [4:0]  obs_ccr,
  // retire port (simulation, lockstep against a reference CPU): one record per instruction leaving WB
  output reg         ret_v,
  output reg  [31:0] ret_pc,
  output wire        ret_wen,      // register write from WB
  output wire [3:0]  ret_wreg,
  output wire [31:0] ret_wval,
  output reg         ret_ws_en,    // An update from AG, source EA ((An)+, -(An))
  output reg  [2:0]  ret_ws_r,
  output reg  [31:0] ret_ws_v,
  output reg         ret_wd_en,    // An update from AG, destination EA or LEA
  output reg  [2:0]  ret_wd_r,
  output reg  [31:0] ret_wd_v,
  output wire        ret_st,       // store
  output wire [31:0] ret_saddr,
  output wire [31:0] ret_sdata,
  output wire [3:0]  ret_sbe,
  output wire [4:0]  ret_ccr       // CCR after the instruction
) /* synthesis syn_maxfan = 16 */;

localparam C_NOP=4'd0, C_MOVEQ=4'd1, C_MOVE=4'd2, C_ALU=4'd3, C_ADDA=4'd4, C_ADDQ=4'd5, C_BCC=4'd6, C_LEA=4'd7,
           C_SHF=4'd8;
localparam O_ADD=3'd0, O_SUB=3'd1, O_AND=3'd2, O_OR=3'd3, O_CMP=3'd4, O_MOV=3'd5, O_SHF=3'd6, O_EOR=3'd7;
localparam SB=2'd0, SW=2'd1, SL=2'd2;
localparam A_REG=2'd0, A_MEM=2'd1, A_IMM=2'd2;

integer i;

// =====================================================================================================
// Register file D0-D7 = 0..7, A0-A7 = 8..15 (packed, flip-flops: 3 write ports, 5 read ports)
// =====================================================================================================
reg  [511:0] rf;
reg          wb_wen;  reg [3:0] wb_wreg; reg [31:0] wb_val;
reg          wb_st;   reg [31:0] wb_saddr; reg [1:0] wb_sz; reg wb_hit;

function [31:0] rdbf;  // read with bypass from WB (all inputs as arguments, so simulators re-evaluate it)
  input [3:0] r; input [511:0] f; input we; input [3:0] wr; input [31:0] wv;
  begin rdbf = (we && wr == r) ? wv : f[r*32 +: 32]; end
endfunction
`define RDB(x) rdbf(x, rf, wb_wen, wb_wreg, wb_val)

// =====================================================================================================
// Fetch: I-cache, instruction queue
// =====================================================================================================
reg  [255:0] iq;           // 16 words, circular
reg  [3:0]   head;
reg  [4:0]   cnt;          // valid words 0..16
reg  [31:0]  fpc;          // next fetch address (byte, even)
reg          if2_v;
reg  [31:0]  if2_pc;
reg  [63:0]  ic_data [0:511];
reg  [20:0]  ic_tag  [0:511];
reg  [63:0]  ic_q;
reg  [20:0]  ict_q;
reg          ic_we;  reg [8:0] ic_wa; reg [63:0] ic_wd; reg [20:0] ic_wt;
reg  [1:0]   dm_st;
reg          im_busy, im_pend, im_half; reg [31:3] im_addr; reg [31:0] im_w0;

always @(posedge clk) begin   // write, else read (PA2122)
  if (ic_we) begin ic_data[ic_wa] <= ic_wd; ic_tag[ic_wa] <= ic_wt; end
  else begin ic_q <= ic_data[fpc[11:3]]; ict_q <= ic_tag[fpc[11:3]]; end
end

wire        flush;              // from EX or AG (below)
wire [31:0] flush_tgt;
wire        issue;
wire [2:0]  d_len;
reg  [47:0]  iql;          // predecoded instruction length per queue word (as if it were an opcode)

wire        ic_hit  = ict_q[20] && (ict_q[19:0] == if2_pc[31:12]);
wire        im_now  = if2_v && !ic_hit && !flush;
wire        q_wr    = if2_v && ic_hit && !flush;
wire [1:0]  q_off   = if2_pc[2:1];
wire [2:0]  q_n     = 3'd4 - {1'b0, q_off};
wire [3:0]  tail    = head + cnt[3:0];
wire        if_go   = !im_busy && !ic_we && (dm_st == 2'd0) && ({1'b0, cnt} + (if2_v ? 6'd4 : 6'd0) <= 6'd12);
wire [15:0] blk0 = ic_q[63:48], blk1 = ic_q[47:32], blk2 = ic_q[31:16], blk3 = ic_q[15:0];

always @(posedge clk) begin
  if (rst) begin
    fpc <= 32'h0; if2_v <= 1'b0; head <= 4'd0; cnt <= 5'd0;
  end else begin
    if (flush)       fpc <= flush_tgt;
    else if (im_now) fpc <= if2_pc;
    else if (if_go)  fpc <= {fpc[31:3] + 29'd1, 3'b000};
    if2_v  <= if_go && !flush && !im_now;
    if2_pc <= fpc;
    if (flush) begin head <= 4'd0; cnt <= 5'd0; end
    else begin
      head <= head + (issue ? {1'b0, d_len} : 4'd0);
      cnt  <= cnt - (issue ? {2'b0, d_len} : 5'd0) + (q_wr ? {2'b0, q_n} : 5'd0);
    end
  end
end

function [1:0] eax;  // extension words of an effective address
  input [2:0] m; input [2:0] r; input [1:0] s;
  begin
    if (m == 3'd5 || m == 3'd6) eax = 2'd1;
    else if (m == 3'd7) eax = (r == 3'd1) ? 2'd2 : (r == 3'd4) ? ((s == SL) ? 2'd2 : 2'd1) : (r <= 3'd3) ? 2'd1 : 2'd0;
    else eax = 2'd0;
  end
endfunction
function [2:0] lenf;  // instruction length in words from the opcode word alone (same rules as the DS decode)
  input [15:0] w;
  reg [3:0] g; reg [1:0] sz, xs_, xd_; reg mv, ea;
  begin
    g  = w[15:12];
    mv = (g == 4'h1 || g == 4'h2 || g == 4'h3);
    sz = mv ? ((g == 4'h1) ? SB : (g == 4'h3) ? SW : SL) : (w[7:6] == 2'b11) ? (w[8] ? SL : SW) : w[7:6];
    xs_ = eax(w[5:3], w[2:0], sz);
    xd_ = eax(w[8:6], w[11:9], SW);
    ea = (g == 4'h5 && w[7:6] != 2'b11) || (g == 4'h4 && w[8:6] == 3'b111) ||
         ((g == 4'h8 || g == 4'h9 || g == 4'hB || g == 4'hC || g == 4'hD) &&
          ((w[7:6] == 2'b11) ? (g == 4'h9 || g == 4'hB || g == 4'hD)
                             : !(w[8] && (w[5:3] == 3'd1 || (w[5:3] == 3'd0 && g != 4'hB)))));
    if (mv)      lenf = 3'd1 + {1'b0, xs_} + {1'b0, xd_};
    else if (ea) lenf = 3'd1 + {1'b0, xs_};
    else if (g == 4'h6 && w[11:8] != 4'h1) lenf = (w[7:0] == 8'h00) ? 3'd2 : (w[7:0] == 8'hFF) ? 3'd3 : 3'd1;
    else         lenf = 3'd1;
  end
endfunction
wire [2:0] pl0 = lenf(blk0), pl1 = lenf(blk1), pl2 = lenf(blk2), pl3 = lenf(blk3);

genvar e;
generate for (e = 0; e < 16; e = e + 1) begin : qe
  wire [3:0] rel = e[3:0] - tail;
  wire [1:0] k   = rel[1:0] + q_off;
  wire       we  = q_wr && ({1'b0, rel} < {2'b0, q_n});
  always @(posedge clk)
    if (we)
      case (k)
        2'd0: begin iq[e*16 +: 16] <= blk0; iql[e*3 +: 3] <= pl0; end
        2'd1: begin iq[e*16 +: 16] <= blk1; iql[e*3 +: 3] <= pl1; end
        2'd2: begin iq[e*16 +: 16] <= blk2; iql[e*3 +: 3] <= pl2; end
        default: begin iq[e*16 +: 16] <= blk3; iql[e*3 +: 3] <= pl3; end
      endcase
end endgenerate

// =====================================================================================================
// DS: decode and length
// =====================================================================================================
wire [3:0]  h1 = head + 4'd1, h2 = head + 4'd2, h3 = head + 4'd3;
wire [15:0] w0 = iq[head*16 +: 16];
wire [15:0] w1 = iq[h1*16 +: 16];
wire [15:0] w2 = iq[h2*16 +: 16];
wire [15:0] w3 = iq[h3*16 +: 16];
wire [3:0]  g    = w0[15:12];
wire [2:0]  f_r9 = w0[11:9], f_m6 = w0[8:6], f_m3 = w0[5:3], f_r0 = w0[2:0];

reg [3:0] d_cls; reg [2:0] d_op; reg [1:0] d_sz; reg d_dir;
always @* begin
  d_cls = C_NOP; d_op = O_MOV; d_sz = SL; d_dir = 1'b0;
  case (g)
    4'h1, 4'h2, 4'h3: begin d_cls = C_MOVE; d_sz = (g == 4'h1) ? SB : (g == 4'h3) ? SW : SL; end
    4'h7: if (!w0[8]) d_cls = C_MOVEQ;
    4'h5: if (w0[7:6] != 2'b11) begin d_cls = C_ADDQ; d_op = w0[8] ? O_SUB : O_ADD; d_sz = w0[7:6]; end
    4'h6: if (w0[11:8] != 4'h1) d_cls = C_BCC;
    4'h4: if (w0[8:6] == 3'b111) d_cls = C_LEA;
    4'h8, 4'h9, 4'hB, 4'hC, 4'hD: begin
      d_op = (g == 4'h8) ? O_OR : (g == 4'h9) ? O_SUB : (g == 4'hC) ? O_AND : (g == 4'hD) ? O_ADD :
             (w0[8] ? O_EOR : O_CMP);
      if (w0[7:6] == 2'b11) begin
        if (g == 4'h9 || g == 4'hB || g == 4'hD) begin
          d_cls = C_ADDA; d_sz = w0[8] ? SL : SW; if (g == 4'hB) d_op = O_CMP;
        end
      end else begin
        d_sz = w0[7:6]; d_dir = w0[8];
        if (!(w0[8] && (f_m3 == 3'd1 || (f_m3 == 3'd0 && g != 4'hB)))) d_cls = C_ALU;
      end
    end
    4'hE: if (w0[7:6] != 2'b11 && w0[4:3] != 2'b10) begin d_cls = C_SHF; d_op = O_SHF; d_sz = w0[7:6]; end
    default: ;
  endcase
end

wire [1:0] xs = eax(f_m3, f_r0, d_sz);
assign d_len = iql[head*3 +: 3];   // predecoded when the word entered the queue

wire [31:0] sx16 = {{16{w1[15]}}, w1};
wire [31:0] sx8  = {{24{w1[7]}}, w1[7:0]};
wire [8:0]  d8p2 = {w1[7], w1[7:0]} + 9'd2;      // d8(PC,Xn): disp + 2 (PC of the extension word)
reg  [31:0] d_sval;
always @* begin
  if (d_cls == C_MOVEQ) d_sval = {{24{w0[7]}}, w0[7:0]};
  else if (d_cls == C_BCC)
    d_sval = (w0[7:0] == 8'h00) ? sx16 : (w0[7:0] == 8'hFF) ? {w1, w2} : {{24{w0[7]}}, w0[7:0]};
  else case (f_m3)
    3'd5: d_sval = sx16;
    3'd6: d_sval = sx8;
    3'd7: case (f_r0)
            3'd0, 3'd2: d_sval = sx16;
            3'd1:       d_sval = {w1, w2};
            3'd3:       d_sval = {{23{d8p2[8]}}, d8p2};
            default:    d_sval = (d_sz == SL) ? {w1, w2} : {16'h0, w1};
          endcase
    default: d_sval = 32'h0;
  endcase
end
wire [15:0] d_dd = (xs == 2'd0) ? w1 : (xs == 2'd1) ? w2 : w3;

// instruction address: one add behind the issue (no length decode in front of the adder)
reg  [31:0] pcq; reg [2:0] lenq;
wire [31:0] pc_cur = pcq + {28'h0, lenq, 1'b0};
wire        ds_ok  = (cnt != 5'd0) && ({2'b0, d_len} <= cnt);

// =====================================================================================================
// AG
// =====================================================================================================
reg        ag_v; reg [3:0] ag_cls; reg [2:0] ag_op; reg [1:0] ag_sz; reg ag_dir;
reg [2:0]  ag_r9, ag_m6, ag_m3, ag_r0; reg [31:0] ag_sval; reg [15:0] ag_dd; reg [7:0] ag_brief;
reg [3:0]  ag_cc; reg [3:0] ag_sh; reg [31:0] ag_pc;

reg        oc_v; reg [3:0] oc_cls; reg [2:0] oc_op; reg [1:0] oc_sz, oc_lsz; reg [1:0] oc_asel; reg [3:0] oc_ra;
reg        oc_bsel; reg [3:0] oc_rb; reg [31:0] oc_imm; reg oc_ld, oc_st; reg [31:0] oc_addr, oc_saddr;
reg        oc_wen; reg [3:0] oc_wreg; reg oc_fl, oc_sexta; reg [3:0] oc_cc; reg [31:0] oc_tgt, oc_pc;
reg        oc_ws_en, oc_wd_en; reg [2:0] oc_ws_r, oc_wd_r; reg [31:0] oc_ws_v, oc_wd_v;

reg        ex_v; reg [3:0] ex_cls; reg [2:0] ex_op; reg [1:0] ex_sz; reg [31:0] ex_a, ex_b;
reg        ex_afwd, ex_bfwd; reg ex_sexta, ex_wen; reg [3:0] ex_wreg;
reg        ex_st; reg [31:0] ex_saddr; reg ex_hit, ex_fl; reg [3:0] ex_cc; reg [31:0] ex_tgt, ex_pc;
reg        ex_rep, ex_miss; reg [31:0] ex_laddr;
reg        ex_ws_en, ex_wd_en; reg [2:0] ex_ws_r, ex_wd_r; reg [31:0] ex_ws_v, ex_wd_v;
wire       ex_commit = ex_v && !ex_rep;   // Bcc carries no AG writes

wire       stall_oc;
wire       redirect_ex;
wire [31:0] ex_target;

wire [3:0]  bidx = ag_brief[7:4];
wire        bwl  = ag_brief[3];
wire [1:0]  bsc  = ag_brief[2:1];
wire        ea_cls = (ag_cls == C_MOVE) || (ag_cls == C_ALU) || (ag_cls == C_ADDA) || (ag_cls == C_ADDQ) ||
                     (ag_cls == C_LEA);
wire [3:0]  ia_s = {1'b1, ag_r0};
wire [3:0]  ia_d = {1'b1, ag_r9};
// WB forwarding for AG reads: the select is decided one clock early (the instruction in EX now is in WB next clock)
reg         ag_fs, ag_fd, ag_fx;
wire [31:0] an_s = ag_fs ? wb_val : rf[ia_s*32 +: 32];
wire [31:0] an_d = ag_fd ? wb_val : rf[ia_d*32 +: 32];
wire [31:0] xr   = ag_fx ? wb_val : rf[bidx*32 +: 32];
wire [31:0] xw   = bwl ? xr : {{16{xr[15]}}, xr[15:0]};
reg  [31:0] xi;
always @* case (bsc) 2'd0: xi = xw; 2'd1: xi = {xw[30:0], 1'b0}; 2'd2: xi = {xw[29:0], 2'b0};
                     default: xi = {xw[28:0], 3'b0}; endcase
wire [31:0] szb_s = (ag_sz == SB) ? ((ag_r0 == 3'd7) ? 32'd2 : 32'd1) : (ag_sz == SW) ? 32'd2 : 32'd4;
wire [31:0] szb_d = (ag_sz == SB) ? ((ag_r9 == 3'd7) ? 32'd2 : 32'd1) : (ag_sz == SW) ? 32'd2 : 32'd4;
wire        s_an    = ea_cls && ag_m3 >= 3'd2 && ag_m3 <= 3'd6;
wire        s_pcrel = ea_cls && ag_m3 == 3'd7 && (ag_r0 == 3'd2 || ag_r0 == 3'd3);
wire        s_idx   = ea_cls && (ag_m3 == 3'd6 || (ag_m3 == 3'd7 && ag_r0 == 3'd3));
wire        is_bcc  = ag_cls == C_BCC;
wire [31:0] e_base = s_an ? an_s : (s_pcrel || is_bcc) ? ag_pc : 32'h0;
wire [31:0] e_idx  = s_idx ? xi : ((ea_cls && ag_m3 == 3'd7 && ag_r0 == 3'd2) || is_bcc) ? 32'd2 : 32'h0;
wire [31:0] e_disp = ((ea_cls && ag_m3 >= 3'd5) || is_bcc) ? ag_sval : 32'h0;
wire [31:0] cs_s = e_base ^ e_idx ^ e_disp;                                   // 3:2 compressor, one carry chain
wire [31:0] cs_c = (e_base & e_idx) | (e_base & e_disp) | (e_idx & e_disp);
wire [31:0] ea_s   = cs_s + {cs_c[30:0], 1'b0};
wire [31:0] upd_s  = an_s + ((ag_m3 == 3'd4) ? (~szb_s + 32'd1) : szb_s);
wire [31:0] addr_s = (ag_m3 == 3'd4) ? upd_s : ea_s;
// destination EA (MOVE). If the source EA was (An)+ / -(An) on the same register, the destination sees the updated
// register: the source step is a third input of the same 3:2 compressor and carry chain.
wire        d_same = ag_cls == C_MOVE && (ag_m3 == 3'd3 || ag_m3 == 3'd4) && ag_r0 == ag_r9;
wire [31:0] off_s  = !d_same ? 32'h0 : (ag_m3 == 3'd3) ? szb_s : (~szb_s + 32'd1);
wire [31:0] t_disp = (ag_m6 == 3'd5) ? {{16{ag_dd[15]}}, ag_dd} : (ag_m6 == 3'd4) ? (~szb_d + 32'd1) : 32'h0;
wire [31:0] dcs_s  = an_d ^ off_s ^ t_disp;
wire [31:0] dcs_c  = (an_d & off_s) | (an_d & t_disp) | (off_s & t_disp);
wire [31:0] addr_d = dcs_s + {dcs_c[30:0], 1'b0};
wire [31:0] pcs_s  = an_d ^ off_s ^ szb_d;
wire [31:0] pcs_c  = (an_d & off_s) | (an_d & szb_d) | (off_s & szb_d);
wire [31:0] upd_d  = (ag_m6 == 3'd4) ? addr_d : (pcs_s + {pcs_c[30:0], 1'b0});

wire        e_reg  = ag_m3 <= 3'd1;
wire        e_imm  = ag_m3 == 3'd7 && ag_r0 == 3'd4;
wire [1:0]  e_asel = e_reg ? A_REG : e_imm ? A_IMM : A_MEM;
wire [3:0]  e_ra   = {ag_m3[0], ag_r0};
wire [31:0] q8     = (ag_r9 == 3'd0) ? 32'd8 : {29'h0, ag_r9};

reg [3:0] n_cls; reg [2:0] n_op; reg [1:0] n_sz, n_asel; reg [3:0] n_ra, n_rb, n_wreg, n_cc; reg n_bsel;
reg [31:0] n_imm, n_saddr; reg n_ld, n_st, n_wen, n_fl, n_sexta, n_bra;
reg ws_en, wd_en; reg [31:0] wd_val;
always @* begin
  n_cls = ag_cls; n_op = ag_op; n_sz = ag_sz; n_asel = e_asel; n_ra = e_ra; n_bsel = 1'b0; n_rb = {1'b0, ag_r9};
  n_imm = ag_sval; n_ld = 1'b0; n_st = 1'b0; n_saddr = addr_s; n_wen = 1'b0; n_wreg = {1'b0, ag_r9};
  n_fl = 1'b1; n_sexta = 1'b0; n_cc = ag_cc; n_bra = 1'b0;
  ws_en = ea_cls && ag_cls != C_LEA && (ag_m3 == 3'd3 || ag_m3 == 3'd4);
  wd_en = 1'b0; wd_val = upd_d;
  case (ag_cls)
    C_MOVEQ: begin n_asel = A_IMM; n_wen = 1'b1; n_sz = SL; n_op = O_MOV; end
    C_MOVE: begin
      n_op = O_MOV; n_ld = (e_asel == A_MEM);
      case (ag_m6)
        3'd0: begin n_wen = 1'b1; end
        3'd1: begin n_wen = 1'b1; n_wreg = {1'b1, ag_r9}; n_fl = 1'b0; n_sexta = (ag_sz == SW); n_sz = SL; end
        3'd2, 3'd3, 3'd4, 3'd5: begin n_st = 1'b1; n_saddr = addr_d; wd_en = (ag_m6 == 3'd3 || ag_m6 == 3'd4); end
        default: begin n_cls = C_NOP; ws_en = 1'b0; end
      endcase
    end
    C_ALU: if (!ag_dir) begin
      n_ld = (e_asel == A_MEM); n_wen = (ag_op != O_CMP);
    end else begin
      n_asel = A_REG; n_ra = {1'b0, ag_r9};
      if (ag_m3 == 3'd0) begin n_rb = {1'b0, ag_r0}; n_wen = 1'b1; n_wreg = {1'b0, ag_r0}; end
      else begin n_bsel = 1'b1; n_ld = 1'b1; n_st = 1'b1; end
    end
    C_ADDA: begin
      n_ld = (e_asel == A_MEM); n_sexta = (ag_sz == SW); n_rb = {1'b1, ag_r9}; n_wreg = {1'b1, ag_r9};
      n_wen = (ag_op != O_CMP); n_fl = (ag_op == O_CMP); n_sz = SL;
    end
    C_ADDQ: begin
      n_asel = A_IMM; n_imm = q8;
      if (ag_m3 <= 3'd1) begin
        n_rb = {ag_m3[0], ag_r0}; n_wen = 1'b1; n_wreg = {ag_m3[0], ag_r0};
        if (ag_m3[0]) begin n_fl = 1'b0; n_sz = SL; end
      end else begin n_bsel = 1'b1; n_ld = 1'b1; n_st = 1'b1; end
    end
    C_BCC: begin n_fl = 1'b0; if (ag_cc == 4'h0) begin n_bra = 1'b1; n_cls = C_NOP; end end
    C_LEA: begin n_cls = C_NOP; n_fl = 1'b0; wd_en = 1'b1; wd_val = ea_s; end
    C_SHF: begin
      n_asel = ag_sh[2] ? A_REG : A_IMM; n_ra = {1'b0, ag_r9}; n_imm = q8;
      n_rb = {1'b0, ag_r0}; n_wen = 1'b1; n_wreg = {1'b0, ag_r0}; n_cc = {ag_sh[3], ag_sh[1:0] == 2'b11, ag_sh[1:0] == 2'b00, 1'b0};
    end
    default: begin n_fl = 1'b0; ws_en = 1'b0; end
  endcase
  // a source register that this instruction's AG also updates ((An)+ / -(An)) is read before the update
  if (n_asel == A_REG && n_ra[3]) begin
    if (wd_en && n_ra[2:0] == ag_r9)      begin n_asel = A_IMM; n_imm = an_d; end
    else if (ws_en && n_ra[2:0] == ag_r0) begin n_asel = A_IMM; n_imm = an_s; end
  end
end

// change/use interlock: AG needs a register an older instruction in OC or EX will still write (WB is bypassed)
function pendf; input [3:0] r; input ov, ow; input [3:0] owr; input ev, ew; input [3:0] ewr;
  begin pendf = (ov && ow && owr == r) || (ev && ew && ewr == r); end
endfunction
`define PEND(x) pendf(x, oc_v, oc_wen, oc_wreg, ex_v, ex_wen, ex_wreg)
wire hazard = (s_an && `PEND(ia_s)) || (s_idx && `PEND(bidx)) ||
              (ag_cls == C_MOVE && ag_m6 >= 3'd2 && ag_m6 <= 3'd5 && `PEND(ia_d)) ||
              (ag_cls == C_LEA && `PEND(ia_d));
wire stall_ag = ag_v && (stall_oc || hazard);
wire ag_go    = ag_v && !stall_ag && !redirect_ex;
wire bra_ag   = ag_go && n_bra;

assign flush     = redirect_ex || bra_ag;
assign flush_tgt = redirect_ex ? ex_target : ea_s;
assign issue     = ds_ok && !stall_ag && !flush;

always @(posedge clk) begin
  if (rst) begin
    ag_v <= 1'b0; pcq <= 32'h0; lenq <= 3'd0;
  end else begin
    if (flush) begin pcq <= flush_tgt; lenq <= 3'd0; end
    else if (issue) begin pcq <= pc_cur; lenq <= d_len; end
    if (flush) ag_v <= 1'b0;
    else if (issue) ag_v <= 1'b1;
    else if (!stall_ag) ag_v <= 1'b0;
  end
  begin : agfwd
    reg [3:0] ns, nd, nx; reg ew;
    ns = issue ? {1'b1, f_r0} : ia_s;
    nd = issue ? {1'b1, f_r9} : ia_d;
    nx = issue ? w1[15:12]    : bidx;
    ew = ex_v && ex_wen && !ex_rep;
    ag_fs <= ew && ex_wreg == ns; ag_fd <= ew && ex_wreg == nd; ag_fx <= ew && ex_wreg == nx;
  end
  if (issue) begin
    ag_cls <= d_cls; ag_op <= d_op; ag_sz <= d_sz; ag_dir <= d_dir;
    ag_r9 <= f_r9; ag_m6 <= f_m6; ag_m3 <= f_m3; ag_r0 <= f_r0; ag_sval <= d_sval; ag_dd <= d_dd;
    ag_brief <= {w1[15:12], w1[11], w1[10:9], w1[8]}; ag_cc <= w0[11:8]; ag_sh <= {w0[8], w0[5], w0[4:3]};
    ag_pc <= pc_cur;
  end
end

// register file writes: AG (An updates, LEA) before WB.
// AG writes An early (speculative). ash holds the committed A0-A7: AG writes enter it when their instruction leaves
// EX without redirect, WB writes always. A taken Bcc or a replay copies ash back.
reg  [255:0] ash;
reg  [255:0] rf_na;
always @* begin
  for (i = 0; i < 8; i = i + 1) begin
    if (redirect_ex)                         rf_na[i*32 +: 32] = (wb_wen && wb_wreg == 8 + i) ? wb_val : ash[i*32 +: 32];
    else if (ag_go && wd_en && ag_r9 == i)   rf_na[i*32 +: 32] = wd_val;
    else if (ag_go && ws_en && ag_r0 == i)   rf_na[i*32 +: 32] = upd_s;
    else if (wb_wen && wb_wreg == 8 + i)     rf_na[i*32 +: 32] = wb_val;
    else                                     rf_na[i*32 +: 32] = rf[(8 + i)*32 +: 32];
  end
end
always @(posedge clk) begin
  rf[511:256] <= rf_na;
  for (i = 0; i < 8; i = i + 1)
    if (wb_wen && wb_wreg == i) rf[i*32 +: 32] <= wb_val;
  for (i = 0; i < 8; i = i + 1)
    if (ex_commit && ex_wd_en && ex_wd_r == i)      ash[i*32 +: 32] <= ex_wd_v;
    else if (ex_commit && ex_ws_en && ex_ws_r == i) ash[i*32 +: 32] <= ex_ws_v;
    else if (wb_wen && wb_wreg == 8 + i)            ash[i*32 +: 32] <= wb_val;
end

// =====================================================================================================
// OC: D-cache (4 byte lanes + tag for loads + tag copy for stores), forwarding, register read
// =====================================================================================================
reg  [7:0]  dc0 [0:1023]; reg [7:0] dc1 [0:1023]; reg [7:0] dc2 [0:1023]; reg [7:0] dc3 [0:1023];
reg  [20:0] dct [0:1023]; reg [20:0] dct2 [0:1023];
reg  [31:0] dc_q; reg [20:0] dct_q, dct2_q;
reg  [31:0] dm_data; reg [31:2] dm_addr;
reg         ls_v; reg [31:2] ls_addr; reg [31:0] ls_data; reg [3:0] ls_be;

wire [9:0]  dc_ra  = stall_oc ? oc_addr[11:2]  : addr_s[11:2];
wire [9:0]  dc_ra2 = stall_oc ? oc_saddr[11:2] : n_saddr[11:2];
wire        refill_we = (dm_st == 2'd2) && !wb_st;
reg  [31:0] sd; reg [3:0] sbe;
always @* begin
  case (wb_sz)
    SB: begin sd = {4{wb_val[7:0]}};
          case (wb_saddr[1:0]) 2'd0: sbe = 4'b1000; 2'd1: sbe = 4'b0100; 2'd2: sbe = 4'b0010; default: sbe = 4'b0001; endcase end
    SW: begin sd = {2{wb_val[15:0]}}; sbe = wb_saddr[1] ? 4'b0011 : 4'b1100; end
    default: begin sd = wb_val; sbe = 4'b1111; end
  endcase
end
wire        st_we  = wb_st && wb_hit;
wire [9:0]  dc_wa  = refill_we ? dm_addr[11:2] : wb_saddr[11:2];
wire [31:0] dc_wd  = refill_we ? dm_data : sd;
wire [3:0]  dc_wbe = refill_we ? 4'b1111 : (st_we ? sbe : 4'b0000);

always @(posedge clk) begin
  if (dc_wbe[3]) dc0[dc_wa] <= dc_wd[31:24];
  if (dc_wbe[2]) dc1[dc_wa] <= dc_wd[23:16];
  if (dc_wbe[1]) dc2[dc_wa] <= dc_wd[15:8];
  if (dc_wbe[0]) dc3[dc_wa] <= dc_wd[7:0];
  if (refill_we) begin dct[dm_addr[11:2]] <= {1'b1, dm_addr[31:12]}; dct2[dm_addr[11:2]] <= {1'b1, dm_addr[31:12]}; end
end
always @(posedge clk) begin
  dc_q   <= {dc0[dc_ra], dc1[dc_ra], dc2[dc_ra], dc3[dc_ra]};
  dct_q  <= dct[dc_ra];
  dct2_q <= dct2[dc_ra2];
end

wire        f_wb = wb_st && (wb_saddr[31:2] == oc_addr[31:2]);
wire        f_ls = ls_v  && (ls_addr == oc_addr[31:2]);
reg  [31:0] dqm;
always @* begin
  dqm[31:24] = (f_wb && sbe[3]) ? sd[31:24] : (f_ls && ls_be[3]) ? ls_data[31:24] : dc_q[31:24];
  dqm[23:16] = (f_wb && sbe[2]) ? sd[23:16] : (f_ls && ls_be[2]) ? ls_data[23:16] : dc_q[23:16];
  dqm[15:8]  = (f_wb && sbe[1]) ? sd[15:8]  : (f_ls && ls_be[1]) ? ls_data[15:8]  : dc_q[15:8];
  dqm[7:0]   = (f_wb && sbe[0]) ? sd[7:0]   : (f_ls && ls_be[0]) ? ls_data[7:0]   : dc_q[7:0];
end
reg  [31:0] ld_v;
always @* begin
  case (oc_lsz)
    SB: case (oc_addr[1:0]) 2'd0: ld_v = {24'h0, dqm[31:24]}; 2'd1: ld_v = {24'h0, dqm[23:16]};
                            2'd2: ld_v = {24'h0, dqm[15:8]};  default: ld_v = {24'h0, dqm[7:0]}; endcase
    SW: ld_v = oc_addr[1] ? {16'h0, dqm[15:0]} : {16'h0, dqm[31:16]};
    default: ld_v = dqm;
  endcase
end
wire dc_hit   = dct_q[20]  && (dct_q[19:0]  == oc_addr[31:12]);
wire st_hit   = dct2_q[20] && (dct2_q[19:0] == oc_saddr[31:12]);
wire st_match = oc_ld && ex_v && ex_st && (ex_saddr[31:2] == oc_addr[31:2]);
assign stall_oc = 1'b0;                          // OC never stalls: a miss or a store conflict replays in EX
wire   oc_miss  = oc_ld && !dc_hit;
wire   oc_rep   = oc_miss || st_match;

wire [31:0] ra_v = `RDB(oc_ra);
wire [31:0] rb_v = `RDB(oc_rb);
wire [31:0] op_a = (oc_asel == A_REG) ? ra_v : (oc_asel == A_MEM) ? ld_v : oc_imm;
wire [31:0] op_b = oc_bsel ? ld_v : rb_v;

always @(posedge clk) begin
  if (rst) oc_v <= 1'b0;
  else if (redirect_ex) oc_v <= 1'b0;
  else if (!stall_oc) oc_v <= ag_go;
  if (!stall_oc) begin
    oc_cls <= n_cls; oc_op <= n_op; oc_sz <= n_sz; oc_lsz <= ag_sz; /* load size = operand size */ oc_asel <= n_asel; oc_ra <= n_ra; oc_bsel <= n_bsel;
    oc_rb <= n_rb; oc_imm <= n_imm; oc_ld <= n_ld; oc_st <= n_st; oc_addr <= addr_s; oc_saddr <= n_saddr;
    oc_wen <= n_wen; oc_wreg <= n_wreg; oc_fl <= n_fl; oc_sexta <= n_sexta; oc_cc <= n_cc; oc_tgt <= ea_s;
    oc_pc <= ag_pc;
    oc_ws_en <= ws_en; oc_ws_r <= ag_r0; oc_ws_v <= upd_s; oc_wd_en <= wd_en; oc_wd_r <= ag_r9; oc_wd_v <= wd_val;
  end
end

// =====================================================================================================
// Bus: D-miss refill (one word) has priority over I-miss refill (two words)
// =====================================================================================================
reg bus_own;  // 1 = D
always @(posedge clk) begin
  ic_we <= 1'b0;
  if (rst) begin
    bus_req <= 1'b0; dm_st <= 2'd0; im_busy <= 1'b0; im_pend <= 1'b0; im_half <= 1'b0;
  end else begin
    case (dm_st)
      2'd0: if (ex_v && ex_rep && ex_miss) begin dm_st <= 2'd1; dm_addr <= ex_laddr[31:2]; end
      2'd1: if (bus_req && bus_own && bus_ack) begin dm_data <= bus_rdata; dm_st <= 2'd2; end
      2'd2: if (!wb_st) dm_st <= 2'd0;
      default: dm_st <= 2'd0;
    endcase
    if (im_now && !im_busy) begin im_busy <= 1'b1; im_pend <= 1'b1; im_half <= 1'b0; im_addr <= if2_pc[31:3]; end
    if (bus_req) begin
      if (bus_ack) begin
        bus_req <= 1'b0;
        if (!bus_own) begin
          if (!im_half) begin im_w0 <= bus_rdata; im_half <= 1'b1; im_pend <= 1'b1; end
          else begin
            ic_we <= 1'b1; ic_wa <= im_addr[11:3]; ic_wd <= {im_w0, bus_rdata}; ic_wt <= {1'b1, im_addr[31:12]};
            im_busy <= 1'b0;
          end
        end
      end
    end else if (dm_st == 2'd1) begin
      bus_req <= 1'b1; bus_own <= 1'b1; bus_addr <= dm_addr;
    end else if (im_pend) begin
      bus_req <= 1'b1; bus_own <= 1'b0; bus_addr <= {im_addr, im_half}; im_pend <= 1'b0;
    end
  end
end

// =====================================================================================================
// EX
// =====================================================================================================
reg  [4:0]  ccr;   // X N Z V C
wire [31:0] a0 = ex_afwd ? wb_val : ex_a;   // forward select decided one stage earlier (registered)
wire [31:0] b  = ex_bfwd ? wb_val : ex_b;
wire [31:0] a  = ex_sexta ? {{16{a0[15]}}, a0[15:0]} : a0;
wire        sub = (ex_op == O_SUB) || (ex_op == O_CMP);
wire [31:0] ax  = sub ? ~a : a;
wire [33:0] s2  = {1'b0, b, 1'b1} + {1'b0, ax, sub};      // one carry chain with carry in
wire [32:0] sum = s2[33:1];
wire        c8  = sum[8] ^ b[8] ^ ax[8], c16 = sum[16] ^ b[16] ^ ax[16], c32 = sum[32];

// shifter: one 32-bit rotator, masks for shifts
wire        sh_left = ex_cc[3], sh_rot = ex_cc[2], sh_ari = ex_cc[1];
wire [5:0]  sc   = a[5:0];
wire [31:0] bext = (ex_sz == SB) ? {{24{sh_ari & b[7]}},  b[7:0]}  :
                   (ex_sz == SW) ? {{16{sh_ari & b[15]}}, b[15:0]} : b;
wire [31:0] brep = (ex_sz == SB) ? {4{b[7:0]}} : (ex_sz == SW) ? {2{b[15:0]}} : b;
wire [31:0] rin  = (sh_left || sh_rot) ? brep : bext;
wire [4:0]  rc   = sh_left ? sc[4:0] : (~sc[4:0] + 5'd1);
wire [31:0] r1 = rc[0] ? {rin[30:0], rin[31]}    : rin;
wire [31:0] r2 = rc[1] ? {r1[29:0],  r1[31:30]}  : r1;
wire [31:0] r3 = rc[2] ? {r2[27:0],  r2[31:28]}  : r2;
wire [31:0] r4 = rc[3] ? {r3[23:0],  r3[31:24]}  : r3;
wire [31:0] rot = rc[4] ? {r4[15:0], r4[31:16]}  : r4;
reg  [31:0] tlo, thi;   // tlo[i] = i < sc ; thi[i] = i >= 32 - sc
always @* for (i = 0; i < 32; i = i + 1) begin tlo[i] = (i < sc); thi[i] = (i + sc >= 32); end
wire        fill = sh_ari & bext[31];
wire [31:0] shres = sh_rot ? rot : sh_left ? (rot & ~tlo) : ((rot & ~thi) | ({32{fill}} & thi));
wire        msb_rot = (ex_sz == SB) ? rot[7] : (ex_sz == SW) ? rot[15] : rot[31];
wire        sh_big = (ex_sz == SB) ? (sc > 6'd8) : (ex_sz == SW) ? (sc > 6'd16) : (sc > 6'd32);  // count beyond the size
wire        sh_c = (sc == 6'd0) ? 1'b0 : sh_rot ? (sh_left ? rot[0] : msb_rot) :
                   sh_big ? (!sh_left && sh_ari && bext[31]) : sh_left ? rot[0] : rot[31];

reg [31:0] r;
always @* begin
  case (ex_op)
    O_ADD, O_SUB, O_CMP: r = sum[31:0];
    O_AND: r = a & b;
    O_OR:  r = a | b;
    O_EOR: r = a ^ b;
    O_SHF: r = shres;
    default: r = a;
  endcase
end
wire [31:0] rm = (ex_sz == SB) ? {b[31:8], r[7:0]} : (ex_sz == SW) ? {b[31:16], r[15:0]} : r;
wire n_f = (ex_sz == SB) ? r[7] : (ex_sz == SW) ? r[15] : r[31];
wire z_f = (ex_sz == SB) ? (r[7:0] == 8'h0) : (ex_sz == SW) ? (r[15:0] == 16'h0) : (r == 32'h0);
wire bm  = (ex_sz == SB) ? b[7]  : (ex_sz == SW) ? b[15]  : b[31];
wire axm = (ex_sz == SB) ? ax[7] : (ex_sz == SW) ? ax[15] : ax[31];
wire cy  = (ex_sz == SB) ? c8 : (ex_sz == SW) ? c16 : c32;
wire v_add = (bm == axm) && (n_f != bm);
wire arith = (ex_op == O_ADD) || sub;
wire c_f = arith ? (cy ^ sub) : (ex_op == O_SHF) ? sh_c : 1'b0;
// ASL: V set when the top bit changes during the shift = any change between neighbour bits in the top (count+1) bits
wire [31:0] vla = (ex_sz == SB) ? {b[7:0], 24'h0} : (ex_sz == SW) ? {b[15:0], 16'h0} : b;
wire [31:0] vtg = vla ^ {vla[30:0], 1'b0};
wire asl_v = sh_left && sh_ari && (sc != 6'd0) && |(vtg & thi);
wire v_f = arith ? v_add : (ex_op == O_SHF) ? asl_v : 1'b0;
wire x_upd = (ex_op == O_ADD) || (ex_op == O_SUB) || (ex_op == O_SHF && !sh_rot && sc != 6'd0);  // ROx leave X

reg cond;
always @* begin
  case (ex_cc)
    4'h0: cond = 1'b1;                  4'h1: cond = 1'b0;
    4'h2: cond = !ccr[0] && !ccr[2];    4'h3: cond = ccr[0] || ccr[2];
    4'h4: cond = !ccr[0];               4'h5: cond = ccr[0];
    4'h6: cond = !ccr[2];               4'h7: cond = ccr[2];
    4'h8: cond = !ccr[1];               4'h9: cond = ccr[1];
    4'hA: cond = !ccr[3];               4'hB: cond = ccr[3];
    4'hC: cond = ccr[3] == ccr[1];      4'hD: cond = ccr[3] != ccr[1];
    4'hE: cond = !ccr[2] && (ccr[3] == ccr[1]);
    default: cond = ccr[2] || (ccr[3] != ccr[1]);
  endcase
end
assign redirect_ex = ex_v && (ex_rep || (ex_cls == C_BCC && cond));
assign ex_target   = ex_rep ? ex_pc : ex_tgt;   // replay restarts at the instruction itself

always @(posedge clk) begin
  if (rst) begin ex_v <= 1'b0; ccr <= 5'h0; end
  else begin
    ex_v <= oc_v && !stall_oc && !redirect_ex;
    if (ex_v && !ex_rep && ex_fl) ccr <= {x_upd ? c_f : ccr[4], n_f, z_f, v_f, c_f};
  end
  ex_cls <= oc_cls; ex_op <= oc_op; ex_sz <= oc_sz; ex_a <= op_a; ex_b <= op_b;
  ex_afwd <= (oc_asel == A_REG) && ex_v && ex_wen && (ex_wreg == oc_ra);   // EX now = WB next clock
  ex_bfwd <= !oc_bsel && ex_v && ex_wen && (ex_wreg == oc_rb);
  ex_sexta <= oc_sexta; ex_wen <= oc_wen; ex_wreg <= oc_wreg; ex_st <= oc_st; ex_saddr <= oc_saddr;
  ex_hit <= st_hit; ex_fl <= oc_fl; ex_cc <= oc_cc; ex_tgt <= oc_tgt; ex_pc <= oc_pc;
  ex_rep <= oc_rep; ex_miss <= oc_miss; ex_laddr <= oc_addr;
  ex_ws_en <= oc_ws_en; ex_ws_r <= oc_ws_r; ex_ws_v <= oc_ws_v; ex_wd_en <= oc_wd_en; ex_wd_r <= oc_wd_r; ex_wd_v <= oc_wd_v;
end

// =====================================================================================================
// WB
// =====================================================================================================
always @(posedge clk) begin
  if (rst) begin wb_wen <= 1'b0; wb_st <= 1'b0; ls_v <= 1'b0; bw_en <= 1'b0; end
  else begin
    wb_wen <= ex_v && !ex_rep && ex_wen;
    wb_st  <= ex_v && !ex_rep && ex_st;
    ls_v   <= wb_st;
    bw_en  <= wb_st;
  end
  wb_wreg <= ex_wreg; wb_val <= rm; wb_saddr <= ex_saddr; wb_sz <= ex_sz; wb_hit <= ex_hit;
  ls_addr <= wb_saddr[31:2]; ls_data <= sd; ls_be <= sbe;
  bw_addr <= wb_saddr[31:2]; bw_data <= sd; bw_be <= sbe;
end

always @(posedge clk) begin
  ret_v <= !rst && ex_commit; ret_pc <= ex_pc;
  ret_ws_en <= ex_ws_en; ret_ws_r <= ex_ws_r; ret_ws_v <= ex_ws_v;
  ret_wd_en <= ex_wd_en; ret_wd_r <= ex_wd_r; ret_wd_v <= ex_wd_v;
end
assign ret_wen = wb_wen; assign ret_wreg = wb_wreg; assign ret_wval = wb_val;
assign ret_st = wb_st; assign ret_saddr = wb_saddr; assign ret_sdata = sd; assign ret_sbe = sbe; assign ret_ccr = ccr;

assign obs_a   = wb_val ^ ex_pc;
assign obs_b   = fpc ^ oc_addr;
assign obs_ccr = ccr;

endmodule
`default_nettype wire
