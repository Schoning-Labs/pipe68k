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
// ==== p68_top.v: timing top for a core-alone build ===================================================================
//
// What it is for. This is not a computer. It is the smallest top level that lets Gowin's tools place and route the
// pipe68k core on its own and report how fast it is, without the whole Amiga 1200 system around it (that system lives
// in another repository and takes half an hour to two hours per build, against a few minutes here). tools/build.sh
// builds this top with hw/p68.cst, which gives it a clock pin and two data pins. The design runs on one clock, clk.
//
// The problem it solves. A synthesizer removes every piece of logic whose result never reaches an output pin, and it
// replaces every signal that is constant by its value. A CPU core with its memory port left open, or tied to fixed
// values, would shrink to almost nothing and the timing report would be meaningless. So this top makes everything look
// alive with only two pins:
//   - din, a serial input, is shifted into a 64-bit register sh. Its bits drive the reset, the interrupt level, the
//     snoop requests and part of the memory data, so none of these are constants.
//   - a fake memory answers the core's memory port: 2 clocks after a request it returns one 64-bit beat per clock, with
//     data mixed from the address and the shift register, so the instruction stream is not constant either.
//   - every output of the core that matters (the memory port's request fields, the observation outputs, the system
//     pins) is XORed together, folded down by parity in two register stages, and sent out on dout, so the synthesizer
//     must keep all of it.
// The fake memory has the same timing shape as a fast real one (2 clocks of latency, one beat per clock), so the paths
// around the memory port are realistic. The uncached map is fixed at $7C03 (chip RAM and I/O ranges as on an Amiga) so
// the uncached-path logic is not optimized away either.
//
// Nothing about this design is meant to run a program; it is for timing only.
`default_nettype none
module p68_top (input wire clk, input wire din, output wire dout);   // clock, serial in, serial out
  reg [63:0] sh = 64'h5a3c96e1_0f1e2d3c;                // shift register of the serial input (any non-trivial start value)
  always @(posedge clk) sh <= {sh[62:0], din};          // shift one bit in per clock
  reg rst_r = 1'b1;                                     // reset, starts high after configuration
  always @(posedge clk) rst_r <= sh[63] & sh[62] & sh[61];   // later driven by three input bits, so reset is not a constant

  wire        m_req, m_wr, m_unc, m_ins;                // memory port request, write, uncached, instruction fetch
  wire [31:2] m_addr;                                   // memory port longword address
  wire [1:0]  m_len;                                    // read beats - 1
  wire [3:0]  m_be;                                     // byte enables of a write
  wire [31:0] m_wdata, obs_a, obs_b;                    // write data; the core's observation outputs
  wire [4:0]  obs_ccr;                                  // the core's condition codes (observation)
  wire        iack, reset_o, stopped, lock;             // system pins of the core
  wire [2:0]  iack_lvl;                                 // level of an interrupt acknowledge
  wire [15:0] sr;                                       // status register (from the retire port)
  // fake memory behind the memory port: 2 clocks to the first beat, one beat per clock, data from address and input
  reg         busy = 1'b0, rdy = 1'b0, done = 1'b0;     // a transaction runs; a read beat is valid; the transaction ends
  reg  [1:0]  lat = 2'd0, nb = 2'd0, len = 2'd0;        // latency counter; beat counter; beats - 1 of this read
  reg         wr = 1'b0;                                // the transaction is a write
  reg  [31:2] ad;                                       // its address
  reg  [63:0] rdata;                                    // the read beat
  always @(posedge clk) begin
    rdy <= 1'b0; done <= 1'b0;                          // one-clock pulses
    if (m_req) begin busy <= 1'b1; lat <= 2'd0; nb <= 2'd0; len <= m_len; wr <= m_wr; ad <= m_addr; end   // take a request
    else if (busy) begin
      if (lat != 2'd2) lat <= lat + 2'd1;               // wait two clocks
      else if (wr) begin done <= 1'b1; busy <= 1'b0; end   // a write simply ends
      else begin
        // a read beat: address, beat number and input bits mixed, so every bit of the data varies
        rdy <= 1'b1; rdata <= {ad, nb, 1'b0, sh[30:0]} ^ {sh[63:32], ad[31:2] ^ sh[29:0], 2'b01};
        nb <= nb + 2'd1;
        if (nb == len) begin done <= 1'b1; busy <= 1'b0; end   // last beat ends the read
      end
    end
  end

  // The core. Snoop requests and their addresses, and the interrupt level, come from the shift register; the
  // uncached map is a fixed Amiga-like pattern; the upper-memory windows and the speed mode are off.
  p68_core core (
    .clk(clk), .rst(rst_r),
    .m_req(m_req), .m_wr(m_wr), .m_addr(m_addr), .m_len(m_len), .m_unc(m_unc), .m_ins(m_ins), .m_be(m_be),
    .m_wdata(m_wdata), .m_rdy(rdy), .m_rdata(rdata), .m_done(done),
    .m_snoop(sh[40] & sh[41] & sh[42]), .m_snoop_addr({sh[61:32], sh[1:0]} ^ {sh[29:0], 2'b00}), .unc_map(16'h7C03), .unc_hi(8'h00), .cac_hi(7'h00), .slow(1'b0),
    .obs_a(obs_a), .obs_b(obs_b), .obs_ccr(obs_ccr),
    .ipl(sh[10:8] & {3{sh[20]}}), .iack(iack), .iack_lvl(iack_lvl), .reset_o(reset_o), .stopped(stopped),
    .ret_sr(sr), .lock(lock));

  // Every write request folds into an accumulator, so the store path and its address are kept.
  reg [31:0] acc = 32'h0;
  always @(posedge clk) if (m_req) acc <= acc ^ m_wdata ^ {m_addr, m_len} ^ {26'h0, m_be, m_unc, m_ins};
  // Fold everything down to one bit in three register stages (32 bits, then 8 parity bits, then 1). The registers keep
  // the XOR tree short, so the output logic never shows up among the worst paths.
  reg [31:0] o1; reg [7:0] o2; reg o3;
  always @(posedge clk) begin
    o1 <= obs_a ^ obs_b ^ acc ^ {27'h0, obs_ccr} ^ {sr, 9'h0, lock, iack, iack_lvl, reset_o, stopped};
    o2 <= {^o1[31:28], ^o1[27:24], ^o1[23:20], ^o1[19:16], ^o1[15:12], ^o1[11:8], ^o1[7:4], ^o1[3:0]};
    o3 <= ^o2;
  end
  assign dout = o3;                                     // the one output pin
endmodule
`default_nettype wire
